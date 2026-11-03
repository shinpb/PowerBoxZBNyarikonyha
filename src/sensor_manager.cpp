//#define DUMMY_SENSOR_MODE 1

// Sys  
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs.h"
#include "esp_system.h"   // esp_get_free_heap_size()
#include "esp_timer.h"    // esp_timer_get_time()
// Drivers
#include "driver/gpio.h"
#include "driver/temperature_sensor.h"
// External Libraries
#include "esp_zigbee.h"
#include "ezbee/zha.h"
#include <math.h>
#ifndef DUMMY_SENSOR_MODE
#include "dht.h"
#include "ina3221.h"
#endif
// Internal components
#include "sensor_manager.h"
#include "zigbee_manager.h"
#include "health.h"
#include "heartbeat.h"
#include "config.h"


#define ZB_SET(ep, cl, at, val) do {                                              \
    auto st_ = ezb_zcl_set_attr_value((ep), (cl), EZB_ZCL_CLUSTER_SERVER, (at),   \
                                      EZB_ZCL_STD_MANUF_CODE, (val), false);      \
    static uint32_t n_ = 0;                                                       \
    if ((int)st_ != 0 && (n_++ % Config.Logging.SetAttrFailLogEvery) == 0)                                       \
        LOGW_ZB("set_attr ep=0x%02X cl=0x%04X attr=0x%04X st=%d", \
                 (unsigned)(ep), (unsigned)(cl), (unsigned)(at), (int)st_);       \
} while (0)

#ifndef DUMMY_SENSOR_MODE
static ina3221_t ina3221_arr[n_INA];
#endif

static temperature_sensor_handle_t board_tsens = NULL;

// ---- Fault handling state (see health.h) ----
// Consecutive failed cycles; the error flag is set at Config.Health.FailThreshold.
static uint8_t s_board_fail = 0;
// "invalid value already written to the Zigbee attributes" flags (4.7): the invalid value is written once
// per failure streak (retried until the write is allowed, i.e. after the join); a good measurement clears them.
static bool    s_board_inv = false;
#ifndef DUMMY_SENSOR_MODE
static uint8_t s_dht_fail[n_DHT] = {0};
static bool    s_dht_inv[n_DHT] = {false};
static bool    s_ina_inv[n_INA] = {false};
static bool    s_i2c_ready = false;
static bool    s_ina_desc_ok[n_INA] = {false};   // ina3221_init_desc() done (must run only once: it creates a mutex)
static bool    s_ina_ok[n_INA] = {false};        // INA fully initialized (readable)
static uint8_t s_ina_fail[n_INA] = {0};
static TickType_t s_ina_retry_tick[n_INA] = {0};
#endif

static inline uint8_t sat_inc(uint8_t v) { return v < 255 ? v + 1 : v; }

// Cycle duration statistics for the heartbeat (see heartbeat.h). over: optional overrun counter.
static void cycle_stat(volatile uint32_t &last, volatile uint32_t &max, volatile uint32_t *over,
                       int64_t t0_us, int period_ms) {
    uint32_t ms = (uint32_t)((esp_timer_get_time() - t0_us) / 1000);
    last = ms;
    if (ms > max) max = ms;
    if (over != NULL && ms > (uint32_t)period_ms) diag_inc(*over);
}

// Sensor init failures do not abort the firmware (the Zigbee side keeps running);
// Config.Health.AbortOnInitFail = 1 restores the old abort for development.
static void init_failed(const char *what, esp_err_t e) {
    LOGE_INIT_SENSOR("%s failed: %s", what, esp_err_to_name(e));
    if (Config.Health.AbortOnInitFail) ESP_ERROR_CHECK(e);
}

void init_sensor_board() {
    temperature_sensor_config_t cfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(
        Config.Sensor.Board.MinTemp, Config.Sensor.Board.MaxTemp);
    esp_err_t e = temperature_sensor_install(&cfg, &board_tsens);
    if (e != ESP_OK) {
        board_tsens = NULL;
    } else {
        e = temperature_sensor_enable(board_tsens);
    }
    if (e != ESP_OK) {
        init_failed("board temperature sensor init", e);
        s_board_fail = Config.Health.FailThreshold;
        health_set(HM_SENSOR, HF_SENS_BOARD);
    }
}

void init_native_i2c_buses() {
#ifndef DUMMY_SENSOR_MODE
    esp_err_t e = i2cdev_init();
    if (e == ESP_OK) {
        s_i2c_ready = true;
    } else {
        init_failed("i2cdev_init", e);
        health_set(HM_BUS, HF_BUS_I2C_INIT);
    }
#endif
}

void init_sensors_dht() {
#ifndef DUMMY_SENSOR_MODE
    for (int idx = 0; idx < Config.Sensor.Dht.Count; idx++) {
        gpio_set_pull_mode(Config.Sensor.Dht.Pins[idx], GPIO_PULLUP_ONLY);
    }
#endif
}

#ifndef DUMMY_SENSOR_MODE
static inline uint8_t ina_bus_of(int idx) { return (idx < Config.Sensor.Ina.CountBus0) ? 0 : 1; }

// Initializes (or re-initializes) one INA3221 module. Returns ESP_OK on success.
static esp_err_t ina_init_one(int idx) {
    esp_err_t e;
    if (!s_ina_desc_ok[idx]) {
        for (uint8_t channel = 0; channel < INA3221_BUS_NUMBER; channel++) {
            ina3221_arr[idx].shunt[channel] =
                (uint16_t)(Config.Sensor.Ina.Shunts[idx][channel] * 1000); // Ohm -> mOhm
        }
        uint8_t bus_idx = ina_bus_of(idx);
        uint8_t i2c_addr = Config.Sensor.Ina.BaseAddr + (
            (idx < Config.Sensor.Ina.CountBus0) ?
            (idx) :
            (idx - Config.Sensor.Ina.CountBus0)
        );
        e = ina3221_init_desc(
            &ina3221_arr[idx],
            i2c_addr,
            Config.I2C.bus[bus_idx].port,
            Config.I2C.bus[bus_idx].pin.sda,
            Config.I2C.bus[bus_idx].pin.scl
        );
        if (e != ESP_OK) return e;
        s_ina_desc_ok[idx] = true;
        // I2C clock from Config (ina3221_init_desc sets 1 MHz; the device is created lazily on first transaction)
        ina3221_arr[idx].i2c_dev.cfg.master.clk_speed = Config.I2C.bus[bus_idx].Speed;
    }
    e = ina3221_set_options(
        &ina3221_arr[idx],
        true,   // continuous mode
        true,   // bus measurement enabled
        true    // shunt measurement enabled
    );
    if (e != ESP_OK) return e;
    return ina3221_enable_channel(&ina3221_arr[idx], true, true, true);
}

// Bus / sensor error flags from the per-module failure counters.
// If EVERY module on an I2C bus fails, the cause is the bus (typically wiring): the bus flag is set
// and the module flags of that bus are cleared. Otherwise the failing modules get their own flag.
static void ina_update_health() {
    const uint8_t thr = Config.Health.FailThreshold;
    for (uint8_t b = 0; b < n_I2C; b++) {
        int members = 0, failed = 0;
        for (int i = 0; i < Config.Sensor.Ina.Count; i++) {
            if (ina_bus_of(i) != b) continue;
            members++;
            if (s_ina_fail[i] >= thr) failed++;
        }
        const bool bus_failed = (members > 0) && (failed == members);
        if (bus_failed) health_set(HM_BUS, HF_BUS_I2C(b)); else health_clear(HM_BUS, HF_BUS_I2C(b));
        for (int i = 0; i < Config.Sensor.Ina.Count; i++) {
            if (ina_bus_of(i) != b) continue;
            if (!bus_failed && s_ina_fail[i] >= thr) health_set(HM_SENSOR, HF_SENS_INA(i));
            else health_clear(HM_SENSOR, HF_SENS_INA(i));
        }
    }
}
#endif

void init_sensors_ina() {
#ifndef DUMMY_SENSOR_MODE
  if (!s_i2c_ready) {
    LOGE_INIT_SENSOR("I2C not initialized, INA modules are skipped");
    for (int idx = 0; idx < Config.Sensor.Ina.Count; idx++) s_ina_fail[idx] = Config.Health.FailThreshold;
    ina_update_health();
    return;
  }
  for (int idx = 0; idx < Config.Sensor.Ina.Count; idx++) {
    esp_err_t e = ina_init_one(idx);
    if (e == ESP_OK) {
        s_ina_ok[idx] = true;
        s_ina_fail[idx] = 0;
    } else {
        LOGE_INIT_SENSOR("INA %d init failed: %s (retry every %u s)",
                 idx, esp_err_to_name(e), (unsigned)Config.Health.RetryS);
        if (Config.Health.AbortOnInitFail) ESP_ERROR_CHECK(e);
        s_ina_fail[idx] = Config.Health.FailThreshold;
        s_ina_retry_tick[idx] = xTaskGetTickCount();
    }
  }
  ina_update_health();
#endif
}

// ZCL attribute write access.
// Normal operation (measurement tasks): only after the network join, under the Zigbee lock.
// pre_start (measure_all_once, called from the stack context): no join needed, no lock needed.
static inline bool zb_write_allowed(bool pre_start) { return pre_start || is_zigbee_connected; }
static inline void zb_lock(bool pre_start)   { if (!pre_start) esp_zigbee_lock_acquire(portMAX_DELAY); }
static inline void zb_unlock(bool pre_start) { if (!pre_start) esp_zigbee_lock_release(); }

// Attribute writers (one lock section each). Return false if the write is not allowed yet (not joined).
static bool zb_write_board(int16_t temp, bool pre_start) {
    if (!zb_write_allowed(pre_start)) return false;
    zb_lock(pre_start);
    ZB_SET(Config.Zigbee.Board.Endpoint, EZB_ZCL_CLUSTER_ID_TEMPERATURE_MEASUREMENT,
           EZB_ZCL_ATTR_TEMPERATURE_MEASUREMENT_MEASURED_VALUE_ID, &temp);
    zb_unlock(pre_start);
    return true;
}

static bool zb_write_dht(uint8_t ep, int16_t temp, uint16_t hum, bool pre_start) {
    if (!zb_write_allowed(pre_start)) return false;
    zb_lock(pre_start);
    ZB_SET(ep, EZB_ZCL_CLUSTER_ID_TEMPERATURE_MEASUREMENT,
           EZB_ZCL_ATTR_TEMPERATURE_MEASUREMENT_MEASURED_VALUE_ID, &temp);
    ZB_SET(ep, EZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
           EZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_MEASURED_VALUE_ID, &hum);
    zb_unlock(pre_start);
    return true;
}

static bool zb_write_dc(uint8_t ep, int16_t v, int16_t i, int16_t p, bool pre_start) {
    if (!zb_write_allowed(pre_start)) return false;
    zb_lock(pre_start);
    ZB_SET(ep, EZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_VOLTAGE_ID, &v);
    ZB_SET(ep, EZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_CURRENT_ID, &i);
    ZB_SET(ep, EZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_POWER_ID, &p);
    zb_unlock(pre_start);
    return true;
}

// 4.8: device fault indication on the board endpoint (Binary Input + fault mask attribute).
// Called at the end of every measurement cycle; writes only when the mask changed (under the Zigbee lock
// so the two measurement tasks cannot interleave their compare/write).
static bool     s_diag_written = false;
static uint32_t s_diag_last = 0;

static void diag_publish(bool pre_start) {
    if (!zb_write_allowed(pre_start)) return;
    zb_lock(pre_start);
    uint32_t mask = health_zb_mask();
    if (!s_diag_written || mask != s_diag_last) {
        bool    pv    = (mask != 0);
        uint8_t flags = pv ? EZB_ZCL_BINARY_INPUT_STATUS_FLAGS_FAULT : 0;
        const uint8_t ep = Config.Zigbee.Board.Endpoint;
        ZB_SET(ep, EZB_ZCL_CLUSTER_ID_BINARY_INPUT, EZB_ZCL_ATTR_BINARY_INPUT_PRESENT_VALUE_ID, &pv);
        ZB_SET(ep, EZB_ZCL_CLUSTER_ID_BINARY_INPUT, EZB_ZCL_ATTR_BINARY_INPUT_STATUS_FLAGS_ID, &flags);
        ZB_SET(ep, ZB_DIAG_CLUSTER_ID, ZB_DIAG_ATTR_FAULT_MASK, &mask);
        s_diag_last = mask;
        s_diag_written = true;
    }
    zb_unlock(pre_start);
}

// Set by measure_all_once(): the tasks then skip their first immediate cycle
// (the DHT sensor needs >= 2 s between two reads).
static volatile bool s_pre_measured = false;

static void dht_cycle(bool pre_start) {
        float board_c = 0.0f;
        if (board_tsens != NULL && temperature_sensor_get_celsius(board_tsens, &board_c) == ESP_OK) {
            if (s_board_fail >= Config.Health.FailThreshold) health_clear(HM_SENSOR, HF_SENS_BOARD);
            s_board_fail = 0;
            diag_mark(g_diag.board_last_ok_s);
            if (Config.Logging.EnabledMeasurements) {
                LOGI_MEAS("Board: %.2f°C", board_c);
            }
            int16_t measured_board = (int16_t)lroundf(board_c * 100.0f);
            s_board_inv = false;
            zb_write_board(measured_board, pre_start);
        } else {
            if (board_tsens != NULL) {   // read error (a NULL handle = init failure, already flagged, counter preset)
                s_board_fail = sat_inc(s_board_fail);
                diag_inc(g_diag.board_fail);
                if (s_board_fail >= Config.Health.FailThreshold) health_set(HM_SENSOR, HF_SENS_BOARD);
            }
            // sensor unavailable: report the ZCL invalid value (once per streak)
            if (s_board_fail >= Config.Health.FailThreshold && !s_board_inv) {
                s_board_inv = zb_write_board(ZCL_INVALID_TEMP, pre_start);
            }
        }
        for (int idx = 0; idx < Config.Sensor.Dht.Count; idx++) {
#ifndef DUMMY_SENSOR_MODE
            float temperature = 0.0;
            float humidity = 0.0;
            esp_err_t dht_err = dht_read_float_data(Config.Sensor.Dht.Type, Config.Sensor.Dht.Pins[idx], &humidity, &temperature);
            if (dht_err == ESP_OK) {
                diag_inc(g_diag.dht_ok[idx]);
                diag_mark(g_diag.dht_last_ok_s[idx]);
                if (s_dht_fail[idx] > 0) {
                    // WARN on purpose (PLAN.md 4.6a): as INFO this line was intermittently missing from the
                    // serial log (also in a raw terminal capture); the failure line is WARN as well.
                    LOGW_MEAS("DHT %d recovered after %u failed reads",
                             idx, (unsigned)s_dht_fail[idx]);
                }
                if (s_dht_fail[idx] >= Config.Health.FailThreshold) health_clear(HM_SENSOR, HF_SENS_DHT(idx));
                s_dht_fail[idx] = 0;
                s_dht_inv[idx] = false;
#else
            static float tick_dht = 0.0;
            tick_dht += 0.1;
            float temperature = 22.0 + (sin(tick_dht * 0.5 + idx) * 1.5);
            float humidity = 45.0 + (cos(tick_dht * 0.5 + idx) * 5.0);
#endif
                if(Config.Logging.EnabledMeasurements) {
                  LOGI_MEAS("DHT %d: %.2f°C %.0f%%", idx, temperature, humidity);
                }

                int16_t measured_temp = (int16_t)(temperature * 100);
                uint16_t measured_hum = (uint16_t)(humidity * 100);
                uint8_t ep = Config.Zigbee.Dht.Endpoints[idx];

                zb_write_dht(ep, measured_temp, measured_hum, pre_start);
#ifndef DUMMY_SENSOR_MODE
            } else {
                diag_inc(g_diag.dht_fail[idx]);
                s_dht_fail[idx] = sat_inc(s_dht_fail[idx]);
                if (s_dht_fail[idx] >= Config.Health.FailThreshold) {
                    health_set(HM_SENSOR, HF_SENS_DHT(idx));
                    // sensor unavailable: report the ZCL invalid values (once per streak)
                    if (!s_dht_inv[idx]) {
                        s_dht_inv[idx] = zb_write_dht(Config.Zigbee.Dht.Endpoints[idx], ZCL_INVALID_TEMP, ZCL_INVALID_HUM, pre_start);
                    }
                }
                LOGW_MEAS("DHT %d read failed: %s (ok=%u fail=%u)",
                         idx, esp_err_to_name(dht_err), (unsigned)g_diag.dht_ok[idx], (unsigned)g_diag.dht_fail[idx]);
            }
#endif
        }
        diag_publish(pre_start);
}

static void dht_measurement_task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(Config.Measurement.DhtMs);

    if (s_pre_measured) {
        vTaskDelay(xFrequency);
        xLastWakeTime = xTaskGetTickCount();
    }
    while (true) {
        int64_t t0 = esp_timer_get_time();
        dht_cycle(false);
        cycle_stat(g_diag.dht_cycle_last_ms, g_diag.dht_cycle_max_ms, NULL, t0, Config.Measurement.DhtMs);
        // (the periodic heap/NVS status moved to the independent heartbeat task, see heartbeat.cpp)
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

static void dc_cycle(bool pre_start) {
        for (int idx = 0; idx < Config.Sensor.Ina.Count; idx++) {
#ifndef DUMMY_SENSOR_MODE
            if (!s_ina_ok[idx]) {
                // Not initialized: retry periodically (not from the stack context of measure_all_once)
                if (!pre_start && s_i2c_ready &&
                    (xTaskGetTickCount() - s_ina_retry_tick[idx]) >= pdMS_TO_TICKS(Config.Health.RetryS * 1000UL)) {
                    s_ina_retry_tick[idx] = xTaskGetTickCount();
                    diag_inc(g_diag.ina_reinit_try);
                    esp_err_t e = ina_init_one(idx);
                    LOGI_INIT_SENSOR("INA %d re-init: %s", idx, esp_err_to_name(e));
                    if (e == ESP_OK) { s_ina_ok[idx] = true; s_ina_fail[idx] = 0; diag_inc(g_diag.ina_reinit_ok); }
                }
                if (!s_ina_ok[idx]) {
                    // offline module: report the ZCL invalid values (once per outage)
                    if (!s_ina_inv[idx]) {
                        const uint8_t *eps = Config.Zigbee.Ina.Endpoints[idx];
                        bool all = true;
                        for (uint8_t ch = 0; ch < INA3221_BUS_NUMBER; ch++)
                            all = zb_write_dc(eps[ch], ZCL_INVALID_DC, ZCL_INVALID_DC, ZCL_INVALID_DC, pre_start) && all;
                        s_ina_inv[idx] = all;
                    }
                    continue;
                }
            }
            bool ina_failed = false;
#endif
            for (uint8_t channel = 0; channel < INA3221_BUS_NUMBER; channel++) {
#ifndef DUMMY_SENSOR_MODE
                float voltage_V = 0.0;
                auto err_v = ina3221_get_bus_voltage(
                    &ina3221_arr[idx],
                    (ina3221_channel_t)channel,
                    &voltage_V
                );
                
                float shuntVoltage_mV = 0.0f;
                float current_mA = 0.0; 
                // A failed access can take ~1-2 s (i2cdev retries): no second access after a failed one
                esp_err_t err_i = ESP_FAIL;
                if (err_v == ESP_OK) {
                    err_i = ina3221_get_shunt_value(
                        &ina3221_arr[idx],
                        (ina3221_channel_t)channel,
                        &shuntVoltage_mV,
                        &current_mA
                    );
                }
                
                if ((err_v == ESP_OK) && (err_i == ESP_OK)) { // measurement success
#else
                static float tick_dc = 0.0;
                tick_dc += 0.1;
                float voltage_V = 12.0 + (sin(tick_dc + idx*3+channel) * 0.5); 
                float current_mA = (1.5 + (cos(tick_dc + idx*3+channel) * 0.2))*1000;
#endif
                    //const float r_shunt = Config.Sensor.Ina.Shunts[idx][channel];
                    //current_mA = shuntVoltage_mV / r_shunt;
                    float power_mW = voltage_V * current_mA;

                    if(Config.Logging.EnabledMeasurements) {
                        LOGI_MEAS("DC %d/%d: %.4fV %7.1fmA", idx, channel, voltage_V, current_mA);
                    }

                    int16_t measured_v = (int16_t)(voltage_V * 1000.0f);
                    int16_t measured_i = (int16_t)(current_mA);
                    int16_t measured_p = (int16_t)(power_mW / 10.0); //cW because of int16

                    zb_write_dc(Config.Zigbee.Ina.Endpoints[idx][channel], measured_v, measured_i, measured_p, pre_start);
#ifndef DUMMY_SENSOR_MODE
                } else { // Measurement failed (the attribute keeps its last value)
                    ina_failed = true;
                    // log the first failure of a streak and then every 60th cycle only
                    if (s_ina_fail[idx] == 0 || (s_ina_fail[idx] % Config.Logging.InaFailLogEvery) == 0) {
                        LOGW_MEAS("Measure DC %d/%d failed: %d, %d",
                          idx, channel,
                          err_v, err_i );
                    }
                    break;   // skip the remaining channels of a failing module (each failed access may time out)
                }
#endif
            }
#ifndef DUMMY_SENSOR_MODE
            s_ina_fail[idx] = ina_failed ? sat_inc(s_ina_fail[idx]) : 0;
            if (!ina_failed) { diag_mark(g_diag.ina_last_ok_s[idx]); s_ina_inv[idx] = false; }
            if (s_ina_fail[idx] >= Config.Health.FailThreshold) {
                // Offline: stop polling (every failed access costs ~1-2 s and slows every other module),
                // probe it again with a re-init every Config.Health.RetryS seconds.
                s_ina_ok[idx] = false;
                s_ina_retry_tick[idx] = xTaskGetTickCount();
                diag_inc(g_diag.ina_offline[idx]);
                LOGW_MEAS("INA %d offline, re-init every %u s",
                         idx, (unsigned)Config.Health.RetryS);
            }
#endif
        }
#ifndef DUMMY_SENSOR_MODE
        ina_update_health();
#endif
        diag_publish(pre_start);
}

static void dc_measurement_task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(Config.Measurement.DcMs); // 1 másodperces precíz mérés

    if (s_pre_measured) {
        vTaskDelay(xFrequency);
        xLastWakeTime = xTaskGetTickCount();
    }
    while (true) {
        int64_t t0 = esp_timer_get_time();
        dc_cycle(false);
        cycle_stat(g_diag.dc_cycle_last_ms, g_diag.dc_cycle_max_ms, &g_diag.dc_overruns, t0, Config.Measurement.DcMs);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void measure_all_once() {
    dht_cycle(true);
    dc_cycle(true);
    s_pre_measured = true;
}

void start_measurement_tasks() {
    static bool started = false;
    if (started) return;
    started = true;
    LOGI_SETUP("Starting measurement tasks");
    xTaskCreate(dht_measurement_task, "Task_DHT", Config.Tasks.MeasDht.Stack, NULL, Config.Tasks.MeasDht.Prio, NULL);
    xTaskCreate(dc_measurement_task,  "Task_DC",  Config.Tasks.MeasDc.Stack, NULL, Config.Tasks.MeasDc.Prio, NULL);
}

#ifndef DUMMY_SENSOR_MODE
static uint16_t ina_rd(ina3221_t *d, uint8_t reg) {
    uint8_t b[2] = {0, 0};
    I2C_DEV_TAKE_MUTEX(&d->i2c_dev);
    esp_err_t e = i2c_dev_read_reg(&d->i2c_dev, reg, b, 2);
    I2C_DEV_GIVE_MUTEX(&d->i2c_dev);
    if (e != ESP_OK) { LOGW_INADIAG("reg 0x%02X read err %s", reg, esp_err_to_name(e)); return 0; }
    return (uint16_t)((b[0] << 8) | b[1]);
}

void ina_diag() {
    for (int i = 0; i < Config.Sensor.Ina.Count; i++) {
        // A module that failed its init would block here for seconds (I2C retries) and delay the Zigbee start
        if (!s_ina_ok[i]) { LOGW_INADIAG("INA%d: not initialized, skipped", i); continue; }
        ina3221_t *d = &ina3221_arr[i];
        LOGI_INADIAG("INA%d mfr=0x%04X (vart 0x5449) die=0x%04X (vart 0x3220) cfg=0x%04X",
                 i, ina_rd(d, 0xFE), ina_rd(d, 0xFF), ina_rd(d, 0x00));
        for (int ch = 0; ch < 3; ch++) {
            uint16_t sh = ina_rd(d, 0x01 + 2 * ch);
            uint16_t bu = ina_rd(d, 0x02 + 2 * ch);
            LOGI_INADIAG("  ch%d shunt_raw=0x%04X bus_raw=0x%04X", ch, sh, bu);
        }
    }
}
#else
void ina_diag() {}   // no real INA devices in dummy mode
#endif
