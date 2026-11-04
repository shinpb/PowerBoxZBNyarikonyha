// INA3221 current/voltage modules on the I2C buses (15 DC channels)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <math.h>
#ifndef DUMMY_SENSOR_MODE
#include "ina3221.h"
#endif
// Internal components
#include "sensor_common.h"
#include "sensor_manager.h"
#include "zigbee_attr.h"
#include "zcl_values.h"
#include "health.h"
#include "heartbeat.h"
#include "config.h"

#ifndef DUMMY_SENSOR_MODE
static ina3221_t ina3221_arr[n_INA];
static bool    s_ina_inv[n_INA] = {false};
static bool    s_i2c_ready = false;
static bool    s_ina_desc_ok[n_INA] = {false};   // ina3221_init_desc() done (must run only once: it creates a mutex)
static bool    s_ina_ok[n_INA] = {false};        // INA fully initialized (readable)
static uint32_t s_ina_fail[n_INA] = {0};
static uint32_t s_ina_good[n_INA] = {0};   // consecutive good cycles while the error flag is set (recovery hysteresis)
static TickType_t s_ina_retry_tick[n_INA] = {0};
#endif

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

void ina_cycle(bool pre_start) {
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
                    if (e == ESP_OK) {
                        // back online, but the error flag stays until RecoverThreshold good cycles (any failure = offline again)
                        s_ina_ok[idx] = true; s_ina_fail[idx] = Config.Health.FailThreshold; s_ina_good[idx] = 0;
                        diag_inc(g_diag.ina_reinit_ok);
                    }
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
            const bool publish = (s_ina_fail[idx] < Config.Health.FailThreshold);   // false while recovering
#else
            const bool publish = true;
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

                    int16_t measured_v = (int16_t)lroundf(voltage_V * 1000.0f);
                    int16_t measured_i = (int16_t)lroundf(current_mA);
                    int16_t measured_p = (int16_t)lroundf(power_mW / 10.0f); // cW because of int16 (max 327.67 W: 12.6 A at 26 V; the planned 12..15 V / <10 A fits)

                    if (publish) zb_write_dc(Config.Zigbee.Ina.Endpoints[idx][channel], measured_v, measured_i, measured_p, pre_start);
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
            if (ina_failed) {
                s_ina_fail[idx] = sat_inc(s_ina_fail[idx]);
                s_ina_good[idx] = 0;
            } else {
                diag_mark(g_diag.ina_last_ok_s[idx]);
                if (s_ina_fail[idx] >= Config.Health.FailThreshold) {   // recovering
                    s_ina_good[idx] = sat_inc(s_ina_good[idx]);
                    if (s_ina_good[idx] >= Config.Health.RecoverThreshold) {
                        s_ina_fail[idx] = 0; s_ina_good[idx] = 0; s_ina_inv[idx] = false;
                    }
                } else {
                    s_ina_fail[idx] = 0;
                }
            }
            if (ina_failed && s_ina_fail[idx] >= Config.Health.FailThreshold) {
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
