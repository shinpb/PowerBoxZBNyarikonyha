//#define DUMMY_SENSOR_MODE 1

// Sys  
#include "esp_log.h"
#include "nvs.h"
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
#include "config.h"

#define ZB_SET(ep, cl, at, val) do {                                              \
    auto st_ = ezb_zcl_set_attr_value((ep), (cl), EZB_ZCL_CLUSTER_SERVER, (at),   \
                                      EZB_ZCL_STD_MANUF_CODE, (val), false);      \
    static uint32_t n_ = 0;                                                       \
    if ((int)st_ != 0 && (n_++ % 200) == 0)                                       \
        ESP_LOGW(Config.Logging.TagZigbee, "set_attr ep=0x%02X cl=0x%04X attr=0x%04X st=%d", \
                 (unsigned)(ep), (unsigned)(cl), (unsigned)(at), (int)st_);       \
} while (0)

ina3221_t ina3221_arr[n_INA]; 

static temperature_sensor_handle_t board_tsens = NULL;

void init_sensor_board() {
    temperature_sensor_config_t cfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(
        Config.Sensor.Board.MinTemp, Config.Sensor.Board.MaxTemp);
    ESP_ERROR_CHECK(temperature_sensor_install(&cfg, &board_tsens));
    ESP_ERROR_CHECK(temperature_sensor_enable(board_tsens));
}

void init_native_i2c_buses() {
#ifndef DUMMY_SENSOR_MODE
    ESP_ERROR_CHECK(i2cdev_init());
#endif
}

void init_sensors_dht() {
#ifndef DUMMY_SENSOR_MODE
    for (int idx = 0; idx < Config.Sensor.Dht.Count; idx++) {
        gpio_set_pull_mode(Config.Sensor.Dht.Pins[idx], GPIO_PULLUP_ONLY);
    }
#endif
}

void init_sensors_ina() {
#ifndef DUMMY_SENSOR_MODE
  for (int idx = 0; idx < Config.Sensor.Ina.Count; idx++) {
    for (uint8_t channel = 0; channel < INA3221_BUS_NUMBER; channel++) {
        ina3221_arr[idx].shunt[channel] = 
            (uint16_t)(Config.Sensor.Ina.Shunts[idx][channel] * 1000); // Ohm -> mOhm
    }
    uint8_t bus_idx = (idx < Config.Sensor.Ina.CountBus0) ? (0) : (1);
    uint8_t i2c_addr = 0x40 + (
        (idx < Config.Sensor.Ina.CountBus0) ? 
        (idx) : 
        (idx - Config.Sensor.Ina.CountBus0)
    );

    ESP_ERROR_CHECK(
        ina3221_init_desc(
            &ina3221_arr[idx],
            i2c_addr,
            Config.I2C.bus[bus_idx].port,
            Config.I2C.bus[bus_idx].pin.sda,
            Config.I2C.bus[bus_idx].pin.scl
        )
    );
    ina3221_arr[idx].i2c_dev.cfg.master.clk_speed = Config.I2C.bus[bus_idx].Speed;
    ESP_ERROR_CHECK(
        ina3221_set_options(
            &ina3221_arr[idx],
            true,   // continuous mode
            true,   // bus measurement enabled
            true    // shunt measurement enabled
        )
    );
    ESP_ERROR_CHECK(
        ina3221_enable_channel(
            &ina3221_arr[idx],
            true,
            true,
            true
        )
    );
  }
#endif
}

void dht_measurement_task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(Config.Measurement.DhtMs);

    while (true) {
        float board_c = 0.0f;
        if (temperature_sensor_get_celsius(board_tsens, &board_c) == ESP_OK) {
            if (Config.Logging.EnabledMeasurements) {
                ESP_LOGI(Config.Logging.TagMeasurement, "Board: %.2f°C", board_c);
            }
            int16_t measured_board = (int16_t)lroundf(board_c * 100.0f);
            if (is_zigbee_connected) {
                esp_zigbee_lock_acquire(portMAX_DELAY);
                ZB_SET(
                    Config.Zigbee.Board.Endpoint,
                    EZB_ZCL_CLUSTER_ID_TEMPERATURE_MEASUREMENT,
                    EZB_ZCL_ATTR_TEMPERATURE_MEASUREMENT_MEASURED_VALUE_ID,
                    &measured_board
                );
                esp_zigbee_lock_release();
            }
        }
        for (int idx = 0; idx < Config.Sensor.Dht.Count; idx++) {
#ifndef DUMMY_SENSOR_MODE
            float temperature = 0.0;
            float humidity = 0.0;
            static uint32_t dht_ok[n_DHT] = {0}, dht_fail[n_DHT] = {0};
            esp_err_t dht_err = dht_read_float_data(Config.Sensor.Dht.Type, Config.Sensor.Dht.Pins[idx], &humidity, &temperature);
            if (dht_err == ESP_OK) {
                dht_ok[idx]++;
#else
            static float tick_dht = 0.0;
            tick_dht += 0.1;
            float temperature = 22.0 + (sin(tick_dht * 0.5 + idx) * 1.5);
            float humidity = 45.0 + (cos(tick_dht * 0.5 + idx) * 5.0);
#endif
                if(Config.Logging.EnabledMeasurements) {
                  ESP_LOGI(Config.Logging.TagMeasurement, "DHT %d: %.2f°C %.0f%%", idx, temperature, humidity);
                }

                int16_t measured_temp = (int16_t)(temperature * 100);
                uint16_t measured_hum = (uint16_t)(humidity * 100);
                uint8_t ep = Config.Zigbee.Dht.Endpoints[idx];

                if (is_zigbee_connected) {
                    esp_zigbee_lock_acquire(portMAX_DELAY);
                    ZB_SET(
                        ep, 
                        EZB_ZCL_CLUSTER_ID_TEMPERATURE_MEASUREMENT,
                        EZB_ZCL_ATTR_TEMPERATURE_MEASUREMENT_MEASURED_VALUE_ID,
                        &measured_temp
                    );
                    ZB_SET(
                        ep,
                        EZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
                        EZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_MEASURED_VALUE_ID,
                        &measured_hum
                    );
                    esp_zigbee_lock_release();
                }
#ifndef DUMMY_SENSOR_MODE
            } else {
                dht_fail[idx]++;
                ESP_LOGW(Config.Logging.TagMeasurement, "DHT %d read failed: %s (ok=%u fail=%u)",
                         idx, esp_err_to_name(dht_err), (unsigned)dht_ok[idx], (unsigned)dht_fail[idx]);
            }
#endif
        }
        static uint32_t n_ = 0;
        if (++n_ % 100 == 0) {   // ~5 percenként
            nvs_stats_t st;
            if (nvs_get_stats(NULL, &st) == ESP_OK)
                ESP_LOGI(Config.Logging.TagZigbee, "NVS used=%u free=%u", (unsigned)st.used_entries, (unsigned)st.free_entries);
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency); 
    }
}

void dc_measurement_task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(Config.Measurement.DcMs); // 1 másodperces precíz mérés

    while (true) {
        for (int idx = 0; idx < Config.Sensor.Ina.Count; idx++) {
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
                auto err_i = ina3221_get_shunt_value(
                    &ina3221_arr[idx],
                    (ina3221_channel_t)channel,
                    &shuntVoltage_mV,
                    &current_mA
                );
                
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
                        ESP_LOGI(Config.Logging.TagMeasurement, "DC %d/%d: %.4fV %7.1fmA", idx, channel, voltage_V, current_mA);
                    }

                    int16_t measured_v = (int16_t)(voltage_V * 1000.0f);
                    int16_t measured_i = (int16_t)(current_mA);
                    int16_t measured_p = (int16_t)(power_mW / 10.0); //cW because of int16

                    if (is_zigbee_connected) {
                        esp_zigbee_lock_acquire(portMAX_DELAY);
                        
                        uint8_t ep_id = Config.Zigbee.Ina.Endpoints[idx][channel];
                        
                        // Feszültség
                        ZB_SET(
                            ep_id, 
                            EZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT,
                            EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_VOLTAGE_ID,
                            &measured_v
                        );
                        
                        // Áram
                        ZB_SET(
                            ep_id,
                            EZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT,
                            EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_CURRENT_ID,
                            &measured_i
                        );
                        
                        // Teljesítmény
                        ZB_SET(
                            ep_id,
                            EZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT,
                            EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_POWER_ID,
                            &measured_p
                        );
                        
                        esp_zigbee_lock_release();
                    }
#ifndef DUMMY_SENSOR_MODE
                } else { // Measurement failed
                    ESP_LOGW(Config.Logging.TagMeasurement, "Measure DC %d/%d failed: %d, %d",
                      idx, channel,
                      err_v, err_i );
                }
#endif
            }
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency); 
    }
}

static uint16_t ina_rd(ina3221_t *d, uint8_t reg) {
    uint8_t b[2] = {0, 0};
    I2C_DEV_TAKE_MUTEX(&d->i2c_dev);
    esp_err_t e = i2c_dev_read_reg(&d->i2c_dev, reg, b, 2);
    I2C_DEV_GIVE_MUTEX(&d->i2c_dev);
    if (e != ESP_OK) { ESP_LOGW("INADiag", "reg 0x%02X read err %s", reg, esp_err_to_name(e)); return 0; }
    return (uint16_t)((b[0] << 8) | b[1]);
}

void ina_diag() {
    for (int i = 0; i < Config.Sensor.Ina.Count; i++) {
        ina3221_t *d = &ina3221_arr[i];
        ESP_LOGI("INADiag", "INA%d mfr=0x%04X (vart 0x5449) die=0x%04X (vart 0x3220) cfg=0x%04X",
                 i, ina_rd(d, 0xFE), ina_rd(d, 0xFF), ina_rd(d, 0x00));
        for (int ch = 0; ch < 3; ch++) {
            uint16_t sh = ina_rd(d, 0x01 + 2 * ch);
            uint16_t bu = ina_rd(d, 0x02 + 2 * ch);
            ESP_LOGI("INADiag", "  ch%d shunt_raw=0x%04X bus_raw=0x%04X", ch, sh, bu);
        }
    }
}