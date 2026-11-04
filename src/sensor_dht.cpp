// DHT (AM2301) temperature / humidity sensors
#include "esp_log.h"
#include "driver/gpio.h"
#include <math.h>
#include "dht.h"
// Internal components
#include "sensor_common.h"
#include "sensor_manager.h"
#include "zigbee_attr.h"
#include "zcl_values.h"
#include "health.h"
#include "heartbeat.h"
#include "config.h"

#ifndef DUMMY_SENSOR_MODE
static uint32_t s_dht_fail[n_DHT] = {0};
static uint32_t s_dht_good[n_DHT] = {0};   // consecutive good reads while the error flag is set (recovery hysteresis)
static bool    s_dht_inv[n_DHT] = {false};   // "invalid value already written" (once per failure streak, 4.7)
#endif

void init_sensors_dht() {
#ifndef DUMMY_SENSOR_MODE
    for (int idx = 0; idx < Config.Sensor.Dht.Count; idx++) {
        gpio_set_pull_mode(Config.Sensor.Dht.Pins[idx], GPIO_PULLUP_ONLY);
    }
#endif
}

void dht_cycle(bool pre_start) {
        for (int idx = 0; idx < Config.Sensor.Dht.Count; idx++) {
            bool publish = true;   // false while recovering (fault flag set, not enough good reads yet)
#ifndef DUMMY_SENSOR_MODE
            float temperature = 0.0;
            float humidity = 0.0;
            esp_err_t dht_err = dht_read_float_data(Config.Sensor.Dht.Type, Config.Sensor.Dht.Pins[idx], &humidity, &temperature);
            if (dht_err == ESP_OK) {
                diag_inc(g_diag.dht_ok[idx]);
                diag_mark(g_diag.dht_last_ok_s[idx]);
                if (s_dht_fail[idx] >= Config.Health.FailThreshold) {
                    // fault active: the flag is cleared (and values are published again) after RecoverThreshold good reads in a row
                    s_dht_good[idx] = sat_inc(s_dht_good[idx]);
                    if (s_dht_good[idx] >= Config.Health.RecoverThreshold) {
                        // WARN on purpose (PLAN.md 4.6a): as INFO this line was intermittently missing from the
                        // serial log (also in a raw terminal capture); the failure line is WARN as well.
                        LOGW_MEAS("DHT %d recovered after %u failed reads", idx, (unsigned)s_dht_fail[idx]);
                        health_clear(HM_SENSOR, HF_SENS_DHT(idx));
                        s_dht_fail[idx] = 0;
                        s_dht_good[idx] = 0;
                        s_dht_inv[idx] = false;
                    } else {
                        publish = false;
                    }
                } else {
                    if (s_dht_fail[idx] > 0) LOGW_MEAS("DHT %d recovered after %u failed reads", idx, (unsigned)s_dht_fail[idx]);
                    s_dht_fail[idx] = 0;
                }
#else
            static float tick_dht = 0.0;
            tick_dht += 0.1;
            float temperature = 22.0 + (sin(tick_dht * 0.5 + idx) * 1.5);
            float humidity = 45.0 + (cos(tick_dht * 0.5 + idx) * 5.0);
#endif
                if (publish) {
                if(Config.Logging.EnabledMeasurements) {
                  LOGI_MEAS("DHT %d: %.2f°C %.0f%%", idx, temperature, humidity);
                }

                int16_t measured_temp = (int16_t)lroundf(temperature * 100.0f);
                uint16_t measured_hum = (uint16_t)lroundf(humidity * 100.0f);
                uint8_t ep = Config.Zigbee.Dht.Endpoints[idx];

                zb_write_dht(ep, measured_temp, measured_hum, pre_start);
                }
#ifndef DUMMY_SENSOR_MODE
            } else {
                diag_inc(g_diag.dht_fail[idx]);
                s_dht_good[idx] = 0;
                s_dht_fail[idx] = sat_inc(s_dht_fail[idx]);
                if (s_dht_fail[idx] >= Config.Health.FailThreshold) {
                    health_set(HM_SENSOR, HF_SENS_DHT(idx));
                    // sensor unavailable: report the ZCL invalid values (once per streak)
                    if (!s_dht_inv[idx]) {
                        s_dht_inv[idx] = zb_write_dht(Config.Zigbee.Dht.Endpoints[idx], ZCL_INVALID_TEMP, ZCL_INVALID_HUM, pre_start);
                    }
                }
                // log the first failure of a streak and then every N-th one only
                if (s_dht_fail[idx] == 1 || (s_dht_fail[idx] % Config.Logging.DhtFailLogEvery) == 0) {
                    LOGW_MEAS("DHT %d read failed: %s (streak=%u ok=%u fail=%u)",
                             idx, esp_err_to_name(dht_err), (unsigned)s_dht_fail[idx],
                             (unsigned)g_diag.dht_ok[idx], (unsigned)g_diag.dht_fail[idx]);
                }
            }
#endif
        }
}
