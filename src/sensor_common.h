#pragma once
// Helpers shared by the sensor modules (sensor_board / sensor_dht / sensor_ina / sensor_manager).
//#define DUMMY_SENSOR_MODE 1   // simulated sensor values instead of the real hardware (development without sensors)
#include <stdint.h>
#include "esp_err.h"
#include "esp_log.h"
#include "config.h"

// Saturating counter increment (consecutive failures).
static inline uint8_t sat_inc(uint8_t v) { return v < 255 ? v + 1 : v; }

// Sensor init failures do not abort the firmware (the Zigbee side keeps running);
// Config.Health.AbortOnInitFail = 1 restores the old abort for development.
static inline void init_failed(const char *what, esp_err_t e) {
    LOGE_INIT_SENSOR("%s failed: %s", what, esp_err_to_name(e));
    if (Config.Health.AbortOnInitFail) ESP_ERROR_CHECK(e);
}
