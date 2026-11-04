// Board temperature (ESP32-C6 internal tsens)
#include "esp_log.h"
#include "driver/temperature_sensor.h"
#include <math.h>
// Internal components
#include "sensor_common.h"
#include "sensor_manager.h"
#include "zigbee_attr.h"
#include "zcl_values.h"
#include "health.h"
#include "heartbeat.h"
#include "config.h"

static temperature_sensor_handle_t board_tsens = NULL;

// ---- Fault handling state (see health.h) ----
// Consecutive failed cycles; the error flag is set at Config.Health.FailThreshold.
static uint32_t s_board_fail = 0;
static uint32_t s_board_good = 0;   // consecutive good reads while the error flag is set (recovery hysteresis)
// "invalid value already written to the Zigbee attributes" flags (4.7): the invalid value is written once
// per failure streak (retried until the write is allowed, i.e. after the join); a good measurement clears them.
static bool    s_board_inv = false;

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

void board_cycle(bool pre_start) {
        float board_c = 0.0f;
        if (board_tsens != NULL && temperature_sensor_get_celsius(board_tsens, &board_c) == ESP_OK) {
            diag_mark(g_diag.board_last_ok_s);
            bool publish = true;
            if (s_board_fail >= Config.Health.FailThreshold) {
                // fault active: cleared (and the value published again) after RecoverThreshold good reads in a row
                s_board_good = sat_inc(s_board_good);
                if (s_board_good >= Config.Health.RecoverThreshold) {
                    health_clear(HM_SENSOR, HF_SENS_BOARD);
                    s_board_fail = 0;
                    s_board_good = 0;
                    s_board_inv = false;
                } else {
                    publish = false;
                }
            } else {
                s_board_fail = 0;
            }
            if (publish) {
                if (Config.Logging.EnabledMeasurements) {
                    LOGI_MEAS("Board: %.2f°C", board_c);
                }
                int16_t measured_board = (int16_t)lroundf(board_c * 100.0f);
                zb_write_board(measured_board, pre_start);
            }
        } else {
            s_board_good = 0;
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
}
