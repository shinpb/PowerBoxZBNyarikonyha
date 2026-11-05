// Sys
#include "esp_log.h"
#include "esp_timer.h"      // esp_timer_get_time()
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
// External Libraries
#include "esp_zigbee.h"
#include "ezbee/zha.h"
#include "ezbee/zcl/cluster/binary_input.h"
#include "ina3221.h"        // INA3221_BUS_NUMBER
// Internal components
#include "zigbee_reporting.h"
#include "zcl_values.h"
#include "heartbeat.h"
#include "config.h"

// ---- Firmware default reporting (D3/N4, Config.Reporting) ----
// Written ONCE, on a factory-new start before the first join, by reporting_defaults_task (below). One update takes
// ~0.5 s (all 52 records: ~26 s), too long for the signal handler, so the task takes the Zigbee lock for every single
// call and releases it between the records. After the join the coordinator (Z2M configure) overwrites the values.
static unsigned s_def_ok = 0, s_def_missing = 0;

static void set_default_report(const char *name, uint8_t ep, uint16_t cluster, uint16_t attr,
                               uint16_t min_s, uint16_t max_s, const ezb_zcl_attr_variable_t *delta)
{
    esp_zigbee_lock_acquire(portMAX_DELAY);
    ezb_zcl_reporting_info_t info = ezb_zcl_reporting_info_find(ep, cluster, EZB_ZCL_CLUSTER_SERVER, attr, 0);
    const bool found = (info != EZB_ZCL_INVALID_REPORTING_INFO);
    ezb_err_t r = EZB_ERR_NONE;
    if (found) r = ezb_zcl_reporting_info_update(info, min_s, max_s, delta);
    esp_zigbee_lock_release();

    if (!found) {
        s_def_missing++;
        LOGW_REP("%s ep=0x%02X: NO reporting record", name, ep);
    } else if (r != EZB_ERR_NONE) {
        s_def_missing++;
        LOGW_REP("%s ep=0x%02X: update failed ret=%d", name, ep, (int)r);
    } else {
        s_def_ok++;
        if (Config.Logging.ReportUpdates) LOGI_REP("%s ep=0x%02X: update ret=%d", name, ep, (int)r);
    }
}

static int16_t scaled_s16(float v, float mul) { return (int16_t)lroundf(v * mul); }

// ---- The list of reportable attributes (single definition, used by the defaults and by the Default Response off) ----
enum rec_kind_t { RK_BOARD, RK_DHT_T, RK_DHT_H, RK_DC_V, RK_DC_I, RK_DC_P, RK_FAULT, RK_FAULT_MASK };

struct rec_t {
    const char *name;
    uint8_t     ep;
    uint16_t    cluster;
    uint16_t    attr;
    rec_kind_t  kind;
};

// Calls f(const rec_t &) for every reportable attribute of the device (52 records: board 1 + fault 2 + DHT 2x2 + DC 15x3).
template <typename F>
static void for_each_record(F &&f)
{
    const uint8_t ep_b = Config.Zigbee.Board.Endpoint;
    f(rec_t{"board temp", ep_b, EZB_ZCL_CLUSTER_ID_TEMPERATURE_MEASUREMENT,
            EZB_ZCL_ATTR_TEMPERATURE_MEASUREMENT_MEASURED_VALUE_ID, RK_BOARD});
    for (int i = 0; i < Config.Sensor.Dht.Count; i++) {
        const uint8_t ep = Config.Zigbee.Dht.Endpoints[i];
        f(rec_t{"dht temp", ep, EZB_ZCL_CLUSTER_ID_TEMPERATURE_MEASUREMENT,
                EZB_ZCL_ATTR_TEMPERATURE_MEASUREMENT_MEASURED_VALUE_ID, RK_DHT_T});
        f(rec_t{"dht hum", ep, EZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
                EZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_MEASURED_VALUE_ID, RK_DHT_H});
    }
    for (int i = 0; i < Config.Sensor.Ina.Count; i++) {
        for (int ch = 0; ch < INA3221_BUS_NUMBER; ch++) {
            const uint8_t ep = Config.Zigbee.Ina.Endpoints[i][ch];
            f(rec_t{"dc V", ep, EZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT,
                    EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_VOLTAGE_ID, RK_DC_V});
            f(rec_t{"dc I", ep, EZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT,
                    EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_CURRENT_ID, RK_DC_I});
            f(rec_t{"dc P", ep, EZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT,
                    EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_POWER_ID, RK_DC_P});
        }
    }
    f(rec_t{"fault", ep_b, EZB_ZCL_CLUSTER_ID_BINARY_INPUT,
            EZB_ZCL_ATTR_BINARY_INPUT_PRESENT_VALUE_ID, RK_FAULT});
    f(rec_t{"fault mask", ep_b, ZB_DIAG_CLUSTER_ID, ZB_DIAG_ATTR_FAULT_MASK, RK_FAULT_MASK});
}

// Reporting parameters of a record from Config.Reporting. false = the global or the category switch is off.
static bool report_params(rec_kind_t kind, uint16_t &min_s, uint16_t &max_s, ezb_zcl_attr_variable_t &delta)
{
    const auto &RP = Config.Reporting;
    delta = {};   // discrete attributes (bool, bitmap32): any change is reported, the delta is not used
    if (!RP.Enabled) return false;
    switch (kind) {
    case RK_BOARD:
        if (!RP.Board.Enabled) return false;
        min_s = RP.Board.MinInterval; max_s = RP.Board.MaxInterval;
        delta.s16 = scaled_s16(RP.Board.ThresholdTemp, 100);
        return true;
    case RK_DHT_T:
        if (!RP.Dht.Enabled) return false;
        min_s = RP.Dht.MinInterval; max_s = RP.Dht.MaxInterval;
        delta.s16 = scaled_s16(RP.Dht.ThresholdTemp, 100);
        return true;
    case RK_DHT_H:
        if (!RP.Dht.Enabled) return false;
        min_s = RP.Dht.MinInterval; max_s = RP.Dht.MaxInterval;
        delta.u16 = (uint16_t)lroundf(RP.Dht.ThresholdHum * 100);
        return true;
    case RK_DC_V:
    case RK_DC_I:
    case RK_DC_P:
        if (!RP.Dc.Enabled) return false;
        min_s = RP.Dc.MinInterval; max_s = RP.Dc.MaxInterval;
        // raw units follow the divisors of the endpoints (ZCL_DC_DIV_*)
        if (kind == RK_DC_V)      delta.s16 = scaled_s16(RP.Dc.ThresholdVolt, ZCL_DC_DIV_V);
        else if (kind == RK_DC_I) delta.s16 = scaled_s16(RP.Dc.ThresholdCur, ZCL_DC_DIV_I);
        else                      delta.s16 = scaled_s16(RP.Dc.ThresholdPwr, ZCL_DC_DIV_P);
        return true;
    case RK_FAULT:
    case RK_FAULT_MASK:
        if (!RP.Fault.Enabled) return false;
        min_s = RP.Fault.MinInterval; max_s = RP.Fault.MaxInterval;
        return true;
    }
    return false;
}

// esp-zigbee-lib >= 2.0.5: disable the Default Response for the automatic reports of one attribute
// (stops the Default Response flood seen in Z2M, PLAN.md 4.3/4.5). Counted for the summary log line.
static uint16_t s_dis_rsp_ok = 0, s_dis_rsp_missing = 0;

static void disable_default_rsp(const rec_t &r)
{
    ezb_zcl_reporting_info_t info = ezb_zcl_reporting_info_find(r.ep, r.cluster, EZB_ZCL_CLUSTER_SERVER, r.attr, 0);
    if (info == EZB_ZCL_INVALID_REPORTING_INFO) { s_dis_rsp_missing++; return; }
    if (ezb_zcl_reporting_info_set_dis_def_rsp(info, true) == EZB_ERR_NONE) s_dis_rsp_ok++;
    else s_dis_rsp_missing++;
}

// Writes the default reporting of every category enabled in Config.Reporting (global + per category switch).
static void write_default_reporting()
{
    s_def_ok = s_def_missing = 0;
    for_each_record([](const rec_t &r) {
        uint16_t min_s = 0, max_s = 0;
        ezb_zcl_attr_variable_t delta;
        if (report_params(r.kind, min_s, max_s, delta)) {
            set_default_report(r.name, r.ep, r.cluster, r.attr, min_s, max_s, &delta);
        }
    });
}

// Default Response off for every reportable attribute (records that do not exist yet are only counted).
// Runs at EVERY start: it is quick (no per-record delay like the updates above) and nothing says the flag
// survives a reboot.
void zb_disable_default_rsp()
{
    s_dis_rsp_ok = s_dis_rsp_missing = 0;
    for_each_record([](const rec_t &r) { disable_default_rsp(r); });
    LOGI_REP("Default Response disabled: %u records ok, %u without record",
             (unsigned)s_dis_rsp_ok, (unsigned)s_dis_rsp_missing);
}

// Factory-new start: write the firmware defaults first, then start the steering (so the coordinator's configure
// after the join always comes AFTER the defaults and overwrites them, never the other way round).
static volatile bool s_defaults_done = false;   // the task has finished (and started the steering)

static void reporting_defaults_task(void *pv)
{
    const int64_t t0 = esp_timer_get_time();
    write_default_reporting();
    g_diag.rep_stack_free = (uint32_t)uxTaskGetStackHighWaterMark(NULL) + 1;   // +1: 0 = the task has not run
    LOGI_REP("Defaults written: %u ok, %u missing/failed, %lld ms, stack_free=%u",
             s_def_ok, s_def_missing, (long long)((esp_timer_get_time() - t0) / 1000), (unsigned)(g_diag.rep_stack_free - 1));

    esp_zigbee_lock_acquire(portMAX_DELAY);
    zb_disable_default_rsp();   // again, after the updates (an update may reset the flag; harmless otherwise)
    if (ezb_bdb_is_factory_new()) {
        (void)ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_NETWORK_STEERING);
    }
    esp_zigbee_lock_release();
    g_diag.rep_stack_free = (uint32_t)uxTaskGetStackHighWaterMark(NULL) + 1;
    s_defaults_done = true;
    vTaskDelete(NULL);
}

// true: the task was started and it will start the steering; false: nothing to write, the caller starts the steering.
bool zb_start_reporting_defaults()
{
    static bool started = false;
    const auto &RP = Config.Reporting;
    if (started) return !s_defaults_done;   // still running: it starts the steering; already done: the caller must
    if (!RP.Enabled || !(RP.Board.Enabled || RP.Dht.Enabled || RP.Dc.Enabled || RP.Fault.Enabled)) {
        LOGI_REP("Firmware default reporting is off (config)");
        return false;
    }
    if (xTaskCreate(reporting_defaults_task, "Rep_defaults", Config.Tasks.ReportDefaults.Stack, NULL, Config.Tasks.ReportDefaults.Prio, NULL) != pdPASS) {
        LOGW_REP("defaults task not started, the coordinator has to configure the reporting");
        return false;
    }
    started = true;
    return true;
}

