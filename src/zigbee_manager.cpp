// Sys
#include "esp_log.h"
#include "esp_system.h"     // esp_restart()
#include "esp_timer.h"      // esp_timer_get_time()
#include "nvs.h"
#include "esp_app_desc.h" // esp_app_get_description()

// External Libraries
#include "esp_zigbee.h"
#include "ezbee/zha.h"
#include "ezbee/zcl/cluster/binary_input.h"
#include "ezbee/zcl/cluster/custom.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <math.h>
#include "ina3221.h"

// Internal components
#include "zigbee_manager.h"
#include "sensor_manager.h"
#include "health.h"
#include "heartbeat.h"
#include "config.h"


#include "alarm_timer.h"

// Component internal functions
static void init_zb_basic_cluster(ezb_af_ep_desc_t cluster_list);
static void init_zb_ep_dht(ezb_af_device_desc_t dev_desc);
static void init_zb_ep_dc(ezb_af_device_desc_t dev_desc);
static bool esp_zigbee_app_signal_handler(const ezb_app_signal_t *app_signal);
static void zigbee_mainloop_task(void *pvParameters);
static void init_zb_storage();
static void init_zb_ep_board(ezb_af_device_desc_t dev_desc);
static void add_temp_cluster(ezb_af_ep_desc_t ep_desc, float min_c, float max_c);
static void add_diag_clusters(ezb_af_ep_desc_t ep_desc);
static uint64_t millis();

// Shared variable single instance
volatile bool is_zigbee_connected = false;


// values
// Diagnostic attributes (4.8): backing storage, written through ezb_zcl_set_attr_value()
static bool     diag_present_value = false;
static uint8_t  diag_status_flags  = 0;
static uint32_t diag_fault_mask    = 0;

static int16_t dc_val_v[n_INA][INA3221_BUS_NUMBER] = {0};
static int16_t dc_val_i[n_INA][INA3221_BUS_NUMBER] = {0};
static int16_t dc_val_p[n_INA][INA3221_BUS_NUMBER] = {0};


// Divisor attribute storage (values: ZCL_DC_DIV_* in sensor_manager.h; the stack keeps the pointers)
static uint16_t div_v_arr[n_INA][INA3221_BUS_NUMBER]    = {0};
static uint16_t div_i_arr[n_INA][INA3221_BUS_NUMBER]    = {0};
static uint16_t div_p_arr[n_INA][INA3221_BUS_NUMBER]    = {0};


static ezb_af_ep_desc_t create_ep(uint8_t ep, uint16_t device_id) {
    ezb_af_ep_config_t cfg = {};
    cfg.ep_id              = ep;
    cfg.app_profile_id     = EZB_AF_HA_PROFILE_ID;
    cfg.app_device_id      = device_id;   // DHT: Temperature Sensor (0x0302), INA: Simple Sensor (0x000C)
    cfg.app_device_version = 0;
    return ezb_af_create_endpoint_desc(&cfg);
}

// ZCL character string (length-prefixed), max 16 chars for SWBuildID / DateCode.
// Truncates `src` to 16 chars. `buf` must be static (the stack keeps the pointer).
static void zcl_str16(uint8_t buf[17], const char *src) {
    size_t n = strnlen(src, 16);
    buf[0] = (uint8_t)n;
    memcpy(buf + 1, src, n);
}

void init_zb_basic_cluster(ezb_af_ep_desc_t cluster_list) {
    LOGI_ZB("init_zb_basic_cluster");
    // ZCLVersion and PowerSource are mandatory attributes: the stack creates them from this config
    // (add_attr on them is not needed). zcl_version 8 = the library default read by Z2M so far.
    ezb_zcl_basic_cluster_server_config_t basic_cfg = {};
    basic_cfg.zcl_version  = 8;
    basic_cfg.power_source = Config.Device.PowerSource;
    ezb_zcl_cluster_desc_t basic_desc = ezb_zcl_basic_create_cluster_desc(&basic_cfg, EZB_ZCL_CLUSTER_SERVER);

    ezb_zcl_basic_cluster_desc_add_attr(basic_desc, EZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, (void *)Config.Device.Manufacturer);
    ezb_zcl_basic_cluster_desc_add_attr(basic_desc, EZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, (void *)Config.Device.Model);

    // Optional attribute values MUST be static so the pointers survive the function exit

    // Attribute IDs: EZB_ZCL_ATTR_BASIC_* (ezbee/zcl/cluster/basic_desc.h, verified in 2.0.4).
    static uint8_t app_version   = Config.Device.AppVersion;
    static uint8_t stack_version = Config.Device.StackVersion;
    static uint8_t hw_version    = Config.Device.HwVersion;
    ezb_zcl_basic_cluster_desc_add_attr(basic_desc, EZB_ZCL_ATTR_BASIC_APPLICATION_VERSION_ID, &app_version);
    ezb_zcl_basic_cluster_desc_add_attr(basic_desc, EZB_ZCL_ATTR_BASIC_STACK_VERSION_ID, &stack_version);
    ezb_zcl_basic_cluster_desc_add_attr(basic_desc, EZB_ZCL_ATTR_BASIC_HW_VERSION_ID, &hw_version);

    // SWBuildID = PROJECT_VER (short git hash, set in root CMakeLists.txt), truncated to 16 chars
    static uint8_t sw_build_id[17];
    zcl_str16(sw_build_id, esp_app_get_description()->version);
    ezb_zcl_basic_cluster_desc_add_attr(basic_desc, EZB_ZCL_ATTR_BASIC_SW_BUILD_ID_ID, sw_build_id);

    // DateCode = compile date ("Oct 28 2026", 11 chars)
    static uint8_t date_code[17];
    zcl_str16(date_code, __DATE__);
    ezb_zcl_basic_cluster_desc_add_attr(basic_desc, EZB_ZCL_ATTR_BASIC_DATE_CODE_ID, date_code);

    // One-time info log of the Basic values (identification check against Z2M).
    // ZCL strings are length-prefixed: byte 0 = length, text starts at byte 1.
    LOGI_ZB("Basic: mfr=%.*s model=%.*s powerSrc=0x%02X appVer=%u stackVer=%u hwVer=%u swBuildId=%.*s dateCode=%.*s",
             (int)Config.Device.Manufacturer[0], Config.Device.Manufacturer + 1,
             (int)Config.Device.Model[0], Config.Device.Model + 1,
             (unsigned)Config.Device.PowerSource, (unsigned)app_version, (unsigned)stack_version, (unsigned)hw_version,
             (int)sw_build_id[0], (const char *)sw_build_id + 1,
             (int)date_code[0], (const char *)date_code + 1);

    ezb_af_endpoint_add_cluster_desc(cluster_list, basic_desc);
}


void init_zb_ep_dht(ezb_af_device_desc_t dev_desc) {
    // Each DHT sensor
    for (int idx = 0; idx < Config.Sensor.Dht.Count; idx++) {
        uint8_t ep = Config.Zigbee.Dht.Endpoints[idx];
        LOGI_ZB("Create EP: DHT %d (EP 0x%X)", idx, ep);

        // Endpoint
        ezb_af_ep_desc_t cluster_list = create_ep(ep, EZB_ZHA_TEMPERATURE_SENSOR_DEVICE_ID);
        if (!cluster_list) { LOGE_ZB("EP 0x%X create failed", ep); continue; }


        // Temperature cluster
        add_temp_cluster(cluster_list, Config.Sensor.Dht.MinTemp, Config.Sensor.Dht.MaxTemp);

        // Humidity cluster
        ezb_zcl_rel_humidity_measurement_cluster_config_t hum_cfg = {
            .measured_value = ZCL_INVALID_HUM, // invalid until the first measurement
            .min_measured_value = (uint16_t)(Config.Sensor.Dht.MinHum * 100),
            .max_measured_value = (uint16_t)(Config.Sensor.Dht.MaxHum * 100)
        };
        ezb_zcl_cluster_desc_t hum_cluster = ezb_zcl_rel_humidity_measurement_create_cluster_desc(&hum_cfg, EZB_ZCL_CLUSTER_SERVER);

        ezb_af_endpoint_add_cluster_desc(cluster_list, hum_cluster);

        ezb_af_device_add_endpoint_desc(dev_desc, cluster_list);
    }
}

void init_zb_ep_dc(ezb_af_device_desc_t dev_desc) {
    // Each Modul sensor
    for (int idx = 0; idx < Config.Sensor.Ina.Count; idx++) {
        // Each channel
        for (uint8_t channel = 0; channel < INA3221_BUS_NUMBER; channel++) {
            
            // ZB Endpiont ID
            uint8_t ep = Config.Zigbee.Ina.Endpoints[idx][channel];
            LOGI_ZB("Create EP: DC %d/%d (EP 0x%X)", idx, channel, ep);
            
            // Endpoint
            ezb_af_ep_desc_t cluster_list = create_ep(ep, EZB_ZHA_SIMPLE_SENSOR_DEVICE_ID);
            if (!cluster_list) { LOGE_ZB("EP 0x%X create failed", ep); continue; }


            // Electrical measurement cluster
            ezb_zcl_electrical_measurement_cluster_config_t em_cfg = {
                .measurement_type = EZB_ZCL_ELECTRICAL_MEASUREMENT_MEASUREMENT_TYPE_DC_MEASUREMENT
            };
            ezb_zcl_cluster_desc_t electrical_cluster = ezb_zcl_electrical_measurement_create_cluster_desc(&em_cfg, EZB_ZCL_CLUSTER_SERVER);

            // No measurement yet = invalid value (not 0: 0 V / 0 A would be a plausible reading)
            dc_val_v[idx][channel] = ZCL_INVALID_DC;
            dc_val_i[idx][channel] = ZCL_INVALID_DC;
            dc_val_p[idx][channel] = ZCL_INVALID_DC;

            div_v_arr[idx][channel] = ZCL_DC_DIV_V;
            div_i_arr[idx][channel] = ZCL_DC_DIV_I;
            div_p_arr[idx][channel] = ZCL_DC_DIV_P;

            ezb_zcl_electrical_measurement_cluster_desc_add_attr(electrical_cluster, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_VOLTAGE_ID, &dc_val_v[idx][channel]);
            ezb_zcl_electrical_measurement_cluster_desc_add_attr(electrical_cluster, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_VOLTAGE_DIVISOR_ID, &div_v_arr[idx][channel]);

            ezb_zcl_electrical_measurement_cluster_desc_add_attr(electrical_cluster, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_CURRENT_ID, &dc_val_i[idx][channel]);
            ezb_zcl_electrical_measurement_cluster_desc_add_attr(electrical_cluster, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_CURRENT_DIVISOR_ID, &div_i_arr[idx][channel]);

            ezb_zcl_electrical_measurement_cluster_desc_add_attr(electrical_cluster, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_POWER_ID, &dc_val_p[idx][channel]);
            ezb_zcl_electrical_measurement_cluster_desc_add_attr(electrical_cluster, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_POWER_DIVISOR_ID, &div_p_arr[idx][channel]);

            ezb_af_endpoint_add_cluster_desc(cluster_list, electrical_cluster);
            ezb_af_device_add_endpoint_desc(dev_desc, cluster_list);
        }
    }
}

static void add_temp_cluster(ezb_af_ep_desc_t ep_desc, float min_c, float max_c) {
    ezb_zcl_temperature_measurement_cluster_config_t cfg = {
        .measured_value     = ZCL_INVALID_TEMP,   // invalid until the first measurement
        .min_measured_value = (int16_t)(min_c * 100),
        .max_measured_value = (int16_t)(max_c * 100)
    };
    ezb_zcl_cluster_desc_t c = ezb_zcl_temperature_measurement_create_cluster_desc(&cfg, EZB_ZCL_CLUSTER_SERVER);
    ezb_af_endpoint_add_cluster_desc(ep_desc, c);
}

void init_zb_ep_board(ezb_af_device_desc_t dev_desc) {
    uint8_t ep = Config.Zigbee.Board.Endpoint;
    LOGI_ZB("Create EP: Board (EP 0x%X)", ep);
    ezb_af_ep_desc_t cluster_list = create_ep(ep, EZB_ZHA_TEMPERATURE_SENSOR_DEVICE_ID);
    if (!cluster_list) { LOGE_ZB("EP 0x%X create failed", ep); return; }

    init_zb_basic_cluster(cluster_list);
    add_temp_cluster(cluster_list, Config.Sensor.Board.MinTemp, Config.Sensor.Board.MaxTemp);
    add_diag_clusters(cluster_list);
    ezb_af_device_add_endpoint_desc(dev_desc, cluster_list);
}

static void diag_cluster_noop(uint8_t) {}

// Device fault indication for the coordinator (4.8)
static void add_diag_clusters(ezb_af_ep_desc_t ep_desc) {
    // Standard Binary Input: PresentValue = any fault active
    ezb_zcl_binary_input_cluster_server_config_t bi_cfg = {
        .out_of_service = false,
        .present_value  = false,
        .status_flags   = 0,
    };
    ezb_zcl_cluster_desc_t bi = ezb_zcl_binary_input_create_cluster_desc(&bi_cfg, EZB_ZCL_CLUSTER_SERVER);
    ezb_af_endpoint_add_cluster_desc(ep_desc, bi);

    // Manufacturer-specific cluster with the fault mask
    ezb_zcl_custom_cluster_config_t cc_cfg = {
        .cluster_id  = ZB_DIAG_CLUSTER_ID,
        .init_func   = diag_cluster_noop,
        .deinit_func = diag_cluster_noop,
    };
    ezb_zcl_cluster_desc_t cc = ezb_zcl_custom_create_cluster_desc(&cc_cfg, EZB_ZCL_CLUSTER_SERVER);
    ezb_err_t r = ezb_zcl_custom_cluster_desc_add_attr(cc, ZB_DIAG_ATTR_FAULT_MASK, EZB_ZCL_ATTR_TYPE_MAP32,
                                                       EZB_ZCL_ATTR_ACCESS_READ_ONLY | EZB_ZCL_ATTR_ACCESS_REPORTING,
                                                       &diag_fault_mask);
    if (r != EZB_ERR_NONE) LOGW_ZB("diag cluster add_attr ret=%d", (int)r);
    ezb_af_endpoint_add_cluster_desc(ep_desc, cc);
}

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
        if (Config.Logging.EnabledReports) LOGI_REP("%s ep=0x%02X: update ret=%d", name, ep, (int)r);
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
static void init_zb_disable_default_rsp()
{
    s_dis_rsp_ok = s_dis_rsp_missing = 0;
    for_each_record([](const rec_t &r) { disable_default_rsp(r); });
    LOGI_REP("Default Response disabled: %u records ok, %u without record",
             (unsigned)s_dis_rsp_ok, (unsigned)s_dis_rsp_missing);
}

// Factory-new start: write the firmware defaults first, then start the steering (so the coordinator's configure
// after the join always comes AFTER the defaults and overwrites them, never the other way round).
static void reporting_defaults_task(void *pv)
{
    (void)pv;
    const int64_t t0 = esp_timer_get_time();
    write_default_reporting();
    LOGI_REP("Defaults written: %u ok, %u missing/failed, %lld ms",
             s_def_ok, s_def_missing, (long long)((esp_timer_get_time() - t0) / 1000));

    esp_zigbee_lock_acquire(portMAX_DELAY);
    init_zb_disable_default_rsp();   // again, after the updates (an update may reset the flag; harmless otherwise)
    if (ezb_bdb_is_factory_new()) {
        (void)ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_NETWORK_STEERING);
    }
    esp_zigbee_lock_release();
    vTaskDelete(NULL);
}

// true: the task was started and it will start the steering; false: nothing to write, the caller starts the steering.
static bool start_reporting_defaults()
{
    static bool started = false;
    const auto &RP = Config.Reporting;
    if (started) return true;
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

void init_zb_stack() {

   init_zb_storage();

    LOGI_ZB("Start ZigBee main thread");
    xTaskCreate(zigbee_mainloop_task, "Zigbee_main", Config.Tasks.ZigbeeMain.Stack, NULL, Config.Tasks.ZigbeeMain.Prio, NULL);
}

void init_zb_storage() {
    LOGI_ZB("Initializing NVS");
    // 1. Initialize Default NVS (Required for PHY/MAC)
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

}


static void esp_zigbee_zcl_core_action_handler(ezb_zcl_core_action_callback_id_t callback_id, void *message)
{
    switch (callback_id) {
    case EZB_ZCL_CORE_DEFAULT_RSP_CB_ID: {
        ezb_zcl_cmd_default_rsp_message_t *default_rsp = (ezb_zcl_cmd_default_rsp_message_t *)message;
        // Counted (heartbeat "zb:" line) instead of logged: this was ~56% of the serial log.
        // Only a non-zero status is worth a log line; successful ones are visible at debug level.
        diag_inc(g_diag.zb_default_rsp);
        if (default_rsp->in.status_code != 0) {
            diag_inc(g_diag.zb_default_rsp_err);
            LOGW_ZB("Received ZCL Default Response: status(0x%02x)", default_rsp->in.status_code);
        } else {
            LOGD_ZB("Received ZCL Default Response: status(0x%02x)", default_rsp->in.status_code);
        }
    } break;
    default:
        LOGW_ZB("ZCL Core Action: ID(0x%04lx)", callback_id);
        break;
    }
}

static void esp_zigbee_alarm_bdb_commissioning(alarm_timer_arg_t arg)
{
    esp_zigbee_lock_acquire(portMAX_DELAY);
    (void)ezb_bdb_start_top_level_commissioning(arg);
    esp_zigbee_lock_release();
}

static void esp_zigbee_alarm_steer_if_factory_new(alarm_timer_arg_t arg)
{
    esp_zigbee_lock_acquire(portMAX_DELAY);
    if (ezb_bdb_is_factory_new()) {
        (void)ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_NETWORK_STEERING);
    }
    esp_zigbee_lock_release();
}

static bool esp_zigbee_app_signal_handler(const ezb_app_signal_t *app_signal)
{
    ezb_app_signal_type_t signal_type = ezb_app_signal_get_type(app_signal);

    switch (signal_type) {
    case EZB_ZDO_SIGNAL_SKIP_STARTUP:
        LOGI_ZB("Initialize Zigbee stack");
        ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_INITIALIZATION);
        break;
    case EZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case EZB_BDB_SIGNAL_DEVICE_REBOOT: {
        ezb_bdb_comm_status_t status = *((ezb_bdb_comm_status_t *)ezb_app_signal_get_params(app_signal));
        if (status == EZB_BDB_STATUS_SUCCESS) {
            LOGI_ZB("Device started up in%s factory-reset mode", ezb_bdb_is_factory_new() ? "" : " non");
            health_clear(HM_ZIGBEE, HF_ZB_START_FAILED);
            // Valid attribute values must exist before the stack sends its first reports
            // (otherwise the invalid initial 0x8000/0xFFFF values go out after every start).
            measure_all_once();
            // D7: the registration (long NVS iteration) is over and the stack runs, the measurement tasks may start
            start_measurement_tasks();
            // D3/N4: the Default Response is disabled at EVERY start (quick); the firmware default intervals and
            // thresholds are written only on a factory-new start (below). A plain reboot must not overwrite what
            // Z2M configured (the stack keeps the records across reboots, PLAN.md R1 - verified by the reboot test).
            init_zb_disable_default_rsp();
            if (ezb_bdb_is_factory_new()) {
                health_set(HM_ZIGBEE, HF_ZB_NOT_JOINED);        // set first: no low glitch on the pin
                health_clear(HM_ZIGBEE, HF_STARTING);
                // the defaults task writes the records (~26 s for all of them) and then starts the steering itself
                if (!start_reporting_defaults()) {
                    ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_NETWORK_STEERING);
                }
            } else {
                LOGI_REP("Reporting defaults not written (configured device, records kept)");
                health_clear(HM_ZIGBEE, HF_ZB_NOT_JOINED | HF_STARTING);
                is_zigbee_connected = true;
                LOGI_ZB("Device reboot");
            }
        } else {
            LOGW_ZB("%s failed with status(0x%02x), please retry", ezb_app_signal_to_string(signal_type), status);
            diag_inc(g_diag.zb_start_fail);
            health_set(HM_ZIGBEE, HF_ZB_START_FAILED);
            alarm_timer_schedule(esp_zigbee_alarm_bdb_commissioning, EZB_BDB_MODE_INITIALIZATION, 1000);
        }
    } break;
    case EZB_BDB_SIGNAL_STEERING: {
        ezb_bdb_comm_status_t status = *((ezb_bdb_comm_status_t *)ezb_app_signal_get_params(app_signal));
        if (status == EZB_BDB_STATUS_SUCCESS) {
            is_zigbee_connected = true;
            diag_inc(g_diag.zb_join_ok);
            health_clear(HM_ZIGBEE, HF_ZB_NOT_JOINED);
            ezb_extpanid_t extended_pan_id;
            ezb_nwk_get_extended_panid(&extended_pan_id);
            LOGI_ZB("Joined network successfully: PAN ID(0x%04hx, EXT: 0x%llx), Channel(%d), Short Address(0x%04hx)",
                     ezb_nwk_get_panid(), extended_pan_id.u64, ezb_nwk_get_current_channel(), ezb_nwk_get_short_address());
        } else {
            LOGW_ZB("Failed to join network with status(0x%02x)", status);
            diag_inc(g_diag.zb_join_fail);
            health_set(HM_ZIGBEE, HF_ZB_NOT_JOINED);
            alarm_timer_schedule(esp_zigbee_alarm_bdb_commissioning, EZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
    } break;
    case EZB_ZDO_SIGNAL_DEVICE_ANNCE: {
        const ezb_zdo_signal_device_annce_params_t *dev_annce_params = (ezb_zdo_signal_device_annce_params_t *)ezb_app_signal_get_params(app_signal);
        LOGI_ZB("New device commissioned or rejoined (short: 0x%04hx)", dev_annce_params->short_addr);
    } break;
    case EZB_ZDO_SIGNAL_LEAVE: {
        is_zigbee_connected = false;
        diag_inc(g_diag.zb_leave);
        health_set(HM_ZIGBEE, HF_ZB_NOT_JOINED);
        const ezb_zdo_signal_leave_params_t *leave_params = (ezb_zdo_signal_leave_params_t *)ezb_app_signal_get_params(app_signal);
        LOGI_ZB("Left network successfully with type(0x%02x)", leave_params->leave_type);
        alarm_timer_schedule(esp_zigbee_alarm_steer_if_factory_new, 0, 2000);
    } break;
    case EZB_NWK_SIGNAL_PERMIT_JOIN_STATUS: {
        uint8_t duration = *(uint8_t *)ezb_app_signal_get_params(app_signal);
        if (duration) {
            LOGI_ZB("Network(0x%04hx) is open for %d seconds", ezb_nwk_get_panid(), duration);
        } else {
            LOGW_ZB("Network(0x%04hx) closed, devices joining not allowed.", ezb_nwk_get_panid());
        }
    } break;
    default:
        LOGI_ZB("Zigbee APP Signal: %s(type: 0x%02x)", ezb_app_signal_to_string(signal_type), signal_type);
        break;
    }
    return true;
}

esp_err_t esp_zigbee_setup_commissioning(void)
{
    ezb_aps_secur_enable_distributed_security(false);
    ESP_ERROR_CHECK(ezb_bdb_set_primary_channel_set(ESP_ZIGBEE_PRIMARY_CHANNEL_MASK));
    ESP_ERROR_CHECK(ezb_bdb_set_secondary_channel_set(ESP_ZIGBEE_SECONDARY_CHANNEL_MASK));
    ESP_ERROR_CHECK(ezb_app_signal_add_handler(esp_zigbee_app_signal_handler));

    return ESP_OK;
}

void zigbee_mainloop_task(void *pvParameters) {

    esp_zigbee_config_t config = ESP_ZIGBEE_DEFAULT_CONFIG();
    ESP_ERROR_CHECK(esp_zigbee_init(&config));

    LOGI_ZB("ezb_config_memory");
   ezb_mem_config_t mem_config = {};
    mem_config.aps_bind_table_src_size = Config.Zigbee.BindTableSize;
    auto mr = ezb_config_memory(&mem_config);
    LOGI_ZB("ezb_config_memory ret=%d, heap=%u",
             (int)mr, (unsigned)esp_get_free_heap_size());

    ESP_ERROR_CHECK(esp_zigbee_setup_commissioning());

    ezb_af_device_desc_t dev_desc = ezb_af_create_device_desc();
    
    LOGI_ZB("Init Zigbee board endpoint");
    init_zb_ep_board(dev_desc);

    LOGI_ZB("Init Zigbee DHT endpoints");
    init_zb_ep_dht(dev_desc);

    LOGI_ZB("Init Zigbee DC endpoints");
    init_zb_ep_dc(dev_desc);

    LOGI_ZB("Registering Endpoints");
    int64_t t0 = esp_timer_get_time();
    auto rr = ezb_af_device_desc_register(dev_desc);
    LOGI_ZB("register ret=%d, %lld ms, heap=%u",
            (int)rr, (long long)((esp_timer_get_time() - t0) / 1000), (unsigned)esp_get_free_heap_size());
    if ((int)rr != 0) health_set(HM_ZIGBEE, HF_ZB_REG_FAILED);

    nvs_stats_t st;
    if (nvs_get_stats(NULL, &st) == ESP_OK) {
        LOGI_ZB("NVS used=%u free=%u total=%u ns=%u",
                (unsigned)st.used_entries, (unsigned)st.free_entries,
                (unsigned)st.total_entries, (unsigned)st.namespace_count);
    }

    ezb_zcl_core_action_handler_register(esp_zigbee_zcl_core_action_handler);

    LOGI_ZB("esp_zigbee_start");
    ESP_ERROR_CHECK(esp_zigbee_start(false));

    LOGI_ZB("esp_zigbee_launch_mainloop");
    esp_zigbee_launch_mainloop();

    esp_zigbee_deinit();

    vTaskDelete(NULL);
}


void button_task(void *pvParameters) {
    uint64_t pressStartTime = 0;
    bool isPressed = false;
    
    while (true) {
        if (gpio_get_level(Config.Device.ButtonPin) == 0) {
            if (!isPressed) {
                isPressed = true;
                pressStartTime = millis(); 
            } else if ((millis() - pressStartTime) > 3000) {
                LOGW_ZB("Resetting Zigbee to factory and rebooting in 2s.");
                esp_zigbee_lock_acquire(portMAX_DELAY);
                esp_zigbee_factory_reset();
                esp_zigbee_lock_release();
                vTaskDelay(pdMS_TO_TICKS(2000));
                esp_restart();
            }
        } else {
            isPressed = false;
        }
        
        vTaskDelay(pdMS_TO_TICKS(50)); 
    }
}

uint64_t millis() {
    return (uint64_t)(esp_timer_get_time() / 1000ULL);
}
