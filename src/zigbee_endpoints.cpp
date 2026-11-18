// Sys
#include "esp_log.h"
#include "esp_app_desc.h"   // esp_app_get_description()
// External Libraries
#include "esp_zigbee.h"
#include "ezbee/zha.h"
#include "ezbee/zcl/cluster/binary_input.h"
#include "ezbee/zcl/cluster/metering.h"
#include "ezbee/zcl/cluster/custom.h"
#include <string.h>
#include "ina3221.h"
// Internal components
#include "zigbee_endpoints.h"
#include "zcl_values.h"
#include "config.h"
#include "sensor_manager.h"

static void add_temp_cluster(ezb_af_ep_desc_t ep_desc, float min_c, float max_c);
static void add_diag_clusters(ezb_af_ep_desc_t ep_desc);

// Diagnostic attribute (4.8): backing storage, written through ezb_zcl_set_attr_value()
static uint32_t diag_fault_mask    = 0;

static int16_t dc_val_v[n_INA][INA3221_BUS_NUMBER] = {0};
static int16_t dc_val_i[n_INA][INA3221_BUS_NUMBER] = {0};
static int16_t dc_val_p[n_INA][INA3221_BUS_NUMBER] = {0};


// PZEM-004T AC meter endpoints: attribute storage (the stack keeps the pointers; written through ezb_zcl_set_attr_value)
static uint16_t ac_val_v[n_PZEM]  = {0};   // RMSVoltage
static uint16_t ac_val_i[n_PZEM]  = {0};   // RMSCurrent
static int16_t  ac_val_p[n_PZEM]  = {0};   // ActivePower
static uint16_t ac_val_f[n_PZEM]  = {0};   // ACFrequency
static int8_t   ac_val_pf[n_PZEM] = {0};   // PowerFactor
// Multiplier / divisor attributes (values: ZCL_AC_* and ZCL_METER_* in zcl_values.h)
static uint16_t ac_mul = ZCL_AC_MUL;
static uint16_t ac_div_v = ZCL_AC_DIV_V, ac_div_i = ZCL_AC_DIV_I, ac_div_p = ZCL_AC_DIV_P, ac_div_f = ZCL_AC_DIV_F;
static uint32_t meter_mul = ZCL_METER_MUL, meter_div = ZCL_METER_DIV_E;   // uint24 attributes

// Divisor attribute storage (values: ZCL_DC_DIV_* in zcl_values.h; the stack keeps the pointers)
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

static void init_zb_basic_cluster(ezb_af_ep_desc_t cluster_list) {
    LOGI_ZB("init_zb_basic_cluster");
    // ZCLVersion and PowerSource are mandatory attributes: the stack creates them from this config
    // (add_attr on them is not needed). ZCLVersion: Config.Device.ZclVersion.
    ezb_zcl_basic_cluster_server_config_t basic_cfg = {};
    basic_cfg.zcl_version  = Config.Device.ZclVersion;
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


static void init_zb_ep_dht(ezb_af_device_desc_t dev_desc) {
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

static void init_zb_ep_dc(ezb_af_device_desc_t dev_desc) {
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

// PZEM-004T: one endpoint per module with AC Electrical Measurement (V, I, P, f, PF) and Metering (energy).
// Z2M / Home Assistant get the standard clusters (haElectricalMeasurement, seMetering). No measurement yet = invalid values.
// ---- Reporting access of attributes the stack creates as read-only ----
// The stack creates PowerFactor and Metering CurrentSummationDelivered without the Reporting access flag: a Configure Reporting
// for them is answered with UNREPORTABLE_ATTRIBUTE (0x8C). The flag can only be added BEFORE the device descriptor is registered
// (after the registration ezb_zcl_attr_desc_set_access() returns EZB_ERR_INV_STATE).
static void make_reportable(ezb_zcl_cluster_desc_t cluster, uint16_t attr, const char *name) {
    ezb_zcl_attr_desc_t d = ezb_zcl_cluster_get_attr_desc(cluster, attr, EZB_ZCL_STD_MANUF_CODE);
    if (d == EZB_INVALID_ZCL_ATTR_DESC) { LOGE_ZB("%s: attribute descriptor not found", name); return; }
    const ezb_zcl_attr_access_t access = ezb_zcl_attr_desc_get_access(d);
    if (access & EZB_ZCL_ATTR_ACCESS_REPORTING) return;
    const ezb_err_t e = ezb_zcl_attr_desc_set_access(d, (ezb_zcl_attr_access_t)(access | EZB_ZCL_ATTR_ACCESS_REPORTING));
    if (e != EZB_ERR_NONE) LOGE_ZB("%s: cannot enable reporting (ret=%d)", name, (int)e);
}

static void custom_cluster_noop(uint8_t) {}
static uint8_t pzem_energy_reset_val[n_PZEM] = {0};

// Control cluster (ZB_CTRL_CLUSTER_ID) handlers (server role). The value is checked before it is stored; after the write only a request flag is
// set (this runs in the Zigbee stack context): the Modbus request is sent from the measurement task (pzem_request_energy_reset).
static ezb_zcl_status_t ctrl_cluster_check_value(uint16_t attr_id, uint8_t ep_id, void *value) {
    if (attr_id == ZB_CTRL_ATTR_ENERGY_RESET) {
        const uint8_t v = *(const uint8_t *)value;
        return (v <= 1) ? EZB_ZCL_STATUS_SUCCESS : EZB_ZCL_STATUS_INVALID_VALUE;
    }
    return EZB_ZCL_STATUS_SUCCESS;
}

static void ctrl_cluster_write_attr(uint8_t ep_id, uint16_t attr_id, void *new_value, uint16_t manuf_code) {
    if (attr_id == ZB_CTRL_ATTR_ENERGY_RESET && *(const uint8_t *)new_value == 1) pzem_request_energy_reset(ep_id);
}

static void register_ctrl_cluster_handlers() {
    ezb_zcl_custom_cluster_handlers_t h = {};
    h.cluster_id     = ZB_CTRL_CLUSTER_ID;
    h.cluster_role   = EZB_ZCL_CLUSTER_SERVER;
    h.check_value_cb = ctrl_cluster_check_value;
    h.write_attr_cb  = ctrl_cluster_write_attr;
    const ezb_err_t r = ezb_zcl_custom_cluster_handlers_register(&h);
    if (r != EZB_ERR_NONE) LOGW_ZB("ctrl cluster handlers register ret=%d", (int)r);
}

static void init_zb_ep_pzem(ezb_af_device_desc_t dev_desc) {
    for (int idx = 0; idx < Config.Sensor.Pzem.Count; idx++) {
        const uint8_t ep = Config.Zigbee.Pzem.Endpoints[idx];
        LOGI_ZB("Create EP: PZEM %d (EP 0x%X)", idx, ep);

        ezb_af_ep_desc_t cluster_list = create_ep(ep, EZB_ZHA_SIMPLE_SENSOR_DEVICE_ID);
        if (!cluster_list) { LOGE_ZB("EP 0x%X create failed", ep); continue; }

        // Electrical Measurement cluster (AC)
        ezb_zcl_electrical_measurement_cluster_config_t em_cfg = {
            .measurement_type = EZB_ZCL_ELECTRICAL_MEASUREMENT_MEASUREMENT_TYPE_ACTIVE_MEASUREMENT_AC |
                                EZB_ZCL_ELECTRICAL_MEASUREMENT_MEASUREMENT_TYPE_PHASE_A_MEASUREMENT
        };
        ezb_zcl_cluster_desc_t em = ezb_zcl_electrical_measurement_create_cluster_desc(&em_cfg, EZB_ZCL_CLUSTER_SERVER);

        ac_val_v[idx]  = ZCL_INVALID_AC_U16;
        ac_val_i[idx]  = ZCL_INVALID_AC_U16;
        ac_val_p[idx]  = ZCL_INVALID_AC_P;
        ac_val_f[idx]  = ZCL_INVALID_AC_U16;
        ac_val_pf[idx] = ZCL_INVALID_AC_PF;

        ezb_zcl_electrical_measurement_cluster_desc_add_attr(em, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_RMS_VOLTAGE_ID, &ac_val_v[idx]);
        ezb_zcl_electrical_measurement_cluster_desc_add_attr(em, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_AC_VOLTAGE_MULTIPLIER_ID, &ac_mul);
        ezb_zcl_electrical_measurement_cluster_desc_add_attr(em, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_AC_VOLTAGE_DIVISOR_ID, &ac_div_v);

        ezb_zcl_electrical_measurement_cluster_desc_add_attr(em, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_RMS_CURRENT_ID, &ac_val_i[idx]);
        ezb_zcl_electrical_measurement_cluster_desc_add_attr(em, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_AC_CURRENT_MULTIPLIER_ID, &ac_mul);
        ezb_zcl_electrical_measurement_cluster_desc_add_attr(em, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_AC_CURRENT_DIVISOR_ID, &ac_div_i);

        ezb_zcl_electrical_measurement_cluster_desc_add_attr(em, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_ACTIVE_POWER_ID, &ac_val_p[idx]);
        ezb_zcl_electrical_measurement_cluster_desc_add_attr(em, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_AC_POWER_MULTIPLIER_ID, &ac_mul);
        ezb_zcl_electrical_measurement_cluster_desc_add_attr(em, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_AC_POWER_DIVISOR_ID, &ac_div_p);

        ezb_zcl_electrical_measurement_cluster_desc_add_attr(em, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_AC_FREQUENCY_ID, &ac_val_f[idx]);
        ezb_zcl_electrical_measurement_cluster_desc_add_attr(em, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_AC_FREQUENCY_MULTIPLIER_ID, &ac_mul);
        ezb_zcl_electrical_measurement_cluster_desc_add_attr(em, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_AC_FREQUENCY_DIVISOR_ID, &ac_div_f);

        ezb_zcl_electrical_measurement_cluster_desc_add_attr(em, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_POWER_FACTOR_ID, &ac_val_pf[idx]);
        make_reportable(em, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_POWER_FACTOR_ID, "PZEM PowerFactor");
        ezb_af_endpoint_add_cluster_desc(cluster_list, em);

        // Metering cluster: CurrentSummationDelivered = energy (Wh raw, divisor 1000 = kWh)
        ezb_zcl_metering_cluster_server_config_t mt_cfg = {};
        mt_cfg.current_summation_delivered = ZCL_INVALID_ENERGY;   // invalid until the first measurement
        mt_cfg.status                      = 0;
        mt_cfg.unit_of_measure             = 0x00;                 // kWh (binary)
        mt_cfg.summation_formatting        = ZCL_METER_FORMAT;
        mt_cfg.metering_device_type        = 0x00;                 // electric metering
        ezb_zcl_cluster_desc_t mt = ezb_zcl_metering_create_cluster_desc(&mt_cfg, EZB_ZCL_CLUSTER_SERVER);
        ezb_zcl_metering_cluster_desc_add_attr(mt, EZB_ZCL_ATTR_METERING_MULTIPLIER_ID, &meter_mul);
        ezb_zcl_metering_cluster_desc_add_attr(mt, EZB_ZCL_ATTR_METERING_DIVISOR_ID, &meter_div);
        make_reportable(mt, EZB_ZCL_ATTR_METERING_CURRENT_SUMMATION_DELIVERED_ID, "PZEM energy");
        ezb_af_endpoint_add_cluster_desc(cluster_list, mt);

        // Manufacturer-specific control cluster: energy counter reset trigger (read/write uint8, write 1 = reset)
        ezb_zcl_custom_cluster_config_t cc_cfg = {
            .cluster_id  = ZB_CTRL_CLUSTER_ID,
            .init_func   = custom_cluster_noop,
            .deinit_func = custom_cluster_noop,
        };
        ezb_zcl_cluster_desc_t cc = ezb_zcl_custom_create_cluster_desc(&cc_cfg, EZB_ZCL_CLUSTER_SERVER);
        pzem_energy_reset_val[idx] = 0;
        const ezb_err_t r = ezb_zcl_custom_cluster_desc_add_attr(cc, ZB_CTRL_ATTR_ENERGY_RESET, EZB_ZCL_ATTR_TYPE_UINT8,
                                                                 EZB_ZCL_ATTR_ACCESS_READ_WRITE, &pzem_energy_reset_val[idx]);
        if (r != EZB_ERR_NONE) LOGW_ZB("PZEM %d energy reset attribute add ret=%d", idx, (int)r);
        ezb_af_endpoint_add_cluster_desc(cluster_list, cc);

        ezb_af_device_add_endpoint_desc(dev_desc, cluster_list);
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

static void init_zb_ep_board(ezb_af_device_desc_t dev_desc) {
    uint8_t ep = Config.Zigbee.Board.Endpoint;
    LOGI_ZB("Create EP: Board (EP 0x%X)", ep);
    ezb_af_ep_desc_t cluster_list = create_ep(ep, EZB_ZHA_TEMPERATURE_SENSOR_DEVICE_ID);
    if (!cluster_list) { LOGE_ZB("EP 0x%X create failed", ep); return; }

    init_zb_basic_cluster(cluster_list);
    add_temp_cluster(cluster_list, Config.Sensor.Board.MinTemp, Config.Sensor.Board.MaxTemp);
    add_diag_clusters(cluster_list);
    ezb_af_device_add_endpoint_desc(dev_desc, cluster_list);
}



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
        .init_func   = custom_cluster_noop,
        .deinit_func = custom_cluster_noop,
    };
    ezb_zcl_cluster_desc_t cc = ezb_zcl_custom_create_cluster_desc(&cc_cfg, EZB_ZCL_CLUSTER_SERVER);
    ezb_err_t r = ezb_zcl_custom_cluster_desc_add_attr(cc, ZB_DIAG_ATTR_FAULT_MASK, EZB_ZCL_ATTR_TYPE_MAP32,
                                                       EZB_ZCL_ATTR_ACCESS_READ_ONLY | EZB_ZCL_ATTR_ACCESS_REPORTING,
                                                       &diag_fault_mask);
    if (r != EZB_ERR_NONE) LOGW_ZB("diag cluster add_attr ret=%d", (int)r);
    ezb_af_endpoint_add_cluster_desc(ep_desc, cc);
}


void zb_build_endpoints(ezb_af_device_desc_t dev_desc) {
    LOGI_ZB("Init Zigbee board endpoint");
    init_zb_ep_board(dev_desc);

    LOGI_ZB("Init Zigbee DHT endpoints");
    init_zb_ep_dht(dev_desc);

    LOGI_ZB("Init Zigbee DC endpoints");
    init_zb_ep_dc(dev_desc);

    LOGI_ZB("Init Zigbee PZEM endpoints");
    init_zb_ep_pzem(dev_desc);
    if (Config.Sensor.Pzem.Count > 0) register_ctrl_cluster_handlers();
}
