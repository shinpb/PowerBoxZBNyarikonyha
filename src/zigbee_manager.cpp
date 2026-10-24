// Sys
#include "esp_log.h"
#include "esp_system.h"     // esp_restart()
#include "esp_timer.h"      // esp_timer_get_time()
#include "nvs.h"

// External Libraries
#include "esp_zigbee.h"
#include "ezbee/zha.h"
#include "nvs_flash.h"
#include "ina3221.h"

// Internal components
#include "zigbee_manager.h"
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
static uint64_t millis();

// Shared variable single instance
volatile bool is_zigbee_connected = false;


// values
static int16_t dc_val_v[n_INA][INA3221_BUS_NUMBER] = {0};
static int16_t dc_val_i[n_INA][INA3221_BUS_NUMBER] = {0};
static int16_t dc_val_p[n_INA][INA3221_BUS_NUMBER] = {0};


// Divisors: voltage mV -> 1000, current mA -> 1000, power cW -> 100
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

void init_zb_basic_cluster(ezb_af_ep_desc_t cluster_list) {
    ESP_LOGI(Config.Logging.TagZigbee, "init_zb_basic_cluster");
    ezb_zcl_cluster_desc_t basic_desc = ezb_zcl_basic_create_cluster_desc(NULL, EZB_ZCL_CLUSTER_SERVER);

    ezb_zcl_basic_cluster_desc_add_attr(basic_desc, EZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, (void *)Config.Device.Manufacturer);
    ezb_zcl_basic_cluster_desc_add_attr(basic_desc, EZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, (void *)Config.Device.Model);

    // MUST be static so the pointer survives the function exit
    static uint8_t power_source = Config.Device.PowerSource;
    ezb_zcl_basic_cluster_desc_add_attr(basic_desc, EZB_ZCL_ATTR_BASIC_POWER_SOURCE_ID, &power_source);

    ezb_af_endpoint_add_cluster_desc(cluster_list, basic_desc);
}


void init_zb_ep_dht(ezb_af_device_desc_t dev_desc) {
    // Each DHT sensor
    for (int idx = 0; idx < Config.Sensor.Dht.Count; idx++) {
        uint8_t ep = Config.Zigbee.Dht.Endpoints[idx];
        ESP_LOGI(Config.Logging.TagZigbee, "Create EP: DHT %d (EP 0x%X)", idx, ep);

        // Endpoint
        ezb_af_ep_desc_t cluster_list = create_ep(ep, EZB_ZHA_TEMPERATURE_SENSOR_DEVICE_ID);
        if (!cluster_list) { ESP_LOGE(Config.Logging.TagZigbee, "EP 0x%X create failed", ep); continue; }


        // Temperature cluster
        add_temp_cluster(cluster_list, Config.Sensor.Dht.MinTemp, Config.Sensor.Dht.MaxTemp);

        // Humidity cluster
        ezb_zcl_rel_humidity_measurement_cluster_config_t hum_cfg = {
            .measured_value = 0xFFFF, // 0xFFFF = Érvénytelen/Nincs még mérés
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
            ESP_LOGI(Config.Logging.TagZigbee, "Create EP: DC %d/%d (EP 0x%X)", idx, channel, ep);
            
            // Endpoint
            ezb_af_ep_desc_t cluster_list = create_ep(ep, EZB_ZHA_SIMPLE_SENSOR_DEVICE_ID);
            if (!cluster_list) { ESP_LOGE(Config.Logging.TagZigbee, "EP 0x%X create failed", ep); continue; }


            // Electrical measurement cluster
            ezb_zcl_electrical_measurement_cluster_config_t em_cfg = {
                .measurement_type = EZB_ZCL_ELECTRICAL_MEASUREMENT_MEASUREMENT_TYPE_DC_MEASUREMENT
            };
            ezb_zcl_cluster_desc_t electrical_cluster = ezb_zcl_electrical_measurement_create_cluster_desc(&em_cfg, EZB_ZCL_CLUSTER_SERVER);

            div_v_arr[idx][channel]    = 1000;

            div_i_arr[idx][channel]    = 1000;

            div_p_arr[idx][channel]    = 100;

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
        .measured_value     = (int16_t)0x8000,   // érvénytelen, amíg nincs mérés
        .min_measured_value = (int16_t)(min_c * 100),
        .max_measured_value = (int16_t)(max_c * 100)
    };
    ezb_zcl_cluster_desc_t c = ezb_zcl_temperature_measurement_create_cluster_desc(&cfg, EZB_ZCL_CLUSTER_SERVER);
    ezb_af_endpoint_add_cluster_desc(ep_desc, c);
}

void init_zb_ep_board(ezb_af_device_desc_t dev_desc) {
    uint8_t ep = Config.Zigbee.Board.Endpoint;
    ESP_LOGI(Config.Logging.TagZigbee, "Create EP: Board (EP 0x%X)", ep);
    ezb_af_ep_desc_t cluster_list = create_ep(ep, EZB_ZHA_TEMPERATURE_SENSOR_DEVICE_ID);
    if (!cluster_list) { ESP_LOGE(Config.Logging.TagZigbee, "EP 0x%X create failed", ep); return; }

    init_zb_basic_cluster(cluster_list);
    add_temp_cluster(cluster_list, Config.Sensor.Board.MinTemp, Config.Sensor.Board.MaxTemp);
    ezb_af_device_add_endpoint_desc(dev_desc, cluster_list);
}

static void set_default_report(const char *name, uint8_t ep, uint16_t cluster, uint16_t attr,
                               uint16_t min_s, uint16_t max_s, const ezb_zcl_attr_variable_t *delta)
{
    ezb_zcl_reporting_info_t info = ezb_zcl_reporting_info_find(ep, cluster, EZB_ZCL_CLUSTER_SERVER, attr, 0);
    if (info == EZB_ZCL_INVALID_REPORTING_INFO) {
        ESP_LOGW(Config.Logging.TagReporting, "%s ep=0x%02X: NO reporting record", name, ep);
        return;
    }
    auto r = ezb_zcl_reporting_info_update(info, min_s, max_s, delta);
    ESP_LOGI(Config.Logging.TagReporting, "%s ep=0x%02X: update ret=%d", name, ep, (int)r);
}

static void init_zb_default_reporting()
{
    const auto &R = Config.Reporting.Dht;

    ezb_zcl_attr_variable_t dt = { .s16 = (int16_t)(R.ThresholdTemp * 100) };
    ezb_zcl_attr_variable_t dh = { .u16 = (uint16_t)(R.ThresholdHum * 100) };

    // board + DHT hőmérséklet
    set_default_report("temp", Config.Zigbee.Board.Endpoint,
        EZB_ZCL_CLUSTER_ID_TEMPERATURE_MEASUREMENT,
        EZB_ZCL_ATTR_TEMPERATURE_MEASUREMENT_MEASURED_VALUE_ID, R.MinInterval, R.MaxInterval, &dt);

    for (int i = 0; i < Config.Sensor.Dht.Count; i++) {
        uint8_t ep = Config.Zigbee.Dht.Endpoints[i];
        set_default_report("temp", ep,
            EZB_ZCL_CLUSTER_ID_TEMPERATURE_MEASUREMENT,
            EZB_ZCL_ATTR_TEMPERATURE_MEASUREMENT_MEASURED_VALUE_ID, R.MinInterval, R.MaxInterval, &dt);
        set_default_report("hum", ep,
            EZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
            EZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_MEASURED_VALUE_ID, R.MinInterval, R.MaxInterval, &dh);
    }
}

void init_zb_stack() {

   init_zb_storage();

    ESP_LOGI(Config.Logging.TagZigbee, "Start ZigBee main thread");
    xTaskCreate(zigbee_mainloop_task, "Zigbee_main", 16384, NULL, 5, NULL);
}

void init_zb_storage() {
    ESP_LOGI(Config.Logging.TagZigbee, "Initializing NVS");
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
        ESP_LOGI(Config.Logging.TagZigbee, "Received ZCL Default Response: status(0x%02x)", default_rsp->in.status_code);
    } break;
    default:
        ESP_LOGW(Config.Logging.TagZigbee, "ZCL Core Action: ID(0x%04lx)", callback_id);
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
        ESP_LOGI(Config.Logging.TagZigbee, "Initialize Zigbee stack");
        ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_INITIALIZATION);
        break;
    case EZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case EZB_BDB_SIGNAL_DEVICE_REBOOT: {
        ezb_bdb_comm_status_t status = *((ezb_bdb_comm_status_t *)ezb_app_signal_get_params(app_signal));
        if (status == EZB_BDB_STATUS_SUCCESS) {
            ESP_LOGI(Config.Logging.TagZigbee, "Device started up in%s factory-reset mode", ezb_bdb_is_factory_new() ? "" : " non");
            init_zb_default_reporting();
            if (ezb_bdb_is_factory_new()) {
                ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_NETWORK_STEERING);
            } else {
                is_zigbee_connected = true;
                ESP_LOGI(Config.Logging.TagZigbee, "Device reboot");
            }
        } else {
            ESP_LOGW(Config.Logging.TagZigbee, "%s failed with status(0x%02x), please retry", ezb_app_signal_to_string(signal_type), status);
            alarm_timer_schedule(esp_zigbee_alarm_bdb_commissioning, EZB_BDB_MODE_INITIALIZATION, 1000);
        }
    } break;
    case EZB_BDB_SIGNAL_STEERING: {
        ezb_bdb_comm_status_t status = *((ezb_bdb_comm_status_t *)ezb_app_signal_get_params(app_signal));
        if (status == EZB_BDB_STATUS_SUCCESS) {
            is_zigbee_connected = true;
            ezb_extpanid_t extended_pan_id;
            ezb_nwk_get_extended_panid(&extended_pan_id);
            ESP_LOGI(Config.Logging.TagZigbee, "Joined network successfully: PAN ID(0x%04hx, EXT: 0x%llx), Channel(%d), Short Address(0x%04hx)",
                     ezb_nwk_get_panid(), extended_pan_id.u64, ezb_nwk_get_current_channel(), ezb_nwk_get_short_address());
        } else {
            ESP_LOGW(Config.Logging.TagZigbee, "Failed to join network with status(0x%02x)", status);
            alarm_timer_schedule(esp_zigbee_alarm_bdb_commissioning, EZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
    } break;
    case EZB_ZDO_SIGNAL_DEVICE_ANNCE: {
        const ezb_zdo_signal_device_annce_params_t *dev_annce_params = (ezb_zdo_signal_device_annce_params_t *)ezb_app_signal_get_params(app_signal);
        ESP_LOGI(Config.Logging.TagZigbee, "New device commissioned or rejoined (short: 0x%04hx)", dev_annce_params->short_addr);
    } break;
    case EZB_ZDO_SIGNAL_LEAVE: {
        is_zigbee_connected = false;
        const ezb_zdo_signal_leave_params_t *leave_params = (ezb_zdo_signal_leave_params_t *)ezb_app_signal_get_params(app_signal);
        ESP_LOGI(Config.Logging.TagZigbee, "Left network successfully with type(0x%02x)", leave_params->leave_type);
        alarm_timer_schedule(esp_zigbee_alarm_steer_if_factory_new, 0, 2000);
    } break;
    case EZB_NWK_SIGNAL_PERMIT_JOIN_STATUS: {
        uint8_t duration = *(uint8_t *)ezb_app_signal_get_params(app_signal);
        if (duration) {
            ESP_LOGI(Config.Logging.TagZigbee, "Network(0x%04hx) is open for %d seconds", ezb_nwk_get_panid(), duration);
        } else {
            ESP_LOGW(Config.Logging.TagZigbee, "Network(0x%04hx) closed, devices joining not allowed.", ezb_nwk_get_panid());
        }
    } break;
    default:
        ESP_LOGI(Config.Logging.TagZigbee, "Zigbee APP Signal: %s(type: 0x%02x)", ezb_app_signal_to_string(signal_type), signal_type);
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

    ESP_LOGI(Config.Logging.TagZigbee, "ezb_config_memory");
   ezb_mem_config_t mem_config = {};
    mem_config.aps_bind_table_src_size = 32;
    auto mr = ezb_config_memory(&mem_config);
    ESP_LOGI(Config.Logging.TagZigbee, "ezb_config_memory ret=%d, heap=%u",
             (int)mr, (unsigned)esp_get_free_heap_size());

    ESP_ERROR_CHECK(esp_zigbee_setup_commissioning());

    ezb_af_device_desc_t dev_desc = ezb_af_create_device_desc();
    
    ESP_LOGI(Config.Logging.TagZigbee, "Init Zigbee board endpoint");
    init_zb_ep_board(dev_desc);

    ESP_LOGI(Config.Logging.TagZigbee, "Init Zigbee DHT endpoints");
    init_zb_ep_dht(dev_desc);

    ESP_LOGI(Config.Logging.TagZigbee, "Init Zigbee DC endpoints");
    init_zb_ep_dc(dev_desc);

    nvs_stats_t st0;
    if (nvs_get_stats(NULL, &st0) == ESP_OK)
        ESP_LOGI(Config.Logging.TagZigbee, "PRE-REG NVS used=%u free=%u",
                 (unsigned)st0.used_entries, (unsigned)st0.free_entries);

    ESP_LOGI(Config.Logging.TagZigbee, "Registering Endpoints");
    int64_t t0 = esp_timer_get_time();
    auto rr = ezb_af_device_desc_register(dev_desc);
    ESP_LOGI(Config.Logging.TagZigbee, "register ret=%d, %lld ms, heap=%u",
            (int)rr, (long long)((esp_timer_get_time() - t0) / 1000), (unsigned)esp_get_free_heap_size());

    nvs_stats_t st;
    if (nvs_get_stats(NULL, &st) == ESP_OK) {
        ESP_LOGI(Config.Logging.TagZigbee, "NVS used=%u free=%u total=%u ns=%u",
                (unsigned)st.used_entries, (unsigned)st.free_entries,
                (unsigned)st.total_entries, (unsigned)st.namespace_count);
    }

    ezb_zcl_core_action_handler_register(esp_zigbee_zcl_core_action_handler);


    ESP_LOGI(Config.Logging.TagZigbee, "esp_zigbee_start");
    ESP_ERROR_CHECK(esp_zigbee_start(false));

    ESP_LOGI(Config.Logging.TagZigbee, "esp_zigbee_launch_mainloop");
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
                ESP_LOGW(Config.Logging.TagZigbee, "Resetting Zigbee to factory and rebooting in 2s.");
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