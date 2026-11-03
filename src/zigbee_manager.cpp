// Sys
#include "esp_log.h"
#include "esp_timer.h"      // esp_timer_get_time()
#include "esp_system.h"     // esp_get_free_heap_size()
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
// External Libraries
#include "esp_zigbee.h"
#include "ezbee/zha.h"
#include "alarm_timer.h"
// Internal components
#include "zigbee_manager.h"
#include "zigbee_config.h"
#include "zigbee_endpoints.h"
#include "zigbee_reporting.h"
#include "sensor_manager.h"   // measure_all_once, start_measurement_tasks
#include "health.h"
#include "heartbeat.h"
#include "config.h"

// Component internal functions
static bool esp_zigbee_app_signal_handler(const ezb_app_signal_t *app_signal);
static void zigbee_mainloop_task(void *pvParameters);
static void init_zb_storage();

// Shared variable single instance
volatile bool is_zigbee_connected = false;

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
            zb_disable_default_rsp();
            if (ezb_bdb_is_factory_new()) {
                health_set(HM_ZIGBEE, HF_ZB_NOT_JOINED);        // set first: no low glitch on the pin
                health_clear(HM_ZIGBEE, HF_STARTING);
                // the defaults task writes the records (~26 s for all of them) and then starts the steering itself
                if (!zb_start_reporting_defaults()) {
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
            alarm_timer_schedule(esp_zigbee_alarm_bdb_commissioning, EZB_BDB_MODE_INITIALIZATION, Config.Zigbee.RetryInitMs);
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
            alarm_timer_schedule(esp_zigbee_alarm_bdb_commissioning, EZB_BDB_MODE_NETWORK_STEERING, Config.Zigbee.RetrySteerMs);
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
        alarm_timer_schedule(esp_zigbee_alarm_steer_if_factory_new, 0, Config.Zigbee.LeaveSteerMs);
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

static esp_err_t esp_zigbee_setup_commissioning(void)
{
    ezb_aps_secur_enable_distributed_security(false);
    ESP_ERROR_CHECK(ezb_bdb_set_primary_channel_set(Config.Zigbee.PrimaryChannelMask));
    ESP_ERROR_CHECK(ezb_bdb_set_secondary_channel_set(Config.Zigbee.SecondaryChannelMask));
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
    zb_build_endpoints(dev_desc);

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
