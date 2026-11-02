// ******** Libraries config ********
// Sys
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_flash.h"
#include "esp_system.h"   // esp_reset_reason()
// Configuration
#include "config.h"
const AppConfig Config;
// Internal components
#include "health.h"
#include "heartbeat.h"
#include "sensor_manager.h"
#include "zigbee_manager.h"


// ******** Local Functions ********
static const char *reset_reason_name(esp_reset_reason_t r) {
    switch (r) {
    case ESP_RST_POWERON:   return "power-on";
    case ESP_RST_EXT:       return "external pin";
    case ESP_RST_SW:        return "esp_restart()";
    case ESP_RST_PANIC:     return "PANIC";
    case ESP_RST_INT_WDT:   return "INTERRUPT WDT";
    case ESP_RST_TASK_WDT:  return "TASK WDT";
    case ESP_RST_WDT:       return "other WDT";
    case ESP_RST_DEEPSLEEP: return "deep sleep wake";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    case ESP_RST_USB:       return "USB (host/serial reset)";
    case ESP_RST_JTAG:      return "JTAG";
    default:                return "other/unknown";
    }
}

// ******** ESP-IDF main entry point ********
extern "C" void app_main() {
    LOGI_SETUP("=== BOOT ===");
    LOGI_SETUP("app_main()");

    health_init();   // error indication pins, every unit starts in the "starting" state
    {
        esp_reset_reason_t rr = esp_reset_reason();
        LOGI_SETUP("Reset reason: %d (%s)", (int)rr, reset_reason_name(rr));
    }
    heartbeat_start();   // independent status line (heap, health, stacks...), covers the whole boot incl. Zigbee registration

    uint32_t flash_size = 0;
    esp_flash_get_physical_size(NULL, &flash_size);
    LOGI_SETUP("Flash physical size: %u KB", (unsigned)(flash_size / 1024));

    LOGI_INIT_SENSOR("Init board temperature sensor");
    init_sensor_board();

    LOGI_INIT_SENSOR("Init DHT sensors");
    init_sensors_dht();

    LOGI_INIT_BUS("Init I2C buses");
    init_native_i2c_buses();

    LOGI_INIT_SENSOR("Init INA sensors");
    init_sensors_ina();

    // Bus and sensor units are up (init failures are already flagged individually)
    health_clear(HM_BUS, HF_STARTING);
    health_clear(HM_SENSOR, HF_STARTING);

    if (Config.Logging.InaDiag) {
        LOGI_INIT_SENSOR("ina_diag()");
        ina_diag();
    }

    if (Config.Zigbee.Disabled) {
        health_clear(HM_ZIGBEE, HF_STARTING);   // no Zigbee unit
    }
    if(!Config.Zigbee.Disabled){
        gpio_config_t btn_conf = {};
        btn_conf.pin_bit_mask = (1ULL << Config.Device.ButtonPin);
        btn_conf.mode = GPIO_MODE_INPUT;
        btn_conf.pull_up_en = GPIO_PULLUP_ENABLE;
        btn_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        btn_conf.intr_type = GPIO_INTR_DISABLE;
        gpio_config(&btn_conf);

        LOGI_ZB("Init Zigbee Stack");
        init_zb_stack();

        xTaskCreate(button_task, "Task_Button", Config.Tasks.Button.Stack, NULL, Config.Tasks.Button.Prio, NULL);
    }

    // D7: with Zigbee enabled the measurement tasks are started by the Zigbee
    // main task after the endpoint registration (see zigbee_manager.cpp).
    // Without Zigbee they only make sense for the serial measurement log.
    if (Config.Zigbee.Disabled && Config.Logging.EnabledMeasurements) {
        start_measurement_tasks();
    }

    LOGI_SETUP("Setup done");
}
