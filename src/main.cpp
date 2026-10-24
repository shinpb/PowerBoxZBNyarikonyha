// ******** Libraries config ********
// Sys
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_flash.h"
// Configuration
#include "config.h"
const AppConfig Config;
// Internal components
#include "sensor_manager.h"
#include "zigbee_manager.h"


// ******** Local Functions ********

// ******** ESP-IDF main entry point ********
extern "C" void app_main() {
    ESP_LOGI(Config.Logging.TagSetup, "=== BOOT ===");
    ESP_LOGI(Config.Logging.TagSetup, "app_main()");

    uint32_t flash_size = 0;
    esp_flash_get_physical_size(NULL, &flash_size);
    ESP_LOGI(Config.Logging.TagSetup, "Flash physical size: %u KB", (unsigned)(flash_size / 1024));

    ESP_LOGI(Config.Logging.TagInitSensor, "Init board temperature sensor");
    init_sensor_board();

    ESP_LOGI(Config.Logging.TagInitSensor, "Init DHT sensors");
    init_sensors_dht();

    ESP_LOGI(Config.Logging.TagInitBus, "Init I2C buses");
    init_native_i2c_buses();

    ESP_LOGI(Config.Logging.TagInitSensor, "Init INA sensors");
    init_sensors_ina();

    ESP_LOGI(Config.Logging.TagInitSensor, "ina_diag()");
    ina_diag();
    
    if(!Config.Zigbee.Disabled){
        gpio_config_t btn_conf = {};
        btn_conf.pin_bit_mask = (1ULL << Config.Device.ButtonPin);
        btn_conf.mode = GPIO_MODE_INPUT;
        btn_conf.pull_up_en = GPIO_PULLUP_ENABLE;
        btn_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        btn_conf.intr_type = GPIO_INTR_DISABLE;
        gpio_config(&btn_conf);

        ESP_LOGI(Config.Logging.TagZigbee, "Init Zigbee Stack");
        init_zb_stack();

        xTaskCreate(
            button_task, 
            "Task_Button",  
            2048,
            NULL, 
            1,
            NULL
        );
    }

    xTaskCreate(
        dht_measurement_task, 
        "Task_DHT",     
        4096,            
        NULL,            
        1,
        NULL             
    );

    xTaskCreate(
        dc_measurement_task,  
        "Task_DC",      
        4096,            
        NULL,            
        1,
        NULL             
    );

    ESP_LOGI(Config.Logging.TagSetup, "Setup done");
}
