// Sys
#include "esp_log.h"
#include "esp_timer.h"      // esp_timer_get_time()
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
// Internal components
#include "button.h"
#include "zigbee_manager.h"
#include "config.h"

static uint64_t millis();

static void button_task(void *pvParameters) {
    uint64_t pressStartTime = 0;
    bool isPressed = false;
    
    while (true) {
        if (gpio_get_level(Config.Device.ButtonPin) == 0) {
            if (!isPressed) {
                isPressed = true;
                pressStartTime = millis(); 
            } else if ((millis() - pressStartTime) > Config.Device.ButtonHoldMs) {
                zb_factory_reset_and_restart();   // does not return
            }
        } else {
            isPressed = false;
        }
        
        vTaskDelay(pdMS_TO_TICKS(Config.Device.ButtonPollMs));
    }
}

static uint64_t millis() {
    return (uint64_t)(esp_timer_get_time() / 1000ULL);
}

void button_init() {
    gpio_config_t btn_conf = {};
    btn_conf.pin_bit_mask = (1ULL << Config.Device.ButtonPin);
    btn_conf.mode = GPIO_MODE_INPUT;
    btn_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    btn_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    btn_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&btn_conf);

    xTaskCreate(button_task, "Task_Button", Config.Tasks.Button.Stack, NULL, Config.Tasks.Button.Prio, NULL);
}
