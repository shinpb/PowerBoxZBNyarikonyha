// Measurement tasks and the start order (the sensors themselves: sensor_board / sensor_dht / sensor_ina)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"    // esp_timer_get_time()
// Internal components
#include "sensor_common.h"
#include "sensor_manager.h"
#include "zigbee_attr.h"
#include "heartbeat.h"
#include "config.h"

static void cycle_stat(volatile uint32_t &last, volatile uint32_t &max, volatile uint32_t *over,
                       int64_t t0_us, int period_ms) {
    uint32_t ms = (uint32_t)((esp_timer_get_time() - t0_us) / 1000);
    last = ms;
    if (ms > max) max = ms;
    if (over != NULL && ms > (uint32_t)period_ms) diag_inc(*over);
}

// Set by measure_all_once(): the tasks then skip their first immediate cycle
// (the DHT sensor needs >= 2 s between two reads).
static volatile bool s_pre_measured = false;
// Set by start_measurement_tasks(): from then on measure_all_once() must not run (it would use the sensors in parallel with the tasks).
static volatile bool s_tasks_started = false;

// One cycle of each task: sensors, then the fault indication (written at the end of every cycle)
static void dht_task_cycle(bool pre_start) {
    board_cycle(pre_start);
    dht_cycle(pre_start);
    zb_publish_fault(pre_start);
}

static void dc_task_cycle(bool pre_start) {
    ina_cycle(pre_start);
    zb_publish_fault(pre_start);
}

static void dht_measurement_task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(Config.Measurement.DhtMs);

    if (s_pre_measured) {
        vTaskDelay(xFrequency);
        xLastWakeTime = xTaskGetTickCount();
    }
    while (true) {
        int64_t t0 = esp_timer_get_time();
        dht_task_cycle(false);
        cycle_stat(g_diag.dht_cycle_last_ms, g_diag.dht_cycle_max_ms, NULL, t0, Config.Measurement.DhtMs);
        // (the periodic heap/NVS status moved to the independent heartbeat task, see heartbeat.cpp)
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

static void dc_measurement_task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(Config.Measurement.DcMs); // 1 másodperces precíz mérés

    if (s_pre_measured) {
        vTaskDelay(xFrequency);
        xLastWakeTime = xTaskGetTickCount();
    }
    while (true) {
        int64_t t0 = esp_timer_get_time();
        dc_task_cycle(false);
        cycle_stat(g_diag.dc_cycle_last_ms, g_diag.dc_cycle_max_ms, &g_diag.dc_overruns, t0, Config.Measurement.DcMs);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void measure_all_once() {
    if (s_tasks_started) return;   // a repeated start signal of the stack must not read the sensors next to the tasks
    dht_task_cycle(true);
    dc_task_cycle(true);
    s_pre_measured = true;
}

void start_measurement_tasks() {
    if (s_tasks_started) return;
    s_tasks_started = true;
    LOGI_SETUP("Starting measurement tasks");
    xTaskCreate(dht_measurement_task, "Task_DHT", Config.Tasks.MeasDht.Stack, NULL, Config.Tasks.MeasDht.Prio, NULL);
    xTaskCreate(dc_measurement_task,  "Task_DC",  Config.Tasks.MeasDc.Stack, NULL, Config.Tasks.MeasDc.Prio, NULL);
}

