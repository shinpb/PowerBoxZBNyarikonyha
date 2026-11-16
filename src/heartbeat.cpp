#include "heartbeat.h"
#include "health.h"
#include "config.h"
#include "zigbee_manager.h"   // is_zigbee_connected
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <stdarg.h>

diag_counters_t g_diag = {};

uint32_t diag_uptime_s() { return (uint32_t)(esp_timer_get_time() / 1000000LL); }
void diag_mark(volatile uint32_t &t) { t = diag_uptime_s() + 1; }

// ---- small helpers ----
// Appends to buf (never overflows).
static void append(char *buf, size_t size, size_t &pos, const char *fmt, ...) {
    if (pos >= size) return;
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf + pos, size - pos, fmt, ap);
    va_end(ap);
    if (n > 0) pos += ((size_t)n < size - pos) ? (size_t)n : (size - pos - 1);
}

// Age in seconds of a "last OK" stamp; "-" if there was none yet.
static void append_age(char *buf, size_t size, size_t &pos, uint32_t stamp) {
    if (stamp == 0) append(buf, size, pos, "-");
    else append(buf, size, pos, "%u", (unsigned)(diag_uptime_s() - (stamp - 1)));
}

// Free stack (minimum ever, bytes) of a task found by name; -1 if it does not exist (any more).
// Looked up every time: the Zigbee main task is not guaranteed to live forever, a cached handle could go stale.
static int stack_free(const char *name) {
    TaskHandle_t h = xTaskGetHandle(name);
    return h ? (int)uxTaskGetStackHighWaterMark(h) : -1;
}

static void print_short() {
    LOGI_HB("up=%us heap=%u min=%u big=%u | health z=0x%08X b=0x%08X s=0x%08X g=0x%08X | stack_free DHT=%d DC=%d PZEM=%d ZB=%d BTN=%d HB=%d",
             (unsigned)diag_uptime_s(),
             (unsigned)esp_get_free_heap_size(),
             (unsigned)esp_get_minimum_free_heap_size(),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
             (unsigned)health_get(HM_ZIGBEE), (unsigned)health_get(HM_BUS), (unsigned)health_get(HM_SENSOR),
             (unsigned)health_get_global(),
             stack_free("Task_DHT"), stack_free("Task_DC"), stack_free("Task_PZEM"), stack_free("Zigbee_main"), stack_free("Task_Button"),
             (int)uxTaskGetStackHighWaterMark(NULL));
}

static void print_long() {
    char b[200];
    size_t p;

    LOGI_HB("cycle ms: dht last=%u max=%u | dc last=%u max=%u over=%u (period %d)",
             (unsigned)g_diag.dht_cycle_last_ms, (unsigned)g_diag.dht_cycle_max_ms,
             (unsigned)g_diag.dc_cycle_last_ms, (unsigned)g_diag.dc_cycle_max_ms,
             (unsigned)g_diag.dc_overruns, Config.Measurement.DcMs);
    LOGI_HB("cycle ms: pzem last=%u max=%u over=%u (period %d)",
             (unsigned)g_diag.pzem_cycle_last_ms, (unsigned)g_diag.pzem_cycle_max_ms,
             (unsigned)g_diag.pzem_overruns, Config.Measurement.PzemMs);

    p = 0; b[0] = 0;
    append(b, sizeof b, p, "dht ok/fail");
    for (int i = 0; i < Config.Sensor.Dht.Count; i++)
        append(b, sizeof b, p, " %d=%u/%u", i, (unsigned)g_diag.dht_ok[i], (unsigned)g_diag.dht_fail[i]);
    append(b, sizeof b, p, " | board fail=%u | ina offline", (unsigned)g_diag.board_fail);
    for (int i = 0; i < Config.Sensor.Ina.Count; i++)
        append(b, sizeof b, p, " %u", (unsigned)g_diag.ina_offline[i]);
    append(b, sizeof b, p, " reinit ok/try=%u/%u", (unsigned)g_diag.ina_reinit_ok, (unsigned)g_diag.ina_reinit_try);
    LOGI_HB("%s", b);

    // PZEM: ok/timeout/crc/proto per module, offline transitions, probes; UART transfers ok/fail per bus
    p = 0; b[0] = 0;
    append(b, sizeof b, p, "pzem ok/tmo/crc/proto");
    for (int i = 0; i < Config.Sensor.Pzem.Count; i++)
        append(b, sizeof b, p, " %d=%u/%u/%u/%u", i, (unsigned)g_diag.pzem_ok[i], (unsigned)g_diag.pzem_timeout[i],
               (unsigned)g_diag.pzem_crc[i], (unsigned)g_diag.pzem_proto[i]);
    append(b, sizeof b, p, " | offline");
    for (int i = 0; i < Config.Sensor.Pzem.Count; i++)
        append(b, sizeof b, p, " %u", (unsigned)g_diag.pzem_offline[i]);
    append(b, sizeof b, p, " reinit ok/try=%u/%u | uart ok/fail",
           (unsigned)g_diag.pzem_reinit_ok, (unsigned)g_diag.pzem_reinit_try);
    for (int i = 0; i < Config.UART.Count; i++)
        append(b, sizeof b, p, " %d=%u/%u", i, (unsigned)g_diag.uart_ok[i], (unsigned)g_diag.uart_fail[i]);
    LOGI_HB("%s", b);

    p = 0; b[0] = 0;
    append(b, sizeof b, p, "age s: board=");
    append_age(b, sizeof b, p, g_diag.board_last_ok_s);
    append(b, sizeof b, p, " dht");
    for (int i = 0; i < Config.Sensor.Dht.Count; i++) {
        append(b, sizeof b, p, " %d=", i);
        append_age(b, sizeof b, p, g_diag.dht_last_ok_s[i]);
    }
    append(b, sizeof b, p, " ina");
    for (int i = 0; i < Config.Sensor.Ina.Count; i++) {
        append(b, sizeof b, p, " %d=", i);
        append_age(b, sizeof b, p, g_diag.ina_last_ok_s[i]);
    }
    append(b, sizeof b, p, " pzem");
    for (int i = 0; i < Config.Sensor.Pzem.Count; i++) {
        append(b, sizeof b, p, " %d=", i);
        append_age(b, sizeof b, p, g_diag.pzem_last_ok_s[i]);
    }
    LOGI_HB("%s", b);

    nvs_stats_t st;
    bool have_nvs = (nvs_get_stats(NULL, &st) == ESP_OK);
    LOGI_HB("zb: joined=%d rsp ok/err=%u/%u join ok/fail=%u/%u leave=%u start_fail=%u | nvs used/free=%u/%u",
             (int)is_zigbee_connected,
             (unsigned)(g_diag.zb_default_rsp - g_diag.zb_default_rsp_err), (unsigned)g_diag.zb_default_rsp_err,
             (unsigned)g_diag.zb_join_ok, (unsigned)g_diag.zb_join_fail,
             (unsigned)g_diag.zb_leave, (unsigned)g_diag.zb_start_fail,
             have_nvs ? (unsigned)st.used_entries : 0u, have_nvs ? (unsigned)st.free_entries : 0u);
    // the one-shot reporting defaults task is gone by now: its last measured minimum (-1 = it has not run in this boot)
    LOGI_HB("stack_free of the one-shot Rep_defaults task: %d", g_diag.rep_stack_free ? (int)(g_diag.rep_stack_free - 1) : -1);
}

static void heartbeat_task(void *pv) {
    const TickType_t period = pdMS_TO_TICKS((uint32_t)Config.Health.HeartbeatS * 1000UL);
    const uint8_t every = Config.Health.HeartbeatLongEvery;
    uint32_t n = 0;

    print_short();   // baseline right after the start (heap at boot)
    TickType_t last = xTaskGetTickCount();
    while (true) {
        vTaskDelayUntil(&last, period);
        n++;
        print_short();
        if (every > 0 && (n % every) == 0) print_long();
    }
}

void heartbeat_start() {
    if (Config.Health.HeartbeatS == 0) return;
    static bool started = false;
    if (started) return;
    started = true;
    LOGI_HB("started: period %u s, extended lines every %u",
             (unsigned)Config.Health.HeartbeatS, (unsigned)Config.Health.HeartbeatLongEvery);
    xTaskCreate(heartbeat_task, "Task_HB", Config.Tasks.Heartbeat.Stack, NULL, Config.Tasks.Heartbeat.Prio, NULL);
}
