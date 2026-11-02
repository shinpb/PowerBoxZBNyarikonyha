#include "health.h"
#include "config.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"

static uint32_t   s_state[HM_COUNT] = {0};
static bool       s_inited = false;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

static int module_pin(health_module_t m) {
    switch (m) {
    case HM_ZIGBEE: return Config.Health.ZigbeePin;
    case HM_BUS:    return Config.Health.BusPin;
    case HM_SENSOR: return Config.Health.SensorPin;
    default:        return -1;
    }
}

static const char *module_name(health_module_t m) {
    switch (m) {
    case HM_ZIGBEE: return "zigbee";
    case HM_BUS:    return "bus";
    case HM_SENSOR: return "sensor";
    default:        return "?";
    }
}

// Pin handling: a negative number = indication disabled
static void pin_setup(int pin) {
    if (pin < 0) return;
    gpio_config_t io = {};
    io.pin_bit_mask = (1ULL << pin);
    io.mode = GPIO_MODE_OUTPUT;
    io.pull_up_en = GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io);
    gpio_set_level((gpio_num_t)pin, 0);
}

static inline void pin_write(int pin, bool high) {
    if (pin >= 0) gpio_set_level((gpio_num_t)pin, high ? 1 : 0);
}

static uint32_t global_locked() {
    uint32_t g = 0;
    for (int i = 0; i < HM_COUNT; i++) g |= s_state[i];
    return g;
}

void health_init() {
    pin_setup(Config.Health.GlobalPin);
    for (int i = 0; i < HM_COUNT; i++) pin_setup(module_pin((health_module_t)i));
    // Start NOT OK: every module is "starting" until its init code clears HF_STARTING.
    portENTER_CRITICAL(&s_lock);
    for (int i = 0; i < HM_COUNT; i++) {
        s_state[i] = HF_STARTING;
        pin_write(module_pin((health_module_t)i), true);
    }
    pin_write(Config.Health.GlobalPin, true);
    s_inited = true;
    portEXIT_CRITICAL(&s_lock);
    LOGI_HEALTH("init: pins global=%d zigbee=%d bus=%d sensor=%d (<0 = off)",
             Config.Health.GlobalPin, Config.Health.ZigbeePin, Config.Health.BusPin, Config.Health.SensorPin);
}

static void update(health_module_t m, uint32_t set_mask, uint32_t clear_mask) {
    if ((int)m < 0 || m >= HM_COUNT) return;
    uint32_t old_state, new_state, new_global;
    bool global_changed = false;

    portENTER_CRITICAL(&s_lock);
    old_state = s_state[m];
    uint32_t old_global = global_locked();
    new_state = (old_state | set_mask) & ~clear_mask;
    s_state[m] = new_state;
    new_global = global_locked();
    if (s_inited) {
        if ((old_state != 0) != (new_state != 0)) pin_write(module_pin(m), new_state != 0);
        if ((old_global != 0) != (new_global != 0)) {
            pin_write(Config.Health.GlobalPin, new_global != 0);
            global_changed = true;
        }
    }
    portEXIT_CRITICAL(&s_lock);

    if (new_state != old_state) {
        LOGW_HEALTH("%s: 0x%08X -> 0x%08X%s", module_name(m),
                 (unsigned)old_state, (unsigned)new_state,
                 new_state == 0 ? " (OK)" : "");
        if (global_changed) {
            LOGW_HEALTH("global: %s", new_global ? "ERROR" : "OK");
        }
    }
}

void health_set(health_module_t m, uint32_t flags)   { update(m, flags, 0); }
void health_clear(health_module_t m, uint32_t flags) { update(m, 0, flags); }

uint32_t health_get(health_module_t m) {
    if ((int)m < 0 || m >= HM_COUNT) return 0;
    portENTER_CRITICAL(&s_lock);
    uint32_t v = s_state[m];
    portEXIT_CRITICAL(&s_lock);
    return v;
}

uint32_t health_zb_mask() {
    portENTER_CRITICAL(&s_lock);
    uint32_t sensor = s_state[HM_SENSOR] & ~HF_STARTING;
    uint32_t bus    = s_state[HM_BUS] & ~HF_STARTING;
    portEXIT_CRITICAL(&s_lock);
    return (sensor & 0xFFFFu) | ((bus & 0xFFu) << 16);
}

uint32_t health_get_global() {
    portENTER_CRITICAL(&s_lock);
    uint32_t g = global_locked();
    portEXIT_CRITICAL(&s_lock);
    return g;
}
