// UART buses (Config.UART): driver setup and the request/response transfer
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "soc/soc_caps.h"
#include "driver/gpio.h"
#include "driver/uart.h"
// Internal components
#include "sensor_common.h"
#include "uart_bus.h"
#include "pzem_proto.h"   // pzem_frame_ms (wire time of a frame)
#include "health.h"
#include "heartbeat.h"
#include "config.h"

#ifndef SOC_UART_HP_NUM
#define SOC_UART_HP_NUM SOC_UART_NUM
#endif

#ifndef DUMMY_SENSOR_MODE
static bool              s_uart_ready[n_UART] = {false};
static SemaphoreHandle_t s_uart_mutex[n_UART] = {NULL};

// true if the GPIO is configured for another function (I2C, DHT, button, error indication pins) or for another UART bus (< b).
static bool pin_used_elsewhere(int pin, uint8_t b) {
    for (uint8_t i = 0; i < b; i++) {
        if ((int)Config.UART.bus[i].pin.tx == pin || (int)Config.UART.bus[i].pin.rx == pin) return true;
    }
    for (uint8_t i = 0; i < Config.I2C.Count; i++) {
        if ((int)Config.I2C.bus[i].pin.sda == pin || (int)Config.I2C.bus[i].pin.scl == pin) return true;
    }
    for (uint8_t i = 0; i < Config.Sensor.Dht.Count; i++) {
        if ((int)Config.Sensor.Dht.Pins[i] == pin) return true;
    }
    if ((int)Config.Device.ButtonPin == pin) return true;
    if (Config.Health.GlobalPin == pin || Config.Health.ZigbeePin == pin ||
        Config.Health.BusPin == pin || Config.Health.SensorPin == pin) return true;
    return false;
}

// Configuration check of one bus (before anything is touched). ESP_OK = usable.
static esp_err_t uart_check_config(uint8_t b) {
    const auto &B = Config.UART.bus[b];
    if ((int)B.port < 0 || (int)B.port >= SOC_UART_HP_NUM) {
        LOGE_INIT_BUS("UART bus %u: port %d is not a high-power UART port (0..%d)", b, (int)B.port, SOC_UART_HP_NUM - 1);
        return ESP_ERR_INVALID_ARG;
    }
#if defined(CONFIG_ESP_CONSOLE_UART_NUM)
    if ((int)B.port == CONFIG_ESP_CONSOLE_UART_NUM) {
        LOGE_INIT_BUS("UART bus %u: port %d is the console UART (CONFIG_ESP_CONSOLE_UART_NUM), use another port", b, (int)B.port);
        return ESP_ERR_INVALID_ARG;
    }
#endif
    const int tx = (int)B.pin.tx, rx = (int)B.pin.rx;
    if (tx < 0 || rx < 0 || !GPIO_IS_VALID_OUTPUT_GPIO(tx) || !GPIO_IS_VALID_GPIO(rx)) {
        LOGE_INIT_BUS("UART bus %u: invalid pin TX=%d RX=%d", b, tx, rx);
        return ESP_ERR_INVALID_ARG;
    }
    if (tx == rx) {
        LOGE_INIT_BUS("UART bus %u: TX and RX are the same pin (GPIO%d)", b, tx);
        return ESP_ERR_INVALID_ARG;
    }
    if (pin_used_elsewhere(tx, b) || pin_used_elsewhere(rx, b)) {
        LOGE_INIT_BUS("UART bus %u: TX=GPIO%d or RX=GPIO%d is already used by another function", b, tx, rx);
        return ESP_ERR_INVALID_ARG;
    }
    if (B.Baud == 0 || B.RxBufBytes <= UART_HW_FIFO_LEN(B.port)) {
        LOGE_INIT_BUS("UART bus %u: baud %u / RX buffer %u invalid (the buffer must exceed the %d byte hardware FIFO)",
                      b, (unsigned)B.Baud, (unsigned)B.RxBufBytes, (int)UART_HW_FIFO_LEN(B.port));
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

static esp_err_t uart_init_one(uint8_t b) {
    esp_err_t e = uart_check_config(b);
    if (e != ESP_OK) return e;
    const auto &B = Config.UART.bus[b];

    uart_config_t cfg = {};
    cfg.baud_rate           = (int)B.Baud;
    cfg.data_bits           = UART_DATA_8_BITS;
    cfg.parity              = UART_PARITY_DISABLE;
    cfg.stop_bits           = UART_STOP_BITS_1;
    cfg.flow_ctrl           = UART_HW_FLOWCTRL_DISABLE;
    cfg.rx_flow_ctrl_thresh = 0;
    cfg.source_clk          = UART_SCLK_DEFAULT;

    e = uart_param_config(B.port, &cfg);
    if (e != ESP_OK) { LOGE_INIT_BUS("UART bus %u: uart_param_config: %s", b, esp_err_to_name(e)); return e; }
    e = uart_set_pin(B.port, (int)B.pin.tx, (int)B.pin.rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (e != ESP_OK) { LOGE_INIT_BUS("UART bus %u: uart_set_pin: %s", b, esp_err_to_name(e)); return e; }
    // RX ring buffer only, no TX buffer (uart_write_bytes blocks until the bytes are in the FIFO), no event queue
    e = uart_driver_install(B.port, B.RxBufBytes, 0, 0, NULL, 0);
    if (e != ESP_OK) { LOGE_INIT_BUS("UART bus %u: uart_driver_install: %s", b, esp_err_to_name(e)); return e; }

    s_uart_mutex[b] = xSemaphoreCreateMutex();
    if (s_uart_mutex[b] == NULL) {
        LOGE_INIT_BUS("UART bus %u: mutex creation failed", b);
        uart_driver_delete(B.port);
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
#endif

void init_uart_buses() {
#ifndef DUMMY_SENSOR_MODE
    bool failed = false;
    for (uint8_t b = 0; b < Config.UART.Count; b++) {
        const esp_err_t e = uart_init_one(b);
        if (e == ESP_OK) {
            s_uart_ready[b] = true;
            LOGI_INIT_BUS("UART bus %u: port %d TX=GPIO%d RX=GPIO%d %u baud 8N1", b, (int)Config.UART.bus[b].port,
                          (int)Config.UART.bus[b].pin.tx, (int)Config.UART.bus[b].pin.rx, (unsigned)Config.UART.bus[b].Baud);
        } else {
            failed = true;
            init_failed("UART bus init", e);   // abort only with Config.Health.AbortOnInitFail
        }
    }
    if (failed) health_set(HM_BUS, HF_BUS_UART_INIT);
#endif
}

bool uart_bus_ready(uint8_t bus) {
#ifndef DUMMY_SENSOR_MODE
    return bus < Config.UART.Count && s_uart_ready[bus];
#else
    (void)bus;
    return false;
#endif
}

esp_err_t uart_bus_transfer(uint8_t bus, const uint8_t *tx, size_t tx_len,
                            uint8_t *rx, size_t rx_max, size_t rx_expected,
                            uint32_t first_byte_timeout_ms, size_t *rx_got) {
    if (rx_got) *rx_got = 0;
#ifdef DUMMY_SENSOR_MODE
    (void)bus; (void)tx; (void)tx_len; (void)rx; (void)rx_max; (void)rx_expected; (void)first_byte_timeout_ms;
    return ESP_ERR_INVALID_STATE;
#else
    if (bus >= Config.UART.Count || tx == NULL || rx == NULL || tx_len == 0 || rx_expected == 0 || rx_expected > rx_max)
        return ESP_ERR_INVALID_ARG;
    if (!s_uart_ready[bus]) return ESP_ERR_INVALID_STATE;
    const uart_port_t port = Config.UART.bus[bus].port;
    const uint32_t baud = Config.UART.bus[bus].Baud;

    // Wire time of the rest of the answer (+ margin for the module's inter-byte gaps and the tick resolution)
    const uint32_t rest_ms = pzem_frame_ms(baud, rx_expected - 1, 20);

    if (xSemaphoreTake(s_uart_mutex[bus], pdMS_TO_TICKS(first_byte_timeout_ms + rest_ms + 200)) != pdTRUE)
        return ESP_ERR_TIMEOUT;

    esp_err_t result = ESP_ERR_TIMEOUT;
    size_t got = 0;
    do {
        uart_flush_input(port);   // stale bytes of an earlier (late / partial) answer must not be taken for this one
        const int w = uart_write_bytes(port, (const char *)tx, tx_len);
        if (w < 0 || (size_t)w != tx_len) {
            result = ESP_FAIL;
            diag_inc(g_diag.uart_fail[bus]);
            break;
        }
        // the 3.5 character silence before the next request is guaranteed by the caller's cycle period;
        // wait until the request has left the FIFO, so the first-byte timeout starts at the end of our frame
        if (uart_wait_tx_done(port, pdMS_TO_TICKS(pzem_frame_ms(baud, tx_len, 20))) != ESP_OK) {
            result = ESP_FAIL;
            diag_inc(g_diag.uart_fail[bus]);
            break;
        }
        int n = uart_read_bytes(port, rx, 1, pdMS_TO_TICKS(first_byte_timeout_ms));
        if (n < 1) { diag_inc(g_diag.uart_fail[bus]); break; }   // result stays ESP_ERR_TIMEOUT
        got = 1;
        if (rx_expected > 1) {
            n = uart_read_bytes(port, rx + 1, rx_expected - 1, pdMS_TO_TICKS(rest_ms));
            if (n > 0) got += (size_t)n;
        }
        result = ESP_OK;
        diag_inc(g_diag.uart_ok[bus]);
    } while (0);
    xSemaphoreGive(s_uart_mutex[bus]);
    if (rx_got) *rx_got = got;
    return result;
#endif
}
