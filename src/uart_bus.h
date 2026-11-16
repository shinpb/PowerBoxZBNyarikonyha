#pragma once
// UART buses (Config.UART): driver setup and one request/response transfer (the UART counterpart of the I2C buses).
// A bus can be shared by several modules (e.g. PZEM-004T meters with different Modbus addresses); the transfers are
// serialized by a mutex per bus. The protocol layer (frames, CRC) is not here: pzem_proto.h.
#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

// Configures every bus of Config.UART (parameters, pins, driver). A failure does not abort the firmware:
// the bus stays unusable, HF_BUS_UART_INIT is set (health.h) and the modules on it report offline.
// Does nothing in DUMMY_SENSOR_MODE.
void init_uart_buses();

// true if bus b was initialized successfully (always false in DUMMY_SENSOR_MODE, where no real bus exists).
bool uart_bus_ready(uint8_t bus);

// One exchange on bus b: discards stale input, sends tx[0..tx_len), then collects the answer:
//  - waits up to first_byte_timeout_ms for the FIRST byte,
//  - then up to the wire time of the remaining (rx_expected - 1) bytes (+ a margin) for the rest,
//  - stops early when rx_expected bytes have arrived. A shorter answer (e.g. a 5 byte Modbus exception) is returned as it is.
// rx_max is the size of rx (>= rx_expected). *rx_got = number of bytes received.
// Returns ESP_OK (>= 1 byte received, the caller validates the frame), ESP_ERR_TIMEOUT (nothing received),
// ESP_ERR_INVALID_STATE (bus not initialized), ESP_ERR_INVALID_ARG, ESP_FAIL (driver write error).
esp_err_t uart_bus_transfer(uint8_t bus, const uint8_t *tx, size_t tx_len,
                            uint8_t *rx, size_t rx_max, size_t rx_expected,
                            uint32_t first_byte_timeout_ms, size_t *rx_got);
