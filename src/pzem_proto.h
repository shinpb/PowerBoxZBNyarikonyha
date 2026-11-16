#pragma once
// PZEM-004T v3 (AC energy meter, 80..260 V): Modbus-RTU frame helpers and the unit conversions.
// Pure functions: no ESP-IDF / FreeRTOS dependency (only stdint), so the whole protocol layer is unit-tested on the host
// (host_test/pzem_proto_test.cpp). The UART transfer is in uart_bus.cpp, the module logic in sensor_pzem.cpp.
//
// Serial: 9600 baud, 8N1. Slave address 0x01..0xF7 (factory default 0x01), 0xF8 = general address (single slave only).
// Functions: 0x03 read holding registers, 0x04 read input registers, 0x06 write single holding register,
//            0x41 calibration, 0x42 reset energy (the last three are not used by the firmware).
// Input registers (0x04, start 0x0000, 10 registers; 32 bit values are LOW word first):
//   0x0000 voltage      0.1 V          0x0005/6 energy    1 Wh (low, high)
//   0x0001/2 current    1 mA (low, high) 0x0007 frequency   0.1 Hz
//   0x0003/4 power      0.1 W (low, high) 0x0008 power factor 0.01
//   0x0009 alarm status 0xFFFF = power above the alarm threshold, 0 = ok
// Holding registers (0x03/0x06): 0x0001 power alarm threshold [W], 0x0002 Modbus-RTU address.
// Error response: [addr][function | 0x80][code][CRC lo][CRC hi], code 1 illegal function, 2 illegal address,
//                 3 illegal data, 4 slave error.
#include <stdint.h>
#include <stddef.h>
#include "zcl_values.h"

#define PZEM_ADDR_DEFAULT    0x01   // factory default Modbus address
#define PZEM_ADDR_GENERAL    0xF8   // general address: any single PZEM answers (discovery / calibration only)

#define PZEM_FN_READ_HOLDING 0x03
#define PZEM_FN_READ_INPUT   0x04

#define PZEM_REG_INPUT_START 0x0000
#define PZEM_REG_INPUT_COUNT 10
#define PZEM_REG_HOLD_ALARM  0x0001   // power alarm threshold [W]
#define PZEM_REG_HOLD_ADDR   0x0002   // Modbus address

#define PZEM_REQ_LEN         8                                   // addr, fn, reg(2), count(2), crc(2)
#define PZEM_MAX_REGS        PZEM_REG_INPUT_COUNT
#define PZEM_MAX_RESP        (5 + 2 * PZEM_MAX_REGS)             // addr, fn, bytes, data, crc(2) = 25

// Modbus CRC-16 (init 0xFFFF, reflected polynomial 0xA001), sent LOW byte first.
static inline uint16_t pzem_crc16(const uint8_t *d, size_t n) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < n; i++) {
        crc ^= d[i];
        for (int b = 0; b < 8; b++) crc = (crc & 1u) ? (uint16_t)((crc >> 1) ^ 0xA001u) : (uint16_t)(crc >> 1);
    }
    return crc;
}

// Read request (function 0x03 or 0x04) into buf[PZEM_REQ_LEN]; returns the frame length.
static inline size_t pzem_build_read(uint8_t *buf, uint8_t addr, uint8_t fn, uint16_t reg, uint16_t count) {
    buf[0] = addr;
    buf[1] = fn;
    buf[2] = (uint8_t)(reg >> 8);
    buf[3] = (uint8_t)(reg & 0xFF);
    buf[4] = (uint8_t)(count >> 8);
    buf[5] = (uint8_t)(count & 0xFF);
    const uint16_t crc = pzem_crc16(buf, 6);
    buf[6] = (uint8_t)(crc & 0xFF);
    buf[7] = (uint8_t)(crc >> 8);
    return PZEM_REQ_LEN;
}

// Length of a normal read response for `count` registers.
static inline size_t pzem_read_resp_len(uint16_t count) { return (size_t)5 + 2u * count; }

// Time of `bytes` bytes on the wire in ms (10 bit per byte at 8N1 + start/stop = 10; rounded up) plus a margin.
static inline uint32_t pzem_frame_ms(uint32_t baud, size_t bytes, uint32_t margin_ms) {
    if (baud == 0) baud = 9600;
    return (uint32_t)((bytes * 10u * 1000u + baud - 1u) / baud) + margin_ms;
}

typedef enum {
    PZEM_PARSE_OK = 0,
    PZEM_PARSE_SHORT,       // fewer bytes than a frame needs
    PZEM_PARSE_CRC,         // CRC mismatch
    PZEM_PARSE_ADDR,        // answer from another address
    PZEM_PARSE_FORMAT,      // wrong function code or byte count
    PZEM_PARSE_EXCEPTION,   // valid Modbus exception frame (code in *exc)
} pzem_parse_t;

// Parses a read response (function fn) for `count` registers into regs[count] (big endian words).
// addr_any = true: the address byte is not checked (general address 0xF8 probe).
static inline pzem_parse_t pzem_parse_read(const uint8_t *rx, size_t got, uint8_t addr, bool addr_any,
                                           uint8_t fn, uint16_t count, uint16_t *regs, uint8_t *exc) {
    if (got < 5) return PZEM_PARSE_SHORT;
    if (rx[1] == (uint8_t)(fn | 0x80)) {                      // exception frame, exactly 5 bytes
        const uint16_t crc = pzem_crc16(rx, 3);
        if (rx[3] != (uint8_t)(crc & 0xFF) || rx[4] != (uint8_t)(crc >> 8)) return PZEM_PARSE_CRC;
        if (!addr_any && rx[0] != addr) return PZEM_PARSE_ADDR;
        if (exc) *exc = rx[2];
        return PZEM_PARSE_EXCEPTION;
    }
    const size_t need = pzem_read_resp_len(count);
    if (got < need) return PZEM_PARSE_SHORT;
    const uint16_t crc = pzem_crc16(rx, need - 2);
    if (rx[need - 2] != (uint8_t)(crc & 0xFF) || rx[need - 1] != (uint8_t)(crc >> 8)) return PZEM_PARSE_CRC;
    if (!addr_any && rx[0] != addr) return PZEM_PARSE_ADDR;
    if (rx[1] != fn || rx[2] != (uint8_t)(2u * count)) return PZEM_PARSE_FORMAT;
    for (uint16_t i = 0; i < count; i++) regs[i] = (uint16_t)(((uint16_t)rx[3 + 2 * i] << 8) | rx[4 + 2 * i]);
    return PZEM_PARSE_OK;
}

// ---- decoded measurement (physical, fixed point exactly as the device delivers it) ----
typedef struct {
    uint16_t voltage_dV;    // 0.1 V
    uint32_t current_mA;    // mA
    uint32_t power_dW;      // 0.1 W
    uint32_t energy_Wh;     // Wh
    uint16_t freq_dHz;      // 0.1 Hz
    uint16_t pf_c;          // 0.01 (0..100)
    bool     alarm;         // power alarm active
} pzem_meas_t;

// regs = the 10 input registers (PZEM_REG_INPUT_COUNT)
static inline void pzem_decode(const uint16_t *r, pzem_meas_t *m) {
    m->voltage_dV = r[0];
    m->current_mA = ((uint32_t)r[2] << 16) | r[1];
    m->power_dW   = ((uint32_t)r[4] << 16) | r[3];
    m->energy_Wh  = ((uint32_t)r[6] << 16) | r[5];
    m->freq_dHz   = r[7];
    m->pf_c       = r[8];
    m->alarm      = (r[9] != 0);
}

// ---- raw ZCL attribute values (divisors: zcl_values.h ZCL_AC_DIV_*, ZCL_METER_DIV_E) ----
typedef struct {
    uint16_t voltage;   // RMSVoltage       0.1 V   (ZCL_AC_DIV_V = 10)
    uint16_t current;   // RMSCurrent       10 mA   (ZCL_AC_DIV_I = 100)
    int16_t  power;     // ActivePower      1 W     (ZCL_AC_DIV_P = 1)
    uint16_t freq;      // ACFrequency      0.1 Hz  (ZCL_AC_DIV_F = 10)
    int8_t   pf;        // PowerFactor      percent
    uint64_t energy;    // CurrentSummationDelivered, Wh (uint48; Metering divisor 1000 -> kWh)
} pzem_zcl_t;

static inline uint16_t pzem_sat_u16(uint32_t v) { return v > 0xFFFEu ? (uint16_t)0xFFFEu : (uint16_t)v; }   // 0xFFFF = invalid

static inline void pzem_to_zcl(const pzem_meas_t *m, pzem_zcl_t *z) {
    z->voltage = pzem_sat_u16(m->voltage_dV);
    z->current = pzem_sat_u16((m->current_mA + 5u) / 10u);
    const uint32_t p_w = (m->power_dW + 5u) / 10u;
    z->power   = (int16_t)(p_w > 32767u ? 32767u : p_w);
    z->freq    = pzem_sat_u16(m->freq_dHz);
    z->pf      = (int8_t)(m->pf_c > 100u ? 100u : m->pf_c);
    z->energy  = (uint64_t)m->energy_Wh;
}

// The "unavailable" values (written once per outage, 4.7 / D13)
static inline void pzem_zcl_invalid(pzem_zcl_t *z) {
    z->voltage = ZCL_INVALID_AC_U16;
    z->current = ZCL_INVALID_AC_U16;
    z->power   = ZCL_INVALID_AC_P;
    z->freq    = ZCL_INVALID_AC_U16;
    z->pf      = ZCL_INVALID_AC_PF;
    z->energy  = ZCL_INVALID_ENERGY;
}
