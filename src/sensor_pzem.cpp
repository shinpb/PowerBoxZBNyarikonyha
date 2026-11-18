// PZEM-004T v3 AC energy meters on the UART buses (Modbus-RTU, own minimal implementation: pzem_proto.h + uart_bus.cpp).
// Same structure as the INA module (sensor_ina.cpp): init with a probe, one cycle function, offline state with a periodic
// probe, hysteresis (D15), ZCL invalid values on a lost module (D13), error flags per module / per bus, dummy mode.
//
// Note: the module's measuring side is powered from the AC mains (L/N). Without mains it does not answer (the 5V / 3V3 pin
// only supplies the optocoupler side of the TTL interface), so a mains outage looks like a failed module: its flag is set
// and its values become unavailable (null in Z2M) after Config.Health.FailThreshold failed polls.
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <math.h>
#include <stdio.h>
// Internal components
#include "sensor_common.h"
#include "sensor_manager.h"
#include "uart_bus.h"
#include "pzem_proto.h"
#include "zigbee_attr.h"
#include "zcl_values.h"
#include "health.h"
#include "heartbeat.h"
#include "config.h"

#ifndef DUMMY_SENSOR_MODE
enum pzem_err_t {
    PZEM_E_OK = 0,
    PZEM_E_BUS,         // bus not usable / driver error
    PZEM_E_TIMEOUT,     // no answer
    PZEM_E_CRC,         // CRC error
    PZEM_E_FRAME,       // short frame, other address, wrong function / byte count
    PZEM_E_EXCEPTION,   // Modbus exception answer
};

static const char *pzem_err_name(pzem_err_t e) {
    switch (e) {
    case PZEM_E_OK:        return "ok";
    case PZEM_E_BUS:       return "bus error";
    case PZEM_E_TIMEOUT:   return "no answer";
    case PZEM_E_CRC:       return "CRC error";
    case PZEM_E_FRAME:     return "bad frame";
    case PZEM_E_EXCEPTION: return "Modbus exception";
    default:               return "?";
    }
}

static bool     s_pzem_cfg_ok[n_PZEM] = {false};   // the module's configuration is valid (bus, address)
static bool     s_pzem_ok[n_PZEM] = {false};       // module answers (polled)
static bool     s_pzem_inv[n_PZEM] = {false};      // the ZCL invalid values are written (once per outage)
static bool     s_pzem_alarm[n_PZEM] = {false};
static uint32_t s_pzem_fail[n_PZEM] = {0};
static uint32_t s_pzem_good[n_PZEM] = {0};   // consecutive good polls while the error flag is set (recovery hysteresis)
static TickType_t s_pzem_retry_tick[n_PZEM] = {0};

static void raw_dump(int idx, const char *dir, const uint8_t *b, size_t n) {
    char s[PZEM_MAX_RESP * 3 + 1];
    size_t p = 0;
    for (size_t i = 0; i < n && p + 3 < sizeof s; i++) p += (size_t)snprintf(s + p, sizeof s - p, "%02X ", b[i]);
    LOGI_PZEMDIAG("PZEM%d %s %s", idx, dir, s);
}

// One Modbus read on the module's bus. regs[count] on success, *exc = the exception code (PZEM_E_EXCEPTION).
static pzem_err_t pzem_exchange(int idx, uint8_t addr, uint8_t fn, uint16_t reg, uint16_t count,
                                uint16_t *regs, uint8_t *exc) {
    const auto &M = Config.Sensor.Pzem.module[idx];
    uint8_t tx[PZEM_REQ_LEN];
    uint8_t rx[PZEM_MAX_RESP];
    const size_t tx_len = pzem_build_read(tx, addr, fn, reg, count);
    const size_t expected = pzem_read_resp_len(count);
    size_t got = 0;
    if (Config.Logging.PzemRaw) raw_dump(idx, "TX", tx, tx_len);
    const esp_err_t e = uart_bus_transfer(M.bus, tx, tx_len, rx, sizeof rx, expected, Config.Sensor.Pzem.TimeoutMs, &got);
    if (Config.Logging.PzemRaw && got > 0) raw_dump(idx, "RX", rx, got);
    if (e == ESP_ERR_TIMEOUT) return PZEM_E_TIMEOUT;
    if (e != ESP_OK) return PZEM_E_BUS;
    switch (pzem_parse_read(rx, got, addr, addr == PZEM_ADDR_GENERAL, fn, count, regs, exc)) {
    case PZEM_PARSE_OK:        return PZEM_E_OK;
    case PZEM_PARSE_CRC:       return PZEM_E_CRC;
    case PZEM_PARSE_EXCEPTION: return PZEM_E_EXCEPTION;
    default:                   return PZEM_E_FRAME;   // short / other address / wrong function or byte count
    }
}

static pzem_err_t pzem_read_input(int idx, uint16_t *regs, uint8_t *exc) {
    return pzem_exchange(idx, Config.Sensor.Pzem.module[idx].addr, PZEM_FN_READ_INPUT,
                         PZEM_REG_INPUT_START, PZEM_REG_INPUT_COUNT, regs, exc);
}

// Reset-energy request (0x42): ERASES the module's energy counter.
static pzem_err_t pzem_reset_energy(int idx, uint8_t *exc) {
    const auto &M = Config.Sensor.Pzem.module[idx];
    uint8_t tx[PZEM_RESET_LEN];
    uint8_t rx[PZEM_MAX_RESP];
    const size_t tx_len = pzem_build_reset(tx, M.addr);
    size_t got = 0;
    if (Config.Logging.PzemRaw) raw_dump(idx, "TX", tx, tx_len);
    const esp_err_t e = uart_bus_transfer(M.bus, tx, tx_len, rx, sizeof rx, PZEM_RESET_LEN, Config.Sensor.Pzem.TimeoutMs, &got);
    if (Config.Logging.PzemRaw && got > 0) raw_dump(idx, "RX", rx, got);
    if (e == ESP_ERR_TIMEOUT) return PZEM_E_TIMEOUT;
    if (e != ESP_OK) return PZEM_E_BUS;
    switch (pzem_parse_reset(rx, got, M.addr, exc)) {
    case PZEM_PARSE_OK:        return PZEM_E_OK;
    case PZEM_PARSE_CRC:       return PZEM_E_CRC;
    case PZEM_PARSE_EXCEPTION: return PZEM_E_EXCEPTION;
    default:                   return PZEM_E_FRAME;
    }
}

// Per module error counters (heartbeat "pzem ok/tmo/crc/proto")
static void pzem_count(int idx, pzem_err_t e) {
    switch (e) {
    case PZEM_E_OK:      diag_inc(g_diag.pzem_ok[idx]); break;
    case PZEM_E_TIMEOUT: diag_inc(g_diag.pzem_timeout[idx]); break;
    case PZEM_E_CRC:     diag_inc(g_diag.pzem_crc[idx]); break;
    default:             diag_inc(g_diag.pzem_proto[idx]); break;
    }
}

static int pzem_members_on_bus(uint8_t bus) {
    int n = 0;
    for (int i = 0; i < Config.Sensor.Pzem.Count; i++) if (Config.Sensor.Pzem.module[i].bus == bus) n++;
    return n;
}

// Configuration check: existing bus, valid and unique Modbus address on the bus.
static bool pzem_cfg_check(int idx) {
    const auto &M = Config.Sensor.Pzem.module[idx];
    if (M.bus >= Config.UART.Count) {
        LOGE_INIT_SENSOR("PZEM %d: UART bus %u does not exist (Config.UART.Count = %u)", idx, M.bus, Config.UART.Count);
        return false;
    }
    if (M.addr == 0 || (M.addr > 0xF7 && M.addr != PZEM_ADDR_GENERAL)) {
        LOGE_INIT_SENSOR("PZEM %d: invalid Modbus address 0x%02X (1..0xF7, or 0xF8 for a single module)", idx, M.addr);
        return false;
    }
    for (int j = 0; j < Config.Sensor.Pzem.Count; j++) {
        if (j == idx || Config.Sensor.Pzem.module[j].bus != M.bus) continue;
        if (M.addr == PZEM_ADDR_GENERAL) {
            LOGE_INIT_SENSOR("PZEM %d: the general address 0xF8 needs a single module on bus %u", idx, M.bus);
            return false;
        }
        if (j < idx && Config.Sensor.Pzem.module[j].addr == M.addr) {
            LOGE_INIT_SENSOR("PZEM %d: Modbus address 0x%02X is already used by PZEM %d on bus %u", idx, M.addr, j, M.bus);
            return false;
        }
    }
    return true;
}

// Bus / sensor error flags from the per-module failure counters (same rule as the INA modules, sensor_ina.cpp):
// if EVERY module of a UART bus fails, the bus gets the flag and the module flags of that bus are cleared.
// A bus with a single module cannot be told from a failed module (and a mains outage looks the same), so the
// bus flag needs >= 2 modules on the bus; a single module gets its own flag.
static void pzem_update_health() {
    const uint8_t thr = Config.Health.FailThreshold;
    for (int i = 0; i < Config.Sensor.Pzem.Count; i++) {
        if (!s_pzem_cfg_ok[i]) health_set(HM_SENSOR, HF_SENS_PZEM(i));   // invalid configuration: permanent fault
    }
    for (uint8_t b = 0; b < Config.UART.Count; b++) {
        int members = 0, failed = 0;
        for (int i = 0; i < Config.Sensor.Pzem.Count; i++) {
            if (!s_pzem_cfg_ok[i] || Config.Sensor.Pzem.module[i].bus != b) continue;
            members++;
            if (s_pzem_fail[i] >= thr) failed++;
        }
        const bool bus_failed = (members >= 2) && (failed == members);
        if (bus_failed) health_set(HM_BUS, HF_BUS_UART(b)); else health_clear(HM_BUS, HF_BUS_UART(b));
        for (int i = 0; i < Config.Sensor.Pzem.Count; i++) {
            if (!s_pzem_cfg_ok[i] || Config.Sensor.Pzem.module[i].bus != b) continue;
            if (!bus_failed && s_pzem_fail[i] >= thr) health_set(HM_SENSOR, HF_SENS_PZEM(i));
            else health_clear(HM_SENSOR, HF_SENS_PZEM(i));
        }
    }
}
#endif

// ---- energy counter reset (Zigbee trigger, Config.Sensor.Pzem.EnergyReset) ----
static volatile bool s_reset_req[n_PZEM] = {false};
#ifdef DUMMY_SENSOR_MODE
static double s_dummy_e_wh[n_PZEM] = {0};
#endif

// Zigbee stack context: only remember the request (the Modbus transfer takes up to Config.Sensor.Pzem.TimeoutMs).
void pzem_request_energy_reset(uint8_t ep) {
    for (int idx = 0; idx < Config.Sensor.Pzem.Count; idx++) {
        if (Config.Zigbee.Pzem.Endpoints[idx] != ep) continue;
        if (Config.Sensor.Pzem.EnergyReset == 0) {
            LOGW_MEAS("PZEM %d: energy reset trigger ignored (Config.Sensor.Pzem.EnergyReset = 0)", idx);
        }
        s_reset_req[idx] = true;
        return;
    }
}

// Measurement task: handle a pending request, then set the trigger attribute back to 0.
static void pzem_handle_reset(int idx, bool pre_start) {
    if (!s_reset_req[idx] || pre_start) return;
    s_reset_req[idx] = false;
    const uint8_t ep = Config.Zigbee.Pzem.Endpoints[idx];
    const uint8_t mode = Config.Sensor.Pzem.EnergyReset;
    if (mode == 1) {
        LOGW_MEAS("PZEM %d: energy reset requested over Zigbee, NOT executed (Config.Sensor.Pzem.EnergyReset = 1: log only)", idx);
    } else if (mode >= 2) {
#ifndef DUMMY_SENSOR_MODE
        if (!s_pzem_cfg_ok[idx] || !s_pzem_ok[idx]) {
            LOGW_MEAS("PZEM %d: energy reset requested, but the module is offline: not executed", idx);
        } else {
            uint8_t exc = 0;
            const pzem_err_t pe = pzem_reset_energy(idx, &exc);
            pzem_count(idx, pe);
            if (pe == PZEM_E_OK) LOGW_MEAS("PZEM %d: energy counter RESET (Modbus 0x42 acknowledged)", idx);
            else LOGW_MEAS("PZEM %d: energy reset failed: %s", idx, pzem_err_name(pe));
        }
#else
        s_dummy_e_wh[idx] = 0;
        LOGW_MEAS("PZEM %d: energy counter reset (dummy mode)", idx);
#endif
    }
    zb_clear_pzem_energy_reset(ep, pre_start);
}

void init_sensors_pzem() {
#ifndef DUMMY_SENSOR_MODE
    const uint8_t thr = Config.Health.FailThreshold;
    for (int idx = 0; idx < Config.Sensor.Pzem.Count; idx++) {
        const auto &M = Config.Sensor.Pzem.module[idx];
        s_pzem_cfg_ok[idx] = pzem_cfg_check(idx);
        s_pzem_fail[idx] = thr;                       // offline until the probe succeeds
        s_pzem_retry_tick[idx] = xTaskGetTickCount();
        if (!s_pzem_cfg_ok[idx]) continue;            // never retried: a configuration error
        if (!uart_bus_ready(M.bus)) {
            LOGE_INIT_SENSOR("PZEM %d: UART bus %u is not initialized, the module is skipped", idx, M.bus);
            continue;
        }
        uint16_t regs[PZEM_REG_INPUT_COUNT];
        uint8_t exc = 0;
        const pzem_err_t pe = pzem_read_input(idx, regs, &exc);
        pzem_count(idx, pe);
        if (pe == PZEM_E_OK) {
            s_pzem_ok[idx] = true;
            s_pzem_fail[idx] = 0;
            LOGI_INIT_SENSOR("PZEM %d: bus %u, address 0x%02X answers", idx, M.bus, M.addr);
        } else {
            LOGW_INIT_SENSOR("PZEM %d (bus %u, address 0x%02X): %s at init (mains on the module? wiring? address?), retry every %u s",
                             idx, M.bus, M.addr, pzem_err_name(pe), (unsigned)Config.Health.RetryS);
        }
    }
    pzem_update_health();
#endif
}

void pzem_cycle(bool pre_start) {
    for (int idx = 0; idx < Config.Sensor.Pzem.Count; idx++) {
        const uint8_t ep = Config.Zigbee.Pzem.Endpoints[idx];
        pzem_handle_reset(idx, pre_start);
        pzem_meas_t m = {};
        bool ok = false;
        bool publish = true;
#ifndef DUMMY_SENSOR_MODE
        const auto &M = Config.Sensor.Pzem.module[idx];
        const uint8_t thr = Config.Health.FailThreshold;
        uint16_t regs[PZEM_REG_INPUT_COUNT];
        uint8_t exc = 0;
        if (!s_pzem_ok[idx]) {
            // Offline: probe periodically (not from the stack context of measure_all_once)
            if (s_pzem_cfg_ok[idx] && !pre_start && uart_bus_ready(M.bus) &&
                (xTaskGetTickCount() - s_pzem_retry_tick[idx]) >= pdMS_TO_TICKS(Config.Health.RetryS * 1000UL)) {
                s_pzem_retry_tick[idx] = xTaskGetTickCount();
                diag_inc(g_diag.pzem_reinit_try);
                const pzem_err_t pe = pzem_read_input(idx, regs, &exc);
                pzem_count(idx, pe);
                LOGI_INIT_SENSOR("PZEM %d probe: %s", idx, pzem_err_name(pe));
                if (pe == PZEM_E_OK) {
                    // back online, but the error flag stays until RecoverThreshold good polls (any failure = offline again)
                    s_pzem_ok[idx] = true; s_pzem_fail[idx] = thr; s_pzem_good[idx] = 0;
                    diag_inc(g_diag.pzem_reinit_ok);
                }
            }
            if (!s_pzem_ok[idx]) {
                // offline module: report the ZCL invalid values (once per outage)
                if (!s_pzem_inv[idx]) {
                    pzem_zcl_t z;
                    pzem_zcl_invalid(&z);
                    s_pzem_inv[idx] = zb_write_pzem(ep, z, pre_start);
                }
                continue;
            }
        }
        publish = (s_pzem_fail[idx] < thr);   // false while recovering
        const pzem_err_t pe = pzem_read_input(idx, regs, &exc);
        pzem_count(idx, pe);
        ok = (pe == PZEM_E_OK);
        if (ok) {
            pzem_decode(regs, &m);
        } else {
            // log the first failure of a streak and then every PzemFailLogEvery-th poll only
            const uint16_t every = Config.Logging.PzemFailLogEvery;
            if (s_pzem_fail[idx] == 0 || (every != 0 && (s_pzem_fail[idx] % every) == 0)) {
                if (pe == PZEM_E_EXCEPTION) {
                    LOGW_MEAS("Measure PZEM %d failed: %s 0x%02X", idx, pzem_err_name(pe), exc);
                } else {
                    LOGW_MEAS("Measure PZEM %d failed: %s", idx, pzem_err_name(pe));
                }
            }
        }
#else
        // simulated values: 230 V / 50 Hz mains, a slowly varying load, the energy integrates the power
        static float tick_ac = 0.0f;
        tick_ac += 0.1f;
        const float v  = 230.0f + 2.0f * sinf(tick_ac + (float)idx);
        const float i  = 1.5f + 0.5f * cosf(tick_ac + (float)idx);
        const float pf = 0.92f;
        const float p  = v * i * pf;
        s_dummy_e_wh[idx] += (double)p * ((double)Config.Measurement.PzemMs / 1000.0) / 3600.0;
        m.voltage_dV = (uint16_t)lroundf(v * 10.0f);
        m.current_mA = (uint32_t)lroundf(i * 1000.0f);
        m.power_dW   = (uint32_t)lroundf(p * 10.0f);
        m.energy_Wh  = (uint32_t)s_dummy_e_wh[idx];
        m.freq_dHz   = (uint16_t)lroundf((50.0f + 0.05f * sinf(tick_ac)) * 10.0f);
        m.pf_c       = (uint16_t)lroundf(pf * 100.0f);
        m.alarm      = false;
        ok = true;
#endif
        if (ok) {
            if (Config.Logging.EnabledMeasurements) {
                LOGI_MEAS("PZEM %d: %.1fV %.3fA %.1fW %uWh %.1fHz pf=%.2f", idx,
                          m.voltage_dV / 10.0f, m.current_mA / 1000.0f, m.power_dW / 10.0f,
                          (unsigned)m.energy_Wh, m.freq_dHz / 10.0f, m.pf_c / 100.0f);
            }
#ifndef DUMMY_SENSOR_MODE
            if (m.alarm != s_pzem_alarm[idx]) {   // the module's own power alarm (threshold: holding register 0x0001)
                s_pzem_alarm[idx] = m.alarm;
                LOGW_MEAS("PZEM %d power alarm %s", idx, m.alarm ? "ACTIVE" : "cleared");
            }
#endif
            if (publish) {
                pzem_zcl_t z;
                pzem_to_zcl(&m, &z);
                zb_write_pzem(ep, z, pre_start);
            }
        }
#ifndef DUMMY_SENSOR_MODE
        if (!ok) {
            s_pzem_fail[idx] = sat_inc(s_pzem_fail[idx]);
            s_pzem_good[idx] = 0;
        } else {
            diag_mark(g_diag.pzem_last_ok_s[idx]);
            if (s_pzem_fail[idx] >= thr) {   // recovering
                s_pzem_good[idx] = sat_inc(s_pzem_good[idx]);
                if (s_pzem_good[idx] >= Config.Health.RecoverThreshold) {
                    s_pzem_fail[idx] = 0; s_pzem_good[idx] = 0; s_pzem_inv[idx] = false;
                }
            } else {
                s_pzem_fail[idx] = 0;
            }
        }
        if (!ok && s_pzem_fail[idx] >= thr) {
            // Offline: stop polling (a failed poll costs Config.Sensor.Pzem.TimeoutMs), probe every Config.Health.RetryS seconds
            s_pzem_ok[idx] = false;
            s_pzem_retry_tick[idx] = xTaskGetTickCount();
            diag_inc(g_diag.pzem_offline[idx]);
            LOGW_MEAS("PZEM %d offline, probe every %u s", idx, (unsigned)Config.Health.RetryS);
        }
#endif
    }
#ifndef DUMMY_SENSOR_MODE
    pzem_update_health();
#endif
}

#ifndef DUMMY_SENSOR_MODE
// Bring-up aid (Config.Logging.PzemDiag): the module's holding registers and one raw input register set.
// If a module does not answer on its configured address and it is alone on its bus, the general address 0xF8 is tried:
// the answer tells the real Modbus address.
void pzem_diag() {
    for (int idx = 0; idx < Config.Sensor.Pzem.Count; idx++) {
        const auto &M = Config.Sensor.Pzem.module[idx];
        if (!s_pzem_cfg_ok[idx]) { LOGW_PZEMDIAG("PZEM%d: invalid configuration, skipped", idx); continue; }
        if (!uart_bus_ready(M.bus)) { LOGW_PZEMDIAG("PZEM%d: UART bus %u not ready, skipped", idx, M.bus); continue; }

        uint16_t h[2] = {0, 0};
        uint8_t exc = 0;
        pzem_err_t pe = pzem_exchange(idx, M.addr, PZEM_FN_READ_HOLDING, PZEM_REG_HOLD_ALARM, 2, h, &exc);
        if (pe == PZEM_E_OK) {
            LOGI_PZEMDIAG("PZEM%d (bus %u, addr 0x%02X): alarm threshold %u W, address register 0x%04X%s", idx, M.bus, M.addr,
                          (unsigned)h[0], (unsigned)h[1], (h[1] == M.addr) ? "" : "  <-- differs from the configured address");
        } else {
            LOGW_PZEMDIAG("PZEM%d (bus %u, addr 0x%02X): holding registers: %s", idx, M.bus, M.addr, pzem_err_name(pe));
            if (pzem_members_on_bus(M.bus) == 1) {
                pe = pzem_exchange(idx, PZEM_ADDR_GENERAL, PZEM_FN_READ_HOLDING, PZEM_REG_HOLD_ALARM, 2, h, &exc);
                if (pe == PZEM_E_OK) {
                    LOGW_PZEMDIAG("PZEM%d answers on the general address 0xF8: its address register is 0x%04X, configured 0x%02X",
                                  idx, (unsigned)h[1], M.addr);
                } else {
                    LOGW_PZEMDIAG("PZEM%d: no answer on 0xF8 either (%s): check the mains on the module, its TTL supply, TX/RX "
                                  "(crossed?) and the baud rate (9600)", idx, pzem_err_name(pe));
                }
            }
            continue;
        }
        uint16_t r[PZEM_REG_INPUT_COUNT] = {0};
        pe = pzem_read_input(idx, r, &exc);
        if (pe != PZEM_E_OK) { LOGW_PZEMDIAG("PZEM%d: input registers: %s", idx, pzem_err_name(pe)); continue; }
        LOGI_PZEMDIAG("PZEM%d raw: V=0x%04X I=0x%04X%04X P=0x%04X%04X E=0x%04X%04X f=0x%04X pf=0x%04X alarm=0x%04X", idx,
                      r[0], r[2], r[1], r[4], r[3], r[6], r[5], r[7], r[8], r[9]);
        pzem_meas_t m;
        pzem_decode(r, &m);
        LOGI_PZEMDIAG("PZEM%d: %.1f V, %.3f A, %.1f W, %u Wh, %.1f Hz, pf %.2f%s", idx, m.voltage_dV / 10.0f,
                      m.current_mA / 1000.0f, m.power_dW / 10.0f, (unsigned)m.energy_Wh, m.freq_dHz / 10.0f, m.pf_c / 100.0f,
                      m.alarm ? ", ALARM" : "");
    }
}
#else
void pzem_diag() {}   // no real PZEM modules in dummy mode
#endif
