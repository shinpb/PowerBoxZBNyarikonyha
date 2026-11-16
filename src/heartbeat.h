#pragma once
// Heartbeat: a periodic, independent status log line + the diagnostic counters it prints.
//
// - Runs in its own low-priority task, started first thing in app_main() (heartbeat_start()), so it
//   also covers the long Zigbee registration and does not depend on any measurement task.
// - Read-only: no Zigbee lock, no attribute access, no Zigbee API call. It only reads the counters below,
//   the health masks and system information, so it cannot slow anything down.
// - Period: Config.Health.HeartbeatS (0 = off). Every Config.Health.HeartbeatLongEvery-th heartbeat
//   additionally prints the extended "Diag" lines (cycle times, counters, data ages, Zigbee counters).
//
// Counters are single-writer (each is written by exactly one task) 32-bit values, so plain volatile
// reads/writes are enough, no lock is needed.
#include <stdint.h>
#include "config.h"   // n_INA, n_DHT, n_PZEM, n_UART

struct diag_counters_t {
    // ---- written by the measurement tasks (sensor_manager.cpp) ----
    volatile uint32_t dht_ok[n_DHT];
    volatile uint32_t dht_fail[n_DHT];
    volatile uint32_t dht_last_ok_s[n_DHT];   // uptime (s) + 1 of the last good read, 0 = never
    volatile uint32_t board_fail;
    volatile uint32_t board_last_ok_s;        // uptime (s) + 1, 0 = never
    volatile uint32_t ina_offline[n_INA];     // number of transitions to the offline state
    volatile uint32_t ina_last_ok_s[n_INA];   // uptime (s) + 1 of the last fully good cycle, 0 = never
    volatile uint32_t ina_reinit_try;         // re-init attempts of offline INA modules (all modules)
    volatile uint32_t ina_reinit_ok;
    volatile uint32_t dht_cycle_last_ms;      // duration of the last / the longest DHT+board cycle
    volatile uint32_t dht_cycle_max_ms;
    volatile uint32_t dc_cycle_last_ms;       // duration of the last / the longest DC cycle
    volatile uint32_t dc_cycle_max_ms;
    volatile uint32_t dc_overruns;            // DC cycles that took longer than Config.Measurement.DcMs
    // PZEM-004T modules and UART buses (sensor_pzem.cpp, uart_bus.cpp; written by the PZEM task, or by the main / stack
    // context while that task does not exist yet)
    volatile uint32_t pzem_ok[n_PZEM];        // good polls
    volatile uint32_t pzem_timeout[n_PZEM];   // no answer within Config.Sensor.Pzem.TimeoutMs
    volatile uint32_t pzem_crc[n_PZEM];       // answers with a CRC error
    volatile uint32_t pzem_proto[n_PZEM];     // short / wrong frames, other address, Modbus exception, bus not usable
    volatile uint32_t pzem_offline[n_PZEM];   // number of transitions to the offline state
    volatile uint32_t pzem_last_ok_s[n_PZEM]; // uptime (s) + 1 of the last good poll, 0 = never
    volatile uint32_t pzem_reinit_try;        // probes of offline PZEM modules (all modules)
    volatile uint32_t pzem_reinit_ok;
    volatile uint32_t pzem_cycle_last_ms;     // duration of the last / the longest PZEM cycle
    volatile uint32_t pzem_cycle_max_ms;
    volatile uint32_t pzem_overruns;          // PZEM cycles that took longer than Config.Measurement.PzemMs
    volatile uint32_t uart_ok[n_UART];        // transfers with an answer (>= 1 byte) / without one or with a driver error
    volatile uint32_t uart_fail[n_UART];
    // ---- written by the Zigbee stack task (zigbee_manager.cpp) ----
    volatile uint32_t zb_default_rsp;         // received ZCL Default Responses (all)
    volatile uint32_t zb_default_rsp_err;     // ... of which with a non-zero status
    volatile uint32_t zb_join_ok;             // steering (join) successes / failures
    volatile uint32_t zb_join_fail;
    volatile uint32_t zb_leave;               // network leave signals
    volatile uint32_t zb_start_fail;          // BDB start/initialization failures
    volatile uint32_t rep_stack_free;         // "Rep_defaults" one-shot task: minimum free stack + 1 (bytes), 0 = it has not run
};

extern diag_counters_t g_diag;

// Seconds since boot.
uint32_t diag_uptime_s();
// Counter helpers (single writer per counter).
static inline void diag_inc(volatile uint32_t &c) { c = c + 1; }
// "Last OK" time stamp: uptime + 1, so that 0 can mean "never" without any initialization.
void diag_mark(volatile uint32_t &t);

// Starts the heartbeat task (no-op if Config.Health.HeartbeatS == 0). Call once, early in app_main().
void heartbeat_start();
