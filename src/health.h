#pragma once
// Global system health: one 32-bit error mask per main unit (0 = OK).
// A GPIO pin per unit and a global pin indicate the state: mask == 0 -> low, anything else -> high.
// Pins are configured in Config.Health (a negative pin number disables that indication).
#include <stdint.h>

typedef enum {
    HM_ZIGBEE = 0,
    HM_BUS,
    HM_SENSOR,
    HM_COUNT
} health_module_t;

// ---- Error flags (bit masks, per module) ----
// Common to every module: the unit is not initialized yet. health_init() sets it for all modules,
// the init code clears it once the unit is up. Rationale: "OK" must be proven, not assumed -
// a unit that hangs or never gets initialized keeps its pin high.
#define HF_STARTING         (1u << 31)
// HM_ZIGBEE
#define HF_ZB_NOT_JOINED    (1u << 0)   // not (yet) joined / left the network
#define HF_ZB_START_FAILED  (1u << 1)   // BDB start/initialization failed (retrying)
#define HF_ZB_REG_FAILED    (1u << 2)   // endpoint registration failed
// HM_BUS (bits 0..7 go to the Zigbee fault mask as bits 16..23)
#define HF_BUS_I2C_INIT     (1u << 0)   // i2cdev_init() failed
#define HF_BUS_I2C(b)       (1u << (1 + (b)))   // all devices on I2C bus b fail (b = 0..2, bits 1..3)
#define HF_BUS_UART_INIT    (1u << 4)   // UART driver / pin setup of a bus failed or its pin configuration is invalid (log: which bus)
#define HF_BUS_UART(b)      (1u << (5 + (b)))   // all PZEM modules of UART bus b fail (b = 0..2, bits 5..7; only with >= 2 modules on the bus)
// HM_SENSOR (bits 0..15 go to the Zigbee fault mask as they are)
#define HF_SENS_BOARD       (1u << 0)   // board temperature sensor
#define HF_SENS_DHT(i)      (1u << (1 + (i)))   // DHT i (consecutive read failures), i = 0..6
#define HF_SENS_INA(i)      (1u << (8 + (i)))   // INA3221 module i, i = 0..4 (bits 8..12)
#define HF_SENS_PZEM(i)     (1u << (13 + (i)))  // PZEM-004T module i, i = 0..2 (bits 13..15)

// Call once, first thing in app_main(): every module starts in the HF_STARTING state (pins high).
void health_init();
// Set / clear error flags of a module. Thread-safe. The pins are updated and the change is logged
// only when the state actually changes.
void health_set(health_module_t m, uint32_t flags);
void health_clear(health_module_t m, uint32_t flags);
uint32_t health_get(health_module_t m);
// Global state: OR of all modules (0 = everything OK).
uint32_t health_get_global();
// Fault mask reported to Zigbee (4.8): sensor faults in bits 0..15 (HF_SENS_*), bus faults in bits 16..23
// (HF_BUS_* << 16). HF_STARTING and the Zigbee module itself are not included (a device that is not
// joined cannot report). 0 = no fault.
uint32_t health_zb_mask();
