// Sys
#include "esp_log.h"
// External Libraries
#include "esp_zigbee.h"
#include "ezbee/zha.h"
#include "ezbee/zcl/cluster/binary_input.h"
#include "ezbee/zcl/cluster/metering.h"
// Internal components
#include "zigbee_attr.h"
#include "zigbee_manager.h"   // is_zigbee_connected
#include "zcl_values.h"
#include "health.h"
#include "config.h"

#define ZB_SET(ep, cl, at, val) do {                                              \
    auto st_ = ezb_zcl_set_attr_value((ep), (cl), EZB_ZCL_CLUSTER_SERVER, (at),   \
                                      EZB_ZCL_STD_MANUF_CODE, (val), false);      \
    static uint32_t n_ = 0;                                                       \
    if ((int)st_ != 0 && (n_++ % Config.Logging.SetAttrFailLogEvery) == 0)                                       \
        LOGW_ZB("set_attr ep=0x%02X cl=0x%04X attr=0x%04X st=%d", \
                 (unsigned)(ep), (unsigned)(cl), (unsigned)(at), (int)st_);       \
} while (0)

// ZCL attribute write access.
// Normal operation (measurement tasks): only after the network join, under the Zigbee lock.
// pre_start (measure_all_once, called from the stack context): no join needed, no lock needed.
static inline bool zb_write_allowed(bool pre_start) { return pre_start || is_zigbee_connected; }
static inline void zb_lock(bool pre_start)   { if (!pre_start) esp_zigbee_lock_acquire(portMAX_DELAY); }
static inline void zb_unlock(bool pre_start) { if (!pre_start) esp_zigbee_lock_release(); }

// Attribute writers (one lock section each). Return false if the write is not allowed yet (not joined).
bool zb_write_board(int16_t temp, bool pre_start) {
    if (!zb_write_allowed(pre_start)) return false;
    zb_lock(pre_start);
    ZB_SET(Config.Zigbee.Board.Endpoint, EZB_ZCL_CLUSTER_ID_TEMPERATURE_MEASUREMENT,
           EZB_ZCL_ATTR_TEMPERATURE_MEASUREMENT_MEASURED_VALUE_ID, &temp);
    zb_unlock(pre_start);
    return true;
}

bool zb_write_dht(uint8_t ep, int16_t temp, uint16_t hum, bool pre_start) {
    if (!zb_write_allowed(pre_start)) return false;
    zb_lock(pre_start);
    ZB_SET(ep, EZB_ZCL_CLUSTER_ID_TEMPERATURE_MEASUREMENT,
           EZB_ZCL_ATTR_TEMPERATURE_MEASUREMENT_MEASURED_VALUE_ID, &temp);
    ZB_SET(ep, EZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
           EZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_MEASURED_VALUE_ID, &hum);
    zb_unlock(pre_start);
    return true;
}

bool zb_write_dc(uint8_t ep, int16_t v, int16_t i, int16_t p, bool pre_start) {
    if (!zb_write_allowed(pre_start)) return false;
    zb_lock(pre_start);
    ZB_SET(ep, EZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_VOLTAGE_ID, &v);
    ZB_SET(ep, EZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_CURRENT_ID, &i);
    ZB_SET(ep, EZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_POWER_ID, &p);
    zb_unlock(pre_start);
    return true;
}

bool zb_write_pzem(uint8_t ep, const pzem_zcl_t &z, bool pre_start) {
    if (!zb_write_allowed(pre_start)) return false;
    // ezb_zcl_set_attr_value takes a non-const pointer: write from local copies
    uint16_t v = z.voltage, i = z.current, f = z.freq;
    int16_t  p = z.power;
    int8_t   pf = z.pf;
    uint64_t e = z.energy;   // uint48 attribute: the stack takes the low 6 bytes (little endian, same as the 6 byte wire format)
    zb_lock(pre_start);
    ZB_SET(ep, EZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_RMS_VOLTAGE_ID, &v);
    ZB_SET(ep, EZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_RMS_CURRENT_ID, &i);
    ZB_SET(ep, EZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_ACTIVE_POWER_ID, &p);
    ZB_SET(ep, EZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_AC_FREQUENCY_ID, &f);
    ZB_SET(ep, EZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_POWER_FACTOR_ID, &pf);
    ZB_SET(ep, EZB_ZCL_CLUSTER_ID_METERING, EZB_ZCL_ATTR_METERING_CURRENT_SUMMATION_DELIVERED_ID, &e);
    zb_unlock(pre_start);
    return true;
}

void zb_clear_pzem_energy_reset(uint8_t ep, bool pre_start) {
    if (!zb_write_allowed(pre_start)) return;
    uint8_t v = 0;
    zb_lock(pre_start);
    ZB_SET(ep, ZB_DIAG_CLUSTER_ID, ZB_DIAG_ATTR_ENERGY_RESET, &v);
    zb_unlock(pre_start);
}

// 4.8: device fault indication on the board endpoint (Binary Input + fault mask attribute).
// Called at the end of every measurement cycle; writes only when the mask changed (under the Zigbee lock
// so the two measurement tasks cannot interleave their compare/write).
static bool     s_diag_written = false;
static uint32_t s_diag_last = 0;

void zb_publish_fault(bool pre_start) {
    if (!zb_write_allowed(pre_start)) return;
    zb_lock(pre_start);
    uint32_t mask = health_zb_mask();
    if (!s_diag_written || mask != s_diag_last) {
        bool    pv    = (mask != 0);
        uint8_t flags = pv ? EZB_ZCL_BINARY_INPUT_STATUS_FLAGS_FAULT : 0;
        const uint8_t ep = Config.Zigbee.Board.Endpoint;
        ZB_SET(ep, EZB_ZCL_CLUSTER_ID_BINARY_INPUT, EZB_ZCL_ATTR_BINARY_INPUT_PRESENT_VALUE_ID, &pv);
        ZB_SET(ep, EZB_ZCL_CLUSTER_ID_BINARY_INPUT, EZB_ZCL_ATTR_BINARY_INPUT_STATUS_FLAGS_ID, &flags);
        ZB_SET(ep, ZB_DIAG_CLUSTER_ID, ZB_DIAG_ATTR_FAULT_MASK, &mask);
        s_diag_last = mask;
        s_diag_written = true;
    }
    zb_unlock(pre_start);
}

