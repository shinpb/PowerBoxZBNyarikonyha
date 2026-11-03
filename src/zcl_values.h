#pragma once
// The ZCL data model shared by the measurement code, the endpoint builders and the reporting setup.
#include <stdint.h>

// ZCL "invalid / not measurable" attribute values (ZCL spec: Temperature 0x8000, Relative Humidity 0xFFFF,
// Electrical Measurement DC int16 values 0x8000). Written to the attributes when a sensor is
// unavailable (4.7); the Z2M converter maps them to null.
static constexpr int16_t  ZCL_INVALID_TEMP = (int16_t)0x8000;
static constexpr uint16_t ZCL_INVALID_HUM  = 0xFFFF;
static constexpr int16_t  ZCL_INVALID_DC   = (int16_t)0x8000;

// Divisors of the DC Electrical Measurement attributes (the same values are written to the endpoints' divisor
// attributes and used for the reporting thresholds). The measurement code produces: voltage in mV, current in mA,
// power in cW (10 mW units, because of int16). Changing a divisor therefore also needs the scaling in ina_cycle().
static constexpr uint16_t ZCL_DC_DIV_V = 1000;
static constexpr uint16_t ZCL_DC_DIV_I = 1000;
static constexpr uint16_t ZCL_DC_DIV_P = 100;

// Diagnostics on the board endpoint (EP 0x01), see PLAN.md 4.8:
//  - standard Binary Input cluster (0x000F): PresentValue = 1 when any sensor/bus fault is active,
//    StatusFlags FAULT bit follows it
//  - manufacturer-specific cluster ZB_DIAG_CLUSTER_ID, attribute ZB_DIAG_ATTR_FAULT_MASK (bitmap32):
//    health_zb_mask() (sensor bits 0..15, bus bits 16..23)
static constexpr uint16_t ZB_DIAG_CLUSTER_ID      = 0xFC11;
static constexpr uint16_t ZB_DIAG_ATTR_FAULT_MASK = 0x0000;
