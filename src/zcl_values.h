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

// PZEM-004T AC meters (Config.Zigbee.Pzem endpoints): Electrical Measurement (AC) + Metering clusters.
// Invalid / unavailable values (ZCL: the "invalid" value of the attribute's data type): uint16 0xFFFF
// (RMSVoltage, RMSCurrent, ACFrequency), int16 0x8000 (ActivePower), int8 0x80 (PowerFactor),
// uint48 0xFFFFFFFFFFFF (CurrentSummationDelivered). The Z2M converter maps them to null.
static constexpr uint16_t ZCL_INVALID_AC_U16  = 0xFFFF;
static constexpr int16_t  ZCL_INVALID_AC_P    = (int16_t)0x8000;
static constexpr int8_t   ZCL_INVALID_AC_PF   = (int8_t)0x80;
static constexpr uint64_t ZCL_INVALID_ENERGY  = 0xFFFFFFFFFFFFULL;

// Raw units of the AC attributes (the same values are written to the endpoints' multiplier/divisor attributes,
// used for the reporting thresholds and by the converter): voltage 0.1 V, current 10 mA (uint16 range 655 A, the PZEM
// measures at most 100 A), power 1 W (int16 range 32.7 kW, the PZEM measures at most 23 kW), frequency 0.1 Hz,
// power factor in percent (int8), energy in Wh with the Metering divisor 1000 (= kWh).
static constexpr uint16_t ZCL_AC_DIV_V = 10;
static constexpr uint16_t ZCL_AC_DIV_I = 100;
static constexpr uint16_t ZCL_AC_DIV_P = 1;
static constexpr uint16_t ZCL_AC_DIV_F = 10;
static constexpr uint16_t ZCL_AC_MUL   = 1;          // all AC multipliers
static constexpr uint32_t ZCL_METER_MUL   = 1;       // Metering Multiplier (uint24)
static constexpr uint32_t ZCL_METER_DIV_E = 1000;    // Metering Divisor (uint24): raw Wh / 1000 = kWh
static constexpr uint8_t  ZCL_METER_FORMAT = 0x3B;   // SummationFormatting: 7 digits left, 3 digits right of the decimal point

// Diagnostics on the board endpoint (EP 0x01), see PLAN.md 4.8:
//  - standard Binary Input cluster (0x000F): PresentValue = 1 when any sensor/bus fault is active,
//    StatusFlags FAULT bit follows it
//  - manufacturer-specific cluster ZB_DIAG_CLUSTER_ID, attribute ZB_DIAG_ATTR_FAULT_MASK (bitmap32):
//    health_zb_mask() (sensor bits 0..15, bus bits 16..23; bit map in health.h)
static constexpr uint16_t ZB_DIAG_CLUSTER_ID      = 0xFC11;
static constexpr uint16_t ZB_DIAG_ATTR_FAULT_MASK = 0x0000;
