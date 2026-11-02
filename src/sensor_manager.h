#pragma once
#include <stdint.h>

// ZCL "invalid / not measurable" attribute values (ZCL spec: Temperature 0x8000, Relative Humidity 0xFFFF,
// Electrical Measurement DC int16 values 0x8000). Written to the attributes when a sensor is
// unavailable (4.7); the Z2M converter maps them to null.
static constexpr int16_t  ZCL_INVALID_TEMP = (int16_t)0x8000;
static constexpr uint16_t ZCL_INVALID_HUM  = 0xFFFF;
static constexpr int16_t  ZCL_INVALID_DC   = (int16_t)0x8000;

// Divisors of the DC Electrical Measurement attributes (the same values are written to the endpoints' divisor
// attributes and used for the reporting thresholds). The measurement code produces: voltage in mV, current in mA,
// power in cW (10 mW units, because of int16). Changing a divisor therefore also needs the scaling in dc_cycle().
static constexpr uint16_t ZCL_DC_DIV_V = 1000;
static constexpr uint16_t ZCL_DC_DIV_I = 1000;
static constexpr uint16_t ZCL_DC_DIV_P = 100;

void init_native_i2c_buses();
void init_sensors_dht();
void init_sensors_ina();
void init_sensor_board();
void start_measurement_tasks();
void measure_all_once();
void ina_diag();   // INA3221 register dump (called from app_main() when Config.Logging.InaDiag is set)