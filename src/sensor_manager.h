#pragma once
// Sensor modules: board temperature (sensor_board.cpp), DHT (sensor_dht.cpp), INA3221 over I2C (sensor_ina.cpp),
// PZEM-004T over UART (sensor_pzem.cpp, buses: uart_bus.h);
// the measurement tasks and the start order live in sensor_manager.cpp.
#include <stdint.h>

// ---- init (called from app_main) ----
void init_sensor_board();
void init_sensors_dht();
void init_native_i2c_buses();
void init_sensors_ina();
void ina_diag();   // INA3221 register dump (called from app_main() when Config.Logging.InaDiag is set)
void init_sensors_pzem();   // after init_uart_buses() (uart_bus.h)
void pzem_diag();   // PZEM register dump (called from app_main() when Config.Logging.PzemDiag is set)

// ---- one measurement cycle of a sensor group (read, health, Zigbee attribute write) ----
// pre_start = true: called from the stack context before the tasks run (see zigbee_attr.h)
void board_cycle(bool pre_start);
void dht_cycle(bool pre_start);
void ina_cycle(bool pre_start);
void pzem_cycle(bool pre_start);

// ---- tasks ----
void start_measurement_tasks();
void measure_all_once();
