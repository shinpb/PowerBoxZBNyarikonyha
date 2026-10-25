#pragma once

void init_native_i2c_buses();
void init_sensors_dht();
void init_sensors_ina();
void init_sensor_board();
void start_measurement_tasks();
void measure_all_once();
void dht_measurement_task(void* pvParameters);
void dc_measurement_task(void* pvParameters);
void ina_diag();