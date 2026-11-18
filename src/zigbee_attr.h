#pragma once
// ZCL attribute writes of the measurement results (the measurement code does not touch the Zigbee API).
// Normal operation (measurement tasks): only after the network join, under the Zigbee lock.
// pre_start = true (measure_all_once, called from the stack context): no join needed, no lock needed.
// The writers return false if the write is not allowed yet (not joined); the caller may retry later.
#include <stdint.h>
#include "pzem_proto.h"   // pzem_zcl_t

bool zb_write_board(int16_t temp, bool pre_start);
bool zb_write_dht(uint8_t ep, int16_t temp, uint16_t hum, bool pre_start);
bool zb_write_dc(uint8_t ep, int16_t v, int16_t i, int16_t p, bool pre_start);
// PZEM-004T endpoint: AC Electrical Measurement (V, I, P, f, PF) + Metering (energy), one lock section.
// The raw units / the invalid values: pzem_proto.h, zcl_values.h.
// Sets the energy reset trigger attribute of a PZEM endpoint back to 0 (after the request was handled).
void zb_clear_pzem_energy_reset(uint8_t ep, bool pre_start);
bool zb_write_pzem(uint8_t ep, const pzem_zcl_t &z, bool pre_start);

// 4.8: device fault indication on the board endpoint (Binary Input + fault mask attribute).
// Called at the end of every measurement cycle; writes only when the mask changed.
void zb_publish_fault(bool pre_start);
