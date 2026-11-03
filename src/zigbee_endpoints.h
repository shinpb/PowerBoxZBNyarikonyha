#pragma once
// The device model: endpoints and clusters (board EP 1: Basic + temperature + fault indication,
// DHT EPs, 15 DC EPs; the endpoint numbers come from Config.Zigbee). Attribute IDs of the manufacturer
// specific diagnostic cluster: zcl_values.h.
#include "ezbee/zha.h"

// Builds all endpoints into dev_desc (the caller registers the device descriptor).
void zb_build_endpoints(ezb_af_device_desc_t dev_desc);
