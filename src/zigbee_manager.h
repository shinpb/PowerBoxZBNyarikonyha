#pragma once
// Zigbee stack lifecycle: NVS, stack task, commissioning (join / rejoin / retry) and the connection state.
// Endpoints: zigbee_endpoints.h, reporting defaults: zigbee_reporting.h, attribute writes: zigbee_attr.h,
// stack/platform configuration: zigbee_config.h, factory-reset button: button.h.
#include <stdint.h>

void init_zb_stack();

// ZB Global flag: set after the network join (or at a start on a configured device), cleared at the leave
extern volatile bool is_zigbee_connected;
