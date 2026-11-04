#pragma once
// Zigbee stack lifecycle: NVS, stack task, commissioning (join / rejoin / retry) and the connection state.
// Endpoints: zigbee_endpoints.h, reporting defaults: zigbee_reporting.h, attribute writes: zigbee_attr.h,
// stack/platform configuration: zigbee_config.h, factory-reset button: button.h.
#include <stdint.h>

void init_zb_stack();

// Factory reset of the Zigbee settings, then a restart after Config.Device.ResetRebootDelayMs (the restart runs the
// normal factory-new start: firmware reporting defaults, then steering). Used by the button and after a network
// leave without rejoin. Blocks until the restart.
void zb_factory_reset_and_restart();

// ZB Global flag: set after the network join (or at a start on a configured device), cleared at the leave
extern volatile bool is_zigbee_connected;
