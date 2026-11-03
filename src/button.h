#pragma once
// Factory-reset button (Config.Device.ButtonPin, e.g. the BOOT button): hold it for Config.Device.ButtonHoldMs
// to reset the Zigbee settings and restart the device.

// Configures the pin and starts the button task. Call after init_zb_stack().
void button_init();
