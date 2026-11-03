#pragma once
// Reporting setup: firmware default reporting (Config.Reporting) and the Default Response disable.

// Default Response off for every reportable attribute. Quick, runs at EVERY start (after the endpoint registration).
void zb_disable_default_rsp();

// Factory-new start: starts the task that writes the default reporting records and then the steering.
// true: the task was started and it will start the steering; false: nothing to write, the caller starts the steering.
bool zb_start_reporting_defaults();
