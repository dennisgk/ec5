#pragma once
#include <stdbool.h>
#include <stdint.h>
void ble_control_init(void);
bool ble_control_available(void);
void ble_control_advertise(bool enabled);
// Fixed-size cold-path diagnostics; never called by acquisition.
void ble_control_diagnostics(uint32_t words[8]);
