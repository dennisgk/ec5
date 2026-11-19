#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
void serial_start(void);
void serial_prepare(void);
bool serial_send_telemetry(const void *data, size_t size);
bool serial_reply(uint32_t request, const void *data, size_t size);
void serial_close(void);
char serial_session(void);
void serial_set_peer_window(unsigned window);
// Called only from the cold command worker; request is operation + arguments.
size_t controller_command(const uint8_t *request, size_t length, uint8_t reply[1024],
                          bool *initialized);
