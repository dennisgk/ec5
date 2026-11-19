#pragma once
#include <stddef.h>
#include <stdint.h>
#define AUTOCONFIG_MAX 32768
void autoconfig_boot(void);
uint8_t autoconfig_begin(uint16_t size);
uint8_t autoconfig_write(uint16_t offset, const uint8_t *data, size_t size);
uint8_t autoconfig_commit(void);
uint8_t autoconfig_delete(void);
void autoconfig_cancel(void);
uint32_t autoconfig_error(void);
