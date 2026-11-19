#pragma once
#include "protocol.h"
#include <stddef.h>

#define TELEMETRY_STREAMS 10
#define TELEMETRY_MESSAGE_MAX 1200
#define TELEMETRY_HEADER 8

typedef struct {
    uint64_t time_us;
    float integral, envelope;
    uint16_t window_ms, reserved;
    uint32_t sequence;
} integration_record;

typedef struct {
    uint64_t time_us;
    uint32_t frame;
    uint16_t rate, size, hop;
    uint8_t window, remove_mean;
    float power[FFT_BINS];
} spectrum_record;

_Static_assert(sizeof(integration_record) == 24, "Integration wire layout");
_Static_assert(sizeof(spectrum_record) == 536, "Spectrum wire layout");

void telemetry_end(void);
uint32_t telemetry_session(void);
uint32_t telemetry_epoch(void);
uint32_t telemetry_changed(void);
bool telemetry_subscribe(uint16_t mask);
void telemetry_append(unsigned stream, const void *record, unsigned size, uint32_t epoch);
void telemetry_flush(unsigned stream);
extern const uint16_t telemetry_sizes[TELEMETRY_STREAMS];
