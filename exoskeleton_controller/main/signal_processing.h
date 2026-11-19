#pragma once
#include "protocol.h"

typedef struct {
    bool enabled;
    uint16_t frequency, bandwidth;
    int16_t attenuation;
} notch_filter_config;

typedef struct {
    bool integration_enabled, fourier_enabled;
    bool notch_enabled;
    uint16_t notch_frequency, notch_bandwidth; // tenths of Hz
    int16_t notch_attenuation;                 // signed tenths of dB at center frequency
    uint8_t notch_count;
    notch_filter_config notches[MAX_NOTCH_FILTERS];
    uint16_t integration_ms, hop;
    uint8_t window, remove_mean;
    uint32_t revision;
} processing_config;

typedef struct {
    bool ready;
    uint64_t time_us;
    uint16_t window_ms;
    float integral, envelope;
} integration_result;

typedef struct {
    bool ready;
    uint64_t time_us;
    uint32_t frame;
    uint16_t hop;
    uint8_t window, remove_mean;
    float power[FFT_BINS];
} fourier_result;

typedef struct {
    processing_config config;
    float integration[1000];
    unsigned integration_head, integration_count;
    double sum, weighted_sum;
    float samples[FFT_SIZE];
    unsigned fft_head, fft_count, since_fft;
    float real[FFT_SIZE], imaginary[FFT_SIZE];
    uint32_t previous_sequence;
    bool has_sequence;
    struct {
        float b0, b1, b2, a2, z1, z2;
    } notch[MAX_NOTCH_FILTERS];
    integration_result integrated;
    fourier_result spectrum;
} processor;

void processing_initialize(void);
// Compare only HP clock values. LP-derived signal timestamps may drift relative
// to HP time and must not determine whether a newly published result is fresh.
static inline bool processing_result_fresh(bool ready, uint64_t now_us, uint64_t published_us,
                                           uint64_t maximum_age_us)
{
    return ready && published_us && now_us >= published_us &&
           now_us - published_us <= maximum_age_us;
}
void processing_reset(processor *state, const processing_config *config);
float processing_push(processor *state, float millivolts, uint64_t time_us, uint32_t sequence);
