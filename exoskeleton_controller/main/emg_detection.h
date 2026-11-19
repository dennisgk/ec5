#pragma once
#include "protocol.h"
#include <math.h>

typedef struct {
    float threshold;
    int16_t velocity; // tenths of rad/s
    float maximum;
    int16_t maximum_velocity;
    bool ignore_enabled;
    float ignore_low, ignore_high;
    uint8_t excluded[17];
    uint32_t revision;
} emg_detection_config;

static inline float emg_detection_peak(const float power[FFT_BINS], const emg_detection_config *c)
{
    float peak = 0;
    for (unsigned bin = 0; bin < FFT_BINS; ++bin) {
        float hz = bin * (SAMPLE_RATE / (float)FFT_SIZE);
        if ((c->excluded[bin / 8] & (1u << (bin % 8))) ||
            (c->ignore_enabled && hz >= c->ignore_low && hz <= c->ignore_high)) {
            continue;
        }
        if (isfinite(power[bin]) && power[bin] > peak) {
            peak = power[bin];
        }
    }
    return peak;
}

static inline float emg_detection_velocity(float peak, const emg_detection_config *c)
{
    if (!isfinite(peak) || peak < c->threshold) {
        return 0;
    }
    if (peak >= c->maximum) {
        return c->maximum_velocity / 10.0f;
    }
    float blend = (peak - c->threshold) / (c->maximum - c->threshold);
    return (c->velocity + blend * (c->maximum_velocity - c->velocity)) / 10.0f;
}
