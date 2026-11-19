#pragma once
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

static inline bool emg_counter_is_new(uint8_t current, uint8_t previous, bool initialized)
{
    // Reset is zero; the counter advances when the first conversion completes.
    // Never fabricate a repeated old sample merely because time has passed.
    return initialized ? current != previous : current != 0;
}

// Matches sketch_sep30a: ADC-input differential mV, not electrode-referred mV.
static inline int emg_gain_code(unsigned gain)
{
    for (int code = 0; code < 8; ++code) {
        if (gain == (1u << code)) {
            return code;
        }
    }
    return -1;
}

static inline int emg_filter_code(float frequency, float capacitance)
{
    if (!isfinite(frequency) || frequency <= 0 || capacitance <= 0) {
        return -1;
    }
    float total = 1.0f / (6.28318530718f * frequency * capacitance);
    float resistance = total - 4700.0f - 75.0f;
    int code = (int)roundf((resistance / 100000.0f) * 256.0f);
    return code >= 0 && code <= 256 ? code : -1;
}

static inline float emg_filter_frequency(unsigned code, float capacitance)
{
    float resistance = 4700.0f + 75.0f + ((float)code / 256.0f) * 100000.0f;
    return 1.0f / (6.28318530718f * resistance * capacitance);
}

static inline int32_t emg_decode24(const uint8_t bytes[3])
{
    uint32_t raw = ((uint32_t)bytes[0] << 16) | ((uint32_t)bytes[1] << 8) | bytes[2];
    return (int32_t)(raw & 0x7FFFFF) - ((raw & 0x800000) ? 8388608 : 0);
}

static inline float emg_mv_per_code(unsigned adc_gain)
{
    return emg_gain_code(adc_gain) >= 0 ? 2048.0f / (8388608.0f * adc_gain) : 0;
}
