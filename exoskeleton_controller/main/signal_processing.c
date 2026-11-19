#include "signal_processing.h"
#include <math.h>
#include <string.h>

static float hann[FFT_SIZE], twiddle_real[FFT_SIZE / 2], twiddle_imaginary[FFT_SIZE / 2];
static float hann_energy;

void processing_initialize(void)
{
    const double pi = 3.14159265358979323846;
    hann_energy = 0;
    for (unsigned n = 0; n < FFT_SIZE; ++n) {
        hann[n] = (float)(0.5 - 0.5 * cos(2 * pi * n / (FFT_SIZE - 1)));
        hann_energy += hann[n] * hann[n];
    }
    for (unsigned n = 0; n < FFT_SIZE / 2; ++n) {
        twiddle_real[n] = (float)cos(-2 * pi * n / FFT_SIZE);
        twiddle_imaginary[n] = (float)sin(-2 * pi * n / FFT_SIZE);
    }
}

void processing_reset(processor *state, const processing_config *config)
{
    memset(state, 0, sizeof(*state));
    state->config = *config;
    unsigned count = config->notch_count ? config->notch_count : (config->notch_enabled ? 1 : 0);
    for (unsigned n = 0; n < count; ++n) {
        notch_filter_config filter =
            config->notch_count
                ? config->notches[n]
                : (notch_filter_config){true, config->notch_frequency, config->notch_bandwidth,
                                        config->notch_attenuation};
        if (!filter.enabled) {
            continue;
        }
        const double pi = 3.14159265358979323846;
        double beta = tan(pi * filter.bandwidth / (10.0 * SAMPLE_RATE));
        // Finite-depth notch. The signed setting directly specifies center gain;
        // bandwidth is measured at half this attenuation in dB (mid-gain).
        double center_gain = pow(10.0, filter.attenuation / 200.0);
        beta /= sqrt(center_gain);
        state->notch[n].b0 = (float)((1.0 + center_gain * beta) / (1.0 + beta));
        state->notch[n].b2 = (float)((1.0 - center_gain * beta) / (1.0 + beta));
        state->notch[n].b1 =
            (float)(-2.0 * cos(2 * pi * filter.frequency / (10.0 * SAMPLE_RATE)) / (1.0 + beta));
        state->notch[n].a2 = (float)((1.0 - beta) / (1.0 + beta));
    }
}

static void integrate(processor *s, float value, uint64_t time_us)
{
    unsigned size = s->config.integration_ms; // 1 sample/ms at nominal 1000 SPS.
    if (s->integration_count == size) {
        s->weighted_sum -= s->sum;
        s->sum -= s->integration[s->integration_head];
        s->weighted_sum += size * (double)value;
    } else {
        ++s->integration_count;
        s->weighted_sum += s->integration_count * (double)value;
    }
    s->sum += value;
    s->integration[s->integration_head] = value;
    s->integration_head = (s->integration_head + 1) % size;
    s->integrated = (integration_result){
        .ready = s->integration_count == size,
        .time_us = time_us,
        .window_ms = (uint16_t)size,
        .integral = (float)(s->weighted_sum / (size * (double)SAMPLE_RATE)),
        .envelope = (float)(s->weighted_sum / (size * (size + 1) * 0.5)),
    };
}

static void transform(processor *s, uint64_t time_us)
{
    float mean = 0;
    if (s->config.remove_mean) {
        for (unsigned n = 0; n < FFT_SIZE; ++n) {
            mean += s->samples[n] / FFT_SIZE;
        }
    }
    for (unsigned n = 0; n < FFT_SIZE; ++n) {
        unsigned reverse = 0;
        for (unsigned bit = 0; bit < 8; ++bit) {
            reverse = (reverse << 1) | ((n >> bit) & 1);
        }
        float value = s->samples[(s->fft_head + n) % FFT_SIZE] - mean;
        s->real[reverse] = value * (s->config.window ? hann[n] : 1);
        s->imaginary[reverse] = 0;
    }
    for (unsigned length = 2; length <= FFT_SIZE; length *= 2) {
        unsigned half = length / 2;
        for (unsigned block = 0; block < FFT_SIZE; block += length) {
            for (unsigned j = 0; j < half; ++j) {
                unsigned w = j * FFT_SIZE / length;
                unsigned a = block + j, b = a + half;
                float re = twiddle_real[w] * s->real[b] - twiddle_imaginary[w] * s->imaginary[b];
                float im = twiddle_real[w] * s->imaginary[b] + twiddle_imaginary[w] * s->real[b];
                s->real[b] = s->real[a] - re;
                s->imaginary[b] = s->imaginary[a] - im;
                s->real[a] += re;
                s->imaginary[a] += im;
            }
        }
    }
    float energy = s->config.window ? hann_energy : FFT_SIZE;
    for (unsigned bin = 0; bin < FFT_BINS; ++bin) {
        float power = s->real[bin] * s->real[bin] + s->imaginary[bin] * s->imaginary[bin];
        s->spectrum.power[bin] =
            power / (SAMPLE_RATE * energy) * (bin && bin != FFT_SIZE / 2 ? 2 : 1);
    }
    s->spectrum.ready = true;
    s->spectrum.time_us = time_us;
    s->spectrum.hop = s->config.hop;
    s->spectrum.window = s->config.window;
    s->spectrum.remove_mean = s->config.remove_mean;
    ++s->spectrum.frame;
}

float processing_push(processor *s, float millivolts, uint64_t time_us, uint32_t sequence)
{
    if (s->has_sequence && sequence != s->previous_sequence + 1) {
        processing_config config = s->config;
        processing_reset(s, &config);
    }
    s->has_sequence = true;
    s->previous_sequence = sequence;
    unsigned count =
        s->config.notch_count ? s->config.notch_count : (s->config.notch_enabled ? 1 : 0);
    for (unsigned n = 0; n < count; ++n) {
        if (s->config.notch_count && !s->config.notches[n].enabled) {
            continue;
        }
        // Transposed direct form II: one biquad, no allocation or per-sample
        // trigonometry. Integration and FFT consume this same output.
        float filtered = s->notch[n].b0 * millivolts + s->notch[n].z1;
        s->notch[n].z1 = s->notch[n].b1 * (millivolts - filtered) + s->notch[n].z2;
        s->notch[n].z2 = s->notch[n].b2 * millivolts - s->notch[n].a2 * filtered;
        millivolts = filtered;
    }
    if (s->config.integration_enabled) {
        integrate(s, fabsf(millivolts), time_us);
    }
    if (s->config.fourier_enabled) {
        s->samples[s->fft_head] = millivolts;
        s->fft_head = (s->fft_head + 1) % FFT_SIZE;
        if (s->fft_count < FFT_SIZE) {
            ++s->fft_count;
            if (s->fft_count != FFT_SIZE) {
                return millivolts;
            }
            s->since_fft = s->config.hop;
        } else {
            ++s->since_fft;
        }
        if (s->since_fft >= s->config.hop) {
            transform(s, time_us);
            s->since_fft = 0;
        }
    }
    return millivolts;
}
