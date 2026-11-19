#include "emg_detection.h"
#include "emg_signal.h"
#include "robstride06.h"
#include "signal_processing.h"
#include "serial_frame.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static processor state;
static const double pi = 3.14159265358979323846;

static processing_config settings(void)
{
    return (processing_config){.integration_enabled = true,
                               .fourier_enabled = true,
                               .integration_ms = 100,
                               .hop = 128,
                               .window = 1,
                               .remove_mean = 1,
                               .notch_attenuation = -30};
}

static void integration_reference(void)
{
    processing_config config = settings();
    config.fourier_enabled = false;
    for (unsigned window = 10; window <= 1000; window += 99) {
        config.integration_ms = (uint16_t)window;
        processing_reset(&state, &config);
        for (unsigned n = 0; n < window * 3; ++n) {
            float value = (float)((int)(n % 17) - 8);
            processing_push(&state, value, (uint64_t)n * 1000, n);
            assert(state.integrated.ready == (n + 1 >= window));
            if (n + 1 >= window) {
                double reference = 0;
                for (unsigned j = 0; j < window; ++j) {
                    unsigned index = n + 1 - window + j;
                    reference += (j + 1) * fabs((double)((int)(index % 17) - 8));
                }
                assert(fabs(state.integrated.integral - reference / (window * 1000.0)) < 1e-6);
                assert(fabs(state.integrated.envelope - reference / (window * (window + 1) * 0.5)) <
                       1e-6);
            }
        }
        processing_push(&state, -5, 9999999, 999999);
        assert(!state.integrated.ready);
    }
}

static double total_power(void)
{
    double result = 0;
    for (unsigned n = 0; n < FFT_BINS; ++n) {
        result += state.spectrum.power[n] * SAMPLE_RATE / FFT_SIZE;
    }
    return result;
}

static void fourier_reference(void)
{
    processing_config config = settings();
    config.window = 0;
    config.remove_mean = 0;
    processing_reset(&state, &config);
    for (unsigned n = 0; n < FFT_SIZE; ++n) {
        processing_push(&state, (float)sin(2 * pi * 32 * n / FFT_SIZE), (uint64_t)n * 1000, n);
    }
    assert(state.spectrum.ready && state.spectrum.frame == 1);
    assert(fabs(state.spectrum.power[32] - 0.128) < 1e-6);
    assert(fabs(total_power() - 0.5) < 1e-5);
    for (unsigned n = FFT_SIZE; n < FFT_SIZE + 127; ++n) {
        processing_push(&state, 0, (uint64_t)n * 1000, n);
    }
    assert(state.spectrum.frame == 1);
    processing_push(&state, 0, 383000, 383);
    assert(state.spectrum.frame == 2);
    processing_push(&state, 0, 400000, 400);
    assert(!state.spectrum.ready);

    for (unsigned mode = 0; mode < 3; ++mode) {
        config.window = (uint8_t)(mode == 2);
        config.remove_mean = (uint8_t)(mode == 1);
        processing_reset(&state, &config);
        for (unsigned n = 0; n < FFT_SIZE; ++n) {
            float value = mode == 2 ? (float)sin(2 * pi * 32 * n / FFT_SIZE) : 1;
            processing_push(&state, value, (uint64_t)n * 1000, n);
        }
        assert(fabs(total_power() - (mode == 0 ? 1.0 : mode == 1 ? 0.0 : 0.5)) < 1e-4);
    }
    config.window = 0;
    config.remove_mean = 0;
    processing_reset(&state, &config);
    for (unsigned n = 0; n < FFT_SIZE; ++n) {
        processing_push(&state, n % 2 ? -1.0f : 1.0f, (uint64_t)n * 1000, n);
    }
    assert(fabs(state.spectrum.power[128] - 0.256) < 1e-6);
    assert(fabs(total_power() - 1.0) < 1e-5);
}

static void hardware_and_wire_vectors(void)
{
    uint8_t out[18], *cursor = out;
    write_u16(&cursor, 200);
    write_u32(&cursor, 0x12345678);
    write_u64(&cursor, UINT64_C(0x0102030405060708));
    write_float(&cursor, 1.0f);
    const uint8_t expected[] = {200, 0, 0x78, 0x56, 0x34, 0x12, 8, 7,   6,
                                5,   4, 3,    2,    1,    0,    0, 128, 63};
    assert(memcmp(out, expected, sizeof(out)) == 0);
    assert(read_u16(out) == 200);
    for (unsigned code = 0; code < 8; ++code) {
        assert(emg_gain_code(1u << code) == (int)code);
    }
    assert(emg_gain_code(256) == -1);
    assert(emg_filter_code(20, 470e-9f) == 31);
    assert(emg_filter_code(450, 10e-9f) == 78);
    uint8_t negative[] = {0xFF, 0xFF, 0xFF};
    assert(emg_decode24(negative) == -1);
    assert(fabs(emg_mv_per_code(1) * 8388608.0 - 2048.0) < 1e-9);
    assert(!emg_counter_is_new(0, 0, false));
    assert(emg_counter_is_new(0, 255, true));
}

static double notch_attenuation_power(double frequency, unsigned bandwidth, int attenuation)
{
    processing_config config = settings();
    config.integration_enabled = config.fourier_enabled = false;
    config.notch_enabled = true;
    config.notch_frequency = 600;
    config.notch_bandwidth = (uint16_t)bandwidth;
    config.notch_attenuation = (int16_t)attenuation;
    processing_reset(&state, &config);
    double energy = 0;
    for (unsigned n = 0; n < 12000; ++n) {
        float input = (float)sin(2 * pi * frequency * n / SAMPLE_RATE);
        float output = processing_push(&state, input, (uint64_t)n * 1000, n);
        if (n >= 8000) {
            energy += output * (double)output;
        }
    }
    return energy / 2000; // Power relative to a unit sine (mean-square 0.5).
}

static double notch_power(double frequency, unsigned bandwidth)
{
    return notch_attenuation_power(frequency, bandwidth, -30);
}

static void notch_reference(void)
{
    assert(fabs(notch_power(60, 20) - pow(10.0, -3.0 / 10.0)) < 0.001);
    assert(notch_power(120, 20) > 0.99);
    assert(fabs(notch_power(59, 20) - pow(10.0, -1.5 / 10.0)) < 0.02);
    assert(fabs(notch_power(61, 20) - pow(10.0, -1.5 / 10.0)) < 0.02);
    assert(notch_power(55, 100) < notch_power(55, 20) * 0.8);
    const int levels[] = {-1, -10, -30, -90, -200, -600};
    // Exact digital crossing frequencies: their separation is 2 Hz. Warping
    // makes them slightly asymmetric around the center frequency.
    double angle = pi * 2 / SAMPLE_RATE;
    double middle = acos(cos(2 * pi * 60 / SAMPLE_RATE) * cos(angle));
    double lower = (middle - angle) * SAMPLE_RATE / (2 * pi);
    double upper = (middle + angle) * SAMPLE_RATE / (2 * pi);
    assert(fabs(upper - lower - 2) < 1e-9);
    for (unsigned n = 0; n < sizeof(levels) / sizeof(levels[0]); ++n) {
        int level = levels[n];
        double expected = pow(10.0, level / 100.0);
        assert(fabs(notch_attenuation_power(60, 20, level) - expected) < expected * 0.02);
        double half_attenuation = sqrt(expected);
        assert(fabs(notch_attenuation_power(lower, 20, level) - half_attenuation) <
               half_attenuation * 0.02);
        assert(fabs(notch_attenuation_power(upper, 20, level) - half_attenuation) <
               half_attenuation * 0.02);
    }
    assert(notch_attenuation_power(60, 20, -90) < notch_attenuation_power(60, 20, -30) * 0.3);

    // Both downstream processors must see the post-notch signal, even when no
    // filtered plot is being queried. Settle on a pure 60 Hz interference tone.
    processing_config config = settings();
    config.notch_enabled = true;
    config.notch_frequency = 600;
    config.notch_bandwidth = 20;
    config.notch_attenuation = -90;
    processing_reset(&state, &config);
    for (unsigned n = 0; n < 8000; ++n) {
        processing_push(&state, (float)sin(2 * pi * 60 * n / SAMPLE_RATE), (uint64_t)n * 1000, n);
    }
    assert(state.integrated.ready && state.spectrum.ready);
    double filtered_envelope = state.integrated.envelope;
    double filtered_power = total_power();
    config.notch_enabled = false;
    processing_reset(&state, &config);
    for (unsigned n = 0; n < 8000; ++n) {
        processing_push(&state, (float)sin(2 * pi * 60 * n / SAMPLE_RATE), (uint64_t)n * 1000, n);
    }
    assert(fabs(filtered_envelope / state.integrated.envelope - pow(10.0, -9.0 / 20.0)) < 0.001);
    assert(fabs(filtered_power / total_power() - pow(10.0, -9.0 / 10.0)) < 0.001);

    // A discontinuity starts with zero filter history; never bridge missing data.
    config.notch_enabled = true;
    processing_reset(&state, &config);
    processing_push(&state, 2, 0, 0);
    float after_gap = processing_push(&state, 1, 9000000, 9000);
    processing_reset(&state, &config);
    assert(fabsf(after_gap - processing_push(&state, 1, 9000000, 9000)) < 1e-7f);
    assert(!state.integrated.ready && !state.spectrum.ready);
    config.notch_enabled = false;
    processing_reset(&state, &config);
    assert(processing_push(&state, -3.25f, 0, 0) == -3.25f);
}

static void emg_detection_reference(void)
{
    float power[FFT_BINS] = {0};
    emg_detection_config c = {
        .threshold = 0.001f, .ignore_enabled = true, .ignore_low = 50, .ignore_high = 70};
    power[16] = 100; // 62.5 Hz must not trigger the motor through mains pickup.
    power[32] = c.threshold;
    assert(emg_detection_peak(power, &c) == c.threshold);
    power[32] = 0;
    assert(emg_detection_peak(power, &c) < c.threshold);
    c.ignore_enabled = false;
    assert(emg_detection_peak(power, &c) == 100);
    c.ignore_enabled = true;
    c.ignore_low = 0;
    c.ignore_high = 500;
    assert(emg_detection_peak(power, &c) == 0);

    processing_config config = settings();
    config.integration_enabled = false;
    config.window = 0;
    config.notch_enabled = false;
    c.ignore_low = 50;
    c.ignore_high = 70;
    for (unsigned pass = 0; pass < 2; ++pass) {
        processing_reset(&state, &config);
        for (unsigned n = 0; n < FFT_SIZE; ++n) {
            float sample = (float)(sin(2 * pi * 16 * n / FFT_SIZE) +
                                   (pass ? 0.2 * sin(2 * pi * 32 * n / FFT_SIZE) : 0));
            processing_push(&state, sample, (uint64_t)n * 1000, n);
        }
        assert(state.spectrum.ready);
        assert((emg_detection_peak(state.spectrum.power, &c) >= c.threshold) == (pass != 0));
    }
    assert(processing_result_fresh(true, 1000000, 999000, 228000));
    assert(!processing_result_fresh(true, 1228000, 999000, 228000));
    assert(!processing_result_fresh(false, 1000000, 999000, 228000));
}

static void graded_curve_and_cascaded_notches(void)
{
    emg_detection_config curve = {
        .threshold = 1, .maximum = 3, .velocity = -200, .maximum_velocity = 400};
    assert(emg_detection_velocity(0.9f, &curve) == 0);
    assert(emg_detection_velocity(1, &curve) == -20);
    assert(emg_detection_velocity(2, &curve) == 10);
    assert(emg_detection_velocity(3, &curve) == 40);
    assert(emg_detection_velocity(300, &curve) == 40);
    assert(emg_detection_velocity(NAN, &curve) == 0);
    float power[FFT_BINS] = {0};
    curve.excluded[2] = 1; // 62.5 Hz
    curve.excluded[4] = 1; // 125 Hz
    power[16] = 1000;
    power[32] = 2000;
    power[64] = 2;
    assert(emg_detection_peak(power, &curve) == 2);
    memset(curve.excluded, 0, sizeof(curve.excluded));
    assert(emg_detection_peak(power, &curve) == 2000);

    processing_config config = settings();
    config.notch_enabled = true;
    config.notch_count = 2;
    config.notches[0] = (notch_filter_config){true, 600, 20, -90};
    config.notches[1] = (notch_filter_config){true, 600, 20, -90};
    processing_reset(&state, &config);
    double input_energy = 0, output_energy = 0;
    for (unsigned n = 0; n < 16000; ++n) {
        float input = (float)sin(2 * pi * 60 * n / SAMPLE_RATE);
        float output = processing_push(&state, input, (uint64_t)n * 1000, n);
        if (n > 8000) {
            input_energy += input * input;
            output_energy += output * output;
        }
    }
    assert(fabs(output_energy / input_energy - pow(10.0, -18.0 / 10.0)) < 0.0001);
    config.notches[0].enabled = config.notches[1].enabled = false;
    processing_reset(&state, &config);
    assert(processing_push(&state, -3.25f, 0, 0) == -3.25f);
}

int main(void)
{
    const uint8_t rs06_fault[8] = {0x85, 0x40, 0, 0, 1, 0, 0, 0};
    assert(rs06_u32_le(rs06_fault) == 0x4085);
    assert(rs06_u32_le(rs06_fault + 4) == 1);
    uint32_t feedback_id = (0x18u << 24) | (2u << 22) | (3u << 16) | (0x7Fu << 8) | 0xFDu;
    assert(rs06_is_feedback(feedback_id >> 24));
    assert(rs06_is_feedback(2) && !rs06_is_feedback(0x15));
    assert(rs06_feedback_faults(feedback_id) == 3);
    assert(rs06_feedback_mode(feedback_id) == 2);
    assert(!serial_watchdog_expired(1999999, 0));
    assert(serial_watchdog_expired(2000000, 0));
    // Source sample timestamps are intentionally not inputs to this check.
    // A sample clock ahead of HP must not underflow and reject a fresh result.
    assert(processing_result_fresh(true, 1000000, 999000, 50000));
    assert(processing_result_fresh(true, 1000000, 1000000, 50000));
    assert(!processing_result_fresh(true, 1000000, 900000, 50000));
    assert(!processing_result_fresh(false, 1000000, 999000, 50000));
    assert(!processing_result_fresh(true, 1000000, 0, 50000));
    processing_initialize();
    integration_reference();
    fourier_reference();
    notch_reference();
    emg_detection_reference();
    graded_curve_and_cascaded_notches();
    hardware_and_wire_vectors();
    puts("Notch attenuation/bandwidth, post-notch DSP, integration, FFT, discontinuity, ADC and "
         "wire references passed");
    return 0;
}
