#pragma once
#include "emg_detection.h"
#include "signal_processing.h"
#include <stddef.h>

typedef struct {
    uint8_t ina_gain, adc_gain;
    uint16_t high_pass, low_pass; // tenths of Hz
    float actual_hp, actual_lp;
    bool active, configured;
    uint32_t revision;
} board_config;

typedef struct {
    uint64_t time_us;
    int32_t raw;
    uint32_t sequence;
} emg_sample;

typedef struct {
    uint64_t time_us;
    float millivolts;
    uint32_t sequence;
} filtered_sample;

typedef struct {
    uint64_t time_us;
    uint32_t frame;
    float peak;
    uint32_t flags; // detected=1, selected=2, motor requested=4
    float velocity;
} emg_detection_record;

_Static_assert(sizeof(emg_sample) == 16 && offsetof(emg_sample, sequence) == 12,
               "Raw telemetry layout");
_Static_assert(sizeof(filtered_sample) == 16 && offsetof(filtered_sample, sequence) == 12,
               "Filtered telemetry layout");
_Static_assert(sizeof(emg_detection_record) == 24 && offsetof(emg_detection_record, velocity) == 20,
               "Detection telemetry layout");

typedef struct {
    bool selected, valid, detected;
    float velocity;
} emg_motor_output;

uint8_t emg_control_adjust(unsigned board, bool enabled, const emg_detection_config *config);
void emg_control_disable(void);
int emg_control_board(void);
emg_motor_output emg_control_output(void);

void emg_init(void);
typedef struct {
    uint32_t state, fault, generation, capacity, occupancy, high_water, configured_mask, heartbeat;
    uint32_t conversions[2], adc_gaps[2], read_errors[2], published[2], discarded[2];
    uint32_t max_read_us[2], raw_overwrites[2], processing_overruns[2];
    uint32_t late_passes, max_pass_us, max_drain_us;
} acquisition_diagnostics;

acquisition_diagnostics emg_diagnostics(void);
void emg_reset(void);
uint8_t emg_activate(bool enabled);
uint8_t emg_resend_parameters(unsigned board, uint16_t milliseconds);
uint8_t emg_notch_adjust(unsigned board, uint8_t enabled, uint16_t frequency, uint16_t bandwidth,
                         int16_t attenuation);
bool emg_notch_enabled(unsigned board);
typedef struct {
    bool enabled;
    float low, high;
} ignore_range;
typedef struct {
    board_config boards[2];
    processing_config processing[2];
    emg_detection_config curves[2];
    uint16_t resend_ms[2];
    int owner;
    bool ignore_enabled;
    uint8_t ignore_count, mask[17];
    ignore_range ranges[MAX_IGNORE_RANGES];
} device_configuration;

uint8_t emg_ignore_adjust(bool enabled, unsigned count, const ignore_range *ranges);
uint8_t emg_notch_bank(unsigned board, unsigned count, const notch_filter_config *filters);
void emg_configuration(device_configuration *configuration);
uint8_t emg_adjust(unsigned board, unsigned property, uint16_t value, float *actual);
uint8_t emg_processing_enable(bool fourier, bool enabled);
uint8_t emg_integration_adjust(unsigned board, uint16_t milliseconds);
uint8_t emg_fourier_adjust(unsigned board, uint16_t hop, uint8_t window, uint8_t remove_mean);
board_config emg_board(unsigned board);
integration_result emg_integration(unsigned board);
fourier_result emg_fourier(unsigned board);
