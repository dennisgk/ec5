#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    float ratio, direction, minimum, maximum;
    float gain, speed, acceleration, deadband;
} angle_config;

typedef struct {
    angle_config config;
    bool calibrated, enabled;
    float zero, target, measured, velocity;
    float previous_encoder, unwrapped_encoder;
    bool encoder_initialized;
} angle_state;

bool control_bluetooth(void);
void control_init(void);
void control_command_lock(void);
void control_command_unlock(void);
uint8_t control_handoff(bool bluetooth);
void control_stop(void);
void angle_reset(void);
bool angle_validate(const angle_config *config);
angle_state angle_snapshot(void);
uint8_t angle_configure(const angle_config *config);
uint8_t angle_calibrate(void);
uint8_t angle_enable(bool enabled);
uint8_t angle_target(float target);
// Called only by the 100 Hz motor task; no allocation or transport work.
bool angle_output(float encoder, bool fresh, float *velocity);
uint8_t control_ble_command(uint8_t operation, const uint8_t *data, size_t size);
