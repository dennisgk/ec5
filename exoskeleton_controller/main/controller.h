#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint64_t time_us;
    uint32_t valid;
    float position_rad, velocity_rad_s;
} motor_feedback;

typedef struct {
    bool running, fault, can_running, ready;
    uint32_t fault_code, inhibit_code;
    float selected_speed, commanded_speed;
    char reason[96], inhibit_reason[96];
} controller_status_t;

typedef struct {
    uint32_t flags, driver_state, tx_pending, rx_pending, tx_errors, rx_errors;
    uint32_t tx_failed, rx_missed, rx_overrun, arbitration_lost, bus_errors, alerts;
    uint32_t tx_queued, tx_confirmed, tx_rejected, rx_frames, rx_motor_frames;
    uint32_t last_error, last_stage, last_id, last_parameter;
    uint32_t failure_state, failure_tx_errors, failure_rx_errors, failure_bus_errors;
    uint32_t motor_faults, motor_warnings, last_rx_id;
    uint32_t feedback_faults, feedback_faults_seen, feedback_mode, fault_frames;
    uint32_t motor_faults_seen, motor_warnings_seen, temperature_tenths, feedback_frames;
    uint64_t failure_time_us;
} motor_can_diagnostics;

void controller_init(void);
controller_status_t controller_status(void);
void controller_request_output(bool enabled, float velocity);
void controller_inhibit(bool inhibited);
motor_feedback controller_feedback(void);
motor_can_diagnostics controller_can_diagnostics(void);
bool controller_retry_configuration(void);
