#include "control.h"
#include "angle_math.h"
#include "ble_control.h"
#include "controller.h"
#include "emg.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "protocol.h"
#include <math.h>

static portMUX_TYPE guard = portMUX_INITIALIZER_UNLOCKED;
static bool bluetooth_owner;
static SemaphoreHandle_t command_mutex;
void control_init(void)
{
    command_mutex = xSemaphoreCreateMutex();
    configASSERT(command_mutex);
}
void control_command_lock(void)
{
    xSemaphoreTake(command_mutex, portMAX_DELAY);
}
void control_command_unlock(void)
{
    xSemaphoreGive(command_mutex);
}
static angle_state angle = {
    .config = {1, 1, 0, 120, 40, 720, 7200, 0.5f},
};

bool control_bluetooth(void)
{
    portENTER_CRITICAL(&guard);
    bool result = bluetooth_owner;
    portEXIT_CRITICAL(&guard);
    return result;
}

void control_stop(void)
{
    portENTER_CRITICAL(&guard);
    angle.enabled = false;
    angle.velocity = 0;
    portEXIT_CRITICAL(&guard);
    emg_control_disable();
    controller_inhibit(true);
}

uint8_t control_handoff(bool bluetooth)
{
    // Failure to start BLE must never strand USB control.
    if (bluetooth && !ble_control_available()) {
        return STATUS_NOT_READY;
    }
    control_stop();
    portENTER_CRITICAL(&guard);
    if (bluetooth_owner != bluetooth) {
        angle.calibrated = false;
    }
    bluetooth_owner = bluetooth;
    portEXIT_CRITICAL(&guard);
    ble_control_advertise(bluetooth);
    return STATUS_OK;
}

angle_state angle_snapshot(void)
{
    portENTER_CRITICAL(&guard);
    angle_state result = angle;
    portEXIT_CRITICAL(&guard);
    return result;
}

bool angle_validate(const angle_config *c)
{
    return angle_math_valid(c);
}

uint8_t angle_configure(const angle_config *c)
{
    if (!angle_validate(c)) {
        return STATUS_INVALID;
    }
    portENTER_CRITICAL(&guard);
    if (angle.enabled) {
        portEXIT_CRITICAL(&guard);
        return STATUS_NOT_READY;
    }
    if (angle.config.ratio != c->ratio || angle.config.direction != c->direction) {
        angle.calibrated = false;
    }
    angle.config = *c;
    portEXIT_CRITICAL(&guard);
    return STATUS_OK;
}

uint8_t angle_calibrate(void)
{
    // Some RobStride firmware emits feedback only after zero-speed Enable.
    controller_status_t motor = controller_status();
    if (!motor.ready || motor.fault) {
        return STATUS_NOT_READY;
    }
    control_stop();
    controller_inhibit(false);
    controller_request_output(true, 0);
    motor_feedback feedback = controller_feedback();
    for (unsigned i = 0;
         i < 20 && (!feedback.valid || (uint64_t)esp_timer_get_time() - feedback.time_us >= 100000);
         ++i) {
        vTaskDelay(pdMS_TO_TICKS(10));
        feedback = controller_feedback();
    }
    control_stop();
    if (!feedback.valid || (uint64_t)esp_timer_get_time() - feedback.time_us >= 100000) {
        return STATUS_NOT_READY;
    }
    control_stop();
    portENTER_CRITICAL(&guard);
    angle.zero = feedback.position_rad;
    angle.previous_encoder = angle.unwrapped_encoder = feedback.position_rad;
    angle.encoder_initialized = true;
    angle.target = angle.measured = 0;
    angle.calibrated = true;
    portEXIT_CRITICAL(&guard);
    return STATUS_OK;
}

void angle_reset(void)
{
    control_stop();
    portENTER_CRITICAL(&guard);
    angle = (angle_state){.config = {1, 1, 0, 120, 40, 720, 7200, 0.5f}};
    portEXIT_CRITICAL(&guard);
}

uint8_t angle_enable(bool enabled)
{
    if (!enabled) {
        control_stop();
        return STATUS_OK;
    }
    motor_feedback feedback = controller_feedback();
    controller_status_t motor = controller_status();
    angle_state state = angle_snapshot();
    if (!state.calibrated || !motor.ready || motor.fault) {
        return STATUS_NOT_READY;
    }
    if (!feedback.valid || (uint64_t)esp_timer_get_time() - feedback.time_us >= 100000) {
        controller_inhibit(false);
        controller_request_output(true, 0);
        for (unsigned i = 0; i < 20; ++i) {
            vTaskDelay(pdMS_TO_TICKS(10));
            feedback = controller_feedback();
            if (feedback.valid && (uint64_t)esp_timer_get_time() - feedback.time_us < 100000) {
                break;
            }
        }
        control_stop();
        motor = controller_status();
        if (!feedback.valid || (uint64_t)esp_timer_get_time() - feedback.time_us >= 100000 ||
            motor.fault) {
            return STATUS_NOT_READY;
        }
    }
    state = angle_snapshot();
    float measured =
        ((state.encoder_initialized ? state.unwrapped_encoder : feedback.position_rad) -
         state.zero) *
        57.295779513f * state.config.direction / state.config.ratio;
    if (measured < state.config.minimum || measured > state.config.maximum) {
        return STATUS_INVALID;
    }
    emg_control_disable();
    portENTER_CRITICAL(&guard);
    angle.enabled = true;
    angle.target = angle.measured = measured;
    angle.velocity = 0;
    portEXIT_CRITICAL(&guard);
    controller_inhibit(false);
    return STATUS_OK;
}

uint8_t angle_target(float target)
{
    if (!isfinite(target)) {
        return STATUS_INVALID;
    }
    portENTER_CRITICAL(&guard);
    if (!angle.enabled) {
        portEXIT_CRITICAL(&guard);
        return STATUS_NOT_READY;
    }
    angle.target = fminf(angle.config.maximum, fmaxf(angle.config.minimum, target));
    portEXIT_CRITICAL(&guard);
    return STATUS_OK;
}

bool angle_output(float encoder, float measured_velocity, bool fresh, float *velocity)
{
    portENTER_CRITICAL(&guard);
    bool enabled = angle_math_step(&angle, encoder, measured_velocity, fresh, velocity);
    portEXIT_CRITICAL(&guard);
    return enabled;
}

uint8_t control_ble_command(uint8_t operation, const uint8_t *p, size_t n)
{
    if (operation == 7 && n == 0) {
        control_stop();
        return STATUS_OK;
    }
    if (!control_bluetooth()) {
        return STATUS_CONTROL_OWNER;
    }
    angle_config c = angle_snapshot().config;
    switch (operation) {
    case 1:
        return n ? STATUS_INVALID : angle_calibrate();
    case 2:
        if (n != 8) {
            return STATUS_INVALID;
        }
        c.minimum = read_float(p);
        c.maximum = read_float(p + 4);
        return angle_configure(&c);
    case 3:
        if (n != 5) {
            return STATUS_INVALID;
        }
        c.ratio = read_float(p);
        c.direction = (int8_t)p[4];
        return angle_configure(&c);
    case 4:
        if (n != 16) {
            return STATUS_INVALID;
        }
        c.gain = read_float(p);
        c.speed = read_float(p + 4);
        c.acceleration = read_float(p + 8);
        c.deadband = read_float(p + 12);
        return angle_configure(&c);
    case 5:
        return n == 4 ? angle_target(read_float(p)) : STATUS_INVALID;
    case 6:
        return n == 1 && p[0] <= 1 ? angle_enable(p[0] != 0) : STATUS_INVALID;
    default:
        return STATUS_INVALID;
    }
}
