#pragma once
#include "control.h"
#include <math.h>

static inline bool angle_math_valid(const angle_config *c)
{
    return isfinite(c->ratio) && c->ratio >= 0.01f && c->ratio <= 1000 &&
           (c->direction == 1 || c->direction == -1) && isfinite(c->minimum) &&
           isfinite(c->maximum) && c->minimum >= -180 && c->maximum <= 180 &&
           c->minimum < c->maximum && isfinite(c->gain) && c->gain > 0 && c->gain <= 50 &&
           isfinite(c->speed) && c->speed > 0 && c->speed <= 720 && isfinite(c->acceleration) &&
           c->acceleration > 0 && c->acceleration <= 10000 && isfinite(c->deadband) &&
           c->deadband >= 0 && c->deadband <= 10;
}

static inline bool angle_math_step(angle_state *angle, float encoder, float measured_velocity,
                                   bool fresh, float *velocity)
{
    if (!fresh || !isfinite(encoder) || !isfinite(measured_velocity)) {
        angle->velocity = *velocity = 0;
        return angle->enabled;
    }
    if (!angle->encoder_initialized) {
        angle->previous_encoder = angle->unwrapped_encoder = encoder;
        angle->encoder_initialized = true;
    } else {
        float delta = encoder - angle->previous_encoder;
        // RobStride feedback maps a 16-bit position to [-4pi, +4pi].
        if (delta > 12.5663706144f) {
            delta -= 25.1327412287f;
        }
        if (delta < -12.5663706144f) {
            delta += 25.1327412287f;
        }
        angle->unwrapped_encoder += delta;
        angle->previous_encoder = encoder;
    }
    angle_config c = angle->config;
    angle->measured =
        (angle->unwrapped_encoder - angle->zero) * 57.295779513f * c.direction / c.ratio;
    if (!angle->enabled) {
        angle->velocity = *velocity = 0;
        return false;
    }
    float error = angle->target - angle->measured;
    float elbow_velocity = measured_velocity * 57.295779513f * c.direction / c.ratio;
    // The inner speed loop and physical arm do not follow commands instantaneously.
    // Preview 150 ms of measured motion to brake before crossing the target. Limit
    // the outer-loop gain accordingly, including for existing aggressive profiles.
    const float braking_preview = 0.15f;
    float gain = c.gain / (1 + c.gain * braking_preview);
    float wanted = gain * (error - elbow_velocity * braking_preview);
    float distance = fmaxf(0, fabsf(error) - c.deadband);
    float step = c.acceleration * 0.01f;
    // Reserve one control period for braking instead of assuming continuous updates.
    float braking_speed = fmaxf(0, sqrtf(2 * c.acceleration * distance + step * step) - step);
    float limit = fminf(c.speed, braking_speed);
    wanted = fminf(limit, fmaxf(-limit, wanted));
    // Slew-limit acceleration only. Never prolong a fast command when braking;
    // pass through zero before accelerating in the opposite direction.
    if (wanted * angle->velocity < 0) {
        angle->velocity = 0;
    } else if (fabsf(wanted) <= fabsf(angle->velocity)) {
        angle->velocity = wanted;
    } else {
        angle->velocity += fminf(step, fmaxf(-step, wanted - angle->velocity));
    }
    if (fabsf(error) <= c.deadband || !isfinite(angle->measured) ||
        (angle->measured <= c.minimum && angle->velocity < 0) ||
        (angle->measured >= c.maximum && angle->velocity > 0)) {
        angle->velocity = 0;
    }
    *velocity = angle->velocity * c.ratio * c.direction / 57.295779513f;
    *velocity = fminf(50, fmaxf(-50, *velocity));
    return true;
}
