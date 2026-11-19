#include "angle_math.h"
#include <assert.h>
#include <stdio.h>

static void check_lagged_arm(float lag, int delay, float direction, float ratio)
{
    angle_state a = {
        .config = {ratio, direction, 0, 120, 40, 720, 7200, 0.5f}, .enabled = true, .target = 90};
    float positions[32] = {0}, speeds[32] = {0};
    float position = 0, speed = 0, maximum = 0, command = 0;
    for (int i = 0; i < 500; ++i) {
        int slot = i % 32;
        positions[slot] = position;
        speeds[slot] = speed;
        int sample = (i + 32 - delay) % 32;
        float conversion = ratio * direction / 57.295779513f;
        angle_math_step(&a, positions[sample] * conversion, speeds[sample] * conversion, true,
                        &command);
        float requested_speed = command / conversion;
        speed += (requested_speed - speed) * (0.01f / (lag + 0.01f));
        position += speed * 0.01f;
        maximum = fmaxf(maximum, position);
        assert(isfinite(position) && fabsf(command) <= 50);
        if (i == 99) {
            assert(fabsf(position - 90) < 8); // Most of a 90-degree move within one second.
        }
    }
    fprintf(stderr, "Lag %.2f s, delay %d ticks: peak %.2f, settled %.2f degrees\n", (double)lag,
            delay, (double)maximum, (double)position);
    assert(maximum < 93);
    assert(fabsf(position - 90) < 1 && fabsf(speed) < 0.1f);
}

int main(void)
{
    angle_state a = {.config = {2, -1, 0, 120, 2, 10, 30, 1},
                     .enabled = true,
                     .calibrated = true,
                     .zero = 1,
                     .target = 60};
    assert(angle_math_valid(&a.config));
    float v = 0;
    assert(angle_math_step(&a, 1, 0, true, &v));
    assert(fabsf(a.measured) < 1e-5f);
    assert(fabsf(a.velocity - 0.3f) < 1e-5f && v < 0);
    for (int i = 0; i < 100; ++i) {
        angle_math_step(&a, 1, 0, true, &v);
    }
    assert(fabsf(a.velocity - 10) < 1e-5f);
    // A motor displacement of -pi/2 is 45 elbow degrees at a 2:1 ratio.
    angle_math_step(&a, 1 - 1.5707963268f, 0, true, &v);
    assert(fabsf(a.measured - 45) < 0.0001f);
    angle_math_step(&a, 1 - 4.5f, 0, true, &v);
    assert(v == 0); // Beyond upper limit: outward motion must stop.
    a.target = 120;
    angle_math_step(&a, 1, 0, false, &v);
    assert(v == 0);
    a.config.direction = 0;
    assert(!angle_math_valid(&a.config));
    a.config.direction = 1;
    a.config.ratio = NAN;
    assert(!angle_math_valid(&a.config));
    a.config.ratio = 1;
    a.enabled = false;
    assert(!angle_math_step(&a, 1, 0, true, &v));
    float stopped_measurement = a.measured;
    angle_math_step(&a, 0.5f, 0, true, &v);
    assert(isfinite(a.measured) && fabsf(a.measured - stopped_measurement) > 20);
    a = (angle_state){.config = {10, 1, -180, 180, 2, 10, 30, 1}, .enabled = true, .zero = 12.5f};
    angle_math_step(&a, 12.5f, 0, true, &v);
    angle_math_step(&a, -12.5f, 0, true, &v);
    assert(a.measured > 0 && a.measured < 1);
    a = (angle_state){.config = {1, 1, 0, 120, 40, 720, 7200, 0.5f}, .enabled = true, .target = 90};
    assert(angle_math_valid(&a.config));
    float position = 0, maximum = 0;
    for (int i = 0; i < 250; ++i) {
        angle_math_step(&a, position / 57.295779513f, v, true, &v);
        position += v * 57.295779513f * 0.01f;
        maximum = fmaxf(maximum, position);
    }
    assert(fabsf(position - 90) <= 1.0f);
    assert(maximum < 93); // Fast default settles with braking in this ideal-plant check.
    check_lagged_arm(0.03f, 0, 1, 1);
    check_lagged_arm(0.1f, 2, 1, 2);
    check_lagged_arm(0.2f, 5, -1, 2);
    a.velocity = 500;
    a.target = a.measured;
    angle_math_step(&a, position / 57.295779513f, 1, true, &v);
    assert(v == 0); // Braking must not wait for the acceleration slew limiter.
    angle_math_step(&a, 0, NAN, true, &v);
    assert(v == 0);
    puts("Angle ratio, direction, acceleration, limits, stale feedback and validation passed");
    return 0;
}
