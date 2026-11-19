#include "angle_math.h"
#include <assert.h>
#include <stdio.h>

int main(void)
{
    angle_state a = {.config = {2, -1, 0, 120, 2, 10, 30, 1},
                     .enabled = true,
                     .calibrated = true,
                     .zero = 1,
                     .target = 60};
    assert(angle_math_valid(&a.config));
    float v = 0;
    assert(angle_math_step(&a, 1, true, &v));
    assert(fabsf(a.measured) < 1e-5f);
    assert(fabsf(a.velocity - 0.3f) < 1e-5f && v < 0);
    for (int i = 0; i < 100; ++i) {
        angle_math_step(&a, 1, true, &v);
    }
    assert(fabsf(a.velocity - 10) < 1e-5f);
    // A motor displacement of -pi/2 is 45 elbow degrees at a 2:1 ratio.
    angle_math_step(&a, 1 - 1.5707963268f, true, &v);
    assert(fabsf(a.measured - 45) < 0.0001f);
    angle_math_step(&a, 1 - 4.5f, true, &v);
    assert(v == 0); // Beyond upper limit: outward motion must stop.
    a.target = 120;
    angle_math_step(&a, 1, false, &v);
    assert(v == 0);
    a.config.direction = 0;
    assert(!angle_math_valid(&a.config));
    a.config.direction = 1;
    a.config.ratio = NAN;
    assert(!angle_math_valid(&a.config));
    a.config.ratio = 1;
    a.enabled = false;
    assert(!angle_math_step(&a, 1, true, &v));
    float stopped_measurement = a.measured;
    angle_math_step(&a, 0.5f, true, &v);
    assert(isfinite(a.measured) && fabsf(a.measured - stopped_measurement) > 20);
    a = (angle_state){.config = {10, 1, -180, 180, 2, 10, 30, 1}, .enabled = true, .zero = 12.5f};
    angle_math_step(&a, 12.5f, true, &v);
    angle_math_step(&a, -12.5f, true, &v);
    assert(a.measured > 0 && a.measured < 1);
    a = (angle_state){.config = {1, 1, 0, 120, 40, 720, 7200, 0.5f}, .enabled = true, .target = 90};
    assert(angle_math_valid(&a.config));
    float position = 0, maximum = 0;
    for (int i = 0; i < 60; ++i) {
        angle_math_step(&a, position / 57.295779513f, true, &v);
        position += v * 57.295779513f * 0.01f;
        maximum = fmaxf(maximum, position);
    }
    assert(fabsf(position - 90) <= 1.0f);
    assert(maximum < 93); // Fast default settles with braking in this ideal-plant check.
    puts("Angle ratio, direction, acceleration, limits, stale feedback and validation passed");
    return 0;
}
