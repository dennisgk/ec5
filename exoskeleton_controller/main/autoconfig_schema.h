#pragma once
#include "control.h"
#include "emg.h"
#include <stdbool.h>
#include <stddef.h>
typedef struct {
    device_configuration device;
    bool acquisition, integration, fourier, bluetooth;
    float manual_velocity;
    angle_config angle;
} saved_config;

// Reject deep or excessively complex documents before recursive JSON parsing.
static inline bool autoconfig_json_bounded(const char *text, size_t size)
{
    if (!size || size > 32768) {
        return false;
    }
    bool quoted = false, escaped = false;
    unsigned depth = 0, tokens = 0;
    for (size_t i = 0; i < size; ++i) {
        char c = text[i];
        if (!c) {
            return false;
        }
        if (quoted) {
            if (escaped) {
                escaped = false;
            } else if (c == '\\') {
                escaped = true;
            } else if (c == '"') {
                quoted = false;
            }
        } else if (c == '"') {
            quoted = true;
        } else if (c == '{' || c == '[') {
            if (++depth > 16) {
                return false;
            }
        } else if (c == '}' || c == ']') {
            if (!depth) {
                return false;
            }
            --depth;
        } else if (c == ':' || c == ',') {
            if (++tokens > 1024) {
                return false;
            }
        }
    }
    return !quoted && depth == 0;
}
bool autoconfig_parse(const char *text, size_t size, saved_config *out);
