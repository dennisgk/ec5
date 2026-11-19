#include "autoconfig_schema.h"
#include "angle_math.h"
#include "cJSON.h"
#include "emg_signal.h"
#include <math.h>
#include <string.h>

static bool number(const cJSON *object, const char *key, double low, double high, double *value)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(object, key);
    if (!cJSON_IsNumber(item) || !isfinite(item->valuedouble) || item->valuedouble < low ||
        item->valuedouble > high) {
        return false;
    }
    *value = item->valuedouble;
    return true;
}
static bool integer(const cJSON *object, const char *key, int low, int high, int *value)
{
    double n;
    if (!number(object, key, low, high, &n) || n != trunc(n)) {
        return false;
    }
    *value = (int)n;
    return true;
}
static bool boolean(const cJSON *object, const char *key, bool *value)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(object, key);
    if (!cJSON_IsBool(item)) {
        return false;
    }
    *value = cJSON_IsTrue(item);
    return true;
}
bool autoconfig_parse(const char *text, size_t size, saved_config *out)
{
    if (!autoconfig_json_bounded(text, size)) {
        return false;
    }
    memset(out, 0, sizeof(*out));
    out->device.owner = -1;
    out->angle = (angle_config){1, 1, 0, 120, 40, 720, 7200, 0.5f};
    const char *end = NULL;
    cJSON *root = cJSON_ParseWithLengthOpts(text, size + 1, &end, true);
    if (!root || end != text + size) {
        cJSON_Delete(root);
        return false;
    }
    bool valid = false;
    int value;
    double n;
    const cJSON *project = cJSON_GetObjectItemCaseSensitive(root, "project");
    if (!cJSON_IsString(project) || strcmp(project->valuestring, "ec5") ||
        !integer(root, "schema_version", 1, 2, &value)) {
        goto done;
    }
    int version = value;
    const cJSON *device = cJSON_GetObjectItemCaseSensitive(root, "device");
    const cJSON *enabled = cJSON_GetObjectItemCaseSensitive(device, "enabled");
    if (cJSON_GetArraySize(enabled) != 3) {
        goto done;
    }
    bool *flags[] = {&out->acquisition, &out->integration, &out->fourier};
    for (unsigned i = 0; i < 3; ++i) {
        const cJSON *item = cJSON_GetArrayItem(enabled, i);
        if (!cJSON_IsBool(item)) {
            goto done;
        }
        *flags[i] = cJSON_IsTrue(item);
    }
    if (!out->acquisition && (out->integration || out->fourier)) {
        goto done;
    }
    const cJSON *owner = cJSON_GetObjectItemCaseSensitive(device, "owner");
    if (!cJSON_IsNull(owner)) {
        if (!integer(device, "owner", 0, 1, &value) || !out->fourier) {
            goto done;
        }
        out->device.owner = value;
    }
    const cJSON *ignore = cJSON_GetObjectItemCaseSensitive(device, "ignore");
    const cJSON *segments = cJSON_GetObjectItemCaseSensitive(ignore, "segments");
    if (!boolean(ignore, "enabled", &out->device.ignore_enabled) || !cJSON_IsArray(segments) ||
        cJSON_GetArraySize(segments) > MAX_IGNORE_RANGES) {
        goto done;
    }
    out->device.ignore_count = (uint8_t)cJSON_GetArraySize(segments);
    for (unsigned i = 0; i < out->device.ignore_count; ++i) {
        const cJSON *row = cJSON_GetArrayItem(segments, i);
        ignore_range *r = &out->device.ranges[i];
        if (!boolean(row, "enabled", &r->enabled) || !number(row, "low_hz", 0, 500, &n)) {
            goto done;
        }
        r->low = (float)n;
        if (!number(row, "high_hz", r->low, 500, &n)) {
            goto done;
        }
        r->high = (float)n;
    }
    const cJSON *boards = cJSON_GetObjectItemCaseSensitive(device, "boards");
    if (!cJSON_IsArray(boards) || cJSON_GetArraySize(boards) != 2) {
        goto done;
    }
    for (unsigned b = 0; b < 2; ++b) {
        const cJSON *board = cJSON_GetArrayItem(boards, b);
        const cJSON *hardware = cJSON_GetObjectItemCaseSensitive(board, "hardware");
        board_config *h = &out->device.boards[b];
        processing_config *p = &out->device.processing[b];
        if (!integer(hardware, "ina_gain", 1, 128, &value) || emg_gain_code(value) < 0) {
            goto done;
        }
        h->ina_gain = (uint8_t)value;
        if (!integer(hardware, "adc_gain", 1, 128, &value) || emg_gain_code(value) < 0) {
            goto done;
        }
        h->adc_gain = (uint8_t)value;
        if (!integer(hardware, "high_pass", 33, 709, &value)) {
            goto done;
        }
        h->high_pass = (uint16_t)value;
        if (!integer(hardware, "low_pass", 1520, 4500, &value)) {
            goto done;
        }
        h->low_pass = (uint16_t)value;
        if (!integer(board, "resend_ms", 0, 65535, &value) || (value && value < 100)) {
            goto done;
        }
        out->device.resend_ms[b] = (uint16_t)value;
        if (!integer(board, "integration_ms", 10, 1000, &value)) {
            goto done;
        }
        p->integration_ms = (uint16_t)value;
        const cJSON *fourier = cJSON_GetObjectItemCaseSensitive(board, "fourier");
        if (!integer(fourier, "hop", 64, 256, &value) ||
            (value != 64 && value != 128 && value != 256)) {
            goto done;
        }
        p->hop = (uint16_t)value;
        if (!integer(fourier, "window", 0, 1, &value)) {
            goto done;
        }
        p->window = (uint8_t)value;
        bool remove_mean;
        if (!boolean(fourier, "remove_mean", &remove_mean)) {
            goto done;
        }
        p->remove_mean = remove_mean;
        const cJSON *notches = cJSON_GetObjectItemCaseSensitive(board, "notches");
        if (!cJSON_IsArray(notches) || cJSON_GetArraySize(notches) > MAX_NOTCH_FILTERS) {
            goto done;
        }
        p->notch_count = (uint8_t)cJSON_GetArraySize(notches);
        for (unsigned i = 0; i < p->notch_count; ++i) {
            notch_filter_config *f = &p->notches[i];
            const cJSON *row = cJSON_GetArrayItem(notches, i);
            if (!boolean(row, "enabled", &f->enabled) ||
                !integer(row, "frequency", 1, 4999, &value)) {
                goto done;
            }
            f->frequency = (uint16_t)value;
            if (!integer(row, "bandwidth", 1, 4999, &value) || value >= 2 * f->frequency ||
                value >= 2 * (5000 - f->frequency)) {
                goto done;
            }
            f->bandwidth = (uint16_t)value;
            if (!integer(row, "attenuation", -600, -1, &value)) {
                goto done;
            }
            f->attenuation = (int16_t)value;
        }
        emg_detection_config *curve = &out->device.curves[b];
        const cJSON *c = cJSON_GetObjectItemCaseSensitive(board, "curve");
        if (!number(c, "threshold", 1e-12, 1e9, &n)) {
            goto done;
        }
        curve->threshold = (float)n;
        if (!number(c, "maximum", curve->threshold, 1e9, &n) || n <= curve->threshold) {
            goto done;
        }
        curve->maximum = (float)n;
        if (!integer(c, "velocity", -500, 500, &value)) {
            goto done;
        }
        curve->velocity = (int16_t)value;
        if (!integer(c, "maximum_velocity", -500, 500, &value)) {
            goto done;
        }
        curve->maximum_velocity = (int16_t)value;
    }
    if (version == 2) {
        const cJSON *motor = cJSON_GetObjectItemCaseSensitive(root, "motor");
        if (!boolean(motor, "bluetooth", &out->bluetooth) ||
            !number(motor, "manual_velocity", -50, 50, &n)) {
            goto done;
        }
        out->manual_velocity = (float)n;
        const cJSON *a = cJSON_GetObjectItemCaseSensitive(motor, "angle");
        const char *keys[] = {"ratio", "direction", "minimum",      "maximum",
                              "gain",  "speed",     "acceleration", "deadband"};
        float *fields[] = {&out->angle.ratio,        &out->angle.direction, &out->angle.minimum,
                           &out->angle.maximum,      &out->angle.gain,      &out->angle.speed,
                           &out->angle.acceleration, &out->angle.deadband};
        for (unsigned i = 0; i < 8; ++i) {
            if (!number(a, keys[i], -10000, 10000, &n)) {
                goto done;
            }
            *fields[i] = (float)n;
        }
        if (!angle_math_valid(&out->angle)) {
            goto done;
        }
    }
    valid = true;
done:
    cJSON_Delete(root);
    return valid;
}
