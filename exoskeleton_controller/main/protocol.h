#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define PROTOCOL_VERSION 1
#define MAX_IGNORE_RANGES 16
#define MAX_NOTCH_FILTERS 8
#define SAMPLE_RATE 1000
#define FFT_SIZE 256
#define FFT_BINS 129
#define RAW_CAPACITY 1024

enum command {
    CMD_INIT = 1,
    CMD_MOTOR = 2,
    CMD_ACTIVATE_EMGS = 3,
    CMD_ACTIVATE_EMGS_INTEGRATION = 4,
    CMD_ACTIVATE_EMGS_FOURIER = 5,
    CMD_ADJUST_EMG = 6,
    CMD_ADJUST_EMGS_INTEGRATION = 7,
    CMD_ADJUST_EMGS_FOURIER = 8,
    CMD_QUERY_EMG = 9,
    CMD_QUERY_FOURIER = 10,
    CMD_QUERY_INTEGRATION = 11,
    CMD_QUERY_ACQUISITION = 12,
    CMD_QUERY_MOTOR = 13,
    CMD_RESEND_EMG_PARAMETERS = 14, // board u8, interval_ms u16; zero disables
    CMD_ADJUST_NOTCH = 15,          // board u8, enabled u8, frequency u16, bandwidth u16 (0.1 Hz)
    CMD_QUERY_FILTERED_EMG = 16,
    CMD_ADJUST_NOTCH_ATTENUATION = 17, // command 15 payload plus signed center attenuation i16
    CMD_QUERY_MOTOR_CAN = 18,
    CMD_RETRY_MOTOR_CAN = 19, // explicit reconfiguration, keeps motor disabled
    CMD_CONFIGURE_EMG_CONTROL = 20,
    CMD_QUERY_EMG_DETECTION = 21,
    CMD_SET_IGNORE_RANGES = 22,
    CMD_SET_NOTCH_BANK = 23,
    CMD_SET_MOTOR_CURVE = 24,
    CMD_GET_CONFIGURATION = 25,
    CMD_SUBSCRIBE = 26,
    CMD_QUERY_EMG_HARDWARE = 27,
    CMD_AUTOCONFIG_BEGIN = 28,
    CMD_AUTOCONFIG_WRITE = 29,
    CMD_AUTOCONFIG_COMMIT = 30,
    CMD_AUTOCONFIG_DELETE = 31,
    CMD_BLUETOOTH = 32,
    CMD_CONTROL_STATE = 33,
    CMD_ANGLE_CONFIG = 34,
};

enum status_code {
    STATUS_OK = 0,
    STATUS_INVALID = 1,
    STATUS_NOT_INITIALIZED = 2,
    STATUS_HARDWARE_0 = 3,
    STATUS_HARDWARE_1 = 4,
    STATUS_HARDWARE_BOTH = 5,
    STATUS_NOT_READY = 6,
    STATUS_MOTOR_FAULT = 7,
    STATUS_VERSION = 8,
    STATUS_INTERNAL = 9,
    STATUS_LP_FAULT = 10,
    STATUS_EMG_CONTROL = 11,
    STATUS_CANCELLED = 12,
    STATUS_CONTROL_OWNER = 13,
    STATUS_STORAGE = 14,
};

enum emg_property {
    EMG_PROPERTY_AD8231_GAIN = 0,
    EMG_PROPERTY_ADC_PGA_GAIN = 1,
    EMG_PROPERTY_HIGH_PASS = 2,
    EMG_PROPERTY_LOW_PASS = 3,
};

// serial values are explicitly encoded. serial uses asserted native record layouts on
// the little-endian C6; neither transport includes compiler padding on the wire.
static inline uint16_t read_u16(const uint8_t *p)
{
    return (uint16_t)(p[0] | (uint16_t)p[1] << 8);
}
static inline float read_float(const uint8_t *p)
{
    float value;
    memcpy(&value, p, sizeof(value));
    return value;
}
static inline void write_u16(uint8_t **p, uint16_t value)
{
    *(*p)++ = (uint8_t)value;
    *(*p)++ = (uint8_t)(value >> 8);
}
static inline void write_u32(uint8_t **p, uint32_t value)
{
    for (unsigned n = 0; n < 4; ++n) {
        *(*p)++ = (uint8_t)(value >> (8 * n));
    }
}
static inline void write_u64(uint8_t **p, uint64_t value)
{
    for (unsigned n = 0; n < 8; ++n) {
        *(*p)++ = (uint8_t)(value >> (8 * n));
    }
}
static inline void write_float(uint8_t **p, float value)
{
    uint32_t bits;
    _Static_assert(sizeof(float) == 4, "Protocol requires float32");
    memcpy(&bits, &value, sizeof(bits));
    write_u32(p, bits);
}
