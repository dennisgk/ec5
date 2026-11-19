#pragma once
#include <stdbool.h>
#include <stdint.h>

// RS06 private CAN protocol: detailed fault frame 0x15, feedback 0x02/0x18.
static inline uint32_t rs06_u32_le(const uint8_t *bytes)
{
    return (uint32_t)bytes[0] | (uint32_t)bytes[1] << 8 | (uint32_t)bytes[2] << 16 |
           (uint32_t)bytes[3] << 24;
}

static inline bool rs06_is_feedback(uint32_t type)
{
    return type == 0x02 || type == 0x18;
}

static inline uint32_t rs06_feedback_faults(uint32_t identifier)
{
    return (identifier >> 16) & 0x3F;
}

static inline uint32_t rs06_feedback_mode(uint32_t identifier)
{
    return (identifier >> 22) & 3;
}
