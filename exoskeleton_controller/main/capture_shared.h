#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define CAPTURE_MAGIC UINT32_C(0x34474D45)
#define CAPTURE_LAYOUT 3
#define CAPTURE_CAPACITY 4096
#define CAPTURE_RAW_MASK UINT32_C(0x00FFFFFF)
#define CAPTURE_BOARD_SHIFT 24
#define CAPTURE_FIRST UINT32_C(0x02000000)
#define CAPTURE_GAP UINT32_C(0x04000000)

enum capture_state { CAPTURE_STARTING, CAPTURE_PAUSED, CAPTURE_RUNNING, CAPTURE_FAULT };
enum capture_request { CAPTURE_PAUSE = 1, CAPTURE_CONFIGURE, CAPTURE_RESUME };

// Four 32-bit words have the same layout on both cores. Publish the index only
// after both timestamp words and the sample words have been written.
typedef struct {
    uint32_t tick_low, tick_high, sequence, raw_board_flags;
} capture_record;

typedef struct {
    uint32_t enabled, ina_code, adc_code, hp_wiper, lp_wiper;
} capture_board_request;

typedef struct {
    uint32_t magic, layout, capacity, generation;
    uint32_t write_index, read_index;
    uint32_t state, heartbeat, high_water;
    uint32_t period_q19, poll_ticks;
    uint32_t request_sequence, acknowledgement, request, board_mask;
    capture_board_request boards[2];
    uint32_t configured_mask;
    uint32_t conversions[2], adc_gap_events[2], read_errors[2];
    uint32_t published[2], discarded[2], max_read_ticks[2];
    uint32_t late_passes, max_pass_ticks;
    uint32_t resend_ms[2], resend_errors[2];
    // First configuration failure: operation, device, register, expected, actual, ESP error.
    // LP writes while paused; HP reads only after mailbox acknowledgement.
    uint32_t configuration_error[2][6];
    capture_record records[CAPTURE_CAPACITY];
} capture_memory;

_Static_assert(sizeof(capture_record) == 16, "Shared records must be 16 bytes");
_Static_assert(offsetof(capture_memory, records) % 4 == 0, "Shared records must be aligned");
_Static_assert(sizeof(((capture_memory *)0)->records) == 65536, "Capture ring must be 64 KiB");

#ifdef _MSC_VER
// Host stress tests use the same ownership algorithm with Windows atomics.
#include <intrin.h>
static inline uint32_t capture_load(const uint32_t *word)
{
    return (uint32_t)_InterlockedCompareExchange((volatile long *)word, 0, 0);
}
static inline void capture_store(uint32_t *word, uint32_t value)
{
    _InterlockedExchange((volatile long *)word, (long)value);
}
#else
_Static_assert(__atomic_always_lock_free(4, 0), "Cross-core indices require lock-free stores");
static inline uint32_t capture_load(const uint32_t *word)
{
    return __atomic_load_n(word, __ATOMIC_ACQUIRE);
}
static inline void capture_store(uint32_t *word, uint32_t value)
{
    __atomic_store_n(word, value, __ATOMIC_RELEASE);
}
#endif

// Mailbox payload writes precede release publication. Only one request may be
// outstanding; the HP writer waits for LP acknowledgement before changing it.
static inline uint32_t capture_publish_request(capture_memory *memory, uint32_t command)
{
    uint32_t sequence = capture_load(&memory->request_sequence) + 1;
    memory->request = command;
    capture_store(&memory->request_sequence, sequence);
    return sequence;
}

static inline bool capture_pending_request(capture_memory *memory, uint32_t *sequence)
{
    *sequence = capture_load(&memory->request_sequence);
    return *sequence != capture_load(&memory->acknowledgement);
}

static inline bool capture_push(capture_memory *memory, const capture_record *record)
{
    uint32_t head = capture_load(&memory->write_index);
    uint32_t tail = capture_load(&memory->read_index);
    if (head - tail == CAPTURE_CAPACITY) {
        return false;
    }
    memory->records[head & (CAPTURE_CAPACITY - 1)] = *record;
    capture_store(&memory->write_index, head + 1);
    uint32_t used = head - tail + 1;
    if (used > capture_load(&memory->high_water)) {
        capture_store(&memory->high_water, used);
    }
    return true;
}

static inline bool capture_pop(capture_memory *memory, capture_record *record)
{
    uint32_t tail = capture_load(&memory->read_index);
    uint32_t head = capture_load(&memory->write_index);
    if (tail == head) {
        return false;
    }
    *record = memory->records[tail & (CAPTURE_CAPACITY - 1)];
    capture_store(&memory->read_index, tail + 1);
    return true;
}

// ADS122C04 with DCNT enabled: counter, ADC MSB, ADC middle, ADC LSB.
// The counter is deliberately excluded from the signed 24-bit signal value.
static inline capture_record capture_adc_record(const uint8_t data[4], unsigned board,
                                                uint64_t ticks, uint32_t sequence, uint32_t flags)
{
    capture_record record = {
        .tick_low = (uint32_t)ticks,
        .tick_high = (uint32_t)(ticks >> 32),
        .sequence = sequence,
        .raw_board_flags = (uint32_t)data[1] << 16 | (uint32_t)data[2] << 8 | data[3] |
                           (board & 1u) << CAPTURE_BOARD_SHIFT | flags,
    };
    return record;
}

static inline int32_t capture_raw(const capture_record *record)
{
    uint32_t value = record->raw_board_flags & CAPTURE_RAW_MASK;
    return (int32_t)(value & 0x7FFFFF) - ((value & 0x800000) ? 8388608 : 0);
}

static inline uint64_t capture_ticks(const capture_record *record)
{
    return ((uint64_t)record->tick_high << 32) | record->tick_low;
}

static inline uint64_t capture_ticks_to_us(uint64_t ticks, uint32_t period_q19)
{
    // Split the product so long uptimes do not overflow a 64-bit intermediate.
    return (ticks >> 19) * period_q19 + ((ticks & 0x7FFFF) * period_q19 >> 19);
}
