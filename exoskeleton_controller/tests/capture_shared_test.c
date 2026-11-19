#include "capture_shared.h"
#include "emg_signal.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <windows.h>

static capture_memory memory;
#define STRESS_RECORDS 200000

static capture_record record_for(uint32_t sequence)
{
    return (capture_record){.tick_low = sequence,
                            .tick_high = ~sequence,
                            .sequence = sequence,
                            .raw_board_flags = (sequence & CAPTURE_RAW_MASK) |
                                               ((sequence & 1) << CAPTURE_BOARD_SHIFT)};
}

static void validate(const capture_record *record, uint32_t sequence)
{
    assert(record->sequence == sequence);
    assert(record->tick_low == sequence);
    assert(record->tick_high == ~sequence);
    assert(record->raw_board_flags == record_for(sequence).raw_board_flags);
}

static void overflow_and_wrap(void)
{
    memset(&memory, 0, sizeof(memory));
    // Exercise unsigned index rollover, not just wrap around the physical array.
    memory.write_index = memory.read_index = UINT32_MAX - 128;
    for (uint32_t n = 0; n < CAPTURE_CAPACITY; ++n) {
        capture_record record = record_for(n);
        assert(capture_push(&memory, &record));
    }
    uint32_t head = memory.write_index;
    capture_record rejected = record_for(999999);
    assert(!capture_push(&memory, &rejected));
    assert(memory.write_index == head);
    assert(memory.high_water == CAPTURE_CAPACITY);
    for (uint32_t n = 0; n < CAPTURE_CAPACITY; ++n) {
        capture_record record;
        assert(capture_pop(&memory, &record));
        validate(&record, n);
    }
    assert(!capture_pop(&memory, &rejected));
    assert(capture_push(&memory, &rejected));
    capture_record record;
    assert(capture_pop(&memory, &record));
    validate(&record, 999999);
}

static DWORD WINAPI producer(void *unused)
{
    (void)unused;
    for (uint32_t n = 0; n < STRESS_RECORDS; ++n) {
        capture_record record = record_for(n);
        while (!capture_push(&memory, &record)) {
            Sleep(0); // Test retries to verify every publication under contention.
        }
    }
    return 0;
}

static DWORD WINAPI consumer(void *unused)
{
    (void)unused;
    Sleep(20); // Force a full ring and a paused consumer before starting.
    for (uint32_t n = 0; n < STRESS_RECORDS; ++n) {
        capture_record record;
        while (!capture_pop(&memory, &record)) {
            Sleep(0);
        }
        validate(&record, n);
        if ((n % 10000) == 0) {
            Sleep(1);
        }
    }
    return 0;
}

static void threaded_publication(void)
{
    memset(&memory, 0, sizeof(memory));
    HANDLE threads[] = {CreateThread(NULL, 0, producer, NULL, 0, NULL),
                        CreateThread(NULL, 0, consumer, NULL, 0, NULL)};
    assert(threads[0] && threads[1]);
    assert(WaitForMultipleObjects(2, threads, TRUE, 30000) == WAIT_OBJECT_0);
    CloseHandle(threads[0]);
    CloseHandle(threads[1]);
    assert(memory.write_index == STRESS_RECORDS);
    assert(memory.read_index == STRESS_RECORDS);
}

static void decoding_and_time(void)
{
    const int32_t codes[] = {-8388608, -4096000, -256, -1, 0, 1, 256, 4096000, 8388607};
    for (unsigned n = 0; n < sizeof(codes) / sizeof(codes[0]); ++n) {
        uint32_t raw = (uint32_t)codes[n] & CAPTURE_RAW_MASK;
        uint8_t bytes[4] = {0xA5, (uint8_t)(raw >> 16), (uint8_t)(raw >> 8), (uint8_t)raw};
        for (unsigned board = 0; board < 2; ++board) {
            capture_record packed =
                capture_adc_record(bytes, board, 123456789, 99, CAPTURE_FIRST | CAPTURE_GAP);
            assert(capture_raw(&packed) == codes[n]);
            assert(capture_raw(&packed) == emg_decode24(bytes + 1));
            assert(((packed.raw_board_flags >> CAPTURE_BOARD_SHIFT) & 1) == board);
            for (unsigned gain = 1; gain <= 128; gain *= 2) {
                double reference_mv = (codes[n] / 8388608.0) * (2.048 / gain) * 1000.0;
                double actual_mv = capture_raw(&packed) * emg_mv_per_code(gain);
                assert(fabs(reference_mv - actual_mv) < 0.0005);
            }
        }
    }
    capture_record record = {.tick_low = 7, .tick_high = 2, .raw_board_flags = 0x01FFFFFF};
    assert(capture_raw(&record) == -1);
    record.raw_board_flags = 0x00800000 | CAPTURE_GAP;
    assert(capture_raw(&record) == -8388608);
    record.raw_board_flags = 0x007FFFFF | CAPTURE_FIRST;
    assert(capture_raw(&record) == 8388607);
    assert(capture_ticks(&record) == (UINT64_C(2) << 32) + 7);
    const uint32_t period = 3500000;
    const uint64_t long_uptime = UINT64_C(1000000000000);
    assert(capture_ticks_to_us(long_uptime, period) == long_uptime * period / 524288);
    assert(capture_ticks_to_us(0, period) == 0);
}

static DWORD WINAPI mailbox_reader(void *unused)
{
    (void)unused;
    for (uint32_t n = 1; n <= 10000; ++n) {
        uint32_t sequence;
        while (!capture_pending_request(&memory, &sequence)) {
            Sleep(0);
        }
        assert(sequence == (uint32_t)(UINT32_MAX - 5 + n));
        assert(memory.request == CAPTURE_CONFIGURE);
        assert(memory.boards[0].ina_code == (n & 7));
        assert(memory.boards[1].hp_wiper == n % 257);
        capture_store(&memory.acknowledgement, sequence);
    }
    return 0;
}

static void mailbox_publication(void)
{
    memset(&memory, 0, sizeof(memory));
    memory.request_sequence = memory.acknowledgement = UINT32_MAX - 5;
    HANDLE reader = CreateThread(NULL, 0, mailbox_reader, NULL, 0, NULL);
    assert(reader);
    for (uint32_t n = 1; n <= 10000; ++n) {
        memory.boards[0].ina_code = n & 7;
        memory.boards[1].hp_wiper = n % 257;
        uint32_t sequence = capture_publish_request(&memory, CAPTURE_CONFIGURE);
        while (capture_load(&memory.acknowledgement) != sequence) {
            Sleep(0);
        }
    }
    assert(WaitForSingleObject(reader, 30000) == WAIT_OBJECT_0);
    CloseHandle(reader);
}

int main(void)
{
    overflow_and_wrap();
    threaded_publication();
    decoding_and_time();
    mailbox_publication();
    puts("10000 mailbox payload/acknowledgement handshakes and sequence rollover passed");
    puts("Shared-ring overflow, rollover, 200000 concurrent records, signed ADC and timestamps "
         "passed");
    return 0;
}
