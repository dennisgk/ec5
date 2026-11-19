#include "serial_cache.h"
#include "serial_frame.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static serial_assembly assembly;
static serial_command_cache cache;
int main(void)
{
    assert(serial_crc(0, "123456789", 9) == 0xCBF43926u);
    FILE *vectors = fopen("tests/protocol_vectors.txt", "rb");
    assert(vectors);
    char input[1024], output[512];
    unsigned count = 0;
    serial_frame frame;
    while (fgets(input, sizeof(input), vectors)) {
        size_t size = strlen(input);
        if (size >= 2 && input[size - 2] == '\r') {
            input[size - 2] = '\n';
            input[--size] = 0;
        }
        assert(serial_frame_decode(input, size, &frame));
        size_t encoded = serial_frame_encode(&frame, output);
        assert(encoded == size && !memcmp(input, output, size));
        input[33] = input[33] == '0' ? '1' : '0';
        assert(!serial_frame_decode(input, size, &frame));
        ++count;
    }
    fclose(vectors);
    assert(count == 7);
    // Full uint16 range, holes, variable chunk sizes, duplicate and conflict.
    frame = (serial_frame){.session = 'K', .id = 55, .total = 65535};
    for (unsigned end = 65535; end > 0;) {
        unsigned start = end > 351 ? end - 351 : 0;
        frame.start = (uint16_t)start;
        frame.end = (uint16_t)end;
        for (unsigned n = start; n < end; ++n) {
            frame.data[n - start] = (uint8_t)n;
        }
        int result = serial_assemble(&assembly, &frame);
        assert(result == (start ? 0 : 1));
        end = start;
    }
    for (unsigned n = 0; n < 65535; ++n) {
        assert(assembly.data[n] == (uint8_t)n);
    }
    assert(serial_assemble(&assembly, &frame) == 2);
    frame.data[0] ^= 1;
    assert(serial_assemble(&assembly, &frame) == -1);
    frame = (serial_frame){.session = 'K', .id = 56, .start = 0, .end = 351, .total = 351};
    memset(frame.data, 0xFF, sizeof(frame.data));
    size_t size = serial_frame_encode(&frame, output);
    assert(size == 511);
    assert(serial_frame_decode(output, size, &frame));
    assert(!serial_frame_decode(output, size - 1, &frame));
    frame.end = 352;
    assert(serial_frame_encode(&frame, output) == 0);
    assert(!serial_watchdog_expired(1999999, 0));
    assert(serial_watchdog_expired(2000000, 0));
    serial_cached_command *entry = NULL;
    unsigned executions = 0;
    if (serial_cache_admit(&cache, 'K', 42, 123, &entry) == CACHE_NEW) {
        ++executions;
    }
    assert(serial_cache_admit(&cache, 'K', 42, 123, &entry) == CACHE_PENDING);
    entry->done = true;
    entry->response_id = 0x80000000u;
    entry->size = 2;
    assert(serial_cache_admit(&cache, 'K', 42, 123, &entry) == CACHE_DONE);
    assert(entry->response_id == 0x80000000u && executions == 1);
    assert(serial_cache_admit(&cache, 'K', 42, 124, &entry) == CACHE_CONFLICT);
    for (uint32_t id = 43; id < 75; ++id) {
        assert(serial_cache_admit(&cache, 'K', id, 123, &entry) == CACHE_NEW);
        entry->done = true;
    }
    assert(serial_cache_admit(&cache, 'K', 42, 123, &entry) == CACHE_STALE);
    memset(&cache, 0, sizeof(cache));
    assert(serial_cache_admit(&cache, 'L', 42, 123, &entry) == CACHE_NEW);
    puts("GOON C golden frames, CRC/Base64, 65535-byte coverage, overlap and watchdog passed");
    return 0;
}
