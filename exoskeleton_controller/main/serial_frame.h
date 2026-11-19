#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define SERIAL_FRAME_MAX 512
#define SERIAL_CHUNK_MAX 351
#define SERIAL_MESSAGE_MAX 65535
#ifndef SERIAL_ASSEMBLY_MAX
#define SERIAL_ASSEMBLY_MAX SERIAL_MESSAGE_MAX
#endif
#define SERIAL_RETRY_US 250000
#define SERIAL_WATCHDOG_US 2000000

enum serial_kind {
    SERIAL_COMMAND = 1,
    SERIAL_RESPONSE,
    SERIAL_ERROR,
    SERIAL_DATA,
    SERIAL_ACK,
    SERIAL_KEEPALIVE,
    SERIAL_DISCONNECT
};
typedef struct {
    char session;
    uint32_t id;
    uint16_t start, end, total;
    uint8_t data[SERIAL_CHUNK_MAX];
} serial_frame;
typedef struct {
    uint8_t data[SERIAL_ASSEMBLY_MAX], coverage[(SERIAL_ASSEMBLY_MAX + 7u) / 8u];
    uint32_t id, received;
    uint16_t total;
    bool active, delivered;
} serial_assembly;

uint32_t serial_crc(uint32_t crc, const void *data, size_t size);
size_t serial_frame_encode(const serial_frame *frame, char out[SERIAL_FRAME_MAX]);
bool serial_frame_decode(const char *line, size_t size, serial_frame *frame);
// -1 conflict/busy, 0 incomplete, 1 newly complete, 2 already delivered.
int serial_assemble(serial_assembly *state, const serial_frame *frame);
static inline bool serial_watchdog_expired(int64_t now, int64_t last)
{
    return now >= last && now - last >= SERIAL_WATCHDOG_US;
}
