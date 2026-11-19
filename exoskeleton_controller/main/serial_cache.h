#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define SERIAL_CACHE_COUNT 16
typedef struct {
    char session;
    uint32_t id, hash, response_id;
    uint16_t size;
    bool used, done;
    uint8_t response[1024];
} serial_cached_command;
typedef struct {
    serial_cached_command entries[SERIAL_CACHE_COUNT];
    unsigned cursor;
    uint32_t latest;
} serial_command_cache;
enum serial_cache_result {
    CACHE_NEW,
    CACHE_PENDING,
    CACHE_DONE,
    CACHE_CONFLICT,
    CACHE_STALE,
    CACHE_FULL
};

// Caller serializes access. Cache eviction never permits an old command to run
// again: IDs at/below latest are stale even if their result was evicted.
static inline enum serial_cache_result serial_cache_admit(serial_command_cache *cache, char session,
                                                          uint32_t id, uint32_t hash,
                                                          serial_cached_command **entry)
{
    for (unsigned n = 0; n < SERIAL_CACHE_COUNT; ++n) {
        serial_cached_command *c = &cache->entries[n];
        if (c->used && c->session == session && c->id == id) {
            *entry = c;
            if (c->hash != hash) {
                return CACHE_CONFLICT;
            }
            return c->done ? CACHE_DONE : CACHE_PENDING;
        }
    }
    if (id <= cache->latest || id >= 0x80000000u) {
        return CACHE_STALE;
    }
    serial_cached_command *c = &cache->entries[cache->cursor % SERIAL_CACHE_COUNT];
    if (c->used && !c->done) {
        return CACHE_FULL;
    }
    *c = (serial_cached_command){.session = session, .id = id, .hash = hash, .used = true};
    ++cache->cursor;
    cache->latest = id;
    *entry = c;
    return CACHE_NEW;
}
