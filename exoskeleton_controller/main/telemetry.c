#include "telemetry.h"
#include "esp_timer.h"
#include "serial_frame.h"
#include "serial_transport.h"
#include <stdatomic.h>
#include <string.h>

const uint16_t telemetry_sizes[TELEMETRY_STREAMS] = {16, 16, 16, 16, 24, 24, 536, 536, 24, 24};
static atomic_uint epoch = 1, subscriptions;
typedef struct {
    uint8_t data[TELEMETRY_MESSAGE_MAX];
    unsigned count, size;
    uint32_t epoch;
    int64_t started;
} stage;
// Each stream has one producer; commands only modify atomic mask/epoch.
static stage stages[TELEMETRY_STREAMS];
uint32_t telemetry_session(void)
{
    return (uint8_t)serial_session();
}
uint32_t telemetry_epoch(void)
{
    return atomic_load(&epoch);
}
uint32_t telemetry_changed(void)
{
    return atomic_fetch_add(&epoch, 1) + 1;
}
void telemetry_end(void)
{
    atomic_store(&subscriptions, 0);
    telemetry_changed();
}
bool telemetry_subscribe(uint16_t mask)
{
    if (mask & ~1023u) {
        return false;
    }
    atomic_store(&subscriptions, mask);
    telemetry_changed();
    return true;
}
void telemetry_flush(unsigned stream)
{
    stage *s = &stages[stream];
    if (s->count && s->epoch == telemetry_epoch() &&
        (atomic_load(&subscriptions) & (1u << stream))) {
        uint8_t *out = s->data;
        *out++ = SERIAL_DATA;
        *out++ = (uint8_t)(stream + 1);
        write_u32(&out, s->epoch);
        write_u16(&out, (uint16_t)s->count);
        serial_send_telemetry(s->data, s->size);
    }
    s->count = 0;
    s->size = 8;
}
void telemetry_append(unsigned stream, const void *record, unsigned size, uint32_t revision)
{
    if (stream >= TELEMETRY_STREAMS || size != telemetry_sizes[stream]) {
        return;
    }
    stage *s = &stages[stream];
    if (s->epoch != revision) {
        s->count = 0;
        s->size = 8;
        s->epoch = revision;
    }
    if (!(atomic_load(&subscriptions) & (1u << stream)) || revision != telemetry_epoch()) {
        return;
    }
    if (s->count && (s->size + size > sizeof(s->data) || s->count >= 64)) {
        telemetry_flush(stream);
    }
    if (!s->count) {
        s->started = esp_timer_get_time();
        s->size = 8;
    }
    memcpy(s->data + s->size, record, size);
    s->size += size;
    ++s->count;
    if (size > 500 || esp_timer_get_time() - s->started >= 20000) {
        telemetry_flush(stream);
    }
}
