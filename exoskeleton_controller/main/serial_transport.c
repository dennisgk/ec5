#include "serial_transport.h"
#include "autoconfig.h"
#include "control.h"
#include "controller.h"
#include "driver/usb_serial_jtag.h"
#include "emg.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "serial_cache.h"
#include "serial_frame.h"
#include "telemetry.h"
#include <stdarg.h>
#include <stdatomic.h>
#include <stdio.h>
#include <string.h>

#define JOB_BYTES 1200
typedef struct {
    char session;
    uint32_t id;
    uint16_t size;
    uint8_t data[JOB_BYTES];
} tx_job;
typedef struct {
    char session;
    uint32_t id, hash;
    uint16_t size;
    uint8_t data[162];
} command_job;
typedef struct {
    uint32_t id;
    uint16_t start, end;
} acknowledgement;
typedef struct {
    char text[256];
} log_line;
static QueueHandle_t commands, responses, samples, frames, acknowledgements, logs;
static SemaphoreHandle_t cache_lock;
static serial_command_cache command_cache;
static serial_assembly assembly;
static atomic_int current_session;
static atomic_uint next_id, last_rx, pending_hello, last_hello, stop_id;
static atomic_bool hello_pending, hello_in_progress, reset_pending;
static atomic_uint peer_window = 1;
static StaticTask_t task_memory[4];
static StackType_t rx_stack[4096 / sizeof(StackType_t)];
static StackType_t tx_stack[6144 / sizeof(StackType_t)];
static StackType_t command_stack[6144 / sizeof(StackType_t)];
static StackType_t watchdog_stack[2048 / sizeof(StackType_t)];

char serial_session(void)
{
    return (char)atomic_load(&current_session);
}
static uint32_t new_id(void)
{
    uint32_t id = atomic_fetch_add(&next_id, 1);
    if (id == UINT32_MAX) {
        serial_close();
    }
    return id;
}
void serial_set_peer_window(unsigned window)
{
    atomic_store(&peer_window,
                 window < CONFIG_EC5_SERIAL_WINDOW ? window : CONFIG_EC5_SERIAL_WINDOW);
}
void serial_close(void)
{
    atomic_store(&current_session, 0);
    telemetry_end();
    atomic_store(&reset_pending, true);
}
static bool queue_frame(const serial_frame *frame)
{
    if (xQueueSend(frames, frame, 0) == pdTRUE) {
        return true;
    }
    serial_close();
    return false;
}
static void ack_frame(const serial_frame *received)
{
    serial_frame ack = {
        .session = received->session, .id = new_id(), .start = 0, .end = 9, .total = 9};
    uint8_t *out = ack.data;
    *out++ = SERIAL_ACK;
    write_u32(&out, received->id);
    write_u16(&out, received->start);
    write_u16(&out, received->end);
    queue_frame(&ack);
}
static bool reply_with_id(char session, uint32_t id, uint32_t request, const void *data,
                          size_t size)
{
    if (!session || size < 2 || size > 1024) {
        return false;
    }
    const uint8_t *wire = data;
    tx_job job = {.session = session, .id = id, .size = (uint16_t)(size + 5)};
    uint8_t *out = job.data;
    *out++ = wire[1] ? SERIAL_ERROR : SERIAL_RESPONSE;
    write_u32(&out, request);
    memcpy(out, wire, size);
    if (xQueueSend(responses, &job, 0) == pdTRUE) {
        return true;
    }
    serial_close();
    return false;
}
bool serial_reply(uint32_t request, const void *data, size_t size)
{
    return reply_with_id(serial_session(), new_id(), request, data, size);
}
bool serial_send_telemetry(const void *data, size_t size)
{
    char session = serial_session();
    if (!samples || !session || size > JOB_BYTES) {
        return false;
    }
    tx_job job = {.session = session, .id = new_id(), .size = (uint16_t)size};
    memcpy(job.data, data, size);
    return xQueueSend(samples, &job, 0) == pdTRUE;
}
static bool motion_request(const uint8_t *p, size_t n)
{
    return (n == 3 && p[0] == CMD_MOTOR && read_u16(p + 1) != 0) ||
           (n == 15 && p[0] == CMD_SET_MOTOR_CURVE && p[2] != 0);
}
static void accept_command(char session, uint32_t id, const uint8_t *p, size_t n)
{
    if (n < 2 || p[0] != SERIAL_COMMAND) {
        return;
    }
    ++p;
    --n;
    uint32_t hash = serial_crc(0, p, n);
    xSemaphoreTake(cache_lock, portMAX_DELAY);
    serial_cached_command *c = NULL;
    enum serial_cache_result admission = serial_cache_admit(&command_cache, session, id, hash, &c);
    if (admission == CACHE_NEW && n > 162) {
        c->response[0] = p[0];
        c->response[1] = STATUS_INVALID;
        c->size = 2;
        c->response_id = new_id();
        c->done = true;
        reply_with_id(session, c->response_id, id, c->response, c->size);
        xSemaphoreGive(cache_lock);
        return;
    }
    if (admission != CACHE_NEW || n > 162) {
        if (admission == CACHE_DONE) {
            reply_with_id(session, c->response_id, id, c->response, c->size);
        } else if (admission != CACHE_PENDING) {
            uint8_t error[] = {p[0], STATUS_INVALID};
            serial_reply(id, error, sizeof(error));
        }
        xSemaphoreGive(cache_lock);
        if (admission == CACHE_FULL) {
            serial_close();
        }
        return;
    }
    xSemaphoreGive(cache_lock);
    command_job job = {.session = session, .id = id, .hash = hash, .size = (uint16_t)n};
    memcpy(job.data, p, n);
    bool stop = (n == 3 && p[0] == CMD_MOTOR && read_u16(p + 1) == 0) ||
                (n == 1 && p[0] == CMD_RETRY_MOTOR_CAN);
    if (stop) {
        atomic_store(&stop_id, id);
        controller_inhibit(true);
    }
    BaseType_t queued = stop ? xQueueSendToFront(commands, &job, 0) : xQueueSend(commands, &job, 0);
    if (queued != pdTRUE) {
        serial_close();
    }
}

static void receive_frame(const serial_frame *f)
{
    char session = serial_session();
    static char assembly_session;
    if (session != assembly_session) {
        assembly.active = false;
        assembly_session = session;
    }
    if (f->session == '0' && f->id < 0x80000000u && !f->start && !f->end && !f->total) {
        if (session && atomic_load(&last_hello) == f->id) {
            atomic_store(&last_rx, (uint32_t)esp_timer_get_time());
            serial_frame response = {.session = session, .id = f->id};
            queue_frame(&response);
        } else {
            if (atomic_load(&hello_in_progress) && atomic_load(&pending_hello) == f->id) {
                return;
            }
            serial_close();
            atomic_store(&pending_hello, f->id);
            atomic_store(&hello_in_progress, true);
            atomic_store(&hello_pending, true);
        }
        return;
    }
    if (!session || f->session != session || f->id >= 0x80000000u || !f->total ||
        f->start == f->end) {
        return;
    }
    atomic_store(&last_rx, (uint32_t)esp_timer_get_time());
    if (f->start == 0 && f->end == f->total) {
        const uint8_t *p = f->data;
        switch (p[0]) {
        case SERIAL_ACK:
            if (f->total == 9) {
                acknowledgement ack = {read_u16(p + 1) | (uint32_t)read_u16(p + 3) << 16,
                                       read_u16(p + 5), read_u16(p + 7)};
                if (xQueueSend(acknowledgements, &ack, 0) != pdTRUE) {
                    serial_close();
                }
            }
            return;
        case SERIAL_KEEPALIVE:
            if (f->total == 1) {
                ack_frame(f);
            }
            return;
        case SERIAL_DISCONNECT:
            if (f->total == 1) {
                ack_frame(f);
                serial_close();
            }
            return;
        default:
            ack_frame(f);
            accept_command(session, f->id, p, f->total);
            return;
        }
    }
    int result = serial_assemble(&assembly, f);
    if (result < 0) {
        serial_close();
        return;
    }
    ack_frame(f);
    if (result == 1) {
        accept_command(session, f->id, assembly.data, assembly.total);
    }
}

static void receiver_task(void *unused)
{
    (void)unused;
    char line[SERIAL_FRAME_MAX], bytes[128];
    size_t used = 0;
    bool overflow = false;
    serial_frame frame;
    while (true) {
        int count = usb_serial_jtag_read_bytes(bytes, sizeof(bytes), pdMS_TO_TICKS(20));
        for (int n = 0; n < count; ++n) {
            if (used < sizeof(line)) {
                line[used++] = bytes[n];
            } else {
                overflow = true;
            }
            if (bytes[n] == '\n') {
                if (!overflow && serial_frame_decode(line, used, &frame)) {
                    receive_frame(&frame);
                }
                used = 0;
                overflow = false;
            }
        }
    }
}

static void command_task(void *unused)
{
    (void)unused;
    bool initialized = false;
    unsigned session_counter = esp_random() % 61;
    const char sessions[] = "123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";
    uint8_t output[1024];
    while (true) {
        bool reset = atomic_exchange(&reset_pending, false);
        bool hello = atomic_exchange(&hello_pending, false);
        uint32_t hello_id = atomic_load(&pending_hello);
        if (reset || hello) {
            autoconfig_cancel();
            initialized = false;
            xQueueReset(commands);
            xSemaphoreTake(cache_lock, portMAX_DELAY);
            memset(&command_cache, 0, sizeof(command_cache));
            xSemaphoreGive(cache_lock);
            // RX owns assembly; session transitions are observed there via ID.
            atomic_store(&stop_id, 0);
        }
        if (hello && hello_id == atomic_load(&pending_hello)) {
            uint32_t id = hello_id;
            char session = sessions[(++session_counter) % 61];
            atomic_store(&next_id, 0x80000000u);
            atomic_store(&peer_window, 1);
            atomic_store(&last_hello, id);
            atomic_store(&last_rx, (uint32_t)esp_timer_get_time());
            atomic_store(&current_session, session);
            atomic_store(&hello_in_progress, false);
            serial_frame response = {.session = session, .id = id};
            queue_frame(&response);
        }
        command_job job;
        if (xQueueReceive(commands, &job, pdMS_TO_TICKS(10)) != pdTRUE) {
            continue;
        }
        if (job.session != serial_session()) {
            continue;
        }
        size_t count;
        if (job.id < atomic_load(&stop_id) && motion_request(job.data, job.size)) {
            output[0] = job.data[0];
            output[1] = STATUS_CANCELLED;
            count = 2;
        } else {
            control_command_lock();
            count = controller_command(job.data, job.size, output, &initialized);
            if (job.id < atomic_load(&stop_id) && motion_request(job.data, job.size)) {
                control_stop();
            }
            control_command_unlock();
        }
        if (job.session != serial_session()) {
            continue;
        }
        xSemaphoreTake(cache_lock, portMAX_DELAY);
        for (unsigned k = 0; k < SERIAL_CACHE_COUNT; ++k) {
            serial_cached_command *c = &command_cache.entries[k];
            if (c->used && c->id == job.id && c->session == job.session) {
                c->size = (uint16_t)count;
                memcpy(c->response, output, count);
                c->response_id = new_id();
                c->done = true;
                reply_with_id(job.session, c->response_id, job.id, c->response, count);
                break;
            }
        }
        xSemaphoreGive(cache_lock);
    }
}

typedef struct {
    tx_job job;
    uint16_t next;
    unsigned pending;
    bool active;
} source;
typedef struct {
    bool active;
    unsigned source, retries;
    uint16_t start, end;
    uint32_t id;
    int64_t sent;
} outstanding;
static bool emit_line(const char *line, size_t count)
{
    size_t sent = 0;
    int64_t deadline = esp_timer_get_time() + 20000;
    while (sent < count && esp_timer_get_time() < deadline) {
        int n = usb_serial_jtag_write_bytes(line + sent, count - sent, pdMS_TO_TICKS(5));
        if (n > 0) {
            sent += (size_t)n;
        }
    }
    if (sent != count) {
        serial_close();
        return false;
    }
    return true;
}
static bool emit_frame(const serial_frame *frame)
{
    char line[SERIAL_FRAME_MAX];
    size_t n = serial_frame_encode(frame, line);
    return n && emit_line(line, n);
}
static void sender_task(void *unused)
{
    (void)unused;
    static source sources[2];
    outstanding pending[16] = {0};
    char previous = 0;
    serial_frame frame;
    tx_job incoming;
    while (true) {
        char session = serial_session();
        if (session != previous) {
            memset(sources, 0, sizeof(sources));
            memset(pending, 0, sizeof(pending));
            previous = session;
        }
        acknowledgement ack;
        while (xQueueReceive(acknowledgements, &ack, 0) == pdTRUE) {
            for (unsigned k = 0; k < 16; ++k) {
                outstanding *p = &pending[k];
                if (p->active && p->id == ack.id && p->start == ack.start && p->end == ack.end) {
                    p->active = false;
                    --sources[p->source].pending;
                }
            }
        }
        for (unsigned n = 0; n < 4 && xQueueReceive(frames, &frame, 0) == pdTRUE; ++n) {
            // A disconnect ACK is allowed to drain after session retirement.
            if (frame.session == session || (frame.total == 9 && frame.data[0] == SERIAL_ACK)) {
                emit_frame(&frame);
            }
        }
        for (unsigned k = 0; k < 2; ++k) {
            source *s = &sources[k];
            if (s->active && s->next == s->job.size && !s->pending) {
                s->active = false;
            }
            QueueHandle_t queue = k ? samples : responses;
            if (session && !s->active && xQueueReceive(queue, &incoming, 0) == pdTRUE &&
                incoming.session == session) {
                s->job = incoming;
                s->next = 0;
                s->pending = 0;
                s->active = true;
            }
        }
        unsigned active = 0;
        int64_t now = esp_timer_get_time();
        for (unsigned k = 0; k < 16; ++k) {
            outstanding *p = &pending[k];
            if (!p->active) {
                continue;
            }
            ++active;
            if (now - p->sent >= SERIAL_RETRY_US) {
                if (p->retries++ >= 3) {
                    serial_close();
                    break;
                }
                source *s = &sources[p->source];
                frame = (serial_frame){.session = session,
                                       .id = p->id,
                                       .start = p->start,
                                       .end = p->end,
                                       .total = s->job.size};
                memcpy(frame.data, s->job.data + p->start, p->end - p->start);
                emit_frame(&frame);
                p->sent = now;
            }
        }
        if (session && active < atomic_load(&peer_window)) {
            for (unsigned channel = 0; channel < 2; ++channel) {
                source *s = &sources[channel];
                if (!s->active || s->next == s->job.size) {
                    continue;
                }
                for (unsigned k = 0; k < 16; ++k) {
                    if (pending[k].active) {
                        continue;
                    }
                    uint16_t end = (uint16_t)(s->job.size - s->next > SERIAL_CHUNK_MAX
                                                  ? s->next + SERIAL_CHUNK_MAX
                                                  : s->job.size);
                    frame = (serial_frame){.session = session,
                                           .id = s->job.id,
                                           .start = s->next,
                                           .end = end,
                                           .total = s->job.size};
                    memcpy(frame.data, s->job.data + s->next, end - s->next);
                    pending[k] = (outstanding){true, channel, 0, s->next, end, s->job.id, now};
                    ++s->pending;
                    s->next = end;
                    emit_frame(&frame);
                    break;
                }
                break;
            }
        }
        log_line log;
        if (session && xQueueReceive(logs, &log, 0) == pdTRUE) {
            emit_line(log.text, strlen(log.text));
        }
        vTaskDelay(1);
    }
}
static int log_output(const char *format, va_list args)
{
    log_line line;
    int n = vsnprintf(line.text, sizeof(line.text) - 2, format, args);
    size_t used = strlen(line.text);
    if (!used || line.text[used - 1] != '\n') {
        line.text[used++] = '\n';
        line.text[used] = 0;
    }
    xQueueSend(logs, &line, 0);
    return n;
}
static void watchdog_task(void *unused)
{
    (void)unused;
    while (true) {
        if (serial_session() && (uint32_t)((uint32_t)esp_timer_get_time() -
                                           atomic_load(&last_rx)) >= SERIAL_WATCHDOG_US) {
            serial_close();
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
void serial_prepare(void)
{
    commands = xQueueCreate(8, sizeof(command_job));
    responses = xQueueCreate(8, sizeof(tx_job));
    samples = xQueueCreate(8, sizeof(tx_job));
    frames = xQueueCreate(32, sizeof(serial_frame));
    acknowledgements = xQueueCreate(32, sizeof(acknowledgement));
    logs = xQueueCreate(8, sizeof(log_line));
    cache_lock = xSemaphoreCreateMutex();
    configASSERT(commands && responses && samples && frames && acknowledgements && logs &&
                 cache_lock);
    usb_serial_jtag_driver_config_t config = {.tx_buffer_size = 4096, .rx_buffer_size = 8192};
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&config));
}
void serial_start(void)
{
    esp_log_set_vprintf(log_output);
    atomic_store(&next_id, 0x80000000u);
    configASSERT(xTaskCreateStatic(receiver_task, "serial_rx", sizeof(rx_stack), NULL, 9, rx_stack,
                                   &task_memory[0]));
    configASSERT(xTaskCreateStatic(sender_task, "serial_tx", sizeof(tx_stack), NULL, 7, tx_stack,
                                   &task_memory[1]));
    configASSERT(xTaskCreateStatic(command_task, "commands", sizeof(command_stack), NULL, 8,
                                   command_stack, &task_memory[2]));
    configASSERT(xTaskCreateStatic(watchdog_task, "serial_watchdog", sizeof(watchdog_stack), NULL,
                                   20, watchdog_stack, &task_memory[3]));
}
