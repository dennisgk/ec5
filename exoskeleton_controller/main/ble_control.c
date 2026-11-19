#include "ble_control.h"
#include "control.h"
#include "controller.h"
#include "esp_heap_caps.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "host/ble_hs.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "protocol.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include <stdatomic.h>
#include <stdio.h>
#include <string.h>

// UUIDs 7c38a1XX-e82d-4ec1-a271-f2186a40c005, XX=00..04.
#define UUID(last)                                                                                 \
    BLE_UUID128_INIT(0x05, 0xc0, 0x40, 0x6a, 0x18, 0xf2, 0x71, 0xa2, 0xc1, 0x4e, 0x2d, 0xe8, last, \
                     0xa1, 0x38, 0x7c)
static const ble_uuid128_t service_uuid = UUID(0x00), identity_uuid = UUID(0x01),
                           command_uuid = UUID(0x02), result_uuid = UUID(0x03),
                           state_uuid = UUID(0x04), mechanics_uuid = UUID(0x05),
                           dynamics_uuid = UUID(0x06);
typedef struct {
    uint16_t connection;
    unsigned generation;
    uint8_t size, bytes[20];
} ble_job;
static QueueHandle_t requests;
static TaskHandle_t worker_handle;
static atomic_bool available, synced, advertise_requested;
static atomic_int connection = BLE_HS_CONN_HANDLE_NONE;
static atomic_uint generation, last_error, queue_full, notification_failures;
static atomic_bool result_subscribed, state_subscribed;
static uint8_t address_type, identity[8], state_bytes[20];
static uint8_t settings_bytes[32];
static uint16_t result_handle, state_handle;
static portMUX_TYPE snapshot_guard = portMUX_INITIALIZER_UNLOCKED;

static int gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        atomic_store(&result_subscribed, false);
        atomic_store(&state_subscribed, false);
        atomic_fetch_add(&generation, 1);
        atomic_store(&connection, event->connect.status == 0 ? event->connect.conn_handle
                                                             : BLE_HS_CONN_HANDLE_NONE);
        if (event->connect.status) {
            atomic_store(&last_error, event->connect.status);
        }
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        atomic_store(&connection, BLE_HS_CONN_HANDLE_NONE);
        atomic_fetch_add(&generation, 1);
        // Transport loss deliberately leaves local motor mode and target intact.
        break;
    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == result_handle) {
            atomic_store(&result_subscribed, event->subscribe.cur_notify);
        }
        if (event->subscribe.attr_handle == state_handle) {
            atomic_store(&state_subscribed, event->subscribe.cur_notify);
        }
        break;
    default:
        break;
    }
    return 0;
}

static int access_characteristic(uint16_t handle, uint16_t attribute,
                                 struct ble_gatt_access_ctxt *context, void *arg)
{
    (void)attribute;
    uintptr_t kind = (uintptr_t)arg;
    if (context->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        if (kind == 1) {
            return os_mbuf_append(context->om, identity, sizeof(identity))
                       ? BLE_ATT_ERR_INSUFFICIENT_RES
                       : 0;
        }
        if (kind == 4) {
            uint8_t bytes[20];
            portENTER_CRITICAL(&snapshot_guard);
            memcpy(bytes, state_bytes, sizeof(bytes));
            portEXIT_CRITICAL(&snapshot_guard);
            return os_mbuf_append(context->om, bytes, sizeof(bytes)) ? BLE_ATT_ERR_INSUFFICIENT_RES
                                                                     : 0;
        }
        if (kind == 5 || kind == 6) {
            uint8_t bytes[16];
            portENTER_CRITICAL(&snapshot_guard);
            memcpy(bytes, settings_bytes + (kind - 5) * 16, sizeof(bytes));
            portEXIT_CRITICAL(&snapshot_guard);
            return os_mbuf_append(context->om, bytes, sizeof(bytes)) ? BLE_ATT_ERR_INSUFFICIENT_RES
                                                                     : 0;
        }
        return BLE_ATT_ERR_READ_NOT_PERMITTED;
    }
    if (kind != 2 || context->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
        return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
    }
    uint16_t size = OS_MBUF_PKTLEN(context->om);
    if (size < 4 || size > 20) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }
    ble_job job = {
        .connection = handle, .generation = atomic_load(&generation), .size = (uint8_t)size};
    if (os_mbuf_copydata(context->om, 0, size, job.bytes) || job.bytes[0] != 1) {
        return BLE_ATT_ERR_UNLIKELY;
    }
    if (xQueueSend(requests, &job, 0) != pdTRUE) {
        atomic_fetch_add(&queue_full, 1);
        return BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    return 0;
}

static const struct ble_gatt_svc_def services[] = {
    {.type = BLE_GATT_SVC_TYPE_PRIMARY,
     .uuid = &service_uuid.u,
     .characteristics =
         (struct ble_gatt_chr_def[]){{.uuid = &identity_uuid.u,
                                      .access_cb = access_characteristic,
                                      .arg = (void *)1,
                                      .flags = BLE_GATT_CHR_F_READ},
                                     {.uuid = &command_uuid.u,
                                      .access_cb = access_characteristic,
                                      .arg = (void *)2,
                                      .flags = BLE_GATT_CHR_F_WRITE},
                                     {.uuid = &result_uuid.u,
                                      .access_cb = access_characteristic,
                                      .arg = (void *)3,
                                      .flags = BLE_GATT_CHR_F_NOTIFY,
                                      .val_handle = &result_handle},
                                     {.uuid = &state_uuid.u,
                                      .access_cb = access_characteristic,
                                      .arg = (void *)4,
                                      .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                                      .val_handle = &state_handle},
                                     {.uuid = &mechanics_uuid.u,
                                      .access_cb = access_characteristic,
                                      .arg = (void *)5,
                                      .flags = BLE_GATT_CHR_F_READ},
                                     {.uuid = &dynamics_uuid.u,
                                      .access_cb = access_characteristic,
                                      .arg = (void *)6,
                                      .flags = BLE_GATT_CHR_F_READ},
                                     {0}}},
    {0}};

static void on_reset(int reason)
{
    atomic_store(&last_error, reason);
    atomic_store(&synced, false);
    atomic_store(&connection, BLE_HS_CONN_HANDLE_NONE);
    atomic_fetch_add(&generation, 1);
}
static void on_sync(void)
{
    int error = ble_hs_id_infer_auto(0, &address_type);
    atomic_store(&synced, error == 0);
    if (error) {
        atomic_store(&last_error, error);
    }
}
static void host_task(void *unused)
{
    (void)unused;
    nimble_port_run();
    atomic_store(&available, false);
    vTaskDelete(NULL);
}
static void notify(uint16_t handle, const void *bytes, size_t size)
{
    int peer = atomic_load(&connection);
    if (peer == BLE_HS_CONN_HANDLE_NONE) {
        return;
    }
    if (handle == result_handle && !atomic_load(&result_subscribed)) {
        return;
    }
    if (handle == state_handle && !atomic_load(&state_subscribed)) {
        return;
    }
    struct os_mbuf *buffer = ble_hs_mbuf_from_flat(bytes, size);
    if (!buffer) {
        atomic_fetch_add(&notification_failures, 1);
        return;
    }
    // NimBLE consumes the mbuf even on error. Never retry or leak it here.
    int error = ble_gatts_notify_custom((uint16_t)peer, handle, buffer);
    if (error) {
        atomic_fetch_add(&notification_failures, 1);
        atomic_store(&last_error, error);
    }
}
static void update_advertising(void)
{
    if (!atomic_load(&synced)) {
        return;
    }
    bool wanted = atomic_load(&advertise_requested);
    int peer = atomic_load(&connection);
    if (!wanted) {
        if (ble_gap_adv_active()) {
            ble_gap_adv_stop();
        }
        if (peer != BLE_HS_CONN_HANDLE_NONE) {
            ble_gap_terminate(peer, BLE_ERR_REM_USER_CONN_TERM);
        }
        return;
    }
    if (peer != BLE_HS_CONN_HANDLE_NONE || ble_gap_adv_active()) {
        return;
    }
    struct ble_hs_adv_fields fields = {.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP,
                                       .uuids128 = (ble_uuid128_t *)&service_uuid,
                                       .num_uuids128 = 1,
                                       .uuids128_is_complete = 1};
    int error = ble_gap_adv_set_fields(&fields);
    if (!error) {
        const char *name = ble_svc_gap_device_name();
        struct ble_hs_adv_fields scan = {
            .name = (uint8_t *)name, .name_len = strlen(name), .name_is_complete = 1};
        error = ble_gap_adv_rsp_set_fields(&scan);
    }
    if (!error) {
        struct ble_gap_adv_params params = {.conn_mode = BLE_GAP_CONN_MODE_UND,
                                            .disc_mode = BLE_GAP_DISC_MODE_GEN,
                                            .itvl_min = 320,
                                            .itvl_max = 480};
        error = ble_gap_adv_start(address_type, NULL, BLE_HS_FOREVER, &params, gap_event, NULL);
    }
    if (error) {
        atomic_store(&last_error, error);
    }
}
static void worker(void *unused)
{
    (void)unused;
    uint32_t sequence = 0;
    unsigned cached_generation = 0;
    uint16_t previous_id = 0;
    uint8_t previous[20] = {0}, result[5] = {1};
    size_t previous_size = 0;
    while (true) {
        update_advertising();
        ble_job job;
        if (xQueueReceive(requests, &job, pdMS_TO_TICKS(control_bluetooth() ? 50 : 250)) ==
                pdTRUE &&
            job.generation == atomic_load(&generation) &&
            job.connection == atomic_load(&connection)) {
            uint16_t id = read_u16(job.bytes + 2);
            bool duplicate =
                cached_generation == job.generation && previous_size && id == previous_id;
            bool stale = cached_generation == job.generation && previous_size && id < previous_id;
            if (stale || (duplicate &&
                          (previous_size != job.size || memcmp(previous, job.bytes, job.size)))) {
                // Same ID, different bytes cannot replay an earlier result.
                uint8_t error[] = {1, job.bytes[1], job.bytes[2], job.bytes[3],
                                   stale ? STATUS_CANCELLED : STATUS_INVALID};
                notify(result_handle, error, sizeof(error));
            } else {
                if (!duplicate) {
                    control_command_lock();
                    uint8_t status =
                        job.generation == atomic_load(&generation)
                            ? control_ble_command(job.bytes[1], job.bytes + 4, job.size - 4)
                            : STATUS_CANCELLED;
                    control_command_unlock();
                    result[1] = job.bytes[1];
                    result[2] = job.bytes[2];
                    result[3] = job.bytes[3];
                    result[4] = status;
                    previous_id = id;
                    previous_size = job.size;
                    cached_generation = job.generation;
                    memcpy(previous, job.bytes, job.size);
                }
                if (job.generation == atomic_load(&generation)) {
                    notify(result_handle, result, sizeof(result));
                }
            }
        }
        angle_state a = angle_snapshot();
        controller_status_t motor = controller_status();
        uint8_t bytes[20] = {1, control_bluetooth(), (a.enabled ? 1 : 0) | (a.calibrated ? 2 : 0),
                             motor.fault ? 1 : 0};
        uint8_t *out = bytes + 4;
        write_float(&out, a.measured);
        write_float(&out, a.target);
        write_float(&out, motor.commanded_speed);
        write_u32(&out, ++sequence);
        uint8_t settings[32];
        out = settings;
        const float fields[] = {a.config.ratio,        a.config.direction, a.config.minimum,
                                a.config.maximum,      a.config.gain,      a.config.speed,
                                a.config.acceleration, a.config.deadband};
        for (unsigned i = 0; i < 8; ++i) {
            write_float(&out, fields[i]);
        }
        portENTER_CRITICAL(&snapshot_guard);
        memcpy(state_bytes, bytes, sizeof(bytes));
        memcpy(settings_bytes, settings, sizeof(settings));
        portEXIT_CRITICAL(&snapshot_guard);
        notify(state_handle, bytes, sizeof(bytes));
    }
}

void ble_control_init(void)
{
    // Reserve room for initialization, two tasks, and USB's already allocated
    // queues. Refuse optional BLE rather than running into allocator asserts.
    if (heap_caps_get_free_size(MALLOC_CAP_INTERNAL) < 65536 ||
        heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL) < 24576) {
        atomic_store(&last_error, ESP_ERR_NO_MEM);
        return;
    }
    requests = xQueueCreate(8, sizeof(ble_job));
    if (!requests) {
        return;
    }
    int error = nimble_port_init();
    if (error) {
        atomic_store(&last_error, error);
        return;
    }
    ble_svc_gap_init();
    ble_svc_gatt_init();
    identity[0] = 1;
    esp_efuse_mac_get_default(identity + 1);
    identity[7] = 0xC5;
    char name[25];
    snprintf(name, sizeof(name), "BrachiGoon-%02X%02X%02X", identity[4], identity[5], identity[6]);
    error = ble_svc_gap_device_name_set(name);
    if (!error) {
        error = ble_gatts_count_cfg(services);
    }
    if (!error) {
        error = ble_gatts_add_svcs(services);
    }
    if (error) {
        atomic_store(&last_error, error);
        nimble_port_deinit();
        return;
    }
    ble_hs_cfg.reset_cb = on_reset;
    ble_hs_cfg.sync_cb = on_sync;
    if (xTaskCreate(worker, "ble_control", 4096, NULL, 5, &worker_handle) != pdPASS) {
        atomic_store(&last_error, STATUS_INTERNAL);
        nimble_port_deinit();
        return;
    }
    TaskHandle_t host_handle;
    if (xTaskCreate(host_task, "nimble_host", 6144, NULL, 8, &host_handle) != pdPASS) {
        vTaskDelete(worker_handle);
        worker_handle = NULL;
        atomic_store(&last_error, ESP_ERR_NO_MEM);
        nimble_port_deinit();
        return;
    }
    atomic_store(&available, true);
}
bool ble_control_available(void)
{
    return atomic_load(&available);
}
void ble_control_advertise(bool enabled)
{
    atomic_store(&advertise_requested, enabled);
}
void ble_control_diagnostics(uint32_t words[8])
{
    words[0] = atomic_load(&available);
    words[1] = atomic_load(&synced);
    words[2] = atomic_load(&last_error);
    words[3] = atomic_load(&queue_full);
    words[4] = atomic_load(&notification_failures);
    words[5] = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    words[6] = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL);
    words[7] = worker_handle ? uxTaskGetStackHighWaterMark(worker_handle) : 0;
}
