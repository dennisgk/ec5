#include "autoconfig.h"
#include "autoconfig_schema.h"
#include "ble_control.h"
#include "cJSON.h"
#include "control.h"
#include "controller.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "emg.h"
#include "emg_signal.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "protocol.h"
#include "sdkconfig.h"
#include <errno.h>
#include <math.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#define CONFIG_PATH "/sdcard/autoconfig.json"
#if !defined(CONFIG_FATFS_LFN_HEAP) && !defined(CONFIG_FATFS_LFN_STACK)
#error "autoconfig.json exceeds FAT 8.3 names; enable FATFS long filename support"
#endif
#define TEMP_PATH "/sdcard/autoconfig.tmp"
#define BACKUP_PATH "/sdcard/autoconfig.bak"
static sdmmc_card_t *card;
static bool mounted;
static char *upload;
static size_t upload_size, upload_received;
static atomic_uint last_error;
static bool mount_card(void)
{
    if (mounted) {
        return true;
    }
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.max_freq_khz = 10000;
    spi_bus_config_t bus = {.mosi_io_num = 20,
                            .miso_io_num = 21,
                            .sclk_io_num = 19,
                            .quadwp_io_num = -1,
                            .quadhd_io_num = -1,
                            .max_transfer_sz = 4096};
    static bool bus_ready;
    esp_err_t error = bus_ready ? ESP_OK : spi_bus_initialize(host.slot, &bus, SPI_DMA_CH_AUTO);
    if (!error) {
        bus_ready = true;
        sdspi_device_config_t slot = SDSPI_DEVICE_CONFIG_DEFAULT();
        slot.gpio_cs = 18;
        slot.host_id = host.slot;
        esp_vfs_fat_sdmmc_mount_config_t config = {
            .format_if_mount_failed = false, .max_files = 3, .allocation_unit_size = 4096};
        error = esp_vfs_fat_sdspi_mount("/sdcard", &host, &slot, &config, &card);
    }
    mounted = error == ESP_OK;
    if (error) {
        atomic_store(&last_error, error);
    }
    return mounted;
}
void autoconfig_cancel(void)
{
    free(upload);
    upload = NULL;
    upload_size = upload_received = 0;
}
uint8_t autoconfig_begin(uint16_t size)
{
    autoconfig_cancel();
    if (!size || size > AUTOCONFIG_MAX) {
        return STATUS_INVALID;
    }
    if (!mount_card()) {
        return STATUS_STORAGE;
    }
    upload = calloc((size_t)size + 1, 1);
    if (!upload) {
        return STATUS_INTERNAL;
    }
    upload_size = size;
    return STATUS_OK;
}
uint8_t autoconfig_write(uint16_t offset, const uint8_t *data, size_t size)
{
    if (!upload || offset != upload_received || !size || size > upload_size - upload_received) {
        return STATUS_INVALID;
    }
    memcpy(upload + offset, data, size);
    upload_received += size;
    return STATUS_OK;
}
uint8_t autoconfig_commit(void)
{
    saved_config configuration;
    if (!upload || upload_received != upload_size ||
        !autoconfig_parse(upload, upload_size, &configuration)) {
        autoconfig_cancel();
        return STATUS_INVALID;
    }
    FILE *file = fopen(TEMP_PATH, "wb");
    bool good = false;
    if (file) {
        good = fwrite(upload, 1, upload_size, file) == upload_size;
        good = fflush(file) == 0 && good;
        good = fsync(fileno(file)) == 0 && good;
        good = fclose(file) == 0 && good;
    }
    if (good) {
        file = fopen(TEMP_PATH, "rb");
        good = file != NULL;
        char bytes[256];
        size_t offset = 0;
        while (file && offset < upload_size && good) {
            size_t count = upload_size - offset;
            if (count > sizeof(bytes)) {
                count = sizeof(bytes);
            }
            good = fread(bytes, 1, count, file) == count && !memcmp(bytes, upload + offset, count);
            offset += count;
        }
        if (file) {
            fclose(file);
        }
    }
    if (good) {
        struct stat st;
        if (stat(CONFIG_PATH, &st) == 0) {
            good = (unlink(BACKUP_PATH) == 0 || errno == ENOENT) &&
                   rename(CONFIG_PATH, BACKUP_PATH) == 0;
        }
        if (good) {
            good = rename(TEMP_PATH, CONFIG_PATH) == 0;
        }
    }
    atomic_store(&last_error, good ? 0 : errno ? errno : EIO);
    autoconfig_cancel();
    return good ? STATUS_OK : STATUS_STORAGE;
}
uint8_t autoconfig_delete(void)
{
    autoconfig_cancel();
    control_stop();
    control_handoff(false);
    emg_reset();
    angle_reset();
    emg_detection_config curve = {.threshold = 0.001f, .maximum = 0.01f};
    emg_control_adjust(0, false, &curve);
    emg_control_adjust(1, false, &curve);
    if (!mount_card()) {
        return STATUS_STORAGE;
    }
    bool good = true;
    const char *paths[] = {CONFIG_PATH, BACKUP_PATH, TEMP_PATH};
    for (unsigned i = 0; i < 3; ++i) {
        if (unlink(paths[i]) != 0 && errno != ENOENT) {
            good = false;
        }
    }
    if (!good) {
        atomic_store(&last_error, errno);
    }
    return good ? STATUS_OK : STATUS_STORAGE;
}
static char *read_file(const char *path, size_t *size)
{
    FILE *file = fopen(path, "rb");
    if (!file) {
        return NULL;
    }
    struct stat st;
    char *text = NULL;
    if (fstat(fileno(file), &st) == 0 && st.st_size > 0 && st.st_size <= AUTOCONFIG_MAX) {
        *size = st.st_size;
        text = calloc(*size + 1, 1);
        if (text && fread(text, 1, *size, file) != *size) {
            free(text);
            text = NULL;
        }
    }
    fclose(file);
    return text;
}
void autoconfig_boot(void)
{
    if (!mount_card()) {
        return;
    }
    saved_config c;
    bool valid = false;
    const char *paths[] = {CONFIG_PATH, BACKUP_PATH};
    for (unsigned i = 0; i < 2 && !valid; ++i) {
        size_t size = 0;
        char *text = read_file(paths[i], &size);
        if (text) {
            valid = autoconfig_parse(text, size, &c);
            free(text);
        }
    }
    if (!valid) {
        struct stat st;
        bool exists = stat(CONFIG_PATH, &st) == 0 || stat(BACKUP_PATH, &st) == 0;
        atomic_store(&last_error, exists ? STATUS_INVALID : 0);
        return;
    }
    control_stop();
    bool good = true;
    for (unsigned b = 0; b < 2; ++b) {
        board_config h = c.device.boards[b];
        uint16_t values[] = {h.ina_gain, h.adc_gain, h.high_pass, h.low_pass};
        for (unsigned property = 0; property < 4; ++property) {
            float actual;
            if (emg_adjust(b, property, values[property], &actual)) {
                good = false;
            }
        }
        processing_config p = c.device.processing[b];
        if (emg_integration_adjust(b, p.integration_ms) ||
            emg_fourier_adjust(b, p.hop, p.window, p.remove_mean) ||
            emg_notch_bank(b, p.notch_count, p.notches)) {
            good = false;
        }
        if (emg_control_adjust(b, false, &c.device.curves[b])) {
            good = false;
        }
    }
    if (emg_ignore_adjust(c.device.ignore_enabled, c.device.ignore_count, c.device.ranges)) {
        good = false;
    }
    if (c.acquisition) {
        uint8_t result = emg_activate(true);
        if (result) {
            atomic_store(&last_error, result);
        }
        if (result != STATUS_OK && (result < STATUS_HARDWARE_0 || result > STATUS_HARDWARE_BOTH)) {
            good = false;
        }
    }
    if (c.integration && emg_processing_enable(false, true)) {
        good = false;
    }
    if (c.fourier && emg_processing_enable(true, true)) {
        good = false;
    }
    for (unsigned b = 0; b < 2; ++b) {
        if (emg_resend_parameters(b, c.device.resend_ms[b])) {
            good = false;
        }
    }
    angle_configure(&c.angle); // Calibration intentionally never restored.
    if (c.bluetooth) {
        control_handoff(true);
    }
    if (!good) {
        atomic_store(&last_error, STATUS_NOT_READY);
        return;
    }
    // The motor task has a bounded startup/configuration retry window.
    for (unsigned i = 0; i < 100 && !controller_status().ready; ++i) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (c.bluetooth) {
        return; // Saved Angle Control needs new calibration.
    }
    if (c.device.owner >= 0) {
        uint8_t error = emg_control_adjust(c.device.owner, true, &c.device.curves[c.device.owner]);
        if (error) {
            atomic_store(&last_error, error);
        }
    } else if (c.manual_velocity && controller_status().ready && !controller_status().fault) {
        controller_inhibit(false);
        controller_request_output(true, c.manual_velocity);
    }
    ESP_LOGI("autoconfig", "Startup configuration applied; BLE %s", c.bluetooth ? "on" : "off");
}
uint32_t autoconfig_error(void)
{
    return atomic_load(&last_error);
}
