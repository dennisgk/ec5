#include "autoconfig.h"
#include "ble_control.h"
#include "control.h"
#include "controller.h"
#include "emg.h"
#include "nvs_flash.h"
#include "serial_transport.h"

void app_main(void)
{
    esp_err_t error = nvs_flash_init();
    if (error == ESP_ERR_NVS_NO_FREE_PAGES || error == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        error = nvs_flash_init();
    }
    ESP_ERROR_CHECK(error);
    control_init();
    processing_initialize();
    controller_init();
    emg_init();
    serial_prepare(); // Reserve essential USB resources before optional BLE.
    ble_control_init();
    autoconfig_boot();
    serial_start();
}
