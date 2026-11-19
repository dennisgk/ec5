#include "control.h"
#include "controller.h"
#include "driver/gpio.h"
#include "driver/twai.h"
#include "emg.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "robstride06.h"
#include "sdkconfig.h"
#include "soc/gpio_sig_map.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#ifdef CONFIG_ESP_CONSOLE_UART_DEFAULT
#error "UART0 console uses motor CAN GPIO16/17; select USB Serial/JTAG console for EC4"
#endif

static SemaphoreHandle_t state_lock;
static controller_status_t state;
static motor_feedback feedback;
static bool desired_enabled;
static bool forced_inhibit = true;
static float desired_velocity;
static motor_can_diagnostics can;
static SemaphoreHandle_t retry_done;
static bool retry_requested, driver_installed, confirming_configuration, fault_stopping;
static uint32_t transmission_stage;
static bool boot_initializing = true;
enum { CAN_INSTALLED = 1, CAN_CONFIGURED = 2, CAN_CONFIGURING = 4 };
enum {
    STAGE_ZERO = 1,
    STAGE_DISABLE,
    STAGE_MODE,
    STAGE_CURRENT,
    STAGE_TORQUE,
    STAGE_REPORT,
    STAGE_RUNTIME,
    STAGE_INSTALL,
    STAGE_START,
    STAGE_RESET,
    STAGE_GPIO
};
static void lock(void)
{
    xSemaphoreTake(state_lock, portMAX_DELAY);
}
static void unlock(void)
{
    xSemaphoreGive(state_lock);
}
controller_status_t controller_status(void)
{
    lock();
    controller_status_t result = state;
    unlock();
    return result;
}
motor_feedback controller_feedback(void)
{
    lock();
    motor_feedback result = feedback;
    unlock();
    return result;
}

motor_can_diagnostics controller_can_diagnostics(void)
{
    lock();
    motor_can_diagnostics result = can;
    unlock();
    return result;
}

bool controller_retry_configuration(void)
{
    lock();
    // Retry CAN/configuration, never clear a motor-reported hardware fault.
    if (boot_initializing || (can.flags & CAN_CONFIGURING) || retry_requested ||
        (state.fault && (state.fault_code == 2 || state.fault_code == 3))) {
        unlock();
        return false;
    }
    desired_enabled = false;
    desired_velocity = 0;
    forced_inhibit = true;
    state.selected_speed = 0;
    state.ready = false;
    xSemaphoreTake(retry_done, 0);
    retry_requested = true;
    unlock();
    if (xSemaphoreTake(retry_done, pdMS_TO_TICKS(2000)) != pdTRUE) {
        return false;
    }
    controller_status_t result = controller_status();
    return result.ready && !result.fault;
}

// Only the motor task touches the TWAI driver. serial reads cached snapshots, so
// a status request cannot race driver uninstall/reinstallation.
static void refresh_can_diagnostics(void)
{
    twai_status_info_t info = {0};
    bool valid = driver_installed && twai_get_status_info(&info) == ESP_OK;
    gpio_io_config_t tx = {0}, rx = {0};
    bool pins_valid = gpio_get_io_config(GPIO_NUM_16, &tx) == ESP_OK &&
                      gpio_get_io_config(GPIO_NUM_17, &rx) == ESP_OK;
    lock();
    can.flags &= 7;
    if (pins_valid) {
        can.flags |= 0x100 | (tx.oe ? 8 : 0) | (rx.ie ? 16 : 0) |
                     (tx.sig_out == TWAI0_TX_IDX ? 32 : 0) | (tx.oe_ctrl_by_periph ? 64 : 0) |
                     (gpio_get_level(GPIO_NUM_17) ? 128 : 0);
    }
    can.driver_state = valid ? (uint32_t)info.state : UINT32_MAX;
    can.tx_pending = valid ? info.msgs_to_tx : 0;
    can.rx_pending = valid ? info.msgs_to_rx : 0;
    can.tx_errors = valid ? info.tx_error_counter : 0;
    can.rx_errors = valid ? info.rx_error_counter : 0;
    can.tx_failed = valid ? info.tx_failed_count : 0;
    can.rx_missed = valid ? info.rx_missed_count : 0;
    can.rx_overrun = valid ? info.rx_overrun_count : 0;
    can.arbitration_lost = valid ? info.arb_lost_count : 0;
    can.bus_errors = valid ? info.bus_error_count : 0;
    unlock();
}

static void note_alerts(uint32_t alerts)
{
    lock();
    can.alerts |= alerts;
    unlock();
}

static void transmission_failed(esp_err_t error, uint32_t id, uint32_t parameter_address)
{
    refresh_can_diagnostics();
    lock();
    ++can.tx_rejected;
    bool first = can.last_error == ESP_OK;
    if (first) {
        can.last_error = (uint32_t)error;
        can.last_stage = transmission_stage;
        can.last_id = id;
        can.last_parameter = parameter_address;
        can.failure_state = can.driver_state;
        can.failure_tx_errors = can.tx_errors;
        can.failure_rx_errors = can.rx_errors;
        can.failure_bus_errors = can.bus_errors;
        can.failure_time_us = (uint64_t)esp_timer_get_time();
    }
    motor_can_diagnostics snapshot = can;
    unlock();
    if (first) {
        ESP_LOGE("motor",
                 "CAN stage=%lu id=0x%08lx parameter=0x%04lx: %s; state=%lu TX=%lu TEC=%lu REC=%lu "
                 "bus_errors=%lu",
                 (unsigned long)snapshot.last_stage, (unsigned long)id,
                 (unsigned long)parameter_address, esp_err_to_name(error),
                 (unsigned long)snapshot.driver_state, (unsigned long)snapshot.tx_pending,
                 (unsigned long)snapshot.tx_errors, (unsigned long)snapshot.rx_errors,
                 (unsigned long)snapshot.bus_errors);
    }
}

void controller_request_output(bool enabled, float velocity)
{
    lock();
    desired_enabled = enabled && isfinite(velocity) && fabsf(velocity) <= 50;
    desired_velocity = desired_enabled ? velocity : 0;
    state.selected_speed = isfinite(velocity) ? velocity : 0;
    unlock();
}
void controller_inhibit(bool inhibited)
{
    lock();
    forced_inhibit = inhibited;
    if (inhibited) {
        desired_enabled = false;
    }
    unlock();
}
static bool transmit_once(unsigned type, const uint8_t bytes[8], esp_err_t *failure)
{
    twai_message_t frame = {.identifier = (type << 24) | (0xFD << 8) | 0x7F,
                            .extd = 1,
                            .ss = confirming_configuration || fault_stopping,
                            .data_length_code = 8};
    memcpy(frame.data, bytes, 8);
    uint32_t alerts = 0;
    if (confirming_configuration && twai_read_alerts(&alerts, 0) == ESP_OK) {
        note_alerts(alerts);
    }
    esp_err_t error = twai_transmit(&frame, confirming_configuration ? pdMS_TO_TICKS(20) : 0);
    if (error == ESP_OK) {
        lock();
        ++can.tx_queued;
        unlock();
    }
    if (error == ESP_OK && confirming_configuration) {
        // Queue acceptance is not CAN transmission success. Single-shot avoids
        // a missing peer endlessly retrying and filling the startup TX queue.
        int64_t deadline = esp_timer_get_time() + 100000;
        error = ESP_ERR_TIMEOUT;
        while (esp_timer_get_time() < deadline) {
            alerts = 0;
            if (twai_read_alerts(&alerts, pdMS_TO_TICKS(10)) != ESP_OK) {
                continue;
            }
            note_alerts(alerts);
            if (alerts & (TWAI_ALERT_TX_FAILED | TWAI_ALERT_BUS_OFF)) {
                error = ESP_FAIL;
                break;
            }
            if (alerts & TWAI_ALERT_TX_SUCCESS) {
                error = ESP_OK;
                lock();
                ++can.tx_confirmed;
                unlock();
                break;
            }
        }
    }
    *failure = error;
    return error == ESP_OK;
}

static bool transmit(unsigned type, const uint8_t bytes[8])
{
    esp_err_t error = ESP_OK;
    unsigned attempts = confirming_configuration ? 3 : 1;
    for (unsigned attempt = 0; attempt < attempts; ++attempt) {
        if (transmit_once(type, bytes, &error)) {
            return true;
        }
        // Retry only a completed single-shot failure, never a still-pending
        // frame (whose eventual ACK could be mistaken for a later frame).
        twai_status_info_t info;
        if (error != ESP_FAIL || twai_get_status_info(&info) != ESP_OK ||
            info.state != TWAI_STATE_RUNNING || info.msgs_to_tx != 0 || attempt + 1 == attempts) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    transmission_failed(error, (type << 24) | (0xFD << 8) | 0x7F,
                        type == 0x12 ? bytes[0] | (uint32_t)bytes[1] << 8 : 0);
    return false;
}
static bool parameter(uint16_t address, float value)
{
    uint8_t bytes[8] = {address & 255, address >> 8};
    memcpy(bytes + 4, &value, 4);
    return transmit(0x12, bytes);
}
static bool enable(bool value)
{
    const uint8_t bytes[8] = {0};
    return transmit(value ? 3 : 4, bytes);
}
static void fail(uint32_t code, const char *reason)
{
    lock();
    bool changed = !state.fault || state.fault_code != code;
    state.fault_code = code;
    state.fault = true;
    state.ready = false;
    can.flags &= ~CAN_CONFIGURED;
    state.running = false;
    state.commanded_speed = 0;
    snprintf(state.reason, sizeof(state.reason), "%s", reason);
    unlock();
    if (changed) {
        // A latched fault does not need repeated stop traffic on every feedback
        // frame. These best-effort stops must not retry forever on a dead bus.
        fault_stopping = true;
        parameter(0x700A, 0);
        enable(false);
        fault_stopping = false;
        ESP_LOGE("motor", "Fault %lu: %s", (unsigned long)code, reason);
    }
}
static void receive_feedback(void)
{
    twai_message_t message;
    for (unsigned i = 0; i < 32 && twai_receive(&message, 0) == ESP_OK; ++i) {
        lock();
        ++can.rx_frames;
        can.last_rx_id = message.identifier;
        unlock();
        if (!message.extd || message.rtr || message.data_length_code != 8 ||
            (message.identifier & 255) != 0xFD || ((message.identifier >> 8) & 255) != 0x7F) {
            continue;
        }
        unsigned type = message.identifier >> 24;
        lock();
        ++can.rx_motor_frames;
        unlock();
        if (type == 0x15) {
            lock();
            can.motor_faults = rs06_u32_le(message.data);
            can.motor_warnings = rs06_u32_le(message.data + 4);
            can.motor_faults_seen |= can.motor_faults;
            can.motor_warnings_seen |= can.motor_warnings;
            ++can.fault_frames;
            unlock();
            // Bytes 0..3 are faults; an all-zero fault word is normal.
            // Bytes 4..7 are warnings, not a reason to invent a latched fault.
            if (message.data[0] || message.data[1] || message.data[2] || message.data[3]) {
                fail(2, "Motor reported a fault");
            }
            continue;
        }
        if (!rs06_is_feedback(type)) {
            continue;
        }
        unsigned position = (message.data[0] << 8) | message.data[1],
                 velocity = (message.data[2] << 8) | message.data[3];
        lock();
        can.feedback_faults = rs06_feedback_faults(message.identifier);
        can.feedback_faults_seen |= can.feedback_faults;
        can.feedback_mode = rs06_feedback_mode(message.identifier);
        can.temperature_tenths = (uint32_t)message.data[6] << 8 | message.data[7];
        ++can.feedback_frames;
        feedback =
            (motor_feedback){.time_us = esp_timer_get_time(),
                             .valid = 1,
                             .position_rad = position * (25.1327412287f / 65535) - 12.5663706144f,
                             .velocity_rad_s = velocity * (100.0f / 65535) - 50.0f};
        unlock();
        if (rs06_feedback_faults(message.identifier)) {
            fail(3, "Motor feedback fault flags set");
        }
    }
}
static bool settle_motor(void)
{
    vTaskDelay(pdMS_TO_TICKS(100));
    receive_feedback();
    return !controller_status().fault;
}

static bool configure_motor(bool reinstall)
{
    lock();
    state.ready = state.running = state.fault = false;
    state.commanded_speed = 0;
    state.fault_code = 0;
    state.reason[0] = 0;
    feedback.valid = 0;
    can.flags = (driver_installed ? CAN_INSTALLED : 0) | CAN_CONFIGURING;
    can.last_error = can.last_stage = can.last_id = can.last_parameter = 0;
    can.failure_time_us = 0;
    unlock();
    confirming_configuration = true;
    bool ok = false;
    esp_err_t error;
    if (reinstall && driver_installed) {
        transmission_stage = STAGE_RESET;
        // Only this task uses the driver; stop cancels stale queued output.
        twai_stop(); // BUS_OFF is already stopped and may reject this call.
        error = twai_driver_uninstall();
        if (error != ESP_OK) {
            transmission_failed(error, 0, 0);
            goto finished;
        }
        driver_installed = false;
        lock();
        can.flags &= ~CAN_INSTALLED;
        unlock();
    }
    transmission_stage = STAGE_GPIO;
    // Do this BEFORE driver install: gpio_set_direction() restores the generic
    // GPIO output route. TWAI install then reconnects its peripheral signal.
    // Explicit OE avoids depending on a previous UART/boot pin configuration.
    error = gpio_set_level(GPIO_NUM_16, 1); // Recessive TX level before routing.
    if (error == ESP_OK) {
        error = gpio_set_direction(GPIO_NUM_16, GPIO_MODE_OUTPUT);
    }
    if (error == ESP_OK) {
        error = gpio_set_direction(GPIO_NUM_17, GPIO_MODE_INPUT);
    }
    if (error == ESP_OK) {
        error = gpio_set_pull_mode(GPIO_NUM_17, GPIO_PULLUP_ONLY);
    }
    if (error != ESP_OK) {
        transmission_failed(error, 0, 0);
        goto finished;
    }
    twai_general_config_t general =
        TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_16, GPIO_NUM_17, TWAI_MODE_NORMAL);
    general.tx_queue_len = 4;
    general.rx_queue_len = 32;
    general.alerts_enabled = TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED | TWAI_ALERT_BUS_OFF |
                             TWAI_ALERT_BUS_ERROR | TWAI_ALERT_ERR_PASS |
                             TWAI_ALERT_ABOVE_ERR_WARN | TWAI_ALERT_RX_QUEUE_FULL |
                             TWAI_ALERT_RX_FIFO_OVERRUN | TWAI_ALERT_ARB_LOST;
    twai_timing_config_t timing = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    transmission_stage = STAGE_INSTALL;
    error = twai_driver_install(&general, &timing, &filter);
    if (error != ESP_OK) {
        transmission_failed(error, 0, 0);
        goto finished;
    }
    driver_installed = true;
    lock();
    can.flags |= CAN_INSTALLED;
    unlock();
    transmission_stage = STAGE_START;
    error = twai_start();
    if (error != ESP_OK) {
        transmission_failed(error, 0, 0);
        goto finished;
    }
    refresh_can_diagnostics();
    motor_can_diagnostics pins = controller_can_diagnostics();
    ESP_LOGI("motor", "CAN GPIO: TX OE=%u TWAI route=%u peripheral OE=%u RX input=%u RX level=%u",
             !!(pins.flags & 8), !!(pins.flags & 32), !!(pins.flags & 64), !!(pins.flags & 16),
             !!(pins.flags & 128));
    transmission_stage = STAGE_ZERO;
    if (!parameter(0x700A, 0)) {
        goto finished;
    }
    transmission_stage = STAGE_DISABLE;
    if (!enable(false) || !settle_motor()) {
        goto finished;
    }
    transmission_stage = STAGE_MODE;
    const uint8_t mode[8] = {5, 0x70, 0, 0, 2};
    if (!transmit(0x12, mode) || !settle_motor()) {
        goto finished;
    }
    transmission_stage = STAGE_CURRENT;
    if (!parameter(0x7018, 3) || !settle_motor()) {
        goto finished;
    }
    transmission_stage = STAGE_TORQUE;
    if (!parameter(0x7022, 20) || !settle_motor()) {
        goto finished;
    }
    transmission_stage = STAGE_REPORT;
    const uint8_t report[8] = {1, 2, 3, 4, 5, 6, 1, 0};
    ok = transmit(0x18, report);
finished:
    confirming_configuration = false;
    refresh_can_diagnostics();
    if (!ok && !controller_status().fault) {
        motor_can_diagnostics d = controller_can_diagnostics();
        char reason[96];
        snprintf(reason, sizeof(reason), "CAN config stage %lu failed: %s; see CAN Diagnostics",
                 (unsigned long)d.last_stage, esp_err_to_name((esp_err_t)d.last_error));
        fail(d.last_stage == STAGE_INSTALL || d.last_stage == STAGE_START ||
                     d.last_stage == STAGE_GPIO
                 ? 1
                 : 4,
             reason);
    }
    lock();
    can.flags &= ~CAN_CONFIGURING;
    if (ok && !state.fault) {
        can.flags |= CAN_CONFIGURED;
    }
    state.ready = ok && !state.fault;
    state.can_running = driver_installed && can.driver_state == TWAI_STATE_RUNNING;
    unlock();
    transmission_stage = STAGE_RUNTIME;
    return ok;
}

static void motor_task(void *argument)
{
    (void)argument;
    // The ESP32 can boot before motor/transceiver power stabilizes. Retry only
    // during this bounded startup window, always inhibited and at zero speed.
    int64_t boot_deadline = esp_timer_get_time() + 8000000;
    vTaskDelay(pdMS_TO_TICKS(500));
    bool configured = configure_motor(false);
    while (!configured && esp_timer_get_time() < boot_deadline) {
        controller_status_t status = controller_status();
        if (status.fault_code == 2 || status.fault_code == 3) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
        configured = configure_motor(true);
    }
    lock();
    boot_initializing = false;
    unlock();
    bool started = driver_installed;
    TickType_t wake = xTaskGetTickCount();
    bool awaiting_feedback = false;
    uint64_t enable_time = 0;
    while (true) {
        lock();
        bool retry = retry_requested;
        retry_requested = false;
        bool motor_fault_latched = state.fault && (state.fault_code == 2 || state.fault_code == 3);
        unlock();
        if (retry) {
            // A motor fault can arrive after serial accepted the retry request.
            if (!motor_fault_latched) {
                configure_motor(true);
            }
            started = driver_installed;
            awaiting_feedback = false;
            wake = xTaskGetTickCount();
            xSemaphoreGive(retry_done);
        }
        if (started) {
            receive_feedback();
            uint32_t alerts = 0;
            if (twai_read_alerts(&alerts, 0) == ESP_OK) {
                note_alerts(alerts);
            }
            refresh_can_diagnostics();
            twai_status_info_t info;
            bool healthy =
                twai_get_status_info(&info) == ESP_OK && info.state == TWAI_STATE_RUNNING;
            lock();
            state.can_running = healthy;
            bool fault = state.fault;
            unlock();
            if (!healthy && !fault) {
                fail(5, "TWAI not running / bus-off");
            }
            uint64_t now = esp_timer_get_time();
            emg_motor_output automatic = emg_control_output();
            lock();
            bool request_source = automatic.selected ? automatic.valid && automatic.detected &&
                                                           automatic.velocity != 0
                                                     : desired_enabled;
            float angle_velocity = 0;
            bool angle_selected =
                angle_output(feedback.position_rad, feedback.velocity_rad_s,
                             feedback.valid && now - feedback.time_us < 100000, &angle_velocity);
            if (angle_selected) {
                request_source = true;
            }
            bool requested = request_source && !forced_inhibit && !state.fault;
            bool fresh_feedback = feedback.valid && now - feedback.time_us < 100000;
            bool run = requested && fresh_feedback;
            float velocity = angle_selected       ? angle_velocity
                             : automatic.selected ? automatic.velocity
                                                  : desired_velocity;
            if (automatic.selected) {
                state.selected_speed = request_source ? velocity : 0;
            }
            bool was_running = state.running;
            const char *inhibit =
                state.fault                              ? state.reason
                : automatic.selected && !automatic.valid ? "Waiting for fresh EMG FFT"
                : !request_source
                    ? (automatic.selected ? "EMG below threshold" : "Motor is stopped")
                : !fresh_feedback ? "Waiting for motor feedback at zero velocity"
                                  : "";
            state.inhibit_code = state.fault ? 1 : !request_source ? 2 : !fresh_feedback ? 4 : 0;
            snprintf(state.inhibit_reason, sizeof(state.inhibit_reason), "%s", inhibit);
            unlock();
            bool ok = true;
            if (requested && !fresh_feedback && !was_running && !awaiting_feedback) {
                // Enable at ZERO first. Some motor firmware only sends feedback
                // in response to Enable; requiring it before Enable deadlocks.
                ok = parameter(0x700A, 0);
                ok = enable(true) && ok;
                awaiting_feedback = true;
                enable_time = now;
            }
            if (awaiting_feedback && !requested) {
                ok = parameter(0x700A, 0) && ok;
                ok = enable(false) && ok;
                awaiting_feedback = false;
            }
            if (awaiting_feedback && fresh_feedback) {
                awaiting_feedback = false;
            }
            if (awaiting_feedback && now - enable_time >= 250000) {
                fail(6, "Motor feedback not received after zero-speed enable");
                awaiting_feedback = false;
                run = false;
            }
            if (was_running && requested && !fresh_feedback) {
                fail(7, "Motor feedback timed out while running");
                run = false;
            }
            if (!run && was_running) {
                ok = parameter(0x700A, 0);
                ok = enable(false) && ok;
            } else if (run) {
                if (!was_running) {
                    ok = parameter(0x700A, 0);
                    ok = enable(true) && ok;
                }
                ok = parameter(0x700A, velocity) && ok;
            }
            if (!ok) {
                fail(8, "Motor command transmission failed");
            }
            lock();
            state.running = run && ok;
            state.commanded_speed = state.running ? velocity : 0;
            unlock();
        }
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(10));
    }
}
void controller_init(void)
{
    state_lock = xSemaphoreCreateMutex();
    retry_done = xSemaphoreCreateBinary();
    can.driver_state = UINT32_MAX;
    configASSERT(state_lock && retry_done);
    configASSERT(xTaskCreate(motor_task, "motor", 4096, NULL, 22, NULL) == pdPASS);
}
