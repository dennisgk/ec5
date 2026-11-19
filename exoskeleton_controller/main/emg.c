#include "emg.h"
#include "capture_bridge.h"
#include "controller.h"
#include "emg_signal.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "telemetry.h"
#include <string.h>

#define PROCESS_CAPACITY 128
typedef struct {
    board_config boards[2];
    bool reset, restart;
} configuration_job;

static TaskHandle_t processing_task_handle;
static QueueHandle_t configuration_queue;
static SemaphoreHandle_t configuration_done;
static uint8_t configuration_result;
static portMUX_TYPE state_lock = portMUX_INITIALIZER_UNLOCKED;
static board_config boards[2];
static processing_config processors[2];
static processor processing_states[2];
static integration_result integrations[2];
static fourier_result spectra[2];
static uint64_t integration_published_us[2], fourier_published_us[2];
static emg_sample process_ring[2][PROCESS_CAPACITY];
static uint32_t process_head[2], process_tail[2];
static uint32_t raw_overwrites[2], processing_overruns[2], max_drain_us;
static float scales[2];
static int control_board = -1;
static emg_detection_config detection_configs[2];
static struct {
    bool ready, detected;
    float peak, velocity;
    uint64_t published_us;
} detections[2];
static bool ignore_enabled;
static uint8_t ignore_count = 1, ignore_mask[17];
static ignore_range ignore_ranges[MAX_IGNORE_RANGES] = {{true, 50, 70}};

static board_config default_board(void)
{
    return (board_config){.ina_gain = 1, .adc_gain = 1, .high_pass = 200, .low_pass = 4500};
}

static processing_config default_processor(void)
{
    return (processing_config){.integration_ms = 100,
                               .hop = 128,
                               .window = 1,
                               .remove_mean = 1,
                               .notch_frequency = 600,
                               .notch_bandwidth = 20,
                               .notch_attenuation = -30};
}

// Caller holds state_lock. Processing owns its private working state and observes
// revisions before publishing. No command can modify buffers under an FFT.
static void invalidate(unsigned b)
{
    ++processors[b].revision;
    telemetry_changed();
    process_head[b] = process_tail[b] = 0;
    detections[b].ready = false;
    detections[b].published_us = 0;
    integrations[b].ready = false;
    spectra[b].ready = false;
    integration_published_us[b] = fourier_published_us[b] = 0;
}

static void clear_buffers(const configuration_job *job, uint32_t configured)
{
    portENTER_CRITICAL(&state_lock);
    for (unsigned b = 0; b < 2; ++b) {
        uint32_t revision = boards[b].revision + 1;
        boards[b] = job->boards[b];
        boards[b].revision = revision;
        boards[b].configured = (configured & (1u << b)) != 0;
        if (boards[b].configured) {
            int hp = emg_filter_code(boards[b].high_pass / 10.0f, 470e-9f);
            int lp = emg_filter_code(boards[b].low_pass / 10.0f, 10e-9f);
            boards[b].actual_hp = emg_filter_frequency((unsigned)hp, 470e-9f);
            boards[b].actual_lp = emg_filter_frequency((unsigned)lp, 10e-9f);
        }
        scales[b] = emg_mv_per_code(boards[b].adc_gain);
        if (job->reset) {
            detection_configs[b] =
                (emg_detection_config){.threshold = 0.001f,
                                       .maximum = 0.01f,
                                       .ignore_low = 50,
                                       .ignore_high = 70,
                                       .revision = detection_configs[b].revision + 1};
            uint32_t previous = processors[b].revision;
            processors[b] = default_processor();
            processors[b].revision = previous;
        }
        if (!boards[b].active) {
            processors[b].integration_enabled = false;
            processors[b].fourier_enabled = false;
        }
        invalidate(b);
    }
    portEXIT_CRITICAL(&state_lock);
    xTaskNotifyGive(processing_task_handle);
}

// Only the drain task performs mailbox operations or advances the shared tail.
// A configuration job therefore quiesces both sides without cross-core locks.
static uint8_t apply_configuration(const configuration_job *job)
{
    bool restarted = false;
    if (!capture_bridge_healthy()) {
        if (job->restart) {
            if (!capture_bridge_start()) {
                clear_buffers(job, 0);
                return STATUS_LP_FAULT;
            }
            restarted = true;
        } else if (job->reset || (!job->boards[0].active && !job->boards[1].active)) {
            clear_buffers(job, 0);
            return STATUS_OK;
        } else {
            return STATUS_LP_FAULT;
        }
    }
    uint32_t changed = restarted || job->reset ? 3 : 0;
    capture_board_request request[2];
    for (unsigned b = 0; b < 2; ++b) {
        board_config next = job->boards[b], old = emg_board(b);
        if (old.active != next.active || (next.active && !old.configured) ||
            old.ina_gain != next.ina_gain || old.adc_gain != next.adc_gain ||
            old.high_pass != next.high_pass || old.low_pass != next.low_pass) {
            changed |= 1u << b;
        }
        request[b] = (capture_board_request){
            .enabled = next.active,
            .ina_code = (uint32_t)emg_gain_code(next.ina_gain),
            .adc_code = (uint32_t)emg_gain_code(next.adc_gain),
            .hp_wiper = (uint32_t)emg_filter_code(next.high_pass / 10.0f, 470e-9f),
            .lp_wiper = (uint32_t)emg_filter_code(next.low_pass / 10.0f, 10e-9f),
        };
    }
    if (!changed) {
        return STATUS_OK;
    }
    if (!capture_bridge_pause()) {
        clear_buffers(job, 0);
        return STATUS_LP_FAULT;
    }
    capture_memory *memory = capture_bridge_memory();
    // LP acknowledged PAUSE and this task is the sole consumer. Old samples are
    // intentionally discarded at the settings boundary, never decoded with new gain.
    capture_store(&memory->read_index, capture_load(&memory->write_index));
    capture_store(&memory->generation, capture_load(&memory->generation) + 1);
    clear_buffers(job, 0);
    if (!capture_bridge_configure(request, changed)) {
        return STATUS_LP_FAULT;
    }
    clear_buffers(job, capture_load(&memory->configured_mask));
    if (!capture_bridge_resume()) {
        clear_buffers(job, 0);
        return STATUS_LP_FAULT;
    }
    return STATUS_OK;
}

static bool accept_record(const capture_record *record)
{
    unsigned b = (record->raw_board_flags >> CAPTURE_BOARD_SHIFT) & 1;
    emg_sample sample = {.time_us = capture_bridge_time_us(record),
                         .raw = capture_raw(record),
                         .sequence = record->sequence};
    telemetry_append(b, &sample, sizeof(sample), telemetry_epoch());
    portENTER_CRITICAL(&state_lock);
    bool processing = processors[b].integration_enabled || processors[b].fourier_enabled ||
                      processors[b].notch_enabled;
    if (processing) {
        if (process_head[b] - process_tail[b] == PROCESS_CAPACITY) {
            ++process_tail[b];
            ++processing_overruns[b];
        }
        process_ring[b][process_head[b]++ % PROCESS_CAPACITY] = sample;
    }
    portEXIT_CRITICAL(&state_lock);
    return processing;
}

static void drain_task(void *unused)
{
    (void)unused;
    TickType_t wake = xTaskGetTickCount();
    int64_t previous = esp_timer_get_time();
    bool fault_seen = false, was_running = false;
    while (true) {
        configuration_job job;
        if (xQueueReceive(configuration_queue, &job, 0) == pdTRUE) {
            configuration_result = apply_configuration(&job);
            xSemaphoreGive(configuration_done);
            previous = esp_timer_get_time();
        }
        capture_bridge_check_progress();
        capture_memory *memory = capture_bridge_memory();
        if (!capture_bridge_healthy()) {
            if (!fault_seen) {
                portENTER_CRITICAL(&state_lock);
                for (unsigned b = 0; b < 2; ++b) {
                    boards[b].configured = false;
                    invalidate(b);
                }
                portEXIT_CRITICAL(&state_lock);
                xTaskNotifyGive(processing_task_handle);
                fault_seen = true;
            }
            was_running = false;
        } else {
            fault_seen = false;
            bool running = capture_load(&memory->state) == CAPTURE_RUNNING;
            int64_t now = esp_timer_get_time();
            if (running && was_running) {
                uint32_t interval = (uint32_t)(now - previous);
                if (interval > capture_load(&max_drain_us)) {
                    capture_store(&max_drain_us, interval);
                }
            }
            was_running = running;
            previous = now;
            bool processing = false;
            for (unsigned n = 0; n < 128; ++n) {
                capture_record record;
                if (!capture_pop(memory, &record)) {
                    break;
                }
                processing |= accept_record(&record);
            }
            if (processing) {
                xTaskNotifyGive(processing_task_handle);
            }
        }
        vTaskDelayUntil(&wake, 1);
        // Avoid catch-up spinning after a cold configuration operation.
        if ((int32_t)(xTaskGetTickCount() - wake) > 2) {
            wake = xTaskGetTickCount();
        }
    }
}

static void processing_task(void *unused)
{
    (void)unused;
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        for (unsigned b = 0; b < 2; ++b) {
            emg_sample batch[16];
            unsigned count;
            do {
                portENTER_CRITICAL(&state_lock);
                processing_config config = processors[b];
                emg_detection_config detection_config = detection_configs[b];
                float scale = scales[b];
                count = 0;
                while (process_tail[b] != process_head[b] && count < 16) {
                    batch[count++] = process_ring[b][process_tail[b]++ % PROCESS_CAPACITY];
                }
                portEXIT_CRITICAL(&state_lock);
                processor *state = &processing_states[b];
                if (state->config.revision != config.revision) {
                    processing_reset(state, &config);
                }
                filtered_sample filtered[16];
                integration_record integrated[16];
                unsigned integrated_count = 0;
                uint32_t epoch = telemetry_epoch();
                uint32_t previous_frame = state->spectrum.frame;
                for (unsigned n = 0; n < count; ++n) {
                    float value = processing_push(state, batch[n].raw * scale, batch[n].time_us,
                                                  batch[n].sequence);
                    filtered[n] = (filtered_sample){batch[n].time_us, value, batch[n].sequence};
                    if (state->integrated.ready) {
                        integrated[integrated_count++] =
                            (integration_record){state->integrated.time_us,
                                                 state->integrated.integral,
                                                 state->integrated.envelope,
                                                 state->integrated.window_ms,
                                                 0,
                                                 batch[n].sequence};
                    }
                }
                bool new_frame = state->spectrum.ready && state->spectrum.frame != previous_frame;
                float peak =
                    new_frame ? emg_detection_peak(state->spectrum.power, &detection_config) : 0;
                controller_status_t motor = {0};
                if (new_frame) {
                    motor = controller_status();
                }
                bool publish = false;
                emg_detection_record decision = {0};
                portENTER_CRITICAL(&state_lock);
                if (processors[b].revision == config.revision) {
                    publish = epoch == telemetry_epoch();
                    if (!state->spectrum.ready) {
                        detections[b].ready = false;
                    }
                    if (new_frame && detection_configs[b].revision == detection_config.revision) {
                        bool detected = peak >= detection_config.threshold;
                        bool selected = control_board == (int)b;
                        float velocity = emg_detection_velocity(peak, &detection_config);
                        bool request =
                            selected && detected && velocity != 0 && motor.ready && !motor.fault;
                        detections[b].ready = true;
                        detections[b].detected = detected;
                        detections[b].peak = peak;
                        detections[b].velocity = velocity;
                        detections[b].published_us = (uint64_t)esp_timer_get_time();
                        decision = (emg_detection_record){
                            state->spectrum.time_us,
                            state->spectrum.frame,
                            peak,
                            (detected ? 1u : 0) | (selected ? 2u : 0) | (request ? 4u : 0),
                            request ? velocity : 0,
                        };
                    }
                    if (state->integrated.ready &&
                        (!integrations[b].ready ||
                         integrations[b].time_us != state->integrated.time_us)) {
                        integration_published_us[b] = (uint64_t)esp_timer_get_time();
                    }
                    integrations[b] = state->integrated;
                    if (spectra[b].time_us != state->spectrum.time_us ||
                        spectra[b].ready != state->spectrum.ready) {
                        spectra[b] = state->spectrum;
                        if (state->spectrum.ready) {
                            fourier_published_us[b] = (uint64_t)esp_timer_get_time();
                        }
                    }
                }
                portEXIT_CRITICAL(&state_lock);
                if (publish) {
                    if (config.notch_enabled) {
                        for (unsigned n = 0; n < count; ++n) {
                            telemetry_append(2 + b, &filtered[n], sizeof(filtered[n]), epoch);
                        }
                    }
                    for (unsigned n = 0; n < integrated_count; ++n) {
                        telemetry_append(4 + b, &integrated[n], sizeof(integrated[n]), epoch);
                    }
                    if (new_frame) {
                        spectrum_record spectrum = {state->spectrum.time_us,
                                                    state->spectrum.frame,
                                                    SAMPLE_RATE,
                                                    FFT_SIZE,
                                                    state->spectrum.hop,
                                                    state->spectrum.window,
                                                    state->spectrum.remove_mean,
                                                    {0}};
                        memcpy(spectrum.power, state->spectrum.power, sizeof(spectrum.power));
                        telemetry_append(6 + b, &spectrum, sizeof(spectrum), epoch);
                        if (decision.frame) {
                            telemetry_append(8 + b, &decision, sizeof(decision), epoch);
                            telemetry_flush(8 + b);
                        }
                    }
                }
                // Bound each board's work. A clearing ring can never make this
                // task monopolize the HP core or starve serial.
            } while (false);
        }
        portENTER_CRITICAL(&state_lock);
        bool pending = process_head[0] != process_tail[0] || process_head[1] != process_tail[1];
        portEXIT_CRITICAL(&state_lock);
        if (pending) {
            vTaskDelay(1);
            xTaskNotifyGive(processing_task_handle);
        }
    }
}

board_config emg_board(unsigned b)
{
    portENTER_CRITICAL(&state_lock);
    board_config config = boards[b];
    portEXIT_CRITICAL(&state_lock);
    return config;
}

static uint8_t submit(configuration_job *job)
{
    xQueueSend(configuration_queue, job, portMAX_DELAY);
    xSemaphoreTake(configuration_done, portMAX_DELAY);
    return configuration_result;
}

void emg_reset(void)
{
    emg_control_disable();
    portENTER_CRITICAL(&state_lock);
    ignore_enabled = false;
    ignore_count = 1;
    memset(ignore_mask, 0, sizeof(ignore_mask));
    memset(ignore_ranges, 0, sizeof(ignore_ranges));
    ignore_ranges[0] = (ignore_range){true, 50, 70};
    portEXIT_CRITICAL(&state_lock);
    capture_memory *memory = capture_bridge_memory();
    capture_store(&memory->resend_ms[0], 0);
    capture_store(&memory->resend_ms[1], 0);
    configuration_job job = {.boards = {default_board(), default_board()}, .reset = true};
    submit(&job);
}

uint8_t emg_resend_parameters(unsigned board, uint16_t milliseconds)
{
    if (board > 1 || (milliseconds && milliseconds < 100)) {
        return STATUS_INVALID;
    }
    if (milliseconds && !capture_bridge_healthy()) {
        return STATUS_LP_FAULT;
    }
    capture_store(&capture_bridge_memory()->resend_ms[board], milliseconds);
    return STATUS_OK;
}

uint8_t emg_activate(bool enabled)
{
    if (!enabled) {
        emg_control_disable();
        capture_store(&capture_bridge_memory()->resend_ms[0], 0);
        capture_store(&capture_bridge_memory()->resend_ms[1], 0);
    }
    configuration_job job = {.boards = {emg_board(0), emg_board(1)}};
    job.boards[0].active = job.boards[1].active = enabled;
    job.restart = enabled;
    uint8_t result = submit(&job);
    if (result != STATUS_OK) {
        return result;
    }
    unsigned failed = 0;
    for (unsigned b = 0; b < 2; ++b) {
        if (enabled && !emg_board(b).configured) {
            failed |= 1u << b;
        }
    }
    return failed ? (uint8_t)(STATUS_HARDWARE_0 + failed - 1) : STATUS_OK;
}

uint8_t emg_adjust(unsigned b, unsigned property, uint16_t value, float *actual)
{
    if (b > 1 || property > EMG_PROPERTY_LOW_PASS) {
        return STATUS_INVALID;
    }
    configuration_job job = {.boards = {emg_board(0), emg_board(1)}};
    board_config *config = &job.boards[b];
    if (property <= EMG_PROPERTY_ADC_PGA_GAIN) {
        if (emg_gain_code(value) < 0) {
            return STATUS_INVALID;
        }
        if (property == EMG_PROPERTY_AD8231_GAIN) {
            config->ina_gain = (uint8_t)value;
        } else {
            config->adc_gain = (uint8_t)value;
        }
        *actual = value;
    } else {
        if ((property == EMG_PROPERTY_HIGH_PASS && (value < 33 || value > 709)) ||
            (property == EMG_PROPERTY_LOW_PASS && (value < 1520 || value > 4500))) {
            return STATUS_INVALID;
        }
        if (property == EMG_PROPERTY_HIGH_PASS) {
            config->high_pass = value;
        } else {
            config->low_pass = value;
        }
        float capacitance = property == EMG_PROPERTY_HIGH_PASS ? 470e-9f : 10e-9f;
        *actual = emg_filter_frequency((unsigned)emg_filter_code(value / 10.0f, capacitance),
                                       capacitance);
    }
    uint8_t result = submit(&job);
    if (result != STATUS_OK) {
        return result;
    }
    board_config applied = emg_board(b);
    return applied.active && !applied.configured ? (uint8_t)(STATUS_HARDWARE_0 + b) : STATUS_OK;
}

uint8_t emg_processing_enable(bool fourier, bool enabled)
{
    if (fourier && !enabled) {
        emg_control_disable();
    }
    if (enabled && !capture_bridge_healthy()) {
        return STATUS_LP_FAULT;
    }
    portENTER_CRITICAL(&state_lock);
    if (enabled && (!boards[0].active || !boards[1].active)) {
        portEXIT_CRITICAL(&state_lock);
        return STATUS_NOT_READY;
    }
    for (unsigned b = 0; b < 2; ++b) {
        if (fourier) {
            processors[b].fourier_enabled = enabled;
        } else {
            processors[b].integration_enabled = enabled;
        }
        invalidate(b);
    }
    portEXIT_CRITICAL(&state_lock);
    xTaskNotifyGive(processing_task_handle);
    return STATUS_OK;
}

int emg_control_board(void)
{
    portENTER_CRITICAL(&state_lock);
    int board = control_board;
    portEXIT_CRITICAL(&state_lock);
    return board;
}

void emg_control_disable(void)
{
    portENTER_CRITICAL(&state_lock);
    control_board = -1;
    portEXIT_CRITICAL(&state_lock);
    controller_request_output(false, 0);
}

uint8_t emg_control_adjust(unsigned b, bool enabled, const emg_detection_config *c)
{
    if (b > 1 || !isfinite(c->threshold) || c->threshold <= 0 || !isfinite(c->maximum) ||
        c->maximum <= c->threshold || c->maximum_velocity < -500 || c->maximum_velocity > 500 ||
        c->velocity < -500 || c->velocity > 500 || !isfinite(c->ignore_low) ||
        !isfinite(c->ignore_high) || c->ignore_low < 0 || c->ignore_high > 500 ||
        c->ignore_low > c->ignore_high) {
        return STATUS_INVALID;
    }
    controller_status_t motor = controller_status();
    if (enabled && (!motor.ready || motor.fault || !motor.can_running)) {
        return STATUS_MOTOR_FAULT;
    }
    portENTER_CRITICAL(&state_lock);
    if (enabled && (!boards[b].active || !boards[b].configured || !processors[b].fourier_enabled)) {
        portEXIT_CRITICAL(&state_lock);
        return STATUS_NOT_READY;
    }
    uint32_t revision = detection_configs[b].revision + 1;
    detection_configs[b] = *c;
    detection_configs[b].ignore_enabled = false;
    memcpy(detection_configs[b].excluded, ignore_mask, 17);
    telemetry_changed();
    detection_configs[b].revision = revision;
    detections[b].ready = false;
    if (enabled) {
        control_board = (int)b;
    } else if (control_board == (int)b) {
        control_board = -1;
    }
    portEXIT_CRITICAL(&state_lock);
    controller_request_output(false, 0); // Never resume an earlier manual command.
    if (enabled) {
        controller_inhibit(false);
    }
    return STATUS_OK;
}

emg_motor_output emg_control_output(void)
{
    emg_motor_output output = {0};
    uint64_t now = (uint64_t)esp_timer_get_time();
    portENTER_CRITICAL(&state_lock);
    int b = control_board;
    if (b >= 0) {
        output.selected = true;
        output.valid = boards[b].active && boards[b].configured && processors[b].fourier_enabled &&
                       processing_result_fresh(detections[b].ready, now, detections[b].published_us,
                                               (uint64_t)(processors[b].hop + 100) * 1000);
        output.detected = detections[b].detected;
        output.velocity = detections[b].velocity;
    }
    portEXIT_CRITICAL(&state_lock);
    return output;
}

bool emg_notch_enabled(unsigned b)
{
    portENTER_CRITICAL(&state_lock);
    bool enabled = processors[b].notch_enabled;
    portEXIT_CRITICAL(&state_lock);
    return enabled;
}

uint8_t emg_notch_adjust(unsigned b, uint8_t enabled, uint16_t frequency, uint16_t bandwidth,
                         int16_t attenuation)
{
    // Center 1–499 Hz, width 0.1–100 Hz. Keep its nominal edges inside Nyquist.
    if (b > 1 || enabled > 1 || frequency < 10 || frequency > 4990 || bandwidth < 1 ||
        bandwidth > 1000 || bandwidth >= 2 * frequency ||
        bandwidth >= 2 * (SAMPLE_RATE * 5 - frequency) || attenuation < -600 || attenuation > -1) {
        return STATUS_INVALID;
    }
    portENTER_CRITICAL(&state_lock);
    processors[b].notch_enabled = enabled != 0;
    processors[b].notch_frequency = frequency;
    processors[b].notch_bandwidth = bandwidth;
    processors[b].notch_attenuation = attenuation;
    processors[b].notch_count = 1;
    processors[b].notches[0] =
        (notch_filter_config){enabled != 0, frequency, bandwidth, attenuation};
    invalidate(b);
    portEXIT_CRITICAL(&state_lock);
    xTaskNotifyGive(processing_task_handle);
    return STATUS_OK;
}

uint8_t emg_ignore_adjust(bool enabled, unsigned count, const ignore_range *ranges)
{
    if (count > MAX_IGNORE_RANGES) {
        return STATUS_INVALID;
    }
    uint8_t mask[17] = {0};
    for (unsigned n = 0; n < count; ++n) {
        if (!isfinite(ranges[n].low) || !isfinite(ranges[n].high) || ranges[n].low < 0 ||
            ranges[n].high > 500 || ranges[n].low > ranges[n].high) {
            return STATUS_INVALID;
        }
        if (enabled && ranges[n].enabled) {
            for (unsigned bin = 0; bin < FFT_BINS; ++bin) {
                float hz = bin * (SAMPLE_RATE / (float)FFT_SIZE);
                if (hz >= ranges[n].low && hz <= ranges[n].high) {
                    mask[bin / 8] |= 1u << (bin % 8);
                }
            }
        }
    }
    portENTER_CRITICAL(&state_lock);
    ignore_enabled = enabled;
    ignore_count = (uint8_t)count;
    memcpy(ignore_ranges, ranges, count * sizeof(*ranges));
    memcpy(ignore_mask, mask, 17);
    for (unsigned b = 0; b < 2; ++b) {
        detection_configs[b].ignore_enabled = false;
        memcpy(detection_configs[b].excluded, mask, 17);
        ++detection_configs[b].revision;
        detections[b].ready = false;
    }
    telemetry_changed();
    portEXIT_CRITICAL(&state_lock);
    return STATUS_OK;
}

uint8_t emg_notch_bank(unsigned b, unsigned count, const notch_filter_config *filters)
{
    if (b > 1 || count > MAX_NOTCH_FILTERS) {
        return STATUS_INVALID;
    }
    bool enabled = false;
    for (unsigned n = 0; n < count; ++n) {
        const notch_filter_config *f = &filters[n];
        if (f->frequency < 10 || f->frequency > 4990 || f->bandwidth < 1 || f->bandwidth > 1000 ||
            f->bandwidth >= 2 * f->frequency || f->bandwidth >= 2 * (5000 - f->frequency) ||
            f->attenuation < -600 || f->attenuation > -1) {
            return STATUS_INVALID;
        }
        enabled |= f->enabled;
    }
    portENTER_CRITICAL(&state_lock);
    processors[b].notch_count = (uint8_t)count;
    processors[b].notch_enabled = enabled;
    memcpy(processors[b].notches, filters, count * sizeof(*filters));
    invalidate(b);
    portEXIT_CRITICAL(&state_lock);
    xTaskNotifyGive(processing_task_handle);
    return STATUS_OK;
}

void emg_configuration(device_configuration *out)
{
    portENTER_CRITICAL(&state_lock);
    memcpy(out->boards, boards, sizeof(boards));
    memcpy(out->processing, processors, sizeof(processors));
    memcpy(out->curves, detection_configs, sizeof(detection_configs));
    out->owner = control_board;
    out->ignore_enabled = ignore_enabled;
    out->ignore_count = ignore_count;
    memcpy(out->ranges, ignore_ranges, sizeof(ignore_ranges));
    memcpy(out->mask, ignore_mask, 17);
    portEXIT_CRITICAL(&state_lock);
    for (unsigned b = 0; b < 2; ++b) {
        out->resend_ms[b] = (uint16_t)capture_load(&capture_bridge_memory()->resend_ms[b]);
    }
}

uint8_t emg_integration_adjust(unsigned b, uint16_t milliseconds)
{
    if (b > 1 || milliseconds < 10 || milliseconds > 1000) {
        return STATUS_INVALID;
    }
    portENTER_CRITICAL(&state_lock);
    processors[b].integration_ms = milliseconds;
    invalidate(b);
    portEXIT_CRITICAL(&state_lock);
    xTaskNotifyGive(processing_task_handle);
    return STATUS_OK;
}

uint8_t emg_fourier_adjust(unsigned b, uint16_t hop, uint8_t window, uint8_t remove_mean)
{
    if (b > 1 || (hop != 64 && hop != 128 && hop != 256) || window > 1 || remove_mean > 1) {
        return STATUS_INVALID;
    }
    portENTER_CRITICAL(&state_lock);
    processors[b].hop = hop;
    processors[b].window = window;
    processors[b].remove_mean = remove_mean;
    invalidate(b);
    portEXIT_CRITICAL(&state_lock);
    xTaskNotifyGive(processing_task_handle);
    return STATUS_OK;
}

integration_result emg_integration(unsigned b)
{
    portENTER_CRITICAL(&state_lock);
    integration_result result = integrations[b];
    uint64_t published_us = integration_published_us[b];
    portEXIT_CRITICAL(&state_lock);
    result.ready =
        processing_result_fresh(result.ready, (uint64_t)esp_timer_get_time(), published_us, 50000);
    return result;
}

fourier_result emg_fourier(unsigned b)
{
    portENTER_CRITICAL(&state_lock);
    fourier_result result = spectra[b];
    uint64_t published_us = fourier_published_us[b];
    portEXIT_CRITICAL(&state_lock);
    result.ready = processing_result_fresh(result.ready, (uint64_t)esp_timer_get_time(),
                                           published_us, (uint64_t)(result.hop + 50) * 1000);
    return result;
}

void emg_init(void)
{
    for (unsigned b = 0; b < 2; ++b) {
        boards[b] = default_board();
        processors[b] = default_processor();
        processors[b].revision = 1;
        scales[b] = emg_mv_per_code(1);
        detection_configs[b] = (emg_detection_config){
            .threshold = 0.001f, .maximum = 0.01f, .ignore_low = 50, .ignore_high = 70};
    }
    configuration_queue = xQueueCreate(1, sizeof(configuration_job));
    configuration_done = xSemaphoreCreateBinary();
    configASSERT(configuration_queue && configuration_done);
    // A failed LP boot leaves serial and motor Stop available; explicit acquisition
    // enable can retry the LP setup. No HP acquisition fallback exists.
    capture_bridge_start();
    configASSERT(xTaskCreate(processing_task, "emg_processing", 6144, NULL, 10,
                             &processing_task_handle) == pdPASS);
    configASSERT(xTaskCreate(drain_task, "capture_drain", 4096, NULL, 21, NULL) == pdPASS);
}

acquisition_diagnostics emg_diagnostics(void)
{
    capture_memory *m = capture_bridge_memory();
    acquisition_diagnostics out = {0};
    out.state = capture_load(&m->state);
    out.fault = capture_bridge_fault();
    out.generation = capture_load(&m->generation);
    out.capacity = CAPTURE_CAPACITY;
    uint32_t tail = capture_load(&m->read_index);
    uint32_t used = capture_load(&m->write_index) - tail;
    out.occupancy = used < CAPTURE_CAPACITY ? used : CAPTURE_CAPACITY;
    out.high_water = capture_load(&m->high_water);
    out.configured_mask = capture_load(&m->configured_mask);
    out.heartbeat = capture_load(&m->heartbeat);
    uint32_t period = m->period_q19;
    for (unsigned b = 0; b < 2; ++b) {
        out.conversions[b] = capture_load(&m->conversions[b]);
        out.adc_gaps[b] = capture_load(&m->adc_gap_events[b]);
        out.read_errors[b] = capture_load(&m->read_errors[b]);
        out.published[b] = capture_load(&m->published[b]);
        out.discarded[b] = capture_load(&m->discarded[b]);
        out.max_read_us[b] =
            (uint32_t)capture_ticks_to_us(capture_load(&m->max_read_ticks[b]), period);
        portENTER_CRITICAL(&state_lock);
        out.raw_overwrites[b] = raw_overwrites[b];
        out.processing_overruns[b] = processing_overruns[b];
        portEXIT_CRITICAL(&state_lock);
    }
    out.late_passes = capture_load(&m->late_passes);
    out.max_pass_us = (uint32_t)capture_ticks_to_us(capture_load(&m->max_pass_ticks), period);
    out.max_drain_us = capture_load(&max_drain_us);
    return out;
}
