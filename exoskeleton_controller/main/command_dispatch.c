#include "autoconfig.h"
#include "ble_control.h"
#include "capture_bridge.h"
#include "control.h"
#include "controller.h"
#include "emg.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "serial_frame.h"
#include "serial_transport.h"
#include "telemetry.h"
#include <string.h>

static uint8_t reply[1024];
static const uint8_t *input;
static size_t remaining, response_size;

static bool take(void *out, size_t count)
{
    if (count > remaining) {
        return false;
    }
    memcpy(out, input, count);
    input += count;
    remaining -= count;
    return true;
}
static bool finish(const void *data, size_t count)
{
    (void)data;
    response_size = count;
    return true;
}

static bool handle(uint8_t type, bool *initialized)
{
    uint8_t payload[160] = {0};
    size_t size;
    switch (type) {
    case CMD_ANGLE_CONFIG:
        size = 32;
        break;
    case CMD_AUTOCONFIG_BEGIN:
        size = 2;
        break;
    case CMD_AUTOCONFIG_WRITE:
        size = remaining;
        if (size < 3 || size > sizeof(payload)) {
            return false;
        }
        break;
    case CMD_AUTOCONFIG_COMMIT:
    case CMD_AUTOCONFIG_DELETE:
    case CMD_CONTROL_STATE:
        size = 0;
        break;
    case CMD_BLUETOOTH:
        size = 1;
        break;
    case CMD_INIT:
        size = 1;
        break;
    case CMD_MOTOR:
        size = 2;
        break;
    case CMD_ACTIVATE_EMGS:
    case CMD_ACTIVATE_EMGS_INTEGRATION:
    case CMD_ACTIVATE_EMGS_FOURIER:
    case CMD_QUERY_EMG:
    case CMD_QUERY_FOURIER:
    case CMD_QUERY_INTEGRATION:
    case CMD_QUERY_FILTERED_EMG:
    case CMD_QUERY_EMG_DETECTION:
        size = 1;
        break;
    case CMD_ADJUST_EMG:
        if (!take(payload, 2) || payload[1] > EMG_PROPERTY_LOW_PASS) {
            return false;
        }
        size = payload[1] <= EMG_PROPERTY_ADC_PGA_GAIN ? 1 : 2;
        if (!take(payload + 2, size)) {
            return false;
        }
        size = 0;
        break;
    case CMD_ADJUST_EMGS_INTEGRATION:
    case CMD_RESEND_EMG_PARAMETERS:
        size = 3;
        break;
    case CMD_ADJUST_EMGS_FOURIER:
        size = 5;
        break;
    case CMD_ADJUST_NOTCH:
        size = 6;
        break;
    case CMD_ADJUST_NOTCH_ATTENUATION:
        size = 8;
        break;
    case CMD_CONFIGURE_EMG_CONTROL:
        size = 17;
        break;
    case CMD_SET_IGNORE_RANGES:
    case CMD_SET_NOTCH_BANK:
        if (!take(payload, 2)) {
            return false;
        }
        if (payload[1] > (type == CMD_SET_IGNORE_RANGES ? MAX_IGNORE_RANGES : MAX_NOTCH_FILTERS)) {
            return false;
        }
        size = payload[1] * (type == CMD_SET_IGNORE_RANGES ? 9u : 7u);
        if (size && !take(payload + 2, size)) {
            return false;
        }
        size = 0;
        break;
    case CMD_SET_MOTOR_CURVE:
        size = 14;
        break;
    case CMD_SUBSCRIBE:
        size = 2;
        break;
    case CMD_GET_CONFIGURATION:
    case CMD_QUERY_EMG_HARDWARE:
        size = 0;
        break;
    case CMD_QUERY_ACQUISITION:
    case CMD_QUERY_MOTOR:
    case CMD_QUERY_MOTOR_CAN:
    case CMD_RETRY_MOTOR_CAN:
        size = 0;
        break;
    default:
        ESP_LOGW("serial", "Unknown operation %u", type);
        reply[0] = type;
        reply[1] = STATUS_INVALID;
        finish(reply, 2);
        return false;
    }
    if (size && !take(payload, size)) {
        return false;
    }
    if (remaining) {
        reply[0] = type;
        reply[1] = STATUS_INVALID;
        return finish(reply, 2);
    }
    reply[0] = type;
    reply[1] = STATUS_OK;
    uint8_t *out = reply + 2;
    if (!*initialized && type != CMD_INIT) {
        reply[1] = STATUS_NOT_INITIALIZED;
        return finish(reply, 2);
    }
    unsigned board = payload[0];
    if ((type == CMD_ADJUST_EMG || type == CMD_ADJUST_EMGS_INTEGRATION ||
         type == CMD_ADJUST_EMGS_FOURIER || type == CMD_RESEND_EMG_PARAMETERS ||
         type == CMD_ADJUST_NOTCH || type == CMD_ADJUST_NOTCH_ATTENUATION ||
         type == CMD_SET_NOTCH_BANK || type == CMD_SET_MOTOR_CURVE) &&
        board > 1) {
        reply[1] = STATUS_INVALID;
        return finish(reply, 2);
    }
    if (type >= CMD_QUERY_EMG && type <= CMD_QUERY_INTEGRATION && !capture_bridge_healthy()) {
        reply[1] = STATUS_LP_FAULT;
        return finish(reply, 2);
    }
    switch (type) {
    case CMD_ANGLE_CONFIG: {
        angle_config a = {read_float(payload),      read_float(payload + 4),
                          read_float(payload + 8),  read_float(payload + 12),
                          read_float(payload + 16), read_float(payload + 20),
                          read_float(payload + 24), read_float(payload + 28)};
        reply[1] = angle_configure(&a);
        break;
    }
    case CMD_AUTOCONFIG_BEGIN:
        reply[1] = autoconfig_begin(read_u16(payload));
        break;
    case CMD_AUTOCONFIG_WRITE:
        reply[1] = autoconfig_write(read_u16(payload), payload + 2, size - 2);
        break;
    case CMD_AUTOCONFIG_COMMIT:
        reply[1] = autoconfig_commit();
        break;
    case CMD_AUTOCONFIG_DELETE:
        reply[1] = autoconfig_delete();
        break;
    case CMD_BLUETOOTH:
        reply[1] = payload[0] <= 1 ? control_handoff(payload[0] != 0) : STATUS_INVALID;
        break;
    case CMD_CONTROL_STATE: {
        angle_state a = angle_snapshot();
        controller_status_t motor = controller_status();
        bool automatic = emg_control_board() >= 0;
        *out++ = control_bluetooth();
        *out++ = a.enabled;
        *out++ = a.calibrated;
        *out++ = a.enabled ? 3 : automatic ? 2 : motor.selected_speed != 0 ? 1 : 0;
        write_float(&out, a.enabled || automatic ? 0 : motor.selected_speed);
        const float fields[] = {a.config.ratio,        a.config.direction, a.config.minimum,
                                a.config.maximum,      a.config.gain,      a.config.speed,
                                a.config.acceleration, a.config.deadband};
        for (unsigned i = 0; i < 8; ++i) {
            write_float(&out, fields[i]);
        }
        uint32_t words[8];
        ble_control_diagnostics(words);
        for (unsigned i = 0; i < 8; ++i) {
            write_u32(&out, words[i]);
        }
        write_u32(&out, autoconfig_error());
        break;
    }
    case CMD_INIT:
        if (*initialized || payload[0] < 1 || payload[0] > 16) {
            reply[1] = STATUS_VERSION;
            finish(reply, 2);
            return false;
        }
        serial_set_peer_window(payload[0]);
        *initialized = true;
        write_u16(&out, PROTOCOL_VERSION);
        write_u16(&out, SAMPLE_RATE);
        write_u16(&out, FFT_SIZE);
        write_u16(&out, RAW_CAPACITY);
        write_u32(&out, telemetry_session());
        *out++ = TELEMETRY_STREAMS;
        for (unsigned n = 0; n < TELEMETRY_STREAMS; ++n) {
            write_u16(&out, telemetry_sizes[n]);
        }
        *out++ = CONFIG_EC5_SERIAL_WINDOW;
        write_u16(&out, SERIAL_ASSEMBLY_MAX);
        break;
    case CMD_MOTOR: {
        uint16_t bits = read_u16(payload);
        int32_t velocity = bits <= INT16_MAX ? bits : (int32_t)bits - 65536;
        controller_status_t motor = controller_status();
        if (velocity < -500 || velocity > 500) {
            reply[1] = STATUS_INVALID;
        } else if (velocity && (control_bluetooth() || angle_snapshot().enabled)) {
            reply[1] = STATUS_CONTROL_OWNER;
        } else if (velocity && emg_control_board() >= 0) {
            reply[1] = STATUS_EMG_CONTROL;
        } else if (velocity && (!motor.ready || motor.fault || !motor.can_running)) {
            reply[1] = STATUS_MOTOR_FAULT;
        } else {
            if (!velocity) {
                control_stop();
            }
            if (velocity) {
                controller_inhibit(false);
            }
            controller_request_output(velocity != 0, velocity / 10.0f);
        }
        break;
    }
    case CMD_ACTIVATE_EMGS:
    case CMD_ACTIVATE_EMGS_INTEGRATION:
    case CMD_ACTIVATE_EMGS_FOURIER:
        if (payload[0] > 1) {
            reply[1] = STATUS_INVALID;
        } else if (type == CMD_ACTIVATE_EMGS) {
            reply[1] = emg_activate(payload[0] != 0);
        } else {
            reply[1] = emg_processing_enable(type == CMD_ACTIVATE_EMGS_FOURIER, payload[0] != 0);
        }
        break;
    case CMD_ADJUST_EMG: {
        float actual = 0;
        uint16_t value =
            payload[1] <= EMG_PROPERTY_ADC_PGA_GAIN ? payload[2] : read_u16(payload + 2);
        reply[1] = emg_adjust(board, payload[1], value, &actual);
        if (!reply[1]) {
            write_u16(&out, value);
            write_float(&out, actual);
        }
        break;
    }
    case CMD_ADJUST_EMGS_INTEGRATION:
        reply[1] = emg_integration_adjust(board, read_u16(payload + 1));
        break;
    case CMD_RESEND_EMG_PARAMETERS:
        reply[1] = emg_resend_parameters(board, read_u16(payload + 1));
        break;
    case CMD_ADJUST_NOTCH:
    case CMD_ADJUST_NOTCH_ATTENUATION: {
        uint16_t bits = read_u16(payload + 6);
        int16_t attenuation =
            type == CMD_ADJUST_NOTCH
                ? -30
                : (int16_t)(bits <= INT16_MAX ? (int32_t)bits : (int32_t)bits - 65536);
        reply[1] = emg_notch_adjust(board, payload[1], read_u16(payload + 2), read_u16(payload + 4),
                                    attenuation);
        break;
    }
    case CMD_ADJUST_EMGS_FOURIER:
        reply[1] = emg_fourier_adjust(board, read_u16(payload + 1), payload[3], payload[4]);
        break;
    case CMD_SET_IGNORE_RANGES: {
        ignore_range ranges[MAX_IGNORE_RANGES] = {0};
        for (unsigned n = 0; n < payload[1]; ++n) {
            const uint8_t *row = payload + 2 + n * 9;
            ranges[n] = (ignore_range){row[0] != 0, read_float(row + 1), read_float(row + 5)};
            if (row[0] > 1) {
                reply[1] = STATUS_INVALID;
            }
        }
        if (!reply[1]) {
            reply[1] = emg_ignore_adjust(payload[0] != 0, payload[1], ranges);
        }
        break;
    }
    case CMD_SET_NOTCH_BANK: {
        notch_filter_config filters[MAX_NOTCH_FILTERS] = {0};
        for (unsigned n = 0; n < payload[1]; ++n) {
            const uint8_t *row = payload + 2 + n * 7;
            filters[n] = (notch_filter_config){row[0] != 0, read_u16(row + 1), read_u16(row + 3),
                                               (int16_t)read_u16(row + 5)};
            if (row[0] > 1) {
                reply[1] = STATUS_INVALID;
            }
        }
        if (!reply[1]) {
            reply[1] = emg_notch_bank(board, payload[1], filters);
        }
        break;
    }
    case CMD_SET_MOTOR_CURVE: {
        emg_detection_config curve = {.threshold = read_float(payload + 2),
                                      .maximum = read_float(payload + 6),
                                      .velocity = (int16_t)read_u16(payload + 10),
                                      .maximum_velocity = (int16_t)read_u16(payload + 12)};
        reply[1] = payload[1] > 1 ? STATUS_INVALID
                   : payload[1] && (control_bluetooth() || angle_snapshot().enabled)
                       ? STATUS_CONTROL_OWNER
                       : emg_control_adjust(board, payload[1] != 0, &curve);
        break;
    }
    case CMD_SUBSCRIBE:
        reply[1] = telemetry_subscribe(read_u16(payload)) ? STATUS_OK : STATUS_INVALID;
        write_u32(&out, telemetry_session());
        write_u32(&out, telemetry_epoch());
        break;
    case CMD_GET_CONFIGURATION: {
        device_configuration c;
        emg_configuration(&c);
        uint8_t *length = out;
        out += 2;
        write_u32(&out, telemetry_epoch());
        *out++ = c.boards[0].active || c.boards[1].active;
        *out++ = c.processing[0].integration_enabled;
        *out++ = c.processing[0].fourier_enabled;
        *out++ = c.owner < 0 ? 255 : (uint8_t)c.owner;
        *out++ = c.ignore_enabled;
        *out++ = c.ignore_count;
        for (unsigned n = 0; n < c.ignore_count; ++n) {
            *out++ = c.ranges[n].enabled;
            write_float(&out, c.ranges[n].low);
            write_float(&out, c.ranges[n].high);
        }
        memcpy(out, c.mask, sizeof(c.mask));
        out += sizeof(c.mask);
        for (unsigned b = 0; b < 2; ++b) {
            *out++ = c.boards[b].ina_gain;
            *out++ = c.boards[b].adc_gain;
            write_u16(&out, c.boards[b].high_pass);
            write_u16(&out, c.boards[b].low_pass);
            write_u16(&out, c.resend_ms[b]);
            write_u16(&out, c.processing[b].integration_ms);
            write_u16(&out, c.processing[b].hop);
            *out++ = c.processing[b].window;
            *out++ = c.processing[b].remove_mean;
            *out++ = c.processing[b].notch_count;
            for (unsigned n = 0; n < c.processing[b].notch_count; ++n) {
                notch_filter_config f = c.processing[b].notches[n];
                *out++ = f.enabled;
                write_u16(&out, f.frequency);
                write_u16(&out, f.bandwidth);
                write_u16(&out, (uint16_t)f.attenuation);
            }
            write_float(&out, c.curves[b].threshold);
            write_float(&out, c.curves[b].maximum);
            write_u16(&out, (uint16_t)c.curves[b].velocity);
            write_u16(&out, (uint16_t)c.curves[b].maximum_velocity);
        }
        // Availability is runtime state, independent of requested acquisition.
        *out++ = (c.boards[0].configured ? 1u : 0u) | (c.boards[1].configured ? 2u : 0u);
        write_u16(&length, (uint16_t)(out - length - 2));
        break;
    }
    case CMD_QUERY_EMG_HARDWARE: {
        capture_memory *memory = capture_bridge_memory();
        for (unsigned b = 0; b < 2; ++b) {
            for (unsigned n = 0; n < 6; ++n) {
                write_u32(&out, capture_load(&memory->configuration_error[b][n]));
            }
        }
        break;
    }
    case CMD_RETRY_MOTOR_CAN:
        emg_control_disable();
        reply[1] = controller_retry_configuration() ? STATUS_OK : STATUS_MOTOR_FAULT;
        break;
    case CMD_CONFIGURE_EMG_CONTROL:
    case CMD_QUERY_EMG_DETECTION:
    case CMD_QUERY_EMG:
    case CMD_QUERY_FILTERED_EMG:
    case CMD_QUERY_INTEGRATION:
    case CMD_QUERY_FOURIER:
        reply[1] = STATUS_INVALID; // Protocol 3 plot data is subscription-only serial.
        break;
    case CMD_QUERY_MOTOR_CAN: {
        motor_can_diagnostics d = controller_can_diagnostics();
        const uint32_t words[] = {
            d.flags,
            d.driver_state,
            d.tx_pending,
            d.rx_pending,
            d.tx_errors,
            d.rx_errors,
            d.tx_failed,
            d.rx_missed,
            d.rx_overrun,
            d.arbitration_lost,
            d.bus_errors,
            d.alerts,
            d.tx_queued,
            d.tx_confirmed,
            d.tx_rejected,
            d.rx_frames,
            d.rx_motor_frames,
            d.last_error,
            d.last_stage,
            d.last_id,
            d.last_parameter,
            d.failure_state,
            d.failure_tx_errors,
            d.failure_rx_errors,
            d.failure_bus_errors,
            d.motor_faults,
            d.motor_warnings,
            d.last_rx_id,
            d.feedback_faults,
            d.feedback_faults_seen,
            d.feedback_mode,
            d.fault_frames,
            d.motor_faults_seen,
            d.motor_warnings_seen,
            d.temperature_tenths,
            d.feedback_frames,
        };
        for (unsigned n = 0; n < sizeof(words) / sizeof(words[0]); ++n) {
            write_u32(&out, words[n]);
        }
        write_u64(&out, d.failure_time_us);
        break;
    }
    case CMD_QUERY_MOTOR: {
        controller_status_t motor = controller_status();
        motor_feedback feedback = controller_feedback();
        write_u32(&out, (motor.ready ? 1u : 0u) | (motor.can_running ? 2u : 0u) |
                            (motor.running ? 4u : 0u) | (motor.fault ? 8u : 0u));
        write_u32(&out, motor.fault_code);
        write_u32(&out, motor.inhibit_code);
        write_float(&out, motor.selected_speed);
        write_float(&out, motor.commanded_speed);
        write_float(&out, feedback.position_rad);
        write_float(&out, feedback.velocity_rad_s);
        write_u64(&out, feedback.time_us);
        memcpy(out, motor.reason, sizeof(motor.reason));
        out += sizeof(motor.reason);
        memcpy(out, motor.inhibit_reason, sizeof(motor.inhibit_reason));
        out += sizeof(motor.inhibit_reason);
        break;
    }
    case CMD_QUERY_ACQUISITION: {
        acquisition_diagnostics d = emg_diagnostics();
        write_u32(&out, d.state);
        write_u32(&out, d.fault);
        write_u32(&out, d.generation);
        write_u32(&out, d.capacity);
        write_u32(&out, d.occupancy);
        write_u32(&out, d.high_water);
        write_u32(&out, d.configured_mask);
        write_u32(&out, d.heartbeat);
        for (unsigned b = 0; b < 2; ++b) {
            write_u32(&out, d.conversions[b]);
        }
        for (unsigned b = 0; b < 2; ++b) {
            write_u32(&out, d.adc_gaps[b]);
        }
        for (unsigned b = 0; b < 2; ++b) {
            write_u32(&out, d.read_errors[b]);
        }
        for (unsigned b = 0; b < 2; ++b) {
            write_u32(&out, d.published[b]);
        }
        for (unsigned b = 0; b < 2; ++b) {
            write_u32(&out, d.discarded[b]);
        }
        for (unsigned b = 0; b < 2; ++b) {
            write_u32(&out, d.max_read_us[b]);
        }
        for (unsigned b = 0; b < 2; ++b) {
            write_u32(&out, d.raw_overwrites[b]);
        }
        for (unsigned b = 0; b < 2; ++b) {
            write_u32(&out, d.processing_overruns[b]);
        }
        write_u32(&out, d.late_passes);
        write_u32(&out, d.max_pass_us);
        write_u32(&out, d.max_drain_us);
        break;
    }
    default:
        break;
    }
    if (!reply[1] && ((type >= 2 && type <= 8) || (type >= 14 && type <= 17) || type == 19 ||
                      (type >= 22 && type <= 24))) {
        telemetry_changed();
    }
    return finish(reply, reply[1] ? 2 : (size_t)(out - reply));
}

size_t controller_command(const uint8_t *request, size_t length, uint8_t out[1024],
                          bool *initialized)
{
    if (!length) {
        return 0;
    }
    input = request + 1;
    remaining = length - 1;
    response_size = 0;
    bool handled = handle(request[0], initialized);
    if (!handled || remaining) {
        reply[0] = request[0];
        reply[1] = STATUS_INVALID;
        response_size = 2;
    }
    memcpy(out, reply, response_size);
    return response_size;
}
