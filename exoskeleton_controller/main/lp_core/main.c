#include "../capture_shared.h"
#include "riscv/rv_utils.h"
#include "ulp_lp_core_i2c.h"
#include "ulp_lp_core_interrupts.h"
#include "ulp_lp_core_lp_timer_shared.h"
#include "ulp_lp_core_utils.h"

// Exported through the generated lp_capture.h header. HP writes this only while
// LP is stopped, after loading the binary and before starting the coprocessor.
volatile uint32_t shared_address;
static capture_memory *shared;
static bool counter_valid[2];
static uint8_t previous_counter[2];
static uint32_t sequences[2];
static uint64_t previous_time[2];
static uint64_t next_pass;
static uint64_t long_gap_ticks;
static struct {
    uint32_t milliseconds;
    uint64_t interval_ticks, next;
    unsigned step;
} resend[2];
static unsigned resend_cursor;
static int configuring_board = -1;

static void configuration_failure(unsigned operation, uint8_t device, uint8_t address,
                                  unsigned expected, unsigned actual, esp_err_t error)
{
    if (configuring_board < 0) {
        return;
    }
    uint32_t *failure = shared->configuration_error[configuring_board];
    if (failure[0]) {
        return;
    }
    failure[1] = device;
    failure[2] = address;
    failure[3] = expected;
    failure[4] = actual;
    failure[5] = (uint32_t)error;
    failure[0] = operation;
}

// The IDF LP driver uses a finite polling budget internally (not an RTOS delay).
// Never pass -1. HP independently bounds complete mailbox commands to 250 ms.
#define I2C_WAIT_BUDGET 1024

void LP_CORE_ISR_ATTR ulp_lp_core_lp_pmu_intr_handler(void)
{
    ulp_lp_core_sw_intr_clear();
}

static uint64_t clock_ticks(void)
{
    return ulp_lp_core_lp_timer_get_cycle_count();
}

static void increment(uint32_t *counter)
{
    capture_store(counter, capture_load(counter) + 1);
}

static void maximum(uint32_t *counter, uint32_t value)
{
    if (value > capture_load(counter)) {
        capture_store(counter, value);
    }
}

static bool send_bytes(uint8_t device, const uint8_t *bytes, unsigned size)
{
    esp_err_t error =
        lp_core_i2c_master_write_to_device(LP_I2C_NUM_0, device, bytes, size, I2C_WAIT_BUDGET);
    if (error != ESP_OK) {
        configuration_failure(1, device, bytes[0], size > 1 ? bytes[1] : 0, 0, error);
    }
    return error == ESP_OK;
}

static bool write_register(uint8_t device, uint8_t address, uint8_t value)
{
    uint8_t bytes[] = {address, value};
    return send_bytes(device, bytes, sizeof(bytes));
}

static bool read_bytes(uint8_t device, uint8_t address, uint8_t *bytes, unsigned size)
{
    esp_err_t error = lp_core_i2c_master_write_read_device(LP_I2C_NUM_0, device, &address, 1, bytes,
                                                           size, I2C_WAIT_BUDGET);
    if (error != ESP_OK) {
        configuration_failure(2, device, address, 0, 0, error);
    }
    return error == ESP_OK;
}

static bool verify(uint8_t device, uint8_t address, uint8_t expected, uint8_t mask)
{
    uint8_t actual = 0;
    if (!read_bytes(device, address, &actual, 1)) {
        return false;
    }
    if ((actual & mask) != (expected & mask)) {
        configuration_failure(3, device, address, expected & mask, actual & mask, ESP_OK);
        return false;
    }
    return true;
}

static bool wiper(unsigned b, uint8_t address, uint32_t value)
{
    uint8_t bytes[2] = {0};
    uint8_t device = (uint8_t)(0x2C + b);
    if (!write_register(device, (address << 4) | (value >> 8), (uint8_t)value) ||
        !read_bytes(device, (address << 4) | 0x0C, bytes, 2)) {
        return false;
    }
    uint32_t actual = ((uint32_t)(bytes[0] & 3) << 8) | bytes[1];
    if (actual != value) {
        configuration_failure(3, device, (address << 4) | 0x0C, value, actual, ESP_OK);
        return false;
    }
    return true;
}

static void configuration_delay(unsigned microseconds)
{
    uint64_t until = clock_ticks() + (((uint64_t)microseconds << 19) / shared->period_q19);
    while (clock_ticks() < until) {
    }
}

static bool configure_once(unsigned b)
{
    const capture_board_request *config = &shared->boards[b];
    uint8_t adc = (uint8_t)(0x40 + b), expander = (uint8_t)(0x20 + b);
    uint8_t command = 2;
    if (!config->enabled) {
        return send_bytes(adc, &command, 1);
    }
    if (config->ina_code > 7 || config->adc_code > 7 || config->hp_wiper > 256 ||
        config->lp_wiper > 256) {
        return false;
    }

    // The expander controls ADC_RESET_N and DIGITPOT_RESET_N. Release
    // those first, as the reference sketch does, before addressing either IC.
    if (!write_register(expander, 1, 0x58 | config->ina_code) || !write_register(expander, 2, 0) ||
        !write_register(expander, 3, 0xA0)) {
        return false;
    }
    configuration_delay(2000);
    if (!verify(expander, 1, 0x58 | config->ina_code, 0xFF) || !verify(expander, 2, 0, 0xFF) ||
        !verify(expander, 3, 0xA0, 0xFF) || !send_bytes(adc, &command, 1)) {
        return false;
    }
    command = 6;
    if (!send_bytes(adc, &command, 1)) {
        return false;
    }
    configuration_delay(5000);
    if (!wiper(b, 0, config->hp_wiper) || !wiper(b, 1, config->hp_wiper) ||
        !wiper(b, 6, config->lp_wiper) || !wiper(b, 7, config->lp_wiper) ||
        !write_register(adc, 0x40, config->adc_code << 1) || !write_register(adc, 0x44, 0xC8) ||
        !write_register(adc, 0x48, 0x40) || !write_register(adc, 0x4C, 0)) {
        return false;
    }
    if (!verify(adc, 0x20, config->adc_code << 1, 0xFF) || !verify(adc, 0x24, 0xC8, 0xFF) ||
        !verify(adc, 0x28, 0x40, 0x7F) || !verify(adc, 0x2C, 0, 0xFF)) {
        return false;
    }
    command = 8;
    return send_bytes(adc, &command, 1);
}

static bool configure(unsigned b)
{
    resend[b].milliseconds = 0;
    resend[b].step = 0;
    counter_valid[b] = false;
    // Retry only cold configuration, never the acquisition/control hot path.
    // Fail fast instead of continuing writes after a failed prerequisite.
    for (unsigned attempt = 0; attempt < 2; ++attempt) {
        for (unsigned n = 0; n < 6; ++n) {
            shared->configuration_error[b][n] = 0;
        }
        configuring_board = (int)b;
        bool ok = configure_once(b);
        configuring_board = -1;
        if (ok) {
            return true;
        }
        uint8_t stop = 2;
        send_bytes((uint8_t)(0x40 + b), &stop, 1);
        configuration_delay(2000);
    }
    return false;
}

// Reassert the current hardware values without STOP, RESET, START, counter
// resets, or HP buffer invalidation. One short write is attempted per pass;
// acquisition of both boards always runs first. No periodic serial traffic.
static void resend_parameters(void)
{
    uint64_t now = clock_ticks();
    for (unsigned n = 0; n < 2; ++n) {
        unsigned b = (resend_cursor + n) & 1;
        uint32_t milliseconds = capture_load(&shared->resend_ms[b]);
        if (!milliseconds || !shared->boards[b].enabled ||
            !(capture_load(&shared->configured_mask) & (1u << b))) {
            resend[b].milliseconds = 0;
            resend[b].step = 0;
            continue;
        }
        if (milliseconds != resend[b].milliseconds) {
            resend[b].milliseconds = milliseconds;
            resend[b].interval_ticks = ((uint64_t)milliseconds * 1000 << 19) / shared->period_q19;
            resend[b].next = now + resend[b].interval_ticks;
            resend[b].step = 0;
        }
        if (now < resend[b].next) {
            continue;
        }
        const capture_board_request *config = &shared->boards[b];
        bool ok;
        if (resend[b].step == 0) {
            ok = write_register((uint8_t)(0x20 + b), 1, 0x58 | config->ina_code);
        } else if (resend[b].step == 1) {
            ok = write_register((uint8_t)(0x40 + b), 0x40, config->adc_code << 1);
        } else {
            unsigned step = resend[b].step - 2;
            uint32_t value = step < 2 ? config->hp_wiper : config->lp_wiper;
            uint8_t address = (uint8_t)(step < 2 ? step : step + 4);
            ok = write_register((uint8_t)(0x2C + b), (address << 4) | (value >> 8), (uint8_t)value);
        }
        if (!ok) {
            increment(&shared->resend_errors[b]);
        }
        if (++resend[b].step == 6) {
            resend[b].step = 0;
            // Never catch up missed intervals with a burst of writes.
            resend[b].next = clock_ticks() + resend[b].interval_ticks;
        }
        resend_cursor = b ^ 1;
        return;
    }
}

static void mailbox(uint32_t sequence)
{
    uint32_t state = capture_load(&shared->state);
    switch (shared->request) {
    case CAPTURE_PAUSE:
        capture_store(&shared->state, CAPTURE_PAUSED);
        break;
    case CAPTURE_CONFIGURE:
        if (state != CAPTURE_PAUSED) {
            capture_store(&shared->state, CAPTURE_FAULT);
            break;
        }
        for (unsigned b = 0; b < 2; ++b) {
            if (shared->board_mask & (1u << b)) {
                bool ok = configure(b);
                uint32_t mask = capture_load(&shared->configured_mask) & ~(1u << b);
                if (ok && shared->boards[b].enabled) {
                    mask |= 1u << b;
                }
                capture_store(&shared->configured_mask, mask);
            }
        }
        break;
    case CAPTURE_RESUME: {
        bool active = false;
        for (unsigned b = 0; b < 2; ++b) {
            active |=
                shared->boards[b].enabled && (capture_load(&shared->configured_mask) & (1u << b));
        }
        next_pass = clock_ticks();
        capture_store(&shared->state, active ? CAPTURE_RUNNING : CAPTURE_PAUSED);
        break;
    }
    default:
        capture_store(&shared->state, CAPTURE_FAULT);
        break;
    }
    capture_store(&shared->acknowledgement, sequence);
}

static void acquire(unsigned b)
{
    uint8_t data[4] = {0};
    uint64_t begin = clock_ticks();
    bool ok = read_bytes((uint8_t)(0x40 + b), 0x10, data, sizeof(data));
    uint64_t now = clock_ticks();
    maximum(&shared->max_read_ticks[b], (uint32_t)(now - begin));
    if (!ok) {
        increment(&shared->read_errors[b]);
        return;
    }
    if ((counter_valid[b] && data[0] == previous_counter[b]) || (!counter_valid[b] && !data[0])) {
        return;
    }
    uint32_t delta = counter_valid[b] ? (uint8_t)(data[0] - previous_counter[b]) : 1;
    uint32_t flags = counter_valid[b] ? 0 : CAPTURE_FIRST;
    // Keep long interruptions distinguishable even when the 8-bit ADC counter
    // wraps to a modulo difference of one. This is not an exact loss count.
    if (counter_valid[b] && now - previous_time[b] > long_gap_ticks && delta == 1) {
        ++delta;
    }
    if (delta != 1) {
        flags |= CAPTURE_GAP;
        increment(&shared->adc_gap_events[b]);
    }
    increment(&shared->conversions[b]);
    sequences[b] += delta;
    counter_valid[b] = true;
    previous_counter[b] = data[0];
    previous_time[b] = now;
    capture_record record = capture_adc_record(data, b, now, sequences[b], flags);
    if (capture_push(shared, &record)) {
        increment(&shared->published[b]);
    } else {
        increment(&shared->discarded[b]);
    }
}

int main(void)
{
    shared = (capture_memory *)(uintptr_t)shared_address;
    if (!shared || shared->magic != CAPTURE_MAGIC || shared->layout != CAPTURE_LAYOUT ||
        shared->capacity != CAPTURE_CAPACITY || !shared->period_q19 || !shared->poll_ticks) {
        ulp_lp_core_stop_lp_core();
        while (1) {
        }
    }
    long_gap_ticks = (uint32_t)(((uint64_t)200000 << 19) / shared->period_q19);
    ulp_lp_core_sw_intr_clear();
    ulp_lp_core_sw_intr_enable(true);
    ulp_lp_core_intr_enable();
    capture_store(&shared->state, CAPTURE_PAUSED);
    while (true) {
        uint32_t request;
        if (capture_pending_request(shared, &request)) {
            mailbox(request);
            continue;
        }
        if (capture_load(&shared->state) != CAPTURE_RUNNING) {
            // Leave the per-source interrupt enabled: RISC-V WFI wakes on it even
            // with global MIE clear. This closes the check-before-sleep race.
            RV_CLEAR_CSR(mstatus, MSTATUS_MIE);
            if (capture_load(&shared->request_sequence) == capture_load(&shared->acknowledgement)) {
                ulp_lp_core_wait_for_intr();
            }
            RV_SET_CSR(mstatus, MSTATUS_MIE);
            continue;
        }
        uint64_t begin = clock_ticks();
        if (begin < next_pass) {
            continue;
        }
        uint32_t period = shared->poll_ticks;
        if (begin > next_pass + period) {
            increment(&shared->late_passes);
            next_pass = begin;
        }
        next_pass += period;
        for (unsigned b = 0; b < 2; ++b) {
            if (shared->boards[b].enabled && (capture_load(&shared->configured_mask) & (1u << b))) {
                acquire(b);
            }
        }
        resend_parameters();
        maximum(&shared->max_pass_ticks, (uint32_t)(clock_ticks() - begin));
        increment(&shared->heartbeat);
    }
}
