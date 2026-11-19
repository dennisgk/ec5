#include "capture_bridge.h"
#include "controller.h"
#include "esp_log.h"
#include "esp_private/esp_clk.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lp_capture.h"
#include "lp_core_i2c.h"
#include "ulp_lp_core.h"
#include "ulp_lp_core_lp_timer_shared.h"
#include <string.h>

extern const uint8_t lp_capture_bin_start[] asm("_binary_lp_capture_bin_start");
extern const uint8_t lp_capture_bin_end[] asm("_binary_lp_capture_bin_end");

// C6 has no external RAM: static BSS is HP internal SRAM, not LP SRAM.
static capture_memory shared;
static uint32_t fault, operation;
static uint32_t loaded;
static uint64_t tick_origin, hp_origin;
static uint32_t last_heartbeat;
static int64_t progress_at;

capture_memory *capture_bridge_memory(void)
{
    return &shared;
}

uint32_t capture_bridge_fault(void)
{
    return capture_load(&fault);
}

bool capture_bridge_healthy(void)
{
    return capture_load(&loaded) && !capture_bridge_fault();
}

void capture_bridge_fail(uint32_t reason)
{
    // LP acquisition does not control the manually requested motor output.
    // serial disconnect and motor CAN/feedback protections retain their authority.
    // Prevent two HP tasks from stopping/reloading the LP subsystem concurrently.
    uint32_t expected = 0;
    if (__atomic_compare_exchange_n(&fault, &expected, reason, false, __ATOMIC_ACQ_REL,
                                    __ATOMIC_ACQUIRE)) {
        ulp_lp_core_stop();
        capture_store(&shared.configured_mask, 0);
        capture_store(&shared.state, CAPTURE_FAULT);
    }
}

static bool wait_word(const uint32_t *word, uint32_t value)
{
    int64_t deadline = esp_timer_get_time() + 250000;
    while (capture_load(word) != value) {
        if (esp_timer_get_time() >= deadline) {
            capture_bridge_fail(1);
            return false;
        }
        vTaskDelay(1);
    }
    return true;
}

static bool request(uint32_t command)
{
    if (!capture_bridge_healthy()) {
        return false;
    }
    capture_store(&operation, 1);
    uint32_t sequence = capture_publish_request(&shared, command);
    ulp_lp_core_sw_intr_trigger();
    bool ok = wait_word(&shared.acknowledgement, sequence);
    if (capture_load(&shared.state) == CAPTURE_FAULT) {
        capture_bridge_fail(4);
    }
    progress_at = esp_timer_get_time();
    last_heartbeat = capture_load(&shared.heartbeat);
    capture_store(&operation, 0);
    return ok && capture_bridge_healthy();
}

bool capture_bridge_start(void)
{
    capture_store(&operation, 1);
    if (capture_load(&loaded)) {
        ulp_lp_core_stop();
    }
    capture_store(&loaded, 0);
    memset(&shared, 0, sizeof(shared));
    shared.magic = CAPTURE_MAGIC;
    shared.layout = CAPTURE_LAYOUT;
    shared.capacity = CAPTURE_CAPACITY;
    shared.period_q19 = esp_clk_slowclk_cal_get();
    if (!shared.period_q19) {
        capture_store(&fault, 3);
        capture_store(&shared.state, CAPTURE_FAULT);
        capture_store(&operation, 0);
        return false;
    }
    shared.poll_ticks = (uint32_t)(((uint64_t)500 << 19) / shared.period_q19);
    lp_core_i2c_cfg_t i2c = LP_CORE_I2C_DEFAULT_CONFIG();
    i2c.i2c_src_clk = LP_I2C_SCLK_XTAL_D2;
    esp_err_t error = lp_core_i2c_master_init(LP_I2C_NUM_0, &i2c);
    if (error == ESP_OK) {
        error = ulp_lp_core_load_binary(lp_capture_bin_start,
                                        lp_capture_bin_end - lp_capture_bin_start);
    }
    if (error != ESP_OK) {
        capture_store(&fault, 3);
        capture_store(&shared.state, CAPTURE_FAULT);
        capture_store(&operation, 0);
        return false;
    }
    ulp_shared_address = (uint32_t)(uintptr_t)&shared;
    uint64_t before = esp_timer_get_time();
    tick_origin = ulp_lp_core_lp_timer_get_cycle_count();
    hp_origin = (before + (uint64_t)esp_timer_get_time()) / 2;
    capture_store(&fault, 0);
    ulp_lp_core_cfg_t config = {.wakeup_source = ULP_LP_CORE_WAKEUP_SOURCE_HP_CPU};
    error = ulp_lp_core_run(&config);
    capture_store(&loaded, error == ESP_OK);
    if (!capture_load(&loaded)) {
        capture_store(&fault, 3);
        capture_store(&shared.state, CAPTURE_FAULT);
    }
    bool ready = capture_load(&loaded) && wait_word(&shared.state, CAPTURE_PAUSED);
    last_heartbeat = capture_load(&shared.heartbeat);
    progress_at = esp_timer_get_time();
    capture_store(&operation, 0);
    return ready;
}

bool capture_bridge_pause(void)
{
    return request(CAPTURE_PAUSE);
}

bool capture_bridge_configure(const capture_board_request boards[2], uint32_t changed_mask)
{
    if (capture_load(&shared.state) != CAPTURE_PAUSED) {
        return false;
    }
    memcpy(shared.boards, boards, sizeof(shared.boards));
    shared.board_mask = changed_mask;
    bool ok = request(CAPTURE_CONFIGURE);
    if (ok) {
        for (unsigned b = 0; b < 2; ++b) {
            const uint32_t *failure = shared.configuration_error[b];
            if ((changed_mask & (1u << b)) && failure[0]) {
                ESP_LOGE("emg",
                         "Board %u configuration: operation=%lu device=0x%02lX register=0x%02lX "
                         "expected=0x%lX actual=0x%lX error=%s (0x%lX)",
                         b, (unsigned long)failure[0], (unsigned long)failure[1],
                         (unsigned long)failure[2], (unsigned long)failure[3],
                         (unsigned long)failure[4], esp_err_to_name((esp_err_t)failure[5]),
                         (unsigned long)failure[5]);
            }
        }
    }
    return ok;
}

bool capture_bridge_resume(void)
{
    return request(CAPTURE_RESUME);
}

uint64_t capture_bridge_time_us(const capture_record *record)
{
    return hp_origin + capture_ticks_to_us(capture_ticks(record) - tick_origin, shared.period_q19);
}

void capture_bridge_check_progress(void)
{
    if (!capture_bridge_healthy() || capture_load(&operation) ||
        capture_load(&shared.state) != CAPTURE_RUNNING) {
        progress_at = esp_timer_get_time();
        last_heartbeat = capture_load(&shared.heartbeat);
        return;
    }
    uint32_t heartbeat = capture_load(&shared.heartbeat);
    if (heartbeat != last_heartbeat) {
        last_heartbeat = heartbeat;
        progress_at = esp_timer_get_time();
    } else if (esp_timer_get_time() - progress_at > 100000) {
        capture_bridge_fail(2);
    }
}
