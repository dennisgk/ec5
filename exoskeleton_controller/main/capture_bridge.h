#pragma once
#include "capture_shared.h"

bool capture_bridge_start(void);
bool capture_bridge_pause(void);
bool capture_bridge_configure(const capture_board_request boards[2], uint32_t changed_mask);
bool capture_bridge_resume(void);
void capture_bridge_fail(uint32_t fault);
bool capture_bridge_healthy(void);
capture_memory *capture_bridge_memory(void);
uint64_t capture_bridge_time_us(const capture_record *record);
uint32_t capture_bridge_fault(void);
void capture_bridge_check_progress(void);
