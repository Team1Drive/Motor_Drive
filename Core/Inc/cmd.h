#pragma once

#include "stm32h7xx_hal.h"
#include <cstdint>

#define RX_RING_SIZE 256

typedef struct {
    uint8_t buffer[RX_RING_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} ring_buffer_t;

/* typedef void (*cmd_handler_t)(const char* cmd);

typedef struct {
    const char* cmd;          // 命令字符串（完整匹配或前缀）
    cmd_handler_t handler;    // 处理函数
    uint8_t prefix:1;         // 1: 前缀匹配（如 "foc "），0: 完全匹配
} cmd_entry_t;

static const cmd_entry_t cmd_table[] = {
    { "start",      cmd_start,      0 },
    { "stop",       cmd_stop,       0 },
    { "reset",      cmd_reset,      0 },
    { "foc ",       cmd_foc,        1 },   // 前缀匹配，注意空格
    { "rpm ",       cmd_rpm,        1 },
    { "foc_status", cmd_foc_status, 0 },
    { "sixstep",    cmd_sixstep,    0 },
    { "speed ",     cmd_speed,      1 },
    { "duty ",      cmd_duty,       1 },
    { "vec ",       cmd_vec,        1 },
    { "tune ",      cmd_tune,       1 },
    { "log ",       cmd_log,        1 },
    { "audible",    cmd_audible,    0 },
}; */

void usb_printf(const char *format, ...);

bool ring_buffer_write(ring_buffer_t* rx_ring, uint8_t data);

bool read_line_from_ring(ring_buffer_t* rx_ring, char* line, int max_len);