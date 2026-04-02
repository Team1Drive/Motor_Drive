#pragma once

#include "stm32h7xx_hal.h"
#include <cstdint>

#define RX_RING_SIZE 256
#define CMD_MAX_LEN 64
#define MAX_ARGC 8

typedef struct {
    uint8_t buffer[RX_RING_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} ring_buffer_t;

typedef void (*cmd_handler_t)(int argc, char** argv);

typedef struct {
    const char* cmd;        // Command string (full match or prefix)
    cmd_handler_t handler;  // Corresponding handler function
    int min_args;           // Minimum number of arguments required (including command itself)
    const char* usage;      // Usage string for help (e.g., "start", "foc <rpm>", "tune speed p 0.1")
} cmd_entry_t;

/**
 * @brief A simple printf-like function that formats a string and sends it over USB using the CDC interface. This function uses a fixed-size buffer to hold the formatted string and supports variable arguments like printf.
 * @param format The format string, similar to printf.
 */
void usb_printf(const char *format, ...);

bool ring_buffer_write(ring_buffer_t* rx_ring, uint8_t data);

bool read_line_from_ring(ring_buffer_t* rx_ring, char* line, int max_len);

void process_command(const char* cmd_str);