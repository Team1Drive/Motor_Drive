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

/**
 * @brief Writes a byte of data into the ring buffer. This function checks if there is space in the buffer before writing and updates the head index accordingly. It returns true if the write was successful, or false if the buffer is full.
 * @param rx_ring Pointer to the ring buffer structure.
 */
bool ring_buffer_write(ring_buffer_t* rx_ring, uint8_t data);

/**
 * @brief Reads a line of text from the ring buffer into a provided buffer. The function looks for newline characters to determine the end of a command. It handles buffering internally and ensures that the output line is null-terminated. The function returns true if a complete line was read, or false if no complete line is available.
 * @param rx_ring Pointer to the ring buffer structure.
 */
bool read_line_from_ring(ring_buffer_t* rx_ring, char* line, int max_len);

/**
 * @brief Processes a command string by parsing it and calling the appropriate handler function.
 * @param cmd_str Pointer to the command string to be processed.
 */
void process_command(const char* cmd_str);

/**
 * @brief Handler for protection mode when any attepmt for restarting the motor without resetting the error flag is made.
 */
void protectionModePrint(void);

/**
 * @brief Handler for battery protection mode when any attempt to use disabled functions is made.
 */
void batteryProtectionPrint(void);