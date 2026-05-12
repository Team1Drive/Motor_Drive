#pragma once

#include "stm32h7xx_hal.h"
#include <cstdint>

#define RX_RING_SIZE 256
#define USB_TX_TEXT_QUEUE_SIZE 2048
#define USB_TX_TELEMETRY_QUEUE_SIZE 16384
#define USB_TX_BULK_QUEUE_SIZE 32768
#define USB_TX_CHUNK_SIZE 500
#define USB_TX_LOW_WATERMARK_PERCENT 50
#define USB_TX_HIGH_WATERMARK_PERCENT 75
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
    int max_args;           // Maximum number of arguments allowed (including command itself)
    const char* usage;      // Usage string for help (e.g., "start", "foc <rpm>", "tune speed p 0.1")
} cmd_entry_t;

/**
 * @brief A simple printf-like function that formats a string and sends it over USB using the CDC interface. This function uses a fixed-size buffer to hold the formatted string and supports variable arguments like printf.
 * @param format The format string, similar to printf.
 */
void usb_printf(const char *format, ...);

/**
 * @brief Queues a binary telemetry packet for USB CDC transmission.
 * @note Data is copied into the USB TX queue before this function returns, so
 *       callers may use stack/local buffers safely. Returns false if the queue
 *       does not have enough free space for the whole packet.
 */
bool usb_sendTelemetry(const uint8_t* buffer, uint16_t length);

/**
 * @brief Queues bulk USB CDC data for future high-throughput transfers.
 * @note This is currently all-or-nothing to avoid partial application packets.
 *       Large captures should be split into chunks by the caller.
 */
bool usb_sendBulk(const uint8_t* buffer, uint16_t length);

/**
 * @brief Pumps queued USB TX data into the CDC endpoint when it is idle.
 * @note Call frequently from the main loop. It is also called opportunistically
 *       by usb_printf(), usb_sendTelemetry(), and usb_sendBulk().
 */
void usb_tx_service(void);

/**
 * @brief Marks the active USB CDC transfer complete and starts the next queued
 *        chunk if data is waiting.
 * @note Call this from the USB CDC transmit-complete callback.
 */
#ifdef __cplusplus
extern "C" {
#endif
void usb_tx_onTransmitComplete(void);
#ifdef __cplusplus
}
#endif

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
