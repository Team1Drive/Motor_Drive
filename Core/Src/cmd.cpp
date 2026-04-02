#include "cmd.h"
#include "parameters.h"
#include <cstdint>
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <stdarg.h>

/**
 * @brief A simple printf-like function that formats a string and sends it over USB using the CDC interface. This function uses a fixed-size buffer to hold the formatted string and supports variable arguments like printf.
 * @param format The format string, similar to printf.
 */
void usb_printf(const char *format, ...) {
    char buffer[256];
    va_list args;
    va_start(args, format);

    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (len > 0) {
        CDC_Transmit_HS((uint8_t*)buffer, len);
    }
}

bool ring_buffer_write(ring_buffer_t* rx_ring, uint8_t data) {
    uint16_t next_head = (rx_ring->head + 1) % RX_RING_SIZE;
    if (next_head != rx_ring->tail) {
        rx_ring->buffer[rx_ring->head] = data;
        rx_ring->head = next_head;
        return true;
    }
    return false;
}

bool read_line_from_ring(ring_buffer_t* rx_ring, char* line, int max_len) {
    static char line_buffer[CMD_MAX_LEN];
    static int idx = 0;

    while (rx_ring->tail != rx_ring->head) {
        uint8_t c = rx_ring->buffer[rx_ring->tail];
        rx_ring->tail = (rx_ring->tail + 1) % RX_RING_SIZE;

        if (c == '\n' || c == '\r') {
            if (idx > 0) {
                line_buffer[idx] = '\0';
                strncpy(line, line_buffer, max_len);
                idx = 0;
                return true;
            }
        } else if (idx < max_len - 1) {
            line_buffer[idx++] = c;
        } else {
            idx = 0;
        }
    }
    return false;
}
