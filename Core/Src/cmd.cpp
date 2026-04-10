#include "cmd.h"
#include "parameters.h"
#include <cstdint>
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <stdarg.h>

extern void cmd_start(int argc, char** argv);
extern void cmd_stop(int argc, char** argv);
extern void cmd_reset(int argc, char** argv);
extern void cmd_foc(int argc, char** argv);
extern void cmd_rpm(int argc, char** argv);
extern void cmd_foc_status(int argc, char** argv);
extern void cmd_sixstep(int argc, char** argv);
extern void cmd_speed(int argc, char** argv);
extern void cmd_duty(int argc, char** argv);
extern void cmd_vec(int argc, char** argv);
extern void cmd_tune(int argc, char** argv);
extern void cmd_log(int argc, char** argv);
extern void cmd_audible(int argc, char** argv);

static const cmd_entry_t cmd_table[] = {
    { "start",       cmd_start,       1,    1,  "Usage: start\r\n"                              }, // 1 means just the command itself
    { "stop",        cmd_stop,        1,    1,  "Usage: stop\r\n"                               },
    { "reset",       cmd_reset,       1,    1,  "Usage: reset\r\n"                              },
    { "foc",         cmd_foc,         2,    3,  "Usage: foc <rpm> or foc status\r\n"            }, // e.g., "foc 1000" = 2 tokens
    { "rpm",         cmd_rpm,         2,    2,  "Usage: rpm <value>\r\n"                        },
    { "sixstep",     cmd_sixstep,     1,    1,  "Usage: sixstep\r\n"                            },
    { "speed",       cmd_speed,       2,    2,  "Usage: speed <value>\r\n"                      },
    { "duty",        cmd_duty,        2,    2,  "Usage: duty <v1>,<v2>,<v3>\r\n"                }, // Arguments can be kept comma-separated internally
    { "vec",         cmd_vec,         2,    2,  "Usage: vec <0-5>\r\n"                          },
    { "tune",        cmd_tune,        4,    4,  "Usage: tune <subsys> <param> <value>\r\n"      }, // e.g., "tune speed p 0.1" = 4 tokens
    { "log",         cmd_log,         2,    3,  "Usage: log <add|rm|preset|utf8|bin> [var]\r\n" },
    { "audible",     cmd_audible,     1,    1,  "Usage: audible\r\n"                            }
};

const int num_commands = sizeof(cmd_table) / sizeof(cmd_entry_t);

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

void process_command(const char* cmd_str) {
    char cmd_copy[CMD_MAX_LEN]; // Temporary buffer for parsing
    strncpy(cmd_copy, cmd_str, sizeof(cmd_copy) - 1);
    cmd_copy[sizeof(cmd_copy) - 1] = '\0';

    char* argv[MAX_ARGC];
    int argc = 0;

    // 1. Universal parsing / Tokenization via space delimiter
    char* token = strtok(cmd_copy, " ");
    while (token != NULL && argc < MAX_ARGC) {
        argv[argc++] = token;
        token = strtok(NULL, " ");
    }

    // Ignore empty commands
    if (argc == 0) return;

    // 2. Command Lookup
    for (int i = 0; i < num_commands; i++) {
        if (strcmp(argv[0], cmd_table[i].cmd) == 0) {
            // Validate arguments
            if (argc < cmd_table[i].min_args || argc > cmd_table[i].max_args) {
                CDC_Transmit_HS((uint8_t*)cmd_table[i].usage, strlen(cmd_table[i].usage));
                return;
            }
            // Execute the mapped handler
            cmd_table[i].handler(argc, argv);
            return;
        }
    }

    // 3. Command not found
    const char* err = "Unknown command\r\n";
    CDC_Transmit_HS((uint8_t*)err, strlen(err));
}

void protectionModePrint(void) {
    const char* msg = "\r\nSystem tripped by overcurrent: Motor in protection mode, reset error to start\r\n\r\n";
    CDC_Transmit_HS((uint8_t*)msg, strlen(msg));
}

void batteryProtectionPrint(void) {
    const char* msg = "\r\nSystem operating under battery protection, this function is disabled\r\nIf supplied with a current-limited source, set BATTERY_PROTECTION to false\r\n\r\n";
    CDC_Transmit_HS((uint8_t*)msg, strlen(msg));
}
