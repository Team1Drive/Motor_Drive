#include "cmd.h"
#include "parameters.h"
#include <cstdint>
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <stdarg.h>
#include <cstring>

extern "C" USBD_HandleTypeDef hUsbDeviceHS;

extern void cmd_start(int argc, char** argv);
extern void cmd_stop(int argc, char** argv);
extern void cmd_align(int argc, char** argv);
extern void cmd_reset(int argc, char** argv);
extern void cmd_foc(int argc, char** argv);
extern void cmd_sixstep(int argc, char** argv);
extern void cmd_speed(int argc, char** argv);
extern void cmd_torque(int argc, char** argv);
extern void cmd_mod(int argc, char** argv);
extern void cmd_duty(int argc, char** argv);
extern void cmd_vec(int argc, char** argv);
extern void cmd_tune(int argc, char** argv);
extern void cmd_increment(int argc, char** argv);
extern void cmd_board(int argc, char** argv);
extern void cmd_log(int argc, char** argv);
extern void cmd_sim(int argc, char** argv);
extern void cmd_audible(int argc, char** argv);
extern void cmd_sin(int argc, char** argv);
extern void cmd_cos(int argc, char** argv);
extern void cmd_arctan(int argc, char** argv);
extern void cmd_hypot(int argc, char** argv);

static const cmd_entry_t cmd_table[] = {
    { "start",       cmd_start,       1,    2,  "Usage: start <mode>\r\n"                       }, // 1 means just the command itself
    { "stop",        cmd_stop,        1,    1,  "Usage: stop\r\n"                               },
    { "align",       cmd_align,       1,    2,  "Usage: align\r\n"                              },
    { "reset",       cmd_reset,       1,    1,  "Usage: reset\r\n"                              },
    { "foc",         cmd_foc,         2,    3,  "Usage: foc <rpm> or foc status\r\n"            }, // e.g., "foc 1000" = 2 tokens
    { "sixstep",     cmd_sixstep,     1,    1,  "Usage: sixstep\r\n"                            },
    { "speed",       cmd_speed,       2,    3,  "Usage: speed <value>\r\n"                      },
    { "torque",      cmd_torque,      2,    3,  "Usage: torque <value>\r\n"                     },
    { "mod",         cmd_mod,         2,    2,  "Usage: mod <type>\r\n"                         }, // Select modulation type, e.g., "mod svpwm"
    { "duty",        cmd_duty,        2,    2,  "Usage: duty <v1>,<v2>,<v3>\r\n"                }, // Arguments can be kept comma-separated internally
    { "vec",         cmd_vec,         2,    2,  "Usage: vec <0-5>\r\n"                          },
    { "tune",        cmd_tune,        4,    4,  "Usage: tune <subsys> <param> <value>\r\n"      }, // e.g., "tune speed p 0.1" = 4 tokens
    { "increment",   cmd_increment,   4,    4,  "Usage: increment <subsys> <param> <value>\r\n" }, // e.g., "increment speed p 0.1" = 4 tokens
    { "log",         cmd_log,         2,    3,  "Usage: log <add|rm|preset|utf8|bin> [var]\r\n" },
    { "sim",         cmd_sim,         2,    2,  "Usage: sim <start|status|reset>\r\n"           },
    { "board",       cmd_board,       1,    2,  "Usage: board <1|2|3>\r\n"                      },
    { "audible",     cmd_audible,     1,    1,  "Usage: audible\r\n"                            },
    { "sin",         cmd_sin,         2,    2,  "Usage: sin <value>\r\n"                        },
    { "cos",         cmd_cos,         2,    2,  "Usage: cos <value>\r\n"                        },
    { "arctan",      cmd_arctan,      3,    3,  "Usage: arctan <y> <x>\r\n"                     },
    { "hypot",       cmd_hypot,       3,    3,  "Usage: hypot <x> <y>\r\n"                      }
};

const int num_commands = sizeof(cmd_table) / sizeof(cmd_entry_t);

namespace {

enum class UsbTxState : uint8_t {
    Idle,
    Pending,
    InFlight,
};

enum class UsbTxQueueKind : uint8_t {
    Text,
    Telemetry,
    Bulk,
};

struct UsbTxQueue {
    uint8_t* buffer;
    uint16_t size;
    volatile uint16_t head;
    volatile uint16_t tail;
};

alignas(32) uint8_t usb_tx_text_ring[USB_TX_TEXT_QUEUE_SIZE] __attribute__((section(".sram_d1")));
alignas(32) uint8_t usb_tx_telemetry_ring[USB_TX_TELEMETRY_QUEUE_SIZE] __attribute__((section(".sram_d1")));
alignas(32) uint8_t usb_tx_bulk_ring[USB_TX_BULK_QUEUE_SIZE] __attribute__((section(".sram_d1")));
alignas(32) uint8_t usb_tx_chunk[USB_TX_CHUNK_SIZE] __attribute__((section(".sram_d1")));

UsbTxQueue usb_tx_text_queue = {usb_tx_text_ring, USB_TX_TEXT_QUEUE_SIZE, 0, 0};
UsbTxQueue usb_tx_telemetry_queue = {usb_tx_telemetry_ring, USB_TX_TELEMETRY_QUEUE_SIZE, 0, 0};
UsbTxQueue usb_tx_bulk_queue = {usb_tx_bulk_ring, USB_TX_BULK_QUEUE_SIZE, 0, 0};

volatile uint16_t usb_tx_chunk_len = 0;
volatile UsbTxQueueKind usb_tx_active_queue = UsbTxQueueKind::Text;
volatile UsbTxState usb_tx_state = UsbTxState::Idle;
volatile bool usb_tx_service_busy = false;
bool usb_tx_bulk_high_active = false;
bool usb_tx_telemetry_high_active = false;
uint8_t usb_tx_stressed_round_robin = 0;

uint32_t irq_save(void) {
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

void irq_restore(uint32_t primask) {
    if (primask == 0U) {
        __enable_irq();
    }
}

bool is_interrupt_context(void) {
    return (__get_IPSR() != 0U);
}

void usb_tx_reconcile_cdc_state(void) {
    USBD_CDC_HandleTypeDef* hcdc =
        reinterpret_cast<USBD_CDC_HandleTypeDef*>(hUsbDeviceHS.pClassData);

    if (hcdc == nullptr) return;

    const uint32_t primask = irq_save();
    if (usb_tx_state == UsbTxState::InFlight && hcdc->TxState == 0U) {
        usb_tx_state = UsbTxState::Idle;
        usb_tx_chunk_len = 0;
    }
    irq_restore(primask);
}

uint16_t usb_tx_used_nolock(const UsbTxQueue& queue) {
    const uint16_t head = queue.head;
    const uint16_t tail = queue.tail;
    if (head >= tail) return head - tail;
    return queue.size - tail + head;
}

uint16_t usb_tx_free_nolock(const UsbTxQueue& queue) {
    return queue.size - usb_tx_used_nolock(queue) - 1;
}

bool usb_tx_is_empty_nolock(const UsbTxQueue& queue) {
    return queue.head == queue.tail;
}

uint16_t usb_tx_high_watermark(const UsbTxQueue& queue) {
    return static_cast<uint16_t>((static_cast<uint32_t>(queue.size) * USB_TX_HIGH_WATERMARK_PERCENT) / 100U);
}

uint16_t usb_tx_low_watermark(const UsbTxQueue& queue) {
    return static_cast<uint16_t>((static_cast<uint32_t>(queue.size) * USB_TX_LOW_WATERMARK_PERCENT) / 100U);
}

void usb_tx_update_pressure_flags_nolock(void) {
    const uint16_t telemetry_used = usb_tx_used_nolock(usb_tx_telemetry_queue);
    const uint16_t bulk_used = usb_tx_used_nolock(usb_tx_bulk_queue);

    if (telemetry_used >= usb_tx_high_watermark(usb_tx_telemetry_queue)) {
        usb_tx_telemetry_high_active = true;
    } else if (telemetry_used <= usb_tx_low_watermark(usb_tx_telemetry_queue)) {
        usb_tx_telemetry_high_active = false;
    }

    if (bulk_used >= usb_tx_high_watermark(usb_tx_bulk_queue)) {
        usb_tx_bulk_high_active = true;
    } else if (bulk_used <= usb_tx_low_watermark(usb_tx_bulk_queue)) {
        usb_tx_bulk_high_active = false;
    }
}

UsbTxQueue* usb_tx_select_queue_nolock(void) {
    usb_tx_update_pressure_flags_nolock();

    if (!usb_tx_is_empty_nolock(usb_tx_text_queue)) {
        usb_tx_active_queue = UsbTxQueueKind::Text;
        return &usb_tx_text_queue;
    }

    const bool telemetry_waiting = !usb_tx_is_empty_nolock(usb_tx_telemetry_queue);
    const bool bulk_waiting = !usb_tx_is_empty_nolock(usb_tx_bulk_queue);

    if (!telemetry_waiting && !bulk_waiting) return nullptr;

    if (telemetry_waiting && bulk_waiting && usb_tx_telemetry_high_active && usb_tx_bulk_high_active) {
        const bool send_bulk = (usb_tx_stressed_round_robin++ >= 2U);
        if (send_bulk) {
            usb_tx_stressed_round_robin = 0;
            usb_tx_active_queue = UsbTxQueueKind::Bulk;
            return &usb_tx_bulk_queue;
        }
        usb_tx_active_queue = UsbTxQueueKind::Telemetry;
        return &usb_tx_telemetry_queue;
    }

    if (telemetry_waiting && usb_tx_telemetry_high_active) {
        usb_tx_active_queue = UsbTxQueueKind::Telemetry;
        return &usb_tx_telemetry_queue;
    }

    if (bulk_waiting && usb_tx_bulk_high_active) {
        usb_tx_active_queue = UsbTxQueueKind::Bulk;
        return &usb_tx_bulk_queue;
    }

    if (telemetry_waiting) {
        usb_tx_active_queue = UsbTxQueueKind::Telemetry;
        return &usb_tx_telemetry_queue;
    }

    usb_tx_active_queue = UsbTxQueueKind::Bulk;
    return &usb_tx_bulk_queue;
}

bool usb_tx_enqueue(UsbTxQueue& queue, const uint8_t* data, uint16_t length) {
    if (data == nullptr || length == 0) {
        usb_tx_service();
        return true;
    }

    bool queued = false;

    const uint32_t primask = irq_save();
    if (length <= usb_tx_free_nolock(queue)) {
        for (uint16_t i = 0; i < length; i++) {
            queue.buffer[queue.head] = data[i];
            queue.head = (queue.head + 1) % queue.size;
        }
        queued = true;
    }
    irq_restore(primask);

    if (queued && !is_interrupt_context()) usb_tx_service();
    return queued;
}

bool usb_tx_load_next_chunk(void) {
    const uint32_t primask = irq_save();
    if (usb_tx_state != UsbTxState::Idle) {
        irq_restore(primask);
        return false;
    }

    UsbTxQueue* queue = usb_tx_select_queue_nolock();
    if (queue == nullptr) {
        irq_restore(primask);
        return false;
    }

    uint16_t len = 0;
    while (queue->tail != queue->head && len < USB_TX_CHUNK_SIZE) {
        usb_tx_chunk[len++] = queue->buffer[queue->tail];
        queue->tail = (queue->tail + 1) % queue->size;
    }

    usb_tx_chunk_len = len;
    usb_tx_state = UsbTxState::Pending;
    irq_restore(primask);

    return len > 0;
}

void usb_tx_try_submit_pending(void) {
    const uint32_t primask = irq_save();
    if (usb_tx_state != UsbTxState::Pending || usb_tx_chunk_len == 0) {
        irq_restore(primask);
        return;
    }
    const uint16_t len = usb_tx_chunk_len;
    irq_restore(primask);

    if (CDC_Transmit_HS(usb_tx_chunk, len) == USBD_OK) {
        const uint32_t state_primask = irq_save();
        usb_tx_state = UsbTxState::InFlight;
        irq_restore(state_primask);
    }
}

bool usb_tx_queue_text(const char* text) {
    if (text == nullptr) return false;
    return usb_tx_enqueue(usb_tx_text_queue, reinterpret_cast<const uint8_t*>(text), strlen(text));
}

}

void usb_tx_service(void) {
    if (is_interrupt_context()) return;

    usb_tx_reconcile_cdc_state();

    const uint32_t primask = irq_save();
    if (usb_tx_service_busy) {
        irq_restore(primask);
        return;
    }
    usb_tx_service_busy = true;
    irq_restore(primask);

    usb_tx_load_next_chunk();
    usb_tx_try_submit_pending();

    const uint32_t done_primask = irq_save();
    usb_tx_service_busy = false;
    irq_restore(done_primask);
}

extern "C" void usb_tx_onTransmitComplete(void) {
    const uint32_t primask = irq_save();
    if (usb_tx_state == UsbTxState::InFlight) {
        usb_tx_state = UsbTxState::Idle;
        usb_tx_chunk_len = 0;
    }
    irq_restore(primask);
}

bool usb_sendTelemetry(const uint8_t* buffer, uint16_t length) {
    return usb_tx_enqueue(usb_tx_telemetry_queue, buffer, length);
}

bool usb_sendBulk(const uint8_t* buffer, uint16_t length) {
    return usb_tx_enqueue(usb_tx_bulk_queue, buffer, length);
}

void usb_printf(const char *format, ...) {
    char buffer[256];
    va_list args;
    va_start(args, format);

    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (len > 0) {
        if (len >= static_cast<int>(sizeof(buffer))) len = sizeof(buffer) - 1;
        usb_tx_enqueue(usb_tx_text_queue, reinterpret_cast<const uint8_t*>(buffer), static_cast<uint16_t>(len));
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
    static char empty_arg[] = "";
    for (int i = 0; i < MAX_ARGC; i++) {
        argv[i] = empty_arg;
    }
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
                usb_tx_queue_text(cmd_table[i].usage);
                return;
            }
            // Execute the mapped handler
            cmd_table[i].handler(argc, argv);
            return;
        }
    }

    // 3. Command not found
    const char* err = "Unknown command\r\n";
    usb_tx_queue_text(err);
}

void protectionModePrint(void) {
    const char* msg = "\r\nSystem tripped by overcurrent: Motor in protection mode, reset error to start\r\n\r\n";
    usb_tx_queue_text(msg);
}

void batteryProtectionPrint(void) {
    const char* msg = "\r\nSystem operating under battery protection, this function is disabled\r\nIf supplied with a current-limited source, set BATTERY_PROTECTION to false\r\n\r\n";
    usb_tx_queue_text(msg);
}
