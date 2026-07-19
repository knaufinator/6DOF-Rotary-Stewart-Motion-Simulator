// CobsTransport.cpp — COBS-framed multiplexed serial transport for ESP32
// Uses VFS stdin/stdout (the proven working path) for I/O.
// No uart_driver_install — avoids conflicts with ESP-IDF console subsystem.

#include "CobsTransport.h"
#include "cobs.h"

#include <cstring>
#include <cstdarg>
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_vfs.h"
#include "driver/uart.h"
#include "driver/uart_vfs.h"

#define COBS_MAX_FRAME  512

static SemaphoreHandle_t s_tx_mutex = NULL;
static cobs_data_cb_t s_data_handler     = NULL;
static cobs_data_cb_t s_data_raw_handler = NULL;
static cobs_cmd_cb_t  s_cmd_handler      = NULL;
static int s_stdin_fd = -1;

// Decoder accumulation buffer
static uint8_t s_rx_acc[COBS_MAX_FRAME];
static int     s_rx_pos = 0;

// ── Init ─────────────────────────────────────────────────────────────

void cobs_transport_init(int baud_rate) {
    // Suppress ESP_LOG globally — raw log text corrupts the COBS stream.
    esp_log_level_set("*", ESP_LOG_NONE);

    // Set baud rate at runtime (sdkconfig defaults can be wrong after fullclean)
    uart_config_t cfg = {};
    cfg.baud_rate  = baud_rate;
    cfg.data_bits  = UART_DATA_8_BITS;
    cfg.parity     = UART_PARITY_DISABLE;
    cfg.stop_bits  = UART_STOP_BITS_1;
    cfg.flow_ctrl  = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_DEFAULT;
    uart_param_config(UART_NUM_0, &cfg);

    // CRITICAL: Disable VFS line-ending conversion on UART0.
    // Default: TX converts \n→\r\n, RX converts \r\n→\n.
    // This corrupts binary COBS frames containing 0x0A or 0x0D bytes.
    // ESP_LINE_ENDINGS_LF = pass-through (no conversion).
    uart_vfs_dev_port_set_tx_line_endings(0, ESP_LINE_ENDINGS_LF);
    uart_vfs_dev_port_set_rx_line_endings(0, ESP_LINE_ENDINGS_LF);

    // TX mutex for thread-safe frame writes
    s_tx_mutex = xSemaphoreCreateMutex();

    // Set stdin to non-blocking (same as original InterfaceMonitorTask)
    s_stdin_fd = fileno(stdin);
    int flags = fcntl(s_stdin_fd, F_GETFL, 0);
    fcntl(s_stdin_fd, F_SETFL, flags | O_NONBLOCK);

    // Make stdout unbuffered — we use direct write() for COBS frames
    setvbuf(stdout, NULL, _IONBF, 0);

    // Cache stdout fd for direct write() calls
    static int s_stdout_fd = -1;
    s_stdout_fd = fileno(stdout);

    // Sync delimiters so receiver can resynchronize after boot garbage
    uint8_t sync[8] = {0};
    write(s_stdout_fd, sync, sizeof(sync));

    // Send init message as a proper COBS LOG frame
    cobs_send_fmt(COBS_CH_LOG, "COBS transport initialized");
}

// ── Send (via VFS stdout — proven working path) ─────────────────────

void cobs_send(uint8_t channel, const uint8_t *payload, int payload_len) {
    if (payload_len < 0 || payload_len > COBS_MAX_FRAME - 1) return;

    // Build raw frame: [channel] [payload...]
    uint8_t raw[COBS_MAX_FRAME];
    raw[0] = channel;
    if (payload_len > 0)
        memcpy(raw + 1, payload, payload_len);
    int raw_len = 1 + payload_len;

    // COBS-encode (output has no zeros)
    uint8_t enc[COBS_MAX_FRAME + 16];
    int enc_len = cobs_encode(raw, raw_len, enc);
    enc[enc_len] = 0x00;  // frame delimiter
    enc_len++;

    // Direct write() bypasses newlib stdio buffering — single VFS call
    static int s_out_fd = -1;
    if (s_out_fd < 0) s_out_fd = fileno(stdout);

    xSemaphoreTake(s_tx_mutex, portMAX_DELAY);
    write(s_out_fd, enc, enc_len);
    xSemaphoreGive(s_tx_mutex);
}

void cobs_send_str(uint8_t channel, const char *str) {
    cobs_send(channel, (const uint8_t *)str, (int)strlen(str));
}

void cobs_send_fmt(uint8_t channel, const char *fmt, ...) {
    char buf[256];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    if (len > 0) {
        if (len >= (int)sizeof(buf)) len = sizeof(buf) - 1;
        cobs_send(channel, (const uint8_t *)buf, len);
    }
}

void cobs_send_telemetry(const float angles[6], const float positions[6]) {
    // Binary telemetry: 12 x float32 LE = 48 bytes
    uint8_t buf[48];
    memcpy(buf, angles, 24);
    memcpy(buf + 24, positions, 24);
    cobs_send(COBS_CH_TEL, buf, 48);
}

// ── Receive & Dispatch (via VFS stdin — proven working path) ────────

static void dispatch_frame(const uint8_t *data, int len) {
    if (len < 1) return;
    uint8_t ch = data[0];
    const uint8_t *payload = data + 1;
    int plen = len - 1;

    switch (ch) {
        case COBS_CH_DATA:
            if (s_data_handler && plen >= 12)
                s_data_handler(payload, plen);
            break;
        case COBS_CH_DATA_RAW:
            if (s_data_raw_handler && plen >= 24)
                s_data_raw_handler(payload, plen);
            break;
        case COBS_CH_CMD:
            if (s_cmd_handler && plen > 0) {
                char cmd[256];
                int clen = plen < 255 ? plen : 255;
                memcpy(cmd, payload, clen);
                cmd[clen] = '\0';
                s_cmd_handler(cmd);
            }
            break;
        default:
            break;
    }
}

int cobs_read_process(int timeout_ms) {
    (void)timeout_ms;
    uint8_t buf[128];
    int len = read(s_stdin_fd, buf, sizeof(buf));
    if (len <= 0) return 0;

    for (int i = 0; i < len; i++) {
        if (buf[i] == 0x00) {
            if (s_rx_pos > 0) {
                uint8_t decoded[COBS_MAX_FRAME];
                int dec_len = cobs_decode(s_rx_acc, s_rx_pos, decoded);
                if (dec_len > 0)
                    dispatch_frame(decoded, dec_len);
            }
            s_rx_pos = 0;
        } else {
            if (s_rx_pos < COBS_MAX_FRAME)
                s_rx_acc[s_rx_pos++] = buf[i];
            else
                s_rx_pos = 0;
        }
    }
    return len;
}

void cobs_set_data_handler(cobs_data_cb_t handler)     { s_data_handler     = handler; }
void cobs_set_data_raw_handler(cobs_data_cb_t handler) { s_data_raw_handler = handler; }
void cobs_set_cmd_handler(cobs_cmd_cb_t handler)       { s_cmd_handler      = handler; }
