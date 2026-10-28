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
#include "esp_timer.h"
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#endif

static SemaphoreHandle_t s_tx_mutex = NULL;
static cobs_data_cb_t s_data_handler = NULL;
static cobs_cmd_cb_t  s_cmd_handler  = NULL;
static cobs_tx_sink_t s_tx_sink      = NULL;
static int s_stdin_fd = -1;

// motion sequence bookkeeping (DATA18_SEQ)
static volatile uint32_t s_seq_frames = 0, s_seq_gaps = 0, s_seq_lost = 0, s_seq_restarts = 0;
static volatile int64_t  s_motion_last_us = 0;
static volatile int      s_motion_last_src = -1;
static uint16_t s_seq_expect = 0;
static bool     s_seq_valid = false;

// Decoder accumulation buffer
static uint8_t s_rx_acc[COBS_MAX_FRAME];
static int     s_rx_pos = 0;

// ── Init ─────────────────────────────────────────────────────────────

void cobs_transport_init(int baud_rate) {
    // Suppress ESP_LOG globally — raw log text corrupts the COBS stream.
    esp_log_level_set("*", ESP_LOG_NONE);

#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    // Console is the native USB (J11). Install the driver so reads block/poll cleanly and
    // writes are interrupt driven; 4 kB each way covers a 1 kHz motion stream with margin.
    usb_serial_jtag_driver_config_t ucfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    ucfg.rx_buffer_size = 4096;
    ucfg.tx_buffer_size = 4096;
    usb_serial_jtag_driver_install(&ucfg);
    usb_serial_jtag_vfs_use_driver();
    (void)baud_rate;   // USB CDC has no baud rate
#else
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
#endif

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
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    // The VFS write blocks forever once the 4 kB TX ring is full and no host drains the USB
    // port (port closed, cable out). Write through the driver with a short bound instead, and
    // skip the console entirely while no host is attached: the Ethernet mirror still gets it.
    (void)s_out_fd;
    // 0 ticks: write what fits in the 4 kB ring and drop the rest. With a terminal attached the
    // ring drains and nothing is lost; without one, no frame costs more than a few microseconds
    // (a 2 ms bound measured as exactly 2 ms added to every Ethernet reply).
    if (usb_serial_jtag_is_connected())
        usb_serial_jtag_write_bytes(enc, enc_len, 0);
#else
    write(s_out_fd, enc, enc_len);
#endif
    xSemaphoreGive(s_tx_mutex);

    // mirror to the secondary carrier (Ethernet peer), outside the console mutex
    cobs_tx_sink_t sink = s_tx_sink;
    if (sink) sink(enc, enc_len);
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

static inline void note_motion(int source) {
    s_motion_last_us = esp_timer_get_time();
    s_motion_last_src = source;
}

void cobs_dispatch_frame(const uint8_t *data, int len, int source) {
    if (len < 1) return;
    uint8_t ch = data[0];
    const uint8_t *payload = data + 1;
    int plen = len - 1;

    switch (ch) {
        case COBS_CH_DATA18:
            if (s_data_handler && plen >= 18) {
                note_motion(source);
                s_data_handler(payload, plen, source);
            }
            break;
        case COBS_CH_DATA18_SEQ:
            // [uint16 seq LE][18 bytes]: count gaps, then hand over the DATA18 part
            if (s_data_handler && plen >= 20) {
                uint16_t seq = (uint16_t)(payload[0] | (payload[1] << 8));
                if (s_seq_valid && seq != s_seq_expect) {
                    uint16_t d = (uint16_t)(seq - s_seq_expect);
                    if (seq == 0 || d > 32768) { s_seq_restarts++; }   // sender restarted / reordered, not loss
                    else { s_seq_gaps++; s_seq_lost += d; }
                }
                s_seq_expect = (uint16_t)(seq + 1);
                s_seq_valid = true;
                s_seq_frames++;
                note_motion(source);
                s_data_handler(payload + 2, plen - 2, source);
            }
            break;
        case COBS_CH_CMD:
            if (s_cmd_handler && plen > 0) {
                char cmd[256];
                int clen = plen < 255 ? plen : 255;
                memcpy(cmd, payload, clen);
                cmd[clen] = '\0';
                s_cmd_handler(cmd, source);
            }
            break;
        default:
            break;
    }
}

static inline void dispatch_frame(const uint8_t *data, int len) {
    cobs_dispatch_frame(data, len, COBS_SRC_SERIAL);
}

void cobs_motion_stats(cobs_motion_stats_t *out) {
    if (!out) return;
    out->seq_frames = s_seq_frames;
    out->seq_gaps = s_seq_gaps;
    out->seq_lost = s_seq_lost;
    out->seq_restarts = s_seq_restarts;
    out->last_rx_age_ms = s_motion_last_us ? (uint32_t)((esp_timer_get_time() - s_motion_last_us) / 1000) : 0xFFFFFFFFu;
    out->last_source = s_motion_last_src;
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

void cobs_set_data_handler(cobs_data_cb_t handler) { s_data_handler = handler; }
void cobs_set_cmd_handler(cobs_cmd_cb_t handler)   { s_cmd_handler  = handler; }
void cobs_set_tx_sink(cobs_tx_sink_t sink)         { s_tx_sink = sink; }
