/*
 * cobs_transport.c — COBS serial transport for the Test Harness
 */

#include "cobs_transport.h"

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <fcntl.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "esp_log.h"

static const char *TAG = "COBS";

#define COBS_MAX_PAYLOAD  1024
#define COBS_MAX_FRAME    (COBS_MAX_PAYLOAD + 2)  /* overhead byte + delimiter */

/* ── TX mutex ────────────────────────────────────────────────────────────── */
static SemaphoreHandle_t s_tx_mutex = NULL;

/* ── RX accumulator ──────────────────────────────────────────────────────── */
static uint8_t  s_rx_buf[COBS_MAX_FRAME];
static int      s_rx_pos = 0;

/* ── Command handler ─────────────────────────────────────────────────────── */
static harness_cmd_cb_t s_cmd_handler = NULL;

static int s_stdin_fd  = -1;
static int s_stdout_fd = -1;

/* ── COBS encode ─────────────────────────────────────────────────────────── */
static int cobs_encode(const uint8_t *in, int in_len, uint8_t *out)
{
    int  out_pos   = 0;
    int  code_pos  = out_pos++;
    uint8_t code   = 1;

    for (int i = 0; i < in_len; i++) {
        if (in[i] == 0) {
            out[code_pos] = code;
            code_pos = out_pos++;
            code = 1;
        } else {
            out[out_pos++] = in[i];
            code++;
            if (code == 0xFF) {
                out[code_pos] = code;
                code_pos = out_pos++;
                code = 1;
            }
        }
    }
    out[code_pos] = code;
    return out_pos;
}

/* ── COBS decode ─────────────────────────────────────────────────────────── */
static int cobs_decode(const uint8_t *in, int in_len, uint8_t *out)
{
    int out_pos = 0;
    int i = 0;
    while (i < in_len) {
        uint8_t code = in[i++];
        if (code == 0) return -1;
        int count = code - 1;
        if (i + count > in_len) return -1;
        for (int k = 0; k < count; k++)
            out[out_pos++] = in[i++];
        if (code < 0xFF && i < in_len)
            out[out_pos++] = 0;
    }
    return out_pos;
}

/* ── Frame dispatch ──────────────────────────────────────────────────────── */
static void dispatch_frame(const uint8_t *data, int len)
{
    if (len < 1) return;
    uint8_t ch = data[0];
    const uint8_t *payload = data + 1;
    int plen = len - 1;

    if (ch == HARNESS_CH_CMD && s_cmd_handler && plen > 0) {
        char cmd[COBS_MAX_PAYLOAD];
        int clen = plen < (int)(sizeof(cmd) - 1) ? plen : (int)(sizeof(cmd) - 1);
        memcpy(cmd, payload, clen);
        cmd[clen] = '\0';
        s_cmd_handler(cmd);
    }
}

/* ── Public API ──────────────────────────────────────────────────────────── */

void cobs_harness_init(int baud_rate)
{
    s_tx_mutex = xSemaphoreCreateMutex();

    /* Mirror the Controller's CobsTransport.cpp approach exactly:
     * 1. uart_param_config to set baud rate at runtime
     * 2. uart_vfs_dev_port_set_*_line_endings to disable \n->\r\n mangling
     * 3. write(fileno(stdout)) for TX, read(fileno(stdin)) for RX
     * No uart_driver_install needed. */
    uart_config_t cfg = {};
    cfg.baud_rate  = baud_rate;
    cfg.data_bits  = UART_DATA_8_BITS;
    cfg.parity     = UART_PARITY_DISABLE;
    cfg.stop_bits  = UART_STOP_BITS_1;
    cfg.flow_ctrl  = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_DEFAULT;
    uart_param_config(UART_NUM_0, &cfg);

    /* CRITICAL: disable VFS line-ending conversion.
     * Default TX converts \n->\r\n which corrupts binary COBS frames. */
    uart_vfs_dev_port_set_tx_line_endings(0, ESP_LINE_ENDINGS_LF);
    uart_vfs_dev_port_set_rx_line_endings(0, ESP_LINE_ENDINGS_LF);

    s_stdin_fd  = fileno(stdin);
    s_stdout_fd = fileno(stdout);

    /* Non-blocking stdin */
    int flags = fcntl(s_stdin_fd, F_GETFL, 0);
    fcntl(s_stdin_fd, F_SETFL, flags | O_NONBLOCK);

    setvbuf(stdout, NULL, _IONBF, 0);

    /* Sync delimiters so host can resync after boot garbage */
    uint8_t sync[8] = {0};
    write(s_stdout_fd, sync, sizeof(sync));

    cobs_harness_send_fmt(HARNESS_CH_LOG, "COBS harness transport initialized @ %d baud", baud_rate);
    ESP_LOGI(TAG, "COBS transport ready @ %d baud", baud_rate);
}

void cobs_harness_send(uint8_t channel, const uint8_t *payload, int len)
{
    if (len < 0 || len > COBS_MAX_PAYLOAD) return;

    static uint8_t raw[COBS_MAX_PAYLOAD + 1];
    static uint8_t enc[COBS_MAX_FRAME + 4];
    raw[0] = channel;
    if (len > 0) memcpy(raw + 1, payload, len);
    int raw_len = 1 + len;
    int enc_len = cobs_encode(raw, raw_len, enc);
    enc[enc_len++] = 0x00;  /* frame delimiter */

    xSemaphoreTake(s_tx_mutex, portMAX_DELAY);
    write(s_stdout_fd, enc, enc_len);
    xSemaphoreGive(s_tx_mutex);
}

void cobs_harness_send_str(uint8_t channel, const char *str)
{
    cobs_harness_send(channel, (const uint8_t *)str, (int)strlen(str));
}

void cobs_harness_send_fmt(uint8_t channel, const char *fmt, ...)
{
    static char buf[768];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    if (len > 0) {
        if (len >= (int)sizeof(buf)) len = (int)sizeof(buf) - 1;
        cobs_harness_send(channel, (const uint8_t *)buf, len);
    }
}

void cobs_harness_send_tel(const harness_tel_packet_t *pkt)
{
    cobs_harness_send(HARNESS_CH_TEL,
                      (const uint8_t *)pkt,
                      (int)sizeof(harness_tel_packet_t));
}

int cobs_harness_rx_process(void)
{
    uint8_t buf[128];
    int len = read(s_stdin_fd, buf, sizeof(buf));
    if (len <= 0) return 0;

    for (int i = 0; i < len; i++) {
        if (buf[i] == 0x00) {
            if (s_rx_pos > 0) {
                uint8_t decoded[COBS_MAX_FRAME];
                int dec_len = cobs_decode(s_rx_buf, s_rx_pos, decoded);
                if (dec_len > 0)
                    dispatch_frame(decoded, dec_len);
            }
            s_rx_pos = 0;
        } else {
            if (s_rx_pos < (int)sizeof(s_rx_buf))
                s_rx_buf[s_rx_pos++] = buf[i];
            else
                s_rx_pos = 0;  /* overflow — discard */
        }
    }
    return len;
}

void cobs_harness_set_cmd_handler(harness_cmd_cb_t cb)
{
    s_cmd_handler = cb;
}
