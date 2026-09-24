/*
 * cobs_transport.h — COBS serial transport for the Test Harness
 *
 * Mirrors the Controller's COBS framing so both ends speak the same protocol.
 * Pure C, IRAM-safe, no dynamic allocation.
 *
 * Channel IDs (shared with Controller cobs.h):
 *   CH_CMD  = 0x02  — ASCII command string (host → harness)
 *   CH_LOG  = 0x04  — Human-readable log line (harness → host)
 *   CH_RESP = 0x05  — ASCII command response (harness → host)
 *   CH_TEL  = 0x10  — Binary telemetry packet (harness → host, 10 Hz)
 *
 * Telemetry packet layout (CH_TEL payload, 52 bytes):
 *   uint32_t t_ms          — uptime in ms
 *   per motor × 6:
 *     int32_t  pos         — net position (steps, signed)
 *     uint32_t steps       — total pulse count since last RESET
 *     int16_t  rate_hz     — step rate in Hz (0–32767)
 *     uint8_t  dir         — current direction pin level
 *     uint8_t  pad         — reserved
 *   Total: 4 + 6×(4+4+2+1+1) = 4 + 72 = 76 bytes
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

/* ── Channel IDs ─────────────────────────────────────────────────────────── */
#define HARNESS_CH_CMD   0x02
#define HARNESS_CH_LOG   0x04
#define HARNESS_CH_RESP  0x05
#define HARNESS_CH_TEL   0x10

/* ── Telemetry packet (packed, little-endian) ────────────────────────────── */
typedef struct __attribute__((packed)) {
    int32_t  pos;       /* net position */
    uint32_t steps;     /* total pulses */
    int16_t  rate_hz;   /* step rate Hz */
    uint8_t  dir;       /* direction pin level */
    uint8_t  pad;       /* reserved */
} harness_motor_tel_t;

typedef struct __attribute__((packed)) {
    uint32_t           t_ms;
    harness_motor_tel_t motors[6];
} harness_tel_packet_t;

/* ── API ─────────────────────────────────────────────────────────────────── */

/* Initialize COBS transport on UART0 at given baud rate.
 * Must be called before any cobs_harness_send* calls. */
void cobs_harness_init(int baud_rate);

/* Send a COBS-framed packet on the given channel. */
void cobs_harness_send(uint8_t channel, const uint8_t *payload, int len);

/* Convenience: send a NUL-terminated string on a channel. */
void cobs_harness_send_str(uint8_t channel, const char *str);

/* Convenience: printf-style send on a channel (max 255 bytes). */
void cobs_harness_send_fmt(uint8_t channel, const char *fmt, ...)
    __attribute__((format(printf, 2, 3)));

/* Send a binary telemetry packet. */
void cobs_harness_send_tel(const harness_tel_packet_t *pkt);

/* Process pending RX bytes. Call frequently from serial_task.
 * Invokes the registered command handler for each complete CMD frame.
 * Returns number of bytes consumed. */
int cobs_harness_rx_process(void);

/* Register the command handler (called with NUL-terminated ASCII string). */
typedef void (*harness_cmd_cb_t)(const char *cmd);
void cobs_harness_set_cmd_handler(harness_cmd_cb_t cb);
