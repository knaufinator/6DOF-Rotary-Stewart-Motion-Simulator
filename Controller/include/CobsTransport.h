// CobsTransport.h — COBS-framed multiplexed serial transport for ESP32
// Provides clean channel separation: DATA, CMD, TEL, LOG, RESP
// Self-recovering framing — boot garbage and corruption auto-discard
#ifndef COBS_TRANSPORT_H
#define COBS_TRANSPORT_H

#include <stdint.h>
#include "cobs.h"  // channel constants + codec

#ifdef __cplusplus
extern "C" {
#endif

// Initialize COBS transport on UART0.
// Installs UART driver, redirects ESP-IDF logs, sends sync delimiters.
// Must be called from app_main() before any other transport I/O.
void cobs_transport_init(int baud_rate);

// Send a COBS-framed message on the given channel.
// Thread-safe (uses internal mutex).
void cobs_send(uint8_t channel, const uint8_t *payload, int payload_len);

// Send a null-terminated string on the given channel.
void cobs_send_str(uint8_t channel, const char *str);

// Send a printf-formatted string on the given channel.
void cobs_send_fmt(uint8_t channel, const char *fmt, ...) __attribute__((format(printf, 2, 3)));

// Convenience: send telemetry (12 x float32 packed binary)
void cobs_send_telemetry(const float angles[6], const float positions[6]);

// Read incoming bytes from UART, decode COBS frames, dispatch to handlers.
// Blocks up to timeout_ms. Call from a FreeRTOS task loop.
int  cobs_read_process(int timeout_ms);

// Callback types
typedef void (*cobs_data_cb_t)(const uint8_t *payload, int len);
typedef void (*cobs_cmd_cb_t)(const char *cmd);

// Register handlers for incoming DATA and CMD channels
void cobs_set_data_handler(cobs_data_cb_t handler);
void cobs_set_cmd_handler(cobs_cmd_cb_t handler);

#ifdef __cplusplus
}
#endif

#endif // COBS_TRANSPORT_H
