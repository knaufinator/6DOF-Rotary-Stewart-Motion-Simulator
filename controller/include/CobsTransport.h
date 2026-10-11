// CobsTransport.h — COBS-framed multiplexed transport for ESP32
// Provides clean channel separation: DATA, CMD, TEL, LOG, RESP
// Self-recovering framing — boot garbage and corruption auto-discard
//
// The console carrier (UART0 or native USB-Serial-JTAG, per sdkconfig) is the primary path.
// Other carriers (Ethernet UDP) decode their own datagrams and hand frames to
// cobs_dispatch_frame() with a source tag, and receive outbound frames through the TX sink.
#ifndef COBS_TRANSPORT_H
#define COBS_TRANSPORT_H

#include <stdint.h>
#include "cobs.h"  // channel constants + codec

#ifdef __cplusplus
extern "C" {
#endif

#define COBS_MAX_FRAME  512

// Frame sources (match InputSource in main.cpp)
#define COBS_SRC_SERIAL    0
#define COBS_SRC_ETHERNET  1
#define COBS_SRC_WIFI      2
#define COBS_SRC_BLE       3

// Initialize COBS transport on the console (UART0 or USB-Serial-JTAG).
// Must be called from app_main() before any other transport I/O.
void cobs_transport_init(int baud_rate);

// Send a COBS-framed message on the given channel.
// Thread-safe (uses internal mutex). Also delivered to the TX sink, if one is set.
void cobs_send(uint8_t channel, const uint8_t *payload, int payload_len);

// Send a null-terminated string on the given channel.
void cobs_send_str(uint8_t channel, const char *str);

// Send a printf-formatted string on the given channel.
void cobs_send_fmt(uint8_t channel, const char *fmt, ...) __attribute__((format(printf, 2, 3)));

// Convenience: send telemetry (12 x float32 packed binary)
void cobs_send_telemetry(const float angles[6], const float positions[6]);

// Read incoming bytes from the console, decode COBS frames, dispatch to handlers.
// Call from a FreeRTOS task loop.
int  cobs_read_process(int timeout_ms);

// Dispatch one already-decoded frame ([channel][payload...]) that arrived on `source`.
// Used by the Ethernet transport; the console path calls it internally.
void cobs_dispatch_frame(const uint8_t *frame, int len, int source);

// Callback types
typedef void (*cobs_data_cb_t)(const uint8_t *payload, int len, int source);
typedef void (*cobs_cmd_cb_t)(const char *cmd, int source);
typedef void (*cobs_tx_sink_t)(const uint8_t *encoded_frame, int len);  // includes trailing 0x00

// Register handlers for incoming DATA and CMD channels
void cobs_set_data_handler(cobs_data_cb_t handler);
void cobs_set_cmd_handler(cobs_cmd_cb_t handler);

// Mirror every outbound frame to a second carrier (e.g. the Ethernet peer). NULL disables.
void cobs_set_tx_sink(cobs_tx_sink_t sink);

// Motion-stream bookkeeping for DATA18_SEQ frames (any source).
typedef struct {
    uint32_t seq_frames;     // sequenced frames received
    uint32_t seq_gaps;       // times the sequence jumped (frames lost or reordered)
    uint32_t seq_lost;       // frames missing, summed over gaps (16-bit wrap aware)
    uint32_t seq_restarts;   // sender sequence restarted at 0 (new session), not counted as loss
    uint32_t last_rx_age_ms; // ms since the last DATA18 / DATA18_SEQ frame (0xFFFFFFFF = never)
    int      last_source;    // COBS_SRC_* of the last motion frame, -1 = none
} cobs_motion_stats_t;

void cobs_motion_stats(cobs_motion_stats_t *out);

#ifdef __cplusplus
}
#endif

#endif // COBS_TRANSPORT_H
