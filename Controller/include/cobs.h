// cobs.h — Consistent Overhead Byte Stuffing (COBS) codec
// Header-only, zero-dependency, works on ESP32 and desktop
// Reference: Cheshire & Baker, "Consistent Overhead Byte Stuffing", IEEE/ACM 1999
#ifndef COBS_H
#define COBS_H

#include <stdint.h>

// Channel IDs for multiplexed COBS transport
#define COBS_CH_DATA  0x01  // App->ESP: motion data (12 bytes: 6x uint16 LE)
#define COBS_CH_CMD   0x02  // App->ESP: ASCII command string
#define COBS_CH_TEL   0x03  // ESP->App: telemetry (48 bytes: 12x float32 LE)
#define COBS_CH_LOG   0x04  // ESP->App: log/debug text
#define COBS_CH_RESP  0x05  // ESP->App: command response text

// Max COBS overhead: 1 byte per 254 input bytes + 1
#define COBS_MAX_ENC_SIZE(n) ((n) + ((n) / 254) + 1)

// Encode `in_len` bytes from `in` into `out` (which must hold COBS_MAX_ENC_SIZE(in_len) bytes).
// Returns encoded length (no trailing 0x00 delimiter — caller appends it).
static inline int cobs_encode(const uint8_t *in, int in_len, uint8_t *out) {
    int ri = 0, wi = 1;
    int ci = 0;           // index of current code byte
    uint8_t code = 1;

    while (ri < in_len) {
        if (in[ri] == 0) {
            out[ci] = code;
            ci = wi++;
            code = 1;
        } else {
            out[wi++] = in[ri];
            if (++code == 0xFF) {
                out[ci] = code;
                ci = wi++;
                code = 1;
            }
        }
        ri++;
    }
    out[ci] = code;
    return wi;
}

// Decode `in_len` bytes (between 0x00 delimiters, no zeros) into `out`.
// Returns decoded length, or -1 on framing error.
static inline int cobs_decode(const uint8_t *in, int in_len, uint8_t *out) {
    if (in_len == 0) return 0;
    int ri = 0, wi = 0;
    while (ri < in_len) {
        uint8_t code = in[ri++];
        if (code == 0) return -1;
        int count = code - 1;
        if (ri + count > in_len) return -1;
        for (int i = 0; i < count; i++)
            out[wi++] = in[ri++];
        if (code < 0xFF && ri < in_len)
            out[wi++] = 0;
    }
    return wi;
}

#endif // COBS_H
