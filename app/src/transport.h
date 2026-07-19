#pragma once
/*
 * ITransport — abstract HIL transport interface.
 *
 * A HIL "device" (Entity::serial) is reached over one of two transports:
 *   - SerialPort  : direct USB COBS to the ESP32 (kind() == Serial)
 *   - UdpTransport: UDP motion + line-JSON TCP control to the Voron bridge (kind() == Network)
 *
 * This is intentionally a SUPERSET interface (not a 5-method sketch): the HIL
 * device card in ui_panels.cpp reads SerialPort-specific stats (byte counts,
 * COBS decode counters, telemetry rate/seq/rejected), so every stat the UI
 * touches is part of the contract. Both transports implement all of it; the
 * network transport surfaces bridge-relayed equivalents.
 *
 * The wire format (COBS frames) is identical for both transports — only the
 * carrier differs (WriteFile vs sendto()).
 */

#include <cstdint>
#include <string>
#include <vector>

// Parsed telemetry from ESP32 (angles + input positions). Shared by all
// transports; defined here so transport.h is the single owner.
struct ESP32Telemetry {
    float  angles[6];     // servo angles (radians) from ESP32 IK
    float  positions[6];  // input positions (arr[] on ESP32)
    double timestamp;     // local time when received
    int    seq;           // increments on each new TEL frame
};

class ITransport {
public:
    enum class Kind { Serial, Network };

    virtual ~ITransport() = default;

    // ── Lifecycle ────────────────────────────────────────────────────
    virtual bool isOpen() const = 0;
    virtual void close() = 0;

    // ── TX ───────────────────────────────────────────────────────────
    virtual bool write(const uint8_t* data, int len) = 0;
    virtual bool sendCommand(const char* cmd) = 0;
    // BAKED motion (0x06): 6x uint24 LE, post-cueing servo-space (legacy path).
    virtual bool sendCobsData(const uint32_t raw[6], int bit_depth) = 0;
    // RAW motion (0x07): 6x float32 LE, PRE-cueing telemetry, app axis order.
    virtual bool sendCobsDataRaw(const float raw[6]) = 0;
    virtual bool sendCobsCommand(const char* cmd) = 0;

    // ── RX ───────────────────────────────────────────────────────────
    virtual std::vector<std::string> drainLines() = 0;
    virtual ESP32Telemetry getLatestTelemetry() const = 0;

    // ── Stats surfaced by the HIL device card ────────────────────────
    virtual const char* portName() const = 0;
    virtual int   rxBytes() const = 0;
    virtual int   txBytes() const = 0;
    virtual int   telemetrySeq() const = 0;
    virtual float telemetryRate() const = 0;
    virtual int   telemetryRejected() const = 0;

    // ── COBS mode + decode counters (device card diagnostics) ────────
    virtual void setCobsMode(bool v) = 0;
    virtual bool isCobsMode() const = 0;
    virtual int  cobsDelimiters() const = 0;  // 0x00 delimiters seen
    virtual int  cobsDecodeOk() const = 0;     // successful decodes
    virtual int  cobsDecodeFail() const = 0;   // failed decodes
    virtual int  cobsTel() const = 0;          // CH_TEL frames
    virtual int  cobsResp() const = 0;         // CH_RESP frames
    virtual int  cobsLog() const = 0;          // CH_LOG frames

    // ── Identity ─────────────────────────────────────────────────────
    virtual Kind kind() const = 0;
};
