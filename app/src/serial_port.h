#pragma once
/*
 * Win32 Serial Port — COM port communication for HIL entities
 *
 * Features:
 *   - COM port enumeration via SetupAPI
 *   - Async reader thread (parses ASCII lines from ESP32)
 *   - Binary motion packet writer (0xAA 0x55 framing)
 *   - Telemetry parsing (TEL,a1..a6,p1..p6 lines)
 */

#include <string>
#include <vector>
#include <functional>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>

#include "dev_log.h"
#include "cobs.h"

// Parsed telemetry from ESP32
struct ESP32Telemetry {
    float angles[6];     // servo angles (radians) from ESP32 IK
    float positions[6];  // input positions (arr[] on ESP32)
    double timestamp;    // local time when received
    int seq;             // increments on each new TEL line
};

// A detected COM port
struct ComPortInfo {
    std::string port;    // e.g. "COM3"
    std::string desc;    // e.g. "USB Serial Device (COM3)"
};

class SerialPort {
public:
    SerialPort();
    ~SerialPort();

    // Enumerate available COM ports on the system
    static std::vector<ComPortInfo> enumerate();

    // Open/close
    bool open(const char* port, int baud = 115200);
    void close();
    bool isOpen() const { return m_open.load(); }

    // Write raw bytes
    bool write(const uint8_t* data, int len);

    // Send a binary motion packet (0xAA 0x55 framing, 6 × uint16 LE + XOR checksum)
    bool sendMotionPacket(const uint16_t raw[6]);

    // Send a CSV motion packet: "<v0>,<v1>,<v2>,<v3>,<v4>,<v5>X" (Mini-6DOF / legacy)
    bool sendMotionCSV(const uint16_t raw[6]);

    // Send a text command (appends 'X' terminator for ESP32 ASCII protocol)
    bool sendCommand(const char* cmd);

    // COBS-framed sends (used when m_cobs_mode is true)
    bool sendCobsData(const uint16_t raw[6]);    // 12-byte motion data on CH_DATA
    bool sendCobsCommand(const char* cmd);        // string command on CH_CMD

    // Enable COBS framing mode (binary protocol uses this)
    void setCobsMode(bool v) { m_cobs_mode = v; }
    bool isCobsMode() const  { return m_cobs_mode; }

    // Latest telemetry (thread-safe read)
    ESP32Telemetry getLatestTelemetry() const;

    // Connection info
    const char* portName() const { return m_port_name; }
    int rxBytes() const { return m_rx_bytes.load(); }
    int txBytes() const { return m_tx_bytes.load(); }
    int telemetrySeq() const { return m_telemetry.seq; }
    float telemetryRate() const { return m_tel_rate.load(); }
    int telemetryRejected() const { return m_tel_rejected.load(); }

    // Line callback (optional — for logging raw lines to console)
    using LineCallback = std::function<void(const char* line)>;
    void setLineCallback(LineCallback cb) { m_line_cb = cb; }

    // When true, ALL non-TEL lines are enqueued (no rate limiting).
    // Used by test harness serial which needs every JSON response.
    void setEnqueueAll(bool v) { m_enqueue_all.store(v); }

    // Thread-safe line queue: reader thread pushes, main thread drains.
    // This replaces direct callback invocation to avoid data races.
    std::vector<std::string> drainLines();

private:
    void readerThread();
    void writerThread();
    void parseLine(const char* line);
    bool rawWrite(const uint8_t* data, int len);  // actual blocking WriteFile

    void* m_handle;  // HANDLE on Windows (INVALID_HANDLE_VALUE = -1)
    char m_port_name[32] = {};
    std::atomic<bool> m_open{false};
    std::thread m_reader;
    std::thread m_writer;
    std::atomic<bool> m_reader_stop{false};
    std::atomic<bool> m_writer_stop{false};

    // Async write queue
    std::mutex m_write_mutex;
    std::condition_variable m_write_cv;
    std::vector<uint8_t> m_write_pending;  // main thread appends here

    mutable std::mutex m_tel_mutex;
    ESP32Telemetry m_telemetry = {};

    std::atomic<int> m_rx_bytes{0};
    std::atomic<int> m_tx_bytes{0};
    std::atomic<float> m_tel_rate{0.0f};
    std::atomic<int> m_tel_rejected{0};

    // Telemetry rate tracking
    double m_tel_times[16] = {};
    int m_tel_time_idx = 0;

    LineCallback m_line_cb;
    double m_last_line_cb_time = 0.0;
    std::atomic<bool> m_enqueue_all{false};

    // Thread-safe line queue (reader pushes, main thread drains)
    std::mutex m_line_queue_mutex;
    std::vector<std::string> m_line_queue;

    // COBS mode
    bool m_cobs_mode = false;
public:
    std::atomic<int> m_cobs_delimiters{0};  // 0x00 bytes seen
    std::atomic<int> m_cobs_decode_ok{0};   // successful decodes
    std::atomic<int> m_cobs_decode_fail{0}; // failed decodes
    std::atomic<int> m_cobs_tel{0};         // TEL frames
    std::atomic<int> m_cobs_resp{0};        // RESP frames
    std::atomic<int> m_cobs_log{0};         // LOG frames
private:
    uint8_t m_cobs_acc[512];  // COBS accumulation buffer
    int m_cobs_pos = 0;
    void processCobsFrame(const uint8_t *data, int len);
    void parseBinaryTelemetry(const uint8_t *payload, int len);
};
