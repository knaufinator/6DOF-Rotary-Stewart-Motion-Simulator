#pragma once
/*
 * UdpTransport — network HIL transport to the Voron bridge.
 *
 *   Motion plane : UDP datagrams, one verbatim COBS frame per datagram
 *                  (CH_DATA18 baked OR CH_DATA_RAW float32 — same bytes the
 *                  serial path would WriteFile()). Fire-and-forget.
 *   Control plane: a line-JSON TCP client to the bridge (default :8789),
 *                  reusing the winsock + cJSON style of control_server.cpp.
 *                  Carries set_source / play / demo-file verbs and receives
 *                  status / telemetry / resp / log events.
 *
 * Implements the full ITransport superset so the existing HIL device-card UI
 * and pipeline code drive it unchanged. Serial-specific COBS decode counters
 * map to bridge-relayed frame counts where meaningful, else 0.
 *
 * WSAStartup is owned by App() — this class does not init/teardown winsock.
 */

#include "transport.h"

#include <atomic>
#include <mutex>
#include <thread>
#include <string>
#include <vector>

class UdpTransport : public ITransport {
public:
    // host: bridge address (IP or hostname). udp_port: motion. tcp_port: control.
    UdpTransport(const char* host, int udp_port, int tcp_port);
    ~UdpTransport() override;

    // ── ITransport ───────────────────────────────────────────────────
    bool isOpen() const override { return m_open.load(); }
    void close() override;

    bool write(const uint8_t* data, int len) override;   // raw UDP datagram
    bool sendCommand(const char* cmd) override;          // UDP COBS CH_CMD (fwd-compat)
    bool sendCobsData(const uint32_t raw[6], int bit_depth) override;  // baked 0x06
    bool sendCobsDataRaw(const float raw[6]) override;                 // raw   0x07
    bool sendCobsCommand(const char* cmd) override;

    std::vector<std::string> drainLines() override;
    ESP32Telemetry getLatestTelemetry() const override;

    const char* portName() const override { return m_name; }
    int   rxBytes() const override { return m_rx_bytes.load(); }
    int   txBytes() const override { return m_tx_bytes.load(); }
    int   telemetrySeq() const override { return m_tel_seq.load(); }
    float telemetryRate() const override { return m_tel_rate.load(); }
    int   telemetryRejected() const override { return m_tel_rejected.load(); }

    void setCobsMode(bool v) override { m_cobs_mode = v; }
    bool isCobsMode() const  override { return m_cobs_mode; }
    int  cobsDelimiters() const override { return 0; }   // bridge frames upstream
    int  cobsDecodeOk()   const override { return m_ctrl_msgs.load(); }
    int  cobsDecodeFail() const override { return 0; }
    int  cobsTel()  const override { return m_cobs_tel.load(); }
    int  cobsResp() const override { return m_cobs_resp.load(); }
    int  cobsLog()  const override { return m_cobs_log.load(); }

    Kind kind() const override { return Kind::Network; }

    // ── Network-specific control surface (used by the HIL device card) ──
    bool controlConnected() const { return m_ctrl_connected.load(); }
    // Send a raw line-JSON request to the bridge control plane. Returns false
    // if the control channel is down. `\n` is appended by the transport.
    bool controlSend(const std::string& json_line);

    // Snapshot of the last bridge status body (thread-safe copies).
    std::string source() const;          // "OFF" | "DEMO" | "LIVE" | ""
    std::string bootSource() const;
    std::string playState() const;
    std::string selectedDemo() const;
    long memUsed() const { return m_mem_used.load(); }   // -1 = unknown
    long memFree() const { return m_mem_free.load(); }   // -1 = unknown
    std::vector<std::string> deviceFiles() const;        // on-device demo files

private:
    void ctrlReaderThread();
    void handleControlLine(const char* line);
    void parseStatusBody(void* cjson_obj);   // void* = cJSON* (avoid header leak)

    char m_name[64] = {};
    std::string m_host;
    int m_udp_port = 0;
    int m_tcp_port = 0;

    std::atomic<bool> m_open{false};
    bool m_cobs_mode = true;

    // UDP motion socket
    uintptr_t m_udp_sock = ~(uintptr_t)0;   // SOCKET; INVALID_SOCKET sentinel
    struct sockaddr_storage* m_udp_addr = nullptr;
    int m_udp_addr_len = 0;
    std::mutex m_udp_mutex;

    // TCP control socket + reader
    std::atomic<uintptr_t> m_ctrl_sock{~(uintptr_t)0};
    std::thread m_ctrl_reader;
    std::atomic<bool> m_ctrl_stop{false};
    std::atomic<bool> m_ctrl_connected{false};
    std::mutex m_ctrl_send_mutex;

    // Counters
    std::atomic<int> m_tx_bytes{0};
    std::atomic<int> m_rx_bytes{0};
    std::atomic<int> m_ctrl_msgs{0};
    std::atomic<int> m_cobs_tel{0};
    std::atomic<int> m_cobs_resp{0};
    std::atomic<int> m_cobs_log{0};

    // Telemetry (relayed from bridge CH_TEL events)
    mutable std::mutex m_tel_mutex;
    ESP32Telemetry m_telemetry = {};
    std::atomic<int>   m_tel_seq{0};
    std::atomic<float> m_tel_rate{0.0f};
    std::atomic<int>   m_tel_rejected{0};
    double m_tel_times[16] = {};
    int    m_tel_time_idx = 0;

    // Line queue (resp/log text → app handleHilLine)
    std::mutex m_line_mutex;
    std::vector<std::string> m_line_queue;

    // Bridge status snapshot
    mutable std::mutex m_status_mutex;
    std::string m_source, m_boot_source, m_play_state, m_selected_demo;
    std::atomic<long> m_mem_used{-1};
    std::atomic<long> m_mem_free{-1};
    std::vector<std::string> m_files;
};
