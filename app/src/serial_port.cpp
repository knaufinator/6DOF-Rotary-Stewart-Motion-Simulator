#include "serial_port.h"

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <chrono>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <setupapi.h>
#include <devguid.h>
#include <initguid.h>
// GUID_DEVINTERFACE_COMPORT
DEFINE_GUID(GUID_DEVINTERFACE_COMPORT_LOCAL,
    0x86E0D1E0, 0x8089, 0x11D0, 0x9C, 0xE4,
    0x08, 0x00, 0x3E, 0x30, 0x1F, 0x73);
// Helper: cast void* m_handle to HANDLE
#define H() ((HANDLE)m_handle)
#define SET_H(v) (m_handle = (void*)(v))
#endif

// ── Helpers ──────────────────────────────────────────────────────────

static double now_seconds() {
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}

// ── Constructor / Destructor ─────────────────────────────────────────

SerialPort::SerialPort() : m_handle((void*)(intptr_t)-1) {}

SerialPort::~SerialPort() {
    close();
}

// ── COM Port Enumeration ─────────────────────────────────────────────

std::vector<ComPortInfo> SerialPort::enumerate() {
    std::vector<ComPortInfo> ports;

#ifdef _WIN32
    HDEVINFO devInfo = SetupDiGetClassDevs(
        &GUID_DEVINTERFACE_COMPORT_LOCAL, nullptr, nullptr,
        DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);

    if (devInfo == INVALID_HANDLE_VALUE) {
        // Fallback: try brute-force COM1..COM32
        for (int i = 1; i <= 32; i++) {
            char name[16];
            snprintf(name, sizeof(name), "COM%d", i);
            char path[32];
            snprintf(path, sizeof(path), "\\\\.\\%s", name);
            HANDLE h = CreateFileA(path, GENERIC_READ | GENERIC_WRITE, 0,
                nullptr, OPEN_EXISTING, 0, nullptr);
            if (h != INVALID_HANDLE_VALUE) {
                CloseHandle(h);
                ports.push_back({name, name});
            }
        }
        return ports;
    }

    SP_DEVINFO_DATA devInfoData = {};
    devInfoData.cbSize = sizeof(devInfoData);

    for (DWORD i = 0; SetupDiEnumDeviceInfo(devInfo, i, &devInfoData); i++) {
        // Get friendly name
        char friendly[256] = {};
        SetupDiGetDeviceRegistryPropertyA(devInfo, &devInfoData,
            SPDRP_FRIENDLYNAME, nullptr, (PBYTE)friendly, sizeof(friendly), nullptr);

        // Get port name from registry
        HKEY hKey = SetupDiOpenDevRegKey(devInfo, &devInfoData,
            DICS_FLAG_GLOBAL, 0, DIREG_DEV, KEY_READ);
        if (hKey != INVALID_HANDLE_VALUE) {
            char portName[32] = {};
            DWORD sz = sizeof(portName);
            DWORD type = 0;
            if (RegQueryValueExA(hKey, "PortName", nullptr, &type,
                    (LPBYTE)portName, &sz) == ERROR_SUCCESS) {
                // Only include COM ports (not LPT)
                if (strncmp(portName, "COM", 3) == 0) {
                    ComPortInfo info;
                    info.port = portName;
                    info.desc = friendly[0] ? friendly : portName;
                    ports.push_back(info);
                }
            }
            RegCloseKey(hKey);
        }
    }

    SetupDiDestroyDeviceInfoList(devInfo);
#endif

    return ports;
}

// ── Open / Close ─────────────────────────────────────────────────────

bool SerialPort::open(const char* port, int baud) {
    if (m_open.load()) close();

    snprintf(m_port_name, sizeof(m_port_name), "%s", port);

#ifdef _WIN32
    char path[64];
    snprintf(path, sizeof(path), "\\\\.\\%s", port);

    SET_H(CreateFileA(path, GENERIC_READ | GENERIC_WRITE, 0,
        nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr));

    if (H() == INVALID_HANDLE_VALUE) {
        return false;
    }

    // Configure serial parameters
    DCB dcb = {};
    dcb.DCBlength = sizeof(dcb);
    if (!GetCommState(H(), &dcb)) {
        CloseHandle(H());
        SET_H(INVALID_HANDLE_VALUE);
        return false;
    }

    dcb.BaudRate = baud;
    dcb.ByteSize = 8;
    dcb.StopBits = ONESTOPBIT;
    dcb.Parity   = NOPARITY;
    dcb.fDtrControl = DTR_CONTROL_DISABLE;  // Don't toggle DTR — prevents ESP32 auto-reset on connect
    dcb.fRtsControl = RTS_CONTROL_DISABLE;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fBinary = TRUE;

    if (!SetCommState(H(), &dcb)) {
        CloseHandle(H());
        SET_H(INVALID_HANDLE_VALUE);
        return false;
    }

    // Set timeouts: very short read timeout so writer thread can interleave
    // (synchronous handle — read and write serialize, so read must release fast)
    COMMTIMEOUTS timeouts = {};
    timeouts.ReadIntervalTimeout = MAXDWORD;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.ReadTotalTimeoutConstant = 1;
    timeouts.WriteTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant = 50;
    SetCommTimeouts(H(), &timeouts);

    // Purge buffers
    PurgeComm(H(), PURGE_RXCLEAR | PURGE_TXCLEAR);

    m_open.store(true);
    m_rx_bytes.store(0);
    m_tx_bytes.store(0);
    m_tel_rate.store(0.0f);
    m_tel_time_idx = 0;
    memset(&m_telemetry, 0, sizeof(m_telemetry));

    // Start reader thread
    m_reader_stop.store(false);
    m_reader = std::thread(&SerialPort::readerThread, this);

    // Start writer thread
    m_writer_stop.store(false);
    m_write_pending.clear();
    m_write_pending.reserve(256);
    m_writer = std::thread(&SerialPort::writerThread, this);

    return true;
#else
    return false;
#endif
}

void SerialPort::close() {
    m_open.store(false);
    m_reader_stop.store(true);
    m_writer_stop.store(true);
    m_write_cv.notify_all();
    if (m_reader.joinable()) m_reader.join();
    if (m_writer.joinable()) m_writer.join();

#ifdef _WIN32
    if (H() != INVALID_HANDLE_VALUE) {
        CloseHandle(H());
        SET_H(INVALID_HANDLE_VALUE);
    }
#endif
}

// ── Write (async queue — never blocks caller) ─────────────────────────────────

bool SerialPort::write(const uint8_t* data, int len) {
    if (!m_open.load()) return false;
    {
        std::lock_guard<std::mutex> lock(m_write_mutex);
        m_write_pending.insert(m_write_pending.end(), data, data + len);
    }
    m_write_cv.notify_one();
    return true;
}

bool SerialPort::rawWrite(const uint8_t* data, int len) {
    if (!m_open.load()) return false;
#ifdef _WIN32
    if (H() == INVALID_HANDLE_VALUE) { m_open.store(false); return false; }
    DWORD written = 0;
    if (!WriteFile(H(), data, len, &written, nullptr)) {
        m_open.store(false);
        return false;
    }
    m_tx_bytes.fetch_add((int)written);
    return (int)written == len;
#else
    return false;
#endif
}

void SerialPort::writerThread() {
    std::vector<uint8_t> buf;
    buf.reserve(256);
    while (!m_writer_stop.load()) {
        {
            std::unique_lock<std::mutex> lock(m_write_mutex);
            m_write_cv.wait_for(lock, std::chrono::milliseconds(50),
                [this]{ return !m_write_pending.empty() || m_writer_stop.load(); });
            if (m_writer_stop.load()) break;
            buf.swap(m_write_pending);
        }
        if (!buf.empty()) {
            rawWrite(buf.data(), (int)buf.size());
            buf.clear();
        }
    }
}

bool SerialPort::sendMotionPacket(const uint16_t raw[6]) {
    // Binary protocol: [0xAA][0x55][uint16_t × 6 LE][XOR checksum] = 15 bytes
    uint8_t pkt[15];
    pkt[0] = 0xAA;
    pkt[1] = 0x55;

    uint8_t xor_check = 0;
    for (int i = 0; i < 6; i++) {
        pkt[2 + i * 2]     = (uint8_t)(raw[i] & 0xFF);
        pkt[2 + i * 2 + 1] = (uint8_t)((raw[i] >> 8) & 0xFF);
        xor_check ^= pkt[2 + i * 2];
        xor_check ^= pkt[2 + i * 2 + 1];
    }
    pkt[14] = xor_check;

    return write(pkt, 15);
}

bool SerialPort::sendMotionCSV(const uint16_t raw[6]) {
    // CSV protocol: "<v0>,<v1>,<v2>,<v3>,<v4>,<v5>X" (Mini-6DOF / legacy)
    char buf[64];
    int n = snprintf(buf, sizeof(buf), "%u,%u,%u,%u,%u,%uX",
                     raw[0], raw[1], raw[2], raw[3], raw[4], raw[5]);
    return write((const uint8_t*)buf, n);
}

bool SerialPort::sendCommand(const char* cmd) {
    if (!m_open.load()) return false;
    int len = (int)strlen(cmd);
    // Send command + 'X' terminator (ESP32 ASCII protocol)
    uint8_t buf[256];
    if (len + 1 > (int)sizeof(buf)) return false;
    memcpy(buf, cmd, len);
    buf[len] = 'X';
    return write(buf, len + 1);
}

// ── Telemetry Access ─────────────────────────────────────────────────

ESP32Telemetry SerialPort::getLatestTelemetry() const {
    std::lock_guard<std::mutex> lock(m_tel_mutex);
    return m_telemetry;
}

// ── Reader Thread ────────────────────────────────────────────────────

void SerialPort::readerThread() {
#ifdef _WIN32
    char line_buf[512];
    int line_pos = 0;

    while (!m_reader_stop.load()) {
        uint8_t buf[256];
        DWORD bytesRead = 0;

        if (!ReadFile(H(), buf, sizeof(buf), &bytesRead, nullptr)) {
            // Read error — port may have been disconnected
            if (GetLastError() != ERROR_TIMEOUT) {
                m_open.store(false);
                break;
            }
            continue;
        }

        if (bytesRead == 0) {
            Sleep(1);  // yield CPU when no data available
            continue;
        }
        m_rx_bytes.fetch_add((int)bytesRead);

        // Parse incoming bytes into lines
        for (DWORD i = 0; i < bytesRead; i++) {
            char c = (char)buf[i];
            if (c == '\n' || c == '\r') {
                if (line_pos > 0) {
                    line_buf[line_pos] = '\0';
                    // Telemetry parsing is thread-safe (uses m_tel_mutex)
                    parseLine(line_buf);
                    // Queue non-TEL lines for main-thread processing.
                    // TEL lines are high-frequency and already handled by parseLine.
                    bool is_tel = (strncmp(line_buf, "TEL,", 4) == 0);
                    if (!is_tel) {
                        bool is_handshake = (strncmp(line_buf, "FINGERPRINT:", 12) == 0 ||
                                             strncmp(line_buf, "CONFIG:", 7) == 0 ||
                                             strncmp(line_buf, "SERVO:", 6) == 0 ||
                                             strncmp(line_buf, "BITS:", 5) == 0 ||
                                             strncmp(line_buf, "VERSION:", 8) == 0);
                        bool enqueue = is_handshake;
                        if (!is_handshake) {
                            double t = now_seconds();
                            if (t - m_last_line_cb_time >= 0.1) {
                                m_last_line_cb_time = t;
                                enqueue = true;
                            }
                        }
                        if (enqueue) {
                            std::lock_guard<std::mutex> lock(m_line_queue_mutex);
                            m_line_queue.emplace_back(line_buf);
                        }
                    }
                    line_pos = 0;
                }
            } else {
                if (line_pos < (int)sizeof(line_buf) - 1) {
                    line_buf[line_pos++] = c;
                }
            }
        }
    }
#endif
}

std::vector<std::string> SerialPort::drainLines() {
    std::lock_guard<std::mutex> lock(m_line_queue_mutex);
    std::vector<std::string> out;
    out.swap(m_line_queue);
    return out;
}

void SerialPort::parseLine(const char* line) {
    // Parse telemetry: TEL,a1,a2,a3,a4,a5,a6,p1,p2,p3,p4,p5,p6
    if (strncmp(line, "TEL,", 4) == 0) {
        float vals[12] = {};
        int n = sscanf(line + 4,
            "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
            &vals[0], &vals[1], &vals[2], &vals[3], &vals[4], &vals[5],
            &vals[6], &vals[7], &vals[8], &vals[9], &vals[10], &vals[11]);

        if (n >= 6) {
            double t = now_seconds();
            std::lock_guard<std::mutex> lock(m_tel_mutex);
            memcpy(m_telemetry.angles, vals, 6 * sizeof(float));
            if (n >= 12) memcpy(m_telemetry.positions, vals + 6, 6 * sizeof(float));
            m_telemetry.timestamp = t;
            m_telemetry.seq++;

            // Update telemetry rate
            m_tel_times[m_tel_time_idx % 16] = t;
            m_tel_time_idx++;
            if (m_tel_time_idx >= 2) {
                int oldest = (m_tel_time_idx >= 16) ? m_tel_time_idx - 16 : 0;
                double dt = t - m_tel_times[oldest % 16];
                int count = m_tel_time_idx - oldest;
                if (dt > 0.001 && count > 1)
                    m_tel_rate.store((float)((count - 1) / dt));
            }
        }
    }
}
