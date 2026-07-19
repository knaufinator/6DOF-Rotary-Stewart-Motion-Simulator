#include "udp_transport.h"

#ifdef _WIN32
#  define WIN32_LEAN_AND_MEAN
#  include <winsock2.h>
#  include <ws2tcpip.h>
#  pragma comment(lib, "ws2_32.lib")
#endif

#include "cobs.h"
#include "cJSON.h"

#include <cstring>
#include <cstdio>
#include <cstdint>
#include <chrono>

// ── helpers ─────────────────────────────────────────────────────────
static double now_seconds() {
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}

#ifdef _WIN32
static const uintptr_t kInvalidSock = (uintptr_t)INVALID_SOCKET;
#else
static const uintptr_t kInvalidSock = ~(uintptr_t)0;
#endif

// ── Construction ────────────────────────────────────────────────────
UdpTransport::UdpTransport(const char* host, int udp_port, int tcp_port)
    : m_host(host ? host : ""), m_udp_port(udp_port), m_tcp_port(tcp_port) {
    snprintf(m_name, sizeof(m_name), "%s:%d", m_host.c_str(), udp_port);
    m_udp_sock = kInvalidSock;

#ifdef _WIN32
    // Resolve host once for the UDP motion datagrams.
    char portstr[16];
    snprintf(portstr, sizeof(portstr), "%d", udp_port);
    addrinfo hints = {}; hints.ai_family = AF_INET; hints.ai_socktype = SOCK_DGRAM;
    addrinfo* res = nullptr;
    if (getaddrinfo(m_host.c_str(), portstr, &hints, &res) == 0 && res) {
        SOCKET us = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
        if (us != INVALID_SOCKET) {
            m_udp_sock = (uintptr_t)us;
            m_udp_addr = new sockaddr_storage();
            memset(m_udp_addr, 0, sizeof(sockaddr_storage));
            memcpy(m_udp_addr, res->ai_addr, res->ai_addrlen);
            m_udp_addr_len = (int)res->ai_addrlen;
            m_open.store(true);
        }
        freeaddrinfo(res);
    }

    // Best-effort non-blocking connect for the TCP control plane (<=1.5s).
    {
        char tportstr[16];
        snprintf(tportstr, sizeof(tportstr), "%d", tcp_port);
        addrinfo thints = {}; thints.ai_family = AF_INET; thints.ai_socktype = SOCK_STREAM;
        addrinfo* tres = nullptr;
        if (getaddrinfo(m_host.c_str(), tportstr, &thints, &tres) == 0 && tres) {
            SOCKET cs = socket(tres->ai_family, tres->ai_socktype, tres->ai_protocol);
            if (cs != INVALID_SOCKET) {
                u_long nb = 1; ioctlsocket(cs, FIONBIO, &nb);
                connect(cs, tres->ai_addr, (int)tres->ai_addrlen);
                fd_set wf; FD_ZERO(&wf); FD_SET(cs, &wf);
                timeval tv; tv.tv_sec = 1; tv.tv_usec = 500000;
                bool ok = (select(0, nullptr, &wf, nullptr, &tv) > 0);
                if (ok) {
                    int err = 0; int elen = sizeof(err);
                    getsockopt(cs, SOL_SOCKET, SO_ERROR, (char*)&err, &elen);
                    ok = (err == 0);
                }
                u_long bl = 0; ioctlsocket(cs, FIONBIO, &bl);   // back to blocking
                if (ok) {
                    m_ctrl_sock.store((uintptr_t)cs);
                    m_ctrl_connected.store(true);
                    m_ctrl_stop.store(false);
                    m_ctrl_reader = std::thread(&UdpTransport::ctrlReaderThread, this);
                } else {
                    closesocket(cs);
                }
            }
            freeaddrinfo(tres);
        }
    }
#endif
}

UdpTransport::~UdpTransport() {
    close();
    if (m_udp_addr) { delete (sockaddr_storage*)m_udp_addr; m_udp_addr = nullptr; }
}

void UdpTransport::close() {
    m_open.store(false);
    m_ctrl_stop.store(true);
#ifdef _WIN32
    uintptr_t cs = m_ctrl_sock.exchange(kInvalidSock);
    if (cs != kInvalidSock) { shutdown((SOCKET)cs, SD_BOTH); closesocket((SOCKET)cs); }
    if (m_ctrl_reader.joinable()) m_ctrl_reader.join();
    m_ctrl_connected.store(false);
    if (m_udp_sock != kInvalidSock) { closesocket((SOCKET)m_udp_sock); m_udp_sock = kInvalidSock; }
#endif
}

// ── UDP motion plane ────────────────────────────────────────────────
bool UdpTransport::write(const uint8_t* data, int len) {
#ifdef _WIN32
    if (m_udp_sock == kInvalidSock || !m_udp_addr) return false;
    std::lock_guard<std::mutex> lk(m_udp_mutex);
    int n = sendto((SOCKET)m_udp_sock, (const char*)data, len, 0,
                   (sockaddr*)m_udp_addr, m_udp_addr_len);
    if (n > 0) m_tx_bytes.fetch_add(n);
    return n == len;
#else
    (void)data; (void)len; return false;
#endif
}

bool UdpTransport::sendCobsData(const uint32_t raw[6], int bit_depth) {
    (void)bit_depth;
    uint8_t frame[1 + 18];
    frame[0] = COBS_CH_DATA18;
    for (int i = 0; i < 6; i++) {
        uint32_t v = raw[i] & 0x3FFFFu;
        frame[1 + i * 3]     = (uint8_t)(v & 0xFFu);
        frame[1 + i * 3 + 1] = (uint8_t)((v >> 8) & 0xFFu);
        frame[1 + i * 3 + 2] = (uint8_t)((v >> 16) & 0xFFu);
    }
    uint8_t enc[64];
    int enc_len = cobs_encode(frame, sizeof(frame), enc);
    enc[enc_len++] = 0x00;
    return write(enc, enc_len);   // one datagram = one COBS frame
}

bool UdpTransport::sendCobsDataRaw(const float raw[6]) {
    uint8_t frame[1 + 24];
    frame[0] = COBS_CH_DATA_RAW;
    memcpy(frame + 1, raw, 24);   // 6 x float32 LE, pre-cueing, app axis order
    uint8_t enc[64];
    int enc_len = cobs_encode(frame, sizeof(frame), enc);
    enc[enc_len++] = 0x00;
    return write(enc, enc_len);
}

bool UdpTransport::sendCommand(const char* cmd) {
    return sendCobsCommand(cmd);
}

bool UdpTransport::sendCobsCommand(const char* cmd) {
    // Forward-compat: emit a COBS CH_CMD frame as a UDP datagram. Today's bridge
    // forwards motion datagrams to the UART only while LIVE; real control verbs
    // travel over the TCP control plane (controlSend). Kept so app command paths
    // (e.g. TELRATE) are functional if the bridge later passes CH_CMD through.
    if (!cmd) return false;
    int slen = (int)strlen(cmd);
    if (slen <= 0 || slen > 254) return false;
    uint8_t frame[256];
    frame[0] = COBS_CH_CMD;
    memcpy(frame + 1, cmd, slen);
    uint8_t enc[300];
    int enc_len = cobs_encode(frame, 1 + slen, enc);
    enc[enc_len++] = 0x00;
    return write(enc, enc_len);
}

// ── TCP control plane ───────────────────────────────────────────────
bool UdpTransport::controlSend(const std::string& json_line) {
#ifdef _WIN32
    uintptr_t cs = m_ctrl_sock.load();
    if (cs == kInvalidSock || !m_ctrl_connected.load()) return false;
    std::string out = json_line;
    if (out.empty() || out.back() != '\n') out.push_back('\n');
    std::lock_guard<std::mutex> lk(m_ctrl_send_mutex);
    int sent = 0, total = (int)out.size();
    while (sent < total) {
        int n = send((SOCKET)cs, out.data() + sent, total - sent, 0);
        if (n <= 0) { m_ctrl_connected.store(false); return false; }
        sent += n;
    }
    m_tx_bytes.fetch_add(total);
    return true;
#else
    (void)json_line; return false;
#endif
}

void UdpTransport::ctrlReaderThread() {
#ifdef _WIN32
    std::string buf;
    char rx[2048];
    while (!m_ctrl_stop.load()) {
        uintptr_t cs = m_ctrl_sock.load();
        if (cs == kInvalidSock) break;
        int n = recv((SOCKET)cs, rx, sizeof(rx), 0);
        if (n <= 0) break;
        m_rx_bytes.fetch_add(n);
        buf.append(rx, n);
        size_t nl;
        while ((nl = buf.find('\n')) != std::string::npos) {
            std::string line = buf.substr(0, nl);
            buf.erase(0, nl + 1);
            if (!line.empty() && line.back() == '\r') line.pop_back();
            if (!line.empty()) handleControlLine(line.c_str());
        }
    }
    m_ctrl_connected.store(false);
#endif
}

void UdpTransport::handleControlLine(const char* line) {
    cJSON* root = cJSON_Parse(line);
    if (!root) return;
    m_ctrl_msgs.fetch_add(1);

    const cJSON* type = cJSON_GetObjectItem(root, "type");
    const char* ts = (type && cJSON_IsString(type)) ? type->valuestring : "";

    if (strcmp(ts, "event") == 0) {
        const cJSON* ev = cJSON_GetObjectItem(root, "event");
        const char* es = (ev && cJSON_IsString(ev)) ? ev->valuestring : "";
        if (strcmp(es, "status") == 0) {
            parseStatusBody(root);
        } else if (strcmp(es, "telemetry") == 0) {
            m_cobs_tel.fetch_add(1);
            const cJSON* hx = cJSON_GetObjectItem(root, "hex");
            if (hx && cJSON_IsString(hx)) {
                const char* h = hx->valuestring;
                int hlen = (int)strlen(h);
                uint8_t bytes[64]; int nb = 0;
                for (int i = 0; i + 1 < hlen && nb < (int)sizeof(bytes); i += 2) {
                    auto hv = [](char c)->int {
                        if (c >= '0' && c <= '9') return c - '0';
                        if (c >= 'a' && c <= 'f') return c - 'a' + 10;
                        if (c >= 'A' && c <= 'F') return c - 'A' + 10;
                        return 0;
                    };
                    bytes[nb++] = (uint8_t)((hv(h[i]) << 4) | hv(h[i + 1]));
                }
                if (nb >= 24) {
                    float vals[12] = {};
                    memcpy(vals, bytes, nb < 48 ? nb : 48);
                    bool valid = true;
                    for (int i = 0; i < 6; i++)
                        if (vals[i] != vals[i] || vals[i] > 1e10f || vals[i] < -1e10f) { valid = false; break; }
                    if (valid) {
                        double t = now_seconds();
                        {
                            std::lock_guard<std::mutex> lk(m_tel_mutex);
                            memcpy(m_telemetry.angles, vals, 6 * sizeof(float));
                            if (nb >= 48) memcpy(m_telemetry.positions, vals + 6, 6 * sizeof(float));
                            m_telemetry.timestamp = t;
                            m_telemetry.seq++;
                            m_tel_seq.store(m_telemetry.seq);
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
                    } else {
                        m_tel_rejected.fetch_add(1);
                    }
                }
            }
        } else if (strcmp(es, "resp") == 0 || strcmp(es, "log") == 0) {
            if (strcmp(es, "resp") == 0) m_cobs_resp.fetch_add(1); else m_cobs_log.fetch_add(1);
            const cJSON* tx = cJSON_GetObjectItem(root, "text");
            if (tx && cJSON_IsString(tx) && tx->valuestring[0]) {
                std::lock_guard<std::mutex> lk(m_line_mutex);
                m_line_queue.emplace_back(tx->valuestring);
            }
        }
    } else if (strcmp(ts, "resp") == 0) {
        // Direct reply to one of our verbs. `status` replies carry the full body.
        const cJSON* verb = cJSON_GetObjectItem(root, "verb");
        const char* vs = (verb && cJSON_IsString(verb)) ? verb->valuestring : "";
        if (strcmp(vs, "status") == 0) parseStatusBody(root);
        // list_files results (Phase 3) may carry a "files" array — parse if present.
        const cJSON* files = cJSON_GetObjectItem(root, "files");
        if (files && cJSON_IsArray(files)) {
            std::lock_guard<std::mutex> lk(m_status_mutex);
            m_files.clear();
            cJSON* it = nullptr;
            cJSON_ArrayForEach(it, files) {
                if (cJSON_IsString(it)) m_files.emplace_back(it->valuestring);
                else { cJSON* nm = cJSON_GetObjectItem(it, "name");
                       if (nm && cJSON_IsString(nm)) m_files.emplace_back(nm->valuestring); }
            }
        }
    }
    // "hello" → nothing to do (auth disabled on the LAN by default).

    cJSON_Delete(root);
}

void UdpTransport::parseStatusBody(void* cjson_obj) {
    cJSON* o = (cJSON*)cjson_obj;
    auto getstr = [&](const char* k) -> const char* {
        cJSON* v = cJSON_GetObjectItem(o, k);
        return (v && cJSON_IsString(v)) ? v->valuestring : nullptr;
    };
    std::lock_guard<std::mutex> lk(m_status_mutex);
    if (const char* s = getstr("source"))        m_source = s;
    if (const char* s = getstr("boot_source"))   m_boot_source = s;
    if (const char* s = getstr("play_state"))    m_play_state = s;
    cJSON* demo = cJSON_GetObjectItem(o, "selected_demo");
    if (demo) m_selected_demo = cJSON_IsString(demo) ? demo->valuestring : "";
    cJSON* mem = cJSON_GetObjectItem(o, "mem");
    if (mem && cJSON_IsObject(mem)) {
        cJSON* u = cJSON_GetObjectItem(mem, "used");
        cJSON* f = cJSON_GetObjectItem(mem, "free");
        if (u && cJSON_IsNumber(u)) m_mem_used.store((long)u->valuedouble);
        if (f && cJSON_IsNumber(f)) m_mem_free.store((long)f->valuedouble);
    }
}

// ── RX accessors ────────────────────────────────────────────────────
std::vector<std::string> UdpTransport::drainLines() {
    std::lock_guard<std::mutex> lk(m_line_mutex);
    std::vector<std::string> out;
    out.swap(m_line_queue);
    return out;
}

ESP32Telemetry UdpTransport::getLatestTelemetry() const {
    std::lock_guard<std::mutex> lk(m_tel_mutex);
    return m_telemetry;
}

std::string UdpTransport::source() const {
    std::lock_guard<std::mutex> lk(m_status_mutex); return m_source;
}
std::string UdpTransport::bootSource() const {
    std::lock_guard<std::mutex> lk(m_status_mutex); return m_boot_source;
}
std::string UdpTransport::playState() const {
    std::lock_guard<std::mutex> lk(m_status_mutex); return m_play_state;
}
std::string UdpTransport::selectedDemo() const {
    std::lock_guard<std::mutex> lk(m_status_mutex); return m_selected_demo;
}
std::vector<std::string> UdpTransport::deviceFiles() const {
    std::lock_guard<std::mutex> lk(m_status_mutex); return m_files;
}
