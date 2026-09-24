/*
 * Control Server implementation — see control_server.h.
 */
#ifdef _WIN32
#  define WIN32_LEAN_AND_MEAN
#  include <winsock2.h>
#  include <ws2tcpip.h>
#  pragma comment(lib, "ws2_32.lib")
#endif

#include "control_server.h"
#include "app.h"
#include "automation.h"
#include "ui_panels.h"
#include "serial_port.h"
#include "udp_transport.h"
#include "cJSON.h"

#include <string>
#include <vector>
#include <deque>
#include <mutex>
#include <future>
#include <thread>
#include <cstring>
#include <cstdio>
#include <cstdint>
#include <ctime>
#include <cmath>
#include <chrono>
#include <filesystem>
#include <cctype>

ControlServer g_ctrl;

// ── Cross-thread request queue ──────────────────────────────────────
namespace {

struct Req {
    std::string                 line;
    std::promise<std::string>   resp;
};

std::mutex                 g_qmtx;
std::deque<Req>            g_queue;
std::atomic<bool>          g_stop{false};
std::thread                g_accept_thread;
std::vector<std::thread>   g_conn_threads;
std::mutex                 g_conn_mtx;
#ifdef _WIN32
SOCKET                     g_listen = INVALID_SOCKET;
#endif

// ── JSON response helpers ───────────────────────────────────────────
std::string print_and_free(cJSON* root) {
    char* s = cJSON_PrintUnformatted(root);
    std::string out = s ? s : "{}";
    if (s) cJSON_free(s);
    cJSON_Delete(root);
    return out;
}
std::string ok_resp(int id, cJSON* result) {
    cJSON* r = cJSON_CreateObject();
    cJSON_AddNumberToObject(r, "id", id);
    cJSON_AddBoolToObject(r, "ok", true);
    cJSON_AddItemToObject(r, "result", result ? result : cJSON_CreateObject());
    return print_and_free(r);
}
std::string err_resp(int id, const char* msg) {
    cJSON* r = cJSON_CreateObject();
    cJSON_AddNumberToObject(r, "id", id);
    cJSON_AddBoolToObject(r, "ok", false);
    cJSON_AddStringToObject(r, "error", msg ? msg : "error");
    return print_and_free(r);
}

// ── arg helpers ─────────────────────────────────────────────────────
double num(cJSON* a, const char* k, double def) {
    cJSON* v = a ? cJSON_GetObjectItem(a, k) : nullptr;
    return (v && cJSON_IsNumber(v)) ? v->valuedouble : def;
}
bool has(cJSON* a, const char* k) {
    return a && cJSON_GetObjectItem(a, k) != nullptr;
}
bool boolarg(cJSON* a, const char* k, bool def) {
    cJSON* v = a ? cJSON_GetObjectItem(a, k) : nullptr;
    if (!v) return def;
    if (cJSON_IsBool(v)) return cJSON_IsTrue(v);
    if (cJSON_IsNumber(v)) return v->valuedouble != 0.0;
    return def;
}
const char* strarg(cJSON* a, const char* k, const char* def) {
    cJSON* v = a ? cJSON_GetObjectItem(a, k) : nullptr;
    return (v && cJSON_IsString(v)) ? v->valuestring : def;
}

Entity* pick_entity(cJSON* a) {
    if (g_app.entities.empty()) return nullptr;
    cJSON* e = a ? cJSON_GetObjectItem(a, "entity") : nullptr;
    if (e && cJSON_IsNumber(e)) {
        Entity* p = g_app.findEntity(e->valueint);
        if (p) return p;
    }
    return &g_app.entities[0];
}

const char* source_str(InputSource s) {
    switch (s) {
        case InputSource::CapturePlayback: return "capture";
        case InputSource::Plugin:          return "plugin";
    }
    return "?";
}

cJSON* farr(const float* v, int n) {
    cJSON* a = cJSON_CreateArray();
    for (int i = 0; i < n; i++) cJSON_AddItemToArray(a, cJSON_CreateNumber(v[i]));
    return a;
}

// Resolve a recording index from {index} or {name}. Returns -1 if not found.
int resolve_recording(cJSON* a) {
    if (has(a, "index")) {
        int idx = (int)num(a, "index", -1);
        if (idx >= 0 && idx < (int)g_app.saved_recordings.size()) return idx;
        return -1;
    }
    const char* name = strarg(a, "name", nullptr);
    if (name) {
        for (int i = 0; i < (int)g_app.saved_recordings.size(); i++)
            if (strcmp(g_app.saved_recordings[i].name, name) == 0) return i;
    }
    return -1;
}

bool valid_keys(const cJSON* args, const char* const* keys, size_t count) {
    if (!args) return true;
    if (!cJSON_IsObject(args)) return false;
    for (const cJSON* item = args->child; item; item = item->next) {
        bool found = false;
        for (size_t i = 0; i < count; ++i)
            if (item->string && !strcmp(item->string, keys[i])) found = true;
        if (!found) return false;
    }
    return true;
}

bool valid_png_target(const char* path) {
    const std::filesystem::path target(path);
    std::string extension = target.extension().string();
    for (char& c : extension) c = char(std::tolower(static_cast<unsigned char>(c)));
    if (extension != ".png") return false;
    std::error_code error;
    const bool exists = std::filesystem::exists(target, error);
    if (error) return false;
    if (!exists) return true;
    // A screenshot may refresh an existing PNG, but must not overwrite an
    // unrelated file merely renamed with a .png suffix.
    FILE* file = fopen(path, "rb");
    if (!file) return false;
    unsigned char signature[8]{};
    const size_t count = fread(signature, 1, sizeof(signature), file);
    fclose(file);
    const unsigned char png[] = {137, 80, 78, 71, 13, 10, 26, 10};
    return count == sizeof(signature) && !memcmp(signature, png, sizeof(png));
}

// Synthetic documentation inputs, never a hardware capture or a test result.
// Normal capture playback performs filtering, cueing and IK on these samples.
void add_documentation_fixture() {
    if (g_app.entities.empty()) g_app.addEntity("Documentation SIL", EntityType::SIL);
    const char* names[] = {"SYNTHETIC - Six-axis demo", "SYNTHETIC - Step response",
                          "SYNTHETIC - Frequency sweep"};
    for (int fixture = 0; fixture < 3; ++fixture) {
        bool exists = false;
        for (const auto& recording : g_app.saved_recordings)
            if (!strcmp(recording.name, names[fixture])) exists = true;
        if (exists) continue;
        SavedRecording recording{};
        snprintf(recording.name, sizeof(recording.name), "%s", names[fixture]);
        snprintf(recording.source, sizeof(recording.source), "synthetic-docs");
        recording.sample_rate_hz = 100.0;
        recording.created_time = 0.0;
        for (int i = 0; i <= 3000; ++i) {
            RecordSample sample{};
            sample.time = i / recording.sample_rate_hz;
            for (int axis = 0; axis < 6; ++axis) {
                const double frequency = 0.08 + axis * 0.025;
                if (fixture == 0)
                    sample.input[axis] = float((16.0 - axis) * sin(6.283185307179586 * frequency * sample.time));
                else if (fixture == 1)
                    sample.input[axis] = float((sample.time >= 3.0 && sample.time < 15.0) ? (12.0 - axis) : 0.0);
                else {
                    // A 0.5 -> 8 Hz linear chirp on Heave only; illustrative
                    // spectrum content, not measured bandwidth or validation.
                    const double t = sample.time;
                    sample.input[axis] = axis == 2 ? float(5.0 * sin(6.283185307179586 * (0.5 * t + 0.125 * t * t))) : 0.0f;
                }
            }
            recording.samples.push_back(sample);
        }
        g_app.saved_recordings.push_back(std::move(recording));
    }
    g_app.log(-1, "docs", "Loaded synthetic documentation recordings (not hardware measurements)");
}

// ── Offline sequence bake (Phase 3 freeze) ─────────────────────────
// Runs the entity's CURRENT pipeline (input filter + MCA + intensity/gain)
// over a recording at `rate` Hz and encodes each sample to device raw exactly
// as the HIL TX path does (app.cpp:2334-2341): r = scaled/100*home + home,
// clamp, round, swap surge<->sway. Result is byte-identical to live HIL, so
// on-device playback == what the app streams with the same tuning.
struct BakeStats { float mn[6], mx[6]; double sat[6]; int n; };

void interp_recording(const SavedRecording& sr, double t, float out[6]) {
    if (sr.samples.empty()) { for (int i = 0; i < 6; i++) out[i] = 0; return; }
    double dur = sr.samples.back().time;
    if (t < 0) t = 0; if (t > dur) t = dur;
    double fidx = t * sr.sample_rate_hz;
    int idx = (int)fidx;
    if (idx >= (int)sr.samples.size() - 1) {
        memcpy(out, sr.samples.back().input, 6 * sizeof(float)); return;
    }
    float a = (float)(fidx - idx);
    for (int i = 0; i < 6; i++)
        out[i] = sr.samples[idx].input[i] * (1.0f - a) + sr.samples[idx + 1].input[i] * a;
}

bool bake_sequence(int rec_idx, Entity& e, int rate, int bits, double preroll_s,
                   std::vector<uint16_t>& out, BakeStats& st) {
    if (rec_idx < 0 || rec_idx >= (int)g_app.saved_recordings.size()) return false;
    const SavedRecording& sr = g_app.saved_recordings[rec_idx];
    double dur = sr.samples.empty() ? 0.0 : sr.samples.back().time;
    int n = (int)(dur * rate) + 1;

    MotionCueingConfig mca = e.config.mca;            // copy live config (incl. tuning)
    resetMotionCueing(&mca);
    mcaUpdateSampleRate(&mca, (float)rate);
    InputFilterConfig flt = e.config.input_filter;
    resetInputFilter(&flt);
    inputFilterUpdateSampleRate(&flt, (float)rate);
    bool use_flt = flt.enabled != 0;
    bool use_mca = mca.enabled != 0;

    float intensity = e.config.intensity;
    float max_raw = (float)((1u << bits) - 1u);
    float home = max_raw * 0.5f;
    for (int i = 0; i < 6; i++) { st.mn[i] = 1e9f; st.mx[i] = -1e9f; st.sat[i] = 0; }

    // Pre-roll on sample 0 so washout IIRs are settled at output sample 0.
    float s0[6]; interp_recording(sr, 0.0, s0);
    int pre = (int)(preroll_s * rate);
    for (int k = 0; k < pre; k++) {
        float pct[6], t1[6]; memcpy(pct, s0, sizeof(pct));
        if (use_flt) { processInputFilter(&flt, pct, t1); memcpy(pct, t1, sizeof(pct)); }
        if (use_mca) { processMotionCueing(&mca, pct, t1); memcpy(pct, t1, sizeof(pct)); }
    }

    out.clear(); out.reserve((size_t)n * 6);
    for (int k = 0; k < n; k++) {
        float pct[6], t1[6]; interp_recording(sr, (double)k / rate, pct);
        if (use_flt) { processInputFilter(&flt, pct, t1); memcpy(pct, t1, sizeof(pct)); }
        if (use_mca) { processMotionCueing(&mca, pct, t1); memcpy(pct, t1, sizeof(pct)); }
        uint16_t raw[6];
        for (int i = 0; i < 6; i++) {
            float inv = e.config.axis_invert[i] ? -1.0f : 1.0f;
            float sc = pct[i] * (intensity / 100.0f) * (e.config.axis_gain[i] / 100.0f) * inv;
            if (sc < st.mn[i]) st.mn[i] = sc;
            if (sc > st.mx[i]) st.mx[i] = sc;
            if (sc > 100.0f || sc < -100.0f) st.sat[i] += 1.0;
            float r = (sc / 100.0f) * home + home;
            if (r < 0.0f) r = 0.0f; if (r > max_raw) r = max_raw;
            raw[i] = (uint16_t)(r + 0.5f);
        }
        uint16_t tmp = raw[0]; raw[0] = raw[1]; raw[1] = tmp;   // surge<->sway (app->wire)
        for (int i = 0; i < 6; i++) out.push_back(raw[i]);
    }
    st.n = n;
    for (int i = 0; i < 6; i++) st.sat[i] = n ? st.sat[i] / n * 100.0 : 0.0;
    return true;
}

uint32_t crc32_buf(const uint8_t* d, size_t n) {
    uint32_t c = 0xFFFFFFFFu;
    for (size_t i = 0; i < n; i++) {
        c ^= d[i];
        for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320u & (uint32_t)(-(int)(c & 1)));
    }
    return ~c;
}

cJSON* stats_json(const BakeStats& st, int rate) {
    static const char* AX[6] = {"surge","sway","heave","roll","pitch","yaw"};
    cJSON* r = cJSON_CreateObject();
    cJSON_AddNumberToObject(r, "samples", st.n);
    cJSON_AddNumberToObject(r, "rate_hz", rate);
    cJSON_AddNumberToObject(r, "duration_s", rate ? (double)(st.n - 1) / rate : 0.0);
    cJSON* ax = cJSON_CreateObject();
    for (int i = 0; i < 6; i++) {
        cJSON* o = cJSON_CreateObject();
        cJSON_AddNumberToObject(o, "min_pct", st.mn[i]);
        cJSON_AddNumberToObject(o, "max_pct", st.mx[i]);
        cJSON_AddNumberToObject(o, "saturated_pct", st.sat[i]);   // % of samples clipped at rails
        cJSON_AddItemToObject(ax, AX[i], o);
    }
    cJSON_AddItemToObject(r, "axes", ax);
    return r;
}

// ── Command dispatch (runs on the render thread via drain) ──────────
std::string handle(const std::string& line) {
    cJSON* root = cJSON_Parse(line.c_str());
    if (!root) return err_resp(0, "invalid JSON");
    int id = (int)num(root, "id", 0);
    const char* cmd = strarg(root, "cmd", "");
    cJSON* a = cJSON_GetObjectItem(root, "args");   // may be null
    std::string out;

    auto RESULT = [&](cJSON* r){ out = ok_resp(id, r); };
    auto ERR    = [&](const char* m){ out = err_resp(id, m); };

    // Defense in depth: the transport layer also refuses hardware I/O in this
    // mode. Reject before creating a transport, queueing or changing state.
    const bool hardware_command = !strcmp(cmd, "hil_connect") ||
        !strcmp(cmd, "hil_stream_test") || !strcmp(cmd, "hil_command") ||
        !strcmp(cmd, "geometry_push") || !strcmp(cmd, "mca_push") ||
        !strcmp(cmd, "mca_save") ||
        (!strcmp(cmd, "plugin") && !strcmp(strarg(a, "action", "list"), "activate"));
    if (IsDocumentationMode() && hardware_command) {
        cJSON_Delete(root);
        return err_resp(id, "hardware and live plugin activation are disabled in documentation mode");
    }
    if (IsDocumentationMode() && !strcmp(cmd, "export_sequence")) {
        cJSON_Delete(root);
        return err_resp(id, "sequence file export is disabled in documentation mode");
    }

    if (!strcmp(cmd, "ping")) {
        cJSON* r = cJSON_CreateObject();
        cJSON_AddStringToObject(r, "app", "stewart-platform");
        cJSON_AddNumberToObject(r, "api", 2);
        cJSON_AddBoolToObject(r, "documentation_mode", IsDocumentationMode());
        cJSON_AddNumberToObject(r, "frame_count", g_app.frame_count);
        cJSON_AddNumberToObject(r, "fps", g_app.fps);
        cJSON_AddBoolToObject(r, "running", g_app.running);
        RESULT(r);
    }
    else if (!strcmp(cmd, "capabilities")) {
        static const char* commands[] = {
            "ping", "status", "capabilities", "ui_get", "ui_set", "window_get", "window_set",
            "docs_fixture", "screenshot", "log_tail", "recordings_list", "recordings_reload",
            "play", "stop", "playback_status", "playback_seek", "playback_speed", "playback_loop",
            "mca_presets", "mca_apply_preset", "mca_get", "mca_set", "mca_tilt", "intensity",
            "axis_set", "state", "entities_list", "entity_add", "entity_remove", "hil_connect",
            "hil_stream_test", "plugin", "plugin_params", "plugin_param_set", "serial_ports",
            "settings_save", "motion", "analyze", "export_sequence", "hil_command", "hil_status",
            "hil_raw_mode", "hil_tx_rate", "estop", "entity_enable", "record", "recording_delete",
            "recording_rename", "recordings_save", "geometry_get", "geometry_set", "geometry_push",
            "mca_push", "mca_save", "console_rate", "quit"
        };
        cJSON* r = cJSON_CreateObject();
        cJSON_AddNumberToObject(r, "api", 2);
        cJSON_AddBoolToObject(r, "documentation_mode", IsDocumentationMode());
        cJSON_AddBoolToObject(r, "hardware_io_allowed", !IsDocumentationMode());
        cJSON_AddNumberToObject(r, "frame_count", g_app.frame_count);
        cJSON_AddItemToObject(r, "commands", cJSON_CreateStringArray(commands, int(sizeof(commands) / sizeof(commands[0]))));
        cJSON_AddItemToObject(r, "ui", UIAutomationState());
        RESULT(r);
    }
    else if (!strcmp(cmd, "ui_get")) {
        RESULT(UIAutomationState());
    }
    else if (!strcmp(cmd, "ui_set")) {
        std::string error;
        cJSON* r = UIAutomationConfigure(a, error);
        if (r) RESULT(r); else ERR(error.c_str());
    }
    else if (!strcmp(cmd, "window_get") || !strcmp(cmd, "window_set")) {
        const char* keys[] = {"width", "height"};
        int width = 0, height = 0;
        GetAppWindowSize(&width, &height);
        if (!strcmp(cmd, "window_set")) {
            cJSON* w = a ? cJSON_GetObjectItem(a, "width") : nullptr;
            cJSON* h = a ? cJSON_GetObjectItem(a, "height") : nullptr;
            if (!valid_keys(a, keys, 2) || !cJSON_IsNumber(w) || !cJSON_IsNumber(h) ||
                !std::isfinite(w->valuedouble) || !std::isfinite(h->valuedouble) ||
                w->valuedouble != w->valueint || h->valuedouble != h->valueint ||
                w->valueint < 960 || w->valueint > 3840 || h->valueint < 640 || h->valueint > 2160) {
                ERR("window_set requires integer width 960..3840 and height 640..2160 only");
            } else if (!SetAppWindowSize(w->valueint, h->valueint)) {
                ERR("window resize unavailable");
            } else GetAppWindowSize(&width, &height);
        }
        if (out.empty()) {
            cJSON* r = cJSON_CreateObject();
            cJSON_AddNumberToObject(r, "width", width);
            cJSON_AddNumberToObject(r, "height", height);
            RESULT(r);
        }
    }
    else if (!strcmp(cmd, "docs_fixture")) {
        if (!IsDocumentationMode()) ERR("docs_fixture requires an isolated documentation-mode launch");
        else if (a && (!cJSON_IsObject(a) || a->child)) ERR("docs_fixture takes no arguments");
        else {
            add_documentation_fixture();
            cJSON* r = cJSON_CreateObject();
            cJSON_AddStringToObject(r, "source", "synthetic-docs");
            cJSON_AddBoolToObject(r, "hardware_data", false);
            cJSON_AddNumberToObject(r, "recordings", double(g_app.saved_recordings.size()));
            cJSON_AddStringToObject(r, "note", "In-memory synthetic inputs; use play for the real playback/MCA/IK pipeline");
            RESULT(r);
        }
    }
    else if (!strcmp(cmd, "status")) {
        cJSON* r = cJSON_CreateObject();
        cJSON_AddBoolToObject(r, "documentation_mode", IsDocumentationMode());
        cJSON_AddBoolToObject(r, "running", g_app.running);
        cJSON_AddNumberToObject(r, "fps", g_app.fps);
        cJSON_AddNumberToObject(r, "frame_count", g_app.frame_count);
        cJSON_AddBoolToObject(r, "motion_started", g_app.motion_started);
        cJSON_AddStringToObject(r, "input_source", source_str(g_app.input_source));
        cJSON_AddNumberToObject(r, "recordings", (double)g_app.saved_recordings.size());
        cJSON* ents = cJSON_CreateArray();
        for (auto& e : g_app.entities) {
            cJSON* o = cJSON_CreateObject();
            cJSON_AddNumberToObject(o, "id", e.id);
            cJSON_AddStringToObject(o, "name", e.name);
            cJSON_AddStringToObject(o, "type", e.type == EntityType::HIL ? "HIL" : "SIL");
            cJSON_AddBoolToObject(o, "enabled", e.enabled);
            cJSON_AddItemToArray(ents, o);
        }
        cJSON_AddItemToObject(r, "entities", ents);
        cJSON* cap = cJSON_CreateObject();
        cJSON_AddBoolToObject(cap, "playing", g_app.capture_playing);
        cJSON_AddNumberToObject(cap, "index", g_app.capture_playback_idx);
        cJSON_AddBoolToObject(cap, "loop", g_app.capture_loop);
        cJSON_AddNumberToObject(cap, "speed", g_app.capture_speed);
        if (g_app.capture_playback_idx >= 0 &&
            g_app.capture_playback_idx < (int)g_app.saved_recordings.size())
            cJSON_AddStringToObject(cap, "name",
                g_app.saved_recordings[g_app.capture_playback_idx].name);
        cJSON_AddItemToObject(r, "capture", cap);
        RESULT(r);
    }
    else if (!strcmp(cmd, "screenshot")) {
        std::error_code path_error;
        std::filesystem::path temporary = std::filesystem::temp_directory_path(path_error);
        const std::string default_path = path_error ? "stewart_shot.png" :
            (temporary / "stewart_shot.png").string();
        const char* path = strarg(a, "path", default_path.c_str());
        int w = 0, h = 0;
        if (!valid_png_target(path)) {
            ERR("screenshot target must be a .png path; existing non-PNG files cannot be overwritten");
        } else if (CaptureWindowPNG(path, &w, &h)) {
            cJSON* r = cJSON_CreateObject();
            cJSON_AddStringToObject(r, "path", path);
            cJSON_AddNumberToObject(r, "width", w);
            cJSON_AddNumberToObject(r, "height", h);
            cJSON_AddNumberToObject(r, "frame_count", g_app.frame_count);
            cJSON_AddBoolToObject(r, "documentation_mode", IsDocumentationMode());
            RESULT(r);
        } else ERR("capture failed");
    }
    else if (!strcmp(cmd, "log_tail")) {
        int n = (int)num(a, "n", 30);
        if (n < 1) n = 1; if (n > 500) n = 500;
        const char* src_filter = strarg(a, "source", nullptr);
        int total = (int)g_app.console_log.size();
        // Walk backwards collecting up to n (filtered) lines, then emit in order.
        std::vector<int> keep;
        for (int i = total - 1; i >= 0 && (int)keep.size() < n; i--) {
            if (src_filter && strcmp(g_app.console_log[i].source, src_filter) != 0)
                continue;
            keep.push_back(i);
        }
        cJSON* arr = cJSON_CreateArray();
        for (int k = (int)keep.size() - 1; k >= 0; k--) {
            auto& L = g_app.console_log[keep[k]];
            cJSON* o = cJSON_CreateObject();
            cJSON_AddNumberToObject(o, "t", L.timestamp);
            cJSON_AddStringToObject(o, "source", L.source);
            cJSON_AddStringToObject(o, "msg", L.message);
            cJSON_AddItemToArray(arr, o);
        }
        cJSON* r = cJSON_CreateObject();
        cJSON_AddItemToObject(r, "lines", arr);
        RESULT(r);
    }
    else if (!strcmp(cmd, "recordings_list")) {
        cJSON* arr = cJSON_CreateArray();
        for (int i = 0; i < (int)g_app.saved_recordings.size(); i++) {
            auto& s = g_app.saved_recordings[i];
            cJSON* o = cJSON_CreateObject();
            cJSON_AddNumberToObject(o, "index", i);
            cJSON_AddStringToObject(o, "name", s.name);
            cJSON_AddNumberToObject(o, "rate_hz", s.sample_rate_hz);
            cJSON_AddNumberToObject(o, "duration_s", s.duration());
            cJSON_AddNumberToObject(o, "samples", (double)s.samples.size());
            cJSON_AddStringToObject(o, "source", s.source);
            cJSON_AddItemToArray(arr, o);
        }
        cJSON* r = cJSON_CreateObject();
        cJSON_AddItemToObject(r, "recordings", arr);
        RESULT(r);
    }
    else if (!strcmp(cmd, "recordings_reload")) {
        g_app.loadRecordingsFromDisk();
        cJSON* r = cJSON_CreateObject();
        cJSON_AddNumberToObject(r, "count", (double)g_app.saved_recordings.size());
        RESULT(r);
    }
    else if (!strcmp(cmd, "play")) {
        int idx = resolve_recording(a);
        if (idx < 0) { ERR("recording not found"); }
        else {
            if (has(a, "loop"))  g_app.capture_loop  = boolarg(a, "loop", true);
            if (has(a, "speed")) g_app.capture_speed = (float)num(a, "speed", 1.0);
            g_app.motion_started = true;               // required: update() gates on this
            g_app.start_ramp_active = false;
            g_app.startCapturePlayback(idx);
            cJSON* r = cJSON_CreateObject();
            cJSON_AddBoolToObject(r, "playing", true);
            cJSON_AddNumberToObject(r, "index", idx);
            cJSON_AddStringToObject(r, "name", g_app.saved_recordings[idx].name);
            cJSON_AddBoolToObject(r, "loop", g_app.capture_loop);
            cJSON_AddNumberToObject(r, "speed", g_app.capture_speed);
            RESULT(r);
        }
    }
    else if (!strcmp(cmd, "stop")) {
        g_app.stopCapturePlayback();
        RESULT(nullptr);
    }
    else if (!strcmp(cmd, "playback_status")) {
        cJSON* r = cJSON_CreateObject();
        cJSON_AddBoolToObject(r, "playing", g_app.capture_playing);
        cJSON_AddNumberToObject(r, "index", g_app.capture_playback_idx);
        cJSON_AddNumberToObject(r, "cursor", g_app.capture_play_cursor);
        cJSON_AddBoolToObject(r, "loop", g_app.capture_loop);
        cJSON_AddNumberToObject(r, "speed", g_app.capture_speed);
        if (g_app.capture_playing)
            cJSON_AddNumberToObject(r, "elapsed_s",
                (g_app.frame_time - g_app.capture_start_time) * g_app.capture_speed);
        if (g_app.capture_playback_idx >= 0 &&
            g_app.capture_playback_idx < (int)g_app.saved_recordings.size()) {
            auto& s = g_app.saved_recordings[g_app.capture_playback_idx];
            cJSON_AddNumberToObject(r, "duration_s", s.duration());
            cJSON_AddStringToObject(r, "name", s.name);
        }
        RESULT(r);
    }
    else if (!strcmp(cmd, "playback_seek")) {
        if (!g_app.capture_playing) { ERR("not playing"); }
        else {
            double t = num(a, "t", 0.0);
            double sp = g_app.capture_speed > 0 ? g_app.capture_speed : 1.0;
            g_app.capture_start_time = g_app.frame_time - t / sp;
            RESULT(nullptr);
        }
    }
    else if (!strcmp(cmd, "playback_speed")) {
        float v = (float)num(a, "value", 1.0);
        if (v < 0.05f) v = 0.05f; if (v > 5.0f) v = 5.0f;
        g_app.capture_speed = v;
        cJSON* r = cJSON_CreateObject();
        cJSON_AddNumberToObject(r, "speed", v);
        RESULT(r);
    }
    else if (!strcmp(cmd, "playback_loop")) {
        g_app.capture_loop = boolarg(a, "value", true);
        cJSON* r = cJSON_CreateObject();
        cJSON_AddBoolToObject(r, "loop", g_app.capture_loop);
        RESULT(r);
    }
    else if (!strcmp(cmd, "mca_presets")) {
        cJSON* arr = cJSON_CreateArray();
        for (int i = 0; i < (int)g_app.mca_presets.size(); i++) {
            auto& p = g_app.mca_presets[i];
            cJSON* o = cJSON_CreateObject();
            cJSON_AddNumberToObject(o, "index", i);
            cJSON_AddStringToObject(o, "name", p.name);
            cJSON_AddBoolToObject(o, "builtin", p.is_builtin);
            cJSON_AddItemToArray(arr, o);
        }
        cJSON* r = cJSON_CreateObject();
        cJSON_AddItemToObject(r, "presets", arr);
        RESULT(r);
    }
    else if (!strcmp(cmd, "mca_apply_preset")) {
        Entity* e = pick_entity(a);
        if (!e) { ERR("no entity"); }
        else {
            int idx = -1;
            if (has(a, "index")) idx = (int)num(a, "index", -1);
            else {
                const char* nm = strarg(a, "preset", strarg(a, "name", nullptr));
                if (nm) for (int i = 0; i < (int)g_app.mca_presets.size(); i++)
                    if (strcmp(g_app.mca_presets[i].name, nm) == 0) { idx = i; break; }
            }
            if (idx < 0 || idx >= (int)g_app.mca_presets.size()) ERR("preset not found");
            else {
                g_app.loadMcaPreset(idx, e->config.mca, e->config.intensity,
                                    e->config.axis_gain);
                g_app.settings_dirty = true;
                g_app.pushCueSettingsToDevice(*e);
                cJSON* r = cJSON_CreateObject();
                cJSON_AddStringToObject(r, "applied", g_app.mca_presets[idx].name);
                cJSON_AddNumberToObject(r, "entity", e->id);
                cJSON_AddNumberToObject(r, "intensity", e->config.intensity);
                RESULT(r);
            }
        }
    }
    else if (!strcmp(cmd, "mca_get")) {
        Entity* e = pick_entity(a);
        if (!e) { ERR("no entity"); }
        else {
            cJSON* r = cJSON_CreateObject();
            cJSON_AddNumberToObject(r, "entity", e->id);
            cJSON_AddBoolToObject(r, "mca_enabled", e->config.mca.enabled);
            cJSON_AddNumberToObject(r, "intensity", e->config.intensity);
            cJSON_AddItemToObject(r, "axis_gain", farr(e->config.axis_gain, 6));
            float inv[6]; for (int i = 0; i < 6; i++) inv[i] = e->config.axis_invert[i] ? 1.f : 0.f;
            cJSON_AddItemToObject(r, "axis_invert", farr(inv, 6));
            cJSON_AddBoolToObject(r, "input_filter_enabled", e->config.input_filter.enabled);
            cJSON* tilt = cJSON_CreateObject();
            cJSON_AddBoolToObject(tilt, "enabled",    e->config.mca.tilt.enabled);
            cJSON_AddNumberToObject(tilt, "surge_gain", e->config.mca.tilt.surge_gain); // surge->pitch
            cJSON_AddNumberToObject(tilt, "sway_gain",  e->config.mca.tilt.sway_gain);  // sway->roll
            cJSON_AddNumberToObject(tilt, "fc",         e->config.mca.tilt.fc);
            cJSON_AddNumberToObject(tilt, "Q",          e->config.mca.tilt.Q);
            cJSON_AddItemToObject(r, "tilt", tilt);
            RESULT(r);
        }
    }
    else if (!strcmp(cmd, "mca_set")) {
        Entity* e = pick_entity(a);
        if (!e) { ERR("no entity"); }
        else {
            if (has(a, "mca_enabled")) e->config.mca.enabled = boolarg(a, "mca_enabled", true);
            if (has(a, "intensity"))   e->config.intensity   = (float)num(a, "intensity", 100.0);
            cJSON* g = a ? cJSON_GetObjectItem(a, "axis_gain") : nullptr;
            if (g && cJSON_IsArray(g))
                for (int i = 0; i < 6 && i < cJSON_GetArraySize(g); i++)
                    e->config.axis_gain[i] = (float)cJSON_GetArrayItem(g, i)->valuedouble;
            cJSON* iv = a ? cJSON_GetObjectItem(a, "axis_invert") : nullptr;
            if (iv && cJSON_IsArray(iv))
                for (int i = 0; i < 6 && i < cJSON_GetArraySize(iv); i++)
                    e->config.axis_invert[i] = cJSON_IsTrue(cJSON_GetArrayItem(iv, i)) ||
                                               cJSON_GetArrayItem(iv, i)->valuedouble != 0.0;
            g_app.settings_dirty = true;
            g_app.pushCueSettingsToDevice(*e);
            RESULT(nullptr);
        }
    }
    else if (!strcmp(cmd, "mca_tilt")) {
        // Tilt coordination: route sustained surge->pitch and sway->roll so
        // gravity simulates sustained accel (and it reads visually).
        Entity* e = pick_entity(a);
        if (!e) { ERR("no entity"); }
        else {
            MotionCueingConfig* m = &e->config.mca;
            if (has(a, "enabled"))    mcaSetTiltEnabled(m, boolarg(a, "enabled", true) ? 1 : 0);
            if (has(a, "surge_gain")) mcaSetTiltSurgeGain(m, (float)num(a, "surge_gain", 0));
            if (has(a, "sway_gain"))  mcaSetTiltSwayGain(m, (float)num(a, "sway_gain", 0));
            if (has(a, "fc"))         mcaSetTiltFc(m, (float)num(a, "fc", 0));
            if (has(a, "Q"))          mcaSetTiltQ(m, (float)num(a, "Q", 0));
            g_app.settings_dirty = true;
            g_app.pushCueSettingsToDevice(*e);
            cJSON* r = cJSON_CreateObject();
            cJSON_AddBoolToObject(r, "enabled", m->tilt.enabled);
            cJSON_AddNumberToObject(r, "surge_gain", m->tilt.surge_gain);
            cJSON_AddNumberToObject(r, "sway_gain", m->tilt.sway_gain);
            cJSON_AddNumberToObject(r, "fc", m->tilt.fc);
            cJSON_AddNumberToObject(r, "Q", m->tilt.Q);
            RESULT(r);
        }
    }
    else if (!strcmp(cmd, "intensity")) {
        Entity* e = pick_entity(a);
        if (!e) { ERR("no entity"); }
        else { e->config.intensity = (float)num(a, "value", 100.0);
               g_app.settings_dirty = true;
               g_app.pushCueSettingsToDevice(*e);
               cJSON* r = cJSON_CreateObject();
               cJSON_AddNumberToObject(r, "intensity", e->config.intensity);
               RESULT(r); }
    }
    else if (!strcmp(cmd, "axis_set")) {
        Entity* e = pick_entity(a);
        int ax = (int)num(a, "axis", -1);
        if (!e) ERR("no entity");
        else if (ax < 0 || ax > 5) ERR("axis must be 0..5");
        else {
            if (has(a, "gain"))   e->config.axis_gain[ax]   = (float)num(a, "gain", 100.0);
            if (has(a, "invert")) e->config.axis_invert[ax] = boolarg(a, "invert", false);
            g_app.settings_dirty = true;
            g_app.pushCueSettingsToDevice(*e);
            RESULT(nullptr);
        }
    }
    else if (!strcmp(cmd, "state")) {
        Entity* e = pick_entity(a);
        if (!e) { ERR("no entity"); }
        else {
            auto& st = e->state;
            cJSON* r = cJSON_CreateObject();
            cJSON_AddNumberToObject(r, "entity", e->id);
            cJSON_AddItemToObject(r, "input_pct",       farr(st.input_pct, 6));
            cJSON_AddItemToObject(r, "input_physical",  farr(st.input_physical, 6));
            cJSON_AddItemToObject(r, "output_angles_deg", farr(st.output_angles_deg, 6));
            cJSON_AddItemToObject(r, "servo_util",      farr(st.servo_util, 6));
            cJSON_AddNumberToObject(r, "max_util", st.max_util);
            cJSON_AddNumberToObject(r, "valid_mask", st.valid_mask);
            RESULT(r);
        }
    }
    else if (!strcmp(cmd, "entities_list")) {
        cJSON* arr = cJSON_CreateArray();
        for (auto& e : g_app.entities) {
            cJSON* o = cJSON_CreateObject();
            cJSON_AddNumberToObject(o, "id", e.id);
            cJSON_AddStringToObject(o, "name", e.name);
            cJSON_AddStringToObject(o, "type", e.type == EntityType::HIL ? "HIL" : "SIL");
            cJSON_AddBoolToObject(o, "enabled", e.enabled);
            cJSON_AddItemToArray(arr, o);
        }
        cJSON* r = cJSON_CreateObject();
        cJSON_AddItemToObject(r, "entities", arr);
        RESULT(r);
    }
    else if (!strcmp(cmd, "entity_add")) {
        const char* nm = strarg(a, "name", "Entity");
        EntityType t = (strarg(a, "type", "SIL")[0] == 'H') ? EntityType::HIL : EntityType::SIL;
        Entity& e = g_app.addEntity(nm, t);
        cJSON* r = cJSON_CreateObject();
        cJSON_AddNumberToObject(r, "id", e.id);
        RESULT(r);
    }
    else if (!strcmp(cmd, "entity_remove")) {
        g_app.removeEntity((int)num(a, "id", -1));
        RESULT(nullptr);
    }
    else if (!strcmp(cmd, "hil_connect")) {
        // Headless equivalent of the GUI "Connect (Network)" button: attach a
        // UdpTransport (raw-HIL path) to a HIL entity so motion streams over UDP
        // to the Linux bridge, with source/control over TCP. Runs on the render
        // thread (queued) exactly like the GUI, so it's thread-safe.
        Entity* e = pick_entity(a);
        if (!e || e->type != EntityType::HIL)
            e = &g_app.addEntity(strarg(a, "name", "Network HIL"), EntityType::HIL);
        snprintf(e->hil_host, sizeof(e->hil_host), "%s", strarg(a, "host", "192.168.1.168"));
        e->hil_udp_port = (int)num(a, "udp_port", 8767);
        e->hil_tcp_port = (int)num(a, "tcp_port", 8789);
        auto up = std::make_shared<UdpTransport>(e->hil_host, e->hil_udp_port, e->hil_tcp_port);
        up->setCobsMode(true);
        e->serial = up;
        e->hil_network = true;
        e->enabled = true;
        snprintf(e->transport.udp_ip, sizeof(e->transport.udp_ip), "%s", e->hil_host);
        e->hil_tel_seq = 0;
        e->hil_cap_raw = true;
        e->hil_raw_mode = true;
        e->hil_handshake_ok = true;
        e->hil_handshake_pending = false;
        e->hil_handshake_phase = HandshakePhase::Ready;
        e->hil_last_error[0] = '\0';
        snprintf(e->hil_handshake_msg, sizeof(e->hil_handshake_msg), "%s",
                 up->controlConnected() ? "Bridge connected" : "control unreachable");
        cJSON* r = cJSON_CreateObject();
        cJSON_AddNumberToObject(r, "id", e->id);
        cJSON_AddStringToObject(r, "host", e->hil_host);
        cJSON_AddNumberToObject(r, "udp_port", e->hil_udp_port);
        cJSON_AddNumberToObject(r, "tcp_port", e->hil_tcp_port);
        cJSON_AddBoolToObject(r, "open", up->isOpen());
        cJSON_AddBoolToObject(r, "control_connected", up->controlConnected());
        RESULT(r);
    }
    else if (!strcmp(cmd, "hil_stream_test")) {
        // Stream a live surge/sway/heave motion sweep straight through a
        // connected network-HIL entity's transport (app -> UDP -> bridge -> mini).
        // Values are signed PERCENT (the raw-HIL path). Blocks the render thread
        // for `secs`, streaming at 50 Hz — proves the app is feeding real motion.
        Entity* tgt = nullptr;
        for (auto& e : g_app.entities)
            if (e.type == EntityType::HIL && e.serial && e.serial->isOpen()) { tgt = &e; break; }
        if (!tgt) { ERR("no connected HIL entity"); }
        else {
            double secs = num(a, "secs", 6.0);
            double freq = num(a, "freq", 0.35);
            double amp  = num(a, "amp",  30.0);
            int total = (int)(secs * 50.0); if (total < 1) total = 1;
            int seg = total / 3; if (seg < 1) seg = 1;
            const int axes[3] = { 0, 1, 2 };   // surge, sway, heave
            int sent = 0;
            for (int i = 0; i < total; i++) {
                float raw[6] = { 0, 0, 0, 0, 0, 0 };
                int aidx = i / seg; if (aidx > 2) aidx = 2;
                double t = (double)(i % seg) / 50.0;
                raw[axes[aidx]] = (float)(amp * sin(2.0 * 3.14159265358979 * freq * t));
                tgt->serial->sendCobsDataRaw(raw);
                sent++;
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
            float zero[6] = { 0, 0, 0, 0, 0, 0 };
            tgt->serial->sendCobsDataRaw(zero);
            cJSON* r = cJSON_CreateObject();
            cJSON_AddNumberToObject(r, "frames_sent", sent);
            cJSON_AddNumberToObject(r, "entity", tgt->id);
            RESULT(r);
        }
    }
    else if (!strcmp(cmd, "plugin")) {
        // List input plugins, or activate one as the live motion source. The GUI
        // only activates a plugin on a source *switch*; if the source is already
        // Plugin at Start, activation never fires -> zero motion. This forces it.
        const char* action = strarg(a, "action", "list");
        if (!strcmp(action, "activate")) {
            int idx = has(a, "index") ? (int)num(a, "index", -1) : -1;
            if (idx < 0) {
                const char* nm = strarg(a, "name", nullptr);
                if (nm) for (int i = 0; i < g_app.plugin_mgr.pluginCount(); i++) {
                    const char* pn = g_app.plugin_mgr.pluginName(i);
                    if (pn && strstr(pn, nm)) { idx = i; break; }
                }
            }
            if (idx < 0 || idx >= g_app.plugin_mgr.pluginCount()) { ERR("plugin not found"); }
            else {
                float sr = (float)num(a, "rate", 50.0);
                g_app.active_plugin_idx = idx;
                g_app.input_source = InputSource::Plugin;
                g_app.motion_started = true;
                bool ok = g_app.plugin_mgr.activatePlugin(idx, sr);
                const char* pn = g_app.plugin_mgr.pluginName(idx);
                cJSON* r = cJSON_CreateObject();
                cJSON_AddNumberToObject(r, "active", idx);
                cJSON_AddStringToObject(r, "name", pn ? pn : "");
                cJSON_AddBoolToObject(r, "activated", ok);
                RESULT(r);
            }
        } else {
            cJSON* arr = cJSON_CreateArray();
            for (int i = 0; i < g_app.plugin_mgr.pluginCount(); i++) {
                cJSON* o = cJSON_CreateObject();
                cJSON_AddNumberToObject(o, "index", i);
                const char* pn = g_app.plugin_mgr.pluginName(i);
                cJSON_AddStringToObject(o, "name", pn ? pn : "?");
                cJSON_AddItemToArray(arr, o);
            }
            cJSON* r = cJSON_CreateObject();
            cJSON_AddItemToObject(r, "plugins", arr);
            cJSON_AddNumberToObject(r, "active", g_app.active_plugin_idx);
            RESULT(r);
        }
    }
    else if (!strcmp(cmd, "plugin_params") || !strcmp(cmd, "plugin_param_set")) {
        const char* keys[] = {"index", "name", "value"};
        const bool setting = !strcmp(cmd, "plugin_param_set");
        const int active = g_app.plugin_mgr.activeIndex();
        cJSON* index_arg = a ? cJSON_GetObjectItem(a, "index") : nullptr;
        const int index = index_arg && cJSON_IsNumber(index_arg) ? index_arg->valueint : active;
        if (!valid_keys(a, keys, setting ? 3 : 1) ||
            (index_arg && (!cJSON_IsNumber(index_arg) || index_arg->valuedouble != index_arg->valueint))) {
            ERR("invalid plugin parameter arguments");
        } else if (index < 0 || index >= g_app.plugin_mgr.pluginCount()) {
            ERR("plugin index not found (specify index or activate a plugin)");
        } else {
            const auto& plugin = g_app.plugin_mgr.plugins()[index];
            const StewartPluginInfo* info = plugin.info;
            if (!info) ERR("plugin metadata unavailable");
            else if (setting) {
                const char* name = strarg(a, "name", "");
                cJSON* value = a ? cJSON_GetObjectItem(a, "value") : nullptr;
                const StewartParamDef* param = nullptr;
                for (int i = 0; i < info->param_count; ++i)
                    if (!strcmp(info->params[i].name, name)) param = &info->params[i];
                if (IsDocumentationMode()) ERR("live plugin parameters cannot be changed in documentation mode");
                else if (index != active || !plugin.fn_set_param) ERR("plugin must be active and support parameter changes");
                else if (!param) ERR("unknown plugin parameter");
                else if (!cJSON_IsNumber(value) || !std::isfinite(value->valuedouble) ||
                         value->valuedouble < param->min_val || value->valuedouble > param->max_val ||
                         (param->type != STEWART_PARAM_FLOAT && floor(value->valuedouble) != value->valuedouble)) {
                    ERR("parameter value is outside its declared range or type");
                } else {
                    g_app.plugin_mgr.setParam(name, float(value->valuedouble));
                    g_app.settings_dirty = true;
                    cJSON* r = cJSON_CreateObject();
                    cJSON_AddNumberToObject(r, "index", index);
                    cJSON_AddStringToObject(r, "name", name);
                    cJSON_AddNumberToObject(r, "value", value->valuedouble);
                    RESULT(r);
                }
            } else {
                cJSON* r = cJSON_CreateObject();
                cJSON_AddNumberToObject(r, "index", index);
                cJSON_AddStringToObject(r, "name", info->name);
                cJSON* params = cJSON_CreateArray();
                for (int i = 0; i < info->param_count; ++i) {
                    const auto& param = info->params[i];
                    float value = param.default_val;
                    for (const auto& current : plugin.param_values)
                        if (current.name == param.name) value = current.value;
                    cJSON* p = cJSON_CreateObject();
                    cJSON_AddStringToObject(p, "name", param.name);
                    cJSON_AddStringToObject(p, "label", param.display_name ? param.display_name : param.name);
                    cJSON_AddNumberToObject(p, "type", param.type);
                    cJSON_AddNumberToObject(p, "min", param.min_val);
                    cJSON_AddNumberToObject(p, "max", param.max_val);
                    cJSON_AddNumberToObject(p, "value", value);
                    cJSON_AddItemToArray(params, p);
                }
                cJSON_AddItemToObject(r, "parameters", params);
                RESULT(r);
            }
        }
    }
    else if (!strcmp(cmd, "serial_ports")) {
        cJSON* arr = cJSON_CreateArray();
        for (auto& p : SerialPort::enumerate()) {
            cJSON* o = cJSON_CreateObject();
            cJSON_AddStringToObject(o, "port", p.port.c_str());
            cJSON_AddStringToObject(o, "desc", p.desc.c_str());
            cJSON_AddItemToArray(arr, o);
        }
        cJSON* r = cJSON_CreateObject();
        cJSON_AddItemToObject(r, "ports", arr);
        RESULT(r);
    }
    else if (!strcmp(cmd, "settings_save")) {
        g_app.saveSettings();
        RESULT(nullptr);
    }
    else if (!strcmp(cmd, "motion")) {
        g_app.motion_started = boolarg(a, "on", true);
        cJSON* r = cJSON_CreateObject();
        cJSON_AddBoolToObject(r, "motion_started", g_app.motion_started);
        RESULT(r);
    }
    else if (!strcmp(cmd, "analyze")) {
        // Offline-bake stats for the entity's current tuning over a recording,
        // WITHOUT writing a file. Fast envelope check for MCA/intensity sweeps.
        Entity* e = pick_entity(a);
        int idx = g_app.capture_playback_idx;
        cJSON* rec = a ? cJSON_GetObjectItem(a, "recording") : nullptr;
        if (rec) {
            if (cJSON_IsNumber(rec)) idx = rec->valueint;
            else if (cJSON_IsString(rec)) {
                idx = -1;
                for (int i = 0; i < (int)g_app.saved_recordings.size(); i++)
                    if (strcmp(g_app.saved_recordings[i].name, rec->valuestring) == 0) { idx = i; break; }
            }
        }
        int rate = (int)num(a, "rate", 50);
        int bits = (int)num(a, "bits", 12);
        double pre = num(a, "preroll_s", 5.0);
        if (!e) ERR("no entity");
        else {
            std::vector<uint16_t> seq; BakeStats st;
            if (!bake_sequence(idx, *e, rate, bits, pre, seq, st)) ERR("bake failed (bad recording?)");
            else RESULT(stats_json(st, rate));
        }
    }
    else if (!strcmp(cmd, "export_sequence")) {
        // Phase 3 freeze: bake the entity's current tuning to a .m6p device blob.
        Entity* e = pick_entity(a);
        int idx = resolve_recording(a);
        if (idx < 0 && has(a, "recording")) {
            if (cJSON_IsNumber(cJSON_GetObjectItem(a, "recording")))
                idx = (int)num(a, "recording", -1);
            else { const char* nm = strarg(a, "recording", nullptr);
                   if (nm) for (int i = 0; i < (int)g_app.saved_recordings.size(); i++)
                       if (strcmp(g_app.saved_recordings[i].name, nm) == 0) { idx = i; break; } }
        }
        if (idx < 0) idx = g_app.capture_playback_idx;
        int rate = (int)num(a, "rate", 50);
        int bits = (int)num(a, "bits", 12);
        double pre = num(a, "preroll_s", 5.0);
        int loop_point = (int)num(a, "loop_point", 0);
        const char* path = strarg(a, "path", "sequence.m6p");
        bool raw = boolarg(a, "raw", false);   // M6P2 float32 raw (pre-cueing) export
        if (!e) { ERR("no entity"); }
        else if (idx < 0) { ERR("no recording"); }
        else if (raw) {
            // ── M6P2 RAW export: 6x float32 LE, PRE-cueing, app axis order (no swap).
            // The device runs the single cue engine at playback, so this is the
            // lossless master (change-list (a) / DECISIONS round 3 float32 decision).
            const SavedRecording& sr = g_app.saved_recordings[idx];
            double dur = sr.samples.empty() ? 0.0 : sr.samples.back().time;
            int count = (int)(dur * rate) + 1;
            std::vector<uint8_t> data((size_t)count * 6 * 4);
            BakeStats st; for (int i = 0; i < 6; i++) { st.mn[i] = 1e9f; st.mx[i] = -1e9f; st.sat[i] = 0; }
            for (int k = 0; k < count; k++) {
                float pct[6]; interp_recording(sr, (double)k / rate, pct);
                for (int i = 0; i < 6; i++) {
                    if (pct[i] < st.mn[i]) st.mn[i] = pct[i];
                    if (pct[i] > st.mx[i]) st.mx[i] = pct[i];
                    if (pct[i] > 100.0f || pct[i] < -100.0f) st.sat[i] += 1.0;
                    uint32_t bitsLE; memcpy(&bitsLE, &pct[i], 4);   // host LE
                    uint8_t* d = &data[((size_t)k * 6 + i) * 4];
                    d[0] = (uint8_t)(bitsLE & 0xFF);       d[1] = (uint8_t)((bitsLE >> 8) & 0xFF);
                    d[2] = (uint8_t)((bitsLE >> 16) & 0xFF); d[3] = (uint8_t)((bitsLE >> 24) & 0xFF);
                }
            }
            st.n = count;
            for (int i = 0; i < 6; i++) st.sat[i] = count ? st.sat[i] / count * 100.0 : 0.0;
            uint32_t crc = crc32_buf(data.data(), data.size());
            uint8_t hdr[64] = {0};
            memcpy(hdr, "M6P2", 4);
            uint16_t ver = 2, r16 = (uint16_t)rate;
            memcpy(hdr + 4, &ver, 2);
            memcpy(hdr + 6, &r16, 2);
            uint32_t c32 = (uint32_t)count, lp = (uint32_t)loop_point;
            memcpy(hdr + 8,  &c32, 4);
            memcpy(hdr + 12, &lp, 4);
            const char* nm = g_app.saved_recordings[idx].name;
            strncpy((char*)hdr + 16, nm, 31);
            hdr[48] = 1;   // format: 1 = float32 (6 channels)
            hdr[49] = 6;   // channel count
            memcpy(hdr + 52, &crc, 4);
            FILE* f = fopen(path, "wb");
            if (!f) ERR("cannot open output path");
            else {
                fwrite(hdr, 1, 64, f);
                fwrite(data.data(), 1, data.size(), f);
                fclose(f);
                cJSON* r = stats_json(st, rate);
                cJSON_AddStringToObject(r, "path", path);
                cJSON_AddStringToObject(r, "format", "M6P2-float32");
                cJSON_AddNumberToObject(r, "bytes", (double)(64 + data.size()));
                cJSON_AddNumberToObject(r, "loop_point", loop_point);
                char crchex[16]; snprintf(crchex, sizeof(crchex), "%08X", crc);
                cJSON_AddStringToObject(r, "crc32", crchex);
                cJSON_AddStringToObject(r, "name", nm);
                g_app.log(-1, "system", "Exported RAW %s (%d samples @ %dHz float32, %u B)",
                          path, count, rate, (unsigned)(64 + data.size()));
                RESULT(r);
            }
        }
        else {
            std::vector<uint16_t> seq; BakeStats st;
            if (!bake_sequence(idx, *e, rate, bits, pre, seq, st)) ERR("bake failed");
            else {
                int count = st.n;
                std::vector<uint8_t> data(seq.size() * 2);
                for (size_t i = 0; i < seq.size(); i++) {   // little-endian u16
                    data[i*2]   = (uint8_t)(seq[i] & 0xFF);
                    data[i*2+1] = (uint8_t)(seq[i] >> 8);
                }
                uint32_t crc = crc32_buf(data.data(), data.size());
                // 64-byte header
                uint8_t hdr[64] = {0};
                memcpy(hdr, "M6P1", 4);
                uint16_t ver = 1, r16 = (uint16_t)rate;
                memcpy(hdr + 4, &ver, 2);
                memcpy(hdr + 6, &r16, 2);
                uint32_t c32 = (uint32_t)count, lp = (uint32_t)loop_point;
                memcpy(hdr + 8,  &c32, 4);
                memcpy(hdr + 12, &lp, 4);
                const char* nm = g_app.saved_recordings[idx].name;
                strncpy((char*)hdr + 16, nm, 31);
                uint16_t bd = (uint16_t)bits;
                memcpy(hdr + 48, &bd, 2);
                memcpy(hdr + 52, &crc, 4);
                FILE* f = fopen(path, "wb");
                if (!f) ERR("cannot open output path");
                else {
                    fwrite(hdr, 1, 64, f);
                    fwrite(data.data(), 1, data.size(), f);
                    fclose(f);
                    cJSON* r = stats_json(st, rate);
                    cJSON_AddStringToObject(r, "path", path);
                    cJSON_AddNumberToObject(r, "bytes", (double)(64 + data.size()));
                    cJSON_AddNumberToObject(r, "bits", bits);
                    cJSON_AddNumberToObject(r, "loop_point", loop_point);
                    char crchex[16]; snprintf(crchex, sizeof(crchex), "%08X", crc);
                    cJSON_AddStringToObject(r, "crc32", crchex);
                    cJSON_AddStringToObject(r, "name", nm);
                    g_app.log(-1, "system", "Exported %s (%d samples @ %dHz, %u B)",
                              path, count, rate, (unsigned)(64 + data.size()));
                    RESULT(r);
                }
            }
        }
    }
    else if (!strcmp(cmd, "hil_command")) {
        // Device command passthrough (UI I/O-tab equivalent): queue an ASCII
        // command (e.g. "SOURCE:LIVE", "VERSION?") to a HIL entity's device.
        // Responses arrive as esp32 log lines — read them via log_tail.
        Entity* e = pick_entity(a);
        const char* c = strarg(a, "command", nullptr);
        if (!e || e->type != EntityType::HIL) ERR("no HIL entity");
        else if (!e->serial || !e->serial->isOpen()) ERR("HIL not connected");
        else if (!c || !c[0]) ERR("missing command");
        else {
            bool direct = boolarg(a, "direct", false);
            if (direct) e->serial->sendCommand(c);
            else        e->hil_cmd_queue.push_back(c);
            cJSON* r = cJSON_CreateObject();
            cJSON_AddNumberToObject(r, "entity", e->id);
            cJSON_AddStringToObject(r, "command", c);
            cJSON_AddStringToObject(r, "mode", direct ? "sent" : "queued");
            RESULT(r);
        }
    }
    else if (!strcmp(cmd, "hil_status")) {
        // Rich HIL link state (nothing else exposes handshake/raw-mode/tel).
        Entity* e = pick_entity(a);
        if (!e || e->type != EntityType::HIL) ERR("no HIL entity");
        else {
            cJSON* r = cJSON_CreateObject();
            cJSON_AddNumberToObject(r, "entity", e->id);
            cJSON_AddStringToObject(r, "port", e->hil_port);
            cJSON_AddBoolToObject(r, "network", e->hil_network);
            cJSON_AddBoolToObject(r, "open", e->serial && e->serial->isOpen());
            cJSON_AddBoolToObject(r, "handshake_ok", e->hil_handshake_ok);
            cJSON_AddStringToObject(r, "handshake_msg", e->hil_handshake_msg);
            cJSON_AddBoolToObject(r, "cap_raw", e->hil_cap_raw);
            cJSON_AddBoolToObject(r, "raw_mode", e->hil_raw_mode);
            cJSON_AddNumberToObject(r, "tx_hz", e->hil_tx_hz);
            cJSON_AddBoolToObject(r, "tel_active", e->hil_tel_active);
            cJSON_AddNumberToObject(r, "tel_rate_hz", e->rate_tel_hz);
            cJSON_AddNumberToObject(r, "bit_depth", e->config.bit_depth);
            cJSON_AddStringToObject(r, "last_error", e->hil_last_error);
            RESULT(r);
        }
    }
    else if (!strcmp(cmd, "hil_raw_mode")) {
        // Toggle RAW (ESP cues, CH_DATA_RAW float32) vs BAKED streaming.
        Entity* e = pick_entity(a);
        if (!e || e->type != EntityType::HIL) ERR("no HIL entity");
        else if (boolarg(a, "value", true) && !e->hil_cap_raw) ERR("device does not advertise raw-HIL");
        else {
            e->hil_raw_mode = boolarg(a, "value", true);
            g_app.settings_dirty = true;
            cJSON* r = cJSON_CreateObject();
            cJSON_AddBoolToObject(r, "raw_mode", e->hil_raw_mode);
            RESULT(r);
        }
    }
    else if (!strcmp(cmd, "hil_tx_rate")) {
        Entity* e = pick_entity(a);
        if (!e || e->type != EntityType::HIL) ERR("no HIL entity");
        else {
            int hz = (int)num(a, "hz", e->hil_tx_hz);
            if (hz < 10) hz = 10; if (hz > 1000) hz = 1000;
            e->hil_tx_hz = hz;
            g_app.settings_dirty = true;
            cJSON* r = cJSON_CreateObject();
            cJSON_AddNumberToObject(r, "tx_hz", e->hil_tx_hz);
            RESULT(r);
        }
    }
    else if (!strcmp(cmd, "estop")) {
        // Mirror of the toolbar E-STOP: kill motion, playback, plugin, inputs.
        g_app.motion_started = false;
        if (g_app.capture_playing) g_app.stopCapturePlayback();
        if (g_app.plugin_mgr.activeIndex() >= 0)
            g_app.plugin_mgr.deactivateActive();
        for (auto& e : g_app.entities)
            memset(e.state.input_pct, 0, sizeof(e.state.input_pct));
        {
            std::lock_guard<std::mutex> lock(g_app.input_mutex);
            memset(g_app.shared_input, 0, sizeof(g_app.shared_input));
        }
        g_app.log(-1, "system", "E-STOP via control API");
        RESULT(nullptr);
    }
    else if (!strcmp(cmd, "entity_enable")) {
        Entity* e = g_app.findEntity((int)num(a, "id", -1));
        if (!e) ERR("entity not found");
        else {
            e->enabled = boolarg(a, "enabled", true);
            g_app.settings_dirty = true;
            cJSON* r = cJSON_CreateObject();
            cJSON_AddNumberToObject(r, "id", e->id);
            cJSON_AddBoolToObject(r, "enabled", e->enabled);
            RESULT(r);
        }
    }
    else if (!strcmp(cmd, "record")) {
        // {action:"start"|"stop", rate?}  — REC button equivalent.
        const char* action = strarg(a, "action", "");
        if (has(a, "rate")) {
            int rr = (int)num(a, "rate", g_app.record_rate_hz);
            if (rr < 10) rr = 10; if (rr > 1000) rr = 1000;
            g_app.record_rate_hz = rr;
        }
        bool is_rec = (g_app.recording.mode == RecordMode::Recording);
        if (!strcmp(action, "start")) {
            if (is_rec) ERR("already recording");
            else if (!g_app.motion_started) ERR("motion not started");
            else { g_app.startRecording(); RESULT(nullptr); }
        } else if (!strcmp(action, "stop")) {
            if (!is_rec) ERR("not recording");
            else { g_app.stopRecording(); RESULT(nullptr); }
        } else {
            cJSON* r = cJSON_CreateObject();
            cJSON_AddBoolToObject(r, "recording", is_rec);
            cJSON_AddNumberToObject(r, "rate_hz", g_app.record_rate_hz);
            cJSON_AddNumberToObject(r, "samples", (double)g_app.recording.samples.size());
            RESULT(r);
        }
    }
    else if (!strcmp(cmd, "recording_delete")) {
        int idx = resolve_recording(a);
        if (idx < 0) ERR("recording not found");
        else { g_app.deleteRecordingFromLibrary(idx); RESULT(nullptr); }
    }
    else if (!strcmp(cmd, "recording_rename")) {
        int idx = resolve_recording(a);
        const char* nn = strarg(a, "new_name", nullptr);
        if (idx < 0) ERR("recording not found");
        else if (!nn || !nn[0]) ERR("missing new_name");
        else {
            snprintf(g_app.saved_recordings[idx].name,
                     sizeof(g_app.saved_recordings[idx].name), "%s", nn);
            g_app.saveRecordingsToDisk();
            cJSON* r = cJSON_CreateObject();
            cJSON_AddNumberToObject(r, "index", idx);
            cJSON_AddStringToObject(r, "name", nn);
            RESULT(r);
        }
    }
    else if (!strcmp(cmd, "recordings_save")) {
        g_app.saveRecordingsToDisk();
        cJSON* r = cJSON_CreateObject();
        cJSON_AddNumberToObject(r, "count", (double)g_app.saved_recordings.size());
        RESULT(r);
    }
    else if (!strcmp(cmd, "geometry_get")) {
        Entity* e = pick_entity(a);
        if (!e) ERR("no entity");
        else {
            auto& g = e->config.geometry;
            cJSON* r = cJSON_CreateObject();
            cJSON_AddNumberToObject(r, "entity", e->id);
            cJSON_AddNumberToObject(r, "RD", g.RD);
            cJSON_AddNumberToObject(r, "PD", g.PD);
            cJSON_AddNumberToObject(r, "L1", g.ServoArmLengthL1);
            cJSON_AddNumberToObject(r, "L2", g.ConnectingArmLengthL2);
            cJSON_AddNumberToObject(r, "height", g.platformHeight);
            cJSON_AddNumberToObject(r, "theta_r", g.theta_r);
            cJSON_AddNumberToObject(r, "theta_p", g.theta_p);
            RESULT(r);
        }
    }
    else if (!strcmp(cmd, "geometry_set")) {
        Entity* e = pick_entity(a);
        if (!e) ERR("no entity");
        else {
            auto& g = e->config.geometry;
            if (has(a, "RD"))      g.RD = (float)num(a, "RD", g.RD);
            if (has(a, "PD"))      g.PD = (float)num(a, "PD", g.PD);
            if (has(a, "L1"))      g.ServoArmLengthL1 = (float)num(a, "L1", g.ServoArmLengthL1);
            if (has(a, "L2"))      g.ConnectingArmLengthL2 = (float)num(a, "L2", g.ConnectingArmLengthL2);
            if (has(a, "height"))  g.platformHeight = (float)num(a, "height", g.platformHeight);
            if (has(a, "theta_r")) g.theta_r = (float)num(a, "theta_r", g.theta_r);
            if (has(a, "theta_p")) g.theta_p = (float)num(a, "theta_p", g.theta_p);
            g_app.settings_dirty = true;
            RESULT(nullptr);
        }
    }
    else if (!strcmp(cmd, "geometry_push")) {
        // "Push Settings to ESP32" button equivalent: queue CONFIG: commands.
        Entity* e = pick_entity(a);
        if (!e || e->type != EntityType::HIL) ERR("no HIL entity");
        else if (!e->serial || !e->serial->isOpen()) ERR("HIL not connected");
        else {
            auto& geo = e->config.geometry;
            char c[64];
            e->hil_cmd_queue.clear();
            snprintf(c, sizeof(c), "CONFIG:RD=%.4f", geo.RD);                    e->hil_cmd_queue.push_back(c);
            snprintf(c, sizeof(c), "CONFIG:PD=%.4f", geo.PD);                    e->hil_cmd_queue.push_back(c);
            snprintf(c, sizeof(c), "CONFIG:L1=%.4f", geo.ServoArmLengthL1);      e->hil_cmd_queue.push_back(c);
            snprintf(c, sizeof(c), "CONFIG:L2=%.4f", geo.ConnectingArmLengthL2); e->hil_cmd_queue.push_back(c);
            snprintf(c, sizeof(c), "CONFIG:height=%.4f", geo.platformHeight);    e->hil_cmd_queue.push_back(c);
            snprintf(c, sizeof(c), "CONFIG:theta_r=%.4f", geo.theta_r);          e->hil_cmd_queue.push_back(c);
            snprintf(c, sizeof(c), "CONFIG:theta_p=%.4f", geo.theta_p);          e->hil_cmd_queue.push_back(c);
            e->hil_cmd_queue.push_back("CONFIG?");
            g_app.log(e->id, "hil", "Queued geometry push via control API (8 commands)");
            cJSON* r = cJSON_CreateObject();
            cJSON_AddNumberToObject(r, "queued", 8);
            RESULT(r);
        }
    }
    else if (!strcmp(cmd, "mca_push")) {
        // Explicitly forward the entity's cue settings to the raw-HIL device.
        Entity* e = pick_entity(a);
        if (!e) ERR("no entity");
        else {
            g_app.pushCueSettingsToDevice(*e);
            cJSON* r = cJSON_CreateObject();
            cJSON_AddNumberToObject(r, "queued", (double)e->hil_cmd_queue.size());
            RESULT(r);
        }
    }
    else if (!strcmp(cmd, "mca_save")) {
        // Push cue settings then persist them on the device (MCA:SAVE → NVS).
        Entity* e = pick_entity(a);
        if (!e || e->type != EntityType::HIL) ERR("no HIL entity");
        else if (!e->serial || !e->serial->isOpen()) ERR("HIL not connected");
        else {
            g_app.pushCueSettingsToDevice(*e);
            e->hil_cmd_queue.push_back("MCA:SAVE");
            g_app.saveSettings();
            cJSON* r = cJSON_CreateObject();
            cJSON_AddNumberToObject(r, "queued", (double)e->hil_cmd_queue.size());
            cJSON_AddBoolToObject(r, "device_persist", true);
            RESULT(r);
        }
    }
    else if (!strcmp(cmd, "console_rate")) {
        // Console input-log rate: 0=off 1=1s 2=100ms 3=30Hz 4=60Hz 5=every frame.
        int rr = (int)num(a, "rate", g_app.console_log_rate);
        if (rr < 0) rr = 0; if (rr > 5) rr = 5;
        g_app.console_log_rate = rr;
        cJSON* r = cJSON_CreateObject();
        cJSON_AddNumberToObject(r, "rate", g_app.console_log_rate);
        RESULT(r);
    }
    else if (!strcmp(cmd, "quit")) {
        g_app.running = false;
        RESULT(nullptr);
    }
    else {
        ERR("unknown cmd");
    }

    cJSON_Delete(root);
    return out;
}

} // namespace

// ── drain: execute queued commands on the render thread ─────────────
void ControlServer::drain() {
    for (;;) {
        Req r;
        {
            std::lock_guard<std::mutex> lk(g_qmtx);
            if (g_queue.empty()) break;
            r = std::move(g_queue.front());
            g_queue.pop_front();
        }
        std::string resp;
        try { resp = handle(r.line); }
        catch (...) { resp = err_resp(0, "handler exception"); }
        r.resp.set_value(resp);
    }
}

#ifdef _WIN32
// ── per-connection reader thread ────────────────────────────────────
static void connection_loop(SOCKET s) {
    std::string buf;
    char rx[2048];
    while (!g_stop.load()) {
        int n = recv(s, rx, sizeof(rx), 0);
        if (n <= 0) break;
        buf.append(rx, n);
        size_t nl;
        while ((nl = buf.find('\n')) != std::string::npos) {
            std::string line = buf.substr(0, nl);
            buf.erase(0, nl + 1);
            if (!line.empty() && line.back() == '\r') line.pop_back();
            if (line.empty()) continue;

            std::future<std::string> fut;
            {
                std::lock_guard<std::mutex> lk(g_qmtx);
                g_queue.emplace_back();
                g_queue.back().line = line;
                fut = g_queue.back().resp.get_future();
            }
            std::string resp;
            if (fut.wait_for(std::chrono::seconds(10)) == std::future_status::ready)
                resp = fut.get();
            else
                resp = err_resp(0, "timeout");
            resp.push_back('\n');
            if (send(s, resp.data(), (int)resp.size(), 0) <= 0) { closesocket(s); return; }
        }
    }
    closesocket(s);
}

static void accept_loop() {
    while (!g_stop.load()) {
        sockaddr_in cli; int len = sizeof(cli);
        SOCKET c = accept(g_listen, (sockaddr*)&cli, &len);
        if (c == INVALID_SOCKET) {
            if (g_stop.load()) break;
            continue;
        }
        std::lock_guard<std::mutex> lk(g_conn_mtx);
        g_conn_threads.emplace_back(connection_loop, c);
    }
}
#endif

bool ControlServer::start(int port) {
#ifdef _WIN32
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) return false;
    g_listen = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (g_listen == INVALID_SOCKET) return false;
    int yes = 1;
    setsockopt(g_listen, SOL_SOCKET, SO_REUSEADDR, (char*)&yes, sizeof(yes));
    sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons((u_short)port);
    inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);
    if (bind(g_listen, (sockaddr*)&addr, sizeof(addr)) != 0) {
        closesocket(g_listen); g_listen = INVALID_SOCKET; return false;
    }
    if (listen(g_listen, 4) != 0) {
        closesocket(g_listen); g_listen = INVALID_SOCKET; return false;
    }
    g_stop.store(false);
    m_port = port;
    m_running.store(true);
    g_accept_thread = std::thread(accept_loop);
    g_app.log(-1, "system", "Control API listening on 127.0.0.1:%d", port);
    return true;
#else
    (void)port; return false;
#endif
}

void ControlServer::stop() {
#ifdef _WIN32
    if (!m_running.load()) return;
    g_stop.store(true);
    if (g_listen != INVALID_SOCKET) { closesocket(g_listen); g_listen = INVALID_SOCKET; }
    if (g_accept_thread.joinable()) g_accept_thread.join();
    {
        std::lock_guard<std::mutex> lk(g_conn_mtx);
        for (auto& t : g_conn_threads) if (t.joinable()) t.detach();
        g_conn_threads.clear();
    }
    // Unblock any waiters still in the queue.
    {
        std::lock_guard<std::mutex> lk(g_qmtx);
        for (auto& r : g_queue) r.resp.set_value(err_resp(0, "shutdown"));
        g_queue.clear();
    }
    WSACleanup();
    m_running.store(false);
#endif
}
