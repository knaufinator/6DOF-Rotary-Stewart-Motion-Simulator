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
#include "serial_port.h"
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

    if (!strcmp(cmd, "ping")) {
        cJSON* r = cJSON_CreateObject();
        cJSON_AddStringToObject(r, "app", "stewart-platform");
        cJSON_AddNumberToObject(r, "api", 1);
        cJSON_AddNumberToObject(r, "fps", g_app.fps);
        cJSON_AddBoolToObject(r, "running", g_app.running);
        RESULT(r);
    }
    else if (!strcmp(cmd, "status")) {
        cJSON* r = cJSON_CreateObject();
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
        char def[512];
        snprintf(def, sizeof(def),
                 "C:\\Users\\Chris\\AppData\\Local\\Temp\\claude\\stewart_shot.png");
        const char* path = strarg(a, "path", def);
        int w = 0, h = 0;
        if (CaptureWindowPNG(path, &w, &h)) {
            cJSON* r = cJSON_CreateObject();
            cJSON_AddStringToObject(r, "path", path);
            cJSON_AddNumberToObject(r, "width", w);
            cJSON_AddNumberToObject(r, "height", h);
            RESULT(r);
        } else ERR("capture failed");
    }
    else if (!strcmp(cmd, "log_tail")) {
        int n = (int)num(a, "n", 30);
        if (n < 1) n = 1; if (n > 500) n = 500;
        int total = (int)g_app.console_log.size();
        int start = total > n ? total - n : 0;
        cJSON* arr = cJSON_CreateArray();
        for (int i = start; i < total; i++) {
            auto& L = g_app.console_log[i];
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
        if (!e) { ERR("no entity"); }
        else if (idx < 0) { ERR("no recording"); }
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
