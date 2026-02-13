#include "app.h"
#include "serial_port.h"
#include <cstdarg>
#include <cstdio>
#include <cmath>
#include <ctime>
#include <chrono>
#include "cJSON.h"

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <fcntl.h>
#define INVALID_SOCKET -1
#define SOCKET_ERROR -1
#define closesocket close
typedef int SOCKET;
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Global instance
App g_app;

// ── PipelineConfig ──────────────────────────────────────────────────

void PipelineConfig::initDefaults() {
    initDefaultStewartConfig(&geometry);
    computeStepsPerDegree(&geometry);
    rebuildPlatform();
    computeAxisScalesFromGeometry(&axis_scales, &geometry, 0.9f);
    memset(&mca, 0, sizeof(mca));
    mca.enabled = 0;  // MCA off by default for manual sliders
    occupant[0] = 0.0f;
    occupant[1] = 0.0f;
    occupant[2] = 800.0f;
    bit_depth = 12;
    // Phase D: default intensity and per-axis gains
    intensity = 100.0f;
    for (int i = 0; i < 6; i++) axis_gain[i] = 100.0f;
}

void PipelineConfig::rebuildPlatform() {
    buildPlatformFromConfig(&geometry, &platform);
}

// ── App ─────────────────────────────────────────────────────────────

static float g_entity_colors[][4] = {
    {0.38f, 0.65f, 0.96f, 1.0f},  // blue
    {0.20f, 0.83f, 0.60f, 1.0f},  // green
    {0.98f, 0.74f, 0.18f, 1.0f},  // amber
    {0.75f, 0.50f, 0.95f, 1.0f},  // purple
    {0.97f, 0.44f, 0.44f, 1.0f},  // red
    {0.13f, 0.83f, 0.87f, 1.0f},  // cyan
};
static const int g_num_colors = sizeof(g_entity_colors) / sizeof(g_entity_colors[0]);

App::App()
    : next_entity_id(0)
    , input_source(InputSource::Manual)
    , simtools_active(false)
    , simtools_port(4123)
    , simtools_bit_depth(12)
    , simtools_rate(0.0f)
    , console_max(500)
    , console_auto_scroll(true)
    , console_filter(-1)
    , running(true)
    , frame_time(0.0)
    , fps(0.0)
    , frame_count(0)
    , last_input_log_time(0.0)
    , console_log_rate(2)  // default: 10 Hz
    , capture_playback_idx(-1)
    , capture_playing(false)
    , capture_play_cursor(0)
    , capture_start_time(0.0)
    , capture_loop(true)
    , capture_speed(1.0f)
    , record_rate_hz(200)
    , test_signal_start_time(0.0)
    , input_history_head(0)
    , input_history_count(0)
    , input_history_last_push(0.0)
    , input_spectrum_freq_max(100.0f)
    , settings_dirty(false)
{
    memset(shared_input, 0, sizeof(shared_input));
    memset(input_history, 0, sizeof(input_history));
    memset(input_history_time, 0, sizeof(input_history_time));
    memset(input_spectrum, 0, sizeof(input_spectrum));

#ifdef _WIN32
    WSADATA wsa;
    WSAStartup(MAKEWORD(2, 2), &wsa);
#endif
}

App::~App() {
    stopUdpListener();
#ifdef _WIN32
    WSACleanup();
#endif
}

static const char* SETTINGS_FILE = "stewart_settings.json";

static void saveEntityToJSON(cJSON* ej, const Entity& e) {
    cJSON_AddStringToObject(ej, "name", e.name);
    cJSON_AddNumberToObject(ej, "type", (int)e.type);
    cJSON_AddBoolToObject(ej, "enabled", e.enabled);
    cJSON_AddNumberToObject(ej, "intensity", e.config.intensity);
    cJSON_AddNumberToObject(ej, "bit_depth", e.config.bit_depth);

    // Color
    cJSON* col = cJSON_AddArrayToObject(ej, "color");
    for (int i = 0; i < 4; i++) cJSON_AddItemToArray(col, cJSON_CreateNumber(e.color[i]));

    // Per-axis gain
    cJSON* gains = cJSON_AddArrayToObject(ej, "axis_gain");
    for (int i = 0; i < 6; i++) cJSON_AddItemToArray(gains, cJSON_CreateNumber(e.config.axis_gain[i]));

    // Geometry
    cJSON* geo = cJSON_AddObjectToObject(ej, "geometry");
    cJSON_AddNumberToObject(geo, "RD", e.config.geometry.RD);
    cJSON_AddNumberToObject(geo, "PD", e.config.geometry.PD);
    cJSON_AddNumberToObject(geo, "L1", e.config.geometry.ServoArmLengthL1);
    cJSON_AddNumberToObject(geo, "L2", e.config.geometry.ConnectingArmLengthL2);
    cJSON_AddNumberToObject(geo, "z_home", e.config.geometry.platformHeight);
    cJSON_AddNumberToObject(geo, "theta_r", e.config.geometry.theta_r);
    cJSON_AddNumberToObject(geo, "theta_p", e.config.geometry.theta_p);
    cJSON_AddNumberToObject(geo, "encoder_ppr", e.config.geometry.encoder_ppr);
    cJSON_AddNumberToObject(geo, "virtual_gear", e.config.geometry.virtual_gear);
    cJSON_AddNumberToObject(geo, "planetary_ratio", e.config.geometry.planetary_ratio);

    // HIL-specific
    if (e.type == EntityType::HIL) {
        cJSON* hil = cJSON_AddObjectToObject(ej, "hil");
        cJSON_AddStringToObject(hil, "port", e.hil_port);
        cJSON_AddNumberToObject(hil, "tx_hz", e.hil_tx_hz);
        cJSON_AddBoolToObject(hil, "auto_connect", e.hil_auto_connect);
    }
}

static void loadEntityFromJSON(Entity& e, cJSON* ej) {
    cJSON* val;
    if ((val = cJSON_GetObjectItem(ej, "name")))      snprintf(e.name, sizeof(e.name), "%s", val->valuestring);
    if ((val = cJSON_GetObjectItem(ej, "enabled")))    e.enabled = cJSON_IsTrue(val);
    if ((val = cJSON_GetObjectItem(ej, "intensity")))  e.config.intensity = (float)val->valuedouble;
    if ((val = cJSON_GetObjectItem(ej, "bit_depth")))  e.config.bit_depth = val->valueint;

    // Color
    cJSON* col = cJSON_GetObjectItem(ej, "color");
    if (col && cJSON_IsArray(col)) {
        for (int i = 0; i < 4 && i < cJSON_GetArraySize(col); i++)
            e.color[i] = (float)cJSON_GetArrayItem(col, i)->valuedouble;
    }

    // Per-axis gain
    cJSON* gains = cJSON_GetObjectItem(ej, "axis_gain");
    if (gains && cJSON_IsArray(gains)) {
        for (int i = 0; i < 6 && i < cJSON_GetArraySize(gains); i++)
            e.config.axis_gain[i] = (float)cJSON_GetArrayItem(gains, i)->valuedouble;
    }

    // Geometry
    cJSON* geo = cJSON_GetObjectItem(ej, "geometry");
    if (geo) {
        if ((val = cJSON_GetObjectItem(geo, "RD")))                e.config.geometry.RD = (float)val->valuedouble;
        if ((val = cJSON_GetObjectItem(geo, "PD")))                e.config.geometry.PD = (float)val->valuedouble;
        if ((val = cJSON_GetObjectItem(geo, "L1")))                e.config.geometry.ServoArmLengthL1 = (float)val->valuedouble;
        if ((val = cJSON_GetObjectItem(geo, "L2")))                e.config.geometry.ConnectingArmLengthL2 = (float)val->valuedouble;
        if ((val = cJSON_GetObjectItem(geo, "z_home")))            e.config.geometry.platformHeight = (float)val->valuedouble;
        if ((val = cJSON_GetObjectItem(geo, "theta_r")))           e.config.geometry.theta_r = (float)val->valuedouble;
        if ((val = cJSON_GetObjectItem(geo, "theta_p")))           e.config.geometry.theta_p = (float)val->valuedouble;
        if ((val = cJSON_GetObjectItem(geo, "encoder_ppr")))       e.config.geometry.encoder_ppr = val->valueint;
        if ((val = cJSON_GetObjectItem(geo, "virtual_gear")))      e.config.geometry.virtual_gear = (float)val->valuedouble;
        if ((val = cJSON_GetObjectItem(geo, "planetary_ratio")))   e.config.geometry.planetary_ratio = (float)val->valuedouble;
        e.config.rebuildPlatform();
    }

    // HIL-specific
    cJSON* hil = cJSON_GetObjectItem(ej, "hil");
    if (hil) {
        if ((val = cJSON_GetObjectItem(hil, "port")))  snprintf(e.hil_port, sizeof(e.hil_port), "%s", val->valuestring);
        if ((val = cJSON_GetObjectItem(hil, "tx_hz"))) e.hil_tx_hz = val->valueint;
        if ((val = cJSON_GetObjectItem(hil, "auto_connect"))) e.hil_auto_connect = cJSON_IsTrue(val);
    }
}

void App::saveSettings() {
    cJSON* root = cJSON_CreateObject();

    // SimTools
    cJSON_AddNumberToObject(root, "simtools_port", simtools_port);
    cJSON_AddNumberToObject(root, "simtools_bit_depth", simtools_bit_depth);

    // Console
    cJSON_AddNumberToObject(root, "console_log_rate", console_log_rate);
    cJSON_AddNumberToObject(root, "console_max", console_max);
    cJSON_AddBoolToObject(root, "console_auto_scroll", console_auto_scroll);

    // Recording
    cJSON_AddNumberToObject(root, "record_rate_hz", record_rate_hz);

    // Input source
    cJSON_AddNumberToObject(root, "input_source", (int)input_source);

    // Entities — full serialization
    cJSON* ents = cJSON_AddArrayToObject(root, "entities");
    for (auto& e : entities) {
        cJSON* ej = cJSON_CreateObject();
        saveEntityToJSON(ej, e);
        cJSON_AddItemToArray(ents, ej);
    }

    char* str = cJSON_Print(root);
    if (str) {
        FILE* f = fopen(SETTINGS_FILE, "w");
        if (f) {
            fputs(str, f);
            fclose(f);
        }
        cJSON_free(str);
    }
    cJSON_Delete(root);
    settings_dirty = false;
}

void App::loadSettings() {
    FILE* f = fopen(SETTINGS_FILE, "r");
    if (!f) return;

    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (sz <= 0) { fclose(f); return; }

    char* buf = (char*)malloc(sz + 1);
    fread(buf, 1, sz, f);
    buf[sz] = 0;
    fclose(f);

    cJSON* root = cJSON_Parse(buf);
    free(buf);
    if (!root) return;

    // SimTools
    cJSON* val;
    if ((val = cJSON_GetObjectItem(root, "simtools_port")))      simtools_port = val->valueint;
    if ((val = cJSON_GetObjectItem(root, "simtools_bit_depth")))  simtools_bit_depth = val->valueint;

    // Console
    if ((val = cJSON_GetObjectItem(root, "console_log_rate")))    console_log_rate = val->valueint;
    if ((val = cJSON_GetObjectItem(root, "console_max")))         console_max = val->valueint;
    if ((val = cJSON_GetObjectItem(root, "console_auto_scroll"))) console_auto_scroll = cJSON_IsTrue(val);

    // Recording
    if ((val = cJSON_GetObjectItem(root, "record_rate_hz")))    record_rate_hz = val->valueint;

    // Input source
    if ((val = cJSON_GetObjectItem(root, "input_source"))) {
        int src = val->valueint;
        if (src >= 0 && src <= (int)InputSource::TestSignal)
            input_source = (InputSource)src;
    }

    // Entities — clear and recreate from saved data
    cJSON* ents = cJSON_GetObjectItem(root, "entities");
    if (ents && cJSON_IsArray(ents) && cJSON_GetArraySize(ents) > 0) {
        // Close any existing HIL serial connections before clearing
        for (auto& e : entities) {
            if (e.serial) { e.serial->close(); e.serial.reset(); }
        }
        entities.clear();

        cJSON* ej;
        cJSON_ArrayForEach(ej, ents) {
            // Determine type
            EntityType type = EntityType::SIL;
            if ((val = cJSON_GetObjectItem(ej, "type")) && val->valueint == (int)EntityType::HIL)
                type = EntityType::HIL;

            // Get name
            const char* name = "Entity";
            if ((val = cJSON_GetObjectItem(ej, "name"))) name = val->valuestring;

            Entity& e = addEntity(name, type);
            loadEntityFromJSON(e, ej);
        }
    }

    cJSON_Delete(root);
    log(-1, "system", "Settings loaded from %s (%d entities)", SETTINGS_FILE, (int)entities.size());
}

Entity& App::addEntity(const char* name, EntityType type) {
    Entity e = {};
    e.id = next_entity_id++;
    snprintf(e.name, sizeof(e.name), "%s", name);
    e.type = type;
    e.enabled = true;
    e.config.initDefaults();
    memset(&e.state, 0, sizeof(e.state));
    memset(&e.transport, 0, sizeof(e.transport));
    int ci = e.id % g_num_colors;
    memcpy(e.color, g_entity_colors[ci], sizeof(e.color));
    e.show_settings = false;
    e.show_platform = false;
    e.show_dynamics = false;
    e.viz_cam = {0.8f, 0.45f, 0.0f}; // azimuth ~45°, elevation ~25°, auto distance
    e.rate_ik_hz = 0.0f;
    e.rate_tx_hz = 0.0f;
    e.rate_tel_hz = 0.0f;
    // HIL fields
    e.serial = nullptr;
    e.hil_tx_hz = 60;
    e.hil_last_tx_time = 0.0;
    e.hil_tel_seq = 0;
    memset(e.hil_port, 0, sizeof(e.hil_port));
    e.hil_auto_connect = true;
    e.hil_last_reconnect = 0.0;
    // Init history ring buffers
    memset(e.history_angles, 0, sizeof(e.history_angles));
    memset(e.history_input, 0, sizeof(e.history_input));
    memset(e.history_time, 0, sizeof(e.history_time));
    e.history_head = 0;
    e.history_count = 0;
    // Init placeholder spectrum
    memset(e.spectrum, 0, sizeof(e.spectrum));
    e.spectrum_freq_max = 30.0f;  // placeholder max freq
    entities.push_back(e);
    log(e.id, "system", "Entity '%s' created (type: %s)", name, type == EntityType::SIL ? "SIL" : "HIL");
    return entities.back();
}

void App::removeEntity(int id) {
    for (auto it = entities.begin(); it != entities.end(); ++it) {
        if (it->id == id) {
            log(id, "system", "Entity '%s' removed", it->name);
            entities.erase(it);
            return;
        }
    }
}

Entity* App::findEntity(int id) {
    for (auto& e : entities) {
        if (e.id == id) return &e;
    }
    return nullptr;
}

void App::log(int entity_id, const char* source, const char* fmt, ...) {
    LogEntry entry = {};
    entry.timestamp = frame_time;
    entry.entity_id = entity_id;
    snprintf(entry.source, sizeof(entry.source), "%s", source);

    va_list args;
    va_start(args, fmt);
    vsnprintf(entry.message, sizeof(entry.message), fmt, args);
    va_end(args);

    console_log.push_back(entry);
    if ((int)console_log.size() > console_max) {
        console_log.erase(console_log.begin(),
                          console_log.begin() + ((int)console_log.size() - console_max));
    }

    // Also print to stdout
    printf("[E%d/%s] %s\n", entity_id, source, entry.message);
}

// ── Recording / Playback ────────────────────────────────────────────

void App::startRecording() {
    recording.samples.clear();
    recording.start_time = frame_time;
    recording.mode = RecordMode::Recording;
    const char* src = "manual";
    if (input_source == InputSource::SimToolsUDP) src = "udp";
    else if (input_source == InputSource::CapturePlayback) src = "capture";
    else if (input_source == InputSource::TestSignal) src = "test_signal";
    log(-1, "record", "Recording started (source: %s, rate: %d Hz)", src, record_rate_hz);
}

void App::stopRecording() {
    recording.mode = RecordMode::Idle;
    // Compute data range for diagnostic
    float rmin = 1e9f, rmax = -1e9f;
    for (auto& s : recording.samples) {
        for (int i = 0; i < 6; i++) {
            if (s.input[i] < rmin) rmin = s.input[i];
            if (s.input[i] > rmax) rmax = s.input[i];
        }
    }
    bool flat = !recording.samples.empty() && (fabsf(rmax - rmin) < 0.01f);
    log(-1, "record", "Recording stopped — %d samples (%.1fs) range: %.1f..%.1f%s",
        (int)recording.samples.size(), recording.duration(), rmin, rmax,
        flat ? " [FLAT — input was static during recording]" : "");
}


// ── Capture Library ─────────────────────────────────────────────────

void App::saveRecordingToLibrary(const char* name) {
    if (recording.samples.empty()) {
        log(-1, "capture", "Nothing to save — no samples");
        return;
    }
    SavedRecording sr;
    snprintf(sr.name, sizeof(sr.name), "%s", name);
    sr.sample_rate_hz = (double)record_rate_hz;
    sr.created_time = (double)time(nullptr);
    const char* src = "manual";
    if (input_source == InputSource::SimToolsUDP) src = "udp";
    else if (input_source == InputSource::CapturePlayback) src = "capture";
    snprintf(sr.source, sizeof(sr.source), "%s", src);
    sr.samples = recording.samples;
    saved_recordings.push_back(std::move(sr));
    log(-1, "capture", "Saved \"%s\" (%d samples, %.1fs, %d Hz)",
        name, (int)recording.samples.size(), recording.duration(), record_rate_hz);
    saveRecordingsToDisk();
}

void App::deleteRecordingFromLibrary(int idx) {
    if (idx < 0 || idx >= (int)saved_recordings.size()) return;
    if (capture_playing && capture_playback_idx == idx) stopCapturePlayback();
    log(-1, "capture", "Deleted \"%s\"", saved_recordings[idx].name);
    saved_recordings.erase(saved_recordings.begin() + idx);
    if (capture_playback_idx >= (int)saved_recordings.size())
        capture_playback_idx = (int)saved_recordings.size() - 1;
    saveRecordingsToDisk();
}

void App::startCapturePlayback(int idx) {
    if (idx < 0 || idx >= (int)saved_recordings.size()) return;
    capture_playback_idx = idx;
    capture_playing = true;
    capture_play_cursor = 0;
    capture_start_time = frame_time;
    input_source = InputSource::CapturePlayback;

    // Diagnostic: compute data range to detect flat recordings
    auto& sr = saved_recordings[idx];
    float vmin[6], vmax[6];
    for (int i = 0; i < 6; i++) { vmin[i] = 1e9f; vmax[i] = -1e9f; }
    for (auto& s : sr.samples) {
        for (int i = 0; i < 6; i++) {
            if (s.input[i] < vmin[i]) vmin[i] = s.input[i];
            if (s.input[i] > vmax[i]) vmax[i] = s.input[i];
        }
    }
    bool is_flat = true;
    for (int i = 0; i < 6; i++) {
        if (fabsf(vmax[i] - vmin[i]) > 0.01f) { is_flat = false; break; }
    }

    log(-1, "capture", "Playing \"%s\" (%d samples, %.1fs, %s)",
        sr.name, (int)sr.samples.size(), sr.duration(),
        capture_loop ? "loop" : "once");
    log(-1, "capture", "  Range: %.1f..%.1f  %.1f..%.1f  %.1f..%.1f  %.1f..%.1f  %.1f..%.1f  %.1f..%.1f",
        vmin[0], vmax[0], vmin[1], vmax[1], vmin[2], vmax[2],
        vmin[3], vmax[3], vmin[4], vmax[4], vmin[5], vmax[5]);
    if (is_flat)
        log(-1, "capture", "  WARNING: Recording data is flat (no variation). Record while moving sliders or use Test Signal.");
}

void App::stopCapturePlayback() {
    capture_playing = false;
    log(-1, "capture", "Playback stopped");
}

void App::updateCapturePlayback() {
    if (!capture_playing) return;
    if (capture_playback_idx < 0 || capture_playback_idx >= (int)saved_recordings.size()) {
        stopCapturePlayback();
        return;
    }

    auto& sr = saved_recordings[capture_playback_idx];
    if (sr.samples.empty()) { stopCapturePlayback(); return; }

    double elapsed = (frame_time - capture_start_time) * (double)capture_speed;
    double dur = sr.duration();

    if (elapsed >= dur) {
        if (capture_loop) {
            capture_start_time = frame_time;
            elapsed = 0.0;
            capture_play_cursor = 0;
        } else {
            stopCapturePlayback();
            return;
        }
    }

    // Advance cursor
    auto& samples = sr.samples;
    while (capture_play_cursor < (int)samples.size() - 1 &&
           samples[capture_play_cursor + 1].time <= elapsed) {
        capture_play_cursor++;
    }

    // Interpolate and write to all entities
    int idx = capture_play_cursor;
    float vals[6];
    if (idx < (int)samples.size() - 1) {
        double t0 = samples[idx].time;
        double t1 = samples[idx + 1].time;
        float alpha = (t1 > t0) ? (float)((elapsed - t0) / (t1 - t0)) : 0.0f;
        if (alpha > 1.0f) alpha = 1.0f;
        for (int i = 0; i < 6; i++) {
            vals[i] = samples[idx].input[i] * (1.0f - alpha) +
                      samples[idx + 1].input[i] * alpha;
        }
    } else {
        memcpy(vals, samples[idx].input, sizeof(vals));
    }

    // Feed into all entities
    for (auto& e : entities) {
        memcpy(e.state.input_pct, vals, sizeof(vals));
    }
}

static const char* RECORDINGS_DIR = "recordings";

void App::saveRecordingsToDisk() {
#ifdef _WIN32
    CreateDirectoryA(RECORDINGS_DIR, NULL);
#else
    mkdir(RECORDINGS_DIR, 0755);
#endif

    // Write manifest (JSON array with metadata per recording)
    cJSON* root = cJSON_CreateArray();
    for (int ri = 0; ri < (int)saved_recordings.size(); ri++) {
        auto& sr = saved_recordings[ri];

        // Manifest entry
        cJSON* entry = cJSON_CreateObject();
        cJSON_AddStringToObject(entry, "name", sr.name);
        cJSON_AddNumberToObject(entry, "sample_rate_hz", sr.sample_rate_hz);
        cJSON_AddNumberToObject(entry, "created_time", sr.created_time);
        cJSON_AddStringToObject(entry, "source", sr.source);
        cJSON_AddItemToArray(root, entry);

        // Write versioned binary file: recordings/000.stwr
        char path[256];
        snprintf(path, sizeof(path), "%s/%03d.stwr", RECORDINGS_DIR, ri);
        FILE* f = fopen(path, "wb");
        if (f) {
            RecordingFileHeader hdr = {};
            hdr.magic = RECORDING_MAGIC;
            hdr.version = RECORDING_VERSION;
            hdr.sample_rate_hz = sr.sample_rate_hz;
            hdr.duration_sec = sr.duration();
            hdr.sample_count = (int32_t)sr.samples.size();
            hdr.channels = 6;
            hdr.created_time = sr.created_time;
            snprintf(hdr.name, sizeof(hdr.name), "%s", sr.name);
            snprintf(hdr.source, sizeof(hdr.source), "%s", sr.source);
            memset(hdr.reserved, 0, sizeof(hdr.reserved));

            fwrite(&hdr, sizeof(RecordingFileHeader), 1, f);
            fwrite(sr.samples.data(), sizeof(RecordSample), hdr.sample_count, f);
            fclose(f);
        }
    }

    char* str = cJSON_Print(root);
    if (str) {
        char path[256];
        snprintf(path, sizeof(path), "%s/manifest.json", RECORDINGS_DIR);
        FILE* f = fopen(path, "w");
        if (f) { fputs(str, f); fclose(f); }
        cJSON_free(str);
    }
    cJSON_Delete(root);
}

void App::loadRecordingsFromDisk() {
    char path[256];
    snprintf(path, sizeof(path), "%s/manifest.json", RECORDINGS_DIR);
    FILE* f = fopen(path, "r");
    if (!f) return;

    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (sz <= 0) { fclose(f); return; }

    char* buf = (char*)malloc(sz + 1);
    fread(buf, 1, sz, f);
    buf[sz] = 0;
    fclose(f);

    cJSON* root = cJSON_Parse(buf);
    free(buf);
    if (!root || !cJSON_IsArray(root)) { cJSON_Delete(root); return; }

    saved_recordings.clear();
    int ri = 0;
    cJSON* item;
    cJSON_ArrayForEach(item, root) {
        SavedRecording sr = {};
        sr.sample_rate_hz = 200.0;  // default
        sr.created_time = 0.0;
        snprintf(sr.source, sizeof(sr.source), "unknown");

        // New format: manifest entries are objects with metadata
        if (cJSON_IsObject(item)) {
            cJSON* v;
            if ((v = cJSON_GetObjectItem(item, "name")))
                snprintf(sr.name, sizeof(sr.name), "%s", v->valuestring ? v->valuestring : "Untitled");
            if ((v = cJSON_GetObjectItem(item, "sample_rate_hz")))
                sr.sample_rate_hz = v->valuedouble;
            if ((v = cJSON_GetObjectItem(item, "created_time")))
                sr.created_time = v->valuedouble;
            if ((v = cJSON_GetObjectItem(item, "source")))
                snprintf(sr.source, sizeof(sr.source), "%s", v->valuestring ? v->valuestring : "unknown");
        } else if (cJSON_IsString(item)) {
            // Legacy format: manifest entries are plain name strings
            snprintf(sr.name, sizeof(sr.name), "%s", item->valuestring ? item->valuestring : "Untitled");
        }

        // Try versioned .stwr file first, fall back to legacy .bin
        bool loaded = false;
        char binpath[256];
        snprintf(binpath, sizeof(binpath), "%s/%03d.stwr", RECORDINGS_DIR, ri);
        FILE* bf = fopen(binpath, "rb");
        if (bf) {
            RecordingFileHeader hdr = {};
            if (fread(&hdr, sizeof(RecordingFileHeader), 1, bf) == 1 &&
                hdr.magic == RECORDING_MAGIC && hdr.version >= 1 &&
                hdr.sample_count > 0 && hdr.sample_count < 10000000) {
                sr.sample_rate_hz = hdr.sample_rate_hz;
                sr.created_time = hdr.created_time;
                snprintf(sr.name, sizeof(sr.name), "%s", hdr.name);
                snprintf(sr.source, sizeof(sr.source), "%s", hdr.source);
                sr.samples.resize(hdr.sample_count);
                fread(sr.samples.data(), sizeof(RecordSample), hdr.sample_count, bf);
                loaded = true;
            }
            fclose(bf);
        }

        if (!loaded) {
            // Legacy .bin fallback (v0 format: int count + raw samples)
            snprintf(binpath, sizeof(binpath), "%s/%03d.bin", RECORDINGS_DIR, ri);
            bf = fopen(binpath, "rb");
            if (bf) {
                int count = 0;
                fread(&count, sizeof(int), 1, bf);
                if (count > 0 && count < 10000000) {
                    sr.samples.resize(count);
                    fread(sr.samples.data(), sizeof(RecordSample), count, bf);
                }
                fclose(bf);
            }
        }

        saved_recordings.push_back(std::move(sr));
        ri++;
    }

    cJSON_Delete(root);
    if (!saved_recordings.empty()) {
        log(-1, "capture", "Loaded %d saved recording(s) from disk", (int)saved_recordings.size());
    }
}

// ── Test Signal Presets ──────────────────────────────────────────────

static const char* TS_PRESETS_FILE = "test_signal_presets.json";

void App::saveTestSignalPreset(const char* name) {
    // Check if preset with same name exists — overwrite it
    for (auto& p : test_signal_presets) {
        if (strcmp(p.name, name) == 0) {
            p.config = test_signal;
            p.config.enabled = false;  // don't persist running state
            saveTestSignalPresetsToDisk();
            log(-1, "test_signal", "Preset '%s' updated", name);
            return;
        }
    }
    TestSignalPreset p;
    snprintf(p.name, sizeof(p.name), "%s", name);
    p.config = test_signal;
    p.config.enabled = false;
    test_signal_presets.push_back(p);
    saveTestSignalPresetsToDisk();
    log(-1, "test_signal", "Preset '%s' saved", name);
}

void App::deleteTestSignalPreset(int idx) {
    if (idx >= 0 && idx < (int)test_signal_presets.size()) {
        log(-1, "test_signal", "Preset '%s' deleted", test_signal_presets[idx].name);
        test_signal_presets.erase(test_signal_presets.begin() + idx);
        saveTestSignalPresetsToDisk();
    }
}

void App::saveTestSignalPresetsToDisk() {
    cJSON* root = cJSON_CreateArray();
    for (auto& p : test_signal_presets) {
        cJSON* pj = cJSON_CreateObject();
        cJSON_AddStringToObject(pj, "name", p.name);

        cJSON* freq = cJSON_AddArrayToObject(pj, "frequency");
        cJSON* amp  = cJSON_AddArrayToObject(pj, "amplitude");
        cJSON* phase = cJSON_AddArrayToObject(pj, "phase_offset");
        cJSON* en   = cJSON_AddArrayToObject(pj, "axis_enabled");
        for (int i = 0; i < 6; i++) {
            cJSON_AddItemToArray(freq, cJSON_CreateNumber(p.config.frequency[i]));
            cJSON_AddItemToArray(amp, cJSON_CreateNumber(p.config.amplitude[i]));
            cJSON_AddItemToArray(phase, cJSON_CreateNumber(p.config.phase_offset[i]));
            cJSON_AddItemToArray(en, cJSON_CreateBool(p.config.axis_enabled[i]));
        }
        cJSON_AddBoolToObject(pj, "ramp_up", p.config.ramp_up);
        cJSON_AddNumberToObject(pj, "ramp_duration", p.config.ramp_duration);

        cJSON_AddItemToArray(root, pj);
    }

    char* str = cJSON_Print(root);
    if (str) {
        FILE* f = fopen(TS_PRESETS_FILE, "w");
        if (f) { fputs(str, f); fclose(f); }
        cJSON_free(str);
    }
    cJSON_Delete(root);
}

void App::loadTestSignalPresetsFromDisk() {
    FILE* f = fopen(TS_PRESETS_FILE, "r");
    if (!f) return;

    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (sz <= 0) { fclose(f); return; }

    char* buf = (char*)malloc(sz + 1);
    fread(buf, 1, sz, f);
    buf[sz] = 0;
    fclose(f);

    cJSON* root = cJSON_Parse(buf);
    free(buf);
    if (!root || !cJSON_IsArray(root)) { cJSON_Delete(root); return; }

    test_signal_presets.clear();
    cJSON* pj;
    cJSON_ArrayForEach(pj, root) {
        TestSignalPreset p;
        memset(&p, 0, sizeof(p));
        p.config = TestSignalConfig();  // defaults

        cJSON* val;
        if ((val = cJSON_GetObjectItem(pj, "name")))
            snprintf(p.name, sizeof(p.name), "%s", val->valuestring);

        cJSON* freq = cJSON_GetObjectItem(pj, "frequency");
        cJSON* amp  = cJSON_GetObjectItem(pj, "amplitude");
        cJSON* phase = cJSON_GetObjectItem(pj, "phase_offset");
        cJSON* en   = cJSON_GetObjectItem(pj, "axis_enabled");
        for (int i = 0; i < 6; i++) {
            if (freq && i < cJSON_GetArraySize(freq))  p.config.frequency[i] = (float)cJSON_GetArrayItem(freq, i)->valuedouble;
            if (amp && i < cJSON_GetArraySize(amp))     p.config.amplitude[i] = (float)cJSON_GetArrayItem(amp, i)->valuedouble;
            if (phase && i < cJSON_GetArraySize(phase)) p.config.phase_offset[i] = (float)cJSON_GetArrayItem(phase, i)->valuedouble;
            if (en && i < cJSON_GetArraySize(en))       p.config.axis_enabled[i] = cJSON_IsTrue(cJSON_GetArrayItem(en, i));
        }
        if ((val = cJSON_GetObjectItem(pj, "ramp_up")))       p.config.ramp_up = cJSON_IsTrue(val);
        if ((val = cJSON_GetObjectItem(pj, "ramp_duration")))  p.config.ramp_duration = (float)val->valuedouble;

        test_signal_presets.push_back(p);
    }

    cJSON_Delete(root);
    if (!test_signal_presets.empty())
        log(-1, "test_signal", "Loaded %d preset(s)", (int)test_signal_presets.size());
}

// ── UDP Listener ────────────────────────────────────────────────────

static double GetTimeSeconds() {
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}

static void UdpListenerThread(App* app) {
    SOCKET sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock == INVALID_SOCKET) {
        app->udp.running = false;
        return;
    }

    // Receive timeout so we can check the running flag
#ifdef _WIN32
    DWORD timeout_ms = 100;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout_ms, sizeof(timeout_ms));
#else
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
#endif

    int reuse = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (const char*)&reuse, sizeof(reuse));

    struct sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons((unsigned short)app->simtools_port);
    addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) == SOCKET_ERROR) {
        closesocket(sock);
        app->udp.running = false;
        return;
    }

    app->udp.sock = (int)sock;
    app->udp.rate_window_start = GetTimeSeconds();
    app->udp.rate_window_count = 0;

    char buf[256];
    while (app->udp.running) {
        struct sockaddr_in from = {};
#ifdef _WIN32
        int from_len = sizeof(from);
#else
        socklen_t from_len = sizeof(from);
#endif
        int n = recvfrom(sock, buf, sizeof(buf) - 1, 0, (struct sockaddr*)&from, &from_len);

        if (n <= 0) continue;  // timeout or error

        double now = GetTimeSeconds();
        app->udp.last_packet_time = now;
        app->udp.packets_received++;

        // Rate tracking (1-second window)
        app->udp.rate_window_count++;
        double elapsed = now - app->udp.rate_window_start;
        if (elapsed >= 1.0) {
            app->udp.rate_hz = (float)(app->udp.rate_window_count / elapsed);
            app->udp.rate_window_start = now;
            app->udp.rate_window_count = 0;
        }

        int bit_depth = app->simtools_bit_depth;
        float values[6] = {0};
        bool parsed = false;

        // Try binary parsing first:
        // 8-bit: 1 byte/axis = 6 bytes, values 0-255, center 128
        // 10/12/14/16-bit: 2 bytes/axis LE = 12 bytes, center at midpoint
        if (bit_depth <= 8 && n >= 6 && n < 20) {
            unsigned char* ub = (unsigned char*)buf;
            for (int i = 0; i < 6; i++) {
                float raw = (float)ub[i];
                values[i] = ((raw - 128.0f) / 128.0f) * 100.0f;
            }
            parsed = true;
        } else if (n >= 12 && n < 20) {
            // 2 bytes per axis, little-endian
            unsigned char* ub = (unsigned char*)buf;
            float max_val = (float)((1 << bit_depth) - 1);
            float center = max_val * 0.5f;
            for (int i = 0; i < 6; i++) {
                unsigned short raw = (unsigned short)(ub[i*2] | (ub[i*2+1] << 8));
                values[i] = ((raw - center) / center) * 100.0f;
                if (values[i] > 100.0f) values[i] = 100.0f;
                if (values[i] < -100.0f) values[i] = -100.0f;
            }
            parsed = true;
        }

        // Fallback: try CSV text parsing ("val,val,val,val,val,val")
        if (!parsed) {
            buf[n] = '\0';
            int count = sscanf(buf, "%f,%f,%f,%f,%f,%f",
                &values[0], &values[1], &values[2], &values[3], &values[4], &values[5]);
            if (count >= 6) {
                parsed = true;
                // If values look like raw integers (>100), normalize based on bit depth
                bool needs_normalize = false;
                for (int i = 0; i < 6; i++) {
                    if (fabsf(values[i]) > 100.5f) { needs_normalize = true; break; }
                }
                if (needs_normalize) {
                    float max_val = (float)((1 << bit_depth) - 1);
                    float center = max_val * 0.5f;
                    for (int i = 0; i < 6; i++) {
                        values[i] = ((values[i] - center) / center) * 100.0f;
                        if (values[i] > 100.0f) values[i] = 100.0f;
                        if (values[i] < -100.0f) values[i] = -100.0f;
                    }
                }
            }
        }

        if (parsed) {
            std::lock_guard<std::mutex> lock(app->input_mutex);
            for (int i = 0; i < 6; i++) {
                app->shared_input[i] = values[i];
            }
        } else {
            app->udp.packets_bad++;
        }
    }

    closesocket(sock);
    app->udp.sock = -1;
}

bool App::startUdpListener() {
    if (udp.running) return true;

    udp.running = true;
    udp.packets_received = 0;
    udp.packets_bad = 0;
    udp.rate_hz = 0.0f;
    udp.rate_window_count = 0;
    udp.rate_window_start = 0.0;

    udp.thread = std::thread(UdpListenerThread, this);

    // Wait briefly to see if bind succeeded
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    if (!udp.running) {
        if (udp.thread.joinable()) udp.thread.join();
        return false;
    }

    simtools_active = true;
    input_source = InputSource::SimToolsUDP;
    log(-1, "udp", "UDP listener started on port %d (%d-bit)", simtools_port, simtools_bit_depth);
    return true;
}

void App::stopUdpListener() {
    if (!udp.running) return;

    udp.running = false;
    if (udp.thread.joinable()) {
        udp.thread.join();
    }

    simtools_active = false;
    simtools_rate = 0.0f;
    input_source = InputSource::Manual;
    memset(shared_input, 0, sizeof(shared_input));
    log(-1, "udp", "UDP listener stopped (rx: %d, bad: %d)",
        udp.packets_received.load(), udp.packets_bad.load());
}

// ── Update ──────────────────────────────────────────────────────────

void App::update() {
    // Handle capture playback — writes directly to entity input_pct
    if (capture_playing) {
        updateCapturePlayback();
    }

    // Handle test signal generator — writes to entity input_pct
    if (input_source == InputSource::TestSignal && test_signal.enabled) {
        double t = frame_time - test_signal_start_time;

        // S-curve ramp envelope: smoothstep (3t^2 - 2t^3) over ramp_duration
        float envelope = 1.0f;
        if (test_signal.ramp_up && test_signal.ramp_duration > 0.0f) {
            float r = (float)(t / (double)test_signal.ramp_duration);
            if (r < 1.0f) { envelope = r * r * (3.0f - 2.0f * r); }
        }

        // Smooth interpolation of parameters toward targets
        if (test_signal.smooth_changes) {
            float dt_f = (fps > 0.0) ? (float)(1.0 / fps) : (1.0f / 60.0f);
            float alpha = 1.0f - expf(-test_signal.smooth_rate * dt_f);
            for (int i = 0; i < 6; i++) {
                test_signal.active_freq[i]  += (test_signal.frequency[i]     - test_signal.active_freq[i])  * alpha;
                test_signal.active_amp[i]   += (test_signal.amplitude[i]     - test_signal.active_amp[i])   * alpha;
                test_signal.active_phase[i] += (test_signal.phase_offset[i]  - test_signal.active_phase[i]) * alpha;
            }
        } else {
            for (int i = 0; i < 6; i++) {
                test_signal.active_freq[i]  = test_signal.frequency[i];
                test_signal.active_amp[i]   = test_signal.amplitude[i];
                test_signal.active_phase[i] = test_signal.phase_offset[i];
            }
        }

        float sig[6] = {};
        for (int i = 0; i < 6; i++) {
            if (!test_signal.axis_enabled[i]) continue;
            float freq = test_signal.active_freq[i];
            float amp  = test_signal.active_amp[i];
            float phase = test_signal.active_phase[i] * (float)(M_PI / 180.0);
            double angle = 2.0 * M_PI * freq * t + phase;
            float v = 0.0f;
            switch (test_signal.waveform) {
                case WaveformType::Sine:
                    v = (float)sin(angle);
                    break;
                case WaveformType::Square:
                    v = (float)(fmod(angle, 2.0 * M_PI) < M_PI ? 1.0 : -1.0);
                    break;
                case WaveformType::Triangle:
                    v = (float)(2.0 / M_PI * asin(sin(angle)));
                    break;
                case WaveformType::Sawtooth:
                    v = (float)(2.0 * (angle / (2.0 * M_PI) - floor(0.5 + angle / (2.0 * M_PI))));
                    break;
            }
            sig[i] = v * amp * envelope;
        }
        // Write to all entities and shared_input
        for (auto& e : entities) {
            memcpy(e.state.input_pct, sig, sizeof(sig));
        }
        {
            std::lock_guard<std::mutex> lock(input_mutex);
            memcpy(shared_input, sig, sizeof(sig));
        }
    }

    // ── Input bus sync ────────────────────────────────────────────────
    // Ensure shared_input and entity.input_pct are always consistent.
    // UDP writes to shared_input → push to entities.
    // Manual/Capture/TestSignal write to entity.input_pct → push to shared_input.
    if (input_source == InputSource::SimToolsUDP && simtools_active) {
        // UDP → entities
        float snap[6];
        {
            std::lock_guard<std::mutex> lock(input_mutex);
            memcpy(snap, shared_input, sizeof(snap));
        }
        for (auto& e : entities) {
            memcpy(e.state.input_pct, snap, sizeof(snap));
        }
    } else {
        // Manual / Capture / TestSignal → shared_input
        if (!entities.empty()) {
            std::lock_guard<std::mutex> lock(input_mutex);
            memcpy(shared_input, entities[0].state.input_pct, sizeof(shared_input));
        }
    }

    // Record current input if recording (configurable sample rate)
    // Source-agnostic: always reads from shared_input which is kept in sync
    // with the active input source by the bus sync above.
    if (recording.mode == RecordMode::Recording) {
        double RECORD_INTERVAL = 1.0 / (double)record_rate_hz;
        double elapsed = frame_time - recording.start_time;
        double last_sample_time = recording.samples.empty() ? -RECORD_INTERVAL : recording.samples.back().time;
        if (elapsed - last_sample_time >= RECORD_INTERVAL) {
            RecordSample s;
            s.time = elapsed;
            {
                std::lock_guard<std::mutex> lock(input_mutex);
                memcpy(s.input, shared_input, sizeof(s.input));
            }
            recording.samples.push_back(s);
        }
    }

    // Sync UDP rate for UI display
    if (simtools_active) {
        simtools_rate = udp.rate_hz.load();
    }

    // Grab shared input (thread-safe)
    float current_input[6];
    {
        std::lock_guard<std::mutex> lock(input_mutex);
        memcpy(current_input, shared_input, sizeof(current_input));
    }

    // Console input logging (user-controlled rate via console_log_rate)
    if (console_log_rate > 0) {
        static const double rate_intervals[] = {0.0, 1.0, 0.1, 1.0/30.0, 1.0/60.0, 0.0};
        double interval = rate_intervals[console_log_rate];
        bool should_log = (interval <= 0.0) || (frame_time - last_input_log_time >= interval);
        if (should_log) {
            last_input_log_time = frame_time;
            const char* src_str = "manual";
            if (input_source == InputSource::SimToolsUDP) src_str = "udp";
            else if (input_source == InputSource::CapturePlayback) src_str = "capture";
            else if (input_source == InputSource::TestSignal) src_str = "test";
            log(-1, src_str, "IN: %+.0f %+.0f %+.0f %+.0f %+.0f %+.0f",
                current_input[0], current_input[1], current_input[2],
                current_input[3], current_input[4], current_input[5]);
        }
    }

    // Push to global input history ring buffer (at record_rate_hz)
    {
        double hist_interval = 1.0 / (double)record_rate_hz;
        if (frame_time - input_history_last_push >= hist_interval) {
            input_history_last_push = frame_time;
            int idx = input_history_head;

            // shared_input is always current (bus sync above keeps it in sync)
            float vals[6];
            {
                std::lock_guard<std::mutex> lock(input_mutex);
                memcpy(vals, shared_input, sizeof(vals));
            }

            input_history_time[idx] = (float)frame_time;
            for (int i = 0; i < 6; i++)
                input_history[i][idx] = vals[i];
            input_history_head = (input_history_head + 1) % INPUT_HISTORY_LEN;
            if (input_history_count < INPUT_HISTORY_LEN) input_history_count++;
        }
    }

    // Compute input spectrum (simple DFT, updated every ~100ms)
    {
        static double last_dft_time = 0.0;
        if (frame_time - last_dft_time >= 0.1 && input_history_count >= 64) {
            last_dft_time = frame_time;
            input_spectrum_freq_max = (float)record_rate_hz * 0.5f;  // Nyquist

            int N = input_history_count;
            if (N > INPUT_HISTORY_LEN) N = INPUT_HISTORY_LEN;
            int start = (input_history_head - N + INPUT_HISTORY_LEN) % INPUT_HISTORY_LEN;

            for (int axis = 0; axis < 6; axis++) {
                for (int k = 0; k < SPECTRUM_BINS; k++) {
                    double freq = (double)k / (double)SPECTRUM_BINS * input_spectrum_freq_max;
                    double re = 0.0, im = 0.0;
                    for (int n = 0; n < N; n++) {
                        int si = (start + n) % INPUT_HISTORY_LEN;
                        double angle = -2.0 * M_PI * freq * (double)n / (double)record_rate_hz;
                        re += input_history[axis][si] * cos(angle);
                        im += input_history[axis][si] * sin(angle);
                    }
                    input_spectrum[axis][k] = (float)(sqrt(re * re + im * im) / (double)N);
                }
            }
        }
    }

    // Run pipeline for each enabled entity
    for (auto& e : entities) {
        if (!e.enabled) continue;

        // Common: read input and apply intensity/gain
        float pct[6];
        memcpy(pct, e.state.input_pct, sizeof(pct));

        float scaled_pct[6];
        for (int i = 0; i < 6; i++) {
            scaled_pct[i] = pct[i] * (e.config.intensity / 100.0f) * (e.config.axis_gain[i] / 100.0f);
        }

        if (e.type == EntityType::SIL) {
            // ── SIL Pipeline: local IK computation ──
            float physical[6];
            for (int i = 0; i < 6; i++) {
                physical[i] = (scaled_pct[i] / 100.0f) * e.config.axis_scales.scale[i];
                if (e.config.axis_scales.is_angle[i]) {
                    physical[i] *= (float)(M_PI / 180.0);
                }
            }

            float angles[6];
            calcAllActuatorAngles(physical, &e.config.platform, angles);
            int valid_mask = validatePositionV2(physical, &e.config.platform);

            memcpy(e.state.input_physical, physical, sizeof(physical));
            memcpy(e.state.output_angles, angles, sizeof(angles));

            float max_util = 0.0f;
            float range = e.config.platform.servo_max_rad - e.config.platform.servo_min_rad;
            for (int i = 0; i < 6; i++) {
                e.state.output_angles_deg[i] = angles[i] * (180.0f / (float)M_PI);
                e.state.output_steps[i] = e.state.output_angles_deg[i] * e.config.platform.steps_per_degree;
                float util = fabsf(angles[i]) / (range * 0.5f) * 100.0f;
                if (util > 100.0f) util = 100.0f;
                e.state.servo_util[i] = util;
                if (util > max_util) max_util = util;
            }
            e.state.max_util = max_util;
            e.state.valid_mask = valid_mask;
            e.state.ik_seq++;
            e.rate_ik_hz = (float)fps;

        } else if (e.type == EntityType::HIL) {
            // ── HIL auto-reconnect ──
            // Detect lost connection
            if (e.serial && !e.serial->isOpen()) {
                log(e.id, "hil", "Connection lost on %s", e.hil_port);
                e.serial->close();
                e.serial.reset();
                e.transport.usb_connected = false;
            }
            // Auto-reconnect: retry every 3 seconds if auto_connect is enabled
            if (!e.serial && e.hil_auto_connect
                && frame_time - e.hil_last_reconnect >= 3.0) {
                e.hil_last_reconnect = frame_time;

                // If no port configured, try to auto-detect first available
                if (e.hil_port[0] == '\0') {
                    auto ports = SerialPort::enumerate();
                    if (!ports.empty()) {
                        snprintf(e.hil_port, sizeof(e.hil_port), "%s", ports[0].port.c_str());
                        log(e.id, "hil", "Auto-detected port: %s", e.hil_port);
                    }
                }

                if (e.hil_port[0] != '\0') {
                    auto sp = std::make_shared<SerialPort>();
                    int eid = e.id;
                    sp->setLineCallback([eid](const char* line) {
                        g_app.log(eid, "esp32", "%s", line);
                    });
                    if (sp->open(e.hil_port, 115200)) {
                        e.serial = sp;
                        e.transport.usb_connected = true;
                        snprintf(e.transport.usb_port, sizeof(e.transport.usb_port), "%s", e.hil_port);
                        e.hil_tel_seq = 0;
                        log(e.id, "hil", "Auto-connected to %s", e.hil_port);
                        // Sync bit depth
                        char cmd[32];
                        snprintf(cmd, sizeof(cmd), "BITS:%d", e.config.bit_depth);
                        sp->sendCommand(cmd);
                    } else {
                        log(e.id, "hil", "Auto-connect failed: %s (retrying...)", e.hil_port);
                    }
                }
            }

            // ── HIL Pipeline: send to ESP32, use telemetry for angles ──
            if (e.serial && e.serial->isOpen()) {
                // Send binary motion packet at configured rate
                double tx_interval = 1.0 / (double)e.hil_tx_hz;
                if (frame_time - e.hil_last_tx_time >= tx_interval) {
                    e.hil_last_tx_time = frame_time;

                    // Convert scaled_pct (-100..+100) to raw uint16 for ESP32
                    // ESP32 mapRawToPosition: pos = (raw - home) * (scale / home)
                    // So: raw = scaled_pct/100 * home + home
                    float max_raw = (float)((1 << e.config.bit_depth) - 2);
                    float home = max_raw * 0.5f;
                    uint16_t raw[6];
                    for (int i = 0; i < 6; i++) {
                        float r = (scaled_pct[i] / 100.0f) * home + home;
                        if (r < 0.0f) r = 0.0f;
                        if (r > max_raw) r = max_raw;
                        raw[i] = (uint16_t)(r + 0.5f);
                    }
                    e.serial->sendMotionPacket(raw);
                }

                // Compute input_physical so viz platform top matches what we sent
                for (int i = 0; i < 6; i++) {
                    float p = (scaled_pct[i] / 100.0f) * e.config.axis_scales.scale[i];
                    if (e.config.axis_scales.is_angle[i])
                        p *= (float)(M_PI / 180.0);
                    e.state.input_physical[i] = p;
                }

                // Read telemetry from ESP32
                ESP32Telemetry tel = e.serial->getLatestTelemetry();
                if (tel.seq != e.hil_tel_seq) {
                    e.hil_tel_seq = tel.seq;

                    // Use ESP32's computed angles for visualization
                    memcpy(e.state.output_angles, tel.angles, sizeof(tel.angles));

                    float max_util = 0.0f;
                    float range = e.config.platform.servo_max_rad - e.config.platform.servo_min_rad;
                    for (int i = 0; i < 6; i++) {
                        e.state.output_angles_deg[i] = tel.angles[i] * (180.0f / (float)M_PI);
                        e.state.output_steps[i] = e.state.output_angles_deg[i] * e.config.platform.steps_per_degree;
                        float util = fabsf(tel.angles[i]) / (range * 0.5f) * 100.0f;
                        if (util > 100.0f) util = 100.0f;
                        e.state.servo_util[i] = util;
                        if (util > max_util) max_util = util;
                    }
                    e.state.max_util = max_util;
                    e.state.ik_seq++;
                }

                e.rate_tx_hz = (float)e.hil_tx_hz;
                e.rate_tel_hz = e.serial->telemetryRate();
            } else {
                // No ESP32 connected — hold last valid state, just clear rates
                e.rate_ik_hz = 0.0f;
                e.rate_tx_hz = 0.0f;
                e.rate_tel_hz = 0.0f;
            }
        }

        // Record history
        e.pushHistory(frame_time);

        // Per-entity DFT (for spectrogram, every ~100ms)
        {
            static double last_entity_dft[8] = {};
            int ei = (int)(&e - &entities[0]);
            if (ei < 8 && e.history_count >= 64 && frame_time - last_entity_dft[ei] >= 0.1) {
                last_entity_dft[ei] = frame_time;
                // Estimate sample rate from history timestamps
                int N = e.history_count;
                if (N > HISTORY_LEN) N = HISTORY_LEN;
                int newest = (e.history_head - 1 + HISTORY_LEN) % HISTORY_LEN;
                int oldest = (e.history_head - N + HISTORY_LEN) % HISTORY_LEN;
                double dt = (double)(e.history_time[newest] - e.history_time[oldest]);
                float sr = (dt > 0.01) ? (float)((N - 1) / dt) : 60.0f;
                e.spectrum_freq_max = sr * 0.5f;

                int start = oldest;
                for (int axis = 0; axis < 6; axis++) {
                    for (int k = 0; k < SPECTRUM_BINS; k++) {
                        double freq = (double)k / (double)SPECTRUM_BINS * (double)e.spectrum_freq_max;
                        double re = 0.0, im = 0.0;
                        for (int n = 0; n < N; n++) {
                            int si = (start + n) % HISTORY_LEN;
                            double angle = -2.0 * M_PI * freq * (double)n / (double)sr;
                            re += e.history_input[axis][si] * cos(angle);
                            im += e.history_input[axis][si] * sin(angle);
                        }
                        e.spectrum[axis][k] = (float)(sqrt(re * re + im * im) / (double)N);
                    }
                }
            }
        }
    }

    // Auto-save: compare settings against last-saved snapshot each frame
    {
        static int    s_port = -1, s_bits = -1, s_lograte = -1, s_max = -1, s_rec_rate = -1;
        static int    s_input_source = -1;
        static bool   s_autoscroll = false;
        static float  s_intensity[8] = {};  // up to 8 entities
        static float  s_gain[8][6] = {};
        static int    s_bit_depth[8] = {};
        static int    s_entity_count = 0;
        static char   s_hil_port[8][32] = {};
        static int    s_hil_tx_hz[8] = {};
        static bool   s_hil_auto[8] = {};
        static bool   s_inited = false;

        auto snapshot_matches = [&]() -> bool {
            if (s_port != simtools_port) return false;
            if (s_bits != simtools_bit_depth) return false;
            if (s_lograte != console_log_rate) return false;
            if (s_max != console_max) return false;
            if (s_autoscroll != console_auto_scroll) return false;
            if (s_rec_rate != record_rate_hz) return false;
            if (s_input_source != (int)input_source) return false;
            if (s_entity_count != (int)entities.size()) return false;
            for (int ei = 0; ei < (int)entities.size() && ei < 8; ei++) {
                if (s_intensity[ei] != entities[ei].config.intensity) return false;
                if (s_bit_depth[ei] != entities[ei].config.bit_depth) return false;
                for (int a = 0; a < 6; a++) {
                    if (s_gain[ei][a] != entities[ei].config.axis_gain[a]) return false;
                }
                if (entities[ei].type == EntityType::HIL) {
                    if (strcmp(s_hil_port[ei], entities[ei].hil_port) != 0) return false;
                    if (s_hil_tx_hz[ei] != entities[ei].hil_tx_hz) return false;
                    if (s_hil_auto[ei] != entities[ei].hil_auto_connect) return false;
                }
            }
            return true;
        };

        auto take_snapshot = [&]() {
            s_port = simtools_port;
            s_bits = simtools_bit_depth;
            s_lograte = console_log_rate;
            s_max = console_max;
            s_autoscroll = console_auto_scroll;
            s_rec_rate = record_rate_hz;
            s_input_source = (int)input_source;
            s_entity_count = (int)entities.size();
            for (int ei = 0; ei < (int)entities.size() && ei < 8; ei++) {
                s_intensity[ei] = entities[ei].config.intensity;
                s_bit_depth[ei] = entities[ei].config.bit_depth;
                for (int a = 0; a < 6; a++)
                    s_gain[ei][a] = entities[ei].config.axis_gain[a];
                if (entities[ei].type == EntityType::HIL) {
                    snprintf(s_hil_port[ei], sizeof(s_hil_port[ei]), "%s", entities[ei].hil_port);
                    s_hil_tx_hz[ei] = entities[ei].hil_tx_hz;
                    s_hil_auto[ei] = entities[ei].hil_auto_connect;
                }
            }
        };

        if (!s_inited) { take_snapshot(); s_inited = true; }
        else if (!snapshot_matches()) {
            take_snapshot();
            saveSettings();
        }
    }
}

void Entity::pushHistory(double t) {
    int idx = history_head;
    history_time[idx] = (float)t;
    for (int i = 0; i < 6; i++) {
        history_angles[i][idx] = state.output_angles_deg[i];
        history_input[i][idx] = state.input_physical[i];
    }
    history_head = (history_head + 1) % HISTORY_LEN;
    if (history_count < HISTORY_LEN) history_count++;
}
