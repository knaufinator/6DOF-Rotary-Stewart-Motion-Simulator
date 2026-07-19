#include "app.h"
#include "serial_port.h"
#include "dev_log.h"
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
#include <sys/stat.h>
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

static inline int clampBitDepth(int bits) {
    if (bits < 8) return 8;
    if (bits > 18) return 18;
    return bits;
}

static inline float txMaxRawForBitDepth(int bits) {
    bits = clampBitDepth(bits);
    return (float)((1u << bits) - 2u);
}

// ── PipelineConfig ──────────────────────────────────────────────────

void PipelineConfig::initDefaults() {
    initDefaultStewartConfig(&geometry);
    computeStepsPerDegree(&geometry);
    rebuildPlatform();
    computeAxisScalesFromGeometry(&axis_scales, &geometry, 0.9f);
    memset(&mca, 0, sizeof(mca));
    mca.enabled = 0;  // MCA off by default for manual sliders
    initInputFilter(&input_filter, 60.0f);
    occupant[0] = 0.0f;
    occupant[1] = 0.0f;
    occupant[2] = 800.0f;
    bit_depth = 12;
    platform_type = PlatformType::Stepper;  // default to stepper (main platform)
    servo.initDefaults();
    // Phase D: default intensity and per-axis gains
    intensity = 100.0f;
    for (int i = 0; i < 6; i++) axis_gain[i] = 100.0f;
    for (int i = 0; i < 6; i++) axis_invert[i] = false;
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
    , input_source(InputSource::Plugin)
    , console_max(500)
    , console_auto_scroll(true)
    , console_paused(false)
    , console_filter(-1)
    , running(true)
    , motion_started(false)
    , start_ramp_active(false)
    , start_ramp_begin(0.0)
    , frame_time(0.0)
    , fps(0.0)
    , frame_count(0)
    , last_input_log_time(0.0)
    , console_log_rate(2)  // default: 10 Hz
    , capture_playback_idx(-1)
    , capture_playing(false)
    , capture_play_cursor(0)
    , capture_start_time(0.0)
    , capture_loop(false)
    , capture_speed(1.0f)
    , capture_ramp_phase(CaptureRampPhase::Playing)
    , capture_ramp_start(0.0)
    , capture_stop_requested(false)
    , record_rate_hz(200)
    , input_history_head(0)
    , input_history_count(0)
    , input_history_last_push(0.0)
    , input_spectrum_freq_max(100.0f)
    , settings_dirty(false)
    , source_switch_active(false)
    , source_switch_target(InputSource::Plugin)
    , source_switch_start(0.0)
    , active_plugin_idx(-1)
{
    memset(shared_input, 0, sizeof(shared_input));
    memset(capture_last_vals, 0, sizeof(capture_last_vals));
    memset(input_history, 0, sizeof(input_history));
    memset(input_history_time, 0, sizeof(input_history_time));
    memset(input_spectrum, 0, sizeof(input_spectrum));

    // Start background HIL TX thread
    hil_tx_stop.store(false);
    hil_tx_thread = std::thread(&App::hilTxLoop, this);

#ifdef _WIN32
    WSADATA wsa;
    WSAStartup(MAKEWORD(2, 2), &wsa);
#endif

    // Scan for plugins in the plugins/ directory next to the executable
    plugin_mgr.scanDirectory("plugins");
}

App::~App() {
    plugin_mgr.deactivateActive();
    plugin_mgr.unloadAll();
    hil_tx_stop.store(true);
    if (hil_tx_thread.joinable()) hil_tx_thread.join();
#ifdef _WIN32
    WSACleanup();
#endif
}

void App::hilTxLoop() {
    using clock = std::chrono::steady_clock;
    double last_send[8] = {};
    auto now_sec = []() -> double {
        return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
    };

    while (!hil_tx_stop.load()) {
        int ei = 0;
        for (auto& e : entities) {
            if (ei >= 8) break;
            if (e.type != EntityType::HIL || !e.serial || !e.serial->isOpen() || !e.hil_handshake_ok) { ei++; continue; }

            double interval = 1.0 / (double)e.hil_tx_hz;
            double now = now_sec();
            if (now - last_send[ei] >= interval) {
                last_send[ei] = now;
                if (e.hil_raw_mode && e.hil_cap_raw) {
                    // RAW HIL: stream pre-cueing input_pct (app axis order, no swap).
                    // The ESP runs the single cue engine and swaps axes after cueing.
                    float f[6];
                    memcpy(f, e.hil_tx_raw_f, sizeof(f));
                    e.serial->sendCobsDataRaw(f);
                } else {
                    // BAKED: hil_tx_raw already carries MCA dynamics, tilt
                    // coordination, intensity, axis gain, and the surge<->sway swap.
                    uint32_t raw[6];
                    memcpy(raw, e.hil_tx_raw, sizeof(raw));
                    e.serial->sendCobsData(raw, e.config.bit_depth);
                }
            }
            ei++;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
}

static const char* SETTINGS_FILE = "stewart_settings.json";

// Forward declarations (defined in MCA Dynamics Presets section below)
static cJSON* mcaConfigToJSON(const MotionCueingConfig& mca, float intensity, const float axis_gain[6]);
static void mcaConfigFromJSON(cJSON* obj, MotionCueingConfig& mca, float& intensity, float axis_gain[6]);

static void saveEntityToJSON(cJSON* ej, const Entity& e) {
    cJSON_AddStringToObject(ej, "name", e.name);
    cJSON_AddNumberToObject(ej, "type", (int)e.type);
    cJSON_AddBoolToObject(ej, "enabled", e.enabled);
    cJSON_AddNumberToObject(ej, "intensity", e.config.intensity);
    cJSON_AddNumberToObject(ej, "bit_depth", e.config.bit_depth);

    // Color
    cJSON* col = cJSON_AddArrayToObject(ej, "color");
    for (int i = 0; i < 4; i++) cJSON_AddItemToArray(col, cJSON_CreateNumber(e.color[i]));

    // Per-axis gain + inversion
    cJSON* gains = cJSON_AddArrayToObject(ej, "axis_gain");
    for (int i = 0; i < 6; i++) cJSON_AddItemToArray(gains, cJSON_CreateNumber(e.config.axis_gain[i]));
    cJSON* inverts = cJSON_AddArrayToObject(ej, "axis_invert");
    for (int i = 0; i < 6; i++) cJSON_AddItemToArray(inverts, cJSON_CreateBool(e.config.axis_invert[i]));

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

    // Full MCA dynamics config (replaces old mca_preset-only save)
    cJSON* mca_obj = mcaConfigToJSON(e.config.mca, e.config.intensity, e.config.axis_gain);
    cJSON_AddItemToObject(ej, "mca_config", mca_obj);

    // Input filter config
    cJSON* iflt_obj = cJSON_AddObjectToObject(ej, "input_filter");
    cJSON_AddBoolToObject(iflt_obj, "enabled", e.config.input_filter.enabled != 0);
    cJSON* iflt_axes = cJSON_AddArrayToObject(iflt_obj, "axes");
    for (int i = 0; i < 6; i++) {
        const InputAxisFilter& ax = e.config.input_filter.axes[i];
        cJSON* a = cJSON_CreateObject();
        cJSON_AddBoolToObject(a, "lp_on", ax.lp_enabled != 0);
        cJSON_AddNumberToObject(a, "lp_fc", ax.lp.fc);
        cJSON_AddNumberToObject(a, "lp_Q", ax.lp.Q);
        cJSON_AddBoolToObject(a, "notch_on", ax.notch_enabled != 0);
        cJSON_AddNumberToObject(a, "notch_fc", ax.notch.fc);
        cJSON_AddNumberToObject(a, "notch_Q", ax.notch.Q);
        cJSON_AddItemToArray(iflt_axes, a);
    }

    // Occupant offset
    cJSON* occ = cJSON_AddArrayToObject(ej, "occupant");
    for (int i = 0; i < 3; i++) cJSON_AddItemToArray(occ, cJSON_CreateNumber(e.config.occupant[i]));

    // Platform type + servo config
    cJSON_AddNumberToObject(ej, "platform_type", (int)e.config.platform_type);
    if (e.config.isServo()) {
        cJSON* sv = cJSON_AddObjectToObject(ej, "servo");
        cJSON* centers = cJSON_AddArrayToObject(sv, "center_us");
        for (int i = 0; i < 6; i++) cJSON_AddItemToArray(centers, cJSON_CreateNumber(e.config.servo.center_us[i]));
        cJSON_AddNumberToObject(sv, "pulse_per_rad", e.config.servo.pulse_per_rad);
        cJSON_AddNumberToObject(sv, "min_pulse_us", e.config.servo.min_pulse_us);
        cJSON_AddNumberToObject(sv, "max_pulse_us", e.config.servo.max_pulse_us);
        cJSON_AddNumberToObject(sv, "pwm_freq_hz", e.config.servo.pwm_freq_hz);
        cJSON* inv = cJSON_AddArrayToObject(sv, "inverted");
        for (int i = 0; i < 6; i++) cJSON_AddItemToArray(inv, cJSON_CreateBool(e.config.servo.inverted[i]));
    }

    // HIL-specific
    if (e.type == EntityType::HIL) {
        cJSON* hil = cJSON_AddObjectToObject(ej, "hil");
        cJSON_AddStringToObject(hil, "port", e.hil_port);
        cJSON_AddNumberToObject(hil, "baud", e.hil_baud);
        cJSON_AddNumberToObject(hil, "tx_hz", e.hil_tx_hz);
        cJSON_AddNumberToObject(hil, "tick_rate_us", e.hil_tick_rate_us);
        cJSON_AddBoolToObject(hil, "auto_connect", e.hil_auto_connect);
        // Network bridge transport
        cJSON_AddBoolToObject(hil, "network", e.hil_network);
        cJSON_AddStringToObject(hil, "host", e.hil_host);
        cJSON_AddNumberToObject(hil, "udp_port", e.hil_udp_port);
        cJSON_AddNumberToObject(hil, "tcp_port", e.hil_tcp_port);
        cJSON_AddBoolToObject(hil, "raw_mode", e.hil_raw_mode);
        if (e.hil_fingerprint[0] != '\0')
            cJSON_AddStringToObject(hil, "fingerprint", e.hil_fingerprint);
    }
}

static void loadEntityFromJSON(Entity& e, cJSON* ej) {
    cJSON* val;
    if ((val = cJSON_GetObjectItem(ej, "name")))      snprintf(e.name, sizeof(e.name), "%s", val->valuestring);
    if ((val = cJSON_GetObjectItem(ej, "enabled")))    e.enabled = cJSON_IsTrue(val);
    if ((val = cJSON_GetObjectItem(ej, "intensity")))  e.config.intensity = (float)val->valuedouble;
    if ((val = cJSON_GetObjectItem(ej, "bit_depth")))
        e.config.bit_depth = clampBitDepth(val->valueint);

    // Color
    cJSON* col = cJSON_GetObjectItem(ej, "color");
    if (col && cJSON_IsArray(col)) {
        for (int i = 0; i < 4 && i < cJSON_GetArraySize(col); i++)
            e.color[i] = (float)cJSON_GetArrayItem(col, i)->valuedouble;
    }

    // Per-axis gain + inversion
    cJSON* gains = cJSON_GetObjectItem(ej, "axis_gain");
    if (gains && cJSON_IsArray(gains)) {
        for (int i = 0; i < 6 && i < cJSON_GetArraySize(gains); i++)
            e.config.axis_gain[i] = (float)cJSON_GetArrayItem(gains, i)->valuedouble;
    }
    cJSON* inverts = cJSON_GetObjectItem(ej, "axis_invert");
    if (inverts && cJSON_IsArray(inverts)) {
        for (int i = 0; i < 6 && i < cJSON_GetArraySize(inverts); i++)
            e.config.axis_invert[i] = cJSON_IsTrue(cJSON_GetArrayItem(inverts, i));
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

    // Full MCA dynamics config
    cJSON* mca_obj = cJSON_GetObjectItem(ej, "mca_config");
    if (mca_obj && cJSON_IsObject(mca_obj)) {
        initMotionCueing(&e.config.mca, 60.0f);
        mcaConfigFromJSON(mca_obj, e.config.mca, e.config.intensity, e.config.axis_gain);
        // Recalculate biquad coefficients at default sample rate
        float sr = e.config.mca.sample_rate > 0 ? e.config.mca.sample_rate : 60.0f;
        for (int i = 0; i < 6; i++) {
            if (e.config.mca.channels[i].hp_enabled && e.config.mca.channels[i].hp.fc > 0)
                biquadSetHighpass(&e.config.mca.channels[i].hp, e.config.mca.channels[i].hp.fc, sr, e.config.mca.channels[i].hp.Q);
            if (e.config.mca.channels[i].lp_enabled && e.config.mca.channels[i].lp.fc > 0)
                biquadSetLowpass(&e.config.mca.channels[i].lp, e.config.mca.channels[i].lp.fc, sr, e.config.mca.channels[i].lp.Q);
        }
        if (e.config.mca.tilt.enabled && e.config.mca.tilt.fc > 0) {
            biquadSetLowpass(&e.config.mca.tilt.surge_lp, e.config.mca.tilt.fc, sr, e.config.mca.tilt.Q);
            biquadSetLowpass(&e.config.mca.tilt.sway_lp,  e.config.mca.tilt.fc, sr, e.config.mca.tilt.Q);
        }
        if (e.config.mca.tilt.surge_hp_enabled && e.config.mca.tilt.hp_fc > 0) {
            float hq = e.config.mca.tilt.hp_Q > 0 ? e.config.mca.tilt.hp_Q : 0.707f;
            biquadSetHighpass(&e.config.mca.tilt.surge_hp, e.config.mca.tilt.hp_fc, sr, hq);
        }
        if (e.config.mca.tilt.sway_hp_enabled && e.config.mca.tilt.sway_hp_fc > 0) {
            float hq = e.config.mca.tilt.sway_hp_Q > 0 ? e.config.mca.tilt.sway_hp_Q : 0.707f;
            biquadSetHighpass(&e.config.mca.tilt.sway_hp, e.config.mca.tilt.sway_hp_fc, sr, hq);
        }
    } else if ((val = cJSON_GetObjectItem(ej, "mca_preset"))) {
        // Legacy fallback: old settings with just preset index
        int preset = val->valueint;
        if (preset > 0 && preset < MCA_PRESET_COUNT) {
            initMotionCueing(&e.config.mca, 60.0f);
            setMotionCueingPreset(&e.config.mca, preset);
        }
    }

    // Input filter config
    cJSON* iflt_obj = cJSON_GetObjectItem(ej, "input_filter");
    if (iflt_obj && cJSON_IsObject(iflt_obj)) {
        cJSON* val_if;
        if ((val_if = cJSON_GetObjectItem(iflt_obj, "enabled"))) e.config.input_filter.enabled = cJSON_IsTrue(val_if) ? 1 : 0;
        cJSON* iflt_axes = cJSON_GetObjectItem(iflt_obj, "axes");
        if (iflt_axes && cJSON_IsArray(iflt_axes)) {
            float sr = e.config.input_filter.sample_rate > 0 ? e.config.input_filter.sample_rate : 60.0f;
            for (int i = 0; i < 6 && i < cJSON_GetArraySize(iflt_axes); i++) {
                cJSON* a = cJSON_GetArrayItem(iflt_axes, i);
                InputAxisFilter& ax = e.config.input_filter.axes[i];
                if ((val_if = cJSON_GetObjectItem(a, "lp_on")))    ax.lp_enabled = cJSON_IsTrue(val_if) ? 1 : 0;
                if ((val_if = cJSON_GetObjectItem(a, "lp_fc")))    ax.lp.fc = (float)val_if->valuedouble;
                if ((val_if = cJSON_GetObjectItem(a, "lp_Q")))     ax.lp.Q = (float)val_if->valuedouble;
                if ((val_if = cJSON_GetObjectItem(a, "notch_on"))) ax.notch_enabled = cJSON_IsTrue(val_if) ? 1 : 0;
                if ((val_if = cJSON_GetObjectItem(a, "notch_fc"))) ax.notch.fc = (float)val_if->valuedouble;
                if ((val_if = cJSON_GetObjectItem(a, "notch_Q")))  ax.notch.Q = (float)val_if->valuedouble;
                if (ax.lp_enabled && ax.lp.fc > 0)
                    biquadSetLowpass(&ax.lp, ax.lp.fc, sr, ax.lp.Q > 0 ? ax.lp.Q : 0.707f);
                if (ax.notch_enabled && ax.notch.fc > 0)
                    biquadSetNotch(&ax.notch, ax.notch.fc, sr, ax.notch.Q > 0 ? ax.notch.Q : 5.0f);
            }
        }
    }

    // Occupant offset
    cJSON* occ = cJSON_GetObjectItem(ej, "occupant");
    if (occ && cJSON_IsArray(occ)) {
        for (int i = 0; i < 3 && i < cJSON_GetArraySize(occ); i++)
            e.config.occupant[i] = (float)cJSON_GetArrayItem(occ, i)->valuedouble;
    }

    // Platform type + servo config
    if ((val = cJSON_GetObjectItem(ej, "platform_type")))
        e.config.platform_type = (PlatformType)val->valueint;
    cJSON* sv = cJSON_GetObjectItem(ej, "servo");
    if (sv && cJSON_IsObject(sv)) {
        cJSON* centers = cJSON_GetObjectItem(sv, "center_us");
        if (centers && cJSON_IsArray(centers)) {
            for (int i = 0; i < 6 && i < cJSON_GetArraySize(centers); i++)
                e.config.servo.center_us[i] = cJSON_GetArrayItem(centers, i)->valueint;
        }
        if ((val = cJSON_GetObjectItem(sv, "pulse_per_rad"))) e.config.servo.pulse_per_rad = (float)val->valuedouble;
        if ((val = cJSON_GetObjectItem(sv, "min_pulse_us")))  e.config.servo.min_pulse_us = val->valueint;
        if ((val = cJSON_GetObjectItem(sv, "max_pulse_us")))  e.config.servo.max_pulse_us = val->valueint;
        if ((val = cJSON_GetObjectItem(sv, "pwm_freq_hz")))   e.config.servo.pwm_freq_hz = val->valueint;
        cJSON* inv = cJSON_GetObjectItem(sv, "inverted");
        if (inv && cJSON_IsArray(inv)) {
            for (int i = 0; i < 6 && i < cJSON_GetArraySize(inv); i++)
                e.config.servo.inverted[i] = cJSON_IsTrue(cJSON_GetArrayItem(inv, i));
        }
    }
    cJSON* hil = cJSON_GetObjectItem(ej, "hil");
    if (hil) {
        if ((val = cJSON_GetObjectItem(hil, "port"))) snprintf(e.hil_port, sizeof(e.hil_port), "%s", val->valuestring);
        if ((val = cJSON_GetObjectItem(hil, "baud"))) e.hil_baud = val->valueint;
        if ((val = cJSON_GetObjectItem(hil, "tx_hz"))) e.hil_tx_hz = val->valueint;
        if ((val = cJSON_GetObjectItem(hil, "tick_rate_us"))) {
            int v = val->valueint;
            if (v >= 4 && v <= 100) e.hil_tick_rate_us = v;
        }
        if ((val = cJSON_GetObjectItem(hil, "auto_connect"))) e.hil_auto_connect = cJSON_IsTrue(val);
        if ((val = cJSON_GetObjectItem(hil, "network"))) e.hil_network = cJSON_IsTrue(val);
        if ((val = cJSON_GetObjectItem(hil, "host")) && cJSON_IsString(val)) snprintf(e.hil_host, sizeof(e.hil_host), "%s", val->valuestring);
        if ((val = cJSON_GetObjectItem(hil, "udp_port"))) e.hil_udp_port = val->valueint;
        if ((val = cJSON_GetObjectItem(hil, "tcp_port"))) e.hil_tcp_port = val->valueint;
        if ((val = cJSON_GetObjectItem(hil, "raw_mode"))) e.hil_raw_mode = cJSON_IsTrue(val);
        if ((val = cJSON_GetObjectItem(hil, "fingerprint"))) snprintf(e.hil_fingerprint, sizeof(e.hil_fingerprint), "%s", val->valuestring);
    }
}

void App::saveSettings() {
    cJSON* root = cJSON_CreateObject();

    // Console
    cJSON_AddNumberToObject(root, "console_log_rate", console_log_rate);
    cJSON_AddNumberToObject(root, "console_max", console_max);
    cJSON_AddBoolToObject(root, "console_auto_scroll", console_auto_scroll);

    // Recording
    cJSON_AddNumberToObject(root, "record_rate_hz", record_rate_hz);

    // Input source
    cJSON_AddNumberToObject(root, "input_source", (int)input_source);

    // Active plugin name (saved by name so it survives plugin reorder)
    if (active_plugin_idx >= 0 && active_plugin_idx < plugin_mgr.pluginCount()) {
        const char* pname = plugin_mgr.pluginName(active_plugin_idx);
        if (pname) cJSON_AddStringToObject(root, "active_plugin_name", pname);
    }

    // Dynamics panel selection + panel visibility
    cJSON_AddNumberToObject(root, "selected_dynamics_id", selected_dynamics_id);
    cJSON_AddBoolToObject(root, "show_dynamics",      show_dynamics);
    cJSON_AddBoolToObject(root, "show_console",       show_console);
    cJSON_AddBoolToObject(root, "show_data_streams",  show_data_streams);

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

    cJSON* val;

    // Console
    if ((val = cJSON_GetObjectItem(root, "console_log_rate")))    console_log_rate = val->valueint;
    if ((val = cJSON_GetObjectItem(root, "console_max")))         console_max = val->valueint;
    if ((val = cJSON_GetObjectItem(root, "console_auto_scroll"))) console_auto_scroll = cJSON_IsTrue(val);

    // Recording
    if ((val = cJSON_GetObjectItem(root, "record_rate_hz")))    record_rate_hz = val->valueint;

    // Input source (old Manual=0 maps to Plugin; only CapturePlayback/Plugin are valid now)
    if ((val = cJSON_GetObjectItem(root, "input_source"))) {
        int src = val->valueint;
        if (src == (int)InputSource::CapturePlayback)
            input_source = InputSource::CapturePlayback;
        else
            input_source = InputSource::Plugin;
    }

    // Active plugin name (resolve index after plugins are already scanned)
    if ((val = cJSON_GetObjectItem(root, "active_plugin_name"))) {
        const char* pname = val->valuestring;
        if (pname) {
            for (int i = 0; i < plugin_mgr.pluginCount(); i++) {
                const char* n = plugin_mgr.pluginName(i);
                if (n && strcmp(n, pname) == 0) { active_plugin_idx = i; break; }
            }
        }
    }
    // Default to Manual Sliders plugin if no plugin was saved or found
    if (active_plugin_idx < 0 && input_source == InputSource::Plugin) {
        for (int i = 0; i < plugin_mgr.pluginCount(); i++) {
            const char* n = plugin_mgr.pluginName(i);
            if (n && strcmp(n, "Manual Sliders") == 0) { active_plugin_idx = i; break; }
        }
        if (active_plugin_idx < 0 && plugin_mgr.pluginCount() > 0)
            active_plugin_idx = 0;
    }

    // Dynamics panel selection + panel visibility
    if ((val = cJSON_GetObjectItem(root, "selected_dynamics_id")))
        selected_dynamics_id = val->valueint;
    if ((val = cJSON_GetObjectItem(root, "show_dynamics")))
        show_dynamics = cJSON_IsTrue(val);
    if ((val = cJSON_GetObjectItem(root, "show_console")))
        show_console = cJSON_IsTrue(val);
    if ((val = cJSON_GetObjectItem(root, "show_data_streams")))
        show_data_streams = cJSON_IsTrue(val);

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
    e.show_card = true;
    e.show_settings = false;
    e.show_platform = false;
    e.show_dynamics = false;
    e.show_console = false;
    e.viz_cam = {0.8f, 0.45f, 0.0f}; // azimuth ~45°, elevation ~25°, auto distance
    e.viz_split_ratio = 0.55f;
    e.rate_ik_hz = 0.0f;
    e.rate_tx_hz = 0.0f;
    e.rate_tel_hz = 0.0f;
    // HIL fields
    e.serial = nullptr;
    e.hil_tx_hz = 60;
    e.hil_last_tx_time = 0.0;
    e.hil_tel_seq = 0;
    memset(e.hil_fk_pose, 0, sizeof(e.hil_fk_pose));
    memset(e.hil_tel_prev, 0, sizeof(e.hil_tel_prev));
    memset(e.hil_tel_curr, 0, sizeof(e.hil_tel_curr));
    e.hil_tel_prev_time = 0.0;
    e.hil_tel_curr_time = 0.0;
    e.hil_tel_target_hz = 30;
    e.hil_tick_rate_us = 4;
    memset(e.hil_port, 0, sizeof(e.hil_port));
    e.hil_baud = 921600;
    e.hil_auto_connect = true;
    e.hil_last_reconnect = 0.0;
    memset(e.hil_tx_raw_f, 0, sizeof(e.hil_tx_raw_f));
    e.hil_network = false;
    snprintf(e.hil_host, sizeof(e.hil_host), "%s", "192.168.1.168");  // Voron default
    e.hil_udp_port = 8767;
    e.hil_tcp_port = 8789;
    e.hil_cap_raw = false;
    e.hil_raw_mode = false;
    memset(e.hil_fingerprint, 0, sizeof(e.hil_fingerprint));
    memset(e.hil_fw_version, 0, sizeof(e.hil_fw_version));
    e.hil_proto_ver = 0;
    e.hil_handshake_ok = false;
    e.hil_handshake_pending = false;
    e.hil_handshake_phase = HandshakePhase::Idle;
    e.hil_device_params.clear();
    e.hil_handshake_start = 0.0;
    memset(e.hil_handshake_msg, 0, sizeof(e.hil_handshake_msg));
    e.hil_hs_attempts = 0;
    e.hil_hs_last_send = 0.0;
    // Init TX raw to center (home) so platform doesn't jerk on connect
    {
        float max_raw = txMaxRawForBitDepth(e.config.bit_depth);
        for (int i = 0; i < 6; i++) e.hil_tx_raw[i] = (uint32_t)(max_raw * 0.5f);
    }
    // Init history ring buffers
    memset(e.history_angles, 0, sizeof(e.history_angles));
    memset(e.history_input, 0, sizeof(e.history_input));
    memset(e.history_time, 0, sizeof(e.history_time));
    e.history_head = 0;
    e.history_count = 0;
    // Init placeholder spectrum
    memset(e.spectrum, 0, sizeof(e.spectrum));
    e.spectrum_freq_max = 30.0f;  // placeholder max freq
    // Init dynamics transition
    memset(e.last_scaled_pct, 0, sizeof(e.last_scaled_pct));
    e.dyn_transition_active = false;
    e.dyn_transition_start = 0.0;
    e.dyn_transition_duration = 1.5f;
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

// ── HIL Serial Line Handler (multi-step handshake + logging) ─────────
//
// Handshake sequence:
//   1. FINGERPRINT? → verify device identity (MAC, fw version, protocol)
//   2. CONFIG?      → read geometry (RD, PD, L1, L2, height, theta_r, theta_p, servo config)
//   3. BITS?        → read input bit depth
//   4. Validate     → compare device params against app expectations
//   5. Sync         → push any corrections (BITS:N) then enable motion
//
// Motion packets are BLOCKED until phase reaches Ready.

static void advanceHandshake(App& app, Entity& e) {
    if (!e.serial) return;
    switch (e.hil_handshake_phase) {
        case HandshakePhase::WaitFingerprint:
            // Fingerprint received — query geometry next
            e.hil_handshake_phase = HandshakePhase::WaitConfig;
            e.hil_hs_last_send = app.frame_time;
            e.hil_hs_attempts = 0;
            snprintf(e.hil_handshake_msg, sizeof(e.hil_handshake_msg), "Querying geometry...");
            e.serial->write((const uint8_t*)"\0\0\0\0", 4);
            e.serial->sendCommand("CONFIG?");
            break;

        case HandshakePhase::WaitConfig:
            // Geometry received — query bit depth next
            e.hil_handshake_phase = HandshakePhase::WaitBits;
            e.hil_hs_last_send = app.frame_time;
            e.hil_hs_attempts = 0;
            snprintf(e.hil_handshake_msg, sizeof(e.hil_handshake_msg), "Querying bit depth...");
            e.serial->write((const uint8_t*)"\0\0\0\0", 4);
            e.serial->sendCommand("BITS?");
            break;

        case HandshakePhase::WaitBits: {
            // BITS? response received — now push our desired bit depth and wait for echo.
            // Motion stays BLOCKED until firmware confirms the correct BITS:N back.
            auto& dp = e.hil_device_params;

            // Check protocol version compatibility
            if (dp.proto_ver != 1) {
                app.log(e.id, "hil", "WARNING: Unknown protocol version %d (expected 1)", dp.proto_ver);
            }

            // Send BITS:N immediately — firmware will echo it back as "BITS:N,max_raw=..."
            {
                char cmd[32];
                snprintf(cmd, sizeof(cmd), "BITS:%d", e.config.bit_depth);
                e.serial->sendCommand(cmd);
                app.log(e.id, "hil", "BITS:%d sent — waiting for firmware echo...", e.config.bit_depth);
            }
            e.hil_handshake_phase = HandshakePhase::WaitBitsConfirm;
            e.hil_hs_last_send = app.frame_time;
            e.hil_hs_attempts = 0;
            snprintf(e.hil_handshake_msg, sizeof(e.hil_handshake_msg), "Confirming bit depth...");
            break;
        }

        case HandshakePhase::WaitBitsConfirm: {
            // Firmware echoed BITS:N confirmation — now validate and finalize.
            e.hil_handshake_phase = HandshakePhase::Validating;
            snprintf(e.hil_handshake_msg, sizeof(e.hil_handshake_msg), "Validating parameters...");

            auto& dp = e.hil_device_params;

            // Check geometry sync between app settings and device
            e.hil_geo_synced = true;
            if (dp.config_received) {
                const auto& geo = e.config.geometry;
                float diffs[] = {
                    fabsf(geo.RD - dp.RD),
                    fabsf(geo.PD - dp.PD),
                    fabsf(geo.ServoArmLengthL1 - dp.L1),
                    fabsf(geo.ConnectingArmLengthL2 - dp.L2),
                    fabsf(geo.platformHeight - dp.height),
                    fabsf(geo.theta_r - dp.theta_r),
                    fabsf(geo.theta_p - dp.theta_p),
                };
                for (int i = 0; i < 7; i++) {
                    if (diffs[i] > 0.05f) { e.hil_geo_synced = false; break; }
                }

                if (!e.hil_geo_synced) {
                    app.log(e.id, "hil", "Geometry out of sync — pushing app settings to device.");
                    app.log(e.id, "hil", "  App:    RD=%.2f PD=%.2f L1=%.2f L2=%.2f H=%.2f theta_r=%.2f theta_p=%.2f",
                        geo.RD, geo.PD, geo.ServoArmLengthL1, geo.ConnectingArmLengthL2,
                        geo.platformHeight, geo.theta_r, geo.theta_p);
                    app.log(e.id, "hil", "  Device: RD=%.2f PD=%.2f L1=%.2f L2=%.2f H=%.2f theta_r=%.2f theta_p=%.2f",
                        dp.RD, dp.PD, dp.L1, dp.L2, dp.height, dp.theta_r, dp.theta_p);
                    // Push app geometry to device (app is authoritative)
                    char cmd[128];
                    snprintf(cmd, sizeof(cmd), "CONFIG:RD=%.4f", geo.RD);
                    e.hil_cmd_queue.push_back(cmd);
                    snprintf(cmd, sizeof(cmd), "CONFIG:PD=%.4f", geo.PD);
                    e.hil_cmd_queue.push_back(cmd);
                    snprintf(cmd, sizeof(cmd), "CONFIG:L1=%.4f", geo.ServoArmLengthL1);
                    e.hil_cmd_queue.push_back(cmd);
                    snprintf(cmd, sizeof(cmd), "CONFIG:L2=%.4f", geo.ConnectingArmLengthL2);
                    e.hil_cmd_queue.push_back(cmd);
                    snprintf(cmd, sizeof(cmd), "CONFIG:height=%.4f", geo.platformHeight);
                    e.hil_cmd_queue.push_back(cmd);
                    snprintf(cmd, sizeof(cmd), "CONFIG:theta_r=%.4f", geo.theta_r);
                    e.hil_cmd_queue.push_back(cmd);
                    snprintf(cmd, sizeof(cmd), "CONFIG:theta_p=%.4f", geo.theta_p);
                    e.hil_cmd_queue.push_back(cmd);
                } else {
                    app.log(e.id, "hil", "Geometry in sync with device.");
                }
                // Import theta_s from device (not configurable via app UI, firmware-authoritative)
                if (dp.config_received) {
                    bool has_theta_s = false;
                    for (int i = 0; i < 6; i++) if (dp.theta_s[i] != 0.0f) { has_theta_s = true; break; }
                    if (has_theta_s) {
                        for (int i = 0; i < 6; i++) e.config.geometry.theta_s[i] = dp.theta_s[i];
                        e.config.rebuildPlatform();
                    }
                }
            }

            {
                e.hil_handshake_phase = HandshakePhase::Ready;
                e.hil_handshake_ok = true;
                // Reset FK pose so stale pose from prior connection doesn't
                // corrupt the visualization on the new connection.
                memset(e.hil_fk_pose, 0, sizeof(e.hil_fk_pose));
                e.hil_tel_seq = -1;
                e.state.ik_seq = 0;
                if (e.hil_geo_synced) {
                    snprintf(e.hil_handshake_msg, sizeof(e.hil_handshake_msg),
                        "Ready — fw %s, proto %d", dp.fw_version, dp.proto_ver);
                } else {
                    snprintf(e.hil_handshake_msg, sizeof(e.hil_handshake_msg),
                        "Ready — GEOMETRY OUT OF SYNC");
                }

                // Auto-detect platform type from firmware platform_id
                if (strstr(dp.platform_id, "mini") != nullptr) {
                    e.config.platform_type = PlatformType::PWMServo;
                    // Import servo params from device if available
                    if (dp.config_received) {
                        for (int i = 0; i < 6; i++)
                            e.config.servo.center_us[i] = dp.servo_center[i];
                        if (dp.pulse_per_rad > 0.0f)
                            e.config.servo.pulse_per_rad = dp.pulse_per_rad;
                    }
                    app.log(e.id, "hil", "Platform type: PWM Servo (Mini-6DOF)");
                } else {
                    e.config.platform_type = PlatformType::Stepper;
                    app.log(e.id, "hil", "Platform type: Stepper");
                }

                // Send entity color to RGB LED on controller
                {
                    int r = (int)(e.color[0] * 255.0f);
                    int g = (int)(e.color[1] * 255.0f);
                    int b = (int)(e.color[2] * 255.0f);
                    char ledcmd[32];
                    snprintf(ledcmd, sizeof(ledcmd), "LED:%d,%d,%d", r, g, b);
                    e.hil_cmd_queue.push_back(ledcmd);
                }

                // Lock input source to serial, query stats, set telemetry rate + tick rate
                e.hil_cmd_queue.push_back("INPUT:SERIAL");
                e.hil_cmd_queue.push_back("INPUT_STAT");
                {
                    char telcmd[32];
                    snprintf(telcmd, sizeof(telcmd), "TELRATE:%d", e.hil_tel_target_hz);
                    e.hil_cmd_queue.push_back(telcmd);
                }
                {
                    char tickcmd[32];
                    snprintf(tickcmd, sizeof(tickcmd), "TICKRATE:%d", e.hil_tick_rate_us);
                    e.hil_cmd_queue.push_back(tickcmd);
                }

                app.log(e.id, "hil", "Handshake complete — motion enabled (fw %s, proto %d, bits %d, tel %dHz, tick %dµs)",
                    dp.fw_version, dp.proto_ver,
                    dp.bits_received ? dp.bit_depth : e.config.bit_depth,
                    e.hil_tel_target_hz, e.hil_tick_rate_us);
            }
            e.hil_handshake_pending = false;
            break;
        }
        default:
            break;
    }
}

void App::handleHilLine(int entity_id, const char* line) {
    // Always log the line to console
    log(entity_id, "esp32", "%s", line);

    Entity* e = findEntity(entity_id);
    if (!e) return;

    // ── Phase 1: FINGERPRINT response ────────────────────────────────
    // Format: "FINGERPRINT:AABBCCDDEEFF,fw=1.0.0,proto=1,platform=mini6dof"
    if (strncmp(line, "FINGERPRINT:", 12) == 0) {
        const char* payload = line + 12;

        // Extract MAC (12 hex chars before first comma)
        char mac[16] = {};
        const char* comma1 = strchr(payload, ',');
        if (comma1) {
            int len = (int)(comma1 - payload);
            if (len > 0 && len < (int)sizeof(mac)) {
                memcpy(mac, payload, len);
                mac[len] = '\0';
            }
        }

        // Extract fw version
        char fw[16] = {};
        const char* fw_start = strstr(payload, "fw=");
        if (fw_start) {
            fw_start += 3;
            const char* fw_end = strchr(fw_start, ',');
            int len = fw_end ? (int)(fw_end - fw_start) : (int)strlen(fw_start);
            if (len > 0 && len < (int)sizeof(fw)) {
                memcpy(fw, fw_start, len);
                fw[len] = '\0';
            }
        }

        // Extract protocol version
        int proto = 0;
        const char* proto_start = strstr(payload, "proto=");
        if (proto_start) proto = atoi(proto_start + 6);

        // Extract platform ID
        char plat[16] = {};
        const char* plat_start = strstr(payload, "platform=");
        if (plat_start) {
            plat_start += 9;
            const char* plat_end = strchr(plat_start, ',');
            int len = plat_end ? (int)(plat_end - plat_start) : (int)strlen(plat_start);
            if (len > 0 && len < (int)sizeof(plat)) {
                memcpy(plat, plat_start, len);
                plat[len] = '\0';
            }
        }

        // Store in device params
        auto& dp = e->hil_device_params;
        snprintf(dp.fingerprint, sizeof(dp.fingerprint), "%s", mac);
        snprintf(dp.fw_version, sizeof(dp.fw_version), "%s", fw);
        snprintf(dp.platform_id, sizeof(dp.platform_id), "%s", plat);
        dp.proto_ver = proto;

        // Also store in legacy fields for UI/save compat
        snprintf(e->hil_fw_version, sizeof(e->hil_fw_version), "%s", fw);
        e->hil_proto_ver = proto;

        // Raw-HIL capability gate: newer firmware advertises "rawhil=1" or a
        // caps token containing "raw" (e.g. caps=raw). Old firmware omits it →
        // hil_cap_raw stays false → baked path unchanged.
        {
            bool cap = false;
            if (strstr(payload, "rawhil=1")) cap = true;
            else { const char* caps = strstr(payload, "caps=");
                   if (caps && strstr(caps, "raw")) cap = true; }
            e->hil_cap_raw = cap;
            if (!cap) e->hil_raw_mode = false;  // can't stream raw to a baked-only device
        }

        // Fingerprint verification — ONLY advance handshake during WaitFingerprint.
        // Stale FINGERPRINT responses (from manual button clicks or previous sessions)
        // must not corrupt the state machine when we're at a different phase.
        bool during_fp_phase = (e->hil_handshake_phase == HandshakePhase::WaitFingerprint);

        if (e->hil_fingerprint[0] == '\0') {
            // First connect — store fingerprint
            snprintf(e->hil_fingerprint, sizeof(e->hil_fingerprint), "%s", mac);
            log(e->id, "hil", "Device paired: %s (fw %s, proto %d, platform %s)", mac, fw, proto, plat);
            settings_dirty = true;
            if (during_fp_phase) advanceHandshake(*this, *e);
        } else if (strcmp(e->hil_fingerprint, mac) == 0) {
            // Same device
            if (during_fp_phase) {
                log(e->id, "hil", "Device verified: %s (fw %s, proto %d, platform %s)", mac, fw, proto, plat);
                advanceHandshake(*this, *e);
            }
            // else: stale duplicate — ignore silently
        } else {
            // DIFFERENT device — reject!
            e->hil_handshake_ok = false;
            e->hil_handshake_phase = HandshakePhase::Failed;
            snprintf(e->hil_handshake_msg, sizeof(e->hil_handshake_msg),
                "FINGERPRINT MISMATCH — expected %s", e->hil_fingerprint);
            snprintf(e->hil_last_error, sizeof(e->hil_last_error),
                "FINGERPRINT MISMATCH\nConnected device: %s\nExpected: %s\nUse \"Clear Fingerprint\" to pair a new device.",
                mac, e->hil_fingerprint);
            log(e->id, "hil", "FINGERPRINT MISMATCH! Expected %s, got %s — disconnecting",
                e->hil_fingerprint, mac);
            log(e->id, "hil", "Clear fingerprint in Settings to pair a new device");
            if (e->serial) {
                e->serial->close();
                e->serial.reset();
            }
            e->transport.usb_connected = false;
            e->hil_auto_connect = false;
        }
        return;
    }

    // ── Phase 2: CONFIG response ─────────────────────────────────────
    // Format: "CONFIG:RD=15.75,PD=16.00,L1=7.25,L2=28.50,height=25.52,theta_r=10.00,theta_p=30.00"
    // Also accept "CONFIG:OK key=value" confirmations after push
    if (strncmp(line, "CONFIG:", 7) == 0) {
        // Skip CONFIG:OK and CONFIG:ERR confirmations (just log them)
        if (strncmp(line + 7, "OK ", 3) == 0 || strncmp(line + 7, "ERR", 3) == 0) {
            // Already logged via general serial output
            return;
        }

        // Accept CONFIG response during handshake or after (for re-query)
        bool during_handshake = (e->hil_handshake_phase == HandshakePhase::WaitConfig);
        auto& dp = e->hil_device_params;
        const char* p = line + 7;

        // Parse key=value pairs
        const char* v;
        if ((v = strstr(p, "RD=")))    dp.RD = (float)atof(v + 3);
        if ((v = strstr(p, "PD=")))    dp.PD = (float)atof(v + 3);
        if ((v = strstr(p, "L1=")))    dp.L1 = (float)atof(v + 3);
        if ((v = strstr(p, "L2=")))    dp.L2 = (float)atof(v + 3);
        if ((v = strstr(p, "height="))) dp.height = (float)atof(v + 7);
        if ((v = strstr(p, "theta_r="))) dp.theta_r = (float)atof(v + 8);
        if ((v = strstr(p, "theta_p="))) dp.theta_p = (float)atof(v + 8);
        if ((v = strstr(p, "theta_s="))) {
            sscanf(v + 8, "%f,%f,%f,%f,%f,%f",
                &dp.theta_s[0], &dp.theta_s[1], &dp.theta_s[2],
                &dp.theta_s[3], &dp.theta_s[4], &dp.theta_s[5]);
        }
        dp.config_received = true;

        // Re-check geometry sync after any CONFIG update
        if (!during_handshake) {
            const auto& geo = e->config.geometry;
            e->hil_geo_synced = true;
            float diffs[] = {
                fabsf(geo.RD - dp.RD), fabsf(geo.PD - dp.PD),
                fabsf(geo.ServoArmLengthL1 - dp.L1), fabsf(geo.ConnectingArmLengthL2 - dp.L2),
                fabsf(geo.platformHeight - dp.height),
                fabsf(geo.theta_r - dp.theta_r), fabsf(geo.theta_p - dp.theta_p),
            };
            for (int i = 0; i < 7; i++) {
                if (diffs[i] > 0.05f) { e->hil_geo_synced = false; break; }
            }
            if (e->hil_geo_synced) {
                log(e->id, "hil", "Geometry sync confirmed — device matches app settings.");
                snprintf(e->hil_handshake_msg, sizeof(e->hil_handshake_msg),
                    "Ready — fw %s, proto %d", dp.fw_version, dp.proto_ver);
            } else {
                log(e->id, "hil", "Geometry still out of sync after push.");
            }
        }
        // Advance handshake after CONFIG (stepper platforms don't send SERVO:)
        if (during_handshake) {
            advanceHandshake(*this, *e);
        }
        return;
    }

    // Format: "SERVO:center=1500,1500,1500,1500,1500,1500,pulse_per_rad=1018.6"
    if (strncmp(line, "SERVO:", 6) == 0 &&
        (e->hil_handshake_phase == HandshakePhase::WaitConfig || e->hil_handshake_phase == HandshakePhase::Ready)) {
        auto& dp = e->hil_device_params;
        const char* p = line + 6;

        const char* center = strstr(p, "center=");
        if (center) {
            sscanf(center + 7, "%d,%d,%d,%d,%d,%d",
                &dp.servo_center[0], &dp.servo_center[1], &dp.servo_center[2],
                &dp.servo_center[3], &dp.servo_center[4], &dp.servo_center[5]);
        }
        const char* ppr = strstr(p, "pulse_per_rad=");
        if (ppr) dp.pulse_per_rad = (float)atof(ppr + 14);

        // Both CONFIG and SERVO received — advance
        advanceHandshake(*this, *e);
        return;
    }

    // ── Phase 3: BITS? response ─────────────────────────────────────
    // Format: "BITS:12,max_raw=4095" — firmware reports current bit depth
    if (strncmp(line, "BITS:", 5) == 0 &&
        e->hil_handshake_phase == HandshakePhase::WaitBits) {
        auto& dp = e->hil_device_params;
        dp.bit_depth = atoi(line + 5);
        const char* mr = strstr(line, "max_raw=");
        if (mr) dp.max_raw = (float)atof(mr + 8);
        dp.bits_received = true;
        // Sends BITS:N and transitions to WaitBitsConfirm
        advanceHandshake(*this, *e);
        return;
    }

    // ── Phase 3b: BITS:N echo confirmation ──────────────────────────
    // Firmware echoes "BITS:N,max_raw=..." after we send BITS:N — confirms
    // the correct bit depth is active before motion TX begins.
    if (strncmp(line, "BITS:", 5) == 0 &&
        e->hil_handshake_phase == HandshakePhase::WaitBitsConfirm) {
        auto& dp = e->hil_device_params;
        int confirmed = atoi(line + 5);
        const char* mr = strstr(line, "max_raw=");
        if (mr) dp.max_raw = (float)atof(mr + 8);
        dp.bit_depth = confirmed;
        dp.bits_received = true;
        log(e->id, "hil", "BITS:%d confirmed by firmware (max_raw=%.0f) — motion enabled",
            confirmed, dp.max_raw);
        advanceHandshake(*this, *e);
        return;
    }
}

// ── Recording / Playback ────────────────────────────────────────────

void App::startRecording() {
    recording.samples.clear();
    recording.start_time = frame_time;
    recording.mode = RecordMode::Recording;
    // Initialize first-order hold state
    {
        std::lock_guard<std::mutex> lock(input_mutex);
        memcpy(recording.prev_input, shared_input, sizeof(recording.prev_input));
        memcpy(recording.curr_input, shared_input, sizeof(recording.curr_input));
    }
    recording.prev_time = frame_time;
    recording.curr_time = frame_time;
    const char* src = "plugin";
    if (input_source == InputSource::CapturePlayback) src = "capture";
    else if (active_plugin_idx >= 0) src = plugin_mgr.pluginName(active_plugin_idx);
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

    // Auto-save with date/time + source name
    if (!recording.samples.empty()) {
        time_t now = time(nullptr);
        struct tm lt;
#ifdef _WIN32
        localtime_s(&lt, &now);
#else
        localtime_r(&now, &lt);
#endif
        const char* src = "Plugin";
        if (input_source == InputSource::CapturePlayback) src = "Capture";
        else if (active_plugin_idx >= 0) {
            const char* pn = plugin_mgr.pluginName(active_plugin_idx);
            if (pn) src = pn;
        }
        char auto_name[64];
        snprintf(auto_name, sizeof(auto_name), "%04d-%02d-%02d %02d:%02d:%02d %s",
                 lt.tm_year + 1900, lt.tm_mon + 1, lt.tm_mday,
                 lt.tm_hour, lt.tm_min, lt.tm_sec, src);
        saveRecordingToLibrary(auto_name);
        // Auto-select the newly saved recording
        capture_playback_idx = (int)saved_recordings.size() - 1;
    }
}


// ── Capture Library ─────────────────────────────────────────────────

void App::saveRecordingToLibrary(const char* name) {
    if (recording.samples.empty()) {
        log(-1, "capture", "Nothing to save — no samples");
        return;
    }
    SavedRecording sr;
    snprintf(sr.name, sizeof(sr.name), "%s", name);
    // With fixed-rate oversampling, the configured rate is exact by construction.
    sr.sample_rate_hz = (double)record_rate_hz;
    sr.created_time = (double)time(nullptr);
    const char* src = "plugin";
    if (input_source == InputSource::CapturePlayback) src = "capture";
    snprintf(sr.source, sizeof(sr.source), "%s", src);
    sr.samples = recording.samples;
    saved_recordings.push_back(std::move(sr));
    log(-1, "capture", "Saved \"%s\" (%d samples, %.1fs, %.0f Hz actual)",
        name, (int)saved_recordings.back().samples.size(),
        saved_recordings.back().duration(), saved_recordings.back().sample_rate_hz);
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
    capture_stop_requested = false;
    input_source = InputSource::CapturePlayback;

    // Start with ramp-in phase
    capture_ramp_phase = CaptureRampPhase::RampIn;
    capture_ramp_start = frame_time;
    memset(capture_last_vals, 0, sizeof(capture_last_vals));

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
    if (!capture_playing) return;
    // If currently playing data, initiate ramp-out instead of instant stop
    if (capture_ramp_phase == CaptureRampPhase::RampIn ||
        capture_ramp_phase == CaptureRampPhase::Playing) {
        capture_stop_requested = true;
        capture_ramp_phase = CaptureRampPhase::RampOut;
        capture_ramp_start = frame_time;
        // Snapshot current values for smooth blend to home
        if (!entities.empty())
            memcpy(capture_last_vals, entities[0].state.input_pct, sizeof(capture_last_vals));
        log(-1, "capture", "Ramping down to home...");
        return;
    }
    // Already ramping out or at home — force stop
    capture_playing = false;
    // Zero all entities to home
    for (auto& e : entities)
        memset(e.state.input_pct, 0, sizeof(e.state.input_pct));
    {
        std::lock_guard<std::mutex> lock(input_mutex);
        memset(shared_input, 0, sizeof(shared_input));
    }
    log(-1, "capture", "Playback stopped");
}

// Smoothstep: 0→1 for t in [0,1]
static float smoothstep01(float t) {
    if (t <= 0.0f) return 0.0f;
    if (t >= 1.0f) return 1.0f;
    return t * t * (3.0f - 2.0f * t);
}

// Sample the recording at a given elapsed time, interpolating between samples.
// Uses O(1) direct index lookup via sample_rate_hz (audio-style fixed-rate).
// The cursor parameter is updated for compatibility but not used for lookup.
static void sampleRecording(SavedRecording& sr, double elapsed, int& cursor, float out[6]) {
    auto& samples = sr.samples;
    if (samples.empty()) { memset(out, 0, 6 * sizeof(float)); return; }

    double dur = sr.duration();
    if (elapsed < 0.0) elapsed = 0.0;
    if (elapsed > dur) elapsed = dur;

    // Direct index from sample rate: O(1) lookup (no linear scan)
    double fidx = elapsed * sr.sample_rate_hz;
    int idx = (int)fidx;
    if (idx >= (int)samples.size() - 1) {
        memcpy(out, samples.back().input, 6 * sizeof(float));
        cursor = (int)samples.size() - 1;
        return;
    }
    cursor = idx;
    float alpha = (float)(fidx - (double)idx);
    for (int i = 0; i < 6; i++) {
        out[i] = samples[idx].input[i] * (1.0f - alpha) +
                 samples[idx + 1].input[i] * alpha;
    }
}

void App::updateCapturePlayback() {
    if (!capture_playing) return;
    if (capture_playback_idx < 0 || capture_playback_idx >= (int)saved_recordings.size()) {
        capture_playing = false;
        return;
    }

    auto& sr = saved_recordings[capture_playback_idx];
    if (sr.samples.empty()) { capture_playing = false; return; }

    float output[6] = {};
    double phase_elapsed = frame_time - capture_ramp_start;

    switch (capture_ramp_phase) {
        case CaptureRampPhase::RampIn: {
            // Ramp envelope from 0→1 over CAPTURE_RAMP_IN_S
            float env = smoothstep01((float)(phase_elapsed / CAPTURE_RAMP_IN_S));

            // Sample recording data (data clock runs during ramp-in)
            double data_elapsed = (frame_time - capture_start_time) * (double)capture_speed;
            float data[6];
            sampleRecording(sr, data_elapsed, capture_play_cursor, data);

            for (int i = 0; i < 6; i++)
                output[i] = data[i] * env;

            // Transition to Playing when ramp complete
            if (phase_elapsed >= CAPTURE_RAMP_IN_S)
                capture_ramp_phase = CaptureRampPhase::Playing;
            break;
        }

        case CaptureRampPhase::Playing: {
            double data_elapsed = (frame_time - capture_start_time) * (double)capture_speed;
            double dur = sr.duration();

            if (data_elapsed >= dur) {
                // End of data — start ramp-out
                // Snapshot last values for smooth blend
                if (!entities.empty())
                    memcpy(capture_last_vals, entities[0].state.input_pct, sizeof(capture_last_vals));
                capture_ramp_phase = CaptureRampPhase::RampOut;
                capture_ramp_start = frame_time;
                capture_stop_requested = !capture_loop;
                // Output the last values this frame
                memcpy(output, capture_last_vals, sizeof(output));
            } else {
                sampleRecording(sr, data_elapsed, capture_play_cursor, output);
            }
            break;
        }

        case CaptureRampPhase::RampOut: {
            // Ramp envelope from 1→0 over CAPTURE_RAMP_OUT_S
            float env = 1.0f - smoothstep01((float)(phase_elapsed / CAPTURE_RAMP_OUT_S));

            // Blend last data values toward home (0)
            for (int i = 0; i < 6; i++)
                output[i] = capture_last_vals[i] * env;

            if (phase_elapsed >= CAPTURE_RAMP_OUT_S) {
                if (capture_stop_requested) {
                    // Done — full stop
                    capture_playing = false;
                    for (auto& e : entities)
                        memset(e.state.input_pct, 0, sizeof(e.state.input_pct));
                    {
                        std::lock_guard<std::mutex> lock(input_mutex);
                        memset(shared_input, 0, sizeof(shared_input));
                    }
                    log(-1, "capture", "Playback stopped");
                    return;
                } else {
                    // Loop: hold at home briefly
                    capture_ramp_phase = CaptureRampPhase::HomeHold;
                    capture_ramp_start = frame_time;
                    memset(output, 0, sizeof(output));
                }
            }
            break;
        }

        case CaptureRampPhase::HomeHold: {
            // Hold at home for CAPTURE_HOME_HOLD_S
            memset(output, 0, sizeof(output));

            if (phase_elapsed >= CAPTURE_HOME_HOLD_S) {
                // Restart: reset data clock and cursor, ramp back in
                capture_start_time = frame_time;
                capture_play_cursor = 0;
                capture_ramp_phase = CaptureRampPhase::RampIn;
                capture_ramp_start = frame_time;
                log(-1, "capture", "Looping playback...");
            }
            break;
        }
    }

    // Feed into all entities and shared_input
    for (auto& e : entities) {
        memcpy(e.state.input_pct, output, sizeof(output));
    }
    {
        std::lock_guard<std::mutex> lock(input_mutex);
        memcpy(shared_input, output, sizeof(output));
    }
}

static const char* RECORDINGS_DIR = "recordings";

void App::saveRecordingsToDisk() {
#ifdef _WIN32
    CreateDirectoryA(RECORDINGS_DIR, NULL);
#else
    mkdir(RECORDINGS_DIR, 0755);
#endif

    // Remove stale .stwr/.bin files beyond current recording count
    for (int ri = (int)saved_recordings.size(); ri < 1000; ri++) {
        char old_path[256];
        bool found = false;
        snprintf(old_path, sizeof(old_path), "%s/%03d.stwr", RECORDINGS_DIR, ri);
        if (remove(old_path) == 0) found = true;
        snprintf(old_path, sizeof(old_path), "%s/%03d.bin", RECORDINGS_DIR, ri);
        if (remove(old_path) == 0) found = true;
        if (!found) break;  // no more files at this index, stop scanning
    }

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

        // Recompute actual sample rate from timestamps (fixes stale values
        // saved when configured rate exceeded achievable frame rate)
        if (sr.samples.size() > 1) {
            double dur = sr.samples.back().time;
            if (dur > 0.0)
                sr.sample_rate_hz = (double)(sr.samples.size() - 1) / dur;
        }

        saved_recordings.push_back(std::move(sr));
        ri++;
    }

    cJSON_Delete(root);
    if (!saved_recordings.empty()) {
        log(-1, "capture", "Loaded %d saved recording(s) from disk", (int)saved_recordings.size());
    }
}

// ── MCA Dynamics Presets ─────────────────────────────────────────────

static const char* MCA_PRESETS_FILE = "mca_dynamics_presets.json";

// Helper: serialize a BiquadFilter's tuning params (fc, Q) to JSON
static void biquadToJSON(cJSON* obj, const char* prefix, const BiquadFilter& f) {
    char key[32];
    snprintf(key, sizeof(key), "%s_fc", prefix);  cJSON_AddNumberToObject(obj, key, f.fc);
    snprintf(key, sizeof(key), "%s_Q", prefix);   cJSON_AddNumberToObject(obj, key, f.Q);
}

static void biquadFromJSON(cJSON* obj, const char* prefix, BiquadFilter& f) {
    char key[32]; cJSON* val;
    snprintf(key, sizeof(key), "%s_fc", prefix);
    if ((val = cJSON_GetObjectItem(obj, key))) f.fc = (float)val->valuedouble;
    snprintf(key, sizeof(key), "%s_Q", prefix);
    if ((val = cJSON_GetObjectItem(obj, key))) f.Q = (float)val->valuedouble;
}

static cJSON* mcaConfigToJSON(const MotionCueingConfig& mca, float intensity, const float axis_gain[6]) {
    cJSON* obj = cJSON_CreateObject();
    cJSON_AddNumberToObject(obj, "enabled", mca.enabled);
    cJSON_AddNumberToObject(obj, "preset", mca.preset);
    cJSON_AddNumberToObject(obj, "intensity", intensity);

    cJSON* gains = cJSON_AddArrayToObject(obj, "axis_gain");
    for (int i = 0; i < 6; i++) cJSON_AddItemToArray(gains, cJSON_CreateNumber(axis_gain[i]));

    // Per-axis channel filters
    cJSON* channels = cJSON_AddArrayToObject(obj, "channels");
    for (int i = 0; i < 6; i++) {
        const AxisChannelFilter& ch = mca.channels[i];
        cJSON* cj = cJSON_CreateObject();
        cJSON_AddNumberToObject(cj, "hp_enabled", ch.hp_enabled);
        biquadToJSON(cj, "hp", ch.hp);
        cJSON_AddNumberToObject(cj, "lp_enabled", ch.lp_enabled);
        biquadToJSON(cj, "lp", ch.lp);
        cJSON_AddNumberToObject(cj, "gain", ch.gain);
        cJSON_AddNumberToObject(cj, "rate_limit", ch.rate_limit);
        cJSON_AddItemToArray(channels, cj);
    }

    // Tilt coordination
    cJSON* tilt = cJSON_AddObjectToObject(obj, "tilt");
    cJSON_AddNumberToObject(tilt, "enabled", mca.tilt.enabled);
    cJSON_AddNumberToObject(tilt, "surge_gain", mca.tilt.surge_gain);
    cJSON_AddNumberToObject(tilt, "sway_gain", mca.tilt.sway_gain);
    cJSON_AddNumberToObject(tilt, "fc", mca.tilt.fc);
    cJSON_AddNumberToObject(tilt, "Q", mca.tilt.Q);
    cJSON_AddNumberToObject(tilt, "hp_enabled", mca.tilt.hp_enabled);
    cJSON_AddNumberToObject(tilt, "hp_fc", mca.tilt.hp_fc);
    cJSON_AddNumberToObject(tilt, "hp_Q", mca.tilt.hp_Q);
    cJSON_AddNumberToObject(tilt, "surge_hp_enabled", mca.tilt.surge_hp_enabled);
    cJSON_AddNumberToObject(tilt, "sway_hp_enabled", mca.tilt.sway_hp_enabled);
    cJSON_AddNumberToObject(tilt, "sway_hp_fc", mca.tilt.sway_hp_fc);
    cJSON_AddNumberToObject(tilt, "sway_hp_Q", mca.tilt.sway_hp_Q);
    cJSON_AddNumberToObject(tilt, "hp_linked", mca.tilt.hp_linked);

    return obj;
}

static void mcaConfigFromJSON(cJSON* obj, MotionCueingConfig& mca, float& intensity, float axis_gain[6]) {
    cJSON* val;
    if ((val = cJSON_GetObjectItem(obj, "enabled")))   mca.enabled = val->valueint;
    if ((val = cJSON_GetObjectItem(obj, "preset")))     mca.preset = val->valueint;
    if ((val = cJSON_GetObjectItem(obj, "intensity")))  intensity = (float)val->valuedouble;

    cJSON* gains = cJSON_GetObjectItem(obj, "axis_gain");
    if (gains && cJSON_IsArray(gains)) {
        for (int i = 0; i < 6 && i < cJSON_GetArraySize(gains); i++)
            axis_gain[i] = (float)cJSON_GetArrayItem(gains, i)->valuedouble;
    }

    cJSON* channels = cJSON_GetObjectItem(obj, "channels");
    if (channels && cJSON_IsArray(channels)) {
        for (int i = 0; i < 6 && i < cJSON_GetArraySize(channels); i++) {
            cJSON* cj = cJSON_GetArrayItem(channels, i);
            AxisChannelFilter& ch = mca.channels[i];
            if ((val = cJSON_GetObjectItem(cj, "hp_enabled"))) ch.hp_enabled = val->valueint;
            biquadFromJSON(cj, "hp", ch.hp);
            if ((val = cJSON_GetObjectItem(cj, "lp_enabled"))) ch.lp_enabled = val->valueint;
            biquadFromJSON(cj, "lp", ch.lp);
            if ((val = cJSON_GetObjectItem(cj, "gain")))       ch.gain = (float)val->valuedouble;
            if ((val = cJSON_GetObjectItem(cj, "rate_limit"))) ch.rate_limit = (float)val->valuedouble;
        }
    }

    cJSON* tilt = cJSON_GetObjectItem(obj, "tilt");
    if (tilt && cJSON_IsObject(tilt)) {
        if ((val = cJSON_GetObjectItem(tilt, "enabled")))    mca.tilt.enabled = val->valueint;
        if ((val = cJSON_GetObjectItem(tilt, "surge_gain"))) mca.tilt.surge_gain = (float)val->valuedouble;
        if ((val = cJSON_GetObjectItem(tilt, "sway_gain")))  mca.tilt.sway_gain = (float)val->valuedouble;
        if ((val = cJSON_GetObjectItem(tilt, "fc")))         mca.tilt.fc = (float)val->valuedouble;
        if ((val = cJSON_GetObjectItem(tilt, "Q")))          mca.tilt.Q = (float)val->valuedouble;
        if ((val = cJSON_GetObjectItem(tilt, "hp_enabled"))) mca.tilt.hp_enabled = val->valueint;
        if ((val = cJSON_GetObjectItem(tilt, "hp_fc")))      mca.tilt.hp_fc = (float)val->valuedouble;
        if ((val = cJSON_GetObjectItem(tilt, "hp_Q")))       mca.tilt.hp_Q = (float)val->valuedouble;

        // v5 independent surge/sway HP fields
        if ((val = cJSON_GetObjectItem(tilt, "surge_hp_enabled"))) mca.tilt.surge_hp_enabled = val->valueint;
        if ((val = cJSON_GetObjectItem(tilt, "sway_hp_enabled")))  mca.tilt.sway_hp_enabled = val->valueint;
        if ((val = cJSON_GetObjectItem(tilt, "sway_hp_fc")))       mca.tilt.sway_hp_fc = (float)val->valuedouble;
        if ((val = cJSON_GetObjectItem(tilt, "sway_hp_Q")))        mca.tilt.sway_hp_Q = (float)val->valuedouble;
        if ((val = cJSON_GetObjectItem(tilt, "hp_linked")))        mca.tilt.hp_linked = val->valueint;

        // Backward compat: old configs without per-channel fields → copy from shared
        if (!cJSON_GetObjectItem(tilt, "surge_hp_enabled")) {
            mca.tilt.surge_hp_enabled = mca.tilt.hp_enabled;
            mca.tilt.sway_hp_enabled  = mca.tilt.hp_enabled;
            mca.tilt.sway_hp_fc = mca.tilt.hp_fc;
            mca.tilt.sway_hp_Q  = mca.tilt.hp_Q;
            mca.tilt.hp_linked = 1;
        }
    }
}

void App::saveMcaPreset(const char* name, const MotionCueingConfig& mca, float intensity, const float axis_gain[6]) {
    // Overwrite if same name exists
    for (auto& p : mca_presets) {
        if (strcmp(p.name, name) == 0) {
            p.mca = mca;
            p.intensity = intensity;
            memcpy(p.axis_gain, axis_gain, sizeof(p.axis_gain));
            saveMcaPresetsToDisk();
            log(-1, "dynamics", "Preset '%s' updated", name);
            return;
        }
    }
    McaDynamicsPreset p;
    memset(&p, 0, sizeof(p));
    snprintf(p.name, sizeof(p.name), "%s", name);
    p.is_builtin = false;
    p.mca = mca;
    p.intensity = intensity;
    memcpy(p.axis_gain, axis_gain, sizeof(p.axis_gain));
    mca_presets.push_back(p);
    saveMcaPresetsToDisk();
    log(-1, "dynamics", "Preset '%s' saved", name);
}

void App::deleteMcaPreset(int idx) {
    if (idx >= 0 && idx < (int)mca_presets.size() && !mca_presets[idx].is_builtin) {
        log(-1, "dynamics", "Preset '%s' deleted", mca_presets[idx].name);
        mca_presets.erase(mca_presets.begin() + idx);
        saveMcaPresetsToDisk();
    }
}

void App::loadMcaPreset(int idx, MotionCueingConfig& mca, float& intensity, float axis_gain[6]) {
    if (idx < 0 || idx >= (int)mca_presets.size()) return;
    const McaDynamicsPreset& p = mca_presets[idx];
    float sr = mca.sample_rate;  // preserve current sample rate
    mca = p.mca;
    mca.sample_rate = sr;
    intensity = p.intensity;
    memcpy(axis_gain, p.axis_gain, sizeof(float) * 6);
    // Recalculate biquad coefficients at current sample rate
    for (int i = 0; i < 6; i++) {
        if (mca.channels[i].hp_enabled && mca.channels[i].hp.fc > 0)
            biquadSetHighpass(&mca.channels[i].hp, mca.channels[i].hp.fc, sr, mca.channels[i].hp.Q);
        if (mca.channels[i].lp_enabled && mca.channels[i].lp.fc > 0)
            biquadSetLowpass(&mca.channels[i].lp, mca.channels[i].lp.fc, sr, mca.channels[i].lp.Q);
    }
    if (mca.tilt.enabled && mca.tilt.fc > 0) {
        biquadSetLowpass(&mca.tilt.surge_lp, mca.tilt.fc, sr, mca.tilt.Q);
        biquadSetLowpass(&mca.tilt.sway_lp,  mca.tilt.fc, sr, mca.tilt.Q);
    }
    resetMotionCueing(&mca);
    log(-1, "dynamics", "Loaded preset '%s'", p.name);
}

void App::saveMcaPresetsToDisk() {
    cJSON* root = cJSON_CreateArray();
    for (auto& p : mca_presets) {
        cJSON* pj = mcaConfigToJSON(p.mca, p.intensity, p.axis_gain);
        cJSON_AddStringToObject(pj, "name", p.name);
        cJSON_AddBoolToObject(pj, "is_builtin", p.is_builtin);
        cJSON_AddItemToArray(root, pj);
    }
    char* str = cJSON_Print(root);
    if (str) {
        FILE* f = fopen(MCA_PRESETS_FILE, "w");
        if (f) { fputs(str, f); fclose(f); }
        cJSON_free(str);
    }
    cJSON_Delete(root);
}

void App::loadMcaPresetsFromDisk() {
    mca_presets.clear();

    // Always seed with built-in presets
    const char* builtin_names[] = {"Off", "Gentle", "Moderate", "Aggressive", "Race Pro"};
    for (int i = 0; i < MCA_PRESET_COUNT; i++) {
        McaDynamicsPreset p;
        memset(&p, 0, sizeof(p));
        snprintf(p.name, sizeof(p.name), "%s", builtin_names[i]);
        p.is_builtin = true;
        initMotionCueing(&p.mca, 60.0f);
        setMotionCueingPreset(&p.mca, i);
        p.intensity = 100.0f;
        for (int j = 0; j < 6; j++) p.axis_gain[j] = 100.0f;
        mca_presets.push_back(p);
    }

    // Load user presets from file (may also override built-in presets if user saved over them)
    FILE* f = fopen(MCA_PRESETS_FILE, "r");
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

    cJSON* pj;
    int user_count = 0;
    cJSON_ArrayForEach(pj, root) {
        cJSON* name_val = cJSON_GetObjectItem(pj, "name");
        cJSON* builtin_val = cJSON_GetObjectItem(pj, "is_builtin");
        if (!name_val || !name_val->valuestring) continue;

        bool is_builtin = builtin_val && cJSON_IsTrue(builtin_val);

        if (is_builtin) {
            // Override the matching built-in preset with saved values
            for (auto& bp : mca_presets) {
                if (bp.is_builtin && strcmp(bp.name, name_val->valuestring) == 0) {
                    mcaConfigFromJSON(pj, bp.mca, bp.intensity, bp.axis_gain);
                    break;
                }
            }
        } else {
            // Add user preset
            McaDynamicsPreset p;
            memset(&p, 0, sizeof(p));
            snprintf(p.name, sizeof(p.name), "%s", name_val->valuestring);
            p.is_builtin = false;
            initMotionCueing(&p.mca, 60.0f);
            p.intensity = 100.0f;
            for (int j = 0; j < 6; j++) p.axis_gain[j] = 100.0f;
            mcaConfigFromJSON(pj, p.mca, p.intensity, p.axis_gain);
            mca_presets.push_back(p);
            user_count++;
        }
    }
    cJSON_Delete(root);
    if (user_count > 0)
        log(-1, "dynamics", "Loaded %d user preset(s)", user_count);
}

// ── UDP Listener ────────────────────────────────────────────────────

static double GetTimeSeconds() {
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}


// ── Source Switch Ramp-to-Home ───────────────────────────────────────

void App::requestSourceSwitch(InputSource target) {
    if (target == input_source && !source_switch_active) return;  // already there
    if (source_switch_active && target == source_switch_target) return;  // already ramping to this

    // Snapshot current input values for smooth blend
    {
        std::lock_guard<std::mutex> lock(input_mutex);
        memcpy(source_switch_from, shared_input, sizeof(source_switch_from));
    }
    source_switch_target = target;
    source_switch_start = frame_time;
    source_switch_active = true;
    log(-1, "input", "Ramping to home before switching to %s...",
        target == InputSource::CapturePlayback ? "Capture" :
        target == InputSource::Plugin ? "Plugin" : "?");
}

// ── Update ──────────────────────────────────────────────────────────

void App::update() {
    // ── Source-switch ramp-to-home ──
    if (source_switch_active) {
        double elapsed = frame_time - source_switch_start;
        float t = (SOURCE_RAMP_OUT_S > 0.0f) ? (float)(elapsed / SOURCE_RAMP_OUT_S) : 1.0f;
        if (t >= 1.0f) t = 1.0f;
        // Smoothstep ease-out
        float alpha = 1.0f - t * t * (3.0f - 2.0f * t);

        // Blend from snapshot toward zero
        float ramped[6];
        for (int i = 0; i < 6; i++)
            ramped[i] = source_switch_from[i] * alpha;

        // Write to all entities and shared_input
        for (auto& e : entities)
            memcpy(e.state.input_pct, ramped, sizeof(ramped));
        {
            std::lock_guard<std::mutex> lock(input_mutex);
            memcpy(shared_input, ramped, sizeof(ramped));
        }

        if (t >= 1.0f) {
            // Ramp complete — stop old source, switch to new
            if (capture_playing)        stopCapturePlayback();
            if (plugin_mgr.activeIndex() >= 0) {
                plugin_mgr.deactivateActive();
                active_plugin_idx = -1;
            }

            input_source = source_switch_target;
            source_switch_active = false;

            // Zero everything
            memset(shared_input, 0, sizeof(shared_input));
            for (auto& e : entities)
                memset(e.state.input_pct, 0, sizeof(e.state.input_pct));

            // Activate plugin if switching to Plugin source AND motion is started
            if (motion_started && input_source == InputSource::Plugin && active_plugin_idx >= 0) {
                float sr = (fps > 1.0) ? (float)fps : 60.0f;
                if (!plugin_mgr.activatePlugin(active_plugin_idx, sr)) {
                    log(-1, "plugin", "Failed to activate plugin");
                    active_plugin_idx = -1;
                }
            }

            log(-1, "input", "Source switched, rig at home");
        }
        // Skip normal input processing during ramp
        goto skip_input_processing;
    }

    // ── Motion gate: when stopped, force everything to zero ──
    if (!motion_started) {
        if (capture_playing) {
            capture_playing = false;
            log(-1, "capture", "Playback stopped (motion off)");
        }
        for (auto& e : entities)
            memset(e.state.input_pct, 0, sizeof(e.state.input_pct));
        {
            std::lock_guard<std::mutex> lock(input_mutex);
            memset(shared_input, 0, sizeof(shared_input));
        }
        goto skip_input_processing;
    }

    // ── S-curve ramp from home to current input on START ──
    if (start_ramp_active) {
        float t = (float)(frame_time - start_ramp_begin) / START_RAMP_S;
        if (t >= 1.0f) {
            t = 1.0f;
            start_ramp_active = false;
        }
        // Smoothstep S-curve: 3t² - 2t³
        float s = t * t * (3.0f - 2.0f * t);
        for (auto& e : entities) {
            for (int i = 0; i < 6; i++)
                e.state.input_pct[i] = start_ramp_target[i] * s;
        }
        {
            std::lock_guard<std::mutex> lock(input_mutex);
            for (int i = 0; i < 6; i++)
                shared_input[i] = start_ramp_target[i] * s;
        }
    }

    // Handle capture playback — writes directly to entity input_pct
    if (capture_playing) {
        updateCapturePlayback();
    }

    // ── Plugin input processing ─────────────────────────────────────
    if (input_source == InputSource::Plugin && plugin_mgr.activeIndex() >= 0) {
        float plugin_out[6] = {};
        static double last_plugin_time = 0.0;
        double dt = (last_plugin_time > 0.0) ? (frame_time - last_plugin_time) : 0.0;
        if (dt < 0.0 || dt > 0.1) dt = 0.0;
        last_plugin_time = frame_time;
        float sr = (fps > 1.0) ? (float)fps : 60.0f;
        if (plugin_mgr.processActive(frame_time, dt, frame_count, sr, plugin_out)) {
            std::lock_guard<std::mutex> lock(input_mutex);
            memcpy(shared_input, plugin_out, sizeof(shared_input));
        }
    }

    // ── Input bus sync ────────────────────────────────────────────────
    // Ensure shared_input and entity.input_pct are always consistent.
    // Plugin writes to shared_input → push to entities.
    // Capture writes to entity.input_pct → push to shared_input.
    if (input_source == InputSource::Plugin && plugin_mgr.activeIndex() >= 0) {
        // External source → entities
        float snap[6];
        {
            std::lock_guard<std::mutex> lock(input_mutex);
            memcpy(snap, shared_input, sizeof(snap));
        }
        for (auto& e : entities) {
            memcpy(e.state.input_pct, snap, sizeof(snap));
        }
    } else {
        // Manual / Capture → shared_input
        if (!entities.empty()) {
            std::lock_guard<std::mutex> lock(input_mutex);
            memcpy(shared_input, entities[0].state.input_pct, sizeof(shared_input));
        }
    }

    // Record at exact configured sample rate via oversampling (audio-style).
    // Each frame, fill ALL samples that should exist up to wall-clock time.
    // First-order hold: linearly interpolate between previous and current frame
    // input values for smooth waveforms (eliminates staircase artifacts).
    // Result: exactly rate × duration samples with perfectly spaced timestamps.
    if (recording.mode == RecordMode::Recording) {
        // Advance first-order hold: prev ← old curr, curr ← new input
        memcpy(recording.prev_input, recording.curr_input, sizeof(recording.prev_input));
        recording.prev_time = recording.curr_time;
        {
            std::lock_guard<std::mutex> lock(input_mutex);
            memcpy(recording.curr_input, shared_input, sizeof(recording.curr_input));
        }
        recording.curr_time = frame_time;

        double elapsed = frame_time - recording.start_time;
        int target_count = (int)(elapsed * (double)record_rate_hz) + 1;
        if (target_count > (int)recording.samples.size()) {
            double frame_dt = recording.curr_time - recording.prev_time;
            while ((int)recording.samples.size() < target_count) {
                RecordSample s;
                s.time = (double)recording.samples.size() / (double)record_rate_hz;
                // Interpolate: where does this sample fall between prev and curr frame?
                double sample_wall = recording.start_time + s.time;
                float alpha = (frame_dt > 1e-9)
                    ? (float)((sample_wall - recording.prev_time) / frame_dt)
                    : 1.0f;
                if (alpha < 0.0f) alpha = 0.0f;
                if (alpha > 1.0f) alpha = 1.0f;
                for (int i = 0; i < 6; i++) {
                    s.input[i] = recording.prev_input[i] * (1.0f - alpha)
                               + recording.curr_input[i] * alpha;
                }
                recording.samples.push_back(s);
            }
        }
    }

    // Grab shared input (thread-safe)
    float current_input[6];
    {
        std::lock_guard<std::mutex> lock(input_mutex);
        memcpy(current_input, shared_input, sizeof(current_input));
    }

    // Block scope: everything here can be safely skipped by goto skip_input_processing
    {
    // Console input logging (user-controlled rate via console_log_rate)
    // Only log when the selected source is actually active/connected
    bool source_active = true;
    if (input_source == InputSource::Plugin && plugin_mgr.activeIndex() < 0) source_active = false;

    if (console_log_rate > 0 && source_active) {
        static const double rate_intervals[] = {0.0, 1.0, 0.1, 1.0/30.0, 1.0/60.0, 0.0};
        double interval = rate_intervals[console_log_rate];
        bool should_log = (interval <= 0.0) || (frame_time - last_input_log_time >= interval);
        if (should_log) {
            last_input_log_time = frame_time;
            const char* src_str = "plugin";
            if (input_source == InputSource::CapturePlayback) src_str = "capture";
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

            // Use ~1 second rolling window for responsive spectrum display
            int max_window = record_rate_hz;  // 1 second of samples
            int N = input_history_count;
            if (N > max_window) N = max_window;
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
    } // end block scope (skippable by goto)

skip_input_processing:
    // Run pipeline for each enabled entity
    for (auto& e : entities) {
        if (!e.enabled) continue;

        // HIL entities: block the entire pipeline until handshake completes.
        // Before Ready, we don't know geometry/bit-depth/scales — all output
        // would be garbage. Zero everything so the UI shows idle/home state.
        bool hil_blocked = (e.type == EntityType::HIL && !e.hil_handshake_ok);

        float scaled_pct[6] = {};

        if (!hil_blocked) {
            // Common: read input, apply input filter, MCA dynamics, then intensity/gain
            float pct[6];
            memcpy(pct, e.state.input_pct, sizeof(pct));

            // Pre-MCA input filtering (signal conditioning)
            if (e.config.input_filter.enabled) {
                float sr = (fps > 1.0) ? (float)fps : 60.0f;
                if (fabsf(e.config.input_filter.sample_rate - sr) > 5.0f)
                    inputFilterUpdateSampleRate(&e.config.input_filter, sr);
                float filt_out[6];
                processInputFilter(&e.config.input_filter, pct, filt_out);
                memcpy(pct, filt_out, sizeof(pct));
            }

            // MCA: HP washout + LP smoothing (if enabled via Dynamics panel)
            if (e.config.mca.enabled) {
                // Keep MCA sample rate in sync with actual frame rate
                float sr = (fps > 1.0) ? (float)fps : 60.0f;
                if (fabsf(e.config.mca.sample_rate - sr) > 5.0f)
                    mcaUpdateSampleRate(&e.config.mca, sr);
                float mca_out[6];
                processMotionCueing(&e.config.mca, pct, mca_out);
                memcpy(pct, mca_out, sizeof(pct));
            }

            for (int i = 0; i < 6; i++) {
                float inv = e.config.axis_invert[i] ? -1.0f : 1.0f;
                scaled_pct[i] = pct[i] * (e.config.intensity / 100.0f) * (e.config.axis_gain[i] / 100.0f) * inv;
            }

            // Dynamics apply S-curve transition: blend from old output to new
            if (e.dyn_transition_active) {
                float elapsed = (float)(frame_time - e.dyn_transition_start);
                float dur = e.dyn_transition_duration;
                if (elapsed >= dur) {
                    e.dyn_transition_active = false;
                } else {
                    float t = elapsed / dur;
                    t = t * t * (3.0f - 2.0f * t); // smoothstep
                    for (int i = 0; i < 6; i++)
                        scaled_pct[i] = e.dyn_transition_from[i] + (scaled_pct[i] - e.dyn_transition_from[i]) * t;
                }
            }
        } else {
            // HIL blocked: zero pipeline INPUT state only.
            // Do NOT zero output_angles / servo_util / max_util here —
            // those are owned by the telemetry update section and only
            // change when a new TEL line arrives.  Zeroing them every
            // frame causes flicker (zeroed on non-TEL frames, restored
            // on TEL frames → constant oscillation).
            memset(e.state.input_physical, 0, sizeof(e.state.input_physical));
        }
        memcpy(e.last_scaled_pct, scaled_pct, sizeof(scaled_pct));

        if (e.type == EntityType::SIL) {
            // ── SIL Pipeline: local IK computation ──
            float physical[6];
            for (int i = 0; i < 6; i++) {
                physical[i] = (scaled_pct[i] / 100.0f) * e.config.axis_scales.scale[i];
                if (e.config.axis_scales.is_angle[i]) {
                    physical[i] *= (float)(M_PI / 180.0);
                }
            }
            // IK X-axis = platform lateral, Y-axis = platform longitudinal
            { float tmp = physical[0]; physical[0] = physical[1]; physical[1] = tmp; }
            float angles[6];
            calcAllActuatorAngles(physical, &e.config.platform, angles);
            int valid_mask = validatePositionV2(physical, &e.config.platform);
            // Swap back so input_physical stays in app order for display
            { float tmp = physical[0]; physical[0] = physical[1]; physical[1] = tmp; }
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
            // Auto-reconnect: retry every 3 seconds if auto_connect is enabled.
            // Serial-only — the network transport is (re)built from the device
            // card's Network Connect button, not auto-enumerated here.
            if (!e.hil_network && !e.serial && e.hil_auto_connect
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
                    DEV_LOG("hil", "Opening serial port %s at %d baud", e.hil_port, e.hil_baud);
                    if (sp->open(e.hil_port, e.hil_baud)) {
                        sp->setCobsMode(true);
                        e.serial = sp;
                        e.transport.usb_connected = true;
                        snprintf(e.transport.usb_port, sizeof(e.transport.usb_port), "%s", e.hil_port);
                        e.hil_tel_seq = 0;
                        e.hil_handshake_ok = false;
                        e.hil_handshake_pending = true;
                        e.hil_handshake_phase = HandshakePhase::WaitFingerprint;
                        e.hil_device_params.clear();
                        e.hil_handshake_start = frame_time;
                        e.hil_hs_attempts = 1;
                        e.hil_hs_last_send = frame_time;
                        snprintf(e.hil_handshake_msg, sizeof(e.hil_handshake_msg),
                                 "Requesting fingerprint...");
                        log(e.id, "hil", "Connected to %s — handshaking...", e.hil_port);
                        DEV_LOG("hil", "Connected to %s, sending FINGERPRINT? immediately", e.hil_port);
                        // Flush decoder state with COBS delimiters so the next
                        // command frame starts cleanly.
                        sp->write((const uint8_t*)"\0\0\0\0", 4);
                        sp->sendCommand("FINGERPRINT?");
                    } else {
                        DEV_WARN("hil", "Failed to open %s", e.hil_port);
                        log(e.id, "hil", "Auto-connect failed: %s (retrying...)", e.hil_port);
                    }
                }
            }

            // ── Drain serial line queue on main thread (thread-safe) ──
            if (e.serial && e.serial->isOpen()) {
                auto lines = e.serial->drainLines();
                for (const auto& line : lines) {
                    DEV_LOG("serial", "[E%d] RX: %s", e.id, line.c_str());
                    handleHilLine(e.id, line.c_str());
                }
            }

            // ── Proof-of-life handshake ──────────────────────────────────
            // Instead of blindly sending FINGERPRINT? on connect, we wait
            // for the first TEL line (proof the ESP32 is alive and its
            // serial task is running). Then we send FINGERPRINT? and retry
            // every 1s until we get a response. This is robust against:
            //   - ESP32 boot delay (DTR/RTS reset or power cycle)
            //   - Stale ASCII buffer on ESP32 (binary packet residue)
            //   - OS serial buffer latency
            if (e.serial && e.serial->isOpen()
                && e.hil_handshake_phase == HandshakePhase::WaitFingerprint
                && e.hil_handshake_pending) {

                double since_last = frame_time - e.hil_hs_last_send;

                // Retry every 0.5s — first attempt was sent on connect
                bool should_try = since_last > 0.5;

                if (should_try && e.hil_hs_attempts < 10) {
                    e.hil_hs_attempts++;
                    e.hil_hs_last_send = frame_time;
                    e.serial->write((const uint8_t*)"\0\0\0\0", 4);
                    e.serial->sendCommand("FINGERPRINT?");
                    DEV_LOG("hil", "Sending FINGERPRINT? (attempt %d/10)", e.hil_hs_attempts);
                    snprintf(e.hil_handshake_msg, sizeof(e.hil_handshake_msg),
                             "Requesting fingerprint...%s",
                             e.hil_hs_attempts > 1 ? " (retrying)" : "");
                }

                // Give up after 10 attempts
                if (e.hil_hs_attempts >= 10 && since_last > 1.0) {
                    e.hil_handshake_phase = HandshakePhase::Failed;
                    e.hil_handshake_ok = false;
                    e.hil_handshake_pending = false;
                    snprintf(e.hil_handshake_msg, sizeof(e.hil_handshake_msg),
                             "No response after %d attempts", e.hil_hs_attempts);
                    snprintf(e.hil_last_error, sizeof(e.hil_last_error),
                             "No FINGERPRINT response after %d attempts.\nDevice may be offline, crashed, or wrong firmware.",
                             e.hil_hs_attempts);
                    log(e.id, "hil", "Handshake failed — no FINGERPRINT response after %d attempts",
                        e.hil_hs_attempts);
                }
            }

            // ── Retry for CONFIG? / BITS? / BITS:N confirm phases ───────
            // If the response is lost, resend with flush.
            if (e.serial && e.serial->isOpen() && e.hil_handshake_pending
                && (e.hil_handshake_phase == HandshakePhase::WaitConfig
                 || e.hil_handshake_phase == HandshakePhase::WaitBits
                 || e.hil_handshake_phase == HandshakePhase::WaitBitsConfirm)) {
                bool is_config  = (e.hil_handshake_phase == HandshakePhase::WaitConfig);
                bool is_confirm = (e.hil_handshake_phase == HandshakePhase::WaitBitsConfirm);
                char bits_set_cmd[32];
                snprintf(bits_set_cmd, sizeof(bits_set_cmd), "BITS:%d", e.config.bit_depth);
                const char* cmd = is_config ? "CONFIG?" : (is_confirm ? bits_set_cmd : "BITS?");
                double since_last = frame_time - e.hil_hs_last_send;
                if (since_last > 0.5 && e.hil_hs_attempts < 5) {
                    e.hil_hs_attempts++;
                    e.hil_hs_last_send = frame_time;
                    // Full flush to drain any residual COBS state
                    e.serial->write((const uint8_t*)"\0\0\0\0", 4);
                    e.serial->sendCommand(cmd);
                    DEV_LOG("hil", "Retrying %s (attempt %d/5)", cmd, e.hil_hs_attempts);
                    snprintf(e.hil_handshake_msg, sizeof(e.hil_handshake_msg),
                             "%s (retry %d)...",
                             is_config ? "Querying geometry" : "Querying bit depth",
                             e.hil_hs_attempts);
                }
                if (e.hil_hs_attempts >= 5 && since_last > 2.0) {
                    // Capture label BEFORE changing phase
                    char fail_msg[128];
                    snprintf(fail_msg, sizeof(fail_msg),
                             "No response to %s after 5 attempts", cmd);
                    e.hil_handshake_phase = HandshakePhase::Failed;
                    e.hil_handshake_ok = false;
                    e.hil_handshake_pending = false;
                    snprintf(e.hil_handshake_msg, sizeof(e.hil_handshake_msg), "%s", fail_msg);
                    snprintf(e.hil_last_error, sizeof(e.hil_last_error), "%s", fail_msg);
                    log(e.id, "hil", "Handshake failed — %s", fail_msg);
                }
            }

            // ── HIL Pipeline: only active when ESP32 is connected ──
            // When offline, no work — no IK, no animation, no TX.
            if (e.serial && e.serial->isOpen()) {
                // Compute physical input for TX and history
                float physical[6];
                for (int i = 0; i < 6; i++) {
                    physical[i] = (scaled_pct[i] / 100.0f) * e.config.axis_scales.scale[i];
                    if (e.config.axis_scales.is_angle[i])
                        physical[i] *= (float)(M_PI / 180.0);
                }
                memcpy(e.state.input_physical, physical, sizeof(physical));

                // Drain command queue — one per frame, COBS framing handles interleaving
                if (!e.hil_cmd_queue.empty()) {
                    e.serial->sendCommand(e.hil_cmd_queue.front().c_str());
                    e.hil_cmd_queue.erase(e.hil_cmd_queue.begin());
                }

                // Prepare raw TX packet (only after handshake)
                if (e.hil_handshake_ok) {
                    float max_raw = txMaxRawForBitDepth(e.config.bit_depth);
                    float home = max_raw * 0.5f;
                    uint32_t raw[6];
                    for (int i = 0; i < 6; i++) {
                        float r = (scaled_pct[i] / 100.0f) * home + home;
                        if (r < 0.0f) r = 0.0f;
                        if (r > max_raw) r = max_raw;
                        raw[i] = (uint32_t)(r + 0.5f);
                    }
                    { uint32_t tmp = raw[0]; raw[0] = raw[1]; raw[1] = tmp; }
                    memcpy(e.hil_tx_raw, raw, sizeof(raw));

                    // RAW-HIL snapshot: pre-cueing input_pct in app axis order
                    // (surge=0, sway=1), NO swap — the ESP cues then swaps.
                    // Baked applies MCA/intensity/gain here; raw ships the input
                    // untouched so the device's single cue engine owns the feel.
                    memcpy(e.hil_tx_raw_f, e.state.input_pct, sizeof(e.hil_tx_raw_f));
                }

                // Retry TELRATE if telemetry isn't flowing after handshake
                if (e.hil_handshake_ok && !e.hil_tel_active && e.hil_cmd_queue.empty()) {
                    double now_t = std::chrono::duration<double>(
                        std::chrono::steady_clock::now().time_since_epoch()).count();
                    if (now_t - e.hil_tel_curr_time > 2.0) {
                        e.hil_tel_curr_time = now_t;
                        char cmd[32];
                        snprintf(cmd, sizeof(cmd), "TELRATE:%d", e.hil_tel_target_hz);
                        e.hil_cmd_queue.push_back(cmd);
                    }
                }

                // Telemetry: ESP32 owns output angles when active
                ESP32Telemetry tel = e.serial->getLatestTelemetry();
                e.hil_tel_active = (tel.seq != 0 && e.rate_tel_hz > 0.0f);
                if (tel.seq != e.hil_tel_seq) {
                    e.hil_tel_seq = tel.seq;

                    // NOTE: Spike rejection removed — COBS framing (fail=0) structurally
                    // prevents the stray-transport corruption this was guarding against.
                    // The old 15° threshold was rejecting valid high-intensity telemetry.

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

                e.rate_ik_hz = (float)fps;
                e.rate_tx_hz = (float)e.hil_tx_hz;
                e.rate_tel_hz = e.serial->telemetryRate();
            } else {
                // Offline — zero everything
                e.rate_ik_hz = 0.0f;
                e.rate_tx_hz = 0.0f;
                e.rate_tel_hz = 0.0f;
                e.hil_tel_active = false;
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
        static int    s_lograte = -1, s_max = -1, s_rec_rate = -1;
        static int    s_input_source = -1;
        static bool   s_autoscroll = false;
        static float  s_intensity[8] = {};  // up to 8 entities
        static float  s_gain[8][6] = {};
        static bool   s_invert[8][6] = {};
        static int    s_bit_depth[8] = {};
        static int    s_entity_count = 0;
        static char   s_hil_port[8][32] = {};
        static int    s_hil_baud[8] = {};
        static int    s_hil_tx_hz[8] = {};
        static bool   s_hil_auto[8] = {};
        static bool   s_hil_net[8] = {};
        static char   s_hil_host[8][64] = {};
        static int    s_hil_udp[8] = {};
        static int    s_hil_tcp[8] = {};
        static bool   s_hil_raw[8] = {};
        static int    s_dyn_id = -1;
        static bool   s_inited = false;

        auto snapshot_matches = [&]() -> bool {
            if (s_lograte != console_log_rate) return false;
            if (s_max != console_max) return false;
            if (s_autoscroll != console_auto_scroll) return false;
            if (s_rec_rate != record_rate_hz) return false;
            if (s_input_source != (int)input_source) return false;
            if (s_dyn_id != selected_dynamics_id) return false;
            if (s_entity_count != (int)entities.size()) return false;
            for (int ei = 0; ei < (int)entities.size() && ei < 8; ei++) {
                if (s_intensity[ei] != entities[ei].config.intensity) return false;
                if (s_bit_depth[ei] != entities[ei].config.bit_depth) return false;
                for (int a = 0; a < 6; a++) {
                    if (s_gain[ei][a] != entities[ei].config.axis_gain[a]) return false;
                    if (s_invert[ei][a] != entities[ei].config.axis_invert[a]) return false;
                }
                if (entities[ei].type == EntityType::HIL) {
                    if (strcmp(s_hil_port[ei], entities[ei].hil_port) != 0) return false;
                    if (s_hil_baud[ei] != entities[ei].hil_baud) return false;
                    if (s_hil_tx_hz[ei] != entities[ei].hil_tx_hz) return false;
                    if (s_hil_auto[ei] != entities[ei].hil_auto_connect) return false;
                    if (s_hil_net[ei] != entities[ei].hil_network) return false;
                    if (strcmp(s_hil_host[ei], entities[ei].hil_host) != 0) return false;
                    if (s_hil_udp[ei] != entities[ei].hil_udp_port) return false;
                    if (s_hil_tcp[ei] != entities[ei].hil_tcp_port) return false;
                    if (s_hil_raw[ei] != entities[ei].hil_raw_mode) return false;
                }
            }
            return true;
        };

        auto take_snapshot = [&]() {
            s_lograte = console_log_rate;
            s_max = console_max;
            s_autoscroll = console_auto_scroll;
            s_rec_rate = record_rate_hz;
            s_input_source = (int)input_source;
            s_dyn_id = selected_dynamics_id;
            s_entity_count = (int)entities.size();
            for (int ei = 0; ei < (int)entities.size() && ei < 8; ei++) {
                s_intensity[ei] = entities[ei].config.intensity;
                s_bit_depth[ei] = entities[ei].config.bit_depth;
                for (int a = 0; a < 6; a++) {
                    s_gain[ei][a] = entities[ei].config.axis_gain[a];
                    s_invert[ei][a] = entities[ei].config.axis_invert[a];
                }
                if (entities[ei].type == EntityType::HIL) {
                    snprintf(s_hil_port[ei], sizeof(s_hil_port[ei]), "%s", entities[ei].hil_port);
                    s_hil_baud[ei] = entities[ei].hil_baud;
                    s_hil_tx_hz[ei] = entities[ei].hil_tx_hz;
                    s_hil_auto[ei] = entities[ei].hil_auto_connect;
                    s_hil_net[ei] = entities[ei].hil_network;
                    snprintf(s_hil_host[ei], sizeof(s_hil_host[ei]), "%s", entities[ei].hil_host);
                    s_hil_udp[ei] = entities[ei].hil_udp_port;
                    s_hil_tcp[ei] = entities[ei].hil_tcp_port;
                    s_hil_raw[ei] = entities[ei].hil_raw_mode;
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
