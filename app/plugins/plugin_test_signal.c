/*
 * Test Signal Generator Plugin
 * ============================
 * Generates configurable waveforms on all 6 axes with per-axis frequency,
 * amplitude, phase offset, and DC offset controls. Supports sine, square,
 * triangle, and sawtooth waveforms with optional S-curve ramp-up.
 *
 * Build:
 *   Windows:  cl /LD /I ../src plugin_test_signal.c
 *   Linux:    gcc -shared -fPIC -I ../src -o plugin_test_signal.so plugin_test_signal.c -lm
 */

#include "plugin_api.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── Plugin state ─────────────────────────────────────────────────── */

static int   s_waveform     = 0;     /* 0=Sine, 1=Square, 2=Triangle, 3=Sawtooth */
static float s_frequency[6] = {1,1,1,1,1,1};
static float s_amplitude[6] = {50,50,50,50,50,50};
static float s_phase[6]     = {0,0,0,0,0,0};   /* degrees */
static float s_dc_offset[6] = {0,0,0,0,0,0};
static float s_enabled[6]   = {1,1,1,1,1,1};
static float s_ramp_dur     = 2.0f;  /* seconds */
static int   s_ramp_enable  = 1;
static double s_start_time  = 0.0;
static int   s_started      = 0;

/* Phase accumulators for glitch-free frequency changes */
static double s_phase_accum[6] = {0};
static double s_last_time      = 0.0;

/* ── Parameter declarations ───────────────────────────────────────── */

static const char* s_wave_labels = "Sine\0Square\0Triangle\0Sawtooth\0";

static const StewartParamDef s_params[] = {
    { "waveform",   "Waveform",     "Signal waveform type",
      STEWART_PARAM_ENUM, 0.0f, 0.0f, 3.0f, NULL },
    { "ramp_en",    "S-Curve Ramp", "Smoothly ramp amplitude from zero on start",
      STEWART_PARAM_BOOL, 1.0f, 0.0f, 1.0f, NULL },
    { "ramp_dur",   "Ramp Duration","Ramp-up time in seconds",
      STEWART_PARAM_FLOAT, 2.0f, 0.5f, 10.0f, NULL },

    /* Per-axis frequency */
    { "freq_surge", "Surge Freq",   "Surge frequency (Hz)",
      STEWART_PARAM_FLOAT, 1.0f, 0.01f, 20.0f, NULL },
    { "freq_sway",  "Sway Freq",    "Sway frequency (Hz)",
      STEWART_PARAM_FLOAT, 1.0f, 0.01f, 20.0f, NULL },
    { "freq_heave", "Heave Freq",   "Heave frequency (Hz)",
      STEWART_PARAM_FLOAT, 1.0f, 0.01f, 20.0f, NULL },
    { "freq_roll",  "Roll Freq",    "Roll frequency (Hz)",
      STEWART_PARAM_FLOAT, 1.0f, 0.01f, 20.0f, NULL },
    { "freq_pitch", "Pitch Freq",   "Pitch frequency (Hz)",
      STEWART_PARAM_FLOAT, 1.0f, 0.01f, 20.0f, NULL },
    { "freq_yaw",   "Yaw Freq",     "Yaw frequency (Hz)",
      STEWART_PARAM_FLOAT, 1.0f, 0.01f, 20.0f, NULL },

    /* Per-axis amplitude */
    { "amp_surge",  "Surge Amp",    "Surge amplitude (%)",
      STEWART_PARAM_FLOAT, 50.0f, 0.0f, 100.0f, NULL },
    { "amp_sway",   "Sway Amp",     "Sway amplitude (%)",
      STEWART_PARAM_FLOAT, 50.0f, 0.0f, 100.0f, NULL },
    { "amp_heave",  "Heave Amp",    "Heave amplitude (%)",
      STEWART_PARAM_FLOAT, 50.0f, 0.0f, 100.0f, NULL },
    { "amp_roll",   "Roll Amp",     "Roll amplitude (%)",
      STEWART_PARAM_FLOAT, 50.0f, 0.0f, 100.0f, NULL },
    { "amp_pitch",  "Pitch Amp",    "Pitch amplitude (%)",
      STEWART_PARAM_FLOAT, 50.0f, 0.0f, 100.0f, NULL },
    { "amp_yaw",    "Yaw Amp",      "Yaw amplitude (%)",
      STEWART_PARAM_FLOAT, 50.0f, 0.0f, 100.0f, NULL },

    /* Per-axis phase offset */
    { "phase_surge","Surge Phase",  "Surge phase offset (degrees)",
      STEWART_PARAM_FLOAT, 0.0f, 0.0f, 360.0f, NULL },
    { "phase_sway", "Sway Phase",   "Sway phase offset (degrees)",
      STEWART_PARAM_FLOAT, 0.0f, 0.0f, 360.0f, NULL },
    { "phase_heave","Heave Phase",  "Heave phase offset (degrees)",
      STEWART_PARAM_FLOAT, 0.0f, 0.0f, 360.0f, NULL },
    { "phase_roll", "Roll Phase",   "Roll phase offset (degrees)",
      STEWART_PARAM_FLOAT, 0.0f, 0.0f, 360.0f, NULL },
    { "phase_pitch","Pitch Phase",  "Pitch phase offset (degrees)",
      STEWART_PARAM_FLOAT, 0.0f, 0.0f, 360.0f, NULL },
    { "phase_yaw",  "Yaw Phase",    "Yaw phase offset (degrees)",
      STEWART_PARAM_FLOAT, 0.0f, 0.0f, 360.0f, NULL },

    /* Per-axis DC offset */
    { "dc_surge",   "Surge DC",     "Surge DC offset (%)",
      STEWART_PARAM_FLOAT, 0.0f, -100.0f, 100.0f, NULL },
    { "dc_sway",    "Sway DC",      "Sway DC offset (%)",
      STEWART_PARAM_FLOAT, 0.0f, -100.0f, 100.0f, NULL },
    { "dc_heave",   "Heave DC",     "Heave DC offset (%)",
      STEWART_PARAM_FLOAT, 0.0f, -100.0f, 100.0f, NULL },
    { "dc_roll",    "Roll DC",      "Roll DC offset (%)",
      STEWART_PARAM_FLOAT, 0.0f, -100.0f, 100.0f, NULL },
    { "dc_pitch",   "Pitch DC",     "Pitch DC offset (%)",
      STEWART_PARAM_FLOAT, 0.0f, -100.0f, 100.0f, NULL },
    { "dc_yaw",     "Yaw DC",       "Yaw DC offset (%)",
      STEWART_PARAM_FLOAT, 0.0f, -100.0f, 100.0f, NULL },

    /* Per-axis enable */
    { "en_surge",   "Surge On",     "Enable surge axis",
      STEWART_PARAM_BOOL, 1.0f, 0.0f, 1.0f, NULL },
    { "en_sway",    "Sway On",      "Enable sway axis",
      STEWART_PARAM_BOOL, 1.0f, 0.0f, 1.0f, NULL },
    { "en_heave",   "Heave On",     "Enable heave axis",
      STEWART_PARAM_BOOL, 1.0f, 0.0f, 1.0f, NULL },
    { "en_roll",    "Roll On",      "Enable roll axis",
      STEWART_PARAM_BOOL, 1.0f, 0.0f, 1.0f, NULL },
    { "en_pitch",   "Pitch On",     "Enable pitch axis",
      STEWART_PARAM_BOOL, 1.0f, 0.0f, 1.0f, NULL },
    { "en_yaw",     "Yaw On",       "Enable yaw axis",
      STEWART_PARAM_BOOL, 1.0f, 0.0f, 1.0f, NULL },
};

/* ── Plugin info ──────────────────────────────────────────────────── */

static const StewartPluginInfo s_info = {
    STEWART_PLUGIN_API_VERSION,
    "Test Signal Generator",
    "Stewart Platform Project",
    "1.0.0",
    "Generates configurable waveforms (sine, square, triangle, sawtooth) "
    "on all 6 axes with per-axis frequency, amplitude, phase, DC offset, "
    "and optional S-curve ramp-up envelope.",
    0,   /* preferred_rate_hz */
    6,   /* axis_count */
    { NULL, NULL, NULL, NULL, NULL, NULL },
    sizeof(s_params) / sizeof(s_params[0]),
    s_params
};

/* ── Waveform generation ──────────────────────────────────────────── */

static float generate_waveform(int type, double angle) {
    double phase = fmod(angle, 2.0 * M_PI);
    if (phase < 0.0) phase += 2.0 * M_PI;

    switch (type) {
        case 0: /* Sine */
            return (float)sin(angle);
        case 1: /* Square */
            return phase < M_PI ? 1.0f : -1.0f;
        case 2: /* Triangle */
            return (float)(2.0 / M_PI * asin(sin(angle)));
        case 3: /* Sawtooth */
            return (float)(2.0 * (angle / (2.0 * M_PI) - floor(0.5 + angle / (2.0 * M_PI))));
        default:
            return (float)sin(angle);
    }
}

/* ── Entry points ─────────────────────────────────────────────────── */

STEWART_EXPORT const StewartPluginInfo* stewart_plugin_info(void) {
    return &s_info;
}

STEWART_EXPORT int stewart_plugin_init(float sample_rate) {
    int i;
    (void)sample_rate;
    s_waveform = 0;
    s_ramp_enable = 1;
    s_ramp_dur = 2.0f;
    s_started = 0;
    s_last_time = 0.0;
    for (i = 0; i < 6; i++) {
        s_frequency[i] = 1.0f;
        s_amplitude[i] = 50.0f;
        s_phase[i] = 0.0f;
        s_dc_offset[i] = 0.0f;
        s_enabled[i] = 1.0f;
        s_phase_accum[i] = 0.0;
    }
    return 0;
}

STEWART_EXPORT int stewart_plugin_process(StewartPluginContext* ctx) {
    int i;
    double dt;
    float envelope;

    if (!s_started) {
        s_start_time = ctx->timestamp;
        s_last_time = ctx->timestamp;
        s_started = 1;
    }

    dt = ctx->timestamp - s_last_time;
    if (dt < 0.0 || dt > 0.1) dt = 0.0;
    s_last_time = ctx->timestamp;

    /* S-curve ramp envelope */
    envelope = 1.0f;
    if (s_ramp_enable && s_ramp_dur > 0.0f) {
        double elapsed = ctx->timestamp - s_start_time;
        if (elapsed < (double)s_ramp_dur) {
            float r = (float)(elapsed / (double)s_ramp_dur);
            envelope = r * r * (3.0f - 2.0f * r); /* smoothstep */
        }
    }

    for (i = 0; i < 6; i++) {
        if (s_enabled[i] == 0.0f) {
            ctx->output[i] = 0.0f;
            continue;
        }

        /* Accumulate phase for glitch-free frequency changes */
        s_phase_accum[i] += 2.0 * M_PI * (double)s_frequency[i] * dt;

        double angle = s_phase_accum[i] + (double)s_phase[i] * (M_PI / 180.0);
        float v = generate_waveform(s_waveform, angle);
        float out = v * s_amplitude[i] * envelope + s_dc_offset[i];

        /* Clamp to ±100% */
        if (out > 100.0f) out = 100.0f;
        if (out < -100.0f) out = -100.0f;
        ctx->output[i] = out;
    }

    return 0;
}

STEWART_EXPORT void stewart_plugin_shutdown(void) {
    int i;
    s_started = 0;
    for (i = 0; i < 6; i++) s_phase_accum[i] = 0.0;
}

STEWART_EXPORT void stewart_plugin_set_param(const char* name, float value) {
    /* Globals */
    if (strcmp(name, "waveform") == 0)    { s_waveform = (int)value; return; }
    if (strcmp(name, "ramp_en") == 0)     { s_ramp_enable = (int)value; return; }
    if (strcmp(name, "ramp_dur") == 0)    { s_ramp_dur = value; return; }

    /* Per-axis frequency */
    if (strcmp(name, "freq_surge") == 0)  { s_frequency[0] = value; return; }
    if (strcmp(name, "freq_sway") == 0)   { s_frequency[1] = value; return; }
    if (strcmp(name, "freq_heave") == 0)  { s_frequency[2] = value; return; }
    if (strcmp(name, "freq_roll") == 0)   { s_frequency[3] = value; return; }
    if (strcmp(name, "freq_pitch") == 0)  { s_frequency[4] = value; return; }
    if (strcmp(name, "freq_yaw") == 0)    { s_frequency[5] = value; return; }

    /* Per-axis amplitude */
    if (strcmp(name, "amp_surge") == 0)   { s_amplitude[0] = value; return; }
    if (strcmp(name, "amp_sway") == 0)    { s_amplitude[1] = value; return; }
    if (strcmp(name, "amp_heave") == 0)   { s_amplitude[2] = value; return; }
    if (strcmp(name, "amp_roll") == 0)    { s_amplitude[3] = value; return; }
    if (strcmp(name, "amp_pitch") == 0)   { s_amplitude[4] = value; return; }
    if (strcmp(name, "amp_yaw") == 0)     { s_amplitude[5] = value; return; }

    /* Per-axis phase */
    if (strcmp(name, "phase_surge") == 0) { s_phase[0] = value; return; }
    if (strcmp(name, "phase_sway") == 0)  { s_phase[1] = value; return; }
    if (strcmp(name, "phase_heave") == 0) { s_phase[2] = value; return; }
    if (strcmp(name, "phase_roll") == 0)  { s_phase[3] = value; return; }
    if (strcmp(name, "phase_pitch") == 0) { s_phase[4] = value; return; }
    if (strcmp(name, "phase_yaw") == 0)   { s_phase[5] = value; return; }

    /* Per-axis DC offset */
    if (strcmp(name, "dc_surge") == 0)    { s_dc_offset[0] = value; return; }
    if (strcmp(name, "dc_sway") == 0)     { s_dc_offset[1] = value; return; }
    if (strcmp(name, "dc_heave") == 0)    { s_dc_offset[2] = value; return; }
    if (strcmp(name, "dc_roll") == 0)     { s_dc_offset[3] = value; return; }
    if (strcmp(name, "dc_pitch") == 0)    { s_dc_offset[4] = value; return; }
    if (strcmp(name, "dc_yaw") == 0)      { s_dc_offset[5] = value; return; }

    /* Per-axis enable */
    if (strcmp(name, "en_surge") == 0)    { s_enabled[0] = value; return; }
    if (strcmp(name, "en_sway") == 0)     { s_enabled[1] = value; return; }
    if (strcmp(name, "en_heave") == 0)    { s_enabled[2] = value; return; }
    if (strcmp(name, "en_roll") == 0)     { s_enabled[3] = value; return; }
    if (strcmp(name, "en_pitch") == 0)    { s_enabled[4] = value; return; }
    if (strcmp(name, "en_yaw") == 0)      { s_enabled[5] = value; return; }
}
