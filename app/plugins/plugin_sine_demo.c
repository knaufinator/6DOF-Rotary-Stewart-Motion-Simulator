/*
 * Sine Wave Demo Plugin
 * =====================
 * Generates configurable sine waves on all 6 axes.
 * This serves as a reference implementation for plugin authors.
 *
 * Build:
 *   Windows:  cl /LD /I ../src plugin_sine_demo.c
 *   Linux:    gcc -shared -fPIC -I ../src -o plugin_sine_demo.so plugin_sine_demo.c -lm
 */

#include "plugin_api.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── Plugin state ─────────────────────────────────────────────────── */

static float s_frequency  = 1.0f;    /* Hz */
static float s_amplitude  = 50.0f;   /* % (±100 range) */
static float s_enabled[6] = {1,1,1,0,0,0};  /* surge/sway/heave on, roll/pitch/yaw off */
static float s_phase_spread = 60.0f; /* degrees between axes */
static float s_sample_rate = 60.0f;

/* ── Parameter declarations ───────────────────────────────────────── */

static const StewartParamDef s_params[] = {
    {
        "frequency", "Frequency", "Sine wave frequency in Hz",
        STEWART_PARAM_FLOAT, 1.0f, 0.1f, 20.0f, NULL
    },
    {
        "amplitude", "Amplitude", "Output amplitude (0-100%)",
        STEWART_PARAM_FLOAT, 50.0f, 0.0f, 100.0f, NULL
    },
    {
        "phase_spread", "Phase Spread", "Phase offset between adjacent axes (degrees)",
        STEWART_PARAM_FLOAT, 60.0f, 0.0f, 180.0f, NULL
    },
    {
        "surge_en", "Surge", "Enable surge axis",
        STEWART_PARAM_BOOL, 1.0f, 0.0f, 1.0f, NULL
    },
    {
        "sway_en", "Sway", "Enable sway axis",
        STEWART_PARAM_BOOL, 1.0f, 0.0f, 1.0f, NULL
    },
    {
        "heave_en", "Heave", "Enable heave axis",
        STEWART_PARAM_BOOL, 1.0f, 0.0f, 1.0f, NULL
    },
    {
        "roll_en", "Roll", "Enable roll axis",
        STEWART_PARAM_BOOL, 0.0f, 0.0f, 1.0f, NULL
    },
    {
        "pitch_en", "Pitch", "Enable pitch axis",
        STEWART_PARAM_BOOL, 0.0f, 0.0f, 1.0f, NULL
    },
    {
        "yaw_en", "Yaw", "Enable yaw axis",
        STEWART_PARAM_BOOL, 0.0f, 0.0f, 1.0f, NULL
    },
};

/* ── Plugin info ──────────────────────────────────────────────────── */

static const StewartPluginInfo s_info = {
    STEWART_PLUGIN_API_VERSION,
    "Sine Wave Demo",           /* name */
    "Stewart Platform Project", /* author */
    "1.0.0",                    /* version */
    "Generates sine waves on selectable axes with configurable frequency, amplitude, and phase spread.",
    0,                          /* preferred_rate_hz (0 = app default) */
    6,                          /* axis_count */
    { NULL, NULL, NULL, NULL, NULL, NULL },  /* axis_labels (use defaults) */
    sizeof(s_params) / sizeof(s_params[0]),  /* param_count */
    s_params                    /* params */
};

/* ── Entry points ─────────────────────────────────────────────────── */

STEWART_EXPORT const StewartPluginInfo* stewart_plugin_info(void) {
    return &s_info;
}

STEWART_EXPORT int stewart_plugin_init(float sample_rate) {
    s_sample_rate = sample_rate;
    s_frequency = 1.0f;
    s_amplitude = 50.0f;
    s_phase_spread = 60.0f;
    s_enabled[0] = 1; s_enabled[1] = 1; s_enabled[2] = 1;
    s_enabled[3] = 0; s_enabled[4] = 0; s_enabled[5] = 0;
    return 0;  /* success */
}

STEWART_EXPORT int stewart_plugin_process(StewartPluginContext* ctx) {
    for (int i = 0; i < 6; i++) {
        if (s_enabled[i] != 0.0f) {
            double phase = s_phase_spread * (double)i * (M_PI / 180.0);
            double angle = 2.0 * M_PI * (double)s_frequency * ctx->timestamp + phase;
            ctx->output[i] = (float)(sin(angle) * (double)s_amplitude);
        } else {
            ctx->output[i] = 0.0f;
        }
    }
    return 0;  /* success */
}

STEWART_EXPORT void stewart_plugin_shutdown(void) {
    /* nothing to clean up */
}

STEWART_EXPORT void stewart_plugin_set_param(const char* name, float value) {
    if (strcmp(name, "frequency") == 0)     s_frequency = value;
    else if (strcmp(name, "amplitude") == 0) s_amplitude = value;
    else if (strcmp(name, "phase_spread") == 0) s_phase_spread = value;
    else if (strcmp(name, "surge_en") == 0)  s_enabled[0] = value;
    else if (strcmp(name, "sway_en") == 0)   s_enabled[1] = value;
    else if (strcmp(name, "heave_en") == 0)  s_enabled[2] = value;
    else if (strcmp(name, "roll_en") == 0)   s_enabled[3] = value;
    else if (strcmp(name, "pitch_en") == 0)  s_enabled[4] = value;
    else if (strcmp(name, "yaw_en") == 0)    s_enabled[5] = value;
}
