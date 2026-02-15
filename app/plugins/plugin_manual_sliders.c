/*
 * Manual Sliders Plugin
 * =====================
 * Direct manual control of all 6 platform axes via sliders.
 * Each axis is a tunable parameter (-100% to +100%).
 * process() simply outputs the current slider values.
 */

#include "plugin_api.h"
#include <string.h>

/* ── State ──────────────────────────────────────────────────────────── */

static float s_axes[6] = {0};

/* ── Parameter definitions ──────────────────────────────────────────── */

static const StewartParamDef s_params[] = {
    { "surge", "Surge",  "Forward / Back",   STEWART_PARAM_FLOAT, 0.0f, -100.0f, 100.0f, NULL },
    { "sway",  "Sway",   "Left / Right",     STEWART_PARAM_FLOAT, 0.0f, -100.0f, 100.0f, NULL },
    { "heave", "Heave",  "Up / Down",        STEWART_PARAM_FLOAT, 0.0f, -100.0f, 100.0f, NULL },
    { "roll",  "Roll",   "Lean Left / Right", STEWART_PARAM_FLOAT, 0.0f, -100.0f, 100.0f, NULL },
    { "pitch", "Pitch",  "Tilt Nose Up / Down", STEWART_PARAM_FLOAT, 0.0f, -100.0f, 100.0f, NULL },
    { "yaw",   "Yaw",    "Rotate Left / Right", STEWART_PARAM_FLOAT, 0.0f, -100.0f, 100.0f, NULL },
};

/* ── Plugin info ────────────────────────────────────────────────────── */

static const StewartPluginInfo s_info = {
    STEWART_PLUGIN_API_VERSION,
    "Manual Sliders",                       /* name */
    "Built-in",                             /* author */
    "1.0",                                  /* version */
    "Direct manual control of all 6 platform axes via sliders.",
    0,                                      /* preferred_rate_hz (0 = app default) */
    6,                                      /* axis_count */
    { NULL, NULL, NULL, NULL, NULL, NULL }, /* axis_labels */
    sizeof(s_params) / sizeof(s_params[0]), /* param_count */
    s_params                                /* params */
};

/* ── Entry points ───────────────────────────────────────────────────── */

STEWART_EXPORT const StewartPluginInfo* stewart_plugin_info(void) {
    return &s_info;
}

STEWART_EXPORT int stewart_plugin_init(float sample_rate) {
    (void)sample_rate;
    memset(s_axes, 0, sizeof(s_axes));
    return 0;
}

STEWART_EXPORT int stewart_plugin_process(StewartPluginContext* ctx) {
    for (int i = 0; i < 6; i++)
        ctx->output[i] = s_axes[i];
    return 0;
}

STEWART_EXPORT void stewart_plugin_shutdown(void) {
    memset(s_axes, 0, sizeof(s_axes));
}

STEWART_EXPORT void stewart_plugin_set_param(const char* name, float value) {
    if (!name) return;
    if (strcmp(name, "surge") == 0) s_axes[0] = value;
    else if (strcmp(name, "sway")  == 0) s_axes[1] = value;
    else if (strcmp(name, "heave") == 0) s_axes[2] = value;
    else if (strcmp(name, "roll")  == 0) s_axes[3] = value;
    else if (strcmp(name, "pitch") == 0) s_axes[4] = value;
    else if (strcmp(name, "yaw")   == 0) s_axes[5] = value;
}
