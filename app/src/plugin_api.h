/*
 * Stewart Platform Plugin API
 * ===========================
 *
 * This is the ONLY header a plugin author needs to include.
 * Compile your plugin as a shared library (.dll on Windows, .so on Linux):
 *
 *   Windows:  cl /LD my_plugin.c
 *   Linux:    gcc -shared -fPIC -o my_plugin.so my_plugin.c
 *
 * Drop the resulting file into the plugins/ directory next to the app.
 * The app will discover it on startup and add it as an input source.
 *
 * Plugin output is 6 axes in ±100% range:
 *   [0] Surge  (forward/back)
 *   [1] Sway   (left/right)
 *   [2] Heave  (up/down)
 *   [3] Roll   (lean left/right)
 *   [4] Pitch  (tilt nose up/down)
 *   [5] Yaw    (rotate left/right)
 */

#ifndef STEWART_PLUGIN_API_H
#define STEWART_PLUGIN_API_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Version ──────────────────────────────────────────────────────── */

#define STEWART_PLUGIN_API_VERSION  1

/* ── Export macros ─────────────────────────────────────────────────── */

#ifdef _WIN32
  #define STEWART_EXPORT __declspec(dllexport)
#else
  #define STEWART_EXPORT __attribute__((visibility("default")))
#endif

/* ── Parameter types (for auto-generated UI) ──────────────────────── */

enum StewartParamType {
    STEWART_PARAM_FLOAT = 0,   /* DragFloat slider           */
    STEWART_PARAM_INT   = 1,   /* DragInt slider              */
    STEWART_PARAM_BOOL  = 2,   /* Checkbox                    */
    STEWART_PARAM_ENUM  = 3,   /* Dropdown combo              */
};

/* Parameter declaration — plugin lists these in its info struct.
 * The app reads them and auto-generates a config UI panel.
 * Values are passed back to the plugin via stewart_plugin_set_param(). */
typedef struct {
    const char*         name;           /* unique key, e.g. "port"          */
    const char*         display_name;   /* UI label, e.g. "UDP Port"       */
    const char*         description;    /* tooltip text                     */
    enum StewartParamType type;
    float               default_val;    /* default value (cast as needed)   */
    float               min_val;        /* minimum (float/int)              */
    float               max_val;        /* maximum (float/int)              */
    const char*         enum_labels;    /* for ENUM: "Option A\0Option B\0" (double-null terminated) */
} StewartParamDef;

/* ── Plugin info (returned by stewart_plugin_info) ────────────────── */

typedef struct {
    uint32_t            api_version;    /* must == STEWART_PLUGIN_API_VERSION */
    const char*         name;           /* display name, e.g. "Assetto Corsa SHM" */
    const char*         author;         /* author name                      */
    const char*         version;        /* plugin version string            */
    const char*         description;    /* short description for tooltip    */

    int                 preferred_rate_hz; /* preferred update rate (0 = app default) */

    /* Axis configuration — which axes this plugin provides */
    int                 axis_count;     /* 1-6; axes beyond this are zeroed */
    const char*         axis_labels[6]; /* custom labels (NULL = use defaults) */

    /* Tunable parameters (auto-generates UI) */
    int                 param_count;    /* number of declared parameters    */
    const StewartParamDef* params;      /* array of param_count declarations */
} StewartPluginInfo;

/* ── Process context (passed to stewart_plugin_process) ───────────── */

typedef struct {
    double              timestamp;      /* monotonic time in seconds        */
    double              dt;             /* delta time since last call (sec) */
    int                 frame_number;   /* incrementing frame counter       */
    float               sample_rate;    /* actual sample rate (Hz)          */

    /* Output: plugin fills this with ±100% values */
    float               output[6];
} StewartPluginContext;

/* ── Plugin entry points ──────────────────────────────────────────── *
 *
 * Implement these 4 functions and export them from your shared library.
 * Use the STEWART_EXPORT macro on each one.
 *
 * Lifecycle:
 *   1. App loads .dll/.so, calls stewart_plugin_info() to discover plugin
 *   2. User selects plugin as input source → app calls stewart_plugin_init()
 *   3. Each frame → app calls stewart_plugin_process()
 *   4. User switches away or app exits → stewart_plugin_shutdown()
 *
 * Parameter changes:
 *   When user adjusts a parameter in the UI, app calls
 *   stewart_plugin_set_param() with the parameter name and new value.
 */

/* Return plugin metadata. Called once at load time.
 * The returned pointer must remain valid for the plugin's lifetime. */
typedef const StewartPluginInfo* (*StewartPluginInfoFn)(void);

/* Initialize the plugin. Called when user selects this source.
 * sample_rate = app's current processing rate in Hz. */
typedef int (*StewartPluginInitFn)(float sample_rate);

/* Process one frame. Fill ctx->output[0..5] with ±100% values.
 * Return 0 on success, nonzero on error. */
typedef int (*StewartPluginProcessFn)(StewartPluginContext* ctx);

/* Shutdown. Called when user switches to another source or app exits. */
typedef void (*StewartPluginShutdownFn)(void);

/* Set a parameter value. Called when user changes a UI control.
 * name = parameter key from StewartParamDef.name
 * value = new value (float; cast to int/bool as needed by type) */
typedef void (*StewartPluginSetParamFn)(const char* name, float value);

/* ── Symbol names the app looks for ───────────────────────────────── */

#define STEWART_SYM_INFO       "stewart_plugin_info"
#define STEWART_SYM_INIT       "stewart_plugin_init"
#define STEWART_SYM_PROCESS    "stewart_plugin_process"
#define STEWART_SYM_SHUTDOWN   "stewart_plugin_shutdown"
#define STEWART_SYM_SET_PARAM  "stewart_plugin_set_param"

/* ── Convenience macro for plugin authors ─────────────────────────── *
 *
 * Use STEWART_PLUGIN_DEFINE in your .c file to declare all exports:
 *
 *   static StewartPluginInfo my_info = { ... };
 *
 *   STEWART_EXPORT const StewartPluginInfo* stewart_plugin_info(void) {
 *       return &my_info;
 *   }
 *   STEWART_EXPORT int stewart_plugin_init(float sample_rate) { ... }
 *   STEWART_EXPORT int stewart_plugin_process(StewartPluginContext* ctx) { ... }
 *   STEWART_EXPORT void stewart_plugin_shutdown(void) { ... }
 *   STEWART_EXPORT void stewart_plugin_set_param(const char* name, float value) { ... }
 */

#ifdef __cplusplus
}
#endif

#endif /* STEWART_PLUGIN_API_H */
