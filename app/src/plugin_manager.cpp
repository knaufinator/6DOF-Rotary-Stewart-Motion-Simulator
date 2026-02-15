#include "plugin_manager.h"
#include <cstring>
#include <cstdio>
#include <filesystem>

namespace fs = std::filesystem;

// ── OS-specific helpers ──────────────────────────────────────────────

#ifdef _WIN32

static PluginHandle plugin_load(const char* path) {
    return LoadLibraryA(path);
}

static void plugin_unload(PluginHandle h) {
    if (h) FreeLibrary(h);
}

static void* plugin_sym(PluginHandle h, const char* name) {
    return (void*)GetProcAddress(h, name);
}

static const char* plugin_error() {
    static char buf[256];
    FormatMessageA(FORMAT_MESSAGE_FROM_SYSTEM, NULL, GetLastError(),
                   0, buf, sizeof(buf), NULL);
    return buf;
}

static const char* PLUGIN_EXT = ".dll";

#else

static PluginHandle plugin_load(const char* path) {
    return dlopen(path, RTLD_NOW | RTLD_LOCAL);
}

static void plugin_unload(PluginHandle h) {
    if (h) dlclose(h);
}

static void* plugin_sym(PluginHandle h, const char* name) {
    return dlsym(h, name);
}

static const char* plugin_error() {
    return dlerror();
}

static const char* PLUGIN_EXT = ".so";

#endif

// ── PluginManager ────────────────────────────────────────────────────

PluginManager::PluginManager()
    : m_active_index(-1)
{
}

PluginManager::~PluginManager() {
    unloadAll();
}

void PluginManager::scanDirectory(const std::string& dir) {
    std::error_code ec;
    if (!fs::is_directory(dir, ec)) {
        // Create the directory if it doesn't exist
        fs::create_directories(dir, ec);
        if (!fs::is_directory(dir, ec)) {
            fprintf(stderr, "[PluginManager] Cannot access plugins directory: %s\n", dir.c_str());
            return;
        }
    }

    for (auto& entry : fs::directory_iterator(dir, ec)) {
        if (!entry.is_regular_file()) continue;
        std::string ext = entry.path().extension().string();
        // Case-insensitive extension check
        for (auto& c : ext) c = (char)tolower(c);
        if (ext != PLUGIN_EXT) continue;

        std::string path = entry.path().string();

        // Skip if already loaded
        bool already = false;
        for (auto& p : m_plugins) {
            if (p.filepath == path) { already = true; break; }
        }
        if (already) continue;

        loadPlugin(path);
    }

    printf("[PluginManager] Scanned '%s': %d plugin(s) found\n", dir.c_str(), (int)m_plugins.size());
}

bool PluginManager::loadPlugin(const std::string& path) {
    PluginInstance p = {};
    p.filepath = path;
    p.filename = fs::path(path).filename().string();
    p.active = false;
    p.valid = false;
    p.fn_info = nullptr;
    p.fn_init = nullptr;
    p.fn_process = nullptr;
    p.fn_shutdown = nullptr;
    p.fn_set_param = nullptr;
    p.fn_get_toolbar = nullptr;
    p.fn_toolbar_action = nullptr;

    // Load the shared library
    p.handle = plugin_load(path.c_str());
    if (!p.handle) {
        fprintf(stderr, "[PluginManager] Failed to load '%s': %s\n",
                p.filename.c_str(), plugin_error());
        m_plugins.push_back(p);  // keep it in list as invalid for UI display
        return false;
    }

    // Probe for required symbols
    p.fn_info     = (StewartPluginInfoFn)plugin_sym(p.handle, STEWART_SYM_INFO);
    p.fn_init     = (StewartPluginInitFn)plugin_sym(p.handle, STEWART_SYM_INIT);
    p.fn_process  = (StewartPluginProcessFn)plugin_sym(p.handle, STEWART_SYM_PROCESS);
    p.fn_shutdown = (StewartPluginShutdownFn)plugin_sym(p.handle, STEWART_SYM_SHUTDOWN);

    // Optional
    p.fn_set_param = (StewartPluginSetParamFn)plugin_sym(p.handle, STEWART_SYM_SET_PARAM);
    p.fn_get_toolbar = (StewartPluginGetToolbarFn)plugin_sym(p.handle, STEWART_SYM_GET_TOOLBAR);
    p.fn_toolbar_action = (StewartPluginToolbarActionFn)plugin_sym(p.handle, STEWART_SYM_TOOLBAR_ACTION);

    if (!p.fn_info || !p.fn_init || !p.fn_process || !p.fn_shutdown) {
        fprintf(stderr, "[PluginManager] '%s' missing required symbols (need: %s, %s, %s, %s)\n",
                p.filename.c_str(),
                STEWART_SYM_INFO, STEWART_SYM_INIT, STEWART_SYM_PROCESS, STEWART_SYM_SHUTDOWN);
        plugin_unload(p.handle);
        p.handle = nullptr;
        m_plugins.push_back(p);
        return false;
    }

    // Get plugin info
    p.info = p.fn_info();
    if (!p.info) {
        fprintf(stderr, "[PluginManager] '%s': stewart_plugin_info() returned NULL\n",
                p.filename.c_str());
        plugin_unload(p.handle);
        p.handle = nullptr;
        m_plugins.push_back(p);
        return false;
    }

    // Validate API version
    if (p.info->api_version != STEWART_PLUGIN_API_VERSION) {
        fprintf(stderr, "[PluginManager] '%s': API version mismatch (plugin=%u, host=%u)\n",
                p.filename.c_str(), p.info->api_version, STEWART_PLUGIN_API_VERSION);
        plugin_unload(p.handle);
        p.handle = nullptr;
        m_plugins.push_back(p);
        return false;
    }

    p.valid = true;

    // Initialize parameter defaults
    if (p.info->param_count > 0 && p.info->params) {
        for (int i = 0; i < p.info->param_count; i++) {
            PluginParamValue pv;
            pv.name = p.info->params[i].name;
            pv.value = p.info->params[i].default_val;
            p.param_values.push_back(pv);
        }
    }

    printf("[PluginManager] Loaded '%s' v%s by %s (%d params)\n",
           p.info->name ? p.info->name : p.filename.c_str(),
           p.info->version ? p.info->version : "?",
           p.info->author ? p.info->author : "?",
           p.info->param_count);

    m_plugins.push_back(p);
    return true;
}

void PluginManager::unloadPlugin(PluginInstance& p) {
    if (p.active && p.fn_shutdown) {
        p.fn_shutdown();
        p.active = false;
    }
    if (p.handle) {
        plugin_unload(p.handle);
        p.handle = nullptr;
    }
    p.valid = false;
}

void PluginManager::unloadAll() {
    for (auto& p : m_plugins) {
        unloadPlugin(p);
    }
    m_plugins.clear();
    m_active_index = -1;
}

bool PluginManager::activatePlugin(int index, float sample_rate) {
    if (index < 0 || index >= (int)m_plugins.size()) return false;

    // Deactivate current
    deactivateActive();

    auto& p = m_plugins[index];
    if (!p.valid || !p.fn_init) return false;

    int result = p.fn_init(sample_rate);
    if (result != 0) {
        fprintf(stderr, "[PluginManager] '%s' init failed (returned %d)\n",
                p.info ? p.info->name : p.filename.c_str(), result);
        return false;
    }

    // Push current parameter values to the plugin
    if (p.fn_set_param) {
        for (auto& pv : p.param_values) {
            p.fn_set_param(pv.name.c_str(), pv.value);
        }
    }

    p.active = true;
    m_active_index = index;

    printf("[PluginManager] Activated '%s'\n",
           p.info ? p.info->name : p.filename.c_str());
    return true;
}

void PluginManager::deactivateActive() {
    if (m_active_index < 0 || m_active_index >= (int)m_plugins.size()) return;

    auto& p = m_plugins[m_active_index];
    if (p.active && p.fn_shutdown) {
        p.fn_shutdown();
        printf("[PluginManager] Deactivated '%s'\n",
               p.info ? p.info->name : p.filename.c_str());
    }
    p.active = false;
    m_active_index = -1;
}

bool PluginManager::processActive(double timestamp, double dt, int frame_number,
                                   float sample_rate, float output[6]) {
    if (m_active_index < 0 || m_active_index >= (int)m_plugins.size()) return false;

    auto& p = m_plugins[m_active_index];
    if (!p.active || !p.fn_process) return false;

    StewartPluginContext ctx = {};
    ctx.timestamp = timestamp;
    ctx.dt = dt;
    ctx.frame_number = frame_number;
    ctx.sample_rate = sample_rate;
    memset(ctx.output, 0, sizeof(ctx.output));
    memset(ctx.raw_input, 0, sizeof(ctx.raw_input));

    int result = p.fn_process(&ctx);
    if (result != 0) return false;

    memcpy(output, ctx.output, sizeof(ctx.output));
    memcpy(p.last_raw_input, ctx.raw_input, sizeof(p.last_raw_input));
    return true;
}

void PluginManager::setParam(const char* name, float value) {
    if (m_active_index < 0 || m_active_index >= (int)m_plugins.size()) return;

    auto& p = m_plugins[m_active_index];

    // Update stored value
    for (auto& pv : p.param_values) {
        if (pv.name == name) {
            pv.value = value;
            break;
        }
    }

    // Push to plugin
    if (p.fn_set_param) {
        p.fn_set_param(name, value);
    }
}

const char* PluginManager::pluginName(int index) const {
    if (index < 0 || index >= (int)m_plugins.size()) return "Unknown";
    auto& p = m_plugins[index];
    if (p.info && p.info->name) return p.info->name;
    return p.filename.c_str();
}
