#pragma once

#include "plugin_api.h"
#include <vector>
#include <string>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
typedef HMODULE PluginHandle;
#else
#include <dlfcn.h>
typedef void* PluginHandle;
#endif

struct PluginParamValue {
    std::string     name;
    float           value;
};

struct PluginInstance {
    std::string                 filepath;       // full path to .dll/.so
    std::string                 filename;       // just the filename
    PluginHandle                handle;         // OS library handle
    const StewartPluginInfo*    info;           // metadata (owned by plugin)
    bool                        active;         // currently selected as input source
    bool                        valid;          // loaded and probed successfully

    // Function pointers
    StewartPluginInfoFn         fn_info;
    StewartPluginInitFn         fn_init;
    StewartPluginProcessFn      fn_process;
    StewartPluginShutdownFn     fn_shutdown;
    StewartPluginSetParamFn     fn_set_param;   // optional
    StewartPluginGetToolbarFn   fn_get_toolbar; // optional — plugin provides toolbar items
    StewartPluginToolbarActionFn fn_toolbar_action; // optional — called on toolbar interaction

    // Current parameter values (persisted)
    std::vector<PluginParamValue> param_values;

    // Last raw (pre-scaling) values from plugin — for profiling/auto-cal
    float                       last_raw_input[6];
};

class PluginManager {
public:
    PluginManager();
    ~PluginManager();

    // Scan a directory for plugin shared libraries and load them
    void scanDirectory(const std::string& dir);

    // Unload all plugins
    void unloadAll();

    // Get list of discovered plugins
    const std::vector<PluginInstance>& plugins() const { return m_plugins; }
    std::vector<PluginInstance>& plugins() { return m_plugins; }
    int pluginCount() const { return (int)m_plugins.size(); }

    // Activate a plugin (call init, mark as active)
    bool activatePlugin(int index, float sample_rate);

    // Deactivate the currently active plugin (call shutdown)
    void deactivateActive();

    // Process the active plugin (fills output[6])
    // Returns true if a plugin produced output
    bool processActive(double timestamp, double dt, int frame_number, float sample_rate, float output[6]);

    // Set a parameter on the active plugin
    void setParam(const char* name, float value);

    // Get the index of the currently active plugin (-1 if none)
    int activeIndex() const { return m_active_index; }

    // Get the name of a plugin by index
    const char* pluginName(int index) const;

private:
    std::vector<PluginInstance>  m_plugins;
    int                          m_active_index;

    bool loadPlugin(const std::string& path);
    void unloadPlugin(PluginInstance& p);
};
