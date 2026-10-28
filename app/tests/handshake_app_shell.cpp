// Test-only application lifetime: no TX worker, plugin discovery, persistence
// or global App instance. Protocol methods/configuration are production code.
#include "app.h"
App::App() : console_max(500), frame_time(0), settings_dirty(false) {}
App::~App() = default;
PluginManager::PluginManager() : m_active_index(-1) {}
PluginManager::~PluginManager() = default;
