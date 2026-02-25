#include "ui_panels.h"
#include "serial_port.h"
#include "platform_viz.h"
#include "test_harness_panel.h"
#include "cJSON.h"
#include "imgui.h"
#include "imgui_internal.h"
#include "implot.h"
#include "implot_internal.h"
#include <cstdio>
#include <cmath>
#include <ctime>
#include <cstring>
#include <string>
#include <vector>
#include <algorithm>
#include <filesystem>
#include <mutex>
#include <map>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace fs = std::filesystem;

// ── Colors ──────────────────────────────────────────────────────────

static ImVec4 ColorFromFloat4(const float c[4]) {
    return ImVec4(c[0], c[1], c[2], c[3]);
}

// ── Workspace Save / Load ───────────────────────────────────────────
// A "workspace" is a named snapshot of everything:
//   - Window layout (ImGui .ini)
//   - All entity configs (HIL port, fingerprint, geometry, dynamics, etc.)
//   - App state (input source, console settings, recording rate)
// Stored in workspaces/<name>.ini + workspaces/<name>.json

static const char* WORKSPACES_DIR = "workspaces";

// Pending workspace ini load — set by LoadWorkspace, consumed by PreFrameUI()
static char s_pending_ini_load[512] = "";
static bool s_pending_first_frame = false;

static void EnsureWorkspacesDir() {
    // Migrate old "layouts" directory if present
    std::error_code ec;
    if (fs::exists("layouts") && !fs::exists(WORKSPACES_DIR)) {
        fs::rename("layouts", WORKSPACES_DIR, ec);
    }
    fs::create_directories(WORKSPACES_DIR, ec);
}

static std::vector<std::string> EnumerateWorkspaces() {
    std::vector<std::string> names;
    EnsureWorkspacesDir();
    std::error_code ec;
    for (auto& entry : fs::directory_iterator(WORKSPACES_DIR, ec)) {
        if (entry.is_regular_file() && entry.path().extension() == ".ini") {
            names.push_back(entry.path().stem().string());
        }
    }
    std::sort(names.begin(), names.end());
    return names;
}

static bool SaveWorkspace(const char* name) {
    EnsureWorkspacesDir();
    // Flush current ImGui window layout to disk
    ImGui::SaveIniSettingsToDisk(ImGui::GetIO().IniFilename);
    std::error_code ec;
    fs::path ini_src(ImGui::GetIO().IniFilename);
    fs::path ini_dst = fs::path(WORKSPACES_DIR) / (std::string(name) + ".ini");
    fs::copy_file(ini_src, ini_dst, fs::copy_options::overwrite_existing, ec);

    // Save all entity configs + app state alongside
    g_app.saveSettings();
    fs::path json_src("stewart_settings.json");
    fs::path json_dst = fs::path(WORKSPACES_DIR) / (std::string(name) + ".json");
    fs::copy_file(json_src, json_dst, fs::copy_options::overwrite_existing, ec);

    return !ec;
}

static bool LoadWorkspace(const char* name) {
    fs::path ini_src = fs::path(WORKSPACES_DIR) / (std::string(name) + ".ini");
    if (!fs::exists(ini_src)) return false;

    // Restore entity configs + app state immediately (no frame dependency)
    std::error_code ec;
    fs::path json_src = fs::path(WORKSPACES_DIR) / (std::string(name) + ".json");
    if (fs::exists(json_src)) {
        fs::copy_file(json_src, fs::path("stewart_settings.json"), fs::copy_options::overwrite_existing, ec);
        g_app.loadSettings();
    }

    // Copy ini to active file immediately so it persists on next launch
    fs::copy_file(ini_src, fs::path(ImGui::GetIO().IniFilename), fs::copy_options::overwrite_existing, ec);

    // Defer the actual ImGui ini load to BEFORE the next NewFrame() call.
    // Loading mid-frame corrupts docking state because the dockspace has
    // already been rebuilt for the current frame.
    snprintf(s_pending_ini_load, sizeof(s_pending_ini_load), "%s", ini_src.string().c_str());
    s_pending_first_frame = true;  // force dockspace re-evaluation after load
    return true;
}

static bool DeleteWorkspace(const char* name) {
    std::error_code ec;
    fs::remove(fs::path(WORKSPACES_DIR) / (std::string(name) + ".json"), ec);
    return fs::remove(fs::path(WORKSPACES_DIR) / (std::string(name) + ".ini"), ec);
}

// Workspace popup state
static bool s_show_save_popup = false;
static char s_layout_name_buf[128] = "";
static bool s_reset_layout = false;

// Panel visibility (toggled from View menu, X button on windows)
static bool s_show_input       = true;
static bool s_input_strip_expanded = true;  // collapsible input strip below toolbar
static float s_input_strip_h   = 0.0f;     // final height used for dockspace offset
static float s_input_strip_user_h = 160.0f; // user-adjustable height (draggable)
static float s_input_strip_content_h = 0.0f; // measured content height from last frame
static int   s_input_strip_autofit = 2;      // frames remaining for auto-fit (0 = done)
static int   s_input_strip_last_source = -1; // track source changes for auto-fit
static int   s_input_strip_last_plugin = -1; // track plugin changes for auto-fit
// Panel visibility — backed by g_app so they are persisted per workspace via saveSettings/loadSettings
#define s_show_console      g_app.show_console
#define s_show_data_streams g_app.show_data_streams
#define s_show_dynamics     g_app.show_dynamics
// s_selected_dynamics_id is stored in g_app.selected_dynamics_id (persisted across restarts)
#define s_selected_dynamics_id g_app.selected_dynamics_id
static int  s_dyn_preset_idx = -1;       // currently selected dynamics preset index

// Dynamics staging buffer (file-scope so Copy/Paste can access it)
struct DynStaging {
    MotionCueingConfig mca;
    InputFilterConfig  input_filter;
    float intensity;
    float axis_gain[6];
    bool  axis_invert[6];
    float occupant[3];
    bool  initialized;
};
static std::map<int, DynStaging> s_dyn_staging;

// ── Toolbar / Menu Bar ──────────────────────────────────────────────

static void DrawMainMenuBar() {
    if (ImGui::BeginMainMenuBar()) {
        if (ImGui::BeginMenu("File")) {
            // ── Quick save ──
            if (ImGui::MenuItem("Save", "Ctrl+S")) {
                g_app.saveSettings();
                ImGui::SaveIniSettingsToDisk(ImGui::GetIO().IniFilename);
                g_app.log(-1, "workspace", "Saved current session");
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Save current entity configs and window layout to disk.");

            if (ImGui::MenuItem("Save Workspace As...")) {
                s_show_save_popup = true;
                s_layout_name_buf[0] = '\0';
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Save a named snapshot of all entity configs,\nHIL settings, dynamics presets, and window layout.");

            ImGui::Separator();

            // ── Workspace list ──
            auto workspaces = EnumerateWorkspaces();
            if (ImGui::BeginMenu("Open Workspace")) {
                if (workspaces.empty()) {
                    ImGui::TextDisabled("(no saved workspaces)");
                } else {
                    for (auto& ws : workspaces) {
                        if (ImGui::MenuItem(ws.c_str())) {
                            LoadWorkspace(ws.c_str());
                            g_app.log(-1, "workspace", "Loaded workspace: %s", ws.c_str());
                        }
                    }
                }
                ImGui::EndMenu();
            }

            if (ImGui::BeginMenu("Delete Workspace")) {
                if (workspaces.empty()) {
                    ImGui::TextDisabled("(no saved workspaces)");
                } else {
                    for (auto& ws : workspaces) {
                        if (ImGui::MenuItem(ws.c_str())) {
                            DeleteWorkspace(ws.c_str());
                            g_app.log(-1, "workspace", "Deleted workspace: %s", ws.c_str());
                        }
                    }
                }
                ImGui::EndMenu();
            }

            ImGui::Separator();
            if (ImGui::MenuItem("Reset Window Layout")) {
                s_reset_layout = true;
                g_app.log(-1, "workspace", "Window layout reset to default");
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Reset all window positions and sizes to defaults.\nEntity configs are NOT changed.");

            ImGui::Separator();
            if (ImGui::MenuItem("Exit")) { g_app.running = false; }
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Entity")) {
            if (ImGui::MenuItem("Add SIL Entity")) {
                char name[64];
                snprintf(name, sizeof(name), "SIL #%d", g_app.next_entity_id);
                g_app.addEntity(name, EntityType::SIL);
            }
            if (ImGui::MenuItem("Add HIL Entity")) {
                char name[64];
                snprintf(name, sizeof(name), "HIL #%d", g_app.next_entity_id);
                g_app.addEntity(name, EntityType::HIL);
            }
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("View")) {
            ImGui::MenuItem("Input",        nullptr, &s_show_input);
            ImGui::MenuItem("Console",      nullptr, &s_show_console);
            ImGui::MenuItem("Data Streams", nullptr, &s_show_data_streams);
            ImGui::MenuItem("Dynamics",     nullptr, &s_show_dynamics);
            ImGui::MenuItem("Test Harness", nullptr, &GetTestHarnessState().show_panel);
            ImGui::Separator();
            // Per-entity windows
            for (auto& e : g_app.entities) {
                char label[96];
                snprintf(label, sizeof(label), "%s [%s]", e.name, e.type == EntityType::SIL ? "SIL" : "HIL");
                if (ImGui::BeginMenu(label)) {
                    ImGui::MenuItem("Card",     nullptr, &e.show_card);
                    ImGui::MenuItem("Settings", nullptr, &e.show_settings);
                    ImGui::MenuItem("Platform", nullptr, &e.show_platform);
                    ImGui::MenuItem("Console",  nullptr, &e.show_console);
                    if (ImGui::MenuItem("Select in Dynamics")) {
                        s_show_dynamics = true;
                        s_selected_dynamics_id = e.id;
                    }
                    ImGui::EndMenu();
                }
            }
            ImGui::Separator();
            if (ImGui::MenuItem("Show All Panels")) {
                s_show_input = s_show_console = s_show_data_streams = s_show_dynamics = true;
                GetTestHarnessState().show_panel = true;
                for (auto& e : g_app.entities) {
                    e.show_card = true;
                }
            }
            ImGui::EndMenu();
        }

        // Right-aligned status
        float fps = g_app.fps;
        char fps_text[64];
        snprintf(fps_text, sizeof(fps_text), "%.1f fps | %d entities", fps, (int)g_app.entities.size());
        float text_w = ImGui::CalcTextSize(fps_text).x;
        ImGui::SameLine(ImGui::GetWindowWidth() - text_w - 16);
        ImGui::TextColored(ImVec4(0.6f, 0.6f, 0.6f, 1.0f), "%s", fps_text);

        ImGui::EndMainMenuBar();
    }

    // ── Save Workspace Popup ──
    if (s_show_save_popup) {
        ImGui::OpenPopup("Save Workspace");
        s_show_save_popup = false;
    }
    if (ImGui::BeginPopupModal("Save Workspace", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::TextColored(ImVec4(0.7f, 0.85f, 1.0f, 1.0f), "Save Workspace");
        ImGui::TextDisabled("Saves all entity configs, HIL settings, dynamics presets,");
        ImGui::TextDisabled("fingerprints, and window layout as a named snapshot.");
        ImGui::Spacing();
        ImGui::Text("Workspace name:");
        ImGui::SetNextItemWidth(320);
        bool enter_pressed = ImGui::InputText("##ws_name", s_layout_name_buf, sizeof(s_layout_name_buf),
                                               ImGuiInputTextFlags_EnterReturnsTrue);
        if (ImGui::IsWindowAppearing()) ImGui::SetKeyboardFocusHere(-1);

        bool valid = s_layout_name_buf[0] != '\0';
        ImGui::Spacing();

        ImGui::PushStyleColor(ImGuiCol_Button, valid ? ImVec4(0.15f, 0.5f, 0.8f, 1.0f) : ImVec4(0.3f, 0.3f, 0.3f, 1.0f));
        if (!valid) ImGui::BeginDisabled();
        bool do_save = ImGui::Button("Save Workspace", ImVec2(160, 0)) || (enter_pressed && valid);
        if (!valid) ImGui::EndDisabled();
        ImGui::PopStyleColor();

        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(100, 0))) {
            ImGui::CloseCurrentPopup();
        }

        if (do_save) {
            if (SaveWorkspace(s_layout_name_buf)) {
                g_app.log(-1, "workspace", "Saved workspace: %s", s_layout_name_buf);
            } else {
                g_app.log(-1, "workspace", "Failed to save workspace: %s", s_layout_name_buf);
            }
            ImGui::CloseCurrentPopup();
        }

        ImGui::EndPopup();
    }
}

// ── Input Source Colors (used by toolbar + input panel) ───────────────

static const ImVec4 g_capture_color     = {0.95f, 0.75f, 0.20f, 1.0f};
static const ImVec4 g_capture_color_dim = {0.38f, 0.30f, 0.08f, 1.0f};
static const ImVec4 g_plugin_color      = {0.40f, 0.80f, 0.95f, 1.0f};
static const ImVec4 g_plugin_color_dim  = {0.16f, 0.32f, 0.38f, 1.0f};

// ── Toolbar (ribbon-style, visual mockup) ────────────────────────────

static void ToolbarSeparator() {
    ImGui::SameLine(0, 4);
    float y0 = ImGui::GetCursorScreenPos().y;
    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddLine(ImVec2(ImGui::GetCursorScreenPos().x, y0 + 2),
                ImVec2(ImGui::GetCursorScreenPos().x, y0 + 68),
                IM_COL32(80, 80, 80, 180), 1.0f);
    ImGui::SameLine(0, 8);
}

static void ToolbarGroupLabel(const char* label) {
    ImVec2 pos = ImGui::GetCursorScreenPos();
    ImGui::GetWindowDrawList()->AddText(ImVec2(pos.x, pos.y + 58),
        IM_COL32(120, 120, 120, 200), label);
}

static void DrawToolbar() {
    ImGuiViewport* vp = ImGui::GetMainViewport();
    float menu_h = ImGui::GetFrameHeight();  // main menu bar height
    float toolbar_h = 82.0f;

    ImGui::SetNextWindowPos(ImVec2(vp->WorkPos.x, vp->WorkPos.y));
    ImGui::SetNextWindowSize(ImVec2(vp->WorkSize.x, toolbar_h));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(8, 4));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.14f, 0.14f, 0.16f, 1.0f));

    ImGuiWindowFlags tb_flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse |
        ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoSavedSettings;

    if (ImGui::Begin("##Toolbar", nullptr, tb_flags)) {

        // ── SOURCE (plugin/capture combo) ───────────────────────
        ToolbarGroupLabel("Source");
        {
            // Build preview text: active plugin name or "Capture Playback"
            char preview[128];
            const ImVec4* accent = &g_plugin_color;
            if (g_app.input_source == InputSource::CapturePlayback) {
                snprintf(preview, sizeof(preview), "Capture Playback");
                accent = &g_capture_color;
            } else if (g_app.active_plugin_idx >= 0 && g_app.active_plugin_idx < g_app.plugin_mgr.pluginCount()) {
                snprintf(preview, sizeof(preview), "%s", g_app.plugin_mgr.pluginName(g_app.active_plugin_idx));
            } else {
                snprintf(preview, sizeof(preview), "No source selected");
            }

            // Style the combo with accent color
            ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(accent->x * 0.2f, accent->y * 0.2f, accent->z * 0.2f, 0.6f));
            ImGui::PushStyleColor(ImGuiCol_FrameBgHovered, ImVec4(accent->x * 0.3f, accent->y * 0.3f, accent->z * 0.3f, 0.7f));
            ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(accent->x * 0.6f, accent->y * 0.6f, accent->z * 0.6f, 0.9f));
            ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 1.0f);
            ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 4.0f);
            ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(6, 4));

            ImGui::PushItemWidth(220);
            if (ImGui::BeginCombo("##tb_source", preview, ImGuiComboFlags_HeightLarge)) {

                // Helper: force-stop motion and deactivate current source.
                // Switching input always requires an explicit START.
                auto forceStopAndSwitch = [&]() {
                    g_app.motion_started = false;
                    g_app.source_switch_active = false;
                    g_app.start_ramp_active = false;
                    if (g_app.capture_playing) g_app.stopCapturePlayback();
                    if (g_app.plugin_mgr.activeIndex() >= 0)
                        g_app.plugin_mgr.deactivateActive();
                    for (auto& ent : g_app.entities)
                        memset(ent.state.input_pct, 0, sizeof(ent.state.input_pct));
                    {
                        std::lock_guard<std::mutex> lock(g_app.input_mutex);
                        memset(g_app.shared_input, 0, sizeof(g_app.shared_input));
                    }
                };

                // ── All plugins ──
                for (int pi = 0; pi < g_app.plugin_mgr.pluginCount(); pi++) {
                    auto& plug = g_app.plugin_mgr.plugins()[pi];
                    if (!plug.valid) continue;

                    bool is_active = (g_app.input_source == InputSource::Plugin &&
                                      g_app.active_plugin_idx == pi);
                    const char* pname = plug.info->name ? plug.info->name : plug.filename.c_str();

                    ImGui::PushID(pi);
                    if (ImGui::Selectable(pname, is_active)) {
                        if (!is_active) {
                            forceStopAndSwitch();
                            g_app.input_source = InputSource::Plugin;
                            g_app.active_plugin_idx = pi;
                            g_app.log(-1, "input", "Source changed to '%s' — press START to begin", pname);
                        }
                    }
                    ImGui::PopID();
                }

                // ── Capture Playback ──
                ImGui::Separator();
                {
                    bool is_cap = (g_app.input_source == InputSource::CapturePlayback);
                    if (ImGui::Selectable("Capture Playback", is_cap)) {
                        if (!is_cap) {
                            forceStopAndSwitch();
                            g_app.input_source = InputSource::CapturePlayback;
                            g_app.log(-1, "input", "Source changed to Capture Playback — press START to begin");
                        }
                    }
                }

                ImGui::EndCombo();
            }
            ImGui::PopItemWidth();

            ImGui::PopStyleVar(3);
            ImGui::PopStyleColor(3);

            // Status indicator: STOPPED / LIVE / IDLE
            ImGui::SameLine();
            if (!g_app.motion_started) {
                ImGui::TextColored(ImVec4(0.7f, 0.35f, 0.2f, 1.0f), "STOPPED");
            } else {
                bool source_active = false;
                if (g_app.input_source == InputSource::Plugin)
                    source_active = g_app.plugin_mgr.activeIndex() >= 0;
                else if (g_app.input_source == InputSource::CapturePlayback)
                    source_active = g_app.capture_playing;
                if (source_active)
                    ImGui::TextColored(ImVec4(0.2f, 0.9f, 0.3f, 1.0f), "LIVE");
                else
                    ImGui::TextColored(ImVec4(0.6f, 0.6f, 0.3f, 1.0f), "READY");
            }

            // Collapse/expand toggle for input strip
            ImGui::SameLine(0, 6);
            const char* chevron = s_input_strip_expanded ? "^" : "v";
            if (ImGui::SmallButton(chevron)) {
                s_input_strip_expanded = !s_input_strip_expanded;
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(s_input_strip_expanded ? "Collapse input strip" : "Expand input strip");
        }

        ToolbarSeparator();

        // ── MOTION (START, STOP, E-STOP) ────────────────────────
        ToolbarGroupLabel("Motion");
        {
            bool running = g_app.motion_started;

            // START button (green accent when running)
            if (running) {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.15f, 0.50f, 0.15f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.18f, 0.58f, 0.18f, 1.0f));
            } else {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.45f, 0.20f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.25f, 0.55f, 0.25f, 1.0f));
            }
            if (ImGui::Button("START", ImVec2(52, 26))) {
                if (!running) {
                    g_app.motion_started = true;
                    // Activate plugin if Plugin source is selected
                    if (g_app.input_source == InputSource::Plugin && g_app.active_plugin_idx >= 0 &&
                        g_app.plugin_mgr.activeIndex() < 0) {
                        float sr = (g_app.fps > 1.0) ? (float)g_app.fps : 60.0f;
                        g_app.plugin_mgr.activatePlugin(g_app.active_plugin_idx, sr);
                    }
                }
            }
            ImGui::PopStyleColor(2);

            ImGui::SameLine(0, 2);

            // STOP button
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.35f, 0.35f, 0.35f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.45f, 0.45f, 0.45f, 1.0f));
            if (ImGui::Button("STOP", ImVec2(46, 26))) {
                if (running) {
                    g_app.motion_started = false;
                    if (g_app.plugin_mgr.activeIndex() >= 0)
                        g_app.plugin_mgr.deactivateActive();
                    // Zero all entities
                    for (auto& e : g_app.entities)
                        memset(e.state.input_pct, 0, sizeof(e.state.input_pct));
                    {
                        std::lock_guard<std::mutex> lock(g_app.input_mutex);
                        memset(g_app.shared_input, 0, sizeof(g_app.shared_input));
                    }
                }
            }
            ImGui::PopStyleColor(2);

            ImGui::SameLine(0, 6);

            // E-STOP (bright red, always prominent)
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.75f, 0.1f, 0.1f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.9f, 0.15f, 0.15f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(1.0f, 0.0f, 0.0f, 1.0f));
            if (ImGui::Button("E-STOP", ImVec2(56, 26))) {
                g_app.motion_started = false;
                if (g_app.capture_playing) g_app.stopCapturePlayback();
                if (g_app.plugin_mgr.activeIndex() >= 0)
                    g_app.plugin_mgr.deactivateActive();
                for (auto& e : g_app.entities)
                    memset(e.state.input_pct, 0, sizeof(e.state.input_pct));
                {
                    std::lock_guard<std::mutex> lock(g_app.input_mutex);
                    memset(g_app.shared_input, 0, sizeof(g_app.shared_input));
                }
            }
            ImGui::PopStyleColor(3);
        }

        ToolbarSeparator();

        // ── RECORDING ───────────────────────────────────────────
        ToolbarGroupLabel("Recording");
        {
            bool is_recording = (g_app.recording.mode == RecordMode::Recording);
            bool is_playing   = g_app.capture_playing;
            bool has_captures = !g_app.saved_recordings.empty();

            ImGui::BeginGroup();

            // ── Row 1: Large REC + PLAY buttons ──
            {
                bool can_rec = g_app.motion_started || is_recording;
                if (is_recording) {
                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.8f, 0.15f, 0.15f, 1.0f));
                    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.9f, 0.2f, 0.2f, 1.0f));
                } else {
                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.45f, 0.18f, 0.18f, 1.0f));
                    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.55f, 0.22f, 0.22f, 1.0f));
                }
                if (!can_rec) ImGui::BeginDisabled();
                if (ImGui::Button(is_recording ? "STOP REC" : "REC", ImVec2(90, 32))) {
                    if (is_recording) {
                        g_app.stopRecording();
                    } else {
                        g_app.startRecording();
                    }
                }
                if (!can_rec) ImGui::EndDisabled();
                if (!can_rec && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
                    ImGui::SetTooltip("Start motion first to record input data.");
                ImGui::PopStyleColor(2);

                ImGui::SameLine(0, 3);

                if (is_playing) {
                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.15f, 0.55f, 0.15f, 1.0f));
                    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.2f, 0.65f, 0.2f, 1.0f));
                } else {
                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.35f, 0.35f, 0.35f, 1.0f));
                    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.45f, 0.45f, 0.45f, 1.0f));
                }
                bool can_play = has_captures && g_app.motion_started;
                if (!can_play && !is_playing) ImGui::BeginDisabled();
                if (ImGui::Button(is_playing ? "STOP##rec_play" : "PLAY##rec_play", ImVec2(60, 32))) {
                    if (is_playing) {
                        g_app.stopCapturePlayback();
                    } else if (can_play) {
                        int idx = g_app.capture_playback_idx >= 0 ? g_app.capture_playback_idx : 0;
                        g_app.startCapturePlayback(idx);
                    }
                }
                if (!can_play && !is_playing) ImGui::EndDisabled();
                if (!can_play && !is_playing && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
                    ImGui::SetTooltip("Start motion first to play back captures.");
                ImGui::PopStyleColor(2);
            }

            // ── Row 2: Capture combo + Loop + Speed ──
            {
                // Capture selector combo
                ImGui::PushItemWidth(156);
                const char* preview = (g_app.capture_playback_idx >= 0 &&
                    g_app.capture_playback_idx < (int)g_app.saved_recordings.size())
                    ? g_app.saved_recordings[g_app.capture_playback_idx].name : "(none)";
                if (!has_captures || is_playing) ImGui::BeginDisabled();
                if (ImGui::BeginCombo("##cap_sel", preview, ImGuiComboFlags_HeightLarge)) {
                    for (int ci = 0; ci < (int)g_app.saved_recordings.size(); ci++) {
                        auto& sr = g_app.saved_recordings[ci];
                        bool sel = (g_app.capture_playback_idx == ci);
                        ImGui::PushID(ci);
                        char item_buf[128];
                        snprintf(item_buf, sizeof(item_buf), "%s\n  %.1fs | %d samp | %.0f Hz",
                                 sr.name, sr.duration(), (int)sr.samples.size(), sr.sample_rate_hz);
                        if (ImGui::Selectable(item_buf, sel, 0, ImVec2(0, ImGui::GetTextLineHeight() * 2.4f))) {
                            g_app.capture_playback_idx = ci;
                        }
                        if (sel) ImGui::SetItemDefaultFocus();
                        ImGui::PopID();
                    }
                    ImGui::EndCombo();
                }
                if (!has_captures || is_playing) ImGui::EndDisabled();
                ImGui::PopItemWidth();

                ImGui::SameLine(0, 4);
                ImGui::Checkbox("Loop", &g_app.capture_loop);

                ImGui::SameLine(0, 4);
                ImGui::PushItemWidth(50);
                int spd_pct = (int)(g_app.capture_speed * 100.0f + 0.5f);
                if (ImGui::DragInt("##spd_tb", &spd_pct, 1, 10, 200, "%d%%")) {
                    if (spd_pct < 10) spd_pct = 10;
                    if (spd_pct > 200) spd_pct = 200;
                    g_app.capture_speed = spd_pct / 100.0f;
                }
                ImGui::PopItemWidth();
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Playback speed (10%% - 200%%)");

                ImGui::SameLine(0, 8);
                static const int rate_opts[] = {50, 100, 200, 500, 1000};
                static const char* rate_labels[] = {"50", "100", "200", "500", "1000"};
                int rate_idx = 2;
                for (int i = 0; i < 5; i++) { if (rate_opts[i] == g_app.record_rate_hz) rate_idx = i; }
                if (is_recording) ImGui::BeginDisabled();
                ImGui::PushItemWidth(52);
                if (ImGui::Combo("##rec_rate_tb", &rate_idx, rate_labels, 5)) {
                    g_app.record_rate_hz = rate_opts[rate_idx];
                }
                ImGui::PopItemWidth();
                if (is_recording) ImGui::EndDisabled();
                ImGui::SameLine(0, 2);
                ImGui::TextDisabled("Hz");
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Recording sample rate");
            }

            ImGui::EndGroup();
        }

        ToolbarSeparator();

        // ── PLUGIN TOOLBAR (active plugin's custom items) ──────
        {
            int pidx = g_app.active_plugin_idx;
            if (pidx >= 0 && pidx < g_app.plugin_mgr.pluginCount()) {
                auto& plug = g_app.plugin_mgr.plugins()[pidx];
                if (plug.active && plug.fn_get_toolbar) {
                    int tb_count = 0;
                    StewartToolbarItem* items = plug.fn_get_toolbar(&tb_count);
                    if (items && tb_count > 0) {
                        const char* pname = plug.info ? plug.info->name : plug.filename.c_str();
                        ToolbarGroupLabel(pname);
                        for (int ti = 0; ti < tb_count; ti++) {
                            StewartToolbarItem& item = items[ti];
                            if (ti > 0 && item.type != STEWART_TOOLBAR_SEPARATOR)
                                ImGui::SameLine(0, 4);
                            ImGui::PushID(ti);
                            switch (item.type) {
                                case STEWART_TOOLBAR_BUTTON: {
                                    float w = item.width > 0 ? item.width : 0;
                                    if (ImGui::Button(item.label ? item.label : "?", w > 0 ? ImVec2(w, 26) : ImVec2(0, 26))) {
                                        if (plug.fn_toolbar_action && item.id)
                                            plug.fn_toolbar_action(item.id, 1);
                                    }
                                } break;
                                case STEWART_TOOLBAR_TOGGLE: {
                                    bool toggled = (item.current_value != 0);
                                    if (toggled) {
                                        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.55f, 0.3f, 1.0f));
                                        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.25f, 0.65f, 0.35f, 1.0f));
                                    }
                                    if (ImGui::Button(item.label ? item.label : "?", ImVec2(item.width > 0 ? item.width : 0, 26))) {
                                        int nv = toggled ? 0 : 1;
                                        if (plug.fn_toolbar_action && item.id)
                                            plug.fn_toolbar_action(item.id, nv);
                                    }
                                    if (toggled) ImGui::PopStyleColor(2);
                                } break;
                                case STEWART_TOOLBAR_COMBO: {
                                    ImGui::PushItemWidth(item.width > 0 ? item.width : 120);
                                    // Find current label from double-null options
                                    const char* cur_label = "?";
                                    if (item.options) {
                                        const char* p = item.options;
                                        for (int oi = 0; oi < item.current_value && *p; oi++) {
                                            p += strlen(p) + 1;
                                        }
                                        if (*p) cur_label = p;
                                    }
                                    char combo_id[32];
                                    snprintf(combo_id, sizeof(combo_id), "##tb_%s", item.id ? item.id : "?");
                                    if (ImGui::BeginCombo(combo_id, cur_label)) {
                                        const char* opt = item.options;
                                        for (int oi = 0; oi < item.option_count && opt && *opt; oi++) {
                                            bool osel = (oi == item.current_value);
                                            if (ImGui::Selectable(opt, osel)) {
                                                if (plug.fn_toolbar_action && item.id)
                                                    plug.fn_toolbar_action(item.id, oi);
                                            }
                                            if (osel) ImGui::SetItemDefaultFocus();
                                            opt += strlen(opt) + 1;
                                        }
                                        ImGui::EndCombo();
                                    }
                                    ImGui::PopItemWidth();
                                } break;
                                case STEWART_TOOLBAR_SEPARATOR: {
                                    ToolbarSeparator();
                                } break;
                            }
                            ImGui::PopID();
                        }
                        ToolbarSeparator();
                    }
                }
            }
        }

        // ── PLATFORM ────────────────────────────────────────────
        ToolbarGroupLabel("Platform");
        {
            int n_ent = (int)g_app.entities.size();
            int n_hil = 0, n_sil = 0;
            for (auto& e : g_app.entities) {
                if (e.type == EntityType::HIL) n_hil++;
                else n_sil++;
            }

            if (n_sil > 0) {
                ImGui::TextColored(ImVec4(0.4f, 0.7f, 1.0f, 1.0f), "SIL:%d", n_sil);
                ImGui::SameLine();
            }
            if (n_hil > 0) {
                ImGui::TextColored(ImVec4(0.4f, 1.0f, 0.5f, 1.0f), "HIL:%d", n_hil);
                ImGui::SameLine();
            }
            if (n_ent == 0) {
                ImGui::TextDisabled("No entities");
                ImGui::SameLine();
            }

            ImGui::Button("+SIL", ImVec2(38, 26));
            ImGui::SameLine();
            ImGui::Button("+HIL", ImVec2(38, 26));
        }

        ToolbarSeparator();

        // ── STATUS (right-aligned) ──────────────────────────────
        {
            char status_buf[128];
            float sr = 0;
            if (!g_app.entities.empty())
                sr = g_app.entities[0].config.mca.sample_rate;

            snprintf(status_buf, sizeof(status_buf), "%.0f fps  |  MCA %.0f Hz  |  %d ent",
                     g_app.fps, sr, (int)g_app.entities.size());
            float status_w = ImGui::CalcTextSize(status_buf).x;
            float avail = ImGui::GetContentRegionAvail().x;
            if (avail > status_w + 16) {
                ImGui::SameLine(ImGui::GetWindowWidth() - status_w - 16);
                ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.5f, 0.8f), "%s", status_buf);
            }
        }
    }
    ImGui::End();
    ImGui::PopStyleColor();
    ImGui::PopStyleVar(3);
}

// Forward declarations for plugin param helpers (defined later)
struct PluginInstance;
static void DrawPluginParamsLayout(PluginInstance& plug);
static float* PluginParamPtr(PluginInstance& plug, int pidx);

// Forward declarations for helpers used in capture card list
static void GetCaptureSourceBadge(const char* source, const char** icon, const char** label, ImVec4* color);
static void FormatDateTime(double unix_time, char* buf, int buf_sz);
static void DrawBadge(ImDrawList* dl, ImVec2 pos, const char* text, ImVec4 color, float h);

// ── Input Strip (fixed horizontal band below toolbar) ────────────────

static void DrawInputStrip() {
    ImGuiViewport* vp = ImGui::GetMainViewport();
    float toolbar_h = 82.0f;
    float strip_y = vp->WorkPos.y + toolbar_h;
    float splitter_h = 8.0f;  // bottom resize grip height
    float hdr_h = 22.0f;      // header bar height

    // ── Detect source/plugin change → auto-expand + auto-fit height ──
    int cur_src = (int)g_app.input_source;
    int cur_plug = g_app.active_plugin_idx;
    if (cur_src != s_input_strip_last_source || cur_plug != s_input_strip_last_plugin) {
        s_input_strip_last_source = cur_src;
        s_input_strip_last_plugin = cur_plug;
        s_input_strip_expanded = true;
        s_input_strip_autofit = 3;  // wait 3 frames for content to render and measure
    }

    // Apply auto-fit: count down frames, then use measured content height
    if (s_input_strip_autofit > 0) {
        s_input_strip_autofit--;
        if (s_input_strip_autofit == 0 && s_input_strip_content_h > 0) {
            float max_strip = vp->WorkSize.y * 0.6f;
            float fit_h = s_input_strip_content_h + hdr_h + splitter_h + 12.0f;
            if (fit_h < 60.0f) fit_h = 60.0f;
            if (fit_h > max_strip) fit_h = max_strip;
            s_input_strip_user_h = fit_h;
        }
    }

    // ── Collapsed state: just draw a thin collapse bar ──
    if (!s_input_strip_expanded) {
        float bar_h = 22.0f;
        ImGui::SetNextWindowPos(ImVec2(vp->WorkPos.x, strip_y));
        ImGui::SetNextWindowSize(ImVec2(vp->WorkSize.x, bar_h));

        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
        ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
        ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.10f, 0.10f, 0.12f, 1.0f));

        ImGuiWindowFlags bar_flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
            ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoDocking |
            ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoScrollbar;

        if (ImGui::Begin("##InputStripBar", nullptr, bar_flags)) {
            float w = ImGui::GetContentRegionAvail().x;
            // Clickable expand bar — full width
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.16f, 0.16f, 0.20f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.22f, 0.22f, 0.28f, 1.0f));
            if (ImGui::Button("##expand_strip", ImVec2(w, bar_h))) {
                s_input_strip_expanded = true;
            }
            ImGui::PopStyleColor(2);

            // Draw label on top of the button
            ImDrawList* dl = ImGui::GetWindowDrawList();
            ImVec2 bp = ImGui::GetItemRectMin();
            float cx = bp.x + w * 0.5f;
            float cy = bp.y + bar_h * 0.5f;

            // Source name
            const char* src_name = "Input";
            if (g_app.input_source == InputSource::Plugin && g_app.active_plugin_idx >= 0 &&
                g_app.active_plugin_idx < g_app.plugin_mgr.pluginCount())
                src_name = g_app.plugin_mgr.pluginName(g_app.active_plugin_idx);
            else if (g_app.input_source == InputSource::CapturePlayback)
                src_name = "Capture Playback";

            char bar_label[128];
            snprintf(bar_label, sizeof(bar_label), "  Show Input: %s  ", src_name);
            ImVec2 ts = ImGui::CalcTextSize(bar_label);
            float tx = cx - ts.x * 0.5f;
            float ty = cy - ts.y * 0.5f;
            dl->AddText(ImVec2(tx, ty), IM_COL32(160, 170, 190, 220), bar_label);
            // Draw down-arrow triangles on each side of the text
            float tri_sz = 3.5f;
            dl->AddTriangleFilled(
                ImVec2(tx - 10, cy - tri_sz), ImVec2(tx - 10 - tri_sz, cy + tri_sz), ImVec2(tx - 10 + tri_sz, cy + tri_sz),
                IM_COL32(160, 170, 190, 200));
            float rx = tx + ts.x + 10;
            dl->AddTriangleFilled(
                ImVec2(rx, cy - tri_sz), ImVec2(rx - tri_sz, cy + tri_sz), ImVec2(rx + tri_sz, cy + tri_sz),
                IM_COL32(160, 170, 190, 200));

            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Click to expand input strip");
        }
        ImGui::End();
        ImGui::PopStyleColor();
        ImGui::PopStyleVar(3);

        s_input_strip_h = bar_h;
        return;
    }

    // ── Expanded state ──
    // Clamp user height
    float max_strip = vp->WorkSize.y * 0.6f;
    if (s_input_strip_user_h < 60.0f) s_input_strip_user_h = 60.0f;
    if (s_input_strip_user_h > max_strip) s_input_strip_user_h = max_strip;

    float total_h = s_input_strip_user_h + splitter_h;

    ImGui::SetNextWindowPos(ImVec2(vp->WorkPos.x, strip_y));
    ImGui::SetNextWindowSize(ImVec2(vp->WorkSize.x, total_h));

    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.12f, 0.12f, 0.14f, 1.0f));

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse |
        ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoSavedSettings |
        ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse;

    if (ImGui::Begin("##InputStrip", nullptr, flags)) {
        float win_w = ImGui::GetContentRegionAvail().x;

        // ── Header bar at toolbar/strip intersection — prominent collapse button ──
        {
            ImDrawList* dl = ImGui::GetWindowDrawList();
            ImVec2 hp = ImGui::GetCursorScreenPos();

            // Dark header background to separate from toolbar above
            dl->AddRectFilled(hp, ImVec2(hp.x + vp->WorkSize.x, hp.y + hdr_h),
                              IM_COL32(18, 18, 22, 255));
            // Thin accent line at top edge
            dl->AddLine(ImVec2(hp.x, hp.y), ImVec2(hp.x + vp->WorkSize.x, hp.y),
                        IM_COL32(60, 130, 200, 120), 1.0f);

            // Source label on the left
            const char* src_name = "Input";
            if (g_app.input_source == InputSource::Plugin && g_app.active_plugin_idx >= 0 &&
                g_app.active_plugin_idx < g_app.plugin_mgr.pluginCount()) {
                auto& pl = g_app.plugin_mgr.plugins()[g_app.active_plugin_idx];
                if (pl.info && pl.info->name) src_name = pl.info->name;
            } else if (g_app.input_source == InputSource::CapturePlayback) {
                src_name = "Capture Playback";
            }
            dl->AddText(ImVec2(hp.x + 10, hp.y + 3), IM_COL32(140, 155, 180, 220), src_name);

            // "Hide Input" button — right-aligned, bright accent color, drawn up-arrow
            float btn_w = 110.0f;
            float btn_x = hp.x + vp->WorkSize.x - btn_w - 6.0f;
            ImGui::SetCursorScreenPos(ImVec2(btn_x, hp.y + 1));
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.18f, 0.35f, 0.58f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.25f, 0.45f, 0.70f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.30f, 0.50f, 0.80f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.90f, 0.93f, 1.0f, 1.0f));
            ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 3.0f);
            ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(8, 2));
            if (ImGui::Button("  Hide Input", ImVec2(btn_w, hdr_h - 2))) {
                s_input_strip_expanded = false;
            }
            ImGui::PopStyleVar(2);
            ImGui::PopStyleColor(4);
            // Draw up-arrow triangle on the button
            {
                ImVec2 bmin = ImGui::GetItemRectMin();
                float tri_cx = bmin.x + 12.0f;
                float tri_cy = bmin.y + (hdr_h - 2) * 0.5f;
                float tri_sz = 4.0f;
                dl->AddTriangleFilled(
                    ImVec2(tri_cx, tri_cy - tri_sz),
                    ImVec2(tri_cx - tri_sz, tri_cy + tri_sz),
                    ImVec2(tri_cx + tri_sz, tri_cy + tri_sz),
                    IM_COL32(220, 230, 255, 240));
            }

            // Advance cursor past header
            ImGui::SetCursorScreenPos(ImVec2(hp.x, hp.y + hdr_h));
        }

        // ── Scrollable content area ──
        float content_h = s_input_strip_user_h - hdr_h - splitter_h;
        if (content_h < 20.0f) content_h = 20.0f;

        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10, 4));
        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(6, 3));
        ImGui::BeginChild("##strip_content", ImVec2(-1, content_h), ImGuiChildFlags_None,
                          ImGuiWindowFlags_None);
        {
            if (g_app.source_switch_active || !g_app.motion_started) ImGui::BeginDisabled();

            // ── Plugin source ──
            if (g_app.input_source == InputSource::Plugin) {
                int pi = g_app.active_plugin_idx;
                if (pi >= 0 && pi < g_app.plugin_mgr.pluginCount()) {
                    auto& plug = g_app.plugin_mgr.plugins()[pi];
                    if (plug.valid && plug.info) {
                        if (plug.info->param_count > 0 && plug.info->params) {
                            DrawPluginParamsLayout(plug);

                            // "Home All" — only for plugins with purely float/int axis params
                            // (manual sliders, test signal). Skip for data sources with
                            // enum/bool config params (Assetto, SimTools UDP, etc.)
                            bool all_simple = true;
                            for (int p = 0; p < plug.info->param_count && all_simple; p++) {
                                if (plug.info->params[p].type == STEWART_PARAM_ENUM ||
                                    plug.info->params[p].type == STEWART_PARAM_BOOL)
                                    all_simple = false;
                            }
                            if (all_simple) {
                                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.25f, 0.25f, 0.35f, 1.0f));
                                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.35f, 0.35f, 0.50f, 1.0f));
                                if (ImGui::Button("Home All", ImVec2(70, 0))) {
                                    for (int p = 0; p < plug.info->param_count; p++) {
                                        float def_val = plug.info->params[p].default_val;
                                        float* val = PluginParamPtr(plug, p);
                                        if (val) *val = def_val;
                                        g_app.plugin_mgr.setParam(plug.info->params[p].name, def_val);
                                    }
                                }
                                ImGui::PopStyleColor(2);
                                if (ImGui::IsItemHovered())
                                    ImGui::SetTooltip("Reset all parameters to their default values");
                            }
                        } else {
                            ImGui::TextDisabled("No parameters.");
                        }

                        // Live output bars — always visible (zeros when inactive, no layout shift)
                        {
                            static const char* ax_lbl[6] = {"Surge","Sway","Heave","Roll","Pitch","Yaw"};
                            ImGui::Separator();
                            float vals[6] = {0};
                            if (plug.active) {
                                std::lock_guard<std::mutex> lock(g_app.input_mutex);
                                memcpy(vals, g_app.shared_input, sizeof(vals));
                            }
                            bool live = plug.active && !g_app.entities.empty();
                            float bar_w = (ImGui::GetContentRegionAvail().x - 5 * 6) / 6.0f;
                            if (bar_w < 60.0f) bar_w = 60.0f;
                            for (int i = 0; i < 6; i++) {
                                if (i > 0) ImGui::SameLine(0, 6);
                                float frac = fabsf(vals[i]) / 100.0f;
                                ImVec4 bar_col = !live ? ImVec4(0.25f, 0.25f, 0.30f, 0.5f) :
                                    vals[i] >= 0 ? ImVec4(0.2f, 0.7f, 0.5f, 0.8f) : ImVec4(0.7f, 0.3f, 0.3f, 0.8f);
                                ImGui::PushStyleColor(ImGuiCol_PlotHistogram, bar_col);
                                char overlay[32];
                                snprintf(overlay, sizeof(overlay), "%s %+.0f", ax_lbl[i], vals[i]);
                                ImGui::ProgressBar(frac, ImVec2(bar_w, 16), overlay);
                                ImGui::PopStyleColor();
                            }
                        }
                    }
                } else {
                    ImGui::TextDisabled("No plugin selected. Choose one from the Source dropdown above.");
                }
            }

            // ── Capture Playback source ──
            else if (g_app.input_source == InputSource::CapturePlayback) {
                if (g_app.saved_recordings.empty()) {
                    ImGui::TextDisabled("No saved captures. Record in Data Streams first.");
                } else {
                    // ── Controls row: Speed + Loop + Play + Stop ──
                    ImGui::AlignTextToFramePadding();
                    ImGui::Text("Speed");
                    ImGui::SameLine();
                    ImGui::PushItemWidth(60);
                    int spd_pct = (int)(g_app.capture_speed * 100.0f + 0.5f);
                    if (ImGui::SliderInt("##spd", &spd_pct, 10, 200, "%d%%"))
                        g_app.capture_speed = spd_pct / 100.0f;
                    ImGui::PopItemWidth();

                    ImGui::SameLine();
                    ImGui::Checkbox("Loop", &g_app.capture_loop);

                    ImGui::SameLine(0, 12);
                    bool can_play = !g_app.capture_playing && g_app.capture_playback_idx >= 0 && g_app.motion_started;
                    if (!can_play) ImGui::BeginDisabled();
                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.15f, 0.55f, 0.35f, 1.0f));
                    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.2f, 0.7f, 0.45f, 1.0f));
                    if (ImGui::Button("Play", ImVec2(50, 22)))
                        g_app.startCapturePlayback(g_app.capture_playback_idx);
                    ImGui::PopStyleColor(2);
                    if (!can_play) ImGui::EndDisabled();
                    if (!can_play && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled) && !g_app.motion_started)
                        ImGui::SetTooltip("Start motion first.");

                    ImGui::SameLine(0, 2);
                    bool can_stop = g_app.capture_playing;
                    if (!can_stop) ImGui::BeginDisabled();
                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.75f, 0.18f, 0.18f, 1.0f));
                    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.90f, 0.28f, 0.28f, 1.0f));
                    if (ImGui::Button("Stop", ImVec2(50, 22)))
                        g_app.stopCapturePlayback();
                    ImGui::PopStyleColor(2);
                    if (!can_stop) ImGui::EndDisabled();

                    // ── Progress bar (when playing) ──
                    if (g_app.capture_playing &&
                        g_app.capture_playback_idx >= 0 &&
                        g_app.capture_playback_idx < (int)g_app.saved_recordings.size())
                    {
                        auto& sr = g_app.saved_recordings[g_app.capture_playback_idx];
                        double elapsed = (g_app.frame_time - g_app.capture_start_time) * (double)g_app.capture_speed;
                        double dur = sr.duration();
                        float progress = dur > 0 ? (float)(elapsed / dur) : 0.0f;
                        if (progress < 0.0f) progress = 0.0f;
                        if (progress > 1.0f) progress = 1.0f;

                        const char* phase_str = "";
                        ImU32 bar_col = IM_COL32(60, 160, 60, 255);
                        switch (g_app.capture_ramp_phase) {
                            case App::CaptureRampPhase::RampIn:  phase_str = "IN";   bar_col = IM_COL32(60, 200, 120, 255); break;
                            case App::CaptureRampPhase::Playing: phase_str = "PLAY"; bar_col = IM_COL32(50, 165, 230, 255); break;
                            case App::CaptureRampPhase::RampOut: phase_str = "OUT";  bar_col = IM_COL32(230, 140, 50, 255); break;
                            case App::CaptureRampPhase::HomeHold:phase_str = "HOME"; bar_col = IM_COL32(100, 100, 180, 255); break;
                        }

                        ImVec2 pos = ImGui::GetCursorScreenPos();
                        float w = ImGui::GetContentRegionAvail().x;
                        float h = 20.0f;
                        if (w > 40.0f) {
                            ImDrawList* dl = ImGui::GetWindowDrawList();
                            dl->AddRectFilled(pos, ImVec2(pos.x + w, pos.y + h), IM_COL32(30, 30, 38, 255), 4.0f);
                            float fill_w = w * progress;
                            if (fill_w > 0.0f)
                                dl->AddRectFilled(pos, ImVec2(pos.x + fill_w, pos.y + h), bar_col, 4.0f);
                            char bar_text[128];
                            snprintf(bar_text, sizeof(bar_text), " %s  %.1f/%.1fs%s",
                                     phase_str, elapsed < 0 ? 0.0 : elapsed, dur,
                                     g_app.capture_loop ? "  LOOP" : "");
                            dl->AddText(ImVec2(pos.x + 4, pos.y + 3), IM_COL32(255, 255, 255, 220), bar_text);
                            ImGui::Dummy(ImVec2(w, h));
                        }
                    }

                    ImGui::Spacing();

                    // ── Scrollable card list (shows ~4-5 items) ──
                    float card_h = 56.0f;
                    float list_h = card_h * 4.5f;
                    float avail_h = ImGui::GetContentRegionAvail().y;
                    if (list_h > avail_h && avail_h > card_h * 2) list_h = avail_h;
                    if (list_h < card_h * 2) list_h = card_h * 2;

                    ImGui::BeginChild("##cap_list", ImVec2(-1, list_h), ImGuiChildFlags_Border);
                    {
                        static int s_ctx_idx = -1;          // right-click target
                        static int s_rename_idx = -1;       // rename target
                        static char s_rename_buf[64] = {};
                        static bool s_rename_focus = false;

                        for (int i = 0; i < (int)g_app.saved_recordings.size(); i++) {
                            auto& sr = g_app.saved_recordings[i];
                            bool selected = (g_app.capture_playback_idx == i);

                            const char* src_icon; const char* src_label; ImVec4 src_color;
                            GetCaptureSourceBadge(sr.source, &src_icon, &src_label, &src_color);

                            char dt_buf[64];
                            FormatDateTime(sr.created_time, dt_buf, sizeof(dt_buf));

                            // Compute data range
                            float gmin = 1e9f, gmax = -1e9f;
                            for (size_t si = 0; si < sr.samples.size(); si++) {
                                for (int a = 0; a < 6; a++) {
                                    if (sr.samples[si].input[a] < gmin) gmin = sr.samples[si].input[a];
                                    if (sr.samples[si].input[a] > gmax) gmax = sr.samples[si].input[a];
                                }
                            }
                            if (sr.samples.empty()) { gmin = 0; gmax = 0; }
                            bool flat = (fabsf(gmax - gmin) < 0.01f);

                            ImGui::PushID(i);

                            // Card selectable
                            ImVec2 card_start = ImGui::GetCursorScreenPos();
                            float card_w = ImGui::GetContentRegionAvail().x;

                            if (selected) {
                                ImGui::PushStyleColor(ImGuiCol_Header,        ImVec4(src_color.x * 0.2f, src_color.y * 0.2f, src_color.z * 0.2f, 0.6f));
                                ImGui::PushStyleColor(ImGuiCol_HeaderHovered, ImVec4(src_color.x * 0.3f, src_color.y * 0.3f, src_color.z * 0.3f, 0.7f));
                            }
                            if (ImGui::Selectable("##sc", selected, g_app.capture_playing ? ImGuiSelectableFlags_Disabled : 0, ImVec2(card_w, card_h))) {
                                if (!g_app.capture_playing) g_app.capture_playback_idx = i;
                            }
                            if (!g_app.capture_playing && g_app.motion_started && ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(0)) {
                                g_app.capture_playback_idx = i;
                                g_app.startCapturePlayback(i);
                            }
                            if (ImGui::IsItemHovered() && ImGui::IsMouseClicked(ImGuiMouseButton_Right))
                                s_ctx_idx = i;
                            if (selected) ImGui::PopStyleColor(2);

                            // Custom draw over selectable
                            ImDrawList* dl = ImGui::GetWindowDrawList();
                            float pad_x = 6.0f;
                            float badge_h_px = 18.0f;
                            float row1_y = card_start.y + 3.0f;
                            float row2_y = row1_y + badge_h_px + 2.0f;
                            float row3_y = row2_y + 13.0f;

                            // Row 1: Badge + Name
                            DrawBadge(dl, ImVec2(card_start.x + pad_x, row1_y), src_icon, src_color, badge_h_px);
                            float name_x = card_start.x + pad_x + ImGui::CalcTextSize(src_icon).x + 18.0f;
                            ImU32 name_col = selected
                                ? IM_COL32((int)(src_color.x * 255), (int)(src_color.y * 255), (int)(src_color.z * 255), 255)
                                : IM_COL32(220, 220, 220, 255);
                            dl->AddText(ImVec2(name_x, row1_y + 1.0f), name_col, sr.name);

                            // Duration right-aligned on row 1
                            char dur_buf[32];
                            snprintf(dur_buf, sizeof(dur_buf), "%.1fs", sr.duration());
                            ImVec2 dur_sz = ImGui::CalcTextSize(dur_buf);
                            dl->AddText(ImVec2(card_start.x + card_w - dur_sz.x - 8.0f, row1_y + 1.0f),
                                        IM_COL32(180, 200, 220, 220), dur_buf);

                            // Row 2: Source label + date/time
                            char detail1[192];
                            snprintf(detail1, sizeof(detail1), "%s  |  %s", src_label, dt_buf);
                            dl->AddText(ImVec2(card_start.x + pad_x + 4.0f, row2_y),
                                        IM_COL32(140, 140, 140, 200), detail1);

                            // Row 3: Sample rate, sample count, data range
                            char detail2[128];
                            if (flat)
                                snprintf(detail2, sizeof(detail2), "%.0f Hz  |  %d samples  |  [FLAT @ %.0f%%]",
                                    sr.sample_rate_hz, (int)sr.samples.size(), gmin);
                            else
                                snprintf(detail2, sizeof(detail2), "%.0f Hz  |  %d samples  |  [%.0f..%.0f%%]",
                                    sr.sample_rate_hz, (int)sr.samples.size(), gmin, gmax);
                            ImU32 range_col = flat ? IM_COL32(240, 150, 50, 200) : IM_COL32(120, 120, 120, 180);
                            dl->AddText(ImVec2(card_start.x + pad_x + 4.0f, row3_y), range_col, detail2);

                            // Selected indicator
                            if (selected) {
                                float dot_x = card_start.x + card_w - 10.0f;
                                float dot_y = card_start.y + card_h * 0.5f;
                                dl->AddCircleFilled(ImVec2(dot_x, dot_y), 4.0f,
                                    IM_COL32((int)(src_color.x * 255), (int)(src_color.y * 255),
                                             (int)(src_color.z * 255), 255));
                            }

                            ImGui::PopID();
                        }

                        // Right-click context menu (Rename / Delete)
                        if (s_ctx_idx >= 0)
                            ImGui::OpenPopup("##cap_ctx");
                        if (ImGui::BeginPopup("##cap_ctx")) {
                            if (s_ctx_idx >= 0 && s_ctx_idx < (int)g_app.saved_recordings.size()) {
                                ImGui::TextDisabled("%s", g_app.saved_recordings[s_ctx_idx].name);
                                ImGui::Separator();
                                if (ImGui::MenuItem("Rename")) {
                                    s_rename_idx = s_ctx_idx;
                                    snprintf(s_rename_buf, sizeof(s_rename_buf), "%s", g_app.saved_recordings[s_ctx_idx].name);
                                    s_rename_focus = true;
                                    s_ctx_idx = -1;
                                    ImGui::CloseCurrentPopup();
                                }
                                if (ImGui::MenuItem("Delete")) {
                                    g_app.deleteRecordingFromLibrary(s_ctx_idx);
                                    if (g_app.capture_playback_idx >= (int)g_app.saved_recordings.size())
                                        g_app.capture_playback_idx = (int)g_app.saved_recordings.size() - 1;
                                    s_ctx_idx = -1;
                                    ImGui::CloseCurrentPopup();
                                }
                            } else {
                                s_ctx_idx = -1;
                                ImGui::CloseCurrentPopup();
                            }
                            ImGui::EndPopup();
                        } else {
                            s_ctx_idx = -1;
                        }

                        // Rename popup (modal)
                        if (s_rename_idx >= 0)
                            ImGui::OpenPopup("Rename Recording##ren_cap");
                        if (ImGui::BeginPopupModal("Rename Recording##ren_cap", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
                            if (s_rename_focus) {
                                ImGui::SetKeyboardFocusHere();
                                s_rename_focus = false;
                            }
                            ImGui::Text("New name:");
                            bool enter = ImGui::InputText("##ren_name", s_rename_buf, sizeof(s_rename_buf),
                                                          ImGuiInputTextFlags_EnterReturnsTrue);
                            if (enter || ImGui::Button("OK", ImVec2(80, 0))) {
                                if (s_rename_idx >= 0 && s_rename_idx < (int)g_app.saved_recordings.size() && s_rename_buf[0]) {
                                    snprintf(g_app.saved_recordings[s_rename_idx].name, sizeof(g_app.saved_recordings[s_rename_idx].name),
                                             "%s", s_rename_buf);
                                    g_app.saveRecordingsToDisk();
                                }
                                s_rename_idx = -1;
                                ImGui::CloseCurrentPopup();
                            }
                            ImGui::SameLine();
                            if (ImGui::Button("Cancel", ImVec2(80, 0))) {
                                s_rename_idx = -1;
                                ImGui::CloseCurrentPopup();
                            }
                            ImGui::EndPopup();
                        }
                    }
                    ImGui::EndChild();
                }
            }

            if (g_app.source_switch_active || !g_app.motion_started) ImGui::EndDisabled();

            // Measure actual content height for auto-fit
            s_input_strip_content_h = ImGui::GetCursorPosY();
        }
        ImGui::EndChild();
        ImGui::PopStyleVar(2);

        // ── Bottom edge: slim resize grip ──
        {
            ImVec2 sp = ImGui::GetCursorScreenPos();
            float full_w = vp->WorkSize.x;

            ImGui::InvisibleButton("##strip_resize", ImVec2(full_w, splitter_h));
            bool rh_hovered = ImGui::IsItemHovered();
            bool rh_active  = ImGui::IsItemActive();

            ImDrawList* dl = ImGui::GetWindowDrawList();
            ImU32 grip_col = rh_active  ? IM_COL32(100, 180, 255, 255) :
                             rh_hovered ? IM_COL32(90, 150, 210, 200) :
                                          IM_COL32(45, 45, 55, 180);
            float line_y = sp.y + splitter_h * 0.5f;
            dl->AddLine(ImVec2(sp.x, line_y), ImVec2(sp.x + full_w, line_y), grip_col, 1.0f);
            float grip_cx = sp.x + full_w * 0.5f;
            for (int d = -3; d <= 3; d++)
                dl->AddCircleFilled(ImVec2(grip_cx + d * 8.0f, line_y), 1.5f, grip_col);

            if (rh_hovered || rh_active)
                ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeNS);

            if (rh_active) {
                float delta = ImGui::GetIO().MouseDelta.y;
                if (delta != 0.0f) {
                    s_input_strip_user_h += delta;
                    if (s_input_strip_user_h < 60.0f) s_input_strip_user_h = 60.0f;
                    if (s_input_strip_user_h > max_strip) s_input_strip_user_h = max_strip;
                }
            }
        }
    }
    ImGui::End();
    ImGui::PopStyleColor();
    ImGui::PopStyleVar(3);

    s_input_strip_h = total_h;
}

// Forward declarations for entity tab content functions
static void DrawPlatformSetupContent(Entity& e);
static void DrawEntitySettingsContent(Entity& e);
static void DrawEntityConsoleContent(Entity& e);
static void DrawEntityDynamicsContent(Entity& e);

// ── Entity Card (3D viewport placeholder + readout) ─────────────────

static void DrawEntityCard(Entity& e) {
    if (!e.show_card) return;
    ImGui::PushID(e.id);
    ImVec4 col = ColorFromFloat4(e.color);

    char title[128];
    const char* type_str = (e.type == EntityType::SIL) ? "SIL" : "HIL";
    snprintf(title, sizeof(title), "%s [%s]###entity_%d", e.name, type_str, e.id);

    ImGui::PushStyleColor(ImGuiCol_TitleBg, ImVec4(col.x * 0.3f, col.y * 0.3f, col.z * 0.3f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_TitleBgActive, ImVec4(col.x * 0.5f, col.y * 0.5f, col.z * 0.5f, 1.0f));

    if (ImGui::Begin(title, &e.show_card, ImGuiWindowFlags_None)) {
        // Header row: type badge + enable toggle + dynamics + remove
        ImGui::TextColored(col, "%s", type_str);
        ImGui::SameLine();
        ImGui::Checkbox("Enabled", &e.enabled);
        ImGui::SameLine();
        if (ImGui::SmallButton("Dynamics")) { s_show_dynamics = true; s_selected_dynamics_id = e.id; }
        ImGui::SameLine();
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.6f, 0.15f, 0.15f, 1.0f));
        if (ImGui::SmallButton("Remove")) ImGui::OpenPopup("##confirm_remove");
        ImGui::PopStyleColor();

        bool remove = false;
        if (ImGui::BeginPopupModal("##confirm_remove", nullptr, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoTitleBar)) {
            const char* type_label = (e.type == EntityType::SIL) ? "SIL" : "HIL";
            ImGui::Text("Remove %s entity '%s'?", type_label, e.name);
            ImGui::Spacing();
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.85f, 0.20f, 0.20f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.95f, 0.30f, 0.30f, 1.0f));
            if (ImGui::Button("Remove", ImVec2(100, 0))) { remove = true; ImGui::CloseCurrentPopup(); }
            ImGui::PopStyleColor(2);
            ImGui::SameLine();
            if (ImGui::Button("Cancel", ImVec2(100, 0))) ImGui::CloseCurrentPopup();
            ImGui::EndPopup();
        }

        // ── Tab bar: Overview | Settings | Platform | I/O ──
        if (ImGui::BeginTabBar("##entity_tabs", ImGuiTabBarFlags_None)) {

            // ════════════════════════════════════════════════════════════
            //  Overview tab (3D viz + workspace readout)
            // ════════════════════════════════════════════════════════════
            if (ImGui::BeginTabItem("Overview")) {
                // ── 3D Platform Viewport ──
                ImVec2 avail = ImGui::GetContentRegionAvail();
                if (e.viz_split_ratio < 0.15f) e.viz_split_ratio = 0.15f;
                if (e.viz_split_ratio > 0.92f) e.viz_split_ratio = 0.92f;
                float viz_h = fmaxf(80.0f, avail.y * e.viz_split_ratio);
                ImVec2 p = ImGui::GetCursorScreenPos();
                ImDrawList* dl = ImGui::GetWindowDrawList();

                ImGui::InvisibleButton("##viz3d", ImVec2(avail.x, viz_h));
                bool viz_hovered = ImGui::IsItemHovered();
                bool viz_active  = ImGui::IsItemActive();

                DrawPlatformViz(dl, p, ImVec2(avail.x, viz_h), e, e.viz_cam, viz_hovered, viz_active);

                // HIL status overlay on 3D viewport
                if (e.type == EntityType::HIL) {
                    bool hil_connected = e.serial && e.serial->isOpen();
                    const char* label = nullptr;
                    ImU32 label_col = 0;
                    if (e.hil_tel_active) {
                        label = "ESP32 TELEMETRY";
                        label_col = IM_COL32(60, 200, 120, 255);
                    } else if (hil_connected) {
                        label = "CONNECTED (awaiting telemetry)";
                        label_col = IM_COL32(240, 180, 50, 255);
                    } else {
                        label = "OFFLINE";
                        label_col = IM_COL32(120, 120, 130, 200);
                    }
                    float overlay_y = p.y + 4.0f;
                    if (label) {
                        ImVec2 ts = ImGui::CalcTextSize(label);
                        ImVec2 tp(p.x + avail.x - ts.x - 6.0f, overlay_y);
                        dl->AddRectFilled(ImVec2(tp.x - 3, tp.y - 1), ImVec2(tp.x + ts.x + 3, tp.y + ts.y + 1),
                                          IM_COL32(0, 0, 0, 180), 3.0f);
                        dl->AddText(tp, label_col, label);
                        overlay_y += ts.y + 4.0f;
                    }

                    auto& dp = e.hil_device_params;
                    if (e.hil_handshake_phase == HandshakePhase::Ready && dp.fw_version[0] != '\0') {
                        char info1[128];
                        if (dp.platform_id[0] != '\0')
                            snprintf(info1, sizeof(info1), "fw %s  proto %d  %s", dp.fw_version, dp.proto_ver, dp.platform_id);
                        else
                            snprintf(info1, sizeof(info1), "fw %s  proto %d", dp.fw_version, dp.proto_ver);
                        ImVec2 ts1 = ImGui::CalcTextSize(info1);
                        ImVec2 tp1(p.x + avail.x - ts1.x - 6.0f, overlay_y);
                        dl->AddRectFilled(ImVec2(tp1.x - 3, tp1.y - 1), ImVec2(tp1.x + ts1.x + 3, tp1.y + ts1.y + 1),
                                          IM_COL32(0, 0, 0, 160), 3.0f);
                        dl->AddText(tp1, IM_COL32(160, 180, 200, 200), info1);
                        overlay_y += ts1.y + 2.0f;

                        if (dp.config_received) {
                            char info2[128];
                            snprintf(info2, sizeof(info2), "L1=%.1f  L2=%.1f  H=%.1f  %d-bit",
                                dp.L1, dp.L2, dp.height, dp.bits_received ? dp.bit_depth : 12);
                            ImVec2 ts2 = ImGui::CalcTextSize(info2);
                            ImVec2 tp2(p.x + avail.x - ts2.x - 6.0f, overlay_y);
                            dl->AddRectFilled(ImVec2(tp2.x - 3, tp2.y - 1), ImVec2(tp2.x + ts2.x + 3, tp2.y + ts2.y + 1),
                                              IM_COL32(0, 0, 0, 160), 3.0f);
                            dl->AddText(tp2, IM_COL32(140, 160, 180, 180), info2);
                        }
                    } else if (hil_connected && e.hil_handshake_phase != HandshakePhase::Idle &&
                               e.hil_handshake_phase != HandshakePhase::Ready) {
                        ImVec2 ts = ImGui::CalcTextSize(e.hil_handshake_msg);
                        ImVec2 tp(p.x + avail.x - ts.x - 6.0f, overlay_y);
                        dl->AddRectFilled(ImVec2(tp.x - 3, tp.y - 1), ImVec2(tp.x + ts.x + 3, tp.y + ts.y + 1),
                                          IM_COL32(0, 0, 0, 160), 3.0f);
                        dl->AddText(tp, IM_COL32(240, 190, 60, 220), e.hil_handshake_msg);
                    }
                }

                // Entity color border
                dl->AddRect(p, ImVec2(p.x + avail.x, p.y + viz_h),
                            IM_COL32((int)(col.x*255), (int)(col.y*255), (int)(col.z*255), 80));

                // ── Draggable splitter ──
                {
                    float splitter_h = 6.0f;
                    ImVec2 sp = ImGui::GetCursorScreenPos();
                    ImGui::InvisibleButton("##viz_splitter", ImVec2(avail.x, splitter_h));
                    bool split_hovered = ImGui::IsItemHovered();
                    bool split_active  = ImGui::IsItemActive();

                    ImU32 split_col = split_active  ? IM_COL32(100, 180, 255, 255) :
                                      split_hovered ? IM_COL32(80, 140, 200, 200) :
                                                      IM_COL32(60, 60, 70, 150);
                    float line_y = sp.y + splitter_h * 0.5f;
                    dl->AddLine(ImVec2(sp.x + 4, line_y), ImVec2(sp.x + avail.x - 4, line_y), split_col, split_active ? 2.5f : 1.5f);

                    float grip_cx = sp.x + avail.x * 0.5f;
                    for (int d = -2; d <= 2; d++) {
                        dl->AddCircleFilled(ImVec2(grip_cx + d * 8.0f, line_y), 1.5f, split_col);
                    }

                    if (split_hovered || split_active)
                        ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeNS);

                    if (split_active) {
                        float delta = ImGui::GetIO().MouseDelta.y;
                        if (delta != 0.0f && avail.y > 0.0f) {
                            e.viz_split_ratio += delta / avail.y;
                            if (e.viz_split_ratio < 0.15f) e.viz_split_ratio = 0.15f;
                            if (e.viz_split_ratio > 0.92f) e.viz_split_ratio = 0.92f;
                        }
                    }
                }

                // ── HIL Connection Error Banner ──────────────────────────
                if (e.type == EntityType::HIL && e.hil_last_error[0] != '\0'
                    && e.hil_handshake_phase != HandshakePhase::Ready) {
                    ImGui::Spacing();
                    ImVec2 err_p = ImGui::GetCursorScreenPos();
                    float err_w = ImGui::GetContentRegionAvail().x;

                    // Determine if it's a fingerprint mismatch for special styling
                    bool is_mismatch = (strncmp(e.hil_last_error, "FINGERPRINT MISMATCH", 20) == 0);
                    ImVec4 bg_col = is_mismatch ? ImVec4(0.55f, 0.12f, 0.08f, 0.95f)
                                                : ImVec4(0.45f, 0.12f, 0.08f, 0.95f);
                    ImVec4 border_col = is_mismatch ? ImVec4(0.95f, 0.3f, 0.2f, 1.0f)
                                                    : ImVec4(0.85f, 0.3f, 0.15f, 1.0f);

                    ImGui::PushStyleColor(ImGuiCol_ChildBg, bg_col);
                    ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 4.0f);
                    if (ImGui::BeginChild("##conn_error_banner", ImVec2(err_w, 0), true,
                                         ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_AlwaysAutoResize)) {
                        // Error icon + title
                        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.35f, 0.25f, 1.0f));
                        ImGui::Text(is_mismatch ? "  FINGERPRINT MISMATCH" : "  CONNECTION FAILED");
                        ImGui::PopStyleColor();

                        ImGui::SameLine(err_w - 80.0f);
                        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.3f, 0.3f, 0.35f, 0.8f));
                        if (ImGui::SmallButton("Dismiss")) {
                            e.hil_last_error[0] = '\0';
                        }
                        ImGui::PopStyleColor();

                        ImGui::Separator();

                        // Multi-line error message
                        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.85f, 0.8f, 1.0f));
                        ImGui::TextWrapped("%s", e.hil_last_error);
                        ImGui::PopStyleColor();

                        // Action buttons
                        ImGui::Spacing();
                        if (is_mismatch) {
                            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.6f, 0.15f, 0.1f, 1.0f));
                            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.8f, 0.2f, 0.15f, 1.0f));
                            if (ImGui::Button("Clear Fingerprint & Reconnect")) {
                                memset(e.hil_fingerprint, 0, sizeof(e.hil_fingerprint));
                                memset(e.hil_fw_version, 0, sizeof(e.hil_fw_version));
                                e.hil_proto_ver = 0;
                                e.hil_last_error[0] = '\0';
                                e.hil_handshake_ok = false;
                                e.hil_handshake_phase = HandshakePhase::Idle;
                                e.hil_device_params.clear();
                                g_app.log(e.id, "hil", "Fingerprint cleared — will pair with next device");
                                g_app.settings_dirty = true;
                                // Trigger reconnect
                                if (!e.serial || !e.serial->isOpen()) {
                                    e.hil_auto_connect = true;
                                    e.hil_last_reconnect = 0.0;
                                }
                            }
                            ImGui::PopStyleColor(2);
                        } else {
                            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.4f, 0.7f, 1.0f));
                            if (ImGui::Button("Retry Connection")) {
                                e.hil_last_error[0] = '\0';
                                e.hil_handshake_phase = HandshakePhase::WaitFingerprint;
                                e.hil_handshake_pending = true;
                                e.hil_hs_attempts = 1;
                                e.hil_hs_last_send = g_app.frame_time;
                                if (e.serial && e.serial->isOpen()) {
                                    e.serial->write((const uint8_t*)"\0\0\0\0\0\0\0\0", 8);
                                    e.serial->sendCommand("FINGERPRINT?");
                                } else {
                                    e.hil_auto_connect = true;
                                    e.hil_last_reconnect = 0.0;
                                }
                            }
                            ImGui::PopStyleColor();
                        }
                        ImGui::SameLine();
                        ImGui::TextDisabled("(also check the Settings tab)");
                    }
                    ImGui::EndChild();
                    ImGui::PopStyleVar();
                    ImGui::PopStyleColor();

                    // Draw colored border around error banner
                    ImVec2 err_end = ImGui::GetCursorScreenPos();
                    // border is handled by BeginChild with border flag
                    ImGui::Spacing();
                }

                // HIL: don't show telemetry-derived workspace until handshake confirmed
                bool hil_no_tel = (e.type == EntityType::HIL && !e.hil_handshake_ok);

                // ── Workspace Utilization ──
                {
                    float mu = hil_no_tel ? 0.0f : e.state.max_util;
                    ImVec4 mu_col = mu < 70.0f ? ImVec4(0.2f, 0.83f, 0.6f, 1.0f) :
                                    mu < 90.0f ? ImVec4(0.98f, 0.75f, 0.15f, 1.0f) :
                                                 ImVec4(0.98f, 0.44f, 0.44f, 1.0f);
                    ImGui::TextColored(mu_col, "Workspace: %.0f%%", mu);
                    ImGui::SameLine();
                    ImGui::TextDisabled("(%.0f%% intensity)", e.config.intensity);

                    if (mu > 90.0f && e.config.intensity > 1.0f) {
                        float safe_intensity = floorf(e.config.intensity * (85.0f / mu));
                        if (safe_intensity < 1.0f) safe_intensity = 1.0f;
                        ImGui::SameLine();
                        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.85f, 0.55f, 0.15f, 1.0f));
                        char fit_label[48];
                        snprintf(fit_label, sizeof(fit_label), "Auto-fit to %.0f%%", safe_intensity);
                        if (ImGui::SmallButton(fit_label)) {
                            e.config.intensity = safe_intensity;
                        }
                        ImGui::PopStyleColor();
                        if (ImGui::IsItemHovered())
                            ImGui::SetTooltip("Reduce intensity to keep all servos\nwithin 85%% of workspace.");
                    }
                }

                // Per-servo headroom bars
                for (int i = 0; i < 6; i++) {
                    float util = hil_no_tel ? 0.0f : e.state.servo_util[i];
                    float deg  = hil_no_tel ? 0.0f : e.state.output_angles_deg[i];
                    ImVec4 bar_col = util < 70.0f ? ImVec4(0.2f, 0.83f, 0.6f, 1.0f) :
                                     util < 90.0f ? ImVec4(0.98f, 0.75f, 0.15f, 1.0f) :
                                                    ImVec4(0.98f, 0.44f, 0.44f, 1.0f);
                    ImGui::PushStyleColor(ImGuiCol_PlotHistogram, bar_col);
                    char overlay[48];
                    snprintf(overlay, sizeof(overlay), "S%d  %.1f\xc2\xb0  %.0f%%", i, deg, util);
                    ImGui::ProgressBar(util / 100.0f, ImVec2(-1, 0), overlay);
                    ImGui::PopStyleColor();
                }

                // ── Status line ──
                ImGui::Separator();
                if (e.type == EntityType::SIL) {
                    ImGui::Text("IK: %.0f Hz", e.rate_ik_hz);
                } else {
                    bool hil_conn = e.serial && e.serial->isOpen();
                    if (hil_conn) {
                        ImGui::Text("TX: %.0f Hz | Tel: %.0f Hz", e.rate_tx_hz, e.rate_tel_hz);
                        ImGui::SameLine();
                        ImGui::TextColored(ImVec4(0.2f, 0.83f, 0.6f, 1.0f), "| %s", e.serial->portName());
                        ImGui::SameLine();
                        ImGui::PushItemWidth(80);
                        const char* rate_opts[] = {"10 Hz", "20 Hz", "30 Hz", "50 Hz"};
                        int rate_vals[] = {10, 20, 30, 50};
                        int cur_sel = 2;
                        for (int r = 0; r < 4; r++)
                            if (e.hil_tel_target_hz == rate_vals[r]) cur_sel = r;
                        char combo_lbl[32];
                        snprintf(combo_lbl, sizeof(combo_lbl), "##telrate_%d", e.id);
                        if (ImGui::Combo(combo_lbl, &cur_sel, rate_opts, 4)) {
                            e.hil_tel_target_hz = rate_vals[cur_sel];
                            char cmd[32];
                            snprintf(cmd, sizeof(cmd), "TELRATE:%d", e.hil_tel_target_hz);
                            e.serial->sendCommand(cmd);
                            g_app.log(e.id, "hil", "Set telemetry rate to %d Hz", e.hil_tel_target_hz);
                        }
                        ImGui::PopItemWidth();
                        if (ImGui::IsItemHovered())
                            ImGui::SetTooltip("ESP32 telemetry send rate.\nHigher = smoother viz, more serial traffic.");
                        // Tick rate selector
                        ImGui::SameLine();
                        ImGui::PushItemWidth(90);
                        const char* tick_opts[] = {"4µs 250kHz", "8µs 125kHz", "10µs 100kHz", "20µs 50kHz"};
                        int tick_vals[] = {4, 8, 10, 20};
                        int tick_sel = 0;
                        for (int t = 0; t < 4; t++)
                            if (e.hil_tick_rate_us == tick_vals[t]) tick_sel = t;
                        char tick_lbl[32];
                        snprintf(tick_lbl, sizeof(tick_lbl), "##tickrate_%d", e.id);
                        if (ImGui::Combo(tick_lbl, &tick_sel, tick_opts, 4)) {
                            e.hil_tick_rate_us = tick_vals[tick_sel];
                            char tcmd[32];
                            snprintf(tcmd, sizeof(tcmd), "TICKRATE:%d", e.hil_tick_rate_us);
                            e.serial->sendCommand(tcmd);
                            g_app.settings_dirty = true;
                            g_app.log(e.id, "hil", "Set tick rate to %d µs (%lu Hz max step)",
                                e.hil_tick_rate_us, 1000000UL / ((unsigned long)e.hil_tick_rate_us * 2));
                        }
                        ImGui::PopItemWidth();
                        if (ImGui::IsItemHovered())
                            ImGui::SetTooltip("ISR tick period sent to the ESP32 firmware.\n"
                                "4µs = 250kHz tick, 125kHz max step/motor\n"
                                "8µs = 125kHz tick, 62.5kHz max step/motor\n"
                                "Saved to device NVS. Takes effect immediately.");
                    } else if (e.hil_auto_connect && e.hil_port[0] != '\0') {
                        ImGui::TextColored(ImVec4(0.95f, 0.75f, 0.2f, 1.0f), "Searching %s...", e.hil_port);
                    } else {
                        ImGui::TextDisabled("offline");
                    }
                }

                ImGui::EndTabItem();
            }

            // ════════════════════════════════════════════════════════════
            //  Dynamics tab
            // ════════════════════════════════════════════════════════════
            if (ImGui::BeginTabItem("Dynamics")) {
                DrawEntityDynamicsContent(e);
                ImGui::EndTabItem();
            }

            // ════════════════════════════════════════════════════════════
            //  Settings tab
            // ════════════════════════════════════════════════════════════
            if (ImGui::BeginTabItem("Settings")) {
                DrawEntitySettingsContent(e);
                ImGui::EndTabItem();
            }

            // ════════════════════════════════════════════════════════════
            //  Platform tab
            // ════════════════════════════════════════════════════════════
            if (ImGui::BeginTabItem("Platform")) {
                DrawPlatformSetupContent(e);
                ImGui::EndTabItem();
            }

            // ════════════════════════════════════════════════════════════
            //  Test Harness tab (HIL only)
            // ════════════════════════════════════════════════════════════
            if (e.type == EntityType::HIL) {
                if (ImGui::BeginTabItem("Test Harness")) {
                    DrawTestHarnessTabContent(e);
                    ImGui::EndTabItem();
                }
            }

            // ════════════════════════════════════════════════════════════
            //  I/O tab
            // ════════════════════════════════════════════════════════════
            if (ImGui::BeginTabItem("I/O")) {
                DrawEntityConsoleContent(e);
                ImGui::EndTabItem();
            }

            ImGui::EndTabBar();
        }

        if (remove) {
            int remove_id = e.id;
            ImGui::End();
            ImGui::PopStyleColor(2);
            ImGui::PopID();
            g_app.removeEntity(remove_id);
            return;
        }
    }
    ImGui::End();
    ImGui::PopStyleColor(2);
    ImGui::PopID();
}

// ── Platform Setup Content (drawn inside entity card tab) ────────────

static void DrawPlatformSetupContent(Entity& e) {
    ImGui::PushID(e.id + 2000);
    {
        ImVec4 col = ColorFromFloat4(e.color);

        if (ImGui::BeginTabBar("##plat_tabs")) {

            // ════════════════════════════════════════════════════════
            //  Geometry Tab
            // ════════════════════════════════════════════════════════
            if (ImGui::BeginTabItem("Geometry")) {
                ImGui::Text("Platform Dimensions");
                ImGui::Separator();

                bool geo_changed = false;
                geo_changed |= ImGui::DragFloat("RD (base radius)", &e.config.geometry.RD, 0.5f, 50, 1000, "%.1f mm");
                geo_changed |= ImGui::DragFloat("PD (platform radius)", &e.config.geometry.PD, 0.5f, 50, 1000, "%.1f mm");
                geo_changed |= ImGui::DragFloat("L1 (servo arm)", &e.config.geometry.ServoArmLengthL1, 0.5f, 10, 500, "%.1f mm");
                geo_changed |= ImGui::DragFloat("L2 (connecting rod)", &e.config.geometry.ConnectingArmLengthL2, 0.5f, 50, 2000, "%.1f mm");

                geo_changed |= ImGui::DragFloat("Home Height", &e.config.geometry.platformHeight, 0.5f, 50, 2000, "%.1f mm");
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("Height of the platform above base when all servos are at 0\xc2\xb0.\nClick Auto to compute from geometry.");
                ImGui::SameLine();
                {
                    float computed_h = computeHomeHeight(&e.config.platform);
                    float diff = fabsf(e.config.geometry.platformHeight - computed_h);
                    if (diff > 0.5f)
                        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.7f, 0.5f, 0.1f, 1.0f));
                    else
                        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.2f, 0.2f, 1.0f));
                    if (ImGui::SmallButton("Auto")) {
                        e.config.geometry.platformHeight = computed_h;
                        geo_changed = true;
                        g_app.log(e.id, "platform", "Home height auto-computed: %.1f mm", computed_h);
                    }
                    ImGui::PopStyleColor();
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("Auto-compute home height from geometry.\nComputed: %.1f mm (current: %.1f mm, diff: %.1f mm)",
                            computed_h, e.config.geometry.platformHeight, diff);
                }

                float tr = e.config.geometry.theta_r;
                if (ImGui::DragFloat("Theta R (base angle)", &tr, 0.1f, 0, 90, "%.1f\xc2\xb0")) {
                    e.config.geometry.theta_r = tr;
                    geo_changed = true;
                }
                float tp = e.config.geometry.theta_p;
                if (ImGui::DragFloat("Theta P (platform angle)", &tp, 0.1f, 0, 90, "%.1f\xc2\xb0")) {
                    e.config.geometry.theta_p = tp;
                    geo_changed = true;
                }

                if (geo_changed) {
                    e.config.rebuildPlatform();
                    computeAxisScalesFromGeometry(&e.config.axis_scales, &e.config.geometry, 0.9f);
                    g_app.log(e.id, "platform", "Geometry changed — workspace recalculated");
                }

                ImGui::Spacing();
                ImGui::TextDisabled("Workspace: +/-%.0fmm linear, +/-%.1f\xc2\xb0 angular",
                    e.config.axis_scales.scale[0],
                    e.config.axis_scales.scale[3] * (float)(180.0 / M_PI));

                // Servo limits
                ImGui::Separator();
                ImGui::Text("Servo Limits");
                float smin_deg = e.config.platform.servo_min_rad * (float)(180.0 / M_PI);
                float smax_deg = e.config.platform.servo_max_rad * (float)(180.0 / M_PI);
                if (ImGui::DragFloat("Servo Min", &smin_deg, 0.5f, -90, 0, "%.1f\xc2\xb0")) {
                    e.config.platform.servo_min_rad = smin_deg * (float)(M_PI / 180.0);
                }
                if (ImGui::DragFloat("Servo Max", &smax_deg, 0.5f, 0, 90, "%.1f\xc2\xb0")) {
                    e.config.platform.servo_max_rad = smax_deg * (float)(M_PI / 180.0);
                }

                ImGui::EndTabItem();
            }

            // ════════════════════════════════════════════════════════
            //  Drive Train Tab
            // ════════════════════════════════════════════════════════
            if (ImGui::BeginTabItem("Drive Train")) {

                if (e.config.isStepper()) {
                    // ── Stepper / Closed-Loop Servo Drive ──
                    ImGui::Text("Stepper Drive Parameters");
                    ImGui::Separator();

                    bool dt_changed = false;
                    dt_changed |= ImGui::DragFloat("Virtual Gear", &e.config.geometry.virtual_gear, 1.0f, 1, 1000, "%.0f");
                    if (ImGui::IsItemHovered()) ImGui::SetTooltip("AASD-15A electronic gear numerator (Pn098)");
                    dt_changed |= ImGui::DragFloat("Planetary Ratio", &e.config.geometry.planetary_ratio, 1.0f, 1, 200, "%.0f:1");
                    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Planetary gearbox reduction ratio");
                    int ppr = e.config.geometry.encoder_ppr;
                    if (ImGui::DragInt("Encoder PPR", &ppr, 10, 100, 10000)) {
                        e.config.geometry.encoder_ppr = ppr;
                        dt_changed = true;
                    }
                    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Motor encoder pulses per revolution");
                    if (dt_changed) {
                        computeStepsPerDegree(&e.config.geometry);
                        e.config.platform.steps_per_degree = e.config.geometry.steps_per_degree;
                        g_app.log(e.id, "platform", "Drive train updated — %.2f steps/deg", e.config.geometry.steps_per_degree);
                    }

                    ImGui::Spacing();
                    ImGui::Text("Steps/deg: %.2f", e.config.geometry.steps_per_degree);

                } else {
                    // ── PWM Servo Drive (Mini-6DOF) ──
                    ImGui::Text("PWM Servo Parameters");
                    ImGui::Separator();

                    auto& sv = e.config.servo;

                    bool sv_changed = false;

                    // Per-servo center positions
                    ImGui::TextDisabled("Servo Center Positions (µs)");
                    static const char* servo_labels[] = {"Servo 0", "Servo 1", "Servo 2", "Servo 3", "Servo 4", "Servo 5"};
                    for (int s = 0; s < 6; s++) {
                        ImGui::PushID(s);
                        if (ImGui::DragInt(servo_labels[s], &sv.center_us[s], 1, 500, 2500)) {
                            sv_changed = true;
                        }
                        ImGui::PopID();
                    }

                    ImGui::Spacing();
                    ImGui::Separator();
                    ImGui::TextDisabled("Pulse Configuration");
                    sv_changed |= ImGui::DragFloat("Pulse/Radian", &sv.pulse_per_rad, 1.0f, 50.0f, 1000.0f, "%.1f µs/rad");
                    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Microseconds of pulse width per radian of servo rotation");
                    sv_changed |= ImGui::DragInt("Min Pulse", &sv.min_pulse_us, 1, 500, 1500);
                    sv_changed |= ImGui::DragInt("Max Pulse", &sv.max_pulse_us, 1, 1500, 2500);
                    {
                        int freq = sv.pwm_freq_hz;
                        if (ImGui::DragInt("PWM Frequency", &freq, 1, 40, 400)) {
                            sv.pwm_freq_hz = freq;
                            sv_changed = true;
                        }
                        if (ImGui::IsItemHovered()) ImGui::SetTooltip("PWM signal frequency in Hz (50 Hz = standard hobby servo)");
                    }

                    // Per-servo inversion
                    ImGui::Spacing();
                    ImGui::Separator();
                    ImGui::TextDisabled("Servo Inversion");
                    for (int s = 0; s < 6; s++) {
                        ImGui::PushID(100 + s);
                        char inv_label[16];
                        snprintf(inv_label, sizeof(inv_label), "Invert %d", s);
                        if (ImGui::Checkbox(inv_label, &sv.inverted[s])) sv_changed = true;
                        if (s < 5) ImGui::SameLine();
                        ImGui::PopID();
                    }

                    if (sv_changed && e.serial && e.serial->isOpen()) {
                        // Push changes to firmware
                        char cmd[128];
                        snprintf(cmd, sizeof(cmd), "SERVO:CENTER=%d,%d,%d,%d,%d,%d",
                            sv.center_us[0], sv.center_us[1], sv.center_us[2],
                            sv.center_us[3], sv.center_us[4], sv.center_us[5]);
                        e.serial->sendCommand(cmd);
                        snprintf(cmd, sizeof(cmd), "SERVO:PULSE=%.1f", sv.pulse_per_rad);
                        e.serial->sendCommand(cmd);
                        g_app.log(e.id, "platform", "Servo config pushed to device");
                    }
                }

                // Bit depth (resolution) — common to both platform types
                ImGui::Separator();
                ImGui::Text("Resolution");
                const int bit_opts[] = {8, 10, 12, 14, 16, 18};
                const char* bit_labels[] = {"8-bit", "10-bit", "12-bit", "14-bit", "16-bit", "18-bit"};
                int bit_idx = 2;
                for (int i = 0; i < 6; i++) { if (bit_opts[i] == e.config.bit_depth) bit_idx = i; }
                if (ImGui::Combo("Bit Depth", &bit_idx, bit_labels, 6)) {
                    e.config.bit_depth = bit_opts[bit_idx];
                    g_app.log(e.id, "platform", "Bit depth changed to %d", e.config.bit_depth);
                }

                ImGui::EndTabItem();
            }

            // ════════════════════════════════════════════════════════
            //  Actuator Layout Tab
            // ════════════════════════════════════════════════════════
            if (ImGui::BeginTabItem("Actuator Layout")) {
                // Preset selector
                static const char* presets[] = {"Standard 3-Pair Symmetric", "Custom"};
                static int preset_sel = 0;
                if (ImGui::Combo("Topology Preset", &preset_sel, presets, 2)) {
                    if (preset_sel == 0) {
                        e.config.rebuildPlatform();
                        computeAxisScalesFromGeometry(&e.config.axis_scales, &e.config.geometry, 0.9f);
                        g_app.log(e.id, "platform", "Reset to standard 3-pair symmetric");
                    }
                }
                ImGui::TextDisabled("Custom mode lets you edit each actuator independently.");

                // Top-down visual preview
                ImGui::Separator();
                ImVec2 canvas_sz(ImGui::GetContentRegionAvail().x, 200);
                ImVec2 canvas_p = ImGui::GetCursorScreenPos();
                ImDrawList* dl = ImGui::GetWindowDrawList();

                dl->AddRectFilled(canvas_p, ImVec2(canvas_p.x + canvas_sz.x, canvas_p.y + canvas_sz.y),
                                  IM_COL32(10, 15, 30, 255));
                dl->AddRect(canvas_p, ImVec2(canvas_p.x + canvas_sz.x, canvas_p.y + canvas_sz.y),
                            IM_COL32(60, 60, 60, 255));

                float cx = canvas_p.x + canvas_sz.x * 0.5f;
                float cy = canvas_p.y + canvas_sz.y * 0.5f;

                float max_r = 1.0f;
                for (int k = 0; k < 6; k++) {
                    ActuatorDef& a = e.config.platform.actuators[k];
                    float rb = sqrtf(a.base_pos[0]*a.base_pos[0] + a.base_pos[1]*a.base_pos[1]);
                    float rp = sqrtf(a.plat_pos[0]*a.plat_pos[0] + a.plat_pos[1]*a.plat_pos[1]);
                    if (rb > max_r) max_r = rb;
                    if (rp > max_r) max_r = rp;
                }
                float vis_scale = fminf(canvas_sz.x, canvas_sz.y) * 0.4f / max_r;

                float base_r = e.config.geometry.RD * vis_scale;
                dl->AddCircle(ImVec2(cx, cy), base_r, IM_COL32(60, 80, 100, 120), 48);
                float plat_r = e.config.geometry.PD * vis_scale;
                dl->AddCircle(ImVec2(cx, cy), plat_r, IM_COL32(100, 130, 180, 120), 48);

                ImU32 motor_colors[6] = {
                    IM_COL32(100,180,255,255), IM_COL32(100,220,140,255), IM_COL32(255,200,80,255),
                    IM_COL32(200,140,255,255), IM_COL32(255,120,120,255), IM_COL32(80,220,230,255)
                };
                for (int k = 0; k < 6; k++) {
                    ActuatorDef& a = e.config.platform.actuators[k];
                    float bx = cx + a.base_pos[0] * vis_scale;
                    float by = cy - a.base_pos[1] * vis_scale;
                    float px = cx + a.plat_pos[0] * vis_scale;
                    float py = cy - a.plat_pos[1] * vis_scale;

                    dl->AddLine(ImVec2(bx, by), ImVec2(px, py), motor_colors[k], 1.5f);
                    dl->AddRectFilled(ImVec2(bx-4, by-4), ImVec2(bx+4, by+4), motor_colors[k]);
                    dl->AddCircleFilled(ImVec2(px, py), 3.5f, motor_colors[k]);

                    float arrow_len = 18.0f;
                    float ax_end_x = bx + cosf(a.beta) * arrow_len;
                    float ax_end_y = by - sinf(a.beta) * arrow_len;
                    dl->AddLine(ImVec2(bx, by), ImVec2(ax_end_x, ax_end_y), motor_colors[k], 2.0f);

                    char lbl[8];
                    snprintf(lbl, sizeof(lbl), "M%d", k);
                    dl->AddText(ImVec2(bx + 6, by - 14), motor_colors[k], lbl);
                }

                dl->AddText(ImVec2(canvas_p.x + 4, canvas_p.y + 2), IM_COL32(120,120,120,200), "Top-Down View");
                dl->AddText(ImVec2(canvas_p.x + 4, canvas_p.y + 16), IM_COL32(80,100,130,180), "outer = base, inner = platform");
                ImGui::Dummy(canvas_sz);

                // Per-actuator parameters
                ImGui::Separator();
                bool any_changed = false;

                for (int k = 0; k < 6; k++) {
                    ImGui::PushID(k);
                    ActuatorDef& a = e.config.platform.actuators[k];

                    char hdr[32];
                    snprintf(hdr, sizeof(hdr), "Motor %d", k);
                    ImU32 mc = motor_colors[k];
                    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(
                        ((mc>>0)&0xFF)/255.0f, ((mc>>8)&0xFF)/255.0f, ((mc>>16)&0xFF)/255.0f, 1.0f));

                    bool open = ImGui::CollapsingHeader(hdr, ImGuiTreeNodeFlags_None);
                    ImGui::PopStyleColor();

                    if (open) {
                        ImGui::Indent(12);
                        ImGui::Text("Base Joint (servo shaft)");
                        any_changed |= ImGui::DragFloat("Base X##b", &a.base_pos[0], 0.5f, -1500, 1500, "%.1f mm");
                        any_changed |= ImGui::DragFloat("Base Y##b", &a.base_pos[1], 0.5f, -1500, 1500, "%.1f mm");
                        any_changed |= ImGui::DragFloat("Base Z##b", &a.base_pos[2], 0.5f, -500, 500, "%.1f mm");

                        ImGui::Spacing();
                        ImGui::Text("Platform Joint (ball joint)");
                        any_changed |= ImGui::DragFloat("Plat X##p", &a.plat_pos[0], 0.5f, -1500, 1500, "%.1f mm");
                        any_changed |= ImGui::DragFloat("Plat Y##p", &a.plat_pos[1], 0.5f, -1500, 1500, "%.1f mm");
                        any_changed |= ImGui::DragFloat("Plat Z##p", &a.plat_pos[2], 0.5f, -500, 500, "%.1f mm");

                        ImGui::Spacing();
                        float beta_deg = a.beta * (float)(180.0 / M_PI);
                        if (ImGui::DragFloat("Servo Axis (deg)", &beta_deg, 0.5f, -180, 180, "%.1f\xc2\xb0")) {
                            a.beta = beta_deg * (float)(M_PI / 180.0);
                            any_changed = true;
                        }
                        ImGui::TextDisabled("Direction the servo arm swings in the base x-y plane");

                        ImGui::Spacing();
                        any_changed |= ImGui::DragFloat("L1 (servo arm)", &a.L1, 0.5f, 10, 500, "%.1f mm");
                        any_changed |= ImGui::DragFloat("L2 (connecting rod)", &a.L2, 0.5f, 50, 2000, "%.1f mm");

                        ImGui::Unindent(12);
                    }
                    ImGui::PopID();
                }

                if (any_changed) {
                    preset_sel = 1;
                    g_app.log(e.id, "platform", "Motor layout changed (custom mode)");
                }

                ImGui::EndTabItem();
            }

            // ════════════════════════════════════════════════════════
            //  Measure Tab — Calibration Wizard
            // ════════════════════════════════════════════════════════
            if (ImGui::BeginTabItem("Measure")) {
                static int measure_step = 0;

                struct MeasureStep {
                    const char* title;
                    const char* field;
                    const char* instructions;
                    float* value_ptr;    // set per-entity below
                    float  drag_speed;
                    float  drag_min;
                    float  drag_max;
                    const char* fmt;
                    bool   is_angle;
                    // diagram callback index
                    int    diagram_id;   // 0=RD, 1=PD, 2=L1, 3=L2, 4=Height, 5=ThetaR, 6=ThetaP
                };

                // Build steps pointing at this entity's geometry
                auto& g = e.config.geometry;
                MeasureStep steps[] = {
                    { "Base Radius (RD)",
                      "RD",
                      "Measure from the CENTER of the base plate\n"
                      "to the CENTER of a servo shaft.\n\n"
                      "Use the caliper across the base diameter\n"
                      "and divide by 2, or measure directly from\n"
                      "center to one servo shaft.",
                      &g.RD, 0.25f, 5, 500, "%.2f mm", false, 0 },

                    { "Platform Radius (PD)",
                      "PD",
                      "Measure from the CENTER of the top platform\n"
                      "to the CENTER of a ball joint.\n\n"
                      "Measure across the full platform diameter\n"
                      "through two opposite ball joints, divide by 2,\n"
                      "or measure center to one ball joint directly.",
                      &g.PD, 0.25f, 5, 500, "%.2f mm", false, 1 },

                    { "Servo Arm Length (L1)",
                      "L1",
                      "Measure from the CENTER of the servo shaft\n"
                      "to the CENTER of the ball joint / rod attachment\n"
                      "point at the end of the servo arm.\n\n"
                      "This is the short arm attached to the servo.",
                      &g.ServoArmLengthL1, 0.1f, 1, 200, "%.2f mm", false, 2 },

                    { "Connecting Rod Length (L2)",
                      "L2",
                      "Measure the full length of the connecting rod\n"
                      "from ball joint CENTER to ball joint CENTER.\n\n"
                      "This is the long rod connecting the servo arm\n"
                      "to the platform.",
                      &g.ConnectingArmLengthL2, 0.25f, 5, 500, "%.2f mm", false, 3 },

                    { "Home Height",
                      "Z Home",
                      "With all servos at their neutral/home position,\n"
                      "measure the VERTICAL distance from the top\n"
                      "surface of the base plate to the bottom\n"
                      "surface of the platform.\n\n"
                      "Measure straight up, not along a rod.",
                      &g.platformHeight, 0.25f, 5, 500, "%.2f mm", false, 4 },

                    { "Base Pair Angle (Theta R)",
                      "Theta R",
                      "This is the half-angle between paired servo\n"
                      "shafts on the base. Look at the base from above:\n"
                      "each pair of adjacent servos has a small angle\n"
                      "between them.\n\n"
                      "Measure the angle between two paired servo\n"
                      "shaft axes and divide by 2.\n"
                      "Typical: 5-20 degrees.",
                      &g.theta_r, 0.1f, 0, 60, "%.1f\xc2\xb0", true, 5 },

                    { "Platform Pair Angle (Theta P)",
                      "Theta P",
                      "This is the half-angle between paired ball\n"
                      "joints on the platform. Look from above:\n"
                      "each pair of adjacent ball joints has a small\n"
                      "angle between them.\n\n"
                      "Measure the angle between two paired ball\n"
                      "joint positions and divide by 2.\n"
                      "Typical: 5-30 degrees.",
                      &g.theta_p, 0.1f, 0, 60, "%.1f\xc2\xb0", true, 6 },
                };
                const int num_steps = 7;

                // Step navigation header
                ImGui::TextColored(col, "Step %d / %d", measure_step + 1, num_steps);
                ImGui::SameLine(ImGui::GetContentRegionAvail().x - 160);
                if (measure_step > 0) {
                    if (ImGui::SmallButton("<< Prev")) measure_step--;
                    ImGui::SameLine();
                }
                if (measure_step < num_steps - 1) {
                    if (ImGui::SmallButton("Next >>")) measure_step++;
                }

                ImGui::Separator();

                MeasureStep& s = steps[measure_step];

                // Title
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.9f, 0.5f, 1.0f));
                ImGui::Text("%s", s.title);
                ImGui::PopStyleColor();

                ImGui::Spacing();

                // ── Diagram area ──
                float diag_h = 200.0f;
                ImVec2 diag_sz(ImGui::GetContentRegionAvail().x, diag_h);
                ImVec2 dp = ImGui::GetCursorScreenPos();
                ImDrawList* dl = ImGui::GetWindowDrawList();

                dl->AddRectFilled(dp, ImVec2(dp.x + diag_sz.x, dp.y + diag_sz.y), IM_COL32(8, 12, 24, 255));
                dl->AddRect(dp, ImVec2(dp.x + diag_sz.x, dp.y + diag_sz.y), IM_COL32(50, 50, 60, 255));

                float dcx = dp.x + diag_sz.x * 0.5f;
                float dcy = dp.y + diag_sz.y * 0.5f;

                ImU32 col_base    = IM_COL32(80, 100, 140, 200);
                ImU32 col_plat    = IM_COL32(120, 160, 220, 200);
                ImU32 col_arm     = IM_COL32(180, 140, 80, 200);
                ImU32 col_rod     = IM_COL32(100, 200, 160, 200);
                ImU32 col_measure = IM_COL32(255, 80, 80, 255);
                ImU32 col_dim     = IM_COL32(255, 220, 100, 255);
                ImU32 col_faint   = IM_COL32(60, 70, 90, 150);

                // Helper: draw a dimension line with arrows
                auto drawDimLine = [&](ImVec2 a, ImVec2 b, const char* label, ImU32 c) {
                    dl->AddLine(a, b, c, 2.0f);
                    // Arrowheads
                    float dx = b.x - a.x, dy = b.y - a.y;
                    float len = sqrtf(dx*dx + dy*dy);
                    if (len > 10.0f) {
                        float nx = dx/len, ny = dy/len;
                        float ah = 6.0f;
                        dl->AddTriangleFilled(b,
                            ImVec2(b.x - nx*ah + ny*ah*0.4f, b.y - ny*ah - nx*ah*0.4f),
                            ImVec2(b.x - nx*ah - ny*ah*0.4f, b.y - ny*ah + nx*ah*0.4f), c);
                        dl->AddTriangleFilled(a,
                            ImVec2(a.x + nx*ah + ny*ah*0.4f, a.y + ny*ah - nx*ah*0.4f),
                            ImVec2(a.x + nx*ah - ny*ah*0.4f, a.y + ny*ah + nx*ah*0.4f), c);
                    }
                    ImVec2 mid((a.x+b.x)*0.5f, (a.y+b.y)*0.5f);
                    ImVec2 tsz = ImGui::CalcTextSize(label);
                    dl->AddText(ImVec2(mid.x - tsz.x*0.5f, mid.y - tsz.y - 4), c, label);
                };

                switch (s.diagram_id) {
                case 0: { // RD — top-down view, base radius highlighted
                    float scale = fminf(diag_sz.x, diag_sz.y) * 0.35f / fmaxf(g.RD, g.PD + 1.0f);
                    float br = g.RD * scale;
                    float pr = g.PD * scale;
                    dl->AddCircle(ImVec2(dcx, dcy), br, col_base, 48, 2.0f);
                    dl->AddCircle(ImVec2(dcx, dcy), pr, col_faint, 48, 1.0f);
                    dl->AddCircleFilled(ImVec2(dcx, dcy), 3, IM_COL32(255,255,255,200));
                    // Servo shaft dots on base circle
                    for (int k = 0; k < 6; k++) {
                        float ang = (float)k * (2.0f * (float)M_PI / 6.0f);
                        float sx = dcx + cosf(ang) * br;
                        float sy = dcy - sinf(ang) * br;
                        dl->AddRectFilled(ImVec2(sx-3,sy-3), ImVec2(sx+3,sy+3), col_base);
                    }
                    // Dimension line: center to one servo
                    float ang0 = 0.3f;
                    ImVec2 c0(dcx, dcy);
                    ImVec2 e0(dcx + cosf(ang0)*br, dcy - sinf(ang0)*br);
                    drawDimLine(c0, e0, "RD", col_measure);
                    dl->AddText(ImVec2(dp.x+6, dp.y+4), col_faint, "Top-down view");
                    dl->AddText(ImVec2(dp.x+6, dp.y+18), col_base, "Base circle (servo shafts)");
                    break;
                }
                case 1: { // PD — top-down view, platform radius highlighted
                    float scale = fminf(diag_sz.x, diag_sz.y) * 0.35f / fmaxf(g.RD + 1.0f, g.PD);
                    float br = g.RD * scale;
                    float pr = g.PD * scale;
                    dl->AddCircle(ImVec2(dcx, dcy), br, col_faint, 48, 1.0f);
                    dl->AddCircle(ImVec2(dcx, dcy), pr, col_plat, 48, 2.0f);
                    dl->AddCircleFilled(ImVec2(dcx, dcy), 3, IM_COL32(255,255,255,200));
                    for (int k = 0; k < 6; k++) {
                        float ang = (float)k * (2.0f * (float)M_PI / 6.0f) + 0.3f;
                        float sx = dcx + cosf(ang) * pr;
                        float sy = dcy - sinf(ang) * pr;
                        dl->AddCircleFilled(ImVec2(sx, sy), 3.5f, col_plat);
                    }
                    float ang0 = -0.4f;
                    ImVec2 c0(dcx, dcy);
                    ImVec2 e0(dcx + cosf(ang0)*pr, dcy - sinf(ang0)*pr);
                    drawDimLine(c0, e0, "PD", col_measure);
                    dl->AddText(ImVec2(dp.x+6, dp.y+4), col_faint, "Top-down view");
                    dl->AddText(ImVec2(dp.x+6, dp.y+18), col_plat, "Platform circle (ball joints)");
                    break;
                }
                case 2: { // L1 — side view, servo arm highlighted
                    float base_y = dcy + 60;
                    float plat_y = dcy - 50;
                    float shaft_x = dcx - 80;
                    // Base plate
                    dl->AddLine(ImVec2(dp.x+20, base_y), ImVec2(dp.x+diag_sz.x-20, base_y), col_base, 2.0f);
                    dl->AddText(ImVec2(dp.x+diag_sz.x-80, base_y+4), col_base, "Base");
                    // Platform
                    dl->AddLine(ImVec2(dp.x+60, plat_y), ImVec2(dp.x+diag_sz.x-60, plat_y), col_faint, 1.5f);
                    dl->AddText(ImVec2(dp.x+diag_sz.x-100, plat_y-16), col_faint, "Platform");
                    // Servo body on base
                    dl->AddRectFilled(ImVec2(shaft_x-12, base_y-18), ImVec2(shaft_x+12, base_y), IM_COL32(60,70,90,200));
                    dl->AddCircleFilled(ImVec2(shaft_x, base_y-18), 4, IM_COL32(200,200,200,255));
                    dl->AddText(ImVec2(shaft_x-20, base_y+4), col_faint, "Servo");
                    // Servo arm (L1) — angled up
                    float arm_end_x = shaft_x + g.ServoArmLengthL1 * 2.0f;
                    float arm_end_y = base_y - 50;
                    dl->AddLine(ImVec2(shaft_x, base_y-18), ImVec2(arm_end_x, arm_end_y), col_arm, 3.0f);
                    dl->AddCircleFilled(ImVec2(shaft_x, base_y-18), 3, IM_COL32(255,255,255,200));
                    dl->AddCircleFilled(ImVec2(arm_end_x, arm_end_y), 3, col_arm);
                    // Connecting rod (L2) — faint
                    float rod_end_x = dcx + 60;
                    dl->AddLine(ImVec2(arm_end_x, arm_end_y), ImVec2(rod_end_x, plat_y), col_faint, 1.5f);
                    dl->AddCircleFilled(ImVec2(rod_end_x, plat_y), 3, col_faint);
                    // Dimension: L1
                    drawDimLine(ImVec2(shaft_x-20, base_y-18), ImVec2(arm_end_x-20, arm_end_y), "L1", col_measure);
                    dl->AddText(ImVec2(dp.x+6, dp.y+4), col_faint, "Side view");
                    break;
                }
                case 3: { // L2 — side view, connecting rod highlighted
                    float base_y = dcy + 60;
                    float plat_y = dcy - 50;
                    float shaft_x = dcx - 80;
                    dl->AddLine(ImVec2(dp.x+20, base_y), ImVec2(dp.x+diag_sz.x-20, base_y), col_faint, 1.5f);
                    dl->AddLine(ImVec2(dp.x+60, plat_y), ImVec2(dp.x+diag_sz.x-60, plat_y), col_plat, 1.5f);
                    dl->AddText(ImVec2(dp.x+diag_sz.x-80, base_y+4), col_faint, "Base");
                    dl->AddText(ImVec2(dp.x+diag_sz.x-100, plat_y-16), col_plat, "Platform");
                    // Servo arm faint
                    float arm_end_x = shaft_x + g.ServoArmLengthL1 * 2.0f;
                    float arm_end_y = base_y - 50;
                    dl->AddRectFilled(ImVec2(shaft_x-12, base_y-18), ImVec2(shaft_x+12, base_y), IM_COL32(40,45,55,150));
                    dl->AddLine(ImVec2(shaft_x, base_y-18), ImVec2(arm_end_x, arm_end_y), col_faint, 1.5f);
                    // Connecting rod (L2) — highlighted
                    float rod_end_x = dcx + 60;
                    dl->AddLine(ImVec2(arm_end_x, arm_end_y), ImVec2(rod_end_x, plat_y), col_rod, 3.0f);
                    dl->AddCircleFilled(ImVec2(arm_end_x, arm_end_y), 4, col_rod);
                    dl->AddCircleFilled(ImVec2(rod_end_x, plat_y), 4, col_rod);
                    // Dimension: L2
                    drawDimLine(ImVec2(arm_end_x+15, arm_end_y), ImVec2(rod_end_x+15, plat_y), "L2", col_measure);
                    dl->AddText(ImVec2(dp.x+6, dp.y+4), col_faint, "Side view");
                    break;
                }
                case 4: { // Home Height — side view, vertical distance
                    float base_y = dcy + 60;
                    float plat_y = dcy - 50;
                    dl->AddLine(ImVec2(dp.x+20, base_y), ImVec2(dp.x+diag_sz.x-20, base_y), col_base, 2.0f);
                    dl->AddLine(ImVec2(dp.x+60, plat_y), ImVec2(dp.x+diag_sz.x-60, plat_y), col_plat, 2.0f);
                    dl->AddText(ImVec2(dp.x+diag_sz.x-80, base_y+4), col_base, "Base");
                    dl->AddText(ImVec2(dp.x+diag_sz.x-100, plat_y-16), col_plat, "Platform");
                    // Show a few servos/rods faint
                    for (int k = 0; k < 3; k++) {
                        float sx = dcx - 100 + k * 80;
                        dl->AddRectFilled(ImVec2(sx-6,base_y-12), ImVec2(sx+6,base_y), IM_COL32(40,50,60,120));
                        float ay = base_y - 35;
                        float ax = sx + 25;
                        dl->AddLine(ImVec2(sx, base_y-12), ImVec2(ax, ay), col_faint, 1.0f);
                        dl->AddLine(ImVec2(ax, ay), ImVec2(sx+40, plat_y), col_faint, 1.0f);
                    }
                    // Vertical dimension
                    float dim_x = dp.x + diag_sz.x - 40;
                    drawDimLine(ImVec2(dim_x, plat_y), ImVec2(dim_x, base_y), "Height", col_measure);
                    // Dashed guide lines
                    for (float xx = dim_x - 30; xx < dim_x - 5; xx += 8) {
                        dl->AddLine(ImVec2(xx, plat_y), ImVec2(xx+4, plat_y), col_measure, 1.0f);
                        dl->AddLine(ImVec2(xx, base_y), ImVec2(xx+4, base_y), col_measure, 1.0f);
                    }
                    dl->AddText(ImVec2(dp.x+6, dp.y+4), col_faint, "Side view — measure vertically");
                    break;
                }
                case 5: { // Theta R — top view, angle between base servo pairs
                    float scale = fminf(diag_sz.x, diag_sz.y) * 0.35f / fmaxf(g.RD, 1.0f);
                    float br = g.RD * scale;
                    dl->AddCircle(ImVec2(dcx, dcy), br, col_base, 48, 1.5f);
                    dl->AddCircleFilled(ImVec2(dcx, dcy), 3, IM_COL32(255,255,255,200));
                    // Draw one pair of servos showing the angle
                    float base_ang = (float)M_PI * 0.5f;
                    float half = g.theta_r * (float)(M_PI / 180.0);
                    float a1 = base_ang - half;
                    float a2 = base_ang + half;
                    ImVec2 s1(dcx + cosf(a1)*br, dcy - sinf(a1)*br);
                    ImVec2 s2(dcx + cosf(a2)*br, dcy - sinf(a2)*br);
                    dl->AddLine(ImVec2(dcx, dcy), s1, col_base, 1.5f);
                    dl->AddLine(ImVec2(dcx, dcy), s2, col_base, 1.5f);
                    dl->AddRectFilled(ImVec2(s1.x-4,s1.y-4), ImVec2(s1.x+4,s1.y+4), col_measure);
                    dl->AddRectFilled(ImVec2(s2.x-4,s2.y-4), ImVec2(s2.x+4,s2.y+4), col_measure);
                    // Draw angle arc
                    float arc_r = br * 0.4f;
                    int arc_segs = 20;
                    for (int seg = 0; seg < arc_segs; seg++) {
                        float t0 = a1 + (a2 - a1) * (float)seg / (float)arc_segs;
                        float t1 = a1 + (a2 - a1) * (float)(seg+1) / (float)arc_segs;
                        dl->AddLine(
                            ImVec2(dcx + cosf(t0)*arc_r, dcy - sinf(t0)*arc_r),
                            ImVec2(dcx + cosf(t1)*arc_r, dcy - sinf(t1)*arc_r),
                            col_measure, 2.0f);
                    }
                    // Show other pairs faint
                    for (int p = 1; p < 3; p++) {
                        float off = p * (2.0f * (float)M_PI / 3.0f);
                        ImVec2 ps1(dcx + cosf(a1+off)*br, dcy - sinf(a1+off)*br);
                        ImVec2 ps2(dcx + cosf(a2+off)*br, dcy - sinf(a2+off)*br);
                        dl->AddRectFilled(ImVec2(ps1.x-3,ps1.y-3), ImVec2(ps1.x+3,ps1.y+3), col_faint);
                        dl->AddRectFilled(ImVec2(ps2.x-3,ps2.y-3), ImVec2(ps2.x+3,ps2.y+3), col_faint);
                    }
                    char ang_lbl[32];
                    snprintf(ang_lbl, sizeof(ang_lbl), "Theta R = %.1f\xc2\xb0", g.theta_r);
                    float mid_a = (a1 + a2) * 0.5f;
                    ImVec2 lp(dcx + cosf(mid_a)*(arc_r+16), dcy - sinf(mid_a)*(arc_r+16));
                    dl->AddText(lp, col_measure, ang_lbl);
                    dl->AddText(ImVec2(dp.x+6, dp.y+4), col_faint, "Top-down — angle between servo pair");
                    break;
                }
                case 6: { // Theta P — top view, angle between platform ball joint pairs
                    float scale = fminf(diag_sz.x, diag_sz.y) * 0.35f / fmaxf(g.PD, 1.0f);
                    float pr = g.PD * scale;
                    dl->AddCircle(ImVec2(dcx, dcy), pr, col_plat, 48, 1.5f);
                    dl->AddCircleFilled(ImVec2(dcx, dcy), 3, IM_COL32(255,255,255,200));
                    float base_ang = (float)M_PI * 0.5f;
                    float half = g.theta_p * (float)(M_PI / 180.0);
                    float a1 = base_ang - half;
                    float a2 = base_ang + half;
                    ImVec2 j1(dcx + cosf(a1)*pr, dcy - sinf(a1)*pr);
                    ImVec2 j2(dcx + cosf(a2)*pr, dcy - sinf(a2)*pr);
                    dl->AddLine(ImVec2(dcx, dcy), j1, col_plat, 1.5f);
                    dl->AddLine(ImVec2(dcx, dcy), j2, col_plat, 1.5f);
                    dl->AddCircleFilled(j1, 4, col_measure);
                    dl->AddCircleFilled(j2, 4, col_measure);
                    float arc_r = pr * 0.4f;
                    int arc_segs = 20;
                    for (int seg = 0; seg < arc_segs; seg++) {
                        float t0 = a1 + (a2 - a1) * (float)seg / (float)arc_segs;
                        float t1 = a1 + (a2 - a1) * (float)(seg+1) / (float)arc_segs;
                        dl->AddLine(
                            ImVec2(dcx + cosf(t0)*arc_r, dcy - sinf(t0)*arc_r),
                            ImVec2(dcx + cosf(t1)*arc_r, dcy - sinf(t1)*arc_r),
                            col_measure, 2.0f);
                    }
                    for (int p = 1; p < 3; p++) {
                        float off = p * (2.0f * (float)M_PI / 3.0f);
                        ImVec2 pj1(dcx + cosf(a1+off)*pr, dcy - sinf(a1+off)*pr);
                        ImVec2 pj2(dcx + cosf(a2+off)*pr, dcy - sinf(a2+off)*pr);
                        dl->AddCircleFilled(pj1, 3, col_faint);
                        dl->AddCircleFilled(pj2, 3, col_faint);
                    }
                    char ang_lbl[32];
                    snprintf(ang_lbl, sizeof(ang_lbl), "Theta P = %.1f\xc2\xb0", g.theta_p);
                    float mid_a = (a1 + a2) * 0.5f;
                    ImVec2 lp(dcx + cosf(mid_a)*(arc_r+16), dcy - sinf(mid_a)*(arc_r+16));
                    dl->AddText(lp, col_measure, ang_lbl);
                    dl->AddText(ImVec2(dp.x+6, dp.y+4), col_faint, "Top-down — angle between ball joint pair");
                    break;
                }
                }

                ImGui::Dummy(diag_sz);

                // ── Input field ──
                ImGui::Spacing();
                ImGui::SetNextItemWidth(200);
                char input_label[64];
                snprintf(input_label, sizeof(input_label), "%s##measure_val", s.field);
                bool val_changed = false;
                if (s.is_angle) {
                    float v = *s.value_ptr;
                    if (ImGui::DragFloat(input_label, &v, s.drag_speed, s.drag_min, s.drag_max, s.fmt)) {
                        *s.value_ptr = v;
                        val_changed = true;
                    }
                } else {
                    val_changed = ImGui::DragFloat(input_label, s.value_ptr, s.drag_speed, s.drag_min, s.drag_max, s.fmt);
                }

                // Allow direct keyboard entry
                ImGui::SameLine();
                ImGui::TextDisabled("(Ctrl+click to type)");

                // ── Instructions ──
                ImGui::Spacing();
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.7f, 0.75f, 0.85f, 1.0f));
                ImGui::TextWrapped("%s", s.instructions);
                ImGui::PopStyleColor();

                // ── Apply on every change (live update) ──
                if (val_changed) {
                    e.config.rebuildPlatform();
                    computeAxisScalesFromGeometry(&e.config.axis_scales, &e.config.geometry, 0.9f);
                }

                // ── Bottom navigation + summary ──
                ImGui::Spacing();
                ImGui::Separator();
                ImGui::Spacing();

                // Progress dots
                for (int i = 0; i < num_steps; i++) {
                    if (i > 0) ImGui::SameLine(0, 4);
                    if (i == measure_step)
                        dl->AddCircleFilled(ImGui::GetCursorScreenPos(), 4, col_measure);
                    else
                        dl->AddCircle(ImGui::GetCursorScreenPos(), 4, col_faint, 12, 1.5f);
                    ImGui::Dummy(ImVec2(10, 10));
                }

                // Summary of all current values
                ImGui::SameLine(ImGui::GetContentRegionAvail().x - 280);
                ImGui::TextDisabled("RD:%.1f PD:%.1f L1:%.1f L2:%.1f H:%.1f",
                    g.RD, g.PD, g.ServoArmLengthL1, g.ConnectingArmLengthL2, g.platformHeight);

                if (measure_step == num_steps - 1) {
                    ImGui::Spacing();
                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.65f, 0.40f, 1.0f));
                    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.25f, 0.75f, 0.50f, 1.0f));
                    if (ImGui::Button("Save & Close", ImVec2(140, 0))) {
                        e.config.rebuildPlatform();
                        computeAxisScalesFromGeometry(&e.config.axis_scales, &e.config.geometry, 0.9f);
                        g_app.saveSettings();
                        g_app.log(e.id, "platform", "Calibration complete — geometry saved");
                    }
                    ImGui::PopStyleColor(2);
                    ImGui::SameLine();
                    ImGui::TextDisabled("Workspace: +/-%.0fmm, +/-%.1f\xc2\xb0",
                        e.config.axis_scales.scale[0],
                        e.config.axis_scales.scale[3] * (float)(180.0 / M_PI));
                }

                ImGui::EndTabItem();
            }

            ImGui::EndTabBar();
        }
    }
    ImGui::PopID();
}

// ── Entity Settings Content (drawn inside entity card tab) ──────────

static void DrawEntitySettingsContent(Entity& e) {
    ImGui::PushID(e.id + 1000);
    {
        // Name
        ImGui::InputText("Name", e.name, sizeof(e.name));

        // Entity type
        ImGui::Separator();
        ImGui::Text("Entity Type");
        const char* type_str = (e.type == EntityType::SIL) ? "SIL (Software-in-the-Loop)" : "HIL (Hardware-in-the-Loop)";
        ImGui::TextDisabled("%s", type_str);

        // Color picker
        ImGui::ColorEdit4("Entity Color", e.color, ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_NoLabel);
        ImGui::SameLine();
        ImGui::Text("Entity Color");

        // Pipeline
        ImGui::Separator();
        ImGui::Text("Pipeline");
        ImGui::Checkbox("Enabled##pipe", &e.enabled);

        // Connection (HIL-specific)
        if (e.type == EntityType::HIL) {
            ImGui::Separator();
            ImGui::Text("ESP32 Connection");

            bool connected = e.serial && e.serial->isOpen();

            if (connected) {
                // ── Connected state ──
                ImGui::TextColored(ImVec4(0.2f, 0.83f, 0.6f, 1.0f), "Connected: %s", e.serial->portName());
                ImGui::Text("RX: %d bytes  TX: %d bytes", e.serial->rxBytes(), e.serial->txBytes());
                int rej = e.serial ? e.serial->telemetryRejected() : 0;
                ImGui::Text("Telemetry: %.0f Hz (seq %d)", e.rate_tel_hz, e.hil_tel_seq);
                if (e.serial) {
                    ImGui::Text("COBS: delim=%d ok=%d fail=%d tel=%d resp=%d log=%d",
                        e.serial->m_cobs_delimiters.load(),
                        e.serial->m_cobs_decode_ok.load(),
                        e.serial->m_cobs_decode_fail.load(),
                        e.serial->m_cobs_tel.load(),
                        e.serial->m_cobs_resp.load(),
                        e.serial->m_cobs_log.load());
                }
                if (rej > 0) {
                    ImGui::SameLine();
                    ImGui::TextColored(ImVec4(0.98f, 0.44f, 0.44f, 1.0f), "(%d rejected)", rej);
                }

                // Read-only transport settings (locked while connected)
                ImGui::Spacing();
                ImGui::TextDisabled("TX Rate: %d Hz  |  Bit Depth: %d  |  Protocol: COBS", e.hil_tx_hz, e.config.bit_depth);
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("Disconnect to change TX rate or bit depth.\n"
                        "These are locked during an active session\n"
                        "for transport stability.");
                }

                ImGui::Spacing();
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.85f, 0.20f, 0.20f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.95f, 0.30f, 0.30f, 1.0f));
                if (ImGui::Button("Disconnect", ImVec2(-1, 28))) {
                    g_app.log(e.id, "hil", "Disconnecting from %s", e.serial->portName());
                    e.serial->close();
                    e.serial.reset();
                    e.transport.usb_connected = false;
                    e.hil_auto_connect = false;  // manual disconnect disables auto-reconnect
                }
                ImGui::PopStyleColor(2);

                ImGui::Checkbox("Auto-reconnect", &e.hil_auto_connect);
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("Automatically reconnect if the connection is lost.\n"
                        "Retries every 3 seconds.");
                }

                // Query buttons
                if (ImGui::SmallButton("VERSION?")) { e.serial->sendCommand("VERSION?"); }
                ImGui::SameLine();
                if (ImGui::SmallButton("CONFIG?")) { e.serial->sendCommand("CONFIG?"); }
                ImGui::SameLine();
                if (ImGui::SmallButton("SCALE?")) { e.serial->sendCommand("SCALE?"); }
                ImGui::SameLine();
                if (ImGui::SmallButton("BITS?")) { e.serial->sendCommand("BITS?"); }
                ImGui::SameLine();
                if (ImGui::SmallButton("FINGERPRINT?")) { e.serial->sendCommand("FINGERPRINT?"); }
            } else {
                // ── Disconnected state — COM port selector ──
                if (e.hil_auto_connect && e.hil_port[0] != '\0') {
                    ImGui::TextColored(ImVec4(0.95f, 0.75f, 0.2f, 1.0f), "Searching for %s...", e.hil_port);
                } else {
                    ImGui::TextDisabled("Not connected");
                }
                ImGui::TextWrapped("Select a COM port and configure transport "
                    "settings before connecting. TX rate and bit depth "
                    "are locked during an active session.");

                // Enumerate COM ports (cached, refresh on button)
                static std::vector<ComPortInfo> s_ports;
                static double s_last_enum = 0.0;
                if (s_ports.empty() || ImGui::SmallButton("Refresh##ports")) {
                    s_ports = SerialPort::enumerate();
                    s_last_enum = g_app.frame_time;
                }

                if (s_ports.empty()) {
                    ImGui::TextDisabled("No COM ports found");
                } else {
                    // Port combo
                    int sel_idx = -1;
                    for (int i = 0; i < (int)s_ports.size(); i++) {
                        if (strcmp(e.hil_port, s_ports[i].port.c_str()) == 0) sel_idx = i;
                    }
                    // Auto-select first port if none selected
                    if (sel_idx < 0 && !s_ports.empty()) {
                        snprintf(e.hil_port, sizeof(e.hil_port), "%s", s_ports[0].port.c_str());
                        sel_idx = 0;
                    }

                    const char* preview = sel_idx >= 0 ? s_ports[sel_idx].desc.c_str() : "Select port...";
                    if (ImGui::BeginCombo("Port", preview)) {
                        for (int i = 0; i < (int)s_ports.size(); i++) {
                            bool is_sel = (i == sel_idx);
                            if (ImGui::Selectable(s_ports[i].desc.c_str(), is_sel)) {
                                snprintf(e.hil_port, sizeof(e.hil_port), "%s", s_ports[i].port.c_str());
                            }
                            if (is_sel) ImGui::SetItemDefaultFocus();
                        }
                        ImGui::EndCombo();
                    }

                    ImGui::Spacing();
                    ImGui::Text("Transport Settings");

                    // Baud rate selector
                    {
                        static const int baud_options[] = { 115200, 230400, 460800, 921600 };
                        static const char* baud_labels[] = { "115200", "230400", "460800", "921600" };
                        int baud_idx = 3; // default 921600
                        for (int i = 0; i < 4; i++) {
                            if (e.hil_baud == baud_options[i]) { baud_idx = i; break; }
                        }
                        if (ImGui::Combo("Baud Rate##dc", &baud_idx, baud_labels, 4)) {
                            e.hil_baud = baud_options[baud_idx];
                        }
                        if (ImGui::IsItemHovered()) {
                            ImGui::SetTooltip("Serial baud rate. Must match the ESP32 firmware.\n"
                                "921600 is the default for the Controller firmware.");
                        }
                    }

                    ImGui::TextDisabled("Protocol: COBS (CH_DATA18 + CH_CMD)");
                    if (ImGui::IsItemHovered()) {
                        ImGui::SetTooltip("All serial traffic uses COBS framing.\n"
                            "Motion data: CH_DATA18 (18 bytes: 6x uint24 LE).\n"
                            "Commands: CH_CMD (ASCII text).");
                    }

                    ImGui::SliderInt("TX Rate (Hz)##dc", &e.hil_tx_hz, 10, 1000);
                    if (ImGui::IsItemHovered()) {
                        ImGui::SetTooltip("How often motion packets are sent to the ESP32.\n"
                            "Higher values give smoother motion but more serial traffic.");
                    }
                    ImGui::SliderInt("Bit Depth##dc", &e.config.bit_depth, 8, 18);
                    if (ImGui::IsItemHovered()) {
                        ImGui::SetTooltip("Resolution of motion values sent to the ESP32.\n"
                            "Must match the ESP32 firmware's expected bit depth (8-18).");
                    }

                    ImGui::Checkbox("Auto-reconnect##dc", &e.hil_auto_connect);
                    if (ImGui::IsItemHovered()) {
                        ImGui::SetTooltip("Automatically connect on startup and\n"
                            "reconnect if the connection is lost.\n"
                            "Retries every 3 seconds.");
                    }

                    ImGui::Spacing();
                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.15f, 0.55f, 0.35f, 1.0f));
                    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.2f, 0.7f, 0.45f, 1.0f));
                    if (ImGui::Button("Connect", ImVec2(-1, 28))) {
                        auto sp = std::make_shared<SerialPort>();
                        if (sp->open(e.hil_port, e.hil_baud)) {
                            sp->setCobsMode(true);
                            e.serial = sp;
                            e.transport.usb_connected = true;
                            snprintf(e.transport.usb_port, sizeof(e.transport.usb_port), "%s", e.hil_port);
                            e.hil_tel_seq = 0;
                            e.hil_auto_connect = true;
                            e.hil_handshake_ok = false;
                            e.hil_handshake_pending = true;
                            e.hil_handshake_phase = HandshakePhase::WaitFingerprint;
                            e.hil_device_params.clear();
                            e.hil_handshake_start = g_app.frame_time;
                            e.hil_hs_attempts = 1;
                            e.hil_hs_last_send = g_app.frame_time;
                            e.hil_last_error[0] = '\0';  // clear stale error on new connect
                            snprintf(e.hil_handshake_msg, sizeof(e.hil_handshake_msg),
                                     "Requesting fingerprint...");
                            g_app.log(e.id, "hil", "Connected to %s — handshaking...", e.hil_port);
                            // Flush residual COBS decoder state then send FINGERPRINT? immediately
                            sp->write((const uint8_t*)"\0\0\0\0", 4);
                            sp->sendCommand("FINGERPRINT?");
                        } else {
                            snprintf(e.hil_last_error, sizeof(e.hil_last_error),
                                     "Failed to open %s\nCheck the port is correct and not in use by another program.",
                                     e.hil_port);
                            g_app.log(e.id, "hil", "Failed to open %s", e.hil_port);
                        }
                    }
                    ImGui::PopStyleColor(2);
                }
            }

            // ── Handshake Status (always visible for HIL) ──
            ImGui::Separator();
            {
                auto phase = e.hil_handshake_phase;
                bool connected = e.serial && e.serial->isOpen();

                // Phase indicator with color
                if (phase == HandshakePhase::Ready) {
                    if (e.hil_geo_synced) {
                        ImGui::TextColored(ImVec4(0.2f, 0.83f, 0.6f, 1.0f), "HANDSHAKE OK");
                        ImGui::SameLine();
                        ImGui::TextDisabled("— motion enabled");
                    } else {
                        ImGui::TextColored(ImVec4(0.2f, 0.83f, 0.6f, 1.0f), "HANDSHAKE OK");
                        ImGui::SameLine();
                        ImGui::TextDisabled("— motion enabled");
                        ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.15f, 1.0f),
                            "WARNING: Geometry out of sync with device");
                        ImGui::TextDisabled("Open Geometry Sync below to review and push settings.");
                    }
                } else if (phase == HandshakePhase::Failed) {
                    ImGui::TextColored(ImVec4(0.95f, 0.3f, 0.3f, 1.0f), "HANDSHAKE FAILED");
                    ImGui::SameLine();
                    ImGui::TextDisabled("— motion blocked");
                    ImGui::TextDisabled("%s", e.hil_handshake_msg);
                    ImGui::SameLine();
                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.5f, 0.8f, 1.0f));
                    if (ImGui::SmallButton("Retry Handshake")) {
                        e.hil_handshake_phase = HandshakePhase::WaitFingerprint;
                        e.hil_handshake_pending = true;
                        e.hil_handshake_ok = false;
                        e.hil_hs_attempts = 1;
                        e.hil_hs_last_send = g_app.frame_time;
                        snprintf(e.hil_handshake_msg, sizeof(e.hil_handshake_msg), "Requesting fingerprint...");
                        if (e.serial && e.serial->isOpen()) {
                            e.serial->write((const uint8_t*)"\0\0\0\0\0\0\0\0", 8);
                            e.serial->sendCommand("FINGERPRINT?");
                        }
                    }
                    ImGui::PopStyleColor();
                } else if (connected && phase != HandshakePhase::Idle) {
                    // In-progress phases
                    ImGui::TextColored(ImVec4(0.95f, 0.75f, 0.2f, 1.0f), "Handshaking...");
                    ImGui::SameLine();
                    ImGui::TextDisabled("%s", e.hil_handshake_msg);

                    // Progress bar showing handshake steps
                    int step = 0;
                    if (phase == HandshakePhase::WaitFingerprint) step = 0;
                    else if (phase == HandshakePhase::WaitConfig) step = 1;
                    else if (phase == HandshakePhase::WaitBits) step = 2;
                    else if (phase == HandshakePhase::Validating) step = 3;
                    float progress = (float)(step + 1) / 4.0f;
                    ImGui::PushStyleColor(ImGuiCol_PlotHistogram, ImVec4(0.95f, 0.75f, 0.2f, 0.9f));
                    ImGui::ProgressBar(progress, ImVec2(-1, 14), e.hil_handshake_msg);
                    ImGui::PopStyleColor();

                    // Status details for waiting phases
                    if (phase == HandshakePhase::WaitFingerprint) {
                        bool got_tel = e.serial && e.serial->telemetrySeq() > 0;
                        if (!got_tel && e.hil_hs_attempts == 0) {
                            ImGui::TextColored(ImVec4(0.6f, 0.6f, 0.6f, 1.0f),
                                "Waiting for ESP32 telemetry (proof-of-life)...");
                        } else if (e.hil_hs_attempts > 1) {
                            ImGui::TextColored(ImVec4(0.95f, 0.75f, 0.2f, 1.0f),
                                "Attempt %d/10 — waiting for FINGERPRINT response...",
                                e.hil_hs_attempts);
                        }
                    } else if (phase == HandshakePhase::WaitConfig || phase == HandshakePhase::WaitBits) {
                        ImGui::TextColored(ImVec4(0.6f, 0.6f, 0.6f, 1.0f),
                            "Querying device parameters...");
                    }
                } else {
                    ImGui::TextDisabled("Handshake: idle (not connected)");
                }

                // Device identity
                if (e.hil_fingerprint[0] != '\0') {
                    ImGui::Text("Paired: %s", e.hil_fingerprint);
                    if (e.hil_fw_version[0] != '\0')
                        ImGui::SameLine(), ImGui::TextDisabled("fw %s  proto %d", e.hil_fw_version, e.hil_proto_ver);

                    // ── Geometry Sync — always visible, prominent ──
                    auto& dp = e.hil_device_params;
                    if (dp.config_received) {
                        const auto& geo = e.config.geometry;
                        struct SyncRow { const char* label; float app; float esp; };
                        SyncRow rows[] = {
                            { "RD",      geo.RD,                dp.RD },
                            { "PD",      geo.PD,                dp.PD },
                            { "L1",      geo.ServoArmLengthL1,  dp.L1 },
                            { "L2",      geo.ConnectingArmLengthL2, dp.L2 },
                            { "Height",  geo.platformHeight,    dp.height },
                            { "theta_r", geo.theta_r,           dp.theta_r },
                            { "theta_p", geo.theta_p,           dp.theta_p },
                        };
                        int n_rows = sizeof(rows) / sizeof(rows[0]);

                        bool all_synced = true;
                        for (int i = 0; i < n_rows; i++) {
                            if (fabsf(rows[i].app - rows[i].esp) > 0.05f) { all_synced = false; break; }
                        }

                        ImGui::Spacing();
                        ImGui::Separator();

                        // Colored header bar
                        {
                            ImVec4 bar_col = all_synced ? ImVec4(0.12f, 0.45f, 0.22f, 1.0f) : ImVec4(0.65f, 0.15f, 0.10f, 1.0f);
                            ImGui::PushStyleColor(ImGuiCol_ChildBg, bar_col);
                            ImGui::BeginChild("##sync_hdr", ImVec2(-1, 24), ImGuiChildFlags_None);
                            const char* title = all_synced ? "  GEOMETRY SYNC — ALL MATCHED" : "  GEOMETRY SYNC — OUT OF SYNC";
                            ImGui::SetCursorPosY(3);
                            ImGui::TextUnformatted(title);
                            if (dp.platform_id[0] != '\0') {
                                ImGui::SameLine();
                                ImGui::TextDisabled("(%s)", dp.platform_id);
                            }
                            ImGui::EndChild();
                            ImGui::PopStyleColor();
                        }

                        // Comparison table — always visible
                        if (ImGui::BeginTable("##geo_sync", 4, ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_SizingFixedFit)) {
                            ImGui::TableSetupColumn("Param", ImGuiTableColumnFlags_WidthFixed, 60);
                            ImGui::TableSetupColumn("App",   ImGuiTableColumnFlags_WidthFixed, 70);
                            ImGui::TableSetupColumn("ESP32", ImGuiTableColumnFlags_WidthFixed, 70);
                            ImGui::TableSetupColumn("",      ImGuiTableColumnFlags_WidthFixed, 24);
                            ImGui::TableHeadersRow();

                            for (int i = 0; i < n_rows; i++) {
                                bool match = fabsf(rows[i].app - rows[i].esp) <= 0.05f;
                                ImGui::TableNextRow();
                                ImGui::TableNextColumn(); ImGui::Text("%s", rows[i].label);
                                ImGui::TableNextColumn(); ImGui::Text("%.2f", rows[i].app);
                                ImGui::TableNextColumn();
                                if (!match) ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.4f, 0.3f, 1.0f));
                                ImGui::Text("%.2f", rows[i].esp);
                                if (!match) ImGui::PopStyleColor();
                                ImGui::TableNextColumn();
                                ImGui::TextColored(match ? ImVec4(0.2f, 0.85f, 0.4f, 1.0f) : ImVec4(1.0f, 0.35f, 0.25f, 1.0f),
                                                   match ? "OK" : "X");
                            }

                            // Bit depth row
                            if (dp.bits_received) {
                                bool bits_match = (dp.bit_depth == e.config.bit_depth);
                                ImGui::TableNextRow();
                                ImGui::TableNextColumn(); ImGui::Text("Bits");
                                ImGui::TableNextColumn(); ImGui::Text("%d", e.config.bit_depth);
                                ImGui::TableNextColumn();
                                if (!bits_match) ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.4f, 0.3f, 1.0f));
                                ImGui::Text("%d", dp.bit_depth);
                                if (!bits_match) ImGui::PopStyleColor();
                                ImGui::TableNextColumn();
                                ImGui::TextColored(bits_match ? ImVec4(0.2f, 0.85f, 0.4f, 1.0f) : ImVec4(1.0f, 0.35f, 0.25f, 1.0f),
                                                   bits_match ? "OK" : "X");
                            }

                            ImGui::EndTable();
                        }

                        // Push button (prominent, full-width) or success message
                        if (!all_synced && e.serial && e.serial->isOpen()) {
                            ImGui::Spacing();
                            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.15f, 0.55f, 0.80f, 1.0f));
                            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.20f, 0.65f, 0.90f, 1.0f));
                            ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.10f, 0.45f, 0.70f, 1.0f));
                            if (ImGui::Button("Push Settings to ESP32", ImVec2(-1, 32))) {
                                char cmd[64];
                                e.hil_cmd_queue.clear();
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
                                e.hil_cmd_queue.push_back("CONFIG?");
                                g_app.log(e.id, "hil", "Queued geometry push (8 commands, one per frame)...");
                            }
                            ImGui::PopStyleColor(3);
                            if (ImGui::IsItemHovered()) {
                                ImGui::SetTooltip("Send your app geometry settings to the ESP32.\n"
                                    "This overwrites the device's geometry and recomputes\n"
                                    "axis scales on the firmware side.");
                            }
                        } else if (all_synced) {
                            ImGui::TextColored(ImVec4(0.2f, 0.85f, 0.4f, 0.8f), "All parameters match.");
                        }

                        // Servo info (compact)
                        ImGui::TextDisabled("Servos: [%d,%d,%d,%d,%d,%d]  pulse/rad=%.1f",
                            dp.servo_center[0], dp.servo_center[1], dp.servo_center[2],
                            dp.servo_center[3], dp.servo_center[4], dp.servo_center[5],
                            dp.pulse_per_rad);
                        ImGui::Separator();
                    }

                    if (ImGui::SmallButton("Clear Fingerprint")) {
                        memset(e.hil_fingerprint, 0, sizeof(e.hil_fingerprint));
                        memset(e.hil_fw_version, 0, sizeof(e.hil_fw_version));
                        e.hil_proto_ver = 0;
                        e.hil_handshake_ok = false;
                        e.hil_handshake_phase = HandshakePhase::Idle;
                        e.hil_device_params.clear();
                        g_app.log(e.id, "hil", "Fingerprint cleared — will pair with next device");
                        g_app.settings_dirty = true;
                    }
                    if (ImGui::IsItemHovered()) {
                        ImGui::SetTooltip("Clear the stored device fingerprint.\n"
                            "The next connection will pair with whatever\n"
                            "device responds on the COM port.");
                    }
                } else {
                    ImGui::TextDisabled("No device paired yet");
                    ImGui::TextDisabled("Connect to an ESP32 to auto-pair.");
                }
            }
        }

        // Rates (read-only)
        ImGui::Separator();
        ImGui::Text("Rates");
        if (e.type == EntityType::SIL) {
            ImGui::Text("IK: %.0f Hz", e.rate_ik_hz);
        } else {
            bool hil_connected = e.serial && e.serial->isOpen();
            if (hil_connected) {
                ImGui::Text("TX: %.0f Hz | Telemetry: %.0f Hz", e.rate_tx_hz, e.rate_tel_hz);
            } else {
                ImGui::Text("IK: %.0f Hz (local preview)", e.rate_ik_hz);
            }
        }
    }
    ImGui::PopID();
}

// ── Dynamics Panel (global, single instance with device selector) ────

static void DrawDynamicsPanel() {
    if (!s_show_dynamics) return;

    // Auto-select first entity if none selected or selection invalid
    if (g_app.entities.empty()) {
        ImGui::SetNextWindowSize(ImVec2(780, 860), ImGuiCond_FirstUseEver);
        if (ImGui::Begin("Dynamics###dynamics_global", &s_show_dynamics)) {
            ImGui::TextDisabled("No entities. Add a SIL or HIL entity first.");
        }
        ImGui::End();
        return;
    }
    if (s_selected_dynamics_id < 0 || !g_app.findEntity(s_selected_dynamics_id)) {
        s_selected_dynamics_id = g_app.entities[0].id;
    }
    Entity& e = *g_app.findEntity(s_selected_dynamics_id);

    ImGui::PushID(3000);

    ImGui::SetNextWindowSize(ImVec2(780, 860), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Dynamics###dynamics_global", &s_show_dynamics)) {

        // ── Entity Selector Cards ──
        // Fixed-width cards in a horizontal scrollable strip.
        // Cards are a nominal size, not stretched to fill the row.
        {
            const float card_w = 180.0f;
            const float card_h = 56.0f;
            const float card_gap = 8.0f;
            const float accent_w = 5.0f;
            const float card_pad = 8.0f;   // internal padding
            int n_ents = (int)g_app.entities.size();

            // Horizontal scrollable child region (shows scrollbar if cards exceed width)
            float strip_h = card_h + 16.0f; // card + top/bottom padding + triangle
            ImGui::BeginChild("##dyn_cards_strip", ImVec2(-1, strip_h), false,
                ImGuiWindowFlags_HorizontalScrollbar | ImGuiWindowFlags_NoBackground);
            ImDrawList* dl = ImGui::GetWindowDrawList();
            ImVec2 origin = ImGui::GetCursorScreenPos();
            origin.y += 2.0f; // small top margin

            for (int i = 0; i < n_ents; i++) {
                auto& ent = g_app.entities[i];
                bool is_sel = (ent.id == s_selected_dynamics_id);
                bool is_hil = (ent.type == EntityType::HIL);
                ImVec4 ecol = ColorFromFloat4(ent.color);

                float cx = origin.x + (float)i * (card_w + card_gap);
                float cy = origin.y;
                ImVec2 p0(cx, cy);
                ImVec2 p1(cx + card_w, cy + card_h);

                // Theme colors: SIL = blue tones, HIL = green tones
                ImU32 bg_sel   = is_hil ? IM_COL32(30, 55, 40, 240) : IM_COL32(25, 35, 55, 240);
                ImU32 bg_idle  = IM_COL32(30, 30, 35, 200);
                ImU32 bdr_sel  = is_hil ? IM_COL32(100, 200, 120, 255) : IM_COL32(90, 150, 230, 255);
                ImU32 bdr_idle = IM_COL32(60, 60, 65, 180);

                // Card background
                dl->AddRectFilled(p0, p1, is_sel ? bg_sel : bg_idle, 6.0f);
                dl->AddRect(p0, p1, is_sel ? bdr_sel : bdr_idle, 6.0f, 0, is_sel ? 2.0f : 1.0f);

                // Entity color accent bar (left edge)
                ImU32 accent_col = ImGui::ColorConvertFloat4ToU32(ecol);
                dl->AddRectFilled(ImVec2(cx + 2, cy + 5), ImVec2(cx + accent_w + 1, cy + card_h - 5),
                    accent_col, 2.0f);

                // Type badge (SIL / HIL)
                float badge_x = cx + accent_w + card_pad;
                float badge_y = cy + card_pad;
                const char* type_str = is_hil ? "HIL" : "SIL";
                ImVec2 badge_ts = ImGui::CalcTextSize(type_str);
                float badge_w = badge_ts.x + 10.0f;
                float badge_h = badge_ts.y + 4.0f;
                ImU32 badge_bg = is_hil ? IM_COL32(50, 120, 70, 255) : IM_COL32(50, 80, 140, 255);
                ImU32 badge_bg_dim = is_hil ? IM_COL32(35, 60, 40, 200) : IM_COL32(35, 50, 75, 200);
                dl->AddRectFilled(ImVec2(badge_x, badge_y),
                    ImVec2(badge_x + badge_w, badge_y + badge_h),
                    is_sel ? badge_bg : badge_bg_dim, 3.0f);
                dl->AddText(ImVec2(badge_x + 5.0f, badge_y + 2.0f),
                    is_sel ? IM_COL32(255, 255, 255, 255) : IM_COL32(160, 160, 160, 200),
                    type_str);

                // Entity name (next to badge)
                float name_x = badge_x + badge_w + 6.0f;
                ImU32 name_col = is_sel ? IM_COL32(255, 255, 255, 255) : IM_COL32(140, 140, 140, 200);
                dl->AddText(ImVec2(name_x, badge_y + 1.0f), name_col, ent.name);

                // Status line (smaller, with breathing room below)
                float status_y = badge_y + badge_h + 5.0f;
                char status_buf[64] = "";
                if (is_hil) {
                    bool conn = ent.serial && ent.serial->isOpen();
                    if (conn && ent.hil_handshake_ok)
                        snprintf(status_buf, sizeof(status_buf), "%s (Ready)", ent.transport.usb_port);
                    else if (conn)
                        snprintf(status_buf, sizeof(status_buf), "%s ...", ent.transport.usb_port);
                    else
                        snprintf(status_buf, sizeof(status_buf), "Disconnected");
                } else {
                    snprintf(status_buf, sizeof(status_buf), "Local Engine");
                }
                ImU32 stat_col = is_sel ? IM_COL32(180, 180, 180, 200) : IM_COL32(100, 100, 100, 160);
                ImGui::PushFont(nullptr);
                dl->AddText(ImGui::GetFont(), ImGui::GetFontSize() * 0.85f,
                    ImVec2(badge_x, status_y), stat_col, status_buf);
                ImGui::PopFont();

                // Selected indicator: small triangle at bottom center
                if (is_sel) {
                    float tri_cx = cx + card_w * 0.5f;
                    float tri_y = cy + card_h;
                    dl->AddTriangleFilled(
                        ImVec2(tri_cx - 6, tri_y), ImVec2(tri_cx + 6, tri_y),
                        ImVec2(tri_cx, tri_y + 5), bdr_sel);
                }

                // Invisible button for click detection
                ImGui::SetCursorScreenPos(p0);
                char btn_id[32];
                snprintf(btn_id, sizeof(btn_id), "##dyn_card_%d", ent.id);
                if (ImGui::InvisibleButton(btn_id, ImVec2(card_w, card_h))) {
                    s_selected_dynamics_id = ent.id;
                }
                if (ImGui::IsItemHovered()) {
                    dl->AddRect(p0, p1, IM_COL32(200, 200, 200, 100), 6.0f, 0, 1.5f);
                    ImGui::SetTooltip("Select %s for dynamics editing", ent.name);
                }
            }

            // Set content width so horizontal scroll works
            float total_w = (float)n_ents * (card_w + card_gap) - card_gap;
            ImGui::SetCursorScreenPos(ImVec2(origin.x + total_w, origin.y));
            ImGui::Dummy(ImVec2(0, card_h));

            ImGui::EndChild();
        }
        ImGui::Separator();
        DrawEntityDynamicsContent(e);
    }
    ImGui::End();
    ImGui::PopID();
}

// ── Per-Entity Dynamics Content (used in entity card tab + global panel) ─

static void DrawEntityDynamicsContent(Entity& e) {
        // ── Profile Management Row ────────────────────────────────────
        {
            static char new_preset_name_profile[64] = "";
            static bool show_save_popup_profile = false;
            static bool show_rename_popup = false;

            bool has_preset = (s_dyn_preset_idx >= 0 && s_dyn_preset_idx < (int)g_app.mca_presets.size());
            bool is_user = has_preset && !g_app.mca_presets[s_dyn_preset_idx].is_builtin;

            // ── Row 1: Profile label + combo + management buttons ──
            ImGui::TextColored(ImVec4(0.7f, 0.85f, 1.0f, 1.0f), "Profile:");
            ImGui::SameLine();
            ImGui::PushItemWidth(200);
            const char* preview = has_preset ? g_app.mca_presets[s_dyn_preset_idx].name : "Select Profile...";
            if (ImGui::BeginCombo("##profile_sel", preview)) {
                for (int i = 0; i < (int)g_app.mca_presets.size(); i++) {
                    auto& p = g_app.mca_presets[i];
                    char label[80];
                    snprintf(label, sizeof(label), "%s%s##prof%d", p.name, p.is_builtin ? "" : " *", i);
                    bool selected = (s_dyn_preset_idx == i);
                    if (ImGui::Selectable(label, selected)) {
                        s_dyn_preset_idx = i;
                        auto& stg_ref = s_dyn_staging[e.id];
                        g_app.loadMcaPreset(i, stg_ref.mca, stg_ref.intensity, stg_ref.axis_gain);
                    }
                    if (selected) ImGui::SetItemDefaultFocus();
                    if (ImGui::IsItemHovered()) {
                        ImGui::SetTooltip(p.is_builtin ? "Built-in preset" : "User preset");
                    }
                }
                ImGui::EndCombo();
            }
            ImGui::PopItemWidth();

            // Save (overwrite current)
            if (has_preset) {
                ImGui::SameLine();
                if (ImGui::SmallButton("Save")) {
                    auto& stg_ref = s_dyn_staging[e.id];
                    g_app.saveMcaPreset(g_app.mca_presets[s_dyn_preset_idx].name, stg_ref.mca, stg_ref.intensity, stg_ref.axis_gain);
                }
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("Overwrite '%s' with current settings", g_app.mca_presets[s_dyn_preset_idx].name);
            }

            // Save As
            ImGui::SameLine();
            if (ImGui::SmallButton("Save As")) {
                show_save_popup_profile = true;
                new_preset_name_profile[0] = 0;
            }
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Save current settings as a new profile");

            // Rename (user presets only)
            if (is_user) {
                ImGui::SameLine();
                if (ImGui::SmallButton("Rename")) {
                    show_rename_popup = true;
                    snprintf(new_preset_name_profile, sizeof(new_preset_name_profile), "%s", g_app.mca_presets[s_dyn_preset_idx].name);
                }
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("Rename this user profile");
            }

            // Delete (user presets only)
            if (is_user) {
                ImGui::SameLine();
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.6f, 0.15f, 0.15f, 1.0f));
                if (ImGui::SmallButton("Delete")) {
                    g_app.deleteMcaPreset(s_dyn_preset_idx);
                    s_dyn_preset_idx = -1;
                }
                ImGui::PopStyleColor();
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("Delete this user profile");
            }

            // Restore Default (built-in presets only)
            if (has_preset && g_app.mca_presets[s_dyn_preset_idx].is_builtin) {
                ImGui::SameLine();
                if (ImGui::SmallButton("Restore Default")) {
                    auto& p = g_app.mca_presets[s_dyn_preset_idx];
                    initMotionCueing(&p.mca, 60.0f);
                    setMotionCueingPreset(&p.mca, s_dyn_preset_idx);
                    p.intensity = 100.0f;
                    for (int j = 0; j < 6; j++) p.axis_gain[j] = 100.0f;
                    auto& stg_ref = s_dyn_staging[e.id];
                    g_app.loadMcaPreset(s_dyn_preset_idx, stg_ref.mca, stg_ref.intensity, stg_ref.axis_gain);
                    g_app.saveMcaPresetsToDisk();
                    g_app.log(e.id, "dynamics", "Preset '%s' restored to factory defaults", p.name);
                }
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("Reset to factory default values");
            }

            // ── Profile dirty indicator: staging differs from saved profile ──
            if (has_preset) {
                auto it = s_dyn_staging.find(e.id);
                if (it != s_dyn_staging.end() && it->second.initialized) {
                    auto& stg_ref = it->second;
                    auto& prof = g_app.mca_presets[s_dyn_preset_idx];
                    bool profile_dirty = false;
                    if (stg_ref.intensity != prof.intensity) profile_dirty = true;
                    if (memcmp(stg_ref.axis_gain, prof.axis_gain, sizeof(prof.axis_gain)) != 0) profile_dirty = true;
                    if (stg_ref.mca.enabled != prof.mca.enabled) profile_dirty = true;
                    if (stg_ref.mca.tilt.enabled != prof.mca.tilt.enabled) profile_dirty = true;
                    if (stg_ref.mca.tilt.surge_gain != prof.mca.tilt.surge_gain) profile_dirty = true;
                    if (stg_ref.mca.tilt.sway_gain != prof.mca.tilt.sway_gain) profile_dirty = true;
                    if (stg_ref.mca.tilt.fc != prof.mca.tilt.fc) profile_dirty = true;
                    if (stg_ref.mca.tilt.Q != prof.mca.tilt.Q) profile_dirty = true;
                    for (int i = 0; i < 6 && !profile_dirty; i++) {
                        if (stg_ref.mca.channels[i].hp_enabled != prof.mca.channels[i].hp_enabled) profile_dirty = true;
                        if (stg_ref.mca.channels[i].hp.fc != prof.mca.channels[i].hp.fc) profile_dirty = true;
                        if (stg_ref.mca.channels[i].hp.Q != prof.mca.channels[i].hp.Q) profile_dirty = true;
                        if (stg_ref.mca.channels[i].lp_enabled != prof.mca.channels[i].lp_enabled) profile_dirty = true;
                        if (stg_ref.mca.channels[i].lp.fc != prof.mca.channels[i].lp.fc) profile_dirty = true;
                        if (stg_ref.mca.channels[i].lp.Q != prof.mca.channels[i].lp.Q) profile_dirty = true;
                        if (stg_ref.mca.channels[i].gain != prof.mca.channels[i].gain) profile_dirty = true;
                        if (stg_ref.mca.channels[i].rate_limit != prof.mca.channels[i].rate_limit) profile_dirty = true;
                    }
                    if (profile_dirty) {
                        ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.2f, 1.0f), "Unsaved to profile '%s'", prof.name);
                    }
                }
            }

            // ── Row 2: Copy / Paste (left-aligned) ──
            if (ImGui::SmallButton("Copy")) {
                // Serialize current staging to JSON string for clipboard
                cJSON* root = cJSON_CreateObject();
                cJSON_AddStringToObject(root, "type", "stewart_dynamics");
                cJSON_AddNumberToObject(root, "intensity", e.config.intensity);
                cJSON* gains = cJSON_AddArrayToObject(root, "axis_gain");
                cJSON* inverts = cJSON_AddArrayToObject(root, "axis_invert");
                for (int i = 0; i < 6; i++) {
                    cJSON_AddItemToArray(gains, cJSON_CreateNumber(e.config.axis_gain[i]));
                    cJSON_AddItemToArray(inverts, cJSON_CreateBool(e.config.axis_invert[i]));
                }
                // MCA params
                MotionCueingConfig& mc = e.config.mca;
                cJSON_AddNumberToObject(root, "mca_enabled", mc.enabled);
                cJSON* chs = cJSON_AddArrayToObject(root, "channels");
                for (int i = 0; i < 6; i++) {
                    cJSON* ch = cJSON_CreateObject();
                    cJSON_AddNumberToObject(ch, "hp_enabled", mc.channels[i].hp_enabled);
                    cJSON_AddNumberToObject(ch, "hp_fc", mc.channels[i].hp.fc);
                    cJSON_AddNumberToObject(ch, "hp_Q", mc.channels[i].hp.Q);
                    cJSON_AddNumberToObject(ch, "lp_enabled", mc.channels[i].lp_enabled);
                    cJSON_AddNumberToObject(ch, "lp_fc", mc.channels[i].lp.fc);
                    cJSON_AddNumberToObject(ch, "lp_Q", mc.channels[i].lp.Q);
                    cJSON_AddNumberToObject(ch, "gain", mc.channels[i].gain);
                    cJSON_AddNumberToObject(ch, "rate_limit", mc.channels[i].rate_limit);
                    cJSON_AddItemToArray(chs, ch);
                }
                cJSON* tilt = cJSON_AddObjectToObject(root, "tilt");
                cJSON_AddNumberToObject(tilt, "enabled", mc.tilt.enabled);
                cJSON_AddNumberToObject(tilt, "surge_gain", mc.tilt.surge_gain);
                cJSON_AddNumberToObject(tilt, "sway_gain", mc.tilt.sway_gain);
                cJSON_AddNumberToObject(tilt, "fc", mc.tilt.fc);
                cJSON_AddNumberToObject(tilt, "Q", mc.tilt.Q);
                cJSON_AddNumberToObject(tilt, "hp_enabled", mc.tilt.hp_enabled);
                cJSON_AddNumberToObject(tilt, "hp_fc", mc.tilt.hp_fc);
                cJSON_AddNumberToObject(tilt, "hp_Q", mc.tilt.hp_Q);
                cJSON_AddNumberToObject(tilt, "surge_hp_enabled", mc.tilt.surge_hp_enabled);
                cJSON_AddNumberToObject(tilt, "sway_hp_enabled", mc.tilt.sway_hp_enabled);
                cJSON_AddNumberToObject(tilt, "sway_hp_fc", mc.tilt.sway_hp_fc);
                cJSON_AddNumberToObject(tilt, "sway_hp_Q", mc.tilt.sway_hp_Q);
                cJSON_AddNumberToObject(tilt, "hp_linked", mc.tilt.hp_linked);
                if (has_preset) cJSON_AddStringToObject(root, "profile_name", preview);

                char* str = cJSON_Print(root);
                if (str) {
                    ImGui::SetClipboardText(str);
                    g_app.log(e.id, "dynamics", "Dynamics copied to clipboard");
                    cJSON_free(str);
                }
                cJSON_Delete(root);
            }
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Copy all dynamics settings to clipboard as JSON.\nPaste into another entity or share with others.");

            ImGui::SameLine();
            if (ImGui::SmallButton("Paste")) {
                const char* clip = ImGui::GetClipboardText();
                if (clip && clip[0]) {
                    cJSON* root = cJSON_Parse(clip);
                    if (root) {
                        cJSON* type_val = cJSON_GetObjectItem(root, "type");
                        if (type_val && type_val->valuestring && strcmp(type_val->valuestring, "stewart_dynamics") == 0) {
                            // Parse into staging
                            auto& st = s_dyn_staging[e.id];
                            cJSON* v;
                            if ((v = cJSON_GetObjectItem(root, "intensity"))) st.intensity = (float)v->valuedouble;
                            cJSON* gains_arr = cJSON_GetObjectItem(root, "axis_gain");
                            if (gains_arr && cJSON_IsArray(gains_arr)) {
                                for (int i = 0; i < 6 && i < cJSON_GetArraySize(gains_arr); i++)
                                    st.axis_gain[i] = (float)cJSON_GetArrayItem(gains_arr, i)->valuedouble;
                            }
                            cJSON* inv_arr = cJSON_GetObjectItem(root, "axis_invert");
                            if (inv_arr && cJSON_IsArray(inv_arr)) {
                                for (int i = 0; i < 6 && i < cJSON_GetArraySize(inv_arr); i++)
                                    st.axis_invert[i] = cJSON_IsTrue(cJSON_GetArrayItem(inv_arr, i));
                            }
                            if ((v = cJSON_GetObjectItem(root, "mca_enabled"))) st.mca.enabled = v->valueint;
                            cJSON* chs_arr = cJSON_GetObjectItem(root, "channels");
                            if (chs_arr && cJSON_IsArray(chs_arr)) {
                                for (int i = 0; i < 6 && i < cJSON_GetArraySize(chs_arr); i++) {
                                    cJSON* ch = cJSON_GetArrayItem(chs_arr, i);
                                    if ((v = cJSON_GetObjectItem(ch, "hp_enabled"))) st.mca.channels[i].hp_enabled = v->valueint;
                                    if ((v = cJSON_GetObjectItem(ch, "hp_fc"))) st.mca.channels[i].hp.fc = (float)v->valuedouble;
                                    if ((v = cJSON_GetObjectItem(ch, "hp_Q"))) st.mca.channels[i].hp.Q = (float)v->valuedouble;
                                    if ((v = cJSON_GetObjectItem(ch, "lp_enabled"))) st.mca.channels[i].lp_enabled = v->valueint;
                                    if ((v = cJSON_GetObjectItem(ch, "lp_fc"))) st.mca.channels[i].lp.fc = (float)v->valuedouble;
                                    if ((v = cJSON_GetObjectItem(ch, "lp_Q"))) st.mca.channels[i].lp.Q = (float)v->valuedouble;
                                    if ((v = cJSON_GetObjectItem(ch, "gain"))) st.mca.channels[i].gain = (float)v->valuedouble;
                                    if ((v = cJSON_GetObjectItem(ch, "rate_limit"))) st.mca.channels[i].rate_limit = (float)v->valuedouble;
                                }
                            }
                            cJSON* tilt_obj = cJSON_GetObjectItem(root, "tilt");
                            if (tilt_obj) {
                                if ((v = cJSON_GetObjectItem(tilt_obj, "enabled"))) st.mca.tilt.enabled = v->valueint;
                                if ((v = cJSON_GetObjectItem(tilt_obj, "surge_gain"))) st.mca.tilt.surge_gain = (float)v->valuedouble;
                                if ((v = cJSON_GetObjectItem(tilt_obj, "sway_gain"))) st.mca.tilt.sway_gain = (float)v->valuedouble;
                                if ((v = cJSON_GetObjectItem(tilt_obj, "fc"))) st.mca.tilt.fc = (float)v->valuedouble;
                                if ((v = cJSON_GetObjectItem(tilt_obj, "Q"))) st.mca.tilt.Q = (float)v->valuedouble;
                                if ((v = cJSON_GetObjectItem(tilt_obj, "hp_enabled"))) st.mca.tilt.hp_enabled = v->valueint;
                                if ((v = cJSON_GetObjectItem(tilt_obj, "hp_fc"))) st.mca.tilt.hp_fc = (float)v->valuedouble;
                                if ((v = cJSON_GetObjectItem(tilt_obj, "hp_Q"))) st.mca.tilt.hp_Q = (float)v->valuedouble;
                                if ((v = cJSON_GetObjectItem(tilt_obj, "surge_hp_enabled"))) st.mca.tilt.surge_hp_enabled = v->valueint;
                                if ((v = cJSON_GetObjectItem(tilt_obj, "sway_hp_enabled"))) st.mca.tilt.sway_hp_enabled = v->valueint;
                                if ((v = cJSON_GetObjectItem(tilt_obj, "sway_hp_fc"))) st.mca.tilt.sway_hp_fc = (float)v->valuedouble;
                                if ((v = cJSON_GetObjectItem(tilt_obj, "sway_hp_Q"))) st.mca.tilt.sway_hp_Q = (float)v->valuedouble;
                                if ((v = cJSON_GetObjectItem(tilt_obj, "hp_linked"))) st.mca.tilt.hp_linked = v->valueint;
                            }
                            cJSON* pname = cJSON_GetObjectItem(root, "profile_name");
                            g_app.log(e.id, "dynamics", "Dynamics pasted from clipboard%s%s",
                                pname ? " (from profile: " : "", pname ? pname->valuestring : "");
                            if (pname && pname->valuestring) {
                                g_app.log(e.id, "dynamics", ")");
                            }
                        } else {
                            g_app.log(e.id, "dynamics", "Clipboard does not contain dynamics data");
                        }
                        cJSON_Delete(root);
                    } else {
                        g_app.log(e.id, "dynamics", "Clipboard does not contain valid JSON");
                    }
                }
            }
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Paste dynamics settings from clipboard.\nApply button will commit them to the live pipeline.");

            // ── Save As popup ──
            if (show_save_popup_profile)
                ImGui::OpenPopup("Save As Profile");
            if (ImGui::BeginPopup("Save As Profile")) {
                ImGui::Text("New Profile Name:");
                ImGui::PushItemWidth(250);
                bool enter_pressed = ImGui::InputText("##profname", new_preset_name_profile, sizeof(new_preset_name_profile),
                    ImGuiInputTextFlags_EnterReturnsTrue);
                ImGui::PopItemWidth();
                bool do_save = enter_pressed;
                if (ImGui::Button("Save", ImVec2(120, 0))) do_save = true;
                ImGui::SameLine();
                if (ImGui::Button("Cancel", ImVec2(120, 0))) {
                    show_save_popup_profile = false;
                    ImGui::CloseCurrentPopup();
                }
                if (do_save && new_preset_name_profile[0] != 0) {
                    auto& stg_ref = s_dyn_staging[e.id];
                    g_app.saveMcaPreset(new_preset_name_profile, stg_ref.mca, stg_ref.intensity, stg_ref.axis_gain);
                    for (int i = 0; i < (int)g_app.mca_presets.size(); i++) {
                        if (strcmp(g_app.mca_presets[i].name, new_preset_name_profile) == 0) {
                            s_dyn_preset_idx = i;
                            break;
                        }
                    }
                    show_save_popup_profile = false;
                    ImGui::CloseCurrentPopup();
                }
                ImGui::EndPopup();
            }

            // ── Rename popup ──
            if (show_rename_popup)
                ImGui::OpenPopup("Rename Profile");
            if (ImGui::BeginPopup("Rename Profile")) {
                ImGui::Text("Rename Profile:");
                ImGui::PushItemWidth(250);
                bool enter_pressed = ImGui::InputText("##renname", new_preset_name_profile, sizeof(new_preset_name_profile),
                    ImGuiInputTextFlags_EnterReturnsTrue);
                ImGui::PopItemWidth();
                bool do_rename = enter_pressed;
                if (ImGui::Button("Rename", ImVec2(120, 0))) do_rename = true;
                ImGui::SameLine();
                if (ImGui::Button("Cancel", ImVec2(120, 0))) {
                    show_rename_popup = false;
                    ImGui::CloseCurrentPopup();
                }
                if (do_rename && new_preset_name_profile[0] != 0 && s_dyn_preset_idx >= 0 && s_dyn_preset_idx < (int)g_app.mca_presets.size()) {
                    snprintf(g_app.mca_presets[s_dyn_preset_idx].name, sizeof(g_app.mca_presets[s_dyn_preset_idx].name), "%s", new_preset_name_profile);
                    g_app.saveMcaPresetsToDisk();
                    g_app.log(e.id, "dynamics", "Profile renamed to '%s'", new_preset_name_profile);
                    show_rename_popup = false;
                    ImGui::CloseCurrentPopup();
                }
                ImGui::EndPopup();
            }
        }

        // ── Signal Flow Diagram ──────────────────────────────────────
        // Adapts to entity type:
        //   SIL: all processing local — single "SIL Engine" container
        //   HIL: dynamics on PC, axis scaling + IK + motor control on ESP32
        {
            MotionCueingConfig& mca_ref = e.config.mca;
            float label_row_h = 16.0f;
            float inner_h = 56.0f;
            bool is_hil = (e.type == EntityType::HIL);
            float diagram_h = label_row_h + inner_h + 4.0f + (is_hil ? 22.0f : 0.0f);
            ImVec2 cursor = ImGui::GetCursorScreenPos();
            ImDrawList* dl = ImGui::GetWindowDrawList();
            float avail_w = ImGui::GetContentRegionAvail().x;

            bool mca_on = (mca_ref.enabled != 0);
            bool washout_on = mca_on;
            bool tilt_on = mca_on && (mca_ref.tilt.enabled != 0);

            struct FlowBlock { const char* label; bool enabled; ImU32 col; int sig_idx; };

            // Block definitions (pre-MCA shared, post-MCA adapts to entity type)
            FlowBlock pre_blks[] = {
                { "Input",       true,                                IM_COL32(80,  130, 180, 255), 0 },
                { "Pre-Filter",  e.config.input_filter.enabled != 0,  IM_COL32(100, 140, 100, 255), -1 },
            };
            FlowBlock post_blks[] = {
                { "Gain/Inv",                   true,  IM_COL32(80,  160, 140, 255), 1 },
                { "Scale",                      true,  IM_COL32(140, 130, 70,  255), -1 },
                { "IK",                         true,  IM_COL32(160, 100, 100, 255), -1 },
                { is_hil ? "Motors" : "Output", true,  IM_COL32(100, 100, 160, 255), 2 },
            };

            // Layout constants
            float gap = 5.0f;
            float arrow_w = 14.0f;
            float aw_full = gap + arrow_w + gap;
            // 8 visual slots: Input(0) Pre-Filter(1) MCA(2-3) Gain/Inv(4) Scale(5) IK(6) Output/Motors(7)
            int n_slots = 8;
            int n_arrows = 7;
            float total_gaps = (float)n_arrows * aw_full;
            float slot_w = (avail_w - total_gaps) / (float)n_slots;
            if (slot_w < 30.0f) slot_w = 30.0f;
            float block_h = 24.0f;
            float content_y0 = cursor.y + label_row_h;
            float y_center = content_y0 + inner_h * 0.5f;
            float container_pad = 4.0f;

            // Compute slot X positions
            float block_x[8];
            float x = cursor.x;
            for (int i = 0; i < 2; i++) { block_x[i] = x; x += slot_w + aw_full; }
            block_x[2] = x;
            float mca_w = slot_w * 2.0f + gap;
            x += mca_w + aw_full;
            block_x[3] = block_x[2];
            for (int i = 0; i < 4; i++) {
                block_x[4 + i] = x;
                x += slot_w + ((i < 3) ? aw_full : 0);
            }

            float cont_y0 = cursor.y;
            float cont_y1 = cursor.y + label_row_h + inner_h + 2.0f;

            // ── Draw containers (entity-type dependent) ──
            if (!is_hil) {
                // SIL: single container spanning all blocks
                float c_x0 = block_x[0] - container_pad;
                float c_x1 = block_x[7] + slot_w + container_pad;
                dl->AddRectFilled(ImVec2(c_x0, cont_y0), ImVec2(c_x1, cont_y1),
                    IM_COL32(25, 35, 50, 160), 6.0f);
                dl->AddRect(ImVec2(c_x0, cont_y0), ImVec2(c_x1, cont_y1),
                    IM_COL32(70, 110, 160, 160), 6.0f);
                dl->AddText(ImVec2(c_x0 + 6.0f, cont_y0 + 1.0f),
                    IM_COL32(120, 170, 230, 220), "SIL Engine (Local)");
            } else {
                // HIL: PC container (slots 0-4), ESP32 container (slots 5-7)
                float pc_x0 = block_x[0] - container_pad;
                float pc_x1 = block_x[4] + slot_w + container_pad;
                dl->AddRectFilled(ImVec2(pc_x0, cont_y0), ImVec2(pc_x1, cont_y1),
                    IM_COL32(25, 35, 50, 160), 6.0f);
                dl->AddRect(ImVec2(pc_x0, cont_y0), ImVec2(pc_x1, cont_y1),
                    IM_COL32(70, 110, 160, 160), 6.0f);
                dl->AddText(ImVec2(pc_x0 + 6.0f, cont_y0 + 1.0f),
                    IM_COL32(120, 170, 230, 220), "PC");

                float esp_x0 = block_x[5] - container_pad;
                float esp_x1 = block_x[7] + slot_w + container_pad;
                dl->AddRectFilled(ImVec2(esp_x0, cont_y0), ImVec2(esp_x1, cont_y1),
                    IM_COL32(40, 50, 30, 160), 6.0f);
                dl->AddRect(ImVec2(esp_x0, cont_y0), ImVec2(esp_x1, cont_y1),
                    IM_COL32(100, 160, 70, 160), 6.0f);
                const char* esp_lbl = "ESP32";
                ImVec2 esp_ts = ImGui::CalcTextSize(esp_lbl);
                dl->AddText(ImVec2(esp_x0 + (esp_x1 - esp_x0 - esp_ts.x) * 0.5f, cont_y0 + 1.0f),
                    IM_COL32(140, 200, 100, 220), esp_lbl);
            }

            // ── Helpers ──
            auto drawBlock = [&](float bx, float bw, float by_c, const FlowBlock& blk) {
                ImU32 bg = blk.enabled ? blk.col : IM_COL32(50, 50, 50, 200);
                ImU32 border = blk.enabled ? IM_COL32(200, 200, 200, 180) : IM_COL32(80, 80, 80, 150);
                ImU32 tcol = blk.enabled ? IM_COL32(255, 255, 255, 255) : IM_COL32(120, 120, 120, 200);
                ImVec2 p0(bx, by_c - block_h * 0.5f);
                ImVec2 p1(bx + bw, by_c + block_h * 0.5f);
                dl->AddRectFilled(p0, p1, bg, 4.0f);
                dl->AddRect(p0, p1, border, 4.0f);
                ImVec2 ts = ImGui::CalcTextSize(blk.label);
                dl->AddText(ImVec2(bx + (bw - ts.x) * 0.5f, by_c - ts.y * 0.5f), tcol, blk.label);
                if (blk.sig_idx >= 0) {
                    float max_sig = 0.0f;
                    for (int a = 0; a < 6; a++) {
                        float v = 0;
                        if (blk.sig_idx == 0) v = fabsf(e.state.input_pct[a]);
                        else if (blk.sig_idx == 1) v = fabsf(e.last_scaled_pct[a]);
                        else if (blk.sig_idx == 2) v = e.state.servo_util[a];
                        if (v > max_sig) max_sig = v;
                    }
                    float frac = max_sig / 100.0f;
                    if (frac > 1.0f) frac = 1.0f;
                    float bar_y = p1.y + 2.0f;
                    dl->AddRectFilled(ImVec2(bx, bar_y), ImVec2(bx + bw * frac, bar_y + 3.0f),
                        IM_COL32(100, 200, 100, 180), 1.0f);
                    dl->AddRect(ImVec2(bx, bar_y), ImVec2(bx + bw, bar_y + 3.0f),
                        IM_COL32(60, 60, 60, 120), 1.0f);
                }
            };

            auto drawArrowAt = [&](float ax0, float ax1) {
                ImU32 acol = IM_COL32(140, 140, 140, 200);
                dl->AddLine(ImVec2(ax0, y_center), ImVec2(ax1 - 5, y_center), acol, 1.5f);
                dl->AddTriangleFilled(
                    ImVec2(ax1, y_center),
                    ImVec2(ax1 - 5, y_center - 3),
                    ImVec2(ax1 - 5, y_center + 3), acol);
            };

            // ── Draw inner blocks + arrows ──
            // Pre blocks: Input (slot 0), Pre-Filter (slot 1)
            for (int i = 0; i < 2; i++) {
                drawBlock(block_x[i], slot_w, y_center, pre_blks[i]);
                // Arrow after each pre block
                float a0 = block_x[i] + slot_w + gap;
                float a1 = (i < 1) ? block_x[i + 1] - gap : block_x[2] - gap;
                drawArrowAt(a0, a1);
            }

            // MCA container (nested inside PC)
            {
                float mx0 = block_x[2];
                float mca_y0 = content_y0 + 2.0f;
                float mca_y1 = cont_y1 - 4.0f;

                ImU32 mca_bg = mca_on ? IM_COL32(45, 35, 60, 200) : IM_COL32(35, 35, 38, 180);
                ImU32 mca_border = mca_on ? IM_COL32(140, 120, 180, 180) : IM_COL32(60, 60, 60, 130);
                dl->AddRectFilled(ImVec2(mx0, mca_y0), ImVec2(mx0 + mca_w, mca_y1), mca_bg, 5.0f);
                dl->AddRect(ImVec2(mx0, mca_y0), ImVec2(mx0 + mca_w, mca_y1), mca_border, 5.0f);

                const char* mca_label = "MCA";
                ImVec2 mca_ts = ImGui::CalcTextSize(mca_label);
                ImU32 mca_tcol = mca_on ? IM_COL32(200, 180, 240, 255) : IM_COL32(90, 90, 90, 200);
                dl->AddText(ImVec2(mx0 + (mca_w - mca_ts.x) * 0.5f, mca_y0 + 1.0f), mca_tcol, mca_label);

                float sub_y = mca_y0 + mca_ts.y + 4.0f;
                float sub_h = mca_y1 - sub_y - 3.0f;
                if (sub_h < 14.0f) sub_h = 14.0f;
                float sub_yc = sub_y + sub_h * 0.5f;
                float sub_w = (mca_w - gap * 3) * 0.5f;
                float sub_x1 = mx0 + gap;
                float sub_x2 = sub_x1 + sub_w + gap;

                // Washout sub-block
                {
                    ImU32 bg = washout_on ? IM_COL32(140, 120, 180, 255) : IM_COL32(50, 50, 55, 200);
                    ImU32 bd = washout_on ? IM_COL32(180, 160, 220, 180) : IM_COL32(70, 70, 70, 150);
                    ImU32 tc = washout_on ? IM_COL32(255, 255, 255, 255) : IM_COL32(100, 100, 100, 200);
                    dl->AddRectFilled(ImVec2(sub_x1, sub_y), ImVec2(sub_x1 + sub_w, sub_y + sub_h), bg, 3.0f);
                    dl->AddRect(ImVec2(sub_x1, sub_y), ImVec2(sub_x1 + sub_w, sub_y + sub_h), bd, 3.0f);
                    const char* wl = "Washout";
                    ImVec2 wts = ImGui::CalcTextSize(wl);
                    dl->AddText(ImVec2(sub_x1 + (sub_w - wts.x) * 0.5f, sub_yc - wts.y * 0.5f), tc, wl);
                }
                // Tilt sub-block
                {
                    ImU32 bg = tilt_on ? IM_COL32(180, 140, 80, 255) : IM_COL32(50, 50, 55, 200);
                    ImU32 bd = tilt_on ? IM_COL32(220, 180, 100, 180) : IM_COL32(70, 70, 70, 150);
                    ImU32 tc = tilt_on ? IM_COL32(255, 255, 255, 255) : IM_COL32(100, 100, 100, 200);
                    dl->AddRectFilled(ImVec2(sub_x2, sub_y), ImVec2(sub_x2 + sub_w, sub_y + sub_h), bg, 3.0f);
                    dl->AddRect(ImVec2(sub_x2, sub_y), ImVec2(sub_x2 + sub_w, sub_y + sub_h), bd, 3.0f);
                    const char* tl = "Tilt";
                    ImVec2 tts = ImGui::CalcTextSize(tl);
                    dl->AddText(ImVec2(sub_x2 + (sub_w - tts.x) * 0.5f, sub_yc - tts.y * 0.5f), tc, tl);
                }

                // Arrow after MCA
                float a0 = mx0 + mca_w + gap;
                float a1 = block_x[4] - gap;
                drawArrowAt(a0, a1);
            }

            // ── Post blocks: Gain/Inv(4), Scale(5), IK(6), Output/Motors(7) ──
            for (int i = 0; i < 4; i++) {
                drawBlock(block_x[4 + i], slot_w, y_center, post_blks[i]);
                if (i < 3) {
                    if (is_hil && i == 0) {
                        // USB boundary arrow between Gain/Inv (PC) and Scale (ESP32)
                        float a0 = block_x[4] + slot_w + gap;
                        float a1 = block_x[5] - gap;
                        ImU32 acol = IM_COL32(180, 180, 80, 220);
                        dl->AddLine(ImVec2(a0, y_center), ImVec2(a1 - 5, y_center), acol, 2.0f);
                        dl->AddTriangleFilled(
                            ImVec2(a1, y_center),
                            ImVec2(a1 - 5, y_center - 3),
                            ImVec2(a1 - 5, y_center + 3), acol);
                        const char* usb_lbl = "USB";
                        ImVec2 usb_ts = ImGui::CalcTextSize(usb_lbl);
                        float usb_cx = (a0 + a1) * 0.5f;
                        dl->AddText(ImVec2(usb_cx - usb_ts.x * 0.5f, y_center - usb_ts.y - 3.0f),
                            IM_COL32(180, 180, 80, 180), usb_lbl);
                    } else {
                        float a0 = block_x[4 + i] + slot_w + gap;
                        float a1 = block_x[4 + i + 1] - gap;
                        drawArrowAt(a0, a1);
                    }
                }
            }

            // HIL: telemetry return arrow (dashed, below main flow)
            if (is_hil) {
                float tel_y = cont_y1 + 6.0f;
                float ret_x0 = block_x[7] + slot_w * 0.5f;
                float ret_x1 = block_x[0] + slot_w * 0.5f;
                ImU32 tel_col = IM_COL32(100, 180, 100, 140);
                for (float px = ret_x0; px > ret_x1 + 5; px -= 10.0f) {
                    float px_end = px - 6.0f;
                    if (px_end < ret_x1 + 5) px_end = ret_x1 + 5;
                    dl->AddLine(ImVec2(px, tel_y), ImVec2(px_end, tel_y), tel_col, 1.0f);
                }
                dl->AddTriangleFilled(
                    ImVec2(ret_x1, tel_y),
                    ImVec2(ret_x1 + 5, tel_y - 3),
                    ImVec2(ret_x1 + 5, tel_y + 3), tel_col);
                const char* tel_lbl = "Telemetry";
                ImVec2 tel_ts = ImGui::CalcTextSize(tel_lbl);
                float tel_cx = (ret_x0 + ret_x1) * 0.5f;
                dl->AddText(ImVec2(tel_cx - tel_ts.x * 0.5f, tel_y + 2.0f),
                    IM_COL32(100, 180, 100, 160), tel_lbl);
            }

            ImGui::Dummy(ImVec2(avail_w, diagram_h));
        }
        ImGui::Separator();

        MotionCueingConfig& mca = e.config.mca;
        const char* axis_names[] = {"Surge", "Sway", "Heave", "Roll", "Pitch", "Yaw"};
        const char* axis_feel[]  = {
            "Surge: forward/back push (braking, acceleration)",
            "Sway: side-to-side push (cornering, lane changes)",
            "Heave: up/down (bumps, curbs, elevation changes)",
            "Roll: lean left/right (cornering body roll)",
            "Pitch: tilt nose up/down (braking dive, accel squat)",
            "Yaw: rotate left/right (oversteer, spin, turn-in)"
        };

        // Ensure MCA is initialized before any parameter editing
        if (mca.sample_rate < 1.0f)
            initMotionCueing(&mca, 60.0f);

        // ── Staging buffer: sliders edit this copy, Apply commits to live ──
        auto& stg = s_dyn_staging[e.id];
        if (!stg.initialized) {
            stg.mca = mca;
            stg.input_filter = e.config.input_filter;
            stg.intensity = e.config.intensity;
            memcpy(stg.axis_gain, e.config.axis_gain, sizeof(stg.axis_gain));
            memcpy(stg.axis_invert, e.config.axis_invert, sizeof(stg.axis_invert));
            memcpy(stg.occupant, e.config.occupant, sizeof(stg.occupant));
            stg.initialized = true;
        }

        // Dirty check: compare staging vs live
        bool stg_dirty = false;
        if (stg.intensity != e.config.intensity) stg_dirty = true;
        if (memcmp(stg.axis_gain, e.config.axis_gain, sizeof(stg.axis_gain)) != 0) stg_dirty = true;
        if (memcmp(stg.axis_invert, e.config.axis_invert, sizeof(stg.axis_invert)) != 0) stg_dirty = true;
        if (memcmp(stg.occupant, e.config.occupant, sizeof(stg.occupant)) != 0) stg_dirty = true;
        if (stg.mca.enabled != mca.enabled) stg_dirty = true;
        if (stg.mca.tilt.enabled != mca.tilt.enabled) stg_dirty = true;
        if (stg.mca.tilt.surge_gain != mca.tilt.surge_gain) stg_dirty = true;
        if (stg.mca.tilt.sway_gain != mca.tilt.sway_gain) stg_dirty = true;
        if (stg.mca.tilt.fc != mca.tilt.fc) stg_dirty = true;
        if (stg.mca.tilt.Q != mca.tilt.Q) stg_dirty = true;
        if (stg.mca.tilt.hp_enabled != mca.tilt.hp_enabled) stg_dirty = true;
        if (stg.mca.tilt.hp_fc != mca.tilt.hp_fc) stg_dirty = true;
        if (stg.mca.tilt.hp_Q != mca.tilt.hp_Q) stg_dirty = true;
        if (stg.mca.tilt.surge_hp_enabled != mca.tilt.surge_hp_enabled) stg_dirty = true;
        if (stg.mca.tilt.sway_hp_enabled != mca.tilt.sway_hp_enabled) stg_dirty = true;
        if (stg.mca.tilt.sway_hp_fc != mca.tilt.sway_hp_fc) stg_dirty = true;
        if (stg.mca.tilt.sway_hp_Q != mca.tilt.sway_hp_Q) stg_dirty = true;
        if (stg.mca.tilt.hp_linked != mca.tilt.hp_linked) stg_dirty = true;
        for (int i = 0; i < 6 && !stg_dirty; i++) {
            if (stg.mca.channels[i].hp_enabled != mca.channels[i].hp_enabled) stg_dirty = true;
            if (stg.mca.channels[i].hp.fc != mca.channels[i].hp.fc) stg_dirty = true;
            if (stg.mca.channels[i].hp.Q != mca.channels[i].hp.Q) stg_dirty = true;
            if (stg.mca.channels[i].lp_enabled != mca.channels[i].lp_enabled) stg_dirty = true;
            if (stg.mca.channels[i].lp.fc != mca.channels[i].lp.fc) stg_dirty = true;
            if (stg.mca.channels[i].lp.Q != mca.channels[i].lp.Q) stg_dirty = true;
            if (stg.mca.channels[i].gain != mca.channels[i].gain) stg_dirty = true;
            if (stg.mca.channels[i].rate_limit != mca.channels[i].rate_limit) stg_dirty = true;
        }
        // Input filter dirty check
        if (stg.input_filter.enabled != e.config.input_filter.enabled) stg_dirty = true;
        for (int i = 0; i < 6 && !stg_dirty; i++) {
            if (stg.input_filter.axes[i].lp_enabled != e.config.input_filter.axes[i].lp_enabled) stg_dirty = true;
            if (stg.input_filter.axes[i].lp.fc != e.config.input_filter.axes[i].lp.fc) stg_dirty = true;
            if (stg.input_filter.axes[i].lp.Q != e.config.input_filter.axes[i].lp.Q) stg_dirty = true;
            if (stg.input_filter.axes[i].notch_enabled != e.config.input_filter.axes[i].notch_enabled) stg_dirty = true;
            if (stg.input_filter.axes[i].notch.fc != e.config.input_filter.axes[i].notch.fc) stg_dirty = true;
            if (stg.input_filter.axes[i].notch.Q != e.config.input_filter.axes[i].notch.Q) stg_dirty = true;
        }

        // ═══════════════════════════════════════════════════════════════
        // STICKY HEADER: Apply / Revert (always visible)
        // ═══════════════════════════════════════════════════════════════
        {
            if (stg_dirty) {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.65f, 0.40f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.25f, 0.75f, 0.50f, 1.0f));
                if (ImGui::Button("Apply##dyn_top", ImVec2(120, 0))) {
                    // Snapshot current output for S-curve crossfade (prevents jarring jump)
                    memcpy(e.dyn_transition_from, e.last_scaled_pct, sizeof(e.dyn_transition_from));
                    e.dyn_transition_start = g_app.frame_time;
                    e.dyn_transition_active = true;

                    // Commit staging -> live
                    float sr = mca.sample_rate;  // preserve live sample rate
                    mca = stg.mca;
                    mca.sample_rate = sr;
                    e.config.intensity = stg.intensity;
                    memcpy(e.config.axis_gain, stg.axis_gain, sizeof(e.config.axis_gain));
                    memcpy(e.config.axis_invert, stg.axis_invert, sizeof(e.config.axis_invert));
                    memcpy(e.config.occupant, stg.occupant, sizeof(e.config.occupant));
                    // Commit input filter staging -> live
                    float if_sr = e.config.input_filter.sample_rate;  // preserve live sample rate
                    e.config.input_filter = stg.input_filter;
                    e.config.input_filter.sample_rate = if_sr;
                    // Recalculate input filter biquad coefficients
                    for (int i = 0; i < 6; i++) {
                        InputAxisFilter& ax = e.config.input_filter.axes[i];
                        if (ax.lp_enabled && ax.lp.fc > 0)
                            biquadSetLowpass(&ax.lp, ax.lp.fc, if_sr, ax.lp.Q > 0 ? ax.lp.Q : 0.707f);
                        if (ax.notch_enabled && ax.notch.fc > 0)
                            biquadSetNotch(&ax.notch, ax.notch.fc, if_sr, ax.notch.Q > 0 ? ax.notch.Q : 5.0f);
                    }
                    resetInputFilter(&e.config.input_filter);
                    // Recalculate all MCA biquad coefficients at current sample rate
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
                    if (mca.tilt.surge_hp_enabled && mca.tilt.hp_fc > 0) {
                        biquadSetHighpass(&mca.tilt.surge_hp, mca.tilt.hp_fc, sr, mca.tilt.hp_Q > 0 ? mca.tilt.hp_Q : 0.707f);
                    }
                    if (mca.tilt.sway_hp_enabled && mca.tilt.sway_hp_fc > 0) {
                        biquadSetHighpass(&mca.tilt.sway_hp, mca.tilt.sway_hp_fc, sr, mca.tilt.sway_hp_Q > 0 ? mca.tilt.sway_hp_Q : 0.707f);
                    }
                    resetMotionCueing(&mca);
                    g_app.saveSettings();
                    g_app.log(e.id, "dynamics", "Dynamics settings applied (saved)");
                }
                ImGui::PopStyleColor(2);
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("Commit all staged changes to the live pipeline.\nFilter coefficients will be recalculated.");

                ImGui::SameLine();
                if (ImGui::Button("Revert##dyn_top", ImVec2(120, 0))) {
                    // Discard staging, reload from live
                    stg.mca = mca;
                    stg.input_filter = e.config.input_filter;
                    stg.intensity = e.config.intensity;
                    memcpy(stg.axis_gain, e.config.axis_gain, sizeof(stg.axis_gain));
                    memcpy(stg.axis_invert, e.config.axis_invert, sizeof(stg.axis_invert));
                    memcpy(stg.occupant, e.config.occupant, sizeof(stg.occupant));
                }
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("Discard staged changes and reload from live config.");

                ImGui::SameLine();
                ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.2f, 1.0f), "Unapplied changes");
            } else {
                ImGui::BeginDisabled();
                ImGui::Button("Apply##dyn_top", ImVec2(120, 0));
                ImGui::SameLine();
                ImGui::Button("Revert##dyn_top", ImVec2(120, 0));
                ImGui::EndDisabled();
            }
        }

        // ═══════════════════════════════════════════════════════════════
        // MCA enable + intensity + Reset Filters
        // ═══════════════════════════════════════════════════════════════
        {
            bool enabled = stg.mca.enabled != 0;
            if (ImGui::Checkbox("MCA##en", &enabled)) {
                stg.mca.enabled = enabled ? 1 : 0;
            }
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Master enable for Motion Cueing Algorithm.\nWhen off, raw input passes through unfiltered.");

            ImGui::SameLine();
            ImGui::TextDisabled("%.0f Hz", mca.sample_rate);

            // Reset Filters button (clears biquad memory)
            ImGui::SameLine();
            if (ImGui::SmallButton("Reset Filters")) {
                resetMotionCueing(&mca);
                g_app.log(e.id, "dynamics", "MCA filter state reset");
            }
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Clear all biquad filter memory (stops any ringing).\nDoes not change parameter values.");

            // Intensity slider
            ImGui::PushItemWidth(-1);
            ImGui::SliderFloat("##intensity", &stg.intensity, 0.0f, 150.0f, "Intensity: %.0f%%");
            if (ImGui::IsItemHovered()) ImGui::SetTooltip(
                "Global motion intensity — scales ALL axes equally.\n"
                "100%% = full workspace range.\n"
                "Start at 50%% and increase until comfortable.\n"
                "Feels like: adjusting the overall strength of every motion.");
            ImGui::PopItemWidth();
        }
        ImGui::Separator();

        // ═══════════════════════════════════════════════════════════════
        // SCROLLABLE CONTENT REGION
        // ═══════════════════════════════════════════════════════════════
        ImGui::BeginChild("##dyn_scroll", ImVec2(0, 0), false);

        ImGui::Spacing();
        ImGui::Separator();

        // ═══════════════════════════════════════════════════════════════
        // PER-AXIS GAIN
        // ═══════════════════════════════════════════════════════════════
        ImGui::TextColored(ImVec4(0.7f, 0.85f, 1.0f, 1.0f), "Per-Axis Gain");
        if (ImGui::IsItemHovered()) ImGui::SetTooltip(
            "Individual axis gain trim — fine-tune relative strength of each motion.\n"
            "Applied AFTER MCA filtering, BEFORE inverse kinematics.\n"
            "200%% = double strength, 50%% = half strength.");

        // 2 rows of 3 gains: label + inv checkbox + slider per cell
        if (ImGui::BeginTable("##gain_table", 10, ImGuiTableFlags_SizingStretchSame)) {
            ImGui::TableSetupColumn("l0", ImGuiTableColumnFlags_WidthFixed, 38.0f);
            ImGui::TableSetupColumn("i0", ImGuiTableColumnFlags_WidthFixed, 24.0f);
            ImGui::TableSetupColumn("s0", 0, 1.0f);
            ImGui::TableSetupColumn("l1", ImGuiTableColumnFlags_WidthFixed, 38.0f);
            ImGui::TableSetupColumn("i1", ImGuiTableColumnFlags_WidthFixed, 24.0f);
            ImGui::TableSetupColumn("s1", 0, 1.0f);
            ImGui::TableSetupColumn("l2", ImGuiTableColumnFlags_WidthFixed, 38.0f);
            ImGui::TableSetupColumn("i2", ImGuiTableColumnFlags_WidthFixed, 24.0f);
            ImGui::TableSetupColumn("s2", 0, 1.0f);
            ImGui::TableSetupColumn("btn", ImGuiTableColumnFlags_WidthFixed, 50.0f);

            for (int row = 0; row < 2; row++) {
                ImGui::TableNextRow();
                for (int col = 0; col < 3; col++) {
                    int i = row * 3 + col;
                    ImGui::TableSetColumnIndex(col * 3);
                    ImGui::TextUnformatted(axis_names[i]);
                    ImGui::TableSetColumnIndex(col * 3 + 1);
                    char inv_lbl[32];
                    snprintf(inv_lbl, sizeof(inv_lbl), "##inv_%d", i);
                    ImGui::Checkbox(inv_lbl, &stg.axis_invert[i]);
                    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Invert %s axis (flip sign)", axis_names[i]);
                    ImGui::TableSetColumnIndex(col * 3 + 2);
                    ImGui::PushItemWidth(-1);
                    char lbl[32];
                    snprintf(lbl, sizeof(lbl), "##gain_%d", i);
                    ImGui::SliderFloat(lbl, &stg.axis_gain[i], 0.0f, 200.0f, "%.0f%%");
                    if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", axis_feel[i]);
                    ImGui::PopItemWidth();
                }
                if (row == 0) {
                    ImGui::TableSetColumnIndex(9);
                    if (ImGui::SmallButton("Reset##gains")) {
                        stg.intensity = 100.0f;
                        for (int j = 0; j < 6; j++) { stg.axis_gain[j] = 100.0f; stg.axis_invert[j] = false; }
                    }
                }
            }
            ImGui::EndTable();
        }

        ImGui::Spacing();
        ImGui::Separator();

        // ═══════════════════════════════════════════════════════════════
        // INPUT SIGNAL FILTER (pre-MCA conditioning)
        // ═══════════════════════════════════════════════════════════════
        {
            InputFilterConfig& iflt = stg.input_filter;

            ImGui::TextColored(ImVec4(0.7f, 0.85f, 1.0f, 1.0f), "Input Pre-Filter");
            if (ImGui::IsItemHovered()) ImGui::SetTooltip(
                "Pre-MCA signal conditioning.\n\n"
                "Cleans up noisy or spiky telemetry data BEFORE\n"
                "it reaches the washout stage.\n\n"
                "Pipeline: Input -> [Pre-Filter] -> MCA -> Gain/Inv -> Output");
            bool if_en = iflt.enabled != 0;
            if (ImGui::Checkbox("Enabled##if_en", &if_en))
                iflt.enabled = if_en ? 1 : 0;

            if (iflt.enabled) {
                // Master LP / Notch toggles
                {
                    bool any_lp = false, all_lp = true;
                    bool any_notch = false, all_notch = true;
                    for (int i = 0; i < 6; i++) {
                        if (iflt.axes[i].lp_enabled) any_lp = true; else all_lp = false;
                        if (iflt.axes[i].notch_enabled) any_notch = true; else all_notch = false;
                    }
                    bool lp_m = any_lp;
                    ImGui::PushStyleColor(ImGuiCol_CheckMark,
                        (any_lp && !all_lp) ? ImVec4(0.95f,0.70f,0.30f,1) : ImVec4(0.9f,0.9f,0.9f,1));
                    if (ImGui::Checkbox("Low-Pass##if_lp_all", &lp_m))
                        for (int i = 0; i < 6; i++) iflt.axes[i].lp_enabled = lp_m ? 1 : 0;
                    ImGui::PopStyleColor();
                    if (any_lp && !all_lp) { ImGui::SameLine(); ImGui::TextDisabled("(partial)"); }
                    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Master toggle: LP filter on all axes.\nRemoves high-frequency noise and jitter from input.");

                    ImGui::SameLine(0, 24);
                    bool notch_m = any_notch;
                    ImGui::PushStyleColor(ImGuiCol_CheckMark,
                        (any_notch && !all_notch) ? ImVec4(0.95f,0.70f,0.30f,1) : ImVec4(0.9f,0.9f,0.9f,1));
                    if (ImGui::Checkbox("Notch##if_notch_all", &notch_m))
                        for (int i = 0; i < 6; i++) iflt.axes[i].notch_enabled = notch_m ? 1 : 0;
                    ImGui::PopStyleColor();
                    if (any_notch && !all_notch) { ImGui::SameLine(); ImGui::TextDisabled("(partial)"); }
                    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Master toggle: Notch filter on all axes.\nRemoves a specific resonance frequency from input.");
                }

                // Compact frequency response chart
                {
                    static const int IF_PTS = 128;
                    float sr = iflt.sample_rate > 1.0f ? iflt.sample_rate : 60.0f;
                    float nyquist = sr * 0.5f;
                    float f_min_if = 0.1f;
                    float f_max_if = nyquist > 1.0f ? nyquist : 30.0f;

                    auto bqMagSq = [](const BiquadFilter& f, double w) -> double {
                        double cw = cos(w), sw = sin(w), c2w = cos(2*w), s2w = sin(2*w);
                        double nr = f.b0 + f.b1*cw + f.b2*c2w;
                        double ni = -(f.b1*sw + f.b2*s2w);
                        double dr = 1.0 + f.a1*cw + f.a2*c2w;
                        double di = -(f.a1*sw + f.a2*s2w);
                        double n2 = nr*nr+ni*ni, d2 = dr*dr+di*di;
                        return d2 > 1e-30 ? n2/d2 : 1.0;
                    };

                    static double if_freq[IF_PTS];
                    static double if_db[6][IF_PTS];
                    for (int axis = 0; axis < 6; axis++) {
                        InputAxisFilter& ax = iflt.axes[axis];
                        BiquadFilter lp_tmp = {}, notch_tmp = {};
                        if (ax.lp_enabled && ax.lp.fc > 0)
                            biquadSetLowpass(&lp_tmp, ax.lp.fc, sr, ax.lp.Q > 0 ? ax.lp.Q : 0.707f);
                        if (ax.notch_enabled && ax.notch.fc > 0)
                            biquadSetNotch(&notch_tmp, ax.notch.fc, sr, ax.notch.Q > 0 ? ax.notch.Q : 5.0f);
                        for (int k = 0; k < IF_PTS; k++) {
                            double freq = f_min_if * pow((double)f_max_if/f_min_if, (double)k/(IF_PTS-1));
                            if (axis == 0) if_freq[k] = freq;
                            double w = 2.0*M_PI*freq/(double)sr;
                            double mag2 = 1.0;
                            if (ax.lp_enabled && ax.lp.fc > 0) mag2 *= bqMagSq(lp_tmp, w);
                            if (ax.notch_enabled && ax.notch.fc > 0) mag2 *= bqMagSq(notch_tmp, w);
                            if_db[axis][k] = 10.0*log10(mag2 > 1e-30 ? mag2 : 1e-30);
                        }
                    }

                    static const ImVec4 ax_cols[] = {
                        {0.90f,0.30f,0.30f,1}, {0.30f,0.90f,0.30f,1},
                        {0.30f,0.50f,0.90f,1}, {0.90f,0.90f,0.30f,1},
                        {0.90f,0.30f,0.90f,1}, {0.30f,0.90f,0.90f,1}
                    };
                    static const char* ax_n[] = {"Surge","Sway","Heave","Roll","Pitch","Yaw"};

                    ImGui::Indent(8);
                    if (ImPlot::BeginPlot("##if_response", ImVec2(-16, 160), ImPlotFlags_NoBoxSelect | ImPlotFlags_NoMenus | ImPlotFlags_NoChild)) {
                        ImPlot::SetupAxisScale(ImAxis_X1, ImPlotScale_Log10);
                        ImPlot::SetupAxes("Frequency (Hz)", "dB");
                        ImPlot::SetupAxisLimits(ImAxis_X1, f_min_if, f_max_if, ImPlotCond_Always);
                        ImPlot::SetupAxis(ImAxis_X1, nullptr, ImPlotAxisFlags_Lock | ImPlotAxisFlags_NoMenus);
                        ImPlot::SetupAxisLimits(ImAxis_Y1, -40.0, 6.0, ImPlotCond_Always);
                        ImPlot::SetupAxis(ImAxis_Y1, nullptr, ImPlotAxisFlags_Lock | ImPlotAxisFlags_NoMenus);

                        double zero_x[2] = {f_min_if, f_max_if};
                        double zero_y[2] = {0.0, 0.0};
                        ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(0.5f,0.5f,0.5f,0.3f));
                        ImPlot::PlotLine("##if_zero", zero_x, zero_y, 2);
                        ImPlot::PopStyleColor();

                        // Check if all curves identical
                        bool if_all_same = true;
                        for (int i = 1; i < 6; i++) {
                            if (iflt.axes[i].lp_enabled != iflt.axes[0].lp_enabled ||
                                iflt.axes[i].notch_enabled != iflt.axes[0].notch_enabled ||
                                fabsf(iflt.axes[i].lp.fc - iflt.axes[0].lp.fc) > 0.01f ||
                                fabsf(iflt.axes[i].notch.fc - iflt.axes[0].notch.fc) > 0.01f) {
                                if_all_same = false; break;
                            }
                        }

                        if (if_all_same) {
                            ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(0.9f,0.9f,0.9f,0.9f));
                            ImPlot::PlotLine("All Axes", if_freq, if_db[0], IF_PTS);
                            ImPlot::PopStyleColor();
                        } else {
                            for (int a = 0; a < 6; a++) {
                                ImPlot::PushStyleColor(ImPlotCol_Line, ax_cols[a]);
                                ImPlot::PlotLine(ax_n[a], if_freq, if_db[a], IF_PTS);
                                ImPlot::PopStyleColor();
                            }
                        }
                        ImPlot::EndPlot();
                    }
                    ImGui::Unindent(8);
                }

                // Per-axis table
                int if_tflags = ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingFixedFit;
                if (ImGui::BeginTable("##if_table", 7, if_tflags)) {
                    ImGui::TableSetupColumn("Axis",      ImGuiTableColumnFlags_WidthFixed, 48.0f);
                    ImGui::TableSetupColumn("LP",        ImGuiTableColumnFlags_WidthFixed, 28.0f);
                    ImGui::TableSetupColumn("LP fc",     ImGuiTableColumnFlags_WidthFixed, 82.0f);
                    ImGui::TableSetupColumn("LP Q",      ImGuiTableColumnFlags_WidthFixed, 68.0f);
                    ImGui::TableSetupColumn("Notch",     ImGuiTableColumnFlags_WidthFixed, 38.0f);
                    ImGui::TableSetupColumn("Notch fc",  ImGuiTableColumnFlags_WidthFixed, 82.0f);
                    ImGui::TableSetupColumn("Notch Q",   ImGuiTableColumnFlags_WidthFixed, 68.0f);

                    ImGui::TableNextRow(ImGuiTableRowFlags_Headers);
                    ImGui::TableSetColumnIndex(0); ImGui::Text("Axis");
                    ImGui::TableSetColumnIndex(1); ImGui::Text("LP");
                    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Low-pass: removes high-frequency noise.\nSet cutoff below the noise floor.");
                    ImGui::TableSetColumnIndex(2); ImGui::Text("LP fc");
                    if (ImGui::IsItemHovered()) ImGui::SetTooltip("LP cutoff frequency (Hz).\n5-15 Hz = heavy smoothing\n20-30 Hz = light cleanup");
                    ImGui::TableSetColumnIndex(3); ImGui::Text("LP Q");
                    if (ImGui::IsItemHovered()) ImGui::SetTooltip("LP quality factor.\n0.707 = Butterworth (clean)\nHigher = sharper rolloff with ringing");
                    ImGui::TableSetColumnIndex(4); ImGui::Text("Notch");
                    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Notch filter: removes a specific frequency.\nUseful for engine vibration, FFB resonance, etc.");
                    ImGui::TableSetColumnIndex(5); ImGui::Text("Notch fc");
                    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Notch center frequency (Hz).\nThe exact frequency to eliminate.");
                    ImGui::TableSetColumnIndex(6); ImGui::Text("Notch Q");
                    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Notch Q factor (width).\nHigher = narrower notch (more surgical)\nLower = wider notch (removes more)");

                    for (int i = 0; i < 6; i++) {
                        InputAxisFilter& ax = iflt.axes[i];
                        ImGui::PushID(i + 600);
                        ImGui::TableNextRow();

                        ImGui::TableSetColumnIndex(0);
                        ImGui::TextUnformatted(axis_names[i]);

                        ImGui::TableSetColumnIndex(1);
                        bool lp_on = ax.lp_enabled != 0;
                        if (ImGui::Checkbox("##iflp", &lp_on)) {
                            ax.lp_enabled = lp_on ? 1 : 0;
                            if (lp_on && ax.lp.fc > 0)
                                biquadSetLowpass(&ax.lp, ax.lp.fc, iflt.sample_rate, ax.lp.Q > 0 ? ax.lp.Q : 0.707f);
                        }

                        ImGui::TableSetColumnIndex(2);
                        ImGui::PushItemWidth(-1);
                        if (ImGui::DragFloat("##iflpfc", &ax.lp.fc, 0.1f, 0.5f, 30.0f, "%.1f Hz")) {
                            if (ax.lp_enabled && ax.lp.fc > 0)
                                biquadSetLowpass(&ax.lp, ax.lp.fc, iflt.sample_rate, ax.lp.Q > 0 ? ax.lp.Q : 0.707f);
                        }
                        ImGui::PopItemWidth();

                        ImGui::TableSetColumnIndex(3);
                        ImGui::PushItemWidth(-1);
                        if (ImGui::DragFloat("##iflpq", &ax.lp.Q, 0.01f, 0.1f, 5.0f, "%.2f")) {
                            if (ax.lp_enabled && ax.lp.fc > 0)
                                biquadSetLowpass(&ax.lp, ax.lp.fc, iflt.sample_rate, ax.lp.Q);
                        }
                        ImGui::PopItemWidth();

                        ImGui::TableSetColumnIndex(4);
                        bool notch_on = ax.notch_enabled != 0;
                        if (ImGui::Checkbox("##ifnotch", &notch_on)) {
                            ax.notch_enabled = notch_on ? 1 : 0;
                            if (notch_on && ax.notch.fc > 0)
                                biquadSetNotch(&ax.notch, ax.notch.fc, iflt.sample_rate, ax.notch.Q > 0 ? ax.notch.Q : 5.0f);
                        }

                        ImGui::TableSetColumnIndex(5);
                        ImGui::PushItemWidth(-1);
                        if (ImGui::DragFloat("##ifnfc", &ax.notch.fc, 0.1f, 0.5f, 30.0f, "%.1f Hz")) {
                            if (ax.notch_enabled && ax.notch.fc > 0)
                                biquadSetNotch(&ax.notch, ax.notch.fc, iflt.sample_rate, ax.notch.Q > 0 ? ax.notch.Q : 5.0f);
                        }
                        ImGui::PopItemWidth();

                        ImGui::TableSetColumnIndex(6);
                        ImGui::PushItemWidth(-1);
                        if (ImGui::DragFloat("##ifnq", &ax.notch.Q, 0.1f, 0.5f, 30.0f, "%.1f")) {
                            if (ax.notch_enabled && ax.notch.fc > 0)
                                biquadSetNotch(&ax.notch, ax.notch.fc, iflt.sample_rate, ax.notch.Q);
                        }
                        ImGui::PopItemWidth();

                        ImGui::PopID();
                    }
                    ImGui::EndTable();
                }
            }
        }

        ImGui::Spacing();
        ImGui::Separator();

        // ═══════════════════════════════════════════════════════════════
        // PER-AXIS FILTER TABLE
        // ═══════════════════════════════════════════════════════════════
        ImGui::TextColored(ImVec4(0.7f, 0.85f, 1.0f, 1.0f), "Washout & Smoothing");
        if (ImGui::IsItemHovered()) ImGui::SetTooltip(
            "Controls how the platform returns to center and how smooth the output is.\n\n"
            "  HP Washout = Return-to-Center Speed\n"
            "    Lower fc -> slow, gradual return (preserves more motion)\n"
            "    Higher fc -> fast, snappy return (only transients get through)\n\n"
            "  LP Smoothing = Output Smoothness\n"
            "    Lower fc -> smoother, less detail\n"
            "    Higher fc -> more detail, less smoothing\n\n"
            "Pipeline: Input -> HP Washout -> LP Smoothing -> Gain -> Rate Limit -> Output");

        // Master toggles for Washout (HP) and Smoothing (LP)
        {
            bool any_hp = false, all_hp = true;
            bool any_lp = false, all_lp = true;
            for (int i = 0; i < 6; i++) {
                if (stg.mca.channels[i].hp_enabled) any_hp = true; else all_hp = false;
                if (stg.mca.channels[i].lp_enabled) any_lp = true; else all_lp = false;
            }

            // Washout master checkbox (tri-state via mixed value)
            bool hp_master = any_hp;
            ImGui::PushStyleColor(ImGuiCol_CheckMark,
                (any_hp && !all_hp) ? ImVec4(0.95f, 0.70f, 0.30f, 1.0f) : ImVec4(0.90f, 0.90f, 0.90f, 1.0f));
            if (ImGui::Checkbox("Washout (HP)##master_hp", &hp_master)) {
                for (int i = 0; i < 6; i++) stg.mca.channels[i].hp_enabled = hp_master ? 1 : 0;
            }
            ImGui::PopStyleColor();
            if (any_hp && !all_hp) {
                ImGui::SameLine(); ImGui::TextDisabled("(partial)");
            }
            if (ImGui::IsItemHovered()) ImGui::SetTooltip(
                "Master toggle for Washout (return-to-center) on all axes.\n\n"
                "ON = platform gradually returns to center after each move.\n"
                "     The 'HP fc' column controls HOW FAST it returns.\n"
                "OFF = raw input passes through (may hit travel limits).\n\n"
                "Individual axes can still be toggled in the table below.");

            ImGui::SameLine(0, 24);

            // Smoothing master checkbox
            bool lp_master = any_lp;
            ImGui::PushStyleColor(ImGuiCol_CheckMark,
                (any_lp && !all_lp) ? ImVec4(0.95f, 0.70f, 0.30f, 1.0f) : ImVec4(0.90f, 0.90f, 0.90f, 1.0f));
            if (ImGui::Checkbox("Smoothing (LP)##master_lp", &lp_master)) {
                for (int i = 0; i < 6; i++) stg.mca.channels[i].lp_enabled = lp_master ? 1 : 0;
            }
            ImGui::PopStyleColor();
            if (any_lp && !all_lp) {
                ImGui::SameLine(); ImGui::TextDisabled("(partial)");
            }
            if (ImGui::IsItemHovered()) ImGui::SetTooltip(
                "Master toggle for Low-Pass smoothing on all axes.\n\n"
                "ON = high-frequency noise and jitter are removed.\n"
                "OFF = unfiltered output, may feel harsh or buzzy.\n\n"
                "Individual axes can still be toggled in the table below.");
        }

        ImGui::TextDisabled("Input -> Washout (return speed) -> Smoothing -> Gain -> Rate Limit -> Output");

        // ── EQ-style frequency response graph ──
        {
            static const int EQ_POINTS = 256;
            static const ImVec4 axis_colors[] = {
                {0.90f, 0.30f, 0.30f, 1.0f}, {0.30f, 0.90f, 0.30f, 1.0f},
                {0.30f, 0.50f, 0.90f, 1.0f}, {0.90f, 0.90f, 0.30f, 1.0f},
                {0.90f, 0.30f, 0.90f, 1.0f}, {0.30f, 0.90f, 0.90f, 1.0f}
            };
            float sr = stg.mca.sample_rate > 1.0f ? stg.mca.sample_rate : 60.0f;
            float nyquist = sr * 0.5f;
            float f_min = 0.05f;
            float f_max = nyquist > 1.0f ? nyquist : 30.0f;

            // Biquad magnitude: |H(e^jw)|^2 from coefficients
            auto biquadMagSq = [](const BiquadFilter& f, double w) -> double {
                double cw  = cos(w),  sw  = sin(w);
                double c2w = cos(2*w), s2w = sin(2*w);
                double nr = f.b0 + f.b1 * cw + f.b2 * c2w;
                double ni = -(f.b1 * sw + f.b2 * s2w);
                double dr = 1.0 + f.a1 * cw + f.a2 * c2w;
                double di = -(f.a1 * sw + f.a2 * s2w);
                double num2 = nr*nr + ni*ni;
                double den2 = dr*dr + di*di;
                return den2 > 1e-30 ? num2 / den2 : 1.0;
            };

            // ── Detect if all axes share the same filter config (collapse to 1 curve) ──
            bool all_curves_same = true;
            bool all_hp_same = true;   // all enabled HP fc match
            bool all_lp_same = true;   // all enabled LP fc match
            int  hp_enabled_count = 0, lp_enabled_count = 0;
            float hp_ref = 0, lp_ref = 0;
            for (int i = 0; i < 6; i++) {
                const AxisChannelFilter& ch = stg.mca.channels[i];
                if (ch.hp_enabled) { if (hp_enabled_count == 0) hp_ref = ch.hp.fc; hp_enabled_count++; }
                if (ch.lp_enabled) { if (lp_enabled_count == 0) lp_ref = ch.lp.fc; lp_enabled_count++; }
            }
            for (int i = 0; i < 6; i++) {
                const AxisChannelFilter& c0 = stg.mca.channels[0];
                const AxisChannelFilter& ci = stg.mca.channels[i];
                if (ci.hp_enabled != c0.hp_enabled || ci.lp_enabled != c0.lp_enabled ||
                    ci.gain != c0.gain ||
                    fabsf(ci.hp.fc - c0.hp.fc) > 0.01f || fabsf(ci.hp.Q - c0.hp.Q) > 0.01f ||
                    fabsf(ci.lp.fc - c0.lp.fc) > 0.01f || fabsf(ci.lp.Q - c0.lp.Q) > 0.01f)
                    all_curves_same = false;
                if (ci.hp_enabled && fabsf(ci.hp.fc - hp_ref) > 0.01f) all_hp_same = false;
                if (ci.lp_enabled && fabsf(ci.lp.fc - lp_ref) > 0.01f) all_lp_same = false;
            }

            // Compute frequency response curves
            static double eq_freq[EQ_POINTS];
            static double eq_db[6][EQ_POINTS];
            for (int axis = 0; axis < 6; axis++) {
                const AxisChannelFilter& ch = stg.mca.channels[axis];
                BiquadFilter hp_tmp = {}, lp_tmp = {};
                if (ch.hp_enabled && ch.hp.fc > 0)
                    biquadSetHighpass(&hp_tmp, ch.hp.fc, sr, ch.hp.Q > 0 ? ch.hp.Q : 0.707f);
                if (ch.lp_enabled && ch.lp.fc > 0)
                    biquadSetLowpass(&lp_tmp, ch.lp.fc, sr, ch.lp.Q > 0 ? ch.lp.Q : 0.707f);
                for (int k = 0; k < EQ_POINTS; k++) {
                    double freq = f_min * pow((double)f_max / f_min, (double)k / (EQ_POINTS - 1));
                    if (axis == 0) eq_freq[k] = freq;
                    double w = 2.0 * M_PI * freq / (double)sr;
                    double mag2 = 1.0;
                    if (ch.hp_enabled && ch.hp.fc > 0) mag2 *= biquadMagSq(hp_tmp, w);
                    if (ch.lp_enabled && ch.lp.fc > 0) mag2 *= biquadMagSq(lp_tmp, w);
                    mag2 *= (double)(ch.gain * ch.gain);
                    eq_db[axis][k] = 10.0 * log10(mag2 > 1e-30 ? mag2 : 1e-30);
                }
            }

            // ── Drag handle state (synced from staging each frame) ──
            static double hp_drag[6], lp_drag[6];
            static double hp_drag_all = 0, lp_drag_all = 0;
            for (int i = 0; i < 6; i++) {
                hp_drag[i] = (double)stg.mca.channels[i].hp.fc;
                lp_drag[i] = (double)stg.mca.channels[i].lp.fc;
            }
            if (hp_enabled_count > 0 && all_hp_same) hp_drag_all = hp_ref;
            if (lp_enabled_count > 0 && all_lp_same) lp_drag_all = lp_ref;

            ImGui::Indent(8);
            if (ImPlot::BeginPlot("##eq_response", ImVec2(-16, 220), ImPlotFlags_NoBoxSelect | ImPlotFlags_NoMenus | ImPlotFlags_NoChild)) {
                ImPlot::SetupAxisScale(ImAxis_X1, ImPlotScale_Log10);
                ImPlot::SetupAxes("Frequency (Hz)", "dB");
                ImPlot::SetupAxisLimits(ImAxis_X1, f_min, f_max, ImPlotCond_Always);
                ImPlot::SetupAxis(ImAxis_X1, nullptr, ImPlotAxisFlags_Lock | ImPlotAxisFlags_NoMenus);
                ImPlot::SetupAxisLimits(ImAxis_Y1, -40.0, 12.0, ImPlotCond_Always);
                ImPlot::SetupAxis(ImAxis_Y1, nullptr, ImPlotAxisFlags_Lock | ImPlotAxisFlags_NoMenus);

                // 0 dB reference line
                double zero_x[2] = {f_min, f_max};
                double zero_y[2] = {0.0, 0.0};
                ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(0.5f, 0.5f, 0.5f, 0.3f));
                ImPlot::PlotLine("##zero", zero_x, zero_y, 2);
                ImPlot::PopStyleColor();

                // Legend visibility cache (from previous frame, so spectrum
                // draws behind curves but respects legend toggling)
                static bool s_axis_vis[6] = {true, true, true, true, true, true};
                static bool s_all_vis = true;

                // ── Live input spectrum overlay (shaded, behind filter curves) ──
                float spec_fmax = g_app.input_spectrum_freq_max;
                if (spec_fmax > 0.5f) {
                    static const int SPEC_PTS = 128;
                    static double spec_freq[SPEC_PTS];
                    static double spec_db[6][SPEC_PTS];
                    static double spec_floor[SPEC_PTS];

                    float peak_mag = 0.0f;
                    for (int a = 0; a < 6; a++)
                        for (int k = 1; k < SPECTRUM_BINS; k++)
                            if (g_app.input_spectrum[a][k] > peak_mag)
                                peak_mag = g_app.input_spectrum[a][k];
                    double norm = peak_mag > 1e-6f ? (double)peak_mag : 1.0;

                    for (int k = 0; k < SPEC_PTS; k++) {
                        double freq = f_min * pow((double)f_max / f_min, (double)k / (SPEC_PTS - 1));
                        spec_freq[k] = freq;
                        spec_floor[k] = -40.0;
                        double bin_f = freq / (double)spec_fmax * (double)SPECTRUM_BINS;
                        int b0 = (int)bin_f;
                        float frac = (float)(bin_f - b0);
                        if (b0 < 1) b0 = 1;
                        if (b0 >= SPECTRUM_BINS - 1) { b0 = SPECTRUM_BINS - 2; frac = 1.0f; }
                        for (int a = 0; a < 6; a++) {
                            float mag = g_app.input_spectrum[a][b0] * (1.0f - frac)
                                      + g_app.input_spectrum[a][b0 + 1] * frac;
                            double db = 20.0 * log10((double)mag / norm + 1e-10);
                            if (db < -40.0) db = -40.0;
                            spec_db[a][k] = db;
                        }
                    }

                    for (int axis = 0; axis < 6; axis++) {
                        // Skip axes hidden in the legend
                        bool vis = all_curves_same ? s_all_vis : s_axis_vis[axis];
                        if (!vis) continue;

                        bool has_energy = false;
                        for (int k = 1; k < SPECTRUM_BINS && !has_energy; k++)
                            if (g_app.input_spectrum[axis][k] > peak_mag * 0.001f) has_energy = true;
                        if (!has_energy) continue;
                        ImVec4 sc = axis_colors[axis]; sc.w = 0.12f;
                        ImPlot::PushStyleColor(ImPlotCol_Fill, sc);
                        sc.w = 0.30f;
                        ImPlot::PushStyleColor(ImPlotCol_Line, sc);
                        char slbl[32]; snprintf(slbl, sizeof(slbl), "##spec%d", axis);
                        ImPlot::PlotShaded(slbl, spec_freq, spec_db[axis], spec_floor, SPEC_PTS);
                        ImPlot::PopStyleColor(2);
                    }
                }

                // ── Filter response curves ──
                if (all_curves_same) {
                    // All axes identical — show single white curve
                    ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(0.95f, 0.95f, 0.95f, 1.0f));
                    ImPlot::SetNextLineStyle(IMPLOT_AUTO_COL, 2.0f);
                    ImPlot::PlotLine("All Axes", eq_freq, eq_db[0], EQ_POINTS);
                    ImPlot::PopStyleColor();
                    // Update visibility cache from legend
                    ImPlotItem* item = ImPlot::GetItem("All Axes");
                    s_all_vis = item ? item->Show : true;
                } else {
                    for (int axis = 0; axis < 6; axis++) {
                        ImPlot::PushStyleColor(ImPlotCol_Line, axis_colors[axis]);
                        ImPlot::PlotLine(axis_names[axis], eq_freq, eq_db[axis], EQ_POINTS);
                        ImPlot::PopStyleColor();
                        // Update visibility cache from legend
                        ImPlotItem* item = ImPlot::GetItem(axis_names[axis]);
                        s_axis_vis[axis] = item ? item->Show : true;
                    }
                }

                // ── Interactive draggable cutoff lines ──
                // HP cutoff(s)
                if (hp_enabled_count > 0) {
                    if (all_hp_same) {
                        // Combined: one thick white line, drags all axes
                        ImVec4 hp_col(0.95f, 0.70f, 0.30f, 0.9f);
                        if (ImPlot::DragLineX(1100, &hp_drag_all, hp_col, 2.0f)) {
                            if (hp_drag_all < 0.05) hp_drag_all = 0.05;
                            if (hp_drag_all > 10.0) hp_drag_all = 10.0;
                            for (int i = 0; i < 6; i++)
                                if (stg.mca.channels[i].hp_enabled)
                                    stg.mca.channels[i].hp.fc = (float)hp_drag_all;
                        }
                    } else {
                        // Individual: colored lines per axis
                        for (int i = 0; i < 6; i++) {
                            if (!stg.mca.channels[i].hp_enabled || stg.mca.channels[i].hp.fc <= 0) continue;
                            if (ImPlot::DragLineX(1000 + i, &hp_drag[i], axis_colors[i], 1.5f)) {
                                if (hp_drag[i] < 0.05) hp_drag[i] = 0.05;
                                if (hp_drag[i] > 10.0) hp_drag[i] = 10.0;
                                stg.mca.channels[i].hp.fc = (float)hp_drag[i];
                            }
                        }
                    }
                }

                // LP cutoff(s)
                if (lp_enabled_count > 0) {
                    if (all_lp_same) {
                        ImVec4 lp_col(0.30f, 0.70f, 0.95f, 0.9f);
                        if (ImPlot::DragLineX(2100, &lp_drag_all, lp_col, 2.0f)) {
                            if (lp_drag_all < 1.0) lp_drag_all = 1.0;
                            if (lp_drag_all > 50.0) lp_drag_all = 50.0;
                            for (int i = 0; i < 6; i++)
                                if (stg.mca.channels[i].lp_enabled)
                                    stg.mca.channels[i].lp.fc = (float)lp_drag_all;
                        }
                    } else {
                        for (int i = 0; i < 6; i++) {
                            if (!stg.mca.channels[i].lp_enabled || stg.mca.channels[i].lp.fc <= 0) continue;
                            if (ImPlot::DragLineX(2000 + i, &lp_drag[i], axis_colors[i], 1.5f)) {
                                if (lp_drag[i] < 1.0) lp_drag[i] = 1.0;
                                if (lp_drag[i] > 50.0) lp_drag[i] = 50.0;
                                stg.mca.channels[i].lp.fc = (float)lp_drag[i];
                            }
                        }
                    }
                }

                ImPlot::EndPlot();
            }
            ImGui::Unindent(8);

            // Legend hint when curves are collapsed
            if (all_curves_same) {
                ImGui::SameLine();
                ImGui::TextDisabled("(all 6 axes identical)");
            }
        }

        ImGui::Spacing();

        int table_flags = ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingFixedFit;
        if (ImGui::BeginTable("##mca_table", 9, table_flags)) {
            // Column setup
            ImGui::TableSetupColumn("Axis",     ImGuiTableColumnFlags_WidthFixed, 48.0f);
            ImGui::TableSetupColumn("HP",       ImGuiTableColumnFlags_WidthFixed, 28.0f);
            ImGui::TableSetupColumn("HP fc",    ImGuiTableColumnFlags_WidthFixed, 82.0f);
            ImGui::TableSetupColumn("HP Q",     ImGuiTableColumnFlags_WidthFixed, 68.0f);
            ImGui::TableSetupColumn("LP",       ImGuiTableColumnFlags_WidthFixed, 28.0f);
            ImGui::TableSetupColumn("LP fc",    ImGuiTableColumnFlags_WidthFixed, 82.0f);
            ImGui::TableSetupColumn("LP Q",     ImGuiTableColumnFlags_WidthFixed, 68.0f);
            ImGui::TableSetupColumn("Gain",     ImGuiTableColumnFlags_WidthFixed, 72.0f);
            ImGui::TableSetupColumn("Rate Lim", ImGuiTableColumnFlags_WidthFixed, 82.0f);

            // Header row with tooltips
            ImGui::TableNextRow(ImGuiTableRowFlags_Headers);

            ImGui::TableSetColumnIndex(0); ImGui::Text("Axis");
            ImGui::TableSetColumnIndex(1); ImGui::Text("HP");
            if (ImGui::IsItemHovered()) ImGui::SetTooltip(
                "Washout (return-to-center) enable.\n\n"
                "ON = platform returns to center after sustained input.\n"
                "     Without this, the platform stays tilted after\n"
                "     every corner, hill, or sustained acceleration.\n\n"
                "OFF = raw input, no return-to-center (may hit travel limits).");
            ImGui::TableSetColumnIndex(2); ImGui::Text("HP fc");
            if (ImGui::IsItemHovered()) ImGui::SetTooltip(
                "Return-to-Center Speed (Hz)\n"
                "=============================\n"
                "Controls how fast the platform drifts back to center\n"
                "after a sustained force (acceleration, tilt, etc).\n\n"
                "LOWER = SLOWER return, more motion preserved:\n"
                "  0.2 Hz = very slow drift back (flight sim, trucking)\n"
                "  0.3 Hz = gentle return (relaxed driving)\n\n"
                "HIGHER = FASTER return, snappier feel:\n"
                "  0.8 Hz = balanced (general driving)\n"
                "  1.5 Hz = aggressive snap-back (racing)\n"
                "  2.0 Hz = instant washout (pro rigs)\n\n"
                "Each axis can have a different return speed.\n"
                "e.g. slow heave return + fast surge return.");
            ImGui::TableSetColumnIndex(3); ImGui::Text("HP Q");
            if (ImGui::IsItemHovered()) ImGui::SetTooltip(
                "Return-to-center damping.\n\n"
                "Controls HOW the platform returns — smooth or springy.\n\n"
                "0.5   = overdamped, very gentle, mushy return\n"
                "0.707 = Butterworth (clean, no overshoot) — start here\n"
                "1.0+  = underdamped, slight bounce/overshoot on return\n\n"
                "Start with 0.707 and only adjust if it feels wrong.");
            ImGui::TableSetColumnIndex(4); ImGui::Text("LP");
            if (ImGui::IsItemHovered()) ImGui::SetTooltip(
                "Low-Pass Smoothing enable.\n\n"
                "Removes high-frequency noise, jitter, and harshness.\n"
                "Smooths the output so motion feels polished, not rattly.\n\n"
                "Feels like: ON = smooth, refined motion.\n"
                "            OFF = raw, potentially buzzy/harsh.");
            ImGui::TableSetColumnIndex(5); ImGui::Text("LP fc");
            if (ImGui::IsItemHovered()) ImGui::SetTooltip(
                "Low-Pass cutoff frequency (Hz).\n\n"
                "Higher = more detail preserved, less smoothing.\n"
                "Lower = smoother but can feel sluggish/delayed.\n\n"
                "6 Hz = very smooth (flight sim)\n"
                "10 Hz = moderate smoothing\n"
                "15 Hz = light smoothing (fast racing)\n"
                "20+ Hz = minimal smoothing\n\n"
                "Feels like: low = butter-smooth glide; high = detailed, textured.");
            ImGui::TableSetColumnIndex(6); ImGui::Text("LP Q");
            if (ImGui::IsItemHovered()) ImGui::SetTooltip(
                "Low-Pass Q factor.\n"
                "Same as HP Q — 0.707 is the standard Butterworth starting point.");
            ImGui::TableSetColumnIndex(7); ImGui::Text("Gain");
            if (ImGui::IsItemHovered()) ImGui::SetTooltip(
                "Post-filter gain multiplier for this axis.\n\n"
                "1.0 = unity (no change after filtering)\n"
                "1.5 = 50%% louder — makes this axis more prominent\n"
                "0.5 = half strength — tame this axis\n\n"
                "Use to rebalance axes after HP washout removes energy.");
            ImGui::TableSetColumnIndex(8); ImGui::Text("Rate Lim");
            if (ImGui::IsItemHovered()) ImGui::SetTooltip(
                "Maximum change per frame (%%/frame). 0 = unlimited.\n\n"
                "Prevents sudden large jumps that feel like impacts.\n"
                "Good for taming spiky data sources.\n\n"
                "5-10 = smooth transitions, may feel laggy\n"
                "20-50 = catches worst spikes, mostly transparent\n"
                "0 = disabled (fastest response)");

            // Data rows — edit staging, not live
            for (int i = 0; i < 6; i++) {
                AxisChannelFilter& ch = stg.mca.channels[i];
                ImGui::PushID(i);
                ImGui::TableNextRow();

                // Axis name
                ImGui::TableSetColumnIndex(0);
                ImGui::TextUnformatted(axis_names[i]);
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", axis_feel[i]);

                // HP enable
                ImGui::TableSetColumnIndex(1);
                bool hp_on = ch.hp_enabled != 0;
                if (ImGui::Checkbox("##hp", &hp_on))
                    ch.hp_enabled = hp_on ? 1 : 0;

                // HP fc
                ImGui::TableSetColumnIndex(2);
                ImGui::PushItemWidth(-1);
                ImGui::DragFloat("##hpfc", &ch.hp.fc, 0.01f, 0.05f, 10.0f, "%.2f Hz");
                ImGui::PopItemWidth();

                // HP Q
                ImGui::TableSetColumnIndex(3);
                ImGui::PushItemWidth(-1);
                ImGui::DragFloat("##hpq", &ch.hp.Q, 0.01f, 0.1f, 5.0f, "%.2f");
                ImGui::PopItemWidth();

                // LP enable
                ImGui::TableSetColumnIndex(4);
                bool lp_on = ch.lp_enabled != 0;
                if (ImGui::Checkbox("##lp", &lp_on))
                    ch.lp_enabled = lp_on ? 1 : 0;

                // LP fc
                ImGui::TableSetColumnIndex(5);
                ImGui::PushItemWidth(-1);
                ImGui::DragFloat("##lpfc", &ch.lp.fc, 0.1f, 1.0f, 50.0f, "%.1f Hz");
                ImGui::PopItemWidth();

                // LP Q
                ImGui::TableSetColumnIndex(6);
                ImGui::PushItemWidth(-1);
                ImGui::DragFloat("##lpq", &ch.lp.Q, 0.01f, 0.1f, 5.0f, "%.2f");
                ImGui::PopItemWidth();

                // Gain
                ImGui::TableSetColumnIndex(7);
                ImGui::PushItemWidth(-1);
                ImGui::DragFloat("##gain", &ch.gain, 0.01f, 0.0f, 5.0f, "%.2fx");
                ImGui::PopItemWidth();

                // Rate Limit
                ImGui::TableSetColumnIndex(8);
                ImGui::PushItemWidth(-1);
                ImGui::DragFloat("##rl", &ch.rate_limit, 0.1f, 0.0f, 100.0f, ch.rate_limit > 0.0f ? "%.1f" : "Off");
                ImGui::PopItemWidth();

                ImGui::PopID();
            }

            // ── Virtual Axis: Tilt Washout (Surge→Pitch, Sway→Roll) ──
            {
                // Section separator row with link toggle
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0, IM_COL32(40, 50, 65, 255));
                ImGui::TextColored(ImVec4(0.6f, 0.8f, 1.0f, 1.0f), "Tilt");
                if (ImGui::IsItemHovered()) ImGui::SetTooltip(
                    "Tilt Washout: controls how fast the tilt coordination\n"
                    "output returns to center. Without washout, sustained\n"
                    "tilt stays permanently tilted.");
                ImGui::TableSetColumnIndex(1);
                ImGui::TextColored(ImVec4(0.5f, 0.6f, 0.7f, 1.0f), "HP");
                ImGui::TableSetColumnIndex(2);
                ImGui::TextColored(ImVec4(0.5f, 0.6f, 0.7f, 1.0f), "HP fc");
                ImGui::TableSetColumnIndex(3);
                ImGui::TextColored(ImVec4(0.5f, 0.6f, 0.7f, 1.0f), "HP Q");
                // Link toggle in the LP column area
                ImGui::TableSetColumnIndex(4);
                {
                    bool linked = stg.mca.tilt.hp_linked != 0;
                    if (ImGui::Checkbox("##tlink", &linked)) {
                        stg.mca.tilt.hp_linked = linked ? 1 : 0;
                        if (linked) {
                            // Sync sway to surge values
                            stg.mca.tilt.sway_hp_enabled = stg.mca.tilt.surge_hp_enabled;
                            stg.mca.tilt.sway_hp_fc = stg.mca.tilt.hp_fc;
                            stg.mca.tilt.sway_hp_Q  = stg.mca.tilt.hp_Q;
                        }
                    }
                    ImGui::SameLine();
                    ImGui::TextColored(ImVec4(0.5f, 0.6f, 0.7f, 1.0f), "Link");
                    if (ImGui::IsItemHovered()) ImGui::SetTooltip(
                        "Link Surge\xe2\x86\x92Pitch and Sway\xe2\x86\x92Roll.\n"
                        "When linked, editing one mirrors to the other.");
                }

                // Surge → Pitch tilt washout
                ImGui::PushID(700);
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextDisabled("Pitch");
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("Tilt Pitch washout (from sustained surge).\nControls how fast the pitch tilt returns to center\nafter prolonged acceleration or braking.");

                ImGui::TableSetColumnIndex(1);
                bool surge_hp_on = stg.mca.tilt.surge_hp_enabled != 0;
                if (ImGui::Checkbox("##thp", &surge_hp_on)) {
                    stg.mca.tilt.surge_hp_enabled = surge_hp_on ? 1 : 0;
                    if (stg.mca.tilt.hp_linked)
                        stg.mca.tilt.sway_hp_enabled = stg.mca.tilt.surge_hp_enabled;
                }

                ImGui::TableSetColumnIndex(2);
                ImGui::PushItemWidth(-1);
                if (ImGui::DragFloat("##thpfc", &stg.mca.tilt.hp_fc, 0.01f, 0.02f, 3.0f, "%.2f Hz")) {
                    if (stg.mca.tilt.hp_linked)
                        stg.mca.tilt.sway_hp_fc = stg.mca.tilt.hp_fc;
                }
                if (ImGui::IsItemHovered()) ImGui::SetTooltip(
                    "Surge\xe2\x86\x92Pitch return-to-center speed (Hz).\n\n"
                    "0.1 Hz = slow drift back\n"
                    "0.3 Hz = moderate (default)\n"
                    "1.0 Hz = fast snap-back");
                ImGui::PopItemWidth();

                ImGui::TableSetColumnIndex(3);
                ImGui::PushItemWidth(-1);
                if (ImGui::DragFloat("##thpq", &stg.mca.tilt.hp_Q, 0.01f, 0.1f, 5.0f, "%.2f")) {
                    if (stg.mca.tilt.hp_linked)
                        stg.mca.tilt.sway_hp_Q = stg.mca.tilt.hp_Q;
                }
                if (ImGui::IsItemHovered()) ImGui::SetTooltip(
                    "Tilt washout damping.\n"
                    "0.707 = Butterworth (clean).\n"
                    "Lower = mushier. Higher = springier.");
                ImGui::PopItemWidth();

                // LP/Gain/Rate columns: N/A
                ImGui::TableSetColumnIndex(4); ImGui::TextDisabled("-");
                ImGui::TableSetColumnIndex(5); ImGui::TextDisabled("-");
                ImGui::TableSetColumnIndex(6); ImGui::TextDisabled("-");
                ImGui::TableSetColumnIndex(7); ImGui::TextDisabled("-");
                ImGui::TableSetColumnIndex(8); ImGui::TextDisabled("-");
                ImGui::PopID();

                // Sway → Roll tilt washout (independent or linked)
                ImGui::PushID(701);
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextDisabled("Roll");
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("Tilt Roll washout (from sustained sway).\nControls how fast the roll tilt returns to center\nafter prolonged lateral force (cornering).");

                bool sway_linked = stg.mca.tilt.hp_linked != 0;
                if (sway_linked) ImGui::BeginDisabled();

                ImGui::TableSetColumnIndex(1);
                bool sway_hp_on = stg.mca.tilt.sway_hp_enabled != 0;
                if (ImGui::Checkbox("##thp2", &sway_hp_on))
                    stg.mca.tilt.sway_hp_enabled = sway_hp_on ? 1 : 0;

                ImGui::TableSetColumnIndex(2);
                ImGui::PushItemWidth(-1);
                ImGui::DragFloat("##thpfc2", &stg.mca.tilt.sway_hp_fc, 0.01f, 0.02f, 3.0f, "%.2f Hz");
                if (ImGui::IsItemHovered()) ImGui::SetTooltip(
                    "Sway\xe2\x86\x92Roll return-to-center speed (Hz).");
                ImGui::PopItemWidth();

                ImGui::TableSetColumnIndex(3);
                ImGui::PushItemWidth(-1);
                ImGui::DragFloat("##thpq2", &stg.mca.tilt.sway_hp_Q, 0.01f, 0.1f, 5.0f, "%.2f");
                if (ImGui::IsItemHovered()) ImGui::SetTooltip(
                    "Sway\xe2\x86\x92Roll washout damping.");
                ImGui::PopItemWidth();

                if (sway_linked) ImGui::EndDisabled();

                ImGui::TableSetColumnIndex(4); ImGui::TextDisabled("-");
                ImGui::TableSetColumnIndex(5); ImGui::TextDisabled("-");
                ImGui::TableSetColumnIndex(6); ImGui::TextDisabled("-");
                ImGui::TableSetColumnIndex(7); ImGui::TextDisabled("-");
                ImGui::TableSetColumnIndex(8); ImGui::TextDisabled("-");
                ImGui::PopID();
            }

            ImGui::EndTable();
        }

        ImGui::Spacing();
        ImGui::Separator();

        // ═══════════════════════════════════════════════════════════════
        // TILT COORDINATION
        // ═══════════════════════════════════════════════════════════════
        ImGui::TextColored(ImVec4(0.7f, 0.85f, 1.0f, 1.0f), "Tilt Coordination");
        if (ImGui::IsItemHovered()) ImGui::SetTooltip(
            "Uses gravity to fake sustained linear forces.\n\n"
            "Real race cars push you back under acceleration and forward\n"
            "under braking. A motion platform can't sustain that push,\n"
            "but tilting slowly lets gravity pull you in the right direction.\n\n"
            "Surge (braking/accel) -> platform tilts in pitch\n"
            "Sway (cornering)      -> platform tilts in roll\n\n"
            "The LP filter extracts only the slow, sustained component\n"
            "so quick bumps don't cause unwanted tilt.");

        {
            bool tilt_on = stg.mca.tilt.enabled != 0;
            if (ImGui::Checkbox("Enable##tilt", &tilt_on))
                stg.mca.tilt.enabled = tilt_on ? 1 : 0;

            // ── Tilt LP frequency response chart ──
            {
                static const int TILT_PTS = 256;
                static double tilt_freq[TILT_PTS];
                static double tilt_db[TILT_PTS];

                float sr = stg.mca.sample_rate > 1.0f ? stg.mca.sample_rate : 60.0f;
                float nyquist = sr * 0.5f;
                float f_min = 0.05f;
                float f_max = nyquist > 1.0f ? nyquist : 30.0f;
                float tilt_fc = stg.mca.tilt.fc;
                float tilt_Q  = stg.mca.tilt.Q > 0 ? stg.mca.tilt.Q : 0.707f;
                float tilt_hp_fc = stg.mca.tilt.hp_fc;
                float tilt_hp_Q  = stg.mca.tilt.hp_Q > 0 ? stg.mca.tilt.hp_Q : 0.707f;
                bool  tilt_hp_on = stg.mca.tilt.hp_enabled != 0;

                // Biquad magnitude
                auto biquadMagSq = [](const BiquadFilter& f, double w) -> double {
                    double cw  = cos(w),  sw  = sin(w);
                    double c2w = cos(2*w), s2w = sin(2*w);
                    double nr = f.b0 + f.b1 * cw + f.b2 * c2w;
                    double ni = -(f.b1 * sw + f.b2 * s2w);
                    double dr = 1.0 + f.a1 * cw + f.a2 * c2w;
                    double di = -(f.a1 * sw + f.a2 * s2w);
                    double num2 = nr*nr + ni*ni;
                    double den2 = dr*dr + di*di;
                    return den2 > 1e-30 ? num2 / den2 : 1.0;
                };

                BiquadFilter lp_tmp = {}, hp_tmp = {};
                if (tilt_fc > 0)
                    biquadSetLowpass(&lp_tmp, tilt_fc, sr, tilt_Q);
                if (tilt_hp_on && tilt_hp_fc > 0)
                    biquadSetHighpass(&hp_tmp, tilt_hp_fc, sr, tilt_hp_Q);

                static double tilt_db_combined[TILT_PTS];
                for (int k = 0; k < TILT_PTS; k++) {
                    double freq = f_min * pow((double)f_max / f_min, (double)k / (TILT_PTS - 1));
                    tilt_freq[k] = freq;
                    double w = 2.0 * M_PI * freq / (double)sr;
                    double mag2_lp = (tilt_fc > 0) ? biquadMagSq(lp_tmp, w) : 1.0;
                    tilt_db[k] = 10.0 * log10(mag2_lp > 1e-30 ? mag2_lp : 1e-30);
                    // Combined LP + HP (what actually reaches the output)
                    double mag2_combined = mag2_lp;
                    if (tilt_hp_on && tilt_hp_fc > 0) mag2_combined *= biquadMagSq(hp_tmp, w);
                    tilt_db_combined[k] = 10.0 * log10(mag2_combined > 1e-30 ? mag2_combined : 1e-30);
                }

                // Drag handle for tilt LP cutoff
                static double tilt_drag_fc = 0.5;
                tilt_drag_fc = (double)tilt_fc;

                static bool s_surge_vis = true, s_sway_vis = true;

                ImGui::Indent(8);
                if (ImPlot::BeginPlot("##tilt_response", ImVec2(-16, 150), ImPlotFlags_NoBoxSelect | ImPlotFlags_NoMenus | ImPlotFlags_NoChild)) {
                    ImPlot::SetupAxisScale(ImAxis_X1, ImPlotScale_Log10);
                    ImPlot::SetupAxes("Frequency (Hz)", "dB");
                    ImPlot::SetupAxisLimits(ImAxis_X1, f_min, f_max, ImPlotCond_Always);
                    ImPlot::SetupAxis(ImAxis_X1, nullptr, ImPlotAxisFlags_Lock | ImPlotAxisFlags_NoMenus);
                    ImPlot::SetupAxisLimits(ImAxis_Y1, -40.0, 6.0, ImPlotCond_Always);
                    ImPlot::SetupAxis(ImAxis_Y1, nullptr, ImPlotAxisFlags_Lock | ImPlotAxisFlags_NoMenus);

                    // 0 dB reference
                    double zero_x[2] = {f_min, f_max};
                    double zero_y[2] = {0.0, 0.0};
                    ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(0.5f, 0.5f, 0.5f, 0.3f));
                    ImPlot::PlotLine("##tzero", zero_x, zero_y, 2);
                    ImPlot::PopStyleColor();

                    // Real-time spectrum overlay for surge (axis 0) and sway (axis 1)
                    float spec_fmax_t = g_app.input_spectrum_freq_max;
                    if (spec_fmax_t > 0.5f) {
                        static const int TSPEC_PTS = 128;
                        static double tspec_freq[TSPEC_PTS];
                        static double tspec_db[2][TSPEC_PTS];
                        static double tspec_floor[TSPEC_PTS];

                        float peak_mag = 0.0f;
                        for (int a = 0; a < 2; a++)
                            for (int k = 1; k < SPECTRUM_BINS; k++)
                                if (g_app.input_spectrum[a][k] > peak_mag)
                                    peak_mag = g_app.input_spectrum[a][k];
                        double norm = peak_mag > 1e-6f ? (double)peak_mag : 1.0;

                        for (int k = 0; k < TSPEC_PTS; k++) {
                            double freq = f_min * pow((double)f_max / f_min, (double)k / (TSPEC_PTS - 1));
                            tspec_freq[k] = freq;
                            tspec_floor[k] = -40.0;
                            double bin_f = freq / (double)spec_fmax_t * (double)SPECTRUM_BINS;
                            int b0 = (int)bin_f;
                            float frac = (float)(bin_f - b0);
                            if (b0 < 1) b0 = 1;
                            if (b0 >= SPECTRUM_BINS - 1) { b0 = SPECTRUM_BINS - 2; frac = 1.0f; }
                            for (int a = 0; a < 2; a++) {
                                float mag = g_app.input_spectrum[a][b0] * (1.0f - frac)
                                          + g_app.input_spectrum[a][b0 + 1] * frac;
                                double db = 20.0 * log10((double)mag / norm + 1e-10);
                                if (db < -40.0) db = -40.0;
                                tspec_db[a][k] = db;
                            }
                        }

                        // Surge spectrum (red, axis 0)
                        if (s_surge_vis) {
                            bool has = false;
                            for (int k = 1; k < SPECTRUM_BINS && !has; k++)
                                if (g_app.input_spectrum[0][k] > peak_mag * 0.001f) has = true;
                            if (has) {
                                ImPlot::PushStyleColor(ImPlotCol_Fill, ImVec4(0.90f, 0.30f, 0.30f, 0.12f));
                                ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(0.90f, 0.30f, 0.30f, 0.30f));
                                ImPlot::PlotShaded("##tspec_surge", tspec_freq, tspec_db[0], tspec_floor, TSPEC_PTS);
                                ImPlot::PopStyleColor(2);
                            }
                        }
                        // Sway spectrum (green, axis 1)
                        if (s_sway_vis) {
                            bool has = false;
                            for (int k = 1; k < SPECTRUM_BINS && !has; k++)
                                if (g_app.input_spectrum[1][k] > peak_mag * 0.001f) has = true;
                            if (has) {
                                ImPlot::PushStyleColor(ImPlotCol_Fill, ImVec4(0.30f, 0.90f, 0.30f, 0.12f));
                                ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(0.30f, 0.90f, 0.30f, 0.30f));
                                ImPlot::PlotShaded("##tspec_sway", tspec_freq, tspec_db[1], tspec_floor, TSPEC_PTS);
                                ImPlot::PopStyleColor(2);
                            }
                        }
                    }

                    // LP extraction curve (dimmed when HP is active)
                    if (tilt_hp_on) {
                        ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(0.6f, 0.6f, 0.6f, 0.4f));
                        ImPlot::PlotLine("Tilt LP##dim", tilt_freq, tilt_db, TILT_PTS);
                        ImPlot::PopStyleColor();
                    }

                    // Combined LP + HP curve (what actually reaches output)
                    ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(0.95f, 0.95f, 0.95f, 1.0f));
                    ImPlot::SetNextLineStyle(IMPLOT_AUTO_COL, 2.0f);
                    ImPlot::PlotLine(tilt_hp_on ? "Tilt LP+HP" : "Tilt LP", tilt_freq, tilt_hp_on ? tilt_db_combined : tilt_db, TILT_PTS);
                    ImPlot::PopStyleColor();

                    // Surge & Sway input reference lines (thin, for legend toggling)
                    ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(0.90f, 0.30f, 0.30f, 0.6f));
                    ImPlot::PlotLine("Surge", (double*)nullptr, (double*)nullptr, 0);
                    ImPlot::PopStyleColor();
                    ImPlotItem* surge_item = ImPlot::GetItem("Surge");
                    s_surge_vis = surge_item ? surge_item->Show : true;

                    ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(0.30f, 0.90f, 0.30f, 0.6f));
                    ImPlot::PlotLine("Sway", (double*)nullptr, (double*)nullptr, 0);
                    ImPlot::PopStyleColor();
                    ImPlotItem* sway_item = ImPlot::GetItem("Sway");
                    s_sway_vis = sway_item ? sway_item->Show : true;

                    // Draggable LP cutoff line
                    if (tilt_fc > 0) {
                        ImVec4 tc(0.30f, 0.70f, 0.95f, 0.9f);
                        if (ImPlot::DragLineX(3100, &tilt_drag_fc, tc, 2.0f)) {
                            if (tilt_drag_fc < 0.05) tilt_drag_fc = 0.05;
                            if (tilt_drag_fc > 5.0) tilt_drag_fc = 5.0;
                            stg.mca.tilt.fc = (float)tilt_drag_fc;
                        }
                    }

                    // Draggable HP washout cutoff line
                    if (tilt_hp_on && tilt_hp_fc > 0) {
                        static double tilt_hp_drag_fc = 0.3;
                        tilt_hp_drag_fc = (double)tilt_hp_fc;
                        ImVec4 hc(0.95f, 0.50f, 0.30f, 0.9f);
                        if (ImPlot::DragLineX(3101, &tilt_hp_drag_fc, hc, 2.0f)) {
                            if (tilt_hp_drag_fc < 0.02) tilt_hp_drag_fc = 0.02;
                            if (tilt_hp_drag_fc > 3.0) tilt_hp_drag_fc = 3.0;
                            stg.mca.tilt.hp_fc = (float)tilt_hp_drag_fc;
                        }
                    }

                    ImPlot::EndPlot();
                }
                ImGui::Unindent(8);
            }

            if (ImGui::BeginTable("##tilt_table", 4, ImGuiTableFlags_SizingStretchSame)) {
                ImGui::TableSetupColumn("lbl", ImGuiTableColumnFlags_WidthFixed, 100.0f);
                ImGui::TableSetupColumn("inv", ImGuiTableColumnFlags_WidthFixed, 28.0f);
                ImGui::TableSetupColumn("slider", 0, 1.0f);
                ImGui::TableSetupColumn("unit", ImGuiTableColumnFlags_WidthFixed, 1.0f);

                // Surge -> Pitch gain + invert
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextUnformatted("Surge -> Pitch");
                ImGui::TableSetColumnIndex(1);
                {
                    bool surge_inv = stg.mca.tilt.surge_gain < 0.0f;
                    if (ImGui::Checkbox("##sg_inv", &surge_inv)) {
                        float mag = fabsf(stg.mca.tilt.surge_gain);
                        if (mag < 0.001f) mag = 0.08f;
                        stg.mca.tilt.surge_gain = surge_inv ? -mag : mag;
                    }
                    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Invert surge tilt direction");
                }
                ImGui::TableSetColumnIndex(2);
                ImGui::PushItemWidth(-1);
                {
                    float abs_sg = fabsf(stg.mca.tilt.surge_gain);
                    if (ImGui::DragFloat("##sg", &abs_sg, 0.005f, 0.0f, 2.0f, "%.3f")) {
                        stg.mca.tilt.surge_gain = (stg.mca.tilt.surge_gain < 0.0f) ? -abs_sg : abs_sg;
                    }
                    if (ImGui::IsItemHovered()) ImGui::SetTooltip(
                        "Tilt gain: %% of pitch per %% of sustained surge.\n\n"
                        "Braking tilts the rig forward, acceleration tilts back.\n"
                        "Use the checkbox to invert direction.\n"
                        "0.08 = subtle, 0.15 = moderate, 0.25 = strong, 0.35 = aggressive");
                }
                ImGui::PopItemWidth();

                // Sway -> Roll gain + invert
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextUnformatted("Sway -> Roll");
                ImGui::TableSetColumnIndex(1);
                {
                    bool sway_inv = stg.mca.tilt.sway_gain < 0.0f;
                    if (ImGui::Checkbox("##sw_inv", &sway_inv)) {
                        float mag = fabsf(stg.mca.tilt.sway_gain);
                        if (mag < 0.001f) mag = 0.08f;
                        stg.mca.tilt.sway_gain = sway_inv ? -mag : mag;
                    }
                    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Invert sway tilt direction");
                }
                ImGui::TableSetColumnIndex(2);
                ImGui::PushItemWidth(-1);
                {
                    float abs_sw = fabsf(stg.mca.tilt.sway_gain);
                    if (ImGui::DragFloat("##sw", &abs_sw, 0.005f, 0.0f, 2.0f, "%.3f")) {
                        stg.mca.tilt.sway_gain = (stg.mca.tilt.sway_gain < 0.0f) ? -abs_sw : abs_sw;
                    }
                    if (ImGui::IsItemHovered()) ImGui::SetTooltip(
                        "Tilt gain: %% of roll per %% of sustained sway.\n\n"
                        "Sustained cornering tilts the rig into the turn,\n"
                        "letting gravity simulate lateral G-force.\n"
                        "Use the checkbox to invert direction.");
                }
                ImGui::PopItemWidth();

                // LP cutoff
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextUnformatted("LP Cutoff");
                ImGui::TableSetColumnIndex(2);
                ImGui::PushItemWidth(-1);
                ImGui::DragFloat("##tfc", &stg.mca.tilt.fc, 0.01f, 0.05f, 5.0f, "%.2f Hz");
                if (ImGui::IsItemHovered()) ImGui::SetTooltip(
                    "LP filter cutoff for tilt extraction.\n"
                    "Only forces slower than this produce tilt.\n"
                    "0.3 Hz = sustained only, 0.5 Hz = typical, 1.0 Hz = responsive");
                ImGui::PopItemWidth();

                // LP Q
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextUnformatted("LP Q");
                ImGui::TableSetColumnIndex(2);
                ImGui::PushItemWidth(-1);
                ImGui::DragFloat("##tq", &stg.mca.tilt.Q, 0.01f, 0.1f, 5.0f, "%.3f");
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("LP filter Q for tilt. 0.707 = Butterworth (clean, no overshoot).");
                ImGui::PopItemWidth();

                ImGui::EndTable();
            }
        }

        ImGui::Spacing();
        ImGui::Separator();

        // ═══════════════════════════════════════════════════════════════
        // OCCUPANT OFFSET
        // ═══════════════════════════════════════════════════════════════
        ImGui::TextColored(ImVec4(0.7f, 0.85f, 1.0f, 1.0f), "Occupant Position Offset");
        if (ImGui::IsItemHovered()) ImGui::SetTooltip(
            "Your head sits above the platform pivot point.\n"
            "When the platform tilts, your head traces an arc —\n"
            "this creates unwanted lateral/vertical motion (parasitic motion).\n\n"
            "Set your approximate head position relative to the pivot\n"
            "so the algorithm can compensate.\n\n"
            "Vertical = height of your head above the platform center.\n"
            "800mm is typical for a seated driver.");
        {
            if (ImGui::BeginTable("##occ_table", 4, ImGuiTableFlags_SizingStretchSame)) {
                ImGui::TableSetupColumn("c0", 0, 1.0f);
                ImGui::TableSetupColumn("c1", 0, 1.0f);
                ImGui::TableSetupColumn("c2", 0, 1.0f);
                ImGui::TableSetupColumn("btn", ImGuiTableColumnFlags_WidthFixed, 50.0f);
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::PushItemWidth(-1);
                ImGui::DragFloat("Lateral (X)##occ", &stg.occupant[0], 1.0f, -500, 500, "%.0f mm");
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("Left/right offset from platform center. 0 = centered.");
                ImGui::PopItemWidth();
                ImGui::TableSetColumnIndex(1);
                ImGui::PushItemWidth(-1);
                ImGui::DragFloat("Fore/Aft (Y)##occ", &stg.occupant[1], 1.0f, -500, 500, "%.0f mm");
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("Forward/back from platform center. 0 = centered.");
                ImGui::PopItemWidth();
                ImGui::TableSetColumnIndex(2);
                ImGui::PushItemWidth(-1);
                ImGui::DragFloat("Height (Z)##occ", &stg.occupant[2], 1.0f, 0, 2000, "%.0f mm");
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("Head height above platform. 800mm typical for seated driver.");
                ImGui::PopItemWidth();
                ImGui::TableSetColumnIndex(3);
                if (ImGui::SmallButton("Reset##occ")) {
                    stg.occupant[0] = 0; stg.occupant[1] = 0; stg.occupant[2] = 800;
                }
                ImGui::EndTable();
            }
        }

        // ═══════════════════════════════════════════════════════════════
        // WORKSPACE READOUT
        // ═══════════════════════════════════════════════════════════════
        ImGui::Spacing();
        ImGui::TextDisabled("Workspace: \xc2\xb1%.0fmm surge  \xc2\xb1%.1f\xc2\xb0 roll  |  %s",
            e.config.axis_scales.scale[0],
            e.config.axis_scales.scale[3] * (float)(180.0 / M_PI),
            mcaPresetName(mca.preset));

        ImGui::EndChild(); // end ##dyn_scroll
}

// ── Per-Entity I/O Monitor ──────────────────────────────────────────

static void DrawEntityConsoleContent(Entity& e) {
    ImGui::PushID(e.id + 4000);
    {
        const char* ax[] = {"Surge", "Sway", "Heave", "Roll", "Pitch", "Yaw"};

        // ── Input (% of workspace) ──
        if (ImGui::CollapsingHeader("Input (%)", ImGuiTreeNodeFlags_DefaultOpen)) {
            float col_w = ImGui::GetContentRegionAvail().x * 0.5f;
            for (int i = 0; i < 6; i++) {
                if (i % 2 == 1) ImGui::SameLine(col_w + 8);
                float v = e.state.input_pct[i];
                ImVec4 vc = (fabsf(v) > 80.0f) ? ImVec4(1.0f, 0.5f, 0.3f, 1.0f)
                          : (fabsf(v) > 0.5f)  ? ImVec4(0.85f, 0.85f, 0.9f, 1.0f)
                                                : ImVec4(0.5f, 0.5f, 0.55f, 1.0f);
                ImGui::TextColored(vc, "%-6s %+6.1f%%", ax[i], v);
            }
        }

        // ── Physical input (mm / rad) ──
        if (ImGui::CollapsingHeader("Physical Input")) {
            const char* units[] = {"mm", "mm", "mm", "rad", "rad", "rad"};
            float col_w = ImGui::GetContentRegionAvail().x * 0.5f;
            for (int i = 0; i < 6; i++) {
                if (i % 2 == 1) ImGui::SameLine(col_w + 8);
                ImGui::Text("%-6s %+7.2f %s", ax[i], e.state.input_physical[i], units[i]);
            }
        }

        // ── Servo output ──
        if (ImGui::CollapsingHeader("Servo Output", ImGuiTreeNodeFlags_DefaultOpen)) {
            float col_w = ImGui::GetContentRegionAvail().x * 0.5f;
            for (int i = 0; i < 6; i++) {
                if (i % 2 == 1) ImGui::SameLine(col_w + 8);
                float deg = e.state.output_angles_deg[i];
                float util = e.state.servo_util[i];
                ImVec4 vc = (util > 90.0f) ? ImVec4(0.98f, 0.44f, 0.44f, 1.0f)
                          : (util > 70.0f) ? ImVec4(0.98f, 0.75f, 0.15f, 1.0f)
                                           : ImVec4(0.2f, 0.83f, 0.6f, 1.0f);
                ImGui::TextColored(vc, "S%d %+5.1f\xc2\xb0", i, deg);
            }
            ImGui::Text("Workspace: %.0f%%", e.state.max_util);
        }

        // ── HIL TX raw values ──
        if (e.type == EntityType::HIL) {
            if (ImGui::CollapsingHeader("HIL TX Raw")) {
                float col_w = ImGui::GetContentRegionAvail().x * 0.33f;
                for (int i = 0; i < 6; i++) {
                    if (i > 0 && i % 3 == 0) {} // new row
                    else if (i % 3 != 0) ImGui::SameLine(col_w * (i % 3) + 8);
                    ImGui::Text("[%d] %6u", i, (unsigned)e.hil_tx_raw[i]);
                }

                bool connected = e.serial && e.serial->isOpen();
                if (connected) {
                    ImGui::Separator();
                    ImGui::TextColored(ImVec4(0.2f, 0.83f, 0.6f, 1.0f), "%s", e.serial->portName());
                    ImGui::SameLine();
                    ImGui::Text("TX:%.0fHz Tel:%.0fHz", e.rate_tx_hz, e.rate_tel_hz);
                    ImGui::Text("RX: %d B  TX: %d B", e.serial->rxBytes(), e.serial->txBytes());
                } else {
                    ImGui::TextDisabled("Not connected");
                }
            }
        }

        // ── Rates ──
        if (ImGui::CollapsingHeader("Rates")) {
            ImGui::Text("IK: %.0f Hz", e.rate_ik_hz);
            if (e.type == EntityType::HIL) {
                ImGui::Text("TX: %.0f Hz", e.rate_tx_hz);
                ImGui::Text("Telemetry: %.0f Hz", e.rate_tel_hz);
            }
        }

        // ── Entity log (filtered from global console) ──
        if (ImGui::CollapsingHeader("Log", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::BeginChild("##elog", ImVec2(0, 120), ImGuiChildFlags_Border, ImGuiWindowFlags_HorizontalScrollbar);
            int shown = 0;
            for (auto& entry : g_app.console_log) {
                if (entry.entity_id != e.id) continue;
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.5f, 0.5f, 0.55f, 1.0f));
                ImGui::Text("[%s]", entry.source);
                ImGui::PopStyleColor();
                ImGui::SameLine();
                ImGui::TextUnformatted(entry.message);
                shown++;
            }
            if (shown == 0)
                ImGui::TextDisabled("No log entries for this entity.");
            if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY())
                ImGui::SetScrollHereY(1.0f);
            ImGui::EndChild();
        }
    }
    ImGui::PopID();
}

// Get current active source color + label for combo header / progress bar
static void GetActiveSourceInfo(const char** out_label, const char** out_icon,
                                 const ImVec4** out_color, const ImVec4** out_color_dim) {
    if (g_app.input_source == InputSource::CapturePlayback) {
        *out_label = "Capture Playback";
        *out_icon = "CAP";
        *out_color = &g_capture_color;
        *out_color_dim = &g_capture_color_dim;
    } else {
        if (g_app.active_plugin_idx >= 0 && g_app.active_plugin_idx < g_app.plugin_mgr.pluginCount())
            *out_label = g_app.plugin_mgr.pluginName(g_app.active_plugin_idx);
        else
            *out_label = "No plugin selected";
        *out_icon = "PLG";
        *out_color = &g_plugin_color;
        *out_color_dim = &g_plugin_color_dim;
    }
}

// Get target source info during a switch (target may be a plugin we're switching TO)
static void GetTargetSourceInfo(const char** out_label, const char** out_icon,
                                 const ImVec4** out_color) {
    if (g_app.source_switch_target == InputSource::CapturePlayback) {
        *out_label = "Capture Playback";
        *out_icon = "CAP";
        *out_color = &g_capture_color;
    } else {
        if (g_app.active_plugin_idx >= 0 && g_app.active_plugin_idx < g_app.plugin_mgr.pluginCount())
            *out_label = g_app.plugin_mgr.pluginName(g_app.active_plugin_idx);
        else
            *out_label = "Plugin";
        *out_icon = "PLG";
        *out_color = &g_plugin_color;
    }
}

// Build a one-line status string for capture playback source
static void GetCaptureStatus(char* buf, int buf_sz) {
    if (g_app.capture_playing && g_app.capture_playback_idx >= 0 &&
        g_app.capture_playback_idx < (int)g_app.saved_recordings.size()) {
        auto& sr = g_app.saved_recordings[g_app.capture_playback_idx];
        double elapsed = g_app.frame_time - g_app.capture_start_time;
        snprintf(buf, buf_sz, "Playing: %s  %.1f/%.1fs", sr.name, elapsed, sr.duration());
    } else {
        int n = (int)g_app.saved_recordings.size();
        snprintf(buf, buf_sz, "%d capture%s  %s", n, n != 1 ? "s" : "", n > 0 ? "ready" : "empty");
    }
}

// Draw a colored badge rectangle with text
static void DrawBadge(ImDrawList* dl, ImVec2 pos, const char* text, ImVec4 color, float h) {
    ImVec2 tsz = ImGui::CalcTextSize(text);
    float pad = 4.0f;
    float w = tsz.x + pad * 2;
    ImU32 bg = IM_COL32((int)(color.x * 255 * 0.3f), (int)(color.y * 255 * 0.3f),
                         (int)(color.z * 255 * 0.3f), 220);
    ImU32 fg = IM_COL32((int)(color.x * 255), (int)(color.y * 255),
                         (int)(color.z * 255), 255);
    ImU32 border = IM_COL32((int)(color.x * 255 * 0.6f), (int)(color.y * 255 * 0.6f),
                              (int)(color.z * 255 * 0.6f), 180);
    float top = pos.y + (h - tsz.y) * 0.5f;
    dl->AddRectFilled(pos, ImVec2(pos.x + w, pos.y + h), bg, 4.0f);
    dl->AddRect(pos, ImVec2(pos.x + w, pos.y + h), border, 4.0f);
    dl->AddText(ImVec2(pos.x + pad, top), fg, text);
}

// Map saved recording source string to badge icon + color
static void GetCaptureSourceBadge(const char* source, const char** icon, const char** label, ImVec4* color) {
    if (strcmp(source, "capture") == 0)       { *icon = "CAP"; *label = "Capture Playback"; *color = ImVec4(0.95f, 0.75f, 0.20f, 1.0f); }
    else if (strcmp(source, "plugin") == 0)  { *icon = "PLG"; *label = "Plugin";            *color = ImVec4(0.20f, 0.83f, 0.60f, 1.0f); }
    else                                     { *icon = "MAN"; *label = "Manual";            *color = ImVec4(0.60f, 0.70f, 0.80f, 1.0f); }
}

// Format a unix timestamp to a readable date/time string
static void FormatDateTime(double unix_time, char* buf, int buf_sz) {
    if (unix_time <= 0) { snprintf(buf, buf_sz, "Unknown"); return; }
    time_t t = (time_t)unix_time;
    struct tm lt;
#ifdef _WIN32
    localtime_s(&lt, &t);
#else
    localtime_r(&t, &lt);
#endif
    strftime(buf, buf_sz, "%b %d %Y  %I:%M %p", &lt);
}

// ── Plugin Parameter Auto-Layout Engine ──────────────────────────────
//
// Detects per-axis parameter groups by naming convention:
//   prefix_surge..yaw  (e.g. freq_surge, amp_sway)
//   surge_suffix..yaw  (e.g. surge_en, sway_en)
// Groups with 3+ axes → compact table with Axis rows × property columns.
// Remaining params → responsive 2-column grid (labels left, widgets right).
// Bool-only axis groups with 1 column → inline checkbox row.

static float* PluginParamPtr(PluginInstance& plug, int pidx) {
    const char* name = plug.info->params[pidx].name;
    for (auto& pv : plug.param_values)
        if (pv.name == name) return &pv.value;
    return nullptr;
}

static bool DrawPluginWidget(PluginInstance& plug, int pidx, bool compact) {
    const StewartParamDef& pd = plug.info->params[pidx];
    float* val = PluginParamPtr(plug, pidx);
    if (!val) return false;

    const char* label = compact ? "##v" : (pd.display_name ? pd.display_name : pd.name);
    // Compact mode: fixed-width inputs instead of stretching sliders
    if (compact) {
        switch (pd.type) {
            case STEWART_PARAM_FLOAT: ImGui::SetNextItemWidth(120); break;
            case STEWART_PARAM_INT:   ImGui::SetNextItemWidth(60); break;
            case STEWART_PARAM_ENUM:  ImGui::SetNextItemWidth(150); break;
            default: break;
        }
    }

    bool changed = false;
    switch (pd.type) {
        case STEWART_PARAM_FLOAT: {
            float range = pd.max_val - pd.min_val;
            const char* fmt = range >= 100 ? "%.0f" : range >= 10 ? "%.1f" : "%.2f";
            if (range > 0.0f && pd.min_val != pd.max_val) {
                // Draggable slider for params with a known range
                changed = ImGui::SliderFloat(label, val, pd.min_val, pd.max_val, fmt);
            } else {
                float step = range >= 100 ? 1.0f : range >= 10 ? 0.1f : 0.01f;
                changed = ImGui::InputFloat(label, val, step, step * 10.0f, fmt);
                if (changed) {
                    if (*val < pd.min_val) *val = pd.min_val;
                    if (*val > pd.max_val) *val = pd.max_val;
                }
            }
            break;
        }
        case STEWART_PARAM_INT: {
            int iv = (int)*val;
            if (ImGui::InputInt(label, &iv, 1, 10)) {
                if (iv < (int)pd.min_val) iv = (int)pd.min_val;
                if (iv > (int)pd.max_val) iv = (int)pd.max_val;
                *val = (float)iv; changed = true;
            }
            break;
        }
        case STEWART_PARAM_BOOL: {
            bool bv = *val != 0.0f;
            if (ImGui::Checkbox(label, &bv)) {
                *val = bv ? 1.0f : 0.0f; changed = true;
            }
            break;
        }
        case STEWART_PARAM_ENUM: {
            int ev = (int)*val;
            if (pd.enum_labels && ImGui::Combo(label, &ev, pd.enum_labels)) {
                *val = (float)ev; changed = true;
            }
            break;
        }
    }
    if (changed) g_app.plugin_mgr.setParam(pd.name, *val);
    if (pd.description && ImGui::IsItemHovered()) ImGui::SetTooltip("%s", pd.description);
    return changed;
}

static void DrawPluginParamsLayout(PluginInstance& plug) {
    if (!plug.info || plug.info->param_count <= 0 || !plug.info->params) return;

    const int N = plug.info->param_count;
    const StewartParamDef* P = plug.info->params;

    static const char* ax_suf[6] = {"_surge","_sway","_heave","_roll","_pitch","_yaw"};
    static const char* ax_pre[6] = {"surge_","sway_","heave_","roll_","pitch_","yaw_"};
    static const char* ax_lbl[6] = {"Surge","Sway","Heave","Roll","Pitch","Yaw"};

    // Per-axis group descriptor
    struct AxGrp {
        std::string key, header;
        int pidx[6];       // param index per axis (-1 = missing)
        int type;          // StewartParamType
    };

    std::vector<AxGrp> groups;
    std::vector<bool> used(N, false);

    // Helper: find or create axis group, record param→axis mapping
    auto assign = [&](const std::string& key, int p, int axis) {
        for (auto& g : groups) {
            if (g.key == key) { g.pidx[axis] = p; used[p] = true; return; }
        }
        AxGrp g;
        g.key = key;
        g.type = P[p].type;
        for (int i = 0; i < 6; i++) g.pidx[i] = -1;
        g.pidx[axis] = p;
        // Derive column header: strip axis word from display_name
        std::string dn = P[p].display_name ? P[p].display_name : key;
        for (int a = 0; a < 6; a++) {
            std::string ax = ax_lbl[a];
            if (dn.size() >= ax.size() && dn.substr(0, ax.size()) == ax) {
                dn = dn.substr(ax.size());
                while (!dn.empty() && dn[0] == ' ') dn.erase(0, 1);
                break;
            }
            if (dn.size() >= ax.size() && dn.substr(dn.size() - ax.size()) == ax) {
                dn = dn.substr(0, dn.size() - ax.size());
                while (!dn.empty() && dn.back() == ' ') dn.pop_back();
                break;
            }
        }
        g.header = dn.empty() ? key : dn;
        used[p] = true;
        groups.push_back(g);
    };

    // Scan all params for axis naming patterns
    for (int p = 0; p < N; p++) {
        const char* nm = P[p].name;
        if (!nm) continue;
        size_t len = strlen(nm);
        bool found = false;
        // Suffix pattern: freq_surge, amp_sway, etc.
        for (int a = 0; a < 6 && !found; a++) {
            size_t sl = strlen(ax_suf[a]);
            if (len > sl && strcmp(nm + len - sl, ax_suf[a]) == 0) {
                assign(std::string(nm, len - sl), p, a);
                found = true;
            }
        }
        // Prefix pattern: surge_en, sway_en, etc.
        for (int a = 0; a < 6 && !found; a++) {
            size_t pl = strlen(ax_pre[a]);
            if (len > pl && strncmp(nm, ax_pre[a], pl) == 0) {
                assign(std::string(nm + pl), p, a);
                found = true;
            }
        }
    }

    // Prune groups with < 3 axes (likely coincidental name matches)
    for (int g = (int)groups.size() - 1; g >= 0; g--) {
        int cnt = 0;
        for (int a = 0; a < 6; a++) if (groups[g].pidx[a] >= 0) cnt++;
        if (cnt < 3) {
            for (int a = 0; a < 6; a++)
                if (groups[g].pidx[a] >= 0) used[groups[g].pidx[a]] = false;
            groups.erase(groups.begin() + g);
        }
    }

    // Collect global params (not in any axis group)
    std::vector<int> globals;
    for (int p = 0; p < N; p++) if (!used[p]) globals.push_back(p);

    // ═══ RENDER GLOBAL PARAMS ═══
    if (!globals.empty()) {
        std::vector<int> bools, others;
        for (int p : globals) {
            if (P[p].type == STEWART_PARAM_BOOL) bools.push_back(p);
            else others.push_back(p);
        }

        // Non-bool params in a responsive label + widget grid (scales to strip width)
        if (!others.empty()) {
            float avail = ImGui::GetContentRegionAvail().x;
            int n = (int)others.size();
            int cols = 1;
            if (avail > 900 && n >= 4) cols = 4;
            else if (avail > 600 && n >= 3) cols = 3;
            else if (avail > 400 && n >= 2) cols = 2;

            if (ImGui::BeginTable("##pg", cols * 2, ImGuiTableFlags_SizingFixedFit)) {
                for (int c = 0; c < cols; c++) {
                    ImGui::TableSetupColumn("##l", ImGuiTableColumnFlags_WidthFixed, 90);
                    ImGui::TableSetupColumn("##w", ImGuiTableColumnFlags_WidthStretch);
                }
                for (int i = 0; i < (int)others.size(); i++) {
                    if (i % cols == 0) ImGui::TableNextRow();
                    int p = others[i];
                    ImGui::TableNextColumn();
                    ImGui::AlignTextToFramePadding();
                    ImGui::TextUnformatted(P[p].display_name ? P[p].display_name : P[p].name);
                    if (P[p].description && ImGui::IsItemHovered())
                        ImGui::SetTooltip("%s", P[p].description);
                    ImGui::TableNextColumn();
                    ImGui::PushID(p);
                    DrawPluginWidget(plug, p, true);
                    ImGui::PopID();
                }
                ImGui::EndTable();
            }
        }

        // Bool params: inline row with spacing
        if (!bools.empty()) {
            if (!others.empty()) ImGui::Spacing();
            for (int i = 0; i < (int)bools.size(); i++) {
                if (i > 0) ImGui::SameLine(0, 16);
                ImGui::PushID(bools[i]);
                DrawPluginWidget(plug, bools[i], false);
                ImGui::PopID();
            }
        }
    }

    // ═══ RENDER PER-AXIS TABLE ═══
    if (!groups.empty()) {
        if (!globals.empty()) { ImGui::Spacing(); ImGui::Separator(); }

        // Separate value groups from bool (enable) groups
        std::vector<int> val_gi, bool_gi;
        for (int g = 0; g < (int)groups.size(); g++) {
            if (groups[g].type == STEWART_PARAM_BOOL) bool_gi.push_back(g);
            else val_gi.push_back(g);
        }

        // Special case: only bool axis groups with 1 column → inline checkbox row
        if (val_gi.empty() && bool_gi.size() == 1) {
            auto& bg = groups[bool_gi[0]];
            ImGui::TextDisabled("%s:", bg.header.c_str());
            ImGui::SameLine();
            for (int a = 0; a < 6; a++) {
                if (a > 0) ImGui::SameLine(0, 12);
                int pidx = bg.pidx[a];
                if (pidx >= 0) {
                    float* val = PluginParamPtr(plug, pidx);
                    if (val) {
                        ImGui::PushID(pidx);
                        bool bv = *val != 0.0f;
                        if (ImGui::Checkbox(ax_lbl[a], &bv)) {
                            *val = bv ? 1.0f : 0.0f;
                            g_app.plugin_mgr.setParam(P[pidx].name, *val);
                        }
                        if (P[pidx].description && ImGui::IsItemHovered())
                            ImGui::SetTooltip("%s", P[pidx].description);
                        ImGui::PopID();
                    }
                }
            }
            return;
        }

        // ── Profiling state (persists across frames) ──
        static bool s_profiling_active = false;
        static float s_obs_min[6] = {0,0,0,0,0,0};
        static float s_obs_max[6] = {0,0,0,0,0,0};
        static bool s_obs_has_data = false;

        // Detect if this plugin has min/max groups (for auto-cal support)
        int min_gi = -1, max_gi = -1;
        for (int g = 0; g < (int)groups.size(); g++) {
            std::string k = groups[g].key;
            // Normalize to lowercase for comparison
            for (auto& c : k) c = (char)tolower(c);
            if (k == "min") min_gi = g;
            else if (k == "max") max_gi = g;
        }
        bool has_min_max = (min_gi >= 0 && max_gi >= 0);

        // Update observed min/max from live raw_input while profiling
        if (s_profiling_active && plug.active) {
            bool any_nonzero = false;
            for (int a = 0; a < 6; a++) {
                if (plug.last_raw_input[a] != 0.0f) any_nonzero = true;
            }
            if (any_nonzero) {
                if (!s_obs_has_data) {
                    for (int a = 0; a < 6; a++) {
                        s_obs_min[a] = plug.last_raw_input[a];
                        s_obs_max[a] = plug.last_raw_input[a];
                    }
                    s_obs_has_data = true;
                } else {
                    for (int a = 0; a < 6; a++) {
                        if (plug.last_raw_input[a] < s_obs_min[a]) s_obs_min[a] = plug.last_raw_input[a];
                        if (plug.last_raw_input[a] > s_obs_max[a]) s_obs_max[a] = plug.last_raw_input[a];
                    }
                }
            }
        }

        // ── Profiling toolbar ──
        if (has_min_max) {
            if (s_profiling_active) {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.7f, 0.2f, 0.2f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.85f, 0.3f, 0.3f, 1.0f));
                if (ImGui::SmallButton("Stop Profiling")) {
                    s_profiling_active = false;
                }
                ImGui::PopStyleColor(2);
            } else {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.15f, 0.5f, 0.3f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.2f, 0.65f, 0.4f, 1.0f));
                if (ImGui::SmallButton("Profile")) {
                    s_profiling_active = true;
                    s_obs_has_data = false;
                    for (int a = 0; a < 6; a++) { s_obs_min[a] = 0; s_obs_max[a] = 0; }
                }
                ImGui::PopStyleColor(2);
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("Start recording observed min/max raw values.\nDrive around in-game to capture the range of each axis.");
            }

            if (s_obs_has_data) {
                ImGui::SameLine();
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.45f, 0.7f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.3f, 0.55f, 0.8f, 1.0f));
                if (ImGui::SmallButton("Auto-Cal")) {
                    // Apply observed min/max to the actual min/max params
                    for (int a = 0; a < 6; a++) {
                        if (min_gi >= 0 && groups[min_gi].pidx[a] >= 0) {
                            float* val = PluginParamPtr(plug, groups[min_gi].pidx[a]);
                            if (val) {
                                float obs = s_obs_min[a];
                                // Round to 1 decimal place and add 5% headroom
                                obs *= 1.05f;
                                obs = floorf(obs * 10.0f) / 10.0f;
                                const StewartParamDef& pd = P[groups[min_gi].pidx[a]];
                                if (obs < pd.min_val) obs = pd.min_val;
                                if (obs > pd.max_val) obs = pd.max_val;
                                *val = obs;
                                g_app.plugin_mgr.setParam(pd.name, obs);
                            }
                        }
                        if (max_gi >= 0 && groups[max_gi].pidx[a] >= 0) {
                            float* val = PluginParamPtr(plug, groups[max_gi].pidx[a]);
                            if (val) {
                                float obs = s_obs_max[a];
                                obs *= 1.05f;
                                obs = ceilf(obs * 10.0f) / 10.0f;
                                const StewartParamDef& pd = P[groups[max_gi].pidx[a]];
                                if (obs < pd.min_val) obs = pd.min_val;
                                if (obs > pd.max_val) obs = pd.max_val;
                                *val = obs;
                                g_app.plugin_mgr.setParam(pd.name, obs);
                            }
                        }
                    }
                }
                ImGui::PopStyleColor(2);
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("Apply observed min/max to parameters (with 5%% headroom).");

                ImGui::SameLine();
                if (ImGui::SmallButton("Reset")) {
                    s_obs_has_data = false;
                    for (int a = 0; a < 6; a++) { s_obs_min[a] = 0; s_obs_max[a] = 0; }
                }
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("Clear observed min/max data.");
            }

            if (s_profiling_active) {
                ImGui::SameLine();
                ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.3f, 1.0f), "PROFILING...");
            }
        }

        // General case: table with Axis + param columns + enable columns + observed columns
        bool show_obs = has_min_max && s_obs_has_data;
        int n_cols = 1 + (int)val_gi.size() + (int)bool_gi.size();
        if (show_obs) n_cols += 3;  // Raw, Obs Min, Obs Max
        ImGuiTableFlags tf = ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
            ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_PadOuterX;

        if (ImGui::BeginTable("##axtbl", n_cols, tf)) {
            ImGui::TableSetupColumn("Axis", ImGuiTableColumnFlags_WidthFixed, 46);
            for (int g : val_gi) {
                float col_w = 130;
                if (!groups[g].key.empty()) {
                    int sample = -1;
                    for (int a = 0; a < 6 && sample < 0; a++) sample = groups[g].pidx[a];
                    if (sample >= 0 && P[sample].type == STEWART_PARAM_ENUM) col_w = 160;
                    else if (sample >= 0 && P[sample].type == STEWART_PARAM_INT) col_w = 90;
                }
                ImGui::TableSetupColumn(groups[g].header.c_str(), ImGuiTableColumnFlags_WidthFixed, col_w);
            }
            for (int g : bool_gi)
                ImGui::TableSetupColumn(groups[g].header.c_str(), ImGuiTableColumnFlags_WidthFixed, 30);
            if (show_obs) {
                ImGui::TableSetupColumn("Raw", ImGuiTableColumnFlags_WidthFixed, 56);
                ImGui::TableSetupColumn("Obs Min", ImGuiTableColumnFlags_WidthFixed, 56);
                ImGui::TableSetupColumn("Obs Max", ImGuiTableColumnFlags_WidthFixed, 56);
            }
            ImGui::TableHeadersRow();

            for (int a = 0; a < 6; a++) {
                ImGui::TableNextRow();
                ImGui::PushID(a);
                ImGui::TableNextColumn();
                ImGui::AlignTextToFramePadding();
                ImGui::TextUnformatted(ax_lbl[a]);

                for (int g : val_gi) {
                    ImGui::TableNextColumn();
                    int pidx = groups[g].pidx[a];
                    if (pidx >= 0) {
                        ImGui::PushID(pidx);
                        DrawPluginWidget(plug, pidx, true);
                        ImGui::PopID();
                    }
                }

                for (int g : bool_gi) {
                    ImGui::TableNextColumn();
                    int pidx = groups[g].pidx[a];
                    if (pidx >= 0) {
                        float* val = PluginParamPtr(plug, pidx);
                        if (val) {
                            ImGui::PushID(pidx);
                            bool bv = *val != 0.0f;
                            if (ImGui::Checkbox("##en", &bv)) {
                                *val = bv ? 1.0f : 0.0f;
                                g_app.plugin_mgr.setParam(P[pidx].name, *val);
                            }
                            if (P[pidx].description && ImGui::IsItemHovered())
                                ImGui::SetTooltip("%s", P[pidx].description);
                            ImGui::PopID();
                        }
                    }
                }

                if (show_obs) {
                    // Raw (live)
                    ImGui::TableNextColumn();
                    float raw = plug.last_raw_input[a];
                    ImGui::TextColored(ImVec4(0.6f, 0.8f, 1.0f, 0.9f), "%+.2f", raw);

                    // Obs Min
                    ImGui::TableNextColumn();
                    ImGui::TextColored(ImVec4(0.4f, 0.8f, 0.5f, 1.0f), "%.2f", s_obs_min[a]);

                    // Obs Max
                    ImGui::TableNextColumn();
                    ImGui::TextColored(ImVec4(0.9f, 0.7f, 0.3f, 1.0f), "%.2f", s_obs_max[a]);
                }

                ImGui::PopID();
            }
            ImGui::EndTable();
        }
    }

}

// ── Input Panel (combobox card selector + per-source content) ────────

static void DrawInputPanel() {
    if (!s_show_input) return;
    if (ImGui::Begin("Input", &s_show_input)) {
        const char* axis_labels[] = {"Surge", "Sway", "Heave", "Roll", "Pitch", "Yaw"};

        // ══════════════════════════════════════════════════════════════
        //  PER-SOURCE CONTENT (plugin parameters / capture controls)
        // ══════════════════════════════════════════════════════════════

        // Disable all controls while source switch is in progress
        if (g_app.source_switch_active) ImGui::BeginDisabled();

        // ── Capture Playback ──
        if (g_app.input_source == InputSource::CapturePlayback) {
            if (g_app.saved_recordings.empty()) {
                ImGui::TextDisabled("No saved captures.");
                ImGui::TextDisabled("Record in Data Streams and save to library first.");
            } else {
                // ── Controls (always visible) ──
                // Speed + Loop on one line
                ImGui::AlignTextToFramePadding();
                ImGui::Text("Speed");
                ImGui::SameLine();
                ImGui::SetNextItemWidth(80);
                int spd_pct = (int)(g_app.capture_speed * 100.0f + 0.5f);
                if (ImGui::SliderInt("##spd", &spd_pct, 10, 200, "%d%%")) {
                    g_app.capture_speed = spd_pct / 100.0f;
                }
                ImGui::SameLine();
                ImGui::Checkbox("Loop", &g_app.capture_loop);

                // Play / Stop buttons
                {
                    bool can_play = !g_app.capture_playing && g_app.capture_playback_idx >= 0 && g_app.motion_started;
                    float btn_w = (ImGui::GetContentRegionAvail().x - 4) * 0.5f;

                    if (!can_play) ImGui::BeginDisabled();
                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.15f, 0.55f, 0.35f, 1.0f));
                    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.2f, 0.7f, 0.45f, 1.0f));
                    if (ImGui::Button("Play", ImVec2(btn_w, 26))) {
                        g_app.startCapturePlayback(g_app.capture_playback_idx);
                    }
                    ImGui::PopStyleColor(2);
                    if (!can_play) ImGui::EndDisabled();
                    if (!can_play && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled) && !g_app.motion_started)
                        ImGui::SetTooltip("Start motion first.");

                    ImGui::SameLine();

                    bool can_stop = g_app.capture_playing;
                    if (!can_stop) ImGui::BeginDisabled();
                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.75f, 0.18f, 0.18f, 1.0f));
                    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.90f, 0.28f, 0.28f, 1.0f));
                    if (ImGui::Button("Stop", ImVec2(-1, 26))) {
                        g_app.stopCapturePlayback();
                    }
                    ImGui::PopStyleColor(2);
                    if (!can_stop) ImGui::EndDisabled();
                }

                // ── Progress bar (when playing) ──
                if (g_app.capture_playing &&
                    g_app.capture_playback_idx >= 0 &&
                    g_app.capture_playback_idx < (int)g_app.saved_recordings.size())
                {
                    auto& sr = g_app.saved_recordings[g_app.capture_playback_idx];
                    double elapsed = (g_app.frame_time - g_app.capture_start_time) * (double)g_app.capture_speed;
                    double dur = sr.duration();
                    float progress = dur > 0 ? (float)(elapsed / dur) : 0.0f;
                    if (progress < 0.0f) progress = 0.0f;
                    if (progress > 1.0f) progress = 1.0f;

                    const char* phase_str = "";
                    ImU32 bar_col = IM_COL32(60, 160, 60, 255);
                    switch (g_app.capture_ramp_phase) {
                        case App::CaptureRampPhase::RampIn:
                            phase_str = "RAMP IN"; bar_col = IM_COL32(60, 200, 120, 255); break;
                        case App::CaptureRampPhase::Playing:
                            phase_str = "PLAYING"; bar_col = IM_COL32(50, 165, 230, 255); break;
                        case App::CaptureRampPhase::RampOut:
                            phase_str = "RAMP OUT"; bar_col = IM_COL32(230, 140, 50, 255); break;
                        case App::CaptureRampPhase::HomeHold:
                            phase_str = "HOME"; bar_col = IM_COL32(100, 100, 180, 255); break;
                    }

                    ImGui::Spacing();
                    ImVec2 pos = ImGui::GetCursorScreenPos();
                    float w = ImGui::GetContentRegionAvail().x;
                    float h = 22.0f;
                    ImDrawList* dl = ImGui::GetWindowDrawList();
                    dl->AddRectFilled(pos, ImVec2(pos.x + w, pos.y + h),
                                      IM_COL32(30, 30, 38, 255), 4.0f);
                    float fill_w = w * progress;
                    if (fill_w > 0.0f)
                        dl->AddRectFilled(pos, ImVec2(pos.x + fill_w, pos.y + h),
                                          bar_col, 4.0f);
                    char bar_text[128];
                    snprintf(bar_text, sizeof(bar_text), " %s  %.1f / %.1fs",
                             phase_str, elapsed < 0 ? 0.0 : elapsed, dur);
                    dl->AddText(ImVec2(pos.x + 6, pos.y + 3), IM_COL32(255, 255, 255, 230), bar_text);
                    if (g_app.capture_loop) {
                        const char* loop_txt = "LOOP";
                        ImVec2 lt = ImGui::CalcTextSize(loop_txt);
                        dl->AddText(ImVec2(pos.x + w - lt.x - 8, pos.y + 3),
                                    IM_COL32(255, 255, 255, 160), loop_txt);
                    }
                    ImGui::Dummy(ImVec2(w, h));
                }

                ImGui::Spacing();

                // ── Scrollable card list (always visible) ──
                float avail = ImGui::GetContentRegionAvail().y;
                if (avail < 100.0f) avail = 100.0f;
                ImGui::BeginChild("##capture_cards", ImVec2(-1, avail), ImGuiChildFlags_Border);

                static int s_cap_delete_idx = -1;
                for (int i = 0; i < (int)g_app.saved_recordings.size(); i++) {
                    auto& sr = g_app.saved_recordings[i];
                    bool selected = (g_app.capture_playback_idx == i);

                    const char* src_icon; const char* src_label; ImVec4 src_color;
                    GetCaptureSourceBadge(sr.source, &src_icon, &src_label, &src_color);

                    char dt_buf[64];
                    FormatDateTime(sr.created_time, dt_buf, sizeof(dt_buf));

                    // Compute data range
                    float gmin = 1e9f, gmax = -1e9f;
                    for (auto& s : sr.samples) {
                        for (int a = 0; a < 6; a++) {
                            if (s.input[a] < gmin) gmin = s.input[a];
                            if (s.input[a] > gmax) gmax = s.input[a];
                        }
                    }
                    bool flat = (fabsf(gmax - gmin) < 0.01f);

                    ImGui::PushID(i);

                    // Card area
                    ImVec2 card_start = ImGui::GetCursorScreenPos();
                    float card_w = ImGui::GetContentRegionAvail().x;
                    float card_h = 58.0f;

                    // Selectable background
                    if (selected) {
                        ImGui::PushStyleColor(ImGuiCol_Header,        ImVec4(src_color.x * 0.2f, src_color.y * 0.2f, src_color.z * 0.2f, 0.6f));
                        ImGui::PushStyleColor(ImGuiCol_HeaderHovered, ImVec4(src_color.x * 0.3f, src_color.y * 0.3f, src_color.z * 0.3f, 0.7f));
                    }
                    if (ImGui::Selectable("##cap_card", selected, 0, ImVec2(card_w, card_h))) {
                        g_app.capture_playback_idx = i;
                    }
                    // Double-click to play (requires motion started)
                    if (g_app.motion_started && !g_app.capture_playing && ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(0)) {
                        g_app.capture_playback_idx = i;
                        g_app.startCapturePlayback(i);
                    }
                    // Right-click to delete
                    if (ImGui::IsItemHovered() && ImGui::IsMouseClicked(ImGuiMouseButton_Right))
                        s_cap_delete_idx = i;
                    if (selected) ImGui::PopStyleColor(2);

                    // Custom draw over the selectable
                    ImDrawList* dl = ImGui::GetWindowDrawList();
                    float pad_x = 6.0f;
                    float badge_h = 20.0f;
                    float row1_y = card_start.y + 4.0f;
                    float row2_y = row1_y + badge_h + 4.0f;
                    float row3_y = row2_y + 14.0f;

                    // Row 1: Badge + Name
                    DrawBadge(dl, ImVec2(card_start.x + pad_x, row1_y), src_icon, src_color, badge_h);
                    float name_x = card_start.x + pad_x + ImGui::CalcTextSize(src_icon).x + 20.0f;
                    ImU32 name_col = selected
                        ? IM_COL32((int)(src_color.x * 255), (int)(src_color.y * 255), (int)(src_color.z * 255), 255)
                        : IM_COL32(220, 220, 220, 255);
                    dl->AddText(ImVec2(name_x, row1_y + 2.0f), name_col, sr.name);

                    // Row 2: Source label + date
                    char detail1[192];
                    snprintf(detail1, sizeof(detail1), "%s  |  %s", src_label, dt_buf);
                    dl->AddText(ImVec2(card_start.x + pad_x + 6.0f, row2_y),
                                IM_COL32(160, 160, 160, 220), detail1);

                    // Row 3: Duration, samples, data range
                    char detail2[128];
                    if (flat)
                        snprintf(detail2, sizeof(detail2), "%.1fs  %d samples  %.0f Hz  [FLAT @ %.0f%%]",
                            sr.duration(), (int)sr.samples.size(), sr.sample_rate_hz, gmin);
                    else
                        snprintf(detail2, sizeof(detail2), "%.1fs  %d samples  %.0f Hz  [%.0f..%.0f%%]",
                            sr.duration(), (int)sr.samples.size(), sr.sample_rate_hz, gmin, gmax);
                    ImU32 range_col = flat ? IM_COL32(240, 150, 50, 220) : IM_COL32(140, 140, 140, 200);
                    dl->AddText(ImVec2(card_start.x + pad_x + 6.0f, row3_y), range_col, detail2);

                    // Selected indicator dot
                    if (selected) {
                        float dot_x = card_start.x + card_w - 14.0f;
                        float dot_y = card_start.y + card_h * 0.5f;
                        dl->AddCircleFilled(ImVec2(dot_x, dot_y), 5.0f,
                            IM_COL32((int)(src_color.x * 255), (int)(src_color.y * 255),
                                     (int)(src_color.z * 255), 255));
                    }

                    ImGui::PopID();
                }

                // Delete confirmation popup (right-click on card)
                if (s_cap_delete_idx >= 0) {
                    ImGui::OpenPopup("##del_capture");
                }
                if (ImGui::BeginPopup("##del_capture")) {
                    if (s_cap_delete_idx >= 0 && s_cap_delete_idx < (int)g_app.saved_recordings.size()) {
                        ImGui::Text("Delete '%s'?", g_app.saved_recordings[s_cap_delete_idx].name);
                        if (ImGui::Button("Delete", ImVec2(80, 0))) {
                            g_app.deleteRecordingFromLibrary(s_cap_delete_idx);
                            if (g_app.capture_playback_idx >= (int)g_app.saved_recordings.size())
                                g_app.capture_playback_idx = (int)g_app.saved_recordings.size() - 1;
                            s_cap_delete_idx = -1;
                            ImGui::CloseCurrentPopup();
                        }
                        ImGui::SameLine();
                        if (ImGui::Button("Cancel", ImVec2(80, 0))) {
                            s_cap_delete_idx = -1;
                            ImGui::CloseCurrentPopup();
                        }
                    }
                    ImGui::EndPopup();
                }

                ImGui::EndChild();
            }
        }

        // ── Plugin ──
        else if (g_app.input_source == InputSource::Plugin) {
            int pi = g_app.active_plugin_idx;
            if (pi >= 0 && pi < g_app.plugin_mgr.pluginCount()) {
                auto& plug = g_app.plugin_mgr.plugins()[pi];
                if (plug.valid && plug.info) {
                    ImGui::TextColored(ImVec4(0.40f, 0.80f, 0.95f, 1.0f), "%s",
                                       plug.info->name ? plug.info->name : plug.filename.c_str());
                    ImGui::SameLine();
                    ImGui::TextDisabled("v%s by %s",
                                        plug.info->version ? plug.info->version : "?",
                                        plug.info->author ? plug.info->author : "?");
                    if (plug.info->description) {
                        ImGui::TextWrapped("%s", plug.info->description);
                    }

                    if (plug.active) {
                        ImGui::SameLine(ImGui::GetContentRegionAvail().x - 60);
                        ImGui::TextColored(ImVec4(0.2f, 0.83f, 0.6f, 1.0f), "Active");
                    }

                    // Auto-generated parameter controls (smart layout)
                    if (plug.info->param_count > 0 && plug.info->params) {
                        ImGui::Spacing();
                        ImGui::Separator();
                        DrawPluginParamsLayout(plug);
                    }
                }
            } else {
                ImGui::TextDisabled("No plugin selected.");
                ImGui::TextWrapped("Drop .dll files into the plugins/ folder and restart the app.");
            }
        }
    }

    // Close the disabled scope for source switching
    if (g_app.source_switch_active) ImGui::EndDisabled();

    ImGui::End();
}

// ── Console Panel ───────────────────────────────────────────────────

static void DrawConsolePanel() {
    if (!s_show_console) return;
    if (ImGui::Begin("Console", &s_show_console)) {
        // Row 1: entity filter buttons + Pause + Copy + Clear
        if (ImGui::SmallButton("All")) g_app.console_filter = -1;
        for (auto& e : g_app.entities) {
            ImGui::SameLine();
            ImVec4 col = ColorFromFloat4(e.color);
            ImGui::PushStyleColor(ImGuiCol_Button,
                g_app.console_filter == e.id ? ImVec4(col.x*0.5f, col.y*0.5f, col.z*0.5f, 1.0f) : ImVec4(0.2f,0.2f,0.2f,1.0f));
            char btn[32];
            const char* tl = (e.type == EntityType::HIL) ? "HIL" : "SIL";
            snprintf(btn, sizeof(btn), "%s #%d", tl, e.id);
            if (ImGui::SmallButton(btn)) {
                g_app.console_filter = (g_app.console_filter == e.id) ? -1 : e.id;
            }
            ImGui::PopStyleColor();
        }
        ImGui::SameLine();
        // Pause button — toggle freeze
        if (g_app.console_paused) {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.85f, 0.55f, 0.15f, 1.0f));
            if (ImGui::SmallButton("Resume")) {
                g_app.console_paused = false;
                g_app.console_log_frozen.clear();
            }
            ImGui::PopStyleColor();
        } else {
            if (ImGui::SmallButton("Pause")) {
                g_app.console_paused = true;
                g_app.console_log_frozen = g_app.console_log;
            }
        }
        ImGui::SameLine();
        // Copy visible log to clipboard
        if (ImGui::SmallButton("Copy")) {
            std::string text;
            const auto& src = g_app.console_paused ? g_app.console_log_frozen : g_app.console_log;
            for (auto& entry : src) {
                if (g_app.console_filter >= 0 && entry.entity_id != g_app.console_filter) continue;
                char line[512];
                Entity* ce = g_app.findEntity(entry.entity_id);
                const char* ct = (ce && ce->type == EntityType::HIL) ? "HIL" : "SIL";
                snprintf(line, sizeof(line), "[%s #%d/%s] %s\n", ct, entry.entity_id, entry.source, entry.message);
                text += line;
            }
            if (!text.empty())
                ImGui::SetClipboardText(text.c_str());
        }
        ImGui::SameLine();
        if (ImGui::SmallButton("Clear")) {
            g_app.console_log.clear();
            g_app.console_log_frozen.clear();
        }

        // Row 2: log rate, max lines, auto-scroll
        const char* rate_labels[] = {"Off", "1 Hz", "10 Hz", "30 Hz", "60 Hz", "Every Frame"};
        ImGui::SetNextItemWidth(100);
        ImGui::Combo("Log Rate", &g_app.console_log_rate, rate_labels, 6);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(80);
        ImGui::DragInt("Max", &g_app.console_max, 10, 100, 10000);
        ImGui::SameLine();
        ImGui::Checkbox("Auto-scroll", &g_app.console_auto_scroll);
        ImGui::SameLine();
        {
            const auto& src = g_app.console_paused ? g_app.console_log_frozen : g_app.console_log;
            char count_str[48];
            if (g_app.console_paused)
                snprintf(count_str, sizeof(count_str), "PAUSED (%d lines)", (int)src.size());
            else
                snprintf(count_str, sizeof(count_str), "(%d lines)", (int)src.size());
            ImGui::TextDisabled("%s", count_str);
        }

        ImGui::Separator();

        // Log output — rolling window (frozen when paused)
        const auto& log_src = g_app.console_paused ? g_app.console_log_frozen : g_app.console_log;
        ImGui::BeginChild("log_scroll", ImVec2(0, 0), ImGuiChildFlags_None, ImGuiWindowFlags_HorizontalScrollbar);
        for (auto& entry : log_src) {
            if (g_app.console_filter >= 0 && entry.entity_id != g_app.console_filter) continue;

            bool is_global = (entry.entity_id < 0);
            Entity* ent = is_global ? nullptr : g_app.findEntity(entry.entity_id);
            ImVec4 col = ent ? ColorFromFloat4(ent->color) : ImVec4(0.55f, 0.55f, 0.60f, 1.0f);

            ImGui::PushStyleColor(ImGuiCol_Text, col);
            if (is_global) {
                ImGui::Text("[SYS/%s]", entry.source);
            } else {
                const char* lt = (ent && ent->type == EntityType::HIL) ? "HIL" : "SIL";
                ImGui::Text("[%s #%d/%s]", lt, entry.entity_id, entry.source);
            }
            ImGui::PopStyleColor();
            ImGui::SameLine();
            ImGui::TextUnformatted(entry.message);
        }
        if (!g_app.console_paused && g_app.console_auto_scroll && ImGui::GetScrollY() >= ImGui::GetScrollMaxY())
            ImGui::SetScrollHereY(1.0f);
        ImGui::EndChild();
    }
    ImGui::End();
}

// ── Ring Buffer Plot Helper ─────────────────────────────────────────

static void PlotRingBuffer(const char* label, const float* time_buf, const float* data_buf,
                           int head, int count, int capacity, ImVec4 color) {
    if (count == 0) return;
    ImPlot::SetNextLineStyle(color, 1.5f);
    if (count < capacity) {
        ImPlot::PlotLine(label, time_buf, data_buf, count);
    } else {
        int oldest = head;
        int tail_len = capacity - oldest;
        ImPlot::PlotLine(label, &time_buf[oldest], &data_buf[oldest], tail_len);
        if (oldest > 0) {
            ImPlot::SetNextLineStyle(color, 1.5f);
            char hidden[128];
            snprintf(hidden, sizeof(hidden), "##%s_wrap", label);
            ImPlot::PlotLine(hidden, time_buf, data_buf, oldest);
        }
    }
}

// ── Data Streams Panel ────────────────────────────────────────────

static void DrawDataStreamsPanel() {
    if (!s_show_data_streams) return;
    if (ImGui::Begin("Data Streams", &s_show_data_streams)) {

        // Top-level tab bar so every section is always one click away
        if (ImGui::BeginTabBar("##cmp_main_tabs")) {

            // ── Tab: Recording / Playback ──
            if (ImGui::BeginTabItem("Recording")) {
                RecordMode mode = g_app.recording.mode;
                bool is_idle = (mode == RecordMode::Idle);
                bool is_recording = (mode == RecordMode::Recording);

                if (is_idle) {
                    // Record rate selector
                    static const int rate_opts[] = {50, 100, 200, 500, 1000};
                    static const char* rate_labels[] = {"50 Hz", "100 Hz", "200 Hz", "500 Hz", "1000 Hz"};
                    int rate_idx = 2; // default 200
                    for (int i = 0; i < 5; i++) { if (rate_opts[i] == g_app.record_rate_hz) rate_idx = i; }
                    ImGui::SetNextItemWidth(100);
                    if (ImGui::Combo("Sample Rate", &rate_idx, rate_labels, 5)) {
                        g_app.record_rate_hz = rate_opts[rate_idx];
                    }
                    ImGui::SameLine();

                    if (ImGui::Button("Record")) g_app.startRecording();
                } else if (is_recording) {
                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.85f, 0.20f, 0.20f, 1.0f));
                    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.95f, 0.30f, 0.30f, 1.0f));
                    if (ImGui::Button("Stop Recording")) g_app.stopRecording();
                    ImGui::PopStyleColor(2);
                    ImGui::SameLine();
                    double elapsed = g_app.frame_time - g_app.recording.start_time;
                    ImGui::TextColored(ImVec4(0.98f, 0.44f, 0.44f, 1.0f),
                        "REC  %.1fs  (%d samples)", elapsed, (int)g_app.recording.samples.size());

                    // Show active source being recorded
                    const char* src_name = "Manual";
                    ImVec4 src_color = ImVec4(0.3f, 0.7f, 0.3f, 1.0f);
                    if (g_app.input_source == InputSource::CapturePlayback) { src_name = "Capture Playback"; src_color = ImVec4(0.9f, 0.7f, 0.2f, 1.0f); }
                    else if (g_app.input_source == InputSource::Plugin) { src_name = "Plugin"; src_color = ImVec4(0.2f, 0.83f, 0.6f, 1.0f); }
                    ImGui::TextColored(src_color, "Recording from: %s", src_name);
                }

                if (is_idle) {
                    if (!g_app.recording.samples.empty()) {
                        ImGui::SameLine();
                        ImGui::TextDisabled("(%d samples, %.1fs)",
                            (int)g_app.recording.samples.size(), g_app.recording.duration());
                        ImGui::SameLine();
                        if (ImGui::SmallButton("Clear")) {
                            g_app.recording.samples.clear();
                            g_app.log(-1, "record", "Recording cleared");
                        }

                        // Data range indicator
                        {
                            float rmin = 1e9f, rmax = -1e9f;
                            for (auto& s : g_app.recording.samples) {
                                for (int a = 0; a < 6; a++) {
                                    if (s.input[a] < rmin) rmin = s.input[a];
                                    if (s.input[a] > rmax) rmax = s.input[a];
                                }
                            }
                            bool flat = (fabsf(rmax - rmin) < 0.01f);
                            if (flat)
                                ImGui::TextColored(ImVec4(0.95f, 0.6f, 0.2f, 1.0f),
                                    "Data: FLAT @ %.1f%% (input was static during recording)", rmin);
                            else
                                ImGui::TextDisabled("Data range: %.1f%% to %.1f%%", rmin, rmax);
                        }

                        // Save to library
                        ImGui::Separator();
                        static char save_name[64] = "Capture";
                        ImGui::SetNextItemWidth(200);
                        ImGui::InputText("Name", save_name, sizeof(save_name));
                        ImGui::SameLine();
                        if (ImGui::Button("Save to Library")) {
                            g_app.saveRecordingToLibrary(save_name);
                            snprintf(save_name, sizeof(save_name), "Capture %d", (int)g_app.saved_recordings.size());
                        }
                    }
                }

                // Show saved recordings library
                if (!g_app.saved_recordings.empty()) {
                    ImGui::Separator();
                    ImGui::Text("Saved Captures (%d)", (int)g_app.saved_recordings.size());
                    int delete_idx = -1;
                    for (int i = 0; i < (int)g_app.saved_recordings.size(); i++) {
                        auto& sr = g_app.saved_recordings[i];
                        ImGui::PushID(i);

                        const char* src_icon = "MAN"; const char* src_label = "Manual"; ImVec4 src_color;
                        GetCaptureSourceBadge(sr.source, &src_icon, &src_label, &src_color);
                        char dt_buf[64];
                        FormatDateTime(sr.created_time, dt_buf, sizeof(dt_buf));

                        // Name line with badge
                        ImDrawList* dl = ImGui::GetWindowDrawList();
                        ImVec2 cpos = ImGui::GetCursorScreenPos();
                        DrawBadge(dl, cpos, src_icon, src_color, 18.0f);
                        float badge_w = ImGui::CalcTextSize(src_icon).x + 12.0f;
                        ImGui::SetCursorScreenPos(ImVec2(cpos.x + badge_w + 6.0f, cpos.y));
                        ImGui::Text("%s", sr.name);
                        ImGui::SameLine();
                        if (ImGui::SmallButton("x")) delete_idx = i;

                        // Detail line: source, date, duration, samples
                        ImGui::TextDisabled("    %s  |  %s  |  %.1fs  %d samples  %.0f Hz",
                            src_label, dt_buf, sr.duration(), (int)sr.samples.size(), sr.sample_rate_hz);

                        ImGui::PopID();
                    }
                    if (delete_idx >= 0) g_app.deleteRecordingFromLibrary(delete_idx);
                }
                ImGui::EndTabItem();
            }

            // ── Tab: Snapshot ──
            if (ImGui::BeginTabItem("Snapshot")) {
                const char* snap_axis[] = {"Surge", "Sway", "Heave", "Roll", "Pitch", "Yaw"};

                // Active source badge
                const char* src_name = "Manual";
                ImVec4 src_color = ImVec4(0.6f, 0.7f, 0.8f, 1.0f);
                if (g_app.input_source == InputSource::CapturePlayback) {
                    src_name = "Capture Playback"; src_color = ImVec4(0.95f, 0.75f, 0.2f, 1.0f);
                } else if (g_app.input_source == InputSource::Plugin) {
                    src_name = "Plugin"; src_color = ImVec4(0.20f, 0.83f, 0.6f, 1.0f);
                }
                ImGui::TextColored(src_color, "Source: %s", src_name);
                ImGui::SameLine();
                ImGui::TextDisabled("@ %d Hz", g_app.record_rate_hz);

                // ── Input Values ──
                ImGui::Separator();
                ImGui::Text("Input (6-DOF %%)");
                if (ImGui::BeginTable("snap_input", 7, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingFixedFit)) {
                    for (int i = 0; i < 6; i++)
                        ImGui::TableSetupColumn(snap_axis[i], ImGuiTableColumnFlags_WidthFixed, 70);
                    ImGui::TableSetupColumn("Max", ImGuiTableColumnFlags_WidthFixed, 50);
                    ImGui::TableHeadersRow();

                    ImGui::TableNextRow();
                    float max_abs = 0.0f;
                    for (int i = 0; i < 6; i++) {
                        ImGui::TableNextColumn();
                        float v = 0.0f;
                        if (!g_app.entities.empty()) v = g_app.entities[0].state.input_pct[i];
                        float av = fabsf(v);
                        if (av > max_abs) max_abs = av;
                        ImVec4 c = av > 80.0f ? ImVec4(0.98f, 0.44f, 0.44f, 1.0f) :
                                   av > 50.0f ? ImVec4(0.98f, 0.75f, 0.15f, 1.0f) :
                                                ImVec4(0.8f, 0.8f, 0.8f, 1.0f);
                        ImGui::TextColored(c, "%+.1f", v);
                    }
                    ImGui::TableNextColumn();
                    ImGui::Text("%.0f%%", max_abs);
                    ImGui::EndTable();
                }

                // ── Per-Entity Servo Angles + Utilization ──
                int ncols = 1 + (int)g_app.entities.size() + 1;
                ImGui::Separator();
                ImGui::Text("Servo Angles");
                if (ImGui::BeginTable("snap_angles", ncols,
                    ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingFixedFit)) {
                    ImGui::TableSetupColumn("Servo", ImGuiTableColumnFlags_WidthFixed, 50);
                    for (auto& e : g_app.entities)
                        ImGui::TableSetupColumn(e.name, ImGuiTableColumnFlags_WidthFixed, 90);
                    ImGui::TableSetupColumn("Delta", ImGuiTableColumnFlags_WidthFixed, 60);
                    ImGui::TableHeadersRow();

                    for (int s = 0; s < 6; s++) {
                        ImGui::TableNextRow();
                        ImGui::TableNextColumn();
                        ImGui::Text("S%d", s);
                        float mn = 1e9f, mx = -1e9f;
                        for (auto& e : g_app.entities) {
                            float deg = e.state.output_angles_deg[s];
                            if (deg < mn) mn = deg;
                            if (deg > mx) mx = deg;
                            ImGui::TableNextColumn();
                            ImVec4 col = ColorFromFloat4(e.color);
                            ImGui::TextColored(col, "%.1f\xc2\xb0", deg);
                        }
                        ImGui::TableNextColumn();
                        float delta = mx - mn;
                        ImVec4 dc = delta > 5.0f ? ImVec4(0.98f, 0.44f, 0.44f, 1.0f) :
                                    delta > 1.0f ? ImVec4(0.98f, 0.75f, 0.15f, 1.0f) :
                                                   ImVec4(0.2f, 0.83f, 0.6f, 1.0f);
                        ImGui::TextColored(dc, "%.1f\xc2\xb0", delta);
                    }
                    ImGui::EndTable();
                }

                // ── Utilization ──
                ImGui::Separator();
                ImGui::Text("Servo Utilization");
                if (ImGui::BeginTable("snap_util", ncols,
                    ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingFixedFit)) {
                    ImGui::TableSetupColumn("Servo", ImGuiTableColumnFlags_WidthFixed, 50);
                    for (auto& e : g_app.entities)
                        ImGui::TableSetupColumn(e.name, ImGuiTableColumnFlags_WidthFixed, 90);
                    ImGui::TableSetupColumn("Max", ImGuiTableColumnFlags_WidthFixed, 60);
                    ImGui::TableHeadersRow();

                    for (int s = 0; s < 6; s++) {
                        ImGui::TableNextRow();
                        ImGui::TableNextColumn();
                        ImGui::Text("S%d", s);
                        float row_max = 0.0f;
                        for (auto& e : g_app.entities) {
                            float u = e.state.servo_util[s];
                            if (u > row_max) row_max = u;
                            ImGui::TableNextColumn();
                            ImVec4 uc = u > 90.0f ? ImVec4(0.98f, 0.44f, 0.44f, 1.0f) :
                                        u > 70.0f ? ImVec4(0.98f, 0.75f, 0.15f, 1.0f) :
                                                    ImVec4(0.2f, 0.83f, 0.6f, 1.0f);
                            ImGui::TextColored(uc, "%.0f%%", u);
                        }
                        ImGui::TableNextColumn();
                        ImGui::Text("%.0f%%", row_max);
                    }
                    ImGui::EndTable();
                }

                // ── Motor Steps ──
                ImGui::Separator();
                ImGui::Text("Motor Steps");
                if (ImGui::BeginTable("snap_steps", ncols,
                    ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingFixedFit)) {
                    ImGui::TableSetupColumn("Motor", ImGuiTableColumnFlags_WidthFixed, 50);
                    for (auto& e : g_app.entities)
                        ImGui::TableSetupColumn(e.name, ImGuiTableColumnFlags_WidthFixed, 90);
                    ImGui::TableSetupColumn("Delta", ImGuiTableColumnFlags_WidthFixed, 60);
                    ImGui::TableHeadersRow();

                    for (int s = 0; s < 6; s++) {
                        ImGui::TableNextRow();
                        ImGui::TableNextColumn();
                        ImGui::Text("M%d", s);
                        float mn = 1e9f, mx = -1e9f;
                        for (auto& e : g_app.entities) {
                            float st = e.state.output_steps[s];
                            if (st < mn) mn = st;
                            if (st > mx) mx = st;
                            ImGui::TableNextColumn();
                            ImVec4 col = ColorFromFloat4(e.color);
                            ImGui::TextColored(col, "%.0f", st);
                        }
                        ImGui::TableNextColumn();
                        ImGui::Text("%.0f", mx - mn);
                    }
                    ImGui::EndTable();
                }

                ImGui::EndTabItem();
            }

            // ── Tab: Time-Series ──
            if (ImGui::BeginTabItem("Time-Series")) {
                static int chart_mode = 0;
                ImGui::RadioButton("Input (Global)", &chart_mode, 0);
                ImGui::SameLine();
                ImGui::RadioButton("Servo Angles", &chart_mode, 1);
                ImGui::SameLine();
                ImGui::RadioButton("Scaled Input", &chart_mode, 2);

                const char* axis_names[] = {"Surge", "Sway", "Heave", "Roll", "Pitch", "Yaw"};
                const char* y_labels[] = {"Input (%)", "Angle (deg)", "Physical"};

                // Show all 6 axes on one plot, or per-axis tabs
                static bool overlay_all = true;
                ImGui::SameLine();
                ImGui::Checkbox("Overlay All", &overlay_all);

                if (overlay_all && chart_mode == 0) {
                    // Global input: all 6 axes overlaid on one plot
                    if (ImPlot::BeginPlot("##ts_global", ImVec2(-1, -1))) {
                        ImPlot::SetupAxes("Time (s)", y_labels[chart_mode],
                                          ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
                        if (g_app.input_history_count > 0) {
                            static const ImVec4 axis_colors[] = {
                                {0.90f, 0.30f, 0.30f, 1.0f}, {0.30f, 0.90f, 0.30f, 1.0f},
                                {0.30f, 0.50f, 0.90f, 1.0f}, {0.90f, 0.90f, 0.30f, 1.0f},
                                {0.90f, 0.30f, 0.90f, 1.0f}, {0.30f, 0.90f, 0.90f, 1.0f}
                            };
                            for (int a = 0; a < 6; a++) {
                                PlotRingBuffer(axis_names[a],
                                    g_app.input_history_time, g_app.input_history[a],
                                    g_app.input_history_head, g_app.input_history_count,
                                    INPUT_HISTORY_LEN, axis_colors[a]);
                            }
                        }
                        ImPlot::EndPlot();
                    }
                } else {
                    if (ImGui::BeginTabBar("##ts_tabs")) {
                        for (int axis = 0; axis < 6; axis++) {
                            if (ImGui::BeginTabItem(axis_names[axis])) {
                                char plot_id[64];
                                snprintf(plot_id, sizeof(plot_id), "##ts_%d", axis);
                                if (ImPlot::BeginPlot(plot_id, ImVec2(-1, -1))) {
                                    ImPlot::SetupAxes("Time (s)", y_labels[chart_mode],
                                                      ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);

                                    if (chart_mode == 0 && g_app.input_history_count > 0) {
                                        // Global input history
                                        PlotRingBuffer("Input",
                                            g_app.input_history_time, g_app.input_history[axis],
                                            g_app.input_history_head, g_app.input_history_count,
                                            INPUT_HISTORY_LEN, ImVec4(0.2f, 0.83f, 0.6f, 1.0f));
                                    } else {
                                        // Per-entity data
                                        for (auto& e : g_app.entities) {
                                            if (!e.enabled || e.history_count == 0) continue;
                                            ImVec4 col = ColorFromFloat4(e.color);
                                            const float* data = chart_mode == 1 ?
                                                e.history_angles[axis] : e.history_input[axis];
                                            PlotRingBuffer(e.name, e.history_time, data,
                                                           e.history_head, e.history_count, HISTORY_LEN, col);
                                        }
                                    }
                                    ImPlot::EndPlot();
                                }
                                ImGui::EndTabItem();
                            }
                        }
                        ImGui::EndTabBar();
                    }
                }

                // Source info
                const char* src_name = "Manual";
                if (g_app.input_source == InputSource::CapturePlayback) src_name = "Capture Playback";
                else if (g_app.input_source == InputSource::Plugin) src_name = "Plugin";
                ImGui::TextDisabled("Source: %s | History: %d/%d samples @ %d Hz",
                    src_name, g_app.input_history_count, INPUT_HISTORY_LEN, g_app.record_rate_hz);

                ImGui::EndTabItem();
            }

            // ── Tab: Spectrogram ──
            if (ImGui::BeginTabItem("Spectrogram")) {
                static const char* axis_short[] = {"Surge","Sway","Heave","Roll","Pitch","Yaw"};

                // ── Rolling waterfall state (reads from global input_spectrum) ──
                static const int SGRAM_COLS = 200;  // time slices (~20s at 10Hz)
                struct SgramLane {
                    int  axis;
                    bool active;
                    float buf[SGRAM_COLS][SPECTRUM_BINS]; // rolling data
                    int  head;
                    int  count;
                };
                static SgramLane lanes[6] = {};
                static bool s_sgram_inited = false;
                static float s_color_max = 0.0f;   // shared color scale
                static bool s_auto_scale = true;

                if (!s_sgram_inited) {
                    s_sgram_inited = true;
                    memset(lanes, 0, sizeof(lanes));
                    for (int a = 0; a < 6; a++) lanes[a].axis = a;
                    // Default: heave axis active
                    lanes[2].active = true;
                }

                // Push new time slices (~10Hz) from global input spectrum
                static double s_last_push = 0.0;
                if (g_app.frame_time - s_last_push >= 0.1) {
                    s_last_push = g_app.frame_time;
                    float global_peak = 0.0f;
                    for (int a = 0; a < 6; a++) {
                        if (!lanes[a].active) continue;
                        if (g_app.input_history_count < 16) continue;

                        int col = lanes[a].head;
                        for (int b = 0; b < SPECTRUM_BINS; b++) {
                            lanes[a].buf[col][b] = g_app.input_spectrum[a][b];
                            if (lanes[a].buf[col][b] > global_peak)
                                global_peak = lanes[a].buf[col][b];
                        }
                        lanes[a].head = (lanes[a].head + 1) % SGRAM_COLS;
                        if (lanes[a].count < SGRAM_COLS) lanes[a].count++;
                    }
                    if (s_auto_scale && global_peak > 0.001f)
                        s_color_max = s_color_max * 0.95f + global_peak * 0.05f; // smooth
                    if (s_color_max < 0.01f) s_color_max = 0.01f;
                }

                // ── Axis selector (flat checkboxes, no entity tree) ──
                int active_count = 0;
                for (int a = 0; a < 6; a++) if (lanes[a].active) active_count++;

                for (int a = 0; a < 6; a++) {
                    if (a > 0) ImGui::SameLine();
                    ImGui::PushID(a);
                    if (ImGui::Checkbox(axis_short[a], &lanes[a].active)) {
                        if (lanes[a].active) {
                            // Reset lane on enable
                            lanes[a].head = 0;
                            lanes[a].count = 0;
                            memset(lanes[a].buf, 0, sizeof(lanes[a].buf));
                        }
                    }
                    ImGui::PopID();
                }

                ImGui::SameLine(0, 16);
                ImGui::Checkbox("Auto Scale", &s_auto_scale);
                if (!s_auto_scale) {
                    ImGui::SameLine();
                    ImGui::SetNextItemWidth(120);
                    ImGui::DragFloat("Max##cscale", &s_color_max, 0.01f, 0.01f, 100.0f, "%.2f");
                }

                // Recount after possible toggle
                active_count = 0;
                for (int a = 0; a < 6; a++) if (lanes[a].active) active_count++;

                // ── Viridis-like colormap ──
                auto viridis = [](float t) -> ImU32 {
                    t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);
                    // Simplified viridis: dark purple → teal → yellow
                    float r, g, b;
                    if (t < 0.25f) {
                        float s = t / 0.25f;
                        r = 0.27f + s * 0.0f;  g = 0.0f + s * 0.23f;  b = 0.33f + s * 0.27f;
                    } else if (t < 0.5f) {
                        float s = (t - 0.25f) / 0.25f;
                        r = 0.27f - s * 0.07f; g = 0.23f + s * 0.27f; b = 0.60f - s * 0.10f;
                    } else if (t < 0.75f) {
                        float s = (t - 0.5f) / 0.25f;
                        r = 0.20f + s * 0.40f; g = 0.50f + s * 0.20f; b = 0.50f - s * 0.20f;
                    } else {
                        float s = (t - 0.75f) / 0.25f;
                        r = 0.60f + s * 0.40f; g = 0.70f + s * 0.25f; b = 0.30f - s * 0.25f;
                    }
                    return IM_COL32((int)(r*255), (int)(g*255), (int)(b*255), 255);
                };

                // ── Draw heatmap lanes ──
                if (active_count > 0) {
                    float fmax = g_app.input_spectrum_freq_max;
                    if (fmax <= 0.0f) fmax = 30.0f;

                    ImVec2 avail = ImGui::GetContentRegionAvail();
                    float lane_h = (avail.y - active_count * 20.0f) / (float)active_count;
                    if (lane_h < 60.0f) lane_h = 60.0f;

                    ImDrawList* dl = ImGui::GetWindowDrawList();

                    for (int a = 0; a < 6; a++) {
                        if (!lanes[a].active) continue;

                        // Label
                        ImGui::Text("%s  (0-%.0f Hz)", axis_short[a], fmax);

                        ImVec2 cpos = ImGui::GetCursorScreenPos();
                        float w = avail.x;
                        float h = lane_h;

                        // Reserve space
                        ImGui::Dummy(ImVec2(w, h));

                        // Draw heatmap cells
                        int cols = lanes[a].count;
                        if (cols < 1) continue;
                        if (cols > SGRAM_COLS) cols = SGRAM_COLS;

                        float cell_w = w / (float)SGRAM_COLS;
                        float cell_h = h / (float)SPECTRUM_BINS;
                        float cmax = s_color_max;

                        dl->PushClipRect(cpos, ImVec2(cpos.x + w, cpos.y + h), true);

                        for (int c = 0; c < cols; c++) {
                            // newest on right, oldest on left
                            int buf_col = (lanes[a].head - 1 - c + SGRAM_COLS) % SGRAM_COLS;
                            float x = cpos.x + w - (c + 1) * cell_w;

                            for (int b = 0; b < SPECTRUM_BINS; b++) {
                                float mag = lanes[a].buf[buf_col][b];
                                float t = mag / cmax;
                                // Y: low freq at bottom, high freq at top
                                float y = cpos.y + h - (b + 1) * cell_h;
                                dl->AddRectFilled(
                                    ImVec2(x, y),
                                    ImVec2(x + cell_w + 0.5f, y + cell_h + 0.5f),
                                    viridis(t));
                            }
                        }

                        // Border
                        dl->AddRect(cpos, ImVec2(cpos.x + w, cpos.y + h),
                                    IM_COL32(60, 70, 90, 200), 0.0f, 0, 1.0f);

                        // Frequency labels on right edge
                        char lbl[16];
                        for (int fi = 0; fi <= 4; fi++) {
                            float frac = (float)fi / 4.0f;
                            float fy = cpos.y + h * (1.0f - frac);
                            snprintf(lbl, sizeof(lbl), "%.0f", frac * fmax);
                            dl->AddText(ImVec2(cpos.x + w - 30, fy - 6),
                                        IM_COL32(200, 200, 200, 180), lbl);
                        }

                        dl->PopClipRect();
                    }
                } else {
                    ImGui::TextDisabled("No axes selected. Check one or more axes above.");
                }

                ImGui::EndTabItem();
            }

            ImGui::EndTabBar();
        }
    }
    ImGui::End();
}

// ── Default Dock Layout ─────────────────────────────────────────────

static void BuildDefaultLayout(ImGuiID dockspace_id) {
    ImGui::DockBuilderRemoveNode(dockspace_id);
    ImGui::DockBuilderAddNode(dockspace_id, ImGuiDockNodeFlags_DockSpace);

    ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::DockBuilderSetNodeSize(dockspace_id, vp->WorkSize);

    // Split: left column for controls, then bottom strip for Data Streams
    ImGuiID left_id, rest_id;
    ImGui::DockBuilderSplitNode(dockspace_id, ImGuiDir_Left, 0.22f, &left_id, &rest_id);

    // Bottom strip from the remaining area
    ImGuiID center_id, bottom_id;
    ImGui::DockBuilderSplitNode(rest_id, ImGuiDir_Down, 0.45f, &bottom_id, &center_id);

    // Console on the left (Input is now the fixed strip below toolbar)
    ImGui::DockBuilderDockWindow("Console", left_id);

    // Data Streams on the bottom
    ImGui::DockBuilderDockWindow("Data Streams", bottom_id);

    // Center stays empty (entity cards float over it)

    ImGui::DockBuilderFinish(dockspace_id);
}

// ── Pre-Frame Hook ──────────────────────────────────────────────────
// Called BEFORE ImGui::NewFrame() so docking state from a loaded .ini
// is already present when the dockspace is (re)built this frame.

void PreFrameUI() {
    if (s_pending_ini_load[0] != '\0') {
        ImGui::LoadIniSettingsFromDisk(s_pending_ini_load);
        s_pending_ini_load[0] = '\0';
    }
}

// ── Main Draw Function ──────────────────────────────────────────────

void DrawUI() {
    DrawMainMenuBar();
    DrawToolbar();
    DrawInputStrip();

    // Explicit full-window dockspace with a FIXED string ID.
    // DockSpaceOverViewport() derives its ID from viewport WorkPos/WorkSize,
    // which we modify dynamically — this makes the ID non-deterministic and
    // breaks save/load because the ID in the .ini never matches at runtime.
    // Using a named DockSpace with ImGui::DockSpaceOverViewport's window flags
    // but a fixed ID ensures the Docking [Data] section in the .ini always
    // resolves to the same node tree.
    ImGuiViewport* main_vp = ImGui::GetMainViewport();
    float top_offset = 82.0f + s_input_strip_h;
    ImVec2 ds_pos  = ImVec2(main_vp->WorkPos.x, main_vp->WorkPos.y + top_offset);
    ImVec2 ds_size = ImVec2(main_vp->WorkSize.x, main_vp->WorkSize.y - top_offset);

    ImGui::SetNextWindowPos(ds_pos);
    ImGui::SetNextWindowSize(ds_size);
    ImGui::SetNextWindowViewport(main_vp->ID);
    ImGuiWindowFlags ds_flags =
        ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse |
        ImGuiWindowFlags_NoResize   | ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoNavFocus | ImGuiWindowFlags_NoBackground |
        ImGuiWindowFlags_NoDocking;
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
    ImGui::Begin("##MainDockspace", nullptr, ds_flags);
    ImGui::PopStyleVar(3);
    ImGuiID dockspace_id = ImGui::GetID("MainDockspace");
    ImGui::DockSpace(dockspace_id, ImVec2(0, 0), ImGuiDockNodeFlags_PassthruCentralNode);
    ImGui::End();

    // Build default layout on first run (no saved .ini), on reset, or after
    // a workspace ini was loaded (s_pending_first_frame resets this flag).
    static bool first_frame = true;
    if (s_pending_first_frame) {
        s_pending_first_frame = false;
        first_frame = true;  // re-evaluate whether dockspace needs default layout
    }
    if (first_frame) {
        first_frame = false;
        ImGuiDockNode* node = ImGui::DockBuilderGetNode(dockspace_id);
        if (!node || node->IsEmpty()) {
            BuildDefaultLayout(dockspace_id);
        }
    }
    if (s_reset_layout) {
        s_reset_layout = false;
        BuildDefaultLayout(dockspace_id);
    }

    // Global panels (docked by default layout)
    DrawConsolePanel();
    DrawDataStreamsPanel();
    DrawDynamicsPanel();
    DrawTestHarnessPanel();

    // Per-entity panels — float as undocked windows
    // Copy IDs first since DrawEntityCard can remove entities
    std::vector<int> ids;
    for (auto& e : g_app.entities) ids.push_back(e.id);
    for (int id : ids) {
        Entity* e = g_app.findEntity(id);
        if (e) {
            DrawEntityCard(*e);
        }
    }
}
