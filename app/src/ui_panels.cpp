#include "ui_panels.h"
#include "serial_port.h"
#include "platform_viz.h"
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

// ── Layout Save / Load ──────────────────────────────────────────────

static const char* LAYOUTS_DIR = "layouts";

static void EnsureLayoutDir() {
    fs::create_directories(LAYOUTS_DIR);
}

static std::vector<std::string> EnumerateLayouts() {
    std::vector<std::string> names;
    EnsureLayoutDir();
    for (auto& entry : fs::directory_iterator(LAYOUTS_DIR)) {
        if (entry.is_regular_file() && entry.path().extension() == ".ini") {
            names.push_back(entry.path().stem().string());
        }
    }
    std::sort(names.begin(), names.end());
    return names;
}

static bool SaveLayout(const char* name) {
    EnsureLayoutDir();
    // First flush current ImGui ini to disk so it's up to date
    ImGui::SaveIniSettingsToDisk(ImGui::GetIO().IniFilename);
    // Copy the ini file to layouts/<name>.ini
    fs::path src(ImGui::GetIO().IniFilename);
    fs::path dst = fs::path(LAYOUTS_DIR) / (std::string(name) + ".ini");
    std::error_code ec;
    fs::copy_file(src, dst, fs::copy_options::overwrite_existing, ec);

    // Also save current settings (entities + app state) alongside
    g_app.saveSettings();
    fs::path settings_src("stewart_settings.json");
    fs::path settings_dst = fs::path(LAYOUTS_DIR) / (std::string(name) + ".json");
    fs::copy_file(settings_src, settings_dst, fs::copy_options::overwrite_existing, ec);

    return !ec;
}

static bool LoadLayout(const char* name) {
    fs::path src = fs::path(LAYOUTS_DIR) / (std::string(name) + ".ini");
    if (!fs::exists(src)) return false;
    ImGui::LoadIniSettingsFromDisk(src.string().c_str());
    // Also copy to the active ini so it persists on next launch
    std::error_code ec;
    fs::copy_file(src, fs::path(ImGui::GetIO().IniFilename), fs::copy_options::overwrite_existing, ec);

    // Restore entities + app state from companion JSON
    fs::path settings_src = fs::path(LAYOUTS_DIR) / (std::string(name) + ".json");
    if (fs::exists(settings_src)) {
        fs::path settings_dst("stewart_settings.json");
        fs::copy_file(settings_src, settings_dst, fs::copy_options::overwrite_existing, ec);
        g_app.loadSettings();
    }

    return true;
}

static bool DeleteLayout(const char* name) {
    fs::path p = fs::path(LAYOUTS_DIR) / (std::string(name) + ".ini");
    fs::path pj = fs::path(LAYOUTS_DIR) / (std::string(name) + ".json");
    std::error_code ec;
    fs::remove(pj, ec);  // remove companion JSON if exists
    return fs::remove(p, ec);
}

// Layout popup / reset state
static bool s_show_save_popup = false;
static char s_layout_name_buf[128] = "";
static bool s_reset_layout = false;

// Panel visibility (toggled from View menu, X button on windows)
static bool s_show_input       = true;
static bool s_show_console     = true;
static bool s_show_data_streams = true;
static bool s_show_dynamics    = true;
static int  s_selected_dynamics_id = -1;  // entity ID for global Dynamics panel (-1 = auto-select first)
static int  s_dyn_preset_idx = -1;       // currently selected dynamics preset index

// Dynamics staging buffer (file-scope so Copy/Paste can access it)
struct DynStaging {
    MotionCueingConfig mca;
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
            if (ImGui::MenuItem("Save Config"))   { /* TODO */ }
            if (ImGui::MenuItem("Load Config"))   { /* TODO */ }
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
        if (ImGui::BeginMenu("Layout")) {
            if (ImGui::MenuItem("Save Layout...")) {
                s_show_save_popup = true;
                s_layout_name_buf[0] = '\0';
            }

            ImGui::Separator();
            auto layouts = EnumerateLayouts();
            if (layouts.empty()) {
                ImGui::TextDisabled("(no saved layouts)");
            } else {
                for (auto& ln : layouts) {
                    if (ImGui::BeginMenu(ln.c_str())) {
                        if (ImGui::MenuItem("Load")) {
                            LoadLayout(ln.c_str());
                            g_app.log(-1, "layout", "Loaded layout: %s", ln.c_str());
                        }
                        if (ImGui::MenuItem("Delete")) {
                            DeleteLayout(ln.c_str());
                            g_app.log(-1, "layout", "Deleted layout: %s", ln.c_str());
                        }
                        ImGui::EndMenu();
                    }
                }
            }

            ImGui::Separator();
            if (ImGui::MenuItem("Reset to Default")) {
                s_reset_layout = true;
                g_app.log(-1, "layout", "Layout reset to default");
            }
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("View")) {
            ImGui::MenuItem("Input",        nullptr, &s_show_input);
            ImGui::MenuItem("Console",      nullptr, &s_show_console);
            ImGui::MenuItem("Data Streams", nullptr, &s_show_data_streams);
            ImGui::MenuItem("Dynamics",     nullptr, &s_show_dynamics);
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

    // ── Save Layout Popup ──
    if (s_show_save_popup) {
        ImGui::OpenPopup("Save Layout");
        s_show_save_popup = false;
    }
    if (ImGui::BeginPopupModal("Save Layout", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::Text("Enter a name for this layout:");
        ImGui::SetNextItemWidth(280);
        bool enter_pressed = ImGui::InputText("##layout_name", s_layout_name_buf, sizeof(s_layout_name_buf),
                                               ImGuiInputTextFlags_EnterReturnsTrue);

        // Auto-focus the input on first frame
        if (ImGui::IsWindowAppearing()) ImGui::SetKeyboardFocusHere(-1);

        bool valid = s_layout_name_buf[0] != '\0';

        if (!valid) ImGui::BeginDisabled();
        bool do_save = ImGui::Button("Save", ImVec2(120, 0)) || (enter_pressed && valid);
        if (!valid) ImGui::EndDisabled();

        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(120, 0))) {
            ImGui::CloseCurrentPopup();
        }

        if (do_save) {
            if (SaveLayout(s_layout_name_buf)) {
                g_app.log(-1, "layout", "Saved layout: %s", s_layout_name_buf);
            } else {
                g_app.log(-1, "layout", "Failed to save layout: %s", s_layout_name_buf);
            }
            ImGui::CloseCurrentPopup();
        }

        ImGui::EndPopup();
    }
}

// ── Toolbar (ribbon-style, visual mockup) ────────────────────────────

static void ToolbarSeparator() {
    ImGui::SameLine(0, 4);
    float y0 = ImGui::GetCursorScreenPos().y;
    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddLine(ImVec2(ImGui::GetCursorScreenPos().x, y0 + 2),
                ImVec2(ImGui::GetCursorScreenPos().x, y0 + 38),
                IM_COL32(80, 80, 80, 180), 1.0f);
    ImGui::SameLine(0, 8);
}

static void ToolbarGroupLabel(const char* label) {
    ImVec2 pos = ImGui::GetCursorScreenPos();
    ImGui::GetWindowDrawList()->AddText(ImVec2(pos.x, pos.y + 28),
        IM_COL32(120, 120, 120, 200), label);
}

static void DrawToolbar() {
    ImGuiViewport* vp = ImGui::GetMainViewport();
    float menu_h = ImGui::GetFrameHeight();  // main menu bar height
    float toolbar_h = 52.0f;

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

        // ── SOURCE ──────────────────────────────────────────────
        ToolbarGroupLabel("Source");
        {
            static const char* source_names[] = { "Manual", "Capture", "Plugin" };
            int src = (int)g_app.input_source;
            ImGui::PushItemWidth(90);
            ImGui::Combo("##tb_src", &src, source_names, IM_ARRAYSIZE(source_names));
            ImGui::PopItemWidth();
            ImGui::SameLine();
            // Status dot
            bool connected = false;
            switch (g_app.input_source) {
                case InputSource::Plugin:         connected = g_app.active_plugin_idx >= 0; break;
                case InputSource::CapturePlayback:connected = g_app.capture_playing; break;
                default:                          connected = true; break;
            }
            ImVec4 dot_col = connected ? ImVec4(0.2f, 0.9f, 0.3f, 1.0f) : ImVec4(0.6f, 0.6f, 0.6f, 0.6f);
            ImGui::TextColored(dot_col, connected ? "LIVE" : "IDLE");
        }

        ToolbarSeparator();

        // ── MOTION ──────────────────────────────────────────────
        ToolbarGroupLabel("Motion");
        {
            bool running = g_app.motion_started;
            if (running) {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.15f, 0.55f, 0.15f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.2f, 0.65f, 0.2f, 1.0f));
            } else {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.35f, 0.35f, 0.35f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.45f, 0.45f, 0.45f, 1.0f));
            }
            ImGui::Button(running ? "STOP" : "START", ImVec2(60, 26));
            ImGui::PopStyleColor(2);

            ImGui::SameLine();

            // E-stop (bright red)
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.75f, 0.1f, 0.1f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.9f, 0.15f, 0.15f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(1.0f, 0.0f, 0.0f, 1.0f));
            ImGui::Button("E-STOP", ImVec2(60, 26));
            ImGui::PopStyleColor(3);

            ImGui::SameLine();

            // Global intensity mini-slider
            if (!g_app.entities.empty()) {
                float intensity = g_app.entities[0].config.intensity;
                ImGui::PushItemWidth(80);
                ImGui::VSliderFloat("##tb_int", ImVec2(18, 26), &intensity, 0.0f, 100.0f, "");
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("Intensity: %.0f%%", intensity);
                ImGui::PopItemWidth();
                ImGui::SameLine();
                ImGui::TextColored(ImVec4(0.7f, 0.85f, 1.0f, 1.0f), "%.0f%%", intensity);
            }
        }

        ToolbarSeparator();

        // ── RECORDING ───────────────────────────────────────────
        ToolbarGroupLabel("Recording");
        {
            bool is_recording = (g_app.recording.mode == RecordMode::Recording);
            bool is_playing   = g_app.capture_playing;

            // Record button (red when recording)
            if (is_recording) {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.8f, 0.15f, 0.15f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.9f, 0.2f, 0.2f, 1.0f));
            } else {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.35f, 0.35f, 0.35f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.45f, 0.45f, 0.45f, 1.0f));
            }
            ImGui::Button("REC", ImVec2(36, 26));
            ImGui::PopStyleColor(2);

            ImGui::SameLine();

            // Play button (green when playing)
            if (is_playing) {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.15f, 0.55f, 0.15f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.2f, 0.65f, 0.2f, 1.0f));
            } else {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.35f, 0.35f, 0.35f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.45f, 0.45f, 0.45f, 1.0f));
            }
            ImGui::Button("PLAY", ImVec2(42, 26));
            ImGui::PopStyleColor(2);

            ImGui::SameLine();

            ImGui::Button("STOP", ImVec2(42, 26));
        }

        ToolbarSeparator();

        // ── PLATFORM ────────────────────────────────────────────
        ToolbarGroupLabel("Platform");
        {
            int n_ent = (int)g_app.entities.size();
            int n_hil = 0, n_sil = 0;
            for (auto& e : g_app.entities) {
                if (e.type == EntityType::HIL) n_hil++;
                else n_sil++;
            }

            // Entity badges
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
        // Header row: type badge + enable toggle + settings + remove
        ImGui::TextColored(col, "%s", type_str);
        ImGui::SameLine();
        ImGui::Checkbox("Enabled", &e.enabled);
        ImGui::SameLine();
        if (ImGui::SmallButton("Settings")) e.show_settings = !e.show_settings;
        ImGui::SameLine();
        if (ImGui::SmallButton("Platform")) e.show_platform = !e.show_platform;
        ImGui::SameLine();
        if (ImGui::SmallButton("Dynamics")) { s_show_dynamics = true; s_selected_dynamics_id = e.id; }
        ImGui::SameLine();
        if (ImGui::SmallButton("I/O")) e.show_console = !e.show_console;
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

        ImGui::Separator();

        // ── 3D Platform Viewport ──
        ImVec2 avail = ImGui::GetContentRegionAvail();
        // Clamp split ratio and compute viz height
        if (e.viz_split_ratio < 0.15f) e.viz_split_ratio = 0.15f;
        if (e.viz_split_ratio > 0.92f) e.viz_split_ratio = 0.92f;
        float viz_h = fmaxf(80.0f, avail.y * e.viz_split_ratio);
        ImVec2 p = ImGui::GetCursorScreenPos();
        ImDrawList* dl = ImGui::GetWindowDrawList();

        // InvisibleButton captures mouse for orbit camera control
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
                label_col = IM_COL32(60, 200, 120, 255);  // green
            } else if (hil_connected) {
                label = "CONNECTED (awaiting telemetry)";
                label_col = IM_COL32(240, 180, 50, 255);  // amber
            } else {
                label = "OFFLINE";
                label_col = IM_COL32(120, 120, 130, 200);  // dim grey
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

            // Handshake info overlay — shows device identity + firmware after successful handshake
            auto& dp = e.hil_device_params;
            if (e.hil_handshake_phase == HandshakePhase::Ready && dp.fw_version[0] != '\0') {
                // Line 1: firmware + protocol + platform
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

                // Line 2: geometry summary
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
                // Handshake in progress — show phase
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

        // ── Draggable splitter between 3D viewport and workspace bars ──
        {
            float splitter_h = 6.0f;
            ImVec2 sp = ImGui::GetCursorScreenPos();
            ImGui::InvisibleButton("##viz_splitter", ImVec2(avail.x, splitter_h));
            bool split_hovered = ImGui::IsItemHovered();
            bool split_active  = ImGui::IsItemActive();

            // Visual: thin line that highlights on hover/drag
            ImU32 split_col = split_active  ? IM_COL32(100, 180, 255, 255) :
                              split_hovered ? IM_COL32(80, 140, 200, 200) :
                                              IM_COL32(60, 60, 70, 150);
            float line_y = sp.y + splitter_h * 0.5f;
            dl->AddLine(ImVec2(sp.x + 4, line_y), ImVec2(sp.x + avail.x - 4, line_y), split_col, split_active ? 2.5f : 1.5f);

            // Grip dots in center
            float grip_cx = sp.x + avail.x * 0.5f;
            for (int d = -2; d <= 2; d++) {
                dl->AddCircleFilled(ImVec2(grip_cx + d * 8.0f, line_y), 1.5f, split_col);
            }

            // Change cursor on hover
            if (split_hovered || split_active)
                ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeNS);

            // Handle drag
            if (split_active) {
                float delta = ImGui::GetIO().MouseDelta.y;
                if (delta != 0.0f && avail.y > 0.0f) {
                    e.viz_split_ratio += delta / avail.y;
                    if (e.viz_split_ratio < 0.15f) e.viz_split_ratio = 0.15f;
                    if (e.viz_split_ratio > 0.92f) e.viz_split_ratio = 0.92f;
                }
            }
        }

        // ── Workspace Utilization (Phase C) ──
        {
            // Overall utilization gauge
            float mu = e.state.max_util;
            ImVec4 mu_col = mu < 70.0f ? ImVec4(0.2f, 0.83f, 0.6f, 1.0f) :
                            mu < 90.0f ? ImVec4(0.98f, 0.75f, 0.15f, 1.0f) :
                                         ImVec4(0.98f, 0.44f, 0.44f, 1.0f);
            ImGui::TextColored(mu_col, "Workspace: %.0f%%", mu);
            ImGui::SameLine();
            ImGui::TextDisabled("(%.0f%% intensity)", e.config.intensity);

            // Auto-fit intensity when exceeding safe workspace
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
            float util = e.state.servo_util[i];
            ImVec4 bar_col = util < 70.0f ? ImVec4(0.2f, 0.83f, 0.6f, 1.0f) :
                             util < 90.0f ? ImVec4(0.98f, 0.75f, 0.15f, 1.0f) :
                                            ImVec4(0.98f, 0.44f, 0.44f, 1.0f);
            ImGui::PushStyleColor(ImGuiCol_PlotHistogram, bar_col);

            char overlay[48];
            snprintf(overlay, sizeof(overlay), "S%d  %.1f\xc2\xb0  %.0f%%", i, e.state.output_angles_deg[i], util);
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
                // Telemetry rate selector
                ImGui::SameLine();
                ImGui::PushItemWidth(80);
                const char* rate_opts[] = {"10 Hz", "20 Hz", "30 Hz", "50 Hz"};
                int rate_vals[] = {10, 20, 30, 50};
                int cur_sel = 2; // default 30Hz
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
            } else if (e.hil_auto_connect && e.hil_port[0] != '\0') {
                ImGui::TextColored(ImVec4(0.95f, 0.75f, 0.2f, 1.0f), "Searching %s...", e.hil_port);
            } else {
                ImGui::TextDisabled("offline");
            }
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

// ── Platform Setup Window ───────────────────────────────────────────

static void DrawPlatformSetup(Entity& e) {
    if (!e.show_platform) return;

    ImGui::PushID(e.id + 2000);
    char title[128];
    snprintf(title, sizeof(title), "Platform Setup: %s###platform_%d", e.name, e.id);

    ImGui::SetNextWindowSize(ImVec2(580, 750), ImGuiCond_FirstUseEver);
    if (ImGui::Begin(title, &e.show_platform)) {
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
    ImGui::End();
    ImGui::PopID();
}

// ── Entity Settings Window (system / pipeline / connection) ─────────

static void DrawEntitySettings(Entity& e) {
    if (!e.show_settings) return;

    ImGui::PushID(e.id + 1000);
    char title[128];
    snprintf(title, sizeof(title), "Settings: %s###settings_%d", e.name, e.id);

    ImGui::SetNextWindowSize(ImVec2(380, 320), ImGuiCond_FirstUseEver);
    if (ImGui::Begin(title, &e.show_settings)) {
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
                ImGui::Text("Telemetry: %.0f Hz (seq %d)", e.rate_tel_hz, e.hil_tel_seq);

                // Read-only transport settings (locked while connected)
                ImGui::Spacing();
                ImGui::TextDisabled("TX Rate: %d Hz  |  Bit Depth: %d  |  %s", e.hil_tx_hz, e.config.bit_depth,
                    e.hil_protocol == HilProtocol::CSV ? "CSV" : "Binary");
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("Disconnect to change TX rate or bit depth.\n"
                        "These are locked during an active session to\n"
                        "prevent protocol mismatches with the ESP32.");
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

                    const char* proto_labels[] = { "Binary", "CSV (legacy)" };
                    int proto_idx = (int)e.hil_protocol;
                    if (ImGui::Combo("Protocol##dc", &proto_idx, proto_labels, 2)) {
                        e.hil_protocol = (HilProtocol)proto_idx;
                    }
                    if (ImGui::IsItemHovered()) {
                        ImGui::SetTooltip("Binary: 15-byte framed packet (big platform Controller firmware)\n"
                            "CSV: comma-separated values + 'X' terminator (Mini-6DOF / legacy)");
                    }

                    ImGui::SliderInt("TX Rate (Hz)##dc", &e.hil_tx_hz, 10, 1000);
                    if (ImGui::IsItemHovered()) {
                        ImGui::SetTooltip("How often motion packets are sent to the ESP32.\n"
                            "Higher values give smoother motion but more serial traffic.");
                    }
                    ImGui::SliderInt("Bit Depth##dc", &e.config.bit_depth, 8, 16);
                    if (ImGui::IsItemHovered()) {
                        ImGui::SetTooltip("Resolution of motion values sent to the ESP32.\n"
                            "Must match the ESP32 firmware's expected bit depth.");
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
                        int eid = e.id;
                        sp->setLineCallback([eid](const char* line) {
                            g_app.handleHilLine(eid, line);
                        });
                        if (sp->open(e.hil_port, 115200)) {
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
                            snprintf(e.hil_handshake_msg, sizeof(e.hil_handshake_msg), "Requesting fingerprint...");
                            g_app.log(e.id, "hil", "Connected to %s — starting handshake...", e.hil_port);
                            sp->write((const uint8_t*)"X", 1); // flush residual ASCII buffer garbage
                            sp->sendCommand("FINGERPRINT?");
                        } else {
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

                    // Timeout check (5s)
                    if (g_app.frame_time - e.hil_handshake_start > 5.0) {
                        ImGui::TextColored(ImVec4(0.95f, 0.4f, 0.3f, 1.0f),
                            "Timeout — device not responding. Check firmware.");
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
    ImGui::End();
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

        // ── Device selector ──
        {
            ImGui::TextColored(ImVec4(0.7f, 0.85f, 1.0f, 1.0f), "Device:");
            ImGui::SameLine();
            ImGui::PushItemWidth(200);
            const char* type_str = (e.type == EntityType::SIL) ? "SIL" : "HIL";
            char combo_preview[96];
            snprintf(combo_preview, sizeof(combo_preview), "%s [%s]", e.name, type_str);
            if (ImGui::BeginCombo("##dyn_device", combo_preview)) {
                for (auto& ent : g_app.entities) {
                    const char* ts = (ent.type == EntityType::SIL) ? "SIL" : "HIL";
                    char label[96];
                    snprintf(label, sizeof(label), "%s [%s]###ddev_%d", ent.name, ts, ent.id);
                    bool selected = (ent.id == s_selected_dynamics_id);
                    if (ImGui::Selectable(label, selected)) {
                        s_selected_dynamics_id = ent.id;
                    }
                    if (selected) ImGui::SetItemDefaultFocus();
                }
                ImGui::EndCombo();
            }
            ImGui::PopItemWidth();
            ImGui::SameLine();
            ImVec4 ecol = ColorFromFloat4(e.color);
            ImGui::TextColored(ecol, "[%s #%d]", type_str, e.id);
        }
        ImGui::Separator();

        // ── Active Profile Banner ────────────────────────────────────
        {
            bool has_preset = (s_dyn_preset_idx >= 0 && s_dyn_preset_idx < (int)g_app.mca_presets.size());
            const char* profile_name = has_preset ? g_app.mca_presets[s_dyn_preset_idx].name : "No Profile Selected";
            bool is_user = has_preset && !g_app.mca_presets[s_dyn_preset_idx].is_builtin;

            // Colored banner
            ImVec4 banner_col = has_preset ? ImVec4(0.25f, 0.55f, 0.85f, 1.0f) : ImVec4(0.5f, 0.5f, 0.5f, 0.7f);
            ImGui::TextColored(banner_col, "Profile:");
            ImGui::SameLine();
            ImGui::PushStyleColor(ImGuiCol_Text, has_preset ? ImVec4(0.9f, 0.95f, 1.0f, 1.0f) : ImVec4(0.6f, 0.6f, 0.6f, 1.0f));
            ImGui::Text("%s%s", profile_name, is_user ? " (user)" : "");
            ImGui::PopStyleColor();

            // Copy / Paste buttons
            ImGui::SameLine(ImGui::GetContentRegionAvail().x - 130);
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
                if (has_preset) cJSON_AddStringToObject(root, "profile_name", profile_name);

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
        }

        // ── Signal Flow Diagram ──────────────────────────────────────
        {
            MotionCueingConfig& mca_ref = e.config.mca;
            float diagram_h = 48.0f;
            ImVec2 cursor = ImGui::GetCursorScreenPos();
            ImDrawList* dl = ImGui::GetWindowDrawList();
            float avail_w = ImGui::GetContentRegionAvail().x;

            // Block definitions: label, enabled flag, color
            struct FlowBlock { const char* label; bool enabled; ImU32 col; };
            FlowBlock blocks[] = {
                { "Input",      true,                                      IM_COL32(80,  130, 180, 255) },
                { "Pre-Filter", e.config.input_filter.enabled != 0,        IM_COL32(100, 140, 100, 255) },
                { "Washout",    mca_ref.enabled != 0,                      IM_COL32(140, 120, 180, 255) },
                { "Tilt",       mca_ref.tilt.enabled != 0,                 IM_COL32(180, 140, 80,  255) },
                { "Gain/Inv",   true,                                      IM_COL32(80,  160, 140, 255) },
                { "IK",         true,                                      IM_COL32(160, 100, 100, 255) },
                { "Output",     true,                                      IM_COL32(100, 100, 160, 255) },
            };
            int n_blocks = sizeof(blocks) / sizeof(blocks[0]);
            float gap = 6.0f;
            float arrow_w = 14.0f;
            float total_gaps = (float)(n_blocks - 1) * (gap + arrow_w + gap);
            float block_w = (avail_w - total_gaps) / (float)n_blocks;
            if (block_w < 40.0f) block_w = 40.0f;
            float block_h = 28.0f;
            float y_center = cursor.y + diagram_h * 0.5f;
            float x = cursor.x;

            for (int i = 0; i < n_blocks; i++) {
                ImU32 bg = blocks[i].enabled ? blocks[i].col : IM_COL32(50, 50, 50, 200);
                ImU32 border = blocks[i].enabled ? IM_COL32(200, 200, 200, 180) : IM_COL32(80, 80, 80, 150);
                ImU32 text_col = blocks[i].enabled ? IM_COL32(255, 255, 255, 255) : IM_COL32(120, 120, 120, 200);

                ImVec2 p0(x, y_center - block_h * 0.5f);
                ImVec2 p1(x + block_w, y_center + block_h * 0.5f);
                dl->AddRectFilled(p0, p1, bg, 4.0f);
                dl->AddRect(p0, p1, border, 4.0f);

                // Centered label
                ImVec2 ts = ImGui::CalcTextSize(blocks[i].label);
                dl->AddText(ImVec2(x + (block_w - ts.x) * 0.5f, y_center - ts.y * 0.5f), text_col, blocks[i].label);

                // Live signal level bar (tiny bar under block)
                if (i == 0 || i == 4 || i == 6) {
                    float max_sig = 0.0f;
                    for (int a = 0; a < 6; a++) {
                        float v = 0;
                        if (i == 0) v = fabsf(e.state.input_pct[a]);
                        else if (i == 4) v = fabsf(e.last_scaled_pct[a]);
                        else if (i == 6) v = e.state.servo_util[a];
                        if (v > max_sig) max_sig = v;
                    }
                    float bar_frac = max_sig / 100.0f;
                    if (bar_frac > 1.0f) bar_frac = 1.0f;
                    float bar_y = p1.y + 2.0f;
                    dl->AddRectFilled(ImVec2(x, bar_y), ImVec2(x + block_w * bar_frac, bar_y + 3.0f),
                        IM_COL32(100, 200, 100, 180), 1.0f);
                    dl->AddRect(ImVec2(x, bar_y), ImVec2(x + block_w, bar_y + 3.0f),
                        IM_COL32(60, 60, 60, 120), 1.0f);
                }

                x += block_w;

                // Arrow between blocks
                if (i < n_blocks - 1) {
                    x += gap;
                    float ay = y_center;
                    ImU32 arrow_col = IM_COL32(140, 140, 140, 200);
                    dl->AddLine(ImVec2(x, ay), ImVec2(x + arrow_w - 4, ay), arrow_col, 1.5f);
                    // Arrowhead
                    dl->AddTriangleFilled(
                        ImVec2(x + arrow_w, ay),
                        ImVec2(x + arrow_w - 5, ay - 3),
                        ImVec2(x + arrow_w - 5, ay + 3),
                        arrow_col);
                    x += arrow_w + gap;
                }
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
                    // Recalculate all biquad coefficients at current sample rate
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
                    stg.intensity = e.config.intensity;
                    memcpy(stg.axis_gain, e.config.axis_gain, sizeof(stg.axis_gain));
                    memcpy(stg.axis_invert, e.config.axis_invert, sizeof(stg.axis_invert));
                    memcpy(stg.occupant, e.config.occupant, sizeof(stg.occupant));
                }
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("Discard staged changes and reload from live config.");

                ImGui::SameLine();
                ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.2f, 1.0f), "Unsaved changes");
            } else {
                ImGui::BeginDisabled();
                ImGui::Button("Apply##dyn_top", ImVec2(120, 0));
                ImGui::SameLine();
                ImGui::Button("Revert##dyn_top", ImVec2(120, 0));
                ImGui::EndDisabled();
            }
        }

        // ═══════════════════════════════════════════════════════════════
        // STICKY ROW 2: Preset selector + MCA enable + management
        // ═══════════════════════════════════════════════════════════════
        {
            int& selected_preset_idx = s_dyn_preset_idx;
            static char new_preset_name[64] = "";
            static bool show_save_popup = false;

            bool enabled = stg.mca.enabled != 0;
            if (ImGui::Checkbox("MCA##en", &enabled)) {
                stg.mca.enabled = enabled ? 1 : 0;
            }
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Master enable for Motion Cueing Algorithm.\nWhen off, raw input passes through unfiltered.");

            ImGui::SameLine();
            ImGui::TextDisabled("%.0f Hz", mca.sample_rate);

            // Preset combo — shows all built-in + user presets
            ImGui::SameLine();
            ImGui::PushItemWidth(180);
            const char* preview = (selected_preset_idx >= 0 && selected_preset_idx < (int)g_app.mca_presets.size())
                ? g_app.mca_presets[selected_preset_idx].name : "Select Preset...";
            if (ImGui::BeginCombo("##preset", preview)) {
                for (int i = 0; i < (int)g_app.mca_presets.size(); i++) {
                    auto& p = g_app.mca_presets[i];
                    char label[80];
                    snprintf(label, sizeof(label), "%s%s##p%d", p.name, p.is_builtin ? "" : " *", i);
                    bool selected = (selected_preset_idx == i);
                    if (ImGui::Selectable(label, selected)) {
                        selected_preset_idx = i;
                        // Load preset into staging (not live — user must Apply)
                        g_app.loadMcaPreset(i, stg.mca, stg.intensity, stg.axis_gain);
                    }
                    if (selected) ImGui::SetItemDefaultFocus();
                    if (ImGui::IsItemHovered()) {
                        if (p.is_builtin)
                            ImGui::SetTooltip("Built-in preset. Click to load.\nYou can save changes over it or restore defaults.");
                        else
                            ImGui::SetTooltip("User preset. Click to load.");
                    }
                }
                ImGui::EndCombo();
            }
            if (ImGui::IsItemHovered()) ImGui::SetTooltip(
                "Select a preset to load all dynamics parameters.\n"
                "Built-in presets: Off, Gentle, Moderate, Aggressive, Race Pro\n"
                "User presets marked with *");
            ImGui::PopItemWidth();

            // Save button — overwrites currently selected preset
            if (selected_preset_idx >= 0 && selected_preset_idx < (int)g_app.mca_presets.size()) {
                ImGui::SameLine();
                if (ImGui::SmallButton("Save")) {
                    g_app.saveMcaPreset(g_app.mca_presets[selected_preset_idx].name, stg.mca, stg.intensity, stg.axis_gain);
                }
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("Overwrite '%s' with current settings.", g_app.mca_presets[selected_preset_idx].name);
            }

            // Save As button — save with a new name
            ImGui::SameLine();
            if (ImGui::SmallButton("Save As")) {
                show_save_popup = true;
                new_preset_name[0] = 0;
            }
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Save current settings as a new preset.");

            // Delete button (only for user presets)
            if (selected_preset_idx >= 0 && selected_preset_idx < (int)g_app.mca_presets.size()
                && !g_app.mca_presets[selected_preset_idx].is_builtin) {
                ImGui::SameLine();
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.6f, 0.15f, 0.15f, 1.0f));
                if (ImGui::SmallButton("Delete")) {
                    g_app.deleteMcaPreset(selected_preset_idx);
                    selected_preset_idx = -1;
                }
                ImGui::PopStyleColor();
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("Delete this user preset.");
            }

            // Restore Default button (only for built-in presets)
            if (selected_preset_idx >= 0 && selected_preset_idx < (int)g_app.mca_presets.size()
                && g_app.mca_presets[selected_preset_idx].is_builtin) {
                ImGui::SameLine();
                if (ImGui::SmallButton("Restore Default")) {
                    auto& p = g_app.mca_presets[selected_preset_idx];
                    initMotionCueing(&p.mca, 60.0f);
                    setMotionCueingPreset(&p.mca, selected_preset_idx);
                    p.intensity = 100.0f;
                    for (int j = 0; j < 6; j++) p.axis_gain[j] = 100.0f;
                    g_app.loadMcaPreset(selected_preset_idx, stg.mca, stg.intensity, stg.axis_gain);
                    g_app.saveMcaPresetsToDisk();
                    g_app.log(e.id, "dynamics", "Preset '%s' restored to factory defaults", p.name);
                }
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("Reset this built-in preset to its factory default values.");
            }

            // Reset Filters button (clears biquad memory)
            ImGui::SameLine();
            if (ImGui::SmallButton("Reset Filters")) {
                resetMotionCueing(&mca);
                g_app.log(e.id, "dynamics", "MCA filter state reset");
            }
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Clear all biquad filter memory (stops any ringing).\nDoes not change parameter values.");

            // Save As popup
            if (show_save_popup)
                ImGui::OpenPopup("Save As Dynamics Preset");

            if (ImGui::BeginPopup("Save As Dynamics Preset")) {
                ImGui::Text("New Preset Name:");
                ImGui::PushItemWidth(250);
                bool enter_pressed = ImGui::InputText("##name", new_preset_name, sizeof(new_preset_name),
                    ImGuiInputTextFlags_EnterReturnsTrue);
                ImGui::PopItemWidth();
                bool do_save = enter_pressed;
                if (ImGui::Button("Save", ImVec2(120, 0))) do_save = true;
                ImGui::SameLine();
                if (ImGui::Button("Cancel", ImVec2(120, 0))) {
                    show_save_popup = false;
                    ImGui::CloseCurrentPopup();
                }
                if (do_save && new_preset_name[0] != 0) {
                    g_app.saveMcaPreset(new_preset_name, stg.mca, stg.intensity, stg.axis_gain);
                    // Select the newly saved preset
                    for (int i = 0; i < (int)g_app.mca_presets.size(); i++) {
                        if (strcmp(g_app.mca_presets[i].name, new_preset_name) == 0) {
                            selected_preset_idx = i;
                            break;
                        }
                    }
                    show_save_popup = false;
                    ImGui::CloseCurrentPopup();
                }
                ImGui::EndPopup();
            }

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
            InputFilterConfig& iflt = e.config.input_filter;

            bool if_en = iflt.enabled != 0;
            if (ImGui::Checkbox("Input Filter##if_en", &if_en))
                iflt.enabled = if_en ? 1 : 0;
            if (ImGui::IsItemHovered()) ImGui::SetTooltip(
                "Pre-MCA signal conditioning.\n\n"
                "Cleans up noisy or spiky telemetry data BEFORE\n"
                "it reaches the washout stage.\n\n"
                "Pipeline: Input -> [Input Filter] -> Washout -> Smoothing -> Output");
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(0.7f, 0.85f, 1.0f, 1.0f), "Input Signal Filter");

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
                    if (ImGui::DragFloat("##sg", &abs_sg, 0.005f, 0.0f, 1.0f, "%.3f")) {
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
                    if (ImGui::DragFloat("##sw", &abs_sw, 0.005f, 0.0f, 1.0f, "%.3f")) {
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
    ImGui::End();
    ImGui::PopID();
}

// ── Per-Entity I/O Monitor ──────────────────────────────────────────

static void DrawEntityConsole(Entity& e) {
    if (!e.show_console) return;

    ImGui::PushID(e.id + 4000);
    char title[128];
    snprintf(title, sizeof(title), "I/O: %s###io_%d", e.name, e.id);

    ImVec4 col = ColorFromFloat4(e.color);
    ImGui::SetNextWindowSize(ImVec2(340, 400), ImGuiCond_FirstUseEver);
    if (ImGui::Begin(title, &e.show_console)) {
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
                    ImGui::Text("[%d] %5u", i, e.hil_tx_raw[i]);
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
    ImGui::End();
    ImGui::PopID();
}

// ── Manual slider state (file-scope so GetSourceStatus can read it) ───
static float s_manual_input[6] = {};

// ── Input Source Card Definitions ─────────────────────────────────────

struct InputSourceDef {
    InputSource  id;
    const char*  label;
    const char*  icon;     // short badge text
    ImVec4       color;
    ImVec4       color_dim;
};

static const InputSourceDef g_input_sources[] = {
    { InputSource::Manual,          "Manual Sliders",   "MAN",  {0.60f, 0.70f, 0.80f, 1.0f}, {0.25f, 0.28f, 0.32f, 1.0f} },
    { InputSource::CapturePlayback, "Capture Playback", "CAP",  {0.95f, 0.75f, 0.20f, 1.0f}, {0.38f, 0.30f, 0.08f, 1.0f} },
    { InputSource::Plugin,          "Plugin",           "PLG",  {0.20f, 0.83f, 0.60f, 1.0f}, {0.08f, 0.33f, 0.24f, 1.0f} },
};
static const int g_num_input_sources = sizeof(g_input_sources) / sizeof(g_input_sources[0]);

static int InputSourceIndex(InputSource s) {
    for (int i = 0; i < g_num_input_sources; i++)
        if (g_input_sources[i].id == s) return i;
    return 0;
}

// Build a one-line status string for a given source
static void GetSourceStatus(InputSource id, char* buf, int buf_sz) {
    switch (id) {
        case InputSource::Manual: {
                float mx = 0;
                for (int i = 0; i < 6; i++) mx = fmaxf(mx, fabsf(s_manual_input[i]));
                snprintf(buf, buf_sz, "6 axes  |  peak %.0f%%", mx);
            }
            break;
        case InputSource::CapturePlayback:
            if (g_app.capture_playing && g_app.capture_playback_idx >= 0 &&
                g_app.capture_playback_idx < (int)g_app.saved_recordings.size()) {
                auto& sr = g_app.saved_recordings[g_app.capture_playback_idx];
                double elapsed = g_app.frame_time - g_app.capture_start_time;
                snprintf(buf, buf_sz, "Playing: %s  %.1f/%.1fs", sr.name, elapsed, sr.duration());
            } else {
                int n = (int)g_app.saved_recordings.size();
                snprintf(buf, buf_sz, "%d capture%s  %s", n, n != 1 ? "s" : "", n > 0 ? "ready" : "empty");
            }
            break;
        case InputSource::Plugin:
            if (g_app.active_plugin_idx >= 0 && g_app.active_plugin_idx < g_app.plugin_mgr.pluginCount()) {
                snprintf(buf, buf_sz, "%s  active", g_app.plugin_mgr.pluginName(g_app.active_plugin_idx));
            } else if (g_app.plugin_mgr.pluginCount() > 0) {
                snprintf(buf, buf_sz, "%d plugin(s) available", g_app.plugin_mgr.pluginCount());
            } else {
                snprintf(buf, buf_sz, "No plugins found");
            }
            break;
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
    if (compact) ImGui::SetNextItemWidth(-1);

    bool changed = false;
    switch (pd.type) {
        case STEWART_PARAM_FLOAT: {
            float range = pd.max_val - pd.min_val;
            float spd = range >= 100 ? 1.0f : range >= 10 ? 0.1f : 0.01f;
            const char* fmt = range >= 100 ? "%.0f" : range >= 10 ? "%.1f" : "%.2f";
            changed = ImGui::DragFloat(label, val, spd, pd.min_val, pd.max_val, fmt);
            break;
        }
        case STEWART_PARAM_INT: {
            int iv = (int)*val;
            if (ImGui::DragInt(label, &iv, 1.0f, (int)pd.min_val, (int)pd.max_val)) {
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

        // Non-bool params in a responsive label + widget grid
        if (!others.empty()) {
            float avail = ImGui::GetContentRegionAvail().x;
            int cols = (avail > 450 && (int)others.size() >= 2) ? 2 : 1;

            if (ImGui::BeginTable("##pg", cols * 2, ImGuiTableFlags_SizingFixedFit)) {
                for (int c = 0; c < cols; c++) {
                    ImGui::TableSetupColumn("##l", ImGuiTableColumnFlags_WidthFixed, 110);
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

        // General case: table with Axis column + value columns + enable columns
        int n_cols = 1 + (int)val_gi.size() + (int)bool_gi.size();
        ImGuiTableFlags tf = ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
            ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_PadOuterX;

        if (ImGui::BeginTable("##axtbl", n_cols, tf)) {
            ImGui::TableSetupColumn("Axis", ImGuiTableColumnFlags_WidthFixed, 52);
            for (int g : val_gi)
                ImGui::TableSetupColumn(groups[g].header.c_str(), ImGuiTableColumnFlags_WidthStretch);
            for (int g : bool_gi)
                ImGui::TableSetupColumn(groups[g].header.c_str(), ImGuiTableColumnFlags_WidthFixed, 30);
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

                ImGui::PopID();
            }
            ImGui::EndTable();
        }
    }

    // ═══ LIVE OUTPUT (when plugin is active) ═══
    if (plug.active && !g_app.entities.empty()) {
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::TextColored(ImVec4(0.2f, 0.83f, 0.6f, 1.0f), "Live Output");
        float vals[6];
        {
            std::lock_guard<std::mutex> lock(g_app.input_mutex);
            memcpy(vals, g_app.shared_input, sizeof(vals));
        }
        for (int i = 0; i < 6; i++) {
            float frac = fabsf(vals[i]) / 100.0f;
            ImGui::PushStyleColor(ImGuiCol_PlotHistogram,
                vals[i] >= 0 ? ImVec4(0.2f, 0.7f, 0.5f, 0.8f) : ImVec4(0.7f, 0.3f, 0.3f, 0.8f));
            char overlay[32];
            snprintf(overlay, sizeof(overlay), "%-6s %+.1f%%", ax_lbl[i], vals[i]);
            ImGui::ProgressBar(frac, ImVec2(-1, 0), overlay);
            ImGui::PopStyleColor();
        }
    }
}

// ── Input Panel (combobox card selector + per-source content) ────────

static void DrawInputPanel() {
    if (!s_show_input) return;
    if (ImGui::Begin("Input", &s_show_input)) {
        const char* axis_labels[] = {"Surge", "Sway", "Heave", "Roll", "Pitch", "Yaw"};

        // ══════════════════════════════════════════════════════════════
        //  SOURCE SELECTOR — rich combobox with card items
        // ══════════════════════════════════════════════════════════════
        {
            bool switching = g_app.source_switch_active;
            int cur_idx = InputSourceIndex(g_app.input_source);
            const auto& cur = g_input_sources[cur_idx];
            char status_buf[128];
            GetSourceStatus(cur.id, status_buf, sizeof(status_buf));

            // Build preview string for the combo header
            char preview[192];
            if (switching) {
                int tgt_idx = InputSourceIndex(g_app.source_switch_target);
                const auto& tgt = g_input_sources[tgt_idx];
                snprintf(preview, sizeof(preview), "[%s] -> [%s]  Switching...", cur.icon, tgt.icon);
            } else {
                snprintf(preview, sizeof(preview), "[%s]  %s  —  %s", cur.icon, cur.label, status_buf);
            }

            // Style the combo frame with the active source color
            ImVec4 frame_color = switching ? ImVec4(0.35f, 0.30f, 0.15f, 0.5f) : ImVec4(cur.color_dim.x, cur.color_dim.y, cur.color_dim.z, 0.5f);
            ImVec4 border_color = switching ? ImVec4(0.9f, 0.7f, 0.2f, 1.0f) : cur.color;
            ImGui::PushStyleColor(ImGuiCol_FrameBg, frame_color);
            ImGui::PushStyleColor(ImGuiCol_FrameBgHovered, ImVec4(frame_color.x * 1.4f, frame_color.y * 1.4f, frame_color.z * 1.4f, 0.6f));
            ImGui::PushStyleColor(ImGuiCol_Border, border_color);
            ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 1.5f);
            ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 6.0f);
            ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(8, 10));

            // Disable combo during switch so user can't re-select
            if (switching) ImGui::BeginDisabled();

            ImGui::SetNextItemWidth(-1);
            if (ImGui::BeginCombo("##input_source_sel", preview, ImGuiComboFlags_HeightLarge)) {

                for (int i = 0; i < g_num_input_sources; i++) {
                    const auto& src = g_input_sources[i];
                    bool selected = (g_app.input_source == src.id);

                    ImGui::PushID(i);

                    // Card background
                    ImVec2 card_start = ImGui::GetCursorScreenPos();
                    float card_w = ImGui::GetContentRegionAvail().x;
                    float card_h = 52.0f;

                    // Invisible selectable for the full card area
                    if (ImGui::Selectable("##card", selected, 0, ImVec2(card_w, card_h))) {
                        if (g_app.input_source != src.id) {
                            g_app.requestSourceSwitch(src.id);
                        }
                    }

                    // Custom draw over the selectable
                    ImDrawList* dl = ImGui::GetWindowDrawList();
                    float badge_h = 20.0f;
                    float badge_x = card_start.x + 6.0f;
                    float badge_y = card_start.y + 4.0f;

                    // Badge icon
                    DrawBadge(dl, ImVec2(badge_x, badge_y), src.icon, src.color, badge_h);

                    // Source name (bold-ish via color)
                    float name_x = badge_x + ImGui::CalcTextSize(src.icon).x + 20.0f;
                    ImU32 name_col = selected
                        ? IM_COL32((int)(src.color.x * 255), (int)(src.color.y * 255), (int)(src.color.z * 255), 255)
                        : IM_COL32(220, 220, 220, 255);
                    dl->AddText(ImVec2(name_x, badge_y + 1.0f), name_col, src.label);

                    // Status detail line
                    char card_status[128];
                    GetSourceStatus(src.id, card_status, sizeof(card_status));
                    dl->AddText(ImVec2(badge_x + 6.0f, badge_y + badge_h + 6.0f),
                                IM_COL32(160, 160, 160, 220), card_status);

                    // Active indicator dot
                    if (selected) {
                        float dot_x = card_start.x + card_w - 14.0f;
                        float dot_y = card_start.y + card_h * 0.5f;
                        dl->AddCircleFilled(ImVec2(dot_x, dot_y), 5.0f,
                            IM_COL32((int)(src.color.x * 255), (int)(src.color.y * 255),
                                     (int)(src.color.z * 255), 255));
                    }

                    ImGui::PopID();
                }

                // ── Plugin entries ──
                if (g_app.plugin_mgr.pluginCount() > 0) {
                    ImGui::Separator();
                    ImGui::TextDisabled("  Plugins");

                    static const ImVec4 plugin_color = {0.40f, 0.80f, 0.95f, 1.0f};
                    static const ImVec4 plugin_color_dim = {0.16f, 0.32f, 0.38f, 1.0f};

                    for (int pi = 0; pi < g_app.plugin_mgr.pluginCount(); pi++) {
                        auto& plug = g_app.plugin_mgr.plugins()[pi];
                        if (!plug.valid) continue;

                        bool is_active_plugin = (g_app.input_source == InputSource::Plugin &&
                                                 g_app.active_plugin_idx == pi);

                        ImGui::PushID(1000 + pi);

                        ImVec2 card_start = ImGui::GetCursorScreenPos();
                        float card_w = ImGui::GetContentRegionAvail().x;
                        float card_h = 52.0f;

                        if (ImGui::Selectable("##plugcard", is_active_plugin, 0, ImVec2(card_w, card_h))) {
                            if (!is_active_plugin) {
                                // Store which plugin to activate after ramp completes
                                g_app.active_plugin_idx = pi;
                                g_app.requestSourceSwitch(InputSource::Plugin);
                            }
                        }

                        ImDrawList* dl = ImGui::GetWindowDrawList();
                        float badge_h = 20.0f;
                        float badge_x = card_start.x + 6.0f;
                        float badge_y = card_start.y + 4.0f;

                        DrawBadge(dl, ImVec2(badge_x, badge_y), "PLG", plugin_color, badge_h);

                        float name_x = badge_x + ImGui::CalcTextSize("PLG").x + 20.0f;
                        ImU32 name_col = is_active_plugin
                            ? IM_COL32(102, 204, 242, 255)
                            : IM_COL32(220, 220, 220, 255);
                        dl->AddText(ImVec2(name_x, badge_y + 1.0f), name_col,
                                    plug.info->name ? plug.info->name : plug.filename.c_str());

                        char plug_status[128];
                        if (is_active_plugin)
                            snprintf(plug_status, sizeof(plug_status), "Active  —  v%s by %s",
                                     plug.info->version ? plug.info->version : "?",
                                     plug.info->author ? plug.info->author : "?");
                        else
                            snprintf(plug_status, sizeof(plug_status), "v%s by %s  —  %d params",
                                     plug.info->version ? plug.info->version : "?",
                                     plug.info->author ? plug.info->author : "?",
                                     plug.info->param_count);
                        dl->AddText(ImVec2(badge_x + 6.0f, badge_y + badge_h + 6.0f),
                                    IM_COL32(160, 160, 160, 220), plug_status);

                        if (is_active_plugin) {
                            float dot_x = card_start.x + card_w - 14.0f;
                            float dot_y = card_start.y + card_h * 0.5f;
                            dl->AddCircleFilled(ImVec2(dot_x, dot_y), 5.0f,
                                IM_COL32(102, 204, 242, 255));
                        }

                        ImGui::PopID();
                    }
                }

                ImGui::EndCombo();
            }

            if (switching) ImGui::EndDisabled();

            ImGui::PopStyleVar(3);
            ImGui::PopStyleColor(3);

            // ── Switching progress bar ──
            if (switching) {
                float elapsed = (float)(g_app.frame_time - g_app.source_switch_start);
                float progress = elapsed / g_app.SOURCE_RAMP_OUT_S;
                if (progress > 1.0f) progress = 1.0f;

                int tgt_idx = InputSourceIndex(g_app.source_switch_target);
                const auto& tgt = g_input_sources[tgt_idx];

                ImGui::PushStyleColor(ImGuiCol_PlotHistogram, ImVec4(tgt.color.x, tgt.color.y, tgt.color.z, 0.9f));
                ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0.15f, 0.15f, 0.15f, 0.8f));
                char overlay[96];
                snprintf(overlay, sizeof(overlay), "Homing... switching to %s", tgt.label);
                ImGui::ProgressBar(progress, ImVec2(-1, 18), overlay);
                ImGui::PopStyleColor(2);
            }
        }

        // ══════════════════════════════════════════════════════════════
        //  START / STOP — global motion gate
        // ══════════════════════════════════════════════════════════════
        ImGui::Spacing();
        {
            float btn_w = ImGui::GetContentRegionAvail().x;
            if (g_app.motion_started) {
                // STOP button — red
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.70f, 0.15f, 0.15f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.85f, 0.20f, 0.20f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.55f, 0.10f, 0.10f, 1.0f));
                if (ImGui::Button("STOP", ImVec2(btn_w, 36))) {
                    g_app.motion_started = false;
                    g_app.start_ramp_active = false;
                    // Deactivate plugin if active
                    if (g_app.plugin_mgr.activeIndex() >= 0) {
                        g_app.plugin_mgr.deactivateActive();
                    }
                    // Stop capture playback
                    if (g_app.capture_playing) g_app.stopCapturePlayback();
                    g_app.log(-1, "input", "Motion STOPPED");
                }
                ImGui::PopStyleColor(3);
            } else {
                // START button — green
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.10f, 0.55f, 0.25f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.15f, 0.70f, 0.35f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.08f, 0.40f, 0.18f, 1.0f));
                if (ImGui::Button("START", ImVec2(btn_w, 36))) {
                    g_app.motion_started = true;

                    // S-curve ramp from home to current slider positions
                    // Check if any manual slider is offset (need ramp)
                    bool need_ramp = false;
                    for (int i = 0; i < 6; i++) {
                        if (fabsf(s_manual_input[i]) > 0.1f) { need_ramp = true; break; }
                    }
                    if (need_ramp && g_app.input_source == InputSource::Manual) {
                        g_app.start_ramp_active = true;
                        g_app.start_ramp_begin = g_app.frame_time;
                        memcpy(g_app.start_ramp_target, s_manual_input, sizeof(s_manual_input));
                    }

                    // Activate plugin if Plugin source is selected
                    if (g_app.input_source == InputSource::Plugin && g_app.active_plugin_idx >= 0 &&
                        g_app.plugin_mgr.activeIndex() < 0) {
                        float sr = (g_app.fps > 1.0) ? (float)g_app.fps : 60.0f;
                        if (!g_app.plugin_mgr.activatePlugin(g_app.active_plugin_idx, sr)) {
                            g_app.log(-1, "plugin", "Failed to activate plugin");
                        }
                    }
                    g_app.log(-1, "input", "Motion STARTED");
                }
                ImGui::PopStyleColor(3);
            }
        }

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        // ══════════════════════════════════════════════════════════════
        //  PER-SOURCE CONTENT (editable details)
        // ══════════════════════════════════════════════════════════════

        // Disable all controls while source switch is in progress
        if (g_app.source_switch_active) ImGui::BeginDisabled();

        // Always zero manual sliders when switching TO Manual from another source
        {
            static InputSource s_prev_source = InputSource::Manual;
            if (g_app.input_source == InputSource::Manual && s_prev_source != InputSource::Manual) {
                memset(s_manual_input, 0, sizeof(s_manual_input));
                for (auto& e : g_app.entities)
                    memset(e.state.input_pct, 0, sizeof(e.state.input_pct));
            }
            s_prev_source = g_app.input_source;
        }

        // ── Manual ──
        if (g_app.input_source == InputSource::Manual) {
            bool changed = false;
            for (int i = 0; i < 6; i++) {
                changed |= ImGui::SliderFloat(axis_labels[i], &s_manual_input[i], -100.0f, 100.0f, "%.0f%%");
            }
            if (ImGui::Button("Home All")) {
                memset(s_manual_input, 0, sizeof(s_manual_input));
                changed = true;
            }
            if (changed && !g_app.capture_playing) {
                if (g_app.start_ramp_active) {
                    // Update ramp target so sliders adjust mid-ramp
                    memcpy(g_app.start_ramp_target, s_manual_input, sizeof(s_manual_input));
                } else {
                    for (auto& e : g_app.entities) {
                        memcpy(e.state.input_pct, s_manual_input, sizeof(s_manual_input));
                    }
                }
            }

            ImGui::Spacing();
            ImGui::TextDisabled("Direct control via sliders. Values propagate to all entities.");
        }

        // ── Capture Playback ──
        else if (g_app.input_source == InputSource::CapturePlayback) {
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
                    bool can_play = !g_app.capture_playing && g_app.capture_playback_idx >= 0;
                    float btn_w = (ImGui::GetContentRegionAvail().x - 4) * 0.5f;

                    if (!can_play) ImGui::BeginDisabled();
                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.15f, 0.55f, 0.35f, 1.0f));
                    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.2f, 0.7f, 0.45f, 1.0f));
                    if (ImGui::Button("Play", ImVec2(btn_w, 26))) {
                        g_app.startCapturePlayback(g_app.capture_playback_idx);
                    }
                    ImGui::PopStyleColor(2);
                    if (!can_play) ImGui::EndDisabled();

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
                    // Double-click to play
                    if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(0)) {
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

            Entity* ent = g_app.findEntity(entry.entity_id);
            ImVec4 col = ent ? ColorFromFloat4(ent->color) : ImVec4(0.6f, 0.6f, 0.6f, 1.0f);

            ImGui::PushStyleColor(ImGuiCol_Text, col);
            const char* lt = (ent && ent->type == EntityType::HIL) ? "HIL" : "SIL";
            ImGui::Text("[%s #%d/%s]", lt, entry.entity_id, entry.source);
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

                // ── Rolling waterfall state ──
                static const int SGRAM_COLS = 200;  // time slices (~20s at 10Hz)
                static const int MAX_LANES = 12;
                struct SgramLane {
                    int  entity_id;
                    int  axis;
                    bool active;
                    float buf[SGRAM_COLS][SPECTRUM_BINS]; // rolling data
                    int  head;
                    int  count;
                    float peak;  // auto color scale
                };
                static SgramLane lanes[MAX_LANES] = {};
                static bool s_sgram_inited = false;
                static float s_color_max = 0.0f;   // shared color scale
                static bool s_auto_scale = true;

                if (!s_sgram_inited) {
                    s_sgram_inited = true;
                    memset(lanes, 0, sizeof(lanes));
                    // Default: first entity, heave axis
                    if (!g_app.entities.empty()) {
                        lanes[0].entity_id = g_app.entities[0].id;
                        lanes[0].axis = 2; // heave
                        lanes[0].active = true;
                    }
                }

                // Push new time slices (~10Hz)
                static double s_last_push = 0.0;
                if (g_app.frame_time - s_last_push >= 0.1) {
                    s_last_push = g_app.frame_time;
                    float global_peak = 0.0f;
                    for (int li = 0; li < MAX_LANES; li++) {
                        if (!lanes[li].active) continue;
                        Entity* ent = g_app.findEntity(lanes[li].entity_id);
                        if (!ent || ent->history_count < 16) continue;

                        int col = lanes[li].head;
                        for (int b = 0; b < SPECTRUM_BINS; b++) {
                            lanes[li].buf[col][b] = ent->spectrum[lanes[li].axis][b];
                            if (lanes[li].buf[col][b] > global_peak)
                                global_peak = lanes[li].buf[col][b];
                        }
                        lanes[li].head = (lanes[li].head + 1) % SGRAM_COLS;
                        if (lanes[li].count < SGRAM_COLS) lanes[li].count++;
                        lanes[li].peak = global_peak;
                    }
                    if (s_auto_scale && global_peak > 0.001f)
                        s_color_max = s_color_max * 0.95f + global_peak * 0.05f; // smooth
                    if (s_color_max < 0.01f) s_color_max = 0.01f;
                }

                // ── Source selector ──
                int active_count = 0;
                for (int li = 0; li < MAX_LANES; li++) if (lanes[li].active) active_count++;

                if (ImGui::TreeNode("Sources")) {
                    for (auto& e : g_app.entities) {
                        ImGui::PushID(e.id);
                        const char* type_str = e.type == EntityType::SIL ? "SIL" : "HIL";
                        if (ImGui::TreeNode(e.name, "%s [%s]", e.name, type_str)) {
                            for (int a = 0; a < 6; a++) {
                                // Check if this entity+axis is already active
                                int found = -1;
                                for (int li = 0; li < MAX_LANES; li++) {
                                    if (lanes[li].active && lanes[li].entity_id == e.id && lanes[li].axis == a) {
                                        found = li; break;
                                    }
                                }
                                bool on = (found >= 0);
                                if (ImGui::Checkbox(axis_short[a], &on)) {
                                    if (on && found < 0) {
                                        // Find empty lane
                                        for (int li = 0; li < MAX_LANES; li++) {
                                            if (!lanes[li].active) {
                                                lanes[li].entity_id = e.id;
                                                lanes[li].axis = a;
                                                lanes[li].active = true;
                                                lanes[li].head = 0;
                                                lanes[li].count = 0;
                                                lanes[li].peak = 0.0f;
                                                memset(lanes[li].buf, 0, sizeof(lanes[li].buf));
                                                break;
                                            }
                                        }
                                    } else if (!on && found >= 0) {
                                        lanes[found].active = false;
                                    }
                                }
                                if (a < 5) ImGui::SameLine();
                            }
                            ImGui::TreePop();
                        }
                        ImGui::PopID();
                    }
                    ImGui::TreePop();
                }

                ImGui::SameLine();
                ImGui::Checkbox("Auto Scale", &s_auto_scale);
                if (!s_auto_scale) {
                    ImGui::SameLine();
                    ImGui::SetNextItemWidth(120);
                    ImGui::DragFloat("Max##cscale", &s_color_max, 0.01f, 0.01f, 100.0f, "%.2f");
                }

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
                    ImVec2 avail = ImGui::GetContentRegionAvail();
                    float lane_h = (avail.y - active_count * 20.0f) / (float)active_count;
                    if (lane_h < 60.0f) lane_h = 60.0f;

                    ImDrawList* dl = ImGui::GetWindowDrawList();

                    for (int li = 0; li < MAX_LANES; li++) {
                        if (!lanes[li].active) continue;

                        Entity* ent = g_app.findEntity(lanes[li].entity_id);
                        const char* ename = ent ? ent->name : "?";
                        const char* aname = axis_short[lanes[li].axis];
                        float fmax = ent ? ent->spectrum_freq_max : 30.0f;
                        if (fmax <= 0.0f) fmax = 30.0f;

                        // Label
                        ImGui::Text("%s / %s  (0-%.0f Hz)", ename, aname, fmax);

                        ImVec2 cpos = ImGui::GetCursorScreenPos();
                        float w = avail.x;
                        float h = lane_h;

                        // Reserve space
                        ImGui::Dummy(ImVec2(w, h));

                        // Draw heatmap cells
                        int cols = lanes[li].count;
                        if (cols < 1) continue;
                        if (cols > SGRAM_COLS) cols = SGRAM_COLS;

                        float cell_w = w / (float)SGRAM_COLS;
                        float cell_h = h / (float)SPECTRUM_BINS;
                        float cmax = s_color_max;

                        dl->PushClipRect(cpos, ImVec2(cpos.x + w, cpos.y + h), true);

                        for (int c = 0; c < cols; c++) {
                            // newest on right, oldest on left
                            int buf_col = (lanes[li].head - 1 - c + SGRAM_COLS) % SGRAM_COLS;
                            float x = cpos.x + w - (c + 1) * cell_w;

                            for (int b = 0; b < SPECTRUM_BINS; b++) {
                                float mag = lanes[li].buf[buf_col][b];
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
                    ImGui::TextDisabled("No sources selected. Open Sources to add entity/axis lanes.");
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

    // Tab Input + Console on the left
    ImGui::DockBuilderDockWindow("Input", left_id);
    ImGui::DockBuilderDockWindow("Console", left_id);

    // Data Streams on the bottom
    ImGui::DockBuilderDockWindow("Data Streams", bottom_id);

    // Center stays empty (entity cards float over it)

    ImGui::DockBuilderFinish(dockspace_id);
}

// ── Main Draw Function ──────────────────────────────────────────────

void DrawUI() {
    DrawMainMenuBar();
    DrawToolbar();

    // Offset viewport work area to account for toolbar height
    ImGuiViewport* main_vp = ImGui::GetMainViewport();
    main_vp->WorkPos.y += 52.0f;
    main_vp->WorkSize.y -= 52.0f;

    // Dockspace over entire window
    ImGuiID dockspace_id = ImGui::DockSpaceOverViewport(0, ImGui::GetMainViewport(), ImGuiDockNodeFlags_PassthruCentralNode);

    // Build default layout on first run (no saved .ini) or on reset request
    static bool first_frame = true;
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
    DrawInputPanel();
    DrawConsolePanel();
    DrawDataStreamsPanel();
    DrawDynamicsPanel();

    // Per-entity panels — float as undocked windows
    // Copy IDs first since DrawEntityCard can remove entities
    std::vector<int> ids;
    for (auto& e : g_app.entities) ids.push_back(e.id);
    for (int id : ids) {
        Entity* e = g_app.findEntity(id);
        if (e) {
            DrawEntityCard(*e);
            DrawEntitySettings(*e);
            DrawPlatformSetup(*e);
            DrawEntityConsole(*e);
        }
    }
}
