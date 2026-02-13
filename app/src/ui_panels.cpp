#include "ui_panels.h"
#include "serial_port.h"
#include "platform_viz.h"
#include "imgui.h"
#include "imgui_internal.h"
#include "implot.h"
#include <cstdio>
#include <cmath>
#include <ctime>
#include <cstring>
#include <string>
#include <vector>
#include <algorithm>
#include <filesystem>
#include <mutex>

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
            ImGui::MenuItem("ImGui Demo", nullptr, &ImGui::GetIO().ConfigFlags);  // placeholder
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

// ── Entity Card (3D viewport placeholder + readout) ─────────────────

static void DrawEntityCard(Entity& e) {
    ImGui::PushID(e.id);
    ImVec4 col = ColorFromFloat4(e.color);

    char title[128];
    const char* type_str = (e.type == EntityType::SIL) ? "SIL" : "HIL";
    snprintf(title, sizeof(title), "%s [%s]###entity_%d", e.name, type_str, e.id);

    ImGui::PushStyleColor(ImGuiCol_TitleBg, ImVec4(col.x * 0.3f, col.y * 0.3f, col.z * 0.3f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_TitleBgActive, ImVec4(col.x * 0.5f, col.y * 0.5f, col.z * 0.5f, 1.0f));

    if (ImGui::Begin(title, nullptr, ImGuiWindowFlags_None)) {
        // Header row: type badge + enable toggle + settings + remove
        ImGui::TextColored(col, "%s", type_str);
        ImGui::SameLine();
        ImGui::Checkbox("Enabled", &e.enabled);
        ImGui::SameLine();
        if (ImGui::SmallButton("Settings")) e.show_settings = !e.show_settings;
        ImGui::SameLine();
        if (ImGui::SmallButton("Platform")) e.show_platform = !e.show_platform;
        ImGui::SameLine();
        if (ImGui::SmallButton("Dynamics")) e.show_dynamics = !e.show_dynamics;
        ImGui::SameLine();
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.6f, 0.15f, 0.15f, 1.0f));
        bool remove = ImGui::SmallButton("Remove");
        ImGui::PopStyleColor();

        ImGui::Separator();

        // ── 3D Platform Viewport ──
        ImVec2 avail = ImGui::GetContentRegionAvail();
        float viz_h = fmaxf(150.0f, avail.y * 0.4f);
        ImVec2 p = ImGui::GetCursorScreenPos();
        ImDrawList* dl = ImGui::GetWindowDrawList();

        // InvisibleButton captures mouse for orbit camera control
        ImGui::InvisibleButton("##viz3d", ImVec2(avail.x, viz_h));
        bool viz_hovered = ImGui::IsItemHovered();
        bool viz_active  = ImGui::IsItemActive();

        DrawPlatformViz(dl, p, ImVec2(avail.x, viz_h), e, e.viz_cam, viz_hovered, viz_active);

        // Entity color border
        dl->AddRect(p, ImVec2(p.x + avail.x, p.y + viz_h),
                    IM_COL32((int)(col.x*255), (int)(col.y*255), (int)(col.z*255), 80));

        // ── Workspace Utilization (Phase C) ──
        ImGui::Separator();
        {
            // Overall utilization gauge
            float mu = e.state.max_util;
            ImVec4 mu_col = mu < 70.0f ? ImVec4(0.2f, 0.83f, 0.6f, 1.0f) :
                            mu < 90.0f ? ImVec4(0.98f, 0.75f, 0.15f, 1.0f) :
                                         ImVec4(0.98f, 0.44f, 0.44f, 1.0f);
            ImGui::TextColored(mu_col, "Workspace: %.0f%%", mu);
            ImGui::SameLine();
            ImGui::TextDisabled("(%.0f%% intensity)", e.config.intensity);
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
            } else if (e.hil_auto_connect && e.hil_port[0] != '\0') {
                ImGui::TextColored(ImVec4(0.95f, 0.75f, 0.2f, 1.0f), "Searching %s...", e.hil_port);
            } else {
                ImGui::TextDisabled("offline");
            }
        }

        if (remove) {
            ImGui::End();
            ImGui::PopStyleColor(2);
            ImGui::PopID();
            g_app.removeEntity(e.id);
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
                ImGui::Text("Motor Drive Parameters");
                ImGui::Separator();

                bool dt_changed = false;
                dt_changed |= ImGui::DragFloat("Virtual Gear", &e.config.geometry.virtual_gear, 1.0f, 1, 1000, "%.0f");
                dt_changed |= ImGui::DragFloat("Planetary Ratio", &e.config.geometry.planetary_ratio, 1.0f, 1, 200, "%.0f:1");
                int ppr = e.config.geometry.encoder_ppr;
                if (ImGui::DragInt("Encoder PPR", &ppr, 10, 100, 10000)) {
                    e.config.geometry.encoder_ppr = ppr;
                    dt_changed = true;
                }
                if (dt_changed) {
                    computeStepsPerDegree(&e.config.geometry);
                    e.config.platform.steps_per_degree = e.config.geometry.steps_per_degree;
                    g_app.log(e.id, "platform", "Drive train updated — %.2f steps/deg", e.config.geometry.steps_per_degree);
                }

                ImGui::Spacing();
                ImGui::Text("Steps/deg: %.2f", e.config.geometry.steps_per_degree);

                // Bit depth (resolution)
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

                float base_r = e.config.geometry.PD * vis_scale;
                dl->AddCircle(ImVec2(cx, cy), base_r, IM_COL32(60, 80, 100, 120), 48);
                float plat_r = e.config.geometry.RD * vis_scale;
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
                ImGui::TextDisabled("TX Rate: %d Hz  |  Bit Depth: %d", e.hil_tx_hz, e.config.bit_depth);
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
                        // Forward ESP32 lines to app console
                        int eid = e.id;
                        sp->setLineCallback([eid](const char* line) {
                            g_app.log(eid, "esp32", "%s", line);
                        });
                        if (sp->open(e.hil_port, 115200)) {
                            e.serial = sp;
                            e.transport.usb_connected = true;
                            snprintf(e.transport.usb_port, sizeof(e.transport.usb_port), "%s", e.hil_port);
                            e.hil_tel_seq = 0;
                            e.hil_auto_connect = true;  // manual connect enables auto-reconnect
                            g_app.log(e.id, "hil", "Connected to %s @ 115200", e.hil_port);
                            // Sync bit depth
                            char cmd[32];
                            snprintf(cmd, sizeof(cmd), "BITS:%d", e.config.bit_depth);
                            sp->sendCommand(cmd);
                        } else {
                            g_app.log(e.id, "hil", "Failed to open %s", e.hil_port);
                        }
                    }
                    ImGui::PopStyleColor(2);
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

// ── Dynamics Window (motion feel parameters) ────────────────────────

static void DrawDynamics(Entity& e) {
    if (!e.show_dynamics) return;

    ImGui::PushID(e.id + 3000);
    char title[128];
    snprintf(title, sizeof(title), "Dynamics: %s###dynamics_%d", e.name, e.id);

    ImGui::SetNextWindowSize(ImVec2(420, 560), ImGuiCond_FirstUseEver);
    if (ImGui::Begin(title, &e.show_dynamics)) {

        // ── Motion Intensity ──
        ImGui::Text("Motion Intensity");
        ImGui::Separator();
        ImGui::SliderFloat("Global Intensity", &e.config.intensity, 0.0f, 150.0f, "%.0f%%");
        ImGui::TextDisabled("Scales all axes uniformly. 100%% = full workspace.");

        // ── Per-Axis Gain Trim ──
        ImGui::Spacing();
        ImGui::Text("Per-Axis Gain");
        ImGui::Separator();

        const char* axis_names[] = {"Surge", "Sway", "Heave", "Roll", "Pitch", "Yaw"};
        for (int i = 0; i < 6; i++) {
            char lbl[32];
            snprintf(lbl, sizeof(lbl), "%s##gain", axis_names[i]);
            ImGui::PushItemWidth(-60);
            ImGui::SliderFloat(lbl, &e.config.axis_gain[i], 0.0f, 200.0f, "%.0f%%");
            ImGui::PopItemWidth();
        }
        if (ImGui::SmallButton("Reset All Gains")) {
            e.config.intensity = 100.0f;
            for (int i = 0; i < 6; i++) e.config.axis_gain[i] = 100.0f;
        }

        // Workspace info (read-only)
        ImGui::Spacing();
        ImGui::TextDisabled("Workspace: +/-%.0fmm surge, +/-%.1f\xc2\xb0 roll",
            e.config.axis_scales.scale[0],
            e.config.axis_scales.scale[3] * (float)(180.0 / M_PI));

        // ── Occupant Position Offset ──
        ImGui::Spacing();
        ImGui::Text("Occupant Position Offset");
        ImGui::Separator();
        ImGui::TextDisabled("Compensates parasitic motion from head offset above pivot.");

        ImGui::DragFloat("Lateral (X)", &e.config.occupant[0], 1.0f, -500, 500, "%.0f mm");
        ImGui::DragFloat("Fore/Aft (Y)", &e.config.occupant[1], 1.0f, -500, 500, "%.0f mm");
        ImGui::DragFloat("Vertical (Z)", &e.config.occupant[2], 1.0f, 0, 2000, "%.0f mm");
        if (ImGui::SmallButton("Reset Offset")) {
            e.config.occupant[0] = 0; e.config.occupant[1] = 0; e.config.occupant[2] = 800;
        }

        // ── Motion Cueing Algorithm ──
        ImGui::Spacing();
        ImGui::Text("Motion Cueing (MCA)");
        ImGui::Separator();

        const char* mca_presets[] = {"Off", "Gentle", "Moderate", "Aggressive", "Race Pro"};
        int mca_idx = e.config.mca.enabled ? 2 : 0;
        if (ImGui::Combo("MCA Preset", &mca_idx, mca_presets, 5)) {
            e.config.mca.enabled = (mca_idx > 0) ? 1 : 0;
            g_app.log(e.id, "dynamics", "MCA preset: %s", mca_presets[mca_idx]);
        }
        ImGui::TextDisabled("Filters telemetry for realistic motion cues.");
    }
    ImGui::End();
    ImGui::PopID();
}

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
    { InputSource::SimToolsUDP,     "SimTools UDP",     "UDP",  {0.20f, 0.83f, 0.60f, 1.0f}, {0.08f, 0.33f, 0.24f, 1.0f} },
    { InputSource::CapturePlayback, "Capture Playback", "CAP",  {0.95f, 0.75f, 0.20f, 1.0f}, {0.38f, 0.30f, 0.08f, 1.0f} },
    { InputSource::TestSignal,      "Test Signal",      "TST",  {0.75f, 0.50f, 0.95f, 1.0f}, {0.30f, 0.20f, 0.38f, 1.0f} },
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
        case InputSource::Manual:
            if (!g_app.entities.empty()) {
                float mx = 0;
                for (int i = 0; i < 6; i++) mx = fmaxf(mx, fabsf(g_app.entities[0].state.input_pct[i]));
                snprintf(buf, buf_sz, "6 axes  |  peak %.0f%%", mx);
            } else {
                snprintf(buf, buf_sz, "6 axes  |  no entity");
            }
            break;
        case InputSource::SimToolsUDP:
            if (g_app.simtools_active)
                snprintf(buf, buf_sz, ":%d  %.0f Hz  %d rx", g_app.simtools_port, g_app.simtools_rate, g_app.udp.packets_received.load());
            else
                snprintf(buf, buf_sz, ":%d  %d-bit  stopped", g_app.simtools_port, g_app.simtools_bit_depth);
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
        case InputSource::TestSignal: {
            if (g_app.test_signal.enabled) {
                double t = g_app.frame_time - g_app.test_signal_start_time;
                snprintf(buf, buf_sz, "Sine  running %.0fs", t);
            } else {
                snprintf(buf, buf_sz, "Sine  stopped");
            }
            break;
        }
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
    if (strcmp(source, "udp") == 0)           { *icon = "UDP"; *label = "SimTools UDP";     *color = ImVec4(0.20f, 0.83f, 0.60f, 1.0f); }
    else if (strcmp(source, "capture") == 0)  { *icon = "CAP"; *label = "Capture Playback"; *color = ImVec4(0.95f, 0.75f, 0.20f, 1.0f); }
    else if (strcmp(source, "test_signal") == 0) { *icon = "TST"; *label = "Test Signal";   *color = ImVec4(0.75f, 0.50f, 0.95f, 1.0f); }
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

// ── Input Panel (combobox card selector + per-source content) ────────

static void DrawInputPanel() {
    if (ImGui::Begin("Input")) {
        const char* axis_labels[] = {"Surge", "Sway", "Heave", "Roll", "Pitch", "Yaw"};

        // ══════════════════════════════════════════════════════════════
        //  SOURCE SELECTOR — rich combobox with card items
        // ══════════════════════════════════════════════════════════════
        {
            int cur_idx = InputSourceIndex(g_app.input_source);
            const auto& cur = g_input_sources[cur_idx];
            char status_buf[128];
            GetSourceStatus(cur.id, status_buf, sizeof(status_buf));

            // Build preview string for the combo header
            char preview[192];
            snprintf(preview, sizeof(preview), "[%s]  %s  —  %s", cur.icon, cur.label, status_buf);

            // Style the combo frame with the active source color
            ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(cur.color_dim.x, cur.color_dim.y, cur.color_dim.z, 0.5f));
            ImGui::PushStyleColor(ImGuiCol_FrameBgHovered, ImVec4(cur.color_dim.x * 1.4f, cur.color_dim.y * 1.4f, cur.color_dim.z * 1.4f, 0.6f));
            ImGui::PushStyleColor(ImGuiCol_Border, cur.color);
            ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 1.5f);
            ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 6.0f);
            ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(8, 10));

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
                        g_app.input_source = src.id;
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
                ImGui::EndCombo();
            }

            ImGui::PopStyleVar(3);
            ImGui::PopStyleColor(3);
        }

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        // ══════════════════════════════════════════════════════════════
        //  PER-SOURCE CONTENT (editable details)
        // ══════════════════════════════════════════════════════════════

        // ── Manual ──
        if (g_app.input_source == InputSource::Manual) {
            static float manual_input[6] = {};
            bool changed = false;
            for (int i = 0; i < 6; i++) {
                changed |= ImGui::SliderFloat(axis_labels[i], &manual_input[i], -100.0f, 100.0f, "%.0f%%");
            }
            if (ImGui::Button("Home All")) {
                memset(manual_input, 0, sizeof(manual_input));
                changed = true;
            }
            if (changed && !g_app.capture_playing) {
                for (auto& e : g_app.entities) {
                    memcpy(e.state.input_pct, manual_input, sizeof(manual_input));
                }
            }

            ImGui::Spacing();
            ImGui::TextDisabled("Direct control via sliders. Values propagate to all entities.");
        }

        // ── SimTools UDP ──
        else if (g_app.input_source == InputSource::SimToolsUDP) {
            // Editable connection settings
            ImGui::Text("Connection");
            ImGui::SetNextItemWidth(100);
            ImGui::InputInt("Port", &g_app.simtools_port, 0, 0);
            ImGui::SameLine();
            ImGui::SetNextItemWidth(80);
            ImGui::InputInt("Bits", &g_app.simtools_bit_depth, 0, 0);
            ImGui::SameLine();
            if (!g_app.simtools_active) {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.65f, 0.40f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.25f, 0.75f, 0.50f, 1.0f));
                if (ImGui::Button("Start")) {
                    g_app.startUdpListener();
                }
                ImGui::PopStyleColor(2);
            } else {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.85f, 0.20f, 0.20f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.95f, 0.30f, 0.30f, 1.0f));
                if (ImGui::Button("Stop")) {
                    g_app.stopUdpListener();
                }
                ImGui::PopStyleColor(2);
            }

            ImGui::Spacing();
            ImGui::Separator();

            if (g_app.simtools_active) {
                ImGui::TextColored(ImVec4(0.2f, 0.83f, 0.6f, 1.0f), "Receiving");
                ImGui::SameLine();
                ImGui::Text("%.0f Hz  |  %d packets", g_app.simtools_rate, g_app.udp.packets_received.load());

                ImGui::Spacing();
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
                    snprintf(overlay, sizeof(overlay), "%-6s %+.1f%%", axis_labels[i], vals[i]);
                    ImGui::ProgressBar(frac, ImVec2(-1, 0), overlay);
                    ImGui::PopStyleColor();
                }
            } else {
                ImGui::TextDisabled("Listener stopped. Press Start to receive data.");
            }
        }

        // ── Capture Playback ──
        else if (g_app.input_source == InputSource::CapturePlayback) {
            if (g_app.saved_recordings.empty()) {
                ImGui::TextDisabled("No saved captures.");
                ImGui::TextDisabled("Record in Data Streams and save to library first.");
            } else if (g_app.capture_playing) {
                // ── Active playback UI ──
                auto& sr = g_app.saved_recordings[g_app.capture_playback_idx];
                double elapsed = (g_app.frame_time - g_app.capture_start_time) * (double)g_app.capture_speed;
                double dur = sr.duration();
                float progress = dur > 0 ? (float)(elapsed / dur) : 0.0f;
                if (progress > 1.0f) progress = 1.0f;

                const char* src_icon; const char* src_label; ImVec4 src_color;
                GetCaptureSourceBadge(sr.source, &src_icon, &src_label, &src_color);
                ImDrawList* dl = ImGui::GetWindowDrawList();
                ImVec2 cpos = ImGui::GetCursorScreenPos();
                DrawBadge(dl, cpos, src_icon, src_color, 18.0f);
                float bw = ImGui::CalcTextSize(src_icon).x + 12.0f;
                ImGui::SetCursorScreenPos(ImVec2(cpos.x + bw + 6.0f, cpos.y));
                ImGui::TextColored(ImVec4(0.95f, 0.75f, 0.2f, 1.0f), "Playing: %s", sr.name);

                ImGui::ProgressBar(progress, ImVec2(-1, 0));
                ImGui::Text("%.1f / %.1fs  %s", elapsed, dur, g_app.capture_loop ? "(loop)" : "");

                // Speed controls
                ImGui::Text("Speed: %.0f%%", g_app.capture_speed * 100.0f);
                ImGui::SameLine();
                if (ImGui::SmallButton("-")) {
                    g_app.capture_speed -= 0.1f;
                    if (g_app.capture_speed < 0.1f) g_app.capture_speed = 0.1f;
                }
                ImGui::SameLine();
                if (ImGui::SmallButton("+")) {
                    g_app.capture_speed += 0.1f;
                    if (g_app.capture_speed > 2.0f) g_app.capture_speed = 2.0f;
                }
                ImGui::SameLine();
                if (g_app.capture_speed != 1.0f) {
                    ImGui::SameLine();
                    if (ImGui::SmallButton("Reset##spd")) g_app.capture_speed = 1.0f;
                }

                if (!g_app.entities.empty()) {
                    for (int i = 0; i < 6; i++) {
                        ImGui::Text("  %-6s %+6.1f%%", axis_labels[i], g_app.entities[0].state.input_pct[i]);
                    }
                }

                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.85f, 0.20f, 0.20f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.95f, 0.30f, 0.30f, 1.0f));
                if (ImGui::Button("Stop", ImVec2(-1, 30))) {
                    g_app.stopCapturePlayback();
                }
                ImGui::PopStyleColor(2);
            } else {
                // ── Scrollable card list for capture selection ──
                ImGui::Checkbox("Loop##capture", &g_app.capture_loop);
                ImGui::SameLine();
                bool can_play = g_app.capture_playback_idx >= 0;
                if (!can_play) ImGui::BeginDisabled();
                if (ImGui::Button("Play Capture", ImVec2(-1, 28))) {
                    g_app.startCapturePlayback(g_app.capture_playback_idx);
                }
                if (!can_play) ImGui::EndDisabled();

                ImGui::Spacing();

                // Full-height scrollable child with capture cards
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

        // ── Test Signal ──
        else if (g_app.input_source == InputSource::TestSignal) {
            auto& ts = g_app.test_signal;

            // Force sine waveform (other waveforms have unsafe discontinuities)
            ts.waveform = WaveformType::Sine;

            // Start/stop
            if (ts.enabled) {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.85f, 0.20f, 0.20f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.95f, 0.30f, 0.30f, 1.0f));
                if (ImGui::Button("Stop", ImVec2(80, 0))) ts.enabled = false;
                ImGui::PopStyleColor(2);
            } else {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.65f, 0.40f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.25f, 0.75f, 0.50f, 1.0f));
                if (ImGui::Button("Start", ImVec2(80, 0))) {
                    ts.enabled = true;
                    g_app.test_signal_start_time = g_app.frame_time;
                    // Sync active values to targets on start
                    memcpy(ts.active_freq, ts.frequency, sizeof(ts.frequency));
                    memcpy(ts.active_amp, ts.amplitude, sizeof(ts.amplitude));
                    memcpy(ts.active_phase, ts.phase_offset, sizeof(ts.phase_offset));
                }
                ImGui::PopStyleColor(2);
            }

            // Ramp-up controls
            ImGui::Checkbox("S-Curve Ramp", &ts.ramp_up);
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Smoothly ramp amplitude from zero on start\n"
                    "to prevent jarring motion. Uses a smoothstep\n"
                    "envelope (3t\xc2\xb2 - 2t\xc2\xb3) over the ramp duration.");
            }
            if (ts.ramp_up) {
                ImGui::SameLine();
                ImGui::SetNextItemWidth(100);
                ImGui::DragFloat("##ramp_dur", &ts.ramp_duration, 0.1f, 0.5f, 10.0f, "%.1fs");
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("Ramp-up duration in seconds");
            }

            ImGui::Checkbox("Smooth Changes", &ts.smooth_changes);
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Smoothly interpolate parameter changes while running.\n"
                    "Prevents abrupt jumps when adjusting frequency,\n"
                    "amplitude, or phase during live output.");
            }
            if (ts.smooth_changes) {
                ImGui::SameLine();
                ImGui::SetNextItemWidth(100);
                ImGui::DragFloat("##smooth_spd", &ts.smooth_rate, 0.1f, 0.5f, 20.0f, "%.1f /s");
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("Interpolation speed (higher = faster transition)");
            }

            ImGui::Spacing();
            ImGui::Separator();

            // Quick presets
            ImGui::Text("Quick:");
            ImGui::SameLine();
            if (ImGui::SmallButton("All 1 Hz")) {
                for (int i = 0; i < 6; i++) { ts.frequency[i] = 1.0f; ts.amplitude[i] = 50.0f; ts.phase_offset[i] = 0.0f; ts.axis_enabled[i] = true; }
            }
            ImGui::SameLine();
            if (ImGui::SmallButton("Sweep")) {
                float freqs[] = {0.5f, 1.0f, 2.0f, 3.0f, 5.0f, 8.0f};
                for (int i = 0; i < 6; i++) { ts.frequency[i] = freqs[i]; ts.amplitude[i] = 50.0f; ts.phase_offset[i] = 0.0f; ts.axis_enabled[i] = true; }
            }
            ImGui::SameLine();
            if (ImGui::SmallButton("Heave Only")) {
                for (int i = 0; i < 6; i++) { ts.axis_enabled[i] = false; ts.phase_offset[i] = 0.0f; }
                ts.axis_enabled[2] = true; ts.frequency[2] = 1.0f; ts.amplitude[2] = 80.0f;
            }
            ImGui::SameLine();
            if (ImGui::SmallButton("Phased")) {
                for (int i = 0; i < 6; i++) { ts.frequency[i] = 1.0f; ts.amplitude[i] = 50.0f; ts.phase_offset[i] = i * 60.0f; ts.axis_enabled[i] = true; }
            }

            // Saved presets
            static char s_preset_name[64] = "";
            static int s_delete_idx = -1;

            if (!g_app.test_signal_presets.empty()) {
                ImGui::Text("Saved:");
                ImGui::SameLine();
                for (int pi = 0; pi < (int)g_app.test_signal_presets.size(); pi++) {
                    auto& p = g_app.test_signal_presets[pi];
                    if (pi > 0) ImGui::SameLine();
                    if (ImGui::SmallButton(p.name)) {
                        ts.frequency[0] = p.config.frequency[0]; ts.frequency[1] = p.config.frequency[1];
                        ts.frequency[2] = p.config.frequency[2]; ts.frequency[3] = p.config.frequency[3];
                        ts.frequency[4] = p.config.frequency[4]; ts.frequency[5] = p.config.frequency[5];
                        memcpy(ts.amplitude, p.config.amplitude, sizeof(ts.amplitude));
                        memcpy(ts.phase_offset, p.config.phase_offset, sizeof(ts.phase_offset));
                        memcpy(ts.axis_enabled, p.config.axis_enabled, sizeof(ts.axis_enabled));
                        ts.ramp_up = p.config.ramp_up;
                        ts.ramp_duration = p.config.ramp_duration;
                    }
                    if (ImGui::IsItemHovered() && ImGui::IsMouseClicked(ImGuiMouseButton_Right))
                        s_delete_idx = pi;
                }
                if (s_delete_idx >= 0) {
                    ImGui::OpenPopup("##del_preset");
                }
                if (ImGui::BeginPopup("##del_preset")) {
                    if (s_delete_idx >= 0 && s_delete_idx < (int)g_app.test_signal_presets.size()) {
                        ImGui::Text("Delete '%s'?", g_app.test_signal_presets[s_delete_idx].name);
                        if (ImGui::Button("Delete", ImVec2(80, 0))) {
                            g_app.deleteTestSignalPreset(s_delete_idx);
                            s_delete_idx = -1;
                            ImGui::CloseCurrentPopup();
                        }
                        ImGui::SameLine();
                        if (ImGui::Button("Cancel", ImVec2(80, 0))) {
                            s_delete_idx = -1;
                            ImGui::CloseCurrentPopup();
                        }
                    }
                    ImGui::EndPopup();
                }
            }

            // Save current as preset
            ImGui::SetNextItemWidth(120);
            ImGui::InputTextWithHint("##preset_name", "Preset name...", s_preset_name, sizeof(s_preset_name));
            ImGui::SameLine();
            bool can_save = s_preset_name[0] != '\0';
            if (!can_save) ImGui::BeginDisabled();
            if (ImGui::SmallButton("Save Preset")) {
                g_app.saveTestSignalPreset(s_preset_name);
                s_preset_name[0] = '\0';
            }
            if (!can_save) ImGui::EndDisabled();

            ImGui::Spacing();

            // Per-axis table
            if (ImGui::BeginTable("##ts_axes", 5, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)) {
                ImGui::TableSetupColumn("Axis",       ImGuiTableColumnFlags_WidthFixed, 60);
                ImGui::TableSetupColumn("Freq (Hz)",  ImGuiTableColumnFlags_WidthStretch);
                ImGui::TableSetupColumn("Amp (%)",    ImGuiTableColumnFlags_WidthStretch);
                ImGui::TableSetupColumn("Phase",      ImGuiTableColumnFlags_WidthFixed, 60);
                ImGui::TableSetupColumn("On",         ImGuiTableColumnFlags_WidthFixed, 30);
                ImGui::TableHeadersRow();

                const char* phase_presets[] = {"0", "30", "45", "60", "90", "120", "180", "270"};
                const float phase_values[]  = {0.0f, 30.0f, 45.0f, 60.0f, 90.0f, 120.0f, 180.0f, 270.0f};
                const int n_phases = 8;

                for (int i = 0; i < 6; i++) {
                    ImGui::TableNextRow();
                    ImGui::PushID(i);
                    ImGui::TableNextColumn(); ImGui::Text("%s", axis_labels[i]);
                    ImGui::TableNextColumn(); ImGui::SetNextItemWidth(-1); ImGui::DragFloat("##freq", &ts.frequency[i], 0.1f, 0.01f, 100.0f, "%.2f");
                    ImGui::TableNextColumn(); ImGui::SetNextItemWidth(-1); ImGui::DragFloat("##amp", &ts.amplitude[i], 1.0f, 0.0f, 100.0f, "%.0f");
                    ImGui::TableNextColumn();
                    {
                        // Phase dropdown
                        int sel = -1;
                        for (int p = 0; p < n_phases; p++) {
                            if (fabsf(ts.phase_offset[i] - phase_values[p]) < 0.5f) { sel = p; break; }
                        }
                        char preview[16];
                        snprintf(preview, sizeof(preview), "%.0f\xc2\xb0", ts.phase_offset[i]);
                        ImGui::SetNextItemWidth(-1);
                        if (ImGui::BeginCombo("##phase", preview, ImGuiComboFlags_NoArrowButton)) {
                            for (int p = 0; p < n_phases; p++) {
                                char lbl[16];
                                snprintf(lbl, sizeof(lbl), "%s\xc2\xb0", phase_presets[p]);
                                if (ImGui::Selectable(lbl, p == sel)) ts.phase_offset[i] = phase_values[p];
                            }
                            ImGui::EndCombo();
                        }
                    }
                    ImGui::TableNextColumn(); ImGui::Checkbox("##en", &ts.axis_enabled[i]);
                    ImGui::PopID();
                }
                ImGui::EndTable();
            }
            // Live output
            if (ts.enabled && !g_app.entities.empty()) {
                ImGui::Spacing();
                ImGui::Separator();
                double t = g_app.frame_time - g_app.test_signal_start_time;

                // Show ramp progress if still ramping
                if (ts.ramp_up && t < (double)ts.ramp_duration) {
                    float ramp_pct = (float)(t / (double)ts.ramp_duration) * 100.0f;
                    char ramp_overlay[48];
                    snprintf(ramp_overlay, sizeof(ramp_overlay), "Ramping: %.0f%%", ramp_pct);
                    ImGui::PushStyleColor(ImGuiCol_PlotHistogram, ImVec4(0.95f, 0.75f, 0.2f, 0.8f));
                    ImGui::ProgressBar(ramp_pct / 100.0f, ImVec2(-1, 0), ramp_overlay);
                    ImGui::PopStyleColor();
                }

                ImGui::TextColored(ImVec4(0.75f, 0.50f, 0.95f, 1.0f), "Live Output");
                for (int i = 0; i < 6; i++) {
                    float v = g_app.entities[0].state.input_pct[i];
                    float frac = fabsf(v) / 100.0f;
                    ImGui::PushStyleColor(ImGuiCol_PlotHistogram, ImVec4(0.55f, 0.35f, 0.80f, 0.8f));
                    char overlay[32];
                    snprintf(overlay, sizeof(overlay), "%-6s %+.1f%%", axis_labels[i], v);
                    ImGui::ProgressBar(frac, ImVec2(-1, 0), overlay);
                    ImGui::PopStyleColor();
                }
                ImGui::TextDisabled("Running: %.1fs | Sine", t);
            } else if (!ts.enabled) {
                ImGui::Spacing();
                ImGui::TextDisabled("Configure signal parameters, then press Start.");
            }
        }
    }
    ImGui::End();
}

// ── Console Panel ───────────────────────────────────────────────────

static void DrawConsolePanel() {
    if (ImGui::Begin("Console")) {
        // Row 1: entity filter buttons + clear
        if (ImGui::SmallButton("All")) g_app.console_filter = -1;
        for (auto& e : g_app.entities) {
            ImGui::SameLine();
            ImVec4 col = ColorFromFloat4(e.color);
            ImGui::PushStyleColor(ImGuiCol_Button,
                g_app.console_filter == e.id ? ImVec4(col.x*0.5f, col.y*0.5f, col.z*0.5f, 1.0f) : ImVec4(0.2f,0.2f,0.2f,1.0f));
            char btn[32];
            snprintf(btn, sizeof(btn), "E%d", e.id);
            if (ImGui::SmallButton(btn)) {
                g_app.console_filter = (g_app.console_filter == e.id) ? -1 : e.id;
            }
            ImGui::PopStyleColor();
        }
        ImGui::SameLine();
        if (ImGui::SmallButton("Clear")) g_app.console_log.clear();

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
        char count_str[32];
        snprintf(count_str, sizeof(count_str), "(%d lines)", (int)g_app.console_log.size());
        ImGui::TextDisabled("%s", count_str);

        ImGui::Separator();

        // Log output — rolling window
        ImGui::BeginChild("log_scroll", ImVec2(0, 0), ImGuiChildFlags_None, ImGuiWindowFlags_HorizontalScrollbar);
        for (auto& entry : g_app.console_log) {
            if (g_app.console_filter >= 0 && entry.entity_id != g_app.console_filter) continue;

            Entity* ent = g_app.findEntity(entry.entity_id);
            ImVec4 col = ent ? ColorFromFloat4(ent->color) : ImVec4(0.6f, 0.6f, 0.6f, 1.0f);

            ImGui::PushStyleColor(ImGuiCol_Text, col);
            ImGui::Text("[E%d/%s]", entry.entity_id, entry.source);
            ImGui::PopStyleColor();
            ImGui::SameLine();
            ImGui::TextUnformatted(entry.message);
        }
        if (g_app.console_auto_scroll && ImGui::GetScrollY() >= ImGui::GetScrollMaxY())
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
    if (ImGui::Begin("Data Streams")) {

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
                    if (g_app.input_source == InputSource::SimToolsUDP) { src_name = "SimTools UDP"; src_color = ImVec4(0.2f, 0.6f, 0.9f, 1.0f); }
                    else if (g_app.input_source == InputSource::CapturePlayback) { src_name = "Capture Playback"; src_color = ImVec4(0.9f, 0.7f, 0.2f, 1.0f); }
                    else if (g_app.input_source == InputSource::TestSignal) { src_name = "Test Signal"; src_color = ImVec4(0.8f, 0.4f, 0.9f, 1.0f); }
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
                if (g_app.input_source == InputSource::SimToolsUDP) {
                    src_name = "SimTools UDP"; src_color = ImVec4(0.2f, 0.83f, 0.6f, 1.0f);
                } else if (g_app.input_source == InputSource::CapturePlayback) {
                    src_name = "Capture Playback"; src_color = ImVec4(0.95f, 0.75f, 0.2f, 1.0f);
                } else if (g_app.input_source == InputSource::TestSignal) {
                    src_name = "Test Signal"; src_color = ImVec4(0.75f, 0.50f, 0.95f, 1.0f);
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
                if (g_app.input_source == InputSource::SimToolsUDP) src_name = "SimTools UDP";
                else if (g_app.input_source == InputSource::CapturePlayback) src_name = "Capture Playback";
                else if (g_app.input_source == InputSource::TestSignal) src_name = "Test Signal";
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

// ── SimTools Panel ──────────────────────────────────────────────────

static void DrawSimToolsPanel() {
    if (ImGui::Begin("SimTools")) {
        ImGui::Text("SimTools UDP Listener");
        ImGui::Separator();

        // Config (disabled while running)
        if (g_app.simtools_active) ImGui::BeginDisabled();

        ImGui::InputInt("Port", &g_app.simtools_port);

        const int bit_opts[] = {8, 10, 12, 14, 16, 18};
        const char* bit_labels[] = {"8-bit", "10-bit", "12-bit", "14-bit", "16-bit", "18-bit"};
        int idx = 2;
        for (int i = 0; i < 6; i++) { if (bit_opts[i] == g_app.simtools_bit_depth) idx = i; }
        if (ImGui::Combo("Bit Depth", &idx, bit_labels, 6)) {
            g_app.simtools_bit_depth = bit_opts[idx];
        }

        if (g_app.simtools_active) ImGui::EndDisabled();

        ImGui::Separator();

        // Start / Stop
        if (!g_app.simtools_active) {
            if (ImGui::Button("Start Listening")) {
                if (!g_app.startUdpListener()) {
                    g_app.log(-1, "udp", "Failed to bind UDP port %d", g_app.simtools_port);
                }
            }
        } else {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.6f, 0.15f, 0.15f, 1.0f));
            if (ImGui::Button("Stop Listening")) {
                g_app.stopUdpListener();
            }
            ImGui::PopStyleColor();
        }

        // Live stats
        if (g_app.simtools_active) {
            ImGui::Separator();
            ImGui::TextColored(ImVec4(0.2f, 0.83f, 0.6f, 1.0f), "Listening on port %d", g_app.simtools_port);
            ImGui::Text("Rate: %.0f Hz", g_app.simtools_rate);
            ImGui::Text("Packets: %d rx, %d bad", g_app.udp.packets_received.load(), g_app.udp.packets_bad.load());

            // Show current values from UDP
            ImGui::Separator();
            ImGui::Text("Current Input:");
            const char* labels[] = {"Surge", "Sway", "Heave", "Roll", "Pitch", "Yaw"};
            for (int i = 0; i < 6; i++) {
                ImGui::Text("  %s: %.1f%%", labels[i], g_app.shared_input[i]);
            }
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

    // Tab Input + Console + SimTools on the left
    ImGui::DockBuilderDockWindow("Input", left_id);
    ImGui::DockBuilderDockWindow("Console", left_id);
    ImGui::DockBuilderDockWindow("SimTools", left_id);

    // Data Streams on the bottom
    ImGui::DockBuilderDockWindow("Data Streams", bottom_id);

    // Center stays empty (entity cards float over it)

    ImGui::DockBuilderFinish(dockspace_id);
}

// ── Main Draw Function ──────────────────────────────────────────────

void DrawUI() {
    DrawMainMenuBar();

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
    DrawSimToolsPanel();
    DrawDataStreamsPanel();

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
            DrawDynamics(*e);
        }
    }
}
