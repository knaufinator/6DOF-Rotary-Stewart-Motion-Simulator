#include "test_harness_panel.h"
#include "serial_port.h"
#include "app.h"
#include "cJSON.h"
#include "imgui.h"
#include "imgui_internal.h"
#include <cstdio>
#include <cstring>
#include <ctime>
#include <algorithm>
#include <sstream>

// ── Constants ───────────────────────────────────────────────────────

static const float POLL_INTERVAL    = 0.15f;          // seconds between STATUS polls
static const float SETTLE_TIME      = 2.0f;           // seconds after RATETEST for analyzer
static const float MAX_RATE_HZ      = 250000.0f;

// ── Singleton State ─────────────────────────────────────────────────

static TestHarnessState s_state;
static int s_console_scan_idx = 0;       // index into console_log for scanning
static int s_ctrl_entity_id   = -1;      // entity ID of controller during test
static float s_estimated_time = 5.0f;    // estimated test duration
static Entity* s_active_entity = nullptr; // set during tab render for entity-bound mode

TestHarnessState& GetTestHarnessState() { return s_state; }

void TestHarnessInit() {
    memset(s_state.port, 0, sizeof(s_state.port));
    s_state.serial = std::make_shared<SerialPort>();
}

// ── Helpers ─────────────────────────────────────────────────────────

static double Now() { return g_app.frame_time; }

static std::string NowISO() {
    time_t t = time(nullptr);
    struct tm lt;
#ifdef _WIN32
    localtime_s(&lt, &t);
#else
    localtime_r(&t, &lt);
#endif
    char buf[64];
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &lt);
    return buf;
}

// ── JSON Parsing ────────────────────────────────────────────────────

static bool ParseAnalyzerJSON(const char* json, std::array<AnalyzerMotor, 6>& out) {
    cJSON* root = cJSON_Parse(json);
    if (!root) return false;

    cJSON* motors = cJSON_GetObjectItem(root, "motors");
    if (!motors || !cJSON_IsArray(motors)) { cJSON_Delete(root); return false; }

    int n = cJSON_GetArraySize(motors);
    for (int i = 0; i < n && i < 6; i++) {
        cJSON* m = cJSON_GetArrayItem(motors, i);
        if (!m) continue;
        int idx = cJSON_GetObjectItem(m, "id") ? cJSON_GetObjectItem(m, "id")->valueint : i;
        if (idx < 0 || idx >= 6) continue;
        auto& o = out[idx];
        o.id = idx;
        auto gi = [&](const char* k) -> int {
            cJSON* v = cJSON_GetObjectItem(m, k);
            return v ? v->valueint : 0;
        };
        auto gf = [&](const char* k) -> float {
            cJSON* v = cJSON_GetObjectItem(m, k);
            return v ? (float)v->valuedouble : 0.0f;
        };
        o.position    = (int32_t)gi("pos");
        o.total_steps = (uint32_t)gi("steps");
        o.rate_hz     = gf("rate");
        o.dir         = gi("dir");
        o.min_us      = (uint32_t)gi("min_us");
        o.max_us      = (uint32_t)gi("max_us");
        o.avg_us      = (uint32_t)gi("avg_us");
        o.dir_changes = (uint32_t)gi("dir_chg");
        o.idle_ms     = (uint32_t)gi("idle_ms");
    }
    cJSON_Delete(root);
    return true;
}

static bool ParsePinsJSON(const char* json, PinCheck& out) {
    cJSON* root = cJSON_Parse(json);
    if (!root) return false;
    cJSON* pins = cJSON_GetObjectItem(root, "pins");
    if (!pins) { cJSON_Delete(root); return false; }
    cJSON* step = cJSON_GetObjectItem(pins, "step");
    cJSON* dir  = cJSON_GetObjectItem(pins, "dir");
    if (!step || !dir) { cJSON_Delete(root); return false; }
    for (int i = 0; i < 6; i++) {
        cJSON* sv = cJSON_GetArrayItem(step, i);
        cJSON* dv = cJSON_GetArrayItem(dir, i);
        out.step_levels[i] = sv ? sv->valueint : -1;
        out.dir_levels[i]  = dv ? dv->valueint : -1;
    }
    out.valid = true;
    out.timestamp = Now();
    cJSON_Delete(root);
    return true;
}

// Helper: parse "NNN steps/s/motor" and "NN.N% of max" from a RATETEST summary line
static void ParseRateLine(const std::string& line, float& rate_hz, float& pct_max, int64_t& time_us) {
    auto p1 = line.find('(');
    auto p2 = line.find(" steps/s/motor");
    if (p1 != std::string::npos && p2 != std::string::npos) {
        rate_hz = (float)atoi(line.c_str() + p1 + 1);
    }
    auto pp = line.find("% of max");
    if (pp != std::string::npos) {
        auto comma = line.rfind(',', pp);
        if (comma != std::string::npos)
            pct_max = (float)atof(line.c_str() + comma + 2);
    }
    // "50000 steps in 200550 us"
    auto in_pos = line.find(" steps in ");
    if (in_pos != std::string::npos) {
        const char* after = line.c_str() + in_pos + 10;
        time_us = (int64_t)atoll(after);
    }
}

// Helper: find M# index in a line (handles both "M0:" and "  M0:" with leading spaces)
static int ParseMotorIndex(const std::string& line) {
    for (size_t i = 0; i < line.size() - 1; i++) {
        if (line[i] == 'M' && line[i+1] >= '0' && line[i+1] <= '5') {
            if (i + 2 < line.size() && line[i+2] == ':')
                return line[i+1] - '0';
        }
    }
    return -1;
}

// Parse RATETEST lines from console log into snapshot
static bool s_in_pipeline_section = false;

static void ParseControllerLines(const std::vector<std::string>& lines, TestSnapshot& snap) {
    s_in_pipeline_section = false;

    for (auto& line : lines) {
        // "RATETEST:PIPELINE 50000 steps in NNN us (NNN steps/s/motor, NN.N% of max)"
        if (line.find("RATETEST:PIPELINE") != std::string::npos) {
            s_in_pipeline_section = true;
            snap.test_name = "RATETEST (PIPELINE+CONTINUOUS)";
            ParseRateLine(line, snap.pipe_rate_hz, snap.pipe_pct_max, snap.pipe_time_us);
        }
        // "RATETEST:CONTINUOUS 50000 steps in NNN us (NNN steps/s/motor, NN.N% of max)"
        if (line.find("RATETEST:CONTINUOUS") != std::string::npos) {
            s_in_pipeline_section = false;
            ParseRateLine(line, snap.ctrl_rate_hz, snap.ctrl_pct_max, snap.ctrl_time_us);
        }

        // Per-motor lines: "  M0: error=0 steps (PCNT)" or "  M0: pos=50000 (target was 50000, error=0)"
        int idx = ParseMotorIndex(line);
        if (idx >= 0 && idx < 6) {
            auto ep = line.find("error=");
            if (ep != std::string::npos) {
                int err = atoi(line.c_str() + ep + 6);
                if (s_in_pipeline_section)
                    snap.pipe_errors[idx] = err;
                else
                    snap.ctrl_errors[idx] = err;
            }
            // Mode tag (continuous section only)
            if (!s_in_pipeline_section) {
                if (line.find("(PCNT)") != std::string::npos) snap.ctrl_mode[idx] = "PCNT";
                else if (line.find("(RMT)") != std::string::npos) snap.ctrl_mode[idx] = "RMT";
                else if (line.find("(ISR)") != std::string::npos) snap.ctrl_mode[idx] = "ISR";
            }
        }
    }
}

// Generate text report
static std::string GenerateReport(const TestSnapshot& snap) {
    std::ostringstream ss;
    ss << "========================================\n";
    ss << "  Test Harness Verification Report\n";
    ss << "  " << snap.timestamp << "\n";
    ss << "========================================\n\n";
    ss << "Steps: " << snap.steps_requested << " x " << snap.num_motors << " motors\n\n";

    if (snap.pipe_rate_hz > 0) {
        ss << "--- Pipeline (handleStepDirection) ---\n";
        char buf[128];
        snprintf(buf, sizeof(buf), "Rate: %.0f Hz/motor (%.1f%% of 250 kHz), %lld us\n",
                 snap.pipe_rate_hz, snap.pipe_pct_max, (long long)snap.pipe_time_us);
        ss << buf;
        for (int i = 0; i < snap.num_motors; i++) {
            ss << "  M" << i << ": error=" << snap.pipe_errors[i] << "\n";
        }
        ss << "\n";
    }

    ss << "--- Continuous (hardware-counted) ---\n";
    {
        char buf[128];
        snprintf(buf, sizeof(buf), "Rate: %.0f Hz/motor (%.1f%% of 250 kHz), %lld us\n",
                 snap.ctrl_rate_hz, snap.ctrl_pct_max, (long long)snap.ctrl_time_us);
        ss << buf;
    }
    for (int i = 0; i < snap.num_motors; i++) {
        ss << "  M" << i << ": error=" << snap.ctrl_errors[i]
           << " (" << snap.ctrl_mode[i] << ")\n";
    }

    if (snap.analyzer_valid) {
        ss << "\n--- External Analyzer ---\n";
        for (int i = 0; i < snap.num_motors; i++) {
            auto& m = snap.analyzer[i];
            char line[128];
            snprintf(line, sizeof(line), "  M%d: pos=%d steps=%u min=%uus avg=%uus max=%uus dir_chg=%u",
                     i, m.position, m.total_steps, m.min_us, m.avg_us, m.max_us, m.dir_changes);
            ss << line << "\n";
        }
    }

    ss << "\nVerdict: " << (snap.passed ? "PASS" : "FAIL") << " - " << snap.verdict << "\n";
    return ss.str();
}

// ── Connection ──────────────────────────────────────────────────────

static void ConnectHarness(const char* port) {
    if (s_state.serial->isOpen()) s_state.serial->close();
    if (s_state.serial->open(port)) {
        s_state.serial->setEnqueueAll(true);  // bypass rate limiter — need every JSON response
        snprintf(s_state.port, sizeof(s_state.port), "%s", port);
        s_state.motors_valid = false;
        // Enable continuous streaming from the analyzer firmware
        s_state.serial->sendCommand("STREAM:1");
        s_state.streaming = true;
        g_app.log(-1, "harness", "Test harness connected on %s (streaming)", port);
    } else {
        g_app.log(-1, "harness", "Failed to open %s", port);
    }
}

static void DisconnectHarness() {
    if (s_state.serial->isOpen()) {
        s_state.serial->sendCommand("STREAM:0");
        s_state.serial->close();
        s_state.motors_valid = false;
        s_state.streaming = false;
        g_app.log(-1, "harness", "Test harness disconnected");
    }
}

// ── Polling ─────────────────────────────────────────────────────────

static void PollHarness() {
    if (!s_state.serial || !s_state.serial->isOpen()) return;
    double now = Now();
    if (now - s_state.last_poll_time < POLL_INTERVAL) return;
    s_state.last_poll_time = now;
    s_state.serial->sendCommand("STATUS");
}

static void ProcessHarnessLines() {
    if (!s_state.serial || !s_state.serial->isOpen()) return;
    auto lines = s_state.serial->drainLines();
    for (auto& line : lines) {
        if (line.size() > 2 && line[0] == '{') {
            if (line.find("\"motors\"") != std::string::npos) {
                if (ParseAnalyzerJSON(line.c_str(), s_state.motors)) {
                    s_state.motors_valid = true;
                    // Update rolling max_us window with recent peaks.
                    // The firmware sends cumulative max_us, so we compute the
                    // delta (new peak since last poll). Values > 1s are idle.
                    int slot = s_state.max_us_ring_head;
                    for (int i = 0; i < 6; i++) {
                        uint32_t raw = s_state.motors[i].max_us;
                        uint32_t prev = s_state.max_us_prev[i];
                        // Use avg_us as the baseline for this period if max hasn't changed
                        uint32_t val = (raw > prev && raw < 1000000)
                            ? raw : s_state.motors[i].avg_us;
                        s_state.max_us_ring[i][slot] = val;
                        s_state.max_us_prev[i] = raw;
                    }
                    s_state.max_us_ring_head = (slot + 1) % TestHarnessState::MAX_US_WINDOW;
                    if (s_state.max_us_ring_count < TestHarnessState::MAX_US_WINDOW)
                        s_state.max_us_ring_count++;
                }
            } else if (line.find("\"pins\"") != std::string::npos) {
                ParsePinsJSON(line.c_str(), s_state.pin_check);
            }
        }
    }
}

// ── Test Execution (scan console log — no serial line competition) ──

static Entity* FindControllerEntity() {
    for (auto& e : g_app.entities) {
        if (e.type == EntityType::HIL && e.serial && e.serial->isOpen())
            return &e;
    }
    return nullptr;
}

// Scan g_app.console_log for RATETEST output from the controller entity.
// This avoids competing with the main app's drainLines() which also
// consumes from the controller's serial port.
static void ScanConsoleForRatetest() {
    int log_size = (int)g_app.console_log.size();
    for (int i = s_console_scan_idx; i < log_size; i++) {
        auto& entry = g_app.console_log[i];
        if (entry.entity_id != s_ctrl_entity_id) continue;
        if (strcmp(entry.source, "esp32") != 0) continue;

        std::string msg(entry.message);
        if (msg.find("RATETEST:") != std::string::npos
            || msg.find("error=") != std::string::npos
            || msg.find("pos=") != std::string::npos
            || ParseMotorIndex(msg) >= 0) {
            s_state.ctrl_lines.push_back(msg);
        }
        if (msg.find("RATETEST:DONE") != std::string::npos) {
            s_state.phase = TestPhase::WaitAnalyzer;
            s_state.test_start_time = Now();
        }
    }
    s_console_scan_idx = log_size;
}

static void StartTest(Entity* entity_hint = nullptr) {
    if (!s_state.serial || !s_state.serial->isOpen()) return;

    // Reset analyzer counters
    s_state.serial->sendCommand("RESET");

    // Estimate duration: (steps / 250kHz) × 4 phases × safety + overhead
    s_estimated_time = (float)s_state.test_steps / MAX_RATE_HZ * 4.0f * 1.2f + 3.0f;

    // Record where to start scanning console
    s_console_scan_idx = (int)g_app.console_log.size();

    // Fire RATETEST on controller — use entity_hint if provided, else search
    Entity* ctrl = entity_hint;
    if (!ctrl || !ctrl->serial || !ctrl->serial->isOpen())
        ctrl = FindControllerEntity();

    char cmd[64];
    snprintf(cmd, sizeof(cmd), "RATETEST:%d:%d",
             s_state.test_steps, s_state.test_motor_count);

    if (ctrl && ctrl->serial && ctrl->serial->isOpen()) {
        s_ctrl_entity_id = ctrl->id;
        ctrl->serial->sendCommand(cmd);
        g_app.log(ctrl->id, "harness", "Started: %s (est. %.1fs) cobs=%d",
                  cmd, s_estimated_time,
                  ctrl->serial->isCobsMode() ? 1 : 0);
    } else {
        s_ctrl_entity_id = -1;
        g_app.log(-1, "harness", "Started: %s (observe-only, no controller connected)", cmd);
        s_estimated_time = 5.0f;
    }

    s_state.phase = TestPhase::WaitRatetest;
    s_state.test_start_time = Now();
    s_state.ctrl_lines.clear();
}

static void FinishTest() {
    g_app.log(-1, "harness", "FinishTest: ctrl_lines=%d console_log=%d scan_idx=%d entity=%d",
              (int)s_state.ctrl_lines.size(), (int)g_app.console_log.size(),
              s_console_scan_idx, s_ctrl_entity_id);
    // Request final analyzer snapshot
    s_state.serial->sendCommand("STATUS");

    TestSnapshot snap;
    snap.timestamp       = NowISO();
    snap.test_name       = "RATETEST";
    snap.steps_requested = s_state.test_steps;
    snap.num_motors      = s_state.test_motor_count;

    ParseControllerLines(s_state.ctrl_lines, snap);

    if (s_state.motors_valid) {
        snap.analyzer       = s_state.motors;
        snap.analyzer_valid = true;
    }

    // Pass/fail
    bool any_error = false;
    for (int i = 0; i < 6; i++)
        if (snap.ctrl_errors[i] != 0) any_error = true;

    if (any_error) {
        snap.passed = false;
        snap.verdict = "Controller reported step errors";
    } else if (snap.ctrl_rate_hz > 0 && snap.ctrl_rate_hz < 230000) {
        snap.passed = false;
        char buf[64]; snprintf(buf, sizeof(buf), "Rate too low: %.0f Hz", snap.ctrl_rate_hz);
        snap.verdict = buf;
    } else if (snap.ctrl_rate_hz == 0) {
        snap.passed = false;
        snap.verdict = "No rate data from controller";
    } else {
        snap.passed = true;
        char buf[64]; snprintf(buf, sizeof(buf), "%.0f Hz (%.1f%%), 0 errors",
                               snap.ctrl_rate_hz, snap.ctrl_pct_max);
        snap.verdict = buf;
    }

    s_state.snapshots.push_back(snap);
    s_state.phase = TestPhase::Done;
    g_app.log(-1, "harness", "Result: %s - %s",
              snap.passed ? "PASS" : "FAIL", snap.verdict.c_str());
}

static void UpdateTest() {
    if (s_state.phase == TestPhase::Idle || s_state.phase == TestPhase::Done) return;

    float elapsed = (float)(Now() - s_state.test_start_time);

    if (s_state.phase == TestPhase::WaitRatetest) {
        if (s_ctrl_entity_id >= 0) {
            ScanConsoleForRatetest();
        }
        // Timeout: with controller wait longer, observe-only finishes quickly
        float timeout = (s_ctrl_entity_id >= 0) ? s_estimated_time + 15.0f : s_estimated_time;
        if (elapsed > timeout) {
            if (s_ctrl_entity_id >= 0)
                g_app.log(-1, "harness", "Controller timeout (%.0fs), collecting analyzer data", elapsed);
            s_state.phase = TestPhase::WaitAnalyzer;
            s_state.test_start_time = Now();
        }
    }

    else if (s_state.phase == TestPhase::WaitAnalyzer) {
        // Recompute elapsed — test_start_time was reset when entering this phase
        float settle_elapsed = (float)(Now() - s_state.test_start_time);
        if (settle_elapsed > SETTLE_TIME) {
            FinishTest();
        }
    }
}

// ── ImGui Drawing ───────────────────────────────────────────────────

static ImVec4 MotorColor(int idx) {
    static const ImVec4 colors[6] = {
        {0.4f, 0.7f, 1.0f, 1.0f}, {0.4f, 1.0f, 0.4f, 1.0f},
        {1.0f, 0.7f, 0.3f, 1.0f}, {1.0f, 0.4f, 0.4f, 1.0f},
        {0.8f, 0.4f, 1.0f, 1.0f}, {1.0f, 1.0f, 0.4f, 1.0f},
    };
    return colors[idx % 6];
}

static void DrawConnectionBar() {
    bool connected = s_state.serial && s_state.serial->isOpen();
    ImVec4 dot = connected ? ImVec4(0.2f, 0.9f, 0.2f, 1.0f) : ImVec4(0.6f, 0.6f, 0.6f, 1.0f);

    ImGui::PushStyleColor(ImGuiCol_Text, dot);
    ImGui::Bullet();
    ImGui::PopStyleColor();
    ImGui::SameLine();

    if (connected) {
        ImGui::Text("Analyzer: %s", s_state.port);
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.5f, 1.0f), "(%d bytes RX)",
                           s_state.serial->rxBytes());
        ImGui::SameLine(ImGui::GetContentRegionAvail().x - 80);
        if (ImGui::SmallButton("Disconnect")) DisconnectHarness();
    } else {
        ImGui::Text("Analyzer: not connected");
        ImGui::SameLine(ImGui::GetContentRegionAvail().x - 250);

        auto ports = SerialPort::enumerate();
        static int sel = 0;
        if (sel >= (int)ports.size()) sel = 0;

        ImGui::SetNextItemWidth(140);
        if (!ports.empty()) {
            if (ImGui::BeginCombo("##th_port", ports[sel].port.c_str(),
                    ImGuiComboFlags_WidthFitPreview)) {
                for (int i = 0; i < (int)ports.size(); i++) {
                    if (ImGui::Selectable(ports[i].port.c_str(), i == sel)) sel = i;
                }
                ImGui::EndCombo();
            }
            ImGui::SameLine();
            if (ImGui::SmallButton("Connect") && sel < (int)ports.size())
                ConnectHarness(ports[sel].port.c_str());
        } else {
            ImGui::TextColored(ImVec4(0.8f, 0.4f, 0.4f, 1.0f), "No COM ports found");
        }
    }
}

static void DrawMotorTable() {
    if (!s_state.motors_valid) {
        ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.5f, 1.0f), "Waiting for data...");
        return;
    }

    // Reset button
    if (s_state.serial && s_state.serial->isOpen()) {
        if (ImGui::SmallButton("Reset Counters")) {
            s_state.serial->sendCommand("RESET");
            g_app.log(-1, "harness", "Analyzer counters reset");
        }
    }

    ImGuiTableFlags flags = ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg
                          | ImGuiTableFlags_Resizable | ImGuiTableFlags_SizingFixedFit;

    if (ImGui::BeginTable("##motors", 8, flags)) {
        ImGui::TableSetupColumn("Motor",   ImGuiTableColumnFlags_WidthFixed, 45);
        ImGui::TableSetupColumn("Pos",     ImGuiTableColumnFlags_WidthFixed, 70);
        ImGui::TableSetupColumn("Steps",   ImGuiTableColumnFlags_WidthFixed, 70);
        ImGui::TableSetupColumn("Rate Hz", ImGuiTableColumnFlags_WidthFixed, 65);
        ImGui::TableSetupColumn("Min us",  ImGuiTableColumnFlags_WidthFixed, 55);
        ImGui::TableSetupColumn("Avg us",  ImGuiTableColumnFlags_WidthFixed, 55);
        ImGui::TableSetupColumn("Peak us", ImGuiTableColumnFlags_WidthFixed, 65);
        ImGui::TableSetupColumn("DirChg",  ImGuiTableColumnFlags_WidthFixed, 55);
        ImGui::TableHeadersRow();

        for (int i = 0; i < 6; i++) {
            auto& m = s_state.motors[i];
            ImGui::TableNextRow();
            ImGui::TableNextColumn(); ImGui::TextColored(MotorColor(i), "M%d", i);
            ImGui::TableNextColumn(); ImGui::Text("%d", m.position);
            ImGui::TableNextColumn(); ImGui::Text("%u", m.total_steps);
            ImGui::TableNextColumn();
            if (m.rate_hz > 0) ImGui::Text("%.0f", m.rate_hz);
            else ImGui::TextColored(ImVec4(0.4f, 0.4f, 0.4f, 1.0f), "-");
            ImGui::TableNextColumn();
            if (m.min_us > 0 && m.min_us < 1000000) ImGui::Text("%u", m.min_us);
            else ImGui::Text("-");
            ImGui::TableNextColumn(); ImGui::Text("%u", m.avg_us);
            // Peak us: rolling window max (last ~4 seconds), not all-time
            ImGui::TableNextColumn();
            {
                uint32_t peak = 0;
                int n = s_state.max_us_ring_count;
                for (int j = 0; j < n; j++)
                    if (s_state.max_us_ring[i][j] > peak)
                        peak = s_state.max_us_ring[i][j];
                if (peak > 0 && peak < 1000000)
                    ImGui::Text("%u", peak);
                else
                    ImGui::TextColored(ImVec4(0.4f, 0.4f, 0.4f, 1.0f), "-");
            }
            ImGui::TableNextColumn(); ImGui::Text("%u", m.dir_changes);
        }
        ImGui::EndTable();
    }
}

static void DrawPinCheck() {
    bool connected = s_state.serial && s_state.serial->isOpen();
    if (!connected) ImGui::BeginDisabled();
    if (ImGui::SmallButton("Read GPIO Pins")) {
        s_state.serial->sendCommand("PINS");
    }
    if (!connected) ImGui::EndDisabled();

    if (!s_state.pin_check.valid) {
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.5f, 1.0f), "Press to read STEP/DIR pin levels");
        return;
    }

    // Table layout for pin status
    ImGuiTableFlags flags = ImGuiTableFlags_Borders | ImGuiTableFlags_SizingFixedFit;
    if (ImGui::BeginTable("##pins", 7, flags)) {
        ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed, 40);
        for (int i = 0; i < 6; i++) {
            char hdr[8]; snprintf(hdr, sizeof(hdr), "M%d", i);
            ImGui::TableSetupColumn(hdr, ImGuiTableColumnFlags_WidthFixed, 40);
        }
        ImGui::TableHeadersRow();

        // STEP row
        ImGui::TableNextRow();
        ImGui::TableNextColumn(); ImGui::Text("STEP");
        for (int i = 0; i < 6; i++) {
            ImGui::TableNextColumn();
            int v = s_state.pin_check.step_levels[i];
            ImVec4 c = (v > 0) ? ImVec4(0.2f, 0.9f, 0.2f, 1.0f) : ImVec4(0.4f, 0.4f, 0.4f, 1.0f);
            ImGui::TextColored(c, "%d", v);
        }
        // DIR row
        ImGui::TableNextRow();
        ImGui::TableNextColumn(); ImGui::Text("DIR");
        for (int i = 0; i < 6; i++) {
            ImGui::TableNextColumn();
            int v = s_state.pin_check.dir_levels[i];
            ImVec4 c = (v > 0) ? ImVec4(0.9f, 0.9f, 0.2f, 1.0f) : ImVec4(0.4f, 0.4f, 0.4f, 1.0f);
            ImGui::TextColored(c, "%d", v);
        }
        ImGui::EndTable();
    }
}

static void DrawTestSection() {
    bool connected = s_state.serial && s_state.serial->isOpen();
    bool busy = (s_state.phase != TestPhase::Idle && s_state.phase != TestPhase::Done);
    bool can_run = connected && !busy;

    // ── Description ──────────────────────────────────────────────────
    ImGui::TextWrapped(
        "Measures the actual stepping rate of the MCPWM hardware motor drivers. "
        "Runs two benchmarks: Pipeline (full motion pipeline under real conditions) "
        "and Continuous (raw hardware pulse generation at maximum rate). "
        "Motors automatically return to their starting position after each test.");
    ImGui::Spacing();

    // ── Steps per motor ──────────────────────────────────────────────
    ImGui::TextColored(ImVec4(0.9f, 0.8f, 0.3f, 1.0f), "Steps Per Motor");
    ImGui::TextWrapped(
        "Number of step pulses to send to each motor. Higher values give more "
        "accurate rate measurements but take longer to complete.");
    ImGui::Spacing();

    // Preset buttons
    ImGui::Text("Presets:");
    ImGui::SameLine();
    if (ImGui::SmallButton("10k Quick")) s_state.test_steps = 10000;
    ImGui::SameLine();
    if (ImGui::SmallButton("50k Standard")) s_state.test_steps = 50000;
    ImGui::SameLine();
    if (ImGui::SmallButton("250k Stress")) s_state.test_steps = 250000;
    ImGui::SameLine();
    if (ImGui::SmallButton("1M Max")) s_state.test_steps = 1000000;

    ImGui::SetNextItemWidth(200);
    ImGui::InputInt("Steps##steps_input", &s_state.test_steps, 10000, 100000);
    if (s_state.test_steps < 1000) s_state.test_steps = 1000;
    if (s_state.test_steps > 1000000) s_state.test_steps = 1000000;
    ImGui::SameLine();
    ImGui::TextColored(ImVec4(0.6f, 0.6f, 0.6f, 1.0f), "(1,000 - 1,000,000)");
    ImGui::Spacing();

    // ── Motor count ──────────────────────────────────────────────────
    ImGui::TextColored(ImVec4(0.9f, 0.8f, 0.3f, 1.0f), "Motor Count");
    ImGui::TextWrapped(
        "How many motors to drive simultaneously. Use 1 to isolate a single "
        "channel for debugging, or 6 to verify full-system parallel throughput.");

    ImGui::SetNextItemWidth(200);
    ImGui::SliderInt("Motors##motor_slider", &s_state.test_motor_count, 1, 6, "%d motor(s)");
    ImGui::Spacing();

    // ── Estimated duration ───────────────────────────────────────────
    float est_sec = (float)s_state.test_steps / MAX_RATE_HZ * 4.0f * 1.2f + 3.0f;
    ImGui::TextColored(ImVec4(0.6f, 0.6f, 0.6f, 1.0f),
        "Estimated duration: ~%.0f sec  |  %d steps x %d motor(s)",
        est_sec, s_state.test_steps, s_state.test_motor_count);
    ImGui::Spacing();

    // ── Run button ───────────────────────────────────────────────────
    if (!can_run) ImGui::BeginDisabled();
    if (ImGui::Button("Run Performance Benchmark", ImVec2(250, 32))) {
        StartTest(s_active_entity);
    }
    if (!can_run) ImGui::EndDisabled();

    // Status / pre-conditions
    if (!connected) {
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(0.8f, 0.4f, 0.4f, 1.0f), "Analyzer not connected");
    } else {
        Entity* ctrl = s_active_entity ? s_active_entity : FindControllerEntity();
        bool ctrl_ok = ctrl && ctrl->serial && ctrl->serial->isOpen();
        if (!ctrl_ok) {
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.3f, 1.0f), "Observe-only (no controller)");
        }
    }

    // Progress bar during test
    if (busy) {
        float elapsed = (float)(Now() - s_state.test_start_time);
        float total_est = s_estimated_time + SETTLE_TIME;
        float frac = elapsed / total_est;
        if (frac > 1.0f) frac = 1.0f;

        const char* phase_str = "Unknown";
        ImVec4 phase_col = ImVec4(1, 1, 1, 1);
        if (s_state.phase == TestPhase::WaitRatetest) {
            phase_str = "Running RATETEST on controller...";
            phase_col = ImVec4(1.0f, 0.9f, 0.3f, 1.0f);
        } else if (s_state.phase == TestPhase::WaitAnalyzer) {
            phase_str = "Settling — collecting analyzer data...";
            phase_col = ImVec4(0.3f, 0.8f, 1.0f, 1.0f);
        }

        ImGui::TextColored(phase_col, "%s", phase_str);
        ImGui::ProgressBar(frac, ImVec2(-1, 0),
            s_state.phase == TestPhase::WaitRatetest ? "Controller" : "Analyzer");

        // Show captured controller lines in real-time
        if (!s_state.ctrl_lines.empty()) {
            ImGui::TextColored(ImVec4(0.6f, 0.6f, 0.6f, 1.0f), "Controller output:");
            for (auto& l : s_state.ctrl_lines) {
                ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "  %s", l.c_str());
            }
        }
    }

    // Last test result display
    if (s_state.phase == TestPhase::Done && !s_state.snapshots.empty()) {
        auto& snap = s_state.snapshots.back();
        ImGui::Spacing();

        // Result banner
        ImVec4 rc = snap.passed ? ImVec4(0.15f, 0.35f, 0.15f, 1.0f) : ImVec4(0.4f, 0.1f, 0.1f, 1.0f);
        ImVec4 tc = snap.passed ? ImVec4(0.2f, 1.0f, 0.2f, 1.0f) : ImVec4(1.0f, 0.3f, 0.3f, 1.0f);

        ImGui::PushStyleColor(ImGuiCol_ChildBg, rc);
        ImGui::BeginChild("##result_banner", ImVec2(0, ImGui::GetTextLineHeightWithSpacing() * 2), true);
        ImGui::TextColored(tc, "  %s  %s", snap.passed ? "PASS" : "FAIL", snap.verdict.c_str());
        ImGui::EndChild();
        ImGui::PopStyleColor();

        int nm = snap.num_motors;

        // ── Pipeline results ──
        if (snap.pipe_rate_hz > 0) {
            ImGui::Spacing();
            ImGui::TextColored(ImVec4(0.9f, 0.8f, 0.3f, 1.0f), "Pipeline (handleStepDirection continuous mode)");
            ImGui::Text("  %.0f Hz/motor (%.1f%% of 250 kHz), %lld us total",
                        snap.pipe_rate_hz, snap.pipe_pct_max, (long long)snap.pipe_time_us);
            bool any_pipe_err = false;
            for (int i = 0; i < nm; i++) if (snap.pipe_errors[i] != 0) any_pipe_err = true;
            if (any_pipe_err) {
                for (int i = 0; i < nm; i++) {
                    ImGui::SameLine();
                    if (snap.pipe_errors[i] == 0)
                        ImGui::TextColored(ImVec4(0.2f, 0.9f, 0.2f, 1.0f), "M%d:0", i);
                    else
                        ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "M%d:%d", i, snap.pipe_errors[i]);
                }
            } else {
                ImGui::SameLine();
                ImGui::TextColored(ImVec4(0.2f, 0.9f, 0.2f, 1.0f), "  All %d motors: error=0", nm);
            }
        }

        // ── Continuous results ──
        if (snap.ctrl_rate_hz > 0) {
            ImGui::Spacing();
            ImGui::TextColored(ImVec4(0.3f, 0.8f, 1.0f, 1.0f), "Continuous (hardware-counted, direct)");
            ImGui::Text("  %.0f Hz/motor (%.1f%% of 250 kHz), %lld us total",
                        snap.ctrl_rate_hz, snap.ctrl_pct_max, (long long)snap.ctrl_time_us);

            ImGuiTableFlags tf = ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg
                               | ImGuiTableFlags_SizingFixedFit;
            if (ImGui::BeginTable("##ctrl_result", 4, tf)) {
                ImGui::TableSetupColumn("Motor", ImGuiTableColumnFlags_WidthFixed, 45);
                ImGui::TableSetupColumn("Mode",  ImGuiTableColumnFlags_WidthFixed, 50);
                ImGui::TableSetupColumn("Error", ImGuiTableColumnFlags_WidthFixed, 50);
                ImGui::TableSetupColumn("",      ImGuiTableColumnFlags_WidthStretch);
                ImGui::TableHeadersRow();
                for (int i = 0; i < nm; i++) {
                    ImGui::TableNextRow();
                    ImGui::TableNextColumn(); ImGui::TextColored(MotorColor(i), "M%d", i);
                    ImGui::TableNextColumn();
                    if (snap.ctrl_mode[i] == "PCNT")
                        ImGui::TextColored(ImVec4(0.3f, 0.8f, 0.3f, 1.0f), "PCNT");
                    else if (snap.ctrl_mode[i] == "RMT")
                        ImGui::TextColored(ImVec4(0.3f, 0.5f, 0.9f, 1.0f), "RMT");
                    else if (snap.ctrl_mode[i] == "ISR")
                        ImGui::TextColored(ImVec4(0.9f, 0.6f, 0.2f, 1.0f), "ISR");
                    else
                        ImGui::Text("-");
                    ImGui::TableNextColumn();
                    if (snap.ctrl_errors[i] == 0)
                        ImGui::TextColored(ImVec4(0.2f, 0.9f, 0.2f, 1.0f), "0");
                    else
                        ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "%d", snap.ctrl_errors[i]);
                    ImGui::TableNextColumn();
                    if (snap.ctrl_mode[i] == "PCNT")
                        ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.5f, 1.0f), "HW pulse counter");
                    else if (snap.ctrl_mode[i] == "RMT")
                        ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.5f, 1.0f), "RMT TX hw loop");
                    else if (snap.ctrl_mode[i] == "ISR")
                        ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.5f, 1.0f), "TEZ ISR fallback");
                }
                ImGui::EndTable();
            }
        }

        // ── Analyzer results ──
        if (snap.analyzer_valid) {
            ImGui::Spacing();
            ImGui::TextColored(ImVec4(0.8f, 0.6f, 1.0f, 1.0f), "External Analyzer (measured on wires)");
            ImGuiTableFlags tf = ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg
                               | ImGuiTableFlags_SizingFixedFit;
            if (ImGui::BeginTable("##anlz_result", 6, tf)) {
                ImGui::TableSetupColumn("Motor",  ImGuiTableColumnFlags_WidthFixed, 45);
                ImGui::TableSetupColumn("Steps",  ImGuiTableColumnFlags_WidthFixed, 70);
                ImGui::TableSetupColumn("Min us", ImGuiTableColumnFlags_WidthFixed, 50);
                ImGui::TableSetupColumn("Avg us", ImGuiTableColumnFlags_WidthFixed, 50);
                ImGui::TableSetupColumn("DirChg", ImGuiTableColumnFlags_WidthFixed, 50);
                ImGui::TableSetupColumn("Status", ImGuiTableColumnFlags_WidthStretch);
                ImGui::TableHeadersRow();
                for (int i = 0; i < nm; i++) {
                    auto& m = snap.analyzer[i];
                    ImGui::TableNextRow();
                    ImGui::TableNextColumn(); ImGui::TextColored(MotorColor(i), "M%d", i);
                    ImGui::TableNextColumn(); ImGui::Text("%u", m.total_steps);
                    ImGui::TableNextColumn();
                    if (m.min_us > 0 && m.min_us < 1000000) ImGui::Text("%u", m.min_us);
                    else ImGui::Text("-");
                    ImGui::TableNextColumn(); ImGui::Text("%u", m.avg_us);
                    ImGui::TableNextColumn(); ImGui::Text("%u", m.dir_changes);
                    ImGui::TableNextColumn();
                    const char* status_str = "No pulses — check wiring";
                    ImVec4 status_col = ImVec4(1.0f, 0.3f, 0.3f, 1.0f);  // red
                    if (m.total_steps > 0) {
                        status_str = "Pulses OK";
                        status_col = ImVec4(0.2f, 0.9f, 0.2f, 1.0f);  // green
                    }
                    ImGui::TextColored(status_col, "%s", status_str);
                }
                ImGui::EndTable();
            }
        }

        // ── Raw controller output (collapsible) ──
        if (!s_state.ctrl_lines.empty()) {
            char tree_label[64];
            snprintf(tree_label, sizeof(tree_label), "Raw Controller Output (%d lines)", (int)s_state.ctrl_lines.size());
            if (ImGui::TreeNode(tree_label)) {
                for (auto& l : s_state.ctrl_lines) {
                    ImGui::TextColored(ImVec4(0.6f, 0.6f, 0.6f, 1.0f), "%s", l.c_str());
                }
                ImGui::TreePop();
            }
        }

        // Actions
        ImGui::Spacing();
        if (ImGui::SmallButton("Copy Report")) {
            std::string report = GenerateReport(snap);
            ImGui::SetClipboardText(report.c_str());
            g_app.log(-1, "harness", "Report copied to clipboard");
        }
        ImGui::SameLine();
        if (ImGui::SmallButton("New Test")) {
            s_state.phase = TestPhase::Idle;
        }
    }
}

static void DrawHistory() {
    if (s_state.snapshots.size() <= 1) return;

    int n = (int)s_state.snapshots.size();
    ImGui::Text("History (%d)", n);
    ImGuiTableFlags flags = ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg
                          | ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_ScrollY;
    float h = ImGui::GetTextLineHeightWithSpacing() * (std::min)(n + 1, 6);
    if (ImGui::BeginTable("##history", 5, flags, ImVec2(0, h))) {
        ImGui::TableSetupColumn("#",      ImGuiTableColumnFlags_WidthFixed, 25);
        ImGui::TableSetupColumn("Time",   ImGuiTableColumnFlags_WidthFixed, 130);
        ImGui::TableSetupColumn("Rate",   ImGuiTableColumnFlags_WidthFixed, 80);
        ImGui::TableSetupColumn("Result", ImGuiTableColumnFlags_WidthFixed, 50);
        ImGui::TableSetupColumn("",       ImGuiTableColumnFlags_WidthFixed, 50);
        ImGui::TableHeadersRow();
        for (int i = n - 1; i >= 0; i--) {
            auto& s = s_state.snapshots[i];
            ImGui::TableNextRow();
            ImGui::TableNextColumn(); ImGui::Text("%d", i + 1);
            ImGui::TableNextColumn(); ImGui::Text("%s", s.timestamp.c_str());
            ImGui::TableNextColumn(); ImGui::Text("%.0f Hz", s.ctrl_rate_hz);
            ImGui::TableNextColumn();
            ImVec4 c = s.passed ? ImVec4(0.2f, 0.9f, 0.2f, 1) : ImVec4(1, 0.3f, 0.3f, 1);
            ImGui::TextColored(c, s.passed ? "PASS" : "FAIL");
            ImGui::TableNextColumn();
            char id[16]; snprintf(id, sizeof(id), "Copy##%d", i);
            if (ImGui::SmallButton(id)) {
                ImGui::SetClipboardText(GenerateReport(s).c_str());
            }
        }
        ImGui::EndTable();
    }
    if (ImGui::SmallButton("Clear History")) s_state.snapshots.clear();
}

// ── Main Panel (standalone window — legacy) ─────────────────────────

void DrawTestHarnessPanel() {
    if (!s_state.show_panel) return;

    s_active_entity = nullptr;  // standalone mode — find controller by search

    PollHarness();
    ProcessHarnessLines();
    UpdateTest();

    if (ImGui::Begin("Test Harness", &s_state.show_panel)) {
        DrawConnectionBar();
        ImGui::Separator();

        if (ImGui::CollapsingHeader("Live Analyzer Metrics", ImGuiTreeNodeFlags_DefaultOpen)) {
            DrawMotorTable();
        }

        if (ImGui::CollapsingHeader("Pin Check")) {
            DrawPinCheck();
        }

        ImGui::Separator();
        if (ImGui::CollapsingHeader("Stepper Motor Performance Benchmark", ImGuiTreeNodeFlags_DefaultOpen)) {
            DrawTestSection();
        }

        ImGui::Separator();
        DrawHistory();
    }
    ImGui::End();
}

// ── Entity Tab Content ──────────────────────────────────────────────
// Embedded in the HIL entity card tab bar. Uses entity's serial for
// controller commands; analyzer connection is managed internally.

void DrawTestHarnessTabContent(Entity& e) {
    s_active_entity = &e;

    PollHarness();
    ProcessHarnessLines();
    UpdateTest();

    ImGui::PushID("test_harness_tab");

    // Analyzer connection bar (compact)
    DrawConnectionBar();
    ImGui::Separator();

    // Controller status — show entity binding
    {
        bool ctrl_ok = e.serial && e.serial->isOpen();
        ImVec4 dot = ctrl_ok ? ImVec4(0.2f, 0.9f, 0.2f, 1.0f) : ImVec4(0.8f, 0.4f, 0.4f, 1.0f);
        ImGui::PushStyleColor(ImGuiCol_Text, dot);
        ImGui::Bullet();
        ImGui::PopStyleColor();
        ImGui::SameLine();
        if (ctrl_ok) {
            ImGui::Text("Controller: %s", e.serial->portName());
            if (e.hil_handshake_phase == HandshakePhase::Ready) {
                ImGui::SameLine();
                ImGui::TextColored(ImVec4(0.2f, 0.9f, 0.2f, 1.0f), "(handshake OK)");
            }
        } else {
            ImGui::TextColored(ImVec4(0.8f, 0.4f, 0.4f, 1.0f), "Controller: not connected");
        }
    }
    ImGui::Separator();

    if (ImGui::CollapsingHeader("Live Analyzer Metrics", ImGuiTreeNodeFlags_DefaultOpen)) {
        DrawMotorTable();
    }

    if (ImGui::CollapsingHeader("Pin Check")) {
        DrawPinCheck();
    }

    ImGui::Separator();
    if (ImGui::CollapsingHeader("Stepper Motor Performance Benchmark", ImGuiTreeNodeFlags_DefaultOpen)) {
        DrawTestSection();
    }

    ImGui::Separator();
    DrawHistory();

    ImGui::PopID();
    s_active_entity = nullptr;
}
