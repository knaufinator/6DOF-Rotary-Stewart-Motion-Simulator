#pragma once

// Test-only facade. SDK and firmware codec headers stay out of desktop translation units.
#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace runtime_fixture {
struct MotionStats { uint32_t frames, gaps, lost, restarts; int source; };
void start();
void enable_handshake();
int handshake_bits();
int handshake_cue_resets();
void record_handshake_event(const char* kind,int value);
void tick();
void finish();
void receive(const std::vector<uint8_t>& wire, bool serial_bridge);
std::vector<uint8_t> take_output();
void telemetry(const float angles[6], const float positions[6]);
size_t motion_count();
std::array<uint32_t,6> last_motion();
MotionStats motion_stats();
bool field_on();
void check_no_arm();
void save_trace(const std::string& path, const std::string& scenario, const std::string& error);
}
