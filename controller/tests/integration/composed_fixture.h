#pragma once
// Test-only facade. Keep pulse SDK/register types out of desktop socket code.
#include <array>
#include <cstdint>
#include <string>

namespace composed_fixture {
struct Snapshot {
    int delivered,source_drops,decoded,rejected,target_writes;
    int alarm_callbacks,io_callbacks,stop_calls,discard_calls;
    uint8_t alarm_mask;
    uint64_t step_edges;
    bool blocked,timer_running;
    std::array<float,6> last_raw;
};
// Fresh-process setup only. Backend initialized, never started or armed.
void start(bool serial_carrier);
void release_software_pause_for_test();
Snapshot snapshot();
void assert_locked();
void save_trace(const std::string& path,const std::string& scenario,const std::string& error);
}
