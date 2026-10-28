#pragma once
#include <array>
#include <cstdint>
#include <string>
#include <vector>
namespace plant {
struct Event {uint64_t cycle;std::string kind;int axis;int64_t value;};
struct Axis {
    int64_t physical=0,pin_steps=0,drive_steps=0;
    uint64_t rising_edges=0,falling_edges=0,last_rise=0,last_dir=0;
    bool step_high=false,stalled=false,wiring_inverted=false,alarm=false,dir_changed=false;
    unsigned miss_every=0;
    int64_t hall_at=8,minimum=-10000,maximum=10000;
    int hall_fault=0; // 0=position-derived NC; 1=stuck closed; 2=stuck open
};
extern std::array<Axis,6> axes;
extern std::vector<Event> events;
extern uint64_t cycle;
void require(bool,const std::string&);
void initialize(uint64_t epoch=0);
void advance(uint64_t cycles);
void run_us(uint64_t us);
void mask_irq(bool);
void independent_inhibit(bool);
bool hall_closed(int);
bool all_step_low();
bool timer_running();
void note(const std::string&,int axis=-1,int64_t value=0);
void save(const std::string& path,const std::string& scenario,const std::string& error,
          const std::array<int32_t,6>& counted,int fault_code,bool stop_timing_fault);
}
