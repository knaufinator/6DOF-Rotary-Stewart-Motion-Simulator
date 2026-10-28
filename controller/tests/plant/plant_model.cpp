#include "plant_model.h"
#include <plant_sdk.h>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <limits>
#include <stdexcept>

plant::Mcpwm MCPWM0;
plant::Gpio GPIO;
McpwmSignals mcpwm_periph_signals;
namespace plant {
std::array<Axis,6> axes;
std::vector<Event> events;
uint64_t cycle=0;
namespace {
// Declared ideal peripheral contract, independent of engine count/history:
// 240 cycles/us; 960-cycle period; shadow latch at phase 955 after TEP;
// compare edges at 1/3 us after that latch. This is NOT a measured MCPWM model.
constexpr uint64_t period=960,phase=955,never=std::numeric_limits<uint64_t>::max();
uint64_t next_irq=never,next_latch=never,next_rise=never,next_fall=never,stop_at=never;
bool running=false,in_isr=false,masked=false,pending_irq=false,inhibited=false;
bool gpio[64]{};
uint32_t shadow[3]{},latched[3]{};
int operators=0,comparators=0,generators=0;
int comparator_value[6]{},comparator_op[6]{},generator_op[6]{},generator_pin[6]{};
int high_comparator[6]{},low_comparator[6]{};
void (*irq)(void*)=nullptr;
void* irq_arg=nullptr;
int id(void* p,int limit) {int i=int(reinterpret_cast<uintptr_t>(p))-1;require(i>=0&&i<limit,"unknown SDK handle");return i;}
void* handle(int i){return reinterpret_cast<void*>(uintptr_t(i+1));}
int dir_pin(int axis){return axis<3?20+axis:33+axis-3;}
bool direction(int axis){return gpio[dir_pin(axis)] != axes[axis].wiring_inverted;}
void set_pin(int pin,bool high) {
    require(pin>=0&&pin<64,"GPIO outside model");
    if(gpio[pin]==high)return;
    for(int i=0;i<6;++i)if(pin==dir_pin(i)) {
        require(!axes[i].step_high,"DIR changed during modeled STEP high");
        axes[i].last_dir=cycle;axes[i].dir_changed=true;note("dir",i,high);
    }
    gpio[pin]=high;
}
void edge(int i,bool high) {
    auto& a=axes[i];if(a.step_high==high)return;
    a.step_high=high;
    if(!high){++a.falling_edges;require(cycle-a.last_rise==480,"modeled pulse width differs from 2 us");return;}
    require(!a.dir_changed || cycle-a.last_dir>=1920,"modeled DIR setup below 8 us");
    a.last_rise=cycle;++a.rising_edges;
    // Decode the pin with the independently specified nominal interface
    // polarity; a miswired drive can then move opposite to that signal.
    const int sign=(gpio[dir_pin(i)] != ((i%2)==1))?1:-1;
    a.pin_steps+=sign;note("step_rise",i,sign);
    if(inhibited){note("inhibited_edge",i);return;}
    ++a.drive_steps;
    if(a.stalled || (a.miss_every && a.drive_steps%a.miss_every==0)) {note("lost_travel",i);return;}
    const int64_t next=a.physical+(direction(i)?1:-1);
    if(next<a.minimum || next>a.maximum){a.alarm=true;note("physical_limit",i,next);return;}
    a.physical=next;
}
void dispatch_irq() {
    if(!irq || in_isr || masked || !(MCPWM0.int_ena.val&(1u<<6)))return;
    pending_irq=false;in_isr=true;irq(irq_arg);in_isr=false;
}
void until(uint64_t target) {
    require(target>=cycle,"model time reversed");
    while(running) {
        uint64_t at=std::min({next_irq,next_latch,next_rise,next_fall,stop_at});
        if(at>target)break;
        cycle=at;
        if(at==next_fall){for(int i=0;i<generators;++i)edge(i,false);next_fall=never;}
        if(at==stop_at){running=false;next_irq=next_latch=next_rise=next_fall=stop_at=never;break;}
        if(at==next_latch){for(int op=0;op<3;++op)latched[op]=shadow[op];
            next_rise=at+240;next_fall=at+720;next_latch+=period;}
        if(at==next_rise){for(int i=0;i<generators;++i) {
                uint32_t bit=1u<<(i%2?8:6);
                if(!(latched[generator_op[i]]&bit))edge(i,true);
            }next_rise=never;}
        if(at==next_irq){next_irq+=period;pending_irq=true;dispatch_irq();}
        // ISR reads advance the same peripheral clock; never move it backwards.
        if(cycle>target)return;
    }
    cycle=target;
}
}
void require(bool value,const std::string& reason){if(!value)throw std::runtime_error(reason);}
void note(const std::string& kind,int axis,int64_t value){events.push_back({cycle,kind,axis,value});}
void initialize(uint64_t epoch) {
    cycle=epoch;axes={};events.clear();
    for(int i=0;i<6;++i){axes[i].wiring_inverted=(i%2)==1;axes[i].last_dir=epoch;}
}
void advance(uint64_t cycles){until(cycle+cycles);}
void run_us(uint64_t us){advance(us*240);}
void mask_irq(bool value){masked=value;note(value?"irq_mask":"irq_unmask");if(!value&&pending_irq)dispatch_irq();}
void independent_inhibit(bool value){inhibited=value;note("independent_inhibit",-1,value);}
bool hall_closed(int i){return axes[i].hall_fault==1 || (axes[i].hall_fault!=2 && axes[i].physical<axes[i].hall_at);}
bool all_step_low(){for(const auto& a:axes)if(a.step_high)return false;return true;}
bool timer_running(){return running;}
void register_write(RegisterKind kind,int index,uint32_t value) {
    if(kind!=GATE){for(int bit=0;bit<32;++bit)if(value&(1u<<bit))set_pin(index*32+bit,kind==GPIO_SET);return;}
    require(index>=0&&index<3,"unknown operator register");
    require((value&~uint32_t(3|(3u<<6)|(3u<<8)))==0,"unmodeled gate bits");
    require((value&3)==1 || (value&3)==0,"unmodeled latch mode");
    shadow[index]=value;
    if((value&3)==0){latched[index]=value;for(int i=2*index;i<2*index+2;++i)if(value&(1u<<(i%2?8:6)))edge(i,false);}
}
void save(const std::string& path,const std::string& scenario,const std::string& error,
          const std::array<int32_t,6>& counted,int fault_code,bool stop_timing_fault) {
    std::ofstream f(path);require(bool(f),"cannot write plant trace");
    f<<"{\"scenario\":\""<<scenario<<"\",\"passed\":"<<(error.empty()?"true":"false")
     <<",\"error\":"<<std::quoted(error)<<",\"execution_lane\":\"isolated-component\",\"production_main_linked\":false"
     <<",\"hardware_timing_qualified\":false,\"cycle_read_cost\":16"
     <<",\"model\":\"ideal-MCPWM-shadow-latch-and-discrete-drive\",\"virtual_cycles\":"<<cycle
     <<",\"cycles_per_us\":240,\"period_cycles\":960,\"latch_phase_cycles\":955,\"fault_code\":"<<fault_code
     <<",\"engine_stop_timing_fault\":"<<(stop_timing_fault?"true":"false")
     <<",\"axes\":[";
    for(int i=0;i<6;++i){if(i)f<<',';const auto& a=axes[i];f<<"{\"axis\":"<<i<<",\"counted\":"<<counted[i]
        <<",\"pin_steps\":"<<a.pin_steps<<",\"rising_edges\":"<<a.rising_edges<<",\"falling_edges\":"<<a.falling_edges
        <<",\"drive_edges\":"<<a.drive_steps<<",\"physical\":"<<a.physical<<",\"alarm\":"<<(a.alarm?"true":"false")<<'}';}
    f<<"],\"events\":[";for(size_t i=0;i<events.size();++i){if(i)f<<',';const auto& e=events[i];
        f<<"{\"cycle\":"<<e.cycle<<",\"kind\":\""<<e.kind<<"\",\"axis\":"<<e.axis<<",\"value\":"<<e.value<<'}';}
    f<<"]}\n";
}
} // namespace plant

int gpio_config(const gpio_config_t* c){plant::require(c->mode==GPIO_MODE_OUTPUT,"unexpected GPIO mode");return ESP_OK;}
int gpio_set_level(gpio_num_t p,int v){plant::set_pin(p,v!=0);return ESP_OK;}
int mcpwm_new_timer(const mcpwm_timer_config_t* c,void** out){plant::require(c->group_id==0&&c->resolution_hz==1000000&&c->period_ticks==4&&c->count_mode==MCPWM_TIMER_COUNT_MODE_UP,"unmodeled timer config");*out=plant::handle(0);return ESP_OK;}
int mcpwm_new_operator(const mcpwm_operator_config_t* c,void** out){plant::require(c->group_id==0&&plant::operators<3,"operator config");*out=plant::handle(plant::operators++);return ESP_OK;}
int mcpwm_operator_connect_timer(void* op,void* timer){plant::id(op,3);plant::id(timer,1);return ESP_OK;}
int mcpwm_new_comparator(void* op,const mcpwm_comparator_config_t*,void** out){int i=plant::comparators++;plant::require(i<6,"comparator overflow");plant::comparator_op[i]=plant::id(op,3);*out=plant::handle(i);return ESP_OK;}
int mcpwm_comparator_set_compare_value(void* h,uint32_t v){plant::comparator_value[plant::id(h,6)]=int(v);return ESP_OK;}
int mcpwm_new_generator(void* op,const mcpwm_generator_config_t* c,void** out){int i=plant::generators++;plant::require(i<6&&c->gen_gpio_num==10+i,"generator wiring");plant::generator_op[i]=plant::id(op,3);plant::generator_pin[i]=c->gen_gpio_num;*out=plant::handle(i);return ESP_OK;}
int mcpwm_generator_set_action_on_compare_event(void* g,mcpwm_gen_compare_event_action_t action){int i=plant::id(g,6),c=plant::id(action.comparator,6);plant::require(action.direction==MCPWM_TIMER_DIRECTION_UP&&plant::comparator_op[c]==plant::generator_op[i],"compare routing");if(action.action==MCPWM_GEN_ACTION_HIGH){plant::require(plant::comparator_value[c]==1,"unmodeled rising compare");plant::high_comparator[i]=c;}else{plant::require(action.action==MCPWM_GEN_ACTION_LOW&&plant::comparator_value[c]==3,"unmodeled falling compare");plant::low_comparator[i]=c;}return ESP_OK;}
int mcpwm_timer_enable(void*){return ESP_OK;}
int mcpwm_timer_disable(void*){plant::running=false;return ESP_OK;}
int mcpwm_timer_start_stop(void*,int command){
    if(command==MCPWM_TIMER_START_NO_STOP){plant::running=true;plant::next_irq=plant::cycle+72;plant::next_latch=plant::cycle+955;plant::next_rise=plant::next_fall=plant::stop_at=plant::never;plant::note("timer_start");}
    else {plant::require(command==MCPWM_TIMER_STOP_FULL,"unmodeled timer command");plant::stop_at=plant::next_irq-72;if(plant::stop_at<plant::cycle)plant::stop_at+=960;plant::note("timer_stop_request");}
    return ESP_OK;
}
int mcpwm_timer_set_period(void*,uint32_t p){plant::require(p==4,"unmodeled period");return ESP_OK;}
int mcpwm_del_generator(void*){return ESP_OK;} int mcpwm_del_comparator(void*){return ESP_OK;}
int mcpwm_del_operator(void*){return ESP_OK;} int mcpwm_del_timer(void*){return ESP_OK;}
int esp_intr_alloc(int,int,void(*callback)(void*),void* arg,void** out){plant::irq=callback;plant::irq_arg=arg;*out=plant::handle(0);return ESP_OK;}
int esp_intr_free(void*){plant::irq=nullptr;return ESP_OK;}
void periph_module_reset(int module){plant::require(module==PERIPH_PWM0_MODULE,"peripheral reset");MCPWM0.int_ena.val=0;for(int i=0;i<3;++i)plant::shadow[i]=plant::latched[i]=(1u<<6)|(1u<<8);}
uint32_t esp_cpu_get_cycle_count(){plant::advance(16);return uint32_t(plant::cycle);}
void esp_rom_delay_us(uint32_t us){plant::run_us(us);}
