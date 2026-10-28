// Component integration only: actual engine/guards, declared ideal peripheral
// and discrete plant. This executable never links the production main task.
#include "plant_model.h"
#include "SharedMcpwmStepEngine.h"
#include "R14SeekInterlock.h"
#include "R14MotionAdmission.h"
#include "R14SixAxisTrajectory.h"
#include <climits>
#include <cstdio>
#include <cstdlib>

namespace {
using plant::require;
auto& engine=SharedMcpwmStepEngine::instance();
int fault_code=0;
void configure() {
    SharedMcpwmStepEngine::MotorConfig cfg[6];
    for(int i=0;i<6;++i){cfg[i].stepPin=10+i;cfg[i].dirPin=i<3?20+i:33+i-3;
        cfg[i].invertDir=(i%2)==1;cfg[i].softLimitMin=-10000;cfg[i].softLimitMax=10000;}
    require(engine.init(cfg),"engine initialization failed");
}
std::array<int32_t,6> counts(){std::array<int32_t,6> c{};for(int i=0;i<6;++i)c[i]=engine.getPosition(i);return c;}
uint64_t edges(){uint64_t n=0;for(const auto& a:plant::axes)n+=a.rising_edges;return n;}
void agree(bool travel=true) {
    require(plant::all_step_low(),"STEP remained high after quiescence");
    for(int i=0;i<6;++i) {
        const auto& a=plant::axes[i];
        require(engine.getPosition(i)==a.pin_steps,"independent pin count disagrees with engine");
        require(engine.getTotalSteps(i)==a.rising_edges,"unsigned edge count disagrees with engine");
        require(a.rising_edges==a.falling_edges,"unpaired modeled STEP edges");
        if(travel)require(a.physical==a.pin_steps,"physical travel differs without an injected fault");
    }
}
void stop_clean() {
    engine.stop();require(!engine.isRunning()&&!plant::timer_running(),"timer did not stop");
    require(!engine.stopTimingFault(),"stop failed to drain within its software budget");
}
void vectors() {
    require(engine.start(),"engine start failed");
    const int32_t targets[6]={12,-24,36,-48,60,-72};
    for(int i=0;i<6;++i)require(engine.setTarget(i,targets[i]),"vector target rejected");
    plant::run_us(500);stop_clean();agree();
    for(int i=0;i<6;++i)require(plant::axes[i].physical==targets[i],"fixed vector target not reached");
    uint64_t before=edges();engine.resume();plant::run_us(12);
    require(edges()==before,"resume quiet periods emitted a step");
    for(int i=0;i<6;++i)require(engine.setTarget(i,0),"return target rejected");
    plant::run_us(500);stop_clean();agree();
    for(int i=0;i<6;++i)require(plant::axes[i].physical==0 &&
        plant::axes[i].rising_edges==2u*uint64_t(std::abs(targets[i])),"signed round-trip lost travel");
}
r14::SeekInterlock::Inputs inputs(uint64_t now) {
    r14::SeekInterlock::Inputs in{};in.sampled_us=now;
    in.estop_closed=in.field_good=in.bottom_verified=in.calibration_valid=true;
    for(int i=0;i<6;++i){in.hall_closed[i]=plant::hall_closed(i);in.drive_ready[i]=true;in.drive_alarm[i]=plant::axes[i].alarm;}
    return in;
}
void seek(const std::string& name) {
    using Guard=r14::SeekInterlock;Guard guard(50,2000);
    for(int i=0;i<6;++i){plant::axes[i].hall_at=6+2*i;
        require(guard.configure_axis(i,{1,3,20,name=="deadline"?100u:1000u}),"seek configuration failed");}
    if(name=="stall")plant::axes[0].stalled=true;
    if(name=="missed_steps"){plant::axes[0].miss_every=2;plant::axes[0].hall_at=16;}
    if(name=="hall_closed")plant::axes[0].hall_fault=1;
    if(name=="hall_open_boot")plant::axes[0].hall_fault=2;
    if(name=="hall_early")plant::axes[0].hall_at=1;
    if(name=="hall_late")plant::axes[0].hall_at=100;
    if(name=="drive_direction")plant::axes[0].wiring_inverted=true;
    if(name=="physical_limit")plant::axes[0].maximum=4;
    if(name=="deadline")for(auto& a:plant::axes)a.hall_at=5;
    require(guard.observe(inputs(0),0),"initial sample rejected");
    bool armed=guard.request_arm(0);
    if(name=="hall_open_boot") {
        require(!armed && guard.state()==Guard::DISARMED,"open boot Hall admitted seek");
        plant::run_us(200);require(edges()==0&&!plant::timer_running(),"rejected boot seek started outputs");return;
    }
    require(armed,"explicit test seek request rejected");
    for(int i=0;i<6;++i)require(guard.begin_seek(i,0),"seek start rejected");
    require(engine.start(),"engine start failed");
    bool stopped=false;
    for(int round=0;round<60;++round) {
        uint64_t now=plant::cycle/240;auto in=inputs(now);
        if(round>=4) {
            if(name=="estop")in.estop_closed=false;
            if(name=="field_loss")in.field_good=false;
            if(name=="stale")in.sampled_us=0;
            if(name.rfind("alarm_",0)==0)in.drive_alarm[std::stoi(name.substr(6))]=true;
        }
        bool healthy=guard.observe(in,now);
        if(!healthy || guard.state()==Guard::ALL_EDGES_CAPTURED){stopped=true;break;}
        for(int i=0;i<6;++i)if(guard.axis_state(i)==Guard::SEEKING) {
            int sign=(name=="wrong_direction"&&i==0)?-1:1;
            if(!guard.reserve_seek_pulse(i,sign,now)){stopped=true;break;}
            require(engine.setTarget(i,engine.getTarget(i)+sign),"seek target rejected");
        }
        if(stopped)break;
        plant::run_us(20); // settle each reserved edge, then sample independent Hall
    }
    require(stopped,"seek loop exceeded its deterministic budget");
    fault_code=int(guard.fault());
    if(name=="seek") {
        require(guard.state()==Guard::ALL_EDGES_CAPTURED,"healthy plant did not capture all Hall edges");
        for(int i=0;i<6;++i)require(plant::axes[i].physical==6+2*i,"Hall not derived from physical travel");
    } else {
        auto expected=Guard::SEEK_PULSE_LIMIT;
        if(name=="hall_early")expected=Guard::HALL_TOO_EARLY;
        if(name=="wrong_direction")expected=Guard::WRONG_DIRECTION;
        if(name=="estop"||name=="field_loss")expected=Guard::INPUT_LOST;
        if(name=="stale")expected=Guard::CLOCK_OR_INPUT_STALE;
        if(name=="physical_limit"||name.rfind("alarm_",0)==0)expected=Guard::DRIVE_FAULT;
        if(name=="deadline")expected=Guard::SEEK_TIME_LIMIT;
        require(guard.state()==Guard::GLOBAL_FAULT && guard.fault()==expected,"wrong bounded-seek fault reason");
        if(name.rfind("alarm_",0)==0)require(guard.fault_axis()==std::stoi(name.substr(6)),"alarm axis identity lost");
        plant::independent_inhibit(true); // explicit test gate, not main.cpp hardware wiring
    }
    std::array<int64_t,6> physical_before{};for(int i=0;i<6;++i)physical_before[i]=plant::axes[i].physical;
    stop_clean();agree(name=="seek"||name=="wrong_direction"||name=="deadline");
    if(name!="seek") {
        for(int i=0;i<6;++i)require(plant::axes[i].physical==physical_before[i],"modeled inhibit allowed post-fault travel");
        require(!guard.request_arm(plant::cycle/240),"fault recovery implicitly re-armed");
    }
    require(!guard.normal_motion_permitted(),"seek edge capture granted normal motion");
    if(name=="stall")require(plant::axes[0].physical==0 && engine.getPosition(0)==20,"stall was hidden by commanded position");
    if(name=="missed_steps")require(plant::axes[0].physical==10 && engine.getPosition(0)==20,"missed steps were not independent");
    if(name=="drive_direction")require(plant::axes[0].physical==-20,"wrong drive direction not modeled independently");
}
void trajectory() {
    r14::SixAxisTrajectory::AxisLimits limits[6];int32_t start[6]{},target[6]{12,-24,36,-48,60,-72},out[6]{};
    for(auto& l:limits)l={-1000,1000,500,2000,10000};
    r14::SixAxisTrajectory golden;
    require(golden.begin(start,target,limits,0,1000000,10000000),"golden trajectory rejected");
    const int expected[3][6]={{1,-2,3,-3,4,-5},{6,-12,18,-24,30,-36},{11,-22,33,-45,56,-67}};
    for(int q=1;q<=3;++q){require(golden.sample(golden.duration_us()*q/4,out),"golden sample failed");
        for(int i=0;i<6;++i)require(out[i]==expected[q-1][i],"independent quarter-point vector changed");}
    r14::SixAxisTrajectory path;require(path.begin(start,target,limits,0,2000,10000000),"trajectory rejected");
    require(engine.start(),"engine start failed");
    while(path.state()==r14::SixAxisTrajectory::RUNNING) {
        require(path.sample(plant::cycle/240,out),"trajectory clock/update fault");
        for(int i=0;i<6;++i)require(engine.setTarget(i,out[i]),"trajectory target rejected");
        plant::run_us(1000);
    }
    plant::run_us(40);stop_clean();agree();
    for(int i=0;i<6;++i)require(plant::axes[i].physical==target[i],"trajectory did not reach independent plant");
}
void scenario(const std::string& name) {
    plant::initialize(name=="wrap"?uint64_t(UINT32_MAX)-2000:0);configure();
    if(name=="vectors"||name=="wrap")vectors();
    else if(name=="trajectory")trajectory();
    else if(name=="locked") {
        r14::MotionAdmission<false> admission(50,2000,2000);
        for(int i=0;i<6;++i)require(!admission.configure_axis(i,{1,3,20,1000}),"staging configuration accepted");
        require(!admission.submit_inputs(inputs(0),0)&&!admission.request_operator_arm(0)&&!admission.request_operator_home(0),"staging requests accepted");
        if(admission.normal_motion_permitted()) {require(engine.start(),"engine start failed");engine.setTarget(0,100);}
        plant::run_us(100000);require(edges()==0&&!engine.isRunning()&&!plant::timer_running(),"locked profile emitted STEP");
    } else if(name=="limits") {
        require(!engine.setTarget(0,10001)&&!engine.setTarget(1,-10001)&&!engine.setTarget(2,INT32_MAX),"engine soft limit bypassed");
        require(engine.start(),"engine start failed");plant::run_us(100);stop_clean();agree();require(edges()==0,"rejected targets emitted STEP");
    } else if(name=="stop_phases") {
        require(engine.start(),"engine start failed");
        for(uint64_t offset:{0u,71u,72u,240u,480u,720u,954u,955u}) {
            plant::independent_inhibit(false);
            for(int i=0;i<6;++i)engine.setTarget(i,engine.getPosition(i)+1000);
            plant::run_us(40);plant::advance(offset);
            std::array<int64_t,6> physical_before{};for(int i=0;i<6;++i)physical_before[i]=plant::axes[i].physical;
            plant::note("asynchronous_fault_phase",-1,offset);plant::independent_inhibit(true);
            stop_clean();agree(false);
            for(int i=0;i<6;++i)require(plant::axes[i].physical==physical_before[i],"modeled inhibit allowed in-flight travel");
            uint64_t before=edges();engine.resume();plant::run_us(12);
            require(edges()==before,"phase-specific resume skipped quiet guard");
        }
        stop_clean();
    } else if(name=="stop_resume"||name=="irq_gap"||name=="stop_timeout") {
        require(engine.start(),"engine start failed");for(int i=0;i<6;++i)engine.setTarget(i,1000);
        plant::run_us(100);
        if(name=="irq_gap"){plant::mask_irq(true);plant::run_us(8);plant::mask_irq(false);plant::run_us(100);}
        if(name=="stop_timeout") {
            plant::independent_inhibit(true);plant::mask_irq(true);engine.stop();
            require(engine.stopTimingFault()&&!engine.isRunning()&&plant::all_step_low(),"masked ISR did not latch stop timing fault");
            uint64_t before=edges();engine.resume();plant::run_us(100);
            require(!engine.isRunning()&&edges()==before,"timing-fault resume was accepted");return;
        }
        stop_clean();agree();
        if(name=="irq_gap")require(engine.isrMissed()>0,"injected interrupt gap did not exercise latch accounting");
        uint64_t before=edges();plant::run_us(100);engine.resume();plant::run_us(100);
        require(edges()==before,"stop/resume replayed a discarded target");
        for(int i=0;i<6;++i)engine.setTarget(i,engine.getPosition(i)+5);
        plant::run_us(100);stop_clean();agree();
    } else seek(name);
}
}
int main(int argc,char** argv) {
    if(argc!=3)return 2;std::string error;
    try{scenario(argv[1]);}catch(const std::exception& e){error=e.what();}
    try{plant::save(argv[2],argv[1],error,counts(),fault_code,engine.stopTimingFault());}catch(const std::exception& e){error=e.what();}
    if(!error.empty()){std::fprintf(stderr,"FAIL %s: %s\n",argv[1],error.c_str());return 1;}
    std::printf("PASS %s: actual engine, independent modeled edges/travel; cycles=%llu\n",argv[1],static_cast<unsigned long long>(plant::cycle));return 0;
}
