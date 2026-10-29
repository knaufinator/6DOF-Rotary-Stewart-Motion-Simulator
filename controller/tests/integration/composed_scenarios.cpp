// Shared production safety/decoder + real board monitor/COBS + unchanged engine.
// Main tasks, NVS and real peripheral timing are NOT linked. The standalone
// runner excludes desktop networking; the test facade also serves its app lane.
#include "runtime_fixture.h"
#include "composed_fixture.h"
#include "../plant/plant_model.h"
#include "SharedMcpwmStepEngine.h"
#include "ControllerMotionPayload.h"
#include "R14RuntimeSafety.h"
#include "R14MotionAdmission.h"
#include "EStopSafety.h"
#include "CobsTransport.h"
#include "AxisScaling.h"
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iterator>

namespace {
using plant::require;
auto& engine=SharedMcpwmStepEngine::instance();
r14::MotionAdmission<false> admission(1,1,1);
r14::DriveAlarmLatch alarms;
bool paused=true,live_estop=false,debounced_estop=false;
bool containment_override=false;
int active_source=COBS_SRC_SERIAL;
int delivered=0,source_drops=0,decoded=0,rejected=0,target_writes=0;
int alarm_callbacks=0,io_callbacks=0,stop_calls=0,discard_calls=0;
std::array<float,6> last_raw{};
std::vector<std::string> stop_order;

bool blocked() {
    if(r14::admission_blocks_motion(admission,alarms))return true;
    return estop_motion_blocked(paused,live_estop,debounced_estop);
}
bool target(int axis,int32_t value) {
    return controller::guarded_motor_target(blocked(),[=] {
        ++target_writes;return engine.setTarget(axis,value);
    });
}
struct StopOps {
    void pause() const {
        require(admission.faulted(),"stop preceded admission invalidation");
        paused=true;stop_order.push_back("pause");plant::note("pause");
    }
    void bench_stop(const char*) const {} // R13 bench mode is OFF
    void inhibit() const {
        stop_order.push_back("inhibit");plant::independent_inhibit(true);
    }
    bool initialized() const {return engine.isInitialized();}
    void stop() const {++stop_calls;stop_order.push_back("stop");engine.stop();}
    void discard_targets() const {
        ++discard_calls;stop_order.push_back("discard");engine.emergencyStop();
    }
};
void alarm(int axis) {
    ++alarm_callbacks;r14::report_alarm_and_stop(admission,alarms,axis,StopOps{});
}
void io_fault() {
    ++io_callbacks;r14::report_reconnect_and_stop(admission,StopOps{});
}
void on_data(const uint8_t* payload,int len,int source) {
    ++delivered;
    // Test adapter for main's persisted input selection, not NVS execution.
    if(source!=active_source){++source_drops;return;}
    float raw[6];
    if(!controller::decode_motion_payload(payload,len,raw))return;
    ++decoded;for(int i=0;i<6;++i)last_raw[i]=raw[i];
    // The live application checks this before IK, and again per target.
    if(blocked()){++rejected;return;}
    StewartConfig config;initDefaultStewartConfig(&config);
    config.virtual_gear=2.0f;computeStepsPerDegree(&config);
    AxisScaleConfig scales;computeAxisScalesFromGeometry(&scales,&config,0.9f);
    float mapped[6],angles[6];mapRawToPosition(raw,&scales,4094,mapped);
    calculateAllServoAngles(mapped,&config,angles);
    for(int i=0;i<6;++i) {
        require(std::isfinite(angles[i]),"nonfinite test IK result");
        target(i,int32_t(angles[i]*IK_RAD_TO_DEG*config.steps_per_degree));
    }
}
std::vector<uint8_t> encode(const std::vector<uint8_t>& frame) {
    std::vector<uint8_t> wire(COBS_MAX_ENC_SIZE(int(frame.size()))+1);
    int n=cobs_encode(frame.data(),int(frame.size()),wire.data());
    wire[n++]=0;wire.resize(n);return wire;
}
void command(const std::string& cmd,bool serial=true) {
    std::vector<uint8_t> frame{COBS_CH_CMD};frame.insert(frame.end(),cmd.begin(),cmd.end());
    runtime_fixture::receive(encode(frame),serial);
}
std::string replies() {
    auto wire=runtime_fixture::take_output();std::string text;size_t begin=0;
    for(size_t i=0;i<wire.size();++i)if(wire[i]==0) {
        uint8_t frame[COBS_MAX_FRAME];int n=cobs_decode(wire.data()+begin,int(i-begin),frame);
        require(n>0,"invalid response frame");
        if(frame[0]==COBS_CH_RESP)text.append(reinterpret_cast<char*>(frame+1),n-1);
        begin=i+1;
    }
    return text;
}
void status(const std::string& expected) {
    runtime_fixture::take_output();command("R14:STATUS");
    require(replies().find(expected)!=std::string::npos,"status missing "+expected);
}
void motion(bool serial=true,int sequence=-1) {
    // Fixed independent byte vector: low-18-bit masking + asymmetric axis order.
    std::vector<uint8_t> frame{sequence<0?uint8_t(COBS_CH_DATA18):uint8_t(COBS_CH_DATA18_SEQ)};
    if(sequence>=0){frame.push_back(uint8_t(sequence));frame.push_back(uint8_t(sequence>>8));}
    const uint8_t data[]={0,0,0, 1,0,0, 0xff,0x0f,0, 0,0,1, 0xff,0xff,3, 0x23,0x01,0xfc};
    frame.insert(frame.end(),std::begin(data),std::end(data));
    runtime_fixture::receive(encode(frame),serial);
}
uint64_t edges() {uint64_t n=0;for(const auto& a:plant::axes)n+=a.rising_edges;return n;}
void no_motion() {
    require(target_writes==0,"locked target reached engine");
    require(blocked()&&!admission.normal_motion_permitted(),"R14 gate opened");
    require(!admission.request_operator_arm(100)&&!admission.request_operator_home(100),"local request admitted");
    require(!admission.home_request_pending(),"home request survived lock");
    require(!engine.isRunning()&&!plant::timer_running()&&plant::all_step_low(),"stopped outputs not quiet");
    for(int i=0;i<6;++i)require(engine.getTarget(i)==engine.getPosition(i),"stale target survived stop");
    runtime_fixture::check_no_arm();
}
void configure_engine() {
    SharedMcpwmStepEngine::MotorConfig config[6];
    for(int i=0;i<6;++i){config[i].stepPin=10+i;config[i].dirPin=i<3?20+i:33+i-3;
        config[i].invertDir=(i%2)==1;config[i].softLimitMin=-10000;config[i].softLimitMax=10000;}
    require(engine.init(config),"engine initialization failed");
}
void configure(const std::string& name) {
    plant::initialize();
    bool early=name=="startup_alarm"||name=="startup_bus_failure";
    if(!early)configure_engine();
    runtime_fixture::set_fault_handlers(alarm,io_fault);
    if(name=="startup_alarm")runtime_fixture::inject_alarm_mask(1);
    if(name=="startup_bus_failure")runtime_fixture::inject_bus_failure(true);
    runtime_fixture::start();cobs_set_data_handler(on_data);
}
void startup_fault(const std::string& name) {
    require(admission.faulted()&&paused,"startup fault did not close admission");
    require(stop_calls==0&&discard_calls==0,"uninitialized engine received stop calls");
    require(stop_order==std::vector<std::string>({"pause","inhibit"}),"startup stop omitted inhibit");
    require(alarm_callbacks==(name=="startup_alarm"?1:0)&&io_callbacks==(name=="startup_bus_failure"?1:0),"startup callback missing");
    configure_engine(); // Later GPIO/backend initialization cannot revive permission.
    paused=false;motion();no_motion();require(edges()==0,"startup fault emitted STEP");
}
void locked_stream(bool serial) {
    active_source=serial?COBS_SRC_SERIAL:COBS_SRC_ETHERNET;
    // Clear only the E-stop software pause in this adversarial test. The
    // production R14 type must remain sufficient even if that layer releases.
    paused=false;
    for(int i=0;i<6;++i)require(!target(i,12*(i+1)),"direct motor guard accepted locked target");
    motion(serial);
    const float expected[]={0,1,4095,65536,262143,291};
    require(decoded==1&&rejected==1,"valid motion did not reach locked admission");
    for(int i=0;i<6;++i)require(last_raw[i]==expected[i],"independent DATA18 decode vector differs");
    motion(!serial);require(source_drops==1,"inactive source not rejected by adapter");
    for(int i=0;i<64;++i)motion(serial,i);
    plant::run_us(500);no_motion();require(edges()==0,"locked stream emitted STEP");
}
void containment(const std::string& name) {
    // Fault-containment injection only: put the actual backend in a state that
    // production R14 boot cannot enter. Never a production arm/unlock path.
    containment_override=true;require(engine.start(),"containment timer start failed");
    for(int i=0;i<6;++i)require(engine.setTarget(i,5000),"preloaded target failed");
    plant::run_us(40);require(edges()>0,"containment injection did not emit edges");
    std::array<int64_t,6> before{};for(int i=0;i<6;++i)before[i]=plant::axes[i].physical;
    if(name=="reconnect")io_fault();
    else {
        bool bus=name.rfind("bus_alarm_",0)==0 || name=="bus_recovery";
        if(bus)runtime_fixture::inject_bus_failure(true);
        int axis=-1;
        if(name.rfind("alarm_",0)==0)axis=std::stoi(name.substr(6));
        if(name.rfind("bus_alarm_",0)==0)axis=std::stoi(name.substr(10));
        if(axis>=0)runtime_fixture::inject_alarm_mask(uint8_t(1u<<axis));
        runtime_fixture::tick();
        require(io_callbacks==(bus?1:0),"wrong I/O fault callback count");
        require(alarm_callbacks==(axis>=0?1:0),"wrong alarm callback count");
        if(axis>=0)require(alarms.axis_mask()==(1u<<axis)&&admission.alarm_mask()==(1u<<axis),"alarm identity lost");
        runtime_fixture::inject_alarm_mask(0);runtime_fixture::inject_bus_failure(false);
        runtime_fixture::tick();status("iox=1 adc=1 fresh=1");
        require(io_callbacks==(bus?1:0)&&alarm_callbacks==(axis>=0?1:0),"recovery repeated fault callback");
    }
    require(!engine.isRunning()&&!plant::timer_running(),"fault stop left timer running");
    require(stop_calls>0&&discard_calls==stop_calls,"stop/discard callback missing");
    require(!engine.stopTimingFault(),"fault stop did not drain");
    require(stop_order.size()==size_t(stop_calls*4),"incomplete fault stop sequence");
    for(size_t i=0;i<stop_order.size();i+=4)
        require(stop_order[i]=="pause"&&stop_order[i+1]=="inhibit"&&stop_order[i+2]=="stop"&&stop_order[i+3]=="discard","fault stop order changed");
    for(int i=0;i<6;++i)require(plant::axes[i].physical==before[i],"post-inhibit plant travel");
    uint64_t stopped=edges();paused=false;
    for(int i=0;i<8;++i)motion(true,i); // queued work and fresh frames cannot revive outputs
    command("R14:FIELD=1");runtime_fixture::tick();
    require(runtime_fixture::field_on(),"local diagnostic feed request failed");
    plant::run_us(500);no_motion();require(edges()==stopped,"fault recovery emitted STEP");
}
void protocol() {
    paused=false;
    for(int n=0;n<18;++n) {
        std::vector<uint8_t> frame(size_t(n)+1,1);frame[0]=COBS_CH_DATA18;
        runtime_fixture::receive(encode(frame),true);
    }
    require(delivered==0,"truncated motion reached callback");
    float raw[6]{};uint8_t data[18]{};
    require(!controller::decode_motion_payload(nullptr,18,raw)&&!controller::decode_motion_payload(data,17,raw),"decoder accepted invalid payload");
    for(int seq:{65535,0,0,2,1})motion(true,seq);
    auto s=runtime_fixture::motion_stats();
    // Current protocol tracks disorder and still delivers it; it is not an
    // anti-replay mechanism. The independent safety gate rejects every frame.
    require(s.frames==5&&s.gaps==1&&s.lost==1&&s.restarts==2,"sequence bookkeeping policy changed");
    require(decoded==5&&rejected==5,"reordered frames bypassed lock");
    no_motion();require(edges()==0,"protocol corpus emitted STEP");
}
void diagnostics() {
    paused=false;command("R14:FIELD=1",false);runtime_fixture::tick();
    require(replies().find("local console required")!=std::string::npos&&!runtime_fixture::field_on(),"remote feed request accepted");
    command("R14:FIELD=1");runtime_fixture::tick();require(runtime_fixture::field_on(),"local feed request failed");
    status("field_good_local=1");status("motion_qualified=0");
    for(const char* cmd:{"R14:ARM","R14:HOME"}) {
        command(cmd);require(replies().find("arm/home blocked")!=std::string::npos,"diagnostic command granted authority");
    }
    command("R14:FIELD=0");runtime_fixture::tick();status("field_good_local=0");
    motion();no_motion();require(edges()==0,"diagnostics emitted STEP");
}
void save(const std::string& path,const std::string& name,const std::string& error) {
    std::ofstream f(path);require(bool(f),"cannot write composed trace");
    f<<"{\"scenario\":"<<std::quoted(name)<<",\"passed\":"<<(error.empty()?"true":"false")
     <<",\"error\":"<<std::quoted(error)<<",\"execution_lane\":\"production-seam-composition\""
     <<",\"production_main_linked\":false,\"hardware_timing_qualified\":false"
     <<",\"containment_test_override\":"<<(containment_override?"true":"false")
     <<",\"clock_policy\":\"monitor ticks and engine cycles advanced separately; no latency assertion\""
     <<",\"delivered\":"<<delivered<<",\"source_drops\":"<<source_drops<<",\"decoded\":"<<decoded
     <<",\"rejected\":"<<rejected<<",\"target_writes\":"<<target_writes<<",\"step_edges\":"<<edges()
     <<",\"alarm_mask\":"<<int(alarms.axis_mask())<<",\"alarm_callbacks\":"<<alarm_callbacks
     <<",\"io_callbacks\":"<<io_callbacks<<",\"stop_calls\":"<<stop_calls<<",\"discard_calls\":"<<discard_calls
     <<",\"normal_motion_permitted\":"<<(admission.normal_motion_permitted()?"true":"false")<<",\"stop_order\":[";
    for(size_t i=0;i<stop_order.size();++i){if(i)f<<',';f<<std::quoted(stop_order[i]);}
    f<<"]}\n";
    runtime_fixture::save_trace(path+".board.json",name,error);
    std::array<int32_t,6> counts{};for(int i=0;i<6;++i)counts[i]=engine.getPosition(i);
    plant::save(path+".plant.json",name,error,counts,int(admission.seek_state()),engine.stopTimingFault());
}
}
namespace composed_fixture {
void start(bool serial_carrier) {
    configure("locked_serial");
    active_source=serial_carrier?COBS_SRC_SERIAL:COBS_SRC_ETHERNET;
}
void release_software_pause_for_test() {paused=false;}
Snapshot snapshot() {
    return {delivered,source_drops,decoded,rejected,target_writes,alarm_callbacks,
        io_callbacks,stop_calls,discard_calls,alarms.axis_mask(),edges(),blocked(),
        engine.isRunning()||plant::timer_running(),last_raw};
}
void assert_locked() {
    plant::run_us(500);no_motion();require(edges()==0,"desktop traffic emitted STEP");
}
void save_trace(const std::string& path,const std::string& name,const std::string& error) {
    save(path,name,error);
}
}
#ifndef R14_COMPOSED_NO_MAIN
int main(int argc,char** argv) {
    if(argc!=3)return 2;std::string name=argv[1],error;
    try {
        configure(name);
        if(name=="locked_serial"||name=="locked_ethernet")locked_stream(name=="locked_serial");
        else if(name=="startup_alarm"||name=="startup_bus_failure")startup_fault(name);
        else if(name=="protocol")protocol();
        else if(name=="diagnostics")diagnostics();
        else if(name=="reconnect"||name=="bus_recovery"||name.rfind("alarm_",0)==0||name.rfind("bus_alarm_",0)==0)containment(name);
        else require(false,"unknown composed scenario");
    }catch(const std::exception& e){error=e.what();}
    runtime_fixture::finish();
    try {save(argv[2],name,error);}catch(const std::exception& e){error=e.what();}
    if(!error.empty()){std::fprintf(stderr,"FAIL %s: %s\n",name.c_str(),error.c_str());return 1;}
    std::printf("PASS %s: shared safety/decoder, board monitor, COBS and actual engine\n",name.c_str());return 0;
}
#endif
