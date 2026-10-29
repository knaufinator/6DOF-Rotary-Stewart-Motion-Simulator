// Compile the real R14Hardware.cpp and CobsTransport.cpp against these adapters.
// No OS serial/network functions or production motor callbacks are linked.
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include "R14Hardware.h"
#include "CobsTransport.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "driver/usb_serial_jtag.h"
#include "runtime_fixture.h"

namespace sim {
struct Stop {};
struct Event { int64_t us; std::string kind; int address,reg,value; };
std::vector<Event> events;
std::vector<uint8_t> rx,tx;
size_t rx_pos=0;
std::atomic<int64_t> now{0};
std::mutex schedule;
std::condition_variable cv;
bool paused=false,release=false,shutdown=false;
std::thread worker;
std::vector<std::unique_ptr<std::recursive_mutex>> semaphores;
uint8_t mcp[0x16]={};
bool bank1=false,bus_failed=false,missing_adc=false;
bool arm_level=false;
int arm_edges=0,io_faults=0,alarm_bits=0;
int64_t conversion_started=0;
int voltage_raw=4000;
uint8_t alarm_gpio=0;
void (*alarm_observer)(int)=nullptr;
void (*io_fault_observer)()=nullptr;

void event(const std::string& kind,int address=0,int reg=0,int value=0) {
    events.push_back({now.load(),kind,address,reg,value});
}
void require(bool condition,const std::string& reason) {
    if (!condition) throw std::runtime_error(reason);
}
void wait_pause(std::unique_lock<std::mutex>& lock) {
    require(cv.wait_for(lock,std::chrono::seconds(3),[]{return paused && !release;}),
            "monitor did not reach a deterministic yield");
}
void tick() {
    std::unique_lock<std::mutex> lock(schedule);
    require(paused,"tick requires a paused monitor");
    release=true;cv.notify_all();wait_pause(lock);
}
void finish() {
    { std::lock_guard<std::mutex> lock(schedule);shutdown=true;cv.notify_all(); }
    if (worker.joinable()) worker.join();
}
int canonical(int reg) {
    if (!bank1) return reg < 0x16 ? reg : -1;
    static const int a[]={0,2,4,6,8,10,12,14,16,18,20};
    if (reg<=10) return a[reg];
    if (reg>=16 && reg<=26) return a[reg-16]+1;
    return -1;
}
uint8_t gpio_b() { return uint8_t((0x10 & mcp[1]) | (mcp[0x15] & ~mcp[1])); }
void observe_arm() {
    bool level=(gpio_b() & 0x80)!=0;
    if (level && !arm_level) { ++arm_edges;event("arm_edge"); }
    arm_level=level;
}
uint8_t mcp_read(int reg) {
    if (reg==0x12) {
        // NC Hall inputs closed; field-good follows modeled feed output.
        uint8_t external=(gpio_b() & 2) ? 0x40 : 0;
        return uint8_t((external & mcp[0]) | (mcp[0x14] & ~mcp[0]));
    }
    if (reg==0x13) return gpio_b();
    return mcp[reg];
}
void command(const std::string& text,int source=COBS_SRC_SERIAL) {
    std::vector<uint8_t> frame{COBS_CH_CMD};frame.insert(frame.end(),text.begin(),text.end());
    event("command",source,0,int(text.size()));
    if (source==COBS_SRC_SERIAL) {
        // Through the real stream decoder, including arbitrary partial reads.
        std::vector<uint8_t> enc(COBS_MAX_ENC_SIZE(int(frame.size()))+1);
        int n=cobs_encode(frame.data(),int(frame.size()),enc.data());enc[n++]=0;
        rx.insert(rx.end(),enc.begin(),enc.begin()+n);
        while (rx_pos<rx.size()) require(cobs_read_process(0)>0,"stream stalled");
    } else cobs_dispatch_frame(frame.data(),int(frame.size()),source);
}
std::string replies() {
    std::string text;size_t begin=0;
    for (size_t i=0;i<tx.size();++i) if (!tx[i]) {
        if (i>begin) {
            uint8_t out[COBS_MAX_FRAME+32];int n=cobs_decode(tx.data()+begin,int(i-begin),out);
            require(n>0,"invalid output COBS frame");
            if (out[0]==COBS_CH_RESP) text.append(reinterpret_cast<char*>(out+1),size_t(n-1));
        }
        begin=i+1;
    }
    return text;
}
void status_contains(const std::string& expected) {
    tx.clear();command("R14:STATUS");
    require(replies().find(expected)!=std::string::npos,"status missing: "+expected+"; "+replies());
}
void no_arm() {
    require(arm_edges==0,"unexpected physical arm-clock rising edge");
    require((mcp[0x14]&0x80)==0,"torque clear released");
    require((mcp[0x15]&0x80)==0,"arm clock asserted");
}
void startup(bool retained,bool bank) {
    mcp[0]=0xff;mcp[1]=0xff;
    if (retained) {mcp[0]=0x7f;mcp[1]=0x7d;mcp[0x14]=0x80;mcp[0x15]=0x82;}
    bank1=bank;mcp[0x0a]=mcp[0x0b]=bank ? 0x80 : 0;
    arm_level=(gpio_b()&0x80)!=0;
    cobs_transport_init(115200);
    cobs_set_cmd_handler([](const char* cmd,int src) {
        require(r14_hardware_command(cmd,src),"unexpected non-R14 command in test route");
    });
    r14_hardware_start();
    {std::unique_lock<std::mutex> lock(schedule);wait_pause(lock);}
    tick(); // real initialization + first 3 ms conversion wait + one actual poll
}
void scenario(const std::string& name) {
    if (name=="startup_bus_failure") bus_failed=true;
    if (name=="missing_adc") missing_adc=true;
    startup(name=="retained_boot" || name=="retained_bank1",name=="retained_bank1");
    if (name=="cold_boot" || name=="retained_boot" || name=="retained_bank1") {
        status_contains("iox=1 adc=1 fresh=1");
        status_contains("vcc5_mv=5000");
        status_contains("hall_nc=0x3f alarms=0x00");
        require(mcp[0]==0x7f && mcp[1]==0x7d,"wrong direction readback");
        // Every direction write must follow safe latches, including BANK recovery.
        bool latch_a=false,latch_b=false;
        for (const auto& e:events) if (e.kind=="write" && e.address==0x20) {
            if (e.reg==0x14 && e.value==0) latch_a=true;
            if (e.reg==0x15 && e.value==0) latch_b=true;
            if (e.reg==0 || e.reg==1) require(latch_a&&latch_b,"directions before clear latches");
        }
        command("R14:FIELD=1");tick();require((mcp[0x15]&2)!=0,"feed request not applied");
        status_contains("field_good_local=1");
        command("R14:FIELD=0");tick();require((mcp[0x15]&2)==0,"feed stayed on");
        require(io_faults==0,"unexpected boot fault");
    } else if (name=="bus_failure") {
        command("R14:FIELD=1");tick();
        bus_failed=true;alarm_gpio=0x20;command("R14:FIELD=0");tick();
        status_contains("iox=0 adc=0 fresh=0");
        require(io_faults==1 && alarm_bits==0x20,"fault or independent alarm not reported");
        tick();require(io_faults==1,"I/O fault storm");
        bus_failed=false;tick();status_contains("iox=1 adc=1 fresh=1");
        require(io_faults==1,"recovery unexpectedly reinitialized/cleared report");
    } else if (name=="startup_bus_failure" || name=="missing_adc") {
        status_contains("iox=0 adc=0 fresh=0");require(io_faults==1,"startup failure not reported");
        // Recovery cannot make an uninitialized BoardIo valid automatically.
        bus_failed=false;missing_adc=false;tick();status_contains("fresh=0");
    } else if (name=="remote_rejection") {
        for (int src:{COBS_SRC_ETHERNET,COBS_SRC_WIFI,COBS_SRC_BLE}) {
            tx.clear();command("R14:FIELD=1",src);tick();
            require(replies().find("local console required")!=std::string::npos,"source accepted");
            require((mcp[0x15]&2)==0,"remote command switched feed");
        }
        tx.clear();command("R14:ARM");require(replies().find("arm/home blocked")!=std::string::npos,"arm accepted");
    } else if (name=="alarm_during_bus_failure") {
        bus_failed=true;
        for (int i=0;i<6;++i) {alarm_gpio=uint8_t(1u<<i);tick();}
        require(alarm_bits==0x3f,"missing direct alarm callback during I2C failure");
        require(io_faults==1,"fault reporting not latched");
    } else if (name=="stale_status") {
        now.fetch_add(50001);status_contains("fresh=0");
    } else if (name=="malformed_frame") {
        rx.insert(rx.end(),{5,COBS_CH_CMD,0}); // truncated COBS block
        tx.clear();cobs_read_process(0);require(replies().empty(),"bad frame dispatched");
        status_contains("motion_qualified=0"); // stream resynchronizes
    } else throw std::runtime_error("unknown scenario: "+name);
    no_arm();status_contains("motion_qualified=0");
}
void save_trace(const std::string& path,const std::string& name,const std::string& error) {
    std::ofstream f(path);
    require(bool(f),"cannot write trace");
    f<<"{\"scenario\":\""<<name<<"\",\"passed\":"<<(error.empty()?"true":"false")
     <<",\"virtual_time_us\":"<<now.load()<<",\"arm_edges\":"<<arm_edges
     <<",\"io_fault_callbacks\":"<<io_faults<<",\"alarm_callback_mask\":"<<alarm_bits
     <<",\"substitutions\":[\"I2C device model\",\"GPIO\",\"virtual tick scheduler\",\"console VFS/USB\",\"main.cpp fault/response callbacks\"],\"events\":[";
    bool first=true;
    for(const auto& e:events) {if(!first)f<<',';first=false;
        f<<"{\"us\":"<<e.us<<",\"kind\":\""<<e.kind<<"\",\"address\":"<<e.address
         <<",\"register\":"<<e.reg<<",\"value\":"<<e.value<<'}';}
    f<<"]}\n";
}
} // namespace sim

SemaphoreHandle_t xSemaphoreCreateMutex() {
    sim::semaphores.emplace_back(new std::recursive_mutex());return sim::semaphores.back().get();
}
BaseType_t xSemaphoreTake(SemaphoreHandle_t h,TickType_t) {
    sim::require(h!=nullptr,"null semaphore");static_cast<std::recursive_mutex*>(h)->lock();return pdTRUE;
}
void xSemaphoreGive(SemaphoreHandle_t h) {static_cast<std::recursive_mutex*>(h)->unlock();}
BaseType_t xTaskCreatePinnedToCore(TaskFunction_t f,const char*,uint32_t,void* arg,unsigned,void*,int) {
    sim::worker=std::thread([=]{try {f(arg);} catch(const sim::Stop&) {}});return pdPASS;
}
void vTaskDelay(TickType_t ticks) {
    std::unique_lock<std::mutex> lock(sim::schedule);sim::paused=true;sim::cv.notify_all();
    sim::cv.wait(lock,[]{return sim::shutdown||sim::release;});
    if(sim::shutdown) throw sim::Stop();
    sim::release=false;sim::paused=false;sim::now.fetch_add(int64_t(ticks)*1000);
}
int64_t esp_timer_get_time() {return sim::now.load();}
int gpio_config(const gpio_config_t* c) {sim::event("gpio_input_config",0,0,int(c->pin_bit_mask&0xffff));return ESP_OK;}
int gpio_get_level(gpio_num_t pin) {
    const int pins[]={1,2,15,16,18,40};
    for(int i=0;i<6;++i)if(pins[i]==pin)return (sim::alarm_gpio>>i)&1;
    throw std::runtime_error("unmodeled GPIO");
}
int i2c_new_master_bus(const i2c_master_bus_config_t* c,i2c_master_bus_handle_t* out) {
    sim::require(c->sda_io_num==41&&c->scl_io_num==42,"wrong I2C pins");
    if(sim::bus_failed)return ESP_FAIL;*out=reinterpret_cast<void*>(1);return ESP_OK;
}
int i2c_master_bus_add_device(i2c_master_bus_handle_t,const i2c_device_config_t* c,i2c_master_dev_handle_t* out) {
    sim::require(c->scl_speed_hz==100000,"unreviewed I2C clock");
    *out=reinterpret_cast<void*>(uintptr_t(c->device_address));return ESP_OK;
}
int i2c_master_transmit(i2c_master_dev_handle_t h,const uint8_t* d,size_t n,int timeout) {
    int address=int(reinterpret_cast<uintptr_t>(h));sim::require(timeout==3&&n>=2&&n<=3,"invalid transaction");
    sim::event(sim::bus_failed?"write_failed":"write",address,d[0],d[1]);
    if(sim::bus_failed || (address==0x40&&sim::missing_adc)) {sim::now.fetch_add(timeout*1000);return ESP_FAIL;}
    if(address==0x40) {
        sim::require(n==3&&d[0]==0&&d[1]==0x41&&d[2]==0x26,"unmodeled INA226 configuration");
        sim::conversion_started=sim::now.load();return ESP_OK;
    }
    sim::require(address==0x20&&n==2,"unmodeled device write");
    int reg=sim::canonical(d[0]);sim::require(reg>=0,"unmodeled register");
    sim::mcp[reg]=d[1];
    if(reg==10||reg==11) {sim::bank1=(d[1]&0x80)!=0;sim::mcp[10]=sim::mcp[11]=d[1];}
    sim::observe_arm();return ESP_OK;
}
int i2c_master_transmit_receive(i2c_master_dev_handle_t h,const uint8_t* r,size_t rn,uint8_t* d,size_t n,int timeout) {
    int a=int(reinterpret_cast<uintptr_t>(h));sim::require(rn==1&&n==2&&timeout==3,"invalid read");
    sim::event(sim::bus_failed?"read_failed":"read",a,r[0],int(n));
    if(sim::bus_failed || (a==0x40&&sim::missing_adc)) {sim::now.fetch_add(timeout*1000);return ESP_FAIL;}
    if(a==0x20) {for(size_t i=0;i<n;++i){int reg=sim::canonical(r[0]+int(i));sim::require(reg>=0,"unmodeled register");d[i]=sim::mcp_read(reg);}return ESP_OK;}
    sim::require(a==0x40,"unmodeled device read");
    int value=0;
    switch(r[0]) {
        case 0xfe:value=0x5449;break;case 0xff:value=0x2260;break;
        case 6:value=sim::now.load()-sim::conversion_started>=1100?8:0;sim::conversion_started=sim::now.load();break;
        case 2:value=sim::voltage_raw;break;
        default:throw std::runtime_error("unmodeled ADC register");
    }
    d[0]=uint8_t(value>>8);d[1]=uint8_t(value);return ESP_OK;
}
int simulated_fileno(FILE* f) {return f==stdin?0:1;}
int simulated_read(int,void* d,size_t n) {
    n=std::min({n,size_t(7),sim::rx.size()-sim::rx_pos});
    if (!n) return 0;
    std::memcpy(d,sim::rx.data()+sim::rx_pos,n);sim::rx_pos+=n;return int(n);
}
int simulated_write(int,const void* d,size_t n) {
    const auto* p=static_cast<const uint8_t*>(d);sim::tx.insert(sim::tx.end(),p,p+n);return int(n);
}
bool usb_serial_jtag_is_connected() {return true;}
int usb_serial_jtag_write_bytes(const void* d,size_t n,unsigned) {return simulated_write(1,d,n);}
extern "C" void serial_printf(const char* fmt,...) {
    char b[1024];va_list args;va_start(args,fmt);int n=vsnprintf(b,sizeof(b),fmt,args);va_end(args);
    sim::require(n>=0&&n<int(sizeof(b)),"response truncated");
    cobs_send(COBS_CH_RESP,reinterpret_cast<const uint8_t*>(b),n);
}
extern "C" void r14_report_normalized_drive_alarm(int axis) {
    sim::alarm_bits|=1<<axis;sim::event("alarm_callback",0,axis);
    if(sim::alarm_observer)sim::alarm_observer(axis);
}
extern "C" void r14_report_transport_reconnect() {
    ++sim::io_faults;sim::event("io_fault_callback");
    if(sim::io_fault_observer)sim::io_fault_observer();
}

namespace runtime_fixture {
void set_fault_handlers(void (*alarm)(int),void (*io_fault)()) {
    sim::alarm_observer=alarm;sim::io_fault_observer=io_fault;
}
void inject_alarm_mask(uint8_t mask) {sim::alarm_gpio=mask;}
void inject_bus_failure(bool failed) {sim::bus_failed=failed;}
namespace {
size_t motion_packets=0;
std::array<uint32_t,6> motion{};
}
void start() {
    sim::startup(false,false);
    sim::tx.clear();
    cobs_set_data_handler([](const uint8_t* payload,int len,int source) {
        sim::require(len==18,"unexpected motion payload size");
        ++motion_packets;
        for(int i=0;i<6;++i) motion[i]=uint32_t(payload[3*i]) |
            (uint32_t(payload[3*i+1])<<8) | (uint32_t(payload[3*i+2])<<16);
        sim::event("motion_callback",source,0,len);
        // Deliberately no motor callback: this observes protocol delivery only.
    });
}
void tick() {sim::tick();}
void record_handshake_event(const char* kind,int value) {sim::event(kind,0,0,value);}
void finish() {sim::finish();}
void receive(const std::vector<uint8_t>& wire,bool serial_bridge) {
    sim::require(!wire.empty() && wire.size()<=COBS_MAX_FRAME,"invalid test datagram size");
    if(serial_bridge) {
        sim::rx.insert(sim::rx.end(),wire.begin(),wire.end());
        while(sim::rx_pos<sim::rx.size()) sim::require(cobs_read_process(0)>0,"stream stalled");
    } else {
        // Test carrier substitutes W5500 datagram delivery; use the real codec/dispatcher.
        sim::require(wire.back()==0,"missing test datagram delimiter");
        uint8_t frame[COBS_MAX_FRAME];
        int n=cobs_decode(wire.data(),int(wire.size()-1),frame);
        if(n>0) cobs_dispatch_frame(frame,n,COBS_SRC_ETHERNET);
    }
}
std::vector<uint8_t> take_output() {std::vector<uint8_t> result;result.swap(sim::tx);return result;}
void telemetry(const float angles[6],const float positions[6]) {cobs_send_telemetry(angles,positions);}
size_t motion_count() {return motion_packets;}
std::array<uint32_t,6> last_motion() {return motion;}
MotionStats motion_stats() {
    cobs_motion_stats_t s{};cobs_motion_stats(&s);
    return {s.seq_frames,s.seq_gaps,s.seq_lost,s.seq_restarts,s.last_source};
}
bool field_on() {return (sim::mcp[0x15]&2)!=0;}
void check_no_arm() {sim::no_arm();}
void save_trace(const std::string& path,const std::string& scenario,const std::string& error) {
    sim::save_trace(path,scenario,error);
}
}

#ifndef R14_RUNTIME_NO_MAIN
int main(int argc,char** argv) {
    if(argc!=3){std::fprintf(stderr,"usage: runtime-scenarios SCENARIO TRACE.json\n");return 2;}
    std::string error;
    try {sim::scenario(argv[1]);}catch(const std::exception& e){error=e.what();}
    sim::finish();
    try {sim::save_trace(argv[2],argv[1],error);}catch(const std::exception& e){error=e.what();}
    if(!error.empty()){std::fprintf(stderr,"FAIL %s: %s\n",argv[1],error.c_str());return 1;}
    std::printf("PASS %s: production COBS/board monitor; no arm edges; virtual_us=%lld\n",argv[1],static_cast<long long>(sim::now.load()));return 0;
}
#endif
