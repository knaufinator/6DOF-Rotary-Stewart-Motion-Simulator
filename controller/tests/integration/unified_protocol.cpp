// Real shared entry, command router, COBS stream and handshake; SDK I/O is modeled.
#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>
#include "ControllerApplication.h"
#include "ControllerHandshake.h"
#include "ControllerGeometry.h"
#include "FirmwareProfile.h"
#include "CobsTransport.h"
#include "freertos/semphr.h"

namespace {
std::vector<uint8_t> rx, tx, received;
size_t offset = 0;
int packets = 0, raw_packets = 0, commands = 0, last_source = -1;
bool booted = false;
std::string command, replies;
void require(bool ok, const char* why) { if (!ok) throw std::runtime_error(why); }
void reply(const char* fmt, ...) {
    char text[1024]; va_list args; va_start(args, fmt);
    vsnprintf(text, sizeof(text), fmt, args); va_end(args); replies += text;
}
void mac(uint8_t* out) { for (int i=0;i<6;++i) out[i]=uint8_t(i+1); }
std::vector<uint8_t> frame(uint8_t channel, int length) {
    std::vector<uint8_t> bytes(size_t(length)+1, 0x0a); bytes[0]=channel; return bytes;
}
void dispatch(uint8_t channel,int length,int source=COBS_SRC_SERIAL) {
    auto bytes=frame(channel,length);cobs_dispatch_frame(bytes.data(),int(bytes.size()),source);
}
void stream(const std::vector<uint8_t>& bytes) {
    uint8_t enc[600]; const int n=cobs_encode(bytes.data(),int(bytes.size()),enc);
    rx.insert(rx.end(),enc,enc+n);rx.push_back(0);
    while(offset<rx.size()) require(cobs_read_process(0)>0,"fragmented stream stalled");
}
std::string response() {
    std::string out;size_t start=0;
    for(size_t i=0;i<tx.size();++i)if(tx[i]==0) {
        if(i>start) {uint8_t b[600];int n=cobs_decode(tx.data()+start,int(i-start),b);
            require(n>0,"invalid framed reply");if(b[0]==COBS_CH_RESP)out.append(reinterpret_cast<char*>(b+1),size_t(n-1));}
        start=i+1;
    }
    return out;
}
}
SemaphoreHandle_t xSemaphoreCreateMutex() { return reinterpret_cast<void*>(1); }
BaseType_t xSemaphoreTake(SemaphoreHandle_t,TickType_t) { return pdTRUE; }
void xSemaphoreGive(SemaphoreHandle_t) {}
int64_t esp_timer_get_time() { return 1000; }
int simulated_fileno(FILE* f) {return f==stdin?0:1;}
int simulated_read(int,void* p,size_t n) {
    n=std::min({n,size_t(3),rx.size()-offset});
    if(n)std::memcpy(p,rx.data()+offset,n);offset+=n;return int(n);
}
int simulated_write(int,const void* p,size_t n) {
    const auto* b=static_cast<const uint8_t*>(p);tx.insert(tx.end(),b,b+n);return int(n);
}
bool usb_serial_jtag_is_connected() {return true;}
int usb_serial_jtag_write_bytes(const void* p,size_t n,unsigned) {return simulated_write(1,p,n);}
void controller_profile_run() {booted=true;}
void controller_profile_command(char* text,int source) {
    ++commands;command=text;last_source=source;text[0]='X'; // owned mutable copy
}
extern "C" void app_main();

int main() {
    try {
        app_main();require(booted,"shared entry did not invoke hardware startup");
        cobs_transport_init(921600);tx.clear();
        cobs_set_data_handler([](const uint8_t* p,int n,int source) {
            ++packets;received.assign(p,p+n);last_source=source;
        });
        cobs_set_data_raw_handler([](const uint8_t*,int,int source) {++raw_packets;last_source=source;});
        cobs_set_cmd_handler(controller_command_dispatch);
#if FIRMWARE_PROFILE_MINI
        for(int n=0;n<12;++n)dispatch(COBS_CH_DATA,n);
        for(int n=0;n<24;++n)dispatch(COBS_CH_DATA_RAW,n);
        require(packets==0 && raw_packets==0,"short Mini frame admitted");
        stream(frame(COBS_CH_DATA,12));require(packets==1 && received.size()==12,"Mini baked stream lost");
        stream(frame(COBS_CH_DATA_RAW,24));require(raw_packets==1,"Mini raw stream lost");
        dispatch(COBS_CH_DATA18,18);dispatch(COBS_CH_DATA18_SEQ,20);
        require(packets==1 && raw_packets==1,"Mini admitted step-controller channel");
#else
        for(int n=0;n<18;++n)dispatch(COBS_CH_DATA18,n);
        for(int n=0;n<20;++n)dispatch(COBS_CH_DATA18_SEQ,n);
        require(packets==0,"short full-controller frame admitted");
        stream(frame(COBS_CH_DATA18,18));require(packets==1 && received.size()==18,"DATA18 stream lost");
        dispatch(COBS_CH_DATA18_SEQ,20,COBS_SRC_ETHERNET);
        require(packets==2 && last_source==COBS_SRC_ETHERNET,"sequence source lost");
        cobs_motion_stats_t stats{};cobs_motion_stats(&stats);
        require(stats.seq_frames==1,"sequence accounting lost");
        dispatch(COBS_CH_DATA,12);dispatch(COBS_CH_DATA_RAW,24);
        require(packets==2 && raw_packets==0,"full controller admitted Mini motion");
#endif
        // A bad frame must not dispatch a command; the next valid one must recover.
        rx.insert(rx.end(),{5,COBS_CH_CMD,0});cobs_read_process(0);
        require(commands==0,"malformed command dispatched");
        std::vector<uint8_t> query{COBS_CH_CMD};const std::string name="PROFILE?";
        query.insert(query.end(),name.begin(),name.end());stream(query);
        require(commands==0,"profile query escaped common router");
        require(response()==std::string("PROFILE:")+controller::profile_name()+",actuator="+controller::actuator_name()+"\r\n","profile identity wrong");
        controller_command_dispatch("CONFIG:RD=20",COBS_SRC_BLE);
        require(command=="CONFIG:RD=20" && last_source==COBS_SRC_BLE,"command/source changed");
        controller_command_dispatch(std::string(256,'a').c_str(),COBS_SRC_SERIAL);
        require(commands==1,"oversized command truncated into a different command");
        stream(frame(COBS_CH_CMD,256));
        require(commands==1,"oversized framed command was truncated and dispatched");

        controller::HandshakeIdentity full(mac,"1.0.0",2,"DATE","TIME");
        controller::HandshakeIdentity mini(mac,"1.0.0",1,"DATE","TIME","mini-6dof","raw");
        require(controller::handshake_identity_query("FINGERPRINT?",16,65535,full,reply),"full query not handled");
        require(replies=="FINGERPRINT:010203040506,fw=1.0.0,proto=2\r\n","full fingerprint changed");
        replies.clear();controller::handshake_identity_query("FINGERPRINT?",16,65535,mini,reply);
        require(replies=="FINGERPRINT:010203040506,fw=1.0.0,proto=1,platform=mini-6dof,caps=raw\r\n","Mini fingerprint changed");
        replies.clear();controller::handshake_identity_query("VERSION?",16,65535,mini,reply);
        require(replies=="VERSION:1.0.0,proto=1,platform=mini-6dof,date=DATE,time=TIME\r\n","Mini version changed");
        struct Config { float RD=1,PD=2,ServoArmLengthL1=3,ConnectingArmLengthL2=4,platformHeight=5,theta_r=6,theta_p=7; } config;
        require(controller::set_geometry_parameter(config,"RD",15.75f) && config.RD==15.75f,"shared geometry setter wrong");
        require(!controller::set_geometry_parameter(config,"not-a-key",0) && config.RD==15.75f,"unknown geometry mutated config");
        std::puts("PASS unified profile entry, protocol isolation, framing, identity and geometry");
        return 0;
    } catch(const std::exception& e) {std::fprintf(stderr,"FAIL: %s\n",e.what());return 1;}
}
