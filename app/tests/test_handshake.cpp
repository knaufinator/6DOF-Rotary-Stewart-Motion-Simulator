#include "loopback_peer.h"
#include "app.h"
#include "udp_transport.h"
#include "automation.h"
#include "cobs.h"
#include <functional>
#include <algorithm>

namespace {
using namespace loopback;
std::vector<std::string> commands;
std::vector<uint8_t> response(const std::string& text) {
    std::vector<uint8_t> raw{COBS_CH_RESP};raw.insert(raw.end(),text.begin(),text.end());
    std::vector<uint8_t> wire(COBS_MAX_ENC_SIZE(int(raw.size()))+1);
    int n=cobs_encode(raw.data(),int(raw.size()),wire.data());wire[n++]=0;wire.resize(n);return wire;
}
std::string command(const std::vector<uint8_t>& wire) {
    if(std::all_of(wire.begin(),wire.end(),[](uint8_t c){return c==0;}))return {};
    std::vector<uint8_t> raw(wire.size());int n=cobs_decode(wire.data(),int(wire.size()-1),raw.data());
    require(n>0 && raw[0]==COBS_CH_CMD,"unexpected app handshake frame");
    return {reinterpret_cast<char*>(raw.data()+1),size_t(n-1)};
}
struct Session {
    Peer peer{false};
    App app;
    std::shared_ptr<UdpTransport> transport;
    Entity& entity() {return app.entities.front();}
    Session() {
        app.entities.emplace_back();auto& e=entity();
        e.id=1;e.config.initDefaults();e.config.bit_depth=18;
        e.hil_tel_target_hz=100;e.hil_tick_rate_us=1000;
        std::snprintf(e.hil_host,sizeof(e.hil_host),"127.0.0.1");
        connect();
    }
    void connect() {
        transport=std::make_shared<UdpTransport>("127.0.0.1",peer.udp_port,peer.tcp_port);
        app.beginNetworkHil(entity(),transport);
        require(!entity().hil_handshake_ok,"connect retained stale ready permission");
    }
    void pump(const std::function<bool(const std::string&,std::vector<uint8_t>&)>& filter={}) {
        auto phase_before=entity().hil_handshake_phase;
        while(peer.readable()) {
            auto wire=peer.receive();auto cmd=command(wire);if(!cmd.empty())commands.push_back(cmd);
            runtime_fixture::receive(wire,false);
            auto out=runtime_fixture::take_output();
            bool send=!filter || filter(cmd,out);
            if(send && !out.empty())peer.send_wire(out);
        }
        for(const auto& line:transport->drainLines())app.handleHilLine(1,line.c_str());
        if(entity().hil_handshake_phase!=phase_before)
            runtime_fixture::record_handshake_event("app_handshake_phase",int(entity().hil_handshake_phase));
    }
    void complete(const std::function<bool(const std::string&,std::vector<uint8_t>&)>& filter={}) {
        eventually([&]{pump(filter);return entity().hil_handshake_phase==HandshakePhase::Ready ||
            entity().hil_handshake_phase==HandshakePhase::Failed;},"handshake did not settle");
    }
};
void rejected(const std::string& command_name,const std::string& injected,const char* message) {
    Session s;
    bool delivered=false;
    auto filter=[&](const std::string& cmd,std::vector<uint8_t>& out){
        if(cmd==command_name){out=response(injected);delivered=true;}return true;
    };
    eventually([&]{s.pump(filter);return delivered;},"injected response request absent");
    // Allow real reader delivery between virtual retry deadlines; avoid racing
    // the socket thread by advancing virtual seconds every few microseconds.
    for(int i=0;i<12;++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));s.pump(filter);
        if(!s.entity().hil_handshake_pending)break;
        s.app.frame_time+=0.6;s.app.pollHilHandshake(s.entity());
    }
    if(s.entity().hil_handshake_pending){s.app.frame_time+=3;s.app.pollHilHandshake(s.entity());}
    require(s.entity().hil_handshake_phase==HandshakePhase::Failed && !s.entity().hil_handshake_ok,message);
    require(!s.entity().hil_handshake_pending,"failed handshake still pending");
    runtime_fixture::check_no_arm();
}
void scenario(const std::string& name) {
    runtime_fixture::start();runtime_fixture::enable_handshake();
    if(name=="happy") {
        Session s;s.complete([](const std::string& cmd,std::vector<uint8_t>& out){
            if(cmd=="BITS:18") {auto stale=response("BITS:12,max_raw=4094");stale.insert(stale.end(),out.begin(),out.end());out=stale;}
            return true;
        });auto& e=s.entity();
        require(e.hil_handshake_ok && !e.hil_handshake_pending,"valid handshake not ready");
        require(e.hil_device_params.config_received && e.hil_device_params.bits_received,"incomplete negotiated parameters");
        require(std::string(e.hil_fingerprint)=="001122334455" && e.hil_device_params.proto_ver==1,"paired identity");
        require(e.hil_device_params.bit_depth==18 && e.hil_device_params.max_raw==262143,"negotiated scaling");
        require(runtime_fixture::handshake_bits()==18 && runtime_fixture::handshake_cue_resets()==1,"firmware bit-setting/reset callback");
        std::vector<std::string> queries;
        for(const auto& c:commands)if(c.rfind("PEERMAC:",0)!=0)queries.push_back(c);
        require(queries==std::vector<std::string>{"FINGERPRINT?","CONFIG?","BITS?","BITS:18"},"handshake query order");
        // Stale identity/bit replies may not change the established session.
        s.app.handleHilLine(1,"FINGERPRINT:INVALID,fw=stale,proto=99");
        s.app.handleHilLine(1,"BITS:12,max_raw=4095");
        require(e.hil_handshake_ok && e.hil_device_params.bit_depth==18 &&
                std::string(e.hil_device_params.fingerprint)=="001122334455","stale response corrupted ready session");
    } else if(name=="retry") {
        Session s;std::array<bool,4> lost{};
        auto filter=[&](const std::string& cmd,std::vector<uint8_t>&){
            const std::array<std::string,4> q{"FINGERPRINT?","CONFIG?","BITS?","BITS:18"};
            for(size_t i=0;i<q.size();++i)if(cmd==q[i] && !lost[i]){lost[i]=true;return false;}
            return true;
        };
        const std::array<HandshakePhase,4> phases{HandshakePhase::WaitFingerprint,HandshakePhase::WaitConfig,
            HandshakePhase::WaitBits,HandshakePhase::WaitBitsConfirm};
        for(size_t i=0;i<phases.size();++i) {
            eventually([&]{s.pump(filter);return lost[i];},"initial response not dropped");
            s.app.frame_time+=0.51;s.app.pollHilHandshake(s.entity());
            eventually([&]{s.pump(filter);return s.entity().hil_handshake_phase!=phases[i];},"lost response retry failed");
        }
        require(std::all_of(lost.begin(),lost.end(),[](bool v){return v;}),"retry phases not exercised");
        require(s.entity().hil_handshake_ok,"retries did not release app handshake gate");
    } else if(name=="timeout") {
        for(const std::string dropped:{"FINGERPRINT?","CONFIG?","BITS?","BITS:18"}) {
            Session s;bool observed=false;
            auto filter=[&](const std::string& cmd,std::vector<uint8_t>&){if(cmd==dropped){observed=true;return false;}return true;};
            eventually([&]{s.pump(filter);return observed;},"timeout request not observed");
            for(int i=0;i<35;++i){s.pump(filter);s.app.frame_time+=0.6;s.app.pollHilHandshake(s.entity());}
            s.app.frame_time+=2;s.app.pollHilHandshake(s.entity());
            require(s.entity().hil_handshake_phase==HandshakePhase::Failed && !s.entity().hil_handshake_ok,"missing handshake response did not time out");
            require(!s.entity().hil_handshake_pending,"timeout retained pending request");
            std::string paired=s.entity().hil_fingerprint;
            // Late replies cannot resurrect a failed handshake or change pairing.
            s.app.handleHilLine(1,"FINGERPRINT:001122334455,fw=test-r14,proto=1");
            require(s.entity().hil_handshake_phase==HandshakePhase::Failed && s.entity().hil_fingerprint==paired,"late reply changed failed session");
        }
    } else if(name=="compatibility") {
        Session s;s.entity().config.bit_depth=16;
        s.complete([](const std::string& cmd,std::vector<uint8_t>& out){
            // Legacy/Mini wire-format compatibility, not a Mini firmware emulator.
            if(cmd=="FINGERPRINT?")out=response("FINGERPRINT:001122334455,fw=1.0.0,proto=2,platform=mini6dof,caps=raw");
            if(cmd=="CONFIG?") {
                out=response("SERVO:center=1400,1400,1400,1400,1400,1400,pulse_per_rad=900");
                auto config=response("CONFIG:RD=400.05,PD=406.40,L1=184.15,L2=723.90,height=648.13,theta_r=10,theta_p=30");
                out.insert(out.end(),config.begin(),config.end());
                auto servo=response("SERVO:center=1500,1501,1502,1503,1504,1505,pulse_per_rad=1018.6");
                out.insert(out.end(),servo.begin(),servo.end());
            }
            return true;
        });
        auto& e=s.entity();
        require(e.hil_handshake_ok && e.config.platform_type==PlatformType::PWMServo && e.hil_cap_raw,"Mini wire-format compatibility");
        require(e.config.servo.center_us[5]==1505 && e.config.servo.pulse_per_rad>1018,"servo response ordering");
    } else if(name=="reconnect") {
        Session s;s.complete();require(s.entity().hil_handshake_ok,"first handshake");
        s.transport->close();s.app.frame_time+=1;s.app.pollHilHandshake(s.entity());
        require(!s.entity().hil_handshake_ok && !s.entity().hil_handshake_pending,"disconnect retained ready permission");
        s.connect();s.complete();require(s.entity().hil_handshake_ok,"reconnected handshake");
        require(!runtime_fixture::field_on(),"reconnect changed field feed");
        Peer bridge(true);App bridge_app;bridge_app.entities.emplace_back();
        auto& e=bridge_app.entities.front();e.config.initDefaults();e.id=2;
        auto up=std::make_shared<UdpTransport>("127.0.0.1",bridge.udp_port,bridge.tcp_port);
        bridge.accept_control();bridge_app.beginNetworkHil(e,up);
        require(e.hil_handshake_ok,"existing bridge bypass behavior changed");
        bridge.control.close();eventually([&]{return !up->controlConnected();},"bridge disconnect absent");
        bridge_app.pollHilHandshake(e);
        require(!e.hil_handshake_ok && !e.hil_handshake_pending,"bridge control loss retained ready permission");
    } else if(name=="identity") {
        rejected("FINGERPRINT?","FINGERPRINT:XYZ,fw=test-r14,proto=1","malformed identity accepted");
        rejected("FINGERPRINT?","FINGERPRINT:001122334455,fw=test-r14,proto=99","unsupported protocol accepted");
        rejected("FINGERPRINT?","FINGERPRINT:001122334455,proto=1","missing firmware identity accepted");
        Session s;std::snprintf(s.entity().hil_fingerprint,sizeof(s.entity().hil_fingerprint),"AABBCCDDEEFF");
        s.complete();require(s.entity().hil_handshake_phase==HandshakePhase::Failed && !s.entity().serial,"different paired device accepted");
    } else if(name=="bad_bits") {
        for(const auto& text:{"BITS:12,max_raw=4095","BITS:18,max_raw=4095", "BITS:18,max_raw=nan","BITS:18oops,max_raw=262143"})
            rejected("BITS:18",text,"incorrect bit-depth confirmation accepted");
    } else if(name=="malformed") {
        rejected("CONFIG?","CONFIG:garbage","incomplete geometry accepted");
        rejected("CONFIG?","CONFIG:RD=nan,PD=16,L1=7.25,L2=28.5,height=25.52,theta_r=10,theta_p=30","NaN geometry accepted");
        rejected("CONFIG?","CONFIG:RD=-1,PD=16,L1=7.25,L2=28.5,height=25.52,theta_r=10,theta_p=30","negative geometry accepted");
    } else throw std::runtime_error("unknown handshake scenario");
    runtime_fixture::check_no_arm();
    require(!runtime_fixture::field_on() && runtime_fixture::motion_count()==0,"handshake generated output request");
}
}
int main(int argc,char** argv) {
    if(argc!=3)return 2;
    WSADATA data{};if(WSAStartup(MAKEWORD(2,2),&data))return 2;
    std::string error;
    try{require(!IsDocumentationMode(),"documentation mode must stay off for loopback");scenario(argv[1]);}
    catch(const std::exception& e){error=e.what();}
    runtime_fixture::finish();
    try{runtime_fixture::save_trace(argv[2],std::string("handshake_")+argv[1],error);}catch(const std::exception& e){error=e.what();}
    WSACleanup();
    if(!error.empty()){std::fprintf(stderr,"FAIL %s: %s\n",argv[1],error.c_str());return 1;}
    std::printf("PASS handshake %s: production app/controller protocol over loopback\n",argv[1]);return 0;
}
