// Actual desktop sockets/handshake -> production COBS/board/safety -> engine model.
// GUI, TX worker, main tasks, NVS, W5500 and real bridge remain outside this lane.
#include "loopback_peer.h"
#include "composed_fixture.h"
#include "app.h"
#include "udp_transport.h"
#include "automation.h"
#include "cobs.h"
#include <algorithm>
#include <iomanip>

namespace {
using namespace loopback;
int udp_receives=0,relayed_frames=0,held_replays=0,raw_ignored=0;
int successful_connections=0,controller_identity_queries=0;
bool bridge_mode=false,app_ready_at_end=false,disconnect_cleared=false;
std::vector<std::string> commands;
const uint32_t values[6]={0,1,0x3ffff,0x40000,0x23456,0xff00};

struct Session {
    Peer peer;
    App app;
    std::shared_ptr<UdpTransport> transport;
    std::string lines;
    Entity& entity() {return app.entities.front();}
    explicit Session(bool bridge):peer(bridge) {
        app.entities.emplace_back();auto& e=entity();e.id=1;e.type=EntityType::HIL;
        e.config.initDefaults();e.config.bit_depth=18;
        std::snprintf(e.hil_host,sizeof(e.hil_host),"127.0.0.1");
        e.hil_udp_port=peer.udp_port;e.hil_tcp_port=peer.tcp_port;
        connect();
    }
    void connect() {
        transport=std::make_shared<UdpTransport>("127.0.0.1",peer.udp_port,peer.tcp_port);
        require(transport->isOpen()&&transport->direct()!=bridge_mode,"network mode selection");
        if(bridge_mode)peer.accept_control();
        app.beginNetworkHil(entity(),transport);
        require(entity().hil_handshake_ok==bridge_mode,"initial readiness contract changed");
    }
    std::vector<uint8_t> receive() {++udp_receives;return peer.receive();}
    void relay(const std::vector<uint8_t>& wire) {
        ++relayed_frames;
        if(!std::all_of(wire.begin(),wire.end(),[](uint8_t c){return c==0;})) {
            std::vector<uint8_t> raw(wire.size());
            int n=cobs_decode(wire.data(),int(wire.size()-1),raw.data());
            if(n>1&&raw[0]==COBS_CH_CMD) {
                std::string cmd(reinterpret_cast<char*>(raw.data()+1),n-1);commands.push_back(cmd);
                if(cmd=="FINGERPRINT?")++controller_identity_queries;
            }
        }
        // Explicit carrier substitution: bridge -> fragmented console stream;
        // direct -> production decoder/dispatcher with Ethernet source tag.
        runtime_fixture::receive(wire,bridge_mode);
        auto out=runtime_fixture::take_output();if(!out.empty())peer.send_wire(out);
    }
    void pump() {
        while(peer.readable())relay(receive());
        for(const auto& line:transport->drainLines()) {
            lines+=line;app.handleHilLine(1,line.c_str());
        }
    }
    void complete() {
        eventually([&]{pump();return entity().hil_handshake_ok;},"desktop safety handshake not ready");
        require(entity().hil_handshake_phase==HandshakePhase::Ready,"incomplete app readiness");
        if(bridge_mode)require(entity().hil_raw_mode&&entity().hil_cap_raw,"bridge did not select raw mode");
        else require(!entity().hil_raw_mode&&entity().hil_device_params.bit_depth==18&&
            runtime_fixture::handshake_bits()==18,"direct negotiated bits differ");
        ++successful_connections;
    }
    void baked() {
        int before=composed_fixture::snapshot().decoded;
        require(transport->sendCobsData(values,18),"desktop motion send failed");relay(receive());
        auto state=composed_fixture::snapshot();
        const std::array<float,6> expected{0,1,262143,0,144470,65280};
        require(state.decoded==before+1,"desktop motion did not reach controller decoder");
        require(state.last_raw==expected,"desktop/controller fixed motion vector differs");
        require(state.rejected==state.decoded,"desktop motion bypassed R14 admission");
        composed_fixture::assert_locked();
    }
    void raw() {
        auto before=composed_fixture::snapshot();
        const float input[6]={0,1,2,-3,4.5f,6};
        require(transport->sendCobsDataRaw(input),"bridge raw send failed");
        auto wire=receive();std::vector<uint8_t> frame(wire.size());
        int n=cobs_decode(wire.data(),int(wire.size()-1),frame.data());
        require(n==25&&frame[0]==0x07,"bridge raw wire format changed");relay(wire);
        auto after=composed_fixture::snapshot();
        require(after.delivered==before.delivered&&after.decoded==before.decoded,"unsupported raw channel reached R14 motion callback");
        ++raw_ignored;composed_fixture::assert_locked();
    }
    void diagnostic(const char* cmd,const char* expected) {
        lines.clear();require(transport->sendCommand(cmd),"diagnostic send failed");relay(receive());
        eventually([&]{pump();return lines.find(expected)!=std::string::npos;},"desktop diagnostic response absent");
    }
    void check() {
        composed_fixture::assert_locked();
        require(entity().hil_handshake_ok,"fault incorrectly treated as a transport disconnect");
        app_ready_at_end=entity().hil_handshake_ok;
    }
    void disconnect(bool control_only=false) {
        entity().hil_cmd_queue.push_back("R14:ARM");
        if(control_only) {
            peer.control.close();
            eventually([&]{return !transport->controlConnected();},"bridge control loss absent");
            require(transport->isOpen(),"control loss unexpectedly closed UDP");
        } else {transport->close();peer.control.close();}
        app.frame_time+=1;app.pollHilHandshake(entity());
        require(!entity().hil_handshake_ok&&!entity().hil_handshake_pending&&
            entity().hil_cmd_queue.empty(),"desktop disconnect retained readiness or queued command");
        disconnect_cleared=true;
    }
};

void scenario(const std::string& name) {
    bridge_mode=name.rfind("bridge_",0)==0;
    composed_fixture::start(bridge_mode);runtime_fixture::enable_handshake();
    composed_fixture::release_software_pause_for_test();
    Session session(bridge_mode);
    if(name=="direct_handshake_fault") {
        require(session.entity().hil_handshake_pending,"handshake fault not injected during negotiation");
        runtime_fixture::inject_alarm_mask(16);runtime_fixture::tick();
    }
    session.complete();
    if(bridge_mode) {
        require(controller_identity_queries==0,"bridge unexpectedly used direct identity handshake");
        session.raw(); // actual bridge raw API is unsupported by this R14 dispatcher
    } else require(controller_identity_queries==1,"direct identity query absent");
    session.baked(); // bridge BAKED API is deliberate adversarial compatibility traffic
    session.diagnostic("R14:STATUS",bridge_mode?"motion_qualified=0":"local console required");

    if(name=="direct_alarm"||name=="bridge_alarm"||name=="direct_handshake_fault") {
        if(name!="direct_handshake_fault") {runtime_fixture::inject_alarm_mask(16);runtime_fixture::tick();}
        auto state=composed_fixture::snapshot();
        require(state.alarm_callbacks==1&&state.alarm_mask==16&&state.stop_calls==1&&
            state.discard_calls==1,"board alarm did not reach shared stop");
        runtime_fixture::inject_alarm_mask(0);runtime_fixture::tick();
        composed_fixture::release_software_pause_for_test();
        for(int i=0;i<8;++i)session.baked();
        require(composed_fixture::snapshot().alarm_mask==16,"alarm recovery cleared latch");
    } else if(name=="direct_bus_recovery"||name=="bridge_bus_recovery") {
        runtime_fixture::inject_bus_failure(true);runtime_fixture::tick();
        auto state=composed_fixture::snapshot();
        require(state.io_callbacks==1&&state.stop_calls==1&&state.discard_calls==1,"bus fault did not reach shared stop");
        session.baked();runtime_fixture::inject_bus_failure(false);runtime_fixture::tick();
        composed_fixture::release_software_pause_for_test();
        for(int i=0;i<8;++i)session.baked();
        require(composed_fixture::snapshot().io_callbacks==1,"bus recovery repeated one-shot fault");
    } else if(name=="direct_reconnect"||name=="bridge_reconnect"||name=="bridge_control_loss") {
        require(session.transport->sendCobsData(values,18),"queued datagram send failed");
        auto held=session.receive(); // test carrier delays this already-sent old-session datagram
        bool control_only=name=="bridge_control_loss";
        session.disconnect(control_only);
        if(control_only)session.baked(); // UDP bypass remains possible despite lost App readiness
        else require(!session.transport->sendCobsData(values,18),"closed transport accepted motion");
        if(control_only)session.transport->close();
        session.connect();session.complete();
        ++held_replays;session.relay(held);session.baked();
        require(composed_fixture::snapshot().io_callbacks==0,"test invented a firmware UDP reconnect callback");
        require(successful_connections==2,"reconnect did not regain transport readiness");
        require(controller_identity_queries==(bridge_mode?0:2),"reconnect identity contract changed");
    } else if(name!="direct_locked"&&name!="bridge_locked")throw std::runtime_error("unknown desktop safety scenario");

    session.diagnostic("R14:FIELD=1",bridge_mode?"FIELD request queued":"local console required");
    runtime_fixture::tick();require(runtime_fixture::field_on()==bridge_mode,"field diagnostic source restriction changed");
    session.baked();
    if(bridge_mode) {
        session.raw();session.diagnostic("R14:ARM","arm/home blocked");
        session.diagnostic("R14:FIELD=0","FIELD request queued");runtime_fixture::tick();
    }
    require(!runtime_fixture::field_on(),"test left diagnostic feed enabled");
    session.check();
}
void save(const std::string& path,const std::string& name,const std::string& error) {
    auto state=composed_fixture::snapshot();std::ofstream f(path);require(bool(f),"cannot write desktop safety trace");
    f<<"{\"scenario\":"<<std::quoted(name)<<",\"passed\":"<<(error.empty()?"true":"false")
     <<",\"error\":"<<std::quoted(error)<<",\"production_main_linked\":false,\"app_tx_worker_linked\":false"
     <<",\"hardware_timing_qualified\":false,\"containment_test_override\":false"
     <<",\"bridge_carrier_substituted\":"<<(bridge_mode?"true":"false")
     <<",\"app_ready_at_end\":"<<(app_ready_at_end?"true":"false")
     <<",\"disconnect_cleared\":"<<(disconnect_cleared?"true":"false")
     <<",\"successful_connections\":"<<successful_connections<<",\"controller_identity_queries\":"<<controller_identity_queries
     <<",\"udp_receives\":"<<udp_receives<<",\"relayed_frames\":"<<relayed_frames<<",\"held_replays\":"<<held_replays
     <<",\"raw_frames_ignored\":"<<raw_ignored<<",\"decoded\":"<<state.decoded<<",\"rejected\":"<<state.rejected
     <<",\"target_writes\":"<<state.target_writes<<",\"step_edges\":"<<state.step_edges
     <<",\"alarm_callbacks\":"<<state.alarm_callbacks<<",\"io_callbacks\":"<<state.io_callbacks
     <<",\"commands\":[";
    for(size_t i=0;i<commands.size();++i){if(i)f<<',';f<<std::quoted(commands[i]);}
    f<<"]}\n";composed_fixture::save_trace(path+".controller.json",name,error);
}
}
int main(int argc,char** argv) {
    if(argc!=3)return 2;WSADATA data{};if(WSAStartup(MAKEWORD(2,2),&data))return 2;
    std::string error;
    try {require(!IsDocumentationMode(),"documentation mode must stay off for loopback");scenario(argv[1]);}
    catch(const std::exception& e){error=e.what();}
    runtime_fixture::finish();
    try{save(argv[2],argv[1],error);}catch(const std::exception& e){error=e.what();}
    WSACleanup();
    if(!error.empty()){std::fprintf(stderr,"FAIL %s: %s\n",argv[1],error.c_str());return 1;}
    std::printf("PASS %s: desktop/handshake to R14 production safety and unchanged engine model\n",argv[1]);return 0;
}
