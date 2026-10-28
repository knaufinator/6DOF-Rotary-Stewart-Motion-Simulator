#include "loopback_peer.h"
#include "udp_transport.h"
#include "automation.h"
#include "cobs.h"

namespace {
using namespace loopback;
std::vector<uint8_t> decode(const std::vector<uint8_t>& wire) {
    require(!wire.empty() && wire.back()==0,"motion delimiter");
    std::vector<uint8_t> bytes(wire.size());int n=cobs_decode(wire.data(),int(wire.size()-1),bytes.data());
    require(n>0,"motion COBS decode");bytes.resize(n);return bytes;
}
void expect_line(UdpTransport& app,const std::string& text) {
    std::string lines;
    eventually([&]{for(const auto& line:app.drainLines())lines+=line;return lines.find(text)!=std::string::npos;},"firmware response absent");
}
void verify_motion(const std::vector<uint8_t>& wire,bool bridge,unsigned sequence) {
    // Independently written byte vector: zero, byte boundary, 18-bit maximum,
    // masked high bits and asymmetric axes catch packing/order regressions.
    const std::vector<uint8_t> expected{0,0,0, 1,0,0, 0xff,0xff,3, 0,0,0, 0x56,0x34,2, 0,0xff,0};
    auto bytes=decode(wire);size_t offset=bridge?1:3;
    require(bytes.size()==offset+expected.size(),"motion size");
    require(bytes[0]==(bridge?0x06:0x08),"motion channel/mode");
    if(!bridge)require(bytes[1]==(sequence&255) && bytes[2]==(sequence>>8),"motion sequence");
    require(std::vector<uint8_t>(bytes.begin()+offset,bytes.end())==expected,"motion axis/18-bit encoding");
}
void scenario(bool bridge) {
    Peer peer(bridge);
    runtime_fixture::start();
    const uint32_t targets[6]={0,1,0x3ffff,0x40000,0x23456,0xff00};
    {
        UdpTransport app("127.0.0.1",peer.udp_port,peer.tcp_port);
        require(app.isOpen() && app.direct()==!bridge && app.controlConnected()==bridge,"transport mode negotiation");
        if(bridge) {
            peer.accept_control();
            require(app.controlSend("{\"verb\":\"status\"}"),"TCP status request");
            require(peer.control_line()=="{\"verb\":\"status\"}\n","TCP request framing");
            // Split/coalesced JSON exercises the actual streaming parser. Bridge is a test carrier.
            send_all(peer.control.value,"bad json\n{\"type\":\"hello\"}\n{\"type\":\"event\",\"event\":\"sta");
            send_all(peer.control.value,"tus\",\"source\":\"LIVE\",\"boot_source\":\"OFF\",\"play_state\":\"stopped\",\"mem\":{\"used\":12,\"free\":34}}\r\n");
            eventually([&]{return app.source()=="LIVE" && app.memFree()==34;},"bridge status parse");
            require(app.bootSource()=="OFF" && app.playState()=="stopped" && app.memUsed()==12,"bridge status snapshot");
            send_all(peer.control.value,"{\"type\":\"event\",\"event\":\"resp\",\"text\":\"bridge carrier ready\"}\n");
            expect_line(app,"bridge carrier ready");
        } else require(!app.controlSend("{}"),"direct mode accepted TCP request");
        for(unsigned sequence=0;sequence<3;++sequence) {
            require(app.sendCobsData(targets,18),"send motion");auto wire=peer.receive();verify_motion(wire,bridge,sequence);
            if(sequence!=1)runtime_fixture::receive(wire,bridge); // deliberate carrier loss
        }
        require(runtime_fixture::motion_count()==2,"production motion dispatch count");
        require(runtime_fixture::last_motion()==std::array<uint32_t,6>{0,1,0x3ffff,0,0x23456,0xff00},"production motion payload");
        auto stats=runtime_fixture::motion_stats();
        require(stats.source==(bridge?0:1),"firmware carrier source tag");
        require(stats.frames==(bridge?0u:2u) && stats.gaps==(bridge?0u:1u) && stats.lost==(bridge?0u:1u),"sequence loss accounting");
        require(app.sendCommand("R14:STATUS"),"send diagnostic");peer.roundtrip(bridge);
        expect_line(app,bridge?"motion_qualified=0":"local console required");
        require(app.sendCommand("R14:FIELD=1"),"send field request");peer.roundtrip(bridge);runtime_fixture::tick();
        expect_line(app,bridge?"FIELD request queued":"local console required");
        require(runtime_fixture::field_on()==bridge,"field request source policy");
        if(bridge) {
            require(app.sendCommand("R14:FIELD=0"),"clear field request");peer.roundtrip(true);runtime_fixture::tick();
            require(!runtime_fixture::field_on(),"field failed to clear");
            require(app.sendCommand("R14:ARM"),"arm rejection request");peer.roundtrip(true);expect_line(app,"arm/home blocked");
        }
        runtime_fixture::check_no_arm();
        const uint8_t malformed[]={5,COBS_CH_CMD,0};
        auto count=runtime_fixture::motion_count();
        require(app.write(malformed,sizeof(malformed)),"send malformed frame");peer.roundtrip(bridge);
        require(runtime_fixture::motion_count()==count,"malformed frame reached motion callback");
        require(app.sendCommand("R14:STATUS"),"post-corruption diagnostic");peer.roundtrip(bridge);
        expect_line(app,bridge?"motion_qualified=0":"local console required");
        float angles[6]={-1,0,1,2,3,4},positions[6]={10,20,30,40,50,60};
        runtime_fixture::telemetry(angles,positions);peer.send_wire(runtime_fixture::take_output());
        eventually([&]{return app.telemetrySeq()==1;},"firmware telemetry absent");
        auto telemetry=app.getLatestTelemetry();
        for(int i=0;i<6;++i)require(telemetry.angles[i]==angles[i] && telemetry.positions[i]==positions[i],"telemetry axis ordering");
        angles[2]=std::numeric_limits<float>::quiet_NaN();
        runtime_fixture::telemetry(angles,positions);peer.send_wire(runtime_fixture::take_output());
        eventually([&]{return app.telemetryRejected()==1;},"NaN telemetry not rejected");
        require(app.telemetrySeq()==1 && app.getLatestTelemetry().angles[2]==1,"invalid telemetry replaced valid snapshot");
        peer.send_wire({5,COBS_CH_RESP,0});
        eventually([&]{return app.cobsDecodeFail()>0;},"malformed firmware output not counted");
        if(bridge) {
            peer.control.close();eventually([&]{return !app.controlConnected();},"TCP disconnect not observed");
            require(!app.controlSend("{}"),"closed TCP accepted command");
        }
        app.close();require(!app.isOpen() && !app.sendCobsData(targets,18),"closed transport accepted motion");
    }
    // App recreates transports for reconnects; UdpTransport has no automatic retry loop.
    {
        UdpTransport app("127.0.0.1",peer.udp_port,peer.tcp_port);
        require(app.isOpen() && app.direct()==!bridge,"reconnect mode");
        if(bridge)peer.accept_control();
        require(app.sendCobsData(targets,18),"reconnect motion send");auto wire=peer.receive();verify_motion(wire,bridge,0);
        runtime_fixture::receive(wire,bridge);
        require(runtime_fixture::motion_count()==3,"reconnect motion dispatch");
        require(runtime_fixture::motion_stats().restarts==(bridge?0u:1u),"sender restart accounting");
        require(app.sendCommand("R14:STATUS"),"reconnect diagnostic");peer.roundtrip(bridge);
        expect_line(app,bridge?"motion_qualified=0":"local console required");
        runtime_fixture::check_no_arm();require(!runtime_fixture::field_on(),"reconnect revived field output");
    }
}
}
int main(int argc,char** argv) {
    if(argc!=3)return 2;
    std::string name=argv[1],error;
    if(name!="direct" && name!="bridge")return 2;
    WSADATA data{};if(WSAStartup(MAKEWORD(2,2),&data))return 2;
    try {require(!IsDocumentationMode(),"active loopback test requires documentation mode unset");scenario(name=="bridge");}
    catch(const std::exception& e){error=e.what();}
    runtime_fixture::finish();
    try{runtime_fixture::save_trace(argv[2],"desktop_"+name,error);}catch(const std::exception& e){error=e.what();}
    WSACleanup();
    if(!error.empty()){std::fprintf(stderr,"FAIL %s: %s\n",name.c_str(),error.c_str());return 1;}
    std::printf("PASS %s: real loopback desktop transport, production COBS/R14 monitor, zero arm edges\n",name.c_str());return 0;
}
