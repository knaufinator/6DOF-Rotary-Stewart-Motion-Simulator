// Real desktop sockets/codec against the production R14/COBS runtime fixture.
// Endpoints are explicitly loopback; no serial enumeration, GUI or live hardware.
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <winsock2.h>
#include <ws2tcpip.h>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include "udp_transport.h"
#include "automation.h"
#include "cobs.h"
#include "runtime_fixture.h"

namespace {
void require(bool ok,const std::string& message) {if(!ok)throw std::runtime_error(message);}
template<class F> void eventually(F predicate,const char* message) {
    auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(3);
    while(!predicate()) {
        require(std::chrono::steady_clock::now()<deadline,message);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}
struct Socket {
    SOCKET value=INVALID_SOCKET;
    explicit Socket(int type) : value(socket(AF_INET,type,0)) {require(value!=INVALID_SOCKET,"socket creation");}
    Socket()=default;
    ~Socket(){close();}
    Socket(const Socket&)=delete;
    Socket& operator=(const Socket&)=delete;
    void close(){if(value!=INVALID_SOCKET){shutdown(value,SD_BOTH);closesocket(value);value=INVALID_SOCKET;}}
    void timeout(){DWORD ms=2000;require(setsockopt(value,SOL_SOCKET,SO_RCVTIMEO,reinterpret_cast<char*>(&ms),sizeof(ms))==0,"socket timeout");}
    int bind_loopback() {
        BOOL exclusive=TRUE;
        require(setsockopt(value,SOL_SOCKET,SO_EXCLUSIVEADDRUSE,reinterpret_cast<char*>(&exclusive),sizeof(exclusive))==0,"exclusive bind");
        sockaddr_in a{};a.sin_family=AF_INET;a.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
        require(bind(value,reinterpret_cast<sockaddr*>(&a),sizeof(a))==0,"loopback bind");
        int n=sizeof(a);require(getsockname(value,reinterpret_cast<sockaddr*>(&a),&n)==0,"socket address");
        timeout();return ntohs(a.sin_port);
    }
};
void send_all(SOCKET s,const std::string& text) {
    size_t pos=0;while(pos<text.size()) {int n=send(s,text.data()+pos,int(text.size()-pos),0);require(n>0,"TCP send");pos+=size_t(n);}
}
struct Peer {
    Socket udp{SOCK_DGRAM},tcp{SOCK_STREAM},control;
    int udp_port=udp.bind_loopback(),tcp_port=tcp.bind_loopback();
    sockaddr_in client{};
    explicit Peer(bool bridge) {if(bridge)require(listen(tcp.value,1)==0,"TCP listen");}
    void accept_control() {
        fd_set ready;FD_ZERO(&ready);FD_SET(tcp.value,&ready);timeval timeout{3,0};
        require(select(0,&ready,nullptr,nullptr,&timeout)==1,"TCP accept readiness");
        control.value=accept(tcp.value,nullptr,nullptr);require(control.value!=INVALID_SOCKET,"TCP accept");control.timeout();
    }
    std::vector<uint8_t> receive() {
        uint8_t data[1024];int size=sizeof(client);
        int n=recvfrom(udp.value,reinterpret_cast<char*>(data),sizeof(data),0,reinterpret_cast<sockaddr*>(&client),&size);
        require(n>0 && client.sin_addr.s_addr==htonl(INADDR_LOOPBACK),"UDP loopback receive");
        return {data,data+n};
    }
    void send_wire(const std::vector<uint8_t>& bytes) {
        require(!bytes.empty(),"empty firmware output");
        require(sendto(udp.value,reinterpret_cast<const char*>(bytes.data()),int(bytes.size()),0,
                       reinterpret_cast<sockaddr*>(&client),sizeof(client))==int(bytes.size()),"UDP send");
    }
    void roundtrip(bool bridge) {
        runtime_fixture::receive(receive(),bridge);
        auto bytes=runtime_fixture::take_output();if(!bytes.empty())send_wire(bytes);
    }
    std::string control_line() {
        std::string line;char c;
        while(line.size()<512) {require(recv(control.value,&c,1,0)==1,"TCP receive line");line+=c;if(c=='\n')return line;}
        throw std::runtime_error("oversized TCP line");
    }
};
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
