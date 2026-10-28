#pragma once
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
#include "runtime_fixture.h"

namespace loopback {
inline void require(bool ok,const std::string& message) {if(!ok)throw std::runtime_error(message);}
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
inline void send_all(SOCKET s,const std::string& text) {
    size_t pos=0;while(pos<text.size()) {int n=send(s,text.data()+pos,int(text.size()-pos),0);require(n>0,"TCP send");pos+=size_t(n);}
}
struct Peer {
    Socket udp{SOCK_DGRAM},tcp{SOCK_STREAM},control;
    int udp_port=udp.bind_loopback(),tcp_port=tcp.bind_loopback();
    sockaddr_in client{};
    explicit Peer(bool bridge) {if(bridge)require(listen(tcp.value,1)==0,"TCP listen");}
    bool readable() {
        fd_set ready;FD_ZERO(&ready);FD_SET(udp.value,&ready);timeval timeout{};
        int n=select(0,&ready,nullptr,nullptr,&timeout);require(n>=0,"UDP readiness");return n==1;
    }
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
}
