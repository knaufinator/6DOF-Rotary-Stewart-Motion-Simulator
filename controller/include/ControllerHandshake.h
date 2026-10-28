#pragma once

// Production command formatting shared by main.cpp and host integration.
// I/O, identity storage and cue-state ownership remain with the caller.
#include <cstdint>
#include <cstdlib>
#include <cstring>

namespace controller {
using HandshakeReply = void (*)(const char*, ...);
struct HandshakeIdentity {
    void (*read_mac)(uint8_t*);
    const char* version;
    int protocol;
    const char* build_date;
    const char* build_time;
};

template<class Config>
bool handshake_query(const char* command,const Config& config,uint8_t bits,float max_raw,
                     const HandshakeIdentity& identity,HandshakeReply reply) {
    if(std::strcmp(command,"FINGERPRINT?")==0) {
        uint8_t mac[6];identity.read_mac(mac);
        reply("FINGERPRINT:%02X%02X%02X%02X%02X%02X,fw=%s,proto=%d\r\n",
              mac[0],mac[1],mac[2],mac[3],mac[4],mac[5],identity.version,identity.protocol);
    } else if(std::strcmp(command,"CONFIG?")==0) {
        reply("CONFIG:RD=%.2f,PD=%.2f,L1=%.2f,L2=%.2f,height=%.2f,theta_r=%.2f,theta_p=%.2f,theta_s=%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
              config.RD,config.PD,config.ServoArmLengthL1,config.ConnectingArmLengthL2,
              config.platformHeight,config.theta_r,config.theta_p,config.theta_s[0],config.theta_s[1],
              config.theta_s[2],config.theta_s[3],config.theta_s[4],config.theta_s[5]);
        reply("DRIVE:encoder=%d,gear=%.4f,planetary=%.2f,steps_deg=%.4f\r\n",
              config.encoder_ppr,config.virtual_gear,config.planetary_ratio,config.steps_per_degree);
    } else if(std::strcmp(command,"BITS?")==0) {
        reply("BITS:%d,max_raw=%.0f\r\n",bits,max_raw);
    } else if(std::strcmp(command,"VERSION?")==0) {
        reply("VERSION:%s,proto=%d,date=%s,time=%s\r\n",
              identity.version,identity.protocol,identity.build_date,identity.build_time);
    } else return false;
    return true;
}

inline bool handshake_set_bits(const char* command,uint8_t& bits,float& max_raw,
                               void (*reset_cueing)(),HandshakeReply reply) {
    if(std::strncmp(command,"BITS:",5)!=0)return false;
    int value=std::atoi(command+5);
    if(value>=8 && value<=18) {
        bits=static_cast<uint8_t>(value);
        max_raw=static_cast<float>((1<<value)-1);
        reset_cueing();
        reply("BITS:%d,max_raw=%.0f\r\n",bits,max_raw);
    } else reply("ERR:BITS range 8-18\r\n");
    return true;
}
}
