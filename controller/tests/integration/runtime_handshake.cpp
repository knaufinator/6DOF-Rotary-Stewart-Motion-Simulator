#include "runtime_fixture.h"
#include "ControllerHandshake.h"
#include "CobsTransport.h"
#include "R14Hardware.h"
#include "InverseKinematics.h"
#include "MotionCueing.h"
#include <cstring>
#include <stdexcept>

extern "C" void serial_printf(const char*,...);
namespace {
StewartConfig config{};
MotionCueingConfig cueing{};
uint8_t bits=12;
float max_raw=4094.0f;
int reset_count=0;
const controller::HandshakeIdentity identity{
    [](uint8_t* mac){const uint8_t fixed[6]={0,0x11,0x22,0x33,0x44,0x55};std::memcpy(mac,fixed,6);},
    "test-r14",1,"test-date","test-time"
};
}
namespace runtime_fixture {
void enable_handshake() {
    initDefaultStewartConfig(&config);computeStepsPerDegree(&config);
    cobs_set_cmd_handler([](const char* command,int source){
        int query=std::strcmp(command,"FINGERPRINT?")==0?1:std::strcmp(command,"CONFIG?")==0?2:
            std::strcmp(command,"BITS?")==0?3:std::strncmp(command,"BITS:",5)==0?4:0;
        record_handshake_event("firmware_handshake_query",query);
        if(r14_hardware_command(command,source))return;
        if(controller::handshake_query(command,config,bits,max_raw,identity,serial_printf))return;
        if(controller::handshake_set_bits(command,bits,max_raw,
                []{resetMotionCueing(&cueing);++reset_count;},serial_printf))return;
        // PEERMAC is W5500-specific and not part of this host protocol fixture.
        if(std::strncmp(command,"PEERMAC:",8)==0)return;
        throw std::runtime_error(std::string("unmodeled handshake command: ")+command);
    });
}
int handshake_bits() {return bits;}
int handshake_cue_resets() {return reset_count;}
}
