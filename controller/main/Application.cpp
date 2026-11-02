#include "ControllerApplication.h"
#include "FirmwareProfile.h"
#include "CobsTransport.h"
#include <cstdio>
#include <cstring>

extern "C" void app_main() {
    // R14's first operation remains its GPIO47 inhibit / GPIO48 clear setup.
    // No shared initialization is allowed ahead of the hardware runtime.
    controller_profile_run();
}

void controller_command_dispatch(const char* command, int source) {
    if (!command) return;
    if (std::strcmp(command, "PROFILE?") == 0) {
        cobs_send_fmt(COBS_CH_RESP, "PROFILE:%s,actuator=%s\r\n",
                      controller::profile_name(), controller::actuator_name());
        return;
    }
    char buffer[256];
    const int n = std::snprintf(buffer, sizeof(buffer), "%s", command);
    if (n > 0 && n < static_cast<int>(sizeof(buffer)))
        controller_profile_command(buffer, source);
}
