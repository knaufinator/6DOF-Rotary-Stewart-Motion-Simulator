#include "automation.h"
#include "serial_port.h"
#include "udp_transport.h"

#include <cstdio>
#include <cstdlib>

int main() {
    int failures = 0;
    auto check = [&](bool ok, const char* name) {
        std::printf("%s %s\n", ok ? "PASS" : "FAIL", name);
        if (!ok) ++failures;
    };
    // CTest sets this before the process starts. Abort rather than testing I/O
    // if the immutable guard was not enabled by the launcher.
    if (!IsDocumentationMode()) {
        std::fprintf(stderr, "Documentation mode required; no transport tests attempted\n");
        return 2;
    }
    _putenv_s("STEWART_DOCUMENTATION_MODE", "0");
    check(IsDocumentationMode(), "documentation policy cannot be disabled at runtime");
    check(SerialPort::enumerate().empty(), "no hardware port enumeration");
    SerialPort serial;
    check(!serial.open("COM_DOCS_TEST_ONLY", 115200), "serial open rejected before device access");
    check(!serial.isOpen(), "serial remains closed");
    UdpTransport network("invalid.docs.test", 8767, 8789);
    check(!network.isOpen(), "network transport remains closed without DNS/socket creation");
    check(!network.controlConnected(), "bridge control transport remains disconnected");
    check(!network.sendCommand("VERSION?"), "network command cannot send");
    check(network.txBytes() == 0 && serial.txBytes() == 0, "no transmitted bytes");
    return failures ? 1 : 0;
}
