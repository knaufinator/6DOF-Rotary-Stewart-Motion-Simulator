#pragma once
#include <cstddef>
struct usb_serial_jtag_driver_config_t { size_t rx_buffer_size=0,tx_buffer_size=0; };
#define USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT() usb_serial_jtag_driver_config_t{}
inline int usb_serial_jtag_driver_install(const usb_serial_jtag_driver_config_t*) { return 0; }
bool usb_serial_jtag_is_connected();
int usb_serial_jtag_write_bytes(const void*,size_t,unsigned);
