#pragma once
#include <cstdint>
#include <cstddef>
using i2c_master_bus_handle_t = void*;
using i2c_master_dev_handle_t = void*;
enum { ESP_OK=0, ESP_FAIL=-1, I2C_NUM_0=0, I2C_CLK_SRC_DEFAULT=0, I2C_ADDR_BIT_LEN_7=7 };
struct i2c_master_bus_config_t { int i2c_port=0,sda_io_num=0,scl_io_num=0,clk_source=0,glitch_ignore_cnt=0; };
struct i2c_device_config_t { int dev_addr_length=0,scl_speed_hz=0,device_address=0; };
int i2c_new_master_bus(const i2c_master_bus_config_t*, i2c_master_bus_handle_t*);
int i2c_master_bus_add_device(i2c_master_bus_handle_t, const i2c_device_config_t*, i2c_master_dev_handle_t*);
int i2c_master_transmit(i2c_master_dev_handle_t,const uint8_t*,size_t,int);
int i2c_master_transmit_receive(i2c_master_dev_handle_t,const uint8_t*,size_t,uint8_t*,size_t,int);
