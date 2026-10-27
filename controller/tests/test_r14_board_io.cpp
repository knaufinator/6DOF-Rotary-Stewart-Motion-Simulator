#include "test_framework.h"
#include "R14BoardIo.h"
#include <cstring>

class FakeBoardBus : public r14::RegisterBus {
public:
    uint8_t mcp[256] = {};
    uint8_t adc[256][2] = {};
    bool failed = false;
    int arm_edges = 0;
    FakeBoardBus() {
        adc[0xfe][0] = 0x54; adc[0xfe][1] = 0x49;
        adc[0xff][0] = 0x22; adc[0xff][1] = 0x60;
        adc[0x06][1] = 0x08;
        adc[0x02][0] = 0x0f; adc[0x02][1] = 0xa0; // 4000 * 1.25 mV
    }
    bool write(uint8_t address,uint8_t reg,const uint8_t* d,size_t n) override {
        if (failed) return false;
        if (address == 0x20) {
            if (reg == 0x15 && (d[0] & 0x80) && !(mcp[reg] & 0x80)) ++arm_edges;
            std::memcpy(mcp+reg,d,n);
        } else if (address == 0x40) std::memcpy(adc[reg],d,n);
        else return false;
        return true;
    }
    bool read(uint8_t address,uint8_t reg,uint8_t* d,size_t n) override {
        if (failed) return false;
        if (address == 0x20) std::memcpy(d,mcp+reg,n);
        else if (address == 0x40) std::memcpy(d,adc[reg],n);
        else return false;
        return true;
    }
};

TEST(r14_io_boot_clears_retained_outputs_without_an_arm_edge) {
    FakeBoardBus b; b.mcp[0x14] = 0x80; b.mcp[0x15] = 0x82;
    r14::BoardIo io(b);
    ASSERT(!io.set_field_feed(true));
    ASSERT(io.initialize());
    ASSERT_EQ(b.mcp[0x00],0x7f);
    ASSERT_EQ(b.mcp[0x01],0x7d);
    ASSERT_EQ(b.mcp[0x14],0);
    ASSERT_EQ(b.mcp[0x15],0);
    ASSERT_EQ(b.arm_edges,0);
    ASSERT(io.set_field_feed(true));
    ASSERT_EQ(b.mcp[0x15],0x02); // field only; arm clock LOW
    ASSERT_EQ(b.mcp[0x14],0);    // torque clear remains asserted
    ASSERT(io.set_field_feed(false));
    ASSERT_EQ(b.mcp[0x15],0);
    ASSERT_EQ(b.arm_edges,0);
}
TEST(r14_io_decodes_actual_nc_and_readback_polarities) {
    FakeBoardBus b; r14::BoardIo io(b); ASSERT(io.initialize());
    b.mcp[0x12] = 0x40 | 0x12; // field good, Hall1/4 open
    b.mcp[0x13] = 0x70;
    auto in=io.sample(100,0x21);
    ASSERT(in.expander_ok && in.adc_ok && in.fresh(150,50));
    ASSERT_EQ(in.hall_closed_mask,0x2d);
    ASSERT_EQ(in.alarm_mask,0x21);
    ASSERT_EQ(in.driver_5v_mv,5000);
    ASSERT(in.field_good && !in.field_fault && in.pulse_enable && in.torque_q_all);
    ASSERT(!in.fresh(151,50));
    ASSERT(!in.fresh(99,50));
}
TEST(r14_io_missing_device_stale_adc_or_bus_failure_never_proves_health) {
    FakeBoardBus wrong; wrong.adc[0xfe][0]=0;
    r14::BoardIo no_adc(wrong); ASSERT(!no_adc.initialize());
    ASSERT(!no_adc.sample(1,0).fresh(1,50));
    FakeBoardBus b; r14::BoardIo io(b); ASSERT(io.initialize());
    b.adc[6][1]=0;
    ASSERT(!io.sample(2,0).fresh(2,50));
    b.failed=true;
    auto in=io.sample(3,0x3f);
    ASSERT(!in.expander_ok && !in.adc_ok && in.field_fault);
    ASSERT_EQ(in.hall_closed_mask,0);
    ASSERT(!io.set_field_feed(true));
}

void register_r14_board_io_tests() {
    RUN_TEST(r14_io_boot_clears_retained_outputs_without_an_arm_edge);
    RUN_TEST(r14_io_decodes_actual_nc_and_readback_polarities);
    RUN_TEST(r14_io_missing_device_stale_adc_or_bus_failure_never_proves_health);
}
