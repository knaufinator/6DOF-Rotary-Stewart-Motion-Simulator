#pragma once
// Deliberately small host substitute for the SDK surfaces used by the unchanged
// SharedMcpwmStepEngine. No production target includes this directory.
#include <cstdint>
#include <cstddef>
#define IRAM_ATTR
#define DRAM_ATTR
#define CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ 240
using esp_err_t=int;
using gpio_num_t=int;
using mcpwm_timer_handle_t=void*;
using mcpwm_oper_handle_t=void*;
using mcpwm_cmpr_handle_t=void*;
using mcpwm_gen_handle_t=void*;
using intr_handle_t=void*;
enum {ESP_OK=0,ESP_FAIL=-1,GPIO_NUM_NC=-1,GPIO_MODE_OUTPUT=2,
      GPIO_PULLUP_DISABLE=0,GPIO_PULLDOWN_DISABLE=0,GPIO_INTR_DISABLE=0,
      MCPWM_TIMER_CLK_SRC_DEFAULT=0,MCPWM_TIMER_COUNT_MODE_UP=1,
      MCPWM_TIMER_DIRECTION_UP=1,MCPWM_GEN_ACTION_HIGH=1,MCPWM_GEN_ACTION_LOW=0,
      MCPWM_TIMER_START_NO_STOP=1,MCPWM_TIMER_STOP_FULL=2,
      PERIPH_PWM0_MODULE=0,ESP_INTR_FLAG_IRAM=1,ESP_INTR_FLAG_LEVEL3=2};
struct gpio_config_t {uint64_t pin_bit_mask=0;int mode=0,pull_up_en=0,pull_down_en=0,intr_type=0;};
struct mcpwm_timer_config_t {
    int group_id=0,clk_src=0,count_mode=0,intr_priority=0;
    uint32_t resolution_hz=0,period_ticks=0;
    struct {bool update_period_on_empty=false;} flags;
};
struct mcpwm_operator_config_t {int group_id=0,intr_priority=0;struct{bool update_gen_action_on_tez=false;}flags;};
struct mcpwm_comparator_config_t {int intr_priority=0;struct{bool update_cmp_on_tez=false;}flags;};
struct mcpwm_generator_config_t {int gen_gpio_num=0;struct{int io_loop_back=0;}flags;};
struct mcpwm_gen_compare_event_action_t {int direction=0;void* comparator=nullptr;int action=0;};
namespace plant {
enum RegisterKind {GATE,GPIO_SET,GPIO_CLEAR};
void register_write(RegisterKind,int,uint32_t);
struct Register {
    RegisterKind kind;int index;uint32_t value=0;
    Register(RegisterKind k,int i):kind(k),index(i){}
    Register& operator=(uint32_t v){value=v;register_write(kind,index,v);return *this;}
    operator uint32_t() const{return value;}
};
struct Word {uint32_t val=0;};
struct Gate {Register val;explicit Gate(int op):val(GATE,op){}};
struct Operator {Gate gen_force;explicit Operator(int op):gen_force(op){}};
struct Mcpwm {Word int_clr,int_ena;Operator operators[3]{Operator(0),Operator(1),Operator(2)};};
struct GpioBank {Register val;GpioBank(RegisterKind k,int bank):val(k,bank){}};
struct Gpio {
    Register out_w1ts{GPIO_SET,0},out_w1tc{GPIO_CLEAR,0};
    GpioBank out1_w1ts{GPIO_SET,1},out1_w1tc{GPIO_CLEAR,1};
};
}
extern plant::Mcpwm MCPWM0;
extern plant::Gpio GPIO;
struct McpwmSignals {struct{int irq_id=0;}groups[1];};
extern McpwmSignals mcpwm_periph_signals;
int gpio_config(const gpio_config_t*);
int gpio_set_level(gpio_num_t,int);
int mcpwm_new_timer(const mcpwm_timer_config_t*,void**);
int mcpwm_new_operator(const mcpwm_operator_config_t*,void**);
int mcpwm_operator_connect_timer(void*,void*);
int mcpwm_new_comparator(void*,const mcpwm_comparator_config_t*,void**);
int mcpwm_comparator_set_compare_value(void*,uint32_t);
int mcpwm_new_generator(void*,const mcpwm_generator_config_t*,void**);
int mcpwm_generator_set_action_on_compare_event(void*,mcpwm_gen_compare_event_action_t);
int mcpwm_timer_enable(void*);
int mcpwm_timer_disable(void*);
int mcpwm_timer_start_stop(void*,int);
int mcpwm_timer_set_period(void*,uint32_t);
int mcpwm_del_generator(void*);
int mcpwm_del_comparator(void*);
int mcpwm_del_operator(void*);
int mcpwm_del_timer(void*);
int esp_intr_alloc(int,int,void(*)(void*),void*,void**);
int esp_intr_free(void*);
void periph_module_reset(int);
uint32_t esp_cpu_get_cycle_count();
void esp_rom_delay_us(uint32_t);
