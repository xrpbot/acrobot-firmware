#include "usart.h"
#include "statusled.h"
#include "ssi.h"
#include "sensors.h"
#include "serencode.h"
#include "trajectory.h"
#include "pwm.h"
#include "quadrature.h"
#include "adc.h"
#include "transport.h"
#include "current.h"
#include "util.h"
#include "vel_ctrl.h"
#include "pumping_ctrl.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/crc.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/scb.h>

#include <libopencm3/stm32/usart.h>

#include <stdio.h>
#include <unistd.h>
#include <math.h>

void gpio_setup(void);
void lb_setup(void);

void gpio_setup(void)
{
    rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPBEN);
    
    /* Debug out */
    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN, GPIO1 | GPIO2);
}

void lb_setup(void)
{
    rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPAEN);
    gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO8);
}

volatile int holdoff = 10;

void tim8_up_tim13_isr(void)
{
    if(timer_get_flag(TIM8, TIM_SR_UIF)) {
        gpio_set(GPIOB, GPIO1);
        holdoff--;
        
        current_ctrl();
        
        gpio_clear(GPIOB, GPIO1);
        timer_clear_flag(TIM8, TIM_SR_UIF);
    }
}

struct __attribute__ ((__packed__)) ctrl_msg {
    int32_t gain;
    int32_t offset;
    int32_t scale;
};

int main(void) {
    rcc_clock_setup_hse_3v3(&hse_8mhz_3v3[CLOCK_3V3_168MHZ]);
    
    gpio_setup();
    usart_setup();
    pwm_setup();
    adc_setup();
    quadrature_setup_tim3();
    ssi_setup();
    lb_setup();
    
    sensors_init();
    
    set_pwm_values(0, 0);
    
    ssi_read_motor();  // dummy read (FIXME)
    as5xxx_data_t mot_data = ssi_read_motor();
    quadrature_init_tim3(mot_data.position / 4);
    
    float gain = 0.f;
    float offset = 0.f;
    float scale = 0.f;
    
    while(1) {
        if(tp_check_msg()) {
            if(msg_data[0] == 0x42) {
                scb_reset_system();
            } else if(msg_data[0] == 0x01) {
                struct ctrl_msg* msg = (struct ctrl_msg*) &msg_data[1];
                gain = 0.01f * ((float) msg->gain);
                offset = 0.01f * ((float) msg->offset);
                scale = 0.01f * ((float) msg->scale);
            }
            tp_msg_done();
        }
        
        gpio_set(GPIOB, GPIO2);
        while(holdoff > 0);
        holdoff = 10;
        gpio_clear(GPIOB, GPIO2);
        
        sensors_update();
        
        pumping_set_params(gain, offset, scale);
        float Iq_des = pumping_ctrl_step(pos_active, vel_active, vel_passive);
        
        Iq_des = clampf(Iq_des, -300.f, 300.f);
        
        set_des_current(Iq_des, 0.f);
        
        se_start_frame(16);
        se_putf32(pos_active);
        se_putf32(pos_passive);
        se_putf32(vel_active);
        se_putf32(vel_passive);
        se_send_frame();
    }
}
