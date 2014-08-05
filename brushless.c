#include "usart.h"
#include "statusled.h"
#include "ssi.h"
#include "serencode.h"
#include "trajectory.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/f4/nvic.h>
#include <libopencm3/cm3/systick.h>

#include <libopencm3/stm32/usart.h>

#include <stdio.h>
#include <unistd.h>
#include <math.h>

void gpio_setup(void);
void delay(uint32_t tick);
void setup_timer(void);
void setup_pwm(void);
void commutate(unsigned int com_phase, unsigned int pwm);
void set_pwm_values(int32_t ua, int32_t ub);

void gpio_setup(void) {
    rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPBEN);
    
    /* Debug out */
    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN, GPIO1 | GPIO2);
}

void delay(uint32_t ms) {
    for (uint32_t i = 0; i <= ms*168000; i++)
    {
        __asm("nop");
    }
}

void setup_quad_decode(void)
{
    rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPBEN);
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO4);
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO5);
    gpio_set_af(GPIOB, GPIO_AF2, GPIO4);
    gpio_set_af(GPIOB, GPIO_AF2, GPIO5);
    
    rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM3EN);
    timer_reset(TIM3);
    timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_ic_set_input(TIM3, TIM_IC1, TIM_IC_IN_TI1);
    timer_ic_set_input(TIM3, TIM_IC2, TIM_IC_IN_TI2);
    // timer_slave_set_trigger(TIM2, TIM_SMCR_TS_IT1FP1);
    timer_slave_set_mode(TIM3, TIM_SMCR_SMS_EM3);
    timer_set_period(TIM3, 1023);
    // timer_enable_counter(TIM2);
}

void setup_timer(void)
{
    /* Enable timer clock. */
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_TIM1EN);
    
    /* Reset timer. */
    timer_reset(TIM1);
    
    /* Configure prescaler. */
    timer_set_prescaler(TIM1, 100);
    
    /* Enable IRQs */
    //nvic_enable_irq(NVIC_TIM1_UP_TIM10_IRQ);
    //timer_enable_irq(TIM1, TIM_DIER_UIE);
}

void setup_pwm(void)
{
    // clock_scale_t clock_setup = hse_8mhz_3v3[CLOCK_3V3_168MHZ];
    // int apb2_clock = clock_setup.apb2_frequency;
    
    rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPAEN);
    rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPBEN);
    rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPCEN);
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_TIM8EN);
    
    /* PC6 -> TIM8_CH1 */
    gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6);
    gpio_set_output_options(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO6);
    gpio_set_af(GPIOC, 3, GPIO6);
    
    /* PA7 -> TIM8_CH1N */
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO7);
    gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO7);
    gpio_set_af(GPIOA, 3, GPIO7);
    
    /* PC7 -> TIM8_CH2 */
    gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO7);
    gpio_set_output_options(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO7);
    gpio_set_af(GPIOC, 3, GPIO7);
    
    /* PB14 -> TIM8_CH2N */
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO14);
    gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO14);
    gpio_set_af(GPIOB, 3, GPIO14);
    
    /* PC8 -> TIM8_CH3 */
    gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8);
    gpio_set_output_options(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO8);
    gpio_set_af(GPIOC, 3, GPIO8);
    
    /* PB15 -> TIM8_CH3N */
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO15);
    gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO15);
    gpio_set_af(GPIOB, 3, GPIO15);
    
    /* PC9 -> TIM8_CH4 (debug only) */
    gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);
    gpio_set_output_options(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO9);
    gpio_set_af(GPIOC, 3, GPIO9);
    
    timer_reset(TIM8);
    timer_set_prescaler(TIM8, 1);  // set prescaler to 2
    timer_set_mode(TIM8, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_CENTER_1,
                   TIM_CR1_DIR_UP);
    timer_set_oc_mode(TIM8, TIM_OC1, TIM_OCM_PWM1);
    timer_set_oc_mode(TIM8, TIM_OC2, TIM_OCM_PWM1);
    timer_set_oc_mode(TIM8, TIM_OC3, TIM_OCM_PWM1);
    
    timer_set_oc_value(TIM8, TIM_OC1, 0);
    timer_set_oc_value(TIM8, TIM_OC2, 0);
    timer_set_oc_value(TIM8, TIM_OC3, 0);
    
    timer_set_deadtime(TIM8, 150);
    
    // This is the default, but set it anyway.
    timer_set_oc_polarity_high(TIM8, TIM_OC1);
    timer_set_oc_polarity_high(TIM8, TIM_OC1N);
    timer_set_oc_polarity_high(TIM8, TIM_OC2);
    timer_set_oc_polarity_high(TIM8, TIM_OC2N);
    timer_set_oc_polarity_high(TIM8, TIM_OC3);
    timer_set_oc_polarity_high(TIM8, TIM_OC3N);
    
    timer_enable_oc_output(TIM8, TIM_OC1);
    timer_enable_oc_output(TIM8, TIM_OC1N);
    timer_enable_oc_output(TIM8, TIM_OC2);
    timer_enable_oc_output(TIM8, TIM_OC2N);
    timer_enable_oc_output(TIM8, TIM_OC3);
    timer_enable_oc_output(TIM8, TIM_OC3N);
    
    /* OC4 */
    timer_set_oc_mode(TIM8, TIM_OC4, TIM_OCM_TOGGLE);
    timer_set_oc_value(TIM8, TIM_OC4, 1);
    timer_enable_oc_output(TIM8, TIM_OC4);
    
    timer_set_period(TIM8, 2048);
    timer_enable_break_main_output(TIM8);
    
    nvic_enable_irq(NVIC_TIM8_UP_TIM13_IRQ);
    timer_enable_irq(TIM8, TIM_DIER_UIE);
    
    timer_enable_counter(TIM8);
}

volatile int holdoff = 1;
volatile int32_t svm_mag = 0;

void tim8_up_tim13_isr(void)
{
    if(timer_get_flag(TIM8, TIM_SR_UIF)) {
        timer_clear_flag(TIM8, TIM_SR_UIF);
        gpio_set(GPIOB, GPIO1);
        holdoff = 0;
        /* float pos = timer_get_counter(TIM3);
        float phi = pos / 512. * 7. * M_PI;
        
        float magf = (float) svm_mag;
        
        int32_t ua, ub;
        ua = magf * cosf(phi);
        ub = magf * sinf(phi); */
        
        // set_pwm_values(ua, ub);
        set_pwm_values(svm_mag, 0);
        
        gpio_clear(GPIOB, GPIO1);
    }
}

/* void setup_adc() {
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1);
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_ADC1EN);

    adc_set_clk_prescale(ADC_CCR_ADCPRE_BY2);
    adc_disable_scan_mode(ADC1);
    adc_set_single_conversion_mode(ADC1);
    adc_set_sample_time(ADC1, ADC_CHANNEL1, ADC_SMPR_SMP_112CYC);

    uint8_t channels[] = {ADC_CHANNEL1};
    adc_set_regular_sequence(ADC1, 1, channels);
    adc_set_multi_mode(ADC_CCR_MULTI_INDEPENDENT);
    adc_power_on(ADC1);
} */

void set_pwm_values(int32_t ua, int32_t ub)
{
    // Determine sector
    int sector;
    
    if(ub > 0) {
        if((1774*ua + 1024*ub) > 0) {
            if((1774*ua - 1024*ub) > 0) {
                sector = 1;
            } else {
                sector = 2;
            }
        } else {
            sector = 3;
        }
    } else {
        if((1774*ua + 1024*ub) > 0) {
            sector = 6;
        } else {
            if((1774*ua - 1024*ub) > 0) {
                sector = 5;
            } else {
                sector = 4;
            }
        }
    }
    
    int tu, tv, tw;
    
    switch(sector) {
        case 1: {
            int t1 = (768*ua - 443*ub)/1024;
            int t2 = (887*ub)/1024;
            int t0 = 1024 - t1 - t2;
            
            tu = t0/2;
            tv = t0/2+t1;
            tw = t0/2+t1+t2;
        } break;
        case 2: {
            int t2 = (768*ua + 443*ub)/1024;
            int t3 = (-768*ua + 443*ub)/1024;
            int t0 = 1024 - t2 - t3;
            
            tv = t0/2;
            tu = t0/2+t3;
            tw = t0/2+t3+t2;
        } break;
        case 3: {
            int t4 = (-768*ua - 443*ub)/1024;
            int t3 = (887*ub)/1024;
            int t0 = 1024 - t3 - t4;
            
            tv = t0/2;
            tw = t0/2+t3;
            tu = t0/2+t3+t4;
        } break;
        case 4: {
            int t4 = (-768*ua + 443*ub)/1024;
            int t5 = (-887*ub)/1024;
            int t0 = 1024 - t4 - t5;
            
            tw = t0/2;
            tv = t0/2+t5;
            tu = t0/2+t5+t4;
        } break;
        case 5: {
            int t6 = (768*ua - 443*ub)/1024;
            int t5 = (-768*ua - 443*ub)/1024;
            int t0 = 1024 - t5 - t6;
            
            tw = t0/2;
            tu = t0/2+t5;
            tv = t0/2+t5+t6;
        } break;
        case 6: {
            int t1 = (768*ua + 443*ub)/1024;
            int t6 = (-887*ub)/1024;
            int t0 = 1024 - t1 - t6;
            
            tu = t0/2;
            tw = t0/2+t1;
            tv = t0/2+t1+t6;
        } break;
    }
    
    timer_set_oc_value(TIM8, TIM_OC1, tu);
    timer_set_oc_value(TIM8, TIM_OC2, tv);
    timer_set_oc_value(TIM8, TIM_OC3, tw);
}

struct __attribute__ ((__packed__)) ctrl_msg {
    int32_t mag;
    int32_t phase;
};

void setup_adc() {
    gpio_mode_setup(GPIOC, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1);   // CHANNEL_11
    gpio_mode_setup(GPIOC, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO2);   // CHANNEL_12
    
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_ADC1EN);
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_ADC2EN);

    adc_set_clk_prescale(ADC_CCR_ADCPRE_BY2);
    adc_disable_scan_mode(ADC1);
    adc_disable_scan_mode(ADC2);
    adc_set_single_conversion_mode(ADC1);
    adc_set_single_conversion_mode(ADC2);
    
    adc_set_sample_time(ADC1, ADC_CHANNEL11, ADC_SMPR_SMP_112CYC);
    adc_set_sample_time(ADC1, ADC_CHANNEL12, ADC_SMPR_SMP_112CYC);

    uint8_t channels1[] = { ADC_CHANNEL11 };
    uint8_t channels2[] = { ADC_CHANNEL12 };
    adc_set_injected_sequence(ADC1, 1, channels1);
    adc_set_injected_sequence(ADC2, 1, channels2);
    
    adc_enable_external_trigger_injected(ADC1, ADC_CR2_JEXTSEL_TIM8_CC4, ADC_CR2_JEXTEN_RISING_EDGE);
    
    // adc_set_multi_mode(ADC_CCR_MULTI_INDEPENDENT);
    // adc_set_multi_mode(ADC_CCR_MULTI_DUAL_REGULAR_SIMUL);
    adc_set_multi_mode(ADC_CCR_MULTI_DUAL_INJECTED_SIMUL);
    
    adc_power_on(ADC1);
    adc_power_on(ADC2);
}

int main(void) {
    rcc_clock_setup_hse_3v3(&hse_8mhz_3v3[CLOCK_3V3_168MHZ]);
    
    gpio_setup();

    usart_setup();
    
    setup_pwm();
    
    setup_adc();
    
    setup_quad_decode();
    
    //setup_timer();
    
    ssi_setup();
    
    set_pwm_values(0, 0);
    
    float mag = 0.;
    float offset = 0.;
    
    /* while(1) {
        as5xxx_data_t data = ssi_read_motor();
        printf("%4d    %d %d %d %d\n", data.position, data.offset_comp_finished,
        data.linearity_alert, data.mag_inc, data.mag_dec);
    } */
    
    /* volatile int i;
    while(1) {
        adc_start_conversion_injected(ADC1);
        while (! adc_eoc_injected(ADC1));
        
        se_start_frame(4);
        se_puti16(adc_read_injected(ADC1, 1));
        se_puti16(adc_read_injected(ADC2, 1));
        se_send_frame();
        for(i = 0; i<10000; i++);
    } */
    
    ssi_read_motor();  // dummy read (FIXME)
    as5xxx_data_t mot_data = ssi_read_motor();
    timer_set_counter(TIM3, mot_data.position / 4);
    timer_enable_counter(TIM3);
    
    while(1) {
        if(msg_ready) {
            if(msg_len == sizeof(struct ctrl_msg)) {
                struct ctrl_msg* msg = (struct ctrl_msg*) msg_buf;
                
                mag = (float) msg->mag;
                offset = (float) msg->phase / 1024.;
                svm_mag = 100;
            }
            msg_ready = false;
        }
        
        as5xxx_data_t data = ssi_read_motor();
        float pos = timer_get_counter(TIM3);
        // float phi = (float) data.position / 2048. * 7. * M_PI + offset;
        float phi = pos / 512. * 7. * M_PI + offset;
        
        /* int32_t ua, ub;
        ua = mag * cosf(phi);
        ub = mag * sinf(phi);
        
        set_pwm_values(ua, ub); */
        
        //adc_start_conversion_injected(ADC1);
        //while (!(adc_eoc_injected(ADC1) && adc_eoc_injected(ADC2)));
        
        float I1 = adc_read_injected(ADC1, 1);
        float I2 = adc_read_injected(ADC2, 1);
        //I1 -= 2047;
        //I2 -= 2051;
        I1 -= 2050.;
        I2 -= 2064.;
        
        float Ia = I1;
        float Ib = (I1 + 2.*I2)/sqrt(3.);
        
        float Ix = cos(phi)*Ia - sin(phi)*Ib;
        float Iy = sin(phi)*Ia + cos(phi)*Ib;
        
        se_start_frame(8);
        se_puti16((uint16_t)(Ix + 4096.));
        se_puti16((uint16_t)(Iy + 4096.));
        //se_puti16((uint16_t)(I1 + 4096.));
        //se_puti16((uint16_t)(I2 + 4096.));
        se_puti16((uint16_t)(phi / M_PI * 1024.));
        // se_puti16(data.position/4 - timer_get_counter(TIM3) + 16);
        se_puti16((uint16_t)(pos));
        se_send_frame();
        
        delay(1);
    }
}
