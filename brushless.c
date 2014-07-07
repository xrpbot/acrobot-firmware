#include "usart.h"
#include "statusled.h"
#include "ssi.h"
#include "serencode.h"
#include "trajectory.h"

#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/f4/adc.h>
#include <libopencm3/stm32/f4/timer.h>
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
    
    timer_reset(TIM8);
    timer_set_prescaler(TIM8, 1);
    timer_set_mode(TIM8, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_CENTER_1,
                   TIM_CR1_DIR_UP);
    timer_set_oc_mode(TIM8, TIM_OC1, TIM_OCM_PWM1);
    timer_set_oc_mode(TIM8, TIM_OC2, TIM_OCM_PWM1);
    timer_set_oc_mode(TIM8, TIM_OC3, TIM_OCM_PWM1);
    
    timer_set_oc_value(TIM8, TIM_OC1, 0);
    timer_set_oc_value(TIM8, TIM_OC2, 0);
    timer_set_oc_value(TIM8, TIM_OC3, 0);
    
    timer_set_deadtime(TIM8, 63);
    timer_set_enabled_off_state_in_run_mode(TIM8);
    
    timer_enable_oc_output(TIM8, TIM_OC1);
    timer_enable_oc_output(TIM8, TIM_OC1N);
    timer_enable_oc_output(TIM8, TIM_OC2);
    timer_enable_oc_output(TIM8, TIM_OC2N);
    timer_enable_oc_output(TIM8, TIM_OC3);
    timer_enable_oc_output(TIM8, TIM_OC3N);
    
    timer_set_oc_polarity_high(TIM8, TIM_OC1N);
    
    timer_set_period(TIM8, 1<<11);
    timer_enable_break_main_output(TIM8);
    
    timer_enable_counter(TIM8);
}

void commutate(unsigned int com_phase, unsigned int pwm)
{
    static unsigned int current_com_phase = -1;
    
    if(com_phase != current_com_phase) {
        /* Switch off all outputs */
        timer_disable_oc_output(TIM8, TIM_OC1N);
        timer_disable_oc_output(TIM8, TIM_OC2N);
        timer_disable_oc_output(TIM8, TIM_OC3N);
        
        timer_set_oc_value(TIM8, TIM_OC1, 0);
        timer_set_oc_value(TIM8, TIM_OC2, 0);
        timer_set_oc_value(TIM8, TIM_OC3, 0);
        
        switch(com_phase) {
            case 0: /* PWM, floating, GND */
                timer_set_oc_value(TIM8, TIM_OC1, pwm);
                timer_enable_oc_output(TIM8, TIM_OC1N);
                timer_enable_oc_output(TIM8, TIM_OC3N);
            break;
            case 1: /* floating, PWM, GND */
                timer_set_oc_value(TIM8, TIM_OC2, pwm);
                timer_enable_oc_output(TIM8, TIM_OC2N);
                timer_enable_oc_output(TIM8, TIM_OC3N);
            break;
            case 2: /* GND, PWM, floating */
                timer_enable_oc_output(TIM8, TIM_OC1N);
                timer_set_oc_value(TIM8, TIM_OC2, pwm);
                timer_enable_oc_output(TIM8, TIM_OC2N);
            break;
            case 3: /* GND, floating, PWM */
                timer_enable_oc_output(TIM8, TIM_OC1N);
                timer_set_oc_value(TIM8, TIM_OC3, pwm);
                timer_enable_oc_output(TIM8, TIM_OC3N);
            break;
            case 4: /* floating, GND, PWM */
                timer_enable_oc_output(TIM8, TIM_OC2N);
                timer_set_oc_value(TIM8, TIM_OC3, pwm);
                timer_enable_oc_output(TIM8, TIM_OC3N);
            break;
            case 5: /* PWM, GND, floating */
                timer_set_oc_value(TIM8, TIM_OC1, pwm);
                timer_enable_oc_output(TIM8, TIM_OC1N);
                timer_enable_oc_output(TIM8, TIM_OC2N);
            break;
        }
        
        current_com_phase = com_phase;
    } else {
        switch(com_phase) {
            case 0:
            case 5:
                timer_set_oc_value(TIM8, TIM_OC1, pwm);
            break;
            case 1:
            case 2:
                timer_set_oc_value(TIM8, TIM_OC2, pwm);
            break;
            case 3:
            case 4:
                timer_set_oc_value(TIM8, TIM_OC3, pwm);
        }
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

struct __attribute__ ((__packed__)) ctrl_msg {
    uint8_t enable;
    int16_t comm_offset_plus;
    int16_t comm_offset_minus;
    int16_t power;
    uint8_t control;
    int32_t amplitude;
    int32_t offset;
    int32_t omega;
    int32_t p_gain;
    int32_t d_gain;
    int32_t ff_gain;
};

int main(void) {
    rcc_clock_setup_hse_3v3(&hse_8mhz_3v3[CLOCK_3V3_168MHZ]);
    
    gpio_setup();

    //led_setup();
    usart_setup();
    
    // setup_adc();
    setup_pwm();
    
    setup_timer();
    
    ssi_setup();
    
    /* while(1) {
        adc_start_conversion_regular(ADC1);
        int val = 0;
        
        while (! adc_eoc(ADC1));
        
        val = adc_read_regular(ADC1);
        
        // timer_set_oc_value(TIM8, TIM_OC1, val/8);
        printf("%d\n", val);
    } */
    
    /* while(1) {
        as5xxx_data_t data = ssi_read();
        //printf("%4d    %d %d %d %d\n", data.position, data.offset_comp_finished,
        //data.linearity_alert, data.mag_inc, data.mag_dec);
        
        while(!usart_dma_tx_complete);
        se_start_frame(8);
        se_puti16(data.position);
        se_puti16(2*data.mag_inc + data.mag_dec);
        se_puti16(0);
        se_puti16(0);
        se_send_frame();
    } */
    
    int last_pos = 0;
    int pos_offset = 0;
    int last_center_pos = 0;
    int center_pos_offset = 0;
    int last_lower_pos = 0;
    int lower_pos_offset = 0;
    
    int pwr = 0;
    // int comm_offset = 0;
    
    uint32_t next_timer = 400;
    
    int data_sample = 0;
    
    float t = 0.;
    
    float A = 0.;
    float offset = 0.;
    float omega = 0.02;
    
    float p_gain = 1.;
    float d_gain = 0.;
    float ff_gain = 0.;
    
    int comm_offset_plus = 0;
    int comm_offset_minus = 0;
    int set_power = 0;
    
    int enable = 0;
    int control = 0;
    
     /* Enable counter. */
    timer_enable_counter(TIM1);
    
    while(1) {
        if(next_timer >= (1<<16)) {
            while(timer_get_counter(TIM1) > 50000);
            next_timer %= (1 << 16);
        }
        while(timer_get_counter(TIM1) < next_timer);
        gpio_toggle(GPIOB, GPIO1);
        
        next_timer += 400;
        
        /* adc_start_conversion_regular(ADC1);
        
        while (! adc_eoc(ADC1));
        int val = adc_read_regular(ADC1); */
        
        if(msg_ready) {
            if(msg_len == sizeof(struct ctrl_msg)) {
                struct ctrl_msg* msg = (struct ctrl_msg*) msg_buf;
                enable = msg->enable;
                comm_offset_plus = msg->comm_offset_plus;
                comm_offset_minus = msg->comm_offset_minus;
                set_power = msg->power;
                
                if(msg->control && !control)
                    t = 0.;
                
                control = msg->control;
                A = ((float)msg->amplitude / 65536);
                offset = ((float)msg->offset / 65536);
                omega = ((float)msg->omega / 65536);
                
                p_gain = ((float)msg->p_gain / 65536);
                d_gain = ((float)msg->d_gain / 65536);
                ff_gain = ((float)msg->ff_gain / 65536);
            }
            msg_ready = false;
        }
        
        /* int16_t next_pos = 98*phase;
        while(data.position < next_pos) {
            data = ssi_read();
            printf("%d %d\n", data.position, next_pos);
        }
        
        phase++;
        phase %= 42;  */
        
        as5xxx_data_t data = ssi_read_motor();
        if(!enable) {
            commutate(0, 0);
        } else if(pwr >= 0) {
            int geo_phase = (data.position + comm_offset_plus) / 98;
            int com_phase = geo_phase % 6;
            commutate(com_phase, pwr);
        } else {
            int geo_phase = (data.position + comm_offset_minus) / 98;
            int com_phase = geo_phase % 6;
            commutate(com_phase, -pwr);
        }
        
        /* Center encoder */
        as5xxx_data_t center_ring_enc = ssi_read_center_ring();
        
        int center_vel = center_ring_enc.position - last_center_pos;
        if(center_vel > 2048) {
            center_vel -= 4096;
            center_pos_offset -= 4096;
        }
        if(center_vel < -2048) {
            center_vel += 4096;
            center_pos_offset += 4096;
        }
        last_center_pos = center_ring_enc.position;
        
        /* Lower ring encoder */
        as5xxx_data_t lower_ring_enc = ssi_read_lower_ring();
        
        int lower_vel = lower_ring_enc.position - last_lower_pos;
        if(lower_vel > 2048) {
            lower_vel -= 4096;
            lower_pos_offset -= 4096;
        }
        if(lower_vel < -2048) {
            lower_vel += 4096;
            lower_pos_offset += 4096;
        }
        last_lower_pos = lower_ring_enc.position;
        
        data_sample++;
        if(data_sample >= 50) {
            int vel = data.position - last_pos;
            if(vel > 2048) {
                vel -= 4096;
                pos_offset -= 4096;
            }
            if(vel < -2048) {
                vel += 4096;
                pos_offset += 4096;
            }
            last_pos = data.position;
            int pos = data.position + pos_offset;
            
            float real_center_vel = center_vel / 128. / 4096. * 2 * M_PI / ((100.*400.)/168000000.);
            
            float real_pos = pos / 4096. * 2. * M_PI / 4.8;   // position in radians
            float real_vel = vel / 4096. * 2. * M_PI / 4.8 / ((100.*400.*50.)/168000000.);   // velocity in rad/s
            float des_pos, des_vel;
            if(control) {
                //des_pos = offset + A*sin(omega*t);
                //des_vel = A*omega*cos(omega*t);
                //float des_acc = -A*omega*omega*sin(omega*t);
                //des_pos = offset + A*get_des_pos(t);
                //des_vel = A*get_des_vel(t);
                //float des_acc = 0;
                des_pos = A * atan(real_center_vel) + offset;
                des_vel = 0.;
                float des_acc = 0.;
                
                float output = p_gain*(des_pos - real_pos) + d_gain*(des_vel - real_vel) + ff_gain*des_acc;
                // error_int += (des_vel - vel);
                
                /* if(error_int > 30000) error_int = 30000;
                if(error_int < -30000) error_int = -30000; */
                
                pwr = 256*output;
                
                if(pwr > 1000) pwr = 1000;
                if(pwr < -1000) pwr = -1000;
            } else {
                des_pos = 0;
                des_vel = 0;
                pwr = set_power;
            }
            
            as5xxx_data_t lower_axis_enc = ssi_read_lower_axis();
            
            data_sample = 0;
            se_start_frame(20);
            se_puti16((int16_t)(real_vel*256));
            se_puti16((int16_t)(des_vel*256));
            se_puti16((int16_t)(real_pos*256));
            se_puti16((int16_t)(des_pos*256));
            se_puti16(lower_axis_enc.position);
            se_puti32(lower_ring_enc.position + lower_pos_offset);
            se_puti32(center_ring_enc.position + center_pos_offset);
            se_puti16(center_vel);
            se_send_frame();
            
            t += (100.*400.*50.)/168000000.;
            t = fmod(omega*t, 2*M_PI) / omega;
        }
    }
}
