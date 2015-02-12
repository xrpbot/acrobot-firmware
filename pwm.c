#include "pwm.h"
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>

/*
    We use TIM8 as the main PWM engine. The internal clock for TIM8
    is at 168 MHz. We set prescaler=1 and period=4096, giving a timer
    period of 24.38 µs (41 kHz).
    
    Note that the timer is configured in alternating up/downcounting
    mode, so the real period is twice that.
*/

void pwm_setup(void)
{
    // Enable clocks for GPIOs and timer
    rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPAEN);
    rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPBEN);
    rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPCEN);
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_TIM8EN);
    
    // * Set up GPIOs *
    // PC6 -> TIM8_CH1
    gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6);
    gpio_set_output_options(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO6);
    gpio_set_af(GPIOC, 3, GPIO6);
    
    // PA7 -> TIM8_CH1N
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO7);
    gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO7);
    gpio_set_af(GPIOA, 3, GPIO7);
    
    // PC7 -> TIM8_CH2
    gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO7);
    gpio_set_output_options(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO7);
    gpio_set_af(GPIOC, 3, GPIO7);
    
    // PB14 -> TIM8_CH2N
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO14);
    gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO14);
    gpio_set_af(GPIOB, 3, GPIO14);
    
    // PC8 -> TIM8_CH3
    gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8);
    gpio_set_output_options(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO8);
    gpio_set_af(GPIOC, 3, GPIO8);
    
    // PB15 -> TIM8_CH3N
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO15);
    gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO15);
    gpio_set_af(GPIOB, 3, GPIO15);
    
    // PC9 -> TIM8_CH4 (debug only)
    gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);
    gpio_set_output_options(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO9);
    gpio_set_af(GPIOC, 3, GPIO9);
    
    timer_reset(TIM8);
    timer_set_prescaler(TIM8, 0);  // set prescaler to 1
    timer_set_mode(TIM8, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_CENTER_1,
                   TIM_CR1_DIR_UP);
    
    // Setup OC polarity (this is the default, but set it anyway)
    timer_set_oc_polarity_high(TIM8, TIM_OC1);
    timer_set_oc_polarity_high(TIM8, TIM_OC1N);
    timer_set_oc_polarity_high(TIM8, TIM_OC2);
    timer_set_oc_polarity_high(TIM8, TIM_OC2N);
    timer_set_oc_polarity_high(TIM8, TIM_OC3);
    timer_set_oc_polarity_high(TIM8, TIM_OC3N);
    
    // Setup OC mode (PWM1)
    timer_set_oc_mode(TIM8, TIM_OC1, TIM_OCM_PWM1);
    timer_set_oc_mode(TIM8, TIM_OC2, TIM_OCM_PWM1);
    timer_set_oc_mode(TIM8, TIM_OC3, TIM_OCM_PWM1);
    
    // Set deadtime to 168 counts, i.e. 1 µs.
    // NOTE: the argument to timer_set_deadtime is NOT in counts!
    // (see documentation)
    timer_set_deadtime(TIM8, 0x94);
    
    // Set all channels to zero
    timer_set_oc_value(TIM8, TIM_OC1, 0);
    timer_set_oc_value(TIM8, TIM_OC2, 0);
    timer_set_oc_value(TIM8, TIM_OC3, 0);
    
    // Enable outputs
    timer_enable_oc_output(TIM8, TIM_OC1);
    timer_enable_oc_output(TIM8, TIM_OC1N);
    timer_enable_oc_output(TIM8, TIM_OC2);
    timer_enable_oc_output(TIM8, TIM_OC2N);
    timer_enable_oc_output(TIM8, TIM_OC3);
    timer_enable_oc_output(TIM8, TIM_OC3N);
    
    // OC4 (debug only)
    timer_set_oc_mode(TIM8, TIM_OC4, TIM_OCM_TOGGLE);
    timer_set_oc_value(TIM8, TIM_OC4, 1);
    timer_enable_oc_output(TIM8, TIM_OC4);
    
    timer_set_period(TIM8, 4096);
    timer_enable_break_main_output(TIM8);
    
    nvic_set_priority(NVIC_TIM8_UP_TIM13_IRQ, 16);
    nvic_enable_irq(NVIC_TIM8_UP_TIM13_IRQ);
    timer_enable_irq(TIM8, TIM_DIER_UIE);
    
    timer_enable_counter(TIM8);
}

/* 
  uu = A/2 * cos(phi)
  uv = A/2 * cos(phi - 2/3*pi)
  uw = A/2 * cos(phi + 2/3*pi)
  
  ux = A*cos(phi)
  uy = A*sin(phi)
  
  The maximum value for A is K_1 * 2/sqrt(3) = 4729 .
*/
void set_pwm_values(int32_t ux, int32_t uy)
{
    const int32_t K_1       = 4096;  // 100% PWM
    const int32_t K_1_4     = 1024;  // 1/4 * K_1
    const int32_t K_SQRT3_4 = 1774;  // sqrt(3)/4 * K_1
    
    // Use cosine addition theorem to calculate u,v,w from ux and uy:
    // cos(phi + dphi) = cos(dphi) cos(phi) - sin(dphi) sin(phi)
    //                 = cos(dphi) ux - sin(dphi) uy
    // where cos(2/3*pi) = -1/2 and sin(2/3*pi) = sqrt(3)/2 .
    int32_t u = ux/2;
    int32_t v = (-K_1_4*ux + K_SQRT3_4*uy)/K_1;
    int32_t w = (-K_1_4*ux - K_SQRT3_4*uy)/K_1;
    
    // u_min = min(u, v, w);
    int32_t u_min = (u < v ? u : v);
    u_min = (u_min < w ? u_min : w);
    
    // u_max = max(u, v, w);
    int32_t u_max = (u > v ? u : v);
    u_max = (u_max > w ? u_max : w);
    
    // Shift all voltages such that the average between minimum and maximum
    // is at the middle of the PWM range, thus maximizing the possible
    // amplitude.
    int32_t u0 = (K_1 - u_min - u_max)/2;
    u += u0;
    v += u0;
    w += u0;
    
    timer_set_oc_value(TIM8, TIM_OC1, u);
    timer_set_oc_value(TIM8, TIM_OC2, v);
    timer_set_oc_value(TIM8, TIM_OC3, w);
}
