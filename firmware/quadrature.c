#include "quadrature.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>

/* We use TIM3 to decode the quadrature signal from the motor encoder. This
   saves us from doing SSI communication in the PWM timer interrupt.
   
   The motor encoder is an AS5040 with 10 bits resolution.
*/
void quadrature_setup_tim3(void)
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
}

void quadrature_init_tim3(int pos)
{
    timer_set_counter(TIM3, pos);
    timer_enable_counter(TIM3);
}

int quadrature_read_pos_tim3(void)
{
    return timer_get_counter(TIM3);
}

/*** TIM2 quadrature ***/
void quadrature_setup_tim2(void)
{
    rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPAEN);
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO1);
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO15);
    gpio_set_af(GPIOA, GPIO_AF1, GPIO1);
    gpio_set_af(GPIOA, GPIO_AF1, GPIO15);
    
    rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM2EN);
    timer_reset(TIM2);
    timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_ic_set_input(TIM2, TIM_IC1, TIM_IC_IN_TI1);
    timer_ic_set_input(TIM2, TIM_IC2, TIM_IC_IN_TI2);
    timer_slave_set_mode(TIM2, TIM_SMCR_SMS_EM3);
}

void quadrature_init_tim2(int pos)
{
    timer_set_counter(TIM2, pos);
    timer_enable_counter(TIM2);
}

int quadrature_read_pos_tim2(void)
{
    return timer_get_counter(TIM2);
}
