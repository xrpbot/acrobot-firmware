#include "adc.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>

void adc_setup(void)
{
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
