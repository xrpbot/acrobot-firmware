#include "ssi.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

void ssi_delay(uint32_t time);
as5xxx_data_t ssi_read(int do_port, int do_pin, int clk_port, int clk_pin, int csn_port, int csn_pin);

void ssi_setup(void) {
    rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPDEN | RCC_AHB1ENR_IOPEEN);
    
    // Motor encoder
    gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO12 | GPIO13);
    gpio_set(GPIOE, GPIO13);   // set CSn to high
    gpio_mode_setup(GPIOE, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO11);
    
    // Active axis center encoder
    gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO10 | GPIO11);
    gpio_set(GPIOD, GPIO11);   // set CSn to high
    gpio_mode_setup(GPIOD, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, GPIO9);
    
    // Passive axis ring encoder
    gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO5 | GPIO6);
    gpio_set(GPIOE, GPIO6);   // set CSn to high
    gpio_mode_setup(GPIOE, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO4);
    
    // Active axis ring encoder
    gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO1 | GPIO2);
    gpio_set(GPIOD, GPIO2);   // set CSn to high
    gpio_mode_setup(GPIOD, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO0);

}

void ssi_delay(uint32_t time) {
    for (uint32_t i = 0; i <= time; i++) {
        __asm("nop");
    }
}

as5xxx_data_t ssi_read(int do_port, int do_pin, int clk_port, int clk_pin, int csn_port, int csn_pin)
{
    uint32_t raw_data = 0;
    
    gpio_set(clk_port, clk_pin);   // CLK = 1
    gpio_clear(csn_port, csn_pin); // CSn = 0
    ssi_delay(3);  // wait t_CLK, FE (500 ns)
    
    gpio_clear(clk_port, clk_pin); // CLK = 0 (latches data)
    ssi_delay(3);   // wait t_CLK/2 (500 ns)
    
    for(uint8_t i=0; i<18; i++) {
        gpio_set(clk_port, clk_pin); // CLK = 1
        ssi_delay(2);  // wait t_CLK/2 (500 ns)
        
        /* Sample data */
        raw_data <<= 1;
        int next_bit = !!gpio_get(do_port, do_pin);
        raw_data |= next_bit;
        
        gpio_clear(clk_port, clk_pin); // CLK = 0
        ssi_delay(1);              // wait t_CLK/2 (500 ns)
    }
    
    raw_data >>= 1;
    
    as5xxx_data_t data = {0};
    data.position = (raw_data >> 5) & 0xFFF;
    data.offset_comp_finished = (raw_data >> 4) & 0x1;
    data.linearity_alert = (raw_data >> 3) & 0x1;
    data.mag_inc = (raw_data >> 2) & 0x1;
    data.mag_dec = (raw_data >> 1) & 0x1;
    
    gpio_set(csn_port, csn_pin);  // CSn = 1
    
    ssi_delay(30);  // ?
    
    return data;
}

as5xxx_data_t ssi_read_motor(void)
{
    // PE11: DATA, PE12: CLK, PE13: CSn
    return ssi_read(GPIOE, GPIO11, GPIOE, GPIO12, GPIOE, GPIO13);
}

as5xxx_data_t ssi_read_active_axis(void)
{
    // PD9: DATA, PD10: CLK, PD11: CSn
    return ssi_read(GPIOD, GPIO9, GPIOD, GPIO10, GPIOD, GPIO11);
}

as5xxx_data_t ssi_read_passive_ring(void)
{
    // PE4: DATA, PE5: CLK, PE6: CSn
    return ssi_read(GPIOE, GPIO4, GPIOE, GPIO5, GPIOE, GPIO6);
}

// Active axis ring encoder
as5xxx_data_t ssi_read_active_ring(void)
{
    // PD0: DATA, PD1: CLK, PD2: CSn
    return ssi_read(GPIOD, GPIO0, GPIOD, GPIO1, GPIOD, GPIO2);
}

