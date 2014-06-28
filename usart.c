#include "usart.h"
#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/f4/dma.h>
#include <libopencm3/stm32/f4/nvic.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>

#define USART_CONSOLE USART2

#define MSG_START_CHAR 0x40
#define MSG_END_CHAR   0x41

volatile uint8_t msg_recv = 0;
volatile uint8_t msg_ready = 0;
volatile uint8_t msg_len = 0;

int _write(int file, char *ptr, int len);

volatile uint8_t msg_buf[256] = { 0 };
volatile int usart_dma_tx_complete = 0;

void usart_setup(void)
{
    /* Setup clock */
    /* Enable GPIOA clock for USART. */
    rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPAEN);
    
    /* Enable clocks for USART2. */
    rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_USART2EN);
    
    /* Enable clock for DMA. */
    rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_DMA1EN);
    
    /* Setup GPIO pin for USART2 transmit. (PA2) */
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2);
    gpio_set_af(GPIOA, GPIO_AF7, GPIO2);
    
    /* Setup GPIO pin for USART2 receive. (PA3) */
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO3);
    gpio_set_af(GPIOA, GPIO_AF7, GPIO3);
    
    /* Enable IRQ */
    nvic_enable_irq(NVIC_USART2_IRQ);
    usart_enable_rx_interrupt(USART2);
    
    /* Setup USART2 parameters. */
    usart_set_baudrate(USART2, 230400);
    usart_set_databits(USART2, 8);
    usart_set_stopbits(USART2, USART_STOPBITS_1);
    usart_set_mode(USART2, USART_MODE_TX_RX);
    usart_set_parity(USART2, USART_PARITY_NONE);
    usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
    
    /* Finally enable the USART. */
    usart_enable(USART2);
    
    /* Enable data. */
    nvic_set_priority(NVIC_DMA1_STREAM6_IRQ, 0);
    nvic_enable_irq(NVIC_DMA1_STREAM6_IRQ);
    
    usart_dma_tx_complete = 1;
}

void usart_dma_write(uint8_t *data, int size)
{
    /*
     * Using stream 6, channel 4 for USART2_TX
     */
    
    /* Reset DMA channel*/
    dma_stream_reset(DMA1, DMA_STREAM6);
    
    dma_channel_select(DMA1, DMA_STREAM6, DMA_SxCR_CHSEL_4);
    dma_set_transfer_mode(DMA1, DMA_STREAM6, DMA_SxCR_DIR_MEM_TO_PERIPHERAL);
    dma_set_peripheral_address(DMA1, DMA_STREAM6, (uint32_t)&USART2_DR);
    dma_set_memory_address(DMA1, DMA_STREAM6, (uint32_t)data);
    dma_set_number_of_data(DMA1, DMA_STREAM6, size);
    dma_enable_memory_increment_mode(DMA1, DMA_STREAM6);
    dma_set_peripheral_size(DMA1, DMA_STREAM6, DMA_SxCR_PSIZE_8BIT);
    dma_set_memory_size(DMA1, DMA_STREAM6, DMA_SxCR_MSIZE_8BIT);
    dma_set_priority(DMA1, DMA_STREAM6, DMA_SxCR_PL_VERY_HIGH);
    
    dma_enable_transfer_complete_interrupt(DMA1, DMA_STREAM6);
    
    dma_enable_stream(DMA1, DMA_STREAM6);
    
    usart_enable_tx_dma(USART2);
    
    usart_dma_tx_complete = 0;
}

void dma1_stream6_isr(void)
{
    if ((DMA1_HISR &DMA_HISR_TCIF6) != 0) {
        DMA1_HIFCR |= DMA_HIFCR_CTCIF6;
        
        usart_dma_tx_complete = 1;
    }
    
    dma_disable_transfer_complete_interrupt(DMA1, DMA_STREAM6);
    
    usart_disable_tx_dma(USART2);
    
    dma_disable_stream(DMA1, DMA_STREAM6);
}


/**
 * Use USART_CONSOLE as a console.
 * @param file
 * @param ptr
 * @param len
 * @return
 */
int _write(int file, char *ptr, int len)
{
    int i;
    
    if (file == STDOUT_FILENO || file == STDERR_FILENO) {
        for (i = 0; i < len; i++) {
            if (ptr[i] == '\n') {
                usart_send_blocking(USART_CONSOLE, '\r');
            }
            usart_send_blocking(USART_CONSOLE, ptr[i]);
        }
        return i;
    }
    errno = EIO;
    return -1;
}

void usart2_isr(void)
{
    if(usart_get_flag(USART2, USART_SR_RXNE)) {
        uint8_t ch = usart_recv(USART2);
        if(!msg_recv) {
            if(!msg_ready && ch == MSG_START_CHAR) {
                msg_recv = true;
                msg_len = 0;
            }
        } else {
            if(ch == MSG_END_CHAR) {
                msg_len = msg_len / 2;
                msg_recv = false;
                msg_ready = true;
            } else {
                if((msg_len % 2) == 0) {
                    msg_buf[msg_len/2] = ch << 4;
                } else {
                    msg_buf[msg_len/2] = msg_buf[msg_len/2] + (ch & 0x0F);
                }
                msg_len++;
            }
        }
        
        USART_SR(USART2) &= ~USART_SR_RXNE;
    }
}

