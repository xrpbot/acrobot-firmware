#ifndef PCBWRITER_USART_H
#define PCBWRITER_USART_H

#include <stdint.h>

void usart_setup(void);
void usart_dma_write(uint8_t *data, int size);

extern volatile uint8_t msg_buf[256];
extern volatile uint8_t msg_recv;    // Message is being received
extern volatile uint8_t msg_ready;   // Message ready to be processed
extern volatile uint8_t msg_len;     // Message length

extern volatile int usart_dma_tx_complete;

#endif
