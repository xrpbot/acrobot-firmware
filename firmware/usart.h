#ifndef XRP_USART_H
#define XRP_USART_H

#include <stdint.h>

extern volatile int usart_dma_tx_complete;

void usart_setup(void);
void usart_dma_write(uint8_t *data, int size);

int usart_tx_complete(void);

#endif
