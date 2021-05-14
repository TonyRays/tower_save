#ifndef _UART4_H_
#define _UART4_H_
#include "main.h"

void UART4_Configuration(void);
void uart4_dma_transmit_handler(uint16_t size);
void UART4_PrintBlock(uint8_t* pdata, uint8_t len);

void UART4_Oscilloscope(float ch1, float ch2, float ch3, float ch4);

#define UART4_DMA_TX_BUFFER_LEN 150u
#define UART4_DMA_RX_BUFFER_LEN 50u

#endif
