#ifndef __USART2_H__
#define __USART2_H__
#include "main.h"

#define USART2_DMA_TX_BUFFER_LEN 150u
#define USART2_DMA_RX_BUFFER_LEN 50u

void USART2_Configuration(void);
void USART2_PrintBlock(uint8_t*, uint8_t);
void USART2_Oscilloscope(float ch1, float ch2, float ch3, float ch4);

#endif
