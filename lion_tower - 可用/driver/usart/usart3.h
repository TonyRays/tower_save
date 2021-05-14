#ifndef __USART3_H__
#define __USART3_H__
#include "main.h"

#define USART3_DMA_TX_BUFFER_LEN 150u
#define USART3_DMA_RX_BUFFER_LEN 50u

void USART3_Configuration(void);
void USART3_Oscilloscope(float ch1, float ch2, float ch3, float ch4);
void USART3_PrintBlock(uint8_t* pdata, uint8_t len);

#endif
