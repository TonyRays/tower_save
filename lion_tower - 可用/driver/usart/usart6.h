#ifndef __USART6_H__
#define __USART6_H__
#include "main.h"

#define USART6_DMA_RX_BUFFER_LEN 60u

void USART6_Configuration(void);
void UART6_PutStr (const char *);

#endif
