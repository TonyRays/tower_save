#include "uart4.h"

static uint8_t _UART4_TX_BUFFER[UART4_DMA_TX_BUFFER_LEN];
static uint8_t _UART4_RX_BUFFER[UART4_DMA_RX_BUFFER_LEN];
uint16_t uart4_receive_msg_size = 0;

void UART4_Configuration(void){
		GPIO_InitTypeDef  gpio;
    NVIC_InitTypeDef  nvic;
    DMA_InitTypeDef   dma;
    USART_InitTypeDef uart;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,  ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,  ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,   ENABLE);

    gpio.GPIO_Pin                           =   GPIO_Pin_0 | GPIO_Pin_1;
    gpio.GPIO_Mode                          =   GPIO_Mode_AF;
    gpio.GPIO_OType                         =   GPIO_OType_PP;
    gpio.GPIO_Speed                         =   GPIO_Speed_100MHz;
    gpio.GPIO_PuPd                          =   GPIO_PuPd_UP;;
    GPIO_Init(GPIOA, &gpio);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_UART4);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_UART4);

    USART_DeInit(UART4);
    USART_StructInit(&uart);
    uart.USART_BaudRate                    =   115200;
    uart.USART_WordLength                  =   USART_WordLength_8b;
    uart.USART_StopBits                    =   USART_StopBits_1;
    uart.USART_Parity                      =   USART_Parity_No;
    uart.USART_Mode                        =   USART_Mode_Tx | USART_Mode_Rx;
    uart.USART_HardwareFlowControl         =   USART_HardwareFlowControl_None;
    USART_Init(UART4, &uart);

    DMA_StructInit(&dma);
    dma.DMA_Mode                            =   DMA_Mode_Normal;
    dma.DMA_PeripheralInc                   =   DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc                       =   DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize              =   DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize                  =   DMA_MemoryDataSize_Byte;
    dma.DMA_Priority                        =   DMA_Priority_Medium;
    dma.DMA_FIFOMode                        =   DMA_FIFOMode_Disable;
    dma.DMA_FIFOThreshold                   =   DMA_FIFOThreshold_1QuarterFull;
    dma.DMA_MemoryBurst                     =   DMA_MemoryBurst_Single;
    dma.DMA_PeripheralBurst                 =   DMA_PeripheralBurst_Single;
    // receive
    DMA_DeInit(DMA1_Stream2);
    dma.DMA_Channel                         =   DMA_Channel_4;
    dma.DMA_PeripheralBaseAddr              =   (uint32_t)(&UART4->DR);
    dma.DMA_Memory0BaseAddr                 =   (uint32_t)_UART4_RX_BUFFER;
    dma.DMA_DIR                             =   DMA_DIR_PeripheralToMemory;
    dma.DMA_BufferSize                      =   UART4_DMA_RX_BUFFER_LEN;
    DMA_Init(DMA1_Stream2, &dma);
    // transmit
    DMA_DeInit(DMA1_Stream4);
    dma.DMA_Channel                         =   DMA_Channel_4;
    dma.DMA_PeripheralBaseAddr              =   (uint32_t)(&UART4->DR);
    dma.DMA_Memory0BaseAddr                 =   (uint32_t)_UART4_TX_BUFFER;
    dma.DMA_DIR                             =   DMA_DIR_MemoryToPeripheral;
    dma.DMA_BufferSize                      =   UART4_DMA_TX_BUFFER_LEN;
    DMA_Init(DMA1_Stream4, &dma);

    // nvic
    nvic.NVIC_IRQChannel                    =   UART4_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority  =   2;
    nvic.NVIC_IRQChannelSubPriority         =   1;
    nvic.NVIC_IRQChannelCmd                 =   ENABLE; 
    NVIC_Init(&nvic);

    nvic.NVIC_IRQChannel                    =   DMA1_Stream2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority  =   2;
    nvic.NVIC_IRQChannelSubPriority         =   2;
    nvic.NVIC_IRQChannelCmd                 =   ENABLE; 
    NVIC_Init(&nvic);

    nvic.NVIC_IRQChannel                    =   DMA1_Stream4_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority  =   2;
    nvic.NVIC_IRQChannelSubPriority         =   3;
    nvic.NVIC_IRQChannelCmd                 =   ENABLE; 
    NVIC_Init(&nvic);

    DMA_ITConfig(DMA1_Stream2, DMA_IT_TC, ENABLE);
    DMA_ITConfig(DMA1_Stream4, DMA_IT_TC, ENABLE);
    USART_ITConfig(UART4, USART_IT_IDLE, ENABLE);
    
    USART_DMACmd(UART4, USART_DMAReq_Tx, ENABLE);
    USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE);
    USART_Cmd(UART4, ENABLE); 
}



void DMA1_Stream4_IRQHandler(void)
{
    if(DMA_GetFlagStatus(DMA1_Stream4, DMA_FLAG_TCIF4) != RESET)
    {
        DMA_ClearFlag(DMA1_Stream4, DMA_FLAG_TCIF4);
    }
}


void DMA1_Stream2_IRQHandler(void)
{
    if(DMA_GetFlagStatus(DMA1_Stream2, DMA_FLAG_TCIF2) != RESET)
    {
        DMA_Cmd(DMA1_Stream2, DISABLE);
        uart4_receive_msg_size = UART4_DMA_RX_BUFFER_LEN - DMA_GetCurrDataCounter(DMA1_Stream2);
//        for (uint16_t i = 0; i < uart4_receive_msg_size; i++)
//            wireless_pid_download_msg_process(_UART4_RX_BUFFER[i]);
        DMA_ClearFlag(DMA1_Stream2, DMA_FLAG_TCIF2);
        DMA_SetCurrDataCounter(DMA1_Stream2, UART4_DMA_RX_BUFFER_LEN);
        DMA_Cmd(DMA1_Stream2, ENABLE);
    }
}


void UART4_IRQHandler(void)
{
    if(USART_GetITStatus(UART4, USART_IT_IDLE) != RESET)
    {
        UART4->SR;
        UART4->DR;

        DMA_Cmd(DMA1_Stream2, DISABLE);
        uart4_receive_msg_size = UART4_DMA_RX_BUFFER_LEN - DMA_GetCurrDataCounter(DMA1_Stream2);
//				for (uint16_t i = 0; i < uart4_receive_msg_size; i++)
//						wireless_pid_download_msg_process(_UART4_RX_BUFFER[i]);
        DMA_SetCurrDataCounter(DMA1_Stream2, UART4_DMA_RX_BUFFER_LEN);
        DMA_Cmd(DMA1_Stream2, ENABLE);
    }
}



void uart4_dma_transmit_handler(uint16_t size)
{
    DMA_Cmd(DMA1_Stream4, DISABLE);                      // ??DMA??

    while (DMA_GetCmdStatus(DMA1_Stream4) != DISABLE){}  // ??DMA?????

    DMA_SetCurrDataCounter(DMA1_Stream4, size);          // ?????
 
    DMA_Cmd(DMA1_Stream4, ENABLE);                       // ??DMA??
}

void UART4_PrintBlock(uint8_t* pdata, uint8_t len)
{
    memcpy((uint8_t *)_UART4_TX_BUFFER, pdata, len);

    uart4_dma_transmit_handler(len);
}

uint8_t uart4_osci_buffer[10];

void UART4_Oscilloscope(float ch1, float ch2, float ch3, float ch4)
{
    // data prepare
    uint16_t _ch1 = ch1 * 10.f + 32768;
    uint16_t _ch2 = ch2 * 10.f + 32768;
    uint16_t _ch3 = ch3 * 10.f + 32768;
    uint16_t _ch4 = ch4 * 10.f + 32768;
    
    // header
    uart4_osci_buffer[0] = 0xFF;
    uart4_osci_buffer[1] = 0xFF;
    
    // channel 1
    uart4_osci_buffer[2] = (uint8_t)(_ch1 >> 8 & 0xFF);
    uart4_osci_buffer[3] = (uint8_t)(_ch1 & 0xFF);
    // channel 2
    uart4_osci_buffer[4] = (uint8_t)(_ch2 >> 8 & 0xFF);
    uart4_osci_buffer[5] = (uint8_t)(_ch2 & 0xFF);
    // channel 3
    uart4_osci_buffer[6] = (uint8_t)(_ch3 >> 8 & 0xFF);
    uart4_osci_buffer[7] = (uint8_t)(_ch3 & 0xFF);
    // channel 4
    uart4_osci_buffer[8] = (uint8_t)(_ch4 >> 8 & 0xFF);
    uart4_osci_buffer[9] = (uint8_t)(_ch4 & 0xFF);
    
    // transmit
    UART4_PrintBlock(uart4_osci_buffer, 10);
}

/*int fputc(int ch, FILE *f)
{
    while (USART_GetFlagStatus(UART4,USART_FLAG_TC) == RESET);
    USART_SendData(UART4, (uint8_t)ch);
    return ch;
}*/
