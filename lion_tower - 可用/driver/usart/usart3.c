#include "usart3.h"

/*-----USART3_TX-----PB10-----*/
/*-----USART3_RX-----PB11-----*/

static uint8_t _USART3_TX_BUFFER[USART3_DMA_TX_BUFFER_LEN];
static uint8_t _USART3_RX_BUFFER[USART3_DMA_RX_BUFFER_LEN];
static uint16_t receive_msg_size = 0;

void USART3_Configuration(void)
{
    GPIO_InitTypeDef  gpio;
    NVIC_InitTypeDef  nvic;
    DMA_InitTypeDef   dma;
    USART_InitTypeDef usart;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,  ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,   ENABLE);

    gpio.GPIO_Pin                           =   GPIO_Pin_10 | GPIO_Pin_11;
    gpio.GPIO_Mode                          =   GPIO_Mode_AF;
    gpio.GPIO_OType                         =   GPIO_OType_PP;
    gpio.GPIO_Speed                         =   GPIO_Speed_100MHz;
    gpio.GPIO_PuPd                          =   GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &gpio);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);

    USART_DeInit(USART3);
    USART_StructInit(&usart);
    usart.USART_BaudRate                    =   115200;
    usart.USART_WordLength                  =   USART_WordLength_8b;
    usart.USART_StopBits                    =   USART_StopBits_1;
    usart.USART_Parity                      =   USART_Parity_No;
    usart.USART_Mode                        =   USART_Mode_Tx | USART_Mode_Rx;
    usart.USART_HardwareFlowControl         =   USART_HardwareFlowControl_None;
    USART_Init(USART3, &usart);

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
    DMA_DeInit(DMA1_Stream1);
    dma.DMA_Channel                         =   DMA_Channel_4;
    dma.DMA_PeripheralBaseAddr              =   (uint32_t)(&USART3->DR);
    dma.DMA_Memory0BaseAddr                 =   (uint32_t)_USART3_RX_BUFFER;
    dma.DMA_DIR                             =   DMA_DIR_PeripheralToMemory;
    dma.DMA_BufferSize                      =   USART3_DMA_RX_BUFFER_LEN;
    DMA_Init(DMA1_Stream1, &dma);
    // transmit
    DMA_DeInit(DMA1_Stream3);
    dma.DMA_Channel                         =   DMA_Channel_4;
    dma.DMA_PeripheralBaseAddr              =   (uint32_t)(&USART3->DR);
    dma.DMA_Memory0BaseAddr                 =   (uint32_t)_USART3_TX_BUFFER;
    dma.DMA_DIR                             =   DMA_DIR_MemoryToPeripheral;
    dma.DMA_BufferSize                      =   USART3_DMA_TX_BUFFER_LEN;
    DMA_Init(DMA1_Stream3, &dma);

    // nvic
    nvic.NVIC_IRQChannel                    =   USART3_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority  =   0;
    nvic.NVIC_IRQChannelSubPriority         =   0;
    nvic.NVIC_IRQChannelCmd                 =   ENABLE; 
    NVIC_Init(&nvic);

    nvic.NVIC_IRQChannel                    =   DMA1_Stream1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority  =   0;
    nvic.NVIC_IRQChannelSubPriority         =   0;
    nvic.NVIC_IRQChannelCmd                 =   ENABLE; 
    NVIC_Init(&nvic);

    nvic.NVIC_IRQChannel                    =   DMA1_Stream3_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority  =   0;
    nvic.NVIC_IRQChannelSubPriority         =   0;
    nvic.NVIC_IRQChannelCmd                 =   ENABLE; 
    NVIC_Init(&nvic);

    DMA_ITConfig(DMA1_Stream1, DMA_IT_TC, ENABLE);
    DMA_ITConfig(DMA1_Stream3, DMA_IT_TC, ENABLE);
    USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
    
    USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);
    USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
    USART_Cmd(USART3, ENABLE); 
}

 
/***
 * @brief 开启一次DMA传输
 * @param DMAx_Streamx    DMA数据流
 * @param size            数据传输量
 */
void usart3_dma_transmit_handler(uint16_t size)
{
    DMA_Cmd(DMA1_Stream3, DISABLE);                      // 关闭DMA传输

    while (DMA_GetCmdStatus(DMA1_Stream3) != DISABLE){}  // 确保DMA可以被设置

    DMA_SetCurrDataCounter(DMA1_Stream3, size);          // 数据传输量
 
    DMA_Cmd(DMA1_Stream3, ENABLE);                       // 开启DMA传输
}

void USART3_PrintBlock(uint8_t* pdata, uint8_t len)
{
    memcpy((uint8_t *)_USART3_TX_BUFFER, pdata, len);

    usart3_dma_transmit_handler(len);
}

/***
 * 发送完成中断
 */
void DMA1_Stream3_IRQHandler(void)
{
    if(DMA_GetFlagStatus(DMA1_Stream3, DMA_FLAG_TCIF3) != RESET)
    {
        DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_TCIF3);
    }
}

/***
 * 接收完成中断: 应对数据长度超过buffer空间的情况
 */
void DMA1_Stream1_IRQHandler(void)
{
    if(DMA_GetFlagStatus(DMA1_Stream1, DMA_FLAG_TCIF1) != RESET)
    {
        DMA_Cmd(DMA1_Stream1, DISABLE);
        receive_msg_size = USART3_DMA_RX_BUFFER_LEN - DMA_GetCurrDataCounter(DMA1_Stream1);
        
        for (uint16_t i = 0; i < receive_msg_size; i++)
            pc_download_msg_process(_USART3_RX_BUFFER[i]);

        DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_TCIF1);
        DMA_SetCurrDataCounter(DMA1_Stream1, USART3_DMA_RX_BUFFER_LEN);
        DMA_Cmd(DMA1_Stream1, ENABLE);
    }
}

/***
 * 串口空闲中断: 应对数据长度不定的情况
 */
void USART3_IRQHandler(void)
{
	//printf("usart3\n");
    if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)
    {
        USART3->SR;
        USART3->DR;

        DMA_Cmd(DMA1_Stream1, DISABLE);
        receive_msg_size = USART3_DMA_RX_BUFFER_LEN - DMA_GetCurrDataCounter(DMA1_Stream1);

        for (uint16_t i = 0; i < receive_msg_size; i++)
            pc_download_msg_process(_USART3_RX_BUFFER[i]);

        DMA_SetCurrDataCounter(DMA1_Stream1, USART3_DMA_RX_BUFFER_LEN);
        DMA_Cmd(DMA1_Stream1, ENABLE);
    }
}

    
// construct new data frame
uint8_t osci_buffer[10];

void USART3_Oscilloscope(float ch1, float ch2, float ch3, float ch4)
{
    // data prepare
    uint16_t _ch1 = ch1 * 10.f + 32768;
    uint16_t _ch2 = ch2 * 10.f + 32768;
    uint16_t _ch3 = ch3 * 10.f + 32768;
    uint16_t _ch4 = ch4 * 10.f + 32768;
    
    // header
    osci_buffer[0] = 0xFF;
    osci_buffer[1] = 0xFF;
    
    // channel 1
    osci_buffer[2] = (uint8_t)(_ch1 >> 8 & 0xFF);
    osci_buffer[3] = (uint8_t)(_ch1 & 0xFF);
    // channel 2
    osci_buffer[4] = (uint8_t)(_ch2 >> 8 & 0xFF);
    osci_buffer[5] = (uint8_t)(_ch2 & 0xFF);
    // channel 3
    osci_buffer[6] = (uint8_t)(_ch3 >> 8 & 0xFF);
    osci_buffer[7] = (uint8_t)(_ch3 & 0xFF);
    // channel 4
    osci_buffer[8] = (uint8_t)(_ch4 >> 8 & 0xFF);
    osci_buffer[9] = (uint8_t)(_ch4 & 0xFF);
    
    // transmit
    USART3_PrintBlock(osci_buffer, 10);
}

int fputc(int ch, FILE *f)
{
    while (USART_GetFlagStatus(USART2,USART_FLAG_TC) == RESET);
    USART_SendData(USART2, (uint8_t)ch);
    return ch;
}
