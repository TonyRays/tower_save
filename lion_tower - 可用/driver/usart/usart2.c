#include "usart2.h"

/*-----USART2_TX-----PD5-----*/
/*-----USART2_RX-----PD6-----*/

static uint8_t _USART2_TX_BUFFER[USART2_DMA_TX_BUFFER_LEN];
static uint8_t _USART2_RX_BUFFER[USART2_DMA_RX_BUFFER_LEN];
static uint16_t receive_msg_size = 0;

void USART2_Configuration(void)
{
    USART_InitTypeDef usart;
    GPIO_InitTypeDef  gpio;
    NVIC_InitTypeDef  nvic;
    DMA_InitTypeDef   dma;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,  ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,   ENABLE);

    gpio.GPIO_Pin                           =   GPIO_Pin_5 | GPIO_Pin_6;
    gpio.GPIO_Mode                          =   GPIO_Mode_AF;
    gpio.GPIO_OType                         =   GPIO_OType_PP;
    gpio.GPIO_Speed                         =   GPIO_Speed_100MHz;
    gpio.GPIO_PuPd                          =   GPIO_PuPd_UP;
    GPIO_Init(GPIOD, &gpio);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2); 

    USART_DeInit(USART2);
    USART_StructInit(&usart);
    usart.USART_BaudRate                    =   115200;
    usart.USART_WordLength                  =   USART_WordLength_8b;
    usart.USART_StopBits                    =   USART_StopBits_1;
    usart.USART_Parity                      =   USART_Parity_No;
    usart.USART_Mode                        =   USART_Mode_Rx | USART_Mode_Tx;
    usart.USART_HardwareFlowControl         =   USART_HardwareFlowControl_None;
    USART_Init(USART2, &usart);

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
    DMA_DeInit(DMA1_Stream5);
    dma.DMA_Channel                         =   DMA_Channel_4;
    dma.DMA_PeripheralBaseAddr              =   (uint32_t)(&USART2->DR);
    dma.DMA_Memory0BaseAddr                 =   (uint32_t)_USART2_RX_BUFFER;
    dma.DMA_DIR                             =   DMA_DIR_PeripheralToMemory;
    dma.DMA_BufferSize                      =   USART2_DMA_RX_BUFFER_LEN;
    DMA_Init(DMA1_Stream5, &dma);
    // transmit
    DMA_DeInit(DMA1_Stream6);
    dma.DMA_Channel                         =   DMA_Channel_4;
    dma.DMA_PeripheralBaseAddr              =   (uint32_t)(&USART2->DR);
    dma.DMA_Memory0BaseAddr                 =   (uint32_t)_USART2_TX_BUFFER;
    dma.DMA_DIR                             =   DMA_DIR_MemoryToPeripheral;
    dma.DMA_BufferSize                      =   USART2_DMA_TX_BUFFER_LEN;
    DMA_Init(DMA1_Stream6, &dma);

    // nvic
    nvic.NVIC_IRQChannel                    =   USART2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority  =   0;
    nvic.NVIC_IRQChannelSubPriority         =   0;
    nvic.NVIC_IRQChannelCmd                 =   ENABLE;
    NVIC_Init(&nvic);
    
    nvic.NVIC_IRQChannel                    =   DMA1_Stream5_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority  =   0;
    nvic.NVIC_IRQChannelSubPriority         =   0;
    nvic.NVIC_IRQChannelCmd                 =   ENABLE; 
    NVIC_Init(&nvic);

    nvic.NVIC_IRQChannel                    =   DMA1_Stream6_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority  =   0;
    nvic.NVIC_IRQChannelSubPriority         =   0;
    nvic.NVIC_IRQChannelCmd                 =   ENABLE; 
    NVIC_Init(&nvic);
    
    DMA_ITConfig(DMA1_Stream5, DMA_IT_TC, ENABLE);
    DMA_ITConfig(DMA1_Stream6, DMA_IT_TC, ENABLE);
    USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);
    
    USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
    USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
    USART_Cmd(USART2, ENABLE);
}


/***
 * @brief 开启一次DMA传输
 * @param DMAx_Streamx    DMA数据流
 * @param size            数据传输量
 */
void usart2_dma_transmit_handler(uint16_t size)
{
    DMA_Cmd(DMA1_Stream6, DISABLE);                      // 关闭DMA传输

    while (DMA_GetCmdStatus(DMA1_Stream6) != DISABLE){}  // 确保DMA可以被设置

    DMA_SetCurrDataCounter(DMA1_Stream6, size);          // 数据传输量
 
    DMA_Cmd(DMA1_Stream6, ENABLE);                       // 开启DMA传输
}

void USART2_PrintBlock(uint8_t* pdata, uint8_t len)
{
    memcpy((uint8_t *)_USART2_TX_BUFFER, pdata, len);

    usart2_dma_transmit_handler(len);
}

/***
 * 发送完成中断
 */
void DMA1_Stream6_IRQHandler(void)
{
    if(DMA_GetFlagStatus(DMA1_Stream6, DMA_FLAG_TCIF6) != RESET)
    {
        DMA_ClearFlag(DMA1_Stream6, DMA_FLAG_TCIF6);
    }
}

/***
 * 接收完成中断: 应对数据长度超过buffer空间的情况
 */
void DMA1_Stream5_IRQHandler(void)
{
    if(DMA_GetFlagStatus(DMA1_Stream5, DMA_FLAG_TCIF5) != RESET)
    {
        DMA_Cmd(DMA1_Stream5, DISABLE);
        receive_msg_size = USART2_DMA_RX_BUFFER_LEN - DMA_GetCurrDataCounter(DMA1_Stream5);
        
        for (uint16_t i = 0; i < receive_msg_size; i++)
            js_download_msg_process(_USART2_RX_BUFFER[i]);

        DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5);
        DMA_SetCurrDataCounter(DMA1_Stream5, USART2_DMA_RX_BUFFER_LEN);
        DMA_Cmd(DMA1_Stream5, ENABLE);
    }
}

/***
 * 串口空闲中断: 应对数据长度不定的情况
 */


void USART2_IRQHandler(void)
{
    if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
    {
        USART2->SR;
        USART2->DR;

        DMA_Cmd(DMA1_Stream5, DISABLE);
        receive_msg_size = USART2_DMA_RX_BUFFER_LEN - DMA_GetCurrDataCounter(DMA1_Stream5);
        
        for (uint16_t i = 0; i < receive_msg_size; i++)
            js_download_msg_process(_USART2_RX_BUFFER[i]);

        DMA_SetCurrDataCounter(DMA1_Stream5, USART2_DMA_RX_BUFFER_LEN);
        DMA_Cmd(DMA1_Stream5, ENABLE);
    }
}
