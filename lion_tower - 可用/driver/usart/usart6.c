#include "main.h"

/*-----USART6_TX-----PC6-----*/
/*-----USART6_RX-----PC7-----*/

static uint8_t _USART6_RX_BUFFER[2][USART6_DMA_RX_BUFFER_LEN];

void USART6_Configuration(void)
{
    GPIO_InitTypeDef  gpio;
    NVIC_InitTypeDef  nvic;
    DMA_InitTypeDef   dma;
    USART_InitTypeDef usart;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,  ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,   ENABLE);

    gpio.GPIO_Pin                           =   GPIO_Pin_7;
    gpio.GPIO_Mode                          =   GPIO_Mode_AF;
    gpio.GPIO_Speed                         =   GPIO_Speed_2MHz;
    gpio.GPIO_PuPd                          =   GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &gpio);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6); 
    
    usart.USART_BaudRate                    =   115200;
    usart.USART_WordLength                  =   USART_WordLength_8b;
    usart.USART_StopBits                    =   USART_StopBits_1;
    usart.USART_Parity                      =   USART_Parity_No;
    usart.USART_Mode                        =   USART_Mode_Rx;
    usart.USART_HardwareFlowControl         =   USART_HardwareFlowControl_None;
    USART_Init(USART6, &usart);

    dma.DMA_Mode                            =   DMA_Mode_Circular;
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
    DMA_DeInit(DMA2_Stream1);
    dma.DMA_Channel                         =   DMA_Channel_5;
    dma.DMA_PeripheralBaseAddr              =   (uint32_t)(&USART6->DR);
    dma.DMA_Memory0BaseAddr                 =   (uint32_t)&_USART6_RX_BUFFER[0][0];
    dma.DMA_BufferSize                      =   USART6_DMA_RX_BUFFER_LEN;
    dma.DMA_DIR                             =   DMA_DIR_PeripheralToMemory;
    DMA_Init(DMA2_Stream1, &dma);

    nvic.NVIC_IRQChannel                    =   USART6_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority  =   1;
    nvic.NVIC_IRQChannelSubPriority         =   1;
    nvic.NVIC_IRQChannelCmd                 =   ENABLE; 
    NVIC_Init(&nvic);

    // 双buffer设计
    DMA_DoubleBufferModeConfig(DMA2_Stream1, (uint32_t)&_USART6_RX_BUFFER[1][0], DMA_Memory_0);   //first used memory configuration
    DMA_DoubleBufferModeCmd(DMA2_Stream1, ENABLE);

    USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);
    USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);
    
    DMA_Cmd(DMA2_Stream1, ENABLE);
    USART_Cmd(USART6, ENABLE);
}

void USART6_IRQHandler(void)
{
    static uint32_t this_time_rx_len = 0;
    if(USART_GetITStatus(USART6, USART_IT_IDLE) != RESET)
    {
        //clear the idle pending flag 
        (void)USART6->SR;
        (void)USART6->DR;
        
        // Target is Memory0
        if(DMA_GetCurrentMemoryTarget(DMA2_Stream1) == DMA_Memory_0)
        {
            DMA_Cmd(DMA2_Stream1, DISABLE);
            this_time_rx_len = USART6_DMA_RX_BUFFER_LEN - DMA_GetCurrDataCounter(DMA2_Stream1);
            DMA2_Stream1->NDTR = (uint16_t)USART6_DMA_RX_BUFFER_LEN;  // relocate the dma memory pointer to the beginning position
            DMA2_Stream1->CR |= (uint32_t)(DMA_SxCR_CT);              // enable the current selected memory is Memory 1
            DMA_Cmd(DMA2_Stream1, ENABLE);
            
            if(this_time_rx_len == IMU_MSG_LENGTH_3)
            {
                imu_download_msg_process(_USART6_RX_BUFFER[0]);
            }
        }
        // Target is Memory1
        else 
        {
            DMA_Cmd(DMA2_Stream1, DISABLE);
            this_time_rx_len = USART6_DMA_RX_BUFFER_LEN - DMA_GetCurrDataCounter(DMA2_Stream1);
            DMA2_Stream1->NDTR = (uint16_t)USART6_DMA_RX_BUFFER_LEN;  // relocate the dma memory pointer to the beginning position
            DMA2_Stream1->CR &= ~(uint32_t)(DMA_SxCR_CT);             // enable the current selected memory is Memory 0
            DMA_Cmd(DMA2_Stream1, ENABLE);
            
            if(this_time_rx_len == IMU_MSG_LENGTH_3)
            {
                imu_download_msg_process(_USART6_RX_BUFFER[1]);
            }
        }
    }
}
