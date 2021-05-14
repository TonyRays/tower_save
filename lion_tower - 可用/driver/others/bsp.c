#include "main.h"
void Led_Configuration()
{
    GPIO_InitTypeDef gpio;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    //GPIO_StructInit(&gpio);
    gpio.GPIO_Pin           =   GPIO_Pin_9;
    gpio.GPIO_Mode          =   GPIO_Mode_OUT;
    gpio.GPIO_Speed         =   GPIO_Speed_2MHz;
    gpio.GPIO_PuPd          =   GPIO_PuPd_UP;
    gpio.GPIO_OType         =   GPIO_OType_PP;
    GPIO_Init(GPIOB, &gpio);
}
    
void BSP_Init(void)
{
    Led_Configuration();
//    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);     
//    PWM_Configuration(); 
//    TIM1_Configuration();
//    Quad_Encoder_Configuration();
    USART2_Configuration();
    USART6_Configuration();
//    TIM6_Start();
//    Encoder_Start();
//    delay_ms(800); 
}

