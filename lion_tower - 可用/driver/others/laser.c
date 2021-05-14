#include "laser.h"

void Laser_Configuration()
{
    GPIO_InitTypeDef gpio;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    
    gpio.GPIO_Pin       =   GPIO_Pin_7;
    gpio.GPIO_Mode      =   GPIO_Mode_OUT;
    gpio.GPIO_OType     =   GPIO_OType_PP;
    gpio.GPIO_Speed     =   GPIO_Speed_100MHz;
    GPIO_Init(GPIOD, &gpio);
}

void laser_turn_on_handler()
{
    GPIO_SetBits(GPIOD, GPIO_Pin_7);
}

void laser_turn_off_handler()
{
    GPIO_ResetBits(GPIOD, GPIO_Pin_7);
}
