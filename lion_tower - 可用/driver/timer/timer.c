#include "main.h"


extern uint32_t time_tick_1ms;

void TIM6_Configuration(void)
{
    TIM_TimeBaseInitTypeDef  tim;
    NVIC_InitTypeDef         nvic;
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
    
    tim.TIM_Prescaler                       =   84 - 1;
    tim.TIM_Period                          =   1000;
    tim.TIM_CounterMode                     =   TIM_CounterMode_Up;
    tim.TIM_ClockDivision                   =   TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM6, &tim);
    
    nvic.NVIC_IRQChannel                    =   TIM6_DAC_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority  =   0;
    nvic.NVIC_IRQChannelSubPriority         =   1;
    nvic.NVIC_IRQChannelCmd                 =   ENABLE;
    NVIC_Init(&nvic);

    TIM_Cmd(TIM6, ENABLE);
    
    TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
    TIM_ClearFlag(TIM6, TIM_FLAG_Update);
}

void TIM6_DAC_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET) 
    {
        TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
        TIM_ClearFlag(TIM6, TIM_FLAG_Update);
        
        control_task();
			
			      
    if (time_tick_1ms % 100 == 0)
		{
			GPIOB->ODR ^= 1 << 9;
			//printf("%f %f\n",imu.gyro.z,imu.mag.z);
			//printf("finish_flag :% d\n",shoot.finish_flag);
      //printf("bullet_count_get : %d\n",shoot.bullet_count_get);
			//printf("set_local : %f get : %f\n",gimbal.joint[YAW].angle_set_local, gimbal.joint[YAW].angle_get);
		}
		
    if(time_tick_1ms % 10 == 0){
			imu_update_local();
			//UART4_Oscilloscope(gimbal.joint[YAW].angle_get, gimbal.joint[YAW].angle_set,gimbal.joint[PITCH].angle_get, gimbal.joint[PITCH].angle_set);
		}
    }
}
