#include "border_measure.h"
u8 left_measure_flag[] = {0,0,0};
u8 right_measure_flag[]= {0,0,0};
u8 left_measure_flag_index=0,right_measure_flag_index=0;
void border_measure_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_4 | GPIO_Pin_5;//管脚设置
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化结构体
}

void border_measure_process(void)
{
  left_measure_flag[left_measure_flag_index++]=border_flag_A;
	left_measure_flag_index%=3;
	right_measure_flag[right_measure_flag_index++]=border_flag_B;
	right_measure_flag_index%=3;
}
