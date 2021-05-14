#include "main.h"

int main(void)
{
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    // core/common
    control_task_init();
    supervisor_task_init();
	  fsm_task_init();
    
    // core/communication
    can_task_init();
    rc_task_init();
    js_task_init();
    pc_task_init();
    imu_task_init();
    
    // core/control
    gimbal_task_init();
    chassis_task_init();
    shoot_task_init();
	  border_measure_init();
    UART4_Configuration();
	  BSP_Init();

    while (1)
    {


    }
}
