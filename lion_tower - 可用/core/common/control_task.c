#include "control_task.h"

uint32_t time_tick_1ms;
extern volatile float virtual_rc_ctrl_yaw;
extern volatile float virtual_rc_ctrl_pitch;

msg_transmit_channel_e msg_transmit_channel = MSG_TRANSMIT_CHANNEL_RC;


//
extern volatile encoder_t gimbal_encoder[GIMBAL_JOINT_TOTAL_NUM];
//
void control_task_init()
{
    // bsp
    TIM6_Configuration();
    
    // 变量初始化
    time_tick_1ms = 0;
}

void control_task()
{
    time_tick_1ms++;
//		if(time_tick_1ms>=10000){time_tick_1ms=0;}

	if(time_tick_1ms<=5000)
	{
	  set_chassis_current(0,0);
		set_gimbal_current(0,0,0);
	}
	else
	{
	  if (time_tick_1ms % 10 == 0){
			work_state_fsm();
		}
    if (time_tick_1ms % 10 == 1)
		  border_measure_process();
		if(work_state != STOP_STATE){
			if (time_tick_1ms % 10 == 2)
			{
				chassis_control();
				set_chassis_current(chassis.current[LF], chassis.current[LR]);
			}
			if (time_tick_1ms %10 == 3)
			{
				gimbal_control();
				shoot_control();
				//set_gimbal_current(gimbal.joint[YAW].current,gimbal.joint[PITCH].current,0);
				//set_gimbal_current(gimbal.joint[YAW].current,0,0);
				//printf("%f\n",gimbal.joint[YAW].angle_get);
				//set_gimbal_current(0,gimbal.joint[PITCH].current,0);
				set_gimbal_current(0,0,shoot.feedmotor.current);
				set_friction_current(shoot.friction.current[0], shoot.friction.current[1]);
				//USART3_Oscilloscope(gimbal.joint[YAW].angle_get+150,gimbal.joint[YAW].angle_set+150,0,0);
				//USART3_Oscilloscope(pid_pitch_position.pout,pid_pitch_position.iout,pid_pitch_position.dout,0);
				//printf("%f,%f\n",gimbal.joint[PITCH].angle_get,gimbal.joint[PITCH].angle_set);
				//printf("%f\n",chassis.speed.y_set);
				
			}
		}
		else{    //all stop
			if (time_tick_1ms % 10 == 2)
			{
				chassis_control();
				set_chassis_current(0,0);
			}
			if (time_tick_1ms %10 == 3)
			{
				gimbal_control();
				shoot_control();
				set_gimbal_current(0,0,0);
				set_friction_current(0,0);
			}
		}
		
		    //debug

	}
		

}

uint32_t get_time_tick()
{
    return time_tick_1ms;
}
