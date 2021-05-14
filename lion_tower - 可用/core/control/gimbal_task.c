#include "gimbal_task.h"

volatile gimbal_t gimbal;

extern volatile encoder_t gimbal_encoder[GIMBAL_JOINT_TOTAL_NUM];
float gimbal_defend_pitch_flag=1.f;
float gimbal_defend_yaw_flag=1.f;
volatile pc_gimbal_download_t pc_gimbal_download;

extern volatile uint8_t pitch_flag;
extern volatile uint8_t rc_yaw_flag;
extern volatile uint8_t rc_pitch_flag;
extern volatile uint8_t virtual_yaw_flag;
extern volatile uint8_t virtual_pitch_flag;
extern volatile float virtual_rc_ctrl_yaw;
extern volatile float virtual_rc_ctrl_pitch;


void gimbal_task_init()
{
    memset((gimbal_t *)&gimbal, 0, sizeof(gimbal_t));
	
    
    gimbal.joint[YAW].angle_max   = GIMBAL_JOINT_YAW_MAX_ANGLE;
    gimbal.joint[YAW].angle_min   = GIMBAL_JOINT_YAW_MIN_ANGLE;
    gimbal.joint[YAW].speed_max   = GIMBAL_JOINT_YAW_MAX_SPEED;
    gimbal.joint[YAW].sensor      = GIMBAL_JOINT_YAW_SENSOR;
    
    gimbal.joint[PITCH].angle_max = GIMBAL_JOINT_PITCH_MAX_ANGLE;
    gimbal.joint[PITCH].angle_min = GIMBAL_JOINT_PITCH_MIN_ANGLE;
    gimbal.joint[PITCH].speed_max = GIMBAL_JOINT_PITCH_MAX_SPEED;
    gimbal.joint[PITCH].sensor    = GIMBAL_JOINT_PITCH_SENSOR;
	
	  
   
//    PID_struct_init(&pid_yaw_position,   POSITION_PID, ANGLE_ERR,  GIMBAL_JOINT_YAW_MAX_SPEED,   3000, 31.5, 0.15, 0.21);//GIMBAL_JOINT_YAW_MAX_SPEED 15，0，120
//	  PID_struct_init(&pid_yaw_speed,      POSITION_PID, COMMON_ERR, GIMBAL_JOINT_YAW_MAX_CURRENT,   2500, 41.0, 0.02, 0.01);//5000,7.5, 0.25, 0.4
    
//	PID_struct_init(&pid_yaw_position,   POSITION_PID, ANGLE_ERR,  GIMBAL_JOINT_YAW_MAX_SPEED,   3000, 6.61, 0.012, 0.015);//GIMBAL_JOINT_YAW_MAX_SPEED 15，0，120
//	  PID_struct_init(&pid_yaw_speed,      POSITION_PID, COMMON_ERR, GIMBAL_JOINT_YAW_MAX_CURRENT,   2500, 128.0, 0.01, 0.03);//5000,7.5, 0.25, 0.4
		PID_struct_init(&pid_yaw_position,   POSITION_PID, ANGLE_ERR,  GIMBAL_JOINT_YAW_MAX_CURRENT,   2500, 200, 0, 100);

//	    PID_struct_init(&pid_yaw_position,   POSITION_PID, ANGLE_ERR,  GIMBAL_JOINT_YAW_MAX_SPEED,   3000, 96.01, 0.53, 0.28);//GIMBAL_JOINT_YAW_MAX_SPEED 15，0，120
//	  PID_struct_init(&pid_yaw_speed,      POSITION_PID, COMMON_ERR, GIMBAL_JOINT_YAW_MAX_CURRENT,   2500, 85.01, 0.01, 0.06);//5000,7.5, 0.25, 0.4
//		    PID_struct_init(&pid_yaw_position,   POSITION_PID, ANGLE_ERR,  GIMBAL_JOINT_YAW_MAX_SPEED,   3000, 10.0, 0.00, 0.00);//GIMBAL_JOINT_YAW_MAX_SPEED 15，0，120
//	  PID_struct_init(&pid_yaw_speed,      POSITION_PID, COMMON_ERR, GIMBAL_JOINT_YAW_MAX_CURRENT,   2500, 10.0, 0.00, 0.00);//5000,7.5, 0.25, 0.4
//    PID_struct_init(&pid_pitch_position, POSITION_PID, ANGLE_ERR,  GIMBAL_JOINT_PITCH_MAX_CURRENT,   10000,500,0.21,1.81);//500, 0, 1.86);             //7.6 12, 0.0, 7
//	  PID_struct_init(&pid_pitch_speed,    POSITION_PID, COMMON_ERR, GIMBAL_JOINT_PITCH_MAX_CURRENT, 2500, 510.5,20.0,0.21);//623.5, 28.5, 0.21);              //18 13.0, 0.1, 0.1

		PID_struct_init(&pid_pitch_position, POSITION_PID, ANGLE_ERR,  GIMBAL_JOINT_PITCH_MAX_CURRENT,   25000,1500,50,1000);//500, 0, 1.86);             //7.6 12, 0.0, 7
//    PID_struct_init(&pid_pitch_position, POSITION_PID, ANGLE_ERR,  GIMBAL_JOINT_PITCH_MAX_CURRENT,   10000,386,0.22,0.08);//500, 0, 1.86);             //7.6 12, 0.0, 7
//	  PID_struct_init(&pid_pitch_speed,    POSITION_PID, COMMON_ERR, GIMBAL_JOINT_PITCH_MAX_CURRENT, 2500, 366.5,0.11,0.01);//623.5, 28.5, 0.21);              //18 13.0, 0.1, 0.1
//    PID_struct_init(&pid_pitch_position, POSITION_PID, ANGLE_ERR,  GIMBAL_JOINT_PITCH_MAX_CURRENT,   10000,20.0,0.,0.0);//500, 0, 1.86);             //7.6 12, 0.0, 7
//	  PID_struct_init(&pid_pitch_speed,    POSITION_PID, COMMON_ERR, GIMBAL_JOINT_PITCH_MAX_CURRENT, 2500, 20.5,0.0,0.0);//623.5, 28.5, 0.21);              //18 13.0, 0.1, 0.1


/*speed 可删*/      //-40.0~-90.0
}

void gimbal_get_sensor_msg_handler()
{
    switch (gimbal.joint[YAW].sensor)
    {
        case mpu6050:
            gimbal.joint[YAW].angle_get = imu.mag.z_local;
            gimbal.joint[YAW].speed_get = imu.gyro.z;
            gimbal.joint[YAW].relative_angle = gimbal_encoder[YAW].local_angle;
        
            break;
        default:
            gimbal.joint[YAW].angle_get = gimbal_encoder[YAW].total_angle;
            gimbal.joint[YAW].speed_get = gimbal_encoder[YAW].filter_rpm;
            gimbal.joint[YAW].relative_angle = 0;
        break;
    }
    
    switch (gimbal.joint[PITCH].sensor)
    {
        case mpu6050:
            gimbal.joint[PITCH].angle_get = imu.mag.x;
            gimbal.joint[PITCH].speed_get = imu.gyro.x;
            break;
        default:
            gimbal.joint[PITCH].angle_get = gimbal_encoder[PITCH].local_angle;// + 358.593811f;
            gimbal.joint[PITCH].speed_get = gimbal_encoder[PITCH].filter_rpm;
				
        break;
    }
		
}

void gimbal_fsm(void){
	  switch(work_state){
			case STOP_STATE:
				gimbal_default_set();
				break;
			case RC_NORMAL_STATE:
//				gimbal.joint[YAW].angle_set = rc_msg.ch3 * 0.1f-150.0f;
				if(rc_yaw_flag)
					gimbal.joint[YAW].angle_set = rc_msg.ch3 * 50.f/660.f + gimbal.joint[YAW].angle_get;
	//			gimbal.joint[YAW].angle_set = 0.0f;

//								gimbal.joint[YAW].angle_set += rc_msg.ch3 * 0.001f;
//				gimbal.joint[YAW].angle_set_local = abs_to_angle(gimbal.joint[YAW].angle_set);
//				gimbal.joint[YAW].speed_set = rc_msg.ch3 * 0.1f;
//				gimbal.joint[PITCH].angle_set = rc_msg.ch4 * 0.3f-80.0f;
				if(rc_pitch_flag)
					gimbal.joint[PITCH].angle_set = rc_msg.ch4 * 25.f/660.f + gimbal.joint[PITCH].angle_get;
//				gimbal.joint[PITCH].angle_set = 0.0f;
//				gimbal.joint[PITCH].angle_set += rc_msg.ch4 * 0.0003f;
				break;
			case AUTO_AIM_STATE:
//				if(work_state_last != AUTO_AIM_STATE){
   //         if(pitch_flag == 1){
						if(virtual_yaw_flag)
						{
								gimbal.joint[YAW].angle_set   = -0.3f*virtual_rc_ctrl_yaw + gimbal.joint[YAW].angle_get;
					//		gimbal.joint[YAW].angle_set_local = abs_to_angle(gimbal.joint[YAW].angle_set);
					//			gimbal.joint[YAW].speed_set   = virtual_rc_ctrl_yaw;
//							  printf("gimbal.joint[YAW].angle_set %f",gimbal.joint[YAW].angle_set);
						}
						
						if(virtual_pitch_flag)
						{
								gimbal.joint[PITCH].angle_set = -0.4f*virtual_rc_ctrl_pitch+ gimbal.joint[PITCH].angle_get;//0.12f
						//		gimbal.joint[PITCH].speed_set = virtual_rc_ctrl_pitch;
//							  printf("gimbal.joint[PITCH].angle_set %f",gimbal.joint[PITCH].angle_set);
						}
						//printf("AUTO_AIM_STATE YAW: %f, PITCH: %f\n", virtual_rc_ctrl_yaw, virtual_rc_ctrl_pitch);
	//				}
//				}
				pitch_flag = 0;
				break;
			case AUTO_NOAIM_STATE:
//				if(work_state_last != AUTO_NOAIM_STATE){
//					  gimbal.joint[PITCH].angle_set = gimbal.joint[PITCH].angle_get;
//					  gimbal.joint[YAW].angle_set   = gimbal.joint[YAW].angle_get;
//					
//				}
//				printf("AUTO_NOAIM_STATE YAW: %f, PITCH: %f\n", virtual_rc_ctrl_yaw, virtual_rc_ctrl_pitch);
				gimbal_defend();
				break;
			default:
				gimbal_default_set();
				break;
		}
}

void gimbal_control()    // pitch 160-179
{
    gimbal_get_sensor_msg_handler();
    gimbal_fsm();
    if(gimbal.joint[YAW].angle_set > -10.f)
			gimbal.joint[YAW].angle_set = -10.f;
		if(gimbal.joint[YAW].angle_set < -300.f)
			gimbal.joint[YAW].angle_set = -300.f;
		if(gimbal.joint[PITCH].angle_set < 160.f)
			gimbal.joint[PITCH].angle_set = 160.f;
		if(gimbal.joint[PITCH].angle_set > 179.f)
			gimbal.joint[PITCH].angle_set = 179.f;
    //gimbal.joint[YAW].speed_set   = pid_calc(&pid_yaw_position,gimbal.joint[YAW].angle_get, gimbal.joint[YAW].angle_set);
		//gimbal.joint[YAW].current     = pid_calc(&pid_yaw_speed,gimbal.joint[YAW].speed_get,gimbal.joint[YAW].speed_set);
   // gimbal.joint[YAW].speed_set   = pid_calc(&pid_yaw_position, gimbal.joint[YAW].angle_get, gimbal.joint[YAW].angle_set_local);
		if(work_state != STOP_STATE){
//	    gimbal.joint[YAW].speed_set   = pid_calc(&pid_yaw_position, gimbal.joint[YAW].angle_get, gimbal.joint[YAW].angle_set);
//			gimbal.joint[YAW].current     = pid_calc(&pid_yaw_speed,gimbal.joint[YAW].speed_get,gimbal.joint[YAW].speed_set);
			gimbal.joint[YAW].current     = pid_calc(&pid_yaw_position,gimbal.joint[YAW].angle_get,gimbal.joint[YAW].angle_set);
	//    gimbal.joint[PITCH].speed_set   = pid_calc(&pid_pitch_position, gimbal.joint[PITCH].angle_get, gimbal.joint[PITCH].angle_set);
			gimbal.joint[PITCH].current = pid_calc(&pid_pitch_position, gimbal.joint[PITCH].angle_get, gimbal.joint[PITCH].angle_set);

			
			// 由于安装问题，电机顺时针转时，云台实际转动方向为逆时针（正向）
			gimbal.joint[PITCH].current *= 1.f;
			gimbal.joint[YAW].current   *= 1.f;
		}
}

void gimbal_default_set(void){
	  gimbal.joint[PITCH].angle_set = gimbal.joint[PITCH].angle_get;
		gimbal.joint[YAW].angle_set   = gimbal.joint[YAW].angle_get;
}

void gimbal_defend(void){
//	gimbal.joint[PITCH].angle_set=150.f;
//	gimbal.joint[PITCH].angle_set+=0.1f*gimbal_defend_pitch_flag;
//	if((gimbal.joint[PITCH].angle_set>=179.f && gimbal_defend_pitch_flag>0)
//	|| (gimbal.joint[PITCH].angle_set<=165.f && gimbal_defend_pitch_flag<0))
//	 gimbal_defend_pitch_flag*=-1.f;
//	
//	gimbal.joint[YAW].angle_set+=1.f*gimbal_defend_yaw_flag;
//	if((gimbal.joint[YAW].angle_set>=-20.f && gimbal_defend_yaw_flag>0)
//	|| (gimbal.joint[YAW].angle_set<=-290.f && gimbal_defend_yaw_flag<0))
//	 gimbal_defend_yaw_flag*=-1.f;
}
