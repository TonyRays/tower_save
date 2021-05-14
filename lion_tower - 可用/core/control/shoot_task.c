#include "shoot_task.h"

volatile shoot_t shoot;
extern volatile encoder_t feedmotor_encoder;
extern volatile encoder_t friction_encoder[FRICTION_TOTAL_NUM];//摩擦轮编码器，用于更新摩擦轮speed_get
uint8_t sw1_last = RC_SW_DOWN;

uint32_t shoot_time_shootout = 0;


void shoot_task_init()
{
    // bsp
    Laser_Configuration();
    
    // 变量初始化
    memset((shoot_t *)&shoot, 0, sizeof(shoot_t));
    
    // 初始化拨弹轮
//    PID_struct_init(&pid_feed_position, POSITION_PID, COMMON_ERR, FEED_MOTOR_MAX_SPEED, 10000, 4.1, 0.02, 0.01);
//    PID_struct_init(&pid_feed_speed, POSITION_PID, COMMON_ERR, FEED_MOTOR_MAX_CURRENT, 10000, 100, 1, 0.02);
	    //PID_struct_init(&pid_feed_position, POSITION_PID, COMMON_ERR, FEED_MOTOR_MAX_SPEED, 10000, 1.5, 0.01, 0.01);
    PID_struct_init(&pid_feed_speed, POSITION_PID, COMMON_ERR, FEED_MOTOR_MAX_CURRENT, 10000, 50, 0.1, 0.01);

    // 初始化摩擦轮
    PID_struct_init(&pid_friction_speed[0], POSITION_PID, COMMON_ERR, FRICTION_MOTOR_MAX_CURRENT , 10000, 5.5, 0.05, 2.5);//2.7 0.05 1.2
    PID_struct_init(&pid_friction_speed[1], POSITION_PID, COMMON_ERR, FRICTION_MOTOR_MAX_CURRENT , 10000, 5.5, 0.05, 2.5);//2.7 0.05 1.2  2.5 , 0.2, 3.0
	
	
}

void shoot_fsm(void){
	  if(shoot.bullet_count_set < shoot.bullet_count_get) shoot.bullet_count_set = shoot.bullet_count_get;
	
	  switch(work_state){
			case RC_NORMAL_STATE:
				printf("%d\n",rc_msg.sw1);
			  if(rc_msg.sw1 == RC_SW_DOWN){
					//shoot.shoot_mode = SHOOT_NOSHOOT;
					shoot.feedmotor.speed_set = 0;
					shoot.friction.speed_set[0] = 0;
					shoot.friction.speed_set[1] = 0;
				}					
				else{
          shoot.friction.speed_set[0] = -6000;
					shoot.friction.speed_set[1] = 6000;
					/*if(((rc_msg.sw1 == RC_SW_UP && sw1_last == RC_SW_MID) ) && !shoot.finish_flag){
						shoot.bullet_count_set++;
						shoot.finish_flag = 1;
						shoot_time_shootout = time_tick_1ms;
					}*/
					if((rc_msg.sw1 == RC_SW_UP)){
						shoot.feedmotor.speed_set = 60;
					}
					
				}
				break;
			case AUTO_NOAIM_STATE:
				if(rc_msg.sw1 == RC_SW_DOWN){
					//shoot.shoot_mode = SHOOT_NOSHOOT;
					shoot.feedmotor.speed_set = 0;
					shoot.friction.speed_set[0] = 0;
					shoot.friction.speed_set[1] = 0;
				}					
				else{
          //shoot.friction.speed_set[0] = -6000;
					//shoot.friction.speed_set[1] = 6000;
//					if((rc_msg.sw1 == RC_SW_UP && sw1_last == RC_SW_MID) && !shoot.finish_flag){
//						shoot.bullet_count_set++;
//						shoot.finish_flag = 1;
//						shoot_time_shootout = time_tick_1ms;
//					}
					
				}
				break;
			case AUTO_AIM_STATE:
				if(object.flag){
					shoot.friction.speed_set[0] = -6000;
					shoot.friction.speed_set[1] = 6000;
					if(object.x < 3 && !shoot.finish_flag){
						shoot.feedmotor.speed_set = 60;
					}
				}
				else if(!object.flag){
					shoot.friction.speed_set[0] = 0;
					shoot.friction.speed_set[1] = 0;
				}
				break;
			default:
				shoot_default_set();
				break;
		}
		
		if(pid_feed_speed.iout > 6000){
			pid_feed_speed.iout = -6000;
		}
		sw1_last = rc_msg.sw1;
}

void shoot_get_sensor_msg_handler(void){
	 for(int ii = 0; ii < FRICTION_TOTAL_NUM; ii++){
		 shoot.friction.speed_get[ii] = friction_encoder[ii].filter_rpm;
	 }
	 shoot.feedmotor.speed_get = feedmotor_encoder.filter_rpm/36.f;
	 shoot.feedmotor.angle_get_total = feedmotor_encoder.total_angle/36.f;
	 
	 //只加不减
//	 shoot.feedmotor.count = ((int32_t)(shoot.feedmotor.angle_get_total / 36.f)) > shoot.feedmotor.count ? ((int32_t)(shoot.feedmotor.angle_get_total / 36.f)) : shoot.feedmotor.count;
//	 
//	 
//	 shoot.bullet_count_get = shoot.feedmotor.count; //管子里的子弹数量
//	 if(shoot.bullet_count_set == shoot.bullet_count_get){
//		   shoot.finish_flag = 0;//完成一次发射
//	 }		 
}

void shoot_control(void){
	  shoot_get_sensor_msg_handler();
	  shoot_fsm();
	  
	  shoot.feedmotor.current = pid_calc(&pid_feed_speed, shoot.feedmotor.speed_get, shoot.feedmotor.speed_set);
	  shoot.friction.current[0] = pid_calc(&pid_friction_speed[0], shoot.friction.speed_get[0], shoot.friction.speed_set[0]);
	  shoot.friction.current[1] = pid_calc(&pid_friction_speed[1], shoot.friction.speed_get[1], shoot.friction.speed_set[1]);
	printf("%f, %f, %f\n",shoot.friction.current[0],shoot.friction.speed_set[0],shoot.friction.speed_get[0]);
}

void shoot_default_set(void){
	  shoot.feedmotor.speed_set = 0;
		shoot.finish_flag = 0;
	  shoot.friction.speed_set[0] = 0;
	  shoot.friction.speed_set[1] = 0;
}


