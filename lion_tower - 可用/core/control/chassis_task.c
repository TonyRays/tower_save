#include "chassis_task.h"

volatile chassis_t chassis;
volatile int time_to_go = 0;

extern volatile encoder_t chassis_encoder[CHASSIS_WHEEL_TOTAL_NUM];

float current_limit=0.f;

void chassis_task_init()
{
    memset((chassis_t *)&chassis, 0, sizeof(chassis_t));
    chassis.info.wheel_base      =   CHASSIS_WHEEL_BASE;
    chassis.info.wheel_perimeter =   CHASSIS_WHEEL_PERIMETER;
    chassis.speed.x_max          =   CHASSIS_MAX_SPEED_X;
    chassis.speed.y_max          =   CHASSIS_MAX_SPEED_Y;
    chassis.speed.z_max          =   CHASSIS_MAX_SPEED_Z;
   	chassis.mode         =   CHASSIS_STOP;
    
    for (uint8_t wheel = 0; wheel < CHASSIS_WHEEL_TOTAL_NUM; wheel++)
        PID_struct_init(&pid_wheel_speed[wheel], POSITION_PID, COMMON_ERR, CHASSIS_WHEEL_MAX_CURRENT, 10000, 4.5, 0.05, 0);
    
    PID_struct_init(&pid_chassis_angle, POSITION_PID, ANGLE_ERR, CHASSIS_MAX_SPEED_Z, 50, 5.f, 0.f, 0.f);
}

void chassis_get_sensor_msg_handler()
{
    for (uint8_t wheel = 0; wheel < CHASSIS_WHEEL_TOTAL_NUM; wheel++)
        chassis.speed.rpm_get[wheel] = chassis_encoder[wheel].filter_rpm;
    chassis.speed.x_get = 0;//0.25f * (-chassis.speed.rpm_get[RF] + chassis.speed.rpm_get[LF] - chassis.speed.rpm_get[LR] + chassis.speed.rpm_get[RR]) * CHASSIS_RPM_TO_MPS;
    chassis.speed.y_get = 0.5f * ( chassis.speed.rpm_get[LR] + chassis.speed.rpm_get[LF] ) * CHASSIS_RPM_TO_MPS;
    chassis.speed.z_get = 0;//0.25f * ( chassis.speed.rpm_get[RF] - chassis.speed.rpm_get[LF] - chassis.speed.rpm_get[LR] + chassis.speed.rpm_get[RR]) * CHASSIS_RPM_TO_MPS;
}


void chassis_fsm(void){
	  switch(work_state){
			case STOP_STATE:
				chassis_default_set();
				break;
			case RC_NORMAL_STATE:
				chassis.speed.y_set = rc_msg.ch2 * chassis.speed.y_max / RC_NORMALIZATION_DENOMINATOR;
				break;
			case AUTO_AIM_STATE:
					chassis_defend();
				break;
			case AUTO_NOAIM_STATE:
			  	chassis_defend();
				break;
			default:
				chassis_default_set();
				break;
		}
 
}
void chassis_control()
{
    
    chassis_get_sensor_msg_handler();
	  chassis_fsm();
    
    for (uint8_t wheel = 0; wheel < CHASSIS_WHEEL_TOTAL_NUM; wheel++)
		  chassis.current[wheel] = pid_calc(&pid_wheel_speed[wheel], chassis.speed.rpm_get[wheel], chassis.speed.y_set);//直接写遥控器的输入
		if(power_heat_data.chassisPpower>chassis_power_limit)
		{
		  chassis.current[LF]*=(chassis_power_limit/power_heat_data.chassisPpower);
			chassis.current[LR]*=(chassis_power_limit/power_heat_data.chassisPpower);
		}
		time_to_go--;
		if(time_to_go <= 0)
			time_to_go = 0;
//    power_limit_handler();
		//printf("%d\n",time_to_go);
}


void chassis_default_set(void){
	  chassis.speed.y_set = 0;
}


void chassis_defend(void){
	chassis_mode_e last_mode = chassis.mode;
	if(left_measure_flag[0]&&left_measure_flag[1]&&left_measure_flag[2])
	{
		chassis.mode=CHASSIS_RIGHT;
		if(right_measure_flag[0]&&right_measure_flag[1]&&right_measure_flag[2])
		chassis.mode = CHASSIS_STOP;
	}
	else if(right_measure_flag[0]&&right_measure_flag[1]&&right_measure_flag[2])
		chassis.mode=CHASSIS_LEFT;
	else if(chassis.mode == CHASSIS_STOP&&!(right_measure_flag[0]&&right_measure_flag[1]&&right_measure_flag[2]))
		chassis.mode = CHASSIS_RIGHT;
	else if(chassis.mode == CHASSIS_STOP&&!(left_measure_flag[0]&&left_measure_flag[1]&&left_measure_flag[2]))
		chassis.mode = CHASSIS_LEFT;
	
	if(last_mode != chassis.mode)
		time_to_go = TIME_TO_WAIT;
	//如果发现底盘方向反了，将chassis
	//printf("%d\n",time_to_go);
	if(time_to_go == 0){
		if(chassis.mode==CHASSIS_LEFT) chassis.speed.y_set=-4000;
		else if(chassis.mode==CHASSIS_RIGHT) chassis.speed.y_set=4000;
	}
	else chassis.speed.y_set=0;
}


float total_cur_limit;
float total_cur;
#define WARNING_ENERGY 40.f

void power_limit_handler(void)
{
  if (offline_msg.device.bit.js)
  {
    //judge system offline, mandatory limit current
    total_cur_limit = 8000.f;
  }
  else
  {
    if (power_heat_data.chassisPowerBuffer < WARNING_ENERGY)
      total_cur_limit = CHASSIS_WHEEL_TOTAL_NUM * CHASSIS_WHEEL_MAX_CURRENT
                      * ((power_heat_data.chassisPowerBuffer * power_heat_data.chassisPowerBuffer) 
                      /  (WARNING_ENERGY * WARNING_ENERGY));
    else
      total_cur_limit = CHASSIS_WHEEL_TOTAL_NUM * CHASSIS_WHEEL_MAX_CURRENT;
  }
  
  total_cur = fabs(chassis.current[RF]) 
            + fabs(chassis.current[LF]) 
            + fabs(chassis.current[LR]) 
            + fabs(chassis.current[RR]);
  
    if (total_cur > total_cur_limit)
    {
        chassis.current[RF] = chassis.current[RF] / total_cur * total_cur_limit;
        chassis.current[LF] = chassis.current[LF] / total_cur * total_cur_limit;
        chassis.current[LR] = chassis.current[LR] / total_cur * total_cur_limit;
        chassis.current[RR] = chassis.current[RR] / total_cur * total_cur_limit;
    }
}
