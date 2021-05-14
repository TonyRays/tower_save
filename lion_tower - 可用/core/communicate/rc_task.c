#include "rc_task.h"

volatile rc_msg_t rc_msg;
volatile rc_msg_t rc_msg_last;
volatile rc_msg_t virtual_rc_msg;


ramp_t rc_chassis_x_speed_ramp = RAMP_GEN_DAFAULT;
ramp_t rc_chassis_y_speed_ramp = RAMP_GEN_DAFAULT;
ramp_t rc_gimbal_yaw_speed_ramp = RAMP_GEN_DAFAULT;
ramp_t rc_gimbal_pitch_speed_ramp = RAMP_GEN_DAFAULT;


volatile uint8_t pitch_flag = 0;
volatile uint8_t rc_yaw_flag = 0;
volatile uint8_t rc_pitch_flag = 0;
volatile uint8_t virtual_yaw_flag = 0;
volatile uint8_t virtual_pitch_flag = 0;
volatile float virtual_rc_ctrl_yaw = 0;
volatile float virtual_rc_ctrl_pitch = 0;


// 遥控器数据、斜坡函数等的初始化
void rc_task_init()
{
    // bsp
    USART1_Configuration(100000);
    
    // 斜坡函数初始化
    rc_chassis_x_speed_ramp.init(&rc_chassis_x_speed_ramp, 300);
    rc_chassis_y_speed_ramp.init(&rc_chassis_y_speed_ramp, 300);
    
    chassis.speed.x_max = 10000;
    chassis.speed.y_max = 10000;
    
    memset((rc_msg_t *)&rc_msg, 0, sizeof(rc_msg_t));
    memset((rc_msg_t *)&virtual_rc_msg, 0, sizeof(rc_msg_t));

    
    // 摩擦轮运行状态初始化
}

//// 遥控器控制模式处理
//void rc_remote_controller_msg_handler()
//{
//    rc_ctrl.x     =  rc_msg.ch1 * chassis.speed.x_max / RC_NORMALIZATION_DENOMINATOR;
//    rc_ctrl.y     =  rc_msg.ch2 * chassis.speed.y_max / RC_NORMALIZATION_DENOMINATOR;
//    
//    rc_ctrl.yaw   = -rc_msg.ch3 * gimbal.joint[YAW].angle_max   / RC_NORMALIZATION_DENOMINATOR;

//    rc_ctrl.pitch =  rc_msg.ch4 * gimbal.joint[PITCH].angle_max / RC_NORMALIZATION_DENOMINATOR;


//}






void rc_msg_process(uint8_t *msg)
{
    if(msg == NULL) return;
		rc_yaw_flag = 1;
		rc_pitch_flag = 1;
    rc_msg_last = rc_msg;
    rc_msg.ch1  = 0x07FF & (msg[0] | msg[1] << 8);
    rc_msg.ch2  = 0x07FF & (msg[1] >> 3 | msg[2] << 5);
    rc_msg.ch3  = 0x07FF & (msg[2] >> 6 | msg[3] << 2 | msg[4] << 10);
    rc_msg.ch4  = 0x07FF & (msg[4] >> 1 | msg[5] << 7);
    
    rc_msg.ch1  -= RC_STICK_OFFSET;
    rc_msg.ch2  -= RC_STICK_OFFSET;
    rc_msg.ch3  -= RC_STICK_OFFSET;
    rc_msg.ch4  -= RC_STICK_OFFSET;
    
    // 消抖和去除超范围的异常数据
    if(abs(rc_msg.ch1)  <= 5 || abs(rc_msg.ch1) > 660) rc_msg.ch1 = 0;
    if(abs(rc_msg.ch2)  <= 5 || abs(rc_msg.ch2) > 660) rc_msg.ch2 = 0;
    if(abs(rc_msg.ch3)  <= 5 || abs(rc_msg.ch3) > 660) rc_msg.ch3 = 0;
    if(abs(rc_msg.ch4)  <= 5 || abs(rc_msg.ch4) > 660) rc_msg.ch4 = 0;
	
		if(rc_msg.ch3 == 0) rc_yaw_flag = 0;
		if(rc_msg.ch4 == 0) rc_pitch_flag = 0;
    
    // sw1: 左  sw2: 右
    rc_msg.last_sw1 = rc_msg.sw1;
    rc_msg.last_sw2 = rc_msg.sw2;
    rc_msg.sw1 = ((msg[5] >> 4) & 0x000C) >> 2;
    rc_msg.sw2 = (msg[5] >> 4) & 0x0003;
    
    // 表示速度
    rc_msg.mouse.x = msg[6]  | msg[7] << 8;
    rc_msg.mouse.y = msg[8]  | msg[9] << 8;
    rc_msg.mouse.z = msg[10] | msg[11] << 8;

    // 1: 按下  0: 未按下
    rc_msg.mouse.l = msg[12];
    rc_msg.mouse.r = msg[13];
    
    // 共有16个键可选
    rc_msg.keyboard.code = msg[14] | msg[15] << 8;

    supervisor_register_handler(SUPERVISOR_RC);
}

void virtual_rc_msg_process(uint8_t *msg)
{
    if(msg == NULL) return;
	
    virtual_yaw_flag = 1;
    virtual_pitch_flag = 1;
    
    virtual_rc_msg.ch1  = 0x07FF & (msg[0] | msg[1] << 8);
    virtual_rc_msg.ch2  = 0x07FF & (msg[1] >> 3 | msg[2] << 5);
    virtual_rc_msg.ch3  = 0x07FF & (msg[2] >> 6 | msg[3] << 2 | msg[4] << 10);
    virtual_rc_msg.ch4  = 0x07FF & (msg[4] >> 1 | msg[5] << 7); 
    
		

    virtual_rc_msg.ch1  -= RC_STICK_OFFSET;
    virtual_rc_msg.ch2  -= RC_STICK_OFFSET;
    virtual_rc_msg.ch3  -= RC_STICK_OFFSET;
    virtual_rc_msg.ch4  -= RC_STICK_OFFSET;
    pitch_flag = 1;
		
		
    
    // 消抖和去除超范围的异常数据
    if(abs(virtual_rc_msg.ch1)  <= 5 || abs(virtual_rc_msg.ch1) > 660) virtual_rc_msg.ch1 = 0;
    if(abs(virtual_rc_msg.ch2)  <= 5 || abs(virtual_rc_msg.ch2) > 660) virtual_rc_msg.ch2 = 0;
    if(abs(virtual_rc_msg.ch3)  <= 5 || abs(virtual_rc_msg.ch3) > 660) virtual_rc_msg.ch3 = 0;
    if(abs(virtual_rc_msg.ch4)  <= 5 || abs(virtual_rc_msg.ch4) > 660) virtual_rc_msg.ch4 = 0;
	
    if(virtual_rc_msg.ch3 == 0) virtual_yaw_flag = 0;
    if(virtual_rc_msg.ch4 == 0) virtual_pitch_flag = 0;
    
    virtual_rc_ctrl_pitch = virtual_rc_msg.ch4 * 40.f / RC_NORMALIZATION_DENOMINATOR;
    virtual_rc_ctrl_yaw = -1*virtual_rc_msg.ch3 * 40.f / RC_NORMALIZATION_DENOMINATOR;
    
    // 表示速度
    virtual_rc_msg.mouse.x = msg[6]  | msg[7] << 8;
    virtual_rc_msg.mouse.y = msg[8]  | msg[9] << 8;
    virtual_rc_msg.mouse.z = msg[10] | msg[11] << 8;
    
    // 1: 按下  0: 未按下
    virtual_rc_msg.mouse.l = msg[12];
    virtual_rc_msg.mouse.r = msg[13];
    
    // 共有16个键可选
    virtual_rc_msg.keyboard.code = msg[14] | msg[15] << 8;

    if(virtual_rc_msg.mouse.z){
			  object.flag = 1;
			  object.x    = virtual_rc_msg.ch3 * 40.f / 660.f;
        object.y    = virtual_rc_msg.ch4 * 40.f / 660.f;			
		}
    else{
			  object.flag = 0;
			  object.x    = 0;
        object.y    = 0;	
		}
		
    supervisor_register_handler(SUPERVISOR_PC);
		//printf("virtual_rc_msg_process YAW: %f, PITCH: %f\n", virtual_rc_ctrl_yaw, virtual_rc_ctrl_pitch);
		
}
