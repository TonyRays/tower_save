#include "can_task.h"

static uint32_t can_count = 0;
static uint32_t can2_count = 0;
volatile encoder_t chassis_encoder[CHASSIS_WHEEL_TOTAL_NUM];
volatile encoder_t gimbal_encoder[GIMBAL_JOINT_TOTAL_NUM];
volatile encoder_t feedmotor_encoder;
volatile encoder_t friction_encoder[FRICTION_TOTAL_NUM];
void can_task_init()
{
    // bsp
    CAN1_Configuration();
    CAN2_Configuration();
    
    // Initial encoders
    memset((encoder_t *)&chassis_encoder, 0, CHASSIS_WHEEL_TOTAL_NUM * sizeof(chassis_encoder));
    memset((encoder_t *)&gimbal_encoder, 0, GIMBAL_JOINT_TOTAL_NUM * sizeof(gimbal_encoder));
		memset((encoder_t *)&friction_encoder, 0, FRICTION_TOTAL_NUM * sizeof(friction_encoder));
    
    // YAW轴的偏差量大约为4785，这个偏差来源于电机的安装位置
    gimbal_encoder[YAW].ecd_bias = 4785;
    gimbal_encoder[YAW].ecd_value = gimbal_encoder[YAW].ecd_bias;
    gimbal_encoder[YAW].last_raw_value = gimbal_encoder[YAW].ecd_bias;
    
    // PITCH轴的偏差量大约为4758，这个偏差来源于电机的安装位置
	  gimbal_encoder[PITCH].ecd_bias = 7566;
    gimbal_encoder[PITCH].ecd_value = gimbal_encoder[PITCH].ecd_bias;
    gimbal_encoder[PITCH].last_raw_value = gimbal_encoder[PITCH].ecd_bias;
}


/**
  * @brief     保存底盘电机编码器的值作为偏差
  * @param[in] v: 对应的编码器
  * @param[in] msg: 当前can报文
  * @retval    null
  */
void get_encoder_bias(volatile encoder_t *v, CanRxMsg * msg)
{
    v->ecd_bias = (msg->Data[0] << 8) | msg->Data[1];  
    v->ecd_value = v->ecd_bias;
    v->last_raw_value = v->ecd_bias;
    v->temp_count++;
}


/**
  * @brief     处理接收到的电机电调回传信息并针对不同的电调获取不同的信息
  * @param[in] v: 对应的编码器
  * @param[in] msg: 当前can报文
  * @retval    null
  */
void encoder_process_handler(volatile encoder_t *v, CanRxMsg * msg)
{
    int32_t temp_sum = 0;
    
    v->last_raw_value   =   v->raw_value;
    v->raw_value        =   (msg->Data[0] << 8) | msg->Data[1];
    v->diff             =   v->raw_value - v->last_raw_value;

    // 两次编码器的反馈值差别太大表示圈数大概率发生了变化
    if(v->diff < -4096)    
    {
        v->round_cnt++;
        v->ecd_raw_rate = v->diff + 8192;
    }
    else if(v->diff > 4096)//如果两次编码器的值相差大于4096，，理论上是转过了一圈，但实际上在两次读取编码器值之间的时间内
			//电机是不可能转到超过半圈的，说明这是不正常的，所以要进行处理
    {
        v->round_cnt--;//圈数减1
        v->ecd_raw_rate = v->diff - 8192;//当前的编码器数值的差要diff减8192
    }
    else
    {
        v->ecd_raw_rate = v->diff;
    }

    // 计算得到连续的编码器输出值
    v->ecd_value = v->raw_value + v->round_cnt * 8192;

    // 计算得到角度值
    v->local_angle = (float)((v->raw_value - v->ecd_bias) * 360.f / 8192.f);
    v->total_angle = (float)(v->local_angle + v->round_cnt * 360.f);
		// 将local_angle的角度范围限制在(-180, 180]的范围内
		if (v->local_angle > 180)        v->local_angle -= 360;
		else if (v->local_angle <= -180) v->local_angle += 360;

    v->rate_buf[v->buf_count++] = v->ecd_raw_rate;

    if(v->buf_count == RATE_BUF_SIZE)
    {
        v->buf_count = 0;
    }

    // 计算速度平均值
    for(uint8_t i = 0;i < RATE_BUF_SIZE; i++)
    {
        temp_sum += v->rate_buf[i];
    }

    v->filter_rpm = (temp_sum == 0) ? 0 : (int32_t)(temp_sum * ENCODER_RPM_COEFFICIENT / RATE_BUF_SIZE);
}

/**
  * @brief     接收can的回传信息并根据标识符采取不同的处理办法
  * @param[in] msg: 当前can报文
  * @retval    null
  */
void can_receive_msg_process(CanRxMsg * msg)
{
    can_count++;

    switch(msg->StdId)
    {
			 case CAN_CHASSIS_LEFT_FORWARD_FEEDBACK_MSG_ID:
        {
            (can_count <= 50) 
            ? get_encoder_bias(&chassis_encoder[LF], msg)
            : encoder_process_handler(&chassis_encoder[LF], msg);
            
            supervisor_register_handler(SUPERVISOR_CM2);
        }break;

        case CAN_CHASSIS_LEFT_REAR_FEEDBACK_MSG_ID:
        {
            (can_count <= 50) 
            ? get_encoder_bias(&chassis_encoder[LR], msg)
            : encoder_process_handler(&chassis_encoder[LR], msg);
            
            supervisor_register_handler(SUPERVISOR_CM3); 
        }break;

        default: { }
    }
}

void can2_receive_msg_process(CanRxMsg * msg)
{
    can2_count++;

    switch(msg->StdId)
    {
			
        case CAN_GIMBAL_PITCH_FEEDBACK_MSG_ID:
        {
 
					   encoder_process_handler(&gimbal_encoder[PITCH], msg);
          supervisor_register_handler(SUPERVISOR_GMP); 
        }break;
				
				case CAN_GIMBAL_YAW_FEEDBACK_MSG_ID:
				{
					  
					   encoder_process_handler(&gimbal_encoder[YAW], msg);
          supervisor_register_handler(SUPERVISOR_GMY); 
				}break;
				
			 case CAN_FRICTION_ONE_FEEDBACK_MSG_ID:
        {
            (can_count <= 50) 
            ? get_encoder_bias(&friction_encoder[0], msg)
            : encoder_process_handler(&friction_encoder[0], msg);
            
            supervisor_register_handler(SUPERVISOR_FR1); 
        }break;
				
        case CAN_FRICTION_TWO_FEEDBACK_MSG_ID:
        {
            (can_count <= 50) 
            ? get_encoder_bias(&friction_encoder[1], msg)
            : encoder_process_handler(&friction_encoder[1], msg);
            
            supervisor_register_handler(SUPERVISOR_FR2); 
        }break;
			
				case CAN_SHOOT_FEED_MOTOR_FEEDBACK_MSG_ID:
				{
            (can_count <= 50) 
            ? get_encoder_bias(&feedmotor_encoder, msg)
            : encoder_process_handler(&feedmotor_encoder, msg);
            
            supervisor_register_handler(SUPERVISOR_FEED); 
				}
				
        default:
					break;
    }
}

/**
  * @brief     根据协议给底盘电调发送电流指令，波特率为1Mbps
  * @param[in] cm1_iq: 给右前轮电调的电流指令，单位为mA
  * @param[in] cm2_iq: 给左前轮电调的电流指令，单位为mA
  * @param[in] cm3_iq: 给左后轮电调的电流指令，单位为mA
  * @param[in] cm4_iq: 给右后轮电调的电流指令，单位为mA
  * @retval    null
  */
void set_chassis_current(int16_t cm1_iq, int16_t cm2_iq)
{
    CanTxMsg tx_message;

    // 标识符ID        0x200
    tx_message.StdId = 0x200;

    // 帧类型       标准帧
    tx_message.IDE = CAN_Id_Standard;

    // 帧格式       DATA
    tx_message.RTR = CAN_RTR_Data;

    // DLC          8字节
    tx_message.DLC = 0x08;

    // 电调驱动ID 0x201
    // 给定16位电流指令 (-32768 ~ +32767)
    tx_message.Data[0] = (uint8_t)(cm1_iq >> 8);
    tx_message.Data[1] = (uint8_t)cm1_iq;

    // 电调驱动ID 0x202
    // 给定16位电流指令 (-32768 ~ +32767)
    tx_message.Data[2] = (uint8_t)(cm2_iq >> 8);
    tx_message.Data[3] = (uint8_t)cm2_iq;

    // 电调驱动ID 0x203
    // 给定16位电流指令 (-32768 ~ +32767)
    tx_message.Data[4] =  0x00;
    tx_message.Data[5] =  0x00;

    // 电调驱动ID 0x204
    // 给定16位电流指令 (-32768 ~ +32767)
    tx_message.Data[6] =  0x00;
    tx_message.Data[7] =  0x00;

    CAN_Transmit(CAN_PORT_CHASSIS, &tx_message);
}

/**
  * @brief     根据协议给云台电调发送电流指令，波特率为1Mbps
  * @param[in] gimbal_yaw_iq:   给YAW轴电调的电流指令，单位为mA
  * @param[in] gimbal_pitch_iq: 给PITCH轴电调的电流指令，单位为mA
  * @retval    null
  */
void set_gimbal_current(int16_t gimbal_yaw_iq, int16_t gimbal_pitch_iq, int16_t feedmotor_iq)
{
    CanTxMsg tx_message;

    // 标识符ID        0x1FF
    tx_message.StdId = 0x1FF;

    // 帧类型       标准帧
    tx_message.IDE = CAN_Id_Standard;

    // 帧格式       DATA
    tx_message.RTR = CAN_RTR_Data;

    // DLC          8字节
    tx_message.DLC = 0x08;

    // YAW轴       0x205
    // 给定16位电流 (-5000 ~ +5000)
    tx_message.Data[0] = (uint8_t)(gimbal_yaw_iq >> 8);
    tx_message.Data[1] = (uint8_t)gimbal_yaw_iq;

    // 预留
    // 给定16位电流 (-5000 ~ +5000)
    tx_message.Data[2] = (uint8_t)(gimbal_pitch_iq >> 8);
    tx_message.Data[3] = (uint8_t)gimbal_pitch_iq;

    // 拨弹电机    0x207
    // 给定16位电流 (-5000 ~ +5000)
    tx_message.Data[4] = (uint8_t)(feedmotor_iq >> 8);
    tx_message.Data[5] = (uint8_t)feedmotor_iq ;

    // 预留
    // 给定16位电流 (-5000 ~ +5000)
    tx_message.Data[6] = 0x00;
    tx_message.Data[7] = 0x00;

    CAN_Transmit(CAN_PORT_GIMBAL, &tx_message);
}
void set_friction_current(int16_t friction1 , int16_t friction2)
{
	  CanTxMsg tx_message;

    // 标识符ID        0x200
    tx_message.StdId = 0x200;

    // 帧类型       标准帧
    tx_message.IDE = CAN_Id_Standard;

    // 帧格式       DATA
    tx_message.RTR = CAN_RTR_Data;

    // DLC          8字节
    tx_message.DLC = 0x08;
	
	  // 摩擦轮1电调驱动ID 0x201
    // 给定16位电流指令 (-32768 ~ +32767)
    tx_message.Data[0] = (uint8_t)(friction1 >> 8);
    tx_message.Data[1] = (uint8_t)friction1;

    // 摩擦轮2电调驱动ID 0x202
    // 给定16位电流指令 (-32768 ~ +32767)
    tx_message.Data[2] = (uint8_t)(friction2 >> 8);
    tx_message.Data[3] = (uint8_t)friction2;
		
		// 预留
    // 给定16位电流 (-5000 ~ +5000)
    tx_message.Data[4] = 0x00;
    tx_message.Data[5] = 0x00;
		
		// 预留
    // 给定16位电流 (-5000 ~ +5000)
    tx_message.Data[6] = 0x00;
    tx_message.Data[7] = 0x00;

    CAN_Transmit(CAN2/*CAN_PORT_SHOOT*/, &tx_message);

}

//void set_pitch_current(int16_t gimbal_pitch_iq)
//{
//    CanTxMsg tx_message;

//    // 标识符ID        0x1FF
//    tx_message.StdId = 0x1FF;

//    // 帧类型       标准帧
//    tx_message.IDE = CAN_Id_Standard;

//    // 帧格式       DATA
//    tx_message.RTR = CAN_RTR_Data;

//    // DLC          8字节
//    tx_message.DLC = 0x08;

//    // 预留       0x205
//    // 给定16位电流 (-5000 ~ +5000)
//    tx_message.Data[0] = 0x00;
//    tx_message.Data[1] = 0x00;

//    // PITCH轴     0x206
//    // 给定16位电流 (-5000 ~ +5000)
//    tx_message.Data[2] = (uint8_t)(gimbal_pitch_iq >> 8);
//    tx_message.Data[3] = (uint8_t)gimbal_pitch_iq;

//    // 预留    0x207
//    // 给定16位电流 (-5000 ~ +5000)
//    tx_message.Data[4] = 0x00;
//    tx_message.Data[5] = 0x00;

//    // 预留
//    // 给定16位电流 (-5000 ~ +5000)
//    tx_message.Data[6] = 0x00;
//    tx_message.Data[7] = 0x00;

//    CAN_Transmit(CAN2/*CAN_PORT_PITCH*/, &tx_message);
//}
