#ifndef _CAN_TASK_H_
#define _CAN_TASK_H_

#include "main.h"
#define CAN_CHASSIS_LEFT_FORWARD_FEEDBACK_MSG_ID     0x201
#define CAN_CHASSIS_LEFT_REAR_FEEDBACK_MSG_ID        0x202

#define CAN_GIMBAL_YAW_FEEDBACK_MSG_ID               0x205
#define CAN_GIMBAL_PITCH_FEEDBACK_MSG_ID             0x206

#define CAN_FRICTION_ONE_FEEDBACK_MSG_ID             0x201
#define CAN_FRICTION_TWO_FEEDBACK_MSG_ID             0x202
#define CAN_SHOOT_FEED_MOTOR_FEEDBACK_MSG_ID         0x207

#define CAN_PORT_CHASSIS                             CAN1
#define CAN_PORT_GIMBAL                              CAN2
#define CAN_PORT_SHOOT                               CAN2

#define ENCODER_RPM_COEFFICIENT                      7.32f // = 1000.f * 60.f / 8192.f

#define RATE_BUF_SIZE 6

typedef struct{
    int32_t raw_value;                // 编码器不经处理的原始值
    int32_t last_raw_value;           // 上一次编码器的原始值
    int32_t ecd_value;                // 经过处理后连续的编码器值
    int32_t diff;                     // 两次编码器之间的差值
    int32_t temp_count;               // 用于计数
    uint8_t buf_count;                // 均值滤波器的buffer下标
    int32_t ecd_bias;                 // 初始编码器值
    int32_t ecd_raw_rate;             // ͨ通过编码器计算得到的速度原始值
    int32_t rate_buf[RATE_BUF_SIZE];  // 用于均值滤波器
    int32_t round_cnt;                // 当前圈数
    float filter_rpm;                 // 速度
    float local_angle;                // 角度(-180, 180]
    float total_angle;                // 角度(-∞, ∞)
}encoder_t;

void can_task_init(void);
void get_encoder_bias(volatile encoder_t *v, CanRxMsg * msg);
void encoder_process_handler(volatile encoder_t *v, CanRxMsg * msg);
void can_receive_msg_process(CanRxMsg * msg);
void can2_receive_msg_process(CanRxMsg * msg);

void set_gimbal_current(int16_t gimbal_yaw_iq, int16_t gimbal_pitch_iq, int16_t feedmotor_iq);
void set_chassis_current(int16_t cm1_iq, int16_t cm2_iq);
void set_friction_current(int16_t friction1, int16_t friction2);
#endif

