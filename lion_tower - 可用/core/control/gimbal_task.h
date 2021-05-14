#ifndef _GIMBAL_TASK_H_
#define _GIMBAL_TASK_H_

#include "main.h"

#define GIMBAL_JOINT_TOTAL_NUM          2
#define GIMBAL_JOINT_YAW_ID             0
#define GIMBAL_JOINT_PITCH_ID           1
#define YAW                             GIMBAL_JOINT_YAW_ID
#define PITCH                           GIMBAL_JOINT_PITCH_ID

#define RC_GIMBAL_INTEGRAL_SCALE_YAW        0.002f
#define RC_GIMBAL_INTEGRAL_SCALE_PITCH      0.0006f

#define GIMBAL_JOINT_YAW_MAX_ANGLE      60      // deg
#define GIMBAL_JOINT_YAW_MIN_ANGLE      -60     // deg
#define GIMBAL_JOINT_YAW_MAX_SPEED      500     // deg/s
#define GIMBAL_JOINT_YAW_MAX_CURRENT    25000    // mA
#define GIMBAL_JOINT_PITCH_MAX_ANGLE    60      // deg
#define GIMBAL_JOINT_PITCH_MIN_ANGLE    -60     // deg
#define GIMBAL_JOINT_PITCH_MAX_SPEED    300     // deg/sec
#define GIMBAL_JOINT_PITCH_MAX_CURRENT  25000    // mA

//#define GIMBAL_JOINT_YAW_SENSOR         mpu6050
#define GIMBAL_JOINT_YAW_SENSOR         encoder
#define GIMBAL_JOINT_PITCH_SENSOR       encoder

//#define GIMBAL_YAW_POSITION_LOOP_CONTROL
//#define GIMBAL_PITCH_POSITION_LOOP_CONTROL

typedef struct {
    uint16_t gimbal_offset_x;      // (mm) 云台中心到底盘正中心的距离在x轴上的分量
    uint16_t gimbal_offset_y;      // (mm) 云台中心到底盘正中心的距离在y轴上的分量
} gimbal_info_t;

typedef enum {
    mpu6050,
    encoder,
} joint_sensor_e;

typedef struct {
    float           angle_set;
	  float           angle_set_local; 
    float           angle_get;
    float           angle_max;
    float           angle_min;
      
	
    float           relative_angle;
    
    float           speed_set;
    float           speed_get;
    float           speed_max;
    float           speed_limit;
    
    float           current;
    
    joint_sensor_e sensor;
} gimbal_joint_t;

typedef enum {
    STOP_MODE,
    RC_MODE,
    PC_MODE,
} gimbal_mode_e;

typedef struct {
    gimbal_info_t       info;
    gimbal_joint_t      joint[GIMBAL_JOINT_TOTAL_NUM];
    
    gimbal_mode_e       control_mode;
} gimbal_t;

extern volatile gimbal_t gimbal;

void gimbal_task_init(void);
void gimbal_fsm(void);
void gimbal_control(void);
void gimbal_default_set(void);
void gimbal_defend(void);
#endif
