#ifndef _CHASSIS_TASK_H_
#define _CHASSIS_TASK_H_

#include "main.h"
#define chassis_power_limit 20.f
#define CHASSIS_WHEEL_TOTAL_NUM          4
//#define CHASSIS_WHEEL_RIGHT_FORWARD_ID   0
//#define CHASSIS_WHEEL_LEFT_FORWARD_ID    1
//#define CHASSIS_WHEEL_LEFT_REAR_ID       2
//#define CHASSIS_WHEEL_RIGHT_REAR_ID      3
#define RF                              0
#define LF                              1
#define LR                              2
#define RR                              3

// chassis structure
#define CHASSIS_WHEEL_BASE               285 // mm  车轮轴心到底盘中心的距离
#define CHASSIS_WHEEL_PERIMETER          478 // mm  车轮周长
#define CHASSIS_ROTATE_RATIO             8.34f // (CHASSIS_WHEEL_BASE  / RAD_TO_DEG)
#define CHASSIS_WHEEL_RPM_RARIO          2.39f // (SEC_PER_MIN / (chassis.info.wheel_perimeter * CHASSIS_DECELE_RATIO))

#define CHASSIS_RPM_TO_MPS               0.42f // mm/s  = 1 / CHASSIS_WHEEL_RPM_RARIO

#define CHASSIS_SPEED_FACT_FAST          9
#define CHASSIS_SPEED_FACT_NORMAL        5
#define CHASSIS_SPEED_FACT_SLOW          3

#define CHASSIS_WHEEL_MAX_RPM            8500
#define CHASSIS_DECELE_RATIO             (1.f/19.f)
#define CHASSIS_MAX_SPEED_X              3000  // mm/s
#define CHASSIS_MAX_SPEED_Y              8000  // mm/s
#define CHASSIS_MAX_SPEED_Z              1000   // deg/s

#define CHASSIS_ROTATE_MAX_CURRENT       40000 // mA
#define CHASSIS_WHEEL_MAX_CURRENT        8000

#define CHASSIS_CTRL_PERIOD              10    // ms
#define TIME_TO_WAIT                     20    // 10ms

typedef struct {
    uint16_t wheel_base;                // mm 轮子轴心到底盘正中心的距离
    uint16_t wheel_perimeter;           // mm 轮子周长
} chassis_info_t;

typedef struct {
    // 对于底盘整体而言
    float x_set;
    float x_get;
    float x_max;
    float x_limit;
    
    float y_set;
    float y_get;
    float y_max;
    float y_limit;
    
    float z_set;
    float z_get;
    float z_max;
    float z_limit;
    
    // 对于每个轮子而言
    float rpm_set[CHASSIS_WHEEL_TOTAL_NUM];
    float rpm_get[CHASSIS_WHEEL_TOTAL_NUM];
} chassis_speed_t;

typedef struct {
    // 对于底盘整体而言
    float x_set;
    float x_get;
    
    float y_set;
    float y_get;
    
    float z_set;
    float z_get;
    
    // 对于每个轮子而言
    float angel_set[CHASSIS_WHEEL_TOTAL_NUM];
    float angel_get[CHASSIS_WHEEL_TOTAL_NUM];
    
    float distance_set[CHASSIS_WHEEL_TOTAL_NUM];
    float distance_get[CHASSIS_WHEEL_TOTAL_NUM];
} chassis_position_t;

typedef enum 
{
    CHASSIS_LEFT,
	  CHASSIS_RIGHT,
	  CHASSIS_STOP,
} chassis_mode_e;

typedef struct {
    chassis_info_t       info;
    chassis_position_t   position;
    chassis_speed_t      speed;
    float                current[CHASSIS_WHEEL_TOTAL_NUM];
    
    chassis_mode_e       mode;
} chassis_t;


extern volatile chassis_t chassis;

void chassis_task_init(void);
void chassis_get_sensor_msg_handler(void);
void chassis_control(void);
void chassis_fsm(void);
void chassis_default_set(void);
void chassis_defend(void);
void chassis_mecanum_calc(float vx, float vy, float vw, float speed[]);
void power_limit_handler(void);
#endif
