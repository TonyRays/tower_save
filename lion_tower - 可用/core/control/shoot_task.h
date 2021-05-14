#ifndef _SHOOT_TASK_H_
#define _SHOOT_TASK_H_

#include "main.h"

#define NUMBER_OF_BULLETS_PER_ROUND 10
#define FRICTION_TOTAL_NUM          2

#define SHOOT_MAX_FREQUENCE         20      // per sec
#define SHOOT_MAX_SPEED             5400    // deg/sec
#define FEED_MOTOR_MAX_SPEED        7200    // deg/sec  (= SHOOT_MAX_FREQUENCE * 360)
#define FEED_MOTOR_MAX_CURRENT      1000   // mA
#define FEED_MOTOR_DECELE_RATIO     0.028f  // (= 1.0 / 36.0)
#define FRICTION_MOTOR_MAX_CURRENT  10000   // mA





typedef struct {
    float           angle_set_total;
    float           angle_get_total;
    float           angle_last_total;
    
    float           speed_set;
    float           speed_get;
    float           speed_max;
    float           speed_limit;
    
    float           current;
	  int32_t         count;    //摩擦轮拨的子弹的计数

} feedmotor_t;

   

typedef enum
{
	  SHOOT_NOSHOOT,   //不开摩擦轮状态
	  SHOOT_WAITING,   //摩擦轮开启但不发子弹的等待状态
	  SHOOT_SHOOTING,  //摩擦轮开启，发子弹的拨弹轮旋转状态
	  SHOOT_STUCk,     //拨弹轮卡弹状态
} shoot_mode_e;

typedef struct{
    float   	   speed_set[FRICTION_TOTAL_NUM];
    float 	     speed_get[FRICTION_TOTAL_NUM];
    float        speed_max;
    float        speed_limit;

    float        current[FRICTION_TOTAL_NUM];
} friction_t;

typedef struct {
    float                   frequence;  // 射频
    float                   speed;      // 射速
    shoot_mode_e            shoot_mode; // 射击模式
    
    feedmotor_t             feedmotor;

    friction_t              friction;
	  
	  int32_t                 bullet_count_set;
	  int32_t                 bullet_count_get;
	
    uint8_t                 stuck_time;
	  uint8_t                 finish_flag;
} shoot_t;

extern volatile shoot_t shoot;
	
void shoot_task_init(void);

void shoot_control(void);

void shoot_fsm(void);

void shoot_default_set(void);

#endif
