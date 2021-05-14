#ifndef _FSM_TASK_H_
#define _FSM_TASK_H_

#include "main.h"

#define PREPARE_TIME_TICK_MS 4000 // ms

typedef enum
{

	  RC_NORMAL_STATE,            // 手动控制底盘云台发弹
	  AUTO_NOAIM_STATE,           // 自动模式下，没有目标
	  AUTO_AIM_STATE,             // 自动模式下，发现目标
	  STOP_STATE,                 // 停止运动状态
}work_state_e;

typedef struct{
	  uint8_t flag;
	  float x;
	  float y;
}object_t;

void work_state_fsm(void);
extern work_state_e work_state;
extern work_state_e work_state_last;
extern object_t object;
void fsm_task_init(void);
#endif
