#ifndef __MAIN_H__
#define __MAIN_H__

// application/configureation
#include "configureation.h"

// library/std
#include "math.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"

// library/st 
#include "stm32f4xx.h"

// library/rm
#include "ramp.h"
#include "common.h"

// library/lion
#include "pid.h"
#include "common.h"
#include "crc_protocal.h"

// driver/can
#include "can1.h"
#include "can2.h"
// driver/usart 
#include "usart1.h"
#include "usart2.h" 
#include "usart3.h"
#include "usart6.h"
#include "uart4.h"
// driver/timer
#include "timer.h"
#include "pwm.h"
// driver/others
#include "bsp.h"
#include "laser.h"
#include "border_measure.h"
// core/common
#include "control_task.h"
#include "fsm_task.h"
#include "supervisor_task.h"
// core/communicate
#include "can_task.h"
#include "rc_task.h"
#include "js_task.h"
#include "tx2_task.h"
#include "imu_task.h"
// core/control
#include "chassis_task.h"
#include "gimbal_task.h"
#include "shoot_task.h"

#endif 
