#ifndef _SUPERVISOR_TASK_H_
#define _SUPERVISOR_TASK_H_

#include "main.h"

//#define SUPERVISOR_DEBUG_MODE

#define SUPERVISOR_PERIOD   1000   // ms
#define MS_TO_SEC           1000.f   
#define TOLERANCE           0.6f

typedef enum 
{
    SUPERVISOR_RC   = 0,
    SUPERVISOR_CM1,
    SUPERVISOR_CM2,
    SUPERVISOR_CM3,
    SUPERVISOR_CM4,
    SUPERVISOR_IMU,
    SUPERVISOR_JS,
    SUPERVISOR_GMP,
    SUPERVISOR_GMY,
    SUPERVISOR_FEED,
    SUPERVISOR_PC,
    SUPERVISOR_TOTAL,
		
		SUPERVISOR_FR1,
		SUPERVISOR_FR2,
} device_e;

typedef enum
{
    ONLINE = 0,
    OFFLINE,
} device_status_e;

typedef enum
{
    NORMAL,
    WARNING,
    SERIOUS,
} error_level_e;

typedef __packed struct 
{
    uint32_t timestamp;
    
    __packed union 
    {
        uint16_t code;
        __packed struct 
        {
            uint16_t rc:1;
            uint16_t cm1:1;
            uint16_t cm2:1;
            uint16_t cm3:1;
            uint16_t cm4:1;
            uint16_t imu:1;
            uint16_t js:1;
            uint16_t gmp:1;
            uint16_t gmy:1;
            uint16_t feed:1;
            uint16_t pc:1;
            uint16_t reserve1:1;
            uint16_t reserve2:1;
            uint16_t reserve3:1;
            uint16_t reserve4:1;
            uint16_t reserve5:1;
        } bit;
    } device;
    
    error_level_e error_level;
} offline_msg_t;
    
typedef __packed struct 
{
    uint16_t priority;
    uint16_t freq_set;
    uint16_t freq_get;
    uint16_t error_occur;
    uint16_t enable;
} device_t;

extern volatile offline_msg_t offline_msg;

void supervisor_task_init(void);
void supervisor_judge_process(void);

void supervisor_reset_handler(void);
void supervisor_warning_process(void);
void supervisor_register_handler(uint16_t id);

uint8_t serious_error_exist(void);
#endif
