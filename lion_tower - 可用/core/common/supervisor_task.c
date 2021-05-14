#include "supervisor_task.h"

volatile offline_msg_t offline_msg;
volatile device_t device[SUPERVISOR_TOTAL];

void supervisor_task_init()
{
    memset((offline_msg_t *)&offline_msg, 0, sizeof(offline_msg_t));
    
    for (uint16_t id = SUPERVISOR_RC; id < SUPERVISOR_TOTAL; id++)
    {
        device[id].priority     =    id;
        device[id].freq_get     =    0;
        device[id].error_occur  =    0;
        device[id].enable       =    1;
    }
    
    device[SUPERVISOR_RC].freq_set   =  (short)(SUPERVISOR_PERIOD / MS_TO_SEC * 72.f);
    device[SUPERVISOR_CM1].freq_set  =  (short)(SUPERVISOR_PERIOD / MS_TO_SEC * 1000.f);
    device[SUPERVISOR_CM2].freq_set  =  (short)(SUPERVISOR_PERIOD / MS_TO_SEC * 1000.f);
    device[SUPERVISOR_CM3].freq_set  =  (short)(SUPERVISOR_PERIOD / MS_TO_SEC * 1000.f);
    device[SUPERVISOR_CM4].freq_set  =  (short)(SUPERVISOR_PERIOD / MS_TO_SEC * 1000.f);
    device[SUPERVISOR_IMU].freq_set  =  (short)(SUPERVISOR_PERIOD / MS_TO_SEC * 100.f);
    device[SUPERVISOR_JS].freq_set   =  (short)(SUPERVISOR_PERIOD / MS_TO_SEC * 100.f);
    device[SUPERVISOR_GMP].freq_set  =  (short)(SUPERVISOR_PERIOD / MS_TO_SEC * 1000.f);
    device[SUPERVISOR_GMY].freq_set  =  (short)(SUPERVISOR_PERIOD / MS_TO_SEC * 1000.f);
    device[SUPERVISOR_FEED].freq_set =  (short)(SUPERVISOR_PERIOD / MS_TO_SEC * 1000.f);
    device[SUPERVISOR_PC].freq_set   =  (short)(SUPERVISOR_PERIOD / MS_TO_SEC * 100.f);
    
    #ifdef SUPERVISOR_DEBUG_MODE
    device[SUPERVISOR_PC].enable     =  0;
    #endif
}

void supervisor_reset_handler()
{
    for (uint16_t id = SUPERVISOR_RC; id < SUPERVISOR_TOTAL; id++)
        device[id].freq_get = 0;
}

void supervisor_register_handler(uint16_t id)
{
    device[id].freq_get++;
}

void supervisor_judge_process()
{
    offline_msg.timestamp++;
    
    for (uint16_t id = SUPERVISOR_RC; id < SUPERVISOR_TOTAL; id++)
    {
        device[id].error_occur = ONLINE;
        offline_msg.device.code &= ~(0x01 << id);
        
        if (device[id].freq_get < (TOLERANCE * device[id].freq_set))
        {
            device[id].error_occur = OFFLINE;
            offline_msg.device.code |= 0x01 << id;
        }
    }
    
    supervisor_warning_process();
    supervisor_reset_handler();
}

void supervisor_warning_process()
{
    // 遥控器掉线
    if (device[SUPERVISOR_RC].enable && (offline_msg.device.bit.rc == OFFLINE))
    {
        #ifdef SUPERVISOR_DEBUG_MODE
        printf("rc offline, code:%x\n", offline_msg.device.code);
        #endif
        
        offline_msg.error_level = SERIOUS;
    }
    
    // 底盘电机掉线
    else if ((device[SUPERVISOR_CM1].enable && offline_msg.device.bit.cm1 == OFFLINE) ||
             (device[SUPERVISOR_CM2].enable && offline_msg.device.bit.cm2 == OFFLINE) ||
             (device[SUPERVISOR_CM3].enable && offline_msg.device.bit.cm3 == OFFLINE) ||
             (device[SUPERVISOR_CM4].enable && offline_msg.device.bit.cm4 == OFFLINE))
    {
        offline_msg.error_level = SERIOUS;
        
        #ifdef SUPERVISOR_DEBUG_MODE
        printf("chassis offline, code:%x\n", offline_msg.device.code);
        #endif
    }
    
    // imu掉线
    else if (device[SUPERVISOR_IMU].enable && offline_msg.device.bit.imu == OFFLINE)
    {
        offline_msg.error_level = SERIOUS;
        
        #ifdef SUPERVISOR_DEBUG_MODE
        printf("imu offline, code:%x\n", offline_msg.device.code);
        #endif
    }
    
    // 裁判系统掉线
    else if (device[SUPERVISOR_JS].enable && offline_msg.device.bit.js == OFFLINE)
    {
        offline_msg.error_level = WARNING;
        
        #ifdef SUPERVISOR_DEBUG_MODE
        printf("judge system offline, code:%x\n", offline_msg.device.code);
        #endif
    }
    
    // 云台电机掉线
    else if ((device[SUPERVISOR_GMP].enable && offline_msg.device.bit.gmp == OFFLINE) ||
             (device[SUPERVISOR_GMY].enable && offline_msg.device.bit.gmy == OFFLINE))
    {
        offline_msg.error_level = WARNING;
        
        #ifdef SUPERVISOR_DEBUG_MODE
        printf("gimbal offline, code:%x\n", offline_msg.device.code);
        #endif
    }
    
    // 拨弹电机掉线
    else if (device[SUPERVISOR_FEED].enable && offline_msg.device.bit.gmp == OFFLINE)
    {
        offline_msg.error_level = WARNING;
        
        #ifdef SUPERVISOR_DEBUG_MODE
        printf("feed motor offline, code:%x\n", offline_msg.device.code);
        #endif
    }
    
    // tx2掉线
    else if (device[SUPERVISOR_PC].enable && offline_msg.device.bit.pc == OFFLINE)
    {
        offline_msg.error_level = WARNING;
        
        #ifdef SUPERVISOR_DEBUG_MODE
        printf("tx2 offline, code:%x\n", offline_msg.device.code);
        #endif
    }
    
    else 
    {
        offline_msg.error_level = NORMAL;
        
        #ifdef SUPERVISOR_DEBUG_MODE
        printf("normal, code:%x\n", offline_msg.device.code);
        #endif
    }
}

uint8_t serious_error_exist()
{
    if (offline_msg.error_level == SERIOUS)
        return 1;
    else
        return 0;
}
