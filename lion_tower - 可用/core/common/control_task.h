#ifndef _CONTROL_TASK_H_
#define _CONTROL_TASK_H_

#include "main.h"

typedef enum
{
    MSG_TRANSMIT_CHANNEL_STATUS = 0,
    MSG_TRANSMIT_CHANNEL_RC,
    MSG_TRANSMIT_CHANNEL_IMU,
    MSG_TRANSMIT_CHANNEL_POSE,
    MSG_TRANSMIT_CHANNEL_TOTAL
} msg_transmit_channel_e;

void control_task_init(void);
uint32_t get_time_tick(void);
void control_task(void);
extern uint32_t time_tick_1ms;
#endif
