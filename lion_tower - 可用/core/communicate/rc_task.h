#ifndef _RC_TASK_H_
#define _RC_TASK_H_
#include "main.h"

// 整理后
#define RC_FRAME_LENGTH                     18u

#define RC_STICK_OFFSET                     1024u   
#define RC_STICK_TO_CHASSIS_SPEED_FACT      0.45f
#define RC_STICK_TO_PITCH_SPEED_FACT        0.004f
#define RC_STICK_TO_YAW_SPEED_FACT          0.0008f
#define RC_NORMALIZATION_DENOMINATOR        660.f

#define RC_MOUSE_TO_PITCH_ANGLE_FACT         0.1625f
#define RC_MOUSE_TO_YAW_ANGLE_FACT             0.1625f

#define RC_SW_UP                            1
#define RC_SW_MID                           3
#define RC_SW_DOWN                          2




// 整理前
//0.004
#define FRICTION_WHEEL_MAX_DUTY             1175

//mouse control parameters
#define FRICTION_RAMP_TICK_COUNT            100
#define MOUSE_LR_RAMP_TICK_COUNT            50
#define MOUSR_FB_RAMP_TICK_COUNT            60


#define INPUT_NONE        0
#define INPUT_RC          1
#define INPUT_PC          2

#define ASSIST_MANUAL     0
#define ASSIST_SEMI_AUTO  1
#define ASSIST_FULL_AUTO  2


typedef __packed struct
{
    /* rocker channel information */
    int16_t ch1;
    int16_t ch2;
    int16_t ch3;
    int16_t ch4;

    /* left and right lever information */
    uint8_t last_sw1;
    uint8_t last_sw2;
    uint8_t sw1;
    uint8_t sw2;

    /* mouse movement and button information */
    __packed struct
    {
        int16_t x;
        int16_t y;
        int16_t z;

        uint8_t l;
        uint8_t r;
    } mouse;

    /* keyboard key information */
    __packed union
    {
        uint16_t code;
        __packed struct 
        {
            uint16_t W:1;
            uint16_t S:1;
            uint16_t A:1;
            uint16_t D:1;
            uint16_t SHIFT:1;
            uint16_t CTRL:1;
            uint16_t Q:1;
            uint16_t E:1;
            uint16_t R:1;
            uint16_t F:1;
            uint16_t G:1;
            uint16_t Z:1;
            uint16_t X:1;
            uint16_t C:1;
            uint16_t V:1;
            uint16_t B:1;
        } bit;
    } keyboard;
} rc_msg_t;



extern volatile rc_msg_t rc_msg;
extern volatile rc_msg_t virtual_rc_msg;
extern volatile rc_msg_t rc_msg_last;
void rc_task_init(void);
void rc_msg_process(uint8_t *msg);
void virtual_rc_msg_process(uint8_t *msg);
uint8_t get_input_mode(void);

extern volatile uint8_t pitch_flag;
extern volatile uint8_t rc_yaw_flag;
extern volatile uint8_t rc_pitch_flag;
extern volatile uint8_t virtual_yaw_flag;
extern volatile uint8_t virtual_pitch_flag;
extern volatile float virtual_rc_ctrl_yaw;
extern volatile float virtual_rc_ctrl_pitch;

#endif

