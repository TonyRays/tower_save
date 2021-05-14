#ifndef _JS_TASK_H_
#define _JS_TASK_H_

#include "main.h"

#define JS_MSG_MAX_LENGTH                      138u //9+128  +1
#define JS_MSG_SOF                             0xA5  

#define JS_GAME_STATE_CMD_ID                   0x0001
#define JS_GAME_RESULT_CMD_ID                  0x0002
#define JS_GAME_ROBOT_SURVIVORS_CMD_ID         0x0003
#define JS_EVENT_DATA_CMD_ID                   0x0101
#define JS_SUPPLY_PROJECTILE_ACTION_CMD_ID     0x0102
#define JS_SUPPLY_PROJECTILE_BOOKING_CMD_ID    0x0103
#define JS_GAME_ROBOT_STATE_CMD_ID             0x0201
#define JS_GAME_POWER_HEAT_DATA_CMD_ID         0x0202
#define JS_GAME_ROBOT_POSITION_CMD_ID          0x0203
#define JS_BUFF_MUSK_CMD_ID                    0x0204
#define JS_AERIAL_ROBOT_ENERGY_CMD_ID          0x0205
#define JS_ROBOT_HURT_CMD_ID                   0x0206
#define JS_SHOOT_DATA_CMD_ID                   0x0207
#define JS_RECEIVE_AND_SHOW_DATA_CMD_ID        0x0301


typedef __packed struct
{
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
} js_game_state_t;

typedef __packed struct
{
    uint8_t winner;
} js_game_result_t;

typedef __packed struct
{
    uint16_t robot_legion;
} js_game_robot_survivors_t;

typedef __packed struct
{
    uint32_t event_type;
} js_event_data_t;

typedef __packed struct
{
    uint8_t supply_projectile_id;
    uint8_t supply_robot_id;
    uint8_t supply_projectile_step;
    uint8_t supply_projectile_num;
} js_supply_projectile_action_t;

typedef __packed struct
{
    uint8_t supply_projectile_id;
    uint8_t supply_robot_id;
    uint8_t supply_num;
} js_supply_projectile_booking_t;

typedef __packed struct
{
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t remain_HP;
    uint16_t max_HP;
    uint16_t shooter_heat0_cooling_rate;
    uint16_t shooter_heat0_cooling_limit;
    uint16_t shooter_heat1_cooling_rate;
    uint16_t shooter_heat1_cooling_limit;
    uint8_t mains_power_gimbal_output : 1;
    uint8_t mains_power_chassis_output : 1;
    uint8_t mains_power_shooter_output : 1;
} js_game_robot_state_t;

typedef __packed struct
{
    uint16_t chassisVolt;
    uint16_t chassisCurrent;
    float chassisPpower;
    uint16_t chassisPowerBuffer;
    uint16_t shooterHeat0;
    uint16_t shooterHeat1;
} js_power_heat_data_t;

typedef __packed struct
{
    float x;
    float y;
    float z;
    float yaw;
} js_game_robot_pos_t;

typedef __packed struct
{
    uint8_t power_rune_buff;
}js_buff_musk_t;

typedef __packed struct
{
    uint8_t energy_point;
    uint8_t attack_time;
} js_aerial_robot_energy_t;

typedef __packed struct
{
    uint8_t armor_id : 4;
    uint8_t hurt_type : 4;
} js_robot_hurt_t;

typedef __packed struct
{
    uint8_t bullet_type;
    uint8_t bullet_freq;
    float bullet_speed;
} js_shoot_data_t;

typedef __packed struct
{
    uint16_t data_cmd_id;
    uint16_t send_ID;
    uint16_t receiver_ID;
} js_student_data_header_t;
 
typedef __packed struct
{
    float data1;
    float data2;
    float data3;
    uint8_t mask;
} js_client_custom_data_t;

typedef __packed struct
{
    uint8_t* data;
} js_robot_interactive_data_t;

void js_task_init(void);
void js_download_msg_process(uint8_t);
void js_upload_msg_process(float data1, float data2, float data3, uint8_t mask);
void js_msg_upack_handler(uint8_t * buffer);


extern volatile js_game_state_t                 game_state;
extern volatile js_game_result_t                game_result;
extern volatile js_game_robot_survivors_t       game_robot_survivors;
extern volatile js_event_data_t                 event_data;
extern volatile js_supply_projectile_action_t   supply_projectile_action;
extern volatile js_supply_projectile_booking_t  supply_projectile_booking;
extern volatile js_game_robot_state_t           game_robot_state;
extern volatile js_power_heat_data_t            power_heat_data;
extern volatile js_game_robot_pos_t             game_robot_pos;
extern volatile js_buff_musk_t                  buff_musk;
extern volatile js_aerial_robot_energy_t        aerial_robot_energy;
extern volatile js_robot_hurt_t                 robot_hurt;
extern volatile js_shoot_data_t                 shoot_data;
extern volatile js_student_data_header_t        student_data_header;
extern volatile js_client_custom_data_t         custom_data;
extern volatile js_robot_interactive_data_t     robot_interactive_data;

#endif

