#include "js_task.h"

volatile js_game_state_t                 game_state;
volatile js_game_result_t                game_result;
volatile js_game_robot_survivors_t       game_robot_survivors;
volatile js_event_data_t                 event_data;
volatile js_supply_projectile_action_t   supply_projectile_action;
volatile js_supply_projectile_booking_t  supply_projectile_booking;
volatile js_game_robot_state_t           game_robot_state;
volatile js_power_heat_data_t            power_heat_data;
volatile js_game_robot_pos_t             game_robot_pos;
volatile js_buff_musk_t                  buff_musk;
volatile js_aerial_robot_energy_t        aerial_robot_energy;
volatile js_robot_hurt_t                 robot_hurt;
volatile js_shoot_data_t                 shoot_data;
volatile js_student_data_header_t        student_data_header;
volatile js_client_custom_data_t         custom_data;
volatile js_robot_interactive_data_t     robot_interactive_data;

/**
 * 由于裁判系统回传的信息可能存在丢帧、一帧分多段发的情况
 * 所以需要先堆接收到的信息进行整合，完成数据接收并校验通过后才进行解码
 */

// 回传信息接收buffer
uint8_t js_tx_buffer[JS_MSG_MAX_LENGTH + 1];
uint8_t js_rx_buffer[JS_MSG_MAX_LENGTH + 1];

// 帧长
uint16_t js_tx_msg_length = JS_MSG_MAX_LENGTH;
uint16_t js_rx_msg_length = JS_MSG_MAX_LENGTH;

// buffer当前正在压入数据的index，压入完成后index自增1
uint8_t js_rx_buffer_index = 0;

// 数据包Data段数据长度
uint16_t data_segment_length = 0;

// 数据包帧头校验flag，校验通过为1，不通过为0
uint8_t frame_header_available = 0;

// 数据包全帧校验flag，校验通过为1，不通过为0
uint8_t frame_total_available = 0;

// 上传到裁判系统的数据包序号
static uint8_t js_tx_seq = 0;

void js_task_init()
{
    // bsp
    USART2_Configuration();
    
    // 变量初始化
    memset((js_game_state_t *)                &game_state,               0, sizeof(js_game_state_t));
		memset((js_game_result_t *)               &game_result,              0, sizeof(js_game_result_t));
		memset((js_game_robot_survivors_t *)      &game_robot_survivors,     0, sizeof(js_game_robot_survivors_t));
		memset((js_event_data_t *)                &event_data,               0, sizeof(js_event_data_t));
		memset((js_supply_projectile_action_t *)  &supply_projectile_action, 0, sizeof(js_supply_projectile_action_t));
		memset((js_supply_projectile_booking_t *) &supply_projectile_booking,0, sizeof(js_supply_projectile_booking_t));
		memset((js_game_robot_state_t *)          &game_robot_state,         0, sizeof(js_game_robot_state_t));
		memset((js_power_heat_data_t *)           &power_heat_data,          0, sizeof(js_power_heat_data_t));
    memset((js_game_robot_pos_t *)            &game_robot_pos,           0, sizeof(js_game_robot_pos_t));
		memset((js_buff_musk_t *)                 &buff_musk,                0, sizeof(js_buff_musk_t));
		memset((js_aerial_robot_energy_t *)       &aerial_robot_energy,      0, sizeof(js_aerial_robot_energy_t));
		memset((js_robot_hurt_t *)                &robot_hurt,               0, sizeof(js_robot_hurt_t));
		memset((js_shoot_data_t *)                &shoot_data,               0, sizeof(js_shoot_data_t));
		memset((js_student_data_header_t *)       &student_data_header,      0, sizeof(js_student_data_header_t));
		memset((js_client_custom_data_t *)        &custom_data,       0, sizeof(js_client_custom_data_t));
		memset((js_robot_interactive_data_t *)    &robot_interactive_data,   0, sizeof(js_robot_interactive_data_t));
}


/**
  * @brief     裁判系统下载信息处理函数
  * @param[in] msg: 接收到的字节流信息
  * @retval    null
  */
void js_download_msg_process(uint8_t msg)
{
    // 遇到帧头JS_MSG_SOF，初始化buffer
    if(msg == JS_MSG_SOF)
    {
        js_rx_buffer_index = 0;
        js_rx_msg_length = JS_MSG_MAX_LENGTH;
    }
    
    // 接受信息直到一帧结束
    if (js_rx_buffer_index < JS_MSG_MAX_LENGTH && js_rx_buffer_index < js_rx_msg_length)
        js_rx_buffer[js_rx_buffer_index++] = msg;
    
    // 接收完帧头信息，校验帧头，校验通过时获取帧长信息
    if (js_rx_buffer_index == UART_MSG_OFFSET_CMD_ID)
    {
        if (verify_crc8_check_sum(js_rx_buffer, UART_MSG_OFFSET_CMD_ID))
            frame_header_available = 1;
        else
        {
            frame_header_available = 0;
            return;
        }
        
        if (frame_header_available)
        {
            data_segment_length = (js_rx_buffer[2] << 8) | (js_rx_buffer[1] & 0xff);
            js_rx_msg_length = data_segment_length + UART_MSG_OFFSET_DATA + 2;
        }
    }
    
    // 接收完全帧信息，校验全帧，只有帧头和全帧校验都通过才对数据帧进行解包
    if (frame_header_available && js_rx_buffer_index >= js_rx_msg_length)
    {
        frame_header_available = 0;
        frame_total_available = 0;
        
        if (verify_crc16_check_sum(js_rx_buffer, js_rx_msg_length))
            frame_total_available = 1;
        else
        {
            frame_total_available = 0;
            return;
        }
        
        if (frame_total_available)
        {
            js_msg_upack_handler(js_rx_buffer);
            
            supervisor_register_handler(SUPERVISOR_JS);
        }
    }
}


/**
  * @brief     裁判系统上传信息处理函数
  * @param[in] d1: 
  * @param[in] d2: 
  * @param[in] d3: 
  * @param[in] mask: 
  * @retval    null
  */
void js_upload_msg_process(float d1, float d2, float d3, uint8_t mask)
{
    // 变量声明
    uint16_t upload_msg_length = 0;
    
    // 清空相关变量
    memset(js_tx_buffer, 0, sizeof(js_tx_buffer));
    memset((js_client_custom_data_t *)&custom_data, 0, sizeof(js_client_custom_data_t));
    
    // 赋值
    custom_data.data1 = d1;
    custom_data.data2 = d2;
    custom_data.data3 = d3;
    custom_data.mask  = mask;
    
    // 数据包整理
    upload_msg_length = uart_msg_pack_handler(js_tx_buffer, 
                                              JS_MSG_SOF,
                                              sizeof(js_client_custom_data_t), 
                                              js_tx_seq++,
                                              JS_RECEIVE_AND_SHOW_DATA_CMD_ID, 
                                              (js_client_custom_data_t *)&custom_data);
    
    // 通过USART2发送结构体
    USART2_PrintBlock(js_tx_buffer, upload_msg_length);
}

/**
  * @brief     裁判系统解包函数
  * @param[in] buffer: 完整的一帧信息
  * @retval    null
  */
void js_msg_upack_handler(uint8_t * buffer)
{   
    switch ((buffer[UART_MSG_OFFSET_CMD_ID]&0xff)|(buffer[UART_MSG_OFFSET_CMD_ID+1]<<8))
    {
        case JS_GAME_STATE_CMD_ID:
            memcpy((js_game_robot_state_t *)&game_robot_state, 
                    buffer + UART_MSG_OFFSET_DATA, 
                    data_segment_length);
        break;
        
        case JS_GAME_RESULT_CMD_ID:
                    memcpy((js_game_result_t *)&game_result, 
                            buffer + UART_MSG_OFFSET_DATA, 
                            data_segment_length);
        break;
        
        case JS_GAME_ROBOT_SURVIVORS_CMD_ID:
                    memcpy((js_game_robot_survivors_t *)&game_robot_survivors, 
                            buffer + UART_MSG_OFFSET_DATA, 
                            data_segment_length);
        break;
        
        case JS_EVENT_DATA_CMD_ID:
            memcpy((js_event_data_t *)&event_data, 
                    buffer + UART_MSG_OFFSET_DATA, 
                    data_segment_length);
        break;
        
        case JS_SUPPLY_PROJECTILE_ACTION_CMD_ID:
                    memcpy((js_supply_projectile_action_t *)&supply_projectile_action, 
                            buffer + UART_MSG_OFFSET_DATA, 
                            data_segment_length);
        break;
        
        case JS_SUPPLY_PROJECTILE_BOOKING_CMD_ID:
                    memcpy((js_supply_projectile_booking_t *)&supply_projectile_booking, 
                            buffer + UART_MSG_OFFSET_DATA, 
                            data_segment_length);
        break;
        
        case JS_GAME_ROBOT_STATE_CMD_ID:
                    memcpy((js_game_robot_state_t *)&game_robot_state, 
                            buffer + UART_MSG_OFFSET_DATA, 
                            data_segment_length);
        break;
        case JS_GAME_POWER_HEAT_DATA_CMD_ID:
                    memcpy((js_power_heat_data_t *)&power_heat_data, 
                            buffer + UART_MSG_OFFSET_DATA, 
                            data_segment_length);
        break;
        case JS_GAME_ROBOT_POSITION_CMD_ID:
                    memcpy((js_game_robot_pos_t *)&game_robot_pos, 
                            buffer + UART_MSG_OFFSET_DATA, 
                            data_segment_length);
        break;
        case JS_BUFF_MUSK_CMD_ID:
                    memcpy((js_buff_musk_t *)&buff_musk, 
                            buffer + UART_MSG_OFFSET_DATA, 
                            data_segment_length);
        break;
        case JS_AERIAL_ROBOT_ENERGY_CMD_ID:
                    memcpy((js_aerial_robot_energy_t *)&aerial_robot_energy, 
                            buffer + UART_MSG_OFFSET_DATA, 
                            data_segment_length);
        break;        
        case JS_ROBOT_HURT_CMD_ID:
                    memcpy((js_robot_hurt_t *)&robot_hurt, 
                            buffer + UART_MSG_OFFSET_DATA, 
                            data_segment_length);
        break;   
				        case JS_SHOOT_DATA_CMD_ID:
                    memcpy((js_shoot_data_t *)&shoot_data, 
                            buffer + UART_MSG_OFFSET_DATA, 
                            data_segment_length);
        break;   
								        case JS_RECEIVE_AND_SHOW_DATA_CMD_ID:
//                    memcpy((js_robot_interactive_data_t *)&robot_interactive_data, 
//                            buffer + UART_MSG_OFFSET_DATA, 
//                            data_segment_length);
        break;   
 
        default: break;
    }
}
