#include "tx2_task.h"

volatile pc_robot_status_t pc_robot_status;
volatile pc_pose_upload_t pc_gimbal_upload;
volatile pc_custom_upload_t pc_custom_upload;

volatile pc_pose_command_t pc_pose_command;

volatile uint8_t pc_rc_download[RC_FRAME_LENGTH];


// 上传、下载buffer
uint8_t pc_tx_buffer[PC_MSG_MAX_LENGTH];
uint8_t pc_rx_buffer[PC_MSG_MAX_LENGTH];

// 帧长
uint16_t pc_tx_msg_length = PC_MSG_MAX_LENGTH;
uint16_t pc_rx_msg_length = PC_MSG_MAX_LENGTH;

// 下载buffer当前正在压入数据的index，压入完成后index自增1
uint16_t pc_rx_buffer_index = 0;

// 数据段长度
uint16_t pc_data_segment_length = 0;

// 上传信息数据包序号
static uint8_t pc_tx_seq = 0;

// 解包状态机的当前状态
unpack_state_e unpack_state;

// 上传数据状态机的当前状态
transmit_channel_e transmit_channel;


void pc_task_init()
{
    // bsp
    USART3_Configuration();
    
    // 上传数据包
    memset((pc_robot_status_t *)     &pc_robot_status,     0, sizeof(pc_robot_status_t));
    memset((pc_pose_upload_t *)      &pc_gimbal_upload,    0, sizeof(pc_pose_upload_t));
    memset((pc_custom_upload_t *)    &pc_custom_upload,    0, sizeof(pc_custom_upload_t));
    
    // 下载数据包
    memset((uint8_t *)               &pc_rc_download,      0, sizeof(uint8_t));
    
    // 状态机初始化
    unpack_state     =  UNPACK_SOF;
    transmit_channel =  TRANSMIT_CHANNEL_RC;
}

/**
  * @brief     TX2 下载信息处理函数
  * @param[in] msg: 接收到的字节流信息
  * @retval    null
  */
void pc_download_msg_process(uint8_t msg)
{
    switch(unpack_state)
    {

        case UNPACK_SOF:
            if (msg == PC_MSG_SOF)
            {
                memset((uint8_t *)&pc_rx_buffer, 0, sizeof(pc_rx_buffer));
            
                pc_rx_buffer_index = 0;
                pc_rx_buffer[pc_rx_buffer_index++] = msg;
                unpack_state = UNPACK_LENGTH_LOW;
            }
            else
            {
                pc_rx_buffer_index = 0;
            }
        break;
        
        case UNPACK_LENGTH_LOW:
            pc_rx_buffer[pc_rx_buffer_index++] = msg;
            pc_data_segment_length = msg & 0xff;
        
            unpack_state = UNPACK_LENGTH_HIGH;
        break;
        
        case UNPACK_LENGTH_HIGH:
            pc_rx_buffer[pc_rx_buffer_index++] = msg;

            pc_data_segment_length |= msg << 8;
            pc_rx_msg_length = pc_data_segment_length + UART_MSG_OFFSET_DATA + 2;
        
            if (pc_rx_msg_length < PC_MSG_MAX_LENGTH)
            {
                unpack_state = UNPACK_SEQ;
            }
            else
            {
                unpack_state = UNPACK_SOF;
            }
        break;
        
        case UNPACK_SEQ:
            pc_rx_buffer[pc_rx_buffer_index++] = msg;
            unpack_state = UNPACK_CRC8;
        break;
        
        case UNPACK_CRC8:
            pc_rx_buffer[pc_rx_buffer_index++] = msg;
        
            if (pc_rx_buffer_index == UART_MSG_OFFSET_CMD_ID 
             && verify_crc8_check_sum(pc_rx_buffer, UART_MSG_OFFSET_CMD_ID))
            {
                unpack_state = UNPACK_CRC16;
            }
            else
            {
                unpack_state = UNPACK_SOF;
            }
            
        break;
        
        case UNPACK_CRC16:
            if (pc_rx_buffer_index < pc_rx_msg_length - 1)
            {
                pc_rx_buffer[pc_rx_buffer_index++] = msg;
            }
            else
            {
							  pc_rx_buffer[pc_rx_buffer_index] = msg;

                if (verify_crc16_check_sum(pc_rx_buffer, pc_rx_msg_length))
                {
                    pc_msg_upack_handler(pc_rx_buffer);
                    //supervisor_register_handler(SUPERVISOR_PC);
                }
                
                unpack_state = UNPACK_SOF;
            }
        break;
        
        default:
            unpack_state = UNPACK_SOF;
        break;
    }
}

/**
  * @brief     TX2通信解包函数
  * @param[in] buffer: 完整的一帧信息
  * @retval    null
  */
void pc_msg_upack_handler(uint8_t * buffer)
{
    uint16_t cmd_id =  buffer[UART_MSG_OFFSET_CMD_ID] 
                    | (buffer[UART_MSG_OFFSET_CMD_ID + 1] << 8);
    
    switch (cmd_id)
    {
        case PC_RC_DOWNLOAD_CMD_ID:
            memcpy((uint8_t *)&pc_rc_download,
                   buffer + UART_MSG_OFFSET_DATA,
                   pc_data_segment_length);
            virtual_rc_msg_process((uint8_t *)&pc_rc_download);
        break;
        
        default: break;
    }
}

/**
  * @brief     信息上传的周期处理函数(100Hz for each case)
  * @param[in] null
  * @retval    null
  */
void pc_msg_upload_handler(uint8_t channel)
{
    switch (channel)
    {
        case TRANSMIT_CHANNEL_RC:
            
            break;
        
        default: break;
    }
}

/**
  * @brief     上传自定义数据处理函数
  * @param[in] d1: 
  * @param[in] d2: 
  * @param[in] d3: 
  * @param[in] d4: 
  * @retval    null
  */
void pc_upload_custom_msg_process(float d1, float d2, float d3, float d4)
{
    // 变量声明
    uint16_t upload_msg_length = 0;
    
    // 清空相关变量
    memset(pc_tx_buffer, 0, sizeof(pc_tx_buffer));
    memset((pc_custom_upload_t *)&pc_custom_upload, 0, sizeof(pc_custom_upload_t));
    
    // 赋值
    pc_custom_upload.data1 = d1;
    pc_custom_upload.data2 = d2;
    pc_custom_upload.data3 = d3;
    pc_custom_upload.data4 = d4;
    
    // 数据包整理
    upload_msg_length = uart_msg_pack_handler(pc_tx_buffer, 
                                              PC_MSG_SOF,
                                              sizeof(pc_pose_upload_t), 
                                              pc_tx_seq++,
                                              PC_CUSTOM_UPLOAD_CMD_ID, 
                                              (pc_custom_upload_t *)&pc_custom_upload);
                                              
    // 通过USART3发送数据包
    USART3_PrintBlock(pc_tx_buffer, upload_msg_length);
}

/**
  * @brief     上传机体状态数据处理函数
  * @param[in] null
  * @retval    null
  */
void pc_upload_status_msg_process(void)
{
    // 变量声明
    uint16_t upload_msg_length = 0;
    
    // 清空相关变量
    memset(pc_tx_buffer, 0, sizeof(pc_tx_buffer));
    
    // 赋值
    pc_robot_status.error_code = offline_msg.device.code;
    
    // 数据包整理
    upload_msg_length = uart_msg_pack_handler(pc_tx_buffer, 
                                              PC_MSG_SOF,
                                              sizeof(pc_robot_status_t), 
                                              pc_tx_seq++,
                                              PC_ROBOT_STATE_CMD_ID, 
                                              (pc_robot_status_t *)&pc_robot_status);
                                              
    // 通过USART3发送数据包
    USART3_PrintBlock(pc_tx_buffer, upload_msg_length);
}

/**
  * @brief     上传遥控数据处理函数
  * @param[in] null
  * @retval    null
  */
//void pc_upload_rc_ctrl_msg_process(void)
//{
//    // 变量声明
//    uint16_t upload_msg_length = 0;
//    
//    // 清空相关变量
//    memset(pc_tx_buffer, 0, sizeof(pc_tx_buffer));
//    
//    // 数据包整理
//    upload_msg_length = uart_msg_pack_handler(pc_tx_buffer, 
//                                              PC_MSG_SOF,
//                                              sizeof(rc_ctrl_t), 
//                                              pc_tx_seq++,
//                                              PC_RC_UPLOAD_CMD_ID, 
//                                              (rc_ctrl_t *)&rc_ctrl);
//                                              
//    // 通过USART3发送数据包
//    USART3_PrintBlock(pc_tx_buffer, upload_msg_length);
//}

/**
  * @brief     上传imu数据处理函数
  * @param[in] null
  * @retval    null
  */
void pc_upload_imu_msg_process(void)
{
    // 变量声明
    uint16_t upload_msg_length = 0;
    
    // 清空相关变量
    memset(pc_tx_buffer, 0, sizeof(pc_tx_buffer));
    
    // 数据包整理
    upload_msg_length = uart_msg_pack_handler(pc_tx_buffer, 
                                              PC_MSG_SOF,
                                              sizeof(imu_t), 
                                              pc_tx_seq++,
                                              PC_IMU_UPLOAD_CMD_ID,
                                              (imu_t *)&imu);
                                              
    // 通过USART3发送数据包
    USART3_PrintBlock(pc_tx_buffer, upload_msg_length);
}
/**
  * @brief     上传云台数据处理函数
  * @param[in] null
  * @retval    null
  */
void pc_upload_pose_msg_process(void)
{
    // 变量声明
    uint16_t upload_msg_length = 0;
    
    // 清空相关变量
    memset(pc_tx_buffer, 0, sizeof(pc_tx_buffer));
    memset((pc_pose_upload_t *)&pc_gimbal_upload, 0, sizeof(pc_pose_upload_t));
    
    // 赋值
    pc_gimbal_upload.chassis.speed.x_get    = chassis.speed.x_get;
    pc_gimbal_upload.chassis.speed.y_get    = chassis.speed.y_get;
    pc_gimbal_upload.chassis.speed.z_get    = chassis.speed.z_get;
    pc_gimbal_upload.gimbal.pitch.angle_get = gimbal.joint[PITCH].angle_get;
    pc_gimbal_upload.gimbal.pitch.speed_get = gimbal.joint[PITCH].speed_get;
    pc_gimbal_upload.gimbal.yaw.angle_get   = gimbal.joint[YAW].angle_get;
    pc_gimbal_upload.gimbal.yaw.speed_get   = gimbal.joint[YAW].speed_get;
    pc_gimbal_upload.control_mode           = gimbal.control_mode;
    
    // 数据包整理
    upload_msg_length = uart_msg_pack_handler(pc_tx_buffer, 
                                              PC_MSG_SOF,
                                              sizeof(pc_pose_upload_t), 
                                              pc_tx_seq++,
                                              PC_POSE_UPLOAD_CMD_ID, 
                                              (pc_pose_upload_t *)&pc_gimbal_upload);
    
    // 通过USART3发送数据包
    USART3_PrintBlock(pc_tx_buffer, upload_msg_length);
}
