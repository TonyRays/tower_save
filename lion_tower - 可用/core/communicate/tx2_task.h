#ifndef _TX2_TASK_H_
#define _TX2_TASK_H_

#include "main.h"

#define PC_MSG_MAX_LENGTH               50u
#define PC_MSG_SOF                      0xFE

#define PC_ROBOT_STATE_CMD_ID           0x0001
#define PC_POSE_UPLOAD_CMD_ID           0x0002
#define PC_POSE_COMMAND_CMD_ID          0x0003
#define PC_GIMBAL_DOWNLOAD_CMD_ID       0x0005
#define PC_RC_UPLOAD_CMD_ID             0x0006
#define PC_RC_DOWNLOAD_CMD_ID           0x0007
#define PC_IMU_UPLOAD_CMD_ID            0x0008
#define PC_IMU_DOWNLOAD_CMD_ID          0x0009
#define PC_CUSTOM_UPLOAD_CMD_ID         0x0020
#define PC_CUSTOM_DOWNLOAD_CMD_ID       0x0021

typedef enum {
    UNPACK_SOF,
    UNPACK_LENGTH_LOW,
    UNPACK_LENGTH_HIGH,
    UNPACK_SEQ,
    UNPACK_CRC8,
    UNPACK_CRC16,
} unpack_state_e;

typedef enum
{
    TRANSMIT_CHANNEL_STATUS = 0,
    TRANSMIT_CHANNEL_RC,
    TRANSMIT_CHANNEL_IMU,
    TRANSMIT_CHANNEL_POSE,
    TRANSMIT_CHANNEL_TOTAL
} transmit_channel_e;

typedef __packed struct
{
    // error_list
    uint16_t error_code;
} pc_robot_status_t;

typedef __packed struct
{
    __packed struct {
        float x_set;
        float y_set;
        float z_set;
    } speed;
} pc_chassis_download_t;

typedef __packed struct
{
    __packed struct {
        __packed struct {
            float x;
            float y;
            float z;
        } speed;
    } chassis;
    
    __packed struct {
        __packed struct {
            float angle;
            float speed;
        } yaw;
        
        __packed struct {
            float angle;
            float speed;
        } pitch;
    } gimbal;
    
    uint8_t control_mode;
} pc_pose_command_t;

typedef __packed struct
{
    __packed struct {
        __packed struct {
            float x_get;
            float y_get;
            float z_get;
        } speed;
    } chassis;
    
    __packed struct {
        __packed struct {
            float angle_get;
            float speed_get;
        } yaw;
        
        __packed struct {
            float angle_get;
            float speed_get;
        } pitch;
    } gimbal;
    
    uint8_t control_mode; // speed-based (0) or position(angle)-based (1)
} pc_pose_upload_t;

typedef __packed struct
{
    __packed struct {
        float yaw;
        float pitch;
    } angle;
    
    __packed struct {
        float yaw;
        float pitch;
    } speed;
    
    uint8_t control_mode;
} pc_gimbal_download_t;

typedef __packed struct
{
    float data1;
    float data2;
    float data3;
    float data4;
} pc_custom_upload_t;

typedef __packed struct
{
    int16_t ch1;
    int16_t ch2;
    int16_t ch3;
    int16_t ch4;
} pc_rc_upload_t;

typedef __packed struct
{
    float data1;
    float data2;
    float data3;
    float data4;
} pc_custom_download_t;


extern volatile pc_chassis_download_t pc_chassis_download;
extern volatile pc_gimbal_download_t pc_gimbal_download;
extern volatile pc_custom_download_t pc_custom_download;

extern volatile pc_pose_command_t pc_pose_command;

void pc_task_init(void);
void pc_download_msg_process(uint8_t);
void pc_upload_custom_msg_process(float data1, float data2, float data3, float d4);
void pc_upload_status_msg_process(void);
void pc_upload_rc_ctrl_msg_process(void);
void pc_upload_imu_msg_process(void);
void pc_upload_pose_msg_process(void);
void pc_msg_upack_handler(uint8_t * buffer);
void pc_msg_upload_handler(uint8_t channel);

#endif
