#ifndef _IMU_TASK_H_
#define _IMU_TASK_H_

#include "main.h"
#define IMU_MSG_LENGTH              11u
#define IMU_MSG_LENGTH_2            (IMU_MSG_LENGTH * 2.f)
#define IMU_MSG_LENGTH_3            (IMU_MSG_LENGTH * 3.f)
#define IMU_MSG_SOF                 0x55
#define IMU_CMD_ID_SEGMENT_OFFSET   0x01

#define IMU_ACCEL_CMD_ID            0x51
#define IMU_GYRO_CMD_ID             0x52
#define IMU_MAG_CMD_ID              0x53

typedef struct {
    __packed struct {
    float x;
    float y;
    float z;
    } acc;
    
    __packed struct {
    float x;
    float y;
    float z;
    } gyro;
    
    __packed struct {
    float x;
    float y;
    float z;
		float z_absolute;
		float z_local;
		float z_local_record;
    } mag;
} imu_t;

extern volatile imu_t imu;

void imu_task_init(void);
void imu_download_msg_process(uint8_t * buffer);
void imu_msg_unpack_handler(uint8_t * buffer);
void imu_update_local(void);
#endif
