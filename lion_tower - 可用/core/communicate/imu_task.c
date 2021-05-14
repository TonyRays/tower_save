#include "imu_task.h"

volatile imu_t imu;

static uint16_t _1L = IMU_MSG_LENGTH;
static uint16_t _2L = IMU_MSG_LENGTH_2;

void imu_task_init()
{
    // bsp
    USART6_Configuration();
    
    memset((imu_t *)&imu, 0, sizeof(imu_t));
}

/**
  * @brief     MPU6050下载信息处理函数
  * @param[in] msg: 接收的字节流
  * @retval    null
  */
void imu_download_msg_process(uint8_t * buffer)
{
    // 确认帧头
    if(buffer[0] == IMU_MSG_SOF)
    {
        imu_msg_unpack_handler(buffer);
        
        supervisor_register_handler(SUPERVISOR_IMU);
    }
}

/**
  * @brief     MPU6050 解包函数
  * @param[in] buffer: 完整的三帧信息
  * @retval    null
  */
void imu_msg_unpack_handler(uint8_t * buffer)
{
    if (buffer[IMU_CMD_ID_SEGMENT_OFFSET] == IMU_ACCEL_CMD_ID)
    {
        // IMU_ACCEL: // 0.005f ≈ 1.f / 32768.f * 16.f * 9.8f
        imu.acc.x =  ((short)(buffer[3]<<8 | buffer[2])) * 0.005f;    // g
        imu.acc.y =  ((short)(buffer[5]<<8 | buffer[4])) * 0.005f;    // g
        imu.acc.z =  ((short)(buffer[7]<<8 | buffer[6])) * 0.005f;    // g
    }
    
    if (buffer[IMU_CMD_ID_SEGMENT_OFFSET + _1L] == IMU_GYRO_CMD_ID)
    {
        // IMU_GYRO:  // 0.061f ≈ 1.f / 32768.f * 2000.f
        imu.gyro.x = ((short)(buffer[_1L + 3]<<8 | buffer[_1L + 2])) * 0.061f;   // deg/sec
        imu.gyro.y = ((short)(buffer[_1L + 5]<<8 | buffer[_1L + 4])) * 0.061f;   // deg/sec
        imu.gyro.z = ((short)(buffer[_1L + 7]<<8 | buffer[_1L + 6])) * 0.061f;   // deg/sec
    }
    
    if (buffer[IMU_CMD_ID_SEGMENT_OFFSET + _2L] == IMU_MAG_CMD_ID)
    {
        // IMU_MAG:   // 0.005f ≈ 1.f / 32768.f * 180.f
        imu.mag.x =  ((short)(buffer[_2L + 3]<<8 | buffer[_2L + 2])) * 0.005f;    // deg
        imu.mag.y =  ((short)(buffer[_2L + 5]<<8 | buffer[_2L + 4])) * 0.005f;    // deg
        imu.mag.z =  ((short)(buffer[_2L + 7]<<8 | buffer[_2L + 6])) * 0.005f;    // deg
    }
}

void imu_update_local(void){
	  imu.mag.z_absolute += imu.gyro.z * 0.01f;
		if(imu.mag.z_absolute > 0){
	      imu.mag.z_local = ((float)((int32_t)imu.mag.z_absolute % 360) >  180) ? (((int32_t)imu.mag.z_absolute % 360) - 360) : ((int32_t)imu.mag.z_absolute % 360);
		}
		else{
			  imu.mag.z_local = ((float)((int32_t)imu.mag.z_absolute % 360) < -180) ? (((int32_t)imu.mag.z_absolute % 360) + 360) : ((int32_t)imu.mag.z_absolute % 360);
		}
}
