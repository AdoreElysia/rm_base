/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-09-16 10:10:42
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-09-30 13:39:53
 * @FilePath: /rm_base/modules/DM_IMU/dm_imu.c
 * @Description: 
 */
#include "dm_imu.h"
#include "bsp_can.h"
#include "modules_config.h"
#include "offline.h"
#include "osal_def.h"
#include "user_lib.h"
#include <string.h>


#ifdef DM_IMU_MODULE

#define log_tag  "dm_imu"
#include "shell_log.h"

#define ACCEL_CAN_MAX (58.8f)
#define ACCEL_CAN_MIN	(-58.8f)
#define GYRO_CAN_MAX	(34.88f)
#define GYRO_CAN_MIN	(-34.88f)
#define PITCH_CAN_MAX	(90.0f)
#define PITCH_CAN_MIN	(-90.0f)
#define ROLL_CAN_MAX	(180.0f)
#define ROLL_CAN_MIN	(-180.0f)
#define YAW_CAN_MAX		(180.0f)
#define YAW_CAN_MIN 	(-180.0f)
#define TEMP_MIN			(0.0f)
#define TEMP_MAX			(60.0f)
#define Quaternion_MIN	(-1.0f)
#define Quaternion_MAX	(1.0f)

#define DM_RID_ACCEL 1
#define DM_RID_GYRO  2
#define DM_RID_EULER 3
#define DM_RID_Quaternion 4


static DM_IMU_Instance_t dm_imu_instance;

void IMU_RequestData(uint16_t can_id,uint8_t reg)
{
    if (dm_imu_instance.can_device == NULL){return;}
    dm_imu_instance.can_device->txconf.DLC = 4;

    dm_imu_instance.can_device->tx_buff[0] = (uint8_t)can_id;
    dm_imu_instance.can_device->tx_buff[1] = (uint8_t)(can_id>>8);
    dm_imu_instance.can_device->tx_buff[2] = reg;
    dm_imu_instance.can_device->tx_buff[3] = 0XCC;
    
    BSP_CAN_SendDevice(dm_imu_instance.can_device);
}


void IMU_UpdateAccel(DM_IMU_Instance_t *ist)
{
	uint16_t accel[3];
	
	accel[0]=ist->can_device->rx_buff[3]<<8|ist->can_device->rx_buff[2];
	accel[1]=ist->can_device->rx_buff[5]<<8|ist->can_device->rx_buff[4];
	accel[2]=ist->can_device->rx_buff[7]<<8|ist->can_device->rx_buff[6];
	
	ist->data.acc[0]=uint_to_float(accel[0],ACCEL_CAN_MIN,ACCEL_CAN_MAX,16);
	ist->data.acc[1]=uint_to_float(accel[1],ACCEL_CAN_MIN,ACCEL_CAN_MAX,16);
	ist->data.acc[2]=uint_to_float(accel[2],ACCEL_CAN_MIN,ACCEL_CAN_MAX,16);
}

void IMU_UpdateGyro(DM_IMU_Instance_t *ist)
{
	uint16_t gyro[3];
	
	gyro[0]=ist->can_device->rx_buff[3]<<8|ist->can_device->rx_buff[2];
	gyro[1]=ist->can_device->rx_buff[5]<<8|ist->can_device->rx_buff[4];
	gyro[2]=ist->can_device->rx_buff[7]<<8|ist->can_device->rx_buff[6];
	
	ist->data.gyro[0]=uint_to_float(gyro[0],GYRO_CAN_MIN,GYRO_CAN_MAX,16);
	ist->data.gyro[1]=uint_to_float(gyro[1],GYRO_CAN_MIN,GYRO_CAN_MAX,16);
	ist->data.gyro[2]=uint_to_float(gyro[2],GYRO_CAN_MIN,GYRO_CAN_MAX,16);
}


void IMU_UpdateEuler(DM_IMU_Instance_t *ist)
{
	int euler[3];
	
	euler[0]=ist->can_device->rx_buff[3]<<8|ist->can_device->rx_buff[2];
	euler[1]=ist->can_device->rx_buff[5]<<8|ist->can_device->rx_buff[4];
	euler[2]=ist->can_device->rx_buff[7]<<8|ist->can_device->rx_buff[6];
	
	ist->estimate.Pitch=uint_to_float(euler[0],PITCH_CAN_MIN,PITCH_CAN_MAX,16);
	ist->estimate.Yaw=uint_to_float(euler[1],YAW_CAN_MIN,YAW_CAN_MAX,16);
	ist->estimate.Roll=uint_to_float(euler[2],ROLL_CAN_MIN,ROLL_CAN_MAX,16);

    if (ist->estimate.Yaw - ist->dm_imu_lastyaw > 180.0f){ist->estimate.YawRoundCount--;}
    else if (ist->estimate.Yaw - ist->dm_imu_lastyaw < -180.0f){ist->estimate.YawRoundCount++;}
    ist->estimate.YawTotalAngle = 360.0f * ist->estimate.YawRoundCount+ ist->estimate.Yaw;
    ist->dm_imu_lastyaw = ist->estimate.Yaw;
}


void IMU_UpdateQuaternion(DM_IMU_Instance_t *ist)
{
	int w = ist->can_device->rx_buff[1]<<6| ((ist->can_device->rx_buff[2]&0xF8)>>2);
	int x = (ist->can_device->rx_buff[2]&0x03)<<12|(ist->can_device->rx_buff[3]<<4)|((ist->can_device->rx_buff[4]&0xF0)>>4);
	int y = (ist->can_device->rx_buff[4]&0x0F)<<10|(ist->can_device->rx_buff[5]<<2)|(ist->can_device->rx_buff[6]&0xC0)>>6;
	int z = (ist->can_device->rx_buff[6]&0x3F)<<8|ist->can_device->rx_buff[7];
	
	ist->q[0] = uint_to_float(w,Quaternion_MIN,Quaternion_MAX,14);
	ist->q[1] = uint_to_float(x,Quaternion_MIN,Quaternion_MAX,14);
	ist->q[2] = uint_to_float(y,Quaternion_MIN,Quaternion_MAX,14);
	ist->q[3] = uint_to_float(z,Quaternion_MIN,Quaternion_MAX,14);
}

void IMU_UpdateData(DM_IMU_Instance_t *ist)
{
    offline_module_device_update(ist->offline_index);
    switch(ist->can_device->rx_buff[0])
    {
        case DM_RID_ACCEL:
            IMU_UpdateAccel(ist);
            break;
        case DM_RID_GYRO:
            IMU_UpdateGyro(ist);
            break;
        case DM_RID_EULER:
            IMU_UpdateEuler(ist);
            break;
        case DM_RID_Quaternion:
            IMU_UpdateQuaternion(ist);
            break;
    }
}

void dm_imu_task_function(void){ 

    osal_status_t status = OSAL_SUCCESS;
    IMU_RequestData(DM_IMU_RX_ID,DM_RID_GYRO);
    osal_delay_ms(1);
    status = BSP_CAN_ReadSingleDevice(dm_imu_instance.can_device,OSAL_WAIT_FOREVER);
    if (status == OSAL_SUCCESS)
    {
        IMU_UpdateData(&dm_imu_instance);
    }
    IMU_RequestData(DM_IMU_RX_ID,DM_RID_EULER);
    osal_delay_ms(1);
    status = BSP_CAN_ReadSingleDevice(dm_imu_instance.can_device,OSAL_WAIT_FOREVER);
    if (status == OSAL_SUCCESS)
    {
        IMU_UpdateData(&dm_imu_instance);
    }
}


osal_status_t dm_imu_init(void){
    OfflineDeviceInit_t offline_init = {
        .name = "dm_imu",
        .timeout_ms = 10,
        .level = OFFLINE_LEVEL_HIGH,
        .beep_times = 1,
        .enable = OFFLINE_ENABLE,
    };
    dm_imu_instance.offline_index = offline_module_device_register(&offline_init);
    if (dm_imu_instance.offline_index == OFFLINE_INVALID_INDEX)
    {
        LOG_ERROR("offline_device_register error");
        return OSAL_ERROR;
    }
    // CAN 设备初始化配置
    Can_Device_Init_Config_s can_config = {
        .can_handle = &DM_IMU_CAN_BUS,
        .tx_id = DM_IMU_TX_ID,
        .rx_id = DM_IMU_RX_ID,
        .tx_mode = CAN_MODE_BLOCKING,
        .rx_mode = CAN_MODE_IT,
    };
    // 注册 CAN 设备并获取引用
    Can_Device *can_dev = BSP_CAN_Device_Init(&can_config);
    if (can_dev == NULL) {
        LOG_ERROR("Failed to initialize CAN device");
        return OSAL_ERROR;
    }
    // 保存设备指针
    dm_imu_instance.can_device = can_dev;
    dm_imu_instance.initflag = 1;

    LOG_INFO("dm_imu init success");
    return OSAL_SUCCESS;
}


DM_IMU_Instance_t* get_dm_imu_instance(void)
{
    return &dm_imu_instance;
}

#else  
osal_status_t dm_imu_init(void){
    return OSAL_SUCCESS;
}
void dm_imu_task_function(void){}

DM_IMU_Instance_t* get_dm_imu_instance(void){
    return NULL;
}
#endif