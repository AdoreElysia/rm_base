/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-09-16 10:10:42
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-10-01 20:20:59
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


#define log_tag  "dm_imu"
#include "shell_log.h"

#define ACCEL_CAN_MAX   (58.8f)
#define ACCEL_CAN_MIN	(-58.8f)
#define GYRO_CAN_MAX	(34.88f)
#define GYRO_CAN_MIN	(-34.88f)
#define PITCH_CAN_MAX	(90.0f)
#define PITCH_CAN_MIN	(-90.0f)
#define ROLL_CAN_MAX	(180.0f)
#define ROLL_CAN_MIN	(-180.0f)
#define YAW_CAN_MAX		(180.0f)
#define YAW_CAN_MIN 	(-180.0f)
#define TEMP_MIN	    (0.0f)
#define TEMP_MAX	    (60.0f)
#define Quaternion_MIN	(-1.0f)
#define Quaternion_MAX	(1.0f)


static DM_IMU_Moudule_t *dm_imu_module = NULL;

void dm_imu_request(uint8_t reg)
{
    if (dm_imu_module == NULL || dm_imu_module->can_device == NULL || dm_imu_module->initflag !=1){return;}

    dm_imu_module->can_device->txconf.DLC = 4;
    dm_imu_module->can_device->tx_buff[0] = (uint8_t)DM_IMU_RX_ID;
    dm_imu_module->can_device->tx_buff[1] = (uint8_t)(DM_IMU_RX_ID>>8);
    dm_imu_module->can_device->tx_buff[2] = reg;
    dm_imu_module->can_device->tx_buff[3] = 0XCC;
    
    BSP_CAN_SendDevice(dm_imu_module->can_device);
}
void dm_imu_update()
{
    static uint16_t tmp[4];
    static float dm_imu_lastyaw = 0.0f;
    if (dm_imu_module != NULL && dm_imu_module->initflag == 1)
    {
        offline_module_device_update(dm_imu_module->offline_index);
        switch(dm_imu_module->can_device->rx_buff[0])
        {
            case DM_RID_ACCEL:
            {
                tmp[0]=dm_imu_module->can_device->rx_buff[3]<<8|dm_imu_module->can_device->rx_buff[2];
                tmp[1]=dm_imu_module->can_device->rx_buff[5]<<8|dm_imu_module->can_device->rx_buff[4];
                tmp[2]=dm_imu_module->can_device->rx_buff[7]<<8|dm_imu_module->can_device->rx_buff[6];
                
                dm_imu_module->data.imu_data.acc[0]=uint_to_float(tmp[0],ACCEL_CAN_MIN,ACCEL_CAN_MAX,16);
                dm_imu_module->data.imu_data.acc[1]=uint_to_float(tmp[1],ACCEL_CAN_MIN,ACCEL_CAN_MAX,16);
                dm_imu_module->data.imu_data.acc[2]=uint_to_float(tmp[2],ACCEL_CAN_MIN,ACCEL_CAN_MAX,16);
                break;
            }
            case DM_RID_GYRO:
            {
                tmp[0]=dm_imu_module->can_device->rx_buff[3]<<8|dm_imu_module->can_device->rx_buff[2];
                tmp[1]=dm_imu_module->can_device->rx_buff[5]<<8|dm_imu_module->can_device->rx_buff[4];
                tmp[2]=dm_imu_module->can_device->rx_buff[7]<<8|dm_imu_module->can_device->rx_buff[6];
                
                dm_imu_module->data.imu_data.gyro[0]=uint_to_float(tmp[0],GYRO_CAN_MIN,GYRO_CAN_MAX,16);
                dm_imu_module->data.imu_data.gyro[1]=uint_to_float(tmp[1],GYRO_CAN_MIN,GYRO_CAN_MAX,16);
                dm_imu_module->data.imu_data.gyro[2]=uint_to_float(tmp[2],GYRO_CAN_MIN,GYRO_CAN_MAX,16);
                break;
            }
            case DM_RID_EULER:
            {
                int euler[3];
                
                euler[0]=dm_imu_module->can_device->rx_buff[3]<<8|dm_imu_module->can_device->rx_buff[2];
                euler[1]=dm_imu_module->can_device->rx_buff[5]<<8|dm_imu_module->can_device->rx_buff[4];
                euler[2]=dm_imu_module->can_device->rx_buff[7]<<8|dm_imu_module->can_device->rx_buff[6];
                
                dm_imu_module->data.estimate.Pitch=uint_to_float(euler[0],PITCH_CAN_MIN,PITCH_CAN_MAX,16);
                dm_imu_module->data.estimate.Yaw=uint_to_float(euler[1],YAW_CAN_MIN,YAW_CAN_MAX,16);
                dm_imu_module->data.estimate.Roll=uint_to_float(euler[2],ROLL_CAN_MIN,ROLL_CAN_MAX,16);

                if (dm_imu_module->data.estimate.Yaw - dm_imu_lastyaw > 180.0f){dm_imu_module->data.estimate.YawRoundCount--;}
                else if (dm_imu_module->data.estimate.Yaw - dm_imu_lastyaw < -180.0f){dm_imu_module->data.estimate.YawRoundCount++;}
                dm_imu_module->data.estimate.YawTotalAngle = 360.0f * dm_imu_module->data.estimate.YawRoundCount+ dm_imu_module->data.estimate.Yaw;
                dm_imu_lastyaw = dm_imu_module->data.estimate.Yaw;
                break;
            }
            case DM_RID_Quaternion:
            {
                int w = dm_imu_module->can_device->rx_buff[1]<<6| ((dm_imu_module->can_device->rx_buff[2]&0xF8)>>2);
                int x = (dm_imu_module->can_device->rx_buff[2]&0x03)<<12|(dm_imu_module->can_device->rx_buff[3]<<4)|((dm_imu_module->can_device->rx_buff[4]&0xF0)>>4);
                int y = (dm_imu_module->can_device->rx_buff[4]&0x0F)<<10|(dm_imu_module->can_device->rx_buff[5]<<2)|(dm_imu_module->can_device->rx_buff[6]&0xC0)>>6;
                int z = (dm_imu_module->can_device->rx_buff[6]&0x3F)<<8|dm_imu_module->can_device->rx_buff[7];
                
                dm_imu_module->data.q[0] = uint_to_float(w,Quaternion_MIN,Quaternion_MAX,14);
                dm_imu_module->data.q[1] = uint_to_float(x,Quaternion_MIN,Quaternion_MAX,14);
                dm_imu_module->data.q[2] = uint_to_float(y,Quaternion_MIN,Quaternion_MAX,14);
                dm_imu_module->data.q[3] = uint_to_float(z,Quaternion_MIN,Quaternion_MAX,14);
                break;
            }
        }
    }
}

DM_IMU_DATA_t* get_dm_imu_data(void)
{
    if (dm_imu_module != NULL && dm_imu_module->initflag == 1)
    {
        return &dm_imu_module->data;
    }
    else
    {
        return NULL;
    }
}
osal_status_t dm_imu_init(DM_IMU_Moudule_t *dm_imu){
    if (dm_imu == NULL)
    {
        LOG_ERROR("dm_imu is NULL");
        return OSAL_ERROR;
    }
    dm_imu_module = dm_imu;
    
    OfflineDeviceInit_t offline_init = {
        .name = "dm_imu",
        .timeout_ms = 10,
        .level = OFFLINE_LEVEL_HIGH,
        .beep_times = 1,
        .enable = OFFLINE_ENABLE,
    };
    dm_imu_module->offline_index = offline_module_device_register(&offline_init);
    if (dm_imu_module->offline_index == OFFLINE_INVALID_INDEX)
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
    dm_imu_module->can_device= BSP_CAN_Device_Init(&can_config);
    if (dm_imu_module->can_device == NULL) {
        LOG_ERROR("Failed to initialize CAN device");
        return OSAL_ERROR;
    }

    dm_imu_module->initflag = 1;
    LOG_INFO("dm_imu init success");
    return OSAL_SUCCESS;
}