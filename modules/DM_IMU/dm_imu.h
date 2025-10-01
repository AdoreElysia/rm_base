/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-09-16 10:10:48
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-10-01 20:21:44
 * @FilePath: /rm_base/modules/DM_IMU/dm_imu.h
 * @Description: 
 */
#ifndef _DM_IMU_H_
#define _DM_IMU_H_

#include "bsp_can.h"
#include "imu_data.h"
#include <stdint.h>

#define DM_RID_ACCEL 1
#define DM_RID_GYRO  2
#define DM_RID_EULER 3
#define DM_RID_Quaternion 4

typedef struct{
    float q[4];
    IMU_Data_t imu_data;
    IMU_Estimate_t estimate;
}DM_IMU_DATA_t;

typedef struct
{	
    DM_IMU_DATA_t data;
	Can_Device *can_device;
	uint8_t offline_index;
    uint8_t initflag;
}DM_IMU_Moudule_t;

/**
 * @description: 初始化dm_imu模块
 * @param {DM_IMU_Moudule_t} *dm_imu
 * @return {osal_status_t},OSAL_SUCCESS表示初始化成功，其他值表示初始化失败
 */
osal_status_t dm_imu_init(DM_IMU_Moudule_t *dm_imu);
/**
 * @description: 请求dm_imu模块数据
 * @param {uint8_t} reg,请求数据类型，见DM_RID_XXX定义
 * @return {void}
 */
void dm_imu_request(uint8_t reg);
/**
 * @description: 更新dm_imu模块数据
 * @param {void}
 * @return {void}
 */
void dm_imu_update();
/**
 * @description: 获取dm_imu模块数据
 * @param {void}
 * @return {DM_IMU_DATA_t*}
 */
DM_IMU_DATA_t* get_dm_imu_data(void);

#endif // _DM_IMU_H_