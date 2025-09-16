/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-09-16 10:10:48
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-09-16 11:39:25
 * @FilePath: /rm_base/modules/DM_IMU/dm_imu.h
 * @Description: 
 */
#ifndef _DM_IMU_H_
#define _DM_IMU_H_

#include "bsp_can.h"
#include "imu_data.h"
#include <stdint.h>

typedef struct
{	
	float q[4];
    IMU_Data_t data;
    IMU_Estimate_t estimate;
    float dm_imu_lastyaw;
	Can_Device *can_device;
	uint8_t offline_index;
    uint8_t initflag;
}DM_IMU_Instance_t;


/**
 * @brief DM_IMU设备任务函数
 */
void dm_imu_task_function(void);
/**
 * @brief 初始化DM_IMU设备
 * @return osal_status_t OSAL_SUCCESS表示初始化成功，其他值表示失败
 */
osal_status_t dm_imu_init(void);
/**
 * @brief 获取DM_IMU实例指针
 * @details 返回指向DM_IMU实例的指针，用于访问IMU数据
 * @return DM_IMU_Instance_t* DM_IMU实例指针
 */
DM_IMU_Instance_t* get_dm_imu_instance(void);


#endif // _DM_IMU_H_