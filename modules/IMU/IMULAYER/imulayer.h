/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-09-13 10:49:45
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-09-13 13:08:29
 * @FilePath: /rm_base/modules/IMU/IMULAYER/imulayer.h
 * @Description: IMU兼容层接口
 */
#ifndef _IMULAYER_H_
#define _IMULAYER_H_

#include "bmi088.h"
#include "imu_data.h"
#include "osal_def.h"
#include <stdint.h>

typedef struct
{
    uint8_t    init_flag;
    IMU_Data_t data;
#if IMU_TYPE == IMU_BMI088
    BMI088_Instance_t imu_handle;
#endif
} IMU_Instance_t;

/**
 * @description: 初始化IMU
 * @param {IMU_Instance_t*},IMU实例指针
 * @return {osal_status_t},获取成功返回OSAL_SUCCESS，否则返回OSAL_ERROR
 */
osal_status_t imu_init(IMU_Instance_t *ist);

/**
 * @description: 获取IMU加速度数据
 * @param {IMU_Instance_t*},IMU实例指针
 * @return {osal_status_t},获取成功返回OSAL_SUCCESS，否则返回OSAL_ERROR
 */
osal_status_t imu_get_accel(IMU_Instance_t *ist);

/**
 * @description: 获取IMU陀螺仪数据
 * @param {IMU_Instance_t*},IMU实例指针
 * @return {osal_status_t},获取成功返回OSAL_SUCCESS，否则返回OSAL_ERROR
 */
osal_status_t imu_get_gyro(IMU_Instance_t *ist);

/**
 * @description: 获取IMU温度数据
 * @param {IMU_Instance_t*},IMU实例指针
 * @return {osal_status_t},获取成功返回OSAL_SUCCESS，否则返回OSAL_ERROR
 */
osal_status_t imu_get_temp(IMU_Instance_t *ist);

/**
 * @description: IMU温度控制
 * @param {IMU_Instance_t*},IMU实例指针
 * @param {float} temp,当前的温度
 * @return {*}
 */
void imu_temp_ctrl(IMU_Instance_t *ist, float temp);

#endif // _IMULAYER_H_