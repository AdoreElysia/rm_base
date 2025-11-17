/*
 * @Author: your name
 * @Date: 2025-09-13 10:54:50
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-09-26 23:04:05
 * @FilePath: /rm_base/modules/IMU/IMULAYER/imulayer.c
 * @Description: IMU兼容层实现
 */

#include "imulayer.h"
#include "modules_config.h"
#include "osal_def.h"
#include <string.h>

#if IMU_TYPE == 1
#include "BMI088/bmi088.h"
#endif

osal_status_t imu_init(IMU_Instance_t *ist)
{
    if (ist == NULL)
    {
        return OSAL_ERROR;
    }

    osal_status_t status = OSAL_SUCCESS;

#if IMU_TYPE == 1
    status = BMI088_init(&ist->imu_handle);
    if (status == OSAL_SUCCESS)
    {
        ist->init_flag = 1; // 设置初始化标志
    }
#endif

    return status;
}

osal_status_t imu_get_accel(IMU_Instance_t *ist)
{
    // 检查参数和初始化状态
    if (ist == NULL || ist->init_flag == 0)
    {
        return OSAL_ERROR;
    }

    osal_status_t status = OSAL_SUCCESS;

#if IMU_TYPE == 1
    status = bmi088_get_accel(&ist->imu_handle, &ist->data);
#endif

    return status;
}

osal_status_t imu_get_gyro(IMU_Instance_t *ist)
{
    // 检查参数和初始化状态
    if (ist == NULL || ist->init_flag == 0)
    {
        return OSAL_ERROR;
    }

    osal_status_t status = OSAL_SUCCESS;

#if IMU_TYPE == 1
    status = bmi088_get_gyro(&ist->imu_handle, &ist->data);
#endif

    return status;
}

osal_status_t imu_get_temp(IMU_Instance_t *ist)
{
    // 检查参数和初始化状态
    if (ist == NULL || ist->init_flag == 0)
    {
        return OSAL_ERROR;
    }

    osal_status_t status = OSAL_SUCCESS;

#if IMU_TYPE == 1
    status = bmi088_get_temp(&ist->imu_handle, &ist->data);
#endif

    return status;
}

void imu_temp_ctrl(IMU_Instance_t *ist, float temp)
{
    // 检查参数和初始化状态
    if (ist == NULL || ist->init_flag == 0)
    {
        return;
    }
#if IMU_TYPE == 1
    bmi088_temp_ctrl(&ist->imu_handle, temp);
#endif
}