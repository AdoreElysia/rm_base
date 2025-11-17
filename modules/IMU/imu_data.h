/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-09-13 12:49:45
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-09-13 14:05:20
 * @FilePath: /rm_base/modules/IMU/imu_data.h
 * @Description:
 */
#ifndef _IMU_DATA_H_
#define _IMU_DATA_H_

#include <stdint.h>

typedef struct
{
    float gyro[3];     // 陀螺仪数据,xyz
    float acc[3];      // 加速度计数据,xyz
    float temperature; // 温度
} IMU_Data_t;

typedef struct
{
    float   Yaw;
    float   Pitch;
    float   Roll;
    float   YawTotalAngle;
    int16_t YawRoundCount;
} IMU_Estimate_t;

#endif // _IMU_DATA_H_