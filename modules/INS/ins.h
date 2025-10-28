/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-10-02 14:23:32
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-10-28 20:28:21
 * @FilePath: \rm_base\modules\INS\ins.h
 * @Description: 
 */
/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-09-14 08:49:46
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-10-28 20:20:11
 * @FilePath: \rm_base\modules\INS\ins.h
 * @Description: INS姿态解算模块
 */

#ifndef _INS_H_
#define _INS_H_

#include "imu_data.h"
#include "imulayer.h"
#include <stdint.h>

// INS核心结构体
typedef struct
{
    // 四元数估计值
    float q[4]; 
    // 机体坐标加速度
    float MotionAccel_b[3]; 
    // 绝对系加速度
    float MotionAccel_n[3]; 
    // 加速度低通滤波系数
    float AccelLPF; 
    // bodyframe在绝对系的向量表示
    float xn[3];
    float yn[3];
    float zn[3];
    // 加速度在机体系和XY两轴的夹角
    // float atanxz;
    // float atanyz;
    // 位姿
    IMU_Estimate_t IMU_Estimate;
    // imu实例
    IMU_Instance_t IMU;
    // 采样时间间隔
    float dt;
    uint32_t dwt_cnt;
} INS_t;

/* 用于修正安装误差的参数 */
typedef struct
{
    uint8_t flag;
    float scale[3];
    float Yaw;
    float Pitch;
    float Roll;
} INS_Param_t;

// 函数声明
/**
 * @description: 初始化INS模块
 * @param {INS_t} *ins,INS结构体指针
 * @param {INS_Param_t} *param,安装参数结构体指针
 * @return {*}
 */
void ins_init(INS_t *ins, INS_Param_t *param);
/**
 * @brief IMU参数修正，用于修正IMU安装误差与标度因数误差
 * @param param IMU参数
 * @param gyro  角速度
 * @param accel 加速度
 */
void IMU_Param_Correction(INS_Param_t *param, float gyro[3], float accel[3]);
/**
 * @brief 使用加速度计的数据初始化Roll和Pitch,Yaw置0
 * @param init_q4 输出的初始四元数
 */
void InitQuaternion(float *init_q4);
/**
 * @brief 将三维向量从机体坐标系转换到地球坐标系
 * @param vecBF 机体坐标系向量
 * @param vecEF 地球坐标系向量
 * @param q 四元数
 */
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);
/**
 * @brief 将三维向量从地球坐标系转换到机体坐标系
 * @param vecEF 地球坐标系向量
 * @param vecBF 机体坐标系向量
 * @param q 四元数
 */
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);
/**
 * @brief 获取INS结构体指针
 * @return {INS_t*},INS结构体指针
 */
INS_t* get_ins_ptr(void);

#endif // _INS_H_  