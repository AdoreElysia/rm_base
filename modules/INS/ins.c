/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-09-14 08:49:07
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-09-26 23:04:27
 * @FilePath: /rm_base/modules/INS/ins.c
 * @Description: INS姿态解算实现
 */
#include "ins.h"
#include "modules_config.h"

#ifdef INS_MODULE
#include "QuaternionEKF.h"
#include "arm_math.h"
#include "bsp_dwt.h"
#include "imulayer.h"
#include "osal_def.h"
#include "user_lib.h"
#include <string.h>
#include <stdint.h>

#define log_tag  "ins"
#include "shell_log.h"

// 常量定义
#define INS_ACCEL_LPF_COEF 0.0085f  // 加速度低通滤波系数
#define INIT_SAMPLE_COUNT 100       // 初始化采样次数
#define GRAVITY_CONSTANT 9.81f      // 重力加速度常数

// 静态变量定义
static INS_t INS = {0};
static INS_Param_t IMU_Param = {0};

// 基向量定义
static const float xb[3] = {1.0f, 0.0f, 0.0f};
static const float yb[3] = {0.0f, 1.0f, 0.0f};
static const float zb[3] = {0.0f, 0.0f, 1.0f};

// 重力向量定义
static const float gravity[3] = {0.0f, 0.0f, GRAVITY_CONSTANT};

// 内部函数声明
static void IMU_Param_Correction(INS_Param_t *param, float gyro[3], float accel[3]);
static void InitQuaternion(float *init_q4);
static void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);
static void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);

/**
 * @brief 使用加速度计的数据初始化Roll和Pitch,Yaw置0
 * @param init_q4 输出的初始四元数
 */
static void InitQuaternion(float *init_q4)
{
    float acc_init[3] = {0.0f, 0.0f, 0.0f};
    static const float gravity_norm[3] = {0.0f, 0.0f, 1.0f}; // 导航系重力加速度矢量,归一化后为(0,0,1)
    float axis_rot[3] = {0.0f, 0.0f, 0.0f};                  // 旋转轴
    
    // 读取多次加速度计数据,取平均值作为初始值
    for (uint8_t i = 0; i < INIT_SAMPLE_COUNT; ++i)
    {
        imu_get_accel(&INS.IMU);
        acc_init[0] += INS.IMU.data.acc[0];
        acc_init[1] += INS.IMU.data.acc[1];
        acc_init[2] += INS.IMU.data.acc[2];
        DWT_Delay(0.001f);
    }
    
    // 计算平均值
    acc_init[0] /= INIT_SAMPLE_COUNT;
    acc_init[1] /= INIT_SAMPLE_COUNT;
    acc_init[2] /= INIT_SAMPLE_COUNT;
    
    // 归一化加速度向量
    Norm3d(acc_init);
    
    // 计算原始加速度矢量和导航系重力加速度矢量的夹角
    float angle = acosf(Dot3d(acc_init, gravity_norm));
    Cross3d(acc_init, gravity_norm, axis_rot);
    Norm3d(axis_rot);
    
    // 转换为四元数 (轴角公式)
    init_q4[0] = cosf(angle / 2.0f);
    init_q4[1] = axis_rot[0] * sinf(angle / 2.0f);
    init_q4[2] = axis_rot[1] * sinf(angle / 2.0f);
    init_q4[3] = axis_rot[2] * sinf(angle / 2.0f);
}
void INS_Init(void)
{
    // 初始化IMU参数修正结构体
    IMU_Param.scale[0] = 1.0f;
    IMU_Param.scale[1] = 1.0f;
    IMU_Param.scale[2] = 1.0f;
    IMU_Param.Yaw = 0.0f;
    IMU_Param.Pitch = 0.0f;
    IMU_Param.Roll = 0.0f;
    IMU_Param.flag = 1;
    
    // 初始化四元数
    float init_quaternion[4] = {0};
    InitQuaternion(init_quaternion);
    
    // 初始化EKF
    IMU_QuaternionEKF_Init(init_quaternion, 10.0f, 0.001f, 1000000.0f, 1.0f, 0.0f);
    
    // 设置加速度低通滤波系数
    INS.AccelLPF = INS_ACCEL_LPF_COEF;
}

void ins_task_function()
{
    if (INS.IMU.init_flag != 0)
    {
        // 获取时间间隔
        INS.dt = DWT_GetDeltaT(&INS.dwt_cnt);
            
        // 获取IMU数据
        imu_get_accel(&INS.IMU);
        imu_get_gyro(&INS.IMU);
        
        // IMU参数修正（安装误差修正）
        IMU_Param_Correction(&IMU_Param, INS.IMU.data.gyro, INS.IMU.data.acc);
        
        // 核心函数,EKF更新四元数
        IMU_QuaternionEKF_Update(INS.IMU.data.gyro[0], INS.IMU.data.gyro[1], INS.IMU.data.gyro[2], 
                                INS.IMU.data.acc[0], INS.IMU.data.acc[1], INS.IMU.data.acc[2], INS.dt);
        
        // 复制四元数结果
        memcpy(INS.q, QEKF_INS.q, sizeof(QEKF_INS.q));
        
        // 机体系基向量转换到导航坐标系（本例选取惯性系为导航系）
        BodyFrameToEarthFrame(xb, INS.xn, INS.q);
        BodyFrameToEarthFrame(yb, INS.yn, INS.q);
        BodyFrameToEarthFrame(zb, INS.zn, INS.q);
        
        // 将重力从导航坐标系n转换到机体系b,随后根据加速度计数据计算运动加速度
        float gravity_b[3];
        EarthFrameToBodyFrame(gravity, gravity_b, INS.q);
            
        // 计算运动加速度（去除重力分量），并进行低通滤波
        for (uint8_t i = 0; i < 3; ++i)
        {
            const float acc_diff = INS.IMU.data.acc[i] - gravity_b[i];
            INS.MotionAccel_b[i] = acc_diff * INS.dt / (INS.AccelLPF + INS.dt) + INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + INS.dt);
        }
            
        // 将运动加速度转换回导航系
        BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q);
        
        // 更新姿态角估计值
        INS.IMU_Estimate.Yaw = QEKF_INS.Yaw;
        INS.IMU_Estimate.Pitch = QEKF_INS.Pitch;
        INS.IMU_Estimate.Roll = QEKF_INS.Roll;
        INS.IMU_Estimate.YawTotalAngle = QEKF_INS.YawTotalAngle;
        INS.IMU_Estimate.YawRoundCount = QEKF_INS.YawRoundCount;
        
        imu_get_temp(&INS.IMU);
        // 温度控制
        imu_temp_ctrl(&INS.IMU, INS.IMU.data.temperature);
    }
}

void ins_init()
{
    osal_status_t status = OSAL_SUCCESS;
    // 初始化imu
    status = imu_init(&INS.IMU);
    if (status != OSAL_SUCCESS)
    {
        LOG_ERROR("Failed to init imu!");
        return;
    }
    // 初始化INS
    INS_Init();
    LOG_INFO("INS init successfully");
}

/**
 * @brief 将三维向量从机体坐标系转换到地球坐标系
 * @param vecBF 机体坐标系向量
 * @param vecEF 地球坐标系向量
 * @param q 四元数
 */
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief 将三维向量从地球坐标系转换到机体坐标系
 * @param vecEF 地球坐标系向量
 * @param vecBF 机体坐标系向量
 * @param q 四元数
 */
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}

/**
 * @brief IMU参数修正，用于修正IMU安装误差与标度因数误差
 * @param param IMU参数
 * @param gyro  角速度
 * @param accel 加速度
 */
static void IMU_Param_Correction(INS_Param_t *param, float gyro[3], float accel[3])
{
    static float lastYawOffset = 0.0f;
    static float lastPitchOffset = 0.0f;
    static float lastRollOffset = 0.0f;
    static float c_11 = 1.0f, c_12 = 0.0f, c_13 = 0.0f;
    static float c_21 = 0.0f, c_22 = 1.0f, c_23 = 0.0f;
    static float c_31 = 0.0f, c_32 = 0.0f, c_33 = 1.0f;
    
    // 检查是否需要重新计算变换矩阵
    if (fabsf(param->Yaw - lastYawOffset) > 0.001f ||
        fabsf(param->Pitch - lastPitchOffset) > 0.001f ||
        fabsf(param->Roll - lastRollOffset) > 0.001f || 
        param->flag)
    {
        // 角度转弧度
        const float yawRad = param->Yaw / 57.295779513f;
        const float pitchRad = param->Pitch / 57.295779513f;
        const float rollRad = param->Roll / 57.295779513f;
        
        // 计算三角函数值
        const float cosYaw = arm_cos_f32(yawRad);
        const float cosPitch = arm_cos_f32(pitchRad);
        const float cosRoll = arm_cos_f32(rollRad);
        const float sinYaw = arm_sin_f32(yawRad);
        const float sinPitch = arm_sin_f32(pitchRad);
        const float sinRoll = arm_sin_f32(rollRad);

        // 计算变换矩阵元素
        // 1.yaw(alpha) 2.pitch(beta) 3.roll(gamma)
        c_11 = cosYaw * cosRoll + sinYaw * sinPitch * sinRoll;
        c_12 = cosPitch * sinYaw;
        c_13 = cosYaw * sinRoll - cosRoll * sinYaw * sinPitch;
        c_21 = cosYaw * sinPitch * sinRoll - cosRoll * sinYaw;
        c_22 = cosYaw * cosPitch;
        c_23 = -sinYaw * sinRoll - cosYaw * cosRoll * sinPitch;
        c_31 = -cosPitch * sinRoll;
        c_32 = sinPitch;
        c_33 = cosPitch * cosRoll;
        
        param->flag = 0;
        lastYawOffset = param->Yaw;
        lastPitchOffset = param->Pitch;
        lastRollOffset = param->Roll;
    }
    
    // 应用标度因子并进行坐标变换
    float gyro_temp[3];
    gyro_temp[0] = gyro[0] * param->scale[0];
    gyro_temp[1] = gyro[1] * param->scale[1];
    gyro_temp[2] = gyro[2] * param->scale[2];

    gyro[0] = c_11 * gyro_temp[0] + c_12 * gyro_temp[1] + c_13 * gyro_temp[2];
    gyro[1] = c_21 * gyro_temp[0] + c_22 * gyro_temp[1] + c_23 * gyro_temp[2];
    gyro[2] = c_31 * gyro_temp[0] + c_32 * gyro_temp[1] + c_33 * gyro_temp[2];

    // 对加速度进行同样的变换
    const float accel_temp[3] = {accel[0], accel[1], accel[2]};
    
    accel[0] = c_11 * accel_temp[0] + c_12 * accel_temp[1] + c_13 * accel_temp[2];
    accel[1] = c_21 * accel_temp[0] + c_22 * accel_temp[1] + c_23 * accel_temp[2];
    accel[2] = c_31 * accel_temp[0] + c_32 * accel_temp[1] + c_33 * accel_temp[2];
}

#else  
void ins_task_function(){}
void ins_init(){};
#endif 
