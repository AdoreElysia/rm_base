/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-09-14 08:49:07
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-10-28 20:29:12
 * @FilePath: \rm_base\modules\INS\ins.c
 * @Description: INS姿态解算实现
 */
#include "ins.h"
#include "QuaternionEKF.h"
#include "arm_math.h"
#include "bsp_dwt.h"
#include "imulayer.h"
#include "osal_def.h"
#include "user_lib.h"
#include <stdint.h>
#include <string.h>

#define log_tag "ins"
#include "shell_log.h"

#define INIT_SAMPLE_COUNT  100     // 初始化采样次数
#define INS_ACCEL_LPF_COEF 0.0085f // 加速度低通滤波系数

static INS_t *ins_module = NULL;

void ins_init(INS_t *ins, INS_Param_t *param)
{
    if (ins == NULL || param == NULL)
    {
        LOG_ERROR(log_tag, "INS or Param is NULL");
        return;
    }

    // 将外部传入的ins指针赋值给模块内部静态变量
    ins_module = ins;

    osal_status_t status = OSAL_SUCCESS;
    // 初始化imu
    status = imu_init(&ins_module->IMU);
    if (status != OSAL_SUCCESS)
    {
        LOG_ERROR("Failed to init imu!");
        return;
    }
    // 初始化IMU参数修正结构体
    param->scale[0] = 1.0f;
    param->scale[1] = 1.0f;
    param->scale[2] = 1.0f;
    param->Yaw      = 0.0f;
    param->Pitch    = 0.0f;
    param->Roll     = 0.0f;
    param->flag     = 1;
    // 初始化四元数
    float init_quaternion[4] = {0};
    InitQuaternion(init_quaternion);
    // 初始化EKF
    IMU_QuaternionEKF_Init(init_quaternion, 10.0f, 0.001f, 1000000.0f, 1.0f, 0.0f);
    // 设置加速度低通滤波系数
    ins_module->AccelLPF = INS_ACCEL_LPF_COEF;

    LOG_INFO("INS init successfully");
}

void InitQuaternion(float *init_q4)
{
    if (ins_module == NULL)
    {
        LOG_ERROR("INS module is NULL");
        return;
    }
    float              acc_init[3]     = {0.0f, 0.0f, 0.0f};
    static const float gravity_norm[3] = {0.0f, 0.0f,
                                          1.0f}; // 导航系重力加速度矢量,归一化后为(0,0,1)
    float              axis_rot[3]     = {0.0f, 0.0f, 0.0f}; // 旋转轴

    // 读取多次加速度计数据,取平均值作为初始值
    for (uint8_t i = 0; i < INIT_SAMPLE_COUNT; ++i)
    {
        imu_get_accel(&ins_module->IMU);
        acc_init[0] += ins_module->IMU.data.acc[0];
        acc_init[1] += ins_module->IMU.data.acc[1];
        acc_init[2] += ins_module->IMU.data.acc[2];
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

void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
    vecEF[0] =
        2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                (q[1] * q[2] - q[0] * q[3]) * vecBF[1] + (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] =
        2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] + (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
    vecBF[0] =
        2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                (q[1] * q[2] + q[0] * q[3]) * vecEF[1] + (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] =
        2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] + (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}
void IMU_Param_Correction(INS_Param_t *param, float gyro[3], float accel[3])
{
    static float lastYawOffset   = 0.0f;
    static float lastPitchOffset = 0.0f;
    static float lastRollOffset  = 0.0f;
    static float c_11 = 1.0f, c_12 = 0.0f, c_13 = 0.0f;
    static float c_21 = 0.0f, c_22 = 1.0f, c_23 = 0.0f;
    static float c_31 = 0.0f, c_32 = 0.0f, c_33 = 1.0f;

    // 检查是否需要重新计算变换矩阵
    if (fabsf(param->Yaw - lastYawOffset) > 0.001f ||
        fabsf(param->Pitch - lastPitchOffset) > 0.001f ||
        fabsf(param->Roll - lastRollOffset) > 0.001f || param->flag)
    {
        // 角度转弧度
        const float yawRad   = param->Yaw / 57.295779513f;
        const float pitchRad = param->Pitch / 57.295779513f;
        const float rollRad  = param->Roll / 57.295779513f;

        // 计算三角函数值
        const float cosYaw   = arm_cos_f32(yawRad);
        const float cosPitch = arm_cos_f32(pitchRad);
        const float cosRoll  = arm_cos_f32(rollRad);
        const float sinYaw   = arm_sin_f32(yawRad);
        const float sinPitch = arm_sin_f32(pitchRad);
        const float sinRoll  = arm_sin_f32(rollRad);

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

        param->flag     = 0;
        lastYawOffset   = param->Yaw;
        lastPitchOffset = param->Pitch;
        lastRollOffset  = param->Roll;
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

INS_t *get_ins_ptr(void)
{
    if (ins_module != NULL)
    {
        return ins_module;
    }
    return NULL;
}