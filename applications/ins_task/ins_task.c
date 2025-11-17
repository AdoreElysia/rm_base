/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-09-14 13:43:30
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-10-02 15:18:00
 * @FilePath: \rm_base\applications\ins_task\ins_task.c
 * @Description:
 */
#include "QuaternionEKF.h"
#include "app_config.h"
#include "bsp_dwt.h"
#include "ins.h"
#include "modules_config.h"
#include "osal_def.h"
#include <string.h>

#ifdef INS_MODULE

#define log_tag "ins_task"
#include "shell_log.h"

static osal_thread_t                    INS_thread                              = {0};
INS_THREAD_STACK_SECTION static uint8_t INS_thread_stack[INS_THREAD_STACK_SIZE] = {0};

// 常量定义
#define GRAVITY_CONSTANT 9.81f // 重力加速度常数

// 静态变量定义
static INS_t       INS       = {0};
static INS_Param_t IMU_Param = {0};

// 基向量定义
static const float xb[3] = {1.0f, 0.0f, 0.0f};
static const float yb[3] = {0.0f, 1.0f, 0.0f};
static const float zb[3] = {0.0f, 0.0f, 1.0f};

// 重力向量定义
static const float gravity[3] = {0.0f, 0.0f, GRAVITY_CONSTANT};

void ins_task(ULONG thread_input)
{
    while (1)
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
            IMU_QuaternionEKF_Update(INS.IMU.data.gyro[0], INS.IMU.data.gyro[1],
                                     INS.IMU.data.gyro[2], INS.IMU.data.acc[0], INS.IMU.data.acc[1],
                                     INS.IMU.data.acc[2], INS.dt);

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
                INS.MotionAccel_b[i] =
                    acc_diff * INS.dt / (INS.AccelLPF + INS.dt) +
                    INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + INS.dt);
            }

            // 将运动加速度转换回导航系
            BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q);

            // 更新姿态角估计值
            INS.IMU_Estimate.Yaw           = QEKF_INS.Yaw;
            INS.IMU_Estimate.Pitch         = QEKF_INS.Pitch;
            INS.IMU_Estimate.Roll          = QEKF_INS.Roll;
            INS.IMU_Estimate.YawTotalAngle = QEKF_INS.YawTotalAngle;
            INS.IMU_Estimate.YawRoundCount = QEKF_INS.YawRoundCount;

            imu_get_temp(&INS.IMU);
            // 温度控制
            imu_temp_ctrl(&INS.IMU, INS.IMU.data.temperature);
        }
        osal_delay_ms(1);
    }
}

void ins_task_init()
{
    memset(&INS, 0, sizeof(INS));
    memset(&IMU_Param, 0, sizeof(IMU_Param));
    ins_init(&INS, &IMU_Param);

    osal_status_t status = osal_thread_create(&INS_thread, "insTask", ins_task, 0, INS_thread_stack,
                                              INS_THREAD_STACK_SIZE, INS_THREAD_PRIORITY);
    if (status != OSAL_SUCCESS)
    {
        LOG_ERROR("Failed to create ins task!");
        return;
    }

    // 启动线程
    osal_thread_start(&INS_thread);
    LOG_INFO("INS init and task created successfully");
}
#else
void ins_task_init()
{
}
#endif
