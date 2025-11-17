/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-10-28 13:26:20
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-10-30 20:28:10
 * @FilePath: \rm_base\applications\gimbal_task\gimbal_task.c
 * @Description:
 */
#include "gimbal_task.h"
#include "app_config.h"
#include "compensation.h"
#include "damiao.h"
#include "dji.h"
#include "dm_imu.h"
#include "imu_data.h"
#include "ins.h"
#include "itc.h"
#include "offline.h"
#include "osal_def.h"
#include "robot_config.h"
#include "robot_def.h"
#include "tx_port.h"
#include <string.h>

#define log_tag "gimbal_task"
#include "shell_log.h"

#ifdef GIMBAL_BOARD

// gimbal 线程定义
static osal_thread_t                       gimbal_thread;
GIMBAL_THREAD_STACK_SECTION static uint8_t gimbal_thread_stack[GIMBAL_THREAD_STACK_SIZE];
// gimbal 电机指针定义
static DJIMotor_t *big_yaw_motor   = NULL; // 大云台yaw电机指针
static DJIMotor_t *small_yaw_motor = NULL; // 小云台yaw电机指针
static DMMOTOR_t  *pitch_motor     = NULL; // pitch电机指针
// 姿态角数据
static DM_IMU_DATA_t *dm_imu_data = NULL;
static INS_t         *ins         = NULL;
// gimbal命令接收
static itc_subscriber_t   gimbal_sub;
static Gimbal_Ctrl_Cmd_s *gimbal_cmd = NULL;

void gimbal_init(void)
{
    dm_imu_data = get_dm_imu_data();
    if (dm_imu_data == NULL)
    {
        LOG_ERROR("dm_imu_data is null");
        return;
    }
    ins = get_ins_ptr();
    if (ins == NULL)
    {
        LOG_ERROR("ins is null");
        return;
    }

    Motor_Init_Config_s yaw_config = {
        .offline_device_motor =
            {
                .name       = "6020_big",
                .timeout_ms = 100,
                .level      = OFFLINE_LEVEL_HIGH,
                .beep_times = 2,
                .enable     = OFFLINE_ENABLE,
            },
        .can_init_config =
            {
                .can_handle = &hcan2,
                .tx_id      = 1,
            },
        .controller_param_init_config = {.other_angle_feedback_ptr =
                                             &dm_imu_data->estimate.YawTotalAngle,
                                         .other_speed_feedback_ptr = &dm_imu_data->imu_data.gyro[2],
                                         .lqr_config =
                                             {
                                                 .K          = {31.62f, 4.76f},
                                                 .output_max = 2.223,
                                                 .output_min = -2.223,
                                                 .state_dim  = 2,
                                             }},
        .controller_setting_init_config =
            {
                .control_algorithm     = CONTROL_LQR,
                .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
                .angle_feedback_source = OTHER_FEED,
                .speed_feedback_source = OTHER_FEED,
                .outer_loop_type       = ANGLE_LOOP,
                .close_loop_type       = ANGLE_LOOP | SPEED_LOOP,
            },
        .Motor_init_Info = {.motor_type      = GM6020_CURRENT,
                            .max_current     = 3.0f,
                            .gear_ratio      = 1,
                            .max_torque      = 2.223,
                            .max_speed       = 320,
                            .torque_constant = 0.741f}};
    big_yaw_motor = DJIMotorInit(&yaw_config);
    if (big_yaw_motor == NULL)
    {
        LOG_ERROR("big_yaw_motor init failed");
        return;
    }

    Motor_Init_Config_s small_yaw_config = {
        .offline_device_motor =
            {
                .name       = "6020_small",
                .timeout_ms = 100,
                .level      = OFFLINE_LEVEL_HIGH,
                .beep_times = 3,
                .enable     = OFFLINE_ENABLE,
            },
        .can_init_config =
            {
                .can_handle = &hcan1,
                .tx_id      = 1,
            },
        .controller_param_init_config = {.other_angle_feedback_ptr =
                                             &ins->IMU_Estimate.YawTotalAngle,
                                         .other_speed_feedback_ptr = &ins->IMU.data.gyro[2],
                                         .lqr_config =
                                             {
                                                 .K          = {17.32f, 1.0f},
                                                 .output_max = 2.223,
                                                 .output_min = -2.223,
                                                 .state_dim  = 2,
                                             }},
        .controller_setting_init_config =
            {
                .control_algorithm     = CONTROL_LQR,
                .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
                .angle_feedback_source = OTHER_FEED,
                .speed_feedback_source = OTHER_FEED,
                .outer_loop_type       = ANGLE_LOOP,
                .close_loop_type       = ANGLE_LOOP | SPEED_LOOP,
            },
        .Motor_init_Info = {.motor_type      = GM6020_CURRENT,
                            .max_current     = 3.0f,
                            .gear_ratio      = 1,
                            .max_torque      = 2.223,
                            .max_speed       = 320,
                            .torque_constant = 0.741f}};
    small_yaw_motor = DJIMotorInit(&small_yaw_config);
    if (small_yaw_motor == NULL)
    {
        LOG_ERROR("small_yaw_motor init failed");
        return;
    }

    // PITCH
    Motor_Init_Config_s pitch_config = {
        .offline_device_motor =
            {
                .name       = "dm4310",           // 设备名称
                .timeout_ms = 100,                // 超时时间
                .level      = OFFLINE_LEVEL_HIGH, // 离线等级
                .beep_times = 4,                  // 蜂鸣次数
                .enable     = OFFLINE_ENABLE,     // 是否启用离线管理
            },
        .can_init_config =
            {
                .can_handle = &hcan1,
                .tx_id      = 0X23,
                .rx_id      = 0X206,
            },
        .controller_param_init_config =
            {.other_angle_feedback_ptr = &ins->IMU_Estimate.Pitch,
             .other_speed_feedback_ptr = &ins->IMU.data.gyro[0],
             .lqr_config               = {.K                = {31.62f, 1.32f}, // 28.7312f,2.5974f
                                          .output_max       = 7,
                                          .output_min       = -7,
                                          .state_dim        = 2,
                                          .feedforward_func = create_gravity_compensation_wrapper(16, 0.09)}},
        .controller_setting_init_config =
            {
                .control_algorithm     = CONTROL_LQR,
                .feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE,
                .angle_feedback_source = OTHER_FEED,
                .speed_feedback_source = OTHER_FEED,
                .outer_loop_type       = ANGLE_LOOP,
                .close_loop_type       = ANGLE_LOOP | SPEED_LOOP,
            },
        .Motor_init_Info = {.motor_type      = DM4310,
                            .max_current     = 7.5f,
                            .gear_ratio      = 10,
                            .max_torque      = 7,
                            .max_speed       = 200,
                            .torque_constant = 0.093f}};
    pitch_motor = DMMotorInit(&pitch_config, DM_MIT_MODE);
    if (pitch_motor == NULL)
    {
        LOG_ERROR("pitch_motor init failed");
        return;
    }

    osal_status_t status;
    // gimbal 命令初始化
    status = itc_subscriber_init(&gimbal_sub, "gimbal_cmd_topic", sizeof(Gimbal_Ctrl_Cmd_s));
    if (status != OSAL_SUCCESS)
    {
        LOG_ERROR("gimbal subscriber init failed");
        return;
    }

    LOG_INFO("gimbal init sucess");
}

void gimbal_task(ULONG thread_input)
{
    while (1)
    {
        gimbal_cmd = (Gimbal_Ctrl_Cmd_s *)itc_receive(&gimbal_sub);
        if (gimbal_cmd != NULL)
        {
            if (!offline_module_get_device_status(big_yaw_motor->offline_index) &&
                !offline_module_get_device_status(small_yaw_motor->offline_index) &&
                !offline_module_get_device_status(pitch_motor->offline_index))
            {
                switch (gimbal_cmd->gimbal_mode)
                {
                case GIMBAL_ZERO_FORCE:
                    DJIMotorStop(big_yaw_motor);
                    DJIMotorStop(small_yaw_motor);
                    DMMotorStop(pitch_motor);
                    break;
                case GIMBAL_GYRO_MODE:
                    DJIMotorEnable(big_yaw_motor);
                    DJIMotorEnable(small_yaw_motor);
                    DMMotorEnable(pitch_motor);
                    // 大小yaw协调控制
                    // 使用电机编码器数据作为小yaw的实际位置反馈
                    float small_yaw_current = small_yaw_motor->measure.total_angle;
                    // 计算小yaw相对于中心位置的偏移
                    float small_yaw_offset = small_yaw_current - SMALL_YAW_ALIGN_ANGLE;
                    // 控制大yaw跟随小yaw同向转动，使小yaw保持在中心
                    // 大yaw需要向相同方向转动相同角度，以抵消小yaw的偏移
                    float big_yaw_motor_target_angle =
                        dm_imu_data->estimate.YawTotalAngle + small_yaw_offset;
                    DJIMotorSetRef(big_yaw_motor, big_yaw_motor_target_angle);
                    DJIMotorSetRef(small_yaw_motor, gimbal_cmd->yaw);
                    DMMotorSetRef(pitch_motor, gimbal_cmd->pitch);
                    break;
                case GIMBAL_AUTO_MODE:
                    DJIMotorEnable(big_yaw_motor);
                    DJIMotorEnable(small_yaw_motor);
                    DMMotorEnable(pitch_motor);
                    break;
                }
            }
            else
            {
                DJIMotorStop(big_yaw_motor);
                DJIMotorStop(small_yaw_motor);
                DMMotorStop(pitch_motor);
            }
        }
        osal_delay_ms(2);
    }
}

void gimbal_task_init()
{
    gimbal_init();
    osal_status_t status;
    status = osal_thread_create(&gimbal_thread, "gimbal_task", gimbal_task, 0, gimbal_thread_stack,
                                GIMBAL_THREAD_STACK_SIZE, GIMBAL_THREAD_PRIORITY);
    if (status != OSAL_SUCCESS)
    {
        LOG_ERROR("gimbal task create failed");
        return;
    }
    osal_thread_start(&gimbal_thread);
    LOG_INFO("gimbal task init sucess");
}
#else
void gimbal_task_init()
{
}
#endif
