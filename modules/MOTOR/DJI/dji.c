#include "dji.h"
#include "bsp_can.h"
#include "can.h"
#include "motor_def.h"
#include "offline.h"
#include "user_lib.h"
#include <stdint.h>
#include <string.h>

#define log_tag "dji"
#include "shell_log.h"

#define ECD_ANGLE_COEF_DJI 0.043945f // (360/8192),将编码器值转化为角度制

static DJIMotor_t *dji_motor_list = NULL;

/**
 * @brief 由于DJI电机发送以四个一组的形式进行,故对其进行特殊处理,
 *        该变量将在 DJIMotorControl() 中使用,分组在 MotorSenderGrouping()中进行
 *
 * C610(m2006)/C620(m3508):0x1ff,0x200;
 * GM6020:0x1ff,0x2ff 0x1fe 0x2fe
 * 反馈(rx_id): GM6020: 0x204+id ; C610/C620: 0x200+id
 * can1: [0]:0x1FF,[1]:0x200,[2]:0x2FF [3]:0x1fe
 * can2: [3]:0x1FF,[4]:0x200,[5]:0x2FF
 */
static CanTxMessage_t sender_assignment[10] = {
    [0] =
        {
            .can_handle   = &hcan1,
            .txconf.StdId = 0x1ff,
            .txconf.IDE   = CAN_ID_STD,
            .txconf.RTR   = CAN_RTR_DATA,
            .txconf.DLC   = 8,
            .tx_buff      = {0},
        },
    [1] =
        {
            .can_handle   = &hcan1,
            .txconf.StdId = 0x200,
            .txconf.IDE   = CAN_ID_STD,
            .txconf.RTR   = CAN_RTR_DATA,
            .txconf.DLC   = 8,
            .tx_buff      = {0},
        },
    [2] =
        {
            .can_handle   = &hcan1,
            .txconf.StdId = 0x2ff,
            .txconf.IDE   = CAN_ID_STD,
            .txconf.RTR   = CAN_RTR_DATA,
            .txconf.DLC   = 8,
            .tx_buff      = {0},
        },
    [3] =
        {
            .can_handle   = &hcan1,
            .txconf.StdId = 0x1fe,
            .txconf.IDE   = CAN_ID_STD,
            .txconf.RTR   = CAN_RTR_DATA,
            .txconf.DLC   = 8,
            .tx_buff      = {0},
        },
    [4] =
        {
            .can_handle   = &hcan1,
            .txconf.StdId = 0x2fe,
            .txconf.IDE   = CAN_ID_STD,
            .txconf.RTR   = CAN_RTR_DATA,
            .txconf.DLC   = 8,
            .tx_buff      = {0},
        },

    [5] =
        {
            .can_handle   = &hcan2,
            .txconf.StdId = 0x1ff,
            .txconf.IDE   = CAN_ID_STD,
            .txconf.RTR   = CAN_RTR_DATA,
            .txconf.DLC   = 8,
            .tx_buff      = {0},
        },
    [6] =
        {
            .can_handle   = &hcan2,
            .txconf.StdId = 0x200,
            .txconf.IDE   = CAN_ID_STD,
            .txconf.RTR   = CAN_RTR_DATA,
            .txconf.DLC   = 8,
            .tx_buff      = {0},
        },
    [7] =
        {
            .can_handle   = &hcan2,
            .txconf.StdId = 0x2ff,
            .txconf.IDE   = CAN_ID_STD,
            .txconf.RTR   = CAN_RTR_DATA,
            .txconf.DLC   = 8,
            .tx_buff      = {0},
        },
    [8] =
        {
            .can_handle   = &hcan2,
            .txconf.StdId = 0x1fe,
            .txconf.IDE   = CAN_ID_STD,
            .txconf.RTR   = CAN_RTR_DATA,
            .txconf.DLC   = 8,
            .tx_buff      = {0},
        },
    [9] =
        {
            .can_handle   = &hcan2,
            .txconf.StdId = 0x2fe,
            .txconf.IDE   = CAN_ID_STD,
            .txconf.RTR   = CAN_RTR_DATA,
            .txconf.DLC   = 8,
            .tx_buff      = {0},
        },
};

/**
 * @brief
 * 6个用于确认是否有电机注册到sender_assignment中的标志位,防止发送空帧,此变量将在DJIMotorControl()使用
 *        flag的初始化在 MotorSenderGrouping()中进行
 */
static uint8_t sender_enable_flag[10] = {0};

/**
 * @brief 根据电调/拨码开关上的ID,根据说明书的默认id分配方式计算发送ID和接收ID,
 *        并对电机进行分组以便处理多电机控制命令
 */
static void MotorSenderGrouping(DJIMotor_t *motor, Can_Device_Init_Config_s *config)
{
    uint8_t motor_id = config->tx_id - 1; // 下标从零开始,先减一方便赋值
    uint8_t motor_send_num;
    uint8_t motor_grouping;

    switch (motor->DJI_Motor_Info.motor_type)
    {
    case M2006:
    case M3508:
        if (motor_id < 4) // 根据ID分组
        {
            motor_send_num = motor_id;
            motor_grouping = config->can_handle == &hcan1 ? 1 : 6;
        }
        else
        {
            motor_send_num = motor_id - 4;
            motor_grouping = config->can_handle == &hcan1 ? 0 : 5;
        }

        // 计算接收id并设置分组发送id
        config->rx_id                      = 0x200 + motor_id + 1; // 把ID+1,进行分组设置
        sender_enable_flag[motor_grouping] = 1;                    // 设置发送标志位,防止发送空帧
        motor->message_num                 = motor_send_num;
        motor->sender_group                = motor_grouping;

        for (size_t i = 0; i < DJI_MOTOR_CNT; ++i)
        {
            if (dji_motor_list[i].can_device &&
                dji_motor_list[i].can_device->can_handle == config->can_handle &&
                dji_motor_list[i].can_device->rx_id == config->rx_id)
            {
                // 6020的id 1-4和2006/3508的id 5-8会发生冲突(若有注册,即1!5,2!6,3!7,4!8) (1!5!,LTC!
                // (((不是)
                LOG_ERROR("ID crash,id [%d], can_bus [%s]", config->rx_id,
                          (config->can_handle == &hcan1 ? "can1" : "can2"));
            }
        }
        break;

    case GM6020_CURRENT:
        if (motor_id < 4)
        {
            motor_send_num = motor_id;
            motor_grouping = config->can_handle == &hcan1 ? 3 : 8;
        }
        else
        {
            motor_send_num = motor_id - 4;
            motor_grouping = config->can_handle == &hcan1 ? 2 : 7;
        }

        config->rx_id = 0x204 + motor_id + 1; // 把ID+1,进行分组设置
        sender_enable_flag[motor_grouping] =
            1; // 只要有电机注册到这个分组,置为1;在发送函数中会通过此标志判断是否有电机注册
        motor->message_num  = motor_send_num;
        motor->sender_group = motor_grouping;
        for (size_t i = 0; i < DJI_MOTOR_CNT; ++i)
        {
            if (dji_motor_list[i].can_device &&
                dji_motor_list[i].can_device->can_handle == config->can_handle &&
                dji_motor_list[i].can_device->rx_id == config->rx_id)
            {
                LOG_ERROR("ID crash,id [%d], can_bus [%s]", config->rx_id,
                          (config->can_handle == &hcan1 ? "can1" : "can2"));
            }
        }
        break;

    case GM6020_VOLTAGE:
        if (motor_id < 4)
        {
            motor_send_num = motor_id;
            motor_grouping = config->can_handle == &hcan1 ? 0 : 5;
        }
        else
        {
            motor_send_num = motor_id - 4;
            motor_grouping = config->can_handle == &hcan1 ? 4 : 9;
        }

        config->rx_id = 0x204 + motor_id + 1; // 把ID+1,进行分组设置
        sender_enable_flag[motor_grouping] =
            1; // 只要有电机注册到这个分组,置为1;在发送函数中会通过此标志判断是否有电机注册
        motor->message_num  = motor_send_num;
        motor->sender_group = motor_grouping;
        for (size_t i = 0; i < DJI_MOTOR_CNT; ++i)
        {
            if (dji_motor_list[i].can_device &&
                dji_motor_list[i].can_device->can_handle == config->can_handle &&
                dji_motor_list[i].can_device->rx_id == config->rx_id)
            {
                LOG_ERROR("ID crash,id [%d], can_bus [%s]", config->rx_id,
                          (config->can_handle == &hcan1 ? "can1" : "can2"));
            }
        }
        break;
    default:
        break;
    }
}

void DecodeDJIMotor(DJIMotor_t *motor)
{
    if (motor == NULL || motor->can_device == NULL)
    {
        return;
    }
    // 更新电机数据
    motor->measure.last_ecd = motor->measure.ecd;
    motor->measure.ecd =
        ((uint16_t)motor->can_device->rx_buff[0] << 8) | motor->can_device->rx_buff[1];
    motor->measure.angle_single_round = ECD_ANGLE_COEF_DJI * (float)motor->measure.ecd;
    motor->measure.speed_rpm =
        (int16_t)(motor->can_device->rx_buff[2] << 8 | motor->can_device->rx_buff[3]);
    motor->measure.speed_rad = RPM_2_RAD_PER_SEC * (int16_t)(motor->can_device->rx_buff[2] << 8 |
                                                             motor->can_device->rx_buff[3]);
    motor->measure.real_current =
        (int16_t)(motor->can_device->rx_buff[4] << 8 | motor->can_device->rx_buff[5]);
    motor->measure.temperature = motor->can_device->rx_buff[6];
    // 多圈角度计算
    int16_t delta_ecd = motor->measure.ecd - motor->measure.last_ecd;
    if (delta_ecd > 4096)
    {
        motor->measure.total_round--;
    }
    else if (delta_ecd < -4096)
    {
        motor->measure.total_round++;
    }
    motor->measure.total_angle =
        motor->measure.total_round * 360.0f + motor->measure.angle_single_round;
    // 更新在线状态
    offline_module_device_update(motor->offline_index);
}

// 电机初始化,返回一个电机实例
DJIMotor_t *DJIMotorInit(Motor_Init_Config_s *config)
{
    // 检查dji_motor_list是否已初始化
    if (dji_motor_list == NULL)
    {
        LOG_ERROR("DJI motor list not initialized\n");
        return NULL;
    }

    // 查找一个空位来初始化电机
    DJIMotor_t *DJIMotor = NULL;
    for (int i = 0; i < DJI_MOTOR_CNT; i++)
    {
        if (dji_motor_list[i].can_device == NULL)
        {
            DJIMotor = &dji_motor_list[i];
            memset(DJIMotor, 0, sizeof(DJIMotor_t));
            break;
        }
    }

    // 检查是否找到空位
    if (DJIMotor == NULL)
    {
        LOG_ERROR("DJI motor count exceeds maximum limit\n");
        return NULL;
    }

    // motor basic setting 电机基本设置
    DJIMotor->DJI_Motor_Info = config->Motor_init_Info;                // 电机信息
    DJIMotor->motor_settings = config->controller_setting_init_config; // 正反转,闭环类型等

    // 电机分组,因为至多4个电机可以共用一帧CAN控制报文
    MotorSenderGrouping(DJIMotor, &config->can_init_config); // 更新rx_id
    // CAN 设备初始化配置
    Can_Device_Init_Config_s can_config = {
        .can_handle = config->can_init_config.can_handle,
        .tx_id      = config->can_init_config.tx_id,
        .rx_id      = config->can_init_config.rx_id,
        .tx_mode    = CAN_MODE_BLOCKING,
        .rx_mode    = CAN_MODE_IT,
    };
    // 注册 CAN 设备并获取引用
    DJIMotor->can_device = BSP_CAN_Device_Init(&can_config);
    if (DJIMotor->can_device == NULL)
    {
        LOG_ERROR("Failed to initialize CAN device for DJI motor");
        return NULL;
    }

    DJIMotor->motor_settings.control_algorithm =
        config->controller_setting_init_config.control_algorithm;
    switch (config->controller_setting_init_config.control_algorithm)
    {
    case CONTROL_PID:
        // motor controller init 电机控制器初始化
        PIDInit(&DJIMotor->motor_controller.speed_PID,
                &config->controller_param_init_config.speed_PID);
        PIDInit(&DJIMotor->motor_controller.angle_PID,
                &config->controller_param_init_config.angle_PID);
        DJIMotor->motor_controller.other_angle_feedback_ptr =
            config->controller_param_init_config.other_angle_feedback_ptr;
        DJIMotor->motor_controller.other_speed_feedback_ptr =
            config->controller_param_init_config.other_speed_feedback_ptr;
        break;
    case CONTROL_LQR:
        LQRInit(&DJIMotor->motor_controller.lqr, &config->controller_param_init_config.lqr_config);
        DJIMotor->motor_controller.other_angle_feedback_ptr =
            config->controller_param_init_config.other_angle_feedback_ptr;
        DJIMotor->motor_controller.other_speed_feedback_ptr =
            config->controller_param_init_config.other_speed_feedback_ptr;
        break;
    case CONTROL_OTHER:
        // 未来添加其他控制算法的初始化
        break;
    }

    // 掉线检测
    DJIMotor->offline_index = offline_module_device_register(&config->offline_device_motor);

    LOG_INFO("DJI motor initialized on CAN bus [%s] with TX ID: 0x%X, RX ID: 0x%X\n",
             (can_config.can_handle == &hcan1 ? "CAN1" : "CAN2"), can_config.tx_id,
             can_config.rx_id);
    return DJIMotor;
}

void DJIMotorChangeFeed(DJIMotor_t *motor, Closeloop_Type_e loop, Feedback_Source_e type)
{
    if (loop == ANGLE_LOOP)
        motor->motor_settings.angle_feedback_source = type;
    else if (loop == SPEED_LOOP)
        motor->motor_settings.speed_feedback_source = type;
    else
        LOG_ERROR("loop type error, check and func param\n");
}

void DJIMotorStop(DJIMotor_t *motor)
{
    motor->stop_flag = MOTOR_STOP;
}

void DJIMotorEnable(DJIMotor_t *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}

/* 修改电机的实际闭环对象 */
void DJIMotorOuterLoop(DJIMotor_t *motor, Closeloop_Type_e outer_loop,
                       LQR_Init_Config_s *lqr_config)
{
    // 更新外环类型
    motor->motor_settings.outer_loop_type = outer_loop;

    // 如果是LQR控制且提供了配置参数，则重新初始化，其他算法传递NULL即可
    if (motor->motor_settings.control_algorithm == CONTROL_LQR && lqr_config != NULL)
    {
        LQRInit(&motor->motor_controller.lqr, lqr_config);
    }
}

// 设置参考值
void DJIMotorSetRef(DJIMotor_t *motor, float ref)
{
    switch (motor->motor_settings.control_algorithm)
    {
    case CONTROL_PID:
    case CONTROL_LQR:
        motor->motor_controller.ref = ref;
        break;
    case CONTROL_OTHER:
        break;
    }
}

static float CalculatePIDOutput(DJIMotor_t *motor)
{
    float pid_measure, pid_ref;

    pid_ref = motor->motor_controller.ref;
    if (motor->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
    {
        pid_ref *= -1;
    }

    // pid_ref会顺次通过被启用的闭环充当数据的载体
    // 计算位置环,只有启用位置环且外层闭环为位置时会计算速度环输出
    if ((motor->motor_settings.close_loop_type & ANGLE_LOOP) &&
        motor->motor_settings.outer_loop_type == ANGLE_LOOP)
    {
        if (motor->motor_settings.angle_feedback_source == OTHER_FEED)
            pid_measure = *motor->motor_controller.other_angle_feedback_ptr;
        else
            pid_measure = motor->measure.total_angle;

        if (motor->motor_settings.feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE)
            pid_measure *= -1;
        // 更新pid_ref进入下一个环
        pid_ref = PIDCalculate(&motor->motor_controller.angle_PID, pid_measure, pid_ref);
    }
    // 计算速度环,(外层闭环为速度或位置)且(启用速度环)时会计算速度环
    if ((motor->motor_settings.close_loop_type & SPEED_LOOP) &&
        (motor->motor_settings.outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)))
    {
        if (motor->motor_settings.speed_feedback_source == OTHER_FEED)
            pid_measure = *motor->motor_controller.other_speed_feedback_ptr;
        else // MOTOR_FEED
            pid_measure = motor->measure.speed_rpm;

        if (motor->motor_settings.feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE)
            pid_measure *= -1;
        // 更新pid_ref
        pid_ref = PIDCalculate(&motor->motor_controller.speed_PID, pid_measure, pid_ref);
    }

    return pid_ref;
}

static float CalculateLQROutput(DJIMotor_t *motor)
{
    float degree = 0.0f, angular_velocity = 0.0f, lqr_ref = 0.0f;

    lqr_ref = motor->motor_controller.ref;
    if (motor->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
    {
        lqr_ref *= -1;
    }

    // 位置状态计算
    if ((motor->motor_settings.close_loop_type & ANGLE_LOOP) &&
        motor->motor_settings.outer_loop_type == ANGLE_LOOP)
    {
        degree = (motor->motor_settings.angle_feedback_source == OTHER_FEED)
                     ? *motor->motor_controller.other_angle_feedback_ptr
                     : motor->measure.total_angle;

        if (motor->motor_settings.feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE)
            degree *= -1;
    }

    // 速度状态计算
    if ((motor->motor_settings.close_loop_type & SPEED_LOOP) &&
        (motor->motor_settings.outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)))
    {
        angular_velocity = (motor->motor_settings.speed_feedback_source == OTHER_FEED)
                               ? *motor->motor_controller.other_speed_feedback_ptr
                               : motor->measure.speed_rad;

        if (motor->motor_settings.feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE)
            angular_velocity *= -1;
    }

    float torque = LQRCalculate(&motor->motor_controller.lqr, degree, angular_velocity, lqr_ref);

    switch (motor->DJI_Motor_Info.motor_type)
    {
    case GM6020_CURRENT:
    {
        return currentToInteger(-3.0f, 3.0f, -16384, 16384,
                                (torque / (motor->DJI_Motor_Info.torque_constant *
                                           motor->DJI_Motor_Info.gear_ratio))); // 力矩常数（N/A）
        break;
    }
    case M3508:
    {
        return currentToInteger(-20.0f, 20.0f, -16384, 16384,
                                (torque / (motor->DJI_Motor_Info.torque_constant *
                                           motor->DJI_Motor_Info.gear_ratio))); // 力矩常数（N/A）
        break;
    }
    case M2006:
    {
        return currentToInteger(-10.0f, 10.0f, -10000, 10000,
                                (torque / (motor->DJI_Motor_Info.torque_constant *
                                           motor->DJI_Motor_Info.gear_ratio))); // 力矩常数（N/A）
        break;
    }
    default:
        return 0;
        break;
    }
}

void DJIMotorControl(void)
{
    // 检查dji_motor_list是否已初始化
    if (dji_motor_list == NULL)
    {
        return;
    }

    uint8_t     group, num;
    float       control_output;
    DJIMotor_t *motor;

    // 计算控制输出
    for (size_t i = 0; i < DJI_MOTOR_CNT; ++i)
    {
        if (dji_motor_list[i].can_device == NULL)
        {
            continue;
        }
        motor = &dji_motor_list[i];

        if (offline_module_get_device_status(motor->offline_index) == STATE_OFFLINE ||
            motor->stop_flag == MOTOR_STOP)
        {
            control_output = 0;
            if (motor->motor_settings.control_algorithm == CONTROL_PID)
            {
                // 当电机停止或离线时，将PID控制器输出设为0
                motor->motor_controller.speed_PID.Output = 0;
                motor->motor_controller.angle_PID.Output = 0;
            }
        }
        else
        {
            // 根据控制算法计算输出
            switch (motor->motor_settings.control_algorithm)
            {
            case CONTROL_PID:
                control_output = CalculatePIDOutput(motor);
                break;

            case CONTROL_LQR:
                control_output = CalculateLQROutput(motor);
                break;

            default:
                control_output = 0;
                break;
            }
        }

        group = motor->sender_group;
        num   = motor->message_num;
        if (group >= 10 || num >= 4)
        {
            continue;
        }

        // 填充发送数据
        sender_assignment[group].tx_buff[2 * num]     = (uint8_t)((int16_t)control_output >> 8);
        sender_assignment[group].tx_buff[2 * num + 1] = (uint8_t)((int16_t)control_output & 0x00ff);
    }
    // 发送CAN消息
    for (size_t i = 0; i < 10; ++i)
    {
        if (sender_enable_flag[i])
        {
            BSP_CAN_SendMessage(&sender_assignment[i], CAN_MODE_BLOCKING);
        }
    }
}
void DJIMotorListInit(DJIMotor_t *motor_list)
{
    dji_motor_list = motor_list;
}
