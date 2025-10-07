#include "benmo.h"
#include "bsp_can.h"
#include "bsp_dwt.h"
#include "can.h"
#include "motor_def.h"
#include "offline.h"
#include "user_lib.h"
#include <stdint.h>
#include <string.h>

#define log_tag              "benmo"
#include "shell_log.h"


/*电机型号,开环模式,电流模式,电压模式,速度模式,位置模式 */
static const int16_t bm_mode_limits[][11] = {
    // 电机型号,       开环模式最大值,开环模式最小值,电流模式最大值,电流模式最小值,电压模式最大值,电压模式最小值,速度模式最大值,速度模式最小值,位置模式最大值,位置模式最小值
    // 注意，对应模式为0的表示电机不支持该模式
{P1010B_BENMO,0,0,7500,-7500,4800, -4800,16000,-16000,-5000,5000}, // P1010B_BENMO
{M1505A_BENMO,16383,-16383,16383,-16383,0,0,5000,-5000,0,0      }  // M1505A_BENMO
};                   

#define LIMIT_MIN_MAX(x, min, max) (x) = (((x) <= (min)) ? (min) : (((x) >= (max)) ? (max) : (x)))

#define ECD_ANGLE_COEF_DENMO 0.010986f // (360/32768),将编码器值转化为角度制

static BenmoMotor_t *benmo_motor_list = NULL; 

static CanTxMessage_t sender_assignment[10] = {
    [0] = {.can_handle = &hcan1, .txconf.StdId =0x32, .txconf.IDE = CAN_ID_STD ,.txconf.RTR = CAN_RTR_DATA , .txconf.DLC =8 ,.tx_buff = {0},},
    [1] = {.can_handle = &hcan1, .txconf.StdId =0x33, .txconf.IDE = CAN_ID_STD ,.txconf.RTR = CAN_RTR_DATA , .txconf.DLC =8 ,.tx_buff = {0},},
    [2] = {.can_handle = &hcan2, .txconf.StdId =0x32, .txconf.IDE = CAN_ID_STD ,.txconf.RTR = CAN_RTR_DATA , .txconf.DLC =8 ,.tx_buff = {0},},
    [3] = {.can_handle = &hcan2, .txconf.StdId =0x33, .txconf.IDE = CAN_ID_STD ,.txconf.RTR = CAN_RTR_DATA , .txconf.DLC =8 ,.tx_buff = {0},},
};
static uint8_t sender_enable_flag[4] = {0};


static void MotorSenderGrouping(BenmoMotor_t *motor, Can_Device_Init_Config_s *config)
{
    uint8_t motor_id = config->tx_id; // 本末电机ID从1开始
    uint8_t motor_send_num;
    uint8_t motor_grouping;

    switch (motor->benmo_motor_info.motor_type) {
        case P1010B_BENMO:
        {
            if (motor_id >= 1 && motor_id <= 4) // ID 1-4分组
            {
                motor_send_num = motor_id - 1; // 数组下标从0开始
                motor_grouping = config->can_handle == &hcan1 ? 0 : 2; // ID 1-4使用0x32命令
            }
            else if (motor_id >= 5 && motor_id <= 8) // ID 5-8分组
            {
                motor_send_num = motor_id - 5; // 数组下标从0开始
                motor_grouping = config->can_handle == &hcan1 ? 1 : 3; // ID 5-8使用0x33命令
            }
            else
            {
                // ID超出范围
                LOG_ERROR("Invalid motor ID: %d. Should be 1-8.", motor_id);
                return;
            }

            // 设置接收ID (0x50 + 电机ID)
            config->rx_id = 0x50 + motor_id;
            sender_enable_flag[motor_grouping] = 1; // 设置发送标志位,防止发送空帧
            motor->message_num = motor_send_num;
            motor->sender_group = motor_grouping;

            // 检查ID冲突
            for (size_t i = 0; i < BENMO_MOTOR_CNT; ++i)
            {
                if (benmo_motor_list && benmo_motor_list[i].can_device && 
                    benmo_motor_list[i].can_device->can_handle == config->can_handle && 
                    benmo_motor_list[i].can_device->rx_id == config->rx_id)
                {
                    LOG_ERROR("ID crash, id [%d], can_bus [%s]", config->rx_id, 
                            (config->can_handle == &hcan1 ? "can1" : "can2"));
                }
            }
            break;
        }
        case M1505A_BENMO:
        {
            if (motor_id >= 1 && motor_id <= 4) // ID 1-4分组
            {
                motor_send_num = motor_id - 1; // 数组下标从0开始
                motor_grouping = config->can_handle == &hcan1 ? 0 : 2; // ID 1-4使用0x32命令
            }
            else if (motor_id >= 5 && motor_id <= 8) // ID 5-8分组
            {
                motor_send_num = motor_id - 5; // 数组下标从0开始
                motor_grouping = config->can_handle == &hcan1 ? 1 : 3; // ID 5-8使用0x33命令
            }
            else
            {
                // ID超出范围
                LOG_ERROR("Invalid motor ID: %d. Should be 1-8.", motor_id);
                return;
            }

            // 设置接收ID (0x96 + 电机ID)
            config->rx_id = 0x96 + motor_id;
            sender_enable_flag[motor_grouping] = 1; // 设置发送标志位,防止发送空帧
            motor->message_num = motor_send_num;
            motor->sender_group = motor_grouping;

            // 检查ID冲突
            for (size_t i = 0; i < BENMO_MOTOR_CNT; ++i)
            {
                if (benmo_motor_list && benmo_motor_list[i].can_device && 
                    benmo_motor_list[i].can_device->can_handle == config->can_handle && 
                    benmo_motor_list[i].can_device->rx_id == config->rx_id)
                {
                    LOG_ERROR("ID crash, id [%d], can_bus [%s]", config->rx_id, 
                            (config->can_handle == &hcan1 ? "can1" : "can2"));
                }
            }
            break;
        }
        default:
        {
            LOG_ERROR("Invalid motor type: %d", motor->benmo_motor_info.motor_type);
            return;
        }
    }
}

void DecodeBenmoMotor(BenmoMotor_t *motor)
{
    if (motor == NULL || motor->can_device == NULL) {return;}
    
    // 保存上次位置，用于计算多圈角度
    motor->measure.last_absolute_position = motor->measure.absolute_position;

    if (motor->benmo_motor_info.motor_type == P1010B_BENMO)
    {
        // --- P1010B 解析逻辑 ---
        // 文档说明：速度分辨率0.1, IQ分辨率0.01, 电压分辨率0.1
        motor->measure.speed_rpm = ((int16_t)((motor->can_device->rx_buff[0] << 8) | motor->can_device->rx_buff[1])) / 10.0f;
        motor->measure.iq = ((int16_t)((motor->can_device->rx_buff[2] << 8) | motor->can_device->rx_buff[3])) / 100.0f;
        motor->measure.absolute_position = (uint16_t)((motor->can_device->rx_buff[4] << 8) | motor->can_device->rx_buff[5]);
        motor->measure.voltage = ((uint16_t)((motor->can_device->rx_buff[6] << 8) | motor->can_device->rx_buff[7])) / 10.0f;
    }
    else if (motor->benmo_motor_info.motor_type == M1505A_BENMO)
    {
        // --- M1505A 解析逻辑 ---
        // 文档说明：速度分辨率0.1, IQ电流用归一化公式
        motor->measure.speed_rpm = ((int16_t)((motor->can_device->rx_buff[0] << 8) | motor->can_device->rx_buff[1])) / 10.0f;
        motor->measure.iq = ((int16_t)((motor->can_device->rx_buff[2] << 8) | motor->can_device->rx_buff[3])) * 55.0f / 32767.0f;
        motor->measure.absolute_position = (uint16_t)((motor->can_device->rx_buff[4] << 8) | motor->can_device->rx_buff[5]);
        motor->measure.Error_Code = motor->can_device->rx_buff[6]; // 故障值
        motor->measure.mode = motor->can_device->rx_buff[7];       // 当前模式
    }

    motor->measure.angle_single_angle = ECD_ANGLE_COEF_DENMO * (float)motor->measure.absolute_position; // 单圈角度
    motor->measure.speed_rad = RPM_2_RAD_PER_SEC * motor->measure.speed_rpm;
    
    // 多圈角度计算
    int16_t delta_ecd = motor->measure.absolute_position - motor->measure.last_absolute_position;
    if (delta_ecd > 16384) {motor->measure.total_round--;} 
    else if (delta_ecd < -16384) {motor->measure.total_round++;}
    motor->measure.total_angle = motor->measure.total_round * 360.0f + motor->measure.angle_single_angle;
    
    // 更新在线状态
    offline_module_device_update(motor->offline_index);
}

BenmoMotor_t *BenmoMotorInit(Motor_Init_Config_s *config,uint8_t mode_type)
{
    // 检查benmo_motor_list是否已初始化
    if (benmo_motor_list == NULL) {
        LOG_ERROR("Benmo motor list not initialized\n");
        return NULL;
    }

    // 查找一个空位来初始化电机
    BenmoMotor_t *benmoMotor = NULL;
    for (int i = 0; i < BENMO_MOTOR_CNT; i++) {
        if (benmo_motor_list[i].can_device == NULL) {
            benmoMotor = &benmo_motor_list[i];
            memset(benmoMotor, 0, sizeof(BenmoMotor_t));
            break;
        }
    }
    
    // 检查是否找到空位
    if (benmoMotor == NULL) {
        LOG_ERROR("Benmo motor count exceeds maximum limit\n");
        return NULL;
    }

    // motor basic setting 电机基本设置
    benmoMotor->benmo_motor_info = config->Motor_init_Info;                 // 电机信息
    benmoMotor->motor_settings = config->controller_setting_init_config;    // 正反转,闭环类型等
    benmoMotor->mode_type = mode_type;                                      // 电机模式类型

    // 电机分组,因为至多4个电机可以共用一帧CAN控制报文
    MotorSenderGrouping(benmoMotor, &config->can_init_config); //更新rx_id
    // CAN 设备初始化配置
    Can_Device_Init_Config_s can_config = {
        .can_handle = config->can_init_config.can_handle,
        .tx_id = config->can_init_config.tx_id,
        .rx_id = config->can_init_config.rx_id,
        .tx_mode = CAN_MODE_BLOCKING,
        .rx_mode = CAN_MODE_IT,
    };
    // 注册 CAN 设备并获取引用
    benmoMotor->can_device = BSP_CAN_Device_Init(&can_config);
    if (benmoMotor->can_device == NULL) {
        LOG_ERROR("Failed to initialize CAN device for Benmo motor");
        return NULL;
    }

    benmoMotor->motor_settings.control_algorithm = config->controller_setting_init_config.control_algorithm;
    switch (config->controller_setting_init_config.control_algorithm) {
        case CONTROL_PID:
            // motor controller init 电机控制器初始化
            PIDInit(&benmoMotor->motor_controller.speed_PID, &config->controller_param_init_config.speed_PID);
            PIDInit(&benmoMotor->motor_controller.angle_PID, &config->controller_param_init_config.angle_PID);
            benmoMotor->motor_controller.other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
            benmoMotor->motor_controller.other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;
            break;
        case CONTROL_LQR:
            LQRInit(&benmoMotor->motor_controller.lqr, &config->controller_param_init_config.lqr_config);
            benmoMotor->motor_controller.other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
            benmoMotor->motor_controller.other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;
            break;
        case CONTROL_OTHER:
            // 未来添加其他控制算法的初始化
            break;
    }

    //掉线检测
    benmoMotor->offline_index = offline_module_device_register(&config->offline_device_motor);

    // 根据电机类型发送不同的使能指令
    if (benmoMotor->benmo_motor_info.motor_type == P1010B_BENMO)
    {
        // 发送使能指令 (CMD=2)
        CanTxMessage_t enable_cmd = {
            .can_handle = benmoMotor->can_device->can_handle,
            .txconf.StdId = 0x38,
            .txconf.IDE = CAN_ID_STD,
            .txconf.RTR = CAN_RTR_DATA,
            .txconf.DLC = 8,
            .tx_buff = {0}
        };
        // 将对应电机ID的位置设置为2 (使能)
        enable_cmd.tx_buff[benmoMotor->can_device->tx_id - 1] = 2;
        BSP_CAN_SendMessage(&enable_cmd, CAN_MODE_BLOCKING);

    }
    else if (benmoMotor->benmo_motor_info.motor_type == M1505A_BENMO)
    {
        // 发送使能模式 (0x0A)
        CanTxMessage_t enable_cmd = {
            .can_handle = benmoMotor->can_device->can_handle,
            .txconf.StdId = 0x105,
            .txconf.IDE = CAN_ID_STD,
            .txconf.RTR = CAN_RTR_DATA,
            .txconf.DLC = 8,
            .tx_buff = {0}
        };
        // 将对应电机ID的位置设置为0x0A (使能)
        enable_cmd.tx_buff[benmoMotor->can_device->tx_id - 1] = 0x0A;
        BSP_CAN_SendMessage(&enable_cmd, CAN_MODE_BLOCKING);
    }
    
    LOG_INFO("Benmo motor initialized on CAN bus [%s] with TX ID: 0x%X, RX ID: 0x%X\n",
             (can_config.can_handle == &hcan1 ? "CAN1" : "CAN2"), can_config.tx_id, can_config.rx_id);
    return benmoMotor;
}

void BenmoMotorChangeFeed(BenmoMotor_t *motor, Closeloop_Type_e loop, Feedback_Source_e type)
{
    if (loop == ANGLE_LOOP)
        motor->motor_settings.angle_feedback_source = type;
    else if (loop == SPEED_LOOP)
        motor->motor_settings.speed_feedback_source = type;
    else
        LOG_ERROR("loop type error, check and func param\n");
}

void BenmoMotorStop(BenmoMotor_t *motor)
{
    motor->stop_flag = MOTOR_STOP;
}

void BenmoMotorEnable(BenmoMotor_t *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}

/* 修改电机的实际闭环对象 */
void BenmoMotorOuterLoop(BenmoMotor_t *motor, Closeloop_Type_e outer_loop, LQR_Init_Config_s *lqr_config)
{
    // 更新外环类型
    motor->motor_settings.outer_loop_type = outer_loop;
    
    // 如果是LQR控制且提供了配置参数，则重新初始化，其他算法传递NULL即可
    if (motor->motor_settings.control_algorithm == CONTROL_LQR && lqr_config != NULL) {
        LQRInit(&motor->motor_controller.lqr, lqr_config);
    }
}

// 设置参考值
void BenmoMotorSetRef(BenmoMotor_t *motor, float ref)
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

static float CalculatePIDOutput(BenmoMotor_t *motor)
{
    float pid_measure, pid_ref;
    
    pid_ref = motor->motor_controller.ref;
    if (motor->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_REVERSE) {pid_ref *= -1;}

    // pid_ref会顺次通过被启用的闭环充当数据的载体
    // 计算位置环,只有启用位置环且外层闭环为位置时会计算速度环输出
    if ((motor->motor_settings.close_loop_type & ANGLE_LOOP) && motor->motor_settings.outer_loop_type == ANGLE_LOOP)
    {
        if (motor->motor_settings.angle_feedback_source == OTHER_FEED)
            pid_measure = *motor->motor_controller.other_angle_feedback_ptr;
        else
            pid_measure = motor->measure.total_angle; 

        if (motor->motor_settings.feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE) pid_measure *= -1;
        // 更新pid_ref进入下一个环
        pid_ref = PIDCalculate(&motor->motor_controller.angle_PID, pid_measure, pid_ref);
    }
    // 计算速度环,(外层闭环为速度或位置)且(启用速度环)时会计算速度环
    if ((motor->motor_settings.close_loop_type & SPEED_LOOP) && (motor->motor_settings.outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)))
    {
        if (motor->motor_settings.speed_feedback_source == OTHER_FEED)
            pid_measure = *motor->motor_controller.other_speed_feedback_ptr;
        else // MOTOR_FEED
            pid_measure = motor->measure.speed_rpm;

        if (motor->motor_settings.feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE) pid_measure *= -1;
        // 更新pid_ref
        pid_ref = PIDCalculate(&motor->motor_controller.speed_PID, pid_measure, pid_ref);
    }

    return pid_ref;
}

static float CalculateLQROutput(BenmoMotor_t *motor)
{
    float degree=0.0f,angular_velocity=0.0f,lqr_ref=0.0f;
    
    lqr_ref = motor->motor_controller.ref;
    if(motor->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_REVERSE){lqr_ref *= -1;}

    // 位置状态计算
    if ((motor->motor_settings.close_loop_type & ANGLE_LOOP) && motor->motor_settings.outer_loop_type == ANGLE_LOOP)
    {
        degree = (motor->motor_settings.angle_feedback_source == OTHER_FEED) ?
                *motor->motor_controller.other_angle_feedback_ptr : motor->measure.total_angle;

        if (motor->motor_settings.feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE) degree *= -1;
    }

    // 速度状态计算
    if ((motor->motor_settings.close_loop_type & SPEED_LOOP) && (motor->motor_settings.outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)))
    {
        angular_velocity = (motor->motor_settings.speed_feedback_source == OTHER_FEED) ?
                *motor->motor_controller.other_speed_feedback_ptr : motor->measure.speed_rad;

        if (motor->motor_settings.feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE) angular_velocity *= -1;
    }

    float torque = LQRCalculate(&motor->motor_controller.lqr, degree, angular_velocity, lqr_ref);

    switch (motor->benmo_motor_info.motor_type) 
    {                                          
        case P1010B_BENMO:
            {
                return (torque * 1.414f) / 1.20f ; // 转矩常数 1.2Nm/A,
                break;
            }
        case M1505A_BENMO:
            {
                return torque /0.83f ; // 转矩常数 0.83Nm/A,
                break;                                
            }
        default:
            return 0;
            break;
    }
}

void BenmoMotorControl(void)
{
    if (benmo_motor_list == NULL) {return;}

    float control_output = 0.0f; 
    BenmoMotor_t *motor;
    
    // 1. 遍历所有电机，计算控制输出
    for (size_t i = 0; i < BENMO_MOTOR_CNT; ++i) {
        motor = &benmo_motor_list[i];
        if (motor->can_device == NULL) continue;
        
        // 如果电机停止或离线，输出为0
        if (offline_module_get_device_status(motor->offline_index) == STATE_OFFLINE || motor->stop_flag == MOTOR_STOP) {
            control_output = 0;
            if (motor->motor_settings.control_algorithm == CONTROL_PID) {
                motor->motor_controller.speed_PID.Output = 0;
                motor->motor_controller.angle_PID.Output = 0;
            }
        } else { 
            // 根据模式计算物理输出值
            switch (motor->mode_type) {
                case BM_CUR_MODE: 
                case BM_VOL_MODE: 
                case BM_OPEN_MODE: 
                    switch (motor->motor_settings.control_algorithm) {
                        case CONTROL_PID: control_output = CalculatePIDOutput(motor); break;
                        case CONTROL_LQR: control_output = CalculateLQROutput(motor); break;
                        default: control_output = 0; break;
                    }
                    break;
                case BM_POS_MODE: 
                case BM_SPD_MODE: 
                    control_output = motor->motor_controller.ref;
                    break;
                default:
                    control_output = 0;
                    break;
            }
        }

        // 2. 根据电机类型和模式进行限幅
        int16_t mode_limits_index = -1;
        if (motor->benmo_motor_info.motor_type == P1010B_BENMO) mode_limits_index = 0;
        else if (motor->benmo_motor_info.motor_type == M1505A_BENMO) mode_limits_index = 1;

        if (mode_limits_index != -1) {
            float min_val, max_val;
            switch (motor->mode_type) {
                case BM_OPEN_MODE:   min_val = bm_mode_limits[mode_limits_index][1]; max_val = bm_mode_limits[mode_limits_index][2]; break;
                case BM_CUR_MODE:    min_val = bm_mode_limits[mode_limits_index][3]; max_val = bm_mode_limits[mode_limits_index][4]; break;
                case BM_VOL_MODE:    min_val = bm_mode_limits[mode_limits_index][5]; max_val = bm_mode_limits[mode_limits_index][6]; break;
                case BM_SPD_MODE:    min_val = bm_mode_limits[mode_limits_index][7]; max_val = bm_mode_limits[mode_limits_index][8]; break;
                case BM_POS_MODE:    min_val = bm_mode_limits[mode_limits_index][9]; max_val = bm_mode_limits[mode_limits_index][10]; break;
                default:             min_val = 0; max_val = 0; break;
            }
            LIMIT_MIN_MAX(control_output, min_val, max_val);
        }

        // 3. 将物理输出值转换为协议规定的整数值
        int16_t send_value = 0;
        if (motor->benmo_motor_info.motor_type == P1010B_BENMO) {
            switch (motor->mode_type) {
                case BM_CUR_MODE:    send_value = (int16_t)(control_output * 100.0f); break; // 电流: A*100
                case BM_VOL_MODE:    send_value = (int16_t)(control_output * 100.0f); break; // 电压: V*100
                case BM_SPD_MODE:    send_value = (int16_t)(control_output * 10.0f);  break; // 速度: RPM*10
                case BM_POS_MODE:    send_value = (int16_t)(control_output * 100.0f); break; // 位置: 圈*100
                default:             send_value = (int16_t)control_output; break;
            }
        } else if (motor->benmo_motor_info.motor_type == M1505A_BENMO) {
            switch (motor->mode_type) {
                case BM_CUR_MODE:    send_value = (int16_t)((control_output / 55.0f) * 32767.0f); break; // 电流归一化
                case BM_VOL_MODE:    send_value = (int16_t)control_output; break; // 电压直接发送 (假设control_output已经是协议值)
                case BM_SPD_MODE:    send_value = (int16_t)(control_output * 10.0f);  break; // 速度: RPM*10
                default:             send_value = (int16_t)control_output; break;
            }
        }

        // 4. 填充发送数据
        uint8_t group = motor->sender_group;
        uint8_t num = motor->message_num;
        if (group < 4 && num < 4) {
            sender_assignment[group].tx_buff[2 * num] = (uint8_t)(send_value >> 8);
            sender_assignment[group].tx_buff[2 * num + 1] = (uint8_t)(send_value & 0xFF);
        }
    }
    
    // 5. 发送CAN消息
    for (size_t i = 0; i < 4; ++i) { 
        if (sender_enable_flag[i]) {
            BSP_CAN_SendMessage(&sender_assignment[i], CAN_MODE_BLOCKING);
        }
    }
}

void BenmoMotorListInit(BenmoMotor_t *motor_list)
{
    benmo_motor_list = motor_list;
}