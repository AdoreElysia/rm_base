/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-10-03 10:23:54
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-10-03 12:45:06
 * @FilePath: \rm_base\modules\MOTOR\DAMIAO\damiao.h
 * @Description:
 */
#ifndef _DAMIAO_H_
#define _DAMIAO_H_

#include "motor_def.h"
#include <stdint.h>

#define DM_MIT_MODE 0x000
#define DM_POS_MODE 0x100
#define DM_SPD_MODE 0x200
#define DM_PSI_MODE 0x300

typedef enum
{
    DM_NO_ERROR               = 0X00,
    OVERVOLTAGE_ERROR         = 0X08,
    UNDERVOLTAGE_ERROR        = 0X09,
    OVERCURRENT_ERROR         = 0X0A,
    MOS_OVERTEMP_ERROR        = 0X0B,
    MOTOR_COIL_OVERTEMP_ERROR = 0X0C,
    COMMUNICATION_LOST_ERROR  = 0X0D,
    OVERLOAD_ERROR            = 0X0E,
} DMMotorError_t;

typedef enum
{
    DM_CMD_MOTOR_START   = 0xfc, // 使能,会响应指令
    DM_CMD_MOTOR_STOP    = 0xfd, // 停止
    DM_CMD_ZERO_POSITION = 0xfe, // 将当前的位置设置为编码器零位
    DM_CMD_CLEAR_ERROR   = 0xfb  // 清除电机错误
} DMMotor_Mode_e;

typedef struct
{
    uint8_t        id;
    uint8_t        state;
    float          velocity;
    float          last_position;
    float          position;
    float          torque;
    float          T_Mos;
    float          T_Rotor;
    float          total_angle; // 总角度,注意方向
    int32_t        total_round; // 总圈数,注意方向
    DMMotorError_t Error_Code;
} DM_Motor_Measure_s;

typedef struct
{
    DM_Motor_Measure_s      measure;           // 电机测量值
    Motor_Control_Setting_s motor_settings;    // 电机设置
    Motor_Controller_s      motor_controller;  // 电机控制器
    Motor_Info_s            dm_motor_info;     // 电机信息
    Motor_Working_Type_e    stop_flag;         // 启停标志
    uint8_t                 offline_index;     // 离线检测索引
    Can_Device             *can_device;        // CAN设备
    uint32_t                DMMotor_Mode_type; // 达妙电机控制模式
} DMMOTOR_t;

DMMOTOR_t *DMMotorInit(Motor_Init_Config_s *config, uint32_t DM_Mode_type);
void       DMMotorSetRef(DMMOTOR_t *motor, float ref);
void       DMMotorChangeFeed(DMMOTOR_t *motor, Closeloop_Type_e loop, Feedback_Source_e type);
void       DMMotorOuterLoop(DMMOTOR_t *motor, Closeloop_Type_e closeloop_type,
                            LQR_Init_Config_s *lqr_config);
void       DMMotorEnable(DMMOTOR_t *motor);
void       DMMotorStop(DMMOTOR_t *motor);
void       DMMotorcontrol(void);
void       DMMotorCmd(DMMOTOR_t *motor, DMMotor_Mode_e cmd);
void       DMMotorDecode(DMMOTOR_t *motor);
void       DMMotorListInit(DMMOTOR_t *motor_list);

#endif // _DAMIAO_H_