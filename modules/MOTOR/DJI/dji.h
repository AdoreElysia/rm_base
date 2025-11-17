/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-09-18 11:05:25
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-10-03 10:02:19
 * @FilePath: \rm_base\modules\MOTOR\DJI\dji.h
 * @Description:
 */
#ifndef _DJI_H_
#define _DJI_H_

#include "motor_def.h"
#include <stdint.h>

/* DJI电机CAN反馈信息*/
typedef struct
{
    uint16_t last_ecd;           // 上一次读取的编码器值
    uint16_t ecd;                // 0-8191,刻度总共有8192格
    float    angle_single_round; // 单圈角度
    float    speed_rpm;          // 转速，单位为:转/分钟
    float    speed_rad;          // 角速度,单位为:弧度/秒
    int16_t  real_current;       // 实际电流
    uint8_t  temperature;        // 温度 Celsius

    float   total_angle; // 总角度,注意方向
    int32_t total_round; // 总圈数,注意方向
} DJI_Motor_Measure_s;

typedef struct
{
    DJI_Motor_Measure_s     measure;          // 电机测量值
    Motor_Control_Setting_s motor_settings;   // 电机设置
    Motor_Controller_s      motor_controller; // 电机控制器
    uint8_t                 sender_group;     // 分组发送组号
    uint8_t                 message_num;      // 分组发送消息数量
    Motor_Info_s            DJI_Motor_Info;   // 电机信息
    Motor_Working_Type_e    stop_flag;        // 启停标志
    uint8_t                 offline_index;    // 离线检测索引
    Can_Device             *can_device;       // CAN设备
} DJIMotor_t;

/**
 * @description: DJI电机初始化
 * @param {Motor_Init_Config_s} *config
 * @return {DJIMotor_t} *motor,返回电机指针
 */
DJIMotor_t *DJIMotorInit(Motor_Init_Config_s *config);
/**
 * @description: DJI电机设置参考值
 * @param {DJIMotor_t} *motor
 * @return {*}
 */
void DJIMotorSetRef(DJIMotor_t *motor, float ref);
/**
 * @description: DJI电机修改对应闭环的反馈数据源
 * @param {DJIMotor_t} *motor，电机指针
 * @param {Closeloop_Type_e} loop，对应的闭环类型
 * @param {Feedback_Source_e} type，修改的反馈数据源
 * @return {*}
 */
void DJIMotorChangeFeed(DJIMotor_t *motor, Closeloop_Type_e loop, Feedback_Source_e type);
/**
 * @description: DJI电机停止
 * @param {DJIMotor_t} *motor，电机指针
 * @return {*}
 */
void DJIMotorStop(DJIMotor_t *motor);
/**
 * @description: DJI电机使能
 * @param {DJIMotor_t} *motor，电机指针
 * @return {*}
 */
void DJIMotorEnable(DJIMotor_t *motor);
/**
 * @description: DJI电机外环修改
 * @param {DJIMotor_t} *motor，电机指针
 * @param {Closeloop_Type_e} outer_loop，外环类型
 * @param {LQR_Init_Config_s} *lqr_config，LQR参数,如果不是lqr算法直接传入NULL即可
 * @return {*}
 */
void DJIMotorOuterLoop(DJIMotor_t *motor, Closeloop_Type_e outer_loop,
                       LQR_Init_Config_s *lqr_config);
/**
 * @description: DJI电机控制,在电机线程中调用
 * @param {*}
 * @return {*}
 */
void DJIMotorControl(void);
/**
 * @description: DJI电机数据解析
 * @param {DJIMotor_t} *motor，电机指针
 * @return {*}
 */
void DecodeDJIMotor(DJIMotor_t *motor);
/**
 * @description: DJI电机列表初始化
 * @param {DJIMotor_t} *motor_list，电机列表指针
 * @return {*}
 */
void DJIMotorListInit(DJIMotor_t *motor_list);

#endif // _DJI_H_