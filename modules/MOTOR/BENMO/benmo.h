#ifndef _BENMO_H_
#define _BENMO_H_

#include "motor_def.h"
#include <stdint.h>

#define BM_CUR_MODE         0
#define BM_VOL_MODE		  	1
#define BM_POS_MODE			2
#define BM_SPD_MODE			3
#define BM_OPEN_MODE		4

typedef enum 
{
    BM_NO_ERROR = 0x00,              // 无故障
    UNDERVOLTAGE_1_ERROR = 0x01,     // 欠压 1（18V < 母线电压 < 20V）
    UNDERVOLTAGE_2_ERROR = 0x02,     // 欠压 2（母线电压 < 18V）
    BM_OVERVOLTAGE_ERROR = 0x03,        // 过压（母线电压 > 36V）
    BM_OVERCURRENT_ERROR = 0x0A,        // 过流（默认：母线电流 > 35A）
    MOTOR_COIL_OVERTEMP_1_ERROR = 0x20,  // 过温 1（电机绕组温度 > 80℃）
    MOTOR_COIL_OVERTEMP_2_ERROR = 0x1F,  // 过温 2（电机绕组温度 > 110℃）
    SAMPLE_RESISTOR_ERROR = 0x29,    // 采样电阻故障
    POSITION_SENSOR_SELF_ERROR = 0x2A,   // 位置传感器自身故障
    POSITION_SENSOR_INTERFERENCE_ERROR = 0x2B,  // 位置传感器信号被干扰
    TEMP_SENSOR_OUT_OF_RANGE_ERROR = 0x2D,      // 温度传感器超出量程
    COMMUNICATION_TIMEOUT_ERROR = 0x3C,         // 通信超时(默认无保护，需用户自行开启)
    STALL_ERROR = 0x62               // 堵转(默认:电流 > 5A 并且 转速为 0)
} BMMotorError_t;

/* 本末电机CAN反馈信息 */
typedef struct
{
    float speed_rpm;            // 转速，单位为:转/分钟
    float iq;                   // IQ电流
    uint16_t last_absolute_position; // 上次绝对位置
    uint16_t absolute_position; // 绝对位置
    float total_round;          // 总圈数,注意方向
    float angle_single_angle;   // 单圈角度
    float voltage;              // 母线电压，分辨率0.1V
    float speed_rad;            // 角速度,单位为:弧度/秒
    float total_angle;          // 总角度
    // 注意，下述为特定电机返回，并不支持所有电机（默认报文下），如有需要，请使用主动查询
    BMMotorError_t Error_Code;  // 电机错误码
    uint8_t mode;              // 电机模式
} Benmo_Motor_Measure_s;

typedef struct
{
    Benmo_Motor_Measure_s measure;          // 电机测量值
    Motor_Control_Setting_s motor_settings; // 电机设置
    Motor_Controller_s motor_controller;    // 电机控制器
    uint8_t sender_group;                   // 分组发送组号
    uint8_t message_num;                    // 分组发送消息数量
    uint8_t mode_type;                      // 电机模式类型
    Motor_Info_s benmo_motor_info;          // 电机信息
    Motor_Working_Type_e stop_flag;         // 启停标志
    uint8_t offline_index;                  // 离线检测索引
    Can_Device *can_device;                 // CAN设备
} BenmoMotor_t;

/**
 * @description: 本末电机初始化
 * @param {Motor_Init_Config_s} *config
 * @return {BenmoMotor_t} *motor,返回电机指针
 */
BenmoMotor_t *BenmoMotorInit(Motor_Init_Config_s *config,uint8_t mode_type);

/**
 * @description: 本末电机设置参考值
 * @param {BenmoMotor_t} *motor
 * @return {*}
 */
void BenmoMotorSetRef(BenmoMotor_t *motor, float ref);

/**
 * @description: 本末电机修改对应闭环的反馈数据源
 * @param {BenmoMotor_t} *motor，电机指针
 * @param {Closeloop_Type_e} loop，对应的闭环类型
 * @param {Feedback_Source_e} type，修改的反馈数据源
 * @return {*}
 */
void BenmoMotorChangeFeed(BenmoMotor_t *motor, Closeloop_Type_e loop, Feedback_Source_e type);

/**
 * @description: 本末电机停止
 * @param {BenmoMotor_t} *motor，电机指针
 * @return {*}
 */
void BenmoMotorStop(BenmoMotor_t *motor);

/**
 * @description: 本末电机使能
 * @param {BenmoMotor_t} *motor，电机指针
 * @return {*}
 */
void BenmoMotorEnable(BenmoMotor_t *motor);

/**
 * @description: 本末电机外环修改
 * @param {BenmoMotor_t} *motor，电机指针
 * @param {Closeloop_Type_e} outer_loop，外环类型
 * @param {LQR_Init_Config_s} *lqr_config，LQR参数,如果不是lqr算法直接传入NULL即可
 * @return {*}
 */
void BenmoMotorOuterLoop(BenmoMotor_t *motor, Closeloop_Type_e outer_loop, LQR_Init_Config_s *lqr_config);

/**
 * @description: 本末电机控制,在电机线程中调用
 * @param {*}
 * @return {*}
 */
void BenmoMotorControl(void);

/**
 * @description: 本末电机数据解析
 * @param {BenmoMotor_t} *motor，电机指针
 * @return {*}
 */
void DecodeBenmoMotor(BenmoMotor_t *motor);

/**
 * @description: 本末电机列表初始化
 * @param {BenmoMotor_t} *motor_list，电机列表指针
 * @return {*}
 */
void BenmoMotorListInit(BenmoMotor_t *motor_list);

#endif // _BENMO_H_