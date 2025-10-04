/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-09-11 10:28:00
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-10-03 18:30:52
 * @FilePath: \rm_base\CONFIG\modules_config.h
 * @Description: 
 */
#ifndef _MODULES_CONFIG_H_
#define _MODULES_CONFIG_H_
#include "robot_config.h"

/* 模块配置文件 */     
/* IST8310 模块 */
#define MAG_SEN                           0.3f                                  // 原始整型数据变成 单位ut
#define IST8310_IIC_ADDRESS               0x0E                                  // IST8310的从设备IIC地址                                                           
/* IMU 模块 */
#define  IMU_TYPE                         1                                     // 选择IMU模块，1为BMI088，2为ICM42688，3为MPU6500
#if IMU_TYPE    == 1 
   #define BMI088_TEMP_ENABLE             1                                     // 启用BMI088模块的温度控制
   #define BMI088_TEMP_SET                35.0f                                 // BMI088的设定温度
#elif IMU_TYPE  == 2
   #define ICM42688P_TEMP_ENABLE          1                                     // 启用ICM42688P模块的温度控制
   #define ICM42688P_TEMP_SET             35.0f                                 // ICM42688P的设定温度
#elif IMU_TYPE  == 3
   #define MPU6500_TEMP_ENABLE            1                                     // 启用MPU6500模块的温度控制
   #define MPU6500_TEMP_SET               35.0f                                 // MPU6500的设定温度
#endif
/* OFFLINE 模块 */ 
#define MAX_OFFLINE_DEVICES               12                                    // 最大离线设备数量，这里根据需要自己修改
#define OFFLINE_WATCHDOG_ENABLE           1                                     // 启用离线检测看门狗功能
#define OFFLINE_BEEP_ENABLE               0                                     // 开启离线蜂鸣器功能 
#define OFFLINE_BEEP_PERIOD               2000                                  //注意这里的周期，由于在定时器(10ms)中,尽量保证整除
#define OFFLINE_BEEP_ON_TIME              100                                   //这里BEEP_ON_TIME BEEP_OFF_TIME 共同影响
#define OFFLINE_BEEP_OFF_TIME             100                                   //最大beep times（BEEP_PERIOD / （这里BEEP_ON_TIME + BEEP_OFF_TIME））
#define OFFLINE_BEEP_TUNE_VALUE           500                                   //这两个部分决定beep的音调，音色
#define OFFLINE_BEEP_CTRL_VALUE           100
/* REMOTE 模块 */
//在这里决定控制来源 注意键鼠控制部分 图传优先遥控器 uart部分c板只有 huart1/huart3/huart6
#define REMOTE_SOURCE                     1                                     //遥控器选择 none 0 sbus 1 dt7 2
#define REMOTE_UART                       huart3                                //遥控器串口   
#define REMOTE_VT_SOURCE                  2                                     //图传选择   none 0 vt02 1 vt03 2  
#define REMOTE_VT_UART                    huart6                                //图传串口
#define REMOTE_UART_RX_BUF_SIZE           30                                    // 定义遥控器接收缓冲区大小
#define REMOTE_DEAD_ZONE                  10                                    // 定义遥控器死区范围
/* DM_IMU 模块 */
#define DM_IMU_RX_ID                      0x11                                  // 达妙IMU的接收
#define DM_IMU_TX_ID                      0x6FF                                 // 达妙IMU的发送
#define DM_IMU_CAN_BUS                    hcan2                                 // 达妙IMU使用的CAN总线
/* 电机模块 */
/* DJI电机模块 */
#define DJI_MOTOR_CNT                     8                                     //DJI电机数量
/* 达妙电机模块 */
#define DM_MOTOR_CNT	                     4                                     //达妙电机数量

#endif // _MODULES_CONFIG_H_