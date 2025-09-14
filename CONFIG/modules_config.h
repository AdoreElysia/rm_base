/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-09-11 10:28:00
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-09-14 13:09:56
 * @FilePath: /rm_base/CONFIG/modules_config.h
 * @Description: 
 */
#ifndef _MODULES_CONFIG_H_
#define _MODULES_CONFIG_H_

/* 模块配置文件 */

/* IST8310 磁力计模块 */
#define IST8310_ENABLE                    0                                     // 启用IST8310模块        
/* IMU 模块 */
#define IMU_BMI088   1
#define IMU_ICM42688 2
#define  IMU_TYPE     IMU_BMI088                                                // 选择IMU模块
#if IMU_TYPE  == IMU_BMI088
   #define BMI088_TEMP_ENABLE             1                                     // 启用BMI088模块的温度控制
   #define BMI088_TEMP_SET                35.0f                                 // BMI088的设定温度
#elif IMU_TYPE  == IMU_ICM42688
#else  
#error "请选择一个IMU模块"
#endif
/* BEEP 模块 */
#define BEEP_ENBALE                       1                                     // 启用BEEP模块
/* OFFLINE 模块 */ 
#define MAX_OFFLINE_DEVICES               12                                    // 最大离线设备数量，这里根据需要自己修改
#define OFFLINE_MODULE_ENABLE             1                                     // 开启离线检测功能,注意下述功能在启用模块才有效 
#if OFFLINE_MODULE_ENABLE
   #define OFFLINE_WATCHDOG_ENABLE        1                                     // 启用离线检测看门狗功能
   #define OFFLINE_BEEP_ENABLE            1                                     // 开启离线蜂鸣器功能 
   #define OFFLINE_BEEP_PERIOD            2000                                  //注意这里的周期，由于在定时器(10ms)中,尽量保证整除
   #define OFFLINE_BEEP_ON_TIME           100                                   //这里BEEP_ON_TIME BEEP_OFF_TIME 共同影响
   #define OFFLINE_BEEP_OFF_TIME          100                                   //最大beep times（BEEP_PERIOD / （这里BEEP_ON_TIME + BEEP_OFF_TIME））
   #define OFFLINE_BEEP_TUNE_VALUE        500                                   //这两个部分决定beep的音调，音色
   #define OFFLINE_BEEP_CTRL_VALUE        100
#endif



#endif // _MODULES_CONFIG_H_