/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-09-14 12:51:31
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-09-16 18:00:31
 * @FilePath: /rm_base/CONFIG/app_config.h
 * @Description: 
 */
#ifndef _APP_CONFIG_H_
#define _APP_CONFIG_H_

/* 线程配置文件 */
/* offline 线程配置 */
#define OFFLINE_THREAD_STACK_SIZE      1024                                  // 离线检测线程栈大小 
#define OFFLINE_THREAD_STACK_SECTION   __attribute__((section(".ccmram")))   // 线程栈内存区域 
#define OFFLINE_THREAD_PRIORITY        1                                     // 离线检测线程优先级 
/* ins 线程配置 */
#define INS_THREAD_STACK_SIZE          1024                                   // INS线程栈大小
#define INS_THREAD_STACK_SECTION       __attribute__((section(".ccmram")))    // 线程栈内存区域
#define INS_THREAD_PRIORITY            2                                      // INS线程优先级
/* remote 线程配置 */
#define REMOTE_THREAD_STACK_SIZE       1024                                   // remote线程栈大小
#define REMOTE_THREAD_STACK_SECTION    __attribute__((section(".ccmram")))    // 线程栈内存区域
#define REMOTE_THREAD_PRIORITY         10                                     // remote线程优先级
#define REMOTE_VT_THREAD_STACK_SIZE    1024                                   // vt图传线程栈大小
#define REMOTE_VT_THREAD_STACK_SECTION __attribute__((section(".ccmram")))    // 线程栈内存区域
#define REMOTE_VT_THREAD_PRIORITY      11                                     // vt图传线程优先级
/* dm_imu 线程配置 */
#define DM_IMU_THREAD_STACK_SIZE       1024                                   // dm_imu线程栈大小
#define DM_IMU_THREAD_STACK_SECTION   __attribute__((section(".ccmram")))     // 线程栈内存区域
#define DM_IMU_THREAD_PRIORITY         12                                     // dm_imu线程优先级

#endif // _APP_CONFIG_H_