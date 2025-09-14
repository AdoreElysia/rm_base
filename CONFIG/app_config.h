/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-09-14 12:51:31
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-09-14 12:52:49
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

#endif // _APP_CONFIG_H_