/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-10-03 12:47:21
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-10-26 11:54:13
 * @FilePath: \rm_base\applications\motor_task\motor_task.h
 * @Description:
 */
#ifndef _MOTOR_TASK_H_
#define _MOTOR_TASK_H_

#include <stdint.h>

void motor_list_init(void);
void motor_task_init(void);
/**
 * @brief 获取指定类型和索引的电机指针
 * @param motor_type 电机类型 (MOTOR_DJI, MOTOR_DM, MOTOR_BENMO)
 * @param idx 电机索引
 * @return void* 对应的电机指针，如果不存在则返回NULL
 */
void *get_motor_ptr(uint8_t motor_type, uint8_t idx);

#endif // _MOTOR_TASK_H_