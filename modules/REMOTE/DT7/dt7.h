/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-09-15 09:29:50
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-10-26 17:11:01
 * @FilePath: \rm_base\modules\REMOTE\DT7\dt7.h
 * @Description: 
 */
#ifndef _DT7_H_
#define _DT7_H_

#include "bsp_uart.h"
#include "remote_data.h"
#include <stdint.h>

#define DT7_CH_VALUE_MIN ((uint16_t)364)
#define DT7_CH_VALUE_OFFSET ((uint16_t)1024)
#define DT7_CH_VALUE_MAX ((uint16_t)1684)

#define DT7_SW_UP ((uint16_t)1)   // 开关向上时的值
#define DT7_SW_MID ((uint16_t)3)  // 开关中间时的值
#define DT7_SW_DOWN ((uint16_t)2) // 开关向下时的值

typedef struct {
    int16_t ch1;
    int16_t ch2;
    int16_t ch3;
    int16_t ch4;
    uint8_t sw1;
    uint8_t sw2;
    mouse_state_t mouse_state;
    keyboard_state_t keyboard_state;
    int16_t wheel;
} DT7_INPUT_t;

typedef struct
{
  DT7_INPUT_t dt7_input; 
  uint8_t offline_index; // 离线索引
  UART_Device *uart_device; // UART实例
}DT7_Instance_t;

/**
 * @description: dt7遥控器初始化
 * @param {DT7_Instance_t} *dt7_instance，实例指针
 * @return {osal_status_t}，OSAL_SSCUCCESS初始化成功
 */
osal_status_t dt7_init(DT7_Instance_t *dt7_instance);
/**
 * @description: dt7数据解码
 * @param {DT7_Instance_t} *dt7_instance，实例指针
 * @param {uint8_t} *buf，数据指针
 */
void dt7_decode(DT7_Instance_t *dt7_instance, uint8_t *buf);
/**
 * @description: 获取dt7通道数据
 * @param {DT7_Instance_t} *dt7_instance，实例指针
 * @param {uint8_t} channel_index，通道索引
 * @return {int16_t}，通道数据
 */
int16_t get_dt7_channel(DT7_Instance_t *dt7_instance, uint8_t channel_index);

#endif // _DT7_H_