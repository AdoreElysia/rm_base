/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-09-15 09:18:31
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-10-01 21:58:37
 * @FilePath: /rm_base/modules/REMOTE/remote.h
 * @Description: 
 */
#ifndef _REMOTE_H_
#define _REMOTE_H_


#include <stdint.h>
#include "dt7.h"
#include "modules_config.h"
#include "sbus.h"
#include "vt02.h"
#include "vt03.h"
#include "osal_def.h"

typedef struct {
#if defined (REMOTE_SOURCE) && REMOTE_SOURCE == 1
    SBUS_Instance_t sbus_instance;    
#elif defined (REMOTE_SOURCE) && REMOTE_SOURCE == 2
    DT7_Instance_t dt7_instance;
#endif
#if defined (REMOTE_VT_SOURCE) && REMOTE_VT_SOURCE == 1
    VT02_Instance_t vt02_instance; 
#elif defined (REMOTE_VT_SOURCE) && REMOTE_VT_SOURCE == 2
    VT03_Instance_t vt03_instance;
#endif
    uint8_t vt_offline_index;
    uint8_t remote_offline_index;
    uint8_t initflag;
} remote_instance_t;

/**
 * @brief 遥控器初始化
 * @param remote_instance 遥控器实例指针
 * @return osal_status_t 状态码
 */
osal_status_t remote_init(remote_instance_t *remote_instance);
/**
 * @brief 获取遥控器通道状态
 * @param channel_index 通道索引
 * @param is_vt_remote 是否为图传遥控器
 * @return uint8_t 通道状态
 */
uint8_t get_remote_channel_state(uint8_t channel_index, uint8_t is_vt_remote);
/**
 * @brief 获取鼠标状态
 * @param is_vt_remote 是否为图传遥控器
 * @return mouse_state_t* 鼠标状态指针
 */
mouse_state_t* get_remote_mouse_state(uint8_t is_vt_remote);
/**
 * @brief 获取键盘按键状态
 * @param is_vt_remote 是否为图传遥控器
 * @return keyboard_state_t* 键盘状态指针
 */
keyboard_state_t * get_remote_keyboard_state(uint8_t is_vt_remote);
/**
 * @brief 遥控器shell命令初始化
 * @param remote_instance 遥控器实例指针
 */
void remote_shell_cmd_init(remote_instance_t *remote_instance);

#endif // _REMOTE_H_