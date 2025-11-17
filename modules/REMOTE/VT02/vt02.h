/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-09-15 09:30:09
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-09-15 19:27:15
 * @FilePath: /rm_base/modules/REMOTE/VT02/vt02.h
 * @Description:
 */
#ifndef _VT02_H_
#define _VT02_H_

#include "bsp_uart.h"
#include "remote_data.h"
#include <stdint.h>

/* 帧头定义 */
typedef struct
{
    uint8_t  SOF;
    uint16_t DataLength;
    uint8_t  Seq;
    uint8_t  CRC8;
} VT02_Frame_Header;

typedef struct
{
    mouse_state_t    mouse_state;
    keyboard_state_t key_state;
} vt02_remote_data_t;

typedef struct
{
    VT02_Frame_Header  FrameHeader; // 接收到的帧头信息
    uint16_t           CmdID;
    vt02_remote_data_t vt02_remote_data;
    uint8_t            offline_index; // 离线索引
    UART_Device       *uart_device;   // UART实例
} VT02_Instance_t;

/**
 * @description: vt02图传初始化
 * @param {VT02_Instance_t} *vt02_instance
 * @return {osal_status_t}，OSAL_SCUCESS 初始化成功
 */
osal_status_t vt02_init(VT02_Instance_t *vt02_instance);
/**
 * @description: vt02图传解码
 * @param {VT02_Instance_t} *vt02_instance
 * @param {uint8_t} *buf
 * @return {void}
 */
void vt02_decode(VT02_Instance_t *vt02_instance, uint8_t *buf);

#endif // _VT02_H_