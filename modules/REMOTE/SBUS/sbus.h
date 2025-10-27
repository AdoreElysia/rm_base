/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-09-15 09:29:45
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-10-26 17:10:39
 * @FilePath: \rm_base\modules\REMOTE\SBUS\sbus.h
 * @Description: 
 */
#ifndef _SBUS_H_
#define _SBUS_H_

#include "bsp_uart.h"
#include "osal_def.h"
#include <stdint.h>

#define SBUS_CHX_BIAS ((uint16_t)1024)
#define SBUS_CHX_UP   ((uint16_t)240)
#define SBUS_CHX_DOWN ((uint16_t)1807)

typedef struct
{
    int16_t CH1;//1通道
    int16_t CH2;//2通道
    int16_t CH3;//3通道
    int16_t CH4;//4通道
    uint16_t CH5;//5通道
    uint16_t CH6;//6通道
    uint16_t CH7;//7通道
    uint16_t CH8;//8通道
    uint16_t CH9;//9通道
    uint16_t CH10;//10通道
    uint16_t CH11;//11通道
    uint16_t CH12;//12通道
    uint16_t CH13;//13通道
    uint16_t CH14;//14通道
    uint16_t CH15;//15通道
    uint16_t CH16;//16通道
    uint8_t ConnectState;   //连接的标志
}SBUS_CH_Struct;

typedef struct{
    SBUS_CH_Struct SBUS_CH;
    uint8_t offline_index; // 离线索引
    UART_Device *uart_device; // UART实例
}SBUS_Instance_t;

/**
 * @description: sbus初始化
 * @param {SBUS_Instance_t} *sbus_instance，sbus实例指针
 * @return {osal_status_t}，OSAL_SCUCCESS初始化成功,其余失败
 */
osal_status_t sbus_init(SBUS_Instance_t *sbus_instance);
/**
 * @description: sbus数据解码
 * @param {SBUS_Instance_t} *sbus_instance，sbus实例指针
 * @param {uint8_t} *buf，缓冲区指针
 * @return {*}
 */
void sbus_decode(SBUS_Instance_t *sbus_instance,uint8_t *buf);
/**
 * @description: 获取sbus通道状态
 * @param {SBUS_Instance_t} *sbus_instance，sbus实例指针
 * @param {uint8_t} channel_index，通道索引
 * @return {int16_t}，通道数据
 */
int16_t get_sbus_channel(SBUS_Instance_t *sbus_instance, uint8_t channel_index);

#endif // _SBUS_H_