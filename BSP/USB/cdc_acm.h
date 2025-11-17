/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-10-18 13:15:38
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-10-18 17:15:04
 * @FilePath: \rm_base\modules\USB\cdc_acm.h
 * @Description:
 */
#ifndef CDC_ACM_H
#define CDC_ACM_H

#include "osal_def.h"
#include <stdint.h>

/* USB CDC事件定义 */
#define USB_CDC_RX_DONE_EVENT (0x01 << 0) // 接收完成事件
#define USB_CDC_TX_DONE_EVENT (0x01 << 1) // 发送完成事件

#define CDC_MAX_MPS           64

// USB CDC设备结构体
typedef struct
{
    uint8_t   busid;
    uintptr_t reg_base;

    // 接收相关
    uint8_t (*rx_buf)[2];             // 指向外部定义的双缓冲区
    uint16_t         rx_buf_size;     // 缓冲区大小
    volatile uint8_t rx_active_buf;   // 当前活动缓冲区
    uint16_t         real_rx_len;     // 实际接收数据长度
    uint16_t         expected_rx_len; // 预期长度（0为不定长）

    // USB CDC事件（用于接收和发送完成通知）
    osal_event_t usb_cdc_event;

    // 发送相关
    osal_sem_t ep_tx_sem;

    // 设备状态
    uint8_t is_initialized;
} USB_CDC_Device;

/**
 * @description: USB CDC初始化函数
 * @details      初始化USB CDC设备
 * @param        busid：总线ID
 * @param        reg_base：寄存器基地址
 * @param        expected_rx_len：预期接收长度（0为不定长）
 * @return       USB_CDC_Device*：初始化成功返回USB_CDC_Device指针，失败返回NULL
 */
USB_CDC_Device *cdc_acm_init(uint8_t busid, uintptr_t reg_base, uint16_t expected_rx_len);

/**
 * @description: USB CDC发送函数
 * @details      发送数据到USB CDC设备
 * @param        device：USB_CDC_Device指针
 * @param        buffer：要发送的数据指针
 * @param        size：数据长度
 * @return       void
 */
void cdc_acm_send(USB_CDC_Device *device, uint8_t *buffer, uint8_t size);

/**
 * @description: USB CDC接收函数
 * @details      从USB CDC设备接收数据
 * @param        device：USB_CDC_Device指针
 * @return       uint8_t*：指向接收数据的指针，失败返回NULL
 */
uint8_t *cdc_acm_read(USB_CDC_Device *device);

/**
 * @description: USB CDC反初始化函数
 * @details      释放USB CDC设备资源
 * @param        device：USB_CDC_Device指针
 * @return       void
 */
void cdc_acm_deinit(USB_CDC_Device *device);

#endif // CDC_ACM_H