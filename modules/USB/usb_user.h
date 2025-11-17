/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-10-18 13:15:38
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-10-19 10:39:15
 * @FilePath: \rm_base\modules\USB\usb_user.h
 * @Description:
 */
#ifndef _USB_USER_H_
#define _USB_USER_H_

#include "cdc_acm.h"
#include "osal_def.h"
#include <stdint.h>

#pragma pack(1)
struct User_Send_s
{
    uint8_t header;
    uint8_t mode;
    float   roll;
    float   pitch;
    float   yaw;
    uint8_t end;
};
struct User_Recv_s
{
    uint8_t header;
    uint8_t fire_advice;
    float   pitch;
    float   yaw;
    float   distance;
    uint8_t end;
};
#pragma pack()

typedef struct
{
    USB_CDC_Device *cdc_device;
    uint8_t         offline_index;
} user_cdc_t;

/**
 * @brief 初始化USB用户设备
 * @details 初始化USB CDC设备并注册到离线检测模块
 * @param[in] instance USB用户设备实例指针
 * @return osal_status_t OSAL_SUCCESS表示成功，OSAL_ERROR表示失败
 */
osal_status_t usb_user_init(user_cdc_t *instance);

/**
 * @brief 发送数据到USB设备
 * @details 通过USB CDC发送用户数据结构体
 * @param[in] send 指向要发送的User_Send_s结构体的指针
 * @return osal_status_t OSAL_SUCCESS表示成功，OSAL_ERROR表示失败
 */
osal_status_t usb_user_send(struct User_Send_s *send);

/**
 * @brief 从USB设备接收数据
 * @details 从USB CDC接收数据并验证数据帧格式
 * @return struct User_Recv_s* 接收到的数据指针，NULL表示接收失败或数据验证失败
 */
struct User_Recv_s *usb_user_recv();
#endif // _USB_USER_H_