/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-09-15 09:29:38
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-09-26 23:05:29
 * @FilePath: /rm_base/modules/REMOTE/SBUS/sbus.c
 * @Description: 
 */
#include "sbus.h"
#include "bsp_uart.h"
#include "modules_config.h"
#include "offline.h"
#include "osal_def.h"
#include <stdint.h>
#include "remote_data.h"

#define log_tag "sbus"
#include "shell_log.h"

#define SBUS_CHX_BIAS ((uint16_t)1024)
#define SBUS_CHX_UP   ((uint16_t)240)
#define SBUS_CHX_DOWN ((uint16_t)1807)

#if defined (REMOTE_SOURCE) && (REMOTE_SOURCE == 1)

static uint8_t sbus_buf[2][REMOTE_UART_RX_BUF_SIZE];

osal_status_t sbus_init(SBUS_Instance_t *sbus_instance){
    if (sbus_instance == NULL)
    {
        LOG_ERROR("sbus_instance is null");
        return OSAL_ERROR;
    }

    // 重新初始化uart
    REMOTE_UART.Init.BaudRate = 115200;
    if (HAL_UART_Init(&REMOTE_UART)!= HAL_OK){
        LOG_ERROR("uart init error");
        return OSAL_ERROR;
    } 

    // 初始化sbus实例
        OfflineDeviceInit_t offline_init = {
        .name = "sbus",
        .timeout_ms = 50,
        .level = OFFLINE_LEVEL_HIGH,
        .beep_times = 0,
        .enable = OFFLINE_ENABLE,
    };
    sbus_instance->offline_index = offline_device_register(&offline_init);
    if (sbus_instance->offline_index == OFFLINE_INVALID_INDEX)
    {
        LOG_ERROR("offline device register error");
        return OSAL_ERROR;
    }

    UART_Device_init_config sbus_cfg = {
        .huart = &REMOTE_UART,
        .expected_rx_len = 25,      
        .rx_buf = (uint8_t (*)[2])sbus_buf,
        .rx_buf_size = REMOTE_UART_RX_BUF_SIZE,
        .rx_mode = UART_MODE_DMA,
        .tx_mode = UART_MODE_BLOCKING,
    };
    sbus_instance->uart_device = BSP_UART_Device_Init(&sbus_cfg);
    if (sbus_instance->uart_device == NULL)
    {
        LOG_ERROR("uart device init error");
        return OSAL_ERROR;
    }

    LOG_INFO("sbus init success");

    return OSAL_SUCCESS;
}

void sbus_decode(SBUS_Instance_t *sbus_instance,uint8_t *buf){

    if (buf == NULL || sbus_instance == NULL || sbus_instance->uart_device->real_rx_len != sbus_instance->uart_device->expected_rx_len)   
    {
        return;
    }
    if(buf[0] == 0x0F && buf[24] == 0x00)
    {
        //获取原始数据 
        sbus_instance->SBUS_CH.CH1      = ((int16_t)buf[1] >> 0 | ((int16_t)buf[2] << 8)) & 0x07FF;
        sbus_instance->SBUS_CH.CH2      = ((int16_t)buf[2] >> 3 | ((int16_t)buf[3] << 5)) & 0x07FF;
        sbus_instance->SBUS_CH.CH3      = ((int16_t)buf[3] >> 6 | ((int16_t)buf[4] << 2) | (int16_t)buf[5] << 10) & 0x07FF;
        sbus_instance->SBUS_CH.CH4      = ((int16_t)buf[5] >> 1 | ((int16_t)buf[6] << 7)) & 0x07FF;

        //通道数据边界检查
        if(sbus_instance->SBUS_CH.CH1 < SBUS_CHX_UP || sbus_instance->SBUS_CH.CH1 > SBUS_CHX_DOWN) sbus_instance->SBUS_CH.CH1 = SBUS_CHX_BIAS;
        if(sbus_instance->SBUS_CH.CH2 < SBUS_CHX_UP || sbus_instance->SBUS_CH.CH2 > SBUS_CHX_DOWN) sbus_instance->SBUS_CH.CH2 = SBUS_CHX_BIAS;
        if(sbus_instance->SBUS_CH.CH3 < SBUS_CHX_UP || sbus_instance->SBUS_CH.CH3 > SBUS_CHX_DOWN) sbus_instance->SBUS_CH.CH3 = SBUS_CHX_BIAS;
        if(sbus_instance->SBUS_CH.CH4 < SBUS_CHX_UP || sbus_instance->SBUS_CH.CH4 > SBUS_CHX_DOWN) sbus_instance->SBUS_CH.CH4 = SBUS_CHX_BIAS;

        //前四个通道减去SBUS_CHX_BIAS是为了让数据的中间值变为0,方便后续使用
        sbus_instance->SBUS_CH.CH1 -= SBUS_CHX_BIAS;
        sbus_instance->SBUS_CH.CH2 -= SBUS_CHX_BIAS;
        sbus_instance->SBUS_CH.CH3 -= SBUS_CHX_BIAS;
        sbus_instance->SBUS_CH.CH4 -= SBUS_CHX_BIAS;

        //防止数据零漂，设置死区
        if(abs(sbus_instance->SBUS_CH.CH1) <= REMOTE_DEAD_ZONE) sbus_instance->SBUS_CH.CH1 = 0;
        if(abs(sbus_instance->SBUS_CH.CH2) <= REMOTE_DEAD_ZONE) sbus_instance->SBUS_CH.CH2 = 0;
        if(abs(sbus_instance->SBUS_CH.CH3) <= REMOTE_DEAD_ZONE) sbus_instance->SBUS_CH.CH3 = 0;
        if(abs(sbus_instance->SBUS_CH.CH4) <= REMOTE_DEAD_ZONE) sbus_instance->SBUS_CH.CH4 = 0;

        //其余通道读取
        sbus_instance->SBUS_CH.CH5      = ((int16_t)buf[6] >> 4 | ((int16_t)buf[7] << 4)) & 0x07FF;
        sbus_instance->SBUS_CH.CH6      = ((int16_t)buf[7] >> 7 | ((int16_t)buf[8] << 1) | (int16_t)buf[9] << 9) & 0x07FF;
        sbus_instance->SBUS_CH.CH7      = ((int16_t)buf[9] >> 2 | ((int16_t)buf[10] << 6)) & 0x07FF;
        sbus_instance->SBUS_CH.CH8      = ((int16_t)buf[10] >> 5 | ((int16_t)buf[11] << 3)) & 0x07FF;
        sbus_instance->SBUS_CH.CH9      = ((int16_t)buf[12] << 0 | ((int16_t)buf[13] << 8)) & 0x07FF;
        sbus_instance->SBUS_CH.CH10     = ((int16_t)buf[13] >> 3 | ((int16_t)buf[14] << 5)) & 0x07FF;
        sbus_instance->SBUS_CH.CH11     = ((int16_t)buf[14] >> 6 | ((int16_t)buf[15] << 2) | (int16_t)buf[16] << 10) & 0x07FF;
        sbus_instance->SBUS_CH.CH12     = ((int16_t)buf[16] >> 1 | ((int16_t)buf[17] << 7)) & 0x07FF;
        sbus_instance->SBUS_CH.CH13     = ((int16_t)buf[17] >> 4 | ((int16_t)buf[18] << 4)) & 0x07FF;
        sbus_instance->SBUS_CH.CH14     = ((int16_t)buf[18] >> 7 | ((int16_t)buf[19] << 1) | (int16_t)buf[20] << 9) & 0x07FF;
        sbus_instance->SBUS_CH.CH15     = ((int16_t)buf[20] >> 2 | ((int16_t)buf[21] << 6)) & 0x07FF;
        sbus_instance->SBUS_CH.CH16     = ((int16_t)buf[21] >> 5 | ((int16_t)buf[22] << 3)) & 0x07FF;
        sbus_instance->SBUS_CH.ConnectState = buf[23];

        if (sbus_instance->SBUS_CH.ConnectState == 0x00)
        {
            offline_device_update(sbus_instance->offline_index);
        }
    }
}

enum channel_state get_sbus_channel_state(SBUS_Instance_t *sbus_instance, uint8_t channel_index)
{ 
    if (sbus_instance == NULL || channel_index < 1 || channel_index > 16) {
        return channel_none;
    }
    
    uint16_t channel_value;
    
    switch (channel_index) {
        case 1:
            channel_value = sbus_instance->SBUS_CH.CH1 + SBUS_CHX_BIAS; // 前4个通道需要加上偏置
            break;
        case 2:
            channel_value = sbus_instance->SBUS_CH.CH2 + SBUS_CHX_BIAS;
            break;
        case 3:
            channel_value = sbus_instance->SBUS_CH.CH3 + SBUS_CHX_BIAS;
            break;
        case 4:
            channel_value = sbus_instance->SBUS_CH.CH4 + SBUS_CHX_BIAS;
            break;
        case 5:
            channel_value = sbus_instance->SBUS_CH.CH5;
            break;
        case 6:
            channel_value = sbus_instance->SBUS_CH.CH6;
            break;
        case 7:
            channel_value = sbus_instance->SBUS_CH.CH7;
            break;
        case 8:
            channel_value = sbus_instance->SBUS_CH.CH8;
            break;
        case 9:
            channel_value = sbus_instance->SBUS_CH.CH9;
            break;
        case 10:
            channel_value = sbus_instance->SBUS_CH.CH10;
            break;
        case 11:
            channel_value = sbus_instance->SBUS_CH.CH11;
            break;
        case 12:
            channel_value = sbus_instance->SBUS_CH.CH12;
            break;
        case 13:
            channel_value = sbus_instance->SBUS_CH.CH13;
            break;
        case 14:
            channel_value = sbus_instance->SBUS_CH.CH14;
            break;
        case 15:
            channel_value = sbus_instance->SBUS_CH.CH15;
            break;
        case 16:
            channel_value = sbus_instance->SBUS_CH.CH16;
            break;
        default:
            return channel_none;
    }

    if (channel_value <= SBUS_CHX_DOWN) {
        return channel_down;
    } else if (channel_value >= SBUS_CHX_UP) {
        return channel_up;
    } else if (channel_value == SBUS_CHX_BIAS) {
        return channel_bias;
    }
    
    return channel_none;
}
#else  
osal_status_t sbus_init(SBUS_Instance_t *sbus_instance){
    return OSAL_ERROR;
}
void sbus_decode(SBUS_Instance_t *sbus_instance,uint8_t *buf){

}
enum channel_state get_sbus_channel_state(SBUS_Instance_t *sbus_instance, uint8_t channel_index){
    return channel_none;
}
#endif
