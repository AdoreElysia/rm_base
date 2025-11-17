/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-09-15 09:29:57
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-10-26 17:05:56
 * @FilePath: \rm_base\modules\REMOTE\DT7\dt7.c
 * @Description:
 */
#include "dt7.h"
#include "modules_config.h"
#include "offline.h"
#include "osal_def.h"
#include <string.h>

#define log_tag "dt7"
#include "shell_log.h"

#if defined(REMOTE_SOURCE) && (REMOTE_SOURCE == 2)

static uint8_t dt7_buf[2][REMOTE_UART_RX_BUF_SIZE];

osal_status_t dt7_init(DT7_Instance_t *dt7_instance)
{
    if (dt7_instance == NULL)
    {
        LOG_ERROR("dt7_instance is null");
        return OSAL_ERROR;
    }

    // 重新初始化uart
    REMOTE_UART.Init.BaudRate = 100000;
    if (HAL_UART_Init(&REMOTE_UART) != HAL_OK)
    {
        LOG_ERROR("uart init error");
        return OSAL_ERROR;
    }

    // 初始化sbus实例
    OfflineDeviceInit_t offline_init = {
        .name       = "dt7",
        .timeout_ms = 50,
        .level      = OFFLINE_LEVEL_HIGH,
        .beep_times = 0,
        .enable     = OFFLINE_ENABLE,
    };
    dt7_instance->offline_index = offline_module_device_register(&offline_init);
    if (dt7_instance->offline_index == OFFLINE_INVALID_INDEX)
    {
        LOG_ERROR("offline device register error");
        return OSAL_ERROR;
    }

    UART_Device_init_config sbus_cfg = {
        .huart           = &REMOTE_UART,
        .expected_rx_len = 18,
        .rx_buf          = (uint8_t(*)[2])dt7_buf,
        .rx_buf_size     = REMOTE_UART_RX_BUF_SIZE,
        .rx_mode         = UART_MODE_DMA,
        .tx_mode         = UART_MODE_BLOCKING,
    };
    dt7_instance->uart_device = BSP_UART_Device_Init(&sbus_cfg);
    if (dt7_instance->uart_device == NULL)
    {
        LOG_ERROR("uart device init error");
        return OSAL_ERROR;
    }

    LOG_INFO("dt7 init success");

    return OSAL_SUCCESS;
}

void dt7_decode(DT7_Instance_t *dt7_instance, uint8_t *buf)
{

    // 长度检查
    if (buf == NULL || dt7_instance == NULL ||
        dt7_instance->uart_device->real_rx_len != dt7_instance->uart_device->expected_rx_len)
    {
        return;
    }

    // 原始数据
    dt7_instance->dt7_input.ch1 = (buf[0] | buf[1] << 8) & 0x07FF;
    dt7_instance->dt7_input.ch2 = (buf[1] >> 3 | buf[2] << 5) & 0x07FF;
    dt7_instance->dt7_input.ch3 = (buf[2] >> 6 | buf[3] << 2 | buf[4] << 10) & 0x07FF;
    dt7_instance->dt7_input.ch4 = (buf[4] >> 1 | buf[5] << 7) & 0x07FF;

    // 通道数据边界检查
    if (dt7_instance->dt7_input.ch1 < DT7_CH_VALUE_MIN ||
        dt7_instance->dt7_input.ch1 > DT7_CH_VALUE_MAX ||
        dt7_instance->dt7_input.ch2 < DT7_CH_VALUE_MIN ||
        dt7_instance->dt7_input.ch2 > DT7_CH_VALUE_MAX ||
        dt7_instance->dt7_input.ch3 < DT7_CH_VALUE_MIN ||
        dt7_instance->dt7_input.ch3 > DT7_CH_VALUE_MAX ||
        dt7_instance->dt7_input.ch4 < DT7_CH_VALUE_MIN ||
        dt7_instance->dt7_input.ch4 > DT7_CH_VALUE_MAX)
    {
        memset(&dt7_instance->dt7_input, 0, sizeof(DT7_INPUT_t));
        return;
    }

    // 前四个通道减去DT7_CH_VALUE_OFFSET是为了让数据的中间值变为0,方便后续使用
    dt7_instance->dt7_input.ch1 -= DT7_CH_VALUE_OFFSET;
    dt7_instance->dt7_input.ch2 -= DT7_CH_VALUE_OFFSET;
    dt7_instance->dt7_input.ch3 -= DT7_CH_VALUE_OFFSET;
    dt7_instance->dt7_input.ch4 -= DT7_CH_VALUE_OFFSET;

    // 防止数据零漂，设置正负DT7_DEAD_ZONE的死区
    if (abs(dt7_instance->dt7_input.ch1) <= REMOTE_DEAD_ZONE)
        dt7_instance->dt7_input.ch1 = 0;
    if (abs(dt7_instance->dt7_input.ch2) <= REMOTE_DEAD_ZONE)
        dt7_instance->dt7_input.ch2 = 0;
    if (abs(dt7_instance->dt7_input.ch3) <= REMOTE_DEAD_ZONE)
        dt7_instance->dt7_input.ch3 = 0;
    if (abs(dt7_instance->dt7_input.ch4) <= REMOTE_DEAD_ZONE)
        dt7_instance->dt7_input.ch4 = 0;

    // 拨杆
    dt7_instance->dt7_input.sw1 = ((buf[5] >> 4) & 0x000C) >> 2;
    dt7_instance->dt7_input.sw2 = (buf[5] >> 4) & 0x0003;

    // 鼠标
    dt7_instance->dt7_input.mouse_state.mouse_x = buf[6] | (buf[7] << 8); // x axis
    dt7_instance->dt7_input.mouse_state.mouse_y = buf[8] | (buf[9] << 8);
    dt7_instance->dt7_input.mouse_state.mouse_z = buf[10] | (buf[11] << 8);
    dt7_instance->dt7_input.mouse_state.mouse_l = buf[12];
    dt7_instance->dt7_input.mouse_state.mouse_r = buf[13];

    // 鼠标数据边界检查
    if (dt7_instance->dt7_input.mouse_state.mouse_x < -32768 ||
        dt7_instance->dt7_input.mouse_state.mouse_x > 32767)
    {
        dt7_instance->dt7_input.mouse_state.mouse_x = 0;
    }
    if (dt7_instance->dt7_input.mouse_state.mouse_y < -32768 ||
        dt7_instance->dt7_input.mouse_state.mouse_y > 32767)
    {
        dt7_instance->dt7_input.mouse_state.mouse_y = 0;
    }
    if (dt7_instance->dt7_input.mouse_state.mouse_z < -32768 ||
        dt7_instance->dt7_input.mouse_state.mouse_z > 32767)
    {
        dt7_instance->dt7_input.mouse_state.mouse_z = 0;
    }

    // 更新按键状态
    dt7_instance->dt7_input.keyboard_state.key_code = buf[14] | buf[15] << 8; // 更新当前状态

    // 滚轮
    dt7_instance->dt7_input.wheel = (buf[16] | buf[17] << 8) - DT7_CH_VALUE_OFFSET;
    if (abs(dt7_instance->dt7_input.wheel) <= REMOTE_DEAD_ZONE * 10)
        dt7_instance->dt7_input.wheel = 0;

    offline_module_device_update(dt7_instance->offline_index);
}

int16_t get_dt7_channel(DT7_Instance_t *dt7_instance, uint8_t channel_index)
{
    if (dt7_instance == NULL || channel_index < 1 || channel_index > 6)
    {
        return 0;
    }
    uint16_t channel_value;
    switch (channel_index)
    {
    case 1:
        channel_value = dt7_instance->dt7_input.ch1;
        break;
    case 2:
        channel_value = dt7_instance->dt7_input.ch2;
        break;
    case 3:
        channel_value = dt7_instance->dt7_input.ch3;
        break;
    case 4:
        channel_value = dt7_instance->dt7_input.ch4;
        break;
    case 5:
        channel_value = dt7_instance->dt7_input.sw1;
        break;
    case 6:
        channel_value = dt7_instance->dt7_input.sw2;
        break;
    case 7:
        channel_value = dt7_instance->dt7_input.wheel;
        break;
    default:
        channel_value = 0;
        break;
    }
    return channel_value;
}
#else
osal_status_t dt7_init(DT7_Instance_t *dt7_instance)
{
    return OSAL_ERROR;
}
void dt7_decode(DT7_Instance_t *dt7_instance, uint8_t *buf)
{
}
int16_t get_dt7_channel(DT7_Instance_t *dt7_instance, uint8_t channel_index)
{
    return 0;
}
#endif
