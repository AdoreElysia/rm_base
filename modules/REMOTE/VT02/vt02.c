/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-09-15 09:30:03
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-09-30 13:37:37
 * @FilePath: /rm_base/modules/REMOTE/VT02/vt02.c
 * @Description: 
 */
#include "vt02.h"
#include "crc_rm.h"
#include "modules_config.h"
#include "offline.h"
#include "osal_def.h"
#include <string.h>

#define log_tag "vt02"
#include "shell_log.h"

#if defined (REMOTE_VT_SOURCE) && (REMOTE_VT_SOURCE == 1)

static uint8_t vt02_buf[2][REMOTE_UART_RX_BUF_SIZE];

osal_status_t vt02_init(VT02_Instance_t *vt02_instance)
{ 
    if (vt02_instance == NULL)
    {
        LOG_ERROR("vt02_instance is null");
        return OSAL_ERROR;
    }

    // 重新初始化uart
    REMOTE_VT_UART.Init.BaudRate = 115200;
    if (HAL_UART_Init(&REMOTE_VT_UART)!= HAL_OK){
        LOG_ERROR("uart init error");
        return OSAL_ERROR;
    } 

    // 初始化sbus实例
    OfflineDeviceInit_t offline_init = {
        .name = "vt02",
        .timeout_ms = 100,
        .level = OFFLINE_LEVEL_HIGH,
        .beep_times = 0,
        .enable = OFFLINE_ENABLE,
    };
    vt02_instance->offline_index = offline_module_device_register(&offline_init);
    if (vt02_instance->offline_index == OFFLINE_INVALID_INDEX)
    {
        LOG_ERROR("offline device register error");
        return OSAL_ERROR;
    }

    UART_Device_init_config sbus_cfg = {
        .huart = &REMOTE_VT_UART,
        .expected_rx_len = 0,      
        .rx_buf = (uint8_t (*)[2])vt02_buf,
        .rx_buf_size = REMOTE_UART_RX_BUF_SIZE,
        .rx_mode = UART_MODE_DMA,
        .tx_mode = UART_MODE_BLOCKING,
    };
    vt02_instance->uart_device = BSP_UART_Device_Init(&sbus_cfg);
    if (vt02_instance->uart_device == NULL)
    {
        LOG_ERROR("uart device init error");
        return OSAL_ERROR;
    }

    LOG_INFO("vt02 init success");

    return OSAL_SUCCESS;
}

void vt02_decode(VT02_Instance_t *vt02_instance, uint8_t *buf)
{
    if (vt02_instance == NULL || buf == NULL || buf[0] != 0xA5) {return;}
    // 验证帧头CRC8校验
    if (!Verify_CRC8_Check_Sum(buf, 5)) {return;}

    // 判断是否为键鼠遥控数据(0x0304)
    uint16_t cmd_id = (buf[6] << 8) | buf[5];
    if (cmd_id != 0x0304) {return;}

    // 解析键鼠遥控数据 (从偏移量7开始)
    uint8_t *data_ptr = &buf[7];
    
    // 鼠标
    vt02_instance->vt02_remote_data.mouse_state.mouse_x = (int16_t)((data_ptr[1] << 8) | data_ptr[0]);
    vt02_instance->vt02_remote_data.mouse_state.mouse_y = (int16_t)((data_ptr[3] << 8) | data_ptr[2]);
    vt02_instance->vt02_remote_data.mouse_state.mouse_z = (int16_t)((data_ptr[5] << 8) | data_ptr[4]);
    vt02_instance->vt02_remote_data.mouse_state.mouse_l = data_ptr[6];
    vt02_instance->vt02_remote_data.mouse_state.mouse_r = data_ptr[7];
    
    // 键盘
    vt02_instance->vt02_remote_data.key_state.key_code = (data_ptr[9] << 8) | data_ptr[8];

    offline_module_device_update(vt02_instance->offline_index);
}
#else  
osal_status_t vt02_init(VT02_Instance_t *vt02_instance)
{
    return OSAL_ERROR;
}
void vt02_decode(VT02_Instance_t *vt02_instance, uint8_t *buf){

}
#endif
