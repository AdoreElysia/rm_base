/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-10-18 13:15:38
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-10-19 10:41:33
 * @FilePath: \rm_base\modules\USB\usb_user.c
 * @Description:
 */
/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-10-18 13:15:38
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-10-18 22:49:35
 * @FilePath: \rm_base\modules\USB\usb_user.c
 * @Description:
 */
#include "usb_user.h"
#include "cdc_acm.h"
#include "gpio.h"
#include "offline.h"
#include "osal_def.h"
#include <stdint.h>
#include <string.h>

#define log_tag "usb_user"
#include "shell_log.h"

static user_cdc_t *g_user_cdc_instance = NULL;

osal_status_t usb_user_init(user_cdc_t *instance)
{
    if (instance == NULL)
    {
        return OSAL_ERROR;
    }
    else
    {
        g_user_cdc_instance = instance;
        g_user_cdc_instance->cdc_device =
            cdc_acm_init(0, USB_OTG_FS_PERIPH_BASE, sizeof(struct User_Recv_s));
        if (g_user_cdc_instance->cdc_device == NULL)
        {
            LOG_ERROR("cdc device init error");
            return OSAL_ERROR;
        }
        OfflineDeviceInit_t offline_init = {
            .name       = "minipc",
            .timeout_ms = 100,
            .level      = OFFLINE_LEVEL_HIGH,
            .beep_times = 0,
            .enable     = OFFLINE_ENABLE,
        };
        g_user_cdc_instance->offline_index = offline_module_device_register(&offline_init);
        if (g_user_cdc_instance->offline_index == OFFLINE_INVALID_INDEX)
        {
            LOG_ERROR("offline device register error");
            return OSAL_ERROR;
        }
    }
    LOG_INFO("usb user init success");
    return OSAL_SUCCESS;
}
osal_status_t usb_user_send(struct User_Send_s *send)
{
    if (g_user_cdc_instance == NULL || g_user_cdc_instance->cdc_device == NULL || send == NULL)
    {
        return OSAL_ERROR;
    }
    cdc_acm_send(g_user_cdc_instance->cdc_device, (uint8_t *)send, sizeof(struct User_Send_s));
    return OSAL_SUCCESS;
}
struct User_Recv_s *usb_user_recv()
{
    if (g_user_cdc_instance == NULL || g_user_cdc_instance->cdc_device == NULL)
    {
        return NULL;
    }
    uint8_t *data = cdc_acm_read(g_user_cdc_instance->cdc_device);
    if (g_user_cdc_instance->cdc_device->real_rx_len == sizeof(struct User_Recv_s))
    {
        if (data[0] == 0xAA && data[sizeof(struct User_Recv_s) - 1] == 0xBB)
        {
            offline_module_device_update(g_user_cdc_instance->offline_index);
            return (struct User_Recv_s *)g_user_cdc_instance->cdc_device
                ->rx_buf[!g_user_cdc_instance->cdc_device->rx_active_buf];
        }
    }
    return NULL;
}
