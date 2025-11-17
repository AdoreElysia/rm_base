/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-09-14 12:50:59
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-09-30 14:30:35
 * @FilePath: /rm_base/applications/offline_task/offline_task.c
 * @Description:
 */

#include "offline_task.h"
#include "app_config.h"
#include "bsp_dwt.h"
#include "modules_config.h"
#include "offline.h"
#include "osal_def.h"
#include "rgb.h"
#include <stdint.h>
#include <string.h>

#ifdef OFFLINE_MODULE

#if OFFLINE_WATCHDOG_ENABLE
#include "iwdg.h"
#endif

#define log_tag "offline_task"
#include "shell_log.h"

static osal_thread_t                        offline_thread;
OFFLINE_THREAD_STACK_SECTION static uint8_t offline_thread_stack[OFFLINE_THREAD_STACK_SIZE];
static OfflineManager_t                     offline_manager;
static uint8_t                              current_beep_times;

void offline_task(ULONG thread_input)
{
#if OFFLINE_WATCHDOG_ENABLE
    __HAL_DBGMCU_FREEZE_IWDG();
    MX_IWDG_Init();
#endif

    while (1)
    {
#if OFFLINE_WATCHDOG_ENABLE
        HAL_IWDG_Refresh(&hiwdg);
#endif

        if (offline_manager.initialized == true)
        {
            uint8_t  highest_error_level = 0;
            uint8_t  alarm_device_index  = OFFLINE_INVALID_INDEX;
            uint32_t current_time        = (uint32_t)DWT_GetTimeline_ms();

            // 重置错误状态
            highest_error_level               = 0;
            alarm_device_index                = OFFLINE_INVALID_INDEX;
            bool any_device_offline           = false;
            bool all_same_level_are_beep_zero = true; // 初始时设定所有相同优先级设备beep_times都为0

            // 检查所有设备状态
            for (uint8_t i = 0; i < offline_manager.device_count; i++)
            {
                OfflineDevice_t *device = &offline_manager.devices[i];

                if (!device->enable)
                {
                    continue;
                }

                if (current_time - device->last_time > device->timeout_ms)
                {
                    device->is_offline = STATE_OFFLINE;
                    any_device_offline = true;

                    // 更新最高优先级设备
                    if (device->level > highest_error_level)
                    {
                        highest_error_level          = device->level;
                        alarm_device_index           = i;
                        current_beep_times           = device->beep_times;
                        all_same_level_are_beep_zero = (device->beep_times == 0); // 更新初始状态
                    }
                    // 相同优先级时的处理
                    else if (device->level == highest_error_level)
                    {
                        // 检查是否所有相同优先级的设备beep_times都为0
                        if (device->beep_times != 0)
                        {
                            all_same_level_are_beep_zero = false;
                        }
                        // 如果当前设备不需要蜂鸣（beep_times=0），保持原来的设备
                        if (device->beep_times == 0)
                        {
                            continue;
                        }
                        // 如果之前选中的设备不需要蜂鸣，或者当前设备蜂鸣次数更少
                        if (current_beep_times == 0 ||
                            (device->beep_times > 0 && device->beep_times < current_beep_times))
                        {
                            alarm_device_index = i;
                            current_beep_times = device->beep_times;
                        }
                    }
                }
                else
                {
                    device->is_offline = STATE_ONLINE;
                }
            }

            // 触发报警或清除报警
            if (alarm_device_index != OFFLINE_INVALID_INDEX && any_device_offline &&
                !all_same_level_are_beep_zero)
            {
                // 有需要蜂鸣的设备离线
            }
            else if (any_device_offline && all_same_level_are_beep_zero)
            {
                // 相同优先级且beep_times都为0的设备离线，常亮红灯
                RGB_show(LED_Red);
            }
            else
            {
                // 所有设备都在线，清除报警
                current_beep_times = 0;
                RGB_show(LED_Green); // 表示所有设备都在线
            }
            osal_delay_ms(10);
        }
        else
        {
            osal_delay_ms(100);
        }
    }
}

void offline_task_init()
{

    memset(&offline_manager, 0, sizeof(offline_manager));

    osal_mutex_create(&offline_manager.mutex, "offline_manager_mutex");

    offline_module_init(&offline_manager, &current_beep_times);

    offline_module_register_shell_cmd(&offline_manager);

    osal_status_t status =
        osal_thread_create(&offline_thread, "offlineTask", offline_task, 0, offline_thread_stack,
                           OFFLINE_THREAD_STACK_SIZE, OFFLINE_THREAD_PRIORITY);

    if (status != TX_SUCCESS)
    {
        LOG_ERROR("Failed to create offline task!");
        return;
    }
    status = osal_thread_start(&offline_thread);

    LOG_INFO("Offline task started successfully.");
}

#else
void offline_task_init()
{
}
#endif
