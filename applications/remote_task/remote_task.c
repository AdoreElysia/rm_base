/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-09-15 23:49:44
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-10-01 21:08:38
 * @FilePath: /rm_base/applications/remote_task/remote_task.c
 * @Description:
 */
#include "remote_task.h"
#include "app_config.h"
#include "osal_def.h"
#include "remote.h"

#include "modules_config.h"
#include <string.h>

#ifdef REMOTE_MODULE

#define log_tag "remote_task"
#include "shell_log.h"

static remote_instance_t remote_instance;

static osal_thread_t                          remote_thread;
REMOTE_THREAD_STACK_SECTION static uint8_t    remote_thread_stack[REMOTE_THREAD_STACK_SIZE];
static osal_thread_t                          remote_vt_thread;
REMOTE_VT_THREAD_STACK_SECTION static uint8_t remote_vt_thread_stack[REMOTE_VT_THREAD_STACK_SIZE];

void remote_task(ULONG thread_input)
{
    (void)thread_input;
    while (1)
    {
        // 根据遥控器类型读取数据
#if defined(REMOTE_SOURCE) && REMOTE_SOURCE == 1
        // SBUS遥控器数据读取
        if (remote_instance.sbus_instance.uart_device != NULL)
        {
            uint8_t *data = BSP_UART_Read(remote_instance.sbus_instance.uart_device);
            if (data != NULL)
            {
                sbus_decode(&remote_instance.sbus_instance, data);
            }
        }
#elif defined(REMOTE_SOURCE) && REMOTE_SOURCE == 2
        // DT7遥控器数据读取
        if (remote_instance.dt7_instance.uart_device != NULL)
        {
            uint8_t *data = BSP_UART_Read(remote_instance.dt7_instance.uart_device);
            if (data != NULL)
            {
                dt7_decode(&remote_instance.dt7_instance, data);
            }
        }
#endif
    }
}

void remote_vt_task(ULONG thread_input)
{
    (void)thread_input;
    while (1)
    {
        // 根据图传遥控器类型读取数据
#if defined(REMOTE_VT_SOURCE) && REMOTE_VT_SOURCE == 1
        // VT02图传遥控器数据读取
        if (remote_instance.vt02_instance.uart_device != NULL)
        {
            uint8_t *data = BSP_UART_Read(remote_instance.vt02_instance.uart_device);
            if (data != NULL)
            {
                vt02_decode(&remote_instance.vt02_instance, data);
            }
        }
#elif defined(REMOTE_VT_SOURCE) && REMOTE_VT_SOURCE == 2
        // VT03图传遥控器数据读取
        if (remote_instance.vt03_instance.uart_device != NULL)
        {
            uint8_t *data = BSP_UART_Read(remote_instance.vt03_instance.uart_device);
            if (data != NULL)
            {
                vt03_decode(&remote_instance.vt03_instance, data);
            }
        }
#endif
    }
}

void remote_task_init()
{
    memset(&remote_instance, 0, sizeof(remote_instance_t));
    remote_init(&remote_instance);

    remote_shell_cmd_init(&remote_instance);

    osal_status_t status =
        osal_thread_create(&remote_thread, "remoteTask", remote_task, 0, remote_thread_stack,
                           REMOTE_THREAD_STACK_SIZE, REMOTE_THREAD_PRIORITY);

    status = osal_thread_create(&remote_vt_thread, "remoteVtTask", remote_vt_task, 0,
                                remote_vt_thread_stack, REMOTE_VT_THREAD_STACK_SIZE,
                                REMOTE_VT_THREAD_PRIORITY);

    if (status != TX_SUCCESS)
    {
        LOG_ERROR("Failed to create remote/vt task!");
        return;
    }
    status = osal_thread_start(&remote_thread);
    status = osal_thread_start(&remote_vt_thread);

    LOG_INFO("remote/vt task started successfully.");
}
#else
void remote_task_init()
{
}
#endif
