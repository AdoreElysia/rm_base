/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-09-17 16:39:19
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-09-26 23:01:16
 * @FilePath: /rm_base/applications/shell_task/shell_task.c
 * @Description:
 */
#include "shell_task.h"
#include "app_config.h"
#include "osal_def.h"
#include "shell.h"
#include <stdint.h>

#ifdef SHELL_MODULE

#define log_tag "shell_task"
#include "shell_log.h"

static Shell_Module_t                     shell_module;
static osal_thread_t                      shell_thread;
SHELL_THREAD_STACK_SECTION static uint8_t shell_thread_stack[SHELL_THREAD_STACK_SIZE];

void shell_task(ULONG thread_input)
{
    while (1)
    {
        shell_module_period_process();
        osal_delay_ms(100);
    }
}

void shell_task_init()
{
    osal_status_t status;
    memset(&shell_module, 0, sizeof(Shell_Module_t));
    shell_module_init(&shell_module);
    status = osal_thread_create(&shell_thread, "shell_task", shell_task, 0, shell_thread_stack,
                                SHELL_THREAD_STACK_SIZE, SHELL_THREAD_PRIORITY);
    if (status != OSAL_SUCCESS)
    {
        LOG_ERROR("create thread failed");
        return;
    }
    status = osal_thread_start(&shell_thread);
    if (status != OSAL_SUCCESS)
    {
        LOG_ERROR("start thread failed");
        return;
    }
    LOG_INFO("shell task init success");
}

#else
void shell_task_init()
{
}
#endif
