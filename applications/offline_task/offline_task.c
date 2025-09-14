/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-09-14 12:50:59
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-09-14 13:09:39
 * @FilePath: /rm_base/applications/offline_task/offline_task.c
 * @Description: 
 */

#include "offline.h"
#include "app_config.h"
#include "modules_config.h"
#include "osal_def.h"
#include "offline_task.h"

#if OFFLINE_WATCHDOG_ENABLE
#include "iwdg.h"
#endif

#define log_tag  "offline_task"
#include "log.h"

static osal_thread_t offline_thread;
OFFLINE_THREAD_STACK_SECTION static uint8_t offline_thread_stack[OFFLINE_THREAD_STACK_SIZE];

void offline_task(ULONG thread_input)
{
    #if OFFLINE_WATCHDOG_ENABLE
        __HAL_DBGMCU_FREEZE_IWDG();
        MX_IWDG_Init();
    #endif    
    (void)(thread_input);
    while (1)
    {
        offline_task_function();
        #if OFFLINE_WATCHDOG_ENABLE
            HAL_IWDG_Refresh(&hiwdg);
        #endif
        osal_delay_ms(10);
    }
}

void offline_task_init(){
    osal_status_t status = osal_thread_create(&offline_thread, "offlineTask", offline_task, 
    0,offline_thread_stack, OFFLINE_THREAD_STACK_SIZE,OFFLINE_THREAD_PRIORITY);

    if(status != TX_SUCCESS) {
        LOG_ERROR("Failed to create offline task!");
        return;
    }
    status = osal_thread_start(&offline_thread);

    LOG_INFO("Offline task started successfully.");
}