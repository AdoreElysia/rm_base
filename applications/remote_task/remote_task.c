/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-09-15 23:49:44
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-09-26 23:06:29
 * @FilePath: /rm_base/applications/remote_task/remote_task.c
 * @Description: 
 */
#include "remote_task.h"
#include "app_config.h"
#include "osal_def.h"
#include "remote.h"

#include "modules_config.h"

#ifdef REMOTE_MODULE

#define log_tag  "remote_task"
#include "shell_log.h"

static osal_thread_t remote_thread;
REMOTE_THREAD_STACK_SECTION static uint8_t remote_thread_stack[REMOTE_THREAD_STACK_SIZE];
static osal_thread_t remote_vt_thread;
REMOTE_VT_THREAD_STACK_SECTION static uint8_t remote_vt_thread_stack[REMOTE_VT_THREAD_STACK_SIZE];




void remote_task(ULONG thread_input){
    (void)thread_input;
    while (1)
    {
        remote_instance_t *remote_instance = get_remote_instance();
        if (remote_instance->initflag == 1) 
        {
            remote_task_function();
        }else
        {
            osal_delay_ms(100);
        }
        
    }
}

void remote_vt_task(ULONG thread_input){
    (void)thread_input;
    while (1)
    {
        remote_instance_t *remote_instance = get_remote_instance();
        if (remote_instance->initflag == 1) 
        {
            remote_vt_task_function();
        }else
        {
            osal_delay_ms(100);
        }
    }
}


void remote_task_init()
{
    osal_status_t status = osal_thread_create(&remote_thread, "remoteTask", remote_task, 
    0,remote_thread_stack, REMOTE_THREAD_STACK_SIZE,REMOTE_THREAD_PRIORITY);
    status = osal_thread_create(&remote_vt_thread, "remoteVtTask", remote_vt_task, 
    0,remote_vt_thread_stack, REMOTE_VT_THREAD_STACK_SIZE,REMOTE_VT_THREAD_PRIORITY);

    if(status != TX_SUCCESS) {
        LOG_ERROR("Failed to create remote/vt task!");
        return;
    }
    status = osal_thread_start(&remote_thread);
    status = osal_thread_start(&remote_vt_thread);

    LOG_INFO("remote/vt task started successfully.");
}
#else 
void remote_task_init(){} 
#endif

