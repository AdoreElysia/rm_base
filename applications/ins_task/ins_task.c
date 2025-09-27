/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-09-14 13:43:30
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-09-26 23:06:11
 * @FilePath: /rm_base/applications/ins_task/ins_task.c
 * @Description: 
 */
#include "app_config.h"
#include "ins.h"
#include "osal_def.h"
#include "modules_config.h"

#ifdef INS_MODULE

#define log_tag  "ins_task"
#include "shell_log.h"

static osal_thread_t INS_thread = {0};
INS_THREAD_STACK_SECTION static uint8_t INS_thread_stack[INS_THREAD_STACK_SIZE] = {0};

void ins_task(ULONG thread_input){
    while (1) 
    {
        ins_task_function();
        osal_delay_ms(1);
    }
}

void ins_task_init(){

    osal_status_t status = osal_thread_create(&INS_thread, "insTask", ins_task, 0,
                                INS_thread_stack, INS_THREAD_STACK_SIZE, INS_THREAD_PRIORITY);
    if(status != OSAL_SUCCESS) {
        LOG_ERROR("Failed to create ins task!");
        return;
    }
    
    // 启动线程
    osal_thread_start(&INS_thread);
    LOG_INFO("INS init and task created successfully");
}
#else 
void ins_task_init(){} 
#endif
