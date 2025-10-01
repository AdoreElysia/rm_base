/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-09-16 12:17:58
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-10-01 20:23:02
 * @FilePath: /rm_base/applications/dm_imu_task/dm_imu_task.c
 * @Description: 
 */
#include "dm_imu_task.h"
#include <stdint.h>
#include <string.h>
#include "app_config.h"
#include "dm_imu.h"
#include "osal_def.h"
#include "modules_config.h"

#ifdef DM_IMU_MODULE

#define log_tag  "dm_imu_task"
#include "shell_log.h"

static DM_IMU_Moudule_t dm_imu_module;

static osal_thread_t dm_imu_thread;
static  uint8_t dm_imu_thread_stack[DM_IMU_THREAD_STACK_SIZE];

void dm_imu_task(ULONG thread_input){
    while (1) {
        osal_status_t status = OSAL_SUCCESS;
        dm_imu_request(DM_RID_GYRO);
        osal_delay_ms(1);
        dm_imu_request(DM_RID_EULER);
        osal_delay_ms(1);
        status = BSP_CAN_ReadSingleDevice(dm_imu_module.can_device,OSAL_WAIT_FOREVER);
        if (status == OSAL_SUCCESS)
        {
            dm_imu_update();
        }
    }
}

void dm_imu_task_init(void)
{
    memset(&dm_imu_module,0,sizeof(DM_IMU_Moudule_t));
    dm_imu_init(&dm_imu_module);
    
    osal_status_t status = osal_thread_create(&dm_imu_thread, "dm_imu_thread", dm_imu_task, 0, 
                       dm_imu_thread_stack,DM_IMU_THREAD_STACK_SIZE, DM_IMU_THREAD_PRIORITY);
    if(status != OSAL_SUCCESS) {
        LOG_ERROR("Failed to create dm_imu task!");
        return;
    }
    // 启动线程
    osal_thread_start(&dm_imu_thread);
    LOG_INFO("dm_imu task start successfully");
}
#else 
void dm_imu_task_init(void)
{} 
#endif
