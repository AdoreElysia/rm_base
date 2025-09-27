/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-09-11 19:45:50
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-09-26 23:05:11
 * @FilePath: /rm_base/modules/OFFLINE/offline.c
 * @Description: 
 */
#include "offline.h"
#include "beep.h"
#include "bsp_dwt.h"
#include "modules_config.h"
#include "osal_def.h"
#include "rgb.h"
#include <stdint.h>
#include <string.h>
#include "shell.h"

#define log_tag  "offline"
#include "shell_log.h"

// 静态变量指针，指向应用层管理的数据
static OfflineManager_t* offline_manager_ptr = NULL;
static uint8_t* current_beep_times_ptr = NULL;
// 函数声明
static void beep_ctrl_times(ULONG timer);


// 设置静态变量指针
void offline_set_data_pointers(OfflineManager_t* manager, uint8_t* beep_times) {
    offline_manager_ptr = manager;
    current_beep_times_ptr = beep_times;
}

void offline_init(void)
{
    beep_init(2000, 10, beep_ctrl_times);
    

    LOG_INFO("Offline init successfully.");
}

uint8_t offline_device_register(const OfflineDeviceInit_t* init)
{
    if (offline_manager_ptr == NULL || init == NULL || offline_manager_ptr->device_count >= MAX_OFFLINE_DEVICES) {
        return OFFLINE_INVALID_INDEX;
    }

    
    osal_mutex_lock(&offline_manager_ptr->mutex, OSAL_WAIT_FOREVER);
    
    uint8_t index = offline_manager_ptr->device_count;
    OfflineDevice_t* device = &offline_manager_ptr->devices[index];
    
    strncpy(device->name, init->name, sizeof(device->name) - 1);
    device->name[sizeof(device->name) - 1] = '\0'; // 确保字符串结束符
    device->timeout_ms = init->timeout_ms;
    device->level = init->level;
    device->beep_times = init->beep_times;
    device->is_offline = STATE_OFFLINE;
    device->last_time = 0;
    device->index = index;
    device->enable = init->enable;
    
    offline_manager_ptr->device_count++;

    osal_mutex_unlock(&offline_manager_ptr->mutex);

    LOG_INFO("offline device register: %s", device->name);

    return index;
}

void offline_device_update(uint8_t device_index)
{
    if (offline_manager_ptr != NULL) {
        if (device_index < offline_manager_ptr->device_count) {
            offline_manager_ptr->devices[device_index].last_time = tx_time_get();
        }   
    }
}

void offline_device_enable(uint8_t device_index)
{
    if (offline_manager_ptr != NULL) {
        if (device_index < offline_manager_ptr->device_count) {
            offline_manager_ptr->devices[device_index].enable = OFFLINE_ENABLE;
        }
    }
}

void offline_device_disable(uint8_t device_index)
{
    if (offline_manager_ptr != NULL) {
        if (device_index < offline_manager_ptr->device_count) {
            offline_manager_ptr->devices[device_index].enable = OFFLINE_DISABLE;
        }
    }
}
uint8_t get_device_status(uint8_t device_index){
    if (offline_manager_ptr != NULL) {
        if(device_index < offline_manager_ptr->device_count){
            if (offline_manager_ptr->devices[device_index].enable == OFFLINE_ENABLE)
            {
                return offline_manager_ptr->devices[device_index].is_offline;
            }else
            {
                return STATE_ONLINE;
            }
        }
    }
    return STATE_OFFLINE;
}

uint8_t get_system_status(void){
    if (offline_manager_ptr == NULL) {
        return STATE_OFFLINE;
    }else {
        uint8_t status = 0;
        for (uint8_t i = 0; i < offline_manager_ptr->device_count; i++) {
            if (offline_manager_ptr->devices[i].is_offline) {
                status |= (1 << i);
            }
        }
        return status;    
    }
}

void beep_ctrl_times(ULONG timer)
{
    static uint32_t beep_period_start_time;
    static uint32_t beep_cycle_start_time;
    static uint8_t remaining_beep_cycles;

    if (current_beep_times_ptr != NULL) {
        if (DWT_GetTimeline_ms() - beep_period_start_time > OFFLINE_BEEP_PERIOD)
        {
            remaining_beep_cycles = *current_beep_times_ptr;
            beep_period_start_time = DWT_GetTimeline_ms();
            beep_cycle_start_time = DWT_GetTimeline_ms();
        }
        else if (remaining_beep_cycles != 0)
        {
            if (DWT_GetTimeline_ms() - beep_cycle_start_time < OFFLINE_BEEP_ON_TIME)
            {
                #if OFFLINE_BEEP_ENABLE == 1
                beep_set_tune(OFFLINE_BEEP_TUNE_VALUE, OFFLINE_BEEP_CTRL_VALUE);
                #else
                beep_set_tune(0, 0);
                #endif
                RGB_show(LED_Red);
            }
            else if (DWT_GetTimeline_ms() - beep_cycle_start_time < OFFLINE_BEEP_ON_TIME + OFFLINE_BEEP_OFF_TIME)
            {
                beep_set_tune(0, 0);
                RGB_show(LED_Black);
            }
            else
            {
                remaining_beep_cycles--;
                beep_cycle_start_time = DWT_GetTimeline_ms();
            }
        }
    }else {
        // 确保在指针无效时关闭蜂鸣器
        beep_set_tune(0, 0);
    }
}

