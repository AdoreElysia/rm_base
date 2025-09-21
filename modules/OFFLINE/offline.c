/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-09-11 19:45:50
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-09-21 10:42:36
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
#include "log.h"

// 静态变量指针，指向应用层管理的数据
static OfflineManager_t* offline_manager_ptr = NULL;
static uint8_t* current_beep_times_ptr = NULL;
// 函数声明
static void beep_ctrl_times(ULONG timer);
static void shell_offline_cmd(int argc, char **argv);


// 设置静态变量指针
void offline_set_data_pointers(OfflineManager_t* manager, uint8_t* beep_times) {
    offline_manager_ptr = manager;
    current_beep_times_ptr = beep_times;
}

// 添加锁操作函数
static void offline_mutex_lock(void) {
    if (offline_manager_ptr) {
        osal_mutex_lock(&offline_manager_ptr->mutex, OSAL_WAIT_FOREVER);
    }
}

static void offline_mutex_unlock(void) {
    if (offline_manager_ptr) {
        osal_mutex_unlock(&offline_manager_ptr->mutex);
    }
}

void offline_init(void)
{
    beep_init(2000, 10, beep_ctrl_times);
    
    shell_register_function("offline", shell_offline_cmd, "Show offline device information");

    LOG_INFO("Offline init successfully.");
}

uint8_t offline_device_register(const OfflineDeviceInit_t* init)
{
    if (offline_manager_ptr == NULL || init == NULL || offline_manager_ptr->device_count >= MAX_OFFLINE_DEVICES) {
        return OFFLINE_INVALID_INDEX;
    }

    offline_mutex_lock(); 
    
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

    offline_mutex_unlock();

    LOG_INFO("offline device register: %s", device->name);

    return index;
}

void offline_device_update(uint8_t device_index)
{
    if (offline_manager_ptr != NULL) {
        offline_mutex_lock();
        if (device_index < offline_manager_ptr->device_count) {
            offline_manager_ptr->devices[device_index].last_time = tx_time_get();
        }   
        offline_mutex_unlock();
    }
}

void offline_device_enable(uint8_t device_index)
{
    if (offline_manager_ptr != NULL) {
        offline_mutex_lock();
        if (device_index < offline_manager_ptr->device_count) {
            offline_manager_ptr->devices[device_index].enable = OFFLINE_ENABLE;
        }
        offline_mutex_unlock();
    }
}

void offline_device_disable(uint8_t device_index)
{
    if (offline_manager_ptr != NULL) {
        offline_mutex_lock();
        if (device_index < offline_manager_ptr->device_count) {
            offline_manager_ptr->devices[device_index].enable = OFFLINE_DISABLE;
        }
        offline_mutex_unlock();
    }
}
uint8_t get_device_status(uint8_t device_index){
    if (offline_manager_ptr != NULL) {
        offline_mutex_lock();
        if(device_index < offline_manager_ptr->device_count){
            if (offline_manager_ptr->devices[device_index].enable == OFFLINE_ENABLE)
            {
                return offline_manager_ptr->devices[device_index].is_offline;
            }else
            {
                return STATE_ONLINE;
            }
        }
        offline_mutex_unlock();
    }
    return STATE_OFFLINE;
}

uint8_t get_system_status(void){
    if (offline_manager_ptr == NULL) {
        return STATE_OFFLINE;
    }else {
        offline_mutex_lock();
        uint8_t status = 0;
        for (uint8_t i = 0; i < offline_manager_ptr->device_count; i++) {
            if (offline_manager_ptr->devices[i].is_offline) {
                status |= (1 << i);
            }
        }
        offline_mutex_unlock();
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

// shell命令处理函数
void shell_offline_cmd(int argc, char **argv)
{
    const OfflineManager_t* manager = offline_manager_ptr;
    
    if (argc < 2) {
        // 显示帮助信息
        shell_printf("Usage: offline <command>\r\n");
        shell_printf("Commands:\r\n");
        shell_printf("  list     - List all registered devices and their status\r\n");
        shell_printf("  status   - Show overall system offline status\r\n");
        shell_printf("\r\n");
        return;
    }
    
    if (strcmp(argv[1], "list") == 0) {
        // 显示所有注册设备及其状态
        shell_printf("Offline Device List:\r\n");
        shell_printf("Index %-20s %-10s %-10s %-8s %-10s\r\n", "Name", "Status", "Timeout", "Level", "BeepTimes");
        shell_printf("-------------------------------------------------------------------------\r\n");
        
        if (manager->device_count == 0) {
            shell_printf("No devices registered.\r\n");
            shell_printf("\r\n");
            return;
        }
        
        offline_mutex_lock(); 

        for (uint8_t i = 0; i < manager->device_count; i++) {
            const OfflineDevice_t* device = &manager->devices[i];
            const char* status_str = device->is_offline ? "OFFLINE" : "ONLINE";
            const char* level_str = "";
            
            switch (device->level) {
                case OFFLINE_LEVEL_LOW:
                    level_str = "LOW";
                    break;
                case OFFLINE_LEVEL_MEDIUM:
                    level_str = "MEDIUM";
                    break;
                case OFFLINE_LEVEL_HIGH:
                    level_str = "HIGH";
                    break;
                default:
                    level_str = "UNKNOWN";
                    break;
            }
            
            shell_printf("%-5d %-20s %-10s %-10lu %-8s %-10d\r\n", 
                        device->index,
                        device->name,
                        status_str,
                        device->timeout_ms,
                        level_str,
                        device->beep_times);
        }

        offline_mutex_unlock(); 
        
        shell_printf("\r\nTotal devices: %d\r\n", manager->device_count);
        shell_printf("\r\n");
    } 
    else if (strcmp(argv[1], "status") == 0) {
        // 显示系统整体离线状态
        uint8_t system_status = get_system_status();
        
        if (system_status) {
            shell_printf("Status: OFFLINE\r\n");
            shell_printf("Some devices are offline!\r\n");
        } else {
            shell_printf("Status: ALL ONLINE\r\n");
            shell_printf("All devices are online.\r\n");
        }
        shell_printf("\r\n");
    }
    else {
        shell_printf("Unknown command: %s\r\n", argv[1]);
        shell_printf("Use 'offline' without arguments to see help.\r\n");
        shell_printf("\r\n");
    }
}

