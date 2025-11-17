/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-09-30 14:18:38
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-09-30 14:28:36
 * @FilePath: /rm_base/modules/OFFLINE/offline_shellcmd.c
 * @Description:
 */
#include "offline.h"
#include "shell.h"

// 静态变量指针，指向应用层管理的数据
static OfflineManager_t *offline_manager_ptr = NULL;

/**
 * @brief 显示设备状态信息
 */
static void show_device_list(void)
{
    if (offline_manager_ptr == NULL || !offline_manager_ptr->initialized)
    {
        shell_module_printf("Offline module not initialized!\r\n");
        return;
    }

    if (offline_manager_ptr->device_count == 0)
    {
        shell_module_printf("No devices registered.\r\n");
        return;
    }

    shell_module_printf("Offline Device List:\r\n");
    shell_module_printf("Idx %-20s %-10s %-10s %-8s %-10s %-10s\r\n", "Name", "Status", "Timeout",
                        "Level", "BeepTimes", "Enabled");
    shell_module_printf(
        "--------------------------------------------------------------------------------\r\n");

    for (uint8_t i = 0; i < offline_manager_ptr->device_count; i++)
    {
        OfflineDevice_t *device      = &offline_manager_ptr->devices[i];
        const char      *status_str  = device->is_offline ? "OFFLINE" : "ONLINE";
        const char      *level_str   = "";
        const char      *enabled_str = device->enable ? "YES" : "NO";

        switch (device->level)
        {
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

        shell_module_printf("%-3d %-20s %-10s %-10lu %-8s %-10d %-10s\r\n", device->index,
                            device->name, status_str, device->timeout_ms, level_str,
                            device->beep_times, enabled_str);
    }
    shell_module_printf("\r\n");
}

/**
 * @brief 显示系统整体状态
 */
static void show_system_status(void)
{
    if (offline_manager_ptr == NULL || !offline_manager_ptr->initialized)
    {
        shell_module_printf("Offline module not initialized!\r\n");
        return;
    }

    uint8_t system_status = offline_module_get_system_status();
    uint8_t offline_count = 0;

    // 计算离线设备数量
    for (uint8_t i = 0; i < offline_manager_ptr->device_count; i++)
    {
        if (offline_manager_ptr->devices[i].is_offline && offline_manager_ptr->devices[i].enable)
        {
            offline_count++;
        }
    }

    shell_module_printf("System Offline Status:\r\n");
    shell_module_printf("  Total Devices: %d\r\n", offline_manager_ptr->device_count);
    shell_module_printf("  Offline Devices: %d\r\n", offline_count);
    shell_module_printf("  Status Code: 0x%02X\r\n", system_status);
    shell_module_printf("  Module Status: %s\r\n",
                        (system_status == 0) ? "ALL ONLINE" : "SOME OFFLINE");
    shell_module_printf("\r\n");
}

/**
 * @brief 显示帮助信息
 */
static void show_help(void)
{
    shell_module_printf("Offline Module Shell Commands:\r\n");
    shell_module_printf("Usage: offline <command>\r\n");
    shell_module_printf("Commands:\r\n");
    shell_module_printf("  help     - Show this help message\r\n");
    shell_module_printf("  list     - List all registered devices and their status\r\n");
    shell_module_printf("  status   - Show overall system offline status\r\n");
    shell_module_printf("\r\n");
}

/**
 * @brief Offline模块的Shell命令处理函数
 * @param argc 参数数量
 * @param argv 参数数组
 */
static void shell_offline_cmd(int argc, char **argv)
{
    if (argc < 2)
    {
        show_help();
        return;
    }

    if (strcmp(argv[1], "help") == 0)
    {
        show_help();
    }
    else if (strcmp(argv[1], "list") == 0)
    {
        show_device_list();
    }
    else if (strcmp(argv[1], "status") == 0)
    {
        show_system_status();
    }
    else
    {
        shell_module_printf("Unknown command: %s\r\n", argv[1]);
        show_help();
    }
}

void offline_module_register_shell_cmd(OfflineManager_t *manager)
{
    // 保存管理器指针供shell命令使用
    offline_manager_ptr = manager;
    // 注册shell命令
    shell_module_register_cmd("offline", shell_offline_cmd, "Offline device status monitoring");
}