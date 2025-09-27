#include "shell.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include "SEGGER_RTT.h"
#include "bsp_uart.h"
#include "osal_def.h"
#include "tools_config.h"


#define log_tag "shell"
#include "shell_log.h"

// 静态变量声明
static Shell_Module_t *shell_module = NULL;
static uint8_t shell_rx_buffer[SHELL_BUFFER_SIZE][2];                    // 命令接收缓冲区

static void shell_help_cmd(int argc, char **argv);
static void shell_clear_cmd(int argc, char **argv);
static void shell_version_cmd(int argc, char **argv);

// -------------------------- 内置命令表 --------------------------
static const shell_cmd_t g_shell_builtin_cmds[] = {
    {"help",    shell_help_cmd,    "显示所有命令（内置+动态） (Show all commands)"},
    {"clear",   shell_clear_cmd,   "清屏（ANSI序列） (Clear screen)"},
    {"version", shell_version_cmd, "显示Shell版本 (Show shell version)"},
    {"ps",      shell_ps_cmd,      "显示系统线程信息 (Show system thread info)"},
    {NULL,      NULL,              NULL} // 结束标记
};

/*-------------------------- Module层内部辅助函数 -------------------------- */
/**
 * @brief 解析命令（拆分cmd_buffer为argc/args，写入shell_module）
 */
static void shell_module_parse_cmd(void) {
    if (shell_module == NULL || !shell_module->initialized || shell_module->cmd_len == 0) return;

    shell_module->argc = 0;
    char *token = NULL;
    char *saveptr = NULL;

    // 给命令缓冲区加结束符
    shell_module->cmd_buffer[shell_module->cmd_len] = '\0';
    // 拆分第一个参数（命令名）
    token = strtok_r(shell_module->cmd_buffer, " ", &saveptr);
    while (token != NULL && shell_module->argc < SHELL_MAX_ARGS) {
        shell_module->args[shell_module->argc++] = token;
        token = strtok_r(NULL, " ", &saveptr);
    }
}

/**
 * @brief 执行命令（查找内置/动态命令，调用对应函数）
 */
static void shell_module_execute_cmd(void) {
    if (shell_module == NULL || !shell_module->initialized || shell_module->argc == 0) return;

    const shell_cmd_t *cmd = NULL;
    // 查找内置命令
    for (int i = 0; g_shell_builtin_cmds[i].name != NULL; i++) {
        if (strcmp(g_shell_builtin_cmds[i].name, shell_module->args[0]) == 0) {
            cmd = &g_shell_builtin_cmds[i];
            break;
        }
    }
    // 查找动态命令
    if (cmd == NULL) {
        for (int i = 0; i < shell_module->dynamic_cmd_count; i++) {
            if (strcmp(shell_module->dynamic_cmds[i].name, shell_module->args[0]) == 0) {
                cmd = &shell_module->dynamic_cmds[i];
                break;
            }
        }
    }
    // 执行命令或提示未知
    if (cmd != NULL && cmd->func != NULL) {
        cmd->func(shell_module->argc, shell_module->args);
    } else {
        shell_module_printf("Unknown command: %s\r\n", shell_module->args[0]);
    }
}

/**
 * @brief 打印命令提示符
 */
static void shell_module_print_prompt(void) {
    if (shell_module == NULL || !shell_module->initialized) return;
    shell_module_printf(SHELL_PROMPT);
}

/**
 * @brief 处理单个字符输入（核心逻辑：转义键、回车、退格等）
 */
static void shell_module_handle_char(char ch) {
    if (shell_module == NULL || !shell_module->initialized) return;

    // -------------------------- 处理转义序列（箭头键） --------------------------
    if (shell_module->escape_sequence == 1) {
        if (ch == '[') {
            shell_module->escape_sequence = 2;
        } else {
            shell_module->escape_sequence = 0; // 无效序列，重置
        }
        return;
    } else if (shell_module->escape_sequence == 2) {
        shell_module->escape_sequence = 0; // 无论是否有效，重置状态
        return;
    }

    // -------------------------- 处理普通字符 --------------------------
    switch (ch) {
        case '\033': // ESC键：启动转义序列检测
            shell_module->escape_sequence = 1;
            return;

        case '\r': // 回车/换行：确认命令
        case '\n':
            if (shell_module->cmd_len > 0) {
                shell_module_printf("\r\n");
                // 解析并执行命令
                shell_module_parse_cmd();
                shell_module_execute_cmd();
            } else {
                shell_module_printf("\r\n"); // 空命令也换行
            }
            shell_module->cmd_len = 0; // 重置命令缓冲区
            shell_module_print_prompt(); // 重新打印提示符
            break;

        case '\b': // 退格/删除：删除当前字符
        case 127:
            if (shell_module->cmd_len > 0) {
                shell_module->cmd_len--;
                shell_module_printf("\b \b"); // 视觉上删除字符
            }
            break;

        case 0x03: // Ctrl+C：强制清空当前命令
            shell_module->cmd_len = 0;
            shell_module_printf("^C\r\n");
            shell_module_print_prompt();
            break;

        default: // 可打印字符（32~126）：加入命令缓冲区
            if (shell_module->cmd_len < SHELL_CMD_MAX_LENGTH - 1 && ch >= 32 && ch <= 126) {
                shell_module->cmd_buffer[shell_module->cmd_len++] = ch;
                shell_module_printf("%c", ch); // 回显字符
            }
            break;
    }
}

// -------------------------- Module层接口实现 --------------------------
osal_status_t  shell_module_init(Shell_Module_t *shell) {
    if (shell == NULL) {
        return OSAL_ERROR;
    }

    // 保存shell指针
    shell_module = shell;

    shell_module->initialized = false;
    // 创建互斥锁
    osal_status_t status = osal_mutex_create(&shell_module->mutex, "Shell_Mutex");
    if (status != OSAL_SUCCESS) {
        LOG_ERROR("create mutex failed");
        return OSAL_ERROR;
    }

#if SHELL_RTT == 1
    SEGGER_RTT_Init();
    shell_module->use_rtt = 1;
#else  
    // UART设备配置（数据存Task层的ctx）
    UART_Device_init_config uart_cfg = {
        .huart = &SHELL_COM,
        .expected_rx_len = 0,
        .rx_buf = (uint8_t (*)[2])shell_rx_buffer, // 用ctx的接收缓冲区
        .rx_buf_size = SHELL_BUFFER_SIZE,
        .rx_mode = UART_MODE_IT,                   // 中断接收模式
        .tx_mode = UART_MODE_IT                    // 中断发送模式
    };
    // 初始化UART设备
    shell_module->uart_dev = BSP_UART_Device_Init(&uart_cfg);
    if (shell_module->uart_dev  == NULL) {
        LOG_ERROR("UART device init failed");
        osal_mutex_delete(&shell_module->mutex); // 初始化失败，释放已创建的互斥锁
        return OSAL_ERROR;
    }
    shell_module->use_rtt = 0;
#endif

    // 初始化功能相关配置
    shell_module->escape_sequence = 0;
    shell_module->cmd_len = 0;
    shell_module->dynamic_cmd_count = 0;
    // 标记功能初始化完成
    shell_module->initialized = true;

    // 打印欢迎信息
    shell_module_printf("\r\n");
    shell_module_printf("  __  __    _    ____    Shell Module v1.0\r\n");
    shell_module_printf(" |  \\/  |  / \\  / ___|   Built with OSAL\r\n");
    shell_module_printf(" | |\\/| | / _ \\ \\___ \\   Use %s\r\n", shell_module->use_rtt ? "RTT" : "UART");
    shell_module_printf(" | |  | |/ ___ \\ ___) |  Type 'help' for commands\r\n");
    shell_module_printf(" |_|  |_/_/   \\_\\____/   \r\n\r\n");

    return OSAL_SUCCESS;
}

void shell_module_period_process() {
    if (shell_module == NULL || !shell_module->initialized) return;

    uint8_t rx_len = 0;
    uint8_t *rx_data = NULL;

    // 数据（RTT或UART，用Task层的shell_module配置）
    if (shell_module->use_rtt) {
        // RTT读取（无锁）
        rx_len = SEGGER_RTT_ReadNoLock(0, shell_rx_buffer[0], SHELL_BUFFER_SIZE);
        rx_data = shell_rx_buffer[0];
    } else {
        // UART读取（调用BSP接口，数据存Task层的rx_buffer）
        rx_data = BSP_UART_Read(shell_module->uart_dev);
        rx_len = (rx_data != NULL) ? shell_module->uart_dev->real_rx_len : 0;
    }

    // 2. 处理读取到的字符（逐个传入字符处理函数）
    if (rx_data != NULL && rx_len > 0) {
        for (uint32_t i = 0; i < rx_len; i++) {
            shell_module_handle_char(rx_data[i]);
        }
    }
}

void shell_module_printf(const char *fmt, ...) {
    if (shell_module == NULL || !shell_module->initialized || fmt == NULL) return;

    static char print_buf[256]; // 静态缓冲区（唯一静态变量，仅用于临时存储）
    va_list args;
    int len = 0;

    // 格式化字符串
    va_start(args, fmt);
    len = vsnprintf(print_buf, sizeof(print_buf), fmt, args);
    va_end(args);
    if (len <= 0 || len >= sizeof(print_buf)) {
        return;
    }

    // 互斥锁
    osal_mutex_lock(&shell_module->mutex, TX_WAIT_FOREVER);

    // 输出到RTT或UART
    if (shell_module->use_rtt) {
        SEGGER_RTT_WriteNoLock(0, (uint8_t *)print_buf, len);
    } else {
        if (shell_module->uart_dev != NULL) {
            BSP_UART_Send(shell_module->uart_dev, (uint8_t *)print_buf, len);
        }
    }

    // 释放互斥锁
    osal_mutex_unlock(&shell_module->mutex);
}

void shell_module_send(uint8_t *data, uint16_t len){
    // 互斥锁
    osal_mutex_lock(&shell_module->mutex, TX_WAIT_FOREVER);

    // 输出到RTT或UART
    if (shell_module->use_rtt) {
        SEGGER_RTT_WriteNoLock(0, (uint8_t *)data, len);
    } else {
        if (shell_module->uart_dev != NULL) {
            BSP_UART_Send(shell_module->uart_dev, (uint8_t *)data, len);
        }
    }

    // 释放互斥锁
    osal_mutex_unlock(&shell_module->mutex);
}

osal_status_t shell_module_register_cmd(const char *name,shell_cmd_func_t func, const char *description) {
    // 参数合法性检查
    if (shell_module == NULL || name == NULL || func == NULL) {
        return OSAL_ERROR;
    }
    // 检查是否超出动态命令最大数量
    if (shell_module->dynamic_cmd_count >= SHELL_MAX_DYNAMIC_COMMANDS) {
        return OSAL_ERROR;
    }
    // 检查命令是否已存在（内置+动态）
    for (int i = 0; g_shell_builtin_cmds[i].name != NULL; i++) {
        if (strcmp(g_shell_builtin_cmds[i].name, name) == 0) {
            return OSAL_ERROR;
        }
    }
    for (int i = 0; i < shell_module->dynamic_cmd_count; i++) {
        if (strcmp(shell_module->dynamic_cmds[i].name, name) == 0) {
            return OSAL_ERROR;
        }
    }

    // 4. 注册命令
    shell_module->dynamic_cmds[shell_module->dynamic_cmd_count].name = name;
    shell_module->dynamic_cmds[shell_module->dynamic_cmd_count].func = func;
    shell_module->dynamic_cmds[shell_module->dynamic_cmd_count].description = (description != NULL) ? description : "No description";
    shell_module->dynamic_cmd_count++;
    return 0;
}

// -------------------------- 内置命令实现  --------------------------
static void shell_help_cmd(int argc, char **argv) {
    if (shell_module == NULL || !shell_module->initialized)
    {
        return;
    }else
    {
        shell_module_printf("Available commands (builtin + dynamic):\r\n");
        // 1. 打印内置命令
        for (int i = 0; g_shell_builtin_cmds[i].name != NULL; i++) {
            shell_module_printf("  %-12s - %s\r\n", g_shell_builtin_cmds[i].name, g_shell_builtin_cmds[i].description);
        }
        // 2. 打印动态命令
        for (int i = 0; i < shell_module->dynamic_cmd_count; i++) {
            shell_module_printf("  %-12s - %s\r\n", shell_module->dynamic_cmds[i].name, shell_module->dynamic_cmds[i].description);
        }
        shell_module_printf("\r\n");
    }
}

static void shell_clear_cmd(int argc, char **argv) {
    if (shell_module == NULL || !shell_module->initialized)
    {
        return;
    }else
    {
        // ANSI清屏序列：\033[2J清屏，\033[H光标移到左上角
        shell_module_printf("\033[2J\033[H");
    }
}

static void shell_version_cmd(int argc, char **argv) {
    if (shell_module == NULL || !shell_module->initialized)
    {
        return;
    }else
    {
        shell_module_printf("Shell Module Version: v1.0\r\n");
        shell_module_printf("Build Time: " __DATE__ " " __TIME__ "\r\n\r\n");
    }
}
