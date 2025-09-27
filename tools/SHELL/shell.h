/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-09-04 12:51:36
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-09-26 22:48:13
 * @FilePath: /rm_base/tools/SHELL/shell.h
 * @Description: 
 */
#ifndef _SHELL_H_
#define _SHELL_H_

#include "osal_def.h"
#include "bsp_uart.h"
#include <stdbool.h>
#include "tools_config.h"

// 命令处理函数指针
typedef void (*shell_cmd_func_t)(int argc, char **argv);

// 命令结构体
typedef struct {
    const char *name;           // 命令名称
    shell_cmd_func_t func;      // 处理函数
    const char *description;    // 命令描述
} shell_cmd_t;

// shell上下文结构体
typedef struct Shell_Module{
    UART_Device *uart_dev;                                            // UART设备
    char cmd_buffer[SHELL_CMD_MAX_LENGTH];                            // 命令缓冲区
    uint16_t cmd_len;                                                 // 命令长度
    char *args[SHELL_MAX_ARGS];                                       // 参数指针数组
    uint8_t argc;                                                     // 参数数量
    osal_mutex_t mutex;                                               // 互斥锁
    shell_cmd_t dynamic_cmds[SHELL_MAX_DYNAMIC_COMMANDS];             // 动态注册命令表
    int dynamic_cmd_count;                                            // 动态注册命令数量
    bool initialized;                                                 // 初始化状态
    bool use_rtt;                                                     // 是否使用rtt作为shell输出
    uint8_t escape_sequence;                                          // 转移序列状态（处理箭头键）
} Shell_Module_t;

/**
 * @brief 初始化Shell模块
 * @param shell Shell模块实例指针
 * @return osal_status_t 初始化状态，OSAL_SUCCESS表示成功，其他值表示失败
 */
osal_status_t shell_module_init(Shell_Module_t *shell);
/**
 * @brief Shell模块周期性处理函数，用于处理输入数据和执行相关操作
 *        该函数应在任务循环中定期调用
 */
void shell_module_period_process();
/**
 * @brief Shell模块格式化打印函数，支持可变参数
 * @param fmt 格式化字符串
 * @param ... 可变参数列表
 */
void shell_module_printf(const char *fmt, ...);
/**
 * @brief Shell模块发送数据函数
 * @param data 要发送的数据缓冲区
 * @param len 要发送的数据长度
 */
void shell_module_send(uint8_t *data, uint16_t len);
/**
 * @brief 注册自定义命令到Shell模块
 * @param name 命令名称
 * @param func 命令处理函数指针
 * @param description 命令描述信息
 * @return osal_status_t 注册状态，OSAL_SUCCESS表示成功，其他值表示失败
 */
osal_status_t shell_module_register_cmd(const char *name, shell_cmd_func_t func, const char *description);
// 内部函数声明
void shell_ps_cmd(int argc, char **argv);

#endif // _SHELL_H_