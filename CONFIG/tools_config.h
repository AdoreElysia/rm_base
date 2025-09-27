/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-09-10 09:54:06
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-09-27 12:55:19
 * @FilePath: /rm_base/CONFIG/tools_config.h
 * @Description: 
 */
#ifndef _TOOLS_CONFIG_H_
#define _TOOLS_CONFIG_H_

#include "robot_config.h"

/* shell 配置 */
#define SHELL_CMD_MAX_LENGTH         128                                  // 最大命令长度
#define SHELL_MAX_ARGS               8                                    // 最大参数数量
#define SHELL_HISTORY_MAX            10                                   // 历史命令数量
#define SHELL_PROMPT                 "shell> "                            // 提示符
#define SHELL_MAX_DYNAMIC_COMMANDS   8                                    // 注册命令的最大数量
#define SHELL_RTT                    1                                    // 使用rtt作为shell通讯方式,当使能时,SHELL_COM无效
#define SHELL_COM                    huart6                               // shell使用的通讯接口(uart)
#define SHELL_BUFFER_SIZE            32                                   // shell缓冲区大小
/* shell_log 配置 */
#define LOG_OUTPUT_LEVEL             LOG_LEVEL_INFO                       // 日志的输出等级
#define LOG_COLOR_ENABLE             1                                    // 启用彩色输出
#define LOG_TIMSTAMP_ENABLE          1                                    // 启用时间戳 

#endif // _TOOLS_CONFIG_H_