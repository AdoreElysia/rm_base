/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-09-26 22:34:35
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-09-26 23:00:09
 * @FilePath: /rm_base/tools/SHELL/LOG/shell_log.h
 * @Description:
 */
#ifndef _SHELL_LOG_H_
#define _SHELL_LOG_H_

#include <stdint.h>

// 日志级别定义
#define LOG_LEVEL_VERBOSE 0
#define LOG_LEVEL_DEBUG   1
#define LOG_LEVEL_INFO    2
#define LOG_LEVEL_WARN    3
#define LOG_LEVEL_ERROR   4
#define LOG_LEVEL_ASSERT  5

// 默认tag
#define LOG_DEFAULT_TAG   "default"

// 通过log_tag宏来确定使用的tag，如果未定义则使用默认tag
#ifdef log_tag
#define LOG_TAG log_tag
#else
#define LOG_TAG LOG_DEFAULT_TAG
#endif

// 日志宏定义，支持使用默认tag
#define LOG_VERBOSE(format, ...)                                                                   \
    shell_log_message(LOG_LEVEL_VERBOSE, LOG_TAG, format, ##__VA_ARGS__)
#define LOG_DEBUG(format, ...)  shell_log_message(LOG_LEVEL_DEBUG, LOG_TAG, format, ##__VA_ARGS__)
#define LOG_INFO(format, ...)   shell_log_message(LOG_LEVEL_INFO, LOG_TAG, format, ##__VA_ARGS__)
#define LOG_WARN(format, ...)   shell_log_message(LOG_LEVEL_WARN, LOG_TAG, format, ##__VA_ARGS__)
#define LOG_ERROR(format, ...)  shell_log_message(LOG_LEVEL_ERROR, LOG_TAG, format, ##__VA_ARGS__)
#define LOG_ASSERT(format, ...) shell_log_message(LOG_LEVEL_ASSERT, LOG_TAG, format, ##__VA_ARGS__)

/**
 * @brief 设置日志输出级别
 * @param level 日志级别
 */
void shell_log_set_level(uint8_t level);

/**
 * @brief 获取当前日志输出级别
 * @return 当前日志级别
 */
uint8_t shell_log_get_level(void);

/**
 * @brief 统一的日志输出函数
 * @param level 日志级别
 * @param tag 日志标签
 * @param format 格式化字符串
 * @param ... 可变参数
 */
void shell_log_message(uint8_t level, const char *tag, const char *format, ...);

#endif // _SHELL_LOG_H_