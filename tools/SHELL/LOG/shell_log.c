/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-09-26 22:34:27
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-09-26 23:15:26
 * @FilePath: /rm_base/tools/SHELL/LOG/shell_log.c
 * @Description: 
 */
#include "shell_log.h"
#include "shell.h"
#include "tools_config.h"
#include "bsp_dwt.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

// 日志级别字符串
static const char *log_level_strings[] = {
    "VERBOSE",
    "DEBUG",
    "INFO",
    "WARN",
    "ERROR",
    "ASSERT"
};

// ANSI颜色代码
#define COLOR_VERBOSE "\033[37m"  // 白色
#define COLOR_DEBUG   "\033[36m"  // 青色
#define COLOR_INFO    "\033[32m"  // 绿色
#define COLOR_WARN    "\033[33m"  // 黄色
#define COLOR_ERROR   "\033[31m"  // 红色
#define COLOR_ASSERT  "\033[35m"  // 紫色
#define COLOR_END     "\033[0m"   // 结束颜色

// 日志级别对应的颜色
static const char *log_level_colors[] = {
    COLOR_VERBOSE,
    COLOR_DEBUG,
    COLOR_INFO,
    COLOR_WARN,
    COLOR_ERROR,
    COLOR_ASSERT
};

// 当前日志输出级别
static uint8_t current_log_level = LOG_OUTPUT_LEVEL;
// 递归调用检查变量
static volatile uint8_t log_recursive_check = 0;

// 使用DWT实现时间戳函数
static char* get_dwt_timestamp(void) {
    static char timestamp_str[32];
    uint64_t us = DWT_GetTimeline_us();
    // 格式化为 [秒.微秒] 格式，例如 [123.456789]
    snprintf(timestamp_str, sizeof(timestamp_str), "[%lu.%06lu]", 
             (unsigned long)(us / 1000000),
             (unsigned long)(us % 1000000));
    return timestamp_str;
}

/**
 * @brief 内部统一的日志输出函数
 * @param level 日志级别
 * @param tag 日志标签
 * @param format 格式化字符串
 * @param args 可变参数列表
 */
static void internal_log_output(uint8_t level, const char *tag, const char *format, va_list args) {
    if (level < current_log_level) {
        return;
    }

    // 递归调用检查，防止死锁
    if (log_recursive_check) {
        return;
    }
    
    // 设置递归检查标志
    log_recursive_check = 1;

    // 构建日志消息到缓冲区
    static char log_buffer[256];
    int offset = 0;

    // 开始颜色
    if (LOG_COLOR_ENABLE) {
        int color_len = strlen(log_level_colors[level]);
        if (offset + color_len < sizeof(log_buffer)) {
            strcpy(log_buffer + offset, log_level_colors[level]);
            offset += color_len;
        }
    }

    // 时间戳
    if (LOG_TIMSTAMP_ENABLE) {
        char* timestamp = get_dwt_timestamp();
        int timestamp_len = strlen(timestamp);
        if (offset + timestamp_len < sizeof(log_buffer)) {
            strcpy(log_buffer + offset, timestamp);
            offset += timestamp_len;
            log_buffer[offset++] = ' ';
        }
    }

    // 输出日志级别
    int level_str_len = strlen(log_level_strings[level]);
    if (offset + level_str_len + 4 < sizeof(log_buffer)) {
        sprintf(log_buffer + offset, "[%s] ", log_level_strings[level]);
        offset = strlen(log_buffer);
    }

    // 输出标签
    if (tag) {
        int tag_len = strlen(tag);
        if (offset + tag_len + 4 < sizeof(log_buffer)) {
            sprintf(log_buffer + offset, "[%s] ", tag);
            offset = strlen(log_buffer);
        }
    }

    // 输出日志内容
    if (offset < sizeof(log_buffer)) {
        int content_len = vsnprintf(log_buffer + offset, sizeof(log_buffer) - offset, format, args);
        if (content_len > 0) {
            offset += content_len;
        }
    }

    // 结束颜色并添加换行
    if (LOG_COLOR_ENABLE) {
        int end_color_len = strlen(COLOR_END);
        if (offset + end_color_len + 2 < sizeof(log_buffer)) {
            strcpy(log_buffer + offset, COLOR_END);
            offset = strlen(log_buffer);
        }
    }
    
    // 自动添加换行符
    if (offset + 2 < sizeof(log_buffer)) {
        log_buffer[offset++] = '\r';
        log_buffer[offset++] = '\n';
    }

    // 确保字符串结束
    if (offset >= sizeof(log_buffer)) {
        offset = sizeof(log_buffer) - 1;
    }
    log_buffer[offset] = '\0';

    // 使用shell_module_send发送日志
    shell_module_send((uint8_t*)log_buffer, strlen(log_buffer));
    
    // 清除递归检查标志
    log_recursive_check = 0;
}

void shell_log_set_level(uint8_t level) {
    if (level <= LOG_LEVEL_ASSERT) {
        current_log_level = level;
    }
}

uint8_t shell_log_get_level(void) {
    return current_log_level;
}

void shell_log_message(uint8_t level, const char *tag, const char *format, ...) {
    // 递归调用检查，防止死锁
    if (log_recursive_check) {
        return;
    }
    
    va_list args;
    va_start(args, format);
    internal_log_output(level, tag, format, args);
    va_end(args);
}