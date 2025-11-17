/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-10-12 10:28:56
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-10-17 14:37:09
 * @FilePath: \rm_base\modules\ITC\itc.h
 * @Description:
 */
#ifndef _ITC_H_
#define _ITC_H_

#include "osal_def.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 配置参数
#define ITC_MAX_SUBSCRIBERS_PER_TOPIC 4  // 每个话题最大订阅者数量
#define ITC_MAX_TOPIC_NAME_LEN        32 // 最大话题名称长度

// 根据不同RTOS类型定义队列缓冲区大小
#if (OSAL_RTOS_TYPE == OSAL_THREADX)
// ThreadX队列缓冲区大小计算: 消息数量(1) * 消息大小(sizeof(void*)) * ULONG大小
#define ITC_QUEUE_BUFFER_SIZE                                                                      \
    (1 * ((sizeof(void *) + sizeof(ULONG) - 1) / sizeof(ULONG)) * sizeof(ULONG))
#elif (OSAL_RTOS_TYPE == OSAL_FREERTOS)
// FreeRTOS队列缓冲区大小计算: 消息数量(1) * (消息大小(sizeof(void*)) + 队列结构体大小)
#define ITC_QUEUE_BUFFER_SIZE (sizeof(void *) + sizeof(StaticQueue_t))
#else
// 默认大小
#define ITC_QUEUE_BUFFER_SIZE sizeof(void *)
#endif

// 前向声明
typedef struct itc_topic      itc_topic_t;
typedef struct itc_subscriber itc_subscriber_t;

// 话题结构体
struct itc_topic
{
    char              name[ITC_MAX_TOPIC_NAME_LEN];               // 话题名称
    volatile uint8_t  is_active;                                  // 是否激活
    volatile uint8_t  subscriber_count;                           // 订阅者数量
    uint8_t           publisher_data_len;                         // 发布者规定的数据长度
    itc_subscriber_t *subscribers[ITC_MAX_SUBSCRIBERS_PER_TOPIC]; // 订阅者数组
    itc_topic_t      *next;                                       // 下一个话题
};

// 订阅者结构体
struct itc_subscriber
{
    volatile uint8_t  is_active;         // 是否激活
    itc_subscriber_t *next;              // 下一个订阅者
    uint32_t          expected_data_len; // 订阅者期望的数据长度

    osal_queue_t data_queue;                          // 用于存储数据指针的队列
    uint8_t      queue_buffer[ITC_QUEUE_BUFFER_SIZE]; // 队列缓冲区
    char         name[32];                            // 订阅者名称

    itc_topic_t *topic_ptr; // 关联的话题指针
};

/**
 * @brief 初始化ITC模块
 * @return OSAL_SUCCESS 成功，其他值 失败
 */
osal_status_t itc_init(void);

/**
 * @brief 初始化一个话题
 * @param {itc_topic_t*} topic 话题结构体指针
 * @param {const char*} topic_name 话题名称
 * @param {uint32_t} data_len 话题数据长度
 * @return {osal_status_t} 返回状态码
 */
osal_status_t itc_topic_init(itc_topic_t *topic, const char *topic_name, uint32_t data_len);

/**
 * @brief 反初始化一个话题
 * @param {itc_topic_t*} topic 要反初始化的话题指针
 * @return {osal_status_t} 返回状态码
 */
osal_status_t itc_topic_deinit(itc_topic_t *topic);

/**
 * @brief 初始化一个订阅者
 * @param {itc_subscriber_t*} subscriber 订阅者结构体指针
 * @param {const char*} topic_name 要订阅的话题名称
 * @param {uint32_t} expected_data_len 订阅者期望的数据长度
 * @return {osal_status_t} 返回状态码
 */
osal_status_t itc_subscriber_init(itc_subscriber_t *subscriber, const char *topic_name,
                                  uint32_t expected_data_len);

/**
 * @brief 反初始化一个订阅者
 * @param {itc_subscriber_t*} subscriber 订阅者指针
 * @return {osal_status_t} 返回状态码
 */
osal_status_t itc_subscriber_deinit(itc_subscriber_t *subscriber);

/**
 * @brief 发布消息到指定话题（只传输数据指针）
 * @param {itc_topic_t*} topic 要发布的目标话题
 * @param {void*} data_ptr 要发布的数据指针
 * @return {osal_status_t} 返回状态码
 */
osal_status_t itc_publish(itc_topic_t *topic, void *data_ptr);

/**
 * @brief 从订阅者接收消息
 * @param {itc_subscriber_t*} subscriber 订阅者指针
 * @return {void*} 返回接收到的数据指针，失败返回NULL
 */
void *itc_receive(itc_subscriber_t *subscriber);

/**
 * @brief 注册ITC模块的Shell命令
 */
void itc_shell_register_cmd(void);

#ifdef __cplusplus
}
#endif

#endif // _ITC_H_