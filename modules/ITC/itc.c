/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-10-12 10:28:56
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-10-17 17:42:13
 * @FilePath: \rm_base\modules\ITC\itc.c
 * @Description:
 */

#include "itc.h"
#include "osal_def.h"
#include <stdio.h>
#include <string.h>

#define log_tag "ITC"
#include "shell_log.h"

// 全局变量
static itc_topic_t      *topic_list      = NULL;
static itc_subscriber_t *subscriber_list = NULL;
static osal_mutex_t      itc_mutex;           // 用于保护ITC操作的互斥锁
static uint8_t           itc_initialized = 0; // ITC模块初始化标志

// 内部函数声明
static int find_topic(const char *topic_name, itc_topic_t **topic);
static int add_topic_to_list(itc_topic_t *topic);
static int remove_topic_from_list(itc_topic_t *topic);
static int add_subscriber_to_list(itc_subscriber_t *subscriber);
static int remove_subscriber_from_list(itc_subscriber_t *subscriber);

// 初始化ITC模块
osal_status_t itc_init(void)
{
    osal_status_t status;

    if (itc_initialized)
    {
        return OSAL_SUCCESS;
    }

    topic_list      = NULL;
    subscriber_list = NULL;

    // 创建互斥锁
    status = osal_mutex_create(&itc_mutex, "itc_mutex");
    if (status != OSAL_SUCCESS)
    {
        LOG_ERROR("Failed to create ITC mutex");
        return status;
    }

    itc_initialized = 1;
    return OSAL_SUCCESS;
}

// 初始化话题
osal_status_t itc_topic_init(itc_topic_t *topic, const char *topic_name, uint32_t data_len)
{
    osal_status_t status;
    itc_topic_t  *existing_topic;

    if (topic == NULL || topic_name == NULL || strlen(topic_name) >= ITC_MAX_TOPIC_NAME_LEN ||
        data_len == 0)
    {
        LOG_ERROR("Invalid topic parameters");
        return OSAL_INVALID_PARAM;
    }

    if (!itc_initialized)
    {
        LOG_ERROR("ITC module not initialized");
        return OSAL_ERROR;
    }

    // 获取互斥锁
    status = osal_mutex_lock(&itc_mutex, OSAL_WAIT_FOREVER);
    if (status != OSAL_SUCCESS)
    {
        LOG_ERROR("Failed to take ITC mutex");
        return status;
    }

    // 检查话题是否已存在
    if (find_topic(topic_name, &existing_topic) == 0)
    {
        LOG_ERROR("Topic %s already exists", topic_name);
        osal_mutex_unlock(&itc_mutex);
        return OSAL_ERROR;
    }

    // 初始化话题
    memset(topic, 0, sizeof(itc_topic_t));
    strcpy(topic->name, topic_name);
    topic->is_active          = 1;
    topic->subscriber_count   = 0;
    topic->publisher_data_len = data_len;

    // 添加到话题列表
    add_topic_to_list(topic);

    // 释放互斥锁
    osal_mutex_unlock(&itc_mutex);

    return OSAL_SUCCESS;
}

// 反初始化话题
osal_status_t itc_topic_deinit(itc_topic_t *topic)
{
    osal_status_t status;

    if (topic == NULL)
    {
        LOG_ERROR("Attempt to deinit NULL topic");
        return OSAL_INVALID_PARAM;
    }

    if (!itc_initialized)
    {
        LOG_ERROR("ITC module not initialized");
        return OSAL_ERROR;
    }

    // 获取互斥锁
    status = osal_mutex_lock(&itc_mutex, OSAL_WAIT_FOREVER);
    if (status != OSAL_SUCCESS)
    {
        LOG_ERROR("Failed to take ITC mutex");
        return status;
    }

    // 反初始化所有订阅者
    for (int i = 0; i < topic->subscriber_count; i++)
    {
        itc_subscriber_t *subscriber = topic->subscribers[i];
        if (subscriber != NULL)
        {
            // 从全局订阅者列表移除
            remove_subscriber_from_list(subscriber);

            // 删除队列
            osal_queue_delete(&subscriber->data_queue);
        }
    }
    topic->subscriber_count = 0;

    // 从列表中移除话题
    remove_topic_from_list(topic);

    // 释放互斥锁
    osal_mutex_unlock(&itc_mutex);

    return OSAL_SUCCESS;
}

// 初始化订阅者
osal_status_t itc_subscriber_init(itc_subscriber_t *subscriber, const char *topic_name,
                                  uint32_t expected_data_len)
{
    osal_status_t status;
    itc_topic_t  *topic;

    if (subscriber == NULL || topic_name == NULL || expected_data_len == 0)
    {
        LOG_ERROR("Invalid subscriber parameters");
        return OSAL_INVALID_PARAM;
    }

    if (!itc_initialized)
    {
        LOG_ERROR("ITC module not initialized");
        return OSAL_ERROR;
    }

    // 获取互斥锁
    status = osal_mutex_lock(&itc_mutex, OSAL_WAIT_FOREVER);
    if (status != OSAL_SUCCESS)
    {
        LOG_ERROR("Failed to take ITC mutex");
        return status;
    }

    // 查找话题
    if (find_topic(topic_name, &topic) != 0)
    {
        LOG_ERROR("Topic %s does not exist", topic_name);
        osal_mutex_unlock(&itc_mutex);
        return OSAL_INVALID_PARAM;
    }

    // 检查数据长度是否匹配
    if (expected_data_len != topic->publisher_data_len)
    {
        LOG_ERROR("Data length mismatch for topic %s: expected %d, got %d", topic->name,
                  topic->publisher_data_len, expected_data_len);
        osal_mutex_unlock(&itc_mutex);
        return OSAL_INVALID_PARAM;
    }

    // 检查话题的订阅者数量限制
    if (topic->subscriber_count >= ITC_MAX_SUBSCRIBERS_PER_TOPIC)
    {
        LOG_ERROR("Maximum subscribers reached for topic %s", topic->name);
        osal_mutex_unlock(&itc_mutex);
        return OSAL_ERROR;
    }

    // 初始化订阅者
    memset(subscriber, 0, sizeof(itc_subscriber_t));
    subscriber->is_active         = 1;
    subscriber->expected_data_len = expected_data_len;
    subscriber->topic_ptr         = topic;
    snprintf(subscriber->name, sizeof(subscriber->name), "subscriber_%p", subscriber);

    // 创建队列用于存储数据指针
    status = osal_queue_create(&subscriber->data_queue, subscriber->name, sizeof(void *), 1,
                               subscriber->queue_buffer);
    if (status != OSAL_SUCCESS)
    {
        LOG_ERROR("Failed to create queue for subscriber of topic %s", topic->name);
        osal_mutex_unlock(&itc_mutex);
        return status;
    }

    // 将订阅者添加到话题的订阅者列表
    topic->subscribers[topic->subscriber_count] = subscriber;
    topic->subscriber_count++;

    // 添加到全局订阅者列表
    add_subscriber_to_list(subscriber);

    // 释放互斥锁
    osal_mutex_unlock(&itc_mutex);

    return OSAL_SUCCESS;
}

// 反初始化订阅者
osal_status_t itc_subscriber_deinit(itc_subscriber_t *subscriber)
{
    osal_status_t status;

    if (subscriber == NULL)
    {
        LOG_ERROR("Attempt to deinit NULL subscriber");
        return OSAL_INVALID_PARAM;
    }

    if (!itc_initialized)
    {
        LOG_ERROR("ITC module not initialized");
        return OSAL_ERROR;
    }

    // 获取互斥锁
    status = osal_mutex_lock(&itc_mutex, OSAL_WAIT_FOREVER);
    if (status != OSAL_SUCCESS)
    {
        LOG_ERROR("Failed to take ITC mutex");
        return status;
    }

    // 从话题的订阅者列表中移除
    if (subscriber->topic_ptr != NULL)
    {
        itc_topic_t *topic = subscriber->topic_ptr;

        for (int i = 0; i < topic->subscriber_count; i++)
        {
            if (topic->subscribers[i] == subscriber)
            {
                // 移除订阅者，保持数组连续
                for (int j = i; j < topic->subscriber_count - 1; j++)
                {
                    topic->subscribers[j] = topic->subscribers[j + 1];
                }
                topic->subscribers[topic->subscriber_count - 1] = NULL;
                topic->subscriber_count--;
                break;
            }
        }
    }

    // 从全局订阅者列表移除
    remove_subscriber_from_list(subscriber);

    // 删除队列
    osal_queue_delete(&subscriber->data_queue);

    // 释放互斥锁
    osal_mutex_unlock(&itc_mutex);

    return OSAL_SUCCESS;
}

// 发布消息（只传输数据指针）
osal_status_t itc_publish(itc_topic_t *topic, void *data_ptr)
{
    // 快速参数检查
    if (!topic || !data_ptr || !topic->is_active)
    {
        LOG_ERROR("Invalid parameters for publish");
        return OSAL_INVALID_PARAM;
    }

    // 遍历所有订阅者并将消息添加到它们的队列
    for (int i = 0; i < topic->subscriber_count; i++)
    {
        itc_subscriber_t *subscriber = topic->subscribers[i];
        if (subscriber && subscriber->is_active)
        {
            // 尝试发送数据指针到队列,在这里不处理发送失败的情况（如队列满）
            osal_queue_send(&subscriber->data_queue, &data_ptr, OSAL_NO_WAIT);
        }
    }

    return OSAL_SUCCESS;
}

// 从订阅者接收消息
void *itc_receive(itc_subscriber_t *subscriber)
{
    if (!subscriber || !subscriber->is_active)
    {
        LOG_ERROR("Invalid parameters for receive");
        return NULL;
    }

    void *data_ptr = NULL;

    // 从队列接收数据指针
    osal_status_t status = osal_queue_recv(&subscriber->data_queue, &data_ptr, OSAL_WAIT_FOREVER);

    if (status == OSAL_SUCCESS)
    {
        return data_ptr;
    }

    return NULL;
}

// 内部函数实现

// 查找话题
static int find_topic(const char *topic_name, itc_topic_t **topic)
{
    itc_topic_t *current = topic_list;
    while (current != NULL)
    {
        if (strcmp(current->name, topic_name) == 0)
        {
            *topic = current;
            return 0;
        }
        current = current->next;
    }
    return -1;
}

// 添加话题到列表
static int add_topic_to_list(itc_topic_t *topic)
{
    topic->next = topic_list;
    topic_list  = topic;
    return 0;
}

// 从列表中移除话题
static int remove_topic_from_list(itc_topic_t *topic)
{
    if (topic_list == topic)
    {
        topic_list = topic->next;
        return 0;
    }

    itc_topic_t *current = topic_list;
    while (current != NULL && current->next != topic)
    {
        current = current->next;
    }

    if (current != NULL)
    {
        current->next = topic->next;
        return 0;
    }

    return -1;
}

// 添加订阅者到列表
static int add_subscriber_to_list(itc_subscriber_t *subscriber)
{
    subscriber->next = subscriber_list;
    subscriber_list  = subscriber;
    return 0;
}

// 从列表中移除订阅者
static int remove_subscriber_from_list(itc_subscriber_t *subscriber)
{
    if (subscriber_list == subscriber)
    {
        subscriber_list = subscriber->next;
        return 0;
    }

    itc_subscriber_t *current = subscriber_list;
    while (current != NULL && current->next != subscriber)
    {
        current = current->next;
    }

    if (current != NULL)
    {
        current->next = subscriber->next;
        return 0;
    }

    return -1;
}