/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-09-07 10:43:45
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-10-13 16:32:18
 * @FilePath: \rm_base\OSAL\osal_queue.c
 * @Description: OSAL队列管理接口实现
 */

#include "osal_def.h"
#include <string.h>

#if (OSAL_RTOS_TYPE == OSAL_THREADX)

/* ThreadX下的队列实现 */
/* ThreadX下的队列实现 */
osal_status_t osal_queue_create(osal_queue_t *queue, 
                                const char *name,
                                unsigned int msg_size,
                                unsigned int msg_count,
                                void *msg_buffer)
{
    UINT result;
    
    if (queue == NULL || msg_buffer == NULL || msg_size == 0 || msg_count == 0) {
        return OSAL_INVALID_PARAM;
    }
    
    /* 计算队列总大小 - 在ThreadX中，队列大小是消息数量乘以消息大小（以ULONG为单位） */
    /* msg_size已经是字节数，需要转换为ULONG数量 */
    ULONG msg_size_in_ulongs = (msg_size + sizeof(ULONG) - 1) / sizeof(ULONG); /* 向上取整 */
    
    /* ThreadX队列创建 */
    result = tx_queue_create((TX_QUEUE*)queue, (CHAR*)name, msg_size_in_ulongs,
                             (VOID*)msg_buffer, msg_count * msg_size_in_ulongs * sizeof(ULONG));
    
    if (result == TX_SUCCESS) {
        return OSAL_SUCCESS;
    } else {
        return OSAL_ERROR;
    }
}

osal_status_t osal_queue_send(osal_queue_t *queue, void *msg_ptr, osal_tick_t timeout)
{
    UINT result;
    
    if (queue == NULL || msg_ptr == NULL) {
        return OSAL_INVALID_PARAM;
    }
    
    if (timeout == OSAL_WAIT_FOREVER) {
        result = tx_queue_send((TX_QUEUE*)queue, msg_ptr, TX_WAIT_FOREVER);
    } else {
        result = tx_queue_send((TX_QUEUE*)queue, msg_ptr, timeout);
    }
    
    if (result == TX_SUCCESS) {
        return OSAL_SUCCESS;
    } else if (result == TX_QUEUE_FULL) {
        return OSAL_TIMEOUT;
    } else {
        return OSAL_ERROR;
    }
}

osal_status_t osal_queue_recv(osal_queue_t *queue, void *msg_ptr, osal_tick_t timeout)
{
    UINT result;
    
    if (queue == NULL || msg_ptr == NULL) {
        return OSAL_INVALID_PARAM;
    }
    
    if (timeout == OSAL_WAIT_FOREVER) {
        result = tx_queue_receive((TX_QUEUE*)queue, msg_ptr, TX_WAIT_FOREVER);
    } else {
        result = tx_queue_receive((TX_QUEUE*)queue, msg_ptr, timeout);
    }
    
    if (result == TX_SUCCESS) {
        return OSAL_SUCCESS;
    } else if (result == TX_QUEUE_EMPTY) {
        return OSAL_TIMEOUT;
    } else {
        return OSAL_ERROR;
    }
}

osal_status_t osal_queue_delete(osal_queue_t *queue)
{
    UINT result;
    
    if (queue == NULL) {
        return OSAL_INVALID_PARAM;
    }
    
    result = tx_queue_delete((TX_QUEUE*)queue);
    
    if (result == TX_SUCCESS) {
        return OSAL_SUCCESS;
    } else {
        return OSAL_ERROR;
    }
}

#elif (OSAL_RTOS_TYPE == OSAL_FREERTOS)

/* FreeRTOS下的队列实现 */
osal_status_t osal_queue_create(osal_queue_t *queue, 
                                const char *name,
                                unsigned int msg_size,
                                unsigned int msg_count,
                                void *msg_buffer)
{
    if (queue == NULL || msg_buffer == NULL || msg_size == 0 || msg_count == 0) {
        return OSAL_INVALID_PARAM;
    }
    
    /* 使用静态内存分配方式创建队列 */
    queue->handle = xQueueCreateStatic(msg_count, msg_size, (uint8_t*)msg_buffer, &queue->buffer);
    
    if (queue->handle != NULL) {
        return OSAL_SUCCESS;
    } else {
        return OSAL_ERROR;
    }
}

osal_status_t osal_queue_send(osal_queue_t *queue, void *msg_ptr, osal_tick_t timeout)
{
    BaseType_t result;
    
    if (queue == NULL || queue->handle == NULL || msg_ptr == NULL) {
        return OSAL_INVALID_PARAM;
    }
    
    /* 判断是否在中断环境中 */
    if (xPortIsInsideInterrupt()) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        if (timeout == OSAL_NO_WAIT) {
            result = xQueueSendToBackFromISR(queue->handle, msg_ptr, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        } else {
            /* 中断中不支持阻塞操作，返回错误 */
            return OSAL_ERROR;
        }
    } else {
        if (timeout == OSAL_WAIT_FOREVER) {
            result = xQueueSendToBack(queue->handle, msg_ptr, portMAX_DELAY);
        } else {
            result = xQueueSendToBack(queue->handle, msg_ptr, timeout);
        }
    }
    
    if (result == pdTRUE) {
        return OSAL_SUCCESS;
    } else {
        return OSAL_TIMEOUT;
    }
}

osal_status_t osal_queue_recv(osal_queue_t *queue, void *msg_ptr, osal_tick_t timeout)
{
    BaseType_t result;
    
    if (queue == NULL || queue->handle == NULL || msg_ptr == NULL) {
        return OSAL_INVALID_PARAM;
    }
    
    /* 判断是否在中断环境中 */
    if (xPortIsInsideInterrupt()) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        if (timeout == OSAL_NO_WAIT) {
            result = xQueueReceiveFromISR(queue->handle, msg_ptr, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        } else {
            /* 中断中不支持阻塞操作，返回错误 */
            return OSAL_ERROR;
        }
    } else {
        if (timeout == OSAL_WAIT_FOREVER) {
            result = xQueueReceive(queue->handle, msg_ptr, portMAX_DELAY);
        } else {
            result = xQueueReceive(queue->handle, msg_ptr, timeout);
        }
    }
    
    if (result == pdTRUE) {
        return OSAL_SUCCESS;
    } else {
        return OSAL_TIMEOUT;
    }
}

osal_status_t osal_queue_delete(osal_queue_t *queue)
{
    if (queue == NULL || queue->handle == NULL) {
        return OSAL_INVALID_PARAM;
    }
    
    /* 静态创建的队列不能被删除，只需将句柄置为NULL */
    queue->handle = NULL;
    
    return OSAL_SUCCESS;
}

#endif