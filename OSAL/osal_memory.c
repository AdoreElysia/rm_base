/*
 * @Author: laladuduqq 17503181697@163.com
 * @Date: 2025-12-11 10:37:53
 * @LastEditors: laladuduqq 17503181697@163.com
 * @LastEditTime: 2025-12-11 20:53:49
 * @FilePath: /rm_base/OSAL/osal_memory.c
 * @Description:
 */
#include "osal_def.h"
#include <stddef.h>

/* ==================== ThreadX 内存池管理 ==================== */
#if (OSAL_RTOS_TYPE == OSAL_THREADX)

/* ThreadX 使用外部字节池（由 app_azure_rtos.c 创建） */
extern TX_BYTE_POOL tx_app_byte_pool;

/**
 * @brief 分配内存（ThreadX）
 */
void *osal_malloc(size_t size)
{
    void *ptr = NULL;
    UINT  status;

    if (size == 0)
    {
        return NULL;
    }

    status = tx_byte_allocate(&tx_app_byte_pool, &ptr, size, TX_WAIT_FOREVER);

    if (status != TX_SUCCESS)
    {
        return NULL;
    }

    return ptr;
}

/**
 * @brief 释放内存（ThreadX）
 */
void osal_free(void *ptr)
{
    if (ptr == NULL)
    {
        return;
    }

    tx_byte_release(ptr);
}

/* ==================== FreeRTOS 内存管理 ==================== */
#elif (OSAL_RTOS_TYPE == OSAL_FREERTOS)

/**
 * @brief 分配内存（FreeRTOS）
 */
void *osal_malloc(size_t size)
{
    void *ptr;

    if (size == 0)
    {
        return NULL;
    }

    ptr = pvPortMalloc(size);

    return ptr;
}

/**
 * @brief 释放内存（FreeRTOS）
 */
void osal_free(void *ptr)
{
    if (ptr == NULL)
    {
        return;
    }

    vPortFree(ptr);
}
#else
#error "Unsupported RTOS type for memory management"
#endif
