/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-09-15 09:18:31
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-10-26 23:51:58
 * @FilePath: \rm_base\modules\REMOTE\remote.c
 * @Description: 
 */
#include "remote.h"
#include "modules_config.h"
#include "offline.h"
#include "osal_def.h"
#include <stdint.h>
#include <string.h>

#define log_tag "remote"
#include "shell_log.h"

static remote_instance_t *remote_instance = NULL;

osal_status_t remote_init(remote_instance_t *remote)
{
    if (remote == NULL)
    {
        return OSAL_ERROR;
    }

    remote_instance = remote;

    osal_status_t ret = OSAL_SUCCESS;

#if defined(REMOTE_SOURCE)  && REMOTE_SOURCE == 1
    ret = sbus_init(&remote_instance->sbus_instance);
    if (ret == OSAL_SUCCESS) {
        remote_instance->remote_offline_index = remote_instance->sbus_instance.offline_index;
    } else {
        ret = OSAL_ERROR;
    }
#elif defined(REMOTE_SOURCE)  && REMOTE_SOURCE == 2
    ret = dt7_init(&remote_instance->dt7_instance);
    if (ret == OSAL_SUCCESS) {
        remote_instance->remote_offline_index = remote_instance->dt7_instance.offline_index;
    } else {
        ret = OSAL_ERROR;
    }
#endif

#if defined(REMOTE_VT_SOURCE) && REMOTE_VT_SOURCE == 1
    ret = vt02_init(&remote_instance->vt02_instance);
    if (ret == OSAL_SUCCESS) {
        remote_instance->vt_offline_index = remote_instance->vt02_instance.offline_index;
    } else {
        ret = OSAL_ERROR;
    }
#elif defined(REMOTE_VT_SOURCE) && REMOTE_VT_SOURCE == 2
    ret = vt03_init(&remote_instance->vt03_instance);
    if (ret == OSAL_SUCCESS) {
        remote_instance->vt_offline_index = remote_instance->vt03_instance.offline_index;
    } else {
        ret = OSAL_ERROR;
    }
#endif

    if (ret == OSAL_SUCCESS && REMOTE_SOURCE !=0 && REMOTE_VT_SOURCE !=0)
    {
        remote_instance->initflag = 1;
        LOG_INFO("remote init success");
    }
    return ret;
}

int16_t get_remote_channel(uint8_t channel_index, uint8_t is_vt_remote)
{
    if (remote_instance == NULL || remote_instance->initflag != 1) {
        return 0;
    }

    if (is_vt_remote) {
#if defined(REMOTE_VT_SOURCE) && REMOTE_VT_SOURCE == 2
        return get_vt03_channel(&remote_instance->vt03_instance, channel_index);
#endif
    } else {
#if defined(REMOTE_SOURCE) && REMOTE_SOURCE == 1
        return get_sbus_channel(&remote_instance->sbus_instance, channel_index);
#elif defined(REMOTE_SOURCE) && REMOTE_SOURCE == 2
        return get_dt7_channel(&remote_instance->dt7_instance, channel_index);
#endif
    }
    return 0;
}

button_state_t* get_remote_button_state()
{
    if (remote_instance == NULL || remote_instance->initflag != 1) {
        return NULL;
    }

#if defined(REMOTE_VT_SOURCE) && REMOTE_VT_SOURCE == 2
    return &remote_instance->vt03_instance.vt03_remote_data.button_state;
#else
    return NULL;
#endif
}

mouse_state_t* get_remote_mouse_state(uint8_t is_vt_remote)
{
    if (remote_instance == NULL  || remote_instance->initflag !=1) {
        return NULL;
    }

    if (is_vt_remote)
    {
#if defined(REMOTE_VT_SOURCE) && REMOTE_VT_SOURCE == 1
       return &remote_instance->vt02_instance.vt02_remote_data.mouse_state;
#elif defined(REMOTE_VT_SOURCE) && REMOTE_VT_SOURCE == 2
        return &remote_instance->vt03_instance.vt03_remote_data.mouse_state;
#endif
    }else
    {
#if defined(REMOTE_SOURCE) && REMOTE_SOURCE == 2
        return &remote_instance->dt7_instance.dt7_input.mouse_state;
#endif       
    }

    return NULL;
}

keyboard_state_t * get_remote_keyboard_state(uint8_t is_vt_remote)
{
    if (remote_instance == NULL || remote_instance->initflag !=1) {
        return NULL;
    }

    if (is_vt_remote)
    {
#if defined(REMOTE_VT_SOURCE) && REMOTE_VT_SOURCE == 1
       return &remote_instance->vt02_instance.vt02_remote_data.key_state;
#elif defined(REMOTE_VT_SOURCE) && REMOTE_VT_SOURCE == 2
        return &remote_instance->vt03_instance.vt03_remote_data.key_state;
#endif
    }else
    {
#if defined(REMOTE_SOURCE) && REMOTE_SOURCE == 2
        return &remote_instance->dt7_instance.dt7_input.keyboard_state;
#endif       
    }

    return NULL;
}

uint8_t remote_device_status(uint8_t is_vt_remote)
{
    if (remote_instance == NULL || remote_instance->initflag !=1) {
        return STATE_OFFLINE;
    }

    if (is_vt_remote)
    {
#if defined(REMOTE_VT_SOURCE) && REMOTE_VT_SOURCE == 2
        return offline_module_get_device_status(remote_instance->vt03_instance.offline_index);
#elif defined(REMOTE_VT_SOURCE) && REMOTE_VT_SOURCE == 1
        return offline_module_get_device_status(remote_instance->vt02_instance.offline_index);
#endif
    }else{
#if defined(REMOTE_SOURCE) && REMOTE_SOURCE == 2
        return offline_module_get_device_status(remote_instance->dt7_instance.offline_index);
#elif defined(REMOTE_SOURCE) && REMOTE_SOURCE == 1
        return offline_module_get_device_status(remote_instance->sbus_instance.offline_index);
#endif       
    }   
}
