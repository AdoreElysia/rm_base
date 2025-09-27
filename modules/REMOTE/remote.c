/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-09-15 09:18:31
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-09-26 23:05:49
 * @FilePath: /rm_base/modules/REMOTE/remote.c
 * @Description: 
 */
#include "remote.h"
#include "modules_config.h"
#include "osal_def.h"
#include <stdint.h>
#include <string.h>

#ifdef REMOTE_MODULE

#define log_tag "remote"
#include "shell_log.h"

static remote_instance_t remote_instance;


osal_status_t remote_init(void)
{
    memset(&remote_instance, 0, sizeof(remote_instance_t));
    osal_status_t ret = OSAL_SUCCESS;

#if defined(REMOTE_SOURCE)  && REMOTE_SOURCE == 1
    ret = sbus_init(&remote_instance.sbus_instance);
    if (ret == OSAL_SUCCESS) {
        remote_instance.remote_offline_index = remote_instance.sbus_instance.offline_index;
    } else {
        ret = OSAL_ERROR;
    }
#elif defined(REMOTE_SOURCE)  && REMOTE_SOURCE == 2
    ret = dt7_init(&remote_instance.dt7_instance);
    if (ret == OSAL_SUCCESS) {
        remote_instance.remote_offline_index = remote_instance.dt7_instance.offline_index;
    } else {
        ret = OSAL_ERROR;
    }
#endif

#if defined(REMOTE_VT_SOURCE) && REMOTE_VT_SOURCE == 1
    ret = vt02_init(&remote_instance.vt02_instance);
    if (ret == OSAL_SUCCESS) {
        remote_instance.vt_offline_index = remote_instance.vt02_instance.offline_index;
    } else {
        ret = OSAL_ERROR;
    }
#elif defined(REMOTE_VT_SOURCE) && REMOTE_VT_SOURCE == 2
    ret = vt03_init(&remote_instance.vt03_instance);
    if (ret == OSAL_SUCCESS) {
        remote_instance.vt_offline_index = remote_instance.vt03_instance.offline_index;
    } else {
        ret = OSAL_ERROR;
    }
#endif

    if (ret == OSAL_SUCCESS && REMOTE_SOURCE !=0 && REMOTE_VT_SOURCE !=0)
    {
        remote_instance.initflag = 1;
        LOG_INFO("remote init success");
    }
    return ret;
}

void remote_task_function(void)
{
    // 根据遥控器类型读取数据
#if defined(REMOTE_SOURCE) && REMOTE_SOURCE == 1
    // SBUS遥控器数据读取
    if (remote_instance.sbus_instance.uart_device != NULL) {
        uint8_t *data = BSP_UART_Read(remote_instance.sbus_instance.uart_device);
        if (data != NULL) {
            sbus_decode(&remote_instance.sbus_instance, data);
        }
    }
#elif defined(REMOTE_SOURCE) && REMOTE_SOURCE == 2
    // DT7遥控器数据读取
    if (remote_instance.dt7_instance.uart_device != NULL) {
        uint8_t *data = BSP_UART_Read(remote_instance.dt7_instance.uart_device);
        if (data != NULL) {
            dt7_decode(&remote_instance.dt7_instance, data);
        }
    }
#endif
}


void remote_vt_task_function(void)
{ 
    // 根据图传遥控器类型读取数据
#if defined(REMOTE_VT_SOURCE) && REMOTE_VT_SOURCE == 1
    // VT02图传遥控器数据读取
    if (remote_instance.vt02_instance.uart_device != NULL) {
        uint8_t *data = BSP_UART_Read(remote_instance.vt02_instance.uart_device);
        if (data != NULL) {
            vt02_decode(&remote_instance.vt02_instance, data);
        }
    }
#elif defined(REMOTE_VT_SOURCE) && REMOTE_VT_SOURCE == 2
    // VT03图传遥控器数据读取
    if (remote_instance.vt03_instance.uart_device != NULL) {
        uint8_t *data = BSP_UART_Read(remote_instance.vt03_instance.uart_device);
        if (data != NULL) {
            vt03_decode(&remote_instance.vt03_instance, data);
        }
    }
#endif
}



uint8_t get_remote_channel_state(remote_instance_t *remote_instance, uint8_t channel_index, uint8_t is_vt_remote)
{
    if (remote_instance == NULL || remote_instance->initflag !=1) {
        return channel_none;
    }

    if (is_vt_remote)
    {
#if defined(REMOTE_VT_SOURCE) && REMOTE_VT_SOURCE == 2
        switch (channel_index) {
            case 1:  
                return remote_instance->vt03_instance.vt03_remote_data.button_state.switch_pos;
            case 2:  
                return remote_instance->vt03_instance.vt03_remote_data.button_state.custom_left;
            case 3:  
                return remote_instance->vt03_instance.vt03_remote_data.button_state.custom_right;
            case 4:  
                return remote_instance->vt03_instance.vt03_remote_data.button_state.pause;
            case 5:
                return remote_instance->vt03_instance.vt03_remote_data.button_state.trigger;
            default:
                return channel_none;
        }
#endif 
        return channel_none;
    }else{
#if defined(REMOTE_SOURCE) && REMOTE_SOURCE == 1
    return get_sbus_channel_state(&remote_instance->sbus_instance, channel_index);
#elif defined(REMOTE_SOURCE) && REMOTE_SOURCE == 2
    return get_dt7_sw_state(&remote_instance->dt7_instance, channel_index);
#else  
    return channel_none;
#endif
    }
}

mouse_state_t* get_remote_mouse_state(remote_instance_t *remote_instance, uint8_t is_vt_remote)
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

keyboard_state_t * get_remote_keyboard_state(remote_instance_t *remote_instance,uint8_t is_vt_remote)
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

remote_instance_t* get_remote_instance(void)
{
    return &remote_instance;
}
#else  
osal_status_t remote_init(void){return OSAL_SUCCESS;}
uint8_t get_remote_channel_state(remote_instance_t *remote_instance, uint8_t channel_index, uint8_t is_vt_remote){
    return channel_none;
}
mouse_state_t* get_remote_mouse(remote_instance_t *remote_instance, uint8_t is_vt_remote){
    return NULL;
}
keyboard_state_t * get_remote_keyboard_state(remote_instance_t *remote_instance,uint8_t is_vt_remote){
    return NULL;
}
void remote_task_function(void){}
void remote_vt_task_function(void){}

remote_instance_t* get_remote_instance(void){return NULL;}
#endif

