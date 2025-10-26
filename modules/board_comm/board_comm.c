/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-10-20 22:19:25
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-10-26 12:49:39
 * @FilePath: \rm_base\modules\board_comm\board_comm.c
 * @Description: 
 */
#include "board_comm.h"
#include "bsp_can.h"
#include "modules_config.h"
#include "offline.h"
#include "osal_def.h"
#include "robot_config.h"
#include "robot_def.h"
#include <stdint.h>
#include <string.h>

static board_comm_t *board_comm_instance = NULL;

osal_status_t board_com_init(board_comm_t *board_comm){
    if (board_comm != NULL)
    {
#ifdef ONE_BOARD
        return OSAL_ERROR;
#elif defined(GIMBAL_BOARD)
        board_comm_instance = board_comm;
        static Can_Device_Init_Config_s board_com_init = {
            .can_handle = &hcan2,
            .rx_id = CHASSIS_ID,
            .tx_id = GIMBAL_ID,
            .rx_mode = CAN_MODE_IT,
            .tx_mode = CAN_MODE_BLOCKING,
        };
        board_comm->candevice = BSP_CAN_Device_Init(&board_com_init);
        if (board_comm->candevice ==NULL)
        {
            return OSAL_ERROR;
        }
        static OfflineDeviceInit_t offline_manage_init = {
            .name = "board_comm",
            .timeout_ms = 100,
            .level = OFFLINE_LEVEL_HIGH,
            .beep_times = 7,
            .enable = OFFLINE_BEEP_ENABLE
        };
        board_comm->offlinemanage_index = offline_module_device_register(&offline_manage_init);
        if (board_comm->offlinemanage_index == OFFLINE_INVALID_INDEX)
        {
            return OSAL_ERROR;
        }
#elif defined(CHASSIS_BOARD)
        board_comm_instance = board_comm;
        static Can_Device_Init_Config_s board_com_init = {
            .can_handle = &hcan2,
            .rx_id = GIMBAL_ID,
            .tx_id = CHASSIS_ID,
            .rx_mode = CAN_MODE_IT,
            .tx_mode = CAN_MODE_BLOCKING,
        };
        board_comm->candevice = BSP_CAN_Device_Init(&board_com_init);
        if (board_comm->candevice ==NULL)
        {
            return OSAL_ERROR;
        }
        static OfflineDeviceInit_t offline_manage_init = {
            .name = "board_comm",
            .timeout_ms = 100,
            .level = OFFLINE_LEVEL_HIGH,
            .beep_times = 7,
            .enable = OFFLINE_BEEP_ENABLE
        };
        board_comm->offlinemanage_index = offline_module_device_register(&offline_manage_init);
        if (board_comm->offlinemanage_index == OFFLINE_INVALID_INDEX)
        {
            return OSAL_ERROR;
        }
#endif 
    }
    return OSAL_SUCCESS;
}

#ifdef GIMBAL_BOARD
osal_status_t board_com_send(const Chassis_Ctrl_Cmd_s *cmd)
{
    // 参数检查
    if (cmd == NULL || board_comm_instance == NULL) {
        return OSAL_ERROR;
    }
    
    // 编码数据
    int16_t angle_int = (int16_t)cmd->offset_angle;
    board_comm_instance->candevice->tx_buff[0] = (int8_t)cmd->vx;
    board_comm_instance->candevice->tx_buff[1] = (int8_t)cmd->vy;
    board_comm_instance->candevice->tx_buff[2] = (int8_t)cmd->wz;
    board_comm_instance->candevice->tx_buff[3] = (angle_int >> 8) & 0xFF;
    board_comm_instance->candevice->tx_buff[4] = angle_int & 0xFF;
    board_comm_instance->candevice->tx_buff[5] = (uint8_t)cmd->chassis_mode;
    board_comm_instance->candevice->tx_buff[6] = cmd->reserved_1;
    board_comm_instance->candevice->tx_buff[7] = cmd->reserved_2;

    // 发送数据
    osal_status_t status = OSAL_SUCCESS;
    status = BSP_CAN_SendDevice(board_comm_instance->candevice);
    return status;
} 
osal_status_t board_com_recv(Chassis_Upload_Data_s *data)
{
    // 参数检查
    if (board_comm_instance == NULL || data == NULL) {
        return OSAL_ERROR;
    }
    
    // 读取CAN数据
    if (BSP_CAN_ReadSingleDevice(board_comm_instance->candevice, OSAL_WAIT_FOREVER) == OSAL_SUCCESS) {
        uint8_t* rx_data = board_comm_instance->candevice->rx_buff;
        
        // 拷贝数据
        memcpy(data, rx_data, sizeof(Chassis_Upload_Data_s));
        return OSAL_SUCCESS;
    }
    
    return OSAL_ERROR;
}
#endif

#ifdef CHASSIS_BOARD
osal_status_t board_com_recv(Chassis_Ctrl_Cmd_s *cmd)
{
    // 参数检查
    if (board_comm_instance == NULL || cmd == NULL) {
        return OSAL_ERROR;
    }
    
    // 读取CAN数据
    if (BSP_CAN_ReadSingleDevice(board_comm_instance->candevice, OSAL_NO_WAIT) == OSAL_SUCCESS) {
        uint8_t* data = board_comm_instance->candevice->rx_buff;
        
        // 解析数据
        cmd->vx = (float)((int8_t)data[0]);
        cmd->vy = (float)((int8_t)data[1]);
        cmd->wz = (float)((int8_t)data[2]);
        // offset_angle 由两个字节组合而成
        int16_t angle_int = ((int16_t)data[3] << 8) | data[4];
        cmd->offset_angle = (float)angle_int;
        // 剩余数据直接赋值
        cmd->chassis_mode = (chassis_mode_e)data[3];
        cmd->reserved_1 = data[4];
        cmd->reserved_2 = data[5];
        return OSAL_SUCCESS;
    }
    
    return OSAL_ERROR;
}
osal_status_t board_com_send(const Chassis_Upload_Data_s *data)
{
    // 参数检查
    if (data == NULL || board_comm_instance == NULL) {
        return OSAL_ERROR;
    }
    
    // 拷贝数据到发送缓冲区
    memcpy(board_comm_instance->candevice->tx_buff, data, sizeof(Chassis_Upload_Data_s));
    
    // 发送数据
    osal_status_t status = OSAL_SUCCESS;
    status = BSP_CAN_SendDevice(board_comm_instance->candevice);
    return status;
}
#endif