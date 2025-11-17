/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-10-20 22:19:33
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-10-23 00:06:07
 * @FilePath: \rm_base\modules\board_comm\board_comm.h
 * @Description:
 */
#ifndef _BOARD_COMM_H_
#define _BOARD_COMM_H_

#include "bsp_can.h"
#include "osal_def.h"
#include "robot_config.h"
#include "robot_def.h"
#include <stdint.h>

// 板间通信结构体
typedef struct
{
    Can_Device *candevice;
    uint8_t     offlinemanage_index;
} board_comm_t;

// 函数声明
/**
 * @description: 初始化板间通信模块
 * @param {board_comm_t} *board_comm, 板间通信结构体指针
 * @return {osal_status_t}, OSAL_SUCCESS 成功, OSAL_ERROR 失败
 */
osal_status_t board_com_init(board_comm_t *board_comm);
#ifdef GIMBAL_BOARD
/**
 * @brief 发送底盘控制命令
 * @param cmd 待发送的命令结构体指针
 * @return OSAL_SUCCESS 成功, OSAL_ERROR 失败
 */
osal_status_t board_com_send(const Chassis_Ctrl_Cmd_s *cmd);
/**
 * @brief 接收底盘上传数据
 * @param data 解码后存储数据的结构体指针
 * @return OSAL_SUCCESS 成功, OSAL_ERROR 失败
 */
osal_status_t board_com_recv(Chassis_Upload_Data_s *data);
#elif defined(CHASSIS_BOARD)
/**
 * @brief 接收并解码底盘控制命令
 * @param cmd 解码后存储命令的结构体指针
 * @return OSAL_SUCCESS 成功, OSAL_ERROR 失败
 */
osal_status_t board_com_recv(Chassis_Ctrl_Cmd_s *cmd);
/**
 * @brief 发送底盘上传数据
 * @param data 待发送的数据结构体指针
 * @return OSAL_SUCCESS 成功, OSAL_ERROR 失败
 */
osal_status_t board_com_send(const Chassis_Upload_Data_s *data);
#endif

#endif // _BOARD_COMM_H_