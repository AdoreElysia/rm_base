/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-10-24 10:28:29
 * @LastEditors: laladuduqq 17503181697@163.com
 * @LastEditTime: 2025-11-17 12:04:11
 * @FilePath: /rm_base/applications/robot_control_task/robot_control_task.c
 * @Description:
 */
#include "robot_control_task.h"
#include "app_config.h"
#include "board_comm.h"
#include "dji.h"
#include "itc.h"
#include "modules_config.h"
#include "motor_task.h"
#include "osal_def.h"
#include "robot_config.h"
#include "robot_control_fun.h"
#include "robot_def.h"
#include "tx_port.h"
#include <stdint.h>
#include <string.h>

#define log_tag "robot_control"
#include "shell_log.h"

static osal_thread_t robot_control_thread;
ROBOT_CONTROL_THREAD_STACK_SECTION static uint8_t
    robot_control_thread_stack[ROBOT_CONTROL_THREAD_STACK_SIZE];

#if defined(ONE_BOARD)
#include "usb_user.h"
// 虚拟串口部分
static user_cdc_t         user_cdc;
static struct User_Send_s user_send;
static struct User_Recv_s user_recv;
// 云台发射机构底盘命令发布
static itc_topic_t        gimbal_cmd_topic;
static Gimbal_Ctrl_Cmd_s  gimbal_cmd;
static itc_topic_t        shoot_cmd_topic;
static Shoot_Ctrl_Cmd_s   shoot_cmd;
static itc_topic_t        chassis_cmd_topic;
static Chassis_Ctrl_Cmd_s chassis_cmd;
#elif defined(GIMBAL_BOARD)
#include "usb_user.h"
// 虚拟串口部分
static user_cdc_t         user_cdc;
static struct User_Send_s user_send;
static struct User_Recv_s user_recv;
// 云台与发射机构命令发布
static itc_topic_t       gimbal_cmd_topic;
static Gimbal_Ctrl_Cmd_s gimbal_cmd;
static itc_topic_t       shoot_cmd_topic;
static Shoot_Ctrl_Cmd_s  shoot_cmd;
// 板间通讯部分
static board_comm_t          board_comm;
static Chassis_Ctrl_Cmd_s    chassis_cmd;
static Chassis_Upload_Data_s chassis_data;
static DJIMotor_t           *yaw_motor; // 云台yaw电机指针,用于获取yaw角度，不会初始化
#elif defined(CHASSIS_BOARD)
// 板间通讯部分
static board_comm_t board_comm;
// chassis命令接收与上传
static itc_topic_t           chassis_cmd_topic;
static Chassis_Ctrl_Cmd_s    chassis_cmd;
static Chassis_Upload_Data_s chassis_data;
#else
#error "Please define board type in robot_config.h!"
#endif
void robot_control_init(void)
{
#if defined(ONE_BOARD)

#elif defined(GIMBAL_BOARD)
    osal_status_t status;
    // 虚拟串口初始化
    status = usb_user_init(&user_cdc);
    if (status != OSAL_SUCCESS)
    {
        LOG_ERROR("usb user init failed!");
        return;
    }
    // gimbal shoot topic 初始化
    status = itc_topic_init(&gimbal_cmd_topic, "gimbal_cmd_topic", sizeof(Gimbal_Ctrl_Cmd_s));
    if (status != OSAL_SUCCESS)
    {
        LOG_ERROR("gimbal topic init failed!");
        return;
    }
    status = itc_topic_init(&shoot_cmd_topic, "shoot_cmd_topic", sizeof(Shoot_Ctrl_Cmd_s));
    if (status != OSAL_SUCCESS)
    {
        LOG_ERROR("shoot topic init failed!");
        return;
    }
    // 板间通讯初始化
    status = board_com_init(&board_comm);
    if (status != OSAL_SUCCESS)
    {
        LOG_ERROR("board comm init failed!");
        return;
    }
#elif defined(CHASSIS_BOARD)
    osal_status_t status;
    // 板间通讯初始化
    status = board_com_init(&board_comm);
    if (status != OSAL_SUCCESS)
    {
        LOG_ERROR("board comm init failed!");
        return;
    }
    // chassis cmd topic 初始化
    status = itc_topic_init(&chassis_cmd_topic, "chassis_cmd_topic", sizeof(Chassis_Ctrl_Cmd_s));
    if (status != OSAL_SUCCESS)
    {
        LOG_ERROR("chassis topic init failed!");
        return;
    }
#else
#error "Please define board type in robot_config.h!"
#endif
}

void robot_control_task(ULONG thread_input)
{
    while (1)
    {
        osal_status_t status;
#if defined(ONE_BOARD)

#elif defined(GIMBAL_BOARD)
        // 获取遥控器控制命令
        RemoteControlSet(&chassis_cmd, &shoot_cmd, &gimbal_cmd);
        // gimbal shoot命令发布
        itc_publish(&gimbal_cmd_topic, &gimbal_cmd);
        itc_publish(&shoot_cmd_topic, &shoot_cmd);
        yaw_motor = get_motor_ptr(MOTOR_DJI, 0); // 获取yaw电机指针;
        // 板间通讯处理
        if (yaw_motor != NULL)
        {
            chassis_cmd.offset_angle = CalcOffsetAngle(yaw_motor->measure.angle_single_round);
        }
        else
        {
            chassis_cmd.offset_angle = 0;
        }
        board_com_send(&chassis_cmd);
        // 接收chassis上传数据
        status = board_com_recv(&chassis_data);
        if (status == OSAL_SUCCESS)
        {
            usb_user_send(&user_send);
        }
        // 虚拟串口接收数据
        // user_recv = *usb_user_recv();
#elif defined(CHASSIS_BOARD)
        // 接收chassis命令
        if (offline_module_get_device_status(board_comm.offlinemanage_index) == STATE_ONLINE)
        {
            status = board_com_recv(&chassis_cmd);
            if (status != OSAL_SUCCESS)
            {
                memset(&chassis_cmd, 0, sizeof(Chassis_Ctrl_Cmd_s));
            }
        }
        else
        {
            memset(&chassis_cmd, 0, sizeof(Chassis_Ctrl_Cmd_s));
        }
        // chassis cmd topic 发布
        itc_publish(&chassis_cmd_topic, &chassis_cmd);
        // 板间通讯处理
        board_com_send(&chassis_data);
#else
#error "Please define board type in robot_config.h!"
#endif
        osal_delay_ms(2);
    }
}

void robot_control_task_init(void)
{
    osal_status_t status;
    robot_control_init();
    status = osal_thread_create(&robot_control_thread, "robot_control_thread", robot_control_task,
                                0, robot_control_thread_stack, ROBOT_CONTROL_THREAD_STACK_SIZE,
                                ROBOT_CONTROL_THREAD_PRIORITY);
    if (status != OSAL_SUCCESS)
    {
        LOG_ERROR("robot_control_task_init failed!");
        return;
    }
    osal_thread_start(&robot_control_thread);
    LOG_INFO("robot_control_task_init success!");
}
