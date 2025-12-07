/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-09-13 10:14:45
 * @LastEditors: Mas sji733055@gmail.com
 * @LastEditTime: 2025-12-07 19:17:48
 * @FilePath: \rm_base\applications\robot_init\robot_init.c
 * @Description:
 */
#include "robot_init.h"
#include "bsp_dwt.h"
#include "dm_imu_task.h"
#include "gimbal_task.h"
#include "gpio.h"
#include "ins_task.h"
#include "itc.h"
#include "motor_task.h"
#include "offline_task.h"
#include "remote_task.h"
#include "rgb.h"
#include "robot_control_task.h"
#include "shell_task.h"

void base_init()
{
    DWT_Init(168);
    RGB_init();
    itc_init();
    motor_list_init();
    // reset cdc_acm
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
    DWT_Delay(0.1);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
    DWT_Delay(0.1);
}

void app_init()
{
    shell_task_init();
    offline_task_init();
    ins_task_init();
    dm_imu_task_init();
    remote_task_init();
    robot_control_task_init();
    gimbal_task_init();
    motor_task_init();

}

void robot_init()
{
    base_init();
    app_init();
}
