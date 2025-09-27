/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-09-13 10:14:45
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-09-26 23:06:36
 * @FilePath: /rm_base/applications/robot_init/robot_init.c
 * @Description: 
 */
#include "robot_init.h"
#include "bsp_dwt.h"
#include "dm_imu.h"
#include "dm_imu_task.h"
#include "ins.h"
#include "offline.h"
#include "offline_task.h"
#include "remote.h"
#include "remote_task.h"
#include "shell.h"
#include "rgb.h"
#include "ins_task.h"
#include "shell_task.h"


void bsp_init()
{
  DWT_Init(168);
  RGB_init();
}

void app_init(){
  shell_task_init();
  offline_task_init();
  ins_init();
  remote_init();
  dm_imu_init();
  ins_task_init();
  remote_task_init();
  dm_imu_task_init();
}


void robot_init()
{
  bsp_init();
  app_init();
}
