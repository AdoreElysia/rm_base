/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-09-13 10:14:45
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-09-14 13:10:43
 * @FilePath: /rm_base/applications/robot_init/robot_init.c
 * @Description: 
 */
#include "robot_init.h"
#include "bsp_dwt.h"
#include "log.h"
#include "offline.h"
#include "offline_task.h"
#include "shell.h"
#include "rgb.h"


void bsp_init()
{
  DWT_Init(168);
  shell_init();
  LOG_INIT();
  RGB_init();
}

void modules_init(){
  offline_init();
}

void app_init(){
  offline_task_init();
}


void robot_init()
{
  bsp_init();
  modules_init();
  app_init();
}
