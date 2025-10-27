/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-10-26 15:51:58
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-10-27 00:00:09
 * @FilePath: \rm_base\applications\robot_control_task\robot_control_fun.h
 * @Description: 
 */
#ifndef _ROBOT_CONTROL_FUN_H_
#define _ROBOT_CONTROL_FUN_H_

#include "robot_def.h"


float CalcOffsetAngle(float getyawangle);
void RemoteControlSet(Chassis_Ctrl_Cmd_s *Chassis_Ctrl, Shoot_Ctrl_Cmd_s *Shoot_Ctrl, Gimbal_Ctrl_Cmd_s *Gimbal_Ctrl);

#endif // _ROBOT_CONTROL_FUN_H_