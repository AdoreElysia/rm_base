/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-10-26 15:51:20
 * @LastEditors: laladuduqq 17503181697@163.com
 * @LastEditTime: 2025-11-17 12:04:44
 * @FilePath: /rm_base/applications/robot_control_task/robot_control_fun.c
 * @Description:
 */
#include "robot_control_fun.h"
#include "dt7.h"
#include "modules_config.h"
#include "offline.h"
#include "remote.h"
#include "remote_data.h"
#include "robot_config.h"
#include "robot_def.h"
#include "sbus.h"
#include "user_lib.h"
#include <stdint.h>
#include <string.h>

float CalcOffsetAngle(float getyawangle)
{
    static float offsetangle;
    // 从云台获取的当前yaw电机单圈角度
#if YAW_ECD_GREATER_THAN_4096 // 如果大于180度
    if (getyawangle > YAW_ALIGN_ANGLE && getyawangle <= 180.0f + YAW_ALIGN_ANGLE)
    {
        offsetangle = getyawangle - YAW_ALIGN_ANGLE;
        return offsetangle;
    }
    else if (getyawangle > 180.0f + YAW_ALIGN_ANGLE)
    {
        offsetangle = getyawangle - YAW_ALIGN_ANGLE - 360.0f;
        return offsetangle;
    }
    else
    {
        offsetangle = getyawangle - YAW_ALIGN_ANGLE;
        return offsetangle;
    }
#else // 小于180度
    if (getyawangle > YAW_ALIGN_ANGLE)
    {
        offsetangle = getyawangle - YAW_ALIGN_ANGLE;
        return offsetangle;
    }
    else if (getyawangle <= YAW_ALIGN_ANGLE && getyawangle >= YAW_ALIGN_ANGLE - 180.0f)
    {
        offsetangle = getyawangle - YAW_ALIGN_ANGLE;
        return offsetangle;
    }
    else
    {
        offsetangle = getyawangle - YAW_ALIGN_ANGLE + 360.0f;
        return offsetangle;
    }
#endif
}

void RemoteControlSet(Chassis_Ctrl_Cmd_s *Chassis_Ctrl, Shoot_Ctrl_Cmd_s *Shoot_Ctrl,
                      Gimbal_Ctrl_Cmd_s *Gimbal_Ctrl)
{
    // 检查参数
    if (Chassis_Ctrl == NULL || Shoot_Ctrl == NULL || Gimbal_Ctrl == NULL)
    {
        return;
    }

#if defined(REMOTE_MODULE) && defined(REMOTE_SOURCE)
// 根据遥控器类型处理控制逻辑
#if REMOTE_SOURCE == 1 // SBUS遥控器
    {
        // 获取SBUS遥控器数据
        uint8_t remote_status = remote_device_status(0);
        if (remote_status == STATE_ONLINE)
        {
            // 摇杆控制底盘移动
            Chassis_Ctrl->vx = -1.0f * get_remote_channel(2, 0);
            Chassis_Ctrl->vy = 1.0f * get_remote_channel(1, 0);

            // 云台控制部分
            int16_t ch6_state = get_remote_channel(6, 0);
            if (ch6_state == SBUS_CHX_UP)
            {
                Gimbal_Ctrl->gimbal_mode = GIMBAL_GYRO_MODE;
                Gimbal_Ctrl->yaw -= 0.001f * (float)(get_remote_channel(4, 0));
                Gimbal_Ctrl->pitch += 0.001f * (float)(get_remote_channel(3, 0));
                VAL_LIMIT(Gimbal_Ctrl->pitch, SMALL_YAW_PITCH_MIN_ANGLE, SMALL_YAW_PITCH_MAX_ANGLE);
            }
            else if (ch6_state == SBUS_CHX_BIAS)
            {
                Gimbal_Ctrl->gimbal_mode = GIMBAL_AUTO_MODE;
            }
            else if (ch6_state == SBUS_CHX_DOWN)
            {
                Gimbal_Ctrl->gimbal_mode = GIMBAL_ZERO_FORCE;
            }

            // 发射机构控制部分
            int16_t ch5_state = get_remote_channel(5, 0);
            if (ch5_state == SBUS_CHX_UP)
            {
                Shoot_Ctrl->shoot_mode    = SHOOT_OFF;
                Shoot_Ctrl->friction_mode = FRICTION_OFF;
                Shoot_Ctrl->load_mode     = LOAD_STOP;
            }
            else if (ch5_state == SBUS_CHX_BIAS)
            {
                Shoot_Ctrl->shoot_mode    = SHOOT_ON;
                Shoot_Ctrl->friction_mode = FRICTION_OFF;
                Shoot_Ctrl->load_mode     = LOAD_STOP;
            }
            else if (ch5_state == SBUS_CHX_DOWN)
            {
                Shoot_Ctrl->shoot_mode    = SHOOT_ON;
                Shoot_Ctrl->friction_mode = FRICTION_ON;

                int16_t ch7_state = get_remote_channel(7, 0);
                if (ch7_state == SBUS_CHX_BIAS)
                {
                    Shoot_Ctrl->load_mode = LOAD_STOP;
                }
                else if (ch7_state == SBUS_CHX_UP)
                {
                    Shoot_Ctrl->load_mode = LOAD_1_BULLET;
                }
                else if (ch7_state == SBUS_CHX_DOWN)
                {
                    Shoot_Ctrl->load_mode = LOAD_BURSTFIRE;
                }
            }

            // 底盘控制部分
            int16_t ch8_state = get_remote_channel(8, 0);
            if (ch8_state == SBUS_CHX_UP)
            {
                Chassis_Ctrl->chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
            }
            else if (ch8_state == SBUS_CHX_BIAS)
            {
                Chassis_Ctrl->chassis_mode = CHASSIS_ROTATE;
            }
            else if (ch8_state == SBUS_CHX_DOWN)
            {
                Chassis_Ctrl->chassis_mode = CHASSIS_ROTATE_REVERSE;
            }
        }
        else
        {
            // 遥控器离线处理
            Gimbal_Ctrl->gimbal_mode   = GIMBAL_ZERO_FORCE;
            Chassis_Ctrl->chassis_mode = CHASSIS_ZERO_FORCE;
            Shoot_Ctrl->shoot_mode     = SHOOT_OFF;
            Shoot_Ctrl->friction_mode  = FRICTION_OFF;
            Shoot_Ctrl->load_mode      = LOAD_STOP;
            memset(Chassis_Ctrl, 0, sizeof(Chassis_Ctrl_Cmd_s));
        }
    }
#elif REMOTE_SOURCE == 2 // DT7遥控器
    {
        // 获取DT7遥控器数据
        uint8_t remote_status = remote_device_status(0);
        if (remote_status == STATE_ONLINE)
        {
            // 摇杆控制底盘移动
            Chassis_Ctrl->vx = -1.0f * get_remote_channel(2, 0);
            Chassis_Ctrl->vy = 1.0f * get_remote_channel(1, 0);

            // 云台控制部分
            uint8_t sw1_state = get_remote_channel(5, 0);
            if (sw1_state == DT7_SW_UP)
            {
                Gimbal_Ctrl->gimbal_mode = GIMBAL_GYRO_MODE;
                Gimbal_Ctrl->yaw -= 0.001f * (float)(get_remote_channel(3, 0));
                Gimbal_Ctrl->pitch += 0.001f * (float)(get_remote_channel(4, 0));
                VAL_LIMIT(Gimbal_Ctrl->pitch, SMALL_YAW_PITCH_MIN_ANGLE, SMALL_YAW_PITCH_MAX_ANGLE);
            }
            else if (sw1_state == DT7_SW_DOWN)
            {
                Gimbal_Ctrl->gimbal_mode = GIMBAL_AUTO_MODE;
            }

            // 发射机构控制部分
            if (sw1_state == DT7_SW_UP)
            {
                Shoot_Ctrl->shoot_mode    = SHOOT_ON;
                Shoot_Ctrl->friction_mode = FRICTION_OFF;
                Shoot_Ctrl->load_mode     = LOAD_STOP;
            }
            else if (sw1_state == DT7_SW_MID)
            {
                Shoot_Ctrl->shoot_mode    = SHOOT_ON;
                Shoot_Ctrl->friction_mode = FRICTION_ON;

                int16_t wheel_value = get_remote_channel(7, 0); // 获取wheel值
                if (wheel_value == 0)
                {
                    Shoot_Ctrl->load_mode = LOAD_STOP;
                }
                else if (wheel_value > 0)
                {
                    Shoot_Ctrl->load_mode = LOAD_REVERSE;
                }
                else if (wheel_value < 0)
                {
                    Shoot_Ctrl->load_mode = LOAD_BURSTFIRE;
                }
            }

            // 底盘控制部分
            uint8_t sw2_state = get_remote_channel(6, 0);
            if (sw2_state == DT7_SW_UP)
            {
                Chassis_Ctrl->chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
            }
            else if (sw2_state == DT7_SW_MID)
            {
                Chassis_Ctrl->chassis_mode = CHASSIS_ROTATE;
            }
            else if (sw2_state == DT7_SW_DOWN)
            {
                Chassis_Ctrl->chassis_mode = CHASSIS_ROTATE_REVERSE;
            }
        }
        else
        {
            // 遥控器离线处理
            Gimbal_Ctrl->gimbal_mode   = GIMBAL_ZERO_FORCE;
            Chassis_Ctrl->chassis_mode = CHASSIS_ZERO_FORCE;
            Shoot_Ctrl->shoot_mode     = SHOOT_OFF;
            Shoot_Ctrl->friction_mode  = FRICTION_OFF;
            Shoot_Ctrl->load_mode      = LOAD_STOP;
            memset(Chassis_Ctrl, 0, sizeof(Chassis_Ctrl_Cmd_s));
        }
    }
#endif
#endif

#if defined(REMOTE_MODULE) && defined(REMOTE_VT_SOURCE)
// 图传遥控器处理
#if REMOTE_VT_SOURCE == 2 // VT03图传遥控器
    {
        static button_state_t button_state, last_button_state;
        static uint8_t        trigger_state = 0; // 0: 关闭状态, 1: 开启状态
        static uint8_t chassis_mode_state = 0; // 0: FOLLOW_GIMBAL_YAW, 1: ROTATE, 2: ROTATE_REVERSE
        uint8_t        vt_status          = remote_device_status(1);
        if (vt_status == STATE_ONLINE)
        {
            // 摇杆控制底盘移动
            Chassis_Ctrl->vx = -1.0f * get_remote_channel(2, 1);
            Chassis_Ctrl->vy = 1.0f * get_remote_channel(1, 1);

            button_state = *get_remote_button_state();

            // 云台控制部分
            uint8_t switch_pos = button_state.switch_pos;
            if (switch_pos == 0)
            {
                Gimbal_Ctrl->gimbal_mode = GIMBAL_GYRO_MODE;
                Gimbal_Ctrl->yaw -= 0.001f * (float)(get_remote_channel(4, 1));
                Gimbal_Ctrl->pitch += 0.001f * (float)(get_remote_channel(3, 1));
                VAL_LIMIT(Gimbal_Ctrl->pitch, SMALL_YAW_PITCH_MIN_ANGLE, SMALL_YAW_PITCH_MAX_ANGLE);
            }
            else if (switch_pos == 1)
            {
                Gimbal_Ctrl->gimbal_mode = GIMBAL_AUTO_MODE;
            }
            else if (switch_pos == 2)
            {
                Gimbal_Ctrl->gimbal_mode = GIMBAL_ZERO_FORCE;
            }

            // 发射机构控制部分，实现按钮切换功能
            // 只有当按钮从松开(0)变为按下(1)时才切换状态
            if (button_state.trigger == 1 && last_button_state.trigger == 0)
            {
                trigger_state = !trigger_state; // 切换状态

                if (trigger_state)
                { // 开启状态
                    Shoot_Ctrl->shoot_mode    = SHOOT_ON;
                    Shoot_Ctrl->friction_mode = FRICTION_ON;

                    int16_t wheel_value = get_remote_channel(5, 1);
                    if (wheel_value == 0)
                    {
                        Shoot_Ctrl->load_mode = LOAD_STOP;
                    }
                    else if (wheel_value > 0)
                    {
                        Shoot_Ctrl->load_mode = LOAD_REVERSE;
                    }
                    else if (wheel_value < 0)
                    {
                        Shoot_Ctrl->load_mode = LOAD_BURSTFIRE;
                    }
                }
                else
                { // 关闭状态
                    Shoot_Ctrl->shoot_mode    = SHOOT_ON;
                    Shoot_Ctrl->friction_mode = FRICTION_OFF;
                    Shoot_Ctrl->load_mode     = LOAD_STOP;
                }
            }

            // 底盘控制部分，实现按钮切换功能
            // 只有当custom_right按钮从松开(0)变为按下(1)时才切换模式
            if (button_state.custom_right == 1 && last_button_state.custom_right == 0)
            {
                chassis_mode_state = (chassis_mode_state + 1) % 3; // 循环切换三种模式

                switch (chassis_mode_state)
                {
                case 0:
                    Chassis_Ctrl->chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
                    break;
                case 1:
                    Chassis_Ctrl->chassis_mode = CHASSIS_ROTATE;
                    break;
                case 2:
                    Chassis_Ctrl->chassis_mode = CHASSIS_ROTATE_REVERSE;
                    break;
                }
            }

            // 更新上一次按钮状态
            last_button_state = button_state;
        }
        else
        {
            // 图传遥控器离线处理
            Gimbal_Ctrl->gimbal_mode   = GIMBAL_ZERO_FORCE;
            Chassis_Ctrl->chassis_mode = CHASSIS_ZERO_FORCE;
            Shoot_Ctrl->shoot_mode     = SHOOT_OFF;
            Shoot_Ctrl->friction_mode  = FRICTION_OFF;
            Shoot_Ctrl->load_mode      = LOAD_STOP;
            memset(Chassis_Ctrl, 0, sizeof(Chassis_Ctrl_Cmd_s));
        }
    }
#endif
#endif
}
