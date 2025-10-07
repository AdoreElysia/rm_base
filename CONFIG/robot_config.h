/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-09-17 10:52:48
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-10-07 20:42:32
 * @FilePath: \rm_base\CONFIG\robot_config.h
 * @Description: 
 */
#ifndef _ROBOT_CONFIG_H_
#define _ROBOT_CONFIG_H_

/* 电机声明 */
#define MOTOR_NONE                        0
#define MOTOR_DJI                         (1 << 0)
#define MOTOR_DM                          (1 << 1)
#define MOTOR_BENMO                       (1 << 2)


/* 机器人通讯定义*/

//#define ONE_BOARD // 单板控制

#ifndef ONE_BOARD // 多板控制 （注意只能有一个生效）
        //#define CHASSIS_BOARD //底盘板
        #define GIMBAL_BOARD  //云台板
        // 检查是否出现主控板定义冲突,只允许一个开发板定义存在,否则编译会自动报错
        #if (defined(CHASSIS_BOARD) + defined(GIMBAL_BOARD)!=1)
        #error Conflict board definition! You can only define one board type.
        #endif
#endif

/*  机器人使用模块声明 */
#ifdef ONE_BOARD
#define RGB_MODULE
#define INS_MODULE   
#define OFFLINE_MODULE
#define BEEP_MODULE
#define REMOTE_MODULE
#define SHELL_MODULE
#define LOG_MODULE
#define MOTOR_MODULE_USE                  (MOTOR_DJI | MOTOR_DM)
#else
    #ifdef GIMBAL_BOARD
        #define RGB_MODULE
        #define INS_MODULE   
        #define OFFLINE_MODULE
        #define BEEP_MODULE
        #define REMOTE_MODULE
        #define DM_IMU_MODULE 
        #define SHELL_MODULE
        #define LOG_MODULE
        #define MOTOR_MODULE_USE                  (MOTOR_DJI | MOTOR_DM)
    #endif 
    #ifdef CHASSIS_BOARD
        #define RGB_MODULE
        #define INS_MODULE  
        #define OFFLINE_MODULE 
        #define BEEP_MODULE
        #define SHELL_MODULE
        #define LOG_MODULE
        #define MOTOR_MODULE_USE                  (MOTOR_DJI | MOTOR_DM)
    #endif 
#endif
/* 可用模块列表
 * #define RGB_MODULE         // RGB灯模块
 * #define IST8310_MODULE     // 磁力计模块
 * #define INS_MODULE         // 姿态解算模块
 * #define BEEP_MODULE        // 蜂鸣器模块
 * #define OFFLINE_MODULE     // 离线检测模块
 * #define REMOTE_MODULE      // 遥控器模块 
 * #define DM_IMU_MODULE      // 达妙IMU模块
 * #define SHELL_MODULE       // shell功能模块
 * #define LOG_MODULE         // 启用日志功能
 * #define MOTOR_MODULE_USE   (MOTOR_DJI | MOTOR_DM)  // 电机模块
*/

/* 机器人关键参数定义 ,(这里参数根据机器人实际自行定义) */  //1 表示开启 0 表示关闭
#define SMALL_YAW_ALIGN_ANGLE                0.0f
#define SMALL_YAW_MIN_ANGLE                  -90.0f
#define SMALL_YAW_MAX_ANGLE                  90.0f
#define SMALL_YAW_PITCH_HORIZON_ANGLE        0.0f       // 云台处于水平位置时编码器值,若对云台有机械改动需要修改
#define SMALL_YAW_PITCH_MAX_ANGLE            20.0f      // 云台竖直方向最大角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
#define SMALL_YAW_PITCH_MIN_ANGLE            -25.0f     // 云台竖直方向最小角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)

#define YAW_CHASSIS_ALIGN_ECD                4088       // 云台和底盘对齐指向相同方向时的电机编码器值,若对云台有机械改动需要修改
#define YAW_ECD_GREATER_THAN_4096            0          // ALIGN_ECD值是否大于4096,是为1,否为0;用于计算云台偏转角度
#define YAW_ALIGN_ANGLE                      (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI)  // 对齐时的角度,0-360
// 发射参数
#define REDUCTION_RATIO_LOADER               36.0f // 2006拨盘电机的减速比,英雄需要修改为3508的19.0f
#define ONE_BULLET_DELTA_ANGLE               60.0f   // 发射一发弹丸拨盘转动的距离,由机械设计图纸给出
#define NUM_PER_CIRCLE                       6             // 拨盘一圈的装载量
// 机器人底盘修改的参数,单位为mm(毫米)
#define CHASSIS_TYPE                         2               // 1 麦克纳姆轮底盘 2 全向轮底盘 3 舵轮底盘 4 平衡底盘
#define WHEEL_R                              500                    //投影点距离地盘中心为r
#define CENTER_GIMBAL_OFFSET_X               0    // 云台旋转中心距底盘几何中心的距离,前后方向,云台位于正中心时默认设为0
#define CENTER_GIMBAL_OFFSET_Y               0    // 云台旋转中心距底盘几何中心的距离,左右方向,云台位于正中心时默认设为0
#define RADIUS_WHEEL                         0.07             // 轮子半径(单位:m)


#endif // _ROBOT_CONFIG_H_