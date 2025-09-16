#ifndef _ROBOT_CONFIG_H_
#define _ROBOT_CONFIG_H_



#include <stdint.h>
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

#else
#ifdef GIMBAL_BOARD

#endif 
#ifdef CHASSIS_BOARD

#endif 
#endif

/* 机器人关键参数定义 ,注意根据不同机器人进行修改 */  //1 表示开启 0 表示关闭

//裁判系统
#define USING_REFREE_SYSTEM 0
//功率控制开关与超电开关
#define USING_POWER_CTRL 1
#define USING_SUPER_CAP  0

//机器人参数定义(这里参数根据机器人实际自行定义)
#define SMALL_YAW_ALIGN_ANGLE 0.0f
#define SMALL_YAW_MIN_ANGLE -90.0f
#define SMALL_YAW_MAX_ANGLE 90.0f
#define SMALL_YAW_PITCH_HORIZON_ANGLE 0.0f     // 云台处于水平位置时编码器值,若对云台有机械改动需要修改
#define SMALL_YAW_PITCH_MAX_ANGLE 20.0f           // 云台竖直方向最大角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
#define SMALL_YAW_PITCH_MIN_ANGLE -25.0f           // 云台竖直方向最小角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)

#define YAW_CHASSIS_ALIGN_ECD 4088  // 云台和底盘对齐指向相同方向时的电机编码器值,若对云台有机械改动需要修改
#define YAW_ECD_GREATER_THAN_4096 0 // ALIGN_ECD值是否大于4096,是为1,否为0;用于计算云台偏转角度
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
// 发射参数
#define REDUCTION_RATIO_LOADER 36.0f // 2006拨盘电机的减速比,英雄需要修改为3508的19.0f
#define ONE_BULLET_DELTA_ANGLE 60.0f   // 发射一发弹丸拨盘转动的距离,由机械设计图纸给出
#define NUM_PER_CIRCLE 6             // 拨盘一圈的装载量
// 机器人底盘修改的参数,单位为mm(毫米)
#define CHASSIS_TYPE 2               // 1 麦克纳姆轮底盘 2 全向轮底盘 3 舵轮底盘 4 平衡底盘
#define WHEEL_R  500                    //投影点距离地盘中心为r
#define CENTER_GIMBAL_OFFSET_X 0    // 云台旋转中心距底盘几何中心的距离,前后方向,云台位于正中心时默认设为0
#define CENTER_GIMBAL_OFFSET_Y 0    // 云台旋转中心距底盘几何中心的距离,左右方向,云台位于正中心时默认设为0
#define RADIUS_WHEEL 0.07             // 轮子半径(单位:m)

#pragma pack(1)

// 云台模式设置
typedef enum
{
    GIMBAL_ZERO_FORCE = 0, // 电流零输入
    GIMBAL_GYRO_MODE,      // 云台陀螺仪反馈模式,反馈值为陀螺仪pitch,total_yaw_angle,底盘可以为小陀螺和跟随模式
    GIMBAL_KEEPING_BIG_YAW, //云台保持模式，检录和debug用
    GIMBAL_KEEPING_SMALL_YAW, //云台保持模式，检录和debug用
    GIMBAL_AUTO_MODE,  //自瞄导航模式
} gimbal_mode_e;


// cmd发布的云台控制数据,由gimbal订阅
typedef struct
{   // 云台角度控制
    float yaw;
    float small_yaw;
    float pitch;
    gimbal_mode_e gimbal_mode;
} Gimbal_Ctrl_Cmd_s;

typedef struct
{
  float yaw_motor_single_round_angle;
} Gimbal_Upload_Data_s;


// 发射模式设置
typedef enum
{
    SHOOT_OFF = 0,
    SHOOT_ON,
} shoot_mode_e;
typedef enum
{
    FRICTION_OFF = 0, // 摩擦轮关闭
    FRICTION_ON,      // 摩擦轮开启
} friction_mode_e;
typedef enum
{
    LOAD_STOP = 0,  // 停止发射
    LOAD_REVERSE,   // 反转
    LOAD_1_BULLET,  // 单发
    LOAD_3_BULLET,  // 三发
    LOAD_BURSTFIRE, // 连发
} loader_mode_e;

// cmd发布的发射控制数据,由shoot订阅
typedef struct
{
    shoot_mode_e shoot_mode;
    loader_mode_e load_mode;
    friction_mode_e friction_mode;
    uint16_t rest_heat;
    uint8_t shoot_rate; // 连续发射的射频,unit per s,发/秒
} Shoot_Ctrl_Cmd_s;

// 底盘模式设置
typedef enum
{
    CHASSIS_ZERO_FORCE = 0,    // 电流零输入
    CHASSIS_FOLLOW_GIMBAL_YAW, // 底盘跟随云台
    CHASSIS_ROTATE,            // 小陀螺模式
    CHASSIS_ROTATE_REVERSE,    // 小陀螺模式反转
    CHASSIS_AUTO_MODE,         // 导航模式
} chassis_mode_e;

// cmd发布的底盘控制数据,由chassis订阅
typedef struct
{
    // 控制部分
    float vx;           // 前进方向速度
    float vy;           // 横移方向速度
    float wz;           // 旋转速度
    float offset_angle; // 底盘和归中位置的夹角
    chassis_mode_e chassis_mode;

} Chassis_Ctrl_Cmd_s;

typedef struct
{
    uint8_t Robot_Color;
    uint16_t projectile_allowance_17mm;  //剩余发弹量
    uint8_t power_management_shooter_output; // 功率管理 shooter 输出
    uint16_t current_hp_percent; // 机器人当前血量百分比
    uint16_t outpost_HP;     //前哨站血量
    uint16_t base_HP;        //基地血量
    uint8_t game_progess;
} Chassis_referee_Upload_Data_s;

typedef struct
{
    float vx;
    float vy;//真实速度
    float wz;
} Chassis_Upload_Data_s;

#pragma pack()

#ifdef __cplusplus
}
#endif


#endif // _ROBOT_CONFIG_H_