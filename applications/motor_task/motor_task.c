/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-10-03 12:47:14
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-10-04 21:44:36
 * @FilePath: \rm_base\applications\motor_task\motor_task.c
 * @Description: 
 */
#include "motor_task.h"
#include "app_config.h"
#include "osal_def.h"
#include "robot_config.h"
#include <stdint.h>

#define log_tag  "motor_task"
#include "shell_log.h"

#if (MOTOR_MODULE_USE & MOTOR_DJI)
#include "dji.h"
static DJIMotor_t dji_motor_list[DJI_MOTOR_CNT];
#endif
#if (MOTOR_MODULE_USE & MOTOR_DM)
#include "damiao.h"
static DMMOTOR_t dm_motor_list[DM_MOTOR_CNT];
#endif

// CAN事件映射结构体
typedef struct {
    uint32_t eventflag;        // 事件标志
    void* motor_ptr;           // 电机指针
    uint8_t motor_type;        // 电机类型
    Can_Device* can_device;    // CAN设备指针
} CanEventMotorMap_t;

// 事件与电机映射表
static CanEventMotorMap_t can_event_motor_map[32] = { {0, NULL, MOTOR_NONE, NULL} };
static uint8_t map_count = 0;
static Can_Device* motor_devices[32]; // 电机设备数组
static uint8_t motor_device_count = 0; // 电机设备数量

static osal_thread_t motor_thread;
MOTOR_THREAD_STACK_SECTION static uint8_t motor_thread_stack[MOTOR_THREAD_STACK_SIZE];
static osal_thread_t motor_decode_thread;
MOTOR_DECODE_THREAD_STACK_SECTION static uint8_t motor_decode_thread_stack[MOTOR_DECODE_THREAD_STACK_SIZE];

void motor_task(ULONG thread_input)
{
    while(1)
    {
        #if (MOTOR_MODULE_USE & MOTOR_DJI)
        DJIMotorControl();
        #endif
        #if (MOTOR_MODULE_USE & MOTOR_DM)
        DMMotorcontrol();
        #endif
        osal_delay_ms(2);
    }
}

void motor_decode_task(ULONG thread_input)
{
    uint32_t triggered_flag;
    
    while(1) {
        if(motor_device_count == 0) {
            osal_delay_ms(10);
            continue;
        }

        // 等待任一设备的事件
        triggered_flag = BSP_CAN_ReadMultipleDevice(motor_devices, motor_device_count, OSAL_WAIT_FOREVER);
        
        if (triggered_flag != 0) {
            // 使用映射表直接找到对应的电机并解码
            for (int i = 0; i < map_count; i++) {
                if (can_event_motor_map[i].eventflag & triggered_flag) {
                    // 根据电机类型调用相应的解码函数
                    switch (can_event_motor_map[i].motor_type) {
                        case MOTOR_DJI:
                            DecodeDJIMotor((DJIMotor_t*)can_event_motor_map[i].motor_ptr);
                            break;
                        case MOTOR_DM:
                            DMMotorDecode((DMMOTOR_t*)can_event_motor_map[i].motor_ptr);
                            break;
                        default:
                            break;
                    }
                }
            }
        }
    }
}

void motor_task_init(void)
{
    
    #if (MOTOR_MODULE_USE & MOTOR_DJI)
    for (int i = 0; i < DJI_MOTOR_CNT; i++) {
        if (dji_motor_list[i].can_device != NULL) {
            motor_devices[motor_device_count++] = dji_motor_list[i].can_device;
            // 建立映射关系
            if (map_count < 32) {
                can_event_motor_map[map_count].eventflag = dji_motor_list[i].can_device->eventflag;
                can_event_motor_map[map_count].motor_ptr = &dji_motor_list[i];
                can_event_motor_map[map_count].motor_type = MOTOR_DJI;
                can_event_motor_map[map_count].can_device = dji_motor_list[i].can_device;
                map_count++;
            }
        }
    }
    #endif

    #if (MOTOR_MODULE_USE & MOTOR_DM)
    for (int i = 0; i < DM_MOTOR_CNT; i++) {
        if (dm_motor_list[i].can_device != NULL) {
            motor_devices[motor_device_count++] = dm_motor_list[i].can_device;
            // 建立映射关系
            if (map_count < 32) {
                can_event_motor_map[map_count].eventflag = dm_motor_list[i].can_device->eventflag;
                can_event_motor_map[map_count].motor_ptr = &dm_motor_list[i];
                can_event_motor_map[map_count].motor_type = MOTOR_DM;
                can_event_motor_map[map_count].can_device = dm_motor_list[i].can_device;
                map_count++;
            }
        }
    }
    #endif

    osal_thread_create(&motor_thread, "motor_task", motor_task, 0,
                       &motor_thread_stack, MOTOR_THREAD_STACK_SIZE,MOTOR_THREAD_PRIORITY);
    osal_thread_create(&motor_decode_thread, "motor_decode_task", motor_decode_task, 0,
                       &motor_decode_thread_stack, MOTOR_DECODE_THREAD_STACK_SIZE,MOTOR_DECODE_THREAD_PRIORITY);
    
    osal_thread_start(&motor_thread);
    osal_thread_start(&motor_decode_thread);
    
    LOG_INFO("motor task init");
}

void motor_list_init(void)
{
    #if (MOTOR_MODULE_USE & MOTOR_DJI)
    DJIMotorListInit(dji_motor_list);
    #endif
    #if (MOTOR_MODULE_USE & MOTOR_DM)
    DMMotorListInit(dm_motor_list);
    #endif
}


