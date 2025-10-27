/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-09-15 09:26:00
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-10-26 16:52:27
 * @FilePath: \rm_base\modules\REMOTE\remote_data.h
 * @Description: 
 */
#ifndef _REMOTE_DATA_H_
#define _REMOTE_DATA_H_

#include <stdint.h>

/* 统一的按键类型定义 */
typedef enum {
    KEY_W = 0,
    KEY_S,
    KEY_A,
    KEY_D,
    KEY_SHIFT,
    KEY_CTRL,
    KEY_Q,
    KEY_E,
    KEY_R,
    KEY_F,
    KEY_G,
    KEY_Z,
    KEY_X,
    KEY_C,
    KEY_V,
    KEY_B,
    KEY_COUNT  // 用于计数
} remote_key_e;

/* keyboard state structure */
typedef union {
    uint16_t key_code;
    struct
    {
        uint16_t KEY_W : 1;
        uint16_t KEY_S : 1;
        uint16_t KEY_A : 1;
        uint16_t KEY_D : 1;
        uint16_t KEY_SHIFT : 1;
        uint16_t KEY_CTRL : 1;
        uint16_t KEY_Q : 1;
        uint16_t KEY_E : 1;
        uint16_t KEY_R : 1;
        uint16_t KEY_F : 1;
        uint16_t KEY_G : 1;
        uint16_t KEY_Z : 1;
        uint16_t KEY_X : 1;
        uint16_t KEY_C : 1;
        uint16_t KEY_V : 1;
        uint16_t KEY_B : 1;
    } bit;
} keyboard_state_t;

typedef struct{
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t mouse_l;
    uint8_t mouse_r;
    uint8_t mouse_m;
}mouse_state_t;

typedef struct {
    uint8_t switch_pos;    
    uint8_t pause;         
    uint8_t custom_left;   
    uint8_t custom_right; 
    uint8_t trigger;       
}button_state_t;


#endif // _REMOTE_DATA_H_