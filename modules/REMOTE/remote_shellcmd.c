/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-10-01 20:46:36
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-10-27 00:08:53
 * @FilePath: \rm_base\modules\REMOTE\remote_shellcmd.c
 * @Description: 
 */
#include "remote.h"
#include "shell.h"

static remote_instance_t *remote = NULL;
 

// 遥控器shell命令实现
void remote_shell_cmd(int argc, char **argv) {

    if (remote == NULL || remote->initflag != 1) {
        shell_module_printf("Remote module not initialized!\r\n");
        return;
    }
    
    // 如果没有参数，显示帮助信息
    if (argc < 2) {
        shell_module_printf("Usage: remote <command>\r\n");
        shell_module_printf("Commands:\r\n");
        shell_module_printf("  status    - Show remote control status\r\n");
        shell_module_printf("  channels  - Show channels status\r\n");
        shell_module_printf("  mouse     - Show mouse status\r\n");
        shell_module_printf("  keyboard  - Show keyboard status\r\n");
        return;
    }
    
    // 根据命令参数执行相应操作
    if (strcmp(argv[1], "status") == 0) {
        shell_module_printf("Remote Control Status:\r\n");
#if defined(REMOTE_SOURCE) && REMOTE_SOURCE == 1
        shell_module_printf("  Type: SBUS\r\n");
#elif defined(REMOTE_SOURCE) && REMOTE_SOURCE == 2
        shell_module_printf("  Type: DT7\r\n");
#else
        shell_module_printf("  Type: None\r\n");
#endif

#if defined(REMOTE_VT_SOURCE) && REMOTE_VT_SOURCE == 1
        shell_module_printf("  VT Type: VT02\r\n");
#elif defined(REMOTE_VT_SOURCE) && REMOTE_VT_SOURCE == 2
        shell_module_printf("  VT Type: VT03\r\n");
#else
        shell_module_printf("  VT Type: None\r\n");
#endif
        
        shell_module_printf("  Initialized: %s\r\n", remote->initflag ? "Yes" : "No");
        shell_module_printf("  Remote Offline Index: %d\r\n", remote->remote_offline_index);
        shell_module_printf("  VT Offline Index: %d\r\n", remote->vt_offline_index);
    }
    else if (strcmp(argv[1], "channels") == 0) {
        shell_module_printf("Channels Status:\r\n");
#if defined(REMOTE_SOURCE) && REMOTE_SOURCE == 1
        for (int i = 1; i <= 16; i++) {
            int16_t state = get_remote_channel(i, 0);
            shell_module_printf("  CH%d: %d\r\n", i, state);
        }
#elif defined(REMOTE_SOURCE) && REMOTE_SOURCE == 2
        for (int i = 1; i <= 7; i++) {
            int16_t state = get_remote_channel(i, 0);
            shell_module_printf("  CH%d: %d\r\n", i, state);
        }
#endif

#if defined(REMOTE_VT_SOURCE) && REMOTE_VT_SOURCE == 2
        shell_module_printf("VT Channels:\r\n");
        for (int i = 1; i <= 5; i++) {
            int16_t state = get_remote_channel(i, 1);
            shell_module_printf("  VT_CH%d: %d\r\n", i, state);
        }
#endif
    }
    else if (strcmp(argv[1], "mouse") == 0) {
        shell_module_printf("Mouse Status:\r\n");
        
        // 主遥控器鼠标状态
#if defined(REMOTE_SOURCE) && REMOTE_SOURCE == 2
        mouse_state_t* mouse = get_remote_mouse_state(0);
        if (mouse != NULL) {
            shell_module_printf("  Main Remote:\r\n");
            shell_module_printf("    X: %d, Y: %d, Z: %d\r\n", mouse->mouse_x, mouse->mouse_y, mouse->mouse_z);
            shell_module_printf("    Left: %d, Right: %d, Middle: %d\r\n", mouse->mouse_l, mouse->mouse_r, mouse->mouse_m);
        } else {
            shell_module_printf("  Main Remote: No data\r\n");
        }
#endif

        // VT遥控器鼠标状态
#if defined(REMOTE_VT_SOURCE) && (REMOTE_VT_SOURCE == 1 || REMOTE_VT_SOURCE == 2)
        mouse_state_t* vt_mouse = get_remote_mouse_state(1);
        if (vt_mouse != NULL) {
            shell_module_printf("  VT Remote:\r\n");
            shell_module_printf("    X: %d, Y: %d, Z: %d\r\n", vt_mouse->mouse_x, vt_mouse->mouse_y, vt_mouse->mouse_z);
            shell_module_printf("    Left: %d, Right: %d, Middle: %d\r\n", vt_mouse->mouse_l, vt_mouse->mouse_r, vt_mouse->mouse_m);
        } else {
            shell_module_printf("  VT Remote: No data\r\n");
        }
#endif
    }
    else if (strcmp(argv[1], "keyboard") == 0) {
        shell_module_printf("Keyboard Status:\r\n");
        
        // 主遥控器键盘状态
#if defined(REMOTE_SOURCE) && REMOTE_SOURCE == 2
        keyboard_state_t* keyboard = get_remote_keyboard_state(0);
        if (keyboard != NULL) {
            shell_module_printf("  Main Remote Keys:\r\n");
            shell_module_printf("    W: %d, S: %d, A: %d, D: %d\r\n", 
                              keyboard->bit.KEY_W, keyboard->bit.KEY_S, keyboard->bit.KEY_A, keyboard->bit.KEY_D);
            shell_module_printf("    SHIFT: %d, CTRL: %d, Q: %d, E: %d\r\n",
                              keyboard->bit.KEY_SHIFT, keyboard->bit.KEY_CTRL, keyboard->bit.KEY_Q, keyboard->bit.KEY_E);
            shell_module_printf("    R: %d, F: %d, G: %d, Z: %d\r\n",
                              keyboard->bit.KEY_R, keyboard->bit.KEY_F, keyboard->bit.KEY_G, keyboard->bit.KEY_Z);
            shell_module_printf("    X: %d, C: %d, V: %d, B: %d\r\n",
                              keyboard->bit.KEY_X, keyboard->bit.KEY_C, keyboard->bit.KEY_V, keyboard->bit.KEY_B);
        } else {
            shell_module_printf("  Main Remote: No data\r\n");
        }
#endif

        // VT遥控器键盘状态
#if defined(REMOTE_VT_SOURCE) && (REMOTE_VT_SOURCE == 1 || REMOTE_VT_SOURCE == 2)
        keyboard_state_t* vt_keyboard = get_remote_keyboard_state(1);
        if (vt_keyboard != NULL) {
            shell_module_printf("  VT Remote Keys:\r\n");
            shell_module_printf("    W: %d, S: %d, A: %d, D: %d\r\n", 
                              vt_keyboard->bit.KEY_W, vt_keyboard->bit.KEY_S, vt_keyboard->bit.KEY_A, vt_keyboard->bit.KEY_D);
            shell_module_printf("    SHIFT: %d, CTRL: %d, Q: %d, E: %d\r\n",
                              vt_keyboard->bit.KEY_SHIFT, vt_keyboard->bit.KEY_CTRL, vt_keyboard->bit.KEY_Q, vt_keyboard->bit.KEY_E);
            shell_module_printf("    R: %d, F: %d, G: %d, Z: %d\r\n",
                              vt_keyboard->bit.KEY_R, vt_keyboard->bit.KEY_F, vt_keyboard->bit.KEY_G, vt_keyboard->bit.KEY_Z);
            shell_module_printf("    X: %d, C: %d, V: %d, B: %d\r\n",
                              vt_keyboard->bit.KEY_X, vt_keyboard->bit.KEY_C, vt_keyboard->bit.KEY_V, vt_keyboard->bit.KEY_B);
        } else {
            shell_module_printf("  VT Remote: No data\r\n");
        }
#endif
    }
    else {
        shell_module_printf("Unknown command: %s\r\n", argv[1]);
        shell_module_printf("Use 'remote' without arguments to see help.\r\n");
    }
}

void remote_shell_cmd_init(remote_instance_t *remote_instance) {
    if (remote_instance != NULL)
    {
        remote = remote_instance;
        shell_module_register_cmd("remote", remote_shell_cmd, "Remote control status");
    }
}

