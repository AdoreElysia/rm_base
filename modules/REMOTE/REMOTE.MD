# REMOTE 遥控器模块文档

## 概述

REMOTE 遥控器模块是一个基于 OSAL（操作系统抽象层）的遥控器数据接收和处理系统。该模块支持多种遥控器协议，包括 SBUS、DT7 以及图传遥控器 VT02 和 VT03。模块通过 UART 接口接收遥控器数据，并解析为统一的数据结构供应用程序使用。

## 特性

- 支持多种遥控器协议（SBUS、DT7、VT02、VT03）

- 统一的数据接口，简化应用程序开发

- 基于RTOS线程的数据接收机制

- 与离线检测模块集成，支持设备在线状态监控

- 支持遥控器通道状态、鼠标状态和键盘状态的获取
  
  ## 支持的遥控器类型
  
  ### SBUS 遥控器

- 使用串口通信，波特率 100000

- 支持 16 个通道

- 数据帧长度为 25 字节
  
  ### DT7 遥控器

- 使用串口通信，波特率 100000

- 支持 4 个通道

- 数据帧长度为 18 字节
  
  ### VT02 图传遥控器

- 使用串口通信，波特率 115200

- 支持鼠标和键盘数据
  
  ### VT03 图传遥控器

- 使用串口通信，波特率 921600

- 支持 4 个通道、鼠标、键盘和按钮数据

- 数据帧长度为 21 字节
  
  ## 数据结构
  
  ### remote_instance_t
  
  ```c
  typedef struct {
  #if defined (REMOTE_SOURCE) && REMOTE_SOURCE == 1
      SBUS_Instance_t sbus_instance;    
  #elif defined (REMOTE_SOURCE) && REMOTE_SOURCE == 2
      DT7_Instance_t dt7_instance;
  #endif
  #if defined (REMOTE_VT_SOURCE) && REMOTE_VT_SOURCE == 1
      VT02_Instance_t vt02_instance; 
  #elif defined (REMOTE_VT_SOURCE) && REMOTE_VT_SOURCE == 2
      VT03_Instance_t vt03_instance;
  #endif
      uint8_t vt_offline_index;
      uint8_t remote_offline_index;
      uint8_t initflag;
  } remote_instance_t;
  ```
  
  ### mouse_state_t
  
  ```c
  typedef struct{
      int16_t mouse_x;
      int16_t mouse_y;
      int16_t mouse_z;
      uint8_t mouse_l;
      uint8_t mouse_r;
      uint8_t mouse_m;
  }mouse_state_t;
  ```
  
  ### keyboard_state_t
  
  ```c
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
  ```
  
  ### button_state_t
  
  ```c
  typedef struct {
      uint8_t switch_pos;    
      uint8_t pause;         
      uint8_t custom_left;   
      uint8_t custom_right; 
      uint8_t trigger;       
  }button_state_t;
  ```
  
  ### channel_state
  
  ```c
  enum channel_state
  {
      channel_none,
      channel_down,
      channel_bias,
      channel_up
  }; 
  ```
  
  ## 配置参数
  
  ```c
  // 遥控器源选择
  #define REMOTE_SOURCE 1    // 1: SBUS, 2: DT7, 0: 不使用
  #define REMOTE_VT_SOURCE 2 // 1: VT02, 2: VT03, 0: 不使用
  
  // 遥控器任务配置
  #define REMOTE_THREAD_PRIORITY  10     // 遥控器任务优先级
  #define REMOTE_THREAD_STACK_SIZE 512   // 遥控器任务栈大小
  #define REMOTE_VT_THREAD_PRIORITY  10  // 图传遥控器任务优先级
  #define REMOTE_VT_THREAD_STACK_SIZE 512 // 图传遥控器任务栈大小
  ```
  
  ## API 接口
  
  ```c
  osal_status_t remote_init(remote_instance_t *remote_instance);
  ```
  
  ```c
  int16_t get_remote_channel(uint8_t channel_index, uint8_t is_vt_remote);
  ```

  ```c
  button_state_t* get_remote_button_state();
  ```
  
  ```c
  mouse_state_t* get_remote_mouse_state(uint8_t is_vt_remote);
  ```
  
  ```c
  keyboard_state_t * get_remote_keyboard_state(uint8_t is_vt_remote);
  ```

  ```c
  uint8_t remote_device_status(uint8_t is_vt_remote);
  ```
  
  ## 任务处理
  
  ```c
  remote_task
  处理主遥控器数据接收和解析的任务函数。
  
  remote_vt_task
  处理图传遥控器数据接收和解析的任务函数。
  
  remote_task_init
  创建并启动遥控器相关任务。
  ```
  
  ## Shell 命令
  
  模块提供了一个Shell命令接口，用于查看遥控器状态信息：
  
  ```c
  remote              # 显示帮助信息
  remote status       # 显示遥控器基本状态
  remote channels     # 显示所有通道状态
  remote mouse        # 显示鼠标状态
  remote keyboard     # 显示键盘状态
  ```
  
  
  
  ## 使用示例
  
  ```c
  // 定义遥控器实例
  static remote_instance_t remote_instance;
  
  // 初始化遥控器模块
  memset(&remote_instance, 0, sizeof(remote_instance_t));
  osal_status_t status = remote_init(&remote_instance);
  
  if (status != OSAL_SUCCESS) {
      // 初始化失败处理
      return;
  }
  
  // 获取遥控器通道状态
  int16_t channel_state = get_remote_channel(1, 0);

  // 获取按钮状态
  button_state_t* button_state = get_remote_button_state();
  
  // 获取鼠标状态
  mouse_state_t* mouse_state = get_remote_mouse_state(0);
  
  // 获取键盘状态
  keyboard_state_t* keyboard_state = get_remote_keyboard_state(0);
  ```
  
  ## 工作原理
  
  ### 遥控器初始化
  
  1. 根据配置的遥控器类型初始化对应的遥控器实例
  2. 初始化 UART 设备用于数据接收
  3. 注册离线检测设备以监控遥控器连接状态
  
  ### 数据接收与解析
  
  1. 在独立的 RTOS 线程中运行数据接收任务
  2. 通过 UART 接收原始遥控器数据
  3. 调用对应遥控器的解码函数解析数据
  4. 更新离线检测模块的状态
  
  ### 数据访问
  
  1. 应用程序通过统一的接口访问遥控器数据
  2. 根据配置的遥控器类型返回相应数据
  3. 提供通道状态、鼠标状态和键盘状态的访问接口
  
  ## 注意事项
  
  1. **配置选择**：使用前需要通过宏定义选择遥控器类型
  2. **内存管理**：模块使用静态内存分配，确保系统有足够的内存空间
  3. **线程安全**：模块内部使用 RTOS 机制保证线程安全
  4. **离线检测**：与离线检测模块集成，可监控遥控器连接状态
  5. **数据有效性**：应检查返回的数据指针是否为空，确保数据有效
  
  ## 错误处理
  
  模块通过以下方式处理错误：
  
  1. **初始化失败**：当遥控器初始化失败时，返回 `OSAL_ERROR`
  2. **空指针检查**：对空指针的操作将被忽略并返回错误值
  3. **数据校验**：对接收到的数据进行校验，无效数据将被丢弃
  4. **系统错误**：通过日志系统记录错误信息，便于调试和问题排查
