# Shell 工具文档

## 概述

Shell 是一个基于 OSAL 的命令行接口工具，提供了统一的接口来与系统进行交互。该工具支持命令注册、命令解析和执行等功能。

## 特性

- 支持命令行交互界面

- 支持 ANSI 转义序列（清屏等）

- 双缓冲区机制提高接收效率

- 基于 OSAL 的线程安全机制

- 支持多个命令动态注册

- 支持 RTT 和 UART 两种通信方式

- 分层设计，模块化实现
  
  支持日志输出功能
  
  ## 数据结构
  
  ### shell_cmd_func_t

- ```c
  typedef void (*shell_cmd_func_t)(int argc, char **argv);
  ```
  
  > 命令处理函数指针类型，所有命令处理函数都应遵循此原型。
  
  ### Shell_Module_t
  
  Shell模块结构体，包含Shell运行所需的所有状态信息。
  
  ```c
  typedef struct Shell_Module{
      UART_Device *uart_dev;                                            // UART设备
      char cmd_buffer[SHELL_CMD_MAX_LENGTH];                            // 命令缓冲区
      uint16_t cmd_len;                                                 // 命令长度
      char *args[SHELL_MAX_ARGS];                                       // 参数指针数组
      uint8_t argc;                                                     // 参数数量
      osal_mutex_t mutex;                                               // 互斥锁
      shell_cmd_t dynamic_cmds[SHELL_MAX_DYNAMIC_COMMANDS];             // 动态注册命令表
      int dynamic_cmd_count;                                            // 动态注册命令数量
      bool initialized;                                                 // 初始化状态
      bool use_rtt;                                                     // 是否使用rtt作为shell输出
      uint8_t escape_sequence;                                          // 转义序列状态（处理箭头键）
  } Shell_Module_t;
  ```
  
  ## 配置参数
  
  ```c
  #define SHELL_ENABLE                 1                                    // 启用shell功能
  #define SHELL_CMD_MAX_LENGTH         128                                  // 最大命令长度
  #define SHELL_MAX_ARGS               8                                    // 最大参数数量
  #define SHELL_HISTORY_MAX            10                                   // 历史命令数量
  #define SHELL_PROMPT                 "shell> "                            // 提示符
  #define SHELL_MAX_DYNAMIC_COMMANDS   8                                    // 注册命令的最大数量
  #define SHELL_RTT                    0                                    // 使用rtt作为shell通讯方式,当使能时,SHELL_COM无效
  #define SHELL_COM                    huart6                               // shell使用的通讯接口(uart)
  #define SHELL_BUFFER_SIZE            32                                   // shell缓冲区大小
  #define SHELL_THREAD_STACK_SIZE      1024                                 // shell线程栈大小
  #define SHELL_THREAD_PRIORITY        30                                   // shell线程优先级
  #define SHELL_THREAD_STACK_SECTION  __attribute__((section(".ccmram")))   //shell线程栈内存区域
  ```
  
  ## API 接口
  
  ```c
  osal_status_t shell_module_init(Shell_Module_t *shell);
  ```
  
  ```c
  void shell_module_printf(const char *fmt, ...);
  ```
  
  ```c
  void shell_module_period_process();
  ```
  
  ```c
  osal_status_t shell_module_register_cmd(const char *name, shell_cmd_func_t func, const char *description);
  ```
  
  ```c
  void shell_module_send(uint8_t *data, uint16_t len);
  ```
  
  ## 内置命令
  
  ### help 命令
  
  显示所有可用命令的帮助信息。
  
  ### clear 命令
  
  清屏命令，使用 ANSI 转义序列清除终端屏幕。
  
  ### version 命令
  
  显示 Shell 系统的版本信息。
  
  ### ps 命令
  
  显示系统信息，包括线程、定时器、互斥量、信号量、事件标志组、队列、字节池和块池等信息。
  
  > 目前只写了threadx的
  
  ## 日志功能
  
  Shell提供了一套完整的日志系统，支持多种日志级别和格式化输出。
  
  ### 日志级别
  
  日志系统支持以下6种日志级别：
  
  - VERBOSE - 详细信息
  - DEBUG - 调试信息
  - INFO - 一般信息
  - WARN - 警告信息
  - ERROR - 错误信息
  - ASSERT - 断言信息
  
  ### 配置参数
  
  ```c
  #define LOG_OUTPUT_LEVEL             LOG_LEVEL_INFO                       // 日志的输出等级
  #define LOG_COLOR_ENABLE             1                                    // 启用彩色输出
  #define LOG_TIMSTAMP_ENABLE          1                                    // 启用时间戳 
  ```
  
  ## 使用示例
  
  ### 注册自定义命令
  
  ```c
  // 定义命令处理函数
  void my_command_handler(int argc, char **argv) {
      if (argc < 2) {
          shell_printf("Usage: mycmd <param>\r\n");
          return;
      }
  
      shell_printf("Received parameter: %s\r\n", argv[1]);
  }
  
  // 在适当位置注册命令
  void register_my_command(void) {
      shell_register_function("mycmd", my_command_handler, "My custom command");
  }
  ```
  
  ### 使用Shell进行调试
  
  ```c
  shell> help
  Available commands:
    help     - 显示帮助信息 (Show help)
    clear    - 清屏 (Clear screen)
    version  - 显示版本信息 (Version info)
    ps       - 显示系统信息 (Show system info)
    mycmd    - My custom command
  
  shell> version
  osal Shell System v1.0
  Built with osal
  
  shell> ps thread
  Thread Information:
  Name                             State    Priority   Stack Size   Run Count
  ------------------------------------------------------------------------
  tx_idle_thread                   READY    30         1024         1000    
  tx_timer_thread                  READY    30         1024         500     
  my_thread                        READY    30         1024         200     
  
  shell> mycmd test_parameter
  Received parameter: test_parameter
  ```
  
  ### 日志使用
  
  ```c
  // 在文件开头定义log_tag（可选，默认为"default"）
  #define log_tag "mymodule"
  
  #include "shell_log.h"
  
  // 使用不同级别的日志宏
  LOG_VERBOSE("详细信息");
  LOG_DEBUG("调试信息: value=%d", value);
  LOG_INFO("一般信息");
  LOG_WARN("警告信息");
  LOG_ERROR("错误信息: %s", error_msg);
  LOG_ASSERT("断言信息");
  ```
  
  #### 特性
  
  1. **自动换行**：所有日志都会自动添加`\r\n`换行符
  2. **彩色输出**：支持ANSI颜色代码，不同级别日志以不同颜色显示
  3. **时间戳**：可选的时间戳功能，显示日志输出的精确时间
  4. **标签支持**：可为每个模块定义标签，便于识别日志来源
  5. **递归保护**：防止在日志输出过程中再次调用日志函数导致死锁
  6. **线程安全**：基于shell模块的互斥锁机制保证线程安全
  
  #### 工作原理
  
  ### 分层架构
  
  Shell采用分层设计：
  
  1. **Module层** - 核心功能实现，提供统一接口
  2. **Task层** - 线程管理，数据持有和周期性调用
  
  ### 命令处理流程
  
  1. 用户输入命令
  2. Shell解析命令和参数
  3. 查找匹配的命令处理函数
  4. 调用命令处理函数执行具体操作
  5. 返回结果通过shell_module_printf输出
  
  ## 注意事项
  
  1. **内存管理**：Shell使用静态内存管理，避免动态内存分配，提高系统稳定性。
  
  2. **线程安全**：Shell内部使用互斥锁保护共享资源，确保线程安全。
  
  3. **缓冲区大小**：根据实际需求调整SHELL_CMD_MAX_LENGTH和SHELL_BUFFER_SIZE等参数。
  
  4. **命令数量**：动态注册的命令数量不能超过SHELL_MAX_DYNAMIC_COMMANDS定义的数量。
  
  5. **通信接口**：可以通过配置选择使用UART或RTT作为通信接口。
  
  6. **日志级别**：合理设置日志输出级别，避免输出过多调试信息影响系统性能。
  
  ## 错误处理
  
  Shell内部处理常见的错误情况，如：
  
  - 未知命令
  - 命令参数错误
  - 命令注册失败（命令已存在或超出最大命令数量）
  
  对于这些情况，Shell会输出相应的错误信息提示用户。
