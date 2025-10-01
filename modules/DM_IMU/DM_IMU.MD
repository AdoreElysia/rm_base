# DM_IMU 惯性测量单元模块文档

## 概述

DM_IMU 惯性测量单元模块是一个基于 OSAL（操作系统抽象层）的 IMU 数据接收和处理系统。该模块通过 CAN 总线与 DM 系列惯性测量单元通信，获取加速度、角速度、欧拉角和四元数等数据，并将其转换为应用程序可用的标准格式。
更多信息请参考: [DM-IMU 相关链接](https://gitee.com/kit-miao/dm-imu)

## 特性

- 通过 CAN 总线进行数据通信
- 支持加速度、角速度、欧拉角和四元数数据获取
- 数据自动转换和校验
- 与离线检测模块集成，支持设备在线状态监控
- 基于 RTOS 线程的后台数据请求和更新机制
  
  ## 数据结构
  
  ### DM_IMU_DATA_t
  
  ```c
  typedef struct{
      float q[4];              // 四元数
      IMU_Data_t imu_data;     // IMU原始数据（加速度、角速度）
      IMU_Estimate_t estimate; // 姿态估计数据（欧拉角等）
  }DM_IMU_DATA_t;
  ```
  
  ### DM_IMU_Moudule_t
  
  ```c
  typedef struct
  {	
      DM_IMU_DATA_t data;      // IMU数据
      Can_Device *can_device;  // CAN设备实例
      uint8_t offline_index;   // 离线索引
      uint8_t initflag;        // 初始化标志
  }DM_IMU_Moudule_t;
  ```
  
  ### IMU_Data_t
  
  ```c
  typedef struct {
      float gyro[3];  // 角速度 (单位: rad/s)
      float acc[3];   // 加速度 (单位: m/s^2)
  } IMU_Data_t;
  ```
  
  ### IMU_Estimate_t
  
  ```c
  typedef struct {
      float Pitch;          // 俯仰角 (单位: 度)
      float Roll;           // 横滚角 (单位: 度)
      float Yaw;            // 偏航角 (单位: 度)
      float YawTotalAngle;  // 总偏航角 (单位: 度)
      int YawRoundCount;    // 偏航角圈数计数
  } IMU_Estimate_t;
  ```
  
  ## 寄存器定义
  
  ```c
  #define DM_RID_ACCEL      1  // 加速度数据寄存器
  #define DM_RID_GYRO       2  // 角速度数据寄存器
  #define DM_RID_EULER      3  // 欧拉角数据寄存器
  #define DM_RID_Quaternion 4  // 四元数数据寄存器
  ```
  
  ## 配置参数
  
  ```c
  #define DM_IMU_THREAD_PRIORITY  10     // IMU任务优先级
  #define DM_IMU_THREAD_STACK_SIZE 512   // IMU任务栈大小
  
  // 数据范围定义
  #define ACCEL_CAN_MAX   (58.8f)    // 加速度最大值
  #define ACCEL_CAN_MIN   (-58.8f)   // 加速度最小值
  #define GYRO_CAN_MAX    (34.88f)   // 角速度最大值
  #define GYRO_CAN_MIN    (-34.88f)  // 角速度最小值
  #define PITCH_CAN_MAX   (90.0f)    // 俯仰角最大值
  #define PITCH_CAN_MIN   (-90.0f)   // 俯仰角最小值
  #define ROLL_CAN_MAX    (180.0f)   // 横滚角最大值
  #define ROLL_CAN_MIN    (-180.0f)  // 横滚角最小值
  #define YAW_CAN_MAX     (180.0f)   // 偏航角最大值
  #define YAW_CAN_MIN     (-180.0f)  // 偏航角最小值
  #define TEMP_MIN        (0.0f)     // 温度最小值
  #define TEMP_MAX        (60.0f)    // 温度最大值
  #define Quaternion_MIN  (-1.0f)    // 四元数最小值
  #define Quaternion_MAX  (1.0f)     // 四元数最大值
  ```
  
  ## API 接口
  
  ```c
  osal_status_t dm_imu_init(DM_IMU_Moudule_t *dm_imu);
  ```
  
  ```c
  void dm_imu_request(uint8_t reg);
  ```
  
  ```c
  void dm_imu_update();
  ```
  
  ```c
  DM_IMU_DATA_t* get_dm_imu_data(void);
  ```
  
  ## 任务处理
  
  ### dm_imu_task
  
  处理 IMU 数据请求和接收的任务函数。该任务周期性地请求不同类型的 IMU 数据，并处理接收到的数据。
  
  ## 使用示例
  
  ```c
  // 定义 IMU 实例
  static DM_IMU_Moudule_t dm_imu_module;
  
  // 初始化 IMU 模块
  memset(&dm_imu_module, 0, sizeof(DM_IMU_Moudule_t));
  osal_status_t status = dm_imu_init(&dm_imu_module);
  
  if (status != OSAL_SUCCESS) {
      // 初始化失败处理
      return;
  }
  
  // 获取 IMU 数据
  DM_IMU_DATA_t* imu_data = get_dm_imu_data();
  if (imu_data != NULL) {
      // 使用 IMU 数据
      float pitch = imu_data->estimate.Pitch;
      float roll = imu_data->estimate.Roll;
      float yaw = imu_data->estimate.Yaw;
      
      float acc_x = imu_data->imu_data.acc[0];
      float acc_y = imu_data->imu_data.acc[1];
      float acc_z = imu_data->imu_data.acc[2];
      
      float gyro_x = imu_data->imu_data.gyro[0];
      float gyro_y = imu_data->imu_data.gyro[1];
      float gyro_z = imu_data->imu_data.gyro[2];
  }
  ```
  
  ## 工作原理
  
  ### IMU 初始化
  1. 注册离线检测设备以监控 IMU 连接状态
  2. 初始化 CAN 设备用于与 IMU 通信
  3. 设置初始化标志表示模块已准备就绪
  
  ### 数据请求与接收
  
  1. 在独立的 RTOS 线程中周期性地请求 IMU 数据
  2. 通过 CAN 总线发送请求命令到 IMU 设备
  3. 等待并接收 IMU 设备返回的数据
  
  ### 数据解析与处理
  
  1. 根据数据类型解析 CAN 帧中的原始数据
  2. 将原始数据转换为实际物理量值
  3. 对角度数据进行连续性处理，解决角度跳变问题
  4. 更新离线检测模块的状态
  
  ### 数据访问
  
  1. 应用程序通过 get_dm_imu_data() 函数获取最新的 IMU 数据
  2. 返回的数据包含加速度、角速度、欧拉角和四元数
  
  ## 注意事项
  
  1. **初始化顺序**：使用模块前必须调用 dm_imu_init() 进行初始化
  2. **任务依赖**：必须创建并运行 `dm_imu_task` 任务以获取实时数据
  3. **数据有效性**：应检查 get_dm_imu_data() 返回的指针是否为空
  4. **内存管理**：模块使用静态内存分配，确保系统有足够的内存空间
  5. **线程安全**：模块内部使用 RTOS 机制保证线程安全
  6. **离线检测**：与离线检测模块集成，可监控 IMU 连接状态
  
  ## 错误处理
  
  模块通过以下方式处理错误：
  
  1. **初始化失败**：当 CAN 设备初始化失败或离线设备注册失败时，返回 `OSAL_ERROR`
  2. **空指针检查**：对空指针的操作将被忽略
  3. **数据校验**：对接收到的数据进行校验，无效数据将被丢弃
  4. **系统错误**：通过日志系统记录错误信息，便于调试和问题排查
  
  ## 数据转换
  
  模块使用 uint_to_float 函数将接收到的整型数据转换为浮点型物理量：
  
  ```c
  // 加速度数据转换
  dm_imu_module->data.imu_data.acc[0]=uint_to_float(tmp[0],ACCEL_CAN_MIN,ACCEL_CAN_MAX,16);
  
  // 角速度数据转换
  dm_imu_module->data.imu_data.gyro[0]=uint_to_float(tmp[0],GYRO_CAN_MIN,GYRO_CAN_MAX,16);
  
  // 欧拉角数据转换
  dm_imu_module->data.estimate.Pitch=uint_to_float(euler[0],PITCH_CAN_MIN,PITCH_CAN_MAX,16);
  ```
  
  


