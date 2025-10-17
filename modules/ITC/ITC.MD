# ITC (Inter-Thread Communication) 模块文档

## 概述

ITC (Inter-Thread Communication) 是一个基于 OSAL (操作系统抽象层) 的轻量级线程间通信模块，专为嵌入式系统设计。该模块实现了发布/订阅模式的消息传递机制，允许不同线程之间通过话题(Topic)进行解耦合的数据交换。ITC模块支持一对多的通信模式，即一个发布者可以向多个订阅者发送消息。

## 特性

- 基于发布/订阅模式的异步消息传递
- 支持一对多通信模式
- 轻量级设计，适用于资源受限的嵌入式系统
- 基于OSAL的跨RTOS平台兼容性
- 静态内存管理，无需动态分配
- 数据完整性检查
- 支持多种数据类型传输
- 线程安全设计

## 数据结构

### itc_topic_t

```c
typedef struct itc_topic {
    char name[ITC_MAX_TOPIC_NAME_LEN];       // 话题名称
    volatile uint8_t is_active;              // 是否激活
    volatile uint8_t subscriber_count;       // 订阅者数量
    uint8_t publisher_data_len;              // 发布者规定的数据长度
    itc_subscriber_t *subscribers[ITC_MAX_SUBSCRIBERS_PER_TOPIC]; // 订阅者数组
    struct itc_topic *next;                  // 下一个话题
} itc_topic_t;
```

### itc_subscriber_t

```c
typedef struct itc_subscriber {
    volatile uint8_t is_active;              // 是否激活
    struct itc_subscriber *next;             // 下一个订阅者
    uint32_t expected_data_len;              // 订阅者期望的数据长度
    
    osal_queue_t data_queue;                 // 用于存储数据指针的队列
    uint8_t queue_buffer[ITC_QUEUE_BUFFER_SIZE];  // 队列缓冲区
    char name[32];                           // 订阅者名称
    
    itc_topic_t *topic_ptr;                  // 关联的话题指针
} itc_subscriber_t;
```

## 配置参数

```c
#define ITC_MAX_SUBSCRIBERS_PER_TOPIC 4      // 每个话题最大订阅者数量
#define ITC_MAX_TOPIC_NAME_LEN 32            // 最大话题名称长度
```

## API 接口

```c
osal_status_t itc_init(void);
```

```c
osal_status_t itc_topic_init(itc_topic_t *topic, const char *topic_name, uint32_t data_len);
```

```c
osal_status_t itc_topic_deinit(itc_topic_t *topic);
```

```c
osal_status_t itc_subscriber_init(itc_subscriber_t *subscriber, const char *topic_name, uint32_t expected_data_len);
```

```c
osal_status_t itc_subscriber_deinit(itc_subscriber_t *subscriber);
```

```c
osal_status_t itc_publish(itc_topic_t *topic, void *data_ptr);
```

```c
void* itc_receive(itc_subscriber_t *subscriber);
```

## 使用示例

```c
// 1. 初始化ITC模块
itc_init();

// 2. 定义话题和订阅者
itc_topic_t topic;
itc_subscriber_t subscriber;

// 3. 初始化话题
itc_topic_init(&topic, "sensor_data", sizeof(sensor_data_t));

// 4. 初始化订阅者
itc_subscriber_init(&subscriber, "sensor_data", sizeof(sensor_data_t));

// 5. 发布消息
sensor_data_t sensor_data;
// ... 填充传感器数据 ...
itc_publish(&topic, &sensor_data);

// 6. 接收消息
sensor_data_t* received_data = (sensor_data_t*)itc_receive(&subscriber);
```

## 工作原理

### 发布/订阅模式

ITC模块采用发布/订阅模式实现线程间通信：

1. 发布者初始化话题并向其发布数据
2. 订阅者通过话题名称订阅感兴趣的话题
3. 当有数据发布到话题时，所有订阅者都会收到该数据的副本

### 数据传输机制

ITC模块不直接复制数据，而是传输数据指针。

- 必须保证发布数据的内存有效性

### 内存管理

ITC模块使用静态内存管理方式：

1. 话题和订阅者对象由用户定义和管理
2. 队列缓冲区内存静态分配在订阅者结构体中
3. 每个订阅者拥有独立的消息队列用于缓冲接收到的数据

### 同步机制

ITC模块使用OSAL提供的队列和互斥锁机制实现线程同步：

1. 发布操作是非阻塞的
2. 接收操作可以阻塞等待直到有数据到达
3. 每个订阅者的消息队列长度为1，确保数据的实时性
4. 使用互斥锁保护话题和订阅者的创建与销毁操作

## 注意事项

1. **话题名称唯一性**: 话题名称在系统中必须唯一，重复创建同名话题会返回错误

2. **数据长度一致性**: 订阅者期望的数据长度必须与话题的数据长度完全一致

3. **内存管理**: 应用程序负责管理话题和订阅者结构体的内存，ITC模块只负责内部队列缓冲区

4. **线程安全**: ITC模块内部使用互斥机制保证线程安全

5. **资源限制**: 系统中的每个话题的订阅者数量受配置参数限制

6. **生命周期管理**: 应用程序需要合理管理话题和订阅者的生命周期，在不需要时调用相应的deinit函数

## 错误处理

模块通过以下方式处理错误：

1. **参数验证**: 所有API接口都会验证输入参数的有效性

2. **状态检查**: 操作前会检查对象的激活状态和模块初始化状态

3. **错误码返回**: 通过标准的osal_status_t错误码返回操作结果

4. **日志记录**: 使用日志系统记录错误信息，便于调试和问题排查
