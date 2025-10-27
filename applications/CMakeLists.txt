# 创建静态库
set(name APPLICATIONS)


add_library(${name} STATIC
    shell_task/shell_task.c
    robot_init/robot_init.c
    offline_task/offline_task.c
    ins_task/ins_task.c
    remote_task/remote_task.c
    dm_imu_task/dm_imu_task.c
    motor_task/motor_task.c
    robot_control_task/robot_control_task.c
    robot_control_task/robot_control_fun.c
)

# 设置包含目录
target_include_directories(${name}
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/robot_init
    ${CMAKE_CURRENT_SOURCE_DIR}/offline_task
    ${CMAKE_CURRENT_SOURCE_DIR}/ins_task
    ${CMAKE_CURRENT_SOURCE_DIR}/remote_task
    ${CMAKE_CURRENT_SOURCE_DIR}/dm_imu_task
    ${CMAKE_CURRENT_SOURCE_DIR}/shell_task
    ${CMAKE_CURRENT_SOURCE_DIR}/motor_task
    ${CMAKE_CURRENT_SOURCE_DIR}/robot_control_task
)

# 链接必要的库
target_link_libraries(${name} stm32cubemx OSAL BSP CONFIG TOOLS MODULES)

# 添加到主项目
target_link_libraries(${PROJECT_NAME} ${name})