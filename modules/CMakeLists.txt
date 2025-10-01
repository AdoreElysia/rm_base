# 创建静态库
set(name MODULES)


add_library(${name} STATIC
    RGB/rgb.c
    IST8310/ist8310.c
    IMU/IMULAYER/imulayer.c
    IMU/BMI088/bmi088.c
    IMU/BMI088/bmi088_cali.c
    algorithm/compensation.c
    algorithm/controller.c
    algorithm/kalman_filter.c
    algorithm/LQR.c
    algorithm/user_lib.c
    algorithm/crc_rm.c
    BEEP/beep.c
    OFFLINE/offline.c
    OFFLINE/offline_shellcmd.c
    INS/ins.c
    INS/QuaternionEKF.c
    REMOTE/remote.c
    REMOTE/SBUS/sbus.c
    REMOTE/DT7/dt7.c
    REMOTE/VT02/vt02.c
    REMOTE/VT03/vt03.c
    REMOTE/remote_shellcmd.c
    DM_IMU/dm_imu.c
    MOTOR/DJI/dji.c
)

# 设置包含目录
target_include_directories(${name}
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/RGB
    ${CMAKE_CURRENT_SOURCE_DIR}/IST8310
    ${CMAKE_CURRENT_SOURCE_DIR}/IMU
    ${CMAKE_CURRENT_SOURCE_DIR}/IMU/IMULAYER
    ${CMAKE_CURRENT_SOURCE_DIR}/IMU/BMI088
    ${CMAKE_CURRENT_SOURCE_DIR}/algorithm
    ${CMAKE_CURRENT_SOURCE_DIR}/BEEP
    ${CMAKE_CURRENT_SOURCE_DIR}/OFFLINE
    ${CMAKE_CURRENT_SOURCE_DIR}/INS
    ${CMAKE_CURRENT_SOURCE_DIR}/REMOTE
    ${CMAKE_CURRENT_SOURCE_DIR}/REMOTE/DT7
    ${CMAKE_CURRENT_SOURCE_DIR}/REMOTE/SBUS
    ${CMAKE_CURRENT_SOURCE_DIR}/REMOTE/VT02
    ${CMAKE_CURRENT_SOURCE_DIR}/REMOTE/VT03
    ${CMAKE_CURRENT_SOURCE_DIR}/DM_IMU
    ${CMAKE_CURRENT_SOURCE_DIR}/MOTOR
    ${CMAKE_CURRENT_SOURCE_DIR}/MOTOR/DJI
)

# 链接必要的库
target_link_libraries(${name} stm32cubemx OSAL BSP CONFIG TOOLS)
target_link_libraries(MODULES -l:libarm_cortexM4lf_math.a) #链接dsp
target_link_libraries(MODULES m) #链接数学库

# 添加到主项目
target_link_libraries(${PROJECT_NAME} ${name})