/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-10-01 23:05:55
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-10-01 23:15:59
 * @FilePath: /rm_base/modules/DM_IMU/dm_imu_shellcmd.c
 * @Description:
 */
#include "dm_imu.h"
#include "shell.h"

static DM_IMU_Moudule_t *dm_imu_module = NULL;
void                     dm_imu_shell_cmd(int argc, char **argv)
{

    if (dm_imu_module == NULL || dm_imu_module->initflag != 1)
    {
        shell_module_printf("DM IMU module not initialized!\r\n");
        return;
    }

    // 如果没有参数，显示帮助信息
    if (argc < 2)
    {
        shell_module_printf("Usage: dm_imu <command>\r\n");
        shell_module_printf("Commands:\r\n");
        shell_module_printf("  status    - Show DM IMU status\r\n");
        shell_module_printf("  data      - Show current IMU data\r\n");
        shell_module_printf("  accel     - Show acceleration data\r\n");
        shell_module_printf("  gyro      - Show gyroscope data\r\n");
        shell_module_printf("  euler     - Show Euler angles\r\n");
        shell_module_printf("  quat      - Show quaternion data\r\n");
        return;
    }

    // 根据命令参数执行相应操作
    if (strcmp(argv[1], "status") == 0)
    {
        shell_module_printf("DM IMU Status:\r\n");
        shell_module_printf("  Initialized: %s\r\n", dm_imu_module->initflag ? "Yes" : "No");
        shell_module_printf("  Offline Index: %d\r\n", dm_imu_module->offline_index);
        shell_module_printf("  CAN Device: %s\r\n",
                            dm_imu_module->can_device ? "Available" : "Not available");
    }
    else if (strcmp(argv[1], "data") == 0)
    {
        DM_IMU_DATA_t *data = get_dm_imu_data();
        if (data != NULL)
        {
            shell_module_printf("DM IMU Data:\r\n");
            shell_module_printf("  Acceleration (m/s^2):\r\n");
            shell_module_printf("    X: %.4f, Y: %.4f, Z: %.4f\r\n", data->imu_data.acc[0],
                                data->imu_data.acc[1], data->imu_data.acc[2]);
            shell_module_printf("  Gyroscope (rad/s):\r\n");
            shell_module_printf("    X: %.4f, Y: %.4f, Z: %.4f\r\n", data->imu_data.gyro[0],
                                data->imu_data.gyro[1], data->imu_data.gyro[2]);
            shell_module_printf("  Euler Angles (deg):\r\n");
            shell_module_printf("    Pitch: %.2f, Roll: %.2f, Yaw: %.2f\r\n", data->estimate.Pitch,
                                data->estimate.Roll, data->estimate.Yaw);
            shell_module_printf("  Total Yaw: %.2f, Yaw Count: %d\r\n",
                                data->estimate.YawTotalAngle, data->estimate.YawRoundCount);
            shell_module_printf("  Quaternion:\r\n");
            shell_module_printf("    W: %.4f, X: %.4f, Y: %.4f, Z: %.4f\r\n", data->q[0],
                                data->q[1], data->q[2], data->q[3]);
        }
        else
        {
            shell_module_printf("No IMU data available\r\n");
        }
    }
    else if (strcmp(argv[1], "accel") == 0)
    {
        DM_IMU_DATA_t *data = get_dm_imu_data();
        if (data != NULL)
        {
            shell_module_printf("Acceleration Data (m/s^2):\r\n");
            shell_module_printf("  X: %.4f\r\n", data->imu_data.acc[0]);
            shell_module_printf("  Y: %.4f\r\n", data->imu_data.acc[1]);
            shell_module_printf("  Z: %.4f\r\n", data->imu_data.acc[2]);
        }
        else
        {
            shell_module_printf("No acceleration data available\r\n");
        }
    }
    else if (strcmp(argv[1], "gyro") == 0)
    {
        DM_IMU_DATA_t *data = get_dm_imu_data();
        if (data != NULL)
        {
            shell_module_printf("Gyroscope Data (rad/s):\r\n");
            shell_module_printf("  X: %.4f\r\n", data->imu_data.gyro[0]);
            shell_module_printf("  Y: %.4f\r\n", data->imu_data.gyro[1]);
            shell_module_printf("  Z: %.4f\r\n", data->imu_data.gyro[2]);
        }
        else
        {
            shell_module_printf("No gyroscope data available\r\n");
        }
    }
    else if (strcmp(argv[1], "euler") == 0)
    {
        DM_IMU_DATA_t *data = get_dm_imu_data();
        if (data != NULL)
        {
            shell_module_printf("Euler Angles (deg):\r\n");
            shell_module_printf("  Pitch: %.2f\r\n", data->estimate.Pitch);
            shell_module_printf("  Roll: %.2f\r\n", data->estimate.Roll);
            shell_module_printf("  Yaw: %.2f\r\n", data->estimate.Yaw);
            shell_module_printf("  Total Yaw: %.2f\r\n", data->estimate.YawTotalAngle);
            shell_module_printf("  Yaw Count: %d\r\n", data->estimate.YawRoundCount);
        }
        else
        {
            shell_module_printf("No Euler angles data available\r\n");
        }
    }
    else if (strcmp(argv[1], "quat") == 0)
    {
        DM_IMU_DATA_t *data = get_dm_imu_data();
        if (data != NULL)
        {
            shell_module_printf("Quaternion Data:\r\n");
            shell_module_printf("  W: %.4f\r\n", data->q[0]);
            shell_module_printf("  X: %.4f\r\n", data->q[1]);
            shell_module_printf("  Y: %.4f\r\n", data->q[2]);
            shell_module_printf("  Z: %.4f\r\n", data->q[3]);
        }
        else
        {
            shell_module_printf("No quaternion data available\r\n");
        }
    }
    else
    {
        shell_module_printf("Unknown command: %s\r\n", argv[1]);
        shell_module_printf("Use 'dm_imu' without arguments to see help.\r\n");
    }
}

void dm_imu_shell_cmd_init(DM_IMU_Moudule_t *module)
{
    if (module != NULL)
    {
        dm_imu_module = module;
        shell_module_register_cmd("dm_imu", dm_imu_shell_cmd, "DM IMU shell command");
    }
}
