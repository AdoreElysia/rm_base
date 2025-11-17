/**
 ******************************************************************************
 * @file    kalman filter.h
 * @author  Wang Hongxi
 * @version V1.2.2
 * @date    2022/1/8
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef __KALMAN_FILTER_H
#define __KALMAN_FILTER_H

// cortex-m4 DSP lib
/*
#define __CC_ARM    // Keil
#define ARM_MATH_CM4
#define ARM_MATH_MATRIX_CHECK
#define ARM_MATH_ROUNDING
#define ARM_MATH_DSP    // define in arm_math.h
*/

#include "arm_math.h"
#include "stdint.h"
#include "tx_api.h"

// 最大支持的状态变量、控制变量和观测变量维度
#define KF_MAX_XHAT_SIZE 10
#define KF_MAX_U_SIZE    1
#define KF_MAX_Z_SIZE    5
// 矩阵定义
#define mat              arm_matrix_instance_f32
#define Matrix_Init      arm_mat_init_f32
#define Matrix_Add       arm_mat_add_f32
#define Matrix_Subtract  arm_mat_sub_f32
#define Matrix_Multiply  arm_mat_mult_f32
#define Matrix_Transpose arm_mat_trans_f32
#define Matrix_Inverse   arm_mat_inverse_f32

typedef struct kf_t
{
    float *FilteredValue;
    float *MeasuredVector;
    float *ControlVector;

    uint8_t xhatSize;
    uint8_t uSize;
    uint8_t zSize;

    uint8_t UseAutoAdjustment;
    uint8_t MeasurementValidNum;

    uint8_t *MeasurementMap;        // 量测与状态的关系 how measurement relates to the state
    float   *MeasurementDegree;     // 测量值对应H矩阵元素值 elements of each measurement in H
    float   *MatR_DiagonalElements; // 量测方差 variance for each measurement
    float   *StateMinVariance; // 最小方差 避免方差过度收敛 suppress filter excessive convergence
    uint8_t *temp;

    // 配合用户定义函数使用,作为标志位用于判断是否要跳过标准KF中五个环节中的任意一个
    uint8_t SkipEq1, SkipEq2, SkipEq3, SkipEq4, SkipEq5;

    // definiion of struct mat: rows & cols & pointer to vars
    mat xhat;      // x(k|k)
    mat xhatminus; // x(k|k-1)
    mat u;         // control vector u
    mat z;         // measurement vector z
    mat P;         // covariance matrix P(k|k)
    mat Pminus;    // covariance matrix P(k|k-1)
    mat F, FT;     // state transition matrix F FT
    mat B;         // control matrix B
    mat H, HT;     // measurement matrix H
    mat Q;         // process noise covariance matrix Q
    mat R;         // measurement noise covariance matrix R
    mat K;         // kalman gain  K
    mat S, temp_matrix, temp_matrix1, temp_vector, temp_vector1;

    int8_t MatStatus;

    // 用户定义函数,可以替换或扩展基准KF的功能
    void (*User_Func0_f)(struct kf_t *kf);
    void (*User_Func1_f)(struct kf_t *kf);
    void (*User_Func2_f)(struct kf_t *kf);
    void (*User_Func3_f)(struct kf_t *kf);
    void (*User_Func4_f)(struct kf_t *kf);
    void (*User_Func5_f)(struct kf_t *kf);
    void (*User_Func6_f)(struct kf_t *kf);

    // 矩阵存储空间指针
    float *xhat_data, *xhatminus_data;
    float *u_data;
    float *z_data;
    float *P_data, *Pminus_data;
    float *F_data, *FT_data;
    float *B_data;
    float *H_data, *HT_data;
    float *Q_data;
    float *R_data;
    float *K_data;
    float *S_data, *temp_matrix_data, *temp_matrix_data1, *temp_vector_data, *temp_vector_data1;

    // 静态内存分配的数据区域
    // measurement flags data
    uint8_t MeasurementMap_Data[KF_MAX_Z_SIZE];
    float   MeasurementDegree_Data[KF_MAX_Z_SIZE];
    float   MatR_DiagonalElements_Data[KF_MAX_Z_SIZE];
    float   StateMinVariance_Data[KF_MAX_XHAT_SIZE];
    uint8_t temp_Data[KF_MAX_Z_SIZE];

    // filter data
    float FilteredValue_Data[KF_MAX_XHAT_SIZE];
    float MeasuredVector_Data[KF_MAX_Z_SIZE];
    float ControlVector_Data[KF_MAX_U_SIZE];

    // xhat and xhatminus data
    float xhat_data_Data[KF_MAX_XHAT_SIZE];
    float xhatminus_data_Data[KF_MAX_XHAT_SIZE];

    // u data
    float u_data_Data[KF_MAX_U_SIZE];

    // z data
    float z_data_Data[KF_MAX_Z_SIZE];

    // P and Pminus data
    float P_data_Data[KF_MAX_XHAT_SIZE * KF_MAX_XHAT_SIZE];
    float Pminus_data_Data[KF_MAX_XHAT_SIZE * KF_MAX_XHAT_SIZE];

    // F and FT data
    float F_data_Data[KF_MAX_XHAT_SIZE * KF_MAX_XHAT_SIZE];
    float FT_data_Data[KF_MAX_XHAT_SIZE * KF_MAX_XHAT_SIZE];

    // B data
    float B_data_Data[KF_MAX_XHAT_SIZE * KF_MAX_U_SIZE];

    // H and HT data
    float H_data_Data[KF_MAX_Z_SIZE * KF_MAX_XHAT_SIZE];
    float HT_data_Data[KF_MAX_XHAT_SIZE * KF_MAX_Z_SIZE];

    // Q data
    float Q_data_Data[KF_MAX_XHAT_SIZE * KF_MAX_XHAT_SIZE];

    // R data
    float R_data_Data[KF_MAX_Z_SIZE * KF_MAX_Z_SIZE];

    // K data
    float K_data_Data[KF_MAX_XHAT_SIZE * KF_MAX_Z_SIZE];

    // temp data
    float S_data_Data[KF_MAX_XHAT_SIZE * KF_MAX_XHAT_SIZE];
    float temp_matrix_data_Data[KF_MAX_XHAT_SIZE * KF_MAX_XHAT_SIZE];
    float temp_matrix_data1_Data[KF_MAX_XHAT_SIZE * KF_MAX_XHAT_SIZE];
    float temp_vector_data_Data[KF_MAX_XHAT_SIZE];
    float temp_vector_data1_Data[KF_MAX_XHAT_SIZE];
} KalmanFilter_t;
/**
 * @brief 初始化矩阵维度信息并为矩阵分配空间
 * @param kf kf类型定义
 * @param xhatSize 状态变量维度
 * @param uSize 控制变量维度
 * @param zSize 观测量维度
 */
void Kalman_Filter_Init(KalmanFilter_t *kf, uint8_t xhatSize, uint8_t uSize, uint8_t zSize);
/**
 * @brief 获取测量数据
 * @param kf kf类型定义
 */
void Kalman_Filter_Measure(KalmanFilter_t *kf);
/**
 * @brief 先验估计 xhat'(k)= A·xhat(k-1) + B·u
 * @param kf kf类型定义
 */
void Kalman_Filter_xhatMinusUpdate(KalmanFilter_t *kf);
/**
 * @brief 预测更新 P'(k) = A·P(k-1)·AT + Q
 * @param kf kf类型定义
 */
void Kalman_Filter_PminusUpdate(KalmanFilter_t *kf);
/**
 * @brief 计算卡尔曼增益 K(k) = P'(k)·HT / (H·P'(k)·HT + R)
 * @param kf kf类型定义
 */
void Kalman_Filter_SetK(KalmanFilter_t *kf);
/**
 * @brief 状态更新 xhat(k) = xhat'(k) + K(k)·(z(k) - H·xhat'(k))
 * @param kf kf类型定义
 */
void Kalman_Filter_xhatUpdate(KalmanFilter_t *kf);
/**
 * @brief 协方差更新 P(k) = (1-K(k)·H)·P'(k) ==> P(k) = P'(k)-K(k)·H·P'(k)
 * @param kf kf类型定义
 */
void Kalman_Filter_P_Update(KalmanFilter_t *kf);
/**
 * @brief
 * 执行卡尔曼滤波黄金五式,提供了用户定义函数,可以替代五个中的任意一个环节,方便自行扩展为EKF/UKF/ESKF/AUKF等
 * @param kf kf类型定义
 * @return float* 返回滤波值
 */
float *Kalman_Filter_Update(KalmanFilter_t *kf);

#endif //__KALMAN_FILTER_H
