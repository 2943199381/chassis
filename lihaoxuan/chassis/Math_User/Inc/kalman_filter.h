//
// Created by YangShuai on 2022/12/28.
//

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include "main.h"
#include "arm_math.h"

#define mat         arm_matrix_instance_f32
#define mat_64      arm_matrix_instance_f64
#define mat_init    arm_mat_init_f32
#define mat_add     arm_mat_add_f32
#define mat_sub     arm_mat_sub_f32
#define mat_mult    arm_mat_mult_f32
#define mat_trans   arm_mat_trans_f32
#define mat_inv     arm_mat_inverse_f32
#define mat_inv_f64 arm_mat_inverse_f64

#define SYS_DT 0.001f

typedef struct
{
	float raw_value;
	float filtered_value[2];
	mat xhat, xhatminus, z, A, H, AT, HT, Q, R, P, Pminus, K;
    /*****xhat后验估计 P后验方差估计 xhatminus先验估计 Pminus先验方差估计 K Kalman增益
    需要改变：Q过程激励噪声方差 R测量方差估计
    Q增大，动态响应变快，收敛稳定性变坏；反之。控制误差
    R增大，动态响应变慢，收敛稳定性变好；反之。控制响应速度*****/
} kalman_filter_t;

typedef struct
{
	float raw_value;
	float filtered_value[2];
	float xhat_data[2], xhatminus_data[2], z_data[2],Pminus_data[4], K_data[4];
	float P_data[4];
	float AT_data[4], HT_data[4];
	float A_data[4];
	float H_data[4];
	float Q_data[4];
	float R_data[4];
} kalman_filter_init_t;


void   kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I);
float *kalman_filter_calc(kalman_filter_t *F, float signal1, float signal2);


#endif //KALMAN_FILTER_H
