#ifndef __FILTER_H
#define __FILTER_H

#include "sys.h"
#include "arm_math.h"
#include "stm32f4xx_hal.h"

#define mat arm_matrix_instance_f32
#define mat_init arm_mat_init_f32
#define mat_add arm_mat_add_f32
#define mat_sub arm_mat_sub_f32
#define mat_mult arm_mat_mult_f32
#define mat_trans arm_mat_trans_f32
#define mat_inv arm_mat_inverse_f32

typedef struct
{
  float raw_value;
  float filtered_value[2];
  mat xhat, xhatminus, z, A, H, AT, HT, Q, R, P, Pminus, K, E;
} kalman_filter_t;

typedef struct
{
  float raw_value;
  float filtered_value[2];
  float xhat_data[2], xhatminus_data[2], z_data[2], Pminus_data[4], K_data[4];
  float P_data[4];
  float AT_data[4], HT_data[4];
  float A_data[4];
  float H_data[4];
  float Q_data[4];
  float R_data[4];
  float E_data[4];
} kalman_filter_init_t;

typedef struct
{
  int delay_cnt;
  int freq;
  int last_time;
  float last_position;
  float speed;
  float last_speed;
  float processed_speed;
} speed_calc_data_t;

extern kalman_filter_t yaw_kalman_filter;
extern kalman_filter_init_t yaw_kalman_filter_data, y_angle_filter_data;
extern speed_calc_data_t yaw_speed_struct;

extern float speed_threshold;

void kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I);
void kalman_filter_calc(kalman_filter_t *F, float signal1, float signal2);
float target_speed_calc(speed_calc_data_t *S, float time, float position);

#endif
