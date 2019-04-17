#include "stm32f4xx_hal.h"
#include "arm_math.h"
#include "filter.h"

kalman_filter_init_t yaw_kalman_filter_data = {
    .P_data = {2, 0, 0, 2},     // Э�������
    .A_data = {1, 0.005, 0, 1}, // Ԥ����� ������ʱ�䣩
    .H_data = {1, 0, 0, 1},     // �������������ݾ���
    .Q_data = {0.1, 0, 0, 0.1}, // �ⲿ�Ĳ�ȷ���ԣ���ȷ���ԣ�
    .R_data = {200, 0, 0, 200}, // ��������������ɼ����ݷ��
    .E_data = {1, 0, 0, 1}      // ��λ����

    //	.P_data = {1,1,1,1},           // Э�������
    //  .A_data = {1,0.001,0,1},        // Ԥ����� ������ʱ�䣩
    //  .H_data = {1, 0, 0, 1}, 					// �������������ݾ���
    //  .Q_data = {0.01,0,0,0.01},           // �ⲿ�Ĳ�ȷ���ԣ���ȷ���ԣ�
    //  .R_data = {0.2879,0.0213,0.0213,1.8787},      // ��������������ɼ����ݷ��
    //	.E_data = {1,0,0,1}            // ��λ����
};

kalman_filter_t yaw_kalman_filter;
speed_calc_data_t yaw_speed_struct;

void kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I)
{

  mat_init(&F->xhat, 2, 1, (float *)I->xhat_data); //  ǰ�к���
  mat_init(&F->xhatminus, 2, 1, (float *)I->xhatminus_data);
  mat_init(&F->z, 2, 1, (float *)I->z_data);
  mat_init(&F->A, 2, 2, (float *)I->A_data);
  mat_init(&F->H, 2, 2, (float *)I->H_data);
  mat_init(&F->AT, 2, 2, (float *)I->AT_data);
  mat_trans(&F->A, &F->AT);
  mat_init(&F->Q, 2, 2, (float *)I->Q_data);
  mat_init(&F->R, 2, 2, (float *)I->R_data);
  mat_init(&F->P, 2, 2, (float *)I->P_data);
  mat_init(&F->Pminus, 2, 2, (float *)I->Pminus_data);
  mat_init(&F->K, 2, 2, (float *)I->K_data);
  mat_init(&F->HT, 2, 2, (float *)I->HT_data);
  mat_trans(&F->H, &F->HT);
  mat_init(&F->E, 2, 2, (float *)I->E_data);
}

void kalman_filter_calc(kalman_filter_t *F, float signal1, float signal2)
{
  float TEMP_data[4] = {0, 0, 0, 0};
  float TEMP_data21[2] = {0, 0};
  mat TEMP, TEMP21;

  mat_init(&TEMP, 2, 2, (float *)TEMP_data);
  mat_init(&TEMP21, 2, 1, (float *)TEMP_data21);

  F->z.pData[0] = signal1; // ����������λ��ֵ
  F->z.pData[1] = signal2; // �����������ٶ�ֵ

  //1. xhat'(k)= A xhat(k-1)
  mat_mult(&F->A, &F->xhat, &F->xhatminus); //����k-1ʱ�̵�ֵԤ��kʱ�̵�ֵ

  //2. P'(k) = A P(k-1) AT + Q
  mat_mult(&F->A, &F->P, &F->Pminus); //����k-1ʱ�̵ķ���Ԥ��kʱ�̵ķ���
  mat_mult(&F->Pminus, &F->AT, &TEMP);
  mat_add(&TEMP, &F->Q, &F->Pminus);

  //3. K(k) = P'(k) HT / (H P'(k) HT + R)     //���㿨��������
  mat_mult(&F->H, &F->Pminus, &F->K); //�������
  mat_mult(&F->K, &F->HT, &TEMP);
  mat_add(&TEMP, &F->R, &F->K);

  mat_inv(&F->K, &F->P); //����ת��
  mat_mult(&F->Pminus, &F->HT, &TEMP);
  mat_mult(&TEMP, &F->P, &F->K); //���¿���������

  //4. xhat(k) = xhat'(k) + K(k) (z(k) - H xhat'(k))
  mat_mult(&F->H, &F->xhatminus, &TEMP21); //Ԥ��ľ�ֵ
  mat_sub(&F->z, &TEMP21, &F->xhat);
  mat_mult(&F->K, &F->xhat, &TEMP21);
  mat_add(&F->xhatminus, &TEMP21, &F->xhat); //�����ѹ���ֵ

  //5. P(k) = (1-K(k)H)P'(k)
  mat_mult(&F->K, &F->H, &F->P); //��������Э����
  mat_sub(&F->E, &F->P, &TEMP);
  mat_mult(&TEMP, &F->Pminus, &F->P);

  F->filtered_value[0] = F->xhat.pData[0];
  F->filtered_value[1] = F->xhat.pData[1];
}

/*********����Ŀ���ٶ�ֵ*********/
float speed_threshold = 10.0f; // �ٶ���ֵ
float target_speed_calc(speed_calc_data_t *S, float time, float position)
{
  S->delay_cnt++;

  if (time != S->last_time)
  {
    S->speed = (position - S->last_position) / (time - S->last_time) * 1000;
#if 1
    if ((S->speed - S->processed_speed) < -speed_threshold)
    {
      S->processed_speed = S->processed_speed - speed_threshold;
    }
    else if ((S->speed - S->processed_speed) > speed_threshold)
    {
      S->processed_speed = S->processed_speed + speed_threshold;
    }
    else
#endif
      S->processed_speed = S->speed;

    S->last_time = time;
    S->last_position = position;
    S->last_speed = S->speed;
    S->delay_cnt = 0;
  }

  if (S->delay_cnt > 200) // delay 200ms speed = 0
  {
    S->processed_speed = 0;
  }

  return S->processed_speed;
}
