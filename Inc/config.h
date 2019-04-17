#ifndef _CONFIG_H_
#define _CONFIG_H_

typedef enum
{
	ON = 1,
	OFF = 0,
} config_switch;

#define USART3_BUFLEN 32
#define USART3_BUFLENx2 USART3_BUFLEN*2
#define referee_system_QueueLength 5
#define referee_system_ItemSize		 USART3_BUFLEN

#define TEST 0		/*1����ģʽ*/
#define SpeedMode 0 /*1�����ٶ�ģʽ*/
#define PTZ_Lock 0  /*1��̨��������*/

/*��̨*/
#define PTZ_X_median 6190
#define PTZ_Y_median 1433

#define PTZ_Y_MAX 1000
#define PTZ_Y_MIN 1900

#define PTZ_X_LIMIT 1200

/*ҡ��������*/
#define Sensitivity_X -0.004f
#define Sensitivity_Y -0.1

#define MOUSE_TO_PITCH_ANGLE_INC_FACT 		2.0f
#define MOUSE_TO_YAW_ANGLE_INC_FACT 	   	0.08f
/*����PID*/
#define KP 3
#define KI 0
#define KD 2
/*Ħ����ת��*/
#define Friction_wheel_speed 1600

/*������ת��Ȧ��*/
#define Pluck_motor_rand 5.14f * 2.2f
#define Pluck_motor_speed 6000

#endif
