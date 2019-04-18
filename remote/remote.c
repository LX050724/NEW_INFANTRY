#include "remote.h"

short can_stop[4] = {0, 0, 0, 0};

unsigned char s1_flag1, s1_flag2;
unsigned char Friction_wheel;

unsigned short mousepL_flag, mousepR_flag;
uint8_t down_play_flag = 1, down_FB_flag = 1, down_RL_flag = 1;
uint8_t play_flag = 1, FB_flag = 1, RL_flag = 1, rr_flag = 1;
int16_t FB_SD, RL_SD;

uint8_t Twist = 0;

static InputMode_e inputmode = REMOTE_INPUT; //����ģʽ�趨
RC_Ctl_t RC_CtrlData = {.rc = {1024,1024,1024,1024,3,1}};												 //remote control data

//����ģʽ����
void SetInputMode(Remote *rc)
{
	if (rc->s2 == 1)
	{
		inputmode = REMOTE_INPUT;
	}
	else if (rc->s2 == 3)
	{
		inputmode = KEY_MOUSE_INPUT;
	}
	else if (rc->s2 == 2)
	{
		inputmode = STOP;
	}
}

InputMode_e GetInputMode()
{
	return inputmode;
}

/*���̿��ƺ���*/
void MouseKeyControlProcess(Mouse *mouse, Key *key)
{
	static uint16_t forward_back_speed = 0;
	static uint16_t left_right_speed = 0;

	forward_back_speed = 5300;
	left_right_speed = 5300;
	//�������///////////////////////////////////////////////////////////
	if (key->v & 0x01) // key: w   ǰ��
	{
		ramp = CHASSIS_RAMP_FB;

		down_FB_flag = 0;
		if (FB_flag == 0)
		{
			ResetSlope(ramp);
			FB_flag = 1;
		}
		Chassis_Speed_Ref.forward_back_ref = forward_back_speed * Slope(70000, ramp);

		FB_SD = Chassis_Speed_Ref.forward_back_ref;
		play_flag = 0;
	}
	else if (key->v & 0x02) //key: s  ����
	{
		ramp = CHASSIS_RAMP_FB;

		down_FB_flag = 0;
		if (play_flag == 0)
		{
			ResetSlope(ramp);
			play_flag = 1;
		}
		FB_flag = 0;
		Chassis_Speed_Ref.forward_back_ref = -forward_back_speed * Slope(70000, ramp);
	}
	else
	{
		ramp = CHASSIS_RAMP_FB;
		ResetSlope(ramp);
		Chassis_Speed_Ref.forward_back_ref = 0;
	}
	if (key->v & 0x08) // key: d ����
	{
		ramp = CHASSIS_RAMP_RL;

		down_RL_flag = 0;
		if (RL_flag == 0)
		{
			ResetSlope(ramp);
			RL_flag = 1;
		}

		Chassis_Speed_Ref.left_right_ref = left_right_speed * Slope(70000, ramp);
		//			RL_SD = Chassis_Speed_Ref.left_right_ref ;

		rr_flag = 0;
		//			printf("d\r\n");
	}
	else if (key->v & 0x04) //key: a ����
	{
		ramp = CHASSIS_RAMP_RL;
		down_RL_flag = 0;

		if (rr_flag == 0)
		{
			ResetSlope(ramp);
			rr_flag = 1;
		}
		Chassis_Speed_Ref.left_right_ref = -left_right_speed * Slope(70000, ramp);
		RL_SD = Chassis_Speed_Ref.left_right_ref;
		RL_flag = 0;
		//			printf("a\r\n");
	}
	else
	{
		ramp = CHASSIS_RAMP_RL;
		down_ramp = CHASSIS_RAMP_RL;
		ResetSlope(ramp);
		Chassis_Speed_Ref.left_right_ref = 0;
		//			if(down_RL_flag == 0)
		//			{
		//			  Reset_down_slope(down_ramp);
		//				down_RL_flag = 1;
		//			}
		//		  Chassis_Speed_Ref.left_right_ref = RL_SD * down_slope(5000,down_ramp);
		//			Chassis_Speed_Ref.left_right_ref = 0;
	}

	///Ħ���ֿ���/////////////////////////////////////////////////////////
	if ((mouse->press_l) && (Friction_wheel == 1)) //���������Ħ���ֿ�
	{
		if (mouse->last_press_l) //�����һ�ΰ��£���ʼ��ʱ
		{
			mousepL_flag++;
			if (mousepL_flag == 15) //����ǵ�100
			{
				mousepL_flag = 0;														//����
				Pluck_motor_PID_Expect += Pluck_motor_rand; //����
			}
		}
		else
			Pluck_motor_PID_Expect += Pluck_motor_rand; //���Ϊ��һ�ΰ���ֱ�Ӳ���
	}
	else
		mousepL_flag = 0; //�ɿ�����

	if (mouse->press_r) //�Ҽ�����
	{
		if ((Friction_wheel == 0) && (!mouse->last_press_r)) //���Ħ���ֹر���Ϊ��һ�ΰ���
		{
			TIM12->CCR1 = Friction_wheel_speed;
			TIM12->CCR2 = Friction_wheel_speed;
			HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
			Friction_wheel = 1;
		}
		else if (Friction_wheel == 1)
		{
			mousepR_flag++;
			if (mousepR_flag == 100) //����ǵ�200
			{
				TIM12->CCR1 = 1000;
				TIM12->CCR2 = 1000;
				HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_RESET);
				Friction_wheel = 0;
				
			}
		}
	}
	else
		mousepR_flag = 0; //����

	///��̨����///////////////////////////////////////////////////////////
	VAL_LIMIT(mouse->x, -90, 90);
	VAL_LIMIT(mouse->y, -120, 120);
	
	if(abs(GM3510_Actual.PTZ_Motor_Actual_Angle_1 - PTZ_X_median)<PTZ_X_LIMIT)
		GM3510_Ref.x += mouse->x * MOUSE_TO_YAW_ANGLE_INC_FACT;
	
	if ((GM3510_Ref.y > PTZ_Y_MAX) && ((mouse->y* MOUSE_TO_PITCH_ANGLE_INC_FACT) < 0))
		GM3510_Ref.y += mouse->y* MOUSE_TO_PITCH_ANGLE_INC_FACT;
	if ((GM3510_Ref.y < PTZ_Y_MIN) && ((mouse->y* MOUSE_TO_PITCH_ANGLE_INC_FACT) > 0))
		GM3510_Ref.y += mouse->y* MOUSE_TO_PITCH_ANGLE_INC_FACT;
	///��������///////////////////////////////////////////////////////////
	
	///Ť��E//////////////////////////////////////////////////////////////
	static uint8_t Key_E_falg = 0;
	if(key->v & Key_E)
	{
		if(Key_E_falg == 0)
		{
			Key_E_falg = 1;
			if(Twist == 0)Twist = 1;
			else Twist = 0;
		}
	}else Key_E_falg = 0;
	
}
/*ң�������ƺ���*/
void RemoteShootControl(int8_t s1)
{
	switch (s1)
	{
	case 1:
	{
		if (s1_flag1 == 0)
		{
			if (Friction_wheel == 0)
			{
				TIM12->CCR1 = Friction_wheel_speed;
				TIM12->CCR2 = Friction_wheel_speed;
				HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
				Friction_wheel = 1;
			}
			else
			{
				TIM12->CCR1 = 1000;
				TIM12->CCR2 = 1000;
				HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_RESET);
				Friction_wheel = 0;
			}
		}
		s1_flag1 = 1;
		break;
	}
	case 3:
	{
		s1_flag1 = 0;
		s1_flag2 = 0;
		HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET);
#if SpeedMode == 1
		Pluck_motor_PID_Expect = 0;
#endif
		break;
	}
	case 2:
	{
		if ((s1_flag2 == 0) && (Friction_wheel == 1))
		{
#if SpeedMode == 0
			Pluck_motor_PID_Expect += Pluck_motor_rand;
#else
			Pluck_motor_PID_Expect = Pluck_motor_speed;
#endif
			HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET);
			s1_flag2 = 1;
		}
		break;
	}
	}
}

//ң��������ģʽ����
void RemoteControlProcess(Remote *rc)
{
	float dx;
	short dy;
	Chassis_Speed_Ref.forward_back_ref = (RC_CtrlData.rc.ch1 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_CHASSIS_SPEED_REF_FACT;
	Chassis_Speed_Ref.left_right_ref = (rc->ch0 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_CHASSIS_SPEED_REF_FACT;

	dx = -(rc->ch2 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * Sensitivity_X;
	dy = (RC_CtrlData.rc.ch3 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * Sensitivity_Y;
	
	if(abs(GM3510_Actual.PTZ_Motor_Actual_Angle_1 - PTZ_X_median)<PTZ_X_LIMIT)
		GM3510_Ref.x += dx;
	
	if ((GM3510_Ref.y > PTZ_Y_MAX) && (dy < 0))
		GM3510_Ref.y += dy;
	if ((GM3510_Ref.y < PTZ_Y_MIN) && (dy > 0))
		GM3510_Ref.y += dy;

	/* not used to control, just as a flag */
	RemoteShootControl(rc->s1);
}

void Remote_Rx(unsigned char *RxMsg)
{
	RC_CtrlData.rc.ch0 = ((int16_t)RxMsg[0] | ((int16_t)RxMsg[1] << 8)) & 0x07FF;
	RC_CtrlData.rc.ch1 = (((int16_t)RxMsg[1] >> 3) | ((int16_t)RxMsg[2] << 5)) & 0x07FF;
	RC_CtrlData.rc.ch2 = (((int16_t)RxMsg[2] >> 6) | ((int16_t)RxMsg[3] << 2) |
												((int16_t)RxMsg[4] << 10)) & 0x07FF;
	RC_CtrlData.rc.ch3 = (((int16_t)RxMsg[4] >> 1) | ((int16_t)RxMsg[5] << 7)) & 0x07FF;

	RC_CtrlData.rc.s1 = ((RxMsg[5] >> 4) & 0x000C) >> 2;
	RC_CtrlData.rc.s2 = ((RxMsg[5] >> 4) & 0x0003);

	RC_CtrlData.mouse.x = ((int16_t)RxMsg[6]) | ((int16_t)RxMsg[7] << 8);
	RC_CtrlData.mouse.y = ((int16_t)RxMsg[8]) | ((int16_t)RxMsg[9] << 8);
	RC_CtrlData.mouse.z = ((int16_t)RxMsg[10]) | ((int16_t)RxMsg[11] << 8);

	RC_CtrlData.mouse.press_l = RxMsg[12];
	RC_CtrlData.mouse.press_r = RxMsg[13];

	RC_CtrlData.key.v = ((int16_t)RxMsg[14]);

	SetInputMode(&RC_CtrlData.rc);

	switch (GetInputMode())
	{
	case REMOTE_INPUT:
	{
		//ң��������ģʽ
		RemoteControlProcess(&(RC_CtrlData.rc));
	}
	break;

	case KEY_MOUSE_INPUT:
	{
		//�������ģʽ
		MouseKeyControlProcess(&RC_CtrlData.mouse, &RC_CtrlData.key);
	}
	break;
	case STOP:
	{
		TIM12->CCR1 = 1000;
		TIM12->CCR2 = 1000;
		Chassis_Speed_Ref_Init();
		HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
		PID_DeInit();
		vTaskDelete(&PID_task_Handler);
		vTaskDelete(&PID_task2_Handler);
		RM3510_CAN_Send(can_stop);
		vTaskDelay(5);
		GM3510_CAN_Send(can_stop);
	}
	break;
	}
	RC_CtrlData.mouse.last_press_l = RC_CtrlData.mouse.press_l;
	RC_CtrlData.mouse.last_press_r = RC_CtrlData.mouse.press_r;
}
