#include "Chassis.h"

Chassis_Motor_PID_Expect RM3510_PID_Expect;
Chassis_Motor_Actual_Speed RM3510_Actual_Speedt;
ChassisSpeed_Ref_t Chassis_Speed_Ref;

PTZ_Motor_PID_Expect GM3510_PID_Expect;
PTZ_Motor_Actual GM3510_Actual;
PTZAngle_Ref_t GM3510_Ref;

float Pluck_motor_Actual_Rand;
short Pluck_motor_Actual_Speed;
float Pluck_motor_PID_Expect;

void Get_PID_Expect(short speed)
{
	RM3510_PID_Expect.Chassis_Motor_PID_Expect_1 = -speed;
	RM3510_PID_Expect.Chassis_Motor_PID_Expect_2 = speed;
	RM3510_PID_Expect.Chassis_Motor_PID_Expect_3 = -speed;
	RM3510_PID_Expect.Chassis_Motor_PID_Expect_4 = speed;
}

void Chassis_Speed_Ref_Init(void)
{
	Chassis_Speed_Ref.forward_back_ref = 0;
	Chassis_Speed_Ref.left_right_ref = 0;
	Chassis_Speed_Ref.rotate_ref = 0;
	GM3510_Ref.x = 0;
	GM3510_Ref.y = PTZ_Y_median;
}

void PTZ_PID_Expect(void) //得到云台电机所期望的角度
{
	GM3510_PID_Expect.PTZ_Motor_PID_Expect_1 = GM3510_Ref.x;
	GM3510_PID_Expect.PTZ_Motor_PID_Expect_2 = GM3510_Ref.y;
}

void PID_Expect(void) //得到底盘电机所期望的速度
{
	float angle = (PTZ_X_median - GM3510_Actual.PTZ_Motor_Actual_Angle_1)/1303.88f;
	
	int16_t forward_back = 0;
	int16_t left_right = 0;
	
	forward_back = Chassis_Speed_Ref.forward_back_ref * cosf(angle) + Chassis_Speed_Ref.left_right_ref * sinf(angle);
	left_right = Chassis_Speed_Ref.forward_back_ref * sinf(angle) + Chassis_Speed_Ref.left_right_ref * cosf(angle);

	RM3510_PID_Expect.Chassis_Motor_PID_Expect_1 = -forward_back -
												   left_right + Chassis_Speed_Ref.rotate_ref;

	RM3510_PID_Expect.Chassis_Motor_PID_Expect_2 = forward_back -
												   left_right + Chassis_Speed_Ref.rotate_ref;

	RM3510_PID_Expect.Chassis_Motor_PID_Expect_4 = forward_back +
												   left_right + Chassis_Speed_Ref.rotate_ref;

	RM3510_PID_Expect.Chassis_Motor_PID_Expect_3 = -forward_back +
												   left_right + Chassis_Speed_Ref.rotate_ref;
}
