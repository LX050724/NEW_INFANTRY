#include "PID.h"

PID RM3510_1_PID;
PID RM3510_2_PID;
PID RM3510_3_PID;
PID RM3510_4_PID;

PID GM3510_1_AnglePID;
PID GM3510_1_SpeedPID;
PID GM3510_2_AnglePID;
PID GM3510_2_SpeedPID;

PID Pluck_motor_AnglePID;
PID Pluck_motor_SpeedPID;

PID Chassis_PID;

void PID_Init_MPU(void)
{
	/***************云台电机1位置PID*****************/
	GM3510_1_AnglePID.error_inter = 0;
	GM3510_1_AnglePID.error_last = 0;
	GM3510_1_AnglePID.error_now = 0;
	GM3510_1_AnglePID.Kd = 1;
	GM3510_1_AnglePID.Ki = 0;
	GM3510_1_AnglePID.Kp = 0.36;

	GM3510_1_SpeedPID.error_inter = 0;
	GM3510_1_SpeedPID.error_last = 0;
	GM3510_1_SpeedPID.error_now = 0;
	GM3510_1_SpeedPID.Kd = 500;
	GM3510_1_SpeedPID.Ki = 0;
	GM3510_1_SpeedPID.Kp = 1000;
	
	/***************云台电机2位置PID*****************/
	GM3510_2_AnglePID.error_inter = 0;
	GM3510_2_AnglePID.error_last = 0;
	GM3510_2_AnglePID.error_now = 0;
	GM3510_2_AnglePID.Kd = 0.001;
	GM3510_2_AnglePID.Ki = 0;
	GM3510_2_AnglePID.Kp = 0.011;

	GM3510_2_SpeedPID.error_inter = 0;
	GM3510_2_SpeedPID.error_last = 0;
	GM3510_2_SpeedPID.error_now = 0;
	GM3510_2_SpeedPID.Kd = 50;
	GM3510_2_SpeedPID.Ki = 0;
	GM3510_2_SpeedPID.Kp = 600;
}

void PID_Init(void)
{
	/****************底盘电机速度PID*****************/
	RM3510_1_PID.error_inter = 0;
	RM3510_1_PID.error_last = 0;
	RM3510_1_PID.error_now = 0;
	RM3510_1_PID.Kd = KD;
	RM3510_1_PID.Ki = KI;
	RM3510_1_PID.Kp = KP;

	RM3510_2_PID.error_inter = 0;
	RM3510_2_PID.error_last = 0;
	RM3510_2_PID.error_now = 0;
	RM3510_2_PID.Kd = KD;
	RM3510_2_PID.Ki = KI;
	RM3510_2_PID.Kp = KP;

	RM3510_3_PID.error_inter = 0;
	RM3510_3_PID.error_last = 0;
	RM3510_3_PID.error_now = 0;
	RM3510_3_PID.Kd = KD;
	RM3510_3_PID.Ki = KI;
	RM3510_3_PID.Kp = KP;

	RM3510_4_PID.error_inter = 0;
	RM3510_4_PID.error_last = 0;
	RM3510_4_PID.error_now = 0;
	RM3510_4_PID.Kd = KD;
	RM3510_4_PID.Ki = KI;
	RM3510_4_PID.Kp = KP;
	/***************云台电机1位置PID(初始化)***************/
	GM3510_1_AnglePID.error_inter = 0;
	GM3510_1_AnglePID.error_last = 0;
	GM3510_1_AnglePID.error_now = 0;
	GM3510_1_AnglePID.Kd = 10;
	GM3510_1_AnglePID.Ki = 0.00999999978;
	GM3510_1_AnglePID.Kp = 2.5;

	GM3510_1_SpeedPID.error_inter = 0;
	GM3510_1_SpeedPID.error_last = 0;
	GM3510_1_SpeedPID.error_now = 0;
	GM3510_1_SpeedPID.Kd = 2;
	GM3510_1_SpeedPID.Ki = 0;
	GM3510_1_SpeedPID.Kp = 4;
	/***************云台电机2位置PID*****************/
	GM3510_2_AnglePID.error_inter = 0;
	GM3510_2_AnglePID.error_last = 0;
	GM3510_2_AnglePID.error_now = 0;
	GM3510_2_AnglePID.Kd = 6;
	GM3510_2_AnglePID.Ki = 0.001;
	GM3510_2_AnglePID.Kp = 0.5;

	GM3510_2_SpeedPID.error_inter = 0;
	GM3510_2_SpeedPID.error_last = 0;
	GM3510_2_SpeedPID.error_now = 0;
	GM3510_2_SpeedPID.Kd = 5;
	GM3510_2_SpeedPID.Ki = 0;
	GM3510_2_SpeedPID.Kp = 4;
	/***************车身姿态PID******************/
	Chassis_PID.error_inter = 0;
	Chassis_PID.error_last = 0;
	Chassis_PID.error_now = 0;
	Chassis_PID.Kd = 10;
	Chassis_PID.Ki = 0;
	Chassis_PID.Kp = 1.5;
	/***************拨弹电机PID******************/
	Pluck_motor_AnglePID.error_inter = 0;
	Pluck_motor_AnglePID.error_last = 0;
	Pluck_motor_AnglePID.error_now = 0;
	Pluck_motor_AnglePID.Kd = 0;
	Pluck_motor_AnglePID.Ki = 0.01;
	Pluck_motor_AnglePID.Kp = 1000;

	Pluck_motor_SpeedPID.error_inter = 0;
	Pluck_motor_SpeedPID.error_last = 0;
	Pluck_motor_SpeedPID.error_now = 0;
	Pluck_motor_SpeedPID.Kd = 8;
	Pluck_motor_SpeedPID.Ki = 0;
	Pluck_motor_SpeedPID.Kp = 3;
}

void PID_Control(float current /*实际值*/, float expected /*期望值*/, PID *motor_type /*参数*/)
{

	motor_type->error_last = motor_type->error_now;
	motor_type->error_now = expected - current;
	motor_type->error_inter += motor_type->error_now;

	if (motor_type->error_inter > 10000)
		motor_type->error_inter = 10000;
	if (motor_type->error_inter < -10000)
		motor_type->error_inter = -10000;
	motor_type->pid_out = motor_type->Kp * motor_type->error_now + motor_type->Ki * motor_type->error_inter +
						  motor_type->Kd * (motor_type->error_now - motor_type->error_last);
}

void PID_DeInit(void)
{
	/****************底盘电机速度PID*****************/
	RM3510_1_PID.error_inter = 0;
	RM3510_1_PID.error_last = 0;
	RM3510_1_PID.error_now = 0;
	RM3510_1_PID.Kd = 0;
	RM3510_1_PID.Ki = 0;
	RM3510_1_PID.Kp = 0;

	RM3510_2_PID.error_inter = 0;
	RM3510_2_PID.error_last = 0;
	RM3510_2_PID.error_now = 0;
	RM3510_2_PID.Kd = 0;
	RM3510_2_PID.Ki = 0;
	RM3510_2_PID.Kp = 0;

	RM3510_3_PID.error_inter = 0;
	RM3510_3_PID.error_last = 0;
	RM3510_3_PID.error_now = 0;
	RM3510_3_PID.Kd = 0;
	RM3510_3_PID.Ki = 0;
	RM3510_3_PID.Kp = 0;

	RM3510_4_PID.error_inter = 0;
	RM3510_4_PID.error_last = 0;
	RM3510_4_PID.error_now = 0;
	RM3510_4_PID.Kd = 0;
	RM3510_4_PID.Ki = 0;
	RM3510_4_PID.Kp = 0;
	/***************云台电机1位置PID***************/
	GM3510_1_AnglePID.error_inter = 0;
	GM3510_1_AnglePID.error_last = 0;
	GM3510_1_AnglePID.error_now = 0;
	GM3510_1_AnglePID.Kd = 0;
	GM3510_1_AnglePID.Ki = 0;
	GM3510_1_AnglePID.Kp = 0;

	GM3510_1_SpeedPID.error_inter = 0;
	GM3510_1_SpeedPID.error_last = 0;
	GM3510_1_SpeedPID.error_now = 0;
	GM3510_1_SpeedPID.Kd = 0;
	GM3510_1_SpeedPID.Ki = 0;
	GM3510_1_SpeedPID.Kp = 0;
	/***************云台电机2位置PID*****************/
	GM3510_2_AnglePID.error_inter = 0;
	GM3510_2_AnglePID.error_last = 0;
	GM3510_2_AnglePID.error_now = 0;
	GM3510_2_AnglePID.Kd = 0;
	GM3510_2_AnglePID.Ki = 0;
	GM3510_2_AnglePID.Kp = 0;

	GM3510_2_SpeedPID.error_inter = 0;
	GM3510_2_SpeedPID.error_last = 0;
	GM3510_2_SpeedPID.error_now = 0;
	GM3510_2_SpeedPID.Kd = 0;
	GM3510_2_SpeedPID.Ki = 0;
	GM3510_2_SpeedPID.Kp = 0;
	/***************车身姿态PID******************/
	Chassis_PID.error_inter = 0;
	Chassis_PID.error_last = 0;
	Chassis_PID.error_now = 0;
	Chassis_PID.Kd = 0;
	Chassis_PID.Ki = 0;
	Chassis_PID.Kp = 0;
	/***************拨弹电机PID******************/
	Pluck_motor_AnglePID.error_inter = 0;
	Pluck_motor_AnglePID.error_last = 0;
	Pluck_motor_AnglePID.error_now = 0;
	Pluck_motor_AnglePID.Kd = 0;
	Pluck_motor_AnglePID.Ki = 0;
	Pluck_motor_AnglePID.Kp = 0;

	Pluck_motor_SpeedPID.error_inter = 0;
	Pluck_motor_SpeedPID.error_last = 0;
	Pluck_motor_SpeedPID.error_now = 0;
	Pluck_motor_SpeedPID.Kd = 0;
	Pluck_motor_SpeedPID.Ki = 0;
	Pluck_motor_SpeedPID.Kp = 0;
}
