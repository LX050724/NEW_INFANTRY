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

PID_ADD Power_limit = {.Kp = 2.0f,.Ki = 0.32f,.Kd = 0.15f,0,0,0,0};
Diff Diff_P = {{0.1, 0.25, 0.2}, {1.2, -0.3, 0.2}};

void PID_Init_MPU(void)
{
	/***************云台电机1位置PID*****************/
	GM3510_1_AnglePID.error_inter = 0;
	GM3510_1_AnglePID.error_last = 0;
	GM3510_1_AnglePID.error_now = 0;
	GM3510_1_AnglePID.Kd = -1;
	GM3510_1_AnglePID.Ki = 0;
	GM3510_1_AnglePID.Kp = 0.2;

	GM3510_1_SpeedPID.error_inter = 0;
	GM3510_1_SpeedPID.error_last = 0;
	GM3510_1_SpeedPID.error_now = 0;
	GM3510_1_SpeedPID.Kd = 1000;
	GM3510_1_SpeedPID.Ki = 0;
	GM3510_1_SpeedPID.Kp = 1700;
	
	/***************云台电机2位置PID*****************/
	GM3510_2_AnglePID.error_inter = 0;
	GM3510_2_AnglePID.error_last = 0;
	GM3510_2_AnglePID.error_now = 0;
	GM3510_2_AnglePID.Kd = 4;
	GM3510_2_AnglePID.Ki = 0;
	GM3510_2_AnglePID.Kp = 6;

	GM3510_2_SpeedPID.error_inter = 0;
	GM3510_2_SpeedPID.error_last = 0;
	GM3510_2_SpeedPID.error_now = 0;
	GM3510_2_SpeedPID.Kd = 0.03;
	GM3510_2_SpeedPID.Ki = 0;
	GM3510_2_SpeedPID.Kp = 7;
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
	GM3510_1_AnglePID.Kd = 80;
	GM3510_1_AnglePID.Ki = 0;
	GM3510_1_AnglePID.Kp = 2.4;

	GM3510_1_SpeedPID.error_inter = 0;
	GM3510_1_SpeedPID.error_last = 0;
	GM3510_1_SpeedPID.error_now = 0;
	GM3510_1_SpeedPID.Kd = 0;
	GM3510_1_SpeedPID.Ki = 0;
	GM3510_1_SpeedPID.Kp = 4;
	/***************云台电机2位置PID*****************/
	GM3510_2_AnglePID.error_inter = 0;
	GM3510_2_AnglePID.error_last = 0;
	GM3510_2_AnglePID.error_now = 0;
	GM3510_2_AnglePID.Kd = 0.25;
	GM3510_2_AnglePID.Ki = 0;
	GM3510_2_AnglePID.Kp = 3;

	GM3510_2_SpeedPID.error_inter = 0;
	GM3510_2_SpeedPID.error_last = 0;
	GM3510_2_SpeedPID.error_now = 0;
	GM3510_2_SpeedPID.Kd = 0.03;
	GM3510_2_SpeedPID.Ki = 0;
	GM3510_2_SpeedPID.Kp = 4;
	/***************车身姿态PID******************/
	Chassis_PID.error_inter = 0;
	Chassis_PID.error_last = 0;
	Chassis_PID.error_now = 0;
	Chassis_PID.Kd = 20;
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

float PID_Increment(float current,float expect,PID_ADD *parameter)
{
	parameter->error_now=expect-current;
	
	parameter->increament=parameter->Kp*(parameter->error_now-parameter->error_next)+parameter->Ki*(parameter->error_now)+
	                       parameter->Kd*(parameter->error_now-2*parameter->error_next+parameter->error_last);
	
	parameter->error_last=parameter->error_next;
	parameter->error_next=parameter->error_now;
	
	return parameter->increament;
}

float PID_diff_convert(float init_data, Diff *type)
{
	type->x_now = init_data;

	type->y_now = (int)(type->NUM[0] * type->x_now + type->NUM[1] * type->x_last + type->NUM[2] * type->x_next - type->DEN[1] * type->y_last - type->DEN[2] * type->y_next);

	type->y_next = type->y_last;
	type->y_last = type->y_now;
	type->x_next = type->x_last;
	type->x_last = type->x_now;
	return type->y_now;
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
