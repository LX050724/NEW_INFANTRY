#ifndef _PID_H_
#define _PID_H_
#include "sys.h"
#include "config.h"

#define POWER_VOLTAGE                24.0f   //电源电压
#define POWER_LIMIT_UP               100     //输出功率上限
#define CURRENT_OFFSET               1000.0f //电流比例系数

#define limit(_IN_, _MAX_, _MIN_) \
	if (_IN_ < _MIN_)             \
		_IN_ = _MIN_;             \
	if (_IN_ > _MAX_)             \
		_IN_ = _MAX_;

typedef struct PID_PARAMETER
{
	float Kp;
	float Ki;
	float Kd;
	float error_now;
	float error_last;
	float error_inter;
	float pid_out;
} PID;

typedef struct PID_INCREASE
{
	float Kp;
	float Ki;
	float Kd;
	float error_now;
	float error_next;
	float error_last;
	float increament;
} PID_ADD;

typedef struct
{
	float NUM[3];
	float DEN[3];
	float x_now;
	float x_last;
	float x_next;
	float y_now;
	float y_last;
	float y_next;

} Diff;

void PID_Control(float current, float expected, PID *motor_type);
float PID_Increment(float current,float expect,PID_ADD *parameter);
void PID_Init(void);
void PID_DeInit(void);
void PID_Init_MPU(void);
float PID_diff_convert(float init_data, Diff *type);

extern Diff Diff_P;

extern PID RM3510_1_PID;
extern PID RM3510_2_PID;
extern PID RM3510_3_PID;
extern PID RM3510_4_PID;

extern PID GM3510_1_AnglePID;
extern PID GM3510_1_SpeedPID;
extern PID GM3510_2_AnglePID;
extern PID GM3510_2_SpeedPID;

extern PID Pluck_motor_AnglePID;
extern PID Pluck_motor_SpeedPID;

extern PID Chassis_PID;

extern PID_ADD Power_limit;

#endif
