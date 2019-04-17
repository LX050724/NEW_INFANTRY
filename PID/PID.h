#ifndef _PID_H_
#define _PID_H_
#include "sys.h"
#include "config.h"

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
	float error_now;
	float error_next;
	float error_last;
	float increament;
} PID_ADD;

void PID_Control(float current, float expected, PID *motor_type);
void PID_Init(void);
void PID_DeInit(void);
void PID_Init_MPU(void);

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

#endif
