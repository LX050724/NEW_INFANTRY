#ifndef _CHASSIS_H_
#define _CHASSIS_H_
#include "sys.h"
#include "PID.h"
#include "remote.h"

typedef __packed struct
{
     short forward_back_ref;
     short left_right_ref;
     short rotate_ref;
} ChassisSpeed_Ref_t;

typedef __packed struct
{
     float x;
     short y;
} PTZAngle_Ref_t;

typedef __packed struct
{
     short Chassis_Motor_PID_Expect_1;
     short Chassis_Motor_PID_Expect_2;
     short Chassis_Motor_PID_Expect_3;
     short Chassis_Motor_PID_Expect_4;
} Chassis_Motor_PID_Expect;

typedef __packed struct
{
     short Chassis_Motor_Actual_Speed_1;
     short Chassis_Motor_Actual_Speed_2;
     short Chassis_Motor_Actual_Speed_3;
     short Chassis_Motor_Actual_Speed_4;
} Chassis_Motor_Actual_Speed;

typedef __packed struct
{
     float PTZ_Motor_PID_Expect_1;
     short PTZ_Motor_PID_Expect_2;
} PTZ_Motor_PID_Expect;

typedef __packed struct
{
     short PTZ_Motor_Actual_Angle_1;
     short PTZ_Motor_Actual_Angle_2;
     short PTZ_Motor_Actual_Speed_1;
     short PTZ_Motor_Actual_Speed_2;
} PTZ_Motor_Actual;

extern float Pluck_motor_Actual_Rand;
extern float Pluck_motor_PID_Expect;
extern short Pluck_motor_Actual_Speed;

void PID_Expect(void);
void Chassis_Speed_Ref_Init(void);
void Get_PID_Expect(short speed);
void PTZ_PID_Expect(void);

extern Chassis_Motor_PID_Expect RM3510_PID_Expect;
extern Chassis_Motor_Actual_Speed RM3510_Actual_Speedt;
extern ChassisSpeed_Ref_t Chassis_Speed_Ref;
extern PTZ_Motor_PID_Expect GM3510_PID_Expect;
extern PTZ_Motor_Actual GM3510_Actual;
extern PTZAngle_Ref_t GM3510_Ref;
#endif
