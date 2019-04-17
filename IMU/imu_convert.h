#ifndef _imu_convert_h
#define _imu_convert_h

typedef struct
{
	float NUM[3];
	float DEN[3];
	short x_now;
	short x_last;
	short x_next;
	short y_now;
	short y_last;
	short y_next;

} Diff_RAW;
typedef struct
{
	float x_now;
	float x_last;
	float x_next;
	float y_now;
	float y_last;
	float y_next;

} Diff_ANGLE;

typedef struct
{
	float x_now;
	float x_last1;
	float x_last2;
	float y_now;
	float y_last1;
	float y_last2;
} Diff_PID;

typedef struct

{
	float NUM[2];
	float DEN[2];
	float x_now;
	float x_last;
	float y_now;
	float y_last;
} Diff_PI;
extern Diff_RAW Diff_GX, Diff_GY, Diff_GZ, Diff_D;
extern Diff_ANGLE Diff_Yaw;
extern Diff_PI Diff_pi, Diff_pd, Diff_angle;
extern Diff_PID Diff_pid;
int diff_convert(int init_data, Diff_RAW *imu_type);
float diff_convert_yaw(float init_data, Diff_ANGLE *imu_type);
float diff_convert_PI(float current, float expect, Diff_PI *imu_type);
float diff_convert_PID(float current, float expect, Diff_PID *imu_type);
float diff_convert_PD(float current, float expect, Diff_PI *imu_type);
#endif
