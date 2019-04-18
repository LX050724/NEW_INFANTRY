#ifndef __remote_H
#define __remote_H
#include "freertos_task.h"

#define VAL_LIMIT(val, min, max) \
	if (val <= min)                \
	{                              \
		val = min;                   \
	}                              \
	else if (val >= max)           \
	{                              \
		val = max;                   \
	}

#define RC_FRAME_LENGTH 18u

#define REMOTE_CONTROLLER_STICK_OFFSET 1024u

#define STICK_TO_CHASSIS_SPEED_REF_FACT 12.0f
#define STICK_TO_ROTATE_SPEED_REF_FACT 12.0f
#define STICK_TO_PITCH_ANGLE_INC_FACT 0.0015f
#define STICK_TO_YAW_ANGLE_INC_FACT 0.0045f
#define FRICTION_WHEEL_MAX_DUTY 1600
	
#define Key_W 0x01
#define Key_A 0x04
#define Key_S 0x02
#define Key_D 0x08
#define Key_Q 0x40
#define Key_E 0x80
#define Key_Ctrl 0x20
#define Key_Shift 0x10

typedef __packed struct
{
	int16_t ch0;
	int16_t ch1;
	int16_t ch2;
	int16_t ch3;
	int8_t s1;
	int8_t s2;
} Remote;

typedef __packed struct
{
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t last_press_l;
	uint8_t last_press_r;
	uint8_t press_l;
	uint8_t press_r;
} Mouse;

//e q ctrl shift d a s w
typedef __packed struct
{
	uint16_t e;
	uint16_t q;
	uint16_t ctrl;
	uint16_t shift;
	uint16_t d;
	uint16_t a;
	uint16_t s;
	uint16_t w;
	uint16_t v;
} Key;

typedef __packed struct
{
	Remote rc;
	Mouse mouse;
	Key key;
} RC_Ctl_t; //控制数据结构体

//输入模式:遥控器/键盘鼠标/停止运行
typedef enum
{
	REMOTE_INPUT = 1,
	KEY_MOUSE_INPUT = 3,
	STOP = 2,
} InputMode_e;

void Remote_Rx(unsigned char *RxMsg);

extern uint8_t Twist;

extern RC_Ctl_t RC_CtrlData;

#endif
