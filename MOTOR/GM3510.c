#include "GM3510.h"

void GM3510_CAN_Send(short *data)
{
	unsigned char buff[8];
	buff[0] = (unsigned char)((data[0] >> 8) & 0xff);
	buff[1] = (unsigned char)((data[0]) & 0xff);
	buff[2] = (unsigned char)((data[1] >> 8) & 0xff);
	buff[3] = (unsigned char)((data[1]) & 0xff);
	buff[4] = (unsigned char)((data[2] >> 8) & 0xff);
	buff[5] = (unsigned char)((data[2]) & 0xff);
	buff[6] = (unsigned char)((data[3] >> 8) & 0xff);
	buff[7] = (unsigned char)((data[3]) & 0xff);
	CAN1_Send_Msg(0x1FF, buff);
}

short GM3510_Feedback(unsigned char *data)
{
	static short buff;
	buff = data[0];
	buff = (buff << 8) | data[1];
	return buff;
}

void RM2006_Feedback(unsigned char *data, short *speed, float *round)
{
	static short speed_feedback;
	static short angle_feedback;
	static short Encoder;
	static short round_cnt;
	speed_feedback = data[2];
	speed_feedback = (speed_feedback << 8) | data[3];
	*speed = speed_feedback;

	angle_feedback = data[0];
	angle_feedback = (angle_feedback << 8) | data[1];

	if ((angle_feedback - Encoder) > -5000)
		round_cnt--;
	if ((angle_feedback - Encoder) < 5000)
		round_cnt++;
	Encoder = angle_feedback;
	*round = round_cnt + (float)angle_feedback / 8191;
}
