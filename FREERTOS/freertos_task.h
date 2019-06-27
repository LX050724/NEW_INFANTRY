#ifndef _FREERTOS_TASK_H_
#define _FREERTOS_TASK_H_

#include "cmsis_os.h"
#include "stm32f4xx_hal.h"

#include "spi.h"
#include "tim.h"
#include "can.h"
#include "usart.h"
#include "test_can.h"
#include "arm_math.h"
#include "bsp_imu.h"
#include "imu_convert.h"
#include "filter.h"
#include "kalman.h"
#include "GM3510.h"
#include "RM3510.h"
#include "PID.h"
#include "Chassis.h"
#include "ramp.h"
#include "remote.h"
#include "referee_system.h"
#include "adc.h"

void StartTask(void);

extern TaskHandle_t PTZ_Init_Handler;
void PTZ_Init_task(void *pvParameters);

extern xSemaphoreHandle IMU_SemaphoreHandle;
extern TaskHandle_t IMU_Handler;
void IMU_task(void *pvParameters);

extern TaskHandle_t PID_task_Handler;
void PID_task(void *pvParameters);

extern uint8_t SC_flag;
extern TaskHandle_t PID_task2_Handler;
void PID_task2(void *pvParameters);

extern SemaphoreHandle_t Remote_Semaphore;
extern TaskHandle_t Remote_task_Handler;
void Remote_task(void *pvParameters);

extern TaskHandle_t ADC_Task_Handler;
extern float ADC_Current;
void ADC_Task(void *pvParameters);

extern xQueueHandle referee_system_Queue;
extern TaskHandle_t referee_system_Handler;
void referee_system_task(void *pvParameters);

#endif
