#include "freertos_task.h"

/*初始任务*/
void StartTask(void)
{
	/*创建二值信号量*/
	IMU_SemaphoreHandle = xSemaphoreCreateBinary();
	Remote_Semaphore = xSemaphoreCreateBinary();

	/*创建队列*/
	referee_system_Queue = xQueueCreate(referee_system_QueueLength, referee_system_ItemSize);

	/*外设初始化*/
	CanFilter_Init(&hcan1);
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	
	HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart1, usart1_dma_buff, BUFLEN);
	
	HAL_TIM_Base_Start(&htim5);
	HAL_TIM_Base_Start(&htim7);
	
	
	
	PID_Init();
	Chassis_Speed_Ref_Init();
	/*模块初始化*/

	/*创建任务*/

	//云台归中进程
	xTaskCreate((TaskFunction_t)PTZ_Init_task,
				(const char *)"PTZ_Init_task",
				(uint16_t)256,
				(void *)NULL,
				(UBaseType_t)1,
				(TaskHandle_t *)&PTZ_Init_Handler);

//	//裁判系统进程
//	xTaskCreate((TaskFunction_t)referee_system_task,
//				(const char *)"referee_system_task",
//				(uint16_t)256,
//				(void *)NULL,
//				(UBaseType_t)1,
//				(TaskHandle_t *)&referee_system_Handler);
}

/*云台归中进程*/
TaskHandle_t PTZ_Init_Handler;
void PTZ_Init_task(void *pvParameters)
{
	portTickType xLastWakeTime;

	int16_t GM3510_buff[4] = {0};
	uint16_t time = 0;
	
	uint16_t LastAngle1 = 0;
	uint16_t LastAngle2 = 0;
	
	LastAngle1 = GM3510_Actual.PTZ_Motor_Actual_Angle_1;
	LastAngle2 = GM3510_Actual.PTZ_Motor_Actual_Angle_2;

	for (;;)
	{
		GM3510_Actual.PTZ_Motor_Actual_Speed_1 = GM3510_Actual.PTZ_Motor_Actual_Angle_1-LastAngle1;
		GM3510_Actual.PTZ_Motor_Actual_Speed_2 = GM3510_Actual.PTZ_Motor_Actual_Angle_2-LastAngle2;
		
		LastAngle1 = GM3510_Actual.PTZ_Motor_Actual_Angle_1;
		LastAngle2 = GM3510_Actual.PTZ_Motor_Actual_Angle_2;
		
		PID_Control(GM3510_Actual.PTZ_Motor_Actual_Angle_1, PTZ_X_median, &GM3510_1_AnglePID);
		limit(GM3510_1_AnglePID.pid_out, 600, -600);
		PID_Control(GM3510_Actual.PTZ_Motor_Actual_Speed_1, GM3510_1_AnglePID.pid_out, &GM3510_1_SpeedPID);
		limit(GM3510_1_SpeedPID.pid_out, 29000, -29000);

		PID_Control(GM3510_Actual.PTZ_Motor_Actual_Angle_2, PTZ_Y_median, &GM3510_2_AnglePID);
		limit(GM3510_2_AnglePID.pid_out, 600, -600);
		PID_Control(GM3510_Actual.PTZ_Motor_Actual_Speed_2, GM3510_2_AnglePID.pid_out, &GM3510_2_SpeedPID);
		limit(GM3510_2_SpeedPID.pid_out, 29000, -29000);

		GM3510_buff[0] = -GM3510_1_SpeedPID.pid_out;
		GM3510_buff[1] = -GM3510_2_SpeedPID.pid_out;

#if TEST == 0
		GM3510_CAN_Send(GM3510_buff);
#endif
		time++;
		if (time == 3000)
		{
			//6s后启动MPU进程
			xTaskCreate((TaskFunction_t)IMU_task,
						(const char *)"IMU_task",
						(uint16_t)512,
						(void *)NULL,
						(UBaseType_t)1,
						(TaskHandle_t *)&IMU_Handler);
		}
		vTaskDelayUntil(&xLastWakeTime, 2);
	}
}

/*IMU进程*/
uint16_t time =0;


xSemaphoreHandle IMU_SemaphoreHandle;
TaskHandle_t IMU_Handler;
float yaw;
void IMU_task(void *pvParameters)
{
	BaseType_t err = pdFALSE;

	//卡尔曼滤波器初始化
	kalman_filter_init(&yaw_kalman_filter, &yaw_kalman_filter_data);
	//MPU初始化
	mpu_device_init();
	
	//创建进程
	xTaskCreate((TaskFunction_t)Remote_task,
				(const char *)"Remote_task",
				(uint16_t)256,
				(void *)NULL,
				(UBaseType_t)1,
				(TaskHandle_t *)&Remote_task_Handler);
				
	//MPU初始化完毕，删除云台编码器归中进程
	vTaskDelete(PTZ_Init_Handler);
	//修改云台PID值
	PID_Init_MPU();
	
	xTaskCreate((TaskFunction_t)PID_task,
				(const char *)"PID_task",
				(uint16_t)256,
				(void *)NULL,
				(UBaseType_t)1,
				(TaskHandle_t *)&PID_task_Handler);
				
	xTaskCreate((TaskFunction_t)PID_task2,
				(const char *)"PID_task2",
				(uint16_t)256,
				(void *)NULL,
				(UBaseType_t)1,
				(TaskHandle_t *)&PID_task2_Handler);
	for (;;)
	{
		err = xSemaphoreTake(IMU_SemaphoreHandle, portMAX_DELAY);
		if (err == pdTRUE)
		{
			time++;
			if(time==100)time=0;
			mpu_get_data();
			imu_ahrs_update();
			kalman_filter_calc(&yaw_kalman_filter, imu.realyaw, imu.wz);
			imu.kalman_yaw = yaw_kalman_filter.filtered_value[0];
		}
		else if (err == pdFALSE)
		{
			vTaskDelay(5);
		}
	}
}

/*PID进程*/
TaskHandle_t PID_task_Handler;
void PID_task(void *pvParameters)
{
	portTickType xLastWakeTime;

	int16_t GM3510_buff[4] = {0,0,0,0};
	
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
	TIM12->CCR1 = 1000;
	TIM12->CCR2 = 1000;
	
	Chassis_Speed_Ref_Init();
	for (;;)
	{
		///拨弹PID////////////////////////////////////////////////////////////
		#if SpeedMode==0
		if(fabs(Pluck_motor_Actual_Rand - Pluck_motor_PID_Expect)>Pluck_motor_rand*2)
			Pluck_motor_PID_Expect = Pluck_motor_Actual_Rand - Pluck_motor_rand*2;
		PID_Control(Pluck_motor_Actual_Rand,
								Pluck_motor_PID_Expect,
								&Pluck_motor_AnglePID);
		limit(Pluck_motor_AnglePID.pid_out,39936,-39936);
		PID_Control(Pluck_motor_Actual_Speed,
								Pluck_motor_AnglePID.pid_out,
								&Pluck_motor_SpeedPID);
		limit(Pluck_motor_SpeedPID.pid_out,10000,-10000);
		#else
		PID_Control(Pluck_motor_Actual_Speed,
								Pluck_motor_PID_Expect,
								&Pluck_motor_SpeedPID);
		limit(Pluck_motor_AnglePID.pid_out,416,-416);
		#endif
		
		///云台PID////////////////////////////////////////////////////////////	
		
		PID_Control(imu.kalman_yaw, GM3510_Ref.x, &GM3510_1_AnglePID);
		limit(GM3510_1_AnglePID.pid_out, 600, -600);
		PID_Control(-imu.wz, GM3510_1_AnglePID.pid_out, &GM3510_1_SpeedPID);
		limit(GM3510_1_SpeedPID.pid_out, 5000, -5000);

		PID_Control(GM3510_Actual.PTZ_Motor_Actual_Angle_2, GM3510_Ref.y, &GM3510_2_AnglePID);
		limit(GM3510_2_AnglePID.pid_out, 600, -600);
		PID_Control(-imu.wx, GM3510_2_AnglePID.pid_out, &GM3510_2_SpeedPID);
		limit(GM3510_2_SpeedPID.pid_out, 5000, -5000);
		
		GM3510_buff[0] = GM3510_1_SpeedPID.pid_out;
		GM3510_buff[1] = -GM3510_2_SpeedPID.pid_out;
		GM3510_buff[2] = Pluck_motor_SpeedPID.pid_out;

#if TEST == 0
		taskENTER_CRITICAL();
		GM3510_CAN_Send(GM3510_buff);
		taskEXIT_CRITICAL();
#endif

		vTaskDelayUntil(&xLastWakeTime, 2);
	}
}

TaskHandle_t PID_task2_Handler;
void PID_task2(void *pvParameters)
{
	portTickType xLastWakeTime;
	
	int16_t RM3510_buff[4] = {0,0,0,0};
	Chassis_Speed_Ref_Init();
	for(;;)
	{
		///姿态PID////////////////////////////////////////////////////////////
		PID_Control(GM3510_Actual.PTZ_Motor_Actual_Angle_1,
								PTZ_X_median,
								&Chassis_PID);
																
		Chassis_Speed_Ref.rotate_ref=Chassis_PID.pid_out*-3;
		
		///底盘PID////////////////////////////////////////////////////////////
		PID_Expect();
		PID_Control(RM3510_Actual_Speedt.Chassis_Motor_Actual_Speed_1,
					RM3510_PID_Expect.Chassis_Motor_PID_Expect_1,
					&RM3510_1_PID);
		limit(RM3510_1_PID.pid_out, 23767, -23767);

		PID_Control(RM3510_Actual_Speedt.Chassis_Motor_Actual_Speed_2,
					RM3510_PID_Expect.Chassis_Motor_PID_Expect_2,
					&RM3510_2_PID);
		limit(RM3510_2_PID.pid_out, 23767, -23767);

		PID_Control(RM3510_Actual_Speedt.Chassis_Motor_Actual_Speed_3,
					RM3510_PID_Expect.Chassis_Motor_PID_Expect_3,
					&RM3510_3_PID);
		limit(RM3510_3_PID.pid_out, 23767, -23767);

		PID_Control(RM3510_Actual_Speedt.Chassis_Motor_Actual_Speed_4,
					RM3510_PID_Expect.Chassis_Motor_PID_Expect_4,
					&RM3510_4_PID);
		limit(RM3510_4_PID.pid_out, 23767, -23767);
		
		RM3510_buff[0] = -RM3510_1_PID.pid_out;
		RM3510_buff[1] = -RM3510_2_PID.pid_out;
		RM3510_buff[2] = -RM3510_3_PID.pid_out;
		RM3510_buff[3] = -RM3510_4_PID.pid_out;
		
#if TEST == 0
		taskENTER_CRITICAL();
		RM3510_CAN_Send(RM3510_buff);
		taskEXIT_CRITICAL();
#endif
		
		vTaskDelayUntil(&xLastWakeTime, 5);
	}
}

/*遥控器数据处理%USARTinterrupt*/
SemaphoreHandle_t Remote_Semaphore;
TaskHandle_t Remote_task_Handler;
void Remote_task(void *pvParameters)
{
	BaseType_t err = pdFALSE;

	for (;;)
	{
		err = xSemaphoreTake(Remote_Semaphore, portMAX_DELAY);
		if (err == pdTRUE)
		{
			Remote_Rx(usart1_dma_buff);
		}
		else if (err == pdFALSE)
		{
			vTaskDelay(10);
		}
	}
}

/*裁判系统进程*/
xQueueHandle referee_system_Queue;
TaskHandle_t referee_system_Handler;
void referee_system_task(void *pvParameters)
{
	uint8_t referee_system_buff[USART3_BUFLENx2]={0};
	uint8_t referee_system_halfbuff[USART3_BUFLEN]={0};
	uint8_t i;
	portBASE_TYPE err;
	UBaseType_t buffer_size;
	CRC_StatusTypeDef CRC_structure;
	referee_system_StatusTypeDef referee_system_Status;
	for(;;)
	{
		err=xQueueReceive(referee_system_Queue,referee_system_halfbuff,portMAX_DELAY);
		if(err==pdPASS)
		{
			buffer_size = uxQueueMessagesWaiting(referee_system_Queue);

			if (buffer_size < threshold)CRC_structure = CRC_ENABLE;
			else CRC_structure=CRC_DISABLE;
			
			for(i=USART3_BUFLEN;i<USART3_BUFLENx2;i++)
			{
				referee_system_buff[i-USART3_BUFLEN] = referee_system_buff[i];
				referee_system_buff[i] = referee_system_halfbuff[i-USART3_BUFLEN];
			}
			
			for(i=0;i<USART3_BUFLEN;i++)
			{
				if(referee_system_buff[i] == 0xA5)
				{
					referee_system_Status = referee_system_Rx(referee_system_buff+i, CRC_structure);
					if(referee_system_Status == OK)
						i += ((uint8_t)referee_system_buff[i+2]<<8|referee_system_buff[i+1])-1;
				}
			}
		}
		else
			vTaskDelay(10);
	}
}
