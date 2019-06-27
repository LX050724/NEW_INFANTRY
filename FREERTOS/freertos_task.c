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
	
	__HAL_UART_ENABLE(&huart6);
	HAL_UART_Receive_DMA(&huart6, usart6_dma_buff,8);
	
	HAL_TIM_Base_Start(&htim5);
	HAL_TIM_Base_Start(&htim7);
	
	
	
	PID_Init();
	Chassis_Speed_Ref_Init();
	/*模块初始化*/
	
	
//	uint8_t msg[8] = {0x13,0xff,0x13,0xff,0x13,0xff,0x13,0xff};
//	CAN_Tx.StdId = 0x200;      //标准标识符
//  CAN_Tx.IDE = CAN_ID_STD;   //使用标准帧
//  CAN_Tx.RTR = CAN_RTR_DATA; //数据帧
//  CAN_Tx.DLC = 8;
//	HAL_CAN_AddTxMessage(&hcan1, &CAN_Tx, msg, (uint32_t *)CAN_TX_MAILBOX1);
		


	/*创建任务*/
#if CHASSIS_FEEDBACK != 0
	//电流计ADC进程
	xTaskCreate((TaskFunction_t)ADC_Task,
			(const char *)"ADC_Task",
			(uint16_t)128,
			(void *)NULL,
			(UBaseType_t)0,
			(TaskHandle_t *)&ADC_Task_Handler);
#endif
	//云台归中进程
	xTaskCreate((TaskFunction_t)PTZ_Init_task,
				(const char *)"PTZ_Init_task",
				(uint16_t)256,
				(void *)NULL,
				(UBaseType_t)1,
				(TaskHandle_t *)&PTZ_Init_Handler);

	//裁判系统进程
	xTaskCreate((TaskFunction_t)referee_system_task,
				(const char *)"referee_system_task",
				(uint16_t)256,
				(void *)NULL,
				(UBaseType_t)1,
				(TaskHandle_t *)&referee_system_Handler);
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
		GM3510_buff[1] = 0;

#if TEST == 0
		GM3510_CAN_Send(GM3510_buff);
#endif
		time++;
		if (time == 2000)
		{
			//4s后启动MPU进程
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
				(UBaseType_t)3,
				(TaskHandle_t *)&PID_task_Handler);
				
	xTaskCreate((TaskFunction_t)PID_task2,
				(const char *)"PID_task2",
				(uint16_t)256,
				(void *)NULL,
				(UBaseType_t)2,
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
		limit(GM3510_1_SpeedPID.pid_out, 29000, -29000);


		//GM3510_Actual.PTZ_Motor_Actual_Angle_2 = PID_diff_convert(GM3510_Actual.PTZ_Motor_Actual_Angle_2,&Diff_P);
		PID_Control(GM3510_Actual.PTZ_Motor_Actual_Angle_2, GM3510_Ref.y, &GM3510_2_AnglePID);
		limit(GM3510_2_AnglePID.pid_out, 600, -600);
		
		PID_Control(-imu.wx, GM3510_2_AnglePID.pid_out, &GM3510_2_SpeedPID);
		limit(GM3510_2_SpeedPID.pid_out, 5000, -5000);
		
		GM3510_buff[0] = GM3510_1_SpeedPID.pid_out;
		GM3510_buff[1] = -GM3510_2_SpeedPID.pid_out;
		GM3510_buff[2] = Pluck_motor_SpeedPID.pid_out;
		GM3510_buff[1] = 0;
#if TEST == 0
		GM3510_CAN_Send(GM3510_buff);
#endif

		vTaskDelayUntil(&xLastWakeTime, 2);
	}
}


uint8_t SC_flag;
TaskHandle_t PID_task2_Handler;
void PID_task2(void *pvParameters)
{
	portTickType xLastWakeTime;
	
	int16_t RM3510_buff[4] = {0,0,0,0};
	Chassis_Speed_Ref_Init();
	
#if CHASSIS_FEEDBACK != 0
	float Current_Motor14 = 0;
	float Current_Motor23 = 0;
	float Current_Motor1234 = 0;
	int expect_sum = 0;
	float bili = 0;
	float current_expect=0;
	float power_expect=0;
	float Last_expect=0;
	float power_current=0;
#endif
	
	float Twistvalue = 0;
	float Twistadd = 0;
	
	for(;;)
	{
		///姿态PID////////////////////////////////////////////////////////////
		
		//扭腰
		if((Twist == 1)||(fabsf(Twistvalue) > 50.0f))
		{
			Twistvalue = sin(Twistadd) * 500;
			Twistadd += 0.03f;
		}
		
		PID_Control(GM3510_Actual.PTZ_Motor_Actual_Angle_1,
								PTZ_X_median + Twistvalue,
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
		
		
#if CHASSIS_FEEDBACK==1||CHASSIS_FEEDBACK==2   //用裁判系统或者电流计做底盘功率闭环			
				Current_Motor14=__fabs(RM3510_1_PID.pid_out) + __fabs(RM3510_2_PID.pid_out);
				Current_Motor23=__fabs(RM3510_3_PID.pid_out) + __fabs(RM3510_4_PID.pid_out);
				Current_Motor1234=Current_Motor14+Current_Motor23;
				if( Current_Motor1234<= 20)
					expect_sum = 0;
				else
					expect_sum = Current_Motor1234; //计算四个电机的电流总和
//				if(__fabs(RM3510_PID_Expect.Chassis_Motor_PID_Expect_1)>7000||__fabs(RM3510_PID_Expect.Chassis_Motor_PID_Expect_2)>7000
//				    ||__fabs(RM3510_PID_Expect.Chassis_Motor_PID_Expect_3)>7000||__fabs(RM3510_PID_Expect.Chassis_Motor_PID_Expect_4)>7000)
//				{
//					power_start=50;
//				}
//				else
//				{
//					power_start=70;
//				}
//				if(Twist == 1) //扭腰
//				{
//					power_start=40;
//				}

#if CHASSIS_FEEDBACK==1						  	       //用裁判系统做闭环	
				if(ext_power_heat_data.chassis_power >= power_start)//当功率大于70W的时候开始进行功率限制
				{
					power_expect=PID_Increment(PowerHeatData.chassisPower,POWER_LIMIT_VALUE,&Power_limit);//计算功率的期望值
					if(power_expect>POWER_LIMIT_UP)                                           //功率限幅
					{
						power_expect=POWER_LIMIT_UP;
					}
					current_expect=power_expect/PowerHeatData.chassisVolt;//计算电流期望值，单位为A


#elif	CHASSIS_FEEDBACK==2							        //用电流计做闭环
				power_current=(ADC_Current/1000) * POWER_VOLTAGE;           //计算功率的期望值
				if(power_current>60)          //当功率大于70W的时候开始进行功率限制
				{
					power_expect = Last_expect + PID_Increment(power_current,POWER_LIMIT_VALUE,&Power_limit);	//计算功率的期望值
					if(power_expect<0)power_expect=20; 
					limit(power_expect,POWER_LIMIT_UP,0);
					current_expect=power_expect/POWER_VOLTAGE;             //计算电流期望值，单位为A		
					Last_expect=power_expect;

#endif
					if(power_expect<0.01f)bili=0;
					else bili = (current_expect * CURRENT_OFFSET / (expect_sum+50));
					
					if(SC_flag == 0)
					{
						RM3510_1_PID.pid_out *= bili; //按比例分配给底盘电机
						RM3510_2_PID.pid_out *= bili;
						RM3510_3_PID.pid_out *= bili;
						RM3510_4_PID.pid_out *= bili;
					}
				}

#endif
		RM3510_buff[0] = (int16_t)-RM3510_1_PID.pid_out;
		RM3510_buff[1] = (int16_t)-RM3510_2_PID.pid_out;
		RM3510_buff[2] = (int16_t)-RM3510_3_PID.pid_out;
		RM3510_buff[3] = (int16_t)-RM3510_4_PID.pid_out;
		
#if TEST == 0
		RM3510_CAN_Send(RM3510_buff);
#endif
		
		vTaskDelayUntil(&xLastWakeTime, 2);
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

/*ADC进程*/
TaskHandle_t ADC_Task_Handler;
float ADC_Current = 0;
float ADC_Aoltage = 0;
void ADC_Task(void *pvParameters)
{
	portTickType xLastWakeTime;
	
	float ADC_Value = 0;
	uint16_t ADC_AverageBuff[10] = {0,0,0,0,0,0,0,0,0,0};
	
	for(uint8_t i = 0;i<10;i++)
		Average(ADC_AverageBuff,Get_Adc(ADC_CHANNEL_11));
	
	for(;;)
	{
		//ADC_Value = Average(ADC_AverageBuff,Get_Adc(ADC_CHANNEL_11));
		ADC_Value = Get_Adc(ADC_CHANNEL_11);
		ADC_Aoltage = ADC_Value * 0.8057f;
		
		ADC_Current = fabsf(ADC_Aoltage - 2540)*10;
		
		vTaskDelayUntil(&xLastWakeTime,1);
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
			vTaskDelay(5);
	}
}
