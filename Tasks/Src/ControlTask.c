/**
  ******************************************************************************
  * File Name          : ControlTask.c
  * Description        : 主控制任务
  ******************************************************************************
  *
  * Copyright (c) 2019 Team JiaoLong-ShanghaiJiaoTong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#include "includes.h"

WorkState_e WorkState = PREPARE_STATE;
uint16_t prepare_time = 0;
uint16_t normal_time = 0;
uint16_t counter = 0;
double rotate_speed = 0;
uint8_t startUp = 0;

MusicNote SuperMario[] = {
	#if GUARD == 'U'
		{H3, 100}, {0, 50},
		{H3, 100}, {0, 50}
	#else
		{H3, 100}, {0, 50}, 
		{H1, 250}, {0, 50}
	#endif
};

extern int32_t auto_counter;

uint8_t playMusicSuperMario(void){
	static int16_t cnt=0;
	if(auto_counter <= 0){
		if(cnt==0) HAL_TIM_PWM_Start(&BUZZER_TIM, TIM_CHANNEL_1);
		if(cnt < sizeof(SuperMario) / sizeof(MusicNote))
		{	
			uint16_t note = SuperMario[cnt].note;
			uint16_t time = SuperMario[cnt].time;
			if(note == 0){
				__HAL_TIM_SET_AUTORELOAD(&BUZZER_TIM, 0);
			}
			else{ 
				__HAL_TIM_SET_AUTORELOAD(&BUZZER_TIM, 1000000 / note); 
				__HAL_TIM_SET_COMPARE(&BUZZER_TIM, TIM_CHANNEL_1, 500000 / note); 
			}
			auto_counter=time;
			cnt++;
		}
		else{
			HAL_TIM_PWM_Stop(&BUZZER_TIM, TIM_CHANNEL_1);
			cnt=0;
			return 1;
		}
	}
	return 0;
}
void sendAllData(uint8_t isStop)
{
	if(isStop) for(int i=0;i<8;i++) {InitMotor(can1[i]);InitMotor(can2[i]);}
	#ifdef CAN11
		setCAN11();
	#endif
	#ifdef CAN12
		setCAN12();
	#endif
	#ifdef CAN21
		setCAN21();
	#endif
	#ifdef CAN22
		setCAN22();
	#endif
	#ifdef CAN23
			static int MSG_cnt=0;
			if(isStop){
				sendData[0].data[0]=-1;
				setCANMessage(0);
				MSG_cnt=0;
			}
			else{
				setCANMessage(MSG_cnt);
				MSG_cnt++;
				MSG_cnt%=CAN23;
			}
		#else
		#ifdef CAN13
			static int MSG_cnt=0;
			if(isStop){
				sendData[0].data[0]=-1;
				setCANMessage(0);
				MSG_cnt=0;
			}
			else{
				setCANMessage(MSG_cnt);
				MSG_cnt++;
				MSG_cnt%=CAN13;
			}
		#endif
		#endif
}

//状态机切换
void WorkStateFSM(void){
	switch (WorkState){
		case PREPARE_STATE:{			//准备模式
			normal_time = 0;
			if(prepare_time < 0xff && imu.InitFinish == 1) prepare_time++;	
			if(prepare_time == 0xff && imu.InitFinish == 1 && isCan11FirstRx == 1 && 
				isCan12FirstRx == 1 && isCan21FirstRx == 1 && isCan22FirstRx == 1){
			//开机2秒后且gyro初始化完成且所有can电机上电完成后进入正常模式
				if(playMusicSuperMario()){
					if(inputmode == STOP) WorkState = STOP_STATE;
					else WorkState = NORMAL_STATE;
					prepare_time = 0;
				}
			}
		}break;
		#ifndef SLAVE_MODE
		case NORMAL_STATE:				//正常模式
		{
			if(normal_time<10000)normal_time++;
			if(normal_time>=10)startUp = 1;
			if(inputmode == STOP) WorkState = STOP_STATE;
			if(functionmode == MIDDLE_POS) WorkState = ADDITIONAL_STATE_ONE;
			if(functionmode == LOWER_POS) WorkState = ADDITIONAL_STATE_TWO;
		}break;
		case ADDITIONAL_STATE_ONE:		//附加模式一
		{
			if(inputmode == STOP) WorkState = STOP_STATE;
			if(functionmode == UPPER_POS) WorkState = NORMAL_STATE;
			if(functionmode == LOWER_POS) WorkState = ADDITIONAL_STATE_TWO;
		}break;
		case ADDITIONAL_STATE_TWO:		//附加模式二
		{
			if(inputmode == STOP) WorkState = STOP_STATE;
			if(functionmode == UPPER_POS) WorkState = NORMAL_STATE;
			if(functionmode == MIDDLE_POS) WorkState = ADDITIONAL_STATE_ONE;
		}break;
		case STOP_STATE:{				//紧急停止
			sendAllData(1);
			if(inputmode == REMOTE_Control || inputmode == SELF_Control){
				WorkState = PREPARE_STATE;
				FunctionTaskInit();
			}
		}break;
		#else
		case NORMAL_STATE:
		case ADDITIONAL_STATE_ONE:
		case ADDITIONAL_STATE_TWO:break;
		case STOP_STATE:{
			sendAllData(1);
		}break;
		#endif
	}
}

void Chassis_Data_Decoding(){
	CML.Target += ChassisSpeedRef.left_right_ref * 0.008;
	CMR.Target = CML.Target;
}
//主控制循环
#ifdef USE_POWER_LIMIT
float rate=1.0f;
#endif
void controlLoop()
{
	getJudgeState();
	WorkStateFSM();
	
	if(WorkState > 0){// && WorkState != STOP_STATE)
		Chassis_Data_Decoding();
		
		for(int i=0;i<8;i++) if(can1[i]!=0) (can1[i]->Handle)(can1[i]);
		for(int i=0;i<8;i++) if(can2[i]!=0) (can2[i]->Handle)(can2[i]);

		#ifdef USE_POWER_LIMIT
			PowerLimitation();
		//for(int i=0;i<4;i++) if(ChassisMotorGroup[i]!=0)ChassisMotorGroup[i]->Intensity*=rate;
		#endif
		
		sendAllData(0);
	}
}
//哨兵：每秒冷却160，2ms为160/500=0.32
void heatCalc(){//2ms
	fakeHeat0-=0.32;
	if(fakeHeat0<0){fakeHeat0=0;}
}
//void heatCalc(){//2ms
//	if(syncCnt0 > 35 && JUDGE_State == ONLINE){fakeHeat0 = realHeat0;}
//	else if(fakeHeat0 >= cooldown0/500)fakeHeat0 -= cooldown0/500;
//	else fakeHeat0 = 0;
//}
//时间中断入口函数
extern int8_t Control_Update;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim->Instance == TWO_MS_TIM.Instance){//2ms时钟
		//HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
		#ifdef USE_IMU
			//imu解算
			mpu_get_data();
			imu_ahrs_update();
			imu_attitude_update();
			if(imu.FirstEnter == 1) imu.InitCount++;
			if(imu.InitCount == 1000) {imu.InitFinish = 1;imu.FirstEnter = 0;imu.InitCount = 0;}
		#endif
		//主循环在时间中断中启动
		controlLoop();
		heatCalc();
		
		#ifdef USE_AUTOAIM
		//自瞄数据解算（6ms）
		static uint8_t aim_cnt=0;
		if(++aim_cnt==3){
			EnemyINFOProcess();
			aim_cnt=0;
		}
		#endif
		
		//HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
	}
	else if (htim->Instance == ONE_MS_TIM.Instance)//ims时钟
	{
		rc_cnt++;
		if(auto_counter > 0) auto_counter--;
		#ifdef SLAVE_MODE
		if(Control_Update==1)
		{
			HAL_IWDG_Refresh(&hiwdg);
			Control_Update=0;
			if(WorkState!=PREPARE_STATE)
			{
				if(WorkState==STOP_STATE&&receiveData[0].data[0]>0) WorkState = PREPARE_STATE;
				else WorkState = (WorkState_e)receiveData[0].data[0];
				RemoteControlProcess();
			}
		}
		#else
		if (rx_free == 1 && tx_free == 1)
		{
			if( (rc_cnt <= 17) && (rc_first_frame == 1))
			{
				RemoteDataProcess(rc_data);				//遥控器数据解算
				HAL_UART_AbortReceive(&RC_UART);
				rx_free = 0;
				while(HAL_UART_Receive_DMA(&RC_UART, rc_data, 18)!= HAL_OK);
				if (counter == 10) 
				{
					tx_free = 0;
					Send_User_Data(); 
					counter = 0;
				}
				else counter++;				
				rc_cnt = 0;
			}
			else
			{
				if(rc_first_frame == 0) 
				{
					WorkState = PREPARE_STATE;
					HAL_UART_AbortReceive(&RC_UART);
					while(HAL_UART_Receive_DMA(&RC_UART, rc_data, 18)!= HAL_OK);
					rc_cnt = 0;
					rc_first_frame = 1;
				}
			}
			rc_update = 0;
		}
		#endif
		
	}
	else if (htim->Instance == htim10.Instance)  //10ms
	{
		
	}
}
