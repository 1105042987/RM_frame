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

WorkState_e WorkState = STATE_pre;
uint16_t prepare_time = 0;
uint16_t normal_time = 0;
uint16_t counter = 0;
double rotate_speed = 0;
uint8_t startUp = 0;

MusicNote SuperMario[]={
	#if GUARD == 'U'
		{H3, 100}, {0, 50},
		{H3, 100}, {0, 50}
	#else
		{H1, 100}, {0, 50}, 
		{H1, 100}, {0, 50}, 
		{H3, 200}, {0, 50}
	#endif
};

extern int32_t auto_counter;

uint8_t playMusicSuperMario(void){
	static int16_t cnt=0;
	if(auto_counter <= 0){
		if(cnt==0) HAL_TIM_PWM_Start(&BUZZER_TIM, TIM_CHANNEL_1);
		if(cnt < sizeof(SuperMario) / sizeof(MusicNote)){	
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
void sendAllData(uint8_t isStop){
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
		case STATE_pre:{			//准备模式
			normal_time = 0;
			if(prepare_time < 0x1ff && imu.InitFinish == 1) prepare_time++;
			HAL_GPIO_WritePin(GPIOG, 0xff, GPIO_PIN_SET);//close all LED
			if(isCan11FirstRx){onLed(1);}
			if(isCan12FirstRx){onLed(2);}
			if(isCan21FirstRx){onLed(3);}
			if(isCan22FirstRx){onLed(4);}
			
			if(prepare_time == 0x1ff && imu.InitFinish == 1 && isCan11FirstRx == 1 && 
				isCan12FirstRx == 1 && isCan21FirstRx == 1 && isCan22FirstRx == 1){
			//开机2秒后且gyro初始化完成且所有can电机上电完成后进入正常模式
				if(playMusicSuperMario()){
					if(RCRightMode == Pos3 && RCLeftMode == Pos3) WorkState = STATE_stop;
					else WorkState = STATE_1;
					prepare_time = 0;
				}
			}
		}break;
		case STATE_1:{
			if(normal_time<10000)normal_time++;
			if(normal_time>=10)startUp = 1;
			//if(RCRightMode == Pos3 && RCLeftMode == Pos3) WorkState = STATE_stop;
			if(RCLeftMode == Pos2) WorkState = STATE_2;
			if(RCLeftMode == Pos3) WorkState = STATE_3;
		}break;
		case STATE_2:{
			//if(RCRightMode == Pos3 && RCLeftMode == Pos3) WorkState = STATE_stop;
			if(RCLeftMode == Pos1) WorkState = STATE_1;
			if(RCLeftMode == Pos3) WorkState = STATE_3;
		}break;
		case STATE_3:{
			//if(RCRightMode == Pos3 && RCLeftMode == Pos3) WorkState = STATE_stop;
			if(RCLeftMode == Pos1) WorkState = STATE_1;
			if(RCLeftMode == Pos2) WorkState = STATE_2;
		}break;
		case STATE_stop:{
			sendAllData(1);
			if(RCRightMode == Pos1 || RCRightMode == Pos2){
				WorkState = STATE_pre;
				FunctionTaskInit();
			}
		}break;
	}
}


//主控制循环
void controlLoop(){
	getJudgeState();
	WorkStateFSM();
	if(WorkState > 0){// && WorkState != STATE_stop)
		CML.Target = ChassisSpeed*3.2;//*60*19/360;
		CMR.Target = CML.Target;
		#ifdef USE_POWER_LIMIT
			PowerLimitation();
		#endif
		for(int i=0;i<8;i++) if(can1[i]!=0) (can1[i]->Handle)(can1[i]);
		for(int i=0;i<8;i++) if(can2[i]!=0) (can2[i]->Handle)(can2[i]);
		sendAllData(0);
	}
}
//哨兵：每秒冷却160，2ms为160/500=0.32
void heatCalc(){//2ms
	RealHeat0-=0.32;
	if(RealHeat0<0){RealHeat0=0;}
}
//时间中断入口函数
extern int8_t Control_Update;
extern int16_t receiveCnt,receiveFps,AimTic;
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
		#ifdef SLAVE_MODE
			if(RCRightMode==Pos1){RCProcess1();}
			else if(RCRightMode==Pos2){RCProcess2();}
			else{RCProcess3();}
		#endif
		//主循环在时间中断中启动
		controlLoop();
		heatCalc();
	}
	else if (htim->Instance == ONE_MS_TIM.Instance){//1ms时钟
		rc_cnt++;
		if(auto_counter > 0) auto_counter--;
		#ifdef SLAVE_MODE
		if(AimTic<1000){AimTic++;}
		if(Control_Update==1){
			HAL_IWDG_Refresh(&hiwdg);
			Control_Update=0;
			if(WorkState!=STATE_pre){
				if(WorkState==STATE_stop&&receiveData[0].data[0]>0) WorkState = STATE_pre;
				else{
					RCLeftMode = (RCMode_e)((receiveData[0].data[0]&0xf)>>2);
					RCRightMode = (RCMode_e)(receiveData[0].data[0]&0x3);
				}
			}
		}
		#else
		if (rx_free == 1 && tx_free == 1){
			if( (rc_cnt <= 17) && (rc_first_frame == 1)){
				RemoteDataProcess(rc_data);				//遥控器数据解算
				HAL_UART_AbortReceive(&RC_UART);
				rx_free = 0;
				while(HAL_UART_Receive_DMA(&RC_UART, rc_data, 18)!= HAL_OK);
				if (counter == 10){
					tx_free = 0;
					Send_User_Data(); 
					counter = 0;
				}
				else counter++;				
				rc_cnt = 0;
			}
			else{
				if(rc_first_frame == 0){
					WorkState = STATE_pre;
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
	else if (htim->Instance == htim10.Instance){  //10ms
		#if GUARD == 'D'
		#endif //GUARD == 'D'
	}
}
