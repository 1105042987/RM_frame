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
uint16_t counter = 0;
double rotate_speed = 0;

#ifdef USE_CHASSIS_FOLLOW
uint8_t ChassisTwistState = 0;
int ChassisTwistGapAngle = 0;
#endif

MusicNote SuperMario[] = {
	{H3, 100}, {0, 50}, 
	{H3, 250}, {0, 50}, 
	{H3, 100}, {0, 50}, 
	{0, 150},
	{H1, 100}, {0, 50},  
	{H3, 250}, {0, 50},
	{H5, 250}, {0, 50},
	{0, 300},
	{M5, 250}, {0, 50},
	{0, 300},
	{H1, 250}, {0, 50}
};

PID_Regulator_t CMRotatePID = CHASSIS_MOTOR_ROTATE_PID_DEFAULT; 
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
	
	//for(int i = 0; i < sizeof(SuperMario) / sizeof(MusicNote); i++){
	//		PLAY(SuperMario[i].note, SuperMario[i].time);
	//}

	return 0;
}

//状态机切换
void WorkStateFSM(void)
{
	switch (WorkState)
	{
		case PREPARE_STATE:				//准备模式
		{
			//if (inputmode == STOP) WorkState = STOP_STATE;
			if(prepare_time < 2000 && gyro_data.InitFinish == 1) prepare_time++;	
			if(prepare_time == 2000 && gyro_data.InitFinish == 1 && isCan11FirstRx == 1 && 
				isCan12FirstRx == 1 && isCan21FirstRx == 1 && isCan22FirstRx == 1)
			//开机2秒后且gyro初始化完成且所有can电机上电完成后进入正常模式
			{
				if(playMusicSuperMario())
				{
					CMRotatePID.Reset(&CMRotatePID);
					WorkState = NORMAL_STATE;
					prepare_time = 0;
				}
			}
		}break;
		case NORMAL_STATE:				//正常模式
		{
			if (inputmode == STOP) WorkState = STOP_STATE;
			if (inputmode == REMOTE_INPUT)
			{
				if(functionmode == MIDDLE_POS) WorkState = ADDITIONAL_STATE_ONE;
				if(functionmode == LOWER_POS) WorkState = ADDITIONAL_STATE_TWO;
			}
		}break;
		case ADDITIONAL_STATE_ONE:		//附加模式一
		{
			if (inputmode == STOP) WorkState = STOP_STATE;
			if (inputmode == REMOTE_INPUT)
			{
				if(functionmode == UPPER_POS) WorkState = NORMAL_STATE;
				if(functionmode == LOWER_POS) WorkState = ADDITIONAL_STATE_TWO;
			}
		}break;
		case ADDITIONAL_STATE_TWO:		//附加模式二
		{
			if (inputmode == STOP) WorkState = STOP_STATE;
			if (inputmode == REMOTE_INPUT)
			{
				if(functionmode == UPPER_POS) WorkState = NORMAL_STATE;
				if(functionmode == MIDDLE_POS) WorkState = ADDITIONAL_STATE_ONE;
			}
		}break;
		case STOP_STATE:				//紧急停止
		{
			for(int i=0;i<8;i++) {InitMotor(can1[i]);InitMotor(can2[i]);}
			setCAN11();setCAN12();setCAN21();setCAN22();
			if (inputmode == REMOTE_INPUT || inputmode == KEY_MOUSE_INPUT)
			{
				WorkState = PREPARE_STATE;
				FunctionTaskInit();
			}
		}break;
	}
}
#ifdef USE_CHASSIS_FOLLOW
extern MotorINFO* GimbalMotorGroup[2];
void ChassisTwist(void)
{
	switch (ChassisTwistGapAngle)
	{
		case 0:
		{
			ChassisTwistGapAngle = CHASSIS_TWIST_ANGLE_LIMIT;
		}break;
		case CHASSIS_TWIST_ANGLE_LIMIT:
		{
			//if(abs((GMY.RxMsg6623.angle - GM_YAW_ZERO) * 360 / 8192.0f - ChassisTwistGapAngle)<3)
			if(abs((GimbalMotorGroup[1]->RxMsg6623.angle - GimbalMotorGroup[1]->RxMsgC6x0.angle) * 360 / 8192.0f - ChassisTwistGapAngle)<3)
				
			{ChassisTwistGapAngle = -CHASSIS_TWIST_ANGLE_LIMIT;}break;
		}
		case -CHASSIS_TWIST_ANGLE_LIMIT:
		{
			//if(abs((GMY.RxMsg6623.angle - GM_YAW_ZERO) * 360 / 8192.0f - ChassisTwistGapAngle)<3)
			if(abs((GimbalMotorGroup[1]->RxMsg6623.angle - GimbalMotorGroup[1]->RxMsgC6x0.angle) * 360 / 8192.0f - ChassisTwistGapAngle)<3)
			{ChassisTwistGapAngle = CHASSIS_TWIST_ANGLE_LIMIT;}break;
		}
	}
}

void ChassisDeTwist(void)
{
	ChassisTwistGapAngle = 0;
}
#endif
void ControlRotate(void)
{	
	#ifdef USE_CHASSIS_FOLLOW
		//ChassisSpeedRef.rotate_ref=(GMY.RxMsg6623.angle - `GM_YAW_ZERO) * 360 / 8192.0f - ChassisTwistGapAngle;
		ChassisSpeedRef.rotate_ref=(GimbalMotorGroup[1]->RxMsg6623.angle - GimbalMotorGroup[1]->RxMsgC6x0.angle) * 360 / 8192.0f - ChassisTwistGapAngle;
		NORMALIZE_ANGLE180(ChassisSpeedRef.rotate_ref);
	#endif
	CMRotatePID.ref = 0;
	CMRotatePID.fdb = ChassisSpeedRef.rotate_ref;
	CMRotatePID.Calc(&CMRotatePID);
	#ifdef USE_CHASSIS_FOLLOW
		if(ChassisTwistState) MINMAX(CMRotatePID.output,-10,10);
	#endif
	rotate_speed = CMRotatePID.output * 13 + ChassisSpeedRef.forward_back_ref * 0.01 + ChassisSpeedRef.left_right_ref * 0.01;
}
extern MotorINFO* ChassisMotorGroup[4];
void Chassis_Data_Decoding()
{
	ControlRotate();
	
	#ifdef USE_CHASSIS_FOLLOW
		float gap = (GimbalMotorGroup[1]->RxMsgC6x0.angle-GimbalMotorGroup[1]->RxMsg6623.angle) * 6.28 / 8192.0f;
		int16_t fb = ChassisSpeedRef.forward_back_ref;
		int16_t rl = ChassisSpeedRef.left_right_ref;
		ChassisSpeedRef.forward_back_ref = cos(gap)*fb-sin(gap)*rl;
		ChassisSpeedRef.left_right_ref = sin(gap)*fb+cos(gap)*rl;
	#endif
	
	if(ChassisMotorGroup[3]!=0){
		ChassisMotorGroup[0]->Target = ( ChassisSpeedRef.forward_back_ref + ChassisSpeedRef.left_right_ref + rotate_speed)*12;
		ChassisMotorGroup[1]->Target = (-ChassisSpeedRef.forward_back_ref + ChassisSpeedRef.left_right_ref + rotate_speed)*12;
		ChassisMotorGroup[2]->Target = ( ChassisSpeedRef.forward_back_ref - ChassisSpeedRef.left_right_ref + rotate_speed)*12;
		ChassisMotorGroup[3]->Target = (-ChassisSpeedRef.forward_back_ref - ChassisSpeedRef.left_right_ref + rotate_speed)*12;
	}
	else{
		ChassisMotorGroup[0]->Target += ChassisSpeedRef.forward_back_ref * 0.008;
		ChassisMotorGroup[1]->Target = -ChassisMotorGroup[0]->Target;
	}
	
	//CMFL.Target = ( ChassisSpeedRef.forward_back_ref + ChassisSpeedRef.left_right_ref + rotate_speed)*12;
	//CMFR.Target = (-ChassisSpeedRef.forward_back_ref + ChassisSpeedRef.left_right_ref + rotate_speed)*12;
	//CMBL.Target = ( ChassisSpeedRef.forward_back_ref - ChassisSpeedRef.left_right_ref + rotate_speed)*12;
	//CMBR.Target = (-ChassisSpeedRef.forward_back_ref - ChassisSpeedRef.left_right_ref + rotate_speed)*12;
}
//主控制循环
#ifdef USE_POWER_LIMIT
float rate=1.0f;
#endif
void controlLoop()
{
	getJudgeState();
	WorkStateFSM();
	
	if(WorkState > 0)// && WorkState != STOP_STATE)
	{
		Chassis_Data_Decoding();
		
		for(int i=0;i<8;i++) if(can1[i]!=0) (can1[i]->Handle)(can1[i]);
		for(int i=0;i<8;i++) if(can2[i]!=0) (can2[i]->Handle)(can2[i]);
		#ifdef USE_SUPER_CAP
			Cap_Run();
		#endif
		#ifdef USE_POWER_LIMIT
			//rate=PowerLimitation();
			for(int i=0;i<4;i++) if(ChassisMotorGroup[i]!=0)ChassisMotorGroup[i]->Intensity*=rate;
		#endif
		
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
			setCANMessage(0);
		#else
		#ifdef CAN13
			setCANMessage(0);
		#endif
		#endif
	}
}

void heatCalc()//2ms
{
	if(syncCnt0 > 35 && JUDGE_State == ONLINE){fakeHeat0 = realHeat0;}
	else if(fakeHeat0 >= cooldown0/500)fakeHeat0 -= cooldown0/500;
	else fakeHeat0 = 0;
}

//时间中断入口函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TWO_MS_TIM.Instance)//2ms时钟
	{
		//HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
		#ifdef USE_IMU
			//imu解算
			mpu_get_data();
			imu_ahrs_update();
			imu_attitude_update();
			if(gyro_data.FirstEnter == 1) gyro_data.InitCount++;
			if(gyro_data.InitCount == 1000) {gyro_data.InitFinish = 1;gyro_data.FirstEnter = 0;gyro_data.InitCount = 0;}
		#endif
		//主循环在时间中断中启动
		controlLoop();
		heatCalc();
		
		//HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
	}
	else if (htim->Instance == ONE_MS_TIM.Instance)//ims时钟
	{
		
		rc_cnt++;
		if(auto_counter > 0) auto_counter--;
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
		
	}
	else if (htim->Instance == htim10.Instance)  //10ms
	{
		
	}
}
