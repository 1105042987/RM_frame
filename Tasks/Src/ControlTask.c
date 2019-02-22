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
MusicNote SuperMario[] = {
	{H1, 100}, {0, 50}, 
	{H3, 250}, {0, 50}
};

PID_Regulator_t CMRotatePID = CHASSIS_MOTOR_ROTATE_PID_DEFAULT; 
extern int32_t auto_counter,auto_counter_stir;

void playMusicSuperMario(void){
	HAL_TIM_PWM_Start(&BUZZER_TIM, TIM_CHANNEL_1);
	for(int i = 0; i < sizeof(SuperMario) / sizeof(MusicNote); i++){
			PLAY(SuperMario[i].note, SuperMario[i].time);
	}
	HAL_TIM_PWM_Stop(&BUZZER_TIM, TIM_CHANNEL_1);
}

//状态机切换
void WorkStateFSM(void)
{
	switch (WorkState)
	{
		case PREPARE_STATE:				//准备模式
		{
			//if (inputmode == STOP) WorkState = STOP_STATE;
			if(prepare_time < 2000) prepare_time++;	
			if(prepare_time >= 2000 && imu.InitFinish == 1 && isCan11FirstRx == 1 && isCan12FirstRx == 1 && isCan21FirstRx == 1 && isCan22FirstRx == 1)//开机二秒后且imu初始化完成且所有can电机上电完成后进入正常模式
			{
				playMusicSuperMario();
				CMRotatePID.Reset(&CMRotatePID);
				WorkState = NORMAL_STATE;
				prepare_time = 0;
			}
			for(int i=0;i<8;i++) 
			{
				{
					InitMotor(can1[i]);
					InitMotor(can2[i]);
				}
			}
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
			FunctionTaskInit();
		}break;
		case NORMAL_STATE:				//正常模式
		{
			if (inputmode == STOP) WorkState = STOP_STATE;
			
			if(functionmode == MIDDLE_POS) WorkState = ADDITIONAL_STATE_ONE;
			if(functionmode == LOWER_POS) WorkState = ADDITIONAL_STATE_TWO;
			/*
			if (inputmode == REMOTE_INPUT)
			{
				if(functionmode == MIDDLE_POS) WorkState = ADDITIONAL_STATE_ONE;
				if(functionmode == LOWER_POS) WorkState = ADDITIONAL_STATE_TWO;
			}*/
		}break;
		case ADDITIONAL_STATE_ONE:		//附加模式一
		{
			if (inputmode == STOP) WorkState = STOP_STATE;
			
			if(functionmode == UPPER_POS) WorkState = NORMAL_STATE;
			if(functionmode == LOWER_POS) WorkState = ADDITIONAL_STATE_TWO;
			/*
			if (inputmode == REMOTE_INPUT)
			{
				if(functionmode == UPPER_POS) WorkState = NORMAL_STATE;
				if(functionmode == LOWER_POS) WorkState = ADDITIONAL_STATE_TWO;
			}*/
		}break;
		case ADDITIONAL_STATE_TWO:		//附加模式二
		{
			if (inputmode == STOP) WorkState = STOP_STATE;
			
			if(functionmode == UPPER_POS) WorkState = NORMAL_STATE;
			if(functionmode == MIDDLE_POS) WorkState = ADDITIONAL_STATE_ONE;
			/*
			if (inputmode == REMOTE_INPUT)
			{
				if(functionmode == UPPER_POS) WorkState = NORMAL_STATE;
				if(functionmode == MIDDLE_POS) WorkState = ADDITIONAL_STATE_ONE;
			}*/
		}break;
		case STOP_STATE:				//紧急停止
		{
			for(int i=0;i<8;i++) 
			{
				if((can1[i]==&FRICL || can1[i]==&FRICR)&&(can2[i]==&CMFL || can2[i]==&CMFR || can2[i]==&CMBL || can2[i]==&CMBR))
				{
					can1[i]->FirstEnter=1;
					can1[i]->lastRead=0;
					can1[i]->RealAngle=0;
					can1[i]->TargetAngle=0;
					can1[i]->offical_speedPID.Reset(&(can1[i]->offical_speedPID));
					(can1[i]->Handle)(can1[i]);
					can2[i]->FirstEnter=1;
					can2[i]->lastRead=0;
					can2[i]->RealAngle=0;
					can2[i]->TargetAngle=0;
					can2[i]->offical_speedPID.Reset(&(can1[i]->offical_speedPID));
					(can2[i]->Handle)(can2[i]);
				}
				else if(can1[i]==&FRICL || can1[i]==&FRICR)
				{
					can1[i]->FirstEnter=1;
					can1[i]->lastRead=0;
					can1[i]->RealAngle=0;
					can1[i]->TargetAngle=0;
					can1[i]->offical_speedPID.Reset(&(can1[i]->offical_speedPID));
					(can1[i]->Handle)(can1[i]);
					InitMotor(can2[i]);
				}
				else if(can2[i]==&CMFL || can2[i]==&CMFR || can2[i]==&CMBL || can2[i]==&CMBR)
				{
					can2[i]->FirstEnter=1;
					can2[i]->lastRead=0;
					can2[i]->RealAngle=0;
					can2[i]->TargetAngle=0;
					can2[i]->offical_speedPID.Reset(&(can1[i]->offical_speedPID));
					(can2[i]->Handle)(can2[i]);
					InitMotor(can1[i]);
				}
				else
				{
					InitMotor(can1[i]);
					InitMotor(can2[i]);
				}
			}
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
			if (inputmode == REMOTE_INPUT || inputmode == KEY_MOUSE_INPUT)
			{
				WorkState = PREPARE_STATE;
				GMYReseted=0;
				GMPReseted=0;
				FunctionTaskInit();
			}
		}break;
		default: break;
	}
}
void ControlRotate(void)
{	
	#ifdef USE_CHASSIS_FOLLOW
	switch (ChassisTwistState)
	{
		case 1: ChassisSpeedRef.rotate_ref=(GMY.RxMsg6623.angle - GM_YAW_ZERO) * 360 / 8192.0f - ChassisTwistGapAngle; break;
		case 2: ChassisSpeedRef.rotate_ref=(GMY.RxMsg6623.angle - GM_YAW_ZERO + 1024) * 360 / 8192.0f - ChassisTwistGapAngle; break;
		default: ChassisSpeedRef.rotate_ref=(GMY.RxMsg6623.angle - GM_YAW_ZERO) * 360 / 8192.0f - ChassisTwistGapAngle; break;
	}
		NORMALIZE_ANGLE180(ChassisSpeedRef.rotate_ref);
	#endif
	CMRotatePID.ref = 0;
	CMRotatePID.fdb = ChassisSpeedRef.rotate_ref;
	CMRotatePID.Calc(&CMRotatePID);
	if(ChassisTwistState) MINMAX(CMRotatePID.output,-20,20);
	//rotate_speed = CMRotatePID.output * 16 + ChassisSpeedRef.forward_back_ref * 0.01 + ChassisSpeedRef.left_right_ref * 0.01;
	rotate_speed = CMRotatePID.output * 16;
}

void Chassis_Data_Decoding()
{
	ControlRotate();
	///
	CMFL.TargetAngle = (  (ChassisSpeedRef.forward_back_ref + fabs(rotate_speed*0.06)) *(cos((GMY.RxMsg6623.angle - GM_YAW_ZERO) * 6.28 / 8192.0f)-sin((GMY.RxMsg6623.angle - GM_YAW_ZERO) * 6.28 / 8192.0f))
						+ ChassisSpeedRef.left_right_ref *(cos((GMY.RxMsg6623.angle - GM_YAW_ZERO) * 6.28 / 8192.0f)+sin((GMY.RxMsg6623.angle - GM_YAW_ZERO) * 6.28 / 8192.0f))
						+ rotate_speed)*12;
	CMFR.TargetAngle = (- (ChassisSpeedRef.forward_back_ref + fabs(rotate_speed*0.06)) *(cos((GMY.RxMsg6623.angle - GM_YAW_ZERO) * 6.28 / 8192.0f)+sin((GMY.RxMsg6623.angle - GM_YAW_ZERO) * 6.28 / 8192.0f))
						+ ChassisSpeedRef.left_right_ref *(cos((GMY.RxMsg6623.angle - GM_YAW_ZERO) * 6.28 / 8192.0f)-sin((GMY.RxMsg6623.angle - GM_YAW_ZERO) * 6.28 / 8192.0f))
						+ rotate_speed)*12;
	CMBL.TargetAngle = (  (ChassisSpeedRef.forward_back_ref + fabs(rotate_speed*0.06)) *(cos((GMY.RxMsg6623.angle - GM_YAW_ZERO) * 6.28 / 8192.0f)+sin((GMY.RxMsg6623.angle - GM_YAW_ZERO) * 6.28 / 8192.0f))
						- ChassisSpeedRef.left_right_ref *(cos((GMY.RxMsg6623.angle - GM_YAW_ZERO) * 6.28 / 8192.0f)-sin((GMY.RxMsg6623.angle - GM_YAW_ZERO) * 6.28 / 8192.0f))
						+ rotate_speed)*12;
	CMBR.TargetAngle = (- (ChassisSpeedRef.forward_back_ref + fabs(rotate_speed*0.06)) *(cos((GMY.RxMsg6623.angle - GM_YAW_ZERO) * 6.28 / 8192.0f)-sin((GMY.RxMsg6623.angle - GM_YAW_ZERO) * 6.28 / 8192.0f))
						- ChassisSpeedRef.left_right_ref *(cos((GMY.RxMsg6623.angle - GM_YAW_ZERO) * 6.28 / 8192.0f)+sin((GMY.RxMsg6623.angle - GM_YAW_ZERO) * 6.28 / 8192.0f))
						+ rotate_speed)*12;
	///
	/*//
	rotate_speed-=0.1;
	CMFL.TargetAngle = (  ChassisSpeedRef.forward_back_ref + ChassisSpeedRef.left_right_ref	+ rotate_speed)*12;
	CMFR.TargetAngle = (- ChassisSpeedRef.forward_back_ref + ChassisSpeedRef.left_right_ref + rotate_speed)*12;
	CMBL.TargetAngle = (  ChassisSpeedRef.forward_back_ref - ChassisSpeedRef.left_right_ref + rotate_speed)*12;
	CMBR.TargetAngle = (- ChassisSpeedRef.forward_back_ref - ChassisSpeedRef.left_right_ref	+ rotate_speed)*12;
	*///
}

//主控制循环
void controlLoop()
{
	getJudgeState();
	WorkStateFSM();
	#ifdef NO_RC_MODE
	CANTxINFO();
	#endif
	
	if(WorkState > 0)
	{
		Chassis_Data_Decoding();
		
		for(int i=0;i<8;i++) if(can1[i]!=0) (can1[i]->Handle)(can1[i]);
		for(int i=0;i<8;i++) if(can2[i]!=0) (can2[i]->Handle)(can2[i]);
/*/混合pid开始(覆盖之前底盘pid)
		static float kp=12,ki=0.17,kd=2;
		static float e1,e2,e3,e4,e12,e13,e14,e23,e24,e34;//偏差，p
		static float s1,s2,s3,s4,s12,s13,s14,s23,s24,s34;//偏差和，i
		static float d1,d2,d3,d4,d12,d13,d14,d23,d24,d34;//偏差差，d
		static float l1,l2,l3,l4,l12,l13,l14,l23,l24,l34;//上一次的偏差，last e
		e1=CMFL.offical_speedPID.ref - CMFL.offical_speedPID.fdb;
		e2=CMFR.offical_speedPID.ref - CMFR.offical_speedPID.fdb;
		e3=CMBL.offical_speedPID.ref - CMBL.offical_speedPID.fdb;
		e4=CMBR.offical_speedPID.ref - CMBR.offical_speedPID.fdb;
		e2=-e2;e4=-e4;
		e12=e1-e2;
		e13=e1-e3;
		e14=e1-e4;
		e23=e2-e3;
		e24=e2-e4;
		e34=e3-e4;
		
		s1+=e1;s2+=e2;s3+=e3;s4+=e4;
		s12+=e12;s13+=e13;s14+=e14;
		s23+=e23;s24+=e24;s34+=e34;
		//让s收敛
		s1*=0.99;s2*=0.99;s3*=0.99;s4*=0.99;
		s12*=0.99;s13*=0.99;s14*=0.99;
		s23*=0.99;s24*=0.99;s34*=0.99;
		
		d1=e1-l1;d2=e2-l2;d3=e3-l3;d4=e4-l4;
		d12=e12-l12;d13=e13-l13;d14=e14-l14;
		d23=e23-l23;d24=e24-l24;d34=e34-l34;
		
		l1=e1;l2=e2;l3=e3;l4=e4;
		l12=e12;l13=e13;l14=e14;
		l23=e23;l24=e24;l34=e34;
		
		CMFL.Intensity=kp*(e1+e12+e13+e14)
									+ki*(s1+s12+s13+s14)
									+kd*(d1-d12-d13-d14);
		CMFR.Intensity=kp*(e2-e12+e23+e24)
									+ki*(s2-s12+s23+s24)
									+kd*(d2+d12-d23-d24);
		CMBL.Intensity=kp*(e3-e13-e23-e34)
									+ki*(s3-s13-s23+s34)
									+kd*(d3+d13+d23-d34);
		CMBR.Intensity=kp*(e4-e14-e24-e34)
									+ki*(s4-s14-s24-s34)
									+kd*(d4+d14+d24+d34);
*///混合pid结束
		OptionalFunction();

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
	}
}

//时间中断入口函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == htim6.Instance)//1ms时钟`
	{
		HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
		//imu解算
		mpu_get_data();
		imu_ahrs_update();
		imu_attitude_update();
		if(imu.FirstEnter == 1) imu.InitCount++;
		if(imu.InitCount == 2000) {imu.InitFinish = 1;imu.FirstEnter = 0;imu.InitCount = 0;}
		//主循环在时间中断中启动
		controlLoop();
		Cap_Run();
		
		//自瞄数据解算（3ms）
		static int aim_cnt=0;
		aim_cnt++;
		if(aim_cnt==3)
		{
			EnemyINFOProcess();
			aim_cnt=0;
		}
		
		HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
	}
	else if (htim->Instance == htim7.Instance)//ims时钟
	{
		
		rc_cnt++;
		if(auto_counter > 0) auto_counter--;
		if(auto_counter_stir > 0) auto_counter_stir--;
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
	else if (htim->Instance == htim10.Instance)  //10ms，处理上位机数据，优先级不高
	{
		#ifdef DEBUG_MODE
		//zykProcessData();
		#endif
	}
}
