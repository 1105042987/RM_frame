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
double rotate_speed = 0;
MusicNote SuperMario[] = {
	{H1, 250}, {0, 50}
};
//150十六分音符 300八分音符  

PID_Regulator_t CMRotatePID = CHASSIS_MOTOR_ROTATE_PID_DEFAULT; 
extern int32_t auto_counter;//自动取弹时间中断常量
extern int32_t auto_wait;   //限位器等待
extern int32_t auto_lock; //锁定底盘
extern int32_t cnt_clk;    //登岛时间中断常量
extern uint32_t ctrl_locker;
extern uint32_t shift_locker;
extern uint32_t ctrl_cnt;
extern uint32_t shift_cnt;
extern int32_t doorcount;
extern uint32_t counting;
extern uint32_t claw_warning;
extern uint32_t warning_cnt;
extern uint32_t Claw_Zero_Counting;
extern uint32_t Claw_Zero_Count;
extern uint32_t AutoGet_TotalStep;
extern uint32_t Yaw_Reset_Cnt;
extern uint32_t Yaw_Set_Cnt;
extern uint32_t rotate_waiter;
extern uint32_t sensorlock_cnt;
extern uint32_t clawback;
extern uint32_t clawback_cnt;
extern uint32_t doorshake_cnt;
extern uint32_t saving;
extern uint32_t Z_count;
uint32_t AutoGet_LastStep = 1;
uint32_t AutoGetCnt = 0;
uint16_t SaveClearCnt = 0;

void playMusicSuperMario(void){
	HAL_TIM_PWM_Start(&BUZZER_TIM, TIM_CHANNEL_1);
	for(int i = 0; i < sizeof(SuperMario) / sizeof(MusicNote); i++){
			PLAY(SuperMario[i].note, SuperMario[i].time);
	}
	HAL_TIM_PWM_Stop(&BUZZER_TIM, TIM_CHANNEL_1);
}

void SaveClear()
{
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_0,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_RESET);
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
			if(prepare_time == 2000)//开机二秒进入正常模式
			{
				playMusicSuperMario();
				CMRotatePID.Reset(&CMRotatePID);
				WorkState = NORMAL_STATE;
				prepare_time = 0;
				HAL_GPIO_WritePin(GPIOH,1<<3,GPIO_PIN_SET);
				
				
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
			CLAWLOOSE;
			CLAWIN;
			for(int i=0;i<8;i++) {InitMotor(can1[i]);InitMotor(can2[i]);}
			setCAN11();setCAN12();setCAN21();setCAN22();
			for(int i=0;i<8;i++) {can1[i]->Intensity = 0;can2[i]->Intensity = 0;}
			if (inputmode == REMOTE_INPUT || inputmode == KEY_MOUSE_INPUT)
			{
				WorkState = PREPARE_STATE;
				FunctionTaskInit();
				HAL_GPIO_WritePin(GPIOH,1<<3,GPIO_PIN_RESET);
			}
		}break;
	}
}

void ControlRotate(void)
{	
//	#ifdef USE_CHASSIS_FOLLOW
//		ChassisSpeedRef.rotate_ref=(YTY.RxMsg6623.angle - GM_YAW_ZERO) * 360 / 8192.0f;
//		NORMALIZE_ANGLE180(ChassisSpeedRef.rotate_ref);
//	#endif
	//if(fabs(imu.target_yaw-imu.now_yaw)<1)
		//CM_AutoRotate90 = 0;
	if(imu_pause==0||CM_AutoRotate90==1)
	{
		CMRotatePID.ref = imu.target_yaw;
		CMRotatePID.fdb = imu.now_yaw;
	}
	else
	{
		CMRotatePID.ref = 0;
		CMRotatePID.fdb = ChassisSpeedRef.rotate_ref;
	}
	
	CMRotatePID.Calc(&CMRotatePID);
	rotate_speed = CMRotatePID.output * 13 ;
}
/*struct wheel_rotate_speed calc_wheel_rpm(int16_t fb_ref, int16_t lr_ref, float GapAngle)
{
	struct wheel_rotate_speed wheel_speed;
	static float rotate_ratio_f = ((WHEELBASE+WHEELTRACK)/2-GIMBAL_Y_OFFSET)/RADIAN_COEF; //改成了乘
	static float rotate_ratio_b = ((WHEELBASE+WHEELTRACK)/2+GIMBAL_Y_OFFSET)/RADIAN_COEF; //改成了乘
	static float wheel_rpm_ratio = 60.0f*CHASSIS_DECELE_RATIO/(PI*WHEEL_DIAMETER);//rpm//改成了乘
	float max = 0;
	float gap_angle_radian = GapAngle / RADIAN_COEF;
	int16_t vy = fb_ref;
	int16_t vx = lr_ref;
	int16_t vw = 0;

		//底盘跟随云台
		rotate_ref = GapAngle - ChassisTwistGapAngle;
		vw = PID_PROCESS_Single(&(CHASSIS_FOLLOW_PID),0, rotate_ref);
		if((rotate_ref >= 20) && (rotate_ref <= 40))
		{
			vw += -pow(rotate_ref - 30, 2) + 100;
		}
	
	MINMAX(vx,-MAX_CHASSIS_VX_SPEED,MAX_CHASSIS_VX_SPEED);//mm/s
	MINMAX(vy,-MAX_CHASSIS_VY_SPEED,MAX_CHASSIS_VY_SPEED);//mm/s
	MINMAX(vw,-MAX_CHASSIS_VW_SPEED,MAX_CHASSIS_VW_SPEED);//degree/s
	float cos_sub_sin = cos(gap_angle_radian)-sin(gap_angle_radian);
	float cos_add_sin = cos(gap_angle_radian)+sin(gap_angle_radian);
	wheel_speed.wheel_rpm[0] = (-vx*cos_sub_sin-vy*cos_add_sin+vw*rotate_ratio_f)*wheel_rpm_ratio; //转子的转每分
	wheel_speed.wheel_rpm[1] = (-vx*cos_add_sin+vy*cos_sub_sin+vw*rotate_ratio_f)*wheel_rpm_ratio;
	wheel_speed.wheel_rpm[2] = (vx*cos_sub_sin+vy*cos_add_sin+vw*rotate_ratio_b)*wheel_rpm_ratio;
	wheel_speed.wheel_rpm[3] = (vx*cos_add_sin-vy*cos_sub_sin+vw*rotate_ratio_b)*wheel_rpm_ratio;
//	wheel_speed.wheel_rpm[0] = (-vx*(cos(gap_angle_radian)-sin(gap_angle_radian))-vy*(cos(gap_angle_radian)+sin(gap_angle_radian))+vw*rotate_ratio_f)*wheel_rpm_ratio; //转子的转每分
//	wheel_speed.wheel_rpm[1] = (-vx*(cos(gap_angle_radian)+sin(gap_angle_radian))+vy*(cos(gap_angle_radian)-sin(gap_angle_radian))+vw*rotate_ratio_f)*wheel_rpm_ratio;
//	wheel_speed.wheel_rpm[2] = (vx*(cos(gap_angle_radian)-sin(gap_angle_radian))+vy*(cos(gap_angle_radian)+sin(gap_angle_radian))+vw*rotate_ratio_b)*wheel_rpm_ratio;
//	wheel_speed.wheel_rpm[3] = (vx*(cos(gap_angle_radian)+sin(gap_angle_radian))-vy*(cos(gap_angle_radian)-sin(gap_angle_radian))+vw*rotate_ratio_b)*wheel_rpm_ratio;
	//find max item
	for(uint8_t i=0;i<4;i++)
	{
		if(abs(wheel_speed.wheel_rpm[i]) > max)
			max = abs(wheel_speed.wheel_rpm[i]);
	}
	//equal propotion
	if(max>MAX_RPM)
	{
		float rate=MAX_RPM/max;
		for(uint8_t i=0;i<4;i++) wheel_speed.wheel_rpm[i]*=rate;
	}
	return wheel_speed;
}*/
uint32_t fancheck=0;
extern Engineer_State_e EngineerState;
void Chassis_Data_Decoding()
{
	ControlRotate();
	//imu.rol 14.4上坡 15.5碉堡上坡-15.9碉堡下坡 -14.3下坡 -60正向翻 60反向翻
	if(imu.rol<=-18&&EngineerState==COMMON_STATE&&AutoClimbing==0&&ON_THE_GROUND)
	{
		ChassisSpeedRef.forward_back_ref+=fabs(imu.rol+18)*55 * RC_CHASSIS_SPEED_REF;
	}
	if(imu.rol>=22&&EngineerState==COMMON_STATE&&AutoClimbing==0&&ON_THE_GROUND)
	{
		ChassisSpeedRef.forward_back_ref-=(imu.rol-18)*55*RC_CHASSIS_SPEED_REF;
		fancheck+=1;
	}
	CMFL.TargetAngle = (  ChassisSpeedRef.forward_back_ref	*0.075 
						+ ChassisSpeedRef.left_right_ref	*0.075 
						+ rotate_speed					*0.075)*160;
	CMFR.TargetAngle = (- ChassisSpeedRef.forward_back_ref	*0.075 
						+ ChassisSpeedRef.left_right_ref	*0.075 
						+ rotate_speed					*0.075)*160;
	CMBL.TargetAngle = (  ChassisSpeedRef.forward_back_ref	*0.075 
						- ChassisSpeedRef.left_right_ref	*0.075 
						+ rotate_speed					*0.075)*160;
	CMBR.TargetAngle = (- ChassisSpeedRef.forward_back_ref	*0.075 
						- ChassisSpeedRef.left_right_ref	*0.075 
						+ rotate_speed					*0.075)*160;
	//CM1.TargetAngle=CM2.TargetAngle*CMFL.TargetAngle/CMFR.TargetAngle;
	
	
//	if(imu.now_rol<=-30)
//	{
//		CMFL.TargetAngle*=2;
//		CMFR.TargetAngle*=2;
//	}
//	if(imu.now_rol>=30)
//	{
//		CMBL.TargetAngle=fabs(CMBL.TargetAngle);
//	}
}

//主控制循环
void controlLoop()
{
	getJudgeState();
	WorkStateFSM();
	
	if(WorkState > 0)
	{
		Chassis_Data_Decoding();
		for(int i=0;i<8;i++) if(can1[i]!=0) (can1[i]->Handle)(can1[i]);
		for(int i=0;i<8;i++) if(can2[i]!=0) (can2[i]->Handle)(can2[i]);
		
		
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

void checkStuck()
{
	for(int i=0;i<8;i++)
	{
		if(can1[i]!=0 && can1[i]->warningMoment!=0 && can1[i]->warningCount<5000 && can1[i]->RxMsgC6x0.moment > can1[i]->warningMoment) 
		{
			if(can1[i]->warningDir == 1)can1[i]->warningCount++;
			else can1[i]->warningCount = 0;
			can1[i]->warningDir = 1;
		}
		else if(can1[i]!=0 && can1[i]->warningMoment!=0 && can1[i]->warningCount<5000 && can1[i]->RxMsgC6x0.moment < -can1[i]->warningMoment) 
		{
			if(can1[i]->warningDir == -1)can1[i]->warningCount++;
			else can1[i]->warningCount = 0;
			can1[i]->warningDir = -1;
		}
		else can1[i]->warningCount = 0;
	}
	for(int i=0;i<8;i++)
	{
		if(can2[i]!=0 && can2[i]->warningMoment!=0 && can2[i]->warningCount<5000 && can2[i]->RxMsgC6x0.moment > can2[i]->warningMoment) 
		{
			if(can2[i]->warningDir == 1)can2[i]->warningCount++;
			else can2[i]->warningCount = 0;
			can2[i]->warningDir = 1;
		}
		else if(can2[i]!=0 && can2[i]->warningMoment!=0 && can2[i]->warningCount<5000 && can2[i]->RxMsgC6x0.moment < -can2[i]->warningMoment) 
		{
			if(can2[i]->warningDir == -1)can2[i]->warningCount++;
			else can2[i]->warningCount = 0;
			can2[i]->warningDir = -1;
		}
		else can2[i]->warningCount = 0;
	}
}

void checkAutoGet()
{
	if(AutoGet_TotalStep > 1)
	{
		if(AutoGet_LastStep == AutoGet_TotalStep)
		{
			AutoGetCnt++;
		}
		else AutoGetCnt = 0;
		if(AutoGetCnt > 2000)
		{
			AutoGet_Error=1;
			if(AutoGet_TotalStep==2)
				custom_data.masks=0x80;
			if(AutoGet_TotalStep==3)
				custom_data.masks=0x40;
			AutoGet_Stop_And_Clear();
			AutoGetCnt = 0;
		}
		AutoGet_LastStep = AutoGet_TotalStep;
	}
}
uint32_t protect_cnt=0;
void Protect_dog()
{
	CLAWLOOSE;
	CLAWIN;
	WorkState=STOP_STATE;
}
//时间中断入口函数
uint16_t dataSendCnt = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == htim6.Instance)//2ms时钟`
	{
		HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
		//imu解算
		mpu_get_data();
		imu_ahrs_update();
		imu_attitude_update();
		imu_yaw_normalize();
		if(imu_pause==1&&CM_AutoRotate90!=1) imu.target_yaw = imu.now_yaw;		//暂停陀螺仪时直接让目标值等于陀螺仪当前值
		//主循环在时间中断中启动
		controlLoop();
		
		HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
	}
	else if (htim->Instance == htim7.Instance)//ims时钟
	{
		rc_cnt++;
		if(auto_counter > 0) auto_counter--;
		if(Yaw_Reset_Cnt>0)  Yaw_Reset_Cnt--;
		if(Yaw_Set_Cnt>0)  Yaw_Set_Cnt--;
		if(rotate_waiter>0) rotate_waiter--;
		if(cnt_clk > 0) cnt_clk--;
		if(auto_wait>0) auto_wait--;
		if(auto_waiter>0)auto_waiter--;
		if(counting==1) doorcount++;
		if(counting==0) doorcount=0;
		if(claw_warning==1) warning_cnt++;
		if(claw_warning==0) warning_cnt=0;
		if(Claw_Zero_Counting==1) Claw_Zero_Count++;
		if(Claw_Zero_Counting==0) Claw_Zero_Count=0;
		if(ctrl_locker==1) ctrl_cnt=300;
		if(ctrl_locker==0&&ctrl_cnt>0) ctrl_cnt--;
		if(shift_locker==1) shift_cnt=300;
		if(shift_locker==0&&shift_cnt>0) shift_cnt--;
		if(sensorlock_cnt>0) sensorlock_cnt--;
		if(clawback==1&&clawback_cnt>0) clawback_cnt--;
		if(clawback==0) clawback_cnt=1000;
		if(doorshake_cnt>0) doorshake_cnt--;
		if(Z_count>0) Z_count--;
		//		if(SaveClearCnt < 1000)SaveClearCnt++;      暂时不知道是干什么用的
//		else
//		{
//			SaveClearCnt = 0;
//			SaveClear();
//			if(saving == 0)saving = 12;
//		}
		protect_cnt++;
		if(protect_cnt>=150)
			Protect_dog();
		checkStuck();
		checkAutoGet();
		
		dataSendCnt++;
		if(dataSendCnt >= 120)
		{
			Referee_Transmit_UserData();
			dataSendCnt = 0;
		}
		
		if (rc_update)
		{
			if( (rc_cnt <= 17) && (rc_first_frame == 1))
			{
				RemoteDataProcess(rc_data);				//遥控器数据解算
				HAL_UART_AbortReceive(&RC_UART);
				HAL_UART_Receive_DMA(&RC_UART, rc_data, 18);
				rc_cnt = 0;
			}
			else
			{
				if(rc_first_frame) 
					WorkState = PREPARE_STATE;
				HAL_UART_AbortReceive(&RC_UART);
				HAL_UART_Receive_DMA(&RC_UART, rc_data, 18);
				rc_cnt = 0;
				rc_first_frame = 1;
				CMBL.TargetAngle=0;
				CMBR.TargetAngle=0;
				CMFL.TargetAngle=0;
				CMFR.TargetAngle=0;
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
