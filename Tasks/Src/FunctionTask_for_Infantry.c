/**
  ******************************************************************************
  * File Name          : FunctionTask.c
  * Description        : 用于记录机器人独有的功能
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#include "includes.h"
#define  STIR_STEP_ANGLE 60
KeyboardMode_e KeyboardMode = NO_CHANGE;
RampGen_t LRSpeedRamp = RAMP_GEN_DAFAULT;   	//斜坡函数
RampGen_t FBSpeedRamp = RAMP_GEN_DAFAULT;
ChassisSpeed_Ref_t ChassisSpeedRef; 
void KeyboardModeFSM(Key *key);
void MouseModeFSM(Mouse *mouse);
void Standardized_Chassis_Move(float Rate);
void ShootOneBullet();

#include "RobotMotor.h"
#ifdef CONFIGURATION
extern MotorINFO CMFL,CMFR,CMBL,CMBR,GMY,GMP,FRICL,FRICR,STIR,CML,CMR;
#endif

int32_t auto_counter=0;		//用于准确延时的完成某事件

int16_t channelrrow = 0;
int16_t channelrcol = 0;
int16_t channellrow = 0;
int16_t channellcol = 0;
uint8_t ShootState = 0;
uint8_t cdflag0 = 0;
uint8_t burst = 0;
uint16_t allowBullet0 = 0;
uint16_t FtictionSpeed = 5000;

//初始化
void FunctionTaskInit()
{
	LRSpeedRamp.SetScale(&LRSpeedRamp, MOUSE_LR_RAMP_TICK_COUNT);
	FBSpeedRamp.SetScale(&FBSpeedRamp, MOUSR_FB_RAMP_TICK_COUNT);
	LRSpeedRamp.ResetCounter(&LRSpeedRamp);
	FBSpeedRamp.ResetCounter(&FBSpeedRamp);
	
	ChassisSpeedRef.forward_back_ref = 0.0f;
	ChassisSpeedRef.left_right_ref = 0.0f;
	ChassisSpeedRef.rotate_ref = 0.0f;
	
	KeyboardMode=NO_CHANGE;
	ShootState = 0;
	ChassisTwistState = 0;
}
//限位与同步
void Limit_and_Synchronization()
{
	//MINMAX(UD1.Target,-900,270);//limit
	FRICR.Target = -FRICL.Target;
}
//******************
//遥控器模式功能编写
//******************
void RemoteControlProcess(Remote *rc)
{
	if(WorkState <= 0) return;
	//max=660
	channelrrow = (rc->ch0 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET); 
	channelrcol = (rc->ch1 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET); 
	channellrow = (rc->ch2 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET); 
	channellcol = (rc->ch3 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET); 
	if(WorkState == NORMAL_STATE)
	{	
		Standardized_Chassis_Move(1);
		#ifdef USE_AUTOAIM
			autoAimGMCTRL();
		#endif
		ShootState = 0;
		FRICL.Target = 0;
		HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
	}
	if(WorkState == ADDITIONAL_STATE_ONE)
	{
		Standardized_Chassis_Move(1);
		ShootState = 1;
		FRICL.Target = FtictionSpeed;
		HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
	}
	if(WorkState == ADDITIONAL_STATE_TWO)
	{
		Standardized_Chassis_Move(1);
		ShootState = 1;
		FRICL.Target = FtictionSpeed;
		Delay(20,{STIR.Target-=STIR_STEP_ANGLE;});
		HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
	}
	//状态保证
	Control_SuperCap.release_power = 0;
	Control_SuperCap.stop_power = 0;
	ChassisTwistState = 0;
	#ifdef USE_SUPER_CAP
		LED_Show_SuperCap_Voltage(0);
	#endif
	Limit_and_Synchronization();
}
//**************************
//遥控器**测试**模式功能编写
//**************************
void RemoteTestProcess(Remote *rc)
{
	if(WorkState <= 0) return;
	//max=660
	channelrrow = (rc->ch0 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET); 
	channelrcol = (rc->ch1 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET); 
	channellrow = (rc->ch2 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET); 
	channellcol = (rc->ch3 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET); 
	if(WorkState == NORMAL_STATE)
	{	
		Control_SuperCap.release_power = 0;
		Control_SuperCap.stop_power = 0;
		ChassisTwistState = 0;
		Standardized_Chassis_Move(1);
	}
	if(WorkState == ADDITIONAL_STATE_ONE)
	{
		Control_SuperCap.release_power = 1;
		Control_SuperCap.stop_power = 0;
		ChassisTwistState = 0;
		if(Control_SuperCap.C_voltage>1200)
			Standardized_Chassis_Move(2);
		else 
			Standardized_Chassis_Move(1);
	}
	if(WorkState == ADDITIONAL_STATE_TWO)
	{	
		Control_SuperCap.release_power = 0;
		Control_SuperCap.stop_power = 0;
		ChassisTwistState = 1;
		Standardized_Chassis_Move(1);
	}
	//底盘摆动
	if(ChassisTwistState) ChassisTwist();
	else ChassisDeTwist();
	//超级电容电量显示
	#ifdef USE_SUPER_CAP
		LED_Show_SuperCap_Voltage(1);
	#endif
	//状态保证
	ShootState = 0;
	FRICL.Target = 0;
	Limit_and_Synchronization();
}
//****************
//键鼠模式功能编写
//****************
uint16_t KM_FORWORD_BACK_SPEED 	= NORMAL_FORWARD_BACK_SPEED;
uint16_t KM_LEFT_RIGHT_SPEED  	= NORMAL_LEFT_RIGHT_SPEED;
void MouseKeyControlProcess(Mouse *mouse, Key *key)
{	
	if(WorkState <= 0) return;
	mouse->last_press_l=mouse->last_press_l*mouse->press_l+mouse->press_l;
	mouse->last_press_r=mouse->last_press_r*mouse->press_r+mouse->press_r;
	MINMAX(mouse->x, -150, 150); 
	MINMAX(mouse->y, -150, 150); 
	
	#ifdef USE_CHASSIS_FOLLOW
		GMY.Target += mouse->x * MOUSE_TO_YAW_ANGLE_INC_FACT;
		GMP.Target -= mouse->y * MOUSE_TO_PITCH_ANGLE_INC_FACT;
	#else
		ChassisSpeedRef.rotate_ref = -mouse->x * RC_ROTATE_SPEED_REF;
	#endif
	
	if(mouse->last_press_l==1)//左短按
	{
		if(ShootState)ShootOneBullet();
	}
	if(mouse->last_press_l>50)//左长按
	{
		if(ShootState)ShootOneBullet();
	}
	if(mouse->last_press_r==1)//右短按
	{
		ShootState=1;
		HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
		FRICL.Target = FtictionSpeed;
	}
	if(mouse->last_press_r>50)//右长按
	{
		ShootState = 0;
		HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_RESET);
		FRICL.Target = 0;
	}
	
	Control_SuperCap.release_power = 0;
	Control_SuperCap.stop_power = 0;
	KeyboardModeFSM(key);
	
	if(key->v & KEY_Z)
	{
		FtictionSpeed = 5000; //13m/s
		realBulletSpeed0 = 13;
		FRICL.Target = FtictionSpeed;
		ShootState=1;
	}
	else if(key->v & KEY_X)
	{
		FtictionSpeed = 6500; //20m/s
		realBulletSpeed0 = 20;
		FRICL.Target = FtictionSpeed;
		ShootState=1;
	}
	else if(key->v & KEY_C)
	{
		FtictionSpeed = 8100; //28m/s
		realBulletSpeed0 = 28;
		FRICL.Target = FtictionSpeed;
		ShootState=1;
	}
	
	switch (KeyboardMode)
	{
		case SHIFT_CTRL:		//State Change
		{
			
			break;
		}
		case CTRL:				//slow
		{
			
		}//DO NOT NEED TO BREAK
		case SHIFT:				//quick
		{
			
		}//DO NOT NEED TO BREAK
		case NO_CHANGE:			//normal
		{//CM Movement Process
			if(key->v & KEY_W)  		//key: w
				ChassisSpeedRef.forward_back_ref =  KM_FORWORD_BACK_SPEED* FBSpeedRamp.Calc(&FBSpeedRamp);
			else if(key->v & KEY_S) 	//key: s
				ChassisSpeedRef.forward_back_ref = -KM_FORWORD_BACK_SPEED* FBSpeedRamp.Calc(&FBSpeedRamp);
			else
			{
				ChassisSpeedRef.forward_back_ref = 0;
				FBSpeedRamp.ResetCounter(&FBSpeedRamp);
			}
			if(key->v & KEY_D)  		//key: d
				ChassisSpeedRef.left_right_ref =  KM_LEFT_RIGHT_SPEED * LRSpeedRamp.Calc(&LRSpeedRamp);
			else if(key->v & KEY_A) 	//key: a
				ChassisSpeedRef.left_right_ref = -KM_LEFT_RIGHT_SPEED * LRSpeedRamp.Calc(&LRSpeedRamp);
			else
			{
				ChassisSpeedRef.left_right_ref = 0;
				LRSpeedRamp.ResetCounter(&LRSpeedRamp);
			}
		}
	}
	Limit_and_Synchronization();
}

void KeyboardModeFSM(Key *key)
{
	if((key->v & 0x30) == 0x30)//Shift_Ctrl
	{
		KM_FORWORD_BACK_SPEED=  LOW_FORWARD_BACK_SPEED;
		KM_LEFT_RIGHT_SPEED = LOW_LEFT_RIGHT_SPEED;
		KeyboardMode=SHIFT_CTRL;
	}
	else if(key->v & KEY_SHIFT)//Shift
	{
		//SuperCap Control
		Control_SuperCap.release_power = 1;
		Control_SuperCap.stop_power = 0;
		if(Control_SuperCap.C_voltage>1200)
		{
			KM_FORWORD_BACK_SPEED=  HIGH_FORWARD_BACK_SPEED;
			KM_LEFT_RIGHT_SPEED = HIGH_LEFT_RIGHT_SPEED;
		}
		else
		{
			KM_FORWORD_BACK_SPEED=  NORMAL_FORWARD_BACK_SPEED;
			KM_LEFT_RIGHT_SPEED = NORMAL_LEFT_RIGHT_SPEED;
		}
		
		KeyboardMode=SHIFT;
	}
	else if(key->v & KEY_CTRL)//Ctrl
	{
		burst = 1;
		KM_FORWORD_BACK_SPEED=  LOW_FORWARD_BACK_SPEED;
		KM_LEFT_RIGHT_SPEED = LOW_LEFT_RIGHT_SPEED;
		KeyboardMode=CTRL;
	}
	else
	{
		burst = 0;
		KM_FORWORD_BACK_SPEED=  NORMAL_FORWARD_BACK_SPEED;
		KM_LEFT_RIGHT_SPEED = NORMAL_LEFT_RIGHT_SPEED;
		KeyboardMode=NO_CHANGE;
	}	
}


void Standardized_Chassis_Move(float Rate)
{
	ChassisSpeedRef.forward_back_ref = channelrcol * RC_CHASSIS_SPEED_REF*Rate;
	ChassisSpeedRef.left_right_ref   = channelrrow * RC_CHASSIS_SPEED_REF/2*Rate;
	#ifdef USE_CHASSIS_FOLLOW
		GMY.Target += channellrow * RC_GIMBAL_SPEED_REF;
		GMP.Target += channellcol * RC_GIMBAL_SPEED_REF;
	#else
		ChassisSpeedRef.rotate_ref = -channellrow * RC_ROTATE_SPEED_REF;
	#endif
}

void ShootOneBullet()
{
	#ifdef USE_HEAT_LIMIT_INFANTRY
	if(JUDGE_State == ONLINE && fakeHeat0 > (maxHeat0 - realBulletSpeed0) && !burst)cdflag0 = 1;
	else cdflag0 = 0;
	if((STIR.Real - STIR.Target <= 50) && burst)
	{
		if(((!cdflag0) || JUDGE_State == OFFLINE))
		{
			if(maxHeat0>fakeHeat0)allowBullet0 = (maxHeat0-fakeHeat0)/realBulletSpeed0;
			else allowBullet0 = 0;
			if(allowBullet0 >= 6)allowBullet0 = 6;
			for(int i=0;i<allowBullet0;i++)
			{
				if(fakeHeat0 < (maxHeat0 - 1*realBulletSpeed0))
				{
					STIR.Target -= STIR_STEP_ANGLE;
					fakeHeat0 += realBulletSpeed0;
				}
				else 
				{
					if(STIR.Real - STIR.Target <= 0)STIR.Target = -STIR_STEP_ANGLE * floor(-STIR.Real/STIR_STEP_ANGLE);
				}
			}
			allowBullet0 = 0;
		}
	}
	else if((STIR.Real - STIR.Target <= 5))
	{
		if(((!cdflag0) || JUDGE_State == OFFLINE) && fakeHeat0 < (maxHeat0 - realBulletSpeed0))
		{
			if(fakeHeat0 < (maxHeat0 - 1*realBulletSpeed0))
			{
				{
					STIR.Target -= STIR_STEP_ANGLE;
					fakeHeat0 += realBulletSpeed0;
				}
			}
			else if(STIR.Real - STIR.Target <= 0)STIR.Target = -STIR_STEP_ANGLE * floor(-STIR.Real/STIR_STEP_ANGLE);
		}
	}
	#else
	STIR.Target -= STIR_STEP_ANGLE;
	#endif
}
