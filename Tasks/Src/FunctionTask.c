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
#define  FRICTION_SPEED 5000
#define  STIR_STEP_ANGLE 60
KeyboardMode_e KeyboardMode = NO_CHANGE;
RampGen_t LRSpeedRamp = RAMP_GEN_DAFAULT;   	//斜坡函数
RampGen_t FBSpeedRamp = RAMP_GEN_DAFAULT;
ChassisSpeed_Ref_t ChassisSpeedRef; 
void KeyboardModeFSM(Key *key);
void MouseModeFSM(Mouse *mouse);
void Standardization_Chassis_Move(float Rate);

int32_t auto_counter=0;		//用于准确延时的完成某事件

int16_t channelrrow = 0;
int16_t channelrcol = 0;
int16_t channellrow = 0;
int16_t channellcol = 0;
uint8_t ShootState = 0;
uint8_t ChassisTwistState = 0;
int 	ChassisTwistGapAngle = 0;

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
	ChassisTwistGapAngle = 0;
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
		Standardization_Chassis_Move(1);
		#ifdef USE_AUTOAIM
			autoAimGMCTRL();
		#endif
		ShootState = 0;
		FRICL.Target = 0;
		HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
	}
	if(WorkState == ADDITIONAL_STATE_ONE)
	{
		Standardization_Chassis_Move(1);
		ShootState = 1;
		FRICL.Target = FRICTION_SPEED;
		HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
	}
	if(WorkState == ADDITIONAL_STATE_TWO)
	{
		Standardization_Chassis_Move(1);
		ShootState = 1;
		FRICL.Target = FRICTION_SPEED;
		Delay(20,{STIR.Target-=STIR_STEP_ANGLE;});
		HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
	}
	//状态保证
	Control_SuperCap.release_power = 0;
	Control_SuperCap.stop_power = 0;
	ChassisTwistState = 0;
	HAL_GPIO_WritePin(GPIOG, 0xff<<1, GPIO_PIN_SET);
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
		Standardization_Chassis_Move(1);
	}
	if(WorkState == ADDITIONAL_STATE_ONE)
	{
		Control_SuperCap.release_power = 1;
		Control_SuperCap.stop_power = 0;
		ChassisTwistState = 0;
		if(Control_SuperCap.C_voltage>1200)
			Standardization_Chassis_Move(2);
		else 
			Standardization_Chassis_Move(1);
	}
	if(WorkState == ADDITIONAL_STATE_TWO)
	{	
		Control_SuperCap.release_power = 0;
		Control_SuperCap.stop_power = 0;
		ChassisTwistState = 1;
		Standardization_Chassis_Move(1);
	}
	//底盘摆动
	if(ChassisTwistState) LJHTwist();
	else ChassisDeTwist();
	//超级电容电量显示
	if(Control_SuperCap.C_voltage<1100)
		HAL_GPIO_WritePin(GPIOG, 0xff<<1, GPIO_PIN_SET);
	else{
		HAL_GPIO_WritePin(GPIOG, 0xff<<1, GPIO_PIN_SET);
		int unlight = 7-(Control_SuperCap.C_voltage-1100)/143;
		if(unlight<0) unlight=0;
		HAL_GPIO_WritePin(GPIOG, 0x1fe>>unlight, GPIO_PIN_RESET);
	}
	//状态保证
	ShootState = 0;
	FRICL.Target = 0;
	FRICR.Target = 0;
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
		if(ShootState && abs(STIR.Target-STIR.Real)<5.0) STIR.Target-=STIR_STEP_ANGLE;
	}
	if(mouse->last_press_l>50)//左长按
	{
		if(ShootState && abs(STIR.Target-STIR.Real)<5.0) STIR.Target-=STIR_STEP_ANGLE;
	}
	if(mouse->last_press_r==1)//右短按
	{
		ShootState=1;
		HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
		FRICL.Target = FRICTION_SPEED;
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
		KM_FORWORD_BACK_SPEED=  LOW_FORWARD_BACK_SPEED;
		KM_LEFT_RIGHT_SPEED = LOW_LEFT_RIGHT_SPEED;
		KeyboardMode=CTRL;
	}
	else
	{
		KM_FORWORD_BACK_SPEED=  NORMAL_FORWARD_BACK_SPEED;
		KM_LEFT_RIGHT_SPEED = NORMAL_LEFT_RIGHT_SPEED;
		KeyboardMode=NO_CHANGE;
	}	
}

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
			if(abs((GMY.RxMsg6623.angle - GM_YAW_ZERO) * 360 / 8192.0f - ChassisTwistGapAngle)<3)
			{ChassisTwistGapAngle = -CHASSIS_TWIST_ANGLE_LIMIT;}break;
		}
		case -CHASSIS_TWIST_ANGLE_LIMIT:
		{
			if(abs((GMY.RxMsg6623.angle - GM_YAW_ZERO) * 360 / 8192.0f - ChassisTwistGapAngle)<3)
			{ChassisTwistGapAngle = CHASSIS_TWIST_ANGLE_LIMIT;}break;
		}
	}
}

void ChassisDeTwist(void)
{
	ChassisTwistGapAngle = 0;
}

void LJHTwist(void)
{
	ChassisTwist();
}
void Standardization_Chassis_Move(float Rate)
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
