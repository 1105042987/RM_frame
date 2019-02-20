/**
  ******************************************************************************
  * File Name          : FunctionTask.c
  * Description        : 用于记录机器人独有的功能
  ******************************************************************************
  *
  * Copyright (c) 2019 Team JiaoLong-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#include "includes.h"
KeyboardMode_e KeyboardMode = NO_CHANGE;
RampGen_t LRSpeedRamp = RAMP_GEN_DAFAULT;   	//斜坡函数
RampGen_t FBSpeedRamp = RAMP_GEN_DAFAULT;
ChassisSpeed_Ref_t ChassisSpeedRef; 
void KeyboardModeFSM(Key *key);
void MouseModeFSM(Mouse *mouse);
void Standardized_Chassis_Move(float Rate);
#include "RobotMotor.h"
#ifdef CONFIGURATION
extern MotorINFO CMFL,CMFR,CMBL,CMBR,GMY,GMP,FRICL,FRICR,STIR,CML,CMR;
#endif

int32_t auto_counter=0;		//用于准确延时的完成某事件

int16_t channelrrow = 0;
int16_t channelrcol = 0;
int16_t channellrow = 0;
int16_t channellcol = 0;

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
}
//限位与同步
void Limit_and_Synchronization()
{
	//MINMAX(UD1.Target,-900,270);//limit
	//CMR.Target =  -CML.Target;
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
	}
	if(WorkState == ADDITIONAL_STATE_ONE)
	{
		
	}
	if(WorkState == ADDITIONAL_STATE_TWO)
	{
		
	}
	/*OnePush(FUNC__RED_RAY_M__READ(),{
		CML.Target = 0;
		CML.Real = 0;
	})
	*/
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
		CML.Target = channelrcol*2;
	}
	if(WorkState == ADDITIONAL_STATE_ONE)
	{
		
	}
	if(WorkState == ADDITIONAL_STATE_TWO)
	{
		
	}
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
		
	}
	if(mouse->last_press_l>50)//左长按
	{
		
	}
	if(mouse->last_press_r==1)//右短按
	{
		
	}
	if(mouse->last_press_r>50)//右长按
	{
		
	}
	
	KeyboardModeFSM(key);
	//*****Don't Use WASD******
	switch (KeyboardMode)	
	{
		case SHIFT_CTRL:		//State Change
		{
			
		}break;
		case CTRL:
		{
			
		}break;
		case SHIFT:
		{
			
		}break;
		case NO_CHANGE:
		{
			
		}break;
	}
	//CM Movement Process 
	//shift: High Speed , ctrl: Low Speed  , shift+ctrl: Don't Move
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
	Limit_and_Synchronization();
}

void KeyboardModeFSM(Key *key)
{
	if((key->v & 0x30) == 0x30)//Shift_Ctrl
	{
		KM_FORWORD_BACK_SPEED=  0;
		KM_LEFT_RIGHT_SPEED = 0;
		KeyboardMode=SHIFT_CTRL;
	}
	else if(key->v & KEY_SHIFT)//Shift
	{
		KM_FORWORD_BACK_SPEED=  HIGH_FORWARD_BACK_SPEED;
		KM_LEFT_RIGHT_SPEED = HIGH_LEFT_RIGHT_SPEED;
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
