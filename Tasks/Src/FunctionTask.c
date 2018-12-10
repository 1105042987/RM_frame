/**
  ******************************************************************************
  * File Name          : FunctionTask.c
  * Description        : ���ڼ�¼�����˶��еĹ���
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#include "includes.h"

KeyboardMode_e KeyboardMode = NO_CHANGE;
RampGen_t LRSpeedRamp = RAMP_GEN_DAFAULT;   	//б�º���
RampGen_t FBSpeedRamp = RAMP_GEN_DAFAULT;
ChassisSpeed_Ref_t ChassisSpeedRef; 

int32_t auto_counter=0;		//����׼ȷ��ʱ�����ĳ�¼�

/**
	***********************************************
	*һ���ǵ�
	*********************************************
	* reset_flag ���ñ�����һ�ο���ִֻ��һ��
	*	
	*/
	uint8_t reset_flag = 1;
	
	
	
	
	
/*****************************************/
int16_t channelrrow = 0;
int16_t channelrcol = 0;
int16_t channellrow = 0;
int16_t channellcol = 0;
int16_t testIntensity = 0;

//��ʼ��
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

void OptionalFunction()
{
	Cap_Control();
	PowerLimitation();
}

void Limit_and_Synchronization()
{
	//demo
//	MINMAX(NMUDL.TargetAngle,-650,0);//limit
//	MINMAX(NMUDR.TargetAngle,0,650);
	
//	MINMAX(NMUDL.TargetAngle,-650,600);//limit
//	MINMAX(NMUDR.TargetAngle,600,650);
	NMUDL.TargetAngle = -NMUDR.TargetAngle;//sychronization
	//demo end
}
//******************
//ң����ģʽ���ܱ�д
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
		ChassisSpeedRef.forward_back_ref = channelrcol * RC_CHASSIS_SPEED_REF;
		ChassisSpeedRef.left_right_ref   = channelrrow * RC_CHASSIS_SPEED_REF/2;
		ChassisSpeedRef.rotate_ref = -channellrow * RC_ROTATE_SPEED_REF;

//TODO/******��δ�õ��Ĳ���  ����������********/		
//		#ifdef USE_CHASSIS_FOLLOW
//		GMY.TargetAngle += channellrow * RC_GIMBAL_SPEED_REF ;
//		GMP.TargetAngle += channellcol * RC_GIMBAL_SPEED_REF ;
//		#else
//		ChassisSpeedRef.rotate_ref = channellrow * RC_ROTATE_SPEED_REF;
//		#endif
//		FRICL.TargetAngle = 0;
//		FRICR.TargetAngle = 0;
/******δ�õ�����********/		
		
	}
	if(WorkState == ADDITIONAL_STATE_ONE)
	{
//TODO/******��δ�õ��Ĳ���  ����������********/
//		FRICL.TargetAngle = -4200;
//		FRICR.TargetAngle = 4200;
/******δ�õ�����********/	
		
		if(channellcol>200){       //UP
		NMUDL.TargetAngle += channellcol * RC_ROTATE_SPEED_REF ;
		NMUDR.TargetAngle -= channellcol * RC_ROTATE_SPEED_REF ;
		}	else if(channellcol<-200){		//DOWN
		NMUDL.TargetAngle += channellcol * RC_ROTATE_SPEED_REF ;
		NMUDR.TargetAngle -= channellcol * RC_ROTATE_SPEED_REF ;
		}
		
		if(channelrcol>200){			//Forward
		NMUDFL.TargetAngle -= channelrcol * RC_ROTATE_SPEED_REF ;
		NMUDFR.TargetAngle += channelrcol * RC_ROTATE_SPEED_REF ;
		}else if(channelrcol<-200){//Back
		NMUDFL.TargetAngle -= channelrcol * RC_ROTATE_SPEED_REF ;
		NMUDFR.TargetAngle += channelrcol * RC_ROTATE_SPEED_REF ;
		}
		
	}
	if(WorkState == ADDITIONAL_STATE_TWO)
	{
//TODO/******��δ�õ��Ĳ���  ����������********/
//		FRICL.TargetAngle = 5000;
//		FRICR.TargetAngle = -5000;
//		Delay(20,{STIR.TargetAngle-=60;});
//TODO/**********�ǵ�����̧����λ************/
		if(reset_flag){
			
				NMUDL.TargetAngle = 300;
				NMUDR.TargetAngle = -300;
				reset_flag = 0;
}
		if(channellcol>200){
				NMUDL.TargetAngle -= 5;
				if(NMUDL.TargetAngle<-640 && NMUDL.RxMsgC6x0.moment>14800){
					NMUDFL.TargetAngle -= 5;
	}
}
/**********************/	
	}
	Limit_and_Synchronization();
}


uint16_t KM_FORWORD_BACK_SPEED 	= NORMAL_FORWARD_BACK_SPEED;
uint16_t KM_LEFT_RIGHT_SPEED  	= NORMAL_LEFT_RIGHT_SPEED;
void KeyboardModeFSM(Key *key);

//****************
//����ģʽ���ܱ�д
//****************
void MouseKeyControlProcess(Mouse *mouse, Key *key)
{	
	if(WorkState <= 0) return;
	
	MINMAX(mouse->x, -150, 150); 
	MINMAX(mouse->y, -150, 150); 

	KeyboardModeFSM(key);
	
	switch (KeyboardMode)
	{
		case SHIFT_CTRL:		//State control
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
