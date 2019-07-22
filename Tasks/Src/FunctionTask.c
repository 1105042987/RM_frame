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

#define STIR_ANGLE 45
KeyboardMode_e KeyboardMode = NO_CHANGE;
MouseMode_e MouseLMode = NO_CLICK;
MouseMode_e MouseRMode = NO_CLICK;
RampGen_t LRSpeedRamp = RAMP_GEN_DAFAULT;   	//斜坡函数
RampGen_t FBSpeedRamp = RAMP_GEN_DAFAULT;
ChassisSpeed_Ref_t ChassisSpeedRef; 

int ChassisTwistGapAngle = 0;

int32_t up_counter=0;
int8_t direction=1;
int32_t auto_counter,auto_counter_stir=-1;		//用于准确延时的完成某事件
int32_t auto_counter_pwm=0;
int32_t auto_change_stir=0;
uint16_t aimcount=0;

int16_t channelrrow,channelrcol,channellrow,channellcol;
int16_t testIntensity = 0;
uint8_t ShootState = 0;


//初始化
void FunctionTaskInit()
{
	LRSpeedRamp.SetScale(&LRSpeedRamp, MOUSE_LR_RAMP_TICK_COUNT);
	FBSpeedRamp.SetScale(&FBSpeedRamp, MOUSR_FB_RAMP_TICK_COUNT);
	LRSpeedRamp.ResetCounter(&LRSpeedRamp);
	FBSpeedRamp.ResetCounter(&FBSpeedRamp);
	
	KeyboardMode=NO_CHANGE;
}

void Limit_and_Synchronization()
{
	//demo
	//MINMAX(UD1.TargetAngle,-900,270);//limit
	//UD2.TargetAngle=-UD1.TargetAngle;//sychronization
	//demo end
}
//******************
//遥控器模式功能编写
//******************
void RemoteControlProcess(Remote *rc){
	static int flag=0;
	if(WorkState <= 0) return;
	//max=660
	channelrrow = (rc->ch0 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET); 
	channelrcol = (rc->ch1 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET); 
	channellrow = (rc->ch2 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET); 
	channellcol = (rc->ch3 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET);
	
	GMY.TargetAngle -= channellrow * RC_GIMBAL_SPEED_REF*0.5;
	GMP.TargetAngle += channellcol * RC_GIMBAL_SPEED_REF*0.5;

	if(WorkState == NORMAL_STATE){
		ShootState = 0;
		FRICL.TargetAngle = 0;
		FRICR.TargetAngle = 0;
		STIR.TargetAngle=STIR.RealAngle;
		HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
	}
	if(WorkState == ADDITIONAL_STATE_ONE){
		ShootState = 0;
		FRICL.TargetAngle = -0;
		FRICR.TargetAngle = 0;
		STIR.TargetAngle=STIR.RealAngle;
		HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
//		
//===================================================================================================
		#ifdef USE_AUTOAIM
		AutoAim();
		#endif /*USE_AUTOAIM*/
//===================================================================================================
	}
	if(WorkState == ADDITIONAL_STATE_TWO){
		ShootState = 1;
		FRICL.TargetAngle = -7000;
		FRICR.TargetAngle = 7000;
		//Delay(5,{STIR.TargetAngle-=60;})
		if(STIR.TargetAngle-STIR.RealAngle>-150){
			Delay(2,{STIR.TargetAngle-=60;});
		}
		else{
			STIR.TargetAngle=STIR.RealAngle+100;
		  Delay(2,{STIR.TargetAngle-=60;});
			//Delay(5,{STIR.TargetAngle-=90;});
		}
		HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
	}
	OnePush(WorkState==ADDITIONAL_STATE_TWO,{
		    STIR.TargetAngle-=STIR_ANGLE;
	      });
	OnePush(abs(STIR.RxMsgC6x0.moment)>6000,{
		    STIR.TargetAngle+=STIR_ANGLE*1.2;
		    direction=0;
	    	auto_counter_stir = 100;
	      });
	OnePush(auto_counter_stir==0,{
		    if(abs(STIR.RxMsgC6x0.moment)<3000){	
		      	STIR.TargetAngle-=STIR_ANGLE*1.2;
			      direction=1;
		   }
		    else{
		    	  STIR.TargetAngle+=STIR_ANGLE;
			      auto_counter_stir = 100;
		   }
	    })
  OnePush(channelrcol>100,{
		flag++;
		up_counter=0;
	})
	OnePush(channelrcol<-100,{
		flag--;
		up_counter=0;
	})
	
	if(channelrcol>0) flag = (channelrcol*7)/661;
	switch(flag)
	{
		case 1:if(up_counter>500){up_counter=0;STIR.TargetAngle-=STIR_ANGLE;}break;
		case 2:if(up_counter>300){up_counter=0;STIR.TargetAngle-=STIR_ANGLE;}break;
		case 3:if(up_counter>200){up_counter=0;STIR.TargetAngle-=STIR_ANGLE;}break;
		case 4:if(up_counter>100){up_counter=0;STIR.TargetAngle-=STIR_ANGLE;}break;
		case 5:if(up_counter>80){up_counter=0;STIR.TargetAngle-=STIR_ANGLE;}break;
		case 6:if(up_counter>30){up_counter=0;STIR.TargetAngle-=STIR_ANGLE;}break;
		default:break;
	}
	Limit_and_Synchronization();
}


uint16_t KM_FORWORD_BACK_SPEED 	= NORMAL_FORWARD_BACK_SPEED;
uint16_t KM_LEFT_RIGHT_SPEED  	= NORMAL_LEFT_RIGHT_SPEED;
void KeyboardModeFSM(Key *key);
void MouseModeFSM(Mouse *mouse);

//****************
//键鼠模式功能编写
//****************
void MouseKeyControlProcess(Mouse *mouse, Key *key){	
	if(WorkState <= 0) return;
	MINMAX(mouse->x, -35, 35); 
	MINMAX(mouse->y, -25, 25); 
	
	//#ifdef USE_AutoAim
	#ifdef USE_AUTOAIM
	switch(aimcount){
		case(0):{
		
		}break;
		case(1):{
			AutoAim();
			
		}break;
	}
	#endif
	
	//#ifdef USE_CHASSIS_FOLLOW
	GMY.TargetAngle -= mouse->x * MOUSE_TO_YAW_ANGLE_INC_FACT*1.5;
	GMP.TargetAngle -= mouse->y * MOUSE_TO_PITCH_ANGLE_INC_FACT*1.2;
	//#else
	//ChassisSpeedRef.rotate_ref = -mouse->x * RC_ROTATE_SPEED_REF;
	//#endif
	if(key->v & KEY_W)
		GMP.TargetAngle+= SHOOTMODE_GM_ADJUST_ANGLE;
	else if(key->v & KEY_S)
		GMP.TargetAngle-= SHOOTMODE_GM_ADJUST_ANGLE;
	else if(key->v & KEY_D)
		GMY.TargetAngle-= SHOOTMODE_GM_ADJUST_ANGLE;
	else if(key->v & KEY_A)
		GMY.TargetAngle+= SHOOTMODE_GM_ADJUST_ANGLE;
	
	MouseModeFSM(mouse);
	
	switch(MouseRMode){
		case SHORT_CLICK:{
			aimcount=(aimcount+1)%2;
//			 AutoAim();
//			ShootState = 1;
//			HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
//			FRICL.TargetAngle = -7000;
//			FRICR.TargetAngle = 7000;
		}break;
		case LONG_CLICK:{
//			if(ShootState){
//				ShootState = 0;
//				HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_RESET);
//				FRICL.TargetAngle = 0;
//				FRICR.TargetAngle = 0;
//					#ifdef USE_AUTOAIM
//		       AutoAim();
//		      #endif /*USE_AUTOAIM*/
//			}
		}break;
		default: break;
	}
	
	switch(MouseLMode){
		case SHORT_CLICK:{
			if(ShootState&&(STIR.TargetAngle-STIR.RealAngle>-120.0)){Delay(5,{STIR.TargetAngle-=45;});}
			else if(ShootState&&(STIR.TargetAngle-STIR.RealAngle<=-120.0)){
					STIR.TargetAngle=STIR.RealAngle+100;
			    Delay(5,{STIR.TargetAngle-=45;});
			}
			
		}break;
		case LONG_CLICK:{
			if(ShootState&&(STIR.TargetAngle-STIR.RealAngle>-120.0)){Delay(2,{STIR.TargetAngle-=45;});}
			else if(ShootState&&(STIR.TargetAngle-STIR.RealAngle<=-120.0)){
				STIR.TargetAngle=STIR.RealAngle+100;
			  Delay(2,{STIR.TargetAngle-=45;});
			}
		}break;
		default:{
			STIR.TargetAngle=STIR.RealAngle;
		}
		break;
	}
	KeyboardModeFSM(key);
	
	switch (KeyboardMode){
		case SHIFT_CTRL:{		//State control
			break;
		}
		case CTRL:{
			
		}//DO NOT NEED TO BREAK
		case SHIFT:{
			#ifdef USE_AUTOAIM
			AutoAim();
			#endif
		}//DO NOT NEED TO BREAK
		case NO_CHANGE:{//normal	//CM Movement Process
//			if(key->v & KEY_W)  		//key: w
//				ChassisSpeedRef.forward_back_ref =  KM_FORWORD_BACK_SPEED* FBSpeedRamp.Calc(&FBSpeedRamp);
//			else if(key->v & KEY_S) 	//key: s
//				ChassisSpeedRef.forward_back_ref = -KM_FORWORD_BACK_SPEED* FBSpeedRamp.Calc(&FBSpeedRamp);
//			else
//			{
//				ChassisSpeedRef.forward_back_ref = 0;
//				FBSpeedRamp.ResetCounter(&FBSpeedRamp);
//			}
//			if(key->v & KEY_D)  		//key: d
//				ChassisSpeedRef.left_right_ref =  KM_LEFT_RIGHT_SPEED * LRSpeedRamp.Calc(&LRSpeedRamp);
//			else if(key->v & KEY_A) 	//key: a
//				ChassisSpeedRef.left_right_ref = -KM_LEFT_RIGHT_SPEED * LRSpeedRamp.Calc(&LRSpeedRamp);
//			else
//			{
//				ChassisSpeedRef.left_right_ref = 0;
//				LRSpeedRamp.ResetCounter(&LRSpeedRamp);
//			}
			if(key->v & KEY_Z)  		//key: Z 13m/s
			{
				ShootState = 1;
				FRICL.TargetAngle = -4600;
				FRICR.TargetAngle = 4600;
				HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
			}else if(key->v & KEY_X) 	//key: X 20m/s
			{
				ShootState = 1;
				FRICL.TargetAngle = -5800;
				FRICR.TargetAngle = 5800;
				HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
			}
			else if(key->v & KEY_C) 	//key: C 28m/s
			{
				ShootState = 1;
				FRICL.TargetAngle = -7000;
				FRICR.TargetAngle = 7000;
				HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
			}
				if(ShootState && key->v & KEY_F)
		{
			ShootState = 0;
			HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_RESET);
			FRICL.TargetAngle = 0;
			FRICR.TargetAngle = 0;
		}
			if( key->v & KEY_Q)
		{
			GMY.TargetAngle+=3;
		}
				if( key->v & KEY_E)
		{
			GMY.TargetAngle-=3;
		}
		}
	}
	//Limit_and_Synchronization();
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
		AimArmor=0;//自瞄装甲板
		KeyboardMode=SHIFT;
	}
	else if(key->v & KEY_CTRL)//Ctrl
	{
		AimArmor=1;//自瞄装甲板
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

void MouseModeFSM(Mouse *mouse)
{
	static uint8_t counterl = 0;
	static uint8_t counterr = 0;
	switch (MouseLMode)
	{
		case SHORT_CLICK:
		{
			counterl++;
			if(mouse->press_l == 0)
			{
				MouseLMode = NO_CLICK;
				counterl = 0;
			}
			else if(counterl>=30)
			{
				MouseLMode = LONG_CLICK;
				counterl = 0;
			}
			else
			{
				MouseLMode = SHORT_CLICK;
			}
		}break;
		case LONG_CLICK:
		{
			if(mouse->press_l==0)
			{
				MouseLMode = NO_CLICK;
			}
			else
			{
				MouseLMode = LONG_CLICK;
			}
		}break;
		case NO_CLICK:
		{
			if(mouse->press_l)
			{
				MouseLMode = SHORT_CLICK;
				if(ShootState && fabs(STIR.TargetAngle-STIR.RealAngle)>-120.0) STIR.TargetAngle-=45;
			}
		}break;
	}
	
	switch (MouseRMode)
	{
		case SHORT_CLICK:
		{
			counterr++;
			if(mouse->press_r == 0)
			{
				MouseRMode = NO_CLICK;
				counterr = 0;
			}
			else if(counterr>=30)
			{
				MouseRMode = LONG_CLICK;
				counterr = 0;
			}
			else
			{
				MouseRMode = SHORT_CLICK;
			}
		}break;
		case LONG_CLICK:
		{
			if(mouse->press_r==0)
			{
				MouseRMode = NO_CLICK;
			}
			else
			{
				MouseRMode = LONG_CLICK;
			}
		}break;
		case NO_CLICK:
		{
			if(mouse->press_r)
			{
				MouseRMode = SHORT_CLICK;
			}
		}break;
	}
}

