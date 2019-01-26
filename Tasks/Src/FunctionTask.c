

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
MouseMode_e MouseLMode = NO_CLICK;
MouseMode_e MouseRMode = NO_CLICK;
RampGen_t LRSpeedRamp = RAMP_GEN_DAFAULT;   	//б�º���
RampGen_t FBSpeedRamp = RAMP_GEN_DAFAULT;
ChassisSpeed_Ref_t ChassisSpeedRef; 
int8_t gjl = 0;
//int16_t flagzc = 0;
int16_t rl1;
int16_t rl2;
int16_t rl3;
int16_t rl4;

int ChassisTwistGapAngle = 0;

int32_t auto_counter=0;		//����׼ȷ��ʱ�����ĳ�¼�

int16_t channelrrow = 0;
int16_t channelrcol = 0;
int16_t channellrow = 0;
int16_t channellcol = 0;
int16_t testIntensity = 0;
uint8_t SuperCTestMode = 0;
uint8_t ShootState = 0;
uint8_t ChassisTwistState = 0;

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
//	PowerLimitation();
}

void Limit_and_Synchronization()
{
	//demo
	//MINMAX(UD1.TargetAngle,-900,270);//limit
	//UD2.TargetAngle=-UD1.TargetAngle;//sychronization
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
		//for recharge SuperC
		Control_SuperCap.release_power = 0;
		Control_SuperCap.stop_power = 0;
		
		
			ChassisSpeedRef.forward_back_ref = channelrcol * RC_CHASSIS_SPEED_REF;
			ChassisSpeedRef.left_right_ref   = channelrrow * RC_CHASSIS_SPEED_REF/2;
		#ifdef USE_CHASSIS_FOLLOW
		GMY.TargetAngle += channellrow * RC_GIMBAL_SPEED_REF;
		GMP.TargetAngle += channellcol * RC_GIMBAL_SPEED_REF;
		#else
			ChassisSpeedRef.rotate_ref = channellrow * RC_ROTATE_SPEED_REF * 0.4f;
		#endif

//		unused
		
//		#ifdef USE_AUTOAIM
//		autoAimGMCTRL();
//		#endif /*USE_AUTOAIM*/
		
//		ChassisTwistState = 0;
//		
//		ShootState = 0;
//		FRICL.TargetAngle = 0;
//		FRICR.TargetAngle = 0;
		HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);

  
// Rescue Function
  if (channellcol<-200)
	{
	  RQ1.TargetAngle = 230;
		RQ2.TargetAngle = -230;
	
	}
	if (channellcol>200)
	{
	  RQ1.TargetAngle = 0;
		RQ2.TargetAngle = 0;
	}
}
	if(WorkState == ADDITIONAL_STATE_ONE)
	{
		//for debug SuperC
		
		
		if(SuperCTestMode==1)
		{
			Control_SuperCap.release_power = 1;
			Control_SuperCap.stop_power = 0;
		}
		else
		{
			Control_SuperCap.stop_power = 1;
		}
		if(Control_SuperCap.C_voltage>1200 && SuperCTestMode==1)
		{
			ChassisSpeedRef.forward_back_ref = channelrcol * RC_CHASSIS_SPEED_REF * 2;
			ChassisSpeedRef.left_right_ref   = channelrrow * RC_CHASSIS_SPEED_REF / 2 * 2;
			ChassisSpeedRef.rotate_ref = channellrow * RC_ROTATE_SPEED_REF * 0.4f;
		}
		else if (SuperCTestMode==1)
		{
			ChassisSpeedRef.forward_back_ref = channelrcol * RC_CHASSIS_SPEED_REF;
			ChassisSpeedRef.left_right_ref   = channelrrow * RC_CHASSIS_SPEED_REF / 2;
			ChassisSpeedRef.rotate_ref = channellrow * RC_ROTATE_SPEED_REF * 0.4f;
		}
//		#ifdef USE_CHASSIS_FOLLOW
//			GMY.TargetAngle += channellrow * RC_GIMBAL_SPEED_REF;
//			GMP.TargetAngle += channellcol * RC_GIMBAL_SPEED_REF;
//		#else
//			ChassisSpeedRef.rotate_ref = channellrow * RC_ROTATE_SPEED_REF;
//		#endif
		
//		ChassisTwistState = 0;

//		RedLine Mode
		else
		{
			rl4=HAL_GPIO_ReadPin(GPIOH,GPIO_PIN_10);
	    rl3=HAL_GPIO_ReadPin(GPIOH,GPIO_PIN_11);
			rl2=HAL_GPIO_ReadPin(GPIOH,GPIO_PIN_12);
			if(rl2 == 0&&rl3 == 1&&rl4 == 0)
			{
				ChassisSpeedRef.forward_back_ref = 80;
				ChassisSpeedRef.rotate_ref  = 0;
			}
			if((rl2 == 0&&rl3 == 0&&rl4 == 1)||(rl2 == 0&&rl3 == 1&&rl4 == 1))
			{
				ChassisSpeedRef.rotate_ref = -120;
				ChassisSpeedRef.forward_back_ref = 80;
			}
			if((rl2 == 1&&rl3 == 0&&rl4 == 0)||(rl2 == 1&&rl3 == 1&&rl4 == 0))
			{
				ChassisSpeedRef.rotate_ref = 120;
				ChassisSpeedRef.forward_back_ref = 80;
			}
			
//
//			ShootState = 1;
//			FRICL.TargetAngle = 5000;
//			FRICR.TargetAngle = -5000;
//		}
//		else
//		{
//			ShootState = 0;
//			FRICL.TargetAngle = 0;
//			FRICR.TargetAngle = 0;
//		}
		
			HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
		}
	}
	if(WorkState == ADDITIONAL_STATE_TWO)
	{
		//for using SuperC
		Control_SuperCap.stop_power = 0;
		Control_SuperCap.release_power = 1;
		ChassisSpeedRef.forward_back_ref = channelrcol * RC_CHASSIS_SPEED_REF;
		ChassisSpeedRef.left_right_ref   = channelrrow * RC_CHASSIS_SPEED_REF/2;
		#ifdef USE_CHASSIS_FOLLOW
		GMY.TargetAngle += channellrow * RC_GIMBAL_SPEED_REF;
		GMP.TargetAngle += channellcol * RC_GIMBAL_SPEED_REF;
		#else
		ChassisSpeedRef.rotate_ref = channellrow * RC_ROTATE_SPEED_REF * 0.4f;
		#endif
//		Unused Codes
//		ChassisTwistState = 0;
		
//		if(SuperCTestMode==0)
//		{
//			ShootState = 1;
//			FRICL.TargetAngle = 5000;
//			FRICR.TargetAngle = -5000;
//			Delay(20,{STIR.TargetAngle-=60;});
//		}
//		else
//		{
//			ShootState = 0;
//			FRICL.TargetAngle = 0;
//			FRICR.TargetAngle = 0;
//			ChassisTwistState = 1;
//		}
//		HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
		
	}
	

	FreshSuperCState();

//	if(ChassisTwistState)
//	{
//		LJHTwist();
//	}
//	else ChassisDeTwist();
//	//Limit_and_Synchronization();
} 

	

uint16_t KM_FORWORD_BACK_SPEED 	= NORMAL_FORWARD_BACK_SPEED;
uint16_t KM_LEFT_RIGHT_SPEED  	= NORMAL_LEFT_RIGHT_SPEED;
void KeyboardModeFSM(Key *key);
void MouseModeFSM(Mouse *mouse);


//****************
//����ģʽ���ܱ�д
//****************
void MouseKeyControlProcess(Mouse *mouse, Key *key)
{	
	if(WorkState <= 0) return;
	
	MINMAX(mouse->x, -150, 150); 
	MINMAX(mouse->y, -150, 150); 
	
//	YT1.TargetAngle -= mouse->x * MOUSE_TO_YAW_ANGLE_INC_FACT;
	YT2.TargetAngle += mouse->y * MOUSE_TO_PITCH_ANGLE_INC_FACT;


	#ifdef USE_CHASSIS_FOLLOW
	GMY.TargetAngle += mouse->x * MOUSE_TO_YAW_ANGLE_INC_FACT;
	GMP.TargetAngle -= mouse->y * MOUSE_TO_PITCH_ANGLE_INC_FACT;
	#else
	ChassisSpeedRef.rotate_ref = mouse->x * RC_ROTATE_SPEED_REF;
	#endif
	
	MouseModeFSM(mouse);
	
	switch(MouseRMode)
	{
		case SHORT_CLICK:
		{
			ShootState = 1;
			HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
			FRICL.TargetAngle = 5000;
			FRICR.TargetAngle = -5000;
		}break;
		case LONG_CLICK:
		{
			if(ShootState)
			{
				ShootState = 0;
				HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_RESET);
				FRICL.TargetAngle = 0;
				FRICR.TargetAngle = 0;
			}
		}break;
		default: break;
	}
	
	switch (MouseLMode)
	{
		case SHORT_CLICK:
		{
			//if(ShootState) Delay(20,{STIR.TargetAngle-=60;});
		}break;
		case LONG_CLICK:
		{
			if(ShootState)
			{
				
			}
		}
		default: break;
	}
	
	Control_SuperCap.release_power = 0;
	Control_SuperCap.stop_power = 0;

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
			{
				if (gjl == 0)
				{
					ChassisSpeedRef.forward_back_ref = KM_FORWORD_BACK_SPEED* FBSpeedRamp.Calc(&FBSpeedRamp);
				}
		
				if (gjl == 1)
				{
					ChassisSpeedRef.forward_back_ref = -KM_FORWORD_BACK_SPEED* FBSpeedRamp.Calc(&FBSpeedRamp);
				}
			}
			else if(key->v & KEY_S) 	//key: s
			{	
				if (gjl == 0 )
				{
					ChassisSpeedRef.forward_back_ref = -KM_FORWORD_BACK_SPEED* FBSpeedRamp.Calc(&FBSpeedRamp);
				}
				if (gjl == 1)
				{
					ChassisSpeedRef.forward_back_ref = KM_FORWORD_BACK_SPEED* FBSpeedRamp.Calc(&FBSpeedRamp);
				}
			}
			else
			{
				ChassisSpeedRef.forward_back_ref = 0;
				FBSpeedRamp.ResetCounter(&FBSpeedRamp);
			}
			if(key->v & KEY_D)  		//key: d
			{
				if (gjl == 0 )
				{
					ChassisSpeedRef.left_right_ref =  KM_LEFT_RIGHT_SPEED * LRSpeedRamp.Calc(&LRSpeedRamp);
				}
				if (gjl == 1)
				{
					ChassisSpeedRef.left_right_ref =  -KM_LEFT_RIGHT_SPEED * LRSpeedRamp.Calc(&LRSpeedRamp);
				}
			}
			else if(key->v & KEY_A) 	//key: a
			{
				if (gjl == 0 )
				{
					ChassisSpeedRef.left_right_ref = - KM_LEFT_RIGHT_SPEED * LRSpeedRamp.Calc(&LRSpeedRamp);
				}
				if (gjl == 1)
				{
					ChassisSpeedRef.left_right_ref =  KM_LEFT_RIGHT_SPEED * LRSpeedRamp.Calc(&LRSpeedRamp);
				}			
			}
			else
			{
				ChassisSpeedRef.left_right_ref = 0;
				LRSpeedRamp.ResetCounter(&LRSpeedRamp);
			}
			if (key->v & KEY_Q)      //key: q
			{	
				gjl = 1;
				YT1.TargetAngle = 180;
				YT2.TargetAngle = 0 ;
			}
			if (key->v & KEY_E)      //key: e
			{
				gjl = 0;
				YT1.TargetAngle = 0;
				YT2.TargetAngle = 0;
			}
			if (key->v & KEY_Z)
			{
				RQ1.TargetAngle = 230;
				RQ2.TargetAngle = -230;
			}
			if (key->v & KEY_X)
			{
				RQ1.TargetAngle = 0;
				RQ2.TargetAngle = 0;
			}
			if (KEY_V & KEY_C)
			{
				RQ1.TargetAngle = -230;
				RQ2.TargetAngle = 230;
			}
			

		}
	}
//	Limit_and_Synchronization();
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
			else if(counterl>=50)
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
				if(ShootState && abs(STIR.TargetAngle-STIR.RealAngle)<5.0) STIR.TargetAngle-=60;
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
			else if(counterr>=50)
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


//����ң����ģʽ�³������ݲ���ģʽ�Ŀ���
void FreshSuperCState(void)
{
	static uint8_t counter = 0;
	if(HAL_GPIO_ReadPin(BUTTON_GPIO_Port,BUTTON_Pin))
	{
		counter++;
		if(counter==40)
		{
			SuperCTestMode = (SuperCTestMode==1)?0:1;
		}
	}
	else
	{
		counter = 0;
	}
	if(SuperCTestMode==1)
	{
		HAL_GPIO_WritePin(GPIOF, LED_GREEN_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE, LED_RED_Pin, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOF, LED_GREEN_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, LED_RED_Pin, GPIO_PIN_SET);
	}
	HAL_GPIO_WritePin(GPIOG, LED8_Pin|LED7_Pin|LED6_Pin 
                          |LED5_Pin|LED4_Pin|LED3_Pin|LED2_Pin 
                          |LED1_Pin, GPIO_PIN_RESET);
	if(Control_SuperCap.C_voltage<1100)
	{
		HAL_GPIO_WritePin(GPIOG, LED8_Pin|LED7_Pin|LED6_Pin|LED5_Pin|LED4_Pin|LED3_Pin|LED2_Pin|LED1_Pin, GPIO_PIN_SET);
	}
	else if(Control_SuperCap.C_voltage<1300)
		HAL_GPIO_WritePin(GPIOG, LED8_Pin|LED7_Pin|LED6_Pin|LED5_Pin|LED4_Pin|LED3_Pin|LED2_Pin, GPIO_PIN_SET);
	else if(Control_SuperCap.C_voltage<1500)
		HAL_GPIO_WritePin(GPIOG, LED8_Pin|LED7_Pin|LED6_Pin|LED5_Pin|LED4_Pin|LED3_Pin, GPIO_PIN_SET);
	else if(Control_SuperCap.C_voltage<1700)
		HAL_GPIO_WritePin(GPIOG, LED8_Pin|LED7_Pin|LED6_Pin|LED5_Pin|LED4_Pin, GPIO_PIN_SET);
	else if(Control_SuperCap.C_voltage<1800)
		HAL_GPIO_WritePin(GPIOG, LED8_Pin|LED7_Pin|LED6_Pin|LED5_Pin, GPIO_PIN_SET);
	else if(Control_SuperCap.C_voltage<1900)
		HAL_GPIO_WritePin(GPIOG, LED8_Pin|LED7_Pin|LED6_Pin, GPIO_PIN_SET);
	else if(Control_SuperCap.C_voltage<2000)
		HAL_GPIO_WritePin(GPIOG, LED8_Pin|LED7_Pin, GPIO_PIN_SET);
	else if(Control_SuperCap.C_voltage<2100)
		HAL_GPIO_WritePin(GPIOG, LED8_Pin, GPIO_PIN_SET);
}

void ChassisTwist(void)
{
	switch (ChassisTwistGapAngle)
	{
		case 0:
		{ChassisTwistGapAngle = CHASSIS_TWIST_ANGLE_LIMIT;}break;
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

