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

//#define qudan
//#define jiuyuan                                                                    
//#define shangdao
Engineer_State_e EngineerState = COMMON_STATE;
Debug_State_e DebugState = DEBUG_GET_STATE;
KeyboardMode_e KeyboardMode = NO_CHANGE;
View_State_e Viewstate = NORMAL_VIEW;
MouseMode_e MouseLMode = NO_CLICK;
MouseMode_e MouseRMode = NO_CLICK;
RampGen_t LRSpeedRamp = RAMP_GEN_DAFAULT;   	//斜坡函数
RampGen_t FBSpeedRamp = RAMP_GEN_DAFAULT;
int16_t forwardRamp=0;
int16_t leftRamp = 0;
float rampRate = 0.1;
ChassisSpeed_Ref_t ChassisSpeedRef; 
#define leftstate HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_12)
#define rightstate HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_13)
#define NORMAL 0
#define REVERSE 1
#define GET 2
int16_t channelrrow = 0;
int16_t channelrcol = 0;
int16_t channellrow = 0;
int16_t channellcol = 0;
int16_t testIntensity = 0;

uint32_t dooropen=0;
uint32_t counting=0;
int32_t doorcount=0;
uint32_t  open_once=0;
uint32_t doorshake_cnt=0;
uint32_t firsttime=0;
uint32_t setdoorzero=0;
uint32_t setzerol=0;
uint32_t setzeror=0;
uint32_t lefttight=0;
uint32_t righttight=0;

uint32_t ctrl_locker=0;
uint32_t shift_locker=0;
uint32_t ctrl_cnt=0;
uint32_t shift_cnt=0;
uint32_t saving_count=0;
uint32_t saving=12;
uint32_t saveing_flag=0;

uint32_t OnePush_Locker=0;

uint32_t openthegay=0;

uint32_t Direction_Indicator=0;

void InitialSave()
{
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_0,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_RESET);
}

void Saving()
{
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_0,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_RESET);
}

void EndSaving()
{
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_0,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_SET);
}

void SavingTest()
{
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_0,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_SET);
}
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

void OptionalFunction()
{
}
void SetDoorZero()
{
	if(setdoorzero==0)
	{
	if(DOOR.RxMsgC6x0.moment<1000)
	{
		counting=0;
		DOOR.TargetAngle+=5;
	}
	if(DOOR.RxMsgC6x0.moment>=1000)
	{
	  counting=1;
	}
	if(doorcount>=1000)
	{
		DOOR.RealAngle=0;
		DOOR.TargetAngle=0;
		setdoorzero=1;
	}
  }
	if(DOOR.RxMsgC6x0.moment>3000)
		DOOR.TargetAngle-=10;
	if(DOOR.RxMsgC6x0.moment<-3000)
		DOOR.TargetAngle+=10;
}
void Door_Shake()
{
	if(dooropen==1&&setdoorzero==1&&doorshake_cnt==0)
	{
		if(DOOR.TargetAngle<-180)
			DOOR.TargetAngle+=180;
		else if(DOOR.TargetAngle>=-180)
			DOOR.TargetAngle-=180;
		doorshake_cnt=80;
	}
}
void Door_SwitchState()
{
	if(dooropen==1&&setdoorzero==1)
	{
	DOOR.TargetAngle=-180;
	if(open_once==0)
	{doorshake_cnt=500;open_once=1;}
	}
	else if(dooropen==0&&setdoorzero==1)
	{DOOR.TargetAngle=-10;open_once=0;}
	
	Door_Shake();
}

void Saving_SwitchState()
{
	if(saving==0)
	{
		EndSaving();
	}
	else if(saving==1)
	{
		Saving();
	}
	else if(saving == 2)
	{
		SavingTest();
	}
}

void Look_Normally()
{
	if(!(Viewstate==REVERSE_VIEW&&AutoClimbing==1))
	{
	YTP.TargetAngle = 60;
	if(Yaw_Reset_Flag==0)
	{
		Yaw_Reset_Flag=1;
		Yaw_Reset_Cnt=150;
	}
	Direction_Indicator=0;
	Viewstate=NORMAL_VIEW;
  }
}
void Look_Reversely()
{
	YTP.TargetAngle = 60;
	if(Yaw_Set_Flag==0)
	{
		Yaw_Set_Flag=2;
		Yaw_Set_Cnt=150;
	}
   Direction_Indicator=1;
	Viewstate=REVERSE_VIEW;
}

void Look_Screen()
{
	Direction_Indicator=2;
	YTP.TargetAngle = 60;
	if(Yaw_Set_Flag==0)
	{
		Yaw_Set_Flag=1;
		Yaw_Set_Cnt=150;
	}
}
void Limit_and_Synchronization()
{
	//demo
	//MINMAX(NMUDL.TargetAngle,-700,700);//limit
//	NMCDL.TargetAngle = NMCDR.TargetAngle;//sychronization
	UM1.TargetAngle=-UM2.TargetAngle;
	
	//demo end
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
		ComeToTop();
		
		SetDoorZero();//右纵向是开关舱门	
		Door_SwitchState();
//		if(channelrcol<-500)
//			dooropen=1;
//		if(channelrcol>500)
//			dooropen=0;
		if(1)
		{
			UM1.TargetAngle+=channelrrow*0.001;
			UM2.TargetAngle-=channelrrow*0.001;//右横向是爪子的上下移动
			NMUDL.TargetAngle+=channelrcol*0.06;
			NMUDR.TargetAngle+=channelrcol*0.06;
			
			if(channellcol>500)
				CLAWOUT;//左纵向是爪子的向前弹出
			if(channellcol<-500)                                  
				CLAWIN;
		}
		else if(DebugState==DEBUG_GET_STATE)                               //取弹模式
		{
			UM1.TargetAngle+=channelrrow*0.001;
			UM2.TargetAngle-=channelrrow*0.001;//右横向是爪子的上下移动
			
			if(channellcol>500)
				CLAWOUT;//左纵向是爪子的向前弹出
			if(channellcol<-500)                                  
				CLAWIN;
	}
	
	else if(DebugState==DEBUG_CLIMB_STATE)
	{
   if(NMCDL.RxMsgC6x0.moment>-12000&&NMCDR.RxMsgC6x0.moment>-14000&&channellcol<0)
		{
		NMCDL.TargetAngle+=channellcol*0.06;
		NMCDR.TargetAngle+=channellcol*0.06;
		}
		if(channellcol>0)
		{
		NMCDL.TargetAngle+=channellcol*0.06;
		NMCDR.TargetAngle+=channellcol*0.06;
		}
		CM1.TargetAngle+=channellrow*0.02;
		CM2.TargetAngle+=channellrow*-0.02;
		
	}
		
	}
	if(WorkState == ADDITIONAL_STATE_ONE)
	{
		ComeToTop();
		//For Debug
//		 NMCDL.TargetAngle = UD_BOTTOM;
//		NMCDR.TargetAngle = UD_BOTTOM;
		///////////////////
   if(DebugState==DEBUG_GET_STATE)
	 {		 
		//手动挡
		if(channellcol>200){       //UP  左纵向是整个机构的上下
			NMUDL.TargetAngle -= channellcol * 0.05;
			NMUDR.TargetAngle -= channellcol * 0.05;
		}	else if(channellcol<-200){		//DOWN 
			NMUDL.TargetAngle -= channellcol * 0.05;
			NMUDR.TargetAngle -= channellcol * 0.05;
		}
	  if(channelrrow>500)
		{CLAWTIGHT;}//右横向是抓紧的开关
			if(channelrrow<-500)
			{		CLAWLOOSE;}
			
			if(channelrcol>500)
				LAUNCH;//右纵向是弹药箱弹出的开关
			if(channelrcol<-500)
				LAND;

			UFM.TargetAngle-=channellrow*0.01;//左横向是水平电机   向左远离（角度++）向右靠近（角度--）
	  }
	  else if(DebugState==DEBUG_CLIMB_STATE)
		{
		ChassisSpeedRef.forward_back_ref = channelrcol * 2*RC_CHASSIS_SPEED_REF;
		ChassisSpeedRef.left_right_ref   = channelrrow * RC_CHASSIS_SPEED_REF;
		ChassisSpeedRef.rotate_ref = -channellrow * RC_ROTATE_SPEED_REF;
		}
			
			
}
	if(WorkState == ADDITIONAL_STATE_TWO)
	{
		//****************自动取弹程序//UM1--是拔出来UM2相反  最大120//大的PH2 小的PH4***************
/****************************自检**********************************/
			//*****调试数据UFM.RxMsgC6x0.moment   >5000 在远端卡住 <-5000 在近端卡住  + 往远端移动  - 近端移动 
			//**targetAngle 总行程830左右
			//360°共11个齿 每个齿12.7mm 
			//最靠近电机moment是负的 realangle=0,中间是410左右 最远端700左右
			//比较健康的moment是3000    靠近电机-3000 远离电机3000 
			//NMUDL840 NMUDR-840
			//红外2000 3000
		
		//For Debug
//		 NMCDL.TargetAngle = UD_TOP;
//		NMCDR.TargetAngle = UD_TOP;
		///////////////////
		
		
			ChassisSpeedRef.forward_back_ref = channelrcol * RC_CHASSIS_SPEED_REF;
		  ChassisSpeedRef.left_right_ref   = channelrrow * RC_CHASSIS_SPEED_REF/2;
			ChassisSpeedRef.rotate_ref = -channellrow * RC_ROTATE_SPEED_REF;
			if(channellcol>500)     //左下救援，上松爪子
				saving=0;
			if(channellcol<-500)
				saving=1;
 	      Saving_SwitchState();
			
			
			ComeToTop();
			if(DebugState==DEBUG_CLIMB_STATE)
			{
			Chassis_Choose(1,1);
      }				
	}
	Limit_and_Synchronization();
}


uint16_t KM_FORWORD_BACK_SPEED 	= NORMAL_FORWARD_BACK_SPEED;
uint16_t KM_LEFT_RIGHT_SPEED  	= NORMAL_LEFT_RIGHT_SPEED;
void MouseModeFSM(Mouse *mouse);
void KeyboardModeFSM(Key *key);

//****************
//键鼠模式功能编写
//****************
void MouseKeyControlProcess(Mouse *mouse, Key *key)
{	
	if(WorkState <= 0) return;
	
	MINMAX(mouse->x, -150, 150); 
	MINMAX(mouse->y, -150, 150); 
	
	#ifdef USE_CHASSIS_FOLLOW

  if(AutoClimbing==0||(AutoClimbing==1&&AlreadyDowned==0))
  {
	  if(EngineerState==COMMON_STATE||EngineerState==CLIMB_STATE)
		{
			if(saving==0||saving==12)
				ChassisSpeedRef.rotate_ref = mouse->x * MOUSE_TO_YAW_ANGLE_INC_FACT*-20;
			else if(saving==1)
				ChassisSpeedRef.rotate_ref = mouse->x * MOUSE_TO_YAW_ANGLE_INC_FACT*-20;	
			else if(saving==2)
				ChassisSpeedRef.rotate_ref = mouse->x * MOUSE_TO_YAW_ANGLE_INC_FACT*-175;	
		}
	  else if(EngineerState==GET_STATE)
		  ChassisSpeedRef.rotate_ref = mouse->x * MOUSE_TO_YAW_ANGLE_INC_FACT*20;
  }
	
	if((EngineerState!=COMMON_STATE||saving==1)||ON_THE_FLOOR)
	{
		if(YTP.RxMsgC6x0.moment<1500&&(mouse->y * MOUSE_TO_PITCH_ANGLE_INC_FACT)>0)
		YTP.TargetAngle += mouse->y * MOUSE_TO_PITCH_ANGLE_INC_FACT*5;
		if(YTP.RxMsgC6x0.moment>-1500&&(mouse->y * MOUSE_TO_PITCH_ANGLE_INC_FACT)<0)
		YTP.TargetAngle += mouse->y * MOUSE_TO_PITCH_ANGLE_INC_FACT*5;
  }

	#else
	ChassisSpeedRef.rotate_ref = mouse->x * RC_ROTATE_SPEED_REF;
	#endif
	
	
	MouseModeFSM(mouse);
	
	switch(MouseRMode)
	{
		case SHORT_CLICK:
		{
			if(ON_THE_FLOOR&&EngineerState==GET_STATE)
			{
				AutoGet_Skill=1;
			}
		  ChassisSpeedRef.rotate_ref = 0;
			if(YTY.TargetAngle<90&&mouse->x * MOUSE_TO_YAW_ANGLE_INC_FACT*-3>0)
			YTY.TargetAngle -= mouse->x * MOUSE_TO_YAW_ANGLE_INC_FACT*3;
			if(YTY.TargetAngle>-180&&mouse->x * MOUSE_TO_YAW_ANGLE_INC_FACT*-3<0)
			YTY.TargetAngle -= mouse->x * MOUSE_TO_YAW_ANGLE_INC_FACT*3;
      if(YTP.RxMsgC6x0.moment<1500&&(mouse->y * MOUSE_TO_PITCH_ANGLE_INC_FACT)>0)
	    YTP.TargetAngle += mouse->y * MOUSE_TO_PITCH_ANGLE_INC_FACT*5;
	    if(YTP.RxMsgC6x0.moment>-1500&&(mouse->y * MOUSE_TO_PITCH_ANGLE_INC_FACT)<0)
	    YTP.TargetAngle += mouse->y * MOUSE_TO_PITCH_ANGLE_INC_FACT*5;			
		}break;
		case LONG_CLICK:
		{
			ChassisSpeedRef.rotate_ref = 0;
			if(YTY.TargetAngle<90&&mouse->x * MOUSE_TO_YAW_ANGLE_INC_FACT*-3>0)
			YTY.TargetAngle -= mouse->x * MOUSE_TO_YAW_ANGLE_INC_FACT*3;
			if(YTY.TargetAngle>-180&&mouse->x * MOUSE_TO_YAW_ANGLE_INC_FACT*-3<0)
			YTY.TargetAngle -= mouse->x * MOUSE_TO_YAW_ANGLE_INC_FACT*3;	
			if(YTP.RxMsgC6x0.moment<1500&&(mouse->y * MOUSE_TO_PITCH_ANGLE_INC_FACT)>0)
	    YTP.TargetAngle += mouse->y * MOUSE_TO_PITCH_ANGLE_INC_FACT*5;
	    if(YTP.RxMsgC6x0.moment>-1500&&(mouse->y * MOUSE_TO_PITCH_ANGLE_INC_FACT)<0)
	    YTP.TargetAngle += mouse->y * MOUSE_TO_PITCH_ANGLE_INC_FACT*5;	
		}break;
		default: break;
	}
	
	switch (MouseLMode)
	{
		case SHORT_CLICK:
		{
			Look_Normally();
		}break;
		case LONG_CLICK:
		{
			Look_Reversely();
		}
		default: break;
	}

	KeyboardModeFSM(key);//下面是移动的控制 在写命令时不要用wasd键
		if(key->v & KEY_W)  		//key: w
		{
			if (Direction_Indicator==NORMAL)
			{
				//ChassisSpeedRef.forward_back_ref =  KM_FORWORD_BACK_SPEED* FBSpeedRamp.Calc(&FBSpeedRamp);
				forwardRamp += rampRate*(KM_FORWORD_BACK_SPEED * FBSpeedRamp.Calc(&FBSpeedRamp) - forwardRamp);
				ChassisSpeedRef.forward_back_ref = forwardRamp;
			}
			if (Direction_Indicator==REVERSE)
			{
				//ChassisSpeedRef.forward_back_ref =  -KM_FORWORD_BACK_SPEED* FBSpeedRamp.Calc(&FBSpeedRamp);
				forwardRamp += rampRate*(-KM_FORWORD_BACK_SPEED * FBSpeedRamp.Calc(&FBSpeedRamp) - forwardRamp);
				ChassisSpeedRef.forward_back_ref = forwardRamp;
			}
			if(Direction_Indicator==GET)
			{
				ChassisSpeedRef.left_right_ref =  -KM_FORWORD_BACK_SPEED* FBSpeedRamp.Calc(&FBSpeedRamp)/2;
			}
		}
			else if(key->v & KEY_S) 	//key: s
			{
				if(Direction_Indicator==NORMAL)
				{
					//ChassisSpeedRef.forward_back_ref = -KM_FORWORD_BACK_SPEED* FBSpeedRamp.Calc(&FBSpeedRamp);
					forwardRamp += rampRate*(-KM_FORWORD_BACK_SPEED * FBSpeedRamp.Calc(&FBSpeedRamp) - forwardRamp);
					ChassisSpeedRef.forward_back_ref = forwardRamp;
				}
				if(Direction_Indicator==REVERSE)
				{
					//ChassisSpeedRef.forward_back_ref = KM_FORWORD_BACK_SPEED* FBSpeedRamp.Calc(&FBSpeedRamp);
					forwardRamp += rampRate*(KM_FORWORD_BACK_SPEED * FBSpeedRamp.Calc(&FBSpeedRamp) - forwardRamp);
					ChassisSpeedRef.forward_back_ref = forwardRamp;
				}
				if(Direction_Indicator==GET)
				{
					ChassisSpeedRef.left_right_ref =  KM_FORWORD_BACK_SPEED* FBSpeedRamp.Calc(&FBSpeedRamp)/2;
				}
			}
			else
			{
				if(Direction_Indicator==NORMAL||Direction_Indicator==REVERSE)
				{
					//ChassisSpeedRef.forward_back_ref = 0;
					forwardRamp += rampRate*(0 - forwardRamp);
					ChassisSpeedRef.forward_back_ref = forwardRamp;
				}
				else if(Direction_Indicator==GET)
				{
					ChassisSpeedRef.left_right_ref = 0;
				}
				FBSpeedRamp.ResetCounter(&FBSpeedRamp);
			}
			if(key->v & KEY_D)  		//key: d
			{	
				if(Direction_Indicator==NORMAL)
				{
					ChassisSpeedRef.left_right_ref =  KM_LEFT_RIGHT_SPEED * LRSpeedRamp.Calc(&LRSpeedRamp);
				}
				if(Direction_Indicator==REVERSE)
				{
					ChassisSpeedRef.left_right_ref =  -KM_LEFT_RIGHT_SPEED * LRSpeedRamp.Calc(&LRSpeedRamp);
				}
				if(Direction_Indicator==GET)
				{
					ChassisSpeedRef.left_right_ref = -KM_LEFT_RIGHT_SPEED* LRSpeedRamp.Calc(&LRSpeedRamp)/12;
					ChassisSpeedRef.forward_back_ref =  -KM_LEFT_RIGHT_SPEED* LRSpeedRamp.Calc(&LRSpeedRamp);
				}
			}
			else if(key->v & KEY_A) 	//key: a
			{	
				if(Direction_Indicator==NORMAL)
				{
					ChassisSpeedRef.left_right_ref = -KM_LEFT_RIGHT_SPEED * LRSpeedRamp.Calc(&LRSpeedRamp);
				}
				if(Direction_Indicator==REVERSE)
				{
					ChassisSpeedRef.left_right_ref = KM_LEFT_RIGHT_SPEED * LRSpeedRamp.Calc(&LRSpeedRamp);
				}
				if(Direction_Indicator==GET)
				{
					ChassisSpeedRef.left_right_ref = -KM_LEFT_RIGHT_SPEED* LRSpeedRamp.Calc(&LRSpeedRamp)/12;
					ChassisSpeedRef.forward_back_ref =  KM_LEFT_RIGHT_SPEED* LRSpeedRamp.Calc(&LRSpeedRamp)/2;
				}
			}
			else
			{
				if(Direction_Indicator==NORMAL||Direction_Indicator==REVERSE)
				{
					ChassisSpeedRef.left_right_ref = 0;
				}
				else if(Direction_Indicator==GET)
				{
					ChassisSpeedRef.forward_back_ref = 0;
				}
				LRSpeedRamp.ResetCounter(&LRSpeedRamp);
			}
			
	switch (KeyboardMode)
	{
		case SHIFT_CTRL:		//State control
		{
			if(key->v & KEY_F)
			{
				NMCDL.TargetAngle = UD_BOTTOM;
				NMCDR.TargetAngle = UD_BOTTOM;
				AlreadyClimbed=0;
				AlreadyDowned=0;
				State_Common();
			}
			else if(key->v & KEY_G)
			{
				NMCDL.TargetAngle = UD_TOP;
				NMCDR.TargetAngle = UD_TOP;
				AlreadyClimbed=0;
				AlreadyDowned=0;
				State_Common();
			}
			else if(key->v & KEY_Q)							
			{
				AutoClimb_Level=0;
			}
			else if(key->v & KEY_E)
			{
				AutoClimb_Level=2;
			}
			else if(key->v & KEY_X)							
			{
				YTY.RealAngle=0;
				YTP.RealAngle=0;
				YTY.TargetAngle=0;
				YTP.TargetAngle=0;
			}
			else if(key->v & KEY_V)
			{
				DebugState=DEBUG_GET_STATE;
			}
      else if(key->v & KEY_B)
			{
				DebugState=DEBUG_CLIMB_STATE;
			}				
			break;
		}
		case CTRL:				//slow
		{
			ctrl_locker=1;
			if(key->v & KEY_C)
			{
				if(EngineerState==COMMON_STATE)
				State_AutoClimb();
			}
			else if(key->v & KEY_V)
			{
				//State_Common();
			}
			else if(key->v & KEY_Q)
			{
				if(EngineerState==GET_STATE)
				if(CLAW_IS_UP)
				CLAWOUT;
			}
			else if(key->v & KEY_E)
			{ 
				if(EngineerState==GET_STATE)
				if(CLAW_IS_UP)
			  CLAWIN;
			}
			else if(key->v & KEY_B)
			{
				if(EngineerState==GET_STATE)
				if(CLAW_INSPECT_SUCCEED&&CLAW_IS_UP)
				AutoGet_Start=3;
			}
			else if(key->v & KEY_G)
			{
				if(Claw_UpToPosition==0)
		      Claw_UpToPosition=1;
			}
			else if(key->v & KEY_F)
			{
				dooropen=0;
			}
			else if(key->v & KEY_R)
			{
				saving = 2;
			}
			
		}break;
		case SHIFT:				//quick
		{
			shift_locker=1;
			if(key->v & KEY_Q)
			{
				Claw_SelfInspecting=1;
			}
			else if(key->v & KEY_C&&Claw_UpToPosition==0)
			{
				if(EngineerState==COMMON_STATE)
				State_AutoGet();
				if(EngineerState==GET_STATE)
				Look_Screen();
			}
			else if(key->v &KEY_V)
			{
				State_Common();
			}
			else if(key->v &KEY_R)
			{
				saving=1;
			}
			else if(key->v &KEY_F)
			{
				if((ON_THE_GROUND&&CLAW_IS_UP)||(ON_THE_FLOOR&&CLAW_IS_DOWN))
				{
				dooropen=1;
				AutoGet_Bullet_S=0;
				AutoGet_Bullet_B=0;
				}
			}
			else if(key->v& KEY_Z)
			{
					if(EngineerState==GET_STATE)
				{
					AutoGet_FillQueue();
				}
				
			}
		}break;
		case NO_CHANGE:			//normal
		{//CM Movement Process
		ctrl_locker=0;
		shift_locker=0;
			if(ctrl_cnt==0&&shift_cnt==0)
		{
			if(key->v & KEY_X)
			{ 
				AutoGet_Stop_And_Clear();
			}
			else if(key->v & KEY_C)
			{
				AutoGet_Enqueue(3);
			}
			else if(key->v & KEY_V)
			{
				AutoGet_Enqueue(2);
			}
			else if(key->v & KEY_B)
			{
				AutoGet_Enqueue(1);
			}
			else if(key->v & KEY_F)
			{
				if(ON_THE_GROUND)
				AutoGet_Enqueue(5);
				else if(ON_THE_FLOOR)
				AutoGet_Enqueue(6);
			}
			else if(key->v & KEY_G)
			{
				if(ON_THE_GROUND)
				AutoGet_Enqueue(4);
				else if(ON_THE_FLOOR)
				AutoGet_Enqueue(6);
			}
			else if(key->v & KEY_Z)
			{
				dooropen=0;
				Claw_DownToPosition=1;
				State_Common();
			}
			else if(key->v & KEY_Q)
			{
				if(EngineerState==GET_STATE)
				{
					Sensor_LongPush++;
					if(CLAW_INSPECT_SUCCEED&&CLAW_IS_UP&ON_THE_GROUND)
						Claw_FindingNextBox_Lower_Forward=1;
					else if(CLAW_INSPECT_SUCCEED&&CLAW_IS_UP&&ON_THE_FLOOR)
						Claw_FindingNextBox_Upper_Forward=1;
					if(Sensor_LongPush>=50)
						Sensor_Lock=1;
				}
				else if(EngineerState==COMMON_STATE&&ON_THE_GROUND)
				{
					if(saving==1)
						ChassisSpeedRef.rotate_ref = 150 * MOUSE_TO_YAW_ANGLE_INC_FACT;
					else
						ChassisSpeedRef.rotate_ref = 150 * MOUSE_TO_YAW_ANGLE_INC_FACT*20;
				}
			}
			else if(key->v & KEY_E)
			{
				if(EngineerState==GET_STATE)
				{
					Sensor_LongPush++;
				if(CLAW_INSPECT_SUCCEED&&CLAW_IS_UP&&ON_THE_GROUND)
				Claw_FindingNextBox_Lower_Backward=1;
				else if(CLAW_INSPECT_SUCCEED&&CLAW_IS_UP&&ON_THE_FLOOR)
				Claw_FindingNextBox_Upper_Backward=1;
				if(Sensor_LongPush>=50)
						Sensor_Lock=1;
			  }
				else if(EngineerState==COMMON_STATE&&ON_THE_GROUND)
				{
					if(saving==1)
						ChassisSpeedRef.rotate_ref = -150 * MOUSE_TO_YAW_ANGLE_INC_FACT;
					else
						ChassisSpeedRef.rotate_ref = -150 * MOUSE_TO_YAW_ANGLE_INC_FACT*20;
				}
			}
			else if(key->v & KEY_R)
			{
				if(EngineerState==COMMON_STATE)
				saving=0;
				else if(EngineerState==GET_STATE&&ON_THE_FLOOR)
				Claw_Wait();
			}
			else
			{
				OnePush_Locker=0;
				Sensor_LongPush=0;
				if(Sensor_LongPush<50)
					Sensor_Lock=0;
			}
		}
			
		}
		SetDoorZero();
		Door_SwitchState();
		ComeToTop();
		Claw_GetSpecifiedBox();
		Claw_SelfInspect();
		Claw_AutoIn();
		AutoGet_SensorControl();
		Claw_Protect();
		Claw_AutoBack();
		Box_Land();
		//AutoGet_SwitchState();
		AutoGet_AutoDown();
		AutoClimb_SwitchState();
		ClawUpDown_SwitchState();
		ClawUpDown_Protect();
		Saving_SwitchState();
		Yaw_Check();
		Rotate_Check();
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
			else if(counterl>=20)
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
			else if(counterr>=20)
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

