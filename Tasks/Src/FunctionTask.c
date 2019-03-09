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
#define leftstate HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_12)
#define rightstate HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_13)
int16_t channelrrow = 0;
int16_t channelrcol = 0;
int16_t channellrow = 0;
int16_t channellcol = 0;
int16_t testIntensity = 0;

int setzerol=0;
int setzeror=0;
int lefttight=0;
int righttight=0;
int teeet=0;
extern uint32_t AutoClimb_ComeToTop;

int openthegay=0;


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
void setzero()
{
	if(setzerol==0)
	{	
	  if(NMUDL.RxMsgC6x0.moment<3000)
			NMUDL.TargetAngle+=10;
		else
		{
			NMUDL.RealAngle=0;
			NMUDL.TargetAngle=0;
			setzerol=1;
		}
	}
	if(setzeror==0)
	{	
	  if(NMUDR.RxMsgC6x0.moment>-3000)
			NMUDR.TargetAngle-=10;
		else
		{
			NMUDR.RealAngle=0;
			NMUDR.TargetAngle=0;
			setzeror=1;
		}
	}
}
void Limit_and_Synchronization()
{
	//demo
	//MINMAX(NMUDL.TargetAngle,-700,700);//limit
	NMCDL.TargetAngle = NMCDR.TargetAngle;//sychronization
	UM1.TargetAngle=-UM2.TargetAngle;
	
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
		//ChassisSpeedRef.rotate_ref = -channellrow * RC_ROTATE_SPEED_REF;
		Sensor_a=adgl;
		Sensor_b=adgr;
		
		
		//�ֶ�������צ��
		if(channellcol>500)
		CLAWOUT;//��������צ�ӵ���ǰ����
		if(channellcol<-500)                                  //          *********����ʱ��ʱ�ر�*************
		CLAWIN;
		
		UM1.TargetAngle+=channellrow*0.001;
		UM2.TargetAngle-=channellrow*0.001;//�������צ�ӵ������ƶ�
		
   /* if(NMCDL.RxMsgC6x0.moment>-12000&&NMCDR.RxMsgC6x0.moment>-14000&&channellcol<0)
		{
		NMCDL.TargetAngle+=channellcol*0.06;
		NMCDR.TargetAngle+=channellcol*0.06;
		}
		if(channellcol>0)
		{
		NMCDL.TargetAngle+=channellcol*0.06;
		NMCDR.TargetAngle+=channellcol*0.06;
		}
		
		CM1.TargetAngle+=channellrow*0.01;
		CM2.TargetAngle+=channellrow*-0.01;
		*/
		
	}
	if(WorkState == ADDITIONAL_STATE_ONE)
	{

		//�ֶ���
		if(channellcol>200){       //UP  ����������������������
			NMUDL.TargetAngle -= channellcol * 0.05;
			NMUDR.TargetAngle -= channellcol * 0.05;
		}	else if(channellcol<-200){		//DOWN 
			NMUDL.TargetAngle -= channellcol * 0.05;
			NMUDR.TargetAngle -= channellcol * 0.05;
		}
	  if(channelrrow>500)
		{CLAWTIGHT;teeet=1;}//�Һ�����ץ���Ŀ���
			if(channelrrow<-500)
			{		CLAWLOOSE;teeet=2;}
			
			if(channelrcol>500)
				LAUNCH;//�������ǵ�ҩ�䵯���Ŀ���
			if(channelrcol<-500)
				LAND;

			UFM.TargetAngle-=channellrow*0.01;//�������ˮƽ���   ����Զ�루�Ƕ�++�����ҿ������Ƕ�--��
     
			/*if(channelrrow>500)
				AutoClimb_ComeToTop=1;
			if(channelrrow<-500)
				AutoClimb_ComeToTop=0;
			
			ComeToTop();*/
			
}
	if(WorkState == ADDITIONAL_STATE_TWO)
	{
		//****************�Զ�ȡ������//UM1--�ǰγ���UM2�෴  ���120//���PH2 С��PH4***************
/****************************�Լ�**********************************/
			//*****��������UFM.RxMsgC6x0.moment   >5000 ��Զ�˿�ס <-5000 �ڽ��˿�ס  + ��Զ���ƶ�  - �����ƶ� 
			//**targetAngle ���г�830����
			//360�㹲11���� ÿ����12.7mm 
			//������moment�Ǹ��� realangle=0,�м���410���� ��Զ��700����
			//�ȽϽ�����moment��3000    �������-3000 Զ����3000 
			//NMUDL840 NMUDR-840
			//����2000 3000
			/*if(channelrcol>500&&Claw_UpToPosition==0)//һ��̧����������
			{
				Claw_UpToPosition=1;
			}
			else if(channelrcol<-500)
			{
				Claw_UpToPosition=0;
				Claw_UpAngle=0;
			}
		  Claw_Up();
			
			if(channellrow>500&&AutoGet_Start==0)//�����Զ�ȡ������
				AutoGet_Start=1;
			if(channellrow<-500)//��;ֹͣ�����ڹ��ϴ���
			  AutoGet_Stop_And_Clear();
			
			
			AutoGet_SwitchState();*/
			/*ChassisSpeedRef.forward_back_ref = channelrcol * RC_CHASSIS_SPEED_REF;
		  ChassisSpeedRef.left_right_ref   = channelrrow * RC_CHASSIS_SPEED_REF/2;
			Chassis_Choose(1,1);*/
			if(channelrrow>500)
			 openthegay=1;
			if(channelrrow<-500)
				openthegay=0;
			
			if(openthegay==1)
				__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,600);
			else
				__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,1800);
			//���Ծ�Ԯ�� ƽ���ر�   ��++ ��--
	/*	ChassisSpeedRef.forward_back_ref = -channelrcol * RC_CHASSIS_SPEED_REF;
		ChassisSpeedRef.left_right_ref   = -channelrrow * RC_CHASSIS_SPEED_REF/2;
		ChassisSpeedRef.rotate_ref = -channellrow * RC_ROTATE_SPEED_REF;
			setzero();
			if(channellcol>500)
			{
				NMUDL.TargetAngle=-80;
				NMUDR.TargetAngle=80;
				lefttight=0;
				righttight=0;
			}
			
			
			if(leftstate==1)
				lefttight=1;
			if(lefttight==1)
			{
				if(NMUDL.RxMsgC6x0.moment<6000)
					NMUDL.TargetAngle+=10;
				if(NMUDL.RxMsgC6x0.moment>8000)
					NMUDL.TargetAngle-=5;
			}
			if(rightstate==1)
				righttight=1;
			if(righttight==1)
			{
				if(NMUDR.RxMsgC6x0.moment>-6000)
					NMUDR.TargetAngle-=10;
				if(NMUDR.RxMsgC6x0.moment<-8000)
					NMUDR.TargetAngle+=5;
			}*/
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

	KeyboardModeFSM(key);//�������ƶ��Ŀ��� ��д����ʱ��Ҫ��wasd��
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
			
	switch (KeyboardMode)
	{
		case SHIFT_CTRL:		//State control
		{
			
			break;
		}
		case CTRL:				//slow
		{
			if(key->v & KEY_C&&Claw_UpToPosition==0)
			{
				Claw_UpToPosition=1;
			}
			if(key->v & KEY_V)
			{
				Claw_UpToPosition=0;
				Claw_UpAngle=0;
			}
			if(key->v & KEY_Q)
			 CLAWOUT;
			if(key->v & KEY_E)
			 CLAWIN;
			
			if(key->v & KEY_Z)
			{
				AutoGet_Start=1;
			}
			if(key->v & KEY_X)
			{
				AutoGet_Start=2;
			}
		}break;
		case SHIFT:				//quick
		{
			
		}break;
		case NO_CHANGE:			//normal
		{//CM Movement Process
		
			if(key->v & KEY_X)
			{ 
				AutoGet_Stop_And_Clear();
			}
			else if(key->v & KEY_C)
			{
				Claw_TakeThisBox=1;
			}
			else if(key->v & KEY_V)
			{
				Claw_TakeThisBox=2;
			}
			else if(key->v & KEY_B)
			{
				Claw_TakeThisBox=3;
			}
			else if(key->v & KEY_F)
			{
				Claw_TakeThisBox=4;
			}
			else if(key->v & KEY_G)
			{
				Claw_TakeThisBox=5;
			}
			else if(key->v & KEY_Z)
			{
				Claw_SelfInspecting=1;
			}
			else if(key->v & KEY_Q)
			{
				Claw_FindingNextBox=1;
			}
			else if(key->v & KEY_E)
			{
				Claw_FindingNextBox=0;
				Sensor_Ready[0]=0;
			}
			
		}
		Claw_GetSpecifiedBox();
		Claw_SelfInspect();
		Claw_GoToNextBox_lower();
		Claw_Up();
		Box_Land();
		AutoGet_SwitchState();
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
