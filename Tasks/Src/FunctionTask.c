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
#define clawout   HAL_GPIO_WritePin(GPIOH,1<<2,1)//爪子弹出
#define clawin    HAL_GPIO_WritePin(GPIOH,1<<2,0)
#define testclawtight HAL_GPIO_WritePin(GPIOI,1<<5,1)
#define testclawloose HAL_GPIO_WritePin(GPIOI,1<<5,0)
#define testlaunch    HAL_GPIO_WritePin(GPIOH,1<<4,1)
#define testland      HAL_GPIO_WritePin(GPIOH,1<<4,0)
KeyboardMode_e KeyboardMode = NO_CHANGE;
RampGen_t LRSpeedRamp = RAMP_GEN_DAFAULT;   	//斜坡函数
RampGen_t FBSpeedRamp = RAMP_GEN_DAFAULT;
ChassisSpeed_Ref_t ChassisSpeedRef; 

int16_t channelrrow = 0;
int16_t channelrcol = 0;
int16_t channellrow = 0;
int16_t channellcol = 0;
int16_t testIntensity = 0;

int32_t auto_counter=0;		//用于准确延时的完成某事件
int32_t auto_lock=0;
int32_t cnt_clk;
int32_t auto_wait=0;
int16_t cnt = 0;
int ifset=-5;//用于自检
int cheeeeck=0;

//存储红外传感器的数值
extern uint32_t ADC_value[100];
extern uint32_t ADC2_value[100];
/**
	***********************************************
	*一键登岛
	*********************************************
	* reset_flag 重置变量，一次开机只执行一次
	*	
	*/
	uint8_t reset_flag = 1;
	uint8_t climb_state = 1;
	uint8_t climb_flag = 0;
	/**********登岛机构抬升复位************/
	
/**************一键复位**************/
	void resetClimbState(){
		
	}
/**************复位判断**************/
	void judgeClimbState(){
		if(channelrcol<-200)climb_flag = 0;
	}
	
/******************自动化取弹*************/
int flag_get=-1;
int  i2=0;
int  im=0;
int checkmode=0;
int resetmode=0;
int ifloose=0;
//下面是调试用变量 复位时记得全部复位
int theposition=0;
int inposition=1;
int laststate=0;
int letsrock=0;
int alreadychanged=0;
int back_a_little=0;
int totalstep=1;
int rollout=0;
int alreadywaited=0;
/*****************红外传感器消抖******************/
int tmp[2];
double count[2];
int isok=0;
int a;
int b;
/************************************************/

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
	Cap_Control();
	PowerLimitation();
}
uint32_t average(uint32_t a[])
{
	uint32_t ave;
	uint32_t sum=0;
	for(int j=0;j<sizeof(a);j++)
	{
		sum+=a[j];
	}
	ave=sum/sizeof(a);
	
	return ave;
}

void isokey()
{
	if(tmp[0]>100&&average(ADC_value)>100)
		count[0]+=0.5;
	else
	{tmp[0]=average(ADC_value);count[0]=0;}
	if(count[0]>1&&isok==0)
		isok=1;
	if(tmp[1]>100&&average(ADC2_value)>100)
		count[1]+=0.5;
	else
	{tmp[1]=average(ADC2_value);count[1]=0;}
	if(count[1]>1&&isok==1)
		isok=2;
		
	
}
void gofar()
{
	alreadychanged=0;
	if(UFM.RxMsgC6x0.moment<4000)
	UFM.TargetAngle+=3;
	if(UFM.RxMsgC6x0.moment>=4000&&alreadychanged==0)
	{ 
		inposition+=1;
		alreadychanged=1;
		back_a_little=0;
		totalstep++;
	}
}

void gonear()
{
	alreadychanged=0;
	if(UFM.RxMsgC6x0.moment>-4000)
	UFM.TargetAngle-=3;
	if(UFM.RxMsgC6x0.moment<=-4000&&alreadychanged==0)
	{
		inposition+=1;
		alreadychanged=1;
		back_a_little=0;
		totalstep++;
	}
}
void clawrollout()
{
	if(auto_counter==0&&i2<42&&rollout==0)
	{
     UM1.TargetAngle=i2*4;
     UM2.TargetAngle=-i2*4;
		 i2++;
		 auto_counter=1;
	}
	if(i2==42&&rollout==0)
		rollout=1;
}
void clawrollin()
{
	if(auto_counter==0&&i2>4&&rollout==1)
	{
		UM1.TargetAngle=i2*4;
    UM2.TargetAngle=-i2*4;
		i2--;
	  auto_counter=1;
	}
	if(i2==4&&rollout==1)
	{
		totalstep++;
		rollout=0;
	}
}


void clawtight()//爪子抓紧与松开
{
	if(i2==42)
	{
		HAL_GPIO_WritePin(GPIOI,1<<5,1);
		auto_counter=500;
		i2--;
	}
}

void clawloose()
{
	HAL_GPIO_WritePin(GPIOI,1<<5,0);
	if(alreadywaited==0)
	{
		auto_wait=300;
		alreadywaited=1;
	}
}
void launch()//弹药箱弹射装置
{
	if(auto_wait==0&&alreadywaited==1)
	{
	  HAL_GPIO_WritePin(GPIOH,1<<4,1);
	  auto_wait=300;
		alreadywaited=2;
	}
}

void land()
{
	if(auto_wait==0&&alreadywaited==2)
	{
	  HAL_GPIO_WritePin(GPIOH,1<<4,0);
		totalstep++;
		alreadywaited=0;
	}
}
void setlimit(int a)
{
	switch(a)
	{
		case 1:{ULM.TargetAngle=45;gonear();break;}
		case 2:{ULM.TargetAngle=0;gofar();break;}
		case 3:
		{
		   if(back_a_little==0)
			 {
		   UFM.TargetAngle-=30;
			 back_a_little=1;
				auto_wait=300;
			 }				 
		   ULM.TargetAngle=45;
			 if(auto_wait==0)
		   gofar();
		break;
		}
		case 4:{ULM.TargetAngle=135;gonear();break;}
		case 5:
		{
			if(back_a_little==0)
			{
		   UFM.TargetAngle+=30;
			 back_a_little=1;
				auto_wait=300;
			}
		   ULM.TargetAngle=-90;
			if(auto_wait==0)
		   gonear();
		   break;
		}
		case 6:
		{
			if(back_a_little==0)
			{
		   UFM.TargetAngle+=30;
				back_a_little=1;
				auto_wait=300;
			}
		   ULM.TargetAngle=45;
			if(auto_wait==0)
		   gonear();
		   break;
		}
		default: return;
	}
}

void gotoanywhere()
{
		if(theposition<6)
		{if(laststate!=inposition)
			 {
				 theposition++;
				laststate=inposition;
			 }
			setlimit(theposition);
		}
}
void getabox()
{
	clawrollout();
	clawtight();
	clawrollin();
}

void fire()
{
	clawloose();
	launch();
	land();
}
void autoget_test()
{
	switch(totalstep)
	{
		case 1:{setlimit(1);   break;}
		case 2:{getabox();     break;}
		case 3:{setlimit(2);   break;}
		case 4:{fire();        break;}
		case 5:{getabox();     break;}
		case 6:{setlimit(3);   break;}
		case 7:{fire();        break;}
		case 8:{getabox();     break;}
		/*case 9:{setlimit(4);   break;}
		case 10:{fire();       break;}
		case 11:{getabox();    break;}
		case 12:{setlimit(5);  break;}
		case 13:{fire();       break;}
		case 14:{getabox();    break;}
		case 15:{setlimit(6);  break;}
		case 16:{fire();       break;}*/
		default: break;
	}
}
void Limit_and_Synchronization()
{
	//demo
//	MINMAX(NMUDL.TargetAngle,-650,0);//limit
//	MINMAX(NMUDR.TargetAngle,0,650);
	
	MINMAX(NMUDL.TargetAngle,-700,700);//limit
	//MINMAX(UFM.TargetAngle,-700,700);
	NMUDL.TargetAngle = -NMUDR.TargetAngle;//sychronization
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
		ChassisSpeedRef.forward_back_ref = channelrcol * RC_CHASSIS_SPEED_REF;
		ChassisSpeedRef.left_right_ref   = channelrrow * RC_CHASSIS_SPEED_REF/2;
		//ChassisSpeedRef.rotate_ref = -channellrow * RC_ROTATE_SPEED_REF;
		
		//手动挡控制爪子
		if(channellcol>500)
		clawout;//左纵向是爪子的向前弹出
		if(channellcol<-500)
		clawin;
		
		UM1.TargetAngle+=channellrow*0.001;
		UM2.TargetAngle-=channellrow*0.001;//左横向是爪子的上下移动

//TODO/******暂未用到的参数  发射机构电机********/		
//		#ifdef USE_CHASSIS_FOLLOW
//		GMY.TargetAngle += channellrow * RC_GIMBAL_SPEED_REF ;
//		GMP.TargetAngle += channellcol * RC_GIMBAL_SPEED_REF ;
//		#else
//		ChassisSpeedRef.rotate_ref = channellrow * RC_ROTATE_SPEED_REF;
//		#endif
//		FRICL.TargetAngle = 0;
//		FRICR.TargetAngle = 0;
/******未用到参数********/		
		
	}
	if(WorkState == ADDITIONAL_STATE_ONE)
	{
//TODO/******暂未用到的参数  发射机构电机********/
//		FRICL.TargetAngle = -4200;
//		FRICR.TargetAngle = 4200;
/************************/	
		//reset_Pos();
		//手动挡
		if(channellcol>200){       //UP  左纵向是整个机构的上下
			NMUDL.TargetAngle += channellcol * 0.01;
			NMUDR.TargetAngle -= channellcol * 0.01;
		}	else if(channellcol<-200){		//DOWN 
			NMUDL.TargetAngle += channellcol * 0.01;
			NMUDR.TargetAngle -= channellcol * 0.01;
		}
	  if(channelrrow>500)
				testclawtight;//右横向是抓紧的开关
			if(channelrrow<-500)
				testclawloose;
			
			if(channelrcol>500)
				testlaunch;//右纵向是弹药箱弹出的开关
			if(channelrcol<-500)
				testland;
			
			UFM.TargetAngle-=channellrow*0.01;//左横向是水平电机   向左远离（角度++）向右靠近（角度--）
			
		
	/***********量化电机转速*************/	
//		if(cnt_clk == 0 && channellcol>200){       //UP  velocity = 1
//			NMUDL.TargetAngle += 10;
//			NMUDR.TargetAngle -= 10;
//			cnt_clk = 10;
//		}	else if(cnt_clk == 0 && channellcol<-200){		//DOWN velocity = 1
//			NMUDL.TargetAngle -= 10;
//			NMUDR.TargetAngle += 10;
//			cnt_clk = 10;
//		}
//		
//		if(cnt_clk == 0 && channellrow > 200){			//Forward
//			NMUDFL.TargetAngle -= 10 ;
//			NMUDFR.TargetAngle += 10 ;
//			cnt_clk = 10;
//		}else if(cnt_clk == 0 && channellrow < -200){//Back
//			NMUDFL.TargetAngle += 10 ;
//			NMUDFR.TargetAngle -= 10 ;
//			cnt_clk = 10;
//		}
		
		
		
		

}
	if(WorkState == ADDITIONAL_STATE_TWO)
	{
//TODO/******暂未用到的参数  发射机构电机********/
//		FRICL.TargetAngle = 5000;
//		FRICR.TargetAngle = -5000;
//		Delay(20,{STIR.TargetAngle-=60;});


		//****************自动取弹程序//UM1--是拔出来UM2相反  最大120//大的PH2 小的PH4***************
     /*if(ifset==1)
		 {autoget();
			 if(auto_lock==0)
			 {
			ChassisSpeedRef.left_right_ref   = (channellrow/(2.7)) * RC_CHASSIS_SPEED_REF;
			ChassisSpeedRef.forward_back_ref = (channellcol/2) * RC_CHASSIS_SPEED_REF;
			 }
		 }*/
		 
/****************************自检**********************************/
      //reset_Pos();
			//*****调试模式UFM.RxMsgC6x0.moment   >5000 在远端卡住 <-5000 在近端卡住  + 往远端移动  - 近端移动 
			//**targetAngle 总行程830左右
			//360°共11个齿 每个齿12.7mm 
			//最靠近电机moment是负的 realangle=0,中间是410左右 最远端700左右
	    //autoget_final();
			ULM.TargetAngle+=channelrrow*0.01;
			//比较健康的moment是3000    靠近电机-3000 远离电机3000 
			
			if(channellrow>500&&letsrock==0)
				letsrock=1;
			if(channellrow<-500)
			{ letsrock=0;
				theposition=0;
				inposition=1;
				back_a_little=0;
				laststate=0;
				totalstep=1;
				i2=0;
				UM1.TargetAngle=0;
				UM2.TargetAngle=0;
				rollout=0;
				testclawloose;
			}
			
			if(letsrock==1)
				autoget_test();
			
	}
	Limit_and_Synchronization();
}


uint16_t KM_FORWORD_BACK_SPEED 	= NORMAL_FORWARD_BACK_SPEED;
uint16_t KM_LEFT_RIGHT_SPEED  	= NORMAL_LEFT_RIGHT_SPEED;
void KeyboardModeFSM(Key *key);

//****************
//键鼠模式功能编写
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
