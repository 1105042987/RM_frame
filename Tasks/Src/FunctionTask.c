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
#define getLeftSr()		!HAL_GPIO_ReadPin(GPIOE,leftSensor_Pin) //红外检测到为低电平，故取非运算。Sr缩写Sensor
#define getRightSr()	!HAL_GPIO_ReadPin(GPIOE,rightSensor_Pin) 
void initCM(void);
RampGen_t LRSpeedRamp = RAMP_GEN_DAFAULT;   	//斜坡函数
RampGen_t FBSpeedRamp = RAMP_GEN_DAFAULT;
ChassisSpeed_Ref_t ChassisSpeedRef; 

int32_t auto_counter=0;		//用于准确延时的完成某事件
int16_t channelrrow = 0;
int16_t channelrcol = 0;
int16_t channellrow = 0;
int16_t channellcol = 0;


//初始化
void FunctionTaskInit()
{
	ChassisSpeedRef.forward_back_ref = 0.0f;
	ChassisSpeedRef.left_right_ref = 0.0f;
	ChassisSpeedRef.rotate_ref = 0.0f;
}
//限位与同步
void limtSync()
{
	MINMAX(GMP.Target,-35,20);//limit
	//MINMAX(GMY.Target,-30,30);//limit
	//CMR.Target =  -CML.Target;
}
//******************
//遥控器模式功能编写
//******************
#if GUARD == 'U'
//上平台代码
void RemoteControlProcess(Remote *rc)
{
	if(WorkState <= 0) return;
	//max=660
	channelrrow = (rc->ch0 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET); 
	channelrcol = (rc->ch1 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET); 
	channellrow = (rc->ch2 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET); 
	channellcol = (rc->ch3 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET); 

	ChassisSpeedRef.left_right_ref =- channelrrow * RC_CHASSIS_SPEED_REF*0.5f;
	sendData[0].data[0]=(int16_t)WorkState;
	sendData[0].data[1]=channellrow;
	sendData[0].data[2]=channellcol;
	sendData[0].data[3]=(int16_t)(fakeHeat0*20);
	if(WorkState == NORMAL_STATE){//手动模式

	}
	if(WorkState == ADDITIONAL_STATE_ONE){//自动模式
		
	}
	if(WorkState == ADDITIONAL_STATE_TWO){
		if(abs(STIR.RxMsgC6x0.moment)<10000){STIR.Target-= 10 + channelrcol/100;}
		else{STIR.Real=0;STIR.Target=16;}
	}
	OnePush(FUNC__RED_RAY_M__READ(),{
		CML.Target = 0;
		CML.Real = 0;
	});
	limtSync();
}
#endif
#if GUARD == 'D'
//下平台代码
void RemoteControlProcess()
{
	if(WorkState <= 0) return;
	//max=660
	channelrrow = 0;
	channelrcol = 0;
	channellrow = receiveData[0].data[1];//leftRight
	channellcol = receiveData[0].data[2];//upDown
	fakeHeat0=receiveData[0].data[3]/(float)(20.0);
	GMY.Target+=channellrow * 0.0006f;
	GMP.Target+=channellcol * 0.0006f;
	if(WorkState == NORMAL_STATE){
		FRICL.Target =0;
		FRICR.Target =0;
		laserOff();
	}
	if(WorkState == ADDITIONAL_STATE_ONE){
		//FRICL.Target =5000;
		//FRICR.Target =-5000;
		aim_mode=1;
		AutoAimGMCTRL();
		laserOn();
	}
	if(WorkState == ADDITIONAL_STATE_TWO){
		FRICL.Target =5000;
		FRICR.Target =-5000;
		laserOn();
	}
	limtSync();
}
#endif

//****************
//键鼠模式功能编写
//****************
#if GUARD == 'U'
//上平台代码
void RemoteControlProcess2(Remote *rc)
{
	static int dir=1;
	if(WorkState <= 0) return;
	//max=660
	channelrrow = (rc->ch0 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET); 
	channelrcol = (rc->ch1 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET); 
	channellrow = (rc->ch2 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET); 
	channellcol = (rc->ch3 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET); 

	sendData[0].data[0]=(int16_t)WorkState;
	sendData[0].data[1]=channellrow;
	sendData[0].data[2]=channellcol;
	sendData[0].data[3]=(int16_t)(fakeHeat0*20);
	if(WorkState == NORMAL_STATE){
		if(getRightSr() || fabs(CML.Real)>36000 || abs(CML.RxMsgC6x0.moment)>10000)  {dir=1; initCM();}//换向保护：红外触发、底盘超程、底盘超载
		if(getLeftSr()  || fabs(CML.Real)>36000 || abs(CML.RxMsgC6x0.moment)>10000)  {dir=-1;initCM();}
		if(getLeftSr()  && getRightSr())  {dir=0;initCM();}
		
		ChassisSpeedRef.left_right_ref =200*dir;
	}
	if(WorkState == ADDITIONAL_STATE_ONE){//自动模式
		
	}
	if(WorkState == ADDITIONAL_STATE_TWO){
		if(abs(STIR.RxMsgC6x0.moment)<10000){STIR.Target-= 10 + channelrcol/100;}
		else{STIR.Real=0;STIR.Target=16;}
	}
	OnePush(FUNC__RED_RAY_M__READ(),{
		CML.Target = 0;
		CML.Real = 0;
	});
	limtSync();
}
#endif
#if GUARD == 'D'
//下平台代码
void RemoteControlProcess2()
{
	if(WorkState <= 0) return;
	//max=660
	channelrrow = 0;
	channelrcol = 0;
	channellrow = receiveData[0].data[1];//leftRight
	channellcol = receiveData[0].data[2];//upDown
	fakeHeat0=receiveData[0].data[3]/(float)(20.0);
	GMY.Target+=channellrow * 0.0006f;
	GMP.Target+=channellcol * 0.0006f;
	if(WorkState == NORMAL_STATE){
		FRICL.Target =0;
		FRICR.Target =0;
		laserOff();
	}
	if(WorkState == ADDITIONAL_STATE_ONE){
		FRICL.Target =2000;
		FRICR.Target =-2000;
		//AutoAimGMCTRL();
		laserOn();
	}
	if(WorkState == ADDITIONAL_STATE_TWO){
		FRICL.Target =2000;
		FRICR.Target =-2000;
		laserOn();
	}
	limtSync();
}
#endif
void initCM(){
	CML.Real=0;
	CML.Target=0;
	CMR.Real=0;
	CMR.Target=0;
}

