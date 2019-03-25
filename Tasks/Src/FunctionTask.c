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
#define onLed(x) 	HAL_GPIO_WritePin(GPIOG,1<<x, GPIO_PIN_RESET)
#define offLed(x) 	HAL_GPIO_WritePin(GPIOG,1<<x, GPIO_PIN_SET)
void initCM(void);
float shootFrq=30;
extern float fakeHeat0;
RampGen_t LRSpeedRamp = RAMP_GEN_DAFAULT;   	//斜坡函数
RampGen_t FBSpeedRamp = RAMP_GEN_DAFAULT;
ChassisSpeed_Ref_t ChassisSpeedRef; 

int32_t auto_counter=0;		//用于准确延时的完成某事件
int16_t channelrrow = 0;
int16_t channelrcol = 0;
int16_t channellrow = 0;
int16_t channellcol = 0;

//初始化
void FunctionTaskInit(){
	ChassisSpeedRef.left_right_ref = 0.0f;
}
//限位与同步
void limtSync(){
	MINMAX(GMP.Target,-35,20);//limit
	//MINMAX(GMY.Target,-30,30);//limit
	//CMR.Target =  -CML.Target;
}
//******************
//遥控器模式功能编写
//******************
#if GUARD == 'U'
//上平台代码
int jam=-1;
void RemoteControlProcess(Remote *rc){
	if(WorkState <= 0) return;
	//max=660
	channelrrow = (rc->ch0 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET);//leftRight
	channelrcol = (rc->ch1 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET);//upDown
	channellrow = (rc->ch2 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET); 
	channellcol = (rc->ch3 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET); 

	ChassisSpeedRef.left_right_ref =- channelrrow * RC_CHASSIS_SPEED_REF*0.5f;
	sendData[0].data[0]=(int16_t)WorkState;
	sendData[0].data[1]=channellrow;
	sendData[0].data[2]=channellcol;
	sendData[0].data[3]=(int16_t)(fakeHeat0*20);//
	if(WorkState == NORMAL_STATE){
		STIRv.Target=0;
	}
	if(WorkState == ADDITIONAL_STATE_ONE){
		STIRv.Target=0;
	}
	if(WorkState == ADDITIONAL_STATE_TWO){
		if(STIRv.RxMsgC6x0.moment<-9000){jam=5;}
		//TargetSpeed = shootFrq*45/360*36*60 = shootFrq*270
		if(jam<0){STIRv.Target=-270*20-channelrcol/660*270*10;}
		else{STIRv.Target=2160;jam--;}
		
//		if(abs(STIRp.RxMsgC6x0.moment)<9000){STIRp.Target-= 15+ channelrcol/66;}
//		else{STIRp.Real=0;STIRp.Target=60;}
	}
	limtSync();
}
#endif
#if GUARD == 'D'
//下平台代码
void RemoteControlProcess(){
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
		FRICL.Target =5000;
		FRICR.Target =-5000;
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
//模式功能编写
//****************
#if GUARD == 'U'
//上平台代码
static float speedRef;
void selfControlProcess(Remote *rc){
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

	}
	if(WorkState == ADDITIONAL_STATE_ONE){

	}
	if(WorkState == ADDITIONAL_STATE_TWO){
		if(fakeHeat0<100){shootFrq=40;}
		else if(fakeHeat0<200){shootFrq=30;}
		else if(fakeHeat0<300){shootFrq=20;}
		else if(fakeHeat0<360 && shootFrq<12){shootFrq+=0.2;}
		else if(fakeHeat0>440 && shootFrq>5){shootFrq--;}
		else if(fakeHeat0>400 && shootFrq>9){shootFrq-=0.1;}
		else{shootFrq=10;}
		if(STIRv.RxMsgC6x0.moment<-9000){jam=5;}
		//TargetSpeed = shootFrq*45/360*36*60 = shootFrq*270
		if(jam<0){STIRv.Target=-shootFrq*270;}
		else{STIRv.Target=2000;jam--;}
		
		
//		
//		if(fakeHeat0<100){shootFrq=40;}
//		else if(fakeHeat0<200){shootFrq=30;}
//		else if(fakeHeat0<300 && shootFrq<18){shootFrq+=0.1;}
//		else if(fakeHeat0<360 && shootFrq<12){shootFrq+=0.02;}
//		else if(fakeHeat0>400 && shootFrq>9){shootFrq-=0.05;}
//		else if(fakeHeat0>420 && shootFrq>5){shootFrq--;}
////		if(PowerHeatData.shooterHeat0<100){shootFrq=20;}
////		else if(PowerHeatData.shooterHeat0<200){shootFrq=18;}
////		else if(PowerHeatData.shooterHeat0<300 && shootFrq<18){shootFrq+=0.02;}
////		else if(PowerHeatData.shooterHeat0>350 && shootFrq>5){shootFrq--;}
//		//fakeFrq = shootFrq*45/1000*14 = shootFrq*0.63
//		float fakeFrq = shootFrq*0.63;
//		if(abs(STIRp.RxMsgC6x0.moment)<9000){STIRp.Target-= fakeFrq;}
//		else{STIRp.Real=0;STIRp.Target=60;}
//		
		
		
		
//		if(getLeftSr() && getRightSr()){dir=0;initCM();}
//		else if(getRightSr() || fabs(CML.Real)>36000 || abs(CML.RxMsgC6x0.moment)>10000){//换向保护：红外触发、底盘超程、底盘超载
//			dir=1; 
//			onePushDir(dir,initCM(););
//		}
//		else if(getLeftSr()  || fabs(CML.Real)>36000 || abs(CML.RxMsgC6x0.moment)>10000){
//			dir=-1;
//			onePushDir(dir,initCM(););
//		}
//		speedRef+=0.05;
//		ChassisSpeedRef.left_right_ref =(20*sin(speedRef)+60)*dir;
	}
	limtSync();
}
#endif
#if GUARD == 'D'
//下平台代码
void selfControlProcess(){
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
		AutoAimGMCTRL();
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

