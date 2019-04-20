/**
  ******************************************************************************
  * File Name          : SentryTask.c
  * Description        : 哨兵2019
  ******************************************************************************
  *
  * Copyright (c) 2019 Team JiaoLong-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#include "includes.h"
#define getLeftSr()		!HAL_GPIO_ReadPin(GPIOE,leftSensor_Pin)// //红外检测到为低电平，故取非运算。Sr缩写Sensor
#define getRightSr()	!HAL_GPIO_ReadPin(GPIOE,rightSensor_Pin) //
#define onLed(x) 	HAL_GPIO_WritePin(GPIOG,1<<x, GPIO_PIN_RESET)
#define offLed(x) 	HAL_GPIO_WritePin(GPIOG,1<<x, GPIO_PIN_SET)
void initCM(void);
void scaning(void);
void routing(void);
void shootCtrl(void);

float shootFrq=20;
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
//	MINMAX(GMY.Target,-160+GMY.imuEncorderDiff,160+GMY.imuEncorderDiff);//limit
	//CMR.Target =  -CML.Target;
}
//******************
//遥控器模式功能编写
//******************
#if GUARD == 'U'
//上平台代码
void RemoteControlProcess(Remote *rc){
	if(WorkState <= 0) return;
	//max=660
	channelrrow = (rc->ch0 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET);//leftRight
	channelrcol = (rc->ch1 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET);//upDown
	channellrow = (rc->ch2 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET);
	channellcol = (rc->ch3 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET);

	ChassisSpeedRef.left_right_ref =- channelrrow * RC_CHASSIS_SPEED_REF*0.5f;
	sendData[0].data[0]=(int16_t)WorkState | (int16_t)inputmode<<8;
	sendData[0].data[1]=channellrow;
	sendData[0].data[2]=channellcol;
	sendData[0].data[3]=(int16_t)(fakeHeat0*20);//
	if(WorkState == NORMAL_STATE){
		STIRp.Real=0;
		STIRp.Target=0;
		STIRv.Target=0;
	}
	if(WorkState == ADDITIONAL_STATE_ONE){
		STIRp.Real=0;
		STIRp.Target=0;
		STIRv.Target=0;
	}
	if(WorkState == ADDITIONAL_STATE_TWO){
		static int jam=-1;
		if(STIRv.RxMsgC6x0.moment>9500){jam=9;}
		///TargetSpeed = shootFrq*45/360*36*60 = shootFrq*270
		if(jam<0){STIRv.Target=(20+channelrcol/66) *270;}
		else{STIRv.Target=-3000;jam--;}
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
	GMY.Target+=channellrow*0.001f;
	GMP.Target+=channellcol*0.001f;
	if(WorkState == NORMAL_STATE){
		STIRv.Target=0;
		FRICL.Target=0;
		FRICR.Target=0;
		laserOn();
//		aimMode=1;
//		autoAimCtrl();
	}
	if(WorkState == ADDITIONAL_STATE_ONE){
		STIRv.Target=0;
//		FRICL.Target =-2000;
//		FRICR.Target =2000;
		laserOn();
		aimMode=1;
		autoAimCtrl();
	}
	if(WorkState == ADDITIONAL_STATE_TWO){
//		FRICL.Target =-2000;
//		FRICR.Target =2000;
		laserOn();
		aimMode=1;
		autoAimCtrl();
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
static int oneShoot=0;
void selfControlProcess(Remote *rc){
	if(WorkState <= 0) return;
	//max=660
	channelrrow = (rc->ch0 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET); 
	channelrcol = (rc->ch1 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET); 
	channellrow = (rc->ch2 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET); 
	channellcol = (rc->ch3 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET); 

	sendData[0].data[0]=(int16_t)WorkState | (int16_t)inputmode<<8;
	sendData[0].data[1]=channellrow;
	sendData[0].data[2]=channellcol;
	sendData[0].data[3]=(int16_t)(fakeHeat0*20);
	if(WorkState == NORMAL_STATE){
		oneShoot=10;
		STIRv.Target=0;
	}
	if(WorkState == ADDITIONAL_STATE_ONE){
		if(oneShoot){STIRv.Target=2700;oneShoot--;}
		else{STIRv.Target=0;}
		//routing();
	}
	if(WorkState == ADDITIONAL_STATE_TWO){
		shootCtrl();
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
	GMY.Target+=channellrow*0.001f;
	GMP.Target+=channellcol*0.001f;
	if(WorkState == NORMAL_STATE){
		FRICL.Target=0;
		FRICR.Target=0;
		laserOn();
	}
	if(WorkState == ADDITIONAL_STATE_ONE){
		FRICL.Target=0;
		FRICR.Target=0;
		laserOn();
		aimMode=1;
		autoAimCtrl();
		//if(!findEnemy){scaning();}
	}
	if(WorkState == ADDITIONAL_STATE_TWO){
		FRICL.Target =-5000;
		FRICR.Target =5000;
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

void routing(){
	static int dir=1;
	static float speedRef;
	if(getLeftSr() && getRightSr()){dir=0;initCM();}
	else if(getRightSr() || fabs(CML.Real)>36000 || abs(CML.RxMsgC6x0.moment)>10000){//换向保护：红外触发、底盘超程、底盘超载
		dir=1; 
		onePushDir(dir,initCM(););
	}
	else if(getLeftSr()  || fabs(CML.Real)>36000 || abs(CML.RxMsgC6x0.moment)>10000){
		dir=-1;
		onePushDir(dir,initCM(););
	}
	speedRef+=0.05;
	ChassisSpeedRef.left_right_ref =(30*sin(speedRef)+100)*dir;
}
void scaning(){
	static int dir=1;
	int lfLmt=-150,rtLmt=150;
	float stepLR=0.014*20;
	static float UDref;
//	MINMAX(GMY.Target,-160+GMY.imuEncorderDiff,160+GMY.imuEncorderDiff);//limit
	if(dir==1&&GMY.Real- GMY.imuEncorderDiff>rtLmt){dir=-1;}
	else if(dir==-1&&GMY.Real- GMY.imuEncorderDiff<lfLmt){dir=1;}
	GMY.Target+=(stepLR*dir);
	
	GMP.Target=10*sin(UDref)-5;
	UDref+=0.016;//= 0.087965*0.2= 2*3.1416*0.014 *0.2
}
void fleeing(){//4200
	
}
int pState;
void getPState(){
	float cmAngle=(CML.Real+CMR.Real)/2;
//	if(cmAngle<){pState=1;}
//	else if(cmAngle<){pState=2;}
//	else if(cmAngle<){pState=3;}
//	else if(cmAngle<){pState=4;}
//	else if(cmAngle<){pState=5;}
//	else if(cmAngle<){pState=6;}
//	else if(cmAngle<){pState=7;}
//	else if(cmAngle<){pState=8;}
}
void shootCtrl(){
	static int jam=-1;
	if(realHeat0<100){shootFrq=40;}
	else if(realHeat0<200){shootFrq=30;}
	else if(realHeat0<300){shootFrq=20;}
	else if(realHeat0<360 && shootFrq<12){shootFrq+=0.2;}
	else if(realHeat0>440 && shootFrq>5){shootFrq--;}
	else if(realHeat0>400 && shootFrq>9){shootFrq-=0.1;}
	else{shootFrq=10;}
	
	if(STIRp.RxMsgC6x0.moment>9500){ STIRp.Real=0; STIRp.Target=-150;}
	///TargetAngle = shootFrq*45/1000*14=shootFrq*0.63
	else{STIRp.Target+=(shootFrq+channelrcol/66)*0.63;}
	
	if(STIRv.RxMsgC6x0.moment>9500){jam=9;}
	///TargetSpeed = shootFrq*45/360*36*60 = shootFrq*270
	if(jam<0){STIRv.Target=(shootFrq+channelrcol/66) *270;}
	else{STIRv.Target=-3000;jam--;}
}

	