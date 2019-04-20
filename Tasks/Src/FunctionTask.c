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
#define getLeftSr()		!HAL_GPIO_ReadPin(GPIOE,leftSensor_Pin)// //红外检测到为低电平，故取非运算。Sr缩写Sensor
#define getRightSr()	!HAL_GPIO_ReadPin(GPIOE,rightSensor_Pin) //
#define onLed(x) 	HAL_GPIO_WritePin(GPIOG,1<<x, GPIO_PIN_RESET)
#define offLed(x) 	HAL_GPIO_WritePin(GPIOG,1<<x, GPIO_PIN_SET)
void initCM(double angle);
void scaning(void);
void routing(void);
void firing(void);

float shootFrq=20;
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
//	MINMAX(GMY.Target,-160+GMY.imuEncorderDiff,160+GMY.imuEncorderDiff);//limit
	//CMR.Target =  -CML.Target;
}
//******************
//遥控器模式功能编写
//******************
#if GUARD == 'U'
//上平台代码
static int oneShoot=0;
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
		oneShoot=8;
		STIRp.Real=0;
		STIRp.Target=0;
		STIRv.Target=0;
	}
	if(WorkState == ADDITIONAL_STATE_ONE){
		STIRp.Real=0;
		STIRp.Target=0;
		if(oneShoot){STIRv.Target=2700;oneShoot--;}
		else{STIRv.Target=0;}
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
		FRICL.Target =-5000;
		FRICR.Target =5000;
		laserOn();
		aimMode=1;
		autoAimCtrl();
	}
	if(WorkState == ADDITIONAL_STATE_TWO){
		FRICL.Target =-5000;
		FRICR.Target =5000;
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
		oneShoot=8;
		STIRv.Target=0;
	}
	if(WorkState == ADDITIONAL_STATE_ONE){
		if(oneShoot){STIRv.Target=2700;oneShoot--;}
		else{STIRv.Target=0;}
		//routing();
	}
	if(WorkState == ADDITIONAL_STATE_TWO){
		firing();
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
		FRICL.Target =-5000;
		FRICR.Target =5000;
		laserOn();
		aimMode=1;
		autoAimCtrl();
		//if(!findEnemy){scaning();}
	}
	if(WorkState == ADDITIONAL_STATE_TWO){
		FRICL.Target =-5000;
		FRICR.Target =5000;
		laserOn();
		aimMode=1;
		autoAimCtrl();
	}
	limtSync();
}
#endif




void initCM(double angle){
	CML.Real=angle;
	CML.Target=angle;
	CMR.Real=angle;
	CMR.Target=angle;
}

void routing(){
	static int dir=1;
	static float speedRef;
	if(getRightSr() || fabs(CML.Real)>10000 ){//换向：红外触发、底盘超程
		dir=1; 
		onePushDir(dir,initCM(8800););
	}
	else if(getLeftSr() || fabs(CML.Real)>10000){
		dir=-1;
		onePushDir(dir,initCM(0););
	}
	speedRef+=0.05;
	ChassisSpeedRef.left_right_ref =(30*sin(speedRef)+100)*dir;
}
void scaning(){
	static int dir=1;
	int lfLmt=-150,rtLmt=150;
	float stepLR=0.014*20;
//	static float UDref;
//	MINMAX(GMY.Target,-160+GMY.imuEncorderDiff,160+GMY.imuEncorderDiff);//limit
	if(dir==1&&GMY.Real- GMY.imuEncorderDiff>rtLmt){dir=-1;}
	else if(dir==-1&&GMY.Real- GMY.imuEncorderDiff<lfLmt){dir=1;}
	GMY.Target+=(stepLR*dir);
	GMP.Target=-5;
//	GMP.Target=10*sin(UDref)-5;
//	UDref+=0.016;//= 0.087965*0.2= 2*3.1416*0.014 *0.2
}
int fleeing(){
	//0,1400,2400 [4400] 6400,7400,8800
	double pos=(CML.Real+CMR.Real)/2;
	if(fabs(pos+7000)>fabs(pos+2000)){
		CML.Target=-2000;
		CMR.Target=-2000;
	}else{
		CML.Target=-7000;
		CMR.Target=-7000;
	}
	if(fabs(CML.positionPID.errorCurr)<100){return 0;}
	return 1;
}
void swaying(){
	static float posRef;
	CMR.Target+=300*cos(posRef);
	CML.Target=CMR.Target;
	posRef+=0.016;//= 0.087965*0.2= 2*3.1416*0.014 *0.2
}

void firing(){
	static int jam=-1;
	if(realHeat0<100){shootFrq=30;}//40
	else if(realHeat0<200){shootFrq=22;}//30
	else if(realHeat0<300){shootFrq=15;}//20
	else if(realHeat0<360 && shootFrq<10){shootFrq+=0.2;}//12
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

	