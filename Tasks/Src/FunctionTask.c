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
void initCM(double angle);
void routing(void);
void fleeing(void);
void swaying(void);
void scaning(void);
void firing1(void);
void firing2(void);
void firing3(void);
void remv(void);

float shootFrq=20;
double chassisAdd;
extern float fakeHeat0;
RampGen_t LRSpeedRamp = RAMP_GEN_DAFAULT;   	//斜坡函数
RampGen_t FBSpeedRamp = RAMP_GEN_DAFAULT;

int32_t auto_counter= 0;		//用于准确延时的完成某事件
int16_t channelrrow = 0;
int16_t channelrcol = 0;
int16_t channellrow = 0;
int16_t channellcol = 0;

static float swayRef;
int8_t stateSway=0;
int8_t stateFlee=0;
int8_t oneShootFlag=0;
int16_t stateCnt=1;
int16_t noEnemyCnt=1;
//初始化
void FunctionTaskInit(){
	chassisAdd=0;
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
uint8_t findEnemy;
//上平台代码
void RemoteControlProcess(Remote *rc){
	if(WorkState <= 0) return;
	//max=660
	channelrrow = (rc->ch0 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET);//leftRight
	channelrcol = (rc->ch1 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET);//upDown
	channellrow = (rc->ch2 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET);
	channellcol = (rc->ch3 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET);

	chassisAdd=-channelrrow*2;
	sendData[0].data[0]=(int16_t)WorkState | (int16_t)inputmode<<8;
	sendData[0].data[1]=channellrow;
	sendData[0].data[2]=channellcol;
	sendData[0].data[3]=(int16_t)(realHeat0*20);//
	if(WorkState == NORMAL_STATE){
		
	}
	if(WorkState == ADDITIONAL_STATE_ONE){
	
	}
	if(WorkState == ADDITIONAL_STATE_TWO){
//		swaying();
	}
	limtSync();
}
//********************上平台代码2
void selfControlProcess(Remote *rc){
	if(WorkState <= 0) return;
	//max=660
	channelrrow = (rc->ch0 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET);
	channelrcol = (rc->ch1 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET);
	channellrow = (rc->ch2 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET);
	channellcol = (rc->ch3 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET);
	
	findEnemy=(uint8_t)receiveData[0].data[0];
	sendData[0].data[0]=(int16_t)WorkState | (int16_t)inputmode<<8;
	sendData[0].data[1]=channellrow;
	sendData[0].data[2]=channellcol;
	sendData[0].data[3]=(int16_t)(realHeat0*20);
	if(WorkState == NORMAL_STATE){
		chassisAdd=-channelrrow*2;
	}
	if(WorkState == ADDITIONAL_STATE_ONE){
		remv();
		//routing();
	}
	if(WorkState == ADDITIONAL_STATE_TWO){
		if(findEnemy){
			if(stateSway||stateFlee){}
			else{stateFlee=1;}
			stateCnt=0;
		}
		if(stateFlee){fleeing();}
		else if(stateSway){swaying();}
		else{routing();}
		if(stateCnt>215){remv();}
	}
	limtSync();
}
#endif
#if GUARD == 'D'
//下平台代码
void RemoteControlProcess(){
	offLed(6);
	if(WorkState <= 0) return;
	//max=660
	channelrrow = 0;
	channelrcol = 0;
	channellrow = -receiveData[0].data[1];//leftRight
	channellcol = receiveData[0].data[2];//upDown
	realHeat0=receiveData[0].data[3]/(float)(20.0);
	GMY.Target+=channellrow*0.001f;
	GMP.Target+=channellcol*0.001f;
	if(WorkState == NORMAL_STATE){
		oneShootFlag=10;
//		STIRp.Real=0;
//		STIRp.Target=0;
		STIRv.Target=0;
		FRICL.Target=0;
		FRICR.Target=0;
		laserOn();
	}
	if(WorkState == ADDITIONAL_STATE_ONE){
//		STIRp.Real=0;
//		STIRp.Target=0;
		if(oneShootFlag){
			STIRv.Target=2700;oneShootFlag--;}
		else{STIRv.Target=0;}
		FRICL.Target =-5500;
		FRICR.Target = 5500;
		laserOn();
		if(findEnemy){autoAim();}
	}
	if(WorkState == ADDITIONAL_STATE_TWO){
		FRICL.Target =-5500;
		FRICR.Target = 5500;
		laserOn();
		if(findEnemy){autoAim();}
		firing2();
	}
	limtSync();
}
//*******************下平台代码2

void selfControlProcess(){
	offLed(6);
	if(WorkState <= 0) return;
	//max=660
	channelrrow = 0;
	channelrcol = 0;
	channellrow = -receiveData[0].data[1];//leftRight
	channellcol = receiveData[0].data[2];//upDown
	realHeat0=receiveData[0].data[3]/(float)(20.0);
	GMY.Target+=channellrow*0.001f;
	GMP.Target+=channellcol*0.001f;
	if(WorkState == NORMAL_STATE){
		oneShootFlag=8;
		STIRv.Target=0;
		FRICL.Target=0;
		FRICR.Target=0;
		laserOn();
	}
	if(WorkState == ADDITIONAL_STATE_ONE){
		if(oneShootFlag){STIRv.Target=2700;oneShootFlag--;}
		else{STIRv.Target=0;}
		
		FRICL.Target =-5500;
		FRICR.Target = 5500;
		laserOn();
		//if(!findEnemy){scaning();}
	}
	if(WorkState == ADDITIONAL_STATE_TWO){
		FRICL.Target =-5500;
		FRICR.Target = 5500;
		laserOn();
		if(findEnemy){
			autoAim();
			
			if(fabs(aim.yaw)<4){firing2();}
			else {firing1();}
			
			noEnemyCnt=-100;
			sendData[0].data[0]=(int16_t)1;
		}
		else if(noEnemyCnt==1){
			scaning();
			sendData[0].data[0]=(int16_t)0;
		}
		else if(noEnemyCnt<1){noEnemyCnt++;}
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
void remv(){
	stateFlee=0;
	stateSway=0;
	stateCnt=0;
	CML.Target=CML.Real;
	CMR.Target=CMR.Real;
	CML.positionPID.outputMax=4000;
	CMR.positionPID.outputMax=4000;
}
void routing(){
	static int dir=1;
//	static float speedRef;
	if(getRightSr() || fabs(CML.Real)>10000){//换向：红外触发、底盘超程
		dir=1; 
		onePushDir(dir,initCM(-8800););
	}
	else if(getLeftSr() || fabs(CML.Real)>10000){
		dir=-1;
		onePushDir(dir,initCM(0););
	}
	//speedRef+=0.05f;
	if(PowerHeat.chassis_power_buffer>100){chassisAdd =600*dir;}//(30*sin(speedRef)+100)*dir;
	else{chassisAdd =450*dir;}
}

void fleeing(){
	//0,1400,2400 [4400] 6400,7400,8800
	if(PowerHeat.chassis_power_buffer<100){
		CML.positionPID.outputMax=2000;
		CMR.positionPID.outputMax=2000;
	}
	else if(stateFlee==2){
		CML.positionPID.outputMax=3500;
		CMR.positionPID.outputMax=3500;
	}
	else if(stateFlee==1){
		CML.positionPID.outputMax=2500;
		CMR.positionPID.outputMax=2500;
	}
	
	double pos=(CML.Real+CMR.Real)/2;
	if(pos>-4400){
		CML.Target=-1800;
		CMR.Target=-1800;
	}else{
		CML.Target=-7000;
		CMR.Target=-7000;
	}
	if(fabs(CML.positionPID.errorCurr)<50){
		stateFlee=0;
		stateSway=1;
		swayRef=0;
		CML.positionPID.outputMax=3500;
		CMR.positionPID.outputMax=3500;
	}
	stateCnt++;
	if(getRightSr() || getLeftSr()){remv();}
}
void fleeing2(){
	static int8_t lock=0,dir=0;
	static float yaw,pos;
	if(!lock){
		yaw=imu.yaw;
		pos=(CML.Real+CMR.Real)/2;
		//0,1400,2400 [4400] 6400,7400,8800
		if(pos>-1800){dir=-1;}
		else if(pos>-4400){dir=1;}
		else if(pos>-7000){dir=-1;}
		else{dir=1;}
		lock=1;
	}
	if(PowerHeat.chassis_power_buffer<100){
		CML.positionPID.outputMax=2000;
		CMR.positionPID.outputMax=2000;
	}
	if(fabs(imu.yaw-yaw)>20){
		stateFlee=0;
		stateSway=1;
		swayRef=0;
		CML.positionPID.outputMax=4000;
		CMR.positionPID.outputMax=4000;
	}
	stateCnt++;
	if(getRightSr() || getLeftSr()){remv();}
}

void swaying(){
	chassisAdd=850*cos(swayRef);
	swayRef+=0.032f;//0.016= 0.087965*0.5= 2*3.1416*0.014 *0.5
	stateCnt++;
	if(getRightSr() || getLeftSr()){remv();}
}
void scaning(){
	GMY.Target-=0.5;
	GMP.Target=-10;
}



void firing1(){
	static int8_t jam=-1;
	if(STIRv.RxMsgC6x0.moment>8000){jam=30;}
	///TargetSpeed = shootFrq*45/360*36*60 = shootFrq*270
	if(jam<0){STIRv.Target=(2+channelrcol/66) *270;}
	else{STIRv.Target=-3500;jam--;}
}
void firing2(){
	static int8_t jam=-1;
	if(realHeat0<100){shootFrq=25;}//40
	else if(realHeat0<200){shootFrq=20;}//30
	else if(realHeat0<300){shootFrq=12;}//20
	else if(realHeat0<360 && shootFrq<8){shootFrq+=0.2f;}//12
	else if(realHeat0>440 && shootFrq>5){shootFrq--;}
	else if(realHeat0>400 && shootFrq>7){shootFrq-=0.1f;}
	else{shootFrq=8;}
	
	if(STIRv.RxMsgC6x0.moment>8000){jam=30;}
	///TargetSpeed = shootFrq*45/360*36*60 = shootFrq*270
	if(jam<0){STIRv.Target=shootFrq *270;}
	else{STIRv.Target=-3500;jam--;}
}
void firing3(){
	static int8_t jam=-1;
	if(STIRv.RxMsgC6x0.moment>8000){jam=9;}
	///TargetSpeed = shootFrq*45/360*36*60 = shootFrq*270
	if(jam<0){STIRv.Target=(20+channelrcol/66) *270;}
	else{STIRv.Target=-3000;jam--;}
}


