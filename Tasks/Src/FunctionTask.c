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
double ChassisSpeed;
extern float fakeHeat0;
RampGen_t LRSpeedRamp = RAMP_GEN_DAFAULT;   	//斜坡函数
RampGen_t FBSpeedRamp = RAMP_GEN_DAFAULT;
int32_t auto_counter= 0;		//用于准确延时的完成某事件
int16_t channelrrow,channelrcol,channellrow,channellcol;
int8_t StateSway,StateFlee,StateHurt,StateRand=1;
int16_t StateCnt=1;
int16_t noEnemyCnt=1;
void strategyShoot(void);
//初始化
void FunctionTaskInit(){
	ChassisSpeed=0;
}
//限位与同步

//******************
//遥控器模式功能编写
//******************
#if GUARD == 'U'
void generalProcess(Remote *rc);
uint8_t findEnemy;
//********************上平台代码1
void RCProcess1(Remote *rc){
	generalProcess(rc);
	if(getLeftSw()){onLed(7);}
	else{offLed(7);}
	if(WorkState == STATE_1){

	}
	if(WorkState == STATE_2){

	}
	if(WorkState == STATE_3){
		
	}
	limtSync();
}
//********************上平台代码2

void RCProcess2(Remote *rc){
	generalProcess(rc);
	if(WorkState == STATE_1){
		int8_t dir=sgn(channelrrow);
		onePushDir(dir,LimitCnt=500);
	}
	if(WorkState == STATE_2){
		routing1();
	}
	if(WorkState == STATE_3){
		routing1();
	}
	limtSync();
}
//********************上平台代码3
void RCProcess3(Remote *rc){
	generalProcess(rc);
	if(WorkState == STATE_1){
		
	}
	if(WorkState == STATE_2){
		
	}
	limtSync();
}


//===================================
//********************上平台通用代码
//===================================
void generalProcess(Remote *rc){	
	if(WorkState <= 0) return;
	//max=660
	channelrrow = (rc->ch0 - (int16_t)Pos1LER_STICK_OFFSET);//leftRight
	channelrcol = (rc->ch1 - (int16_t)Pos1LER_STICK_OFFSET);
	channellrow = (rc->ch2 - (int16_t)Pos1LER_STICK_OFFSET);
	channellcol = (rc->ch3 - (int16_t)Pos1LER_STICK_OFFSET);
	
	ChassisSpeed=channelrrow*3;
	findEnemy=(uint8_t)receiveData[0].data[0];
	sendData[0].data[0]=(int16_t)WorkState | (int16_t)RCRightMode<<8;
	
	if(GameRobotState.robot_id==7){sendData[0].data[1]=channellrow+5000;}
	else{sendData[0].data[1]=channellrow;}
	
	if(channelrcol>600){sendData[0].data[2]=channellcol+5000;}
	else{sendData[0].data[2]=channellcol;}
	sendData[0].data[3]=(int16_t)(RealHeat0*20);
}
void limtSync(){
	MINMAX(GMP.Target,-60,0);//limit
//	MINMAX(GMY.Target,-160+GMY.imuEncorderDiff,160+GMY.imuEncorderDiff);//limit
	//CMR.Target =  -CML.Target;
}
#endif  //GUARD == 'U'
#if GUARD == 'D'
//下平台代码
void generalProcess();
void RCProcess1(){
	generalProcess();
	if(WorkState == STATE_1){
		STIRv.Target=0;
		FRICL.Target=0;
		FRICR.Target=0;
	}
	if(WorkState == STATE_2){
		STIRv.Target=0;
		FRICL.Target=-0;
		FRICR.Target= 0;
		uartSend();
		if(findEnemy){autoAim();}
	}
	if(WorkState == STATE_3){
		FRICL.Target =-5500;
		FRICR.Target = 5500;
		uartSend();
		if(findEnemy){autoAim();}
		if(receiveData[0].data[2]>4000){firing2();}
		else{firing1();}
	}
	limtSync();
}
//*******************下平台代码2
void RCProcess2(){
	generalProcess();
	if(WorkState == STATE_1){
		STIRv.Target=0;
		FRICL.Target=0;
		FRICR.Target=0;
	}
	if(WorkState == STATE_2){
		STIRv.Target=0;
		FRICL.Target=-0;
		FRICR.Target= 0;
		uartSend();
		if(findEnemy){autoAim();}
	}
	if(WorkState == STATE_3){
		strategyShoot();
		uartSend();
	}
	limtSync();
}

//*******************下平台代码3
void RCProcess3(){
	generalProcess();
	if(WorkState == STATE_1){
		STIRv.Target=0;
		FRICL.Target=0;
		FRICR.Target=0;
	}
	if(WorkState == STATE_2){
		strategyShoot();
		uartSend();
	}
	limtSync();
}

void strategyShoot(){
	FRICL.Target =-5500;
	FRICR.Target = 5500;
	laserOn();
	if(findEnemy){
		autoAim();
		if(fabs(aim.yaw)<3 && aim.dis==0){firing2();}
		//else {firing1();}
		noEnemyCnt=-300;
		sendData[0].data[0]=(int16_t)1;
	}
	else if(noEnemyCnt>1){
		scaning1();
		sendData[0].data[0]=(int16_t)0;
	}
	else if(noEnemyCnt<-200){
		noEnemyCnt++;
		if(fabs(aim.yaw)<4 && aim.dis==0){firing2();}
		else{STIRv.Target=0;}
	}
	else{
		noEnemyCnt++;
		STIRv.Target=0;
	}
}

//===================================
//*******************下平台通用代码
//===================================
void generalProcess(){
	offLed(6);
	laserOn();
	if(WorkState <= 0) return;
	//max=660
	channelrrow = 0;
	channelrcol = 0;
	channellrow = -receiveData[0].data[1];//leftRight
	channellcol = receiveData[0].data[2];//upDown
	
	if(channellrow<-4000){channellrow+=5000;}
	if(channellcol>4000){channellcol-=5000;}
	
	RealHeat0=receiveData[0].data[3]/(float)(20.0);
	GMY.Target+=channellrow*0.001f;
	GMP.Target+=channellcol*0.001f;
}

void limtSync(){
	MINMAX(GMP.Target,-60,0);//limit
//	MINMAX(GMY.Target,-160+GMY.imuEncorderDiff,160+GMY.imuEncorderDiff);//limit
	//CMR.Target =  -CML.Target;
}

#endif // GUARD == 'D'


