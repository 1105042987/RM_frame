/**
  ******************************************************************************
  * File Name          : FunctionLib.c
  * Description        : 哨兵功能函数集
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

float shootFrq=20;
double chassisAdd;
extern float fakeHeat0;

static float swayRef;
int8_t stateSway=0;
int8_t stateFlee=0;
int stateCnt=1;
int8_t oneShootFlag=0;
int8_t noEnemyCnt=1;


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
	static float speedRef;
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



