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
float ShootFrq=20;

int sgn(float x){return x>0?1:(x<0?-1:0);}

void routing1(){
//-100,210,315,425,530,610
	static int8_t dir=-1,dirCnt=10;
	LimitRate=1;
	
	if(getLeftSw()){
		if(dirCnt){dirCnt--;LimitRate=0;}
		else{dir=1;LimitCnt=500;}
		onLed(7);
		CMA.Real=0;
		
	}else{
		offLed(7);
		dirCnt=10;
	}
	
	if(CMA.Real<-350 ||getRightSr()){dir=-1;LimitCnt=500;}
	ChassisSpeed=2400*dir;
}


void scaning1(){
	static float ref;
	STIRv.Target=0;
	GMY.Target-=0.32;
	GMP.Target=-10+15*sin(ref);
	ref+=0.015;
}
void scaning2(){
	static float ref;
	STIRv.Target=0;
	if(receiveData[0].data[1]>4000){//enemy blue
		GMY.Target-=0.32;
		GMP.Target=-10+15*sin(ref);
		ref+=0.015;
	}
	else{//enemy red
		GMY.Target-=0.25;
		GMP.Target=-12+10*sin(ref);
		ref+=0.03;
	}
}



void firing1(){
	static int8_t jam=-1;
	if(STIRv.RxMsgC6x0.moment>8000){jam=30;}
	///TargetSpeed = ShootFrq*45/360*36*60 = ShootFrq*270
	if(jam<0){STIRv.Target=(1+channelrcol/66) *270;}
	else{STIRv.Target=-3500;jam--;}
}
void firing2(){
	static int8_t jam=-1;
	
	ShootFrq=(440-realHeat0)*0.1;
	
	MINMAX(ShootFrq,0,22);
//	if(realHeat0<100){ShootFrq=25;}//40
//	else if(realHeat0<200){ShootFrq=20;}//30
//	else if(realHeat0<300){ShootFrq=12;}//20
//	else if(realHeat0<360 && ShootFrq<8){ShootFrq+=0.02f;}//12
//	else if(realHeat0>440 && ShootFrq>5){ShootFrq--;}
//	else if(realHeat0>400 && ShootFrq>7){ShootFrq-=0.01f;}
//	else{ShootFrq=8;}
	
	if(STIRv.RxMsgC6x0.moment>8000){jam=30;}
	///TargetSpeed = ShootFrq*45/360*36*60 = ShootFrq*270
	if(jam<0){STIRv.Target=ShootFrq *270;}
	else{STIRv.Target=-3500;jam--;}
}
void firing3(){
	static int8_t jam=-1;
	if(STIRv.RxMsgC6x0.moment>8000){jam=9;}
	///TargetSpeed = ShootFrq*45/360*36*60 = ShootFrq*270
	if(jam<0){STIRv.Target=(20+channelrcol/66) *270;}
	else{STIRv.Target=-3000;jam--;}
}


