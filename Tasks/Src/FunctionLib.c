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
int sgn(float x){return x>0?1:(x<0?-1:0);}
int8_t preventAnchor();
int8_t NutCnt=8;
int16_t SpeedRef=2100;
float yawZero;

void routing0(){//全程
//100,210,315,425,530,610
	static int8_t dir=-1,dirCnt=5;
	LimitRate=1;
	dir*=preventAnchor();
	if(getRightSr()){//撞墙
		if(dirCnt){dirCnt--;LimitRate=0;}
		dir=1;
		powLmtCnt=50;
		onLed(7);
		CMA.Real=0;
		NutCnt=0;
	}else if(getLeftSr()){//飞机侧
		if(dirCnt){dirCnt--;LimitRate=0;}
		dir=-1;
		powLmtCnt=50;
		onLed(7);
		CMA.Real=-610;
		NutCnt=8;
	}else{
		offLed(7);
		dirCnt=5;
	}
//	if(NutCnt==0 && dir==-1){LimitRate=0;}
//	else if(NutCnt==8 && dir==1){LimitRate=0;}

	brakeOff();
	ChassisSpeed=SpeedRef*dir;
}

void routing1(){//躲飞机
//100,210,315,425,530,610
	static int8_t dir=-1,dirCnt=5;
	LimitRate=1;
	dir*=preventAnchor();
	if(getRightSr()){
		if(dirCnt){dirCnt--;LimitRate=0;}
		dir=1;
		powLmtCnt=50;
		onLed(7);
		CMA.Real=0;
		NutCnt=0;
	}else{
		offLed(7);
		dirCnt=5;
	}
	//if(NutCnt==0 && dir==-1 && CMA.RxMsgC6x0.rotateSpeed>20){LimitRate=0;}
	if(NutCnt==8 || CMA.Real<-540){dir=-1;powLmtCnt=50;}
	if(CMA.RxMsgC6x0.rotateSpeed<-20 && dir==-1){brakeOn();}//刹车
	else{brakeOff();}
	ChassisSpeed=SpeedRef*dir;
}
void routing2(){//躲碉堡
//100,210,315,425,530,610
	static int8_t dir=1,dirCnt=5;
	LimitRate=1;
	dir*=preventAnchor();
	if(getLeftSr()){
		if(dirCnt){dirCnt--;LimitRate=0;}
		dir=-1;
		powLmtCnt=50;
		onLed(7);
		CMA.Real=-610;
		NutCnt=8;
	}else{
		offLed(7);
		dirCnt=5;
	}
	if(NutCnt==6 || CMA.Real>-500){dir=1;powLmtCnt=50;}
	if(CMA.RxMsgC6x0.rotateSpeed>20 && dir==1){brakeOn();}//刹车
	else{brakeOff();}
	ChassisSpeed=SpeedRef*dir;
}

void routing3(){//保护模式
//100,210,315,425,530,610
	static int8_t dir=-1,dirCnt=5;
	LimitRate=1;
	dir*=preventAnchor();
	if(getRightSr()){
		if(dirCnt){dirCnt--;LimitRate=0;}
		dir=1;
		powLmtCnt=50;
		onLed(7);
		CMA.Real=0;
		NutCnt=0;
	}else{
		offLed(7);
		dirCnt=5;
	}
	if(NutCnt==0 && dir==-1){LimitRate=0;}
	
	if(NutCnt==8 || CMA.Real<-480){dir=-1;powLmtCnt=50;}
	if(CMA.RxMsgC6x0.rotateSpeed*dir>0 && CMA.Real<-200 && abs(CMA.RxMsgC6x0.rotateSpeed)>10){brakeOn();}//刹车
	else{brakeOff();}
	ChassisSpeed=SpeedRef*dir;
}

void routing10(){//随机换向,不使用
//100,210,315,425,530,610
	static int8_t dir=-1,dirCnt=2,randOpen=1;
	LimitRate=1;
	dir*=preventAnchor();
	if(getRightSr()){//撞墙
		if(dirCnt){dirCnt--;LimitRate=0;}
		dir=1;
		powLmtCnt=50;
		onLed(7);
		CMA.Real=0;
		NutCnt=0;
		randOpen=1;
	}else{
		offLed(7);
		dirCnt=2;
	}
	if(NutCnt==0 && dir==-1){LimitRate=0;}
	if(CMA.Real<-210 && CMA.Real>-310 && randOpen){//随机折返
		static uint8_t timDiv;
		if(timDiv<120){timDiv++;}
		else if(rand()%20<2){
			dir*=-1;
			powLmtCnt=50;
			randOpen=0;
			timDiv=0;
		}
	}
	if(NutCnt==8 || CMA.Real<-460 ||getLeftSr()){
		dir=-1;
		powLmtCnt=50;
		randOpen=1;
	}
	if(CMA.RxMsgC6x0.rotateSpeed*dir>0 && CMA.Real<-200 && abs(CMA.RxMsgC6x0.rotateSpeed)>10){brakeOn();}//刹车
	else{brakeOff();}
	ChassisSpeed=SpeedRef*dir;
}



void nutDetect(){
	static int8_t nutOpen=1;
	if(getMidSr()){
		if(nutOpen){
			if(CMA.RxMsgC6x0.rotateSpeed<0){NutCnt++;}
			else{NutCnt--;}
		}
		nutOpen=0;
	}else{nutOpen=1;}
}
int8_t preventAnchor(){
	static int16_t anchorCnt=0;
	if(CMA.RxMsgC6x0.rotateSpeed==0 && CMR.RxMsgC6x0.rotateSpeed==0){anchorCnt++;}
	else{anchorCnt=0;}
	if(anchorCnt>10){return -1;}
	else return 1;
}


void scaning1(){
	static float ref;
	STIRv.Target=0;
	GMY.Target-=0.3;
	GMP.Target=-20+10*sin(ref);
	ref+=0.02;
}

void scaning2(){
	static float ref;
	STIRv.Target=0;
	if(receiveData[0].data[1]>4000){//enemy blue
		GMY.Target-=0.32;
		GMP.Target=-30+15*sin(ref);
		ref+=0.015;
	}
	else{//enemy red
		GMY.Target-=0.25;
		GMP.Target=-30+10*sin(ref);
		ref+=0.03;
	}
}
void scaning3(){
	static float tic=0;
	STIRv.Target=0;
	if(tic>150){
		GMY.Target-=40;
		tic=0;
	}
	GMP.Target=-25;
	tic++;
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
	static float ShootFrq=20;
	ShootFrq=(440-RealHeat0)*0.1;
	MINMAX(ShootFrq,0,22);
	if(STIRv.RxMsgC6x0.moment>8000){jam=30;}
	///TargetSpeed = ShootFrq*45/360*36*60 = ShootFrq*270
	if(jam<0){STIRv.Target=ShootFrq *270;}
	else{STIRv.Target=-3500;jam--;}
}
void firing3(){//发光蛋测试
	static int8_t jam=-1;
	static float ShootFrq=20;
	ShootFrq=(440-RealHeat0)*0.1;
	MINMAX(ShootFrq,0,19);
	if(STIRv.RxMsgC6x0.moment>8000){jam=30;}
	///TargetSpeed = ShootFrq*45/360*36*60 = ShootFrq*270
	if(jam<0){STIRv.Target=ShootFrq *270;}
	else{STIRv.Target=-3500;jam--;}
}

uint8_t msgRed[]="1\n",msgBlue[]="2\n";
void uartSend(){
	static int16_t cnt;
	if(cnt>500){
		if(receiveData[0].data[0] & 0x10){HAL_UART_Transmit_IT(&AUTOAIM_UART,(uint8_t*)msgBlue,2);}
		else{HAL_UART_Transmit_IT(&AUTOAIM_UART,(uint8_t*)msgRed,2);}
		cnt=0;
	}
	cnt++;
}
