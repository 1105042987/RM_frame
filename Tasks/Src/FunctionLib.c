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
float SwayRef;

int sgn(float x){return x>0?1:(x<0?-1:0);}


void remv1(){
	StateFlee=0;
	StateSway=0;
	StateCnt=0;
}
void remv2(){
	StateFlee=0;
	StateSway=0;
	StateCnt=0;
	CML.Target=CML.Real;
	CMR.Target=CMR.Real;
}


void routing(){
	static int dir=1;
	if(getLeftSr() || CMA.Real>-10){//换向：红外触发、底盘超程
		dir=-1;
		CMA.Real=-20;
	}
	else if(getRightSr() || CMA.Real<-480){
		dir=1; 
	}
	if(PowerHeat.chassis_power_buffer>100){ChassisAdd =600*dir;}
	else{ChassisAdd =450*dir;}
}
void routing2(){
	static int dir=-1;
	if(getLeftSr() || CMA.Real>-70){//换向
		dir=-1;
	}
	else if(getRightSr() || CMA.Real<-410){
		dir=1;
	}
	if(PowerHeat.chassis_power_buffer>100){ChassisAdd =600*dir;}
	else{ChassisAdd =450*dir;}
}



void fleeing1(){
	static int8_t dir=0,lock=0;
	static int16_t pos,tgt;
	pos=CMA.Real;
	//125,252,370,488
	if(!lock){
		if(GMY.encoderAngle<0){tgt=-125;dir=sgn(tgt-pos);}
		else{tgt=-370;dir=sgn(tgt-pos);}
	}
	ChassisAdd=1200*dir;
	if(abs(pos-tgt)<8){ChassisAdd*=0.5;}
	if(abs(pos-tgt)<4){
		StateFlee=0;
		StateSway=1;
		SwayRef=0;
	}
	StateCnt++;
	if(getRightSr() || getLeftSr()){remv2();}
}
void fleeing2(){
	static int8_t dir=0;
	static int16_t pos,tgt;
	pos=CMA.Real;
	//125,252,370,488
	if(pos>-125){dir=-1;tgt=-125;}
	else if(pos>-250){dir=1;tgt=-125;}
	else if(pos>-375){dir=-1;tgt=-375;}
	else{dir=1;tgt=-375;}
	
	if(StateFlee==2){
		ChassisAdd=1800*dir;
	}
	else if(StateFlee==1){
		ChassisAdd=1200*dir;
	}
	if(abs(pos-tgt)<8){ChassisAdd*=0.5;}
	if(abs(pos-tgt)<4){
		StateSway=1;
		SwayRef=0;
	}
	StateCnt++;
	if(getRightSr() || getLeftSr()){remv2();}
}
void fleeing3(){
	static int8_t dir=0,lock=0;
	static int16_t pos,tgt;
	pos=CMA.Real;
	//125,252,375,490
	if(!lock){
		if(pos>-80){dir=-1;tgt=-125;}
		else if(pos<-400){dir=1;tgt=-375;}
		else if(pos>-250){dir=-1;tgt=-375;}
		else{dir=1;tgt=-110;}
		lock=1;
	}
	ChassisAdd=1800*dir;
	if(abs(pos-tgt)<8){ChassisAdd*=0.5;}
	if(abs(pos-tgt)<4){
		StateFlee=0;
		StateSway=1;
		SwayRef=0;
		lock=0;
	}
	StateCnt++;
	if(getRightSr() || getLeftSr()){remv2();lock=0;}
}




void tossing(int8_t dir,int16_t tgt){
	int16_t pos=CMA.Real;
	ChassisAdd=1800*dir;
	if(abs(pos-tgt)<8){ChassisAdd*=0.5;}
	if(abs(pos-tgt)<4){
		StateFlee=0;
		StateSway=1;
		SwayRef=0;
	}
}


void swaying(){
	ChassisAdd=950*cos(SwayRef);
	SwayRef+=0.028f;//0.016= 0.087965*0.5= 2*3.1416*0.014 *0.5
	StateCnt++;
	if(getRightSr() || getLeftSr()){remv2();}
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


void randing1(int8_t spd){
	static int8_t dir=0,sspd=-1,cnt=1;
	if(getLeftSr()){
		sspd=-1;
		CMA.Real=-20;
		CML.Target=CML.Real;
		CMR.Target=CMR.Real;
		ChassisAdd=-2000;
		return;
	}
	else if(getRightSr()){
		sspd=-1;
		CMA.Real=-480;
		CML.Target=CML.Real;
		CMR.Target=CMR.Real;
		ChassisAdd=2000;
		return;
	}
	static int16_t pos,tgt,tgtLast;
	pos=CMA.Real;
	//125,252,370,488
	if(sspd!=spd){
		tgt=-rand()%45*10-20;
		while(abs(tgt-tgtLast)>160 || abs(tgt-tgtLast)<60){tgt=-rand()%45*10-20;}
		tgtLast=tgt;
		dir=sgn(tgt-pos);
		sspd=spd;
		CML.Target=CML.Real;
		CMR.Target=CMR.Real;
	}
	if(spd>5){cnt=1;}
	
	if(cnt<0){
		cnt++;
		ChassisAdd=0;
		return;
	}
	ChassisAdd=(spd*100+700)*dir;
	if(abs(pos-tgt)<8){ChassisAdd*=0.5;}
	if(abs(pos-tgt)<4){sspd=-1;cnt=-10;}
}


void randing2(int8_t spd){
	static int8_t dir=0,sspd=-1,pauseCnt=1,aeriaLock;
	static int16_t pos,tgt,tgtLast,lockCnt;
	if(getLeftSr()){
		sspd=-1;
		CMA.Real=-20;
		CML.Target=CML.Real;
		CMR.Target=CMR.Real;
		ChassisAdd=-2000;
		return;
	}
	else if(getRightSr()){
		sspd=-1;
		CMA.Real=-480;
		CML.Target=CML.Real;
		CMR.Target=CMR.Real;
		ChassisAdd=2000;
		return;
	}
	pos=CMA.Real;
	//125,252,370,488
	if(sspd!=spd){
		tgt=-rand()%45*10-20;
		while(abs(tgt-tgtLast)>160 || abs(tgt-tgtLast)<60){tgt=-rand()%45*10-20;}
		if(aeriaLock){
			while(tgt>-120 || abs(tgt-tgtLast)>160 || abs(tgt-tgtLast)<60){tgt=-rand()%45*10-20;}
		}
		tgtLast=tgt;
		dir=sgn(tgt-pos);
		sspd=spd;
		CML.Target=CML.Real;
		CMR.Target=CMR.Real;
		if(spd>9){aeriaLock=1;}
	}
	if(spd>5){pauseCnt=1;}
	if(pauseCnt<0){
		pauseCnt++;
		ChassisAdd=0;
		return;
	}
	if(aeriaLock){
		lockCnt++;
		if(lockCnt>15000){
			aeriaLock=0;
			lockCnt=0;
		}
	}
	ChassisAdd=(spd*100+700)*dir;
	if(abs(pos-tgt)<8){ChassisAdd*=0.5;}
	if(abs(pos-tgt)<4){sspd=-1;pauseCnt=-10;}
}





