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
RampGen_t LRSpeedRamp = RAMP_GEN_DAFAULT;   	//斜坡函数
RampGen_t FBSpeedRamp = RAMP_GEN_DAFAULT;
int32_t auto_counter= 0;		//用于准确延时的完成某事件
int16_t channelrrow,channelrcol,channellrow,channellcol;
int8_t StateSway,StateFlee,StateHurt,StateRand=1,ExtCmd,ExtCmd2;
int16_t StateCnt=1,CmdTic;
int16_t noEnemyCnt=1;
void strategyShoot(void);
void strategyShoot2(void);
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
uint8_t FindEnemy;
//********************上平台代码1
void RCProcess1(Remote *rc){
	generalProcess(rc);
	if(getLeftSr()){onLed(7);}
	else{offLed(7);}
	if(WorkState == STATE_1){
		brakeOff();
	}
	if(WorkState == STATE_2){
		//brakeOn();
	}
	if(WorkState == STATE_3){
		
	}
	limtSync();
}
//********************上平台代码2
void RCProcess2(Remote *rc){
	generalProcess(rc);
	if(WorkState == STATE_2){
		switch(ExtCmd){
			case 1:{//躲飞机
				if(Anchor==0 || Anchor==1){routing1();}
				else{routing3();}
				break;
			}
			case 2:{
				if(Anchor==0 || Anchor==2){routing2();}
				else{routing3();}
				break;
			}
			default:{
				if(Anchor==0){routing0();}
				else if(Anchor==1){routing1();}
				else if(Anchor==2){routing4();}
			}
		}
	}
	if(WorkState == STATE_3){
		routing0();
//		switch(ExtCmd){
//			case 1:{//躲飞机
//				if(Anchor==0 || Anchor==1){routing1();}
//				else{routing3();}
//				break;
//			}
//			case 2:{//躲碉堡
//				if(Anchor==0 || Anchor==2){routing2();}
//				else{routing3();}
//				break;
//			}
//			default:{
//				if(Anchor==0){routing0();}
//				else if(Anchor==1){routing1();}
//				else if(Anchor==2){routing4();}
//			}
//		}
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
	if(WorkState == STATE_3){
		
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
	FindEnemy=(uint8_t)receiveData[0].data[0];
	
	sendData[0].data[0]=(uint16_t)RCRightMode | (uint16_t)RCLeftMode<<2 | ((uint16_t)CMA.RxMsgC6x0.rotateSpeed/8+127)<<8;//右拨杆，左拨杆，底盘速度
	if(GameRobotState.robot_id==7){sendData[0].data[0]=sendData[0].data[0] | 1<<4;}//是否为蓝
	if(channelrcol>600){sendData[0].data[0]=sendData[0].data[0] | 1<<5;}//右推到顶
	if(ExtCmd2){sendData[0].data[0]=sendData[0].data[0] | 1<<6;ExtCmd2--;}//看碉堡
	if(StateHurt){sendData[0].data[0]=sendData[0].data[0] | 1<<7;StateHurt--;}//看基地
	sendData[0].data[1]=(uint16_t)(channellrow+660)/6 | ((uint16_t)(channellcol+660)/6)<<8 ;
	sendData[0].data[2]=(int16_t)CMA.Real;
	sendData[0].data[3]=(int16_t)(RealHeat0*20);//热量
	
	nutDetect();
}
void limtSync(){
	MINMAX(GMP.Target,-60,0);//limit
//	MINMAX(GMY.Target,-160+GMY.imuEncorderDiff,160+GMY.imuEncorderDiff);//limit
	//CMR.Target =  -CML.Target;
}
#endif  //GUARD == 'U'
#if GUARD == 'D'
//下平台代
float ChaSpdSin,ChaSpdCos;
int8_t tmp;
void generalProcess();
void RCProcess1(){
	generalProcess();
	laserOn();
	if(WorkState == STATE_1){
		STIRv.Target=0;
		FRICL.Target=0;
		FRICR.Target=0;
//		vy+=imu.vy;
//		GMY.Target-=vy/rate;
	}
	if(WorkState == STATE_2){
		STIRv.Target=0;
		FRICL.Target=-0;
		FRICR.Target= 0;
		uartSend();
		if(FindEnemy){autoAim();}
	}
	if(WorkState == STATE_3){
		FRICL.Target =-5500;
		FRICR.Target = 5500;
		uartSend();
		strategyShoot2();
	}
	limtSync();
}
//*******************下平台代码2
void RCProcess2(){
	generalProcess();
	if(WorkState == STATE_1){
		laserOn();
		STIRv.Target=0;
		FRICL.Target=0;
		FRICR.Target=0;
	}
	if(WorkState == STATE_2){
		laserOn();
		STIRv.Target=0;
		FRICL.Target=-0;
		FRICR.Target= 0;
		uartSend();
		if(FindEnemy){autoAim();}
	}
	if(WorkState == STATE_3){
		laserOff();
		strategyShoot2();
		uartSend();
	}
	limtSync();
}

//*******************下平台代码3
void RCProcess3(){
	laserOn();
	generalProcess();
	if(WorkState == STATE_1){
		FRICL.Target =-4200;
		FRICR.Target = 4200;
		uartSend();
		if(FindEnemy){autoAim();}
		if(receiveData[0].data[0] & 0x20){firing3();}
		else{firing1();}
	}
	if(WorkState == STATE_2){
		FRICL.Target =-5500;
		FRICR.Target = 5500;
		uartSend();
		AimMode=0;
		if(FindEnemy){autoAim();}
		if(receiveData[0].data[0] & 0x20){firing2();}
		else{firing1();}
	}
	if(WorkState == STATE_3){
		FRICL.Target =-6600;
		FRICR.Target = 6600;
		uartSend();
		AimMode=5;
		if(FindEnemy){autoAim();}
		if(receiveData[0].data[0] & 0x20){firing5m();}
		else{firing1();}
	}
	limtSync();
}

//打击策略1==================
void strategyShoot(){
	FRICL.Target =-5500;
	FRICR.Target = 5500;
	laserOn();
	if(receiveData[0].data[0] & 0x40){
		aimAtBox();
		noEnemyCnt=80;
	}
	if(FindEnemy){
		if(aim.dis==0){noEnemyCnt=-2;}
		else{autoAim();}
		if(fabs(GMY.Real-opt.yaw)<1.5 && fabs(GMP.Real-opt.pit)<1.5 && aim.dis>500){firing2();}
		if(aim.dis==500){noEnemyCnt=100;}
		else{noEnemyCnt=240;}
		sendData[0].data[0]=(int16_t)1;
	}
	else if(noEnemyCnt<1){
		scaning1();
		sendData[0].data[0]=(int16_t)0;
	}
	else if(noEnemyCnt>150){
		noEnemyCnt--;
		if(fabs(GMY.Real-opt.yaw)<1.5 && fabs(GMP.Real-opt.pit)<1.5 && aim.dis>500){firing2();}
		else{STIRv.Target=0;}
	}
	else if(noEnemyCnt>80){
		noEnemyCnt--;
		STIRv.Target=0;
	}
	else{
		GMY.Target+=0.2;
		noEnemyCnt--;
		STIRv.Target=0;
	}
}
//打击策略5m外高速低频=====================
void strategyShoot2(){
	if(AimMode){
		FRICL.Target =-6600;
		FRICR.Target = 6600;
	}else{
		FRICL.Target =-5500;
		FRICR.Target = 5500;
	}
	if((receiveData[0].data[0] & 0x40) && noEnemyCnt<100){//看碉堡
		aimAtBox();
		noEnemyCnt=250;
		AimMode=0;
	}
	if((receiveData[0].data[0] & 0x80) && (noEnemyCnt<100 || aim.dis==6000)&& GMY.encoderAngle>0){//看基地
		aimAtBase();
		noEnemyCnt=250;
		AimMode=0;
	}
	if(FindEnemy){
		if(aim.dis==0){noEnemyCnt=-2;}
		else{autoAim();}
		if(fabs(GMY.Real-opt.yaw)<1.5 && fabs(GMP.Real-opt.pit)<1.5 && aim.dis>500){
			if(AimMode){firing5m();}
			else{firing2();}
		}
		if(aim.dis==500){noEnemyCnt=100;}
		else{noEnemyCnt=290;}
		sendData[0].data[0]=(int16_t)1;
	}
	else if(noEnemyCnt<1){
		AimMode=0;
		scaning1();
		sendData[0].data[0]=(int16_t)0;
	}
	else if(noEnemyCnt>150){
		noEnemyCnt--;
		if(fabs(GMY.Real-opt.yaw)<1.5 && fabs(GMP.Real-opt.pit)<1.5 && aim.dis>500){
			if(AimMode){
				if(opt.pit>-7){firing5m();}
				else{firing5m2();}
			}
			else{firing2();}
		}
		else{STIRv.Target=0;}
	}
	else if(noEnemyCnt>80){
		noEnemyCnt--;
		STIRv.Target=0;
	}
	else{
		GMY.Target+=0.2;
		noEnemyCnt--;
		STIRv.Target=0;
	}
}
////打击策略5m外高速低频=====================
//void strategyShoot2(){
//	if(AimMode){
//		FRICL.Target =-6600;
//		FRICR.Target = 6600;
//	}
//	else{
//		FRICL.Target =-5500;
//		FRICR.Target = 5500;
//	}
//	if(receiveData[0].data[0] & 0x40){
//		aimAtBox();
//		noEnemyCnt=-100;
//		AimMode=0;
//	}
//	if(FindEnemy){
//		if(aim.dis==0){noEnemyCnt=2;}
//		else{autoAim();}
//		if(fabs(GMY.Real-opt.yaw)<1.5 && fabs(GMP.Real-opt.pit)<1.5 && aim.dis>500){
//			if(AimMode){firing5m();}
//			else{firing2();}
//		}
//		if(aim.dis==500){noEnemyCnt=-50;}
//		else{noEnemyCnt=-350;}
//		sendData[0].data[0]=(int16_t)1;
//	}
//	else if(noEnemyCnt>1){
//		scaning1();
//		sendData[0].data[0]=(int16_t)0;
//	}
//	else if(noEnemyCnt<-250){
//		noEnemyCnt++;
//		if(fabs(GMY.Real-opt.yaw)<1.5 && fabs(GMP.Real-opt.pit)<1.5 && aim.dis>500){
//			if(AimMode){firing5m();}
//			else{firing2();}
//		}
//		else{STIRv.Target=0;}
//	}
//	else if(noEnemyCnt<-120){
//		noEnemyCnt++;
//		STIRv.Target=0;
//	}
//	else{
//		GMY.Target+=0.2;
//		noEnemyCnt++;
//		STIRv.Target=0;
//	}
//}
//===================================
//*******************下平台通用代码
//===================================

void generalProcess(){
	offLed(5);
	if(WorkState <= 0) return;
	//max=660
	channelrrow = 0;channelrcol = 0;
	channellrow = -(int16_t)((uint16_t)(receiveData[0].data[1] & 0xff)*3)+330;//leftRight
	channellcol = (int16_t)((uint16_t)(receiveData[0].data[1])>>8)*3-330;//upDown
	GMY.Target+=channellrow*0.001f;
	GMP.Target+=channellcol*0.001f;
	CMA.Real=receiveData[0].data[2];
	RealHeat0=receiveData[0].data[3]/(float)(20.0);
	//基于顶盘速度的运动补偿
	ChaSpdSin=((int16_t)((uint16_t)(receiveData[0].data[0])>>8) -127) * sin((GMY.encoderAngle)/57.3);
	ChaSpdCos=((int16_t)((uint16_t)(receiveData[0].data[0])>>8) -127) * cos((GMY.encoderAngle)/57.3)*sin(GMP.Real/57.3);
	float SinPit=sin(GMP.Real/57.3),tmpY,tmpP;
	if(GMY.encoderAngle>0){
		tmpY=ChaSpdSin * SinPit/380;//375;//460;400
		tmpP=ChaSpdCos * SinPit/380;//360;380
	}else{
		tmpY=ChaSpdSin * SinPit/310;//375;//460;
		tmpP=ChaSpdCos * SinPit/310;//360;
	}
	GMY.Target+=tmpY;
	GMP.Target+=tmpP;
	opt.yaw+=tmpY;
	opt.pit+=tmpP;
	if(receiveData[0].data[0] & 0x80)tmp=1;
}

void limtSync(){
	MINMAX(GMP.Target,-60,0);
}

#endif // GUARD == 'D'


