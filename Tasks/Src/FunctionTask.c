/**
  ******************************************************************************
  * File Name          : FunctionTask.c
  * Description        : ���ڼ�¼�����˶��еĹ���
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
RampGen_t LRSpeedRamp = RAMP_GEN_DAFAULT;   	//б�º���
RampGen_t FBSpeedRamp = RAMP_GEN_DAFAULT;
int32_t auto_counter= 0;		//����׼ȷ��ʱ�����ĳ�¼�
int16_t channelrrow,channelrcol,channellrow,channellcol;
int8_t StateSway,StateFlee,StateHurt,StateRand=1;
int16_t StateCnt=1;
int16_t noEnemyCnt=1;
void strategyShoot(void);
//��ʼ��
void FunctionTaskInit(){
	ChassisSpeed=0;
}
//��λ��ͬ��

//******************
//ң����ģʽ���ܱ�д
//******************
#if GUARD == 'U'
void generalProcess(Remote *rc);
uint8_t findEnemy;
//********************��ƽ̨����1
void RCProcess1(Remote *rc){
	generalProcess(rc);
	if(getLeftSw()){onLed(6);}
	else{offLed(6);}
	if(getLeftSr()){onLed(7);}
	else{offLed(7);}
	if(WorkState == STATE_1){

	}
	if(WorkState == STATE_2){

	}
	if(WorkState == STATE_3){
		
	}
	limtSync();
}
//********************��ƽ̨����2
void RCProcess2(Remote *rc){
	generalProcess(rc);
	if(WorkState == STATE_1){
		int8_t dir=sgn(channelrrow);
		onePushDir(dir,powLmtCnt=500);
	}
	if(WorkState == STATE_2){
		routing3();
	}
	if(WorkState == STATE_3){
		routing3();
	}
	limtSync();
}
//********************��ƽ̨����3
void RCProcess3(Remote *rc){
	generalProcess(rc);
	if(WorkState == STATE_1){
		
	}
	if(WorkState == STATE_2){
		
	}
	limtSync();
}


//===================================
//********************��ƽ̨ͨ�ô���
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
	
	sendData[0].data[0]=(int16_t)WorkState | (int16_t)RCRightMode<<4;
	uint8_t upSpd=CMA.RxMsgC6x0.rotateSpeed/8+127;
	sendData[0].data[0]=sendData[0].data[0] | ((uint16_t)upSpd)<<8;
	
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
//��ƽ̨����
float vx,dx;//����imu��α����
void generalProcess();
void RCProcess1(){
	generalProcess();
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
//*******************��ƽ̨����2
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

//*******************��ƽ̨����3
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
		//�������ݻ��о�������ĸ��ǣ����²���
		GMY.Target-=(((uint16_t)(receiveData[0].data[0])>>8) -127)/1100.0;
		GMP.Target-=dx*15;
		if(fabs(aim.yaw)<8 && aim.dis==0){firing2();}
		if(aim.dis==500){noEnemyCnt=1;}
		else{noEnemyCnt=-300;}
		sendData[0].data[0]=(int16_t)1;
	}
	else if(noEnemyCnt>1){
		scaning1();
		sendData[0].data[0]=(int16_t)0;
	}
	else if(noEnemyCnt<-200){
		noEnemyCnt++;
		if(fabs(aim.yaw)<6 && aim.dis==0){firing2();}
		else{STIRv.Target=0;}
	}
	else{
		noEnemyCnt++;
		STIRv.Target=0;
	}
}

//===================================
//*******************��ƽ̨ͨ�ô���
//===================================
void generalProcess(){
	offLed(5);
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
	//���ڶ����ٶȵ�yaw�˶�����
	GMY.Target+=channellrow*0.001f-(((uint16_t)(receiveData[0].data[0])>>8) -127)/1100.0;
	GMP.Target+=channellcol*0.001f;
	//����imu��pit�˶����� 
	dx=imu.vx-dx;
	vx+=dx;
	vx*=0.99;
	GMP.Target-=dx*15;
	dx=imu.vx;
}

void limtSync(){
	MINMAX(GMP.Target,-60,0);//limit
//	MINMAX(GMY.Target,-160+GMY.imuEncorderDiff,160+GMY.imuEncorderDiff);//limit
	//CMR.Target =  -CML.Target;
}

#endif // GUARD == 'D'


