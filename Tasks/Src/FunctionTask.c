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
RampGen_t LRSpeedRamp = RAMP_GEN_DAFAULT;   	//б�º���
RampGen_t FBSpeedRamp = RAMP_GEN_DAFAULT;
int32_t auto_counter= 0;		//����׼ȷ��ʱ�����ĳ�¼�
int16_t channelrrow,channelrcol,channellrow,channellcol;
int8_t StateSway,StateFlee,StateHurt,StateRand=1,ExtCmd,ExtCmd2;
int16_t StateCnt=1,CmdTic;
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
uint8_t FindEnemy;
//********************��ƽ̨����1
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
//********************��ƽ̨����2
void RCProcess2(Remote *rc){
	generalProcess(rc);
	if(WorkState == STATE_1){
		int8_t dir=sgn(channelrrow);
		onePushDir(dir,powLmtCnt=500);
	}
	if(WorkState == STATE_2){
		switch(ExtCmd){
			case 1:{//��ɻ�
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
		switch(ExtCmd){
			case 1:{//��ɻ�
				if(Anchor==0 || Anchor==1){routing1();}
				else{routing3();}
				break;
			}
			case 2:{//��ﱤ
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
	FindEnemy=(uint8_t)receiveData[0].data[0];
	
	sendData[0].data[0]=(uint16_t)RCRightMode | (uint16_t)RCLeftMode<<2 | ((uint16_t)CMA.RxMsgC6x0.rotateSpeed/8+127)<<8;//�Ҳ��ˣ��󲦸ˣ������ٶ�
	if(GameRobotState.robot_id==7){sendData[0].data[0]=sendData[0].data[0] | 1<<4;}//�Ƿ�Ϊ��
	if(channelrcol>600){sendData[0].data[0]=sendData[0].data[0] | 1<<5;}//
	if(ExtCmd2){sendData[0].data[0]=sendData[0].data[0] | 1<<6;ExtCmd2--;}//
	sendData[0].data[1]=(uint16_t)(channellrow+660)/6 | ((uint16_t)(channellcol+660)/6)<<8 ;
	sendData[0].data[2]=(int16_t)CMA.Real;
	sendData[0].data[3]=(int16_t)(RealHeat0*20);//����
	
	nutDetect();
}
void limtSync(){
	MINMAX(GMP.Target,-60,0);//limit
//	MINMAX(GMY.Target,-160+GMY.imuEncorderDiff,160+GMY.imuEncorderDiff);//limit
	//CMR.Target =  -CML.Target;
}
#endif  //GUARD == 'U'
#if GUARD == 'D'
//��ƽ̨��
float ChaSpdSin,ChaSpdCos;
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
		if(FindEnemy){autoAim();}
	}
	if(WorkState == STATE_3){
		FRICL.Target =-5500;
		FRICR.Target = 5500;
		uartSend();
		strategyShoot();
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
		if(FindEnemy){autoAim();}
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
		FRICL.Target =-3300;
		FRICR.Target = 3300;
		uartSend();
		if(FindEnemy){autoAim();}
		if(receiveData[0].data[0] & 0x20){firing3();}
		else{firing1();}
	}
	if(WorkState == STATE_2){
		FRICL.Target =-5500;
		FRICR.Target = 5500;
		uartSend();
		if(FindEnemy){autoAim();}
		if(receiveData[0].data[0] & 0x20){firing2();}
		else{firing1();}
	}
	if(WorkState == STATE_3){//ֹͣλ������������
		FRICL.Target=0;
		FRICR.Target=0;
		STIRv.Target=0;
	}
	limtSync();
}
void strategyShoot(){
	FRICL.Target =-5500;
	FRICR.Target = 5500;
	laserOn();
	if(receiveData[0].data[0] & 0x40){
		aimAtBox();
		noEnemyCnt=-100;
	}
	if(FindEnemy){
		autoAim();
		if(fabs(GMY.Real-opt.yaw)<1.5 && fabs(GMP.Real-opt.pit)<1.5 && aim.dis>500){firing2();}
		if(aim.dis==500){noEnemyCnt=-50;}
		if(aim.dis==0){noEnemyCnt=2;}
		else{noEnemyCnt=-350;}
		sendData[0].data[0]=(int16_t)1;
	}
	else if(noEnemyCnt>1){
		scaning1();
		sendData[0].data[0]=(int16_t)0;
	}
	else if(noEnemyCnt<-250){
		noEnemyCnt++;
		if(fabs(GMY.Real-opt.yaw)<1.5 && fabs(GMP.Real-opt.pit)<1.5 && aim.dis>500){firing2();}
		else{STIRv.Target=0;}
	}
	else if(noEnemyCnt<-120){
		noEnemyCnt++;
		STIRv.Target=0;
	}
	else{
		GMY.Target+=0.2;
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
	channellrow = -(int16_t)((uint16_t)(receiveData[0].data[1] & 0xff)*6)+660;//leftRight
	channellcol = (int16_t)((uint16_t)(receiveData[0].data[1])>>8)*6-660;//upDown
	
	CMA.Real=receiveData[0].data[2];
	RealHeat0=receiveData[0].data[3]/(float)(20.0);
	
	GMY.Target+=channellrow*0.001f;
	GMP.Target+=channellcol*0.001f;
	//���ڶ����ٶȵ��˶�����
	ChaSpdSin=((int16_t)((uint16_t)(receiveData[0].data[0])>>8) -127) * sin((GMY.encoderAngle)/57.3);
	ChaSpdCos=((int16_t)((uint16_t)(receiveData[0].data[0])>>8) -127) * cos((GMY.encoderAngle)/57.3)*sin(GMP.Real/57.3);
	float SinPit=sin(GMP.Real/57.3);
	float tmpY=ChaSpdSin * SinPit/400;//460;
	float tmpP=ChaSpdCos * SinPit/400;
	GMY.Target+=tmpY;
	GMP.Target+=tmpP;
	opt.yaw+=tmpY;
	opt.pit+=tmpP;
	
}

void limtSync(){
	MINMAX(GMP.Target,-60,0);
}

#endif // GUARD == 'D'


