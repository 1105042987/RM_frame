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
#define getLeftSr()		!HAL_GPIO_ReadPin(GPIOE,leftSensor_Pin)// //�����⵽Ϊ�͵�ƽ����ȡ�����㡣Sr��дSensor
#define getRightSr()	!HAL_GPIO_ReadPin(GPIOE,rightSensor_Pin) //


double ChassisAdd;
extern float fakeHeat0;
RampGen_t LRSpeedRamp = RAMP_GEN_DAFAULT;   	//б�º���
RampGen_t FBSpeedRamp = RAMP_GEN_DAFAULT;

int32_t auto_counter= 0;		//����׼ȷ��ʱ�����ĳ�¼�
int16_t channelrrow = 0;
int16_t channelrcol = 0;
int16_t channellrow = 0;
int16_t channellcol = 0;

int8_t oneShootFlag=0;
int8_t StateSway=0;
int8_t StateFlee=0;
int8_t StateHurt=0;
int8_t StateRand=1;
int16_t StateCnt=1;
int16_t noEnemyCnt=1;
void strategyAct(void);
void strategyRand(void);
void strategyShoot(void);
void uartSend(int8_t i);
void uartSend2(void);
//��ʼ��
void FunctionTaskInit(){
	ChassisAdd=0;
}
//��λ��ͬ��
void limtSync(){
	MINMAX(GMP.Target,-42,20);//limit
//	MINMAX(GMY.Target,-160+GMY.imuEncorderDiff,160+GMY.imuEncorderDiff);//limit
	//CMR.Target =  -CML.Target;
}
//******************
//ң����ģʽ���ܱ�д
//******************
#if GUARD == 'U'
uint8_t findEnemy;
//��ƽ̨����
void RemoteControlProcess(Remote *rc){
	if(WorkState <= 0) return;
	//max=660
	channelrrow = (rc->ch0 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET);//leftRight
	channelrcol = (rc->ch1 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET);//upDown
	channellrow = (rc->ch2 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET);
	channellcol = (rc->ch3 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET);

	ChassisAdd=-channelrrow*2;
	sendData[0].data[0]=(int16_t)WorkState | (int16_t)inputmode<<8;
	
	if(GameRobotState.robot_id==7){sendData[0].data[1]=channellrow+5000;}
	else{sendData[0].data[1]=channellrow;}
	
	if(channelrcol>600){sendData[0].data[2]=channellcol+5000;}
	else{sendData[0].data[2]=channellcol;}
	
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
//********************��ƽ̨����2
void selfControlProcess(Remote *rc){
	if(WorkState <= 0) return;
	//max=660
	channelrrow = (rc->ch0 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET);
	channelrcol = (rc->ch1 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET);
	channellrow = (rc->ch2 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET);
	channellcol = (rc->ch3 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET);
	
	findEnemy=(uint8_t)receiveData[0].data[0];
	sendData[0].data[0]=(int16_t)WorkState | (int16_t)inputmode<<8;
//	sendData[0].data[1]=channellrow;
//	sendData[0].data[2]=channellcol;
	if(GameRobotState.robot_id==7){sendData[0].data[1]=channellrow+5000;}
	else{sendData[0].data[1]=channellrow;}
	
	if(channellcol>600){sendData[0].data[2]=channellcol+5000;}
	else{sendData[0].data[2]=channellcol;}
	
	sendData[0].data[3]=(int16_t)(realHeat0*20);
	if(WorkState == NORMAL_STATE){
		ChassisAdd=-channelrrow*2;
	}
	if(WorkState == ADDITIONAL_STATE_ONE){
		remv2();
		//routing();
	}
	if(WorkState == ADDITIONAL_STATE_TWO){
		strategyRand();
	}
	limtSync();
}



void strategyRand(){
	if(findEnemy){
		if(StateFlee){}
		else{StateFlee=1;}
	}
	if(StateFlee>7){randing1(10);}
	else if(StateFlee==1){randing1(2);}
	else if(StateFlee==2){randing1(5);}
	else{randing1(0);}
}
void strategyAct(){
	if(findEnemy){
		if(StateSway||StateFlee){}
		else{StateFlee=1;}
		StateCnt=0;
	}
	if(StateFlee>7){fleeing3();}
	else if(StateSway){swaying();}
	else if(StateFlee==1){fleeing1();}
	else if(StateFlee==2){fleeing2();}
	else{routing();}
	if(StateCnt>400){remv2();}
}


#endif  //GUARD == 'U'
#if GUARD == 'D'
//��ƽ̨����
//int8_t flagFireTest=0;
void RemoteControlProcess(){
	offLed(6);
	if(WorkState <= 0) return;
	//max=660
	channelrrow=0;
	channelrcol=0;
	channellrow= -receiveData[0].data[1];//leftRight
	channellcol= receiveData[0].data[2];//upDown
	
	if(channellrow<-4000){channellrow+=5000;}
	if(channellcol>4000){channellcol-=5000;}
	
	realHeat0=receiveData[0].data[3]/(float)(20.0);
	
	GMY.Target+=channellrow*0.001f;
	GMP.Target+=channellcol*0.001f;
	if(WorkState == NORMAL_STATE){
		oneShootFlag=10;
//		STIRp.Real=0;
//		STIRp.Target=0;
//		if(flagFireTest){
//			FRICL.Target =-5400;
//			FRICR.Target = 5400;
//			firing2();
//		}
//		else{
//			STIRv.Target=0;
//			FRICL.Target=0;
//			FRICR.Target=0;
//		}
		
		STIRv.Target=0;
		FRICL.Target=0;
		FRICR.Target=0;
		laserOn();
	}
	if(WorkState == ADDITIONAL_STATE_ONE){
//		STIRp.Real=0;
//		STIRp.Target=0;
//		if(oneShootFlag){
//			STIRv.Target=2700;oneShootFlag--;}
//		else{STIRv.Target=0;}
		STIRv.Target=0;
		FRICL.Target=-0;
		FRICR.Target= 0;
		laserOn();
		uartSend2();
		if(findEnemy){autoAim();}
	}
	if(WorkState == ADDITIONAL_STATE_TWO){
		FRICL.Target =-5400;
		FRICR.Target = 5400;
		laserOn();
		uartSend2();
		if(findEnemy){autoAim();}
		if(receiveData[0].data[2]>4000){firing2();}
		else{firing1();}
	}
	limtSync();
}
//*******************��ƽ̨����2


void selfControlProcess(){
	offLed(6);
	if(WorkState <= 0) return;
	//max=660
	channelrrow = 0;
	channelrcol = 0;
	channellrow = -receiveData[0].data[1];//leftRight
	channellcol = receiveData[0].data[2];//upDown
	
	if(channellrow<-4000){channellrow+=5000;}
	if(channellcol>4000){channellcol-=5000;}
	
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
//		if(oneShootFlag){STIRv.Target=2700;oneShootFlag--;}
//		else{STIRv.Target=0;}
		STIRv.Target=0;
		FRICL.Target=-0;
		FRICR.Target= 0;
		laserOn();
		uartSend2();
		if(findEnemy){autoAim();}
	}
	if(WorkState == ADDITIONAL_STATE_TWO){
		strategyShoot();
		uartSend2();
	}
	limtSync();
}

void strategyShoot(){
	FRICL.Target =-5400;
	FRICR.Target = 5400;
	laserOn();
	if(findEnemy){
		autoAim();
		if(fabs(aim.yaw)<1){firing2();}
		//else {firing1();}
		noEnemyCnt=-400;
		sendData[0].data[0]=(int16_t)1;
	}
	else if(noEnemyCnt>1){
		scaning();
		sendData[0].data[0]=(int16_t)0;
	}
	else if(noEnemyCnt<-370){
		noEnemyCnt++;
		if(fabs(aim.yaw)<1){firing2();}
		else{STIRv.Target=0;}
	}
	else{
		noEnemyCnt++;
		STIRv.Target=0;
	}
}

uint8_t msgRed[]="1\n",msgBlue[]="2\n";
void uartSend(int8_t i){
	static int16_t cnt;
	if(cnt>500){
		if(i==1){HAL_UART_Transmit_IT(&AUTOAIM_UART,(uint8_t*)msgRed,2);}
		if(i==2){HAL_UART_Transmit_IT(&AUTOAIM_UART,(uint8_t*)msgBlue,2);}
		cnt=0;
	}
	cnt++;
}
void uartSend2(){
	static int16_t cnt;
	if(cnt>500){
		if(receiveData[0].data[1]>4000){HAL_UART_Transmit_IT(&AUTOAIM_UART,(uint8_t*)msgBlue,2);}
		else{HAL_UART_Transmit_IT(&AUTOAIM_UART,(uint8_t*)msgRed,2);}
		cnt=0;
	}
	cnt++;
}
#endif // GUARD == 'D'


