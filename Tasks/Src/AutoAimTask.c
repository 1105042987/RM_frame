/**
  ******************************************************************************
  *FileName				: AutoAimTask.c
  *Description		: 自瞄程序
  *Author					: 管易恒
  ******************************************************************************
  *
  * Copyright (c) 2019 Team JiaoLong-ShanghaiJiaoTong University
  * All rights reserved.
  *
  ******************************************************************************
*/
#include "includes.h"
#ifndef DEBUG_MODE
#ifdef	USE_AUTOAIM
#define USE_AUTOAIM_ANGLE
GMAngle_t aimProcess(float yaw,float pit,int16_t *tic);
void fallCompsition();
typedef struct{
	float x,v,k1,k2,t,R;
	float q1,q2,p11,p12,p21,p22;
	float t11,t12,t21,t22;//tmp
}kalman_t;
#define kalmanInit \
{ 0,0,0,0,1,1,\
	0.1,0.1,1,0,0,1,\
	0,0,0,0 \
}
kalman_t kFilter=kalmanInit;
float kalmanCalc(kalman_t *f,float z,int t){
	//x=A*x0
	f->x += f->v * f->t;
	//P=APA'+Q
	f->t11 = f->p11+ (f->p12+ f->p21+ f->p22* f->t)* f->t+ f->q1;
	f->t12 = f->p12+ f->p22* f->t;
	f->t21 = f->p21+ f->p22* f->t;
	f->t22 = f->p22+ f->q2;
	//K=PH/(HPH'+R)
	f->k1= f->t11/ (f->t11+ f->R);
	f->k2= f->t12/ (f->t11+ f->R);
	//x=x+K(z-Hx)
	f->x += f->k1*(z- f->x);
	f->v += f->k2*(z- f->x);
	//P=(1-KH)PH
	f->p11=(1- f->k1)* f->t11;
	f->p12=(1- f->k1)* f->t12;
	f->p21=f->t21- f->k2* f->t11;
	f->p22=f->t22- f->k2* f->t12;
	return f->x + f->v *t;
}
//*****************************************声明变量******************************************//

GMAngle_t aim,aimLast,opt,tmp;//optimize					//目标角度
GMAngle_t adjust;															//校准发射变量
uint8_t Enemy_INFO[8],Tx_INFO[8];							//接收
uint8_t findEnemy=0,aimMode=0,upper_mode;			//aimMode用于选择瞄准模式，0为手动瞄准，1为正常自瞄，2为打符，3暂无（吊射？）

uint16_t aimCnt=0;														//自瞄分频延时变量
int16_t receiveCnt=0,receiveFps=0,AimTic=1;		//检测上位机信号帧数
extern int16_t receiveCnt,receiveFps;
int8_t trackCnt=0;														//追踪变量

//********************************自瞄初始化********************************//

void InitAutoAim(){
	//开启AUTO_AIM_UART的DMA接收
	if(HAL_UART_Receive_DMA(&AUTOAIM_UART,(uint8_t *)&Enemy_INFO,8)!= HAL_OK){Error_Handler();}
	//角度变量初始化（不需要修改）
	aim.yaw=0;				aim.pit=0;
	adjust.yaw=0;			adjust.pit=0;
}
//*******************************UART回调函数********************************//
float tmpr=2.65;
void AutoAimUartRxCpltCallback(){
	if(RX_ENEMY_START=='s'&&RX_ENEMY_END=='e'){
		onLed(6);
		aim.yaw=-(int16_t)((RX_ENEMY_YAW1<<8)|RX_ENEMY_YAW2)*kAngle;
		aim.pit=(int16_t)((RX_ENEMY_PITCH1<<8)|RX_ENEMY_PITCH2)*kAngle;
		aim.dis=(int16_t)((RX_ENEMY_DIS1<<8)|RX_ENEMY_DIS2);
		aim.yaw/=2.7;
		aim.pit/=2.6;
//		aim.yaw=(GMY.Real+aim.yaw+aimLast.yaw)/2;
//		aim.pit=(GMP.Real+aim.pit+aimLast.pit)/2;
//		aimLast=aim;
		tmp.yaw=(GMY.Real+aim.yaw+aimLast.yaw)/2;
		tmp.pit=(GMP.Real+aim.pit+aimLast.pit)/2;
//		tmp.yaw=GMY.Real+aim.yaw;
//		tmp.pit=GMP.Real+aim.pit;
		aimLast=tmp;
		if(GMP.Real+aim.pit<-15){
			opt=aimProcess(tmp.yaw,tmp.pit,&AimTic);
			findEnemy=1;
		}
	}
	HAL_UART_Receive_DMA(&AUTOAIM_UART,(uint8_t*)&Enemy_INFO,8);
}

//**************************普通模式自瞄控制函数****************************//
void autoAim(){
	GMY.Target=opt.yaw+2;
//	GMP.Target=GMP.Real+aim.pit*0.65-aimLast.pit*0.2;
	GMP.Target=opt.pit+3;
	findEnemy=0;
}

//***************************上位机工作模式切换*****************************//
void UpperStateFSM(){
	if(upper_mode != aimMode && upper_mode != 0){
		Tx_INFO[0] = 'c';
		Tx_INFO[1] = aimMode;
		Tx_INFO[2] = 'e';
		HAL_UART_Transmit(&AUTOAIM_UART,Tx_INFO,3,0xff);
		
		upper_mode = aimMode;
	}
}
GMAngle_t aimProcess(float yaw,float pit,int16_t *tic){
/*@尹云鹏，自瞄预测及下坠补偿
	核心思想：
	1.视觉数据需要与真实角度标定，传入参数为目标绝对角度，差分出速度
	2.视觉数据存在误差，以一定时间间隔采样(即高中做实验学过的，高考必考内容)
	3.在循环里速度线性累加与指数衰减对抗
	4.基于pitch的重力下坠补偿
*/
	#define amt 5	//间隔点个数amount，调节amt使时间间隔大约为50ms，即 amt=50ms/1000ms*fps
	static int8_t i,lock;				//计数器，首次进入保护锁
	static float 	y[amt],p[amt],//yaw,pit历史
								tSum,t[amt],	//间隔时间,tic历史
								wy,wp,				//yaw,pit角速度
								wySum,wpSum;	//角速度累加对抗
	static GMAngle_t angle;	//返回值目标角度
	
	tSum+=*tic-t[i];	//与pid的i计算如出一辙，加上本次并减去amt次以前的时间间隔，得到分频后的间隔
	if(*tic>150){			//if两次数据时间间隔大于100*2ms，清空历史，进入保护锁
		lock=amt;
		wy=0;wp=0;
		angle.yaw=0;angle.pit=0;
	}
	if(lock){lock--;}	//函数首次进入保护，只记录数据不预测
	else{
		wy=(wy+(yaw-y[i])/tSum)/2;	//本次与上次平均滤波
		wp=(wp+(pit-p[i])/tSum)/2;
		wySum+=wy;	//角速度累加与指数衰减对抗
		wpSum+=wp;
		wySum*=0.9;//指数衰减限制累加,失去物理意义
		wpSum*=0.9;
	}
	y[i]=yaw;		//yaw历史
	p[i]=pit;		//pit历史
	t[i]=*tic;	//tic历史
	i=(i+1)%amt;//amt次之内循环
	
	angle.yaw=(yaw+wySum*25+angle.yaw)/2;		//实现预测
	angle.pit=(pit+wpSum*10+angle.pit)/2;
//	angle.pit-=40/angle.pit-0.4;//重力下坠补偿
	*tic=1;			//时间中断计时器重新开始
	return angle;
}
#endif /*USE_AUTOAIM*/
#endif /*DEBUG_MODE*/

