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
//*****************************************声明变量******************************************//
GMAngle_t aim,abt,opt,jst;										//校准发射变量
uint8_t Enemy_INFO[8],Tx_INFO[8];						//接收
uint8_t FindEnemy=0;
int16_t AimTic=1;
GMAngle_t aimProcess(float yaw,float pit,int16_t *tic);
//********************************自瞄初始化********************************//
void InitAutoAim(){
	//开启AUTO_AIM_UART的DMA接收
	if(HAL_UART_Receive_DMA(&AUTOAIM_UART,(uint8_t *)&Enemy_INFO,8)!= HAL_OK){Error_Handler();}
	//角度变量初始化（不需要修改）
	aim.yaw=0;				aim.pit=0;
	jst.yaw=2;			jst.pit=0.5;
}
//*******************************UART回调函数********************************/
//float rate1=2.7,rate2=2.85;
void AutoAimUartRxCpltCallback(){
	if(RX_ENEMY_START=='s'&&RX_ENEMY_END=='e'){
		onLed(5);
		aim.yaw=(int16_t)((RX_ENEMY_YAW1<<8)|RX_ENEMY_YAW2)*kAngle;
		aim.pit=-(int16_t)((RX_ENEMY_PITCH1<<8)|RX_ENEMY_PITCH2)*kAngle;
		aim.dis=(int16_t)((RX_ENEMY_DIS1<<8)|RX_ENEMY_DIS2);
		aim.yaw/=2.7;
		aim.pit/=2.85;
		abt.yaw=GMY.Real+aim.yaw;
		abt.pit=GMP.Real+aim.pit;
		//if(abt.pit<0){
			if(aim.dis==2000){//陀螺正对装甲
				opt.yaw=opt.yaw*0.6+abt.yaw*0.4;
				opt.pit=opt.pit*0.6+abt.pit*0.4;
//				opt.yaw-=sgn(GMY.encoderAngle)*(((uint16_t)(receiveData[0].data[0])>>8) -127)/200.0;
			}else if(aim.dis==3000){//陀螺侧边装甲
				opt.yaw=opt.yaw*0.6+(GMY.Real+sgn(aim.yaw)*(fabs(aim.yaw)-3))*0.4;
				opt.pit=opt.pit*0.6+abt.pit*0.4;
//				opt.yaw-=sgn(GMY.encoderAngle)*(((uint16_t)(receiveData[0].data[0])>>8) -127)/200.0;
			}
			else{opt=aimProcess(abt.yaw,abt.pit,&AimTic);}
			FindEnemy=1;
	//	}
	}
	HAL_UART_Receive_DMA(&AUTOAIM_UART,(uint8_t*)&Enemy_INFO,8);
}

//**************************普通模式自瞄控制函数****************************//
float test=2;
void autoAim(){
	GMY.Target=opt.yaw-jst.yaw;
//	GMP.Target=GMP.Real+aim.pit*0.65-aimLast.pit*0.2;
	GMP.Target=opt.pit+jst.pit;
	
	GMY.Target-=((int16_t)((uint16_t)(receiveData[0].data[0])>>8) -127)/557.0 * fabs(sin((GMY.encoderAngle-aim.yaw)/180*3.14))*sgn(GMY.encoderAngle);
	GMP.Target+=((int16_t)((uint16_t)(receiveData[0].data[0])>>8) -127)/800 * fabs(cos((GMY.encoderAngle-aim.yaw)/180*3.14))*sgn(GMY.encoderAngle);

	if(GMP.Target>-5){GMP.Target+=test;}
	else{GMP.Target-=15/GMP.Target+0.5;}

	FindEnemy=0;
}


float wySum,dy;
GMAngle_t aimProcess(float yaw,float pit,int16_t *tic){
/*@尹云鹏，自瞄预测，反陀螺，运动补偿
	参数：绝对角度yaw，pit，计时器地址
	核心思想：
	1.视觉数据需要与真实角度标定，传入参数为目标绝对角度，差分出速度
	2.视觉数据存在误差，以一定时间间隔采样(即高中做实验学过的，高考必考内容)
	3.在循环里速度线性累加与指数衰减对抗
	4.判定陀螺：速度与aim积小于一个负数，计次超过3
	5.反陀螺：加权平均
	6.自身运动补偿：在functionTask
*/
	#define amt 5	//间隔点个数amount，调节amt使时间间隔大约为50ms，即 amt=50ms/1000ms*fps
	static int8_t i,lock,whipCnt,cnt;	//index计数器，首次进入保护锁，陀螺判定计数，调用计数
	static float 	y[amt],p[amt],			//yaw,pit历史
								tSum,t[amt],				//间隔时间,tic历史
								wy,wp,							//yaw,pit角速度
	//							wySum,
	wpSum,				//角速度累加对抗
								dYaw;								//两次yaw差值，判定陀螺用
	static GMAngle_t in,out;					//上一次值，返回值角度
	tSum+=*tic-t[i];	//与pid的i计算如出一辙，加上本次并减去amt次以前的时间间隔，得到分频后的间隔
	cnt++;
	if(*tic>160){			//if两次数据时间间隔大于150*2ms，清空历史，进入保护锁
		lock=amt;
		wy=0;wp=0;dYaw=yaw;
		wySum=0;wpSum=0;
		whipCnt=0;
		in.yaw=yaw;in.pit=pit;
		out=in;
	}
	in.yaw=(yaw+in.yaw)/2;//传入值滤波
	in.pit=(pit+in.pit)/2;
	if(lock){lock--;}			//函数首次进入保护，只记录数据不预测
	else{
		//判定陀螺，应该在本次计算wySum之前
		dYaw=in.yaw-dYaw;
		dy=dYaw;
		if(dYaw*wySum<-0.35){
			if(whipCnt<20){whipCnt++;}
			cnt=0;
		}else if(cnt>120){
			cnt=0;
			whipCnt=0;
		}
		wy=(wy+(in.yaw-y[i])/tSum)/2;	//速度滤波
		wp=(wp+(in.pit-p[i])/tSum)/2;
		wySum+=wy;	//角速度累加与指数衰减对抗
		wpSum+=wp;
		wySum*=0.9;	//指数衰减限制累加,失去物理意义
		wpSum*=0.9;
	}
	dYaw=in.yaw;	//暂存上次yaw，用于下次计算yaw差值
	y[i]=in.yaw;	//yaw历史
	p[i]=in.pit;	//pit历史
	t[i]=*tic;		//tic历史
	i=(i+1)%amt;	//amt次之内循环
	*tic=1;				//时间中断计时器重新开始
	
	if(whipCnt<5){//预测，滤波
		out.yaw=(in.yaw+wySum*25+out.yaw+out.yaw)/3;
		out.pit=(in.pit+wpSum*15+out.pit+out.pit)/3;
		out.dis=1;
	}else{//反陀螺，低权值滤波
		out.yaw=out.yaw*0.8+in.yaw*0.2;
		out.pit=out.pit*0.8+in.pit*0.2;
//		out.yaw-=sgn(GMY.encoderAngle)*(((uint16_t)(receiveData[0].data[0])>>8) -127)/800.0;
		out.dis=2;
	}
	//3.14x8x0.09x8/60/21= 1/557
	return out;
}
#endif /*USE_AUTOAIM*/
#endif /*DEBUG_MODE*/

