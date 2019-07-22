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
//#define AUTOAIM_DEBUG
#ifndef DEBUG_MODE
#ifdef	USE_AUTOAIM
#define USE_AUTOAIM_ANGLE
//*****************************************声明变量******************************************//
GMAngle_t aim,abt,opt,jst0,jst1;									//目标角度
uint8_t Enemy_INFO[8],Tx_INFO[8];								//接收
uint8_t find_enemy=0,upper_mode,aim_mode;
uint16_t aim_cnt=0;															//自瞄分频延时变量
int8_t track_cnt=60;														//追踪变量
double rcd_yaw=0, rcd_pitch=0;
uint8_t Pre_aim=0, clb=0,numState=1,colorAim;
float attack_mark;
int16_t AimTic=1;
GMAngle_t aimProcess(float yaw,float pit,int16_t *tic);
//********************************自瞄初始化********************************//
void InitAutoAim(){
	//开启AUTO_AIM_UART的DMA接收
	if(HAL_UART_Receive_DMA(&AUTOAIM_UART,(uint8_t *)&Enemy_INFO,8)!= HAL_OK){
		Error_Handler();
	}
	//角度变量初始化（不需要修改）
	aim.yaw=0;				aim.pit=0;
	jst0.yaw=1;			jst0.pit=1;
	jst1.yaw=1;			jst1.pit=3;
}

//*******************************UART回调函数********************************//
float rate1=10,rate2=12;
void AutoAimUartRxCpltCallback(){
	if(RX_ENEMY_START=='s'&&RX_ENEMY_END=='e'){
		aim.yaw=-(int16_t)((RX_ENEMY_YAW1<<8)|RX_ENEMY_YAW2)*k_angle;
		aim.pit=(int16_t)((RX_ENEMY_PITCH1<<8)|RX_ENEMY_PITCH2)*k_angle;
		aim.yaw/=rate1*2;
		aim.pit/=rate2*2;
		abt.yaw=GMY.RealAngle+aim.yaw;
		abt.pit=GMP.RealAngle+aim.pit;
		opt=aimProcess(GMY.RealAngle+aim.yaw, GMP.RealAngle+aim.pit, &AimTic);
		find_enemy=1;
	}
	HAL_UART_Receive_DMA(&AUTOAIM_UART,(uint8_t *)&Enemy_INFO,8);
}

//**************************普通模式自瞄控制函数****************************//
float py=0.5, iy=0.0, dy=0.0;
float yawAj=1,pitAj=1;
void AutoAim(){
	if(find_enemy){
//		static float yi = 0, lasty = 0;
//		yi += aim.yaw;
//		//auto attack
//		attack_mark = aim.yaw * py + yi * iy + (aim.yaw-lasty) * dy;
//		GMY.TargetAngle = GMY.RealAngle + attack_mark;  
////		GMY.TargetAngle += aim.yaw * py + yi * iy + (aim.yaw-lasty) * dy;
//		lasty = aim.yaw;
//		GMP.TargetAngle = GMP.RealAngle + aim.pit * 0.3f;
//		GMY.TargetAngle=abt.yaw;
//		GMP.TargetAngle=abt.pit;
		GMY.TargetAngle=opt.yaw;
		GMP.TargetAngle=opt.pit;
		if(AimArmor){
			GMY.TargetAngle += jst0.yaw;
			GMP.TargetAngle += jst0.pit;
		}else{
			GMY.TargetAngle += jst1.yaw;
			GMP.TargetAngle += jst1.pit;
		}
		
		find_enemy=0;
	}
}
//***************************上位机工作模式切换*****************************//
uint8_t AimArmor,msgArmor[]="3\n",msgBase[]="4\n";
void UpperStateFSM(){
	static int16_t cnt;
	if(cnt>500){
//		if(GameRobotState.robot_id==7)
		if(AimArmor){HAL_UART_Transmit_IT(&AUTOAIM_UART,(uint8_t*)msgArmor,2);}
		else{HAL_UART_Transmit_IT(&AUTOAIM_UART,(uint8_t*)msgBase,2);}
		cnt=0;
	}
	cnt++;
}
float wySum;
GMAngle_t aimProcess(float yaw,float pit,int16_t *tic){
/*@尹云鹏，自瞄预测及下坠补偿
	参数：绝对角度yaw，pit，计时器地址
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
								//wySum,
	wpSum;	//角速度累加对抗
	static GMAngle_t in,out;		//上一次值，返回值角度
	tSum+=*tic-t[i];	//与pid的i计算如出一辙，加上本次并减去amt次以前的时间间隔，得到分频后的间隔
	if(*tic>150){			//if两次数据时间间隔大于150*2ms，清空历史，进入保护锁
		lock=amt;
		wy=0;wp=0;
		wySum=0;wpSum=0;
		in.yaw=yaw;in.pit=pit;
		out.yaw=yaw;out.pit=pit;
	}
	in.yaw=(yaw+in.yaw)/2;//传入值滤波
	in.pit=(pit+in.pit)/2;
	if(lock){lock--;}			//函数首次进入保护，只记录数据不预测
	else{
		wy=(wy+(in.yaw-y[i])/tSum)/2;	//速度滤波
		wp=(wp+(in.pit-p[i])/tSum)/2;
		wySum+=wy;	//角速度累加与指数衰减对抗
		wpSum+=wp;
		wySum*=0.9;	//指数衰减限制累加,失去物理意义
		wpSum*=0.9;
	}
	y[i]=in.yaw;	//yaw历史
	p[i]=in.pit;	//pit历史
	t[i]=*tic;		//tic历史
	i=(i+1)%amt;	//amt次之内循环
	out.yaw=(in.yaw+wySum*18+out.yaw)/2;		//实现预测，滤波
	out.pit=(in.pit+wpSum*8+out.pit)/2;
//	angle.pit-=40/angle.pit-0.4;//重力下坠补偿
	*tic=1;			//时间中断计时器重新开始
	return out;
}
#endif /*USE_AUTOAIM*/
#endif /*DEBUG_MODE*/
