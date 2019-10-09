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
GMAngle_t aim,abt,abtLast,opt,jst;
uint8_t Enemy_INFO[8],Tx_INFO[8];						//接收
uint8_t FindEnemy,AimMode;
int16_t AimTic=1;
GMAngle_t aimProcess(float yaw,float pit,int16_t *tic);
//********************************自瞄初始化********************************//
void InitAutoAim(){
	//开启AUTO_AIM_UART的DMA接收
	if(HAL_UART_Receive_DMA(&AUTOAIM_UART,(uint8_t *)&Enemy_INFO,8)!= HAL_OK){Error_Handler();}
	//角度变量初始化（不需要修改）
	aim.yaw=0;				aim.pit=0;
	jst.yaw=1;			jst.pit=0;
}
//*******************************UART回调函数********************************/
//float rate1=2.7,rate2=2.85;
float tmpY,tmpP;
void AutoAimUartRxCpltCallback(){
	if(RX_ENEMY_START=='s'&&RX_ENEMY_END=='e'){
		onLed(5);
		aim.yaw=(int16_t)((RX_ENEMY_YAW1<<8)|RX_ENEMY_YAW2)*kAngle;
		aim.pit=-(int16_t)((RX_ENEMY_PITCH1<<8)|RX_ENEMY_PITCH2)*kAngle;
		aim.dis=(int16_t)((RX_ENEMY_DIS1<<8)|RX_ENEMY_DIS2);//0无，500弱识别，1000正常，2000陀螺正装甲，3000陀螺右装甲，4000陀螺左装甲，5000五米外，6000工程
		if(aim.dis==0){
			HAL_UART_Receive_DMA(&AUTOAIM_UART,(uint8_t*)&Enemy_INFO,8);
			return;
		}
		aim.yaw/=2.7;
		aim.pit/=2.83;
		abt.yaw=GMY.Real+aim.yaw-jst.yaw;
		abt.pit=GMP.Real+aim.pit+jst.pit;
		if(abt.pit>-12 && GMY.encoderAngle>0){AimMode=5;}//外侧远处
		if(abt.pit<-13){AimMode=0;}
		if(abt.pit<-2){
//=========预测===========
			if(aim.dis==500 || aim.dis==1000 || aim.dis==6000){opt=aimProcess(abt.yaw,abt.pit,&AimTic);}
//========================
			else{
				if(aim.dis==2000){//陀螺正对装甲
					abt.yaw=abtLast.yaw*0.6+abt.yaw*0.4;
					abt.pit=abtLast.pit*0.7+abt.pit*0.3;
				}else if(aim.dis==3000){//陀螺右侧边装甲,yaw为负
					abt.pit=abtLast.pit*0.7+abt.pit*0.3;
					abt.yaw=abtLast.yaw*0.6+(abt.yaw-5*sin(abt.pit))*0.4;
				}else if(aim.dis==4000){//陀螺左侧边装甲
					abt.pit=abtLast.pit*0.7+abt.pit*0.3;
					abt.yaw=abtLast.yaw*0.7+(abt.yaw+5*sin(abt.pit))*0.3;
				}
//========速度补偿========
				if(AimMode){
					opt.yaw=abt.yaw-ChaSpdSin /19;//19.3,20
					opt.pit=abt.pit-ChaSpdCos /18;//18
				}else{
					opt.yaw=abt.yaw-ChaSpdSin /14.3;//19.3
					opt.pit=abt.pit-ChaSpdCos /14.3;
				}
//========================
			}
			FindEnemy=1;
		}
		abtLast=abt;
	}
	HAL_UART_Receive_DMA(&AUTOAIM_UART,(uint8_t*)&Enemy_INFO,8);
}

//**************************普通模式自瞄控制函数****************************//
void autoAim(){
	if(AimMode){opt.pit-=12/opt.pit+0.5;}//5m
	else if(GMY.encoderAngle<0){opt.pit-=10/opt.pit-0.5;}
	else{opt.pit-=30/opt.pit+0.5;}//下坠补偿
	GMY.Target=opt.yaw;
	GMP.Target=opt.pit;
	FindEnemy=0;
}
//float wy,wp;
GMAngle_t aimProcess(float yaw,float pit,int16_t *tic){
/*@尹云鹏，自瞄预测，反陀螺，运动补偿
	参数：绝对角度yaw，pit，计时器地址
	核心思想：
	1.视觉数据需要与真实角度标定，传入参数为目标绝对角度，差分出速度
	2.视觉数据存在误差，以一定时间间隔采样
	3.速度累加与指数衰减对抗
	4.判定陀螺：速度与aim积小于一个负数，计次超过3
	5.反陀螺：加权平均
	6.自身运动补偿：在functionTask
*/
	#define amt 6	//间隔点个数amount，调节amt使时间间隔大约为50ms，即 amt=50ms/1000ms*fps
	static int8_t i,lock,whipCnt,cnt;	//index计数器，首次进入保护锁，陀螺判定计数，调用计数
	static float 	y[amt],p[amt],			//yaw,pit历史
								tSum,t[amt],				//间隔时间,tic历史
								wy,wp,							//yaw,pit角速度
								wySum,wpSum,				//角速度累加对抗
								dYaw;								//两次yaw差值，判定陀螺用
	static GMAngle_t in,out;					//上一次值，返回值角度
	tSum+=*tic-t[i];	//与pid的i计算如出一辙，加上本次并减去amt次以前的时间间隔，得到分频后的间隔
	cnt++;
	if(*tic>180){			//if两次数据时间间隔大于XXms，清空历史，进入保护锁
		lock=amt;
		wy=0;wp=0;dYaw=yaw;
		wySum=0;wpSum=0;
		whipCnt=0;
		in.yaw=yaw;in.pit=pit;in.dis=pit;
		out=in;
	}
	in.yaw=in.yaw*0.7+yaw*0.3;//传入值滤波
	in.pit=in.pit*0.7+pit*0.3;
	if(lock){			//函数首次进入保护，只记录数据不预测
		lock--;
		if(AimMode){
			wySum=-ChaSpdSin /610;
			wpSum=-ChaSpdCos /327;
		}else{
			wySum=-ChaSpdSin /434;
			wpSum=-ChaSpdCos /222;
		}
	}
	else{
		//判定陀螺，应该在本次计算wySum之前
		dYaw=in.yaw-dYaw;
		if(dYaw*wySum<-0.35){
			if(whipCnt<20){whipCnt++;}
			cnt=0;
		}else if(cnt>120){
			cnt=0;
			whipCnt=0;
		}
//==============
//		wy=(wy+(in.yaw-y[i])*50/tSum)/2;	//速度滤波,*45本来是预测时间，放到这里提高除法精度
//		wp=(wp+(in.pit-p[i])*50/tSum)/2;
//==============	
		wy=(wy+(in.yaw-y[i])/tSum)/2;	//速度滤波
		wp=0.7*wp+0.3*(in.pit-p[i])/tSum;
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
//==============
//		out.yaw=(in.yaw-wy/sin(in.pit/57.3)+out.yaw)/2;
//		out.pit=(in.pit+wp+out.pit)/2;
//==============	
		out.yaw=0.4*(in.yaw+wySum*26)+ 0.6*out.yaw;
		out.dis=0.3*(in.pit+wpSum*10)+ 0.7*out.pit;
		out.yaw-=ChaSpdSin/100;
		out.pit=out.dis-ChaSpdCos/40;
	}else{//反陀螺，低权值滤波
		out.yaw=0.8*out.yaw+0.2*(in.yaw-ChaSpdSin /19);//19.3
		out.pit=0.8*out.pit+0.2*(in.pit-ChaSpdCos /18);
	}
//3.14x8x0.09x8/60/21= 1/557
	return out;
}
//记录云台角度历史
GMAngle_t GMAngleRcd(){
	#define rcdAmt 30
	static GMAngle_t GMAngleRcd[rcdAmt];
	static uint8_t i = 0;
	GMAngleRcd[i].yaw = GMY.Real;
	GMAngleRcd[i].pit = GMP.Real;
	i=(i+1)%rcdAmt;
	return GMAngleRcd[i];
}

#endif /*USE_AUTOAIM*/
#endif /*DEBUG_MODE*/

