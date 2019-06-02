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

GMAngle_t aim,aimLast;																				//目标角度
GMAngle_t adjust;																							//校准发射变量
Coordinate_t enemy_gun,enemyScope,scopeGun;										//坐标
uint8_t Enemy_INFO[8],Tx_INFO[8];															//接收
uint8_t findEnemy=0,aimMode=0,upper_mode;											//aimMode用于选择瞄准模式，0为手动瞄准，1为正常自瞄，2为打符，3暂无（吊射？）

uint16_t aimCnt=0;																						//自瞄分频延时变量
int16_t receiveCnt=0,receiveFps=0;														//检测上位机信号帧数
extern int16_t receiveCnt,receiveFps;
int8_t trackCnt=0;																						//追踪变量

//********************************自瞄初始化********************************//

void InitAutoAim(){
	//开启AUTO_AIM_UART的DMA接收
	if(HAL_UART_Receive_DMA(&AUTOAIM_UART,(uint8_t *)&Enemy_INFO,8)!= HAL_OK){
		Error_Handler();
	}
	//坐标变量初始化（不需要修改）
	enemyScope.x=0;	enemyScope.y=0;	enemyScope.z=200;
	enemy_gun.x=0;		enemy_gun.y=0;		enemy_gun.z=200;
	
	//角度变量初始化（不需要修改）
	aim.yaw=0;				aim.pit=0;
	adjust.yaw=0;			adjust.pit=0;
	
	//设置坐标初始值（根据不同安装情况调整这3个参数）
	scopeGun.x=0;		scopeGun.y=-10;		scopeGun.z=0;
}

//*******************************UART回调函数********************************//
void AutoAimUartRxCpltCallback(){
	#ifndef USE_AUTOAIM_ANGLE
	//串口数据解码
	if(RX_ENEMY_START=='s'&&RX_ENEMY_END=='e')
	{
		enemyScope.x=(float)((RX_ENEMY_X1<<8)|RX_ENEMY_X2)*k_coordinate;
		enemyScope.y=(float)((RX_ENEMY_Y1<<8)|RX_ENEMY_Y2)*k_coordinate;
//		enemyScope.z=(float)((RX_ENEMY_Z1<<8)|RX_ENEMY_Z2)*k_distance;
		enemyScope.z=350;
		enemyScope.x=(enemyScope.x>coordinate_max)?(enemyScope.x-2*coordinate_max):enemyScope.x;
		enemyScope.y=(enemyScope.y>coordinate_max)?(enemyScope.y-2*coordinate_max):enemyScope.y;
		findEnemy=1;
		receiveCnt++;
	}
	#else
	if(RX_ENEMY_START=='s'&&RX_ENEMY_END=='e'){
		onLed(6);
		aim.yaw=(float)(( (((RX_ENEMY_YAW1<<8)|RX_ENEMY_YAW2)>0x7fff) ? (((RX_ENEMY_YAW1<<8)|RX_ENEMY_YAW2)-0xffff) : (RX_ENEMY_YAW1<<8)|RX_ENEMY_YAW2 )*kAngle);
		aim.pit=-(float)(( (((RX_ENEMY_PITCH1<<8)|RX_ENEMY_PITCH2)>0x7fff) ? (((RX_ENEMY_PITCH1<<8)|RX_ENEMY_PITCH2)-0xffff) : (RX_ENEMY_PITCH1<<8)|RX_ENEMY_PITCH2 )*kAngle);
		aim.dis=(float)(( (((RX_ENEMY_DIS1<<8)|RX_ENEMY_DIS2)>0x7fff) ? (((RX_ENEMY_DIS1<<8)|RX_ENEMY_DIS2)-0xffff) : (RX_ENEMY_DIS1<<8)|RX_ENEMY_DIS2 ));
		aim.yaw+=adjust.yaw;
		aim.pit+=adjust.pit;
		aim.pit*=0.9;
//		enemyScope.z=350;
		adjust.pit=GMP.Real*0.16+0.5;
		
		if(GMP.Real+aim.pit<3){findEnemy=1;}
		MINMAX(aim.yaw,-10,10);
		MINMAX(aim.pit,-8,8);
		receiveCnt++;
	}
	#endif
	HAL_UART_Receive_DMA(&AUTOAIM_UART,(uint8_t*)&Enemy_INFO,8);
	
}
//****************************************坐标角度转换函数*************************************//

//在时间中断中分频后调用该函数
void EnemyINFOProcess(){
	#ifndef USE_AUTOAIM_ANGLE
	//坐标转换
	enemy_gun.x=enemyScope.x+scopeGun.x;
	enemy_gun.x=enemyScope.y+scopeGun.y;
	enemy_gun.z=enemyScope.z+scopeGun.z;
	
	//角度计算（计算消耗内存较多，不能放在2ms以下的时间中断内执行）
	aim.yaw=atan(enemy_gun.x/(enemy_gun.z*cos(GMP_ANGLE)-enemy_gun.y*sin(GMP_ANGLE)))/constPi*180.0-adjust.yaw;
	aim.pit=atan(enemy_gun.y/enemy_gun.z)/constPi*180.0+adjust.pit;
	#endif
	
	//追踪
	if(((aim.yaw>0 && aimLast.yaw>0) || (aim.yaw<0 && aimLast.yaw<0))){
		trackCnt++;
		MINMAX(trackCnt,0,100);
	}
	else{
		trackCnt=0;
	}
}

//**************************普通模式自瞄控制函数****************************//

void autoAim(){
	static float wy,wp;
//	GMY.Target=GMY.Real+(double)aim.yaw*0.5;//sgn(aim.yaw)*0.5*sqrt(fabs(aim.yaw));//(aim.yaw+aimLast.yaw)/kk;//8;
//	GMP.Target=GMP.Real+(double)aim.pit*0.4;//(sgn(aim.pit)*(aim.pit*aim.pit)+aim.pit)*0.3;//(aim.pitch+aimLast.pitch)/kk;//8;
	GMY.Target=GMY.Real+(double)aim.yaw*1.1-aimLast.yaw*0.3;
	GMP.Target=GMP.Real+(double)aim.pit*0.65-aimLast.pit*0.2;
//	wy=GMY.Real+aim.yaw-wy;
//	GMY.Target=GMY.Real+aim.yaw*0.8+wy;
//	GMP.Target=GMP.Real+aim.pit*0.65-aimLast.pit*0.2;
	
	aimLast.yaw=aim.yaw;
	aimLast.pit=aim.pit;
	
//	wy=GMY.Real+aim.yaw;
	findEnemy=0;
}






void autoAimNormal(){
	if(findEnemy){
		if(aimCnt<1){
			GMY.Target=GMY.Real+(double)aim.yaw*0.9;//(sgn(aim.yaw)*(aim.yaw*aim.yaw)+aim.yaw)*0.4;//(aim.yaw+aimLast.yaw)/kk;//8;
			GMP.Target=GMP.Real+(double)aim.pit*0.6;//(sgn(aim.pit)*(aim.pit*aim.pit)+aim.pit)*0.3;//(aim.pitch+aimLast.pitch)/kk;//8;
			aimCnt++;
		}else{
			findEnemy=0;
			aimCnt=0;
			aimLast.yaw=aim.yaw;
			aimLast.pit=aim.pit;
		}
	}
}
void AutoAimYYP(){
	if(findEnemy){
		GMY.Target=kalmanCalc(&kFilter,GMY.Real+aim.yaw,1);
//		GMY.Target=GMY.Real+aim.yaw*0.8;//(sgn(aim.yaw)*(aim.yaw*aim.yaw)+aim.yaw)*1.3;
		GMP.Target=GMP.Real+(double)aim.pit*0.7;//(sgn(aim.pit)*(aim.pit*aim.pit)+aim.pit)*0.85;
		findEnemy=0;
	}
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
//*****************************自瞄控制*******************************//
void autoAimCtrl(){
	//UpperStateFSM();
	switch(aimMode){
		case 1:{//自瞄
			autoAimNormal();
//			AutoAimYYP();
			break;
		}
		case 2:{//打符
//			autoAimPredict();
			AutoAimYYP();
			break;
		}
		default: break;
	}
}
////*****************************************************************//
//float a=1;
//float t=5,k=0.5;//暂时调试用
//float myWYaw,myWPit,myWYawLast,myWPitLast,myAYaw,myAPit;
//float enYawLast,enPitLast,enWYaw,enWPit,enWYawLast,enWPitLast,enAYaw,enAPit;//差分的敌方角速度，角加速度，
//float enYaw,enPit;
//void autoAimPredict(){//@尹云鹏
//	if(findEnemy){
//		float GMYAngle=GMY.encoderAngle,GMPAngle=GMP.encoderAngle;
//		static float GMYAngleLast,GMPAngleLast;
////		static float myWYaw,myWPit,myWYawLast,myWPitLast,myAYaw,myAPit;//差分的本身角速度，角加速度，单位：度/周期(2ms)，所以采用了差分而不是直接读编码器角速度
////		float enYaw=aim.yaw*a,enPit=aim.pit*3;
//		enYaw=aim.yaw*a;
////		static float enYawLast,enPitLast,enWYaw,enWPit,enWYawLast,enWPitLast,enAYaw,enAPit;//差分的敌方角速度，角加速度，
//		myWYaw=GMYAngle-GMYAngleLast;
//		myWPit=GMPAngle-GMPAngleLast;
//		myWYaw=myWYaw<180?(myWYaw>-180?myWYaw:myWYaw+360):myWYaw-360;
//		myWPit=myWPit<180?(myWPit>-180?myWPit:myWPit+360):myWPit-360;
//		myAYaw=myWYaw-myWYawLast;
//		myAPit=myWPit-myWPitLast;
//		
//		enWYaw=enYaw-enYawLast+myWYaw;
////		enWPit=enPit-enPitLast+myWPit;
//		enAYaw=enWYaw-enWYawLast;
//		enAPit=enWPit-enWPitLast;
//		
//		//float t=aim.z*4; //距离*1000ms/子弹速度18/周期2ms，约为4
//		GMY.Target=GMY.Real+(enYaw+enWYaw*t)*k;
////		GMP.Target=GMP.Real+aim.pit*0.7;
//		GMP.Target=GMP.Real+(enPit+enWPit*t)*k;
//		
////		GMY.Target=kalmanCalc(&kFilter,GMYAngle+aim.yaw,1);
//		
//		
//		aimLast=aim;
//		GMYAngleLast=GMYAngle;
//		GMPAngleLast=GMPAngle;
//		enYawLast=enYaw;
//		enPitLast=enPit;
//		myWYawLast=myWYaw;
//		myWPitLast=myWPit;
//		enWYawLast=enWYaw;
//		enWPitLast=enWPit;
//		findEnemy=0;
//	}
//}

#endif /*USE_AUTOAIM*/
#endif /*DEBUG_MODE*/

