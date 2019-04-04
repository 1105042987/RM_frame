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

GMAngle_t aim,aimLast;																				//目标角度
GMAngle_t adjust;																							//校准发射变量
Coordinate_t enemy_gun,enemyScope,scopeGun;										//坐标
uint8_t Enemy_INFO[8],Tx_INFO[8];															//接收
uint8_t find_enemy=0,aimMode=0,upper_mode;										//aimMode用于选择瞄准模式，0为手动瞄准，1为正常自瞄，2为打符，3暂无（吊射？）
uint16_t aimCnt=0;																						//自瞄分频延时变量
uint16_t auto_counter_fps = 1000;															//检测帧率
int16_t receiveCnt=0,receive_rcd=0;							  						//检测上位机信号帧数
int8_t trackCnt=0;																						//追踪变量
float t=5,k=1;//暂时调试用
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
	adjust.yaw=0.0f;			adjust.pit=-0.6f;
	
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
		find_enemy=1;
		receiveCnt++;
	}
	#else
	if(RX_ENEMY_START=='s'&&RX_ENEMY_END=='e'){
		aim.yaw=(float)( (((RX_ENEMY_YAW1<<8)|RX_ENEMY_YAW2)>0x7fff) ? (((RX_ENEMY_YAW1<<8)|RX_ENEMY_YAW2)-0xffff) : (RX_ENEMY_YAW1<<8)|RX_ENEMY_YAW2 )*k_angle;
		aim.pit=(float)( (((RX_ENEMY_PITCH1<<8)|RX_ENEMY_PITCH2)>0x7fff) ? (((RX_ENEMY_PITCH1<<8)|RX_ENEMY_PITCH2)-0xffff) : (RX_ENEMY_PITCH1<<8)|RX_ENEMY_PITCH2 )*k_angle;
		aim.yaw-=adjust.yaw;
		aim.pit+=adjust.pit;
		MINMAX(aim.yaw,-2.0f,2.0f);
		MINMAX(aim.pit,-2.0f,2.0f);
		//enemyScope.z=350;
		find_enemy=1;
		receiveCnt++;
	}
	#endif
	
	HAL_UART_Receive_DMA(&AUTOAIM_UART,(uint8_t *)&Enemy_INFO,8);
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
	aim.yaw=atan(enemy_gun.x/(enemy_gun.z*cos(GMP_ANGLE)-enemy_gun.y*sin(GMP_ANGLE)))/const_pi*180.0-adjust.yaw;
	aim.pit=atan(enemy_gun.y/enemy_gun.z)/const_pi*180.0+adjust.pit;
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
int sgn(float x){return x>0?1:(x<0?-1:0);}
void AutoAimNormal(){
	if(find_enemy){
		if(aimCnt<1){
			GMY.Target+=(sgn(aim.yaw)*(aim.yaw*aim.yaw)+aim.yaw)/6;//(aim.yaw+aimLast.yaw)/kk;//8;
			GMP.Target+=(sgn(aim.pit)*(aim.pit*aim.pit)+aim.pit)/6;//(aim.pitch+aimLast.pitch)/kk;//8;
			aimCnt++;
		}else{
			find_enemy=0;
			aimCnt=0;
			aimLast.yaw=aim.yaw;
			aimLast.pit=aim.pit;
		}
	}
}
void AutoAimYYP(){
	if(find_enemy){
		GMY.Target=GMY.Real+(sgn(aim.yaw)*(aim.yaw*aim.yaw)+aim.yaw)*1.5;
		GMP.Target=GMP.Real+(sgn(aim.pit)*(aim.pit*aim.pit)+aim.pit)*0.9;
		find_enemy=0;
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
//***********************************自瞄控制*******************************//
void AutoAimCTRL(){
	//UpperStateFSM();
	switch(aimMode){
		case 1:{//自瞄
			//AutoAimNormal();
			AutoAimYYP();
			break;
		}
		case 2:{//打符
			autoAimPredict();
			//AutoAimYYP();
			break;
		}
		default: break;
	}
	/************检测帧数*************/
	if(receiveCnt == 0)
		auto_counter_fps = 1000;
	if(auto_counter_fps == 0)	{
		receive_rcd = receiveCnt;
		receiveCnt = 0;
		auto_counter_fps = 1000;
	}
	/*********************************/
}
void autoAimPredict(){//@尹云鹏
	if(find_enemy){
		float GMYAngle=GMY.encoderAngle,GMPAngle=GMP.encoderAngle;
		static float GMYAngleLast,GMPAngleLast;
		static float myWYaw,myWPit,myWYawLast,myWPitLast,myAYaw,myAPit;//差分的本身角速度，角加速度，单位：度/周期(遥控器14ms)，所以采用了差分而不是直接读编码器角速度
		float enYaw=aim.yaw,enPit=aim.pit;
		static float enYawLast,enPitLast,enWYaw,enWPit,enWYawLast,enWPitLast,enAYaw,enAPit;//差分的敌方角速度，角加速度，
		myWYaw=GMYAngle-GMYAngleLast;
		myWPit=GMPAngle-GMPAngleLast;
		myWYaw=myWYaw<180?(myWYaw>-180?myWYaw:myWYaw+360):myWYaw-360;
		myWPit=myWPit<180?(myWPit>-180?myWPit:myWPit+360):myWPit-360;
		myAYaw=myWYaw-myWYawLast;
		myAPit=myWPit-myWPitLast;
		
		enWYaw=enYaw-enYawLast+myWYaw;
		enWPit=enPit-enPitLast+myWPit;
		enAYaw=enWYaw-enWYawLast;
		enAPit=enWPit-enWPitLast;
		
		//float t=aim.z*4; //距离*1000ms/子弹速度18/周期14ms，约为4
//		GMY.Target=GMY.Real+(enYaw+enWYaw*t)*k;
		GMY.Target=GMY.Real+(sgn(aim.yaw)*(aim.yaw*aim.yaw)+aim.yaw)*1.4+enWYaw*t;
		GMP.Target=GMP.Real+(sgn(aim.pit)*(aim.pit*aim.pit)+aim.pit)*0.9;//(enPit+enWPit*t)*k;
		
		aimLast=aim;
		GMYAngleLast=GMYAngle;
		GMPAngleLast=GMPAngle;
		myWYawLast=myWYaw;
		myWPitLast=myWPit;
		enWYawLast=enWYaw;
		enWPitLast=enWPit;
		find_enemy=0;
	}
}




#endif /*USE_AUTOAIM*/
#endif /*DEBUG_MODE*/

