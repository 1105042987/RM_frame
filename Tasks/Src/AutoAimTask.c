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

GMAngle_t aim,aim_rcd;																				//目标角度
GMAngle_t adjust;																							//校准发射变量
Coordinate_t enemy_gun,enemy_scope,scope_gun;									//坐标
uint8_t Enemy_INFO[8],Tx_INFO[8];															//接收
uint8_t find_enemy=0,aim_mode=0,upper_mode;										//aim_mode用于选择瞄准模式，0为手动瞄准，1为正常自瞄，2为打符，3暂无（吊射？）
uint16_t aim_cnt=0;																						//自瞄分频延时变量
uint16_t auto_counter_fps = 1000;															//检测帧率
int16_t current_yaw=0,current_pitch=0;												//当前云台角度
int16_t receive_cnt=0,receive_rcd=0;													//检测上位机信号帧数
int8_t track_cnt=0;																						//追踪变量

//********************************************************************************************//


//********************************自瞄初始化********************************//

void InitAutoAim()
{
	//开启AUTO_AIM_UART的DMA接收
	if(HAL_UART_Receive_DMA(&AUTOAIM_UART,(uint8_t *)&Enemy_INFO,8)!= HAL_OK)
	{
		Error_Handler();
	}

	//坐标变量初始化（不需要修改）
	enemy_scope.x=0;	enemy_scope.y=0;	enemy_scope.z=200;
	enemy_gun.x=0;		enemy_gun.y=0;		enemy_gun.z=200;
	
	//角度变量初始化（不需要修改）
	aim.yaw=0;				aim.pitch=0;
	adjust.yaw=0.0f;			adjust.pitch=-0.3f;
	
	//设置坐标初始值（根据不同安装情况调整这3个参数）
	scope_gun.x=0;		scope_gun.y=-10;		scope_gun.z=0;
}

//**************************************************************************//


//*******************************UART回调函数********************************//

void AutoAimUartRxCpltCallback()
{
	#ifndef USE_AUTOAIM_ANGLE
	//串口数据解码
	if(RX_ENEMY_START=='s'&&RX_ENEMY_END=='e')
	{
		enemy_scope.x=(float)((RX_ENEMY_X1<<8)|RX_ENEMY_X2)*k_coordinate;
		enemy_scope.y=(float)((RX_ENEMY_Y1<<8)|RX_ENEMY_Y2)*k_coordinate;
//		enemy_scope.z=(float)((RX_ENEMY_Z1<<8)|RX_ENEMY_Z2)*k_distance;
		enemy_scope.z=350;
		enemy_scope.x=(enemy_scope.x>coordinate_max)?(enemy_scope.x-2*coordinate_max):enemy_scope.x;
		enemy_scope.y=(enemy_scope.y>coordinate_max)?(enemy_scope.y-2*coordinate_max):enemy_scope.y;
		find_enemy=1;
		receive_cnt++;
	}
	#else
	if(RX_ENEMY_START=='s'&&RX_ENEMY_END=='e')
	{
		aim.yaw=-(float)( (((RX_ENEMY_YAW1<<8)|RX_ENEMY_YAW2)>0x7fff) ? (((RX_ENEMY_YAW1<<8)|RX_ENEMY_YAW2)-0xffff) : (RX_ENEMY_YAW1<<8)|RX_ENEMY_YAW2 )*k_angle;
		aim.pitch=(float)( (((RX_ENEMY_PITCH1<<8)|RX_ENEMY_PITCH2)>0x7fff) ? (((RX_ENEMY_PITCH1<<8)|RX_ENEMY_PITCH2)-0xffff) : (RX_ENEMY_PITCH1<<8)|RX_ENEMY_PITCH2 )*k_angle;
		aim.yaw-=adjust.yaw;
		aim.pitch+=adjust.pitch;
		MINMAX(aim.yaw,-3.0f,3.0f);
		MINMAX(aim.pitch,-2.0f,2.0f);
		//enemy_scope.z=350;
		find_enemy=1;
		receive_cnt++;
	}
	#endif
	
	HAL_UART_Receive_DMA(&AUTOAIM_UART,(uint8_t *)&Enemy_INFO,8);
}
//***************************************************************************//


//*************************************CAN线主控板间通信*************************************//

//用于英雄双云台

//*******************************************************************************************//


//****************************************坐标角度转换函数*************************************//

//在时间中断中分频后调用该函数
void EnemyINFOProcess()
{
	#ifndef USE_AUTOAIM_ANGLE
	//坐标转换
	enemy_gun.x=enemy_scope.x+scope_gun.x;
	enemy_gun.y=enemy_scope.y+scope_gun.y;
	enemy_gun.z=enemy_scope.z+scope_gun.z;
	
	//角度计算（计算消耗内存较多，不能放在2ms以下的时间中断内执行）
	aim.yaw=atan(enemy_gun.x/(enemy_gun.z*cos(GMP_ANGLE)-enemy_gun.y*sin(GMP_ANGLE)))/const_pi*180.0-adjust.yaw;
	aim.pitch=atan(enemy_gun.y/enemy_gun.z)/const_pi*180.0+adjust.pitch;
	#endif
	
	//追踪
	if(((aim.yaw>0 && aim_rcd.yaw>0) || (aim.yaw<0 && aim_rcd.yaw<0)))
	{
		track_cnt--;
		MINMAX(track_cnt,0,60);
	}
	else
	{
		track_cnt=60;
	}
}

//*********************************************************************************************//


//**************************普通模式自瞄控制函数****************************//

//void AutoAimNormal()
//{
//	MINMAX(aim.yaw,-6.0f,6.0f);
//	MINMAX(aim.pitch,-2.0f,2.0f);
//	if(find_enemy)
//	{
////		if(aim_cnt<1)
////		{
//			GMY.Target+=(aim.yaw+aim_rcd.yaw)/10;
//			//GMY.TargetAngle+=(aim.yaw+aim_rcd.yaw)/5*(0.5f+0.012f*track_cnt);
//			GMP.Target+=(aim.pitch+aim_rcd.pitch)/10;
////			aim_cnt++;
////		}
////		else
////		{
//			find_enemy=0;
////			aim_cnt=0;
//			aim_rcd.yaw=aim.yaw;
//			aim_rcd.pitch=aim.pitch;
////		}
//	}
//}
double ky=3, kp=6;
void AutoAimNormal()
{
	MINMAX(aim.yaw,-15.0f,15.0f);
	MINMAX(aim.pitch,-10.0f,10.0f);
//  cps[0] = GMY.TargetAngle;
//  cps[1] = GMY.RealAngle;
//  cps[2] = aim.yaw; 
//  HAL_UART_Transmit_IT(&huart7, (uint8_t*)cps, sizeof(cps));
	if(find_enemy)
	{
//		if(aim_cnt<1)
//		{
//			GMY.TargetAngle+=(aim.yaw)/5*(0.5f+0.012f*track_cnt);
//			GMP.TargetAngle-=(aim.pitch)/6;
      
      GMY.Target+=(aim.yaw)/ky;
			GMP.Target+=(aim.pitch)/kp;
//      if(((aim.yaw>0)?(aim.yaw):(-aim.yaw)) < sp){
//        GMY.TargetAngle+=(aim.yaw)/ksy*(0.5f+kk*track_cnt);
//      }else{
//        GMY.TargetAngle+=(aim.yaw)/ky*(0.5f+kk*track_cnt);
//      }
//			GMP.TargetAngle+=(aim.pitch+aim_rcd.pitch)/kp;
      
//			aim_cnt++;
//		}
//		else
//		{
			find_enemy=0;
//			aim_cnt=0;
//			aim_rcd.yaw=aim.yaw;
//			aim_rcd.pitch=aim.pitch;
		}
	}

//**************************************************************************//


//**************************打符模式自瞄控制函数****************************//

void AutoAimBuff()
{
	if(find_enemy)
	{
		//云台转向目标方向
		if(aim_cnt==0)
		{
			aim_rcd.yaw=aim.pitch;
			GMY.Target+=aim_rcd.yaw/20;
			GMP.Target+=aim_rcd.pitch/20;
			aim_cnt++;
		}
		else if(aim_cnt>=1 && aim_cnt<20)
		{
			GMY.Target+=aim_rcd.yaw/20;
			GMP.Target+=aim_rcd.pitch/20;
			aim_cnt++;
		}
		//等待云台稳定
		else if(aim_cnt>=20 && aim_cnt<30)
		{
			aim_cnt++;
		}
		//发射
		else if(aim_cnt==30)
		{
			//ShootOneBullet();
			aim_cnt++;
		}
		//等待云台稳定
		else if(aim_cnt>30 && aim_cnt<50)
		{
			aim_cnt++;
		}
		//重新指向中心
		else if(aim_cnt>=50 && aim_cnt<70)
		{
			GMY.Target-=aim_rcd.yaw/20;
			GMP.Target-=aim_rcd.pitch/20;
			aim_cnt++;
		}
		else
		{
			aim_cnt=0;
			find_enemy=0;
		}
	}
}

//**************************************************************************//


//***************************上位机工作模式切换*****************************//

void UpperStateFSM()
{
	if(upper_mode != aim_mode && upper_mode != 0)
	{
		Tx_INFO[0] = 'c';
		Tx_INFO[1] = aim_mode;
		Tx_INFO[2] = 'e';
		HAL_UART_Transmit(&AUTOAIM_UART,Tx_INFO,3,0xff);
		
		upper_mode = aim_mode;
	}
}

//**************************************************************************//


//***********************************自瞄控制*******************************//

void AutoAimGMCTRL()
{
	UpperStateFSM();
	switch(aim_mode)
	{
		case 1: 														//自瞄
		{
			AutoAimNormal();
			break;
		}
		case 2:															//打符
		{
			AutoAimBuff();
			break;
		}
		default: break;
	}
	
	/************检测帧数*************/
	if(receive_cnt == 0)
		auto_counter_fps = 1000;
	if(auto_counter_fps == 0)
	{
		receive_rcd = receive_cnt;
		receive_cnt = 0;
		auto_counter_fps = 1000;
	}
	/*********************************/
}

//**************************************************************************//


#endif /*USE_AUTOAIM*/
#endif /*DEBUG_MODE*/

