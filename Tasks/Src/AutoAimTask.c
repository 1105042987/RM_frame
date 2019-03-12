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

//*****************************************声明变量******************************************//

GMINFO_t aim,aim_rcd;																												//目标角度
Coordinate_t enemy_gun,enemy_scope,scope_gun;																//坐标
uint8_t Enemy_INFO[8];																											//接收
uint8_t find_enemy=0,aim_mode=0;																						//aim_mode用于选择瞄准模式，0为不瞄准，1为正常自瞄，2为打符，3暂无（吊射？）
uint16_t aim_cnt=0;																													//自瞄分频延时变量
int16_t current_yaw=0,current_pitch=0;																			//当前云台角度
int16_t receive_cnt=0,receive_rcd=0;																				//检测上位机信号帧数
double bullet_speed=10.0,bullet_speed_adjust=0,yaw_adjust=0,pitch_adjust=0;	//校准发射变量

#define GMP_ANGLE		(double)((GMP.RxMsg6623.angle-GMP.Zero)/8192.0*2*const_pi)

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
	
	//设置坐标初始值（根据不同安装情况调整这3个参数）
	scope_gun.x=0;		scope_gun.y=0;		scope_gun.z=0;
}

//**************************************************************************//


//*******************************UART回调函数********************************//

void AutoAimUartRxCpltCallback()
{
	//串口数据解码
	if(RX_ENEMY_START=='s'&&RX_ENEMY_END=='e')
	{
		enemy_scope.x=(float)((RX_ENEMY_X1<<8)|RX_ENEMY_X2)*k_coordinate;
		enemy_scope.y=(float)((RX_ENEMY_Y1<<8)|RX_ENEMY_Y2)*k_coordinate;
		enemy_scope.z=(float)((RX_ENEMY_Z1<<8)|RX_ENEMY_Z2)*k_distance;
		enemy_scope.x=(enemy_scope.x>coordinate_max)?(enemy_scope.x-2*coordinate_max):enemy_scope.x;
		enemy_scope.y=(enemy_scope.y>coordinate_max)?(enemy_scope.y-2*coordinate_max):enemy_scope.y;
		find_enemy=1;
		receive_cnt++;
	}
	
	HAL_UART_Receive_DMA(&AUTOAIM_UART,(uint8_t *)&Enemy_INFO,8);
}
//***************************************************************************//


//****************************************坐标角度转换函数*************************************//

//在时间中断中分频后调用该函数
void EnemyINFOProcess()
{
	//坐标转换
	enemy_gun.x=enemy_scope.x+scope_gun.x;
	enemy_gun.y=enemy_scope.y+scope_gun.y;
	enemy_gun.z=enemy_scope.z+scope_gun.z;
	
	//角度计算（计算消耗内存较多，不能放在2ms以下的时间中断内执行）
	aim.y=atan(enemy_gun.x/(enemy_gun.z*cos(GMP_ANGLE)-enemy_gun.y*sin(GMP_ANGLE)))/const_pi*180.0-yaw_adjust;
	aim.p=atan(enemy_gun.y/enemy_gun.z)/const_pi*180.0+pitch_adjust;
}

//*********************************************************************************************//


//**************************普通模式自瞄控制函数****************************//

void AutoAimNormal()
{
	MINMAX(aim.y,-4.0f,4.0f);
	MINMAX(aim.p,-2.0f,2.0f);
	if(find_enemy)
	{
		if(aim_cnt<1)
		{
			GMY.Target+=(aim.y+aim_rcd.y)/2;
			GMP.Target+=(aim.p+aim_rcd.p)/2;
			aim_cnt++;
		}
		else
		{
			find_enemy=0;
			aim_cnt=0;
			aim_rcd.y=aim.y;
			aim_rcd.p=aim.p;
		}
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
			aim_rcd.y=aim.y;
			aim_rcd.p=aim.p;
			GMY.Target+=aim_rcd.y/20;
			GMP.Target+=aim_rcd.p/20;
			aim_cnt++;
		}
		else if(aim_cnt>=1 && aim_cnt<20)
		{
			GMY.Target+=aim_rcd.y/20;
			GMP.Target+=aim_rcd.p/20;
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
			GMY.Target-=aim_rcd.y/20;
			GMP.Target-=aim_rcd.p/20;
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


//***********************************自瞄控制*******************************//

void AutoAimGMCTRL()
{
	switch(aim_mode)
	{
		case 1: AutoAimNormal(); break;		//自瞄
		case 2: AutoAimBuff(); break;			//打符
		case 3: break;							//吊射？
		default: break;
	}
	
	//************检测帧数*************
	if(receive_cnt==0)
		auto_counter=1000;
	if(auto_counter==0)
	{
		receive_rcd=receive_cnt;
		receive_cnt=0;
		auto_counter=1000;
	}
	//*********************************
}

//**************************************************************************//


#endif /*USE_AUTOAIM*/
#endif /*DEBUG_MODE*/
