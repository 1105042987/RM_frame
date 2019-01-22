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

GMINFO_t aim;
Coordinate_t enemy_gun,enemy_scope,scope_gun;
uint8_t Rx_enemy_INFO[8];
uint8_t find_enemy=0,aim_cnt=0,aim_mode=0;//后续在function种加入aim_mode，供操作手选择是否吊射
double bullet_speed=bullet_speed_big,bullet_speed_adjust=0;//后续在function中加入bullet_speed_adjust，供操作手手动校准

void InitAutoAim()
{
	InitAutoAimUart();
	scope_gun.x=0;
	scope_gun.y=-7.0;
	scope_gun.z=5.0;
}

void InitAutoAimUart()
{
	if(HAL_UART_Receive_DMA(&AUTOAIM_UART,(uint8_t *)&Rx_enemy_INFO,8)!= HAL_OK)
	{
		Error_Handler();
	}
}

void AutoAimRxEnemyINFO()
{
	if(RX_ENEMY_START=='s'&&RX_ENEMY_END=='e')
	{
		enemy_scope.x=(double)((RX_ENEMY_X1<<8)|RX_ENEMY_X2)*k_coordinate;
		enemy_scope.y=(double)((RX_ENEMY_Y1<<8)|RX_ENEMY_Y2)*k_coordinate;
		enemy_scope.z=(double)((RX_ENEMY_D1<<8)|RX_ENEMY_D2)*k_distance;
		enemy_scope.x=(enemy_scope.x>100)?(enemy_scope.x-200):enemy_scope.x;
		enemy_scope.y=(enemy_scope.y>100)?(enemy_scope.y-200):enemy_scope.y;
		find_enemy=1;
	}
	RX_ENEMY_SIGNAL();
}

void enemyINFOProcess()
{
	enemy_gun.x=enemy_scope.x+scope_gun.x;
	enemy_gun.y=enemy_scope.y+scope_gun.y;
	enemy_gun.z=enemy_scope.z+scope_gun.z;
	aim.y=atan(enemy_gun.x/enemy_gun.z)/const_pi*180.0;
	aim.p=atan(enemy_gun.y/enemy_gun.z)/const_pi*180.0+(GMP.RxMsg6623.angle - GM_PITCH_ZERO) * 360.0 / 8192.0 ;
	//aim.p=atan(k_aim+((aim_mode)?(1.0):(-1.0))*sqrt(k_aim*k_aim-2.0*k_aim*enemy.y/sqrt(enemy.x*enemy.x+enemy.z*enemy.z)-1.0))/const_pi*180.0-GMP.RealAngle;
}

void autoAimGMCTRL()
{
	enemyINFOProcess();
	if(find_enemy)
	{
		if(aim_cnt<5)
		{
			GMY.TargetAngle+=aim.y/5;
			GMP.TargetAngle-=aim.p/5;
			aim_cnt++;
		}
		else
		{
			aim_cnt=0;
			find_enemy=0;
		}
	}
}

#endif /*USE_AUTOAIM*/
#endif /*DEBUG_MODE*/
