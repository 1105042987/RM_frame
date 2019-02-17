/**
  ******************************************************************************
  * File Name          : AutoAimtask.h
  * Description        : 
  ******************************************************************************
  *
  * Copyright (c) 2019 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */

#ifndef __AUTOAIMTASK_H
#define __AUTOAIMTASK_H

#include "includes.h"

#ifndef DEBUG_MODE
#ifdef	USE_AUTOAIM

#define GMP_ANGLE		(double)((GMP.RxMsg6623.angle-GM_PITCH_ZERO)/8192.0*2*const_pi)

#define RX_ENEMY_START		Enemy_INFO[0]
#define RX_ENEMY_X1				Enemy_INFO[1]
#define RX_ENEMY_X2 			Enemy_INFO[2]
#define RX_ENEMY_Y1 			Enemy_INFO[3]
#define RX_ENEMY_Y2				Enemy_INFO[4]
#define RX_ENEMY_Z1				Enemy_INFO[5]
#define RX_ENEMY_Z2 			Enemy_INFO[6]
#define RX_ENEMY_END 			Enemy_INFO[7]

#define const_pi						3.14159
#define coordinate_max			100
#define k_coordinate   			(1*100.0/(32768.0-1.0))
#define k_distance					(1*1000.0/(32768.0-1.0))
#define bullet_speed_real		(bullet_speed+bullet_speed_adjust)
#define k_aim								((bullet_speed_real)*(bullet_speed_real)/(9.8*(enemy_gun.z*cos(GMP_ANGLE)-enemy_gun.y*sin(GMP_ANGLE))*0.01))

typedef struct GMINFO_t
{
	float y;
	float p;
}GMINFO_t;

typedef struct Coordinate_t
{
	float x;
	float y;
	float z;
}Coordinate_t;

extern uint8_t aim_mode;
extern double bullet_speed_adjust,yaw_adjust,pitch_adjust;

void InitAutoAim(void);
void AutoAimUartRxCpltCallback(void);
void EnemyINFOProcess(void);
void AutoAimNormal(void);
void AutoAimBuff(void);
void AutoAimGMCTRL(void);

#endif /*USE_AUTOAIM*/
#endif /*DEBUG_MODE*/

#endif /*__AUTOAIMTASK_H*/
