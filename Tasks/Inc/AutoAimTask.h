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

#define RX_ENEMY_SIGNAL()		HAL_UART_Receive_DMA(&AUTOAIM_UART,(uint8_t *)&Rx_enemy_INFO,8)

#define RX_ENEMY_START		Rx_enemy_INFO[0]
#define RX_ENEMY_X1				Rx_enemy_INFO[1]
#define RX_ENEMY_X2 			Rx_enemy_INFO[2]
#define RX_ENEMY_Y1 			Rx_enemy_INFO[3]
#define RX_ENEMY_Y2				Rx_enemy_INFO[4]
#define RX_ENEMY_D1				Rx_enemy_INFO[5]
#define RX_ENEMY_D2 			Rx_enemy_INFO[6]
#define RX_ENEMY_END 			Rx_enemy_INFO[7]

#define const_pi						3.14159
#define k_coordinate   			(1*100.0/(32768.0-1.0))
#define k_distance					(1*1000.0/(32768.0-1.0))
#define bullet_speed_big		14.0
#define bullet_speed_small	20.0
#define bullet_speed_real		(bullet_speed+bullet_speed_adjust)
#define k_aim								((bullet_speed_real)*(bullet_speed_real)/(9.8*sqrt(enemy_gun.x*enemy_gun.x+enemy_gun.z*enemy_gun.z)*0.01))

typedef struct GMINFO_t
{
	double y;
	double p;
}GMINFO_t;

typedef struct Coordinate_t
{
	double x;
	double y;
	double z;
}Coordinate_t;

extern uint8_t aim_mode;
extern double bullet_speed_adjust;
void InitAutoAim(void);
void InitAutoAimUart(void);
void AutoAimRxEnemyINFO(void);
void enemyINFOProcess(void);
void autoAimGMCTRL(void);

#endif /*USE_AUTOAIM*/
#endif /*DEBUG_MODE*/

#endif /*__AUTOAIMTASK_H*/
