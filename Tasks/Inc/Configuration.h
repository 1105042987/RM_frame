/**
  ******************************************************************************
  * File Name          : Configuration.h
  * Description        : �����������ļ�
  ******************************************************************************
  *
  * Copyright (c) 2018 Team JiaoLong-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
  
#ifndef __AUXDEVICE_H
#define __AUXDEVICE_H

#define GUARD		'U'
//#define GUARD		'D'

#if GUARD == 'U'
	#define USE_POWER_LIMIT 	20
	#define CAN21
	#define CAN13 	1
#else
	#define SLAVE_MODE
	#define USE_IMU
	#define USE_AUTOAIM
	
	#define CAN11
	#define CAN12
	#define CAN13	1
#endif
//***************************
//    ���������������
//***************************

#define STEER_TIM 				htim2
#define TWO_MS_TIM 				htim6
#define ONE_MS_TIM 				htim7
#define TEN_MS_TIM				htim10
#define BUZZER_TIM 				htim12

#define RC_UART 					huart1
#define JUDGE_UART 				huart6
#define GYRO_UART 				huart7
#define AUTOAIM_UART 			huart8
#define DEBUG_UART 				huart8

#define BUZZER_CHANNEL			TIM_CHANNEL_1


//IO������
#define laserOn() 	HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET)
#define laserOff() 	HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_RESET)
#define FUNC__RED_RAY_M__READ()	HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_5)
#define FUNC__RED_RAY_L__READ()	HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5)
#define FUNC__RED_RAY_R__READ()	HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_10)
#endif
