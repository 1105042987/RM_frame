/**
  ******************************************************************************
  * File Name          : Configuration.h
  * Description        : 机器人配置文件
  ******************************************************************************
  *
  * Copyright (c) 2018 Team JiaoLong-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
  
#ifndef __AUXDEVICE_H
#define __AUXDEVICE_H
//*****************************
//         机器人选择
//*****************************
//#define CONFIGURATION
//#define INFANTRY	2
//#define INFANTRY	4
//#define INFANTRY	5
#define GUARD		"UP"
//#define GUARD		"DOWN"
//#define ENGINEER
//#define HERO_MAIN
//#define HERO_SUB

//*********************************************************
//     机器人模块功能配置(机器人选择CONFIGUATION时启用)
//*********************************************************
#ifdef CONFIGURATION
/***模式选择***/
//#define DEBUG_MODE
//#define TEST_MODE

/***功能选择***/
//#define USE_GYRO
#define USE_IMU
#define USE_CHASSIS_FOLLOW
#define USE_SUPER_CAP
#define USE_POWER_LIMIT
#define USE_AUTOAIM

/***can线启用设置***/
#define CAN11
#define CAN12
#define CAN21
//#define CAN22

//can协议板间通信-数字为信号组数
//#define CAN13	1
#define CAN23	1

/*其他设置*/
//#define FRIC_PWM_MODE
#endif
#include "RobotDetail.h"
//***************************
//    外设组件功能配置
//***************************

#define STEER_TIM 				htim2
#define TWO_MS_TIM 				htim6
#define ONE_MS_TIM 				htim7
#define TEN_MS_TIM				htim10
#define BUZZER_TIM 				htim12

#define RC_UART 				huart1
#define JUDGE_UART 				huart6
#define GYRO_UART 				huart7
#define AUTOAIM_UART 			huart8
#define DEBUG_UART 				huart8

#define BUZZER_CHANNEL			TIM_CHANNEL_1


//IO重命名
#define Close_Laser() 	HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET)
#define Open_Laser() 	HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_RESET)


//pwm重命名
//demo
#define Open_DOOR()				__HAL_TIM_SET_COMPARE(&STEER_TIM, TIM_CHANNEL_3, 800)
#define Close_DOOR()			__HAL_TIM_SET_COMPARE(&STEER_TIM, TIM_CHANNEL_3, 1000)
#define SET_STEER_ANGLE(arg)	__HAL_TIM_SET_COMPARE(&STEER_TIM, TIM_CHANNEL_1, arg*100+500)
//demo end


#define InitPWM()\
{\
	/*HAL_TIM_PWM_Start(&STEER_TIM, TIM_CHANNEL_3);\
	HAL_TIM_PWM_Start(&STEER_TIM, TIM_CHANNEL_4);*/\
}
  
  
#endif
