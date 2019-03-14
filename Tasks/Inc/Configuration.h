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
	//#define USE_POWER_LIMIT 	20
	#define CAN21
	#define CAN22
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

#define RC_UART 				huart1
#define JUDGE_UART 				huart6
#define GYRO_UART 				huart7
#define AUTOAIM_UART 			huart8
#define DEBUG_UART 				huart8

#define BUZZER_CHANNEL			TIM_CHANNEL_1


//IO������
#define FUNC__LASER__CLOSE() 	HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET)
#define FUNC__LASER__OPEN() 	HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_RESET)
#define FUNC__RED_RAY_M__READ()	HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_5)
#define FUNC__RED_RAY_L__READ()	HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5)
#define FUNC__RED_RAY_R__READ()	HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_10)


//pwm������
//demo
#define FUNC__BULLET_DOOR__CLOSE()			__HAL_TIM_SET_COMPARE(&STEER_TIM, TIM_CHANNEL_3, 800)
#define FUNC__BULLET_DOOR__OPEN()			__HAL_TIM_SET_COMPARE(&STEER_TIM, TIM_CHANNEL_3, 1000)
#define FUNC__STEER_ANGLE__SET(arg)		__HAL_TIM_SET_COMPARE(&STEER_TIM, TIM_CHANNEL_1, arg*100+500)
//demo end


#define InitPWM()\
{\
	/*HAL_TIM_PWM_Start(&STEER_TIM, TIM_CHANNEL_3);\
	HAL_TIM_PWM_Start(&STEER_TIM, TIM_CHANNEL_4);*/\
}
  
  
#endif
