/**
  ******************************************************************************
  * File Name          : AuxDevice.h
  * Description        : IO、PWM外设驱动
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
  
#ifndef __AUXDEVICE_H
#define __AUXDEVICE_H
  
#include "includes.h"

//已经定义好的pwm中断：	tim2-channel4
//						tim2-channel3
#define STEER_TIM 				htim2
#define BUZZER_TIM 				htim12

#define RC_UART 				huart1
#define MANIFOLD_UART 			huart6
#define UPPER_UART 				huart6
#define JUDGE_UART 				huart3
#define GYRO_UART 				huart7

#define BUZZER_CHANNEL			TIM_CHANNEL_1
#define GIVE_BULLET_CHANNEL		TIM_CHANNEL_4


//IO重命名
#define FUNC__CAP__RECHARGE()	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5 , GPIO_PIN_RESET);\
								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4 , GPIO_PIN_SET)
							
#define FUNC__CAP__OUTPUT()		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5 , GPIO_PIN_SET);\
								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4 , GPIO_PIN_RESET)
							
#define FUNC__CAP__CLOSE()		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5 , GPIO_PIN_RESET);\
								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4 , GPIO_PIN_RESET)
//recharge pin: 4 output pin: 5

//pwm重命名
//demo
#define Open_DOOR()				__HAL_TIM_SET_COMPARE(&STEER_TIM, GIVEBIG_CHANNEL, BDOOR_CLOSE-800)
#define Close_DOOR()			__HAL_TIM_SET_COMPARE(&STEER_TIM, GIVEBIG_CHANNEL, BDOOR_CLOSE)
#define SET_STEER_ANGLE(arg)	__HAL_TIM_SET_COMPARE(&STEER_TIM, TIM_CHANNEL_1, arg*100+500)
//demo end


#define InitPWM()\
{\
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);\
	/*HAL_TIM_PWM_Start(&STEER_TIM, GIVE_BULLET_CHANNEL);\
	__HAL_TIM_SET_COMPARE(&STEER_TIM, GIVE_BULLET_CHANNEL, DOOR_CLOSE);*/\
}
  
  
#endif
