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

#define LED_GREEN_TOGGLE() HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin)
#define LED_RED_TOGGLE()   HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin)
#define LED_GREEN_OFF()    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin,GPIO_PIN_SET)
#define LED_RED_OFF()      HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin,GPIO_PIN_SET)
#define LED_GREEN_ON()     HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin,GPIO_PIN_RESET)
#define LED_RED_ON()       HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin,GPIO_PIN_RESET)
  
//IO重命名
#define STEER_TIM 			&htim2
#define BYPASS_TIM 			&htim12
#define YAW_CHANNEL			TIM_CHANNEL_2
//#define GIVESML_CHANNEL	TIM_CHANNEL_3
#define GIVEBIG_CHANNEL		TIM_CHANNEL_4
//#define SDOOR_CLOSE 		1500
#define BDOOR_CLOSE 		2130
#define E_MAGNET_IO 		GPIOI, GPIO_PIN_2
//#define M_VAVLE_FB_IO		GPIOI, GPIO_PIN_12
#define M_VAVLE_OC_IO		GPIOH, GPIO_PIN_0

#define Open_ElectroMagnet()	HAL_GPIO_WritePin(GPIOI, GPIO_PIN_2, GPIO_PIN_SET)
#define Close_ElectroMagnet()	HAL_GPIO_WritePin(GPIOI, GPIO_PIN_2, GPIO_PIN_RESET)
#define Open_Gripper()			HAL_GPIO_WritePin(GPIOI, GPIO_PIN_0, GPIO_PIN_SET)
#define Close_Gripper()			HAL_GPIO_WritePin(GPIOI, GPIO_PIN_0, GPIO_PIN_RESET)
//#define Open_SDOOR()			__HAL_TIM_SET_COMPARE(STEER_TIM, GIVESML_CHANNEL, SDOOR_CLOSE-900)
//#define Close_SDOOR()			__HAL_TIM_SET_COMPARE(STEER_TIM, GIVESML_CHANNEL, SDOOR_CLOSE)
#define Open_BDOOR()			__HAL_TIM_SET_COMPARE(STEER_TIM, GIVEBIG_CHANNEL, BDOOR_CLOSE-800)
#define Close_BDOOR()			__HAL_TIM_SET_COMPARE(STEER_TIM, GIVEBIG_CHANNEL, BDOOR_CLOSE)


#define InitPWM()\
{\
	/*HAL_TIM_PWM_Start(STEER_TIM, YAW_CHANNEL);\
	__HAL_TIM_SET_COMPARE(STEER_TIM, YAW_CHANNEL,0);\
	HAL_TIM_PWM_Start(BYPASS_TIM, TIM_CHANNEL_1);\
	HAL_TIM_PWM_Start(STEER_TIM, GIVESML_CHANNEL);\
	__HAL_TIM_SET_COMPARE(STEER_TIM, GIVESML_CHANNEL, SDOOR_CLOSE);*/\
	HAL_TIM_PWM_Start(STEER_TIM, GIVEBIG_CHANNEL);\
	__HAL_TIM_SET_COMPARE(STEER_TIM, GIVEBIG_CHANNEL, BDOOR_CLOSE);\
}
  
  
#endif
