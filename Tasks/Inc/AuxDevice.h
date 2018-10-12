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
#define STEER_TIM 				&htim2
#define BUZZER_TIM 				&htim12

#define BUZZER_CHANNEL			TIM_CHANNEL_1
#define GIVE_BULLET_CHANNEL		TIM_CHANNEL_4


//IO重命名
#define LED_GREEN_TOGGLE() HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin)
#define LED_RED_TOGGLE()   HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin)
#define LED_GREEN_OFF()    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin,GPIO_PIN_SET)
#define LED_RED_OFF()      HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin,GPIO_PIN_SET)
#define LED_GREEN_ON()     HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin,GPIO_PIN_RESET)
#define LED_RED_ON()       HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin,GPIO_PIN_RESET)

//pwm重命名
//demo
#define Open_DOOR()				__HAL_TIM_SET_COMPARE(STEER_TIM, GIVEBIG_CHANNEL, BDOOR_CLOSE-800)
#define Close_DOOR()			__HAL_TIM_SET_COMPARE(STEER_TIM, GIVEBIG_CHANNEL, BDOOR_CLOSE)
//demo end


#define InitPWM()\
{\
	/*HAL_TIM_PWM_Start(STEER_TIM, GIVE_BULLET_CHANNEL);\
	__HAL_TIM_SET_COMPARE(STEER_TIM, GIVE_BULLET_CHANNEL, DOOR_CLOSE);*/\
}
  
  
#endif
