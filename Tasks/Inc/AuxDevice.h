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

#define STEER_TIM 				htim2
#define BUZZER_TIM 				htim12

#define RC_UART 					huart1
#define JUDGE_UART 				huart6
#define GYRO_UART 				huart7
#define AUTOAIM_UART 			huart8
#define UPPER_UART 				huart8

#define BUZZER_CHANNEL			TIM_CHANNEL_1


//IO重命名



//pwm重命名
//demo
#define Open_DOOR()				__HAL_TIM_SET_COMPARE(&STEER_TIM, TIM_CHANNEL_3, 800)
#define Close_DOOR()			__HAL_TIM_SET_COMPARE(&STEER_TIM, TIM_CHANNEL_3, 1000)
#define SET_STEER_ANGLE(arg)	__HAL_TIM_SET_COMPARE(&STEER_TIM, TIM_CHANNEL_1, arg*100+500)
//demo end


#define InitPWM()\
{\
	/*HAL_TIM_PWM_Start(&STEER_TIM, GIVE_BULLET_CHANNEL);\
	__HAL_TIM_SET_COMPARE(&STEER_TIM, GIVE_BULLET_CHANNEL, DOOR_CLOSE);*/\
}
  
  
#endif
