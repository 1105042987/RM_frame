/**
  ******************************************************************************
  * File Name          : CapControlTask.h
  * Description        : 
  ******************************************************************************
  *
  * Copyright (c) 2019 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
	
#include "includes.h"
#ifndef __CAP_CONTROL_TASK__
#define __CAP_CONTROL_TASK__

typedef __packed struct
{
	uint8_t release_power;
	uint8_t stop_power;
	uint16_t C_voltage;
}CapControl_t;

extern CapControl_t Control_SuperCap;
extern void Cap_Control(void);

#define FUNC__CAP__RECHARGE()	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5 , GPIO_PIN_RESET);\
								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4 , GPIO_PIN_SET)
							
#define FUNC__CAP__OUTPUT()		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5 , GPIO_PIN_SET);\
								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4 , GPIO_PIN_RESET)
							
#define FUNC__CAP__CLOSE()		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5 , GPIO_PIN_RESET);\
								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4 , GPIO_PIN_RESET)

//recharge pin: 4 output pin: 5
	
#endif /*__CAP_CONTROL_TASK__*/

