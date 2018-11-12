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
}CapControl_t;

extern CapControl_t Control_SuperCap;
extern void Cap_Control(void);
	
#endif /*__CAP_CONTROL_TASK__*/

