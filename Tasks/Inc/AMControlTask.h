/**
  ******************************************************************************
  * File Name          : AMControlTask.h
  * Description        : 机械臂电机控制任务
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
	* 
	* 
  ******************************************************************************
  */
#ifndef AM_CONTROL_TASK_H
#define AM_CONTROL_TASK_H

#include "includes.h"

#define AMReduction 19.0
#define BYPASS_TIM htim12

#define AMFB_ANGLE_STEP -3.5
#define AMUD_ANGLE_STEP 2.8

void vice_controlLoop(void);
void AMControlInit(void);

#endif //AM_CONTROL_TASK_H
