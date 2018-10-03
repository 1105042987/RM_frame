/**
  ******************************************************************************
  * File Name          : GMControlTask.h
  * Description        : 云台电机控制任务
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
	* 
	* 
  ******************************************************************************
  */
#ifndef GM_CONTROL_TASK_H
#define GM_CONTROL_TASK_H

#include "includes.h"

#define GMYAWReduction 96.0
#define GMPITCHReduction 36.0

#define TURN_BACK 160

extern float GSYAW_ZERO;

#define GS_RESET(DIR)\
{\
	if(GSYAW_ZERO < 40) \
		{\
			GSYAW_ZERO += TURN_BACK;\
			GMPITCHAngleTarget = 0;\
			DIR=1;\
		}\
}\

#define GS_SET(DIR)\
{\
	if(GSYAW_ZERO > 120) \
		{\
			GSYAW_ZERO -= TURN_BACK;\
			GMPITCHAngleTarget = 0;\
			DIR=-1;\
		}\
}\

#define GS_REVERSAL(DIR)\
{\
	if(GSYAW_ZERO < 40) \
		{\
			GSYAW_ZERO += TURN_BACK;\
			GMPITCHAngleTarget = 0;\
			DIR=1;\
		}\
	else if(GSYAW_ZERO > 120) \
		{\
			GSYAW_ZERO -= TURN_BACK;\
			GMPITCHAngleTarget = 0;\
			DIR=-1;\
		}\
}\

#define GMANGLE_STEP 1.5

void ControlGSYAW(void);
void ControlGMYAW(void);
void ControlGMPITCH(void);
void setGMMotor(void);
void GMControlInit(void);

#endif //GM_CONTROL_TASK_H
