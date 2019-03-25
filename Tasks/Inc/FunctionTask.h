/**
  ******************************************************************************
  * File Name          : FunctionTask.h
  * Description        : 用于记录机器人独有的功能
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#ifndef __FUNCTIONTASK_H
#define __FUNCTIONTASK_H

#include "includes.h"

//遥控常量区
#define RC_CHASSIS_SPEED_REF    		0.60f
#define RC_ROTATE_SPEED_REF 			0.05f
#define RC_GIMBAL_SPEED_REF				0.006f

#define IGNORE_RANGE 					200

#define NORMAL_FORWARD_BACK_SPEED 		400
#define NORMAL_LEFT_RIGHT_SPEED  		400/2
#define HIGH_FORWARD_BACK_SPEED 		700
#define HIGH_LEFT_RIGHT_SPEED   		700/2
#define LOW_FORWARD_BACK_SPEED 			200
#define LOW_LEFT_RIGHT_SPEED   			200/2

#define onePush(button,execution){\
	static uint8_t cache;\
	static uint8_t cnt=0;\
	if(cache != (button)){cache = (button);cnt = 0;}\
	else if(cnt == 5){if(cache) execution;cnt=11;}\
	else if(cnt < 5) cnt++;\
}
#define onePushShtSpd(button,execution){\
	static uint8_t cache;\
	static uint8_t cnt=0;\
	if(cache!=(button)){cache=(button);cnt=0;}\
	else if(cnt==5){if(cache)execution;cnt=11;}\
	else if(cnt<5)cnt++;\
}
#define onePushDir(button,execution){\
	static uint8_t cache;\
	static uint8_t cnt=0;\
	if(cache != (button)){cache = (button);cnt = 0;}\
	else if(cnt == 5){if(cache) execution;cnt=11;}\
	else if(cnt < 5) cnt++;\
}
#define Delay(TIM,execution){\
	static uint16_t time=TIM;\
	if(!time--){time = TIM;execution;}\
}

typedef __packed struct{
    int16_t forward_back_ref;
    int16_t left_right_ref;
    int16_t rotate_ref;
}ChassisSpeed_Ref_t;

extern ChassisSpeed_Ref_t ChassisSpeedRef; 
extern int32_t auto_counter;

void FunctionTaskInit(void);

#endif /*__FUNCTIONTASK_H*/
