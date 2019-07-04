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

#define onePush(button,execution){\
	static int8_t cache=10;\
	static int8_t cnt=0;\
	if(cache != (button)){cache = (button);cnt=0;}\
	else if(cnt < 5){cnt++;}\
	else if(cnt == 5){execution;cnt=6;}\
}
#define onePushShtSpd(button,execution){\
	static int8_t cache=10;\
	static int8_t cnt=0;\
	if(cache != (button)){cache = (button);cnt=0;}\
	else if(cnt < 5){cnt++;}\
	else if(cnt == 5){execution;cnt=6;}\
}
#define onePushDir(button,execution){\
	static int8_t cache=10;\
	static int8_t cnt=0;\
	if(cache != (button)){cache = (button);cnt=0;}\
	else if(cnt < 5){cnt++;}\
	else if(cnt == 5){execution;cnt=6;}\
}
#define Delay(TIM,execution){\
	static uint16_t time=TIM;\
	if(!time--){time = TIM;execution;}\
}


extern int32_t auto_counter;

void FunctionTaskInit(void);

extern int8_t StateFlee;
extern int8_t StateSway;
extern int8_t StateHurt;
extern int16_t StateCnt;
extern double ChassisSpeed;
extern int16_t channelrrow;
extern int16_t channelrcol;
extern int16_t channellrow;
extern int16_t channellcol;


void limtSync(void);
#endif /*__FUNCTIONTASK_H*/
