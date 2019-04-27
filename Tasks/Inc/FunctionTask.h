/**
  ******************************************************************************
  * File Name          : FunctionTask.h
  * Description        : ���ڼ�¼�����˶��еĹ���
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

extern int8_t stateFlee;
extern int8_t stateSway;
extern int stateCnt;
extern double chassisAdd;

#endif /*__FUNCTIONTASK_H*/
