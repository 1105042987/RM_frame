/**
  ******************************************************************************
  * File Name          : ControlTask.h
  * Description        : 主控制任务
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#ifndef __CONTROLTASK_H
#define __CONTROLTASK_H

#include "includes.h"

//When you change the struct WorkState_e please ensure that 
//*********************************************
//all your element's value must more than zero 
//*********************************************
//execpt the stop and prepare state.

typedef enum
{
	STOP_STATE=-1,
	PREPARE_STATE=0, 
	NORMAL_STATE=1,
	ADDITIONAL_STATE_ONE=2,
	ADDITIONAL_STATE_TWO=3,
}WorkState_e;

extern WorkState_e WorkState;
void WorkStateFSM(void);

#endif /*__ CONTROLTASK_H */
