/**
  ******************************************************************************
  * File Name          : CANTask.h
  * Description        : CAN通信任务
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#ifndef __CANTASK_H
#define __CANTASK_H

#include "includes.h"

typedef __packed struct{
	uint16_t angle;
	int16_t RotateSpeed;//RPM
	uint16_t moment;
}ESCC6x0RxMsg_t;

typedef struct{
	uint16_t angle;
	int16_t realIntensity;
	int16_t giveIntensity;
}ESC6623RxMsg_t;

extern uint8_t can1_update;
extern uint8_t can2_update;
extern uint8_t can1_type;
extern uint8_t can2_type;

void InitCanReception(void);

#endif /*__ CANTASK_H */
