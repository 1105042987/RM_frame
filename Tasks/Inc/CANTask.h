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
	int16_t angle;
	int16_t rotateSpeed;//RPM
	int16_t moment;
}ESCC6x0RxMsg_t;

typedef struct{
	int16_t angle;
	int16_t realIntensity;
	int16_t giveIntensity;
}ESC6623RxMsg_t;

typedef struct{
	int16_t data[4];
}CAN_DATA_t;

extern uint8_t can1_update;
extern uint8_t can2_update;
extern uint8_t can1_type;
extern uint8_t can2_type;

extern uint8_t isCan11FirstRx;
extern uint8_t isCan12FirstRx;
extern uint8_t isCan21FirstRx;
extern uint8_t isCan22FirstRx;

void InitCanReception(void);
void setCANMessage(uint8_t index);

#endif /*__ CANTASK_H */
