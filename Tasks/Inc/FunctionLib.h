/**
  ******************************************************************************
  * File Name          : FunctionLib.h
  * Description        : �ڱ����ܺ�����
  ******************************************************************************
  *
  * Copyright (c) 2019 Team JiaoLong-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#include "includes.h"
#define getLeftSr()		!HAL_GPIO_ReadPin(GPIOE,leftSensor_Pin)		//�����⵽Ϊ�͵�ƽ����ȡ�����㡣Sr��дSensor
#define getRightSr()	!HAL_GPIO_ReadPin(GPIOE,rightSensor_Pin)
#define getLeftSw()		!HAL_GPIO_ReadPin(GPIOE,leftSwitch_Pin)		//΢������
#define getRightSw()	!HAL_GPIO_ReadPin(GPIOE,rightSwitch_Pin)

extern int8_t StateSway;
extern int8_t StateFlee;
extern int16_t StateCnt;

int sgn(float x);

void routing1(void);
void routing2(void);
void routing3(void);


void scaning1(void);
void scaning2(void);
void scaning3(void);

void firing1(void);
void firing2(void);

void uartSend(void);
