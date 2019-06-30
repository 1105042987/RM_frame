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


void routing1(void);

void scaning1(void);
void scaning2(void);
void firing1(void);
void firing2(void);
void firing3(void);

int sgn(float x);

