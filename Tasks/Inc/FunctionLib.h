/**
  ******************************************************************************
  * File Name          : FunctionLib.h
  * Description        : 哨兵功能函数集
  ******************************************************************************
  *
  * Copyright (c) 2019 Team JiaoLong-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#include "includes.h"
#define getLeftSr()		!HAL_GPIO_ReadPin(GPIOE,leftSensor_Pin)		//红外检测到为低电平，故取非运算。Sr缩写Sensor
#define getRightSr()	!HAL_GPIO_ReadPin(GPIOE,rightSensor_Pin)
#define getLeftSw()		!HAL_GPIO_ReadPin(GPIOE,leftSwitch_Pin)		//微动开关
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

