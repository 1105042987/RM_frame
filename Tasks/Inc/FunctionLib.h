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
#define getLeftSr()		!HAL_GPIO_ReadPin(GPIOE,leftSensor_Pin)// //红外检测到为低电平，故取非运算。Sr缩写Sensor
#define getRightSr()	!HAL_GPIO_ReadPin(GPIOE,rightSensor_Pin) //

extern int8_t StateSway;
extern int8_t StateFlee;
extern int16_t StateCnt;


void routing(void);
void routing2(void);
void fleeing1(void);
void fleeing2(void);
void fleeing3(void);
void tossing(int8_t dir,int16_t tgt);
void swaying(void);
void scaning(void);
void firing1(void);
void firing2(void);
void firing3(void);
void remv(void);
int sgn(float x);

