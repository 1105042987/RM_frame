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
#define getMidSr()		!HAL_GPIO_ReadPin(GPIOE,middleSensor_Pin)
#define brakeOn()			HAL_GPIO_WritePin(GPIOH,brake_Pin,1)
#define brakeOff()		HAL_GPIO_WritePin(GPIOH,brake_Pin,0)


extern int8_t StateSway,StateFlee,NutCnt,Anchor;
extern int16_t StateCnt,SpeedRef;

int sgn(float x);

void routing0(void);
void routing1(void);
void routing2(void);
void routing3(void);
void routing4(void);
void routingL(void);

void nutDetect(void);

void scaning1(void);
void scaning2(void);
void scaning3(void);

void firing1(void);
void firing2(void);
void firing3(void);//发光测试
void firing5m(void);//5m外
void firing5m2(void);//5m外
void aimAtBox(void);
void aimAtBase(void);
void uartSend(void);
