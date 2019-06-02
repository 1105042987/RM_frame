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
#define getLeftSr()		!HAL_GPIO_ReadPin(GPIOE,leftSensor_Pin)// //�����⵽Ϊ�͵�ƽ����ȡ�����㡣Sr��дSensor
#define getRightSr()	!HAL_GPIO_ReadPin(GPIOE,rightSensor_Pin) //

extern int8_t StateSway;
extern int8_t StateFlee;
extern int16_t StateCnt;


void routing(void);
void routing2(void);

void randing1(int8_t spd);
void randing2(int8_t spd);
	
void fleeing1(void);
void fleeing2(void);
void fleeing3(void);

void tossing(int8_t dir,int16_t tgt);
void swaying(void);
void scaning1(void);
void scaning2(void);
void firing1(void);
void firing2(void);
void firing3(void);

void remv1(void);
void remv2(void);
int sgn(float x);

