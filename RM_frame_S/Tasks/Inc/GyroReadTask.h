/*******************************************************************
  * File Name   : GyroReadTask.h
  * Description : 
  * Author      : «ÿ…‹∑…
  * Telephone   :
********************************************************************
  *
  * Copyright (c) 2019 Team Jiao Long - ShanghaiJiaoTong University
  * All rights reserved.
  *
*******************************************************************/
#include "includes.h"

void InitGyroUart(void);
void gyroUartRxCpltCallback(void);

extern float gyroXAngle,gyroYAngle,gyroZAngle;
extern float gyroXspeed,gyroYspeed,gyroZspeed;
extern float gyroXacc,gyroYacc,gyroZacc;
