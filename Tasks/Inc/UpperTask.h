/**
  ******************************************************************************
  * File Name          : UpperTask.h
  * Description        : 上位机处理任务，进行串口调试
  ******************************************************************************
  *
  * Copyright (c) 2019 Team JiaoLong-ShanghaiJiaoTong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#ifndef __UPPERTASK_H
#define __UPPERTASK_H

#include "includes.h"

#ifdef DEBUG_MODE
//--------------------底层接收驱动部分-------------------//
void ctrlUartRxCpltCallback(void);
void ctrlUartInit(void);
#define REC_LEN				100
extern char buf[REC_LEN];

//--------------------数据解析协议部分-------------------//
uint8_t ComProtocal(char*rxbuf,char*head,char*end,char* separater,char dataout[15]);

//--------------------任务循环部分-------------------//
//void zykProcessData();
void dataCallBack(void);
#endif 

#endif /*__ UPPERTASK_H */
