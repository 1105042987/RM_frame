/**
  ******************************************************************************
  * File Name          : UpperTask.h
  * Description        : 上位机处理任务，进行串口调试
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
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

#define REC_LEN            1024
#define RX_DONE           (RX_STA&0x8000)
#define RX_LEN	  			   (RX_STA&0x3f)

extern uint8_t buf[REC_LEN];
extern uint16_t RX_STA;

//--------------------数据解析协议部分-------------------//
uint8_t ComProtocal(char*rxbuf,char*head,char*end,char* separater,char dataout[][15]);

//--------------------任务循环部分-------------------//
//void zykProcessData();
void dataCallBack(void);
#endif 

#endif /*__ UPPERTASK_H */
