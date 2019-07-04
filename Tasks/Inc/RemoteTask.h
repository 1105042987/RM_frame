/**
  ******************************************************************************
  * File Name          : RemoteTask.h
  * Description        : 遥控器处理任务
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#ifndef __REMOTETASK_H
#define __REMOTETASK_H

#include "includes.h"

//解算数据区
#define Pos1LER_STICK_OFFSET  1024u

#define REMOTE_SWITCH_VALUE_UP         	0x01u  
#define REMOTE_SWITCH_VALUE_DOWN		0x02u
#define REMOTE_SWITCH_VALUE_CENTRAL		0x03u

#define REMOTE_SWITCH_VALUE_BUF_DEEP   	16u


typedef __packed struct
{
	int16_t ch0;
	int16_t ch1;
	int16_t ch2;
	int16_t ch3;
	int8_t s1;
	int8_t s2;
}Remote;

typedef __packed struct
{
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t last_press_l;
	uint8_t last_press_r;
	uint8_t press_l;
	uint8_t press_r;
}Mouse;	

typedef	__packed struct
{
	uint16_t v;
}Key;

typedef __packed struct
{
	Remote rc;
	Mouse mouse;
	Key key;
}RC_Ctl_t;

typedef enum
{
	Pos1 = 1,
	Pos2 = 3,
	Pos3 = 2,
}RCMode_e;

typedef __packed struct
{
	 uint8_t switch_value_raw;      	// the current switch value
	 uint8_t switch_value1;				//  last value << 2 | value
	 uint8_t switch_value2;				//
	 uint8_t switch_long_value; 		//keep still if no switching
	 uint8_t switch_value_buf[REMOTE_SWITCH_VALUE_BUF_DEEP]; 
	 uint8_t buf_index;
	 uint8_t buf_last_index;
	 uint8_t buf_end_index;
}RemoteSwitch_t;



extern RCMode_e RCRightMode;
extern RCMode_e RCLeftMode;


extern uint8_t rc_data[18];
extern uint8_t rc_first_frame;
extern uint8_t rc_update;
extern uint8_t rc_cnt;

extern uint8_t tx_free;
extern uint8_t rx_free;

void RemoteDataProcess(uint8_t *pData);
void InitRemoteControl(void);
#ifdef SLAVE_MODE
void RCProcess1(void);
void RCProcess2(void);
void RCProcess3(void);
#else
void RCProcess1(Remote *rc);
void RCProcess2(Remote *rc);
void RCProcess3(Remote *rc);
#endif

#endif /*__ REMOTETASK_H */
