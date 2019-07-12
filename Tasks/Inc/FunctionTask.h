/**
  ******************************************************************************
  * File Name          : FunctionTask.h
  * Description        : 用于记录机器人独有的功能
  ******************************************************************************
  *
  * Copyright (c) 2019 Team JiaoLong-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#ifndef __FUNCTIONTASK_H
#define __FUNCTIONTASK_H

#include "includes.h"

//遥控常量区
#define RC_CHASSIS_SPEED_REF    		0.60f
#define RC_ROTATE_SPEED_REF 			0.05f
#define RC_GIMBAL_SPEED_REF				0.006f

#define IGNORE_RANGE 					200

//键鼠常量区
#define KEY_W			0x1
#define KEY_S			0x2
#define KEY_A			0x4
#define KEY_D			0x8
#define KEY_SHIFT		0x10
#define KEY_CTRL		0x20
#define KEY_Q			0x40
#define KEY_E			0x80
#define KEY_R			0x100
#define KEY_F			0x200
#define KEY_G			0x400
#define KEY_Z			0x800
#define KEY_X			0x1000
#define KEY_C			0x2000
#define KEY_V			0x4000
#define KEY_B			0x8000

#define NORMAL_FORWARD_BACK_SPEED 	400
#define NORMAL_LEFT_RIGHT_SPEED  		400
#define HIGH_FORWARD_BACK_SPEED 		800
#define HIGH_LEFT_RIGHT_SPEED   		800
#define LOW_FORWARD_BACK_SPEED 			200
#define LOW_LEFT_RIGHT_SPEED   			200/2

#define MOUSE_LR_RAMP_TICK_COUNT		50
#define MOUSR_FB_RAMP_TICK_COUNT		60

#define MOUSE_TO_YAW_ANGLE_INC_FACT		0.06f
#define MOUSE_TO_PITCH_ANGLE_INC_FACT	0.02f

#define MK_ROTATE_SPEED_REF 			1.20f

#define SCREEN_POSITION					80
#define BACK_POSITION           180

#define OnePush(button,execution)\
{\
	static uint8_t cache;\
	static uint8_t cnt=0;\
	if(cache != (button)){\
		cache = (button);\
		cnt = 0;\
	}\
	else if(cnt == 5){\
		if(cache) execution;\
		cnt=11;\
	}\
	else if(cnt < 5) cnt++;\
}

#define Delay(TIM,execution)\
{\
	static uint16_t time=TIM;\
	if(!time--)\
	{\
		time = TIM;\
		execution;\
	}\
}

typedef enum
{
	SHIFT,
	CTRL,
	SHIFT_CTRL,
	NO_CHANGE,
}KeyboardMode_e;

typedef enum
{
	SHORT_CLICK,
	LONG_CLICK,
	NO_CLICK,
}MouseMode_e;

typedef enum
{
	AUTOSAVING,
	FORCESAVING,
	ENDSAVING,
	CLIMBING,
	COMMON,
	INIT,
}SlaveMode_e;


typedef __packed struct
{
    int16_t forward_back_ref;
    int16_t left_right_ref;
    int16_t rotate_ref;
}ChassisSpeed_Ref_t;

extern ChassisSpeed_Ref_t ChassisSpeedRef; 
	
void funcReset(uint8_t);
void FunctionTaskInit(void);
void Limit_Position(void);
void OptionalFunction(void);
void Wheel(void);
void InitialSave(void);
void Slave_Common(void);

#endif /*__FUNCTIONTASK_H*/
