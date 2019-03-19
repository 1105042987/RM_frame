/**
  ******************************************************************************
  * File Name          : AutoClimbTask.h
  * Description        : 自动上岛控制任务
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#ifndef __AUTO_CLIMB_TASK_H
#define __AUTO_CLIMB_TASK_H

#include "includes.h"

#define GETHIGHT 400
#define CHANGE_POINT 2000
#define CHANGE_POINTr 1700
//1v/5v 	27cm

#define UD_TOP			0
#define UD_BOTTOM		-1055
//-1010
#define FLAG_SET(target) if(target.val_ref<CHANGE_POINT) target.flag = 0; else target.flag = 1;
#define FLAG_SETr(target) if(target.val_ref<CHANGE_POINTr) target.flag = 0; else target.flag = 1;

typedef __packed struct
{
	uint32_t val_ref;
	int8_t flag;						//1阻断，0开放
}Distance_Sensor_t;

typedef __packed struct
{
	Distance_Sensor_t frontf;
	Distance_Sensor_t frontr;
	Distance_Sensor_t frontl;
	Distance_Sensor_t backb;
	Distance_Sensor_t backr;
	Distance_Sensor_t backl;
	Distance_Sensor_t left;
	Distance_Sensor_t right;
	uint16_t move_flags;
}Distance_Couple_t;

//move_flags 16进制：编码准则：lo li ri ro
//偏离   |对 准|    中 间    |对 准|   偏离
//0 1   3 2 6 4 c   d f b   3 2 6 4 c   8 0
//----------------------------------------> 'r'
//<---------------------------------------- 'l'

typedef enum
{
	NOAUTO_STATE,
	START_TEST,
	LEVEL_SHIFT,
	ARM_STRETCH,
	ERROR_HANDLE
}Engineer_State_e;

void Chassis_Choose(uint8_t flag,uint8_t ensure);
void RefreshAnologRead(void);
void ComeToTop(void);

#endif //__AUTO_CLIMB_TASK_H
