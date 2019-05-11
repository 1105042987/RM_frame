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
#define CHANGE_POINT 1500
#define CHANGE_POINTr 1500
#define CHANGE_POINTbl 1900//1500
#define CHANGE_POINTbr 1500//1100
#define CHANGE_POINTdb 1000//1100
//1v/5v 	27cm

#define UD_TOP			0
#define UD_BOTTOM		-1030
//-1010
#define FLAG_SET(target) if(target.val_ref<CHANGE_POINT) target.flag = 0; else target.flag = 1;
#define FLAG_SETr(target) if(target.val_ref<CHANGE_POINTr) target.flag = 0; else target.flag = 1;
#define FLAG_SETbl(target) if(target.val_ref>CHANGE_POINTbl) target.flag = 0; else target.flag = 1;
#define FLAG_SETbr(target) if(target.val_ref>CHANGE_POINTbr) target.flag = 0; else target.flag = 1;
#define FLAG_SETdb(target) if(target.val_ref<CHANGE_POINTdb) target.flag = 0; else target.flag = 1;
#define FLAG_SET_TRICK(target) if((AutoClimbing==1&&ChassisSpeedRef.forward_back_ref>0&&NMCDL.RealAngle>-230)||(AutoClimbing==1&&NMCDL.RealAngle<-950)) target.flag = 1; else target.flag = 0;

#define ON_THE_GROUND AutoClimb_Level==0
#define ON_THE_FLOOR AutoClimb_Level==2
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
	COMMON_STATE,
	GET_STATE,
	CLIMB_STATE,
	RESCUE_STATE
}Engineer_State_e;

typedef enum
{
	NORMAL_VIEW,
	REVERSE_VIEW,
	GET_VIEW
}View_State_e;
typedef enum
{
	DEBUG_GET_STATE,
	DEBUG_CLIMB_STATE,
}Debug_State_e;

extern uint8_t AlreadyClimbed;
extern uint8_t AlreadyDowned;
extern uint32_t AutoClimbing;
extern uint32_t AutoClimb_Level;

void Chassis_Choose(uint8_t flag,uint8_t ensure);
void RefreshAnologRead(void);
void ComeToTop(void);
void AutoClimb_SwitchState(void);
void State_AutoClimb(void);

#endif //__AUTO_CLIMB_TASK_H
