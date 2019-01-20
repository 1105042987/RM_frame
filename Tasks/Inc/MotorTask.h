/**
  ******************************************************************************
  * File Name          : CANMotor.h
  * Description        : CAN线电机控制
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#ifndef __CANMOTOR_H
#define __CANMOTOR_H

#include "includes.h"
/*
#if INFANTRY==2
#define GM_PITCH_GRAVITY_COMPENSATION 0
#define GM_PITCH_ZERO 	3740
#define GM_YAW_ZERO 	1310
#define MAX_YGAP		40
#define MAX_PGAP		20
#endif
#if INFANTRY==4
#define GM_PITCH_GRAVITY_COMPENSATION 800
#define GM_PITCH_ZERO 	7788
#define GM_YAW_ZERO 	4640
#define MAX_YGAP		30
#define MAX_PGAP		15
#endif


#ifdef TEST_MODE
#define GM_PITCH_GRAVITY_COMPENSATION -150
#define GM_PITCH_ZERO 	6700
#define GM_YAW_ZERO 	1900
#define MAX_YGAP		40
#define MAX_PGAP		20
#endif
#ifdef GUARD
//未测试
#define GM_PITCH_GRAVITY_COMPENSATION 0
#define GM_PITCH_ZERO 	3740
#define GM_YAW_ZERO 	1310
#endif
*/
#define CHASSIS_SPEED_ATTENUATION   (1.30f)
#define NORMALIZE_ANGLE180(angle) angle = ((angle) > 180) ? ((angle) - 360) : (((angle) < -180) ? (angle) + 360 : angle)
#define CHASSIS_MOTOR_ROTATE_PID_DEFAULT \
{\
	0,0,{0,0},\
	1.1f,0.0f,0.5f,/*p i d*/\
	0,0,0,\
	20,20,20,\
	0,20,0,0,0,\
	&PID_Calc,&PID_Reset,\
}

#define CHASSIS_MOTOR_SPEED_PID_DEFAULT \
{\
	0,0,{0,0},\
	12.0f,0.17f,8.0f,\
	0,0,0,\
	10000,10000,10000,\
	0,7000,0,0,0,\
	&PID_Calc,&PID_Reset,\
}

#define FW_PID_DEFAULT \
{ \
	0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ,\
	0, 0, 0, 0.0, 0.0, 0.0, \
	0, 0, 0, 0.0, 0, \
	{0.0}, \
	&fw_PID_Calc, &fw_PID_Reset \
}

typedef enum
{
	ESC_C6x0=0,
	ESC_6623=1
}ESCtype_e;

typedef struct MotorINFO
{
	ESCtype_e			ESCtype;
	CAN_HandleTypeDef* 	CAN_TYPE;
	uint16_t 			TXID;
	uint16_t			RXID;
	float 				ReductionRate;
	ESCC6x0RxMsg_t		RxMsgC6x0;
	ESC6623RxMsg_t		RxMsg6623;
	double 				Target;
	uint8_t				s_count;
	uint8_t 			FirstEnter;
	double 				lastRead;
	double 				Real;
	void (*Handle)(struct MotorINFO* id);
	fw_PID_Regulator_t 	positionPID;
	fw_PID_Regulator_t 	speedPID;
	PID_Regulator_t		offical_speedPID;
	int16_t				Intensity;
}MotorINFO;

#define AngleBased_MOTORINFO_Init(rdc,func,ppid,spid)\
{\
	ESC_C6x0,0,0,0,rdc,\
	{0,0,0},{0,0,0},0,0,1,0,0,func,\
	ppid,spid,CHASSIS_MOTOR_SPEED_PID_DEFAULT,0 \
}

#define SpeedBased_MOTORINFO_Init(func,spid)\
{\
	ESC_C6x0,0,0,0,1,\
	{0,0,0},{0,0,0},0,0,1,0,0,func,\
	FW_PID_DEFAULT,FW_PID_DEFAULT,spid,0 \
}

#define Gimbal_MOTORINFO_Init(rdc,func,zero,compensation,maxrange,ppid,spid)\
{\
	ESC_6623,0,0,0,rdc,\
	{zero,compensation,maxrange},{0,0,0},0,0,1,0,0,func,\
	ppid,spid,CHASSIS_MOTOR_SPEED_PID_DEFAULT,0 \
}


extern MotorINFO *can1[8],*can2[8];
void InitMotor(MotorINFO *id);
void Motor_ID_Setting(void);

void setCAN11(void);
void setCAN12(void);
void setCAN21(void);
void setCAN22(void);



#endif /*__ CANMOTOR_H */
