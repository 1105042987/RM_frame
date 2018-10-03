/**
  ******************************************************************************
  * File Name          : pid_regulator.h
  * Description        : PID函数
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * 两套函数
	* fw_前缀为队伍自己编写
	* 无前缀为官方PID，实现封装在RMLib.lib
  ******************************************************************************
  */
#ifndef _PID_REGULATOR_H_
#define _PID_REGULATOR_H_
#include "stm32f4xx.h"

#define MINMAX(value, min, max) value = (value < min) ? min : (value > max ? max : value)
#define fw_PID_INIT(Kp, Ki, Kd, KpMax, KiMax, KdMax, OutputMax) { \
	0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ,\
	Kp, Ki, Kd, 0.0, 0.0, 0.0, \
	KpMax, KiMax, KdMax, 0.0, \
	OutputMax, \
	&fw_PID_Calc, &fw_PID_Reset \
}
typedef __packed struct fw_PID_Regulator_t
{
	float target;
	float feedback;
	float errorCurr;
	float errorSum;
	uint16_t SumCount;
	float errorLast;
	float kp;
	float ki;
	float kd;
	float componentKp;
	float componentKi;
	float componentKd;
	float componentKpMax;
	float componentKiMax;
	float componentKdMax;
	float output;
	float outputMax;
	
	void (*Calc)(struct fw_PID_Regulator_t *pid);
	void (*Reset)(struct fw_PID_Regulator_t *pid);
}fw_PID_Regulator_t;

void fw_PID_Reset(fw_PID_Regulator_t *pid);
void fw_PID_Calc(fw_PID_Regulator_t *pid);

typedef struct PID_Regulator_t
{
	float ref;
	float fdb;
	float err[2];
	float kp;
	float ki;
	float kd;
	float componentKp;
	float componentKi;
	float componentKd;
	float componentKpMax;
	float componentKiMax;
	float componentKdMax;
	float output;
	float outputMax;
	float kp_offset;
	float ki_offset;
	float kd_offset;
	void (*Calc)(struct PID_Regulator_t *pid);
	void (*Reset)(struct PID_Regulator_t *pid);
}PID_Regulator_t;
void PID_Reset(PID_Regulator_t *pid);
void PID_Calc(PID_Regulator_t *pid);
int16_t PID_PROCESS_Double(fw_PID_Regulator_t* pid_position,fw_PID_Regulator_t* pid_speed,
                            float target, float position_feedback, float velocity_feedback);
#endif

