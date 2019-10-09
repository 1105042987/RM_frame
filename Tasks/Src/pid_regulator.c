/**
  ******************************************************************************
  * File Name       : pid_regulator.c
  * Description     : PID函数
  * Author			：林炳辉
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * C语言PID函数实现
  ******************************************************************************
  */

#include "includes.h"

void fw_PID_Reset(fw_PID_Regulator_t *pid){
	pid->errorCurr = 0;
	pid->componentKd = 0;
	pid->componentKi = 0;
	pid->componentKp = 0;
	pid->errorLast = 0;
	pid->errorSum = 0;
	pid->feedback = 0;
	pid->output = 0;
	pid->SumCount = 0;
	pid->target = 0;
	for(int i=0;i<PID_I_CNT;i++) pid->err[i] = 0;
}

void fw_PID_Calc(fw_PID_Regulator_t *pid){
	pid->errorCurr = pid->target - pid->feedback;
	pid->errorSum += pid->errorCurr - pid->err[pid->SumCount];
	pid->err[pid->SumCount] = pid->errorCurr;
	pid->SumCount = (pid->SumCount + 1) % PID_I_CNT;
	
	pid->componentKp = pid->kp * pid->errorCurr;
	MINMAX(pid->componentKp, -pid->componentKpMax, pid->componentKpMax);
	pid->componentKi = pid->ki * pid->errorSum;
	MINMAX(pid->componentKi, -pid->componentKiMax, pid->componentKiMax);
	pid->componentKd = pid->kd * (pid->errorCurr - pid->errorLast);
	MINMAX(pid->componentKd, -pid->componentKdMax, pid->componentKdMax);
	
	pid->errorLast = pid->errorCurr;
	
	pid->output = pid->componentKp + pid->componentKi + pid->componentKd;
	MINMAX(pid->output, -pid->outputMax, pid->outputMax);
}


int16_t PID_PROCESS_Double(fw_PID_Regulator_t* pid_position,fw_PID_Regulator_t* pid_speed,float target, float position_feedback, float velocity_feedback)
{
	//position		
	pid_position->target = target;
	pid_position->feedback = position_feedback;
	pid_position->Calc(pid_position);
	//speed
	pid_speed->target = pid_position->output;
	pid_speed->feedback = velocity_feedback;
	pid_speed->Calc(pid_speed);
	return pid_speed->output;
}

void yyp_PID_Calc(fw_PID_Regulator_t *pid){
	pid->errorCurr = pid->target - pid->feedback;
	pid->errorSum += pid->errorCurr;
	pid->errorSum*=0.9;
	
	pid->componentKp = pid->kp * pid->errorCurr;
	MINMAX(pid->componentKp, -pid->componentKpMax, pid->componentKpMax);
	pid->componentKi = pid->ki * pid->errorSum;
	MINMAX(pid->componentKi, -pid->componentKiMax, pid->componentKiMax);
	pid->componentKd = pid->kd * (pid->errorCurr - pid->errorLast);
	MINMAX(pid->componentKd, -pid->componentKdMax, pid->componentKdMax);
	
	pid->errorLast = pid->errorCurr;
	
	pid->output = pid->componentKp + pid->componentKi + pid->componentKd;
	MINMAX(pid->output, -pid->outputMax, pid->outputMax);
}
