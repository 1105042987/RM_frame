/**
  ******************************************************************************
  * File Name      	: PowerLimitationTask.c
  * Description    	: 底盘功率限制算法实现
  * Author			: 林炳辉，赵振宇
  ******************************************************************************
  *
  * Copyright (c) 2019 Team JiaoLong-ShanghaiJiaoTong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#include "includes.h"
#include "math.h"

//底盘功率限制
int LimitCnt=500;
int8_t LimitRate=1;
#ifdef USE_POWER_LIMIT
void PowerLimitation(){
	static float limitTgt=90;
	float rate;
	if(LimitCnt){
		LimitCnt--;
		limitTgt=40;
	}
	else if(limitTgt<90){
		limitTgt+=0.05;
	}
	float tmp=(float)PowerHeat.chassis_power_buffer-limitTgt;
	if(tmp<150){rate=tmp/(150-limitTgt);}
	else{rate=1;}
	if(rate<0.1){rate=0.1;}
//	ChassisSpeed*=rate;
	if(LimitRate==0){
		CML.offical_speedPID.outputMax=0;
		CMR.offical_speedPID.outputMax=0;
	}
	else{
		CML.offical_speedPID.outputMax=4000*rate;
		CMR.offical_speedPID.outputMax=4000*rate;
	}
}

#endif

