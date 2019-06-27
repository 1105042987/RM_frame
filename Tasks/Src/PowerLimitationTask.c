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
#ifdef USE_POWER_LIMIT
int LimitCnt=500;
void PowerLimitation(){
//	if(JUDGE_State == OFFLINE || PowerHeat.chassis_power_buffer>100){
//		CML.positionPID.outputMax=3500;
//		CMR.positionPID.outputMax=3500;
//		return;
//	}
	static float limitTgt=100;
	if(LimitCnt){
		LimitCnt--;
		limitTgt=50;
	}
	else if(limitTgt<100){
		limitTgt+=0.05;
	}
	float tmp=(float)PowerHeat.chassis_power_buffer-limitTgt;
	float rate=tmp/100;
	if(rate<0.1){rate=0.1;}
	CML.offical_speedPID.outputMax=3000*rate;
	CMR.offical_speedPID.outputMax=3000*rate;
	
//	CML.Intensity*=rate;
//	CMR.Intensity*=rate;
//	if(PowerHeat.chassis_power_buffer>90){
//		MINMAX(CML.Intensity,-2500,2500);
//		MINMAX(CMR.Intensity,-2500,2500);
//	}
//	else if(PowerHeat.chassis_power_buffer>50){
//		MINMAX(CML.Intensity,-2000,2000);
//		MINMAX(CMR.Intensity,-2000,2000);
//	}
//	else{
//		MINMAX(CML.Intensity,-1500,1500);
//		MINMAX(CMR.Intensity,-1500,1500);
//	}
}

#endif

