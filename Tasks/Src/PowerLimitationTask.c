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

float SpeedAttenuation = 1.0f;
float LimitFactor = 1.0f;
uint8_t flag = 1;
fw_PID_Regulator_t PowerLimitationPID = POWER_LIMITATION_PID_DEFAULT;

//底盘功率限制
void PowerLimitation(void)
{
	uint16_t sum = 0;
	int16_t CM_current_max;
	int16_t CMFLIntensity = CMFL.Intensity;
	int16_t CMFRIntensity = CMFR.Intensity;
	int16_t CMBLIntensity = CMBL.Intensity;
	int16_t CMBRIntensity = CMBR.Intensity;
	
	sum = __fabs(CMFLIntensity) + __fabs(CMFRIntensity) + __fabs(CMBLIntensity) + __fabs(CMBRIntensity);
	static int16_t FLILast,FRILast,BLILast,BRILast;
	//离线模式
	if (JUDGE_State == OFFLINE)
	{
		CM_current_max = 4000;
		sum = __fabs(CMFLIntensity) + __fabs(CMFRIntensity) + __fabs(CMBLIntensity) + __fabs(CMBRIntensity);
		if(sum > CM_current_max)
		{
			CMFLIntensity = (CMFLIntensity/(sum+1.0f))*CM_current_max;
			CMFRIntensity = (CMFRIntensity/(sum+1.0f))*CM_current_max;
			CMBLIntensity = (CMBLIntensity/(sum+1.0f))*CM_current_max;
			CMBRIntensity = (CMBRIntensity/(sum+1.0f))*CM_current_max;
		}
		//CM_current_max = CM_current_MAX;
	}
	
	//仿桂电策略
	else if(PowerHeatData.chassisPowerBuffer-((PowerHeatData.chassisPower-80)>0?(PowerHeatData.chassisPower-80):0)*0.5f < 10.0f)
	{
		//CM_current_max = 2730;
		float realPowerBuffer = PowerHeatData.chassisPowerBuffer;
		/*float realPower = PowerHeatData.chassisPower;
		PowerLimitationPID.feedback = realPower;
		PowerLimitationPID.target = 70;
		PowerLimitationPID.Calc(&PowerLimitationPID);
		CM_current_max = PowerLimitationPID.output;*/
		
		//LimitFactor += PowerLimitationPID.output/sum;
		//if(CM_current_max > 0.5) CM_current_max = 0;
		//if(CM_current_max < -sum) CM_current_max = -sum;
		//if(realPowerBuffer < 0) realPowerBuffer = 0;
		//LimitFactor = 50/(((realPower-80)>0?(realPower-80):0)+1) * pow((realPowerBuffer),2);
		LimitFactor = 3200+320*realPowerBuffer;//2500 + 192*pow((realPowerBuffer),1);
		if(LimitFactor > sum) LimitFactor = sum;
		CMFLIntensity *= LimitFactor/sum;
		CMFRIntensity *= LimitFactor/sum;
		CMBLIntensity *= LimitFactor/sum;
		CMBRIntensity *= LimitFactor/sum;
	}
	//else if (Control_SuperCap.release_power==0 || PowerHeatData.chassisPower>30)
	else if(Cap_Get_Cap_State()!=CAP_STATE_RELEASE||Cap_Get_Cap_Voltage()<13)
	{
		//PowerLimitationPID.Reset(&PowerLimitationPID);
		//LimitFactor = 1.0f;
		CM_current_max = 12000;
		if(sum > CM_current_max){
			CMFLIntensity = (CMFLIntensity/(sum+0.0f))*CM_current_max;
			CMFRIntensity = (CMFRIntensity/(sum+0.0f))*CM_current_max;
			CMBLIntensity = (CMBLIntensity/(sum+0.0f))*CM_current_max;
			CMBRIntensity = (CMBRIntensity/(sum+0.0f))*CM_current_max;
		}
	}
	else if(sum>12000&&abs(CMFLIntensity-FLILast)>1000){
		
	  FLILast=(CMFLIntensity>0?1:-1)*abs(FLILast);
	  FRILast=(CMFRIntensity>0?1:-1)*abs(FRILast);
	  BLILast=(CMBLIntensity>0?1:-1)*abs(BLILast);
	  BRILast=(CMBRIntensity>0?1:-1)*abs(BRILast);
		
		CMFLIntensity=FLILast+(CMFLIntensity-FLILast)*0.1;
		CMFRIntensity=FRILast+(CMFRIntensity-FRILast)*0.1;
		CMBLIntensity=BLILast+(CMBLIntensity-BLILast)*0.1;
		CMBRIntensity=BRILast+(CMBRIntensity-BRILast)*0.1;
		/*
		CMFLIntensity=FLILast+(CMFLIntensity-FLILast>0?1:-1)*100;
    CMFRIntensity=FRILast+(CMFRIntensity-FRILast>0?1:-1)*100;
		CMBLIntensity=BLILast+(CMBLIntensity-BLILast>0?1:-1)*100;
		CMBRIntensity=BRILast+(CMBRIntensity-BRILast>0?1:-1)*100;*/
		/*
		if(abs(CMFLIntensity-FLILast)>1000){CMFLIntensity=FLILast+(CMFLIntensity-FLILast>0?1:-1)*100;}
		if(abs(CMFRIntensity-FRILast)>1000){CMFRIntensity=FRILast+(CMFRIntensity-FRILast>0?1:-1)*100;}
		if(abs(CMBLIntensity-BLILast)>1000){CMBLIntensity=BLILast+(CMBLIntensity-BLILast>0?1:-1)*100;}
		if(abs(CMBRIntensity-BRILast)>1000){CMBRIntensity=BRILast+(CMBRIntensity-BRILast>0?1:-1)*100;}*/
	}
	FLILast=CMFLIntensity;
	FRILast=CMFRIntensity;
	BLILast=CMBLIntensity;
	BRILast=CMBRIntensity;
	
	CMFL.Intensity = CMFLIntensity;
	CMFR.Intensity = CMFRIntensity;
	CMBL.Intensity = CMBLIntensity;
	CMBR.Intensity = CMBRIntensity;
}


