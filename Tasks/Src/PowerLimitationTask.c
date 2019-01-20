/**
  ******************************************************************************
  * File Name      	: PowerLimitationTask.c
  * Description    	: ���̹��������㷨ʵ��
  * Author			: �ֱ��ԣ�������
  ******************************************************************************
  *
  * Copyright (c) 2019 Team JiaoLong-ShanghaiJiaoTong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#include "includes.h"
#include "math.h"

//���̹�������
#ifdef USE_POWER_LIMIT
#define POW_M  USE_POWER_LIMIT
float SpeedAttenuation = 1.0f;
float LimitFactor = 1.0f;
uint8_t flag = 1;
fw_PID_Regulator_t PowerLimitationPID = POWER_LIMITATION_PID_DEFAULT;
int16_t PowerBufferMax = 80;

float PowerLimitation()
{
	static float windows = 1.0;
	static float rate = 0.1;
	#ifdef GUARD
	PowerBufferMax = 40;//����
	#endif
	if (JUDGE_State == OFFLINE) return 1.0;
	else if(PowerHeatData.chassisPower < 5) return 1.0;
	else 
	{
		if(PowerHeatData.chassisPower > POW_M) 
		{
			if(PowerHeatData.chassisPowerBuffer > PowerBufferMax*0.6){
				if(windows==rate){
					windows/=2;
					rate = 0.1f*windows;
				}
				else{
					if(rate<windows){
						rate*=2;
						if(rate>windows) rate = windows;
					}
				}
			}
			else{
				rate = rate*0.9f>rate-0.05f?rate*0.9f:rate-0.05f;
				windows = rate;
			}
		}
		else if(PowerHeatData.chassisPower < POW_M * 0.8f)
		{
			if(rate<windows){
				rate*=2;
				if(rate>windows) rate = windows;
			}
			else{
				rate*=1.1f;
				//rate+=0.05f;
				windows=rate;
			}
		}
		#if USE_POWER_LIMIT > 50
		else if(PowerHeatData.chassisPower < POW_M * 0.9f)
		{
			if(rate<windows){
				rate*=2;
				if(rate>windows) rate = windows;
			}
			else{
				rate+=0.02f;
				windows=rate;
			}
		}
		#endif
	}
	return rate;
}
/*
#define POW_M  USE_POWER_LIMIT
void PowerLimitation(void)
{
	int16_t sum = 0;
	int16_t CM_current_max;
	#ifndef GUARD
	int16_t CMFLIntensity = CMFL.Intensity;
	int16_t CMFRIntensity = CMFR.Intensity;
	int16_t CMBLIntensity = CMBL.Intensity;
	int16_t CMBRIntensity = CMBR.Intensity;
	#else
	int16_t CMFLIntensity = CML.Intensity;
	int16_t CMFRIntensity = CMR.Intensity;
	int16_t CMBLIntensity = CML.Intensity;
	int16_t CMBRIntensity = CMR.Intensity;
	#endif
	//����ģʽ
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
	//�¹�����
	else if(PowerHeatData.chassisPowerBuffer-((PowerHeatData.chassisPower-POW_M)>0?(PowerHeatData.chassisPower-POW_M):0)*1.0f < 7.0f)
	{
		//CM_current_max = 2730;
		sum = __fabs(CMFLIntensity) + __fabs(CMFRIntensity) + __fabs(CMBLIntensity) + __fabs(CMBRIntensity);
		float realPowerBuffer = PowerHeatData.chassisPowerBuffer;
		float realPower = PowerHeatData.chassisPower;
		PowerLimitationPID.feedback = realPower;
		PowerLimitationPID.target = POW_M*0.93;
		PowerLimitationPID.Calc(&PowerLimitationPID);
		CM_current_max = PowerLimitationPID.output;
		//LimitFactor += PowerLimitationPID.output/sum;
		//if(CM_current_max > 0.5) CM_current_max = 0;
		//if(CM_current_max < -sum) CM_current_max = -sum;
		if(realPowerBuffer < 0) realPowerBuffer = 0;
		//LimitFactor = 50/(((realPower-80)>0?(realPower-80):0)+1) * pow((realPowerBuffer),2);
		LimitFactor = 2500 + 192*pow((realPowerBuffer),1);
		if(LimitFactor > sum) LimitFactor = sum;
		CMFLIntensity *= LimitFactor/sum;
		CMFRIntensity *= LimitFactor/sum;
		CMBLIntensity *= LimitFactor/sum;
		CMBRIntensity *= LimitFactor/sum;
	}
	#ifdef USE_SUPER_CAP
	else if(Control_SuperCap.release_power==0||PowerHeatData.chassisPower>30)
	#else
	else if (PowerHeatData.chassisPower>30)
	#endif
	{
		//PowerLimitationPID.Reset(&PowerLimitationPID);
		//LimitFactor = 1.0f;
		CM_current_max = 10000;
		sum = __fabs(CMFLIntensity) + __fabs(CMFRIntensity) + __fabs(CMBLIntensity) + __fabs(CMBRIntensity);
		if(sum > CM_current_max)
		{
			CMFLIntensity = (CMFLIntensity/(sum+0.0f))*CM_current_max;
			CMFRIntensity = (CMFRIntensity/(sum+0.0f))*CM_current_max;
			CMBLIntensity = (CMBLIntensity/(sum+0.0f))*CM_current_max;
			CMBRIntensity = (CMBRIntensity/(sum+0.0f))*CM_current_max;
		}
	}
	
	#ifndef GUARD
	CMFL.Intensity = CMFLIntensity;
	CMFR.Intensity = CMFRIntensity;
	CMBL.Intensity = CMBLIntensity;
	CMBR.Intensity = CMBRIntensity;
	#else
	CML.Intensity = CMFLIntensity*2;
	CMR.Intensity = CMFRIntensity*2;
	#endif
}
*/
#endif

