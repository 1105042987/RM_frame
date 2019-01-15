/**
  ******************************************************************************
  *FileName			: CapControlTask.c
  *Description		: �������ݿ��ؿ��Ƴ���
  *Author			: �ֱ��ԣ�������
  ******************************************************************************
  *
  * Copyright (c) 2019 Team JiaoLong-ShanghaiJiaoTong University
  * All rights reserved.
  *
  ******************************************************************************
*/
	
#include "includes.h"
#include "math.h"

uint8_t can_power_in=0;
CapControl_t Control_SuperCap={0,0};
uint8_t i=0;
float v_tem[5];

#ifdef USE_SUPER_CAP
void Cap_Control(void)
{
	for(i=0;i<5;i++)
	{
		
		HAL_ADC_Start(&hadc1);
		while(HAL_ADC_PollForConversion(&hadc1,100) != HAL_OK);
		v_tem[i]=HAL_ADC_GetValue(&hadc1)*29.19f/40.96f;
		HAL_ADC_Stop(&hadc1);
		
	}
	
	for(i=0;i<5;i++){
		Control_SuperCap.C_voltage += v_tem[i];
	}
	Control_SuperCap.C_voltage = Control_SuperCap.C_voltage/5;
	
	user_data.mask = 0xC0;
	uint16_t temcov = Control_SuperCap.C_voltage;
	for (int j = 1;temcov > 1100; temcov = temcov - 155 )
	{
		user_data.mask = user_data.mask + j;
		j = j * 2;
	}
	
	if(RobotState.remainHP < 1){
		FUNC__CAP__CLOSE();
	}
	else{
		if(PowerHeatData.chassisPower < 80 && PowerHeatData.chassisPowerBuffer == 60.0f){
			can_power_in = 1;
		}
		else{
			if(PowerHeatData.chassisPowerBuffer < 40.0f){
				can_power_in = 0;
			}
		}
		if(Control_SuperCap.stop_power){
			FUNC__CAP__CLOSE();
		}
		else
		{
			if(Control_SuperCap.release_power){
				FUNC__CAP__OUTPUT();
			}
			else
			{
				if(fabs(CMFL.offical_speedPID.fdb)<2000&&fabs(CMFR.offical_speedPID.fdb)<2000
					&&fabs(CMBL.offical_speedPID.fdb)<2000&&fabs(CMBR.offical_speedPID.fdb)<2000
					&&can_power_in==1)
				{		
					FUNC__CAP__RECHARGE();
				}
				else{
					FUNC__CAP__CLOSE();
				}
			}
		}
	}
}
void LED_Show_SuperCap_Voltage(uint8_t flag)
{
	if(flag==0)
	{
		HAL_GPIO_WritePin(GPIOG, 0x1fe, GPIO_PIN_SET);
		return;
	}
	if(Control_SuperCap.C_voltage<1100)
		HAL_GPIO_WritePin(GPIOG, 0x1fe, GPIO_PIN_SET);
	else{
		HAL_GPIO_WritePin(GPIOG, 0x1fe, GPIO_PIN_SET);
		int unlight = 7-(Control_SuperCap.C_voltage-1100)/143;
		if(unlight<0) unlight=0;
		HAL_GPIO_WritePin(GPIOG, 0x1fe>>unlight, GPIO_PIN_RESET);
	}
}
#endif 
/**/
