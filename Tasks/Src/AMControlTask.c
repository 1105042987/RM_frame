/**
  ******************************************************************************
  * File Name          : AMControlTask.c
  * Description        : 机械臂控制任务
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#include "includes.h"

uint16_t AMUD1Intensity=0;
uint16_t AMUD2Intensity=0;
extern uint16_t GMPITCHIntensity;

fw_PID_Regulator_t AMUDPositionPID = fw_PID_INIT(1200.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 10000.0);

fw_PID_Regulator_t AMUDSpeedPID = fw_PID_INIT(1, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 4000.0);



//机械臂电机实际物理角度值
double AMUD1RealAngle = 0.0;
double AMUD2RealAngle = 0.0;

//机械臂电机上次物理角度值
uint16_t AMUD1LastAngle = 0.0;
uint16_t AMUD2LastAngle = 0.0;

//是否初次进入
static uint8_t AMUD1FirstEnter = 1;
static uint8_t AMUD2FirstEnter = 1;

//用于减小系统开销
static uint8_t s_AMUD1Count = 0;
static uint8_t s_AMUD2Count = 0;

void setAGMMotor()
{
	CanTxMsgTypeDef pData;
	AGMOTOR_CAN.pTxMsg = &pData;
	
	AGMOTOR_CAN.pTxMsg->StdId = AGM_TXID;
	AGMOTOR_CAN.pTxMsg->ExtId = 0;
	AGMOTOR_CAN.pTxMsg->IDE = CAN_ID_STD;
	AGMOTOR_CAN.pTxMsg->RTR = CAN_RTR_DATA;
	AGMOTOR_CAN.pTxMsg->DLC = 0x08;
	
	AGMOTOR_CAN.pTxMsg->Data[0] = (uint8_t)(AMUD1Intensity >> 8);
	AGMOTOR_CAN.pTxMsg->Data[1] = (uint8_t)AMUD1Intensity;
	AGMOTOR_CAN.pTxMsg->Data[2] = (uint8_t)(GMPITCHIntensity >> 8);
	AGMOTOR_CAN.pTxMsg->Data[3] = (uint8_t)GMPITCHIntensity;
	AGMOTOR_CAN.pTxMsg->Data[4] = (uint8_t)(AMUD2Intensity >> 8);
	AGMOTOR_CAN.pTxMsg->Data[5] = (uint8_t)AMUD2Intensity;
	AGMOTOR_CAN.pTxMsg->Data[6] = 0;
	AGMOTOR_CAN.pTxMsg->Data[7] = 0;

	if(can2_update == 1)
	{
		HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_DisableIRQ(USART1_IRQn);
		HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_DisableIRQ(TIM7_IRQn);
		#ifdef DEBUG_MODE
			HAL_NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
		#endif
		if(HAL_CAN_Transmit_IT(&AGMOTOR_CAN) != HAL_OK)
		{
			Error_Handler();
		}
		can2_update = 0;
		HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_EnableIRQ(USART1_IRQn);
		HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_EnableIRQ(TIM7_IRQn);
		#ifdef DEBUG_MODE
			HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
		#endif
  }
}
//机械臂电机累计角度解算
void StandardlizeAMRealAngle(double* AMRealAngle,uint16_t AMThisAngle,uint16_t AMLastAngle)
{
	if(AMThisAngle<=AMLastAngle)
		{
			if((AMLastAngle-AMThisAngle)>3000)//编码器上溢
				*AMRealAngle = *AMRealAngle + (AMThisAngle+8192-AMLastAngle) * 360 / 8192.0 / AMReduction;
			else//反转
				*AMRealAngle = *AMRealAngle - (AMLastAngle - AMThisAngle) * 360 / 8192.0 / AMReduction;
		}
	else
		{
			if((AMThisAngle-AMLastAngle)>3000)//编码器下溢
				*AMRealAngle = *AMRealAngle - (AMLastAngle+8192-AMThisAngle) *360 / 8192.0 / AMReduction;
			else//正转
				*AMRealAngle = *AMRealAngle + (AMThisAngle - AMLastAngle) * 360 / 8192.0 / AMReduction;
		}
}


void ControlAMUD1()
{
		if(s_AMUD1Count == 1)
		{		
			uint16_t 	ThisAngle;	
			double 		ThisSpeed;	
			ThisAngle = AMUD1Rx.angle;							//未处理角度
			if(AMUD1FirstEnter==1) {AMUD1LastAngle = ThisAngle;AMUD1FirstEnter = 0;return;}
			StandardlizeAMRealAngle(&AMUD1RealAngle,ThisAngle,AMUD1LastAngle);//处理
			ThisSpeed = AMUD1Rx.RotateSpeed * 6;		//单位：度每秒
			
			AMUD1Intensity = PID_PROCESS_Double(AMUDPositionPID,AMUDSpeedPID,AMUDAngleTarget,AMUD1RealAngle,ThisSpeed);
			
			s_AMUD1Count = 0;
			AMUD1LastAngle = ThisAngle;
		}
		else
		{
			s_AMUD1Count++;
		}
}
void ControlAMUD2()
{
		if(s_AMUD2Count == 1)
		{		
			uint16_t 	ThisAngle;	
			double 		ThisSpeed;	
			ThisAngle = AMUD2Rx.angle;							//未处理角度
			if(AMUD2FirstEnter==1) {AMUD2LastAngle = ThisAngle;AMUD2FirstEnter = 0;return;}
			StandardlizeAMRealAngle(&AMUD2RealAngle,ThisAngle,AMUD2LastAngle);//处理
			ThisSpeed = AMUD2Rx.RotateSpeed * 6;		//单位：度每秒
			
			AMUD2Intensity = -PID_PROCESS_Double(AMUDPositionPID,AMUDSpeedPID,-AMUDAngleTarget,AMUD2RealAngle,ThisSpeed);
			
			s_AMUD2Count = 0;
			AMUD2LastAngle = ThisAngle;
		}
		else
		{
			s_AMUD2Count++;
		}
}



//副控制循环
void vice_controlLoop()
{

		ControlAMUD1();
		ControlAMUD2();
	
		//ControlGMYAW();
		ControlGSYAW();
		ControlGMPITCH();
		
		setAGMMotor();
}

//机械臂控制初始化
void AMControlInit()
{
	AMUD1FirstEnter = 1;
	AMUD1RealAngle = 0.0;
	AMUD1LastAngle = 0.0;
	AMUD2FirstEnter = 1;
	AMUD2RealAngle = 0.0;
	AMUD2LastAngle = 0.0;
	
	ControlAMUD1();
	ControlAMUD2();
	HAL_TIM_PWM_Start(&BYPASS_TIM, TIM_CHANNEL_1);
}
