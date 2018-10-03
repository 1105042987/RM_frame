/**
  ******************************************************************************
  * File Name          : AMControlTask.c
  * Description        : 取弹送弹机械臂控制任务
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#include "includes.h"
uint16_t GMYAWIntensity=0,GMPITCHIntensity=0;

fw_PID_Regulator_t GMYAWPositionPID = fw_PID_INIT(1.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 10000.0);	//闲置
fw_PID_Regulator_t GMPITCHPositionPID = fw_PID_INIT(200.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 10000.0);

fw_PID_Regulator_t GMYAWSpeedPID = fw_PID_INIT(1, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 4000.0);				//闲置
fw_PID_Regulator_t GMPITCHSpeedPID = fw_PID_INIT(0.7, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 4000.0);

float GSYAW_ZERO = 4.0;

//云台电机实际物理角度值
double GMYAWRealAngle = 0.0;
double GMPITCHRealAngle = 0.0;
	
//云台电机上次物理角度值
uint16_t GMYAWLastAngle = 0.0;
uint16_t GMPITCHLastAngle = 0.0;

//是否初次进入
static uint8_t GMYAWFirstEnter = 1;
static uint8_t GMPITCHFirstEnter = 1;

//云台电机CAN信号控制
/*void setGMMotor()
{
	CanTxMsgTypeDef pData;
	GMMOTOR_CAN.pTxMsg = &pData;
	
	GMMOTOR_CAN.pTxMsg->StdId = GM_TXID;
	GMMOTOR_CAN.pTxMsg->ExtId = 0;
	GMMOTOR_CAN.pTxMsg->IDE = CAN_ID_STD;
	GMMOTOR_CAN.pTxMsg->RTR = CAN_RTR_DATA;
	GMMOTOR_CAN.pTxMsg->DLC = 0x08;
	
	GMMOTOR_CAN.pTxMsg->Data[0] = (uint8_t)(GMYAWIntensity >> 8);
	GMMOTOR_CAN.pTxMsg->Data[1] = (uint8_t)GMYAWIntensity;
	GMMOTOR_CAN.pTxMsg->Data[2] = (uint8_t)(GMPITCHIntensity >> 8);
	GMMOTOR_CAN.pTxMsg->Data[3] = (uint8_t)GMPITCHIntensity;
	GMMOTOR_CAN.pTxMsg->Data[4] = 0;
	GMMOTOR_CAN.pTxMsg->Data[5] = 0;
	GMMOTOR_CAN.pTxMsg->Data[6] = 0;
	GMMOTOR_CAN.pTxMsg->Data[7] = 0;

	if(can2_update == 1 && can_type == 0)
	{
		HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_DisableIRQ(USART1_IRQn);
		HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_DisableIRQ(TIM7_IRQn);
		#ifdef DEBUG_MODE
			HAL_NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
		#endif
		if(HAL_CAN_Transmit_IT(&GMMOTOR_CAN) != HAL_OK)
		{
			Error_Handler();
		}
		can2_update = 0;
		can_type = 1;
		HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_EnableIRQ(USART1_IRQn);
		HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_EnableIRQ(TIM7_IRQn);
		#ifdef DEBUG_MODE
			HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
		#endif
  }
}*/

void ControlGMYAW()
{
	uint16_t 	ThisAngle;	//当前电机角度
	double 		ThisSpeed;	//当前电机转速
	ThisAngle = GMYAWRx.angle;//未处理角度
	if(GMYAWFirstEnter==1) 
	{//初始化时，记录下当前编码器的值
		GMYAWLastAngle = ThisAngle;
		GMYAWFirstEnter = 0;
		return;
	}
	
	if(ThisAngle<=GMYAWLastAngle)
	{
		if((GMYAWLastAngle-ThisAngle)>3000)//编码器上溢
			GMYAWRealAngle = GMYAWRealAngle + (ThisAngle+8192-GMYAWLastAngle) * 360 / 8192.0 / GMYAWReduction;
		else//反转
			GMYAWRealAngle = GMYAWRealAngle - (GMYAWLastAngle - ThisAngle) * 360 / 8192.0 / GMYAWReduction;
	}
	else
	{
		if((ThisAngle-GMYAWLastAngle)>3000)//编码器下溢
			GMYAWRealAngle = GMYAWRealAngle - (GMYAWLastAngle+8192-ThisAngle) *360 / 8192.0 / GMYAWReduction;
		else//正转
			GMYAWRealAngle = GMYAWRealAngle + (ThisAngle - GMYAWLastAngle) * 360 / 8192.0 / GMYAWReduction;
	}
	
	ThisSpeed = GMYAWRx.RotateSpeed * 6;		//单位：度每秒
	
	GMYAWIntensity = PID_PROCESS_Double(GMYAWPositionPID,GMYAWSpeedPID,GMYAWAngleTarget,GMYAWRealAngle,ThisSpeed);
	
	GMYAWLastAngle = ThisAngle;
}

void ControlGMPITCH()
{
	uint16_t 	ThisAngle;	//当前电机角度
	double 		ThisSpeed;	//当前电机转速
	ThisAngle = GMPITCHRx.angle;//未处理角度
	if(GMPITCHFirstEnter==1) 
	{//初始化时，记录下当前编码器的值
		GMPITCHLastAngle = ThisAngle;
		GMPITCHFirstEnter = 0;
		return;
	}
	
	if(ThisAngle<=GMPITCHLastAngle)
	{
		if((GMPITCHLastAngle-ThisAngle)>3000)//编码器上溢
			GMPITCHRealAngle = GMPITCHRealAngle + (ThisAngle+8192-GMPITCHLastAngle) * 360 / 8192.0 / GMPITCHReduction;
		else//反转
			GMPITCHRealAngle = GMPITCHRealAngle - (GMPITCHLastAngle - ThisAngle) * 360 / 8192.0 / GMPITCHReduction;
	}
	else
	{
		if((ThisAngle-GMPITCHLastAngle)>3000)//编码器下溢
			GMPITCHRealAngle = GMPITCHRealAngle - (GMPITCHLastAngle+8192-ThisAngle) *360 / 8192.0 / GMPITCHReduction;
		else//正转
			GMPITCHRealAngle = GMPITCHRealAngle + (ThisAngle - GMPITCHLastAngle) * 360 / 8192.0 / GMPITCHReduction;
	}
	
	ThisSpeed = GMPITCHRx.RotateSpeed * 6;		//单位：度每秒
	
	GMPITCHIntensity = PID_PROCESS_Double(GMPITCHPositionPID,GMPITCHSpeedPID,GMPITCHAngleTarget,GMPITCHRealAngle,ThisSpeed);
	
	GMPITCHLastAngle = ThisAngle;
}

void Steering_Motor_Set_Angle(float angle)
{
	angle += GSYAW_ZERO;
	if(angle<0)
	{
		angle=0;
	}
	if(angle>220)
	{
		angle=220;
	}
	
	uint16_t x = angle / 180 * 1800 + 600;
	
	__HAL_TIM_SET_COMPARE(STEER_TIM,YAW_CHANNEL,x);
}


void ControlGSYAW()
{
	Steering_Motor_Set_Angle(GMYAWAngleTarget);
}


void GMControlInit()
{
	GMYAWFirstEnter = 1;
	GMPITCHFirstEnter = 1;
	GMYAWRealAngle = 0.0;
	GMPITCHRealAngle = 0.0;
	GMYAWLastAngle = 0.0;
	GMPITCHLastAngle = 0.0;
	
	ControlGMYAW();
	ControlGMPITCH();
	Steering_Motor_Set_Angle(0);
	HAL_TIM_PWM_Start(STEER_TIM, YAW_CHANNEL);
	__HAL_TIM_SET_COMPARE(STEER_TIM, YAW_CHANNEL,0);
	HAL_TIM_PWM_Start(STEER_TIM, GIVESML_CHANNEL);
	__HAL_TIM_SET_COMPARE(STEER_TIM, GIVESML_CHANNEL, SDOOR_CLOSE);
	HAL_TIM_PWM_Start(STEER_TIM, GIVEBIG_CHANNEL);
	__HAL_TIM_SET_COMPARE(STEER_TIM, GIVEBIG_CHANNEL, BDOOR_CLOSE);
}


