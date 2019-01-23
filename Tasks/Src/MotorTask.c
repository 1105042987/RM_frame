/**
  ******************************************************************************
  * File Name          : CANMotot.c
  * Description        : CAN电机统一驱动任务
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#include "includes.h"

#ifdef CONFIGURATION
void ControlNM(MotorINFO *id);
void ControlCM(MotorINFO *id);
void ControlANTI_CM(MotorINFO *id);
void ControlGMY(MotorINFO *id);
void ControlGMP(MotorINFO *id);

//**********************************************************************
//					pid(kp,ki,kd,kprM,kirM,kdrM,rM)
//						kprM:kp result Max
//**********************************************************************

//**********************************************************************
//				SpeedBased_MOTORINFO_Init(func,spid)
//**********************************************************************
MotorINFO CMFL = SpeedBased_MOTORINFO_Init(&ControlCM,CHASSIS_MOTOR_SPEED_PID_DEFAULT);
MotorINFO CMFR = SpeedBased_MOTORINFO_Init(&ControlCM,CHASSIS_MOTOR_SPEED_PID_DEFAULT);
MotorINFO CMBL = SpeedBased_MOTORINFO_Init(&ControlCM,CHASSIS_MOTOR_SPEED_PID_DEFAULT);
MotorINFO CMBR = SpeedBased_MOTORINFO_Init(&ControlCM,CHASSIS_MOTOR_SPEED_PID_DEFAULT);
MotorINFO FRICL = SpeedBased_MOTORINFO_Init(&ControlCM,CHASSIS_MOTOR_SPEED_PID_DEFAULT);
MotorINFO FRICR = SpeedBased_MOTORINFO_Init(&ControlCM,CHASSIS_MOTOR_SPEED_PID_DEFAULT);
//***********************************************************************************
//		     Gimbal_MOTORINFO_Init(rdc,func,zero,compensation,maxrange,ppid,spid)
//***********************************************************************************
//使用云台电机时，请务必确定校准过零点
MotorINFO GMP  = Gimbal_MOTORINFO_Init(2.0,&ControlGMP, 3740 , 0 , 20 ,
									   fw_PID_INIT_EASY(0.5,0,0.9, 	100.0),
									   fw_PID_INIT_EASY(920,30,0, 	5000.0));
									   
MotorINFO GMY  = Gimbal_MOTORINFO_Init(1.0,&ControlGMY, 1310 , 0 , 40 ,
									   fw_PID_INIT_EASY(0.6,0,0.5, 	100.0),
									   fw_PID_INIT_EASY(2500,100,0, 5000.0));

//*************************************************************************
//			AngleBased_MOTORINFO_Init(rdc,func,ppid,spid)
//*************************************************************************
MotorINFO STIR = AngleBased_MOTORINFO_Init(36.0,&ControlNM,
								fw_PID_INIT_EASY(10.0, 0.0, 0.0, 1200.0),
								fw_PID_INIT_EASY(30, 0.0, 0.0,	 15000.0));
MotorINFO CML = AngleBased_MOTORINFO_Init(19.0,&ControlNM,
								fw_PID_INIT_EASY(10.0, 0.0, 0.0, 1200.0),
								fw_PID_INIT_EASY(30, 0.0, 0.0,	 15000.0));
MotorINFO CMR = AngleBased_MOTORINFO_Init(19.0,&ControlNM,
								fw_PID_INIT_EASY(10.0, 0.0, 0.0, 1200.0),
								fw_PID_INIT_EASY(30, 0.0, 0.0,	 15000.0));

MotorINFO* can1[8]={&FRICL,&FRICR,0,0,&GMY,&GMP,&STIR,0};
MotorINFO* can2[8]={&CML,&CMR,&CMBL,&CMBR,0,0,0,0};



MotorINFO* ChassisMotorGroup[4]={&CML,&CMR,0,0};			// 非四电机例子

//MotorINFO* ChassisMotorGroup[4]={&CMFL,&CMFR,&CMBL,&CMBR};


#ifdef USE_CHASSIS_FOLLOW
MotorINFO* GimbalMotorGroup[2]={&GMP,&GMY};
#else
#ifdef USE_AUTOAIM
MotorINFO* GimbalMotorGroup[2]={&GMP,&GMY};
#endif
#endif

#endif


void ControlNM(MotorINFO* id)
{
	if(id==0) return;
	if(id->s_count == 1)
	{		
		uint16_t 	ThisAngle;	
		double 		ThisSpeed;	
		ThisAngle = id->RxMsgC6x0.angle;				//未处理角度
		if(id->FirstEnter==1) {id->lastRead = ThisAngle;id->FirstEnter = 0;return;}
		if(ThisAngle<=id->lastRead)
		{
			if((id->lastRead-ThisAngle)>4000)//编码器上溢
				id->Real = id->Real + (ThisAngle+8192-id->lastRead) * 360 / 8192.0 / id->ReductionRate;
			else//正常
				id->Real = id->Real - (id->lastRead - ThisAngle) * 360 / 8192.0 / id->ReductionRate;
		}
		else
		{
			if((ThisAngle-id->lastRead)>4000)//编码器下溢
				id->Real = id->Real - (id->lastRead+8192-ThisAngle) *360 / 8192.0 / id->ReductionRate;
			else//正常
				id->Real = id->Real + (ThisAngle - id->lastRead) * 360 / 8192.0 / id->ReductionRate;
		}
		ThisSpeed = id->RxMsgC6x0.rotateSpeed * 6 / id->ReductionRate;		//单位：度每秒
		
		id->Intensity = PID_PROCESS_Double(&(id->positionPID),&(id->speedPID),id->Target,id->Real,ThisSpeed);
		
		id->s_count = 0;
		id->lastRead = ThisAngle;
	}
	else
	{
		id->s_count++;
	}		
}
void ControlCM(MotorINFO* id)
{
	//Target 代作为目标速度
	if(id==0) return;
	id->offical_speedPID.ref = (float)(id->Target);
	id->offical_speedPID.fdb = id->RxMsgC6x0.rotateSpeed;
	id->offical_speedPID.Calc(&(id->offical_speedPID));
	id->Intensity=(1.30f)*id->offical_speedPID.output;
}
void ControlANTI_CM(MotorINFO* id)
{
	//Target 代作为目标速度
	if(id==0) return;
	id->offical_speedPID.ref = (float)(id->Target);
	id->offical_speedPID.fdb = id->RxMsgC6x0.rotateSpeed;
	id->offical_speedPID.Calc(&(id->offical_speedPID));
	id->Intensity=(-1.30f)*id->offical_speedPID.output;
}

void ControlGM(MotorINFO* id,float ThisAngle,float ThisSpeed)
{
	if(id==0) return;
	if(id->s_count == 0)
	{
		static uint8_t Reseted = 0;
		uint16_t GM_ZERO = id->RxMsgC6x0.angle;
		uint16_t GM_COMPENSATION = id->RxMsgC6x0.rotateSpeed;
		uint16_t GM_MAX_RANGE = id->RxMsgC6x0.moment;
		double 	encoder_real = (GM_ZERO - id->RxMsg6623.angle) * 360.0 / 8192.0 / id->ReductionRate;
		//NORMALIZE_ANGLE180(encoder_real);
		int8_t 	dir;
		if(id->ReductionRate>=0) dir=1;
		else dir=-1;
		
		if(id->FirstEnter==1) {
			id->lastRead = ThisAngle;
			id->Real = encoder_real;
			NORMALIZE_ANGLE180(id->Real);
			id->FirstEnter = 0;
			return;
		}
		
		if(ThisAngle <= id->lastRead)
		{
			if((id->lastRead-ThisAngle) > 180)
				 id->Real += (ThisAngle + 360 - id->lastRead)*dir;
			else
				 id->Real -= (id->lastRead - ThisAngle)*dir;
		}
		else
		{
			if((ThisAngle-id->lastRead) > 180)
				 id->Real -= (id->lastRead + 360 - ThisAngle)*dir;
			else
				 id->Real += (ThisAngle - id->lastRead)*dir;
		}
		if(abs(id->Real-id->Target)<5) Reseted = 1;
		id->lastRead = ThisAngle ;
		
		MINMAX(id->Target, id->Real - encoder_real - GM_MAX_RANGE, id->Real - encoder_real + GM_MAX_RANGE);
		if(Reseted==0) id->positionPID.outputMax = 1.0;
		else id->positionPID.outputMax = 10.0;
		
		id->Intensity = GM_COMPENSATION - PID_PROCESS_Double(&(id->positionPID),&(id->speedPID),id->Target,id->Real,-ThisSpeed);
		
		MINMAX(id->Intensity,-id->speedPID.outputMax,id->speedPID.outputMax);
		
		id->s_count = 0;
	}
	else
	{
		id->s_count++;
	}		
}

#ifdef OLD_INFANTRY
void ControlGMP(MotorINFO* id)
{
	ControlGM(id,gyro_data.pit+180,gyro_data.wy);
}
void ControlGMY(MotorINFO* id)
{
	ControlGM(id,gyro_data.yaw+180,gyro_data.wz);
}
#endif

#ifdef NEW_INFANTRY
double pitchAngle;
void ControlGMP(MotorINFO* id)
{
	pitchAngle = gyro_data.rol+180;
	NORMALIZE_ANGLE180(pitchAngle);
	ControlGM(id,-gyro_data.rol,gyro_data.wx);
}
void ControlGMY(MotorINFO* id)
{
	ControlGM(id,gyro_data.yaw,-gyro_data.wz);
}
#endif

//CAN
void setCAN11()
{
	CanTxMsgTypeDef pData;
	hcan1.pTxMsg = &pData;
	
	hcan1.pTxMsg->StdId = 0x200;
	hcan1.pTxMsg->ExtId = 0;
	hcan1.pTxMsg->IDE = CAN_ID_STD;
	hcan1.pTxMsg->RTR = CAN_RTR_DATA;
	hcan1.pTxMsg->DLC = 0x08;
	
	for(int i=0;i<4;i++){
		if(can1[i]==0) {
			hcan1.pTxMsg->Data[i*2]   = 0;
			hcan1.pTxMsg->Data[i*2+1] = 0;
		}
		else {
			hcan1.pTxMsg->Data[i*2]   = (uint8_t)(can1[i]->Intensity >> 8);
			hcan1.pTxMsg->Data[i*2+1] = (uint8_t)can1[i]->Intensity;
		}
	}

	if(can1_update == 1 && can1_type == 1)
	{
		HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_DisableIRQ(USART1_IRQn);
		HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
		HAL_NVIC_DisableIRQ(TIM7_IRQn);
		#ifdef DEBUG_MODE
			HAL_NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
		#endif
		if(HAL_CAN_Transmit_IT(&hcan1) != HAL_OK)
		{
			Error_Handler();
		}
		can1_update = 0;
		#ifdef CAN12
			can1_type = 2;
		#else
		#ifdef CAN13
			can1_type = 3;
		#endif
		#endif
		HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_EnableIRQ(USART1_IRQn);
		HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_EnableIRQ(TIM7_IRQn);
		HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
		#ifdef DEBUG_MODE
			HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
		#endif
	}
}
void setCAN12()
{
	CanTxMsgTypeDef pData;
	hcan1.pTxMsg = &pData;
	
	hcan1.pTxMsg->StdId = 0x1ff;
	hcan1.pTxMsg->ExtId = 0;
	hcan1.pTxMsg->IDE = CAN_ID_STD;
	hcan1.pTxMsg->RTR = CAN_RTR_DATA;
	hcan1.pTxMsg->DLC = 0x08;
	
	for(int i=0;i<4;i++){
		if(can1[i+4]==0) {
			hcan1.pTxMsg->Data[i*2]   = 0;
			hcan1.pTxMsg->Data[i*2+1] = 0;
		}
		else {
			hcan1.pTxMsg->Data[i*2]   = (uint8_t)(can1[i+4]->Intensity >> 8);
			hcan1.pTxMsg->Data[i*2+1] = (uint8_t)can1[i+4]->Intensity;
		}
	}

	if(can1_update == 1 && can1_type == 2)
	{
		HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_DisableIRQ(USART1_IRQn);
		HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
		HAL_NVIC_DisableIRQ(TIM7_IRQn);
		#ifdef DEBUG_MODE
			HAL_NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
		#endif
		if(HAL_CAN_Transmit_IT(&hcan1) != HAL_OK)
		{
			Error_Handler();
		}
		can1_update = 0;
		
		#ifdef CAN13
			can1_type = 3;
		#else
		#ifdef CAN11
			can1_type = 1;
		#endif
		#endif
		HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_EnableIRQ(USART1_IRQn);
		HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
		HAL_NVIC_EnableIRQ(TIM7_IRQn);
		#ifdef DEBUG_MODE
			HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
		#endif
  }
}
void setCAN21()
{
	CanTxMsgTypeDef pData;
	hcan2.pTxMsg = &pData;
	
	hcan2.pTxMsg->StdId = 0x200;
	hcan2.pTxMsg->ExtId = 0;
	hcan2.pTxMsg->IDE = CAN_ID_STD;
	hcan2.pTxMsg->RTR = CAN_RTR_DATA;
	hcan2.pTxMsg->DLC = 0x08;
	
	for(int i=0;i<4;i++){
		if(can2[i]==0) {
			hcan2.pTxMsg->Data[i*2]   = 0;
			hcan2.pTxMsg->Data[i*2+1] = 0;
		}
		else {
			hcan2.pTxMsg->Data[i*2]   = (uint8_t)(can2[i]->Intensity >> 8);
			hcan2.pTxMsg->Data[i*2+1] = (uint8_t)can2[i]->Intensity;
		}
	}

	if(can2_update == 1 && can2_type == 1)
	{
		HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_DisableIRQ(USART1_IRQn);
		HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
		HAL_NVIC_DisableIRQ(TIM7_IRQn);
		#ifdef DEBUG_MODE
			HAL_NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
		#endif
		if(HAL_CAN_Transmit_IT(&hcan2) != HAL_OK)
		{
			Error_Handler();
		}
		can2_update = 0;
		#ifdef CAN22
			can2_type = 2;
		#else
		#ifdef CAN23
			can2_type = 3;
		#endif
		#endif
		HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_EnableIRQ(USART1_IRQn);
		HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
		HAL_NVIC_EnableIRQ(TIM7_IRQn);
		#ifdef DEBUG_MODE
			HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
		#endif
  }
}
void setCAN22()
{
	CanTxMsgTypeDef pData;
	hcan2.pTxMsg = &pData;
	
	hcan2.pTxMsg->StdId = 0x1ff;
	hcan2.pTxMsg->ExtId = 0;
	hcan2.pTxMsg->IDE = CAN_ID_STD;
	hcan2.pTxMsg->RTR = CAN_RTR_DATA;
	hcan2.pTxMsg->DLC = 0x08;
	
	for(int i=0;i<4;i++){
		if(can2[i+4]==0) {
			hcan2.pTxMsg->Data[i*2]   = 0;
			hcan2.pTxMsg->Data[i*2+1] = 0;
		}
		else {
			hcan2.pTxMsg->Data[i*2]   = (uint8_t)(can2[i+4]->Intensity >> 8);
			hcan2.pTxMsg->Data[i*2+1] = (uint8_t)can2[i+4]->Intensity;
		}
	}

	if(can2_update == 1 && can2_type == 2)
	{
		HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_DisableIRQ(USART1_IRQn);
		HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
		HAL_NVIC_DisableIRQ(TIM7_IRQn);
		#ifdef DEBUG_MODE
			HAL_NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
		#endif
		if(HAL_CAN_Transmit_IT(&hcan2) != HAL_OK)
		{
			Error_Handler();
		}
		can2_update = 0;
		#ifdef CAN23
			can2_type = 3;
		#else
		#ifdef CAN21
			can2_type = 1;
		#endif
		#endif
		HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_EnableIRQ(USART1_IRQn);
		HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
		HAL_NVIC_EnableIRQ(TIM7_IRQn);
		#ifdef DEBUG_MODE
			HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
		#endif
  }
}

void InitMotor(MotorINFO *id)
{
	if(id==0) return;
	id->FirstEnter=1;
	id->lastRead=0;
	id->Real=0;
	id->Target=0;
	id->offical_speedPID.Reset(&(id->offical_speedPID));
	(id->Handle)(id);
	id->Intensity=0;
}

void Motor_ID_Setting()
{
	for(int i=0;i<4;i++)
	{
		if(can1[i]!=0) 
		{
			can1[i]->CAN_TYPE=&hcan1;
			can1[i]->RXID = 0x201+i;
			can1[i]->TXID = 0x200;
		}
		if(can2[i]!=0) 
		{
			can2[i]->CAN_TYPE=&hcan2;
			can2[i]->RXID = 0x201+i;
			can2[i]->TXID = 0x200;
		}
	}
	for(int i=4;i<8;i++)
	{
		if(can1[i]!=0) 
		{
			can1[i]->CAN_TYPE=&hcan1;
			can1[i]->RXID = 0x201+i;
			can1[i]->TXID = 0x1ff;
		}
		if(can2[i]!=0) 
		{
			can2[i]->CAN_TYPE=&hcan2;
			can2[i]->RXID = 0x201+i;
			can2[i]->TXID = 0x1ff;
		}
	}
}
