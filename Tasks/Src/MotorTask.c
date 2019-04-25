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

void ControlNM(MotorINFO *id);
void ControlUM(MotorINFO *id);
void ControlUFM(MotorINFO *id);
void ControlCM(MotorINFO *id);
#ifdef USE_GYRO
void ControlGMY(MotorINFO *id);
void ControlGMP(MotorINFO *id);
#endif
extern int16_t testIntensity;

//**********************************************************************
//					pid(kp,ki,kd,kprM,kirM,kdrM,rM)
//						kprM:kp result Max
//**********************************************************************

//**********************************************************************
//				Chassis_MOTORINFO_Init(func,spid)
//**********************************************************************
MotorINFO CMFL = Chassis_MOTORINFO_Init(&ControlCM,CHASSIS_MOTOR_SPEED_PID_DEFAULT);
MotorINFO CMFR = Chassis_MOTORINFO_Init(&ControlCM,CHASSIS_MOTOR_SPEED_PID_DEFAULT);
MotorINFO CMBL = Chassis_MOTORINFO_Init(&ControlCM,CHASSIS_MOTOR_SPEED_PID_DEFAULT);
MotorINFO CMBR = Chassis_MOTORINFO_Init(&ControlCM,CHASSIS_MOTOR_SPEED_PID_DEFAULT);

MotorINFO FRICL = Chassis_MOTORINFO_Init(&ControlCM,CHASSIS_MOTOR_SPEED_PID_DEFAULT);
MotorINFO FRICR = Chassis_MOTORINFO_Init(&ControlCM,CHASSIS_MOTOR_SPEED_PID_DEFAULT);


//************************************************************************
//		     Gimbal_MOTORINFO_Init(rdc,func,ppid,spid)
//************************************************************************
//使用云台电机时，请务必确定校准过零点
MotorINFO GMP  = Gimbal_MOTORINFO_Init(2.0,&ControlGMP,
									   fw_PID_INIT(200,0,100, 	10000.0, 10000.0, 10000.0, 6000.0),
									   fw_PID_INIT(0.6,0,13, 	10000.0, 10000.0, 10000.0, 5000.0));
MotorINFO GMY  = Gimbal_MOTORINFO_Init(-1.0,&ControlGMY,
									   fw_PID_INIT(300,0,100, 10000.0, 10000.0, 10000.0, 6000.0),
									   fw_PID_INIT(0.6,0,13, 	10000.0, 10000.0, 10000.0, 5000.0));

//*************************************************************************
//			Normal_MOTORINFO_Init(rdc,func,ppid,spid)
//*************************************************************************
MotorINFO STIR = Normal_MOTORINFO_Init(36.0,&ControlNM,
								fw_PID_INIT(10.0, 0.0, 0.0, 	1080.0, 1080.0, 1080.0, 1080.0),
								fw_PID_INIT(30, 0.0, 0.0, 		15000.0, 15000.0, 15000.0, 15000.0),0);
MotorINFO NMUDL = Normal_MOTORINFO_Init(19.0,&ControlNM,
								fw_PID_INIT(10.0, 0.0, 0.0, 	1080.0, 1080.0, 1080.0, 1080.0),
								fw_PID_INIT(30, 0.0, 0.0, 		15000.0, 15000.0, 15000.0, 15000.0),0);
MotorINFO NMUDR = Normal_MOTORINFO_Init(19.0,&ControlNM,
								fw_PID_INIT(10.0, 0.0, 0.0, 	1080.0, 1080.0, 1080.0, 1080.0),
								fw_PID_INIT(30, 0.0, 0.0, 		15000.0, 15000.0, 15000.0, 15000.0),0);
	
MotorINFO NMUDFL = Normal_MOTORINFO_Init(19.0,&ControlNM,
								fw_PID_INIT(10.0, 0.0, 0.0, 	1080.0, 1080.0, 1080.0, 1080.0),
								fw_PID_INIT(30, 0.0, 0.0, 		15000.0, 15000.0, 15000.0, 15000.0),0);								
MotorINFO NMUDFR = Normal_MOTORINFO_Init(19.0,&ControlNM,
								fw_PID_INIT(10.0, 0.0, 0.0, 	1080.0, 1080.0, 1080.0, 1080.0),
								fw_PID_INIT(30, 0.0, 0.0, 		15000.0, 15000.0, 15000.0, 15000.0),0);		
MotorINFO UM1 = Normal_MOTORINFO_Init(19.0,&ControlNM,
								fw_PID_INIT(10, 0.1, 0.0, 	2000.0, 2000.0, 2000.0, 2000.0),
								fw_PID_INIT(30, 0.0, 0.0, 		15000.0, 15000.0, 15000.0, 15000.0),4000);								
MotorINFO UM2 = Normal_MOTORINFO_Init(19.0,&ControlNM,
								fw_PID_INIT(10, 0.1, 0.0, 	2000.0, 2000.0, 2000.0, 2000.0),
								fw_PID_INIT(30, 0.0, 0.0, 		15000.0, 15000.0, 15000.0, 15000.0),4000);	
MotorINFO NMCDL = Normal_MOTORINFO_Init(19.0,&ControlNM,
								fw_PID_INIT(10.0, 0.0, 0.0, 	1800.0, 1080.0, 1080.0, 1800.0),
								fw_PID_INIT(30, 0.0, 0.0, 		16384.0, 15000.0, 15000.0, 16384.0),0);								
MotorINFO NMCDR = Normal_MOTORINFO_Init(19.0,&ControlNM,
								fw_PID_INIT(10.0, 0.0, 0.0, 	1800.0, 1080.0, 1080.0, 1800.0),
								fw_PID_INIT(30, 0.0, 0.0, 		16384.0, 15000.0, 15000.0, 16384.0),0);		
MotorINFO UFM = Normal_MOTORINFO_Init(19.0,&ControlNM,
								fw_PID_INIT(30.0, 0.1, 0.2, 	3500.0, 3500.0, 3500.0, 3500.0),//500 15000 10
								fw_PID_INIT(30, 0.0, 0.0, 		15000.0, 15000.0, 15000.0, 15000.0),0);	
MotorINFO CM1 = Normal_MOTORINFO_Init(36.0,&ControlNM,
								fw_PID_INIT(10.0, 0.0, 0.0, 	1080.0, 1080.0, 1080.0, 1080.0),
								fw_PID_INIT(30, 0.0, 0.0, 		15000.0, 15000.0, 15000.0, 15000.0),0);
MotorINFO CM2 = Normal_MOTORINFO_Init(36.0,&ControlNM,
								fw_PID_INIT(10.0, 0.0, 0.0, 	1080.0, 1080.0, 1080.0, 1080.0),
								fw_PID_INIT(30, 0.0, 0.0, 		15000.0, 15000.0, 15000.0, 15000.0),0);
MotorINFO YTP = Normal_MOTORINFO_Init(36.0,&ControlNM,
								fw_PID_INIT(10.0, 0.0, 0.0, 	1080.0, 1080.0, 1080.0, 1080.0),
								fw_PID_INIT(30, 0.0, 0.0, 		15000.0, 15000.0, 15000.0, 15000.0),0);	
MotorINFO YTY = Normal_MOTORINFO_Init(36.0,&ControlNM,
								fw_PID_INIT(10.0, 0.0, 0.0, 	1080.0, 1080.0, 1080.0, 1080.0),
								fw_PID_INIT(30, 0.0, 0.0, 		15000.0, 15000.0, 15000.0, 15000.0),0);	
MotorINFO DOOR = Normal_MOTORINFO_Init(36.0,&ControlNM,
								fw_PID_INIT(10.0, 0.0, 0.0, 	1080.0, 1080.0, 1080.0, 1080.0),
								fw_PID_INIT(30, 0.0, 0.0, 		15000.0, 15000.0, 15000.0, 15000.0),0);		
MotorINFO SL = Normal_MOTORINFO_Init(36.0,&ControlNM,
								fw_PID_INIT(10.0, 0.0, 0.0, 	1080.0, 1080.0, 1080.0, 1080.0),
								fw_PID_INIT(30, 0.0, 0.0, 		15000.0, 15000.0, 15000.0, 15000.0),0);
MotorINFO SR = Normal_MOTORINFO_Init(36.0,&ControlNM,
								fw_PID_INIT(10.0, 0.0, 0.0, 	1080.0, 1080.0, 1080.0, 1080.0),
								fw_PID_INIT(30, 0.0, 0.0, 		15000.0, 15000.0, 15000.0, 15000.0),0);								
							
//2006是36 3508是19
//MotorINFO* can1[8]={&FRICL,&FRICR,0,0,&GMY,&GMP,&STIR,0};
//MotorINFO* can2[8]={&CMFL,&CMFR,&CMBL,&CMBR,&NMUDL,&NMUDR,0,0};

MotorINFO* can1[8]={&CMFL,&CMFR,&CMBL,&CMBR,&NMUDL,&NMUDR,&NMCDL,&NMCDR};
MotorINFO* can2[8]={&CM1,&CM2,&UM1,&UM2,&YTP,&YTY,&UFM,&DOOR};
int16_t someIntensity;
uint16_t someEncoder;
double someRealAngle;
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
			if((id->lastRead-ThisAngle)>4096)//编码器上溢
				id->RealAngle = id->RealAngle + (ThisAngle+8192-id->lastRead) * 360 / 8192.0 / id->ReductionRate;
			else//正常
				id->RealAngle = id->RealAngle - (id->lastRead - ThisAngle) * 360 / 8192.0 / id->ReductionRate;
		}
		else
		{
			if((ThisAngle-id->lastRead)>4096)//编码器下溢
				id->RealAngle = id->RealAngle - (id->lastRead+8192-ThisAngle) *360 / 8192.0 / id->ReductionRate;
			else//正常
				id->RealAngle = id->RealAngle + (ThisAngle - id->lastRead) * 360 / 8192.0 / id->ReductionRate;
		}
		ThisSpeed = id->RxMsgC6x0.RotateSpeed * 6 / id->ReductionRate;		//单位：度每秒
		
		id->Intensity = PID_PROCESS_Double(&(id->positionPID),&(id->speedPID),id->TargetAngle,id->RealAngle,ThisSpeed);
		
		id->s_count = 1;
		id->lastRead = ThisAngle;
	}
	else
	{
		id->s_count++;
	}		
}

void ControlCM(MotorINFO* id)
{
	//TargetAngle 代作为目标速度
	if(id==0) return;
	id->offical_speedPID.ref = (float)(id->TargetAngle);
	id->offical_speedPID.fdb = id->RxMsgC6x0.RotateSpeed;
	id->offical_speedPID.Calc(&(id->offical_speedPID));
	id->Intensity=(1.30f)*id->offical_speedPID.output;
}
#ifdef USE_GYRO
void ControlGMY(MotorINFO* id)
{
	if(id==0) return;
	if(id->s_count == 1)
	{		
		uint16_t 	ThisAngle = gyroZAngle;		
		double 		ThisSpeed = gyroZspeed;		
		int8_t 	dir;
		if(id->ReductionRate>=0) dir=1;
		else dir=-1;
		
		if(id->FirstEnter==1) {
			id->lastRead = ThisAngle;
			id->RealAngle =(double)(GM_YAW_ZERO - id->RxMsg6623.angle) * 360.0 / 8192.0 / id->ReductionRate;
			NORMALIZE_ANGLE180(id->RealAngle);
			id->FirstEnter = 0;
			return;
		}
		
		if(ThisAngle <= id->lastRead)
		{
			if((id->lastRead-ThisAngle) > 180)
				 id->RealAngle += (ThisAngle + 360 - id->lastRead)*dir;
			else
				 id->RealAngle -= (id->lastRead - ThisAngle)*dir;
		}
		else
		{
			if((ThisAngle-id->lastRead) > 180)
				 id->RealAngle -= (id->lastRead + 360 - ThisAngle)*dir;
			else
				 id->RealAngle += (ThisAngle - id->lastRead)*dir;
		}
		id->lastRead = ThisAngle ;
			
		id->Intensity = PID_PROCESS_Double(&(id->positionPID),&(id->speedPID),id->TargetAngle,id->RealAngle,ThisSpeed);
		
		id->s_count = 0;
	}
	else
	{
		id->s_count++;
	}		
}
void ControlGMP(MotorINFO* id)
{
	if(id==0) return;
	if(id->s_count == 1)
	{		
		int16_t 	ThisAngle = gyroYAngle;
		double 		ThisSpeed = gyroYspeed;
		int8_t 		dir;
		if(id->ReductionRate>=0) dir=1;
		else dir=-1;
		
		if(id->FirstEnter==1) {
			id->lastRead = ThisAngle;
			id->RealAngle =(double)(GM_PITCH_ZERO - id->RxMsg6623.angle) * 360.0 / 8192.0 / id->ReductionRate;
			NORMALIZE_ANGLE180(id->RealAngle);
			id->FirstEnter = 0;
			return;
		}
		
		if(ThisAngle <= id->lastRead)
		{
			if((id->lastRead-ThisAngle) > 180)
				 id->RealAngle += (ThisAngle + 360 - id->lastRead)*dir;
			else
				 id->RealAngle -= (id->lastRead - ThisAngle)*dir;
		}
		else
		{
			if((ThisAngle-id->lastRead) > 180)
				 id->RealAngle -= (id->lastRead + 360 - ThisAngle)*dir;
			else
				 id->RealAngle += (ThisAngle - id->lastRead)*dir;
		}
		id->lastRead = ThisAngle ;
			
		id->Intensity = GM_PITCH_GRAVITY_COMPENSATION + PID_PROCESS_Double(&(id->positionPID),&(id->speedPID),id->TargetAngle,id->RealAngle,ThisSpeed);
		MINMAX(id->Intensity,-id->speedPID.outputMax,id->speedPID.outputMax);
		
		id->s_count = 0;
	}
	else
	{
		id->s_count++;
	}		
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
	
	someEncoder = UFM.RxMsgC6x0.angle;
	someRealAngle = UFM.RealAngle;
	someIntensity = UFM.Intensity;
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
		can1_type = 1;
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
		can2_type = 1;
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
	id->RealAngle=0;
	id->TargetAngle=0;
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
