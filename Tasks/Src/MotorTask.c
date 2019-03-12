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
void ControlCM(MotorINFO *id);
void ControlANTI_CM(MotorINFO *id);
void ControlGMY(MotorINFO *id);
void ControlGMP(MotorINFO *id);
void ControlGM6020(MotorINFO *id);
void ControlGMY6020(MotorINFO *id);

////**********************************************************************
////					pid(kp,ki,kd,kprM,kirM,kdrM,rM)
////						kprM:kp result Max
////**********************************************************************
MotorINFO FRICL = SpeedBased_MOTORINFO_Init(&ControlCM,CHASSIS_MOTOR_SPEED_PID_DEFAULT);
MotorINFO FRICR = SpeedBased_MOTORINFO_Init(&ControlCM,CHASSIS_MOTOR_SPEED_PID_DEFAULT);


MotorINFO GMP  =  Gimbal6020_MOTORINFO_Init(1.0,&ControlGM6020,1200,-1900,20,
								fw_PID_INIT_EASY(80, 2, 20, 5000.0),
								fw_PID_INIT_EASY(12, 2, 2,	 15000.0));

//MotorINFO GMY  = Gimbal6020_MOTORINFO_Init(1.0,&ControlGM6020,2700,0,20,
//								fw_PID_INIT_EASY(1200.0, 0.0, 0.0, 1080.0),
//								fw_PID_INIT_EASY(2.0, 0.0, 0.1,	 0));

//MotorINFO GMP  =  Gimbal6020_MOTORINFO_Init(1.0,&ControlGMP,1000,-1700,20,
//								fw_PID_INIT_EASY(0.5, 0.01, 0.2, 1000),
//								fw_PID_INIT_EASY(1500, 80, 0,	 15000));
MotorINFO GMY  = Gimbal6020_MOTORINFO_Init(1.0,&ControlGMY,2700,400,20,
								fw_PID_INIT_EASY(0.32, 0.02, 0.2, 10),
								fw_PID_INIT_EASY(3000, 500, 200, 15000));

MotorINFO STIR = AngleBased_MOTORINFO_Init(36.0,&ControlNM,
								fw_PID_INIT_EASY(10.0, 0.0, 0.0, 1200.0),
								fw_PID_INIT_EASY(30, 0.0, 0.0,	 15000.0));
MotorINFO CML = AngleBased_MOTORINFO_Init(19.0,&ControlNM,
								fw_PID_INIT_EASY(10.0, 0.0, 0.0, 1200.0),
								fw_PID_INIT_EASY(40, 0.0, 5.0,	 15000.0));
MotorINFO CMR = AngleBased_MOTORINFO_Init(19.0,&ControlNM,
								fw_PID_INIT_EASY(10.0, 0.0, 0.0, 1500.0),
								fw_PID_INIT_EASY(40, 0.0, 5.0,	 15000.0));
								
MotorINFO* can1[8]={&FRICL,&FRICR,0,0,&GMP,&GMY,0,0};
//MotorINFO* can1[8]={0,0,0,0,0,0,0,0};
MotorINFO* can2[8]={&CML,&CMR,0,0,&STIR,0,0,0};
//MotorINFO* can2[8]={0,0,0,0,&STIR,0,0,0};



void ControlNM(MotorINFO* id){
	if(id==0) return;
	if(id->s_count == 1){		
		uint16_t 	ThisAngle;	
		double 		ThisSpeed;	
		ThisAngle = id->RxMsgC6x0.angle;				//未处理角度
		if(id->FirstEnter==1) {id->lastRead = ThisAngle;id->FirstEnter = 0;return;}
		if(ThisAngle<=id->lastRead){
			if((id->lastRead-ThisAngle)>4000)//编码器上溢
				id->Real = id->Real + (ThisAngle+8192-id->lastRead) * 360 / 8192.0 / id->ReductionRate;
			else//正常
				id->Real = id->Real - (id->lastRead - ThisAngle) * 360 / 8192.0 / id->ReductionRate;
		}
		else{
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
	else{
		id->s_count++;
	}		
}
void ControlGM6020(MotorINFO* id){
	if(id==0) return;
	if(id->s_count == 1){		
		uint16_t 	ThisAngle;	
		double 		ThisSpeed;	
		ThisAngle = id->RxMsgC6x0.angle;				//未处理角度
		float EncoderAngle = (id->RxMsgC6x0.angle - id->Zero) * 360 / 8192.0f / id->ReductionRate;
		NORMALIZE_ANGLE180(EncoderAngle);
		if(id->FirstEnter==1) {id->lastRead = ThisAngle;
			id->Real=EncoderAngle;
		id->FirstEnter = 0;return;}
		if(ThisAngle<=id->lastRead){
			if((id->lastRead-ThisAngle)>4000)//编码器上溢
				id->Real = id->Real + (ThisAngle+8192-id->lastRead) * 360 / 8192.0 / id->ReductionRate;
			else//正常
				id->Real = id->Real - (id->lastRead - ThisAngle) * 360 / 8192.0 / id->ReductionRate;
		}
		else{
			if((ThisAngle-id->lastRead)>4000)//编码器下溢
				id->Real = id->Real - (id->lastRead+8192-ThisAngle) *360 / 8192.0 / id->ReductionRate;
			else//正常
				id->Real = id->Real + (ThisAngle - id->lastRead) * 360 / 8192.0 / id->ReductionRate;
		}
		ThisSpeed = id->RxMsgC6x0.rotateSpeed * 6 / id->ReductionRate;		//单位：度每秒
		
		id->Intensity = id->Compensation +PID_PROCESS_Double(&(id->positionPID),&(id->speedPID),id->Target,id->Real,ThisSpeed);
		
		id->s_count = 0;
		id->lastRead = ThisAngle;
	}
	else{
		id->s_count++;
	}		
}

void ControlCM(MotorINFO* id){
	//Target 代作为目标速度
	if(id==0) return;
	id->offical_speedPID.ref = (float)(id->Target);
	id->offical_speedPID.fdb = id->RxMsgC6x0.rotateSpeed;
	id->offical_speedPID.Calc(&(id->offical_speedPID));
	id->Intensity=(1.30f)*id->offical_speedPID.output;
}

void ControlGMY6020(MotorINFO* id){
	if(id==0) return;
	if(id->s_count == 1){		
		uint16_t 	ThisAngle;	
		double 		ThisSpeed;	
		ThisAngle = imu.yaw;				//未处理角度
		float EncoderAngle = (id->RxMsgC6x0.angle - id->Zero) *360/8192.0f/ id->ReductionRate;
		NORMALIZE_ANGLE180(EncoderAngle);
		if(id->FirstEnter==1){
			id->lastRead = ThisAngle;
			id->Real=EncoderAngle;
			id->FirstEnter = 0;
			return;
		}
		
		if(ThisAngle <= id->lastRead){
			if((id->lastRead-ThisAngle) > 180)
				 id->Real += (ThisAngle + 360 - id->lastRead);
			else
				 id->Real -= (id->lastRead - ThisAngle);
		}
		else{
			if((ThisAngle-id->lastRead) > 180)
				 id->Real -= (id->lastRead + 360 - ThisAngle);
			else
				 id->Real += (ThisAngle - id->lastRead);
		}
		ThisSpeed = id->RxMsgC6x0.rotateSpeed * 6 / id->ReductionRate;		//单位：度每秒
    
		id->Intensity = id->Compensation +PID_PROCESS_Double(&(id->positionPID),&(id->speedPID),id->Target,id->Real,ThisSpeed);
		
		id->s_count = 0;
		id->lastRead = ThisAngle;
	}
	else{
		id->s_count++;
	}		
}
void ControlGM(MotorINFO* id,float ThisAngle,float ThisSpeed, uint8_t type)
{
	if(id==0) return;
	if(id->s_count == 1)
	{
		static uint8_t Reseted = 0;
		float EncoderAngle = -(id->Zero - id->RxMsgC6x0.angle) * 360 / 8192.0f / id->ReductionRate;
		NORMALIZE_ANGLE180(EncoderAngle);
		int8_t 	dir;
	  dir=1;
	
		
		if(id->FirstEnter==1) {
			id->lastRead = ThisAngle;
			id->Real = EncoderAngle;
			id->FirstEnter = 0;
			return;
		}
		
		if(ThisAngle <= id->lastRead)
		{
			if((id->lastRead-ThisAngle) > 180)
				 id->Real += (ThisAngle + 360 - id->lastRead);
			else
				 id->Real -= (id->lastRead - ThisAngle);
		}
		else
		{
			if((ThisAngle-id->lastRead) > 180)
				 id->Real -= (id->lastRead + 360 - ThisAngle)*dir;
			else
				 id->Real += (ThisAngle - id->lastRead)*dir;
		}
		if(fabs(id->Real-id->Target)<5) Reseted = 1;
		id->lastRead = ThisAngle ;
//		if(type == 'Y')
//			MINMAX(id->Target, id->Real - EncoderAngle - id->Maxrange, id->Real - EncoderAngle + id->Maxrange);
//		
//		if(Reseted==0) id->positionPID.outputMax = 1.0;
//		else id->positionPID.outputMax = 8.0;
		
		if(type=='P')
			id->Intensity = id->Compensation + PID_PROCESS_Double(&(id->positionPID),&(id->speedPID),id->Target,id->Real,ThisSpeed);
		else{
			id->Intensity = id->Compensation + PID_PROCESS_Double(&(id->positionPID),&(id->speedPID),id->Target,id->Real,ThisSpeed);
		}  
		MINMAX(id->Intensity,-id->speedPID.outputMax,id->speedPID.outputMax);
		
		//id->s_count = 0;
	}
	else
	{
		id->s_count++;
	}		
}
void ControlGMP(MotorINFO* id){
	ControlGM(id,imu.pit,imu.wy,'P');
}
void ControlGMY(MotorINFO* id){
	ControlGM(id,imu.yaw,-imu.wz,'Y');
}
//#endif

//CAN
void setCAN11(){
	CanTxMsgTypeDef pData;
	hcan1.pTxMsg = &pData;
	
	hcan1.pTxMsg->StdId = 0x200;
	hcan1.pTxMsg->ExtId = 0;
	hcan1.pTxMsg->IDE = CAN_ID_STD;
	hcan1.pTxMsg->RTR = CAN_RTR_DATA;
	hcan1.pTxMsg->DLC = 0x08;
	
	for(int i=0;i<4;i++){
		if(can1[i]==0){
			hcan1.pTxMsg->Data[i*2]   = 0;
			hcan1.pTxMsg->Data[i*2+1] = 0;
		}
		else{
			hcan1.pTxMsg->Data[i*2]   = (uint8_t)(can1[i]->Intensity >> 8);
			hcan1.pTxMsg->Data[i*2+1] = (uint8_t)can1[i]->Intensity;
		}
	}

	if(can1_update == 1 && can1_type == 1){
		HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_DisableIRQ(USART1_IRQn);
		HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
		HAL_NVIC_DisableIRQ(TIM7_IRQn);
		#ifdef DEBUG_MODE
			HAL_NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
		#endif
		if(HAL_CAN_Transmit_IT(&hcan1) != HAL_OK){
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

