/**
  ******************************************************************************
  * File Name          : CANTask.c
  * Description        : CAN通信任务
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#include "includes.h"

#define CanRxGetU16(canRxMsg, num) (((uint16_t)canRxMsg.Data[num * 2] << 8) | (uint16_t)canRxMsg.Data[num * 2 + 1])
#define CAN_COMM_BASE_ID 0x300
uint8_t isRcan1Started = 0, isRcan2Started = 0;
uint8_t isCan11FirstRx = 0, isCan12FirstRx = 0, isCan21FirstRx = 0, isCan22FirstRx = 0;
CanRxMsgTypeDef Can1RxMsg,Can2RxMsg;
#ifdef CAN13
#define CAN3
CAN_DATA_t 	sendData,receiveData[CAN13];
uint8_t 	maxSendSize = CAN13;
#else
#ifdef CAN23
#define CAN3
CAN_DATA_t 	sendData,receiveData[CAN23];
uint8_t 	maxSendSize = CAN23;
#endif
#endif
uint8_t can1_update = 1;
uint8_t can1_type = 1;
uint8_t can2_update = 1;
uint8_t can2_type = 1;

/********************CAN发送*****************************/
//CAN数据标记发送，保证发送资源正常
void HAL_CAN_TxCpltCallback(CAN_HandleTypeDef* hcan)
{
	if(hcan == &hcan1){
		can1_update = 1;
	}
	else if(hcan == &hcan2){
		can2_update = 1;
	}
}

/********************CAN******************************/
void InitCanReception()
{
	#ifndef CAN11
		isCan11FirstRx = 1;
	#endif
	#ifndef CAN12
		isCan12FirstRx = 1;
	#endif
	#ifndef CAN21
		isCan21FirstRx = 1;
	#endif
	#ifndef CAN22
		isCan22FirstRx = 1;
	#endif
	
	can1_type = 1;
	can2_type = 1;
	#ifdef CAN12
		can1_type = 2;
	#endif
	#ifdef CAN22
		can2_type = 2;
	#endif
	
	//http://www.eeworld.com.cn/mcu/article_2016122732674_3.html
	hcan1.pRxMsg = &Can1RxMsg;
	/*##-- Configure the CAN1 Filter ###########################################*/
	CAN_FilterConfTypeDef  sFilterConfig;
	sFilterConfig.FilterNumber = 0;//14 - 27//14
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = 0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.BankNumber = 14;
	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
	if(HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0) != HAL_OK){
		Error_Handler(); 
	}
	isRcan1Started = 1;
	
	hcan2.pRxMsg = &Can2RxMsg;
	/*##-- Configure the CAN2 Filter ###########################################*/
	CAN_FilterConfTypeDef sFilterConfig2;
	sFilterConfig2.FilterNumber = 14;//14 - 27//14
	sFilterConfig2.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig2.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig2.FilterIdHigh = 0x0000;
	sFilterConfig2.FilterIdLow = 0x0000;
	sFilterConfig2.FilterMaskIdHigh = 0x0000;
	sFilterConfig2.FilterMaskIdLow = 0x0000;
	sFilterConfig2.FilterFIFOAssignment = 0;
	sFilterConfig2.FilterActivation = ENABLE;
	sFilterConfig2.BankNumber = 14;
	HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig2);
	if(HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0) != HAL_OK){
		Error_Handler(); 
	}
	isRcan2Started = 1;
}
 
//CAN接收中断入口函数
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan){
	uint8_t flag = 0;
	if(hcan == &hcan1)	//CAN1数据
	{
		for(int i=0;i<8;i++)
		{
			if(can1[i]==0)continue;
			if(Can1RxMsg.StdId==can1[i]->RXID)
			{
				if(i<4) isCan11FirstRx = 1;
				else if(i<8) isCan12FirstRx = 1;
				flag=1;
				switch(can1[i]->ESCtype)
				{
					case ESC_C6x0:
					{
						can1[i]->RxMsgC6x0.angle		 = CanRxGetU16(Can1RxMsg, 0);
						can1[i]->RxMsgC6x0.rotateSpeed   = CanRxGetU16(Can1RxMsg, 1);
						can1[i]->RxMsgC6x0.moment		 = CanRxGetU16(Can1RxMsg, 2);
					}
					case ESC_6623:
					{
						can1[i]->RxMsg6623.angle		 = CanRxGetU16(Can1RxMsg, 0);
						can1[i]->RxMsg6623.realIntensity = CanRxGetU16(Can1RxMsg, 1);
						can1[i]->RxMsg6623.giveIntensity = CanRxGetU16(Can1RxMsg, 2);
					}
				}
			}
			
		}
		#ifdef CAN3
		if(Can1RxMsg.StdId == 0x200||Can1RxMsg.StdId == 0x1ff) flag=1;
		else if(Can1RxMsg.StdId >= CAN_COMM_BASE_ID)
		{
			flag = Can1RxMsg.StdId-CAN_COMM_BASE_ID;
			if(flag<maxSendSize)
			{
				for(int i=0;i<4;i++) receiveData[flag].data[i] = CanRxGetU16(Can1RxMsg, i);
				flag = 1;
			}
		}
		#endif
		if(flag==0) Error_Handler();
		if(HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0) != HAL_OK){
			isRcan1Started = 0;
		}else{
			isRcan1Started = 1;
		}
	}
	else if(hcan == &hcan2)//CAN2数据
	{
		for(int i=0;i<8;i++)
		{
			if(can2[i]==0)continue;
			if(Can2RxMsg.StdId==can2[i]->RXID)
			{
				if(i<4) isCan21FirstRx = 1;
				else if(i<8) isCan22FirstRx = 1;
				flag=1;
				switch(can2[i]->ESCtype)
				{
					case ESC_C6x0:
					{
						can2[i]->RxMsgC6x0.angle		 = CanRxGetU16(Can2RxMsg, 0);
						can2[i]->RxMsgC6x0.rotateSpeed   = CanRxGetU16(Can2RxMsg, 1);
						can2[i]->RxMsgC6x0.moment		 = CanRxGetU16(Can2RxMsg, 2);
					}
					case ESC_6623:
					{
						can2[i]->RxMsg6623.angle		 = CanRxGetU16(Can2RxMsg, 0);
						can2[i]->RxMsg6623.realIntensity = CanRxGetU16(Can2RxMsg, 1);
						can2[i]->RxMsg6623.giveIntensity = CanRxGetU16(Can2RxMsg, 2);
					}
				}
			}
		}
		#ifdef CAN3
		if(Can2RxMsg.StdId == 0x200||Can2RxMsg.StdId == 0x1ff) flag=1;
		else if(Can2RxMsg.StdId >= CAN_COMM_BASE_ID)
		{
			flag = Can2RxMsg.StdId-CAN_COMM_BASE_ID;
			if(flag<maxSendSize)
			{
				for(int i=0;i<4;i++) receiveData[flag].data[i] = CanRxGetU16(Can2RxMsg, i);
				flag = 1;
			}
		}
		#endif
		if(flag==0) Error_Handler();
		if(HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0) != HAL_OK)
		{
			isRcan2Started = 0;
		}else{
			isRcan2Started = 1;
		}
	}
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan == &hcan1) 
	{
		if(HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0) != HAL_OK){
			isRcan1Started = 0;
		}else{
			isRcan1Started = 1;
		}
	}
	else if(hcan == &hcan2)
	{
		if(HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0) != HAL_OK)
		{
			isRcan2Started = 0;
		}else{
			isRcan2Started = 1;
		}
	}
}
#ifdef CAN3
void setCANMessage(uint8_t index)
{
	uint8_t target;
	#ifdef CAN13
	#ifndef CAN23
		CAN_HandleTypeDef* hcan_P = &hcan1;
		uint8_t* type = &can1_type;
		uint8_t* update = &can1_update;
		#ifdef CAN11
			target=1;
		#else
		#ifdef CAN12
			target=2; 
		#else
			target=3;
		#endif
		#endif
	#endif
	#endif
	#ifdef CAN23
	#ifndef CAN13
		CAN_HandleTypeDef* hcan_P = &hcan2;
		uint8_t* type = &can2_type;
		uint8_t* update = &can2_update;
		#ifdef CAN21
			target=1;
		#else
		#ifdef CAN22
			target=2; 
		#else
			target=3;
		#endif
		#endif
	#endif
	#endif
	
	CanTxMsgTypeDef pData;
	hcan_P->pTxMsg = &pData;
	
	hcan_P->pTxMsg->StdId = CAN_COMM_BASE_ID+index;
	hcan_P->pTxMsg->ExtId = 0;
	hcan_P->pTxMsg->IDE = CAN_ID_STD;
	hcan_P->pTxMsg->RTR = CAN_RTR_DATA;
	hcan_P->pTxMsg->DLC = 0x08;

	for(int i=0;i<4;i++){
		hcan_P->pTxMsg->Data[i*2]   = (uint8_t)(sendData.data[i] >> 8);
		hcan_P->pTxMsg->Data[i*2+1] = (uint8_t)sendData.data[i];
	}
	if(*update == 1 && *type == 3)
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
		if(HAL_CAN_Transmit_IT(hcan_P) != HAL_OK)
		{
			Error_Handler();
		}
		*update = 0;
		*type = target;
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
#endif


