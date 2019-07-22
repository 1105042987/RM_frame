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
uint8_t isRcan1Started = 0, isRcan2Started = 0;
uint8_t isCan11FirstRx = 0, isCan12FirstRx = 0, isCan21FirstRx = 0, isCan22FirstRx = 0;
CanRxMsgTypeDef Can1RxMsg,Can2RxMsg;
ESCC6x0RxMsg_t CMFLRx,CMBLRx,CMFRRx,CMBRRx;
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
	else if(hcan == &hcan2)
	{
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
						can1[i]->RxMsgC6x0.RotateSpeed   = CanRxGetU16(Can1RxMsg, 1);
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
		if(!flag) Error_Handler();
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
						can2[i]->RxMsgC6x0.RotateSpeed   = CanRxGetU16(Can2RxMsg, 1);
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
		if(!flag) Error_Handler();
		if(HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0) != HAL_OK)
		{
			isRcan2Started = 0;
		}else{
			isRcan2Started = 1;
		}
	}
}


