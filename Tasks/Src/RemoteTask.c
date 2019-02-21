/**
  ******************************************************************************
  * File Name          : RemoteTask.c
  * Description        : é¥æŽ§å™¨å¤„ç†ä»»åŠ¡
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#include "includes.h"

uint8_t rc_data[18];
RC_Ctl_t RC_CtrlData;
InputMode_e inputmode = REMOTE_INPUT; 
FunctionMode_e functionmode = UPPER_POS;

RemoteSwitch_t g_switch1;


/*²¦¸ËÊý¾Ý´¦Àí*/   
void GetRemoteSwitchAction(RemoteSwitch_t *sw, uint8_t val)
{
	static uint32_t switch_cnt = 0;

	sw->switch_value_raw = val;
	sw->switch_value_buf[sw->buf_index] = sw->switch_value_raw;

	//value1 value2µÄÖµÆäÊµÊÇÒ»ÑùµÄ
	//value1¸ß4Î»Ê¼ÖÕÎª0
	sw->switch_value1 = (sw->switch_value_buf[sw->buf_last_index] << 2)|
	(sw->switch_value_buf[sw->buf_index]);

	sw->buf_end_index = (sw->buf_index + 1)%REMOTE_SWITCH_VALUE_BUF_DEEP;

	sw->switch_value2 = (sw->switch_value_buf[sw->buf_end_index]<<4)|sw->switch_value1;	

	//Èç¹ûÁ½´ÎÊý¾ÝÒ»Ñù£¬¼´Ã»ÓÐ¸üÐÂÊý¾Ý£¬²¦¸Ë²»¶¯
	if(sw->switch_value_buf[sw->buf_index] == sw->switch_value_buf[sw->buf_last_index])
	{
		switch_cnt++;	
	}
	else
	{
		switch_cnt = 0;
	}
	//Èç¹û²¦¸ËÎ¬³ÖÁËÒ»¶¨Ê±¼ä£¬¼´Á¬ÐøÀ´ÁË40Ö¡Ò»ÑùµÄÊý¾Ý£¬Ôò°Ñ²¦¸ËÊý¾ÝÐ´Èëswitch_long_value
	if(switch_cnt >= 40)
	{
		sw->switch_long_value = sw->switch_value_buf[sw->buf_index]; 	
	}
	//Ö¸ÏòÏÂÒ»¸ö»º³åÇø
	sw->buf_last_index = sw->buf_index;
	sw->buf_index++;		
	if(sw->buf_index == REMOTE_SWITCH_VALUE_BUF_DEEP)
	{
		sw->buf_index = 0;	
	}			
}


//Ò£¿ØÆ÷Êý¾Ý½âËã
void RemoteDataProcess(uint8_t *pData)
{
	HAL_IWDG_Refresh(&hiwdg);
	if(pData == NULL)
	{
			return;
	}
	//Ò£¿ØÆ÷ 11*4 + 2*2 = 48£¬ÐèÒª 6 Bytes
	//16Î»£¬Ö»¿´µÍ11Î»
	RC_CtrlData.rc.ch0 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF; 
	RC_CtrlData.rc.ch1 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;
	RC_CtrlData.rc.ch2 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) |
											 ((int16_t)pData[4] << 10)) & 0x07FF;
	RC_CtrlData.rc.ch3 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) & 0x07FF;
	
	//16Î»£¬Ö»¿´×îµÍÁ½Î»
	RC_CtrlData.rc.s1 = ((pData[5] >> 4) & 0x000C) >> 2;
	RC_CtrlData.rc.s2 = ((pData[5] >> 4) & 0x0003);

	//Êó±êÐèÒª 8 Bytes
	RC_CtrlData.mouse.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);
	RC_CtrlData.mouse.y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);
	RC_CtrlData.mouse.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8);    

	RC_CtrlData.mouse.press_l = pData[12];
	RC_CtrlData.mouse.press_r = pData[13];
	
	//¼üÅÌÐèÒª 2 Bytes = 16 bits £¬Ã¿Ò»Î»¶ÔÓ¦Ò»¸ö¼ü
	RC_CtrlData.key.v = ((int16_t)pData[14]) | ((int16_t)pData[15] << 8);

	//ÊäÈë×´Ì¬ÉèÖÃ
	if(RC_CtrlData.rc.s2 == 1) inputmode = REMOTE_INPUT;
	else if(RC_CtrlData.rc.s2 == 3) inputmode = KEY_MOUSE_INPUT; 
	else inputmode = STOP; 
	
	//¹¦ÄÜ×´Ì¬ÉèÖÃ
	if(RC_CtrlData.rc.s1 == 1) functionmode = UPPER_POS; 
	else if(RC_CtrlData.rc.s1 == 3) functionmode = MIDDLE_POS; 
	else functionmode = LOWER_POS;
	
	///×óÉÏ½Ç²¦¸Ë×´Ì¬£¨RC_CtrlData.rc.s1£©»ñÈ¡
	//ÓÃÓÚÒ£¿ØÆ÷·¢Éä¿ØÖÆ
	GetRemoteSwitchAction(&g_switch1, RC_CtrlData.rc.s1);
	
	switch(inputmode)
	{
		case REMOTE_INPUT:               
		{
			if(WorkState > 0)
			{ 
				RemoteControlProcess(&(RC_CtrlData.rc));
			}
		}break;
		case KEY_MOUSE_INPUT:              
		{
			if(WorkState > 0)
			{ 
				MouseKeyControlProcess(&RC_CtrlData.mouse,&RC_CtrlData.key,&(RC_CtrlData.rc));
			}
		}break;
		case STOP:
		{
			 
		}break;
	}	
}

//³õÊ¼»¯Ò£¿ØÆ÷´®¿ÚDMA½ÓÊÕ
void InitRemoteControl(){
	if(HAL_UART_Receive_DMA(&RC_UART, rc_data, 18) != HAL_OK){
			Error_Handler();
	} 
	FunctionTaskInit();
	rx_free = 1;
}

//Ò£¿ØÆ÷´®¿ÚÖÐ¶ÏÈë¿Úº¯Êý£¬´Ó´Ë´¦¿ªÊ¼Ö´ÐÐ
uint8_t rc_first_frame = 0;
uint8_t rc_update = 0;
uint8_t rc_cnt = 0;
uint8_t tx_cnt = 200;

uint8_t  tx_free = 1;
uint8_t  rx_free = 1;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	if(UartHandle == &RC_UART){
		rc_update = 1;
		rx_free = 1;
	}
	else if(UartHandle == &GYRO_UART)
	{
		#ifdef USE_GYRO
		gyroUartRxCpltCallback();
		#endif
	}
	else if(UartHandle == &JUDGE_UART)
	{
		judgeUartRxCpltCallback();  //²ÃÅÐÏµÍ³Êý¾Ý½âËã
	}
	else if(UartHandle == &AUTOAIM_UART)
	{
		#ifdef USE_AUTOAIM
		AutoAimUartRxCpltCallback();
		#endif /*USE_AUTOAIM*/
	}
}
extern uint8_t sendfinish;
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	if(UartHandle == &JUDGE_UART)
	{
		tx_free = 1;
	}else if(UartHandle == &huart8){
		sendfinish = 1;
	}
}
void UART_IDLE_Handler(UART_HandleTypeDef *UartHandle)
{
	if(UartHandle == &UPPER_UART)
	{
		#ifdef DEBUG_MODE
		ctrlUartRxCpltCallback();
		#endif
	}
}
