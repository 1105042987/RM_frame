/**
  ******************************************************************************
  * File Name          : ManifoldTask.c
  * Description        : 妙算通信处理任务，得到自动打击位置
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#include "includes.h"

#ifdef __GNUC__
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
	#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif 
PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&MANIFOLD_UART , (uint8_t *)&ch, 1, 0xFFFF);
	return ch;
}

uint8_t auto_attack_temp = 0;
uint8_t auto_receiving = 0;
uint8_t auto_received = 0;
uint8_t auto_buffer[7] = {0}; 
uint8_t auto_buffercnt = 0;

void manifoldUartRxCpltCallback() //妙算自动打击通信数据处理
{
	if(auto_receiving) 
		{
			auto_buffer[auto_buffercnt] = auto_attack_temp;
			auto_buffercnt++;
			
			if(auto_buffercnt == 4)
			{
				if(auto_buffer[3]==0xA6)
				{
//					find_enemy = 1;
				}
				else if(auto_buffer[3]==0xA4)
				{
//					find_enemy = 0;
				}
				else
				{
					auto_receiving = 0;
					auto_buffercnt = 0;
				}
			}
			
			if(auto_buffercnt == 7)
			{
				if (auto_buffer[6] == 0xA7) 
				{
//					enemy_yaw = (0x0000 | auto_buffer[2]) | (auto_buffer[1]<<8);
//					enemy_pitch = (0x0000 | auto_buffer[5]) | (auto_buffer[4]<<8);
//					enemy_detect_cnt = 0;    //有更新数据
				}
				auto_receiving = 0;
				auto_buffercnt = 0;
			}
		}
		else 
		{
			if(auto_attack_temp == 0xA5)
			{
				auto_receiving = 1;
				auto_buffercnt = 0;
				auto_buffer[0] = auto_attack_temp;
				auto_buffercnt++;
			}
		}
		if(HAL_UART_Receive_DMA(&MANIFOLD_UART, &auto_attack_temp, 1) != HAL_OK)
		{
			Error_Handler();
	  }
}

void InitManifoldUart()
{
	if(HAL_UART_Receive_DMA(&MANIFOLD_UART, &auto_attack_temp, 1) != HAL_OK){  //接收自动打击数据
		Error_Handler();
	} 
}
