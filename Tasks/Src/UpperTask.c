/**
  ******************************************************************************
  * File Name       : UpperTask.c
  * Description     : 上位机处理任务，进行串口调试
  * Author			: 秦绍飞
  ******************************************************************************
  *
  * Copyright (c) 2019 Team JiaoLong-ShanghaiJiaoTong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#include "includes.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#define fw_printf(...) printf(__VA_ARGS__)

#ifdef DEBUG_MODE
//--------------------底层接收驱动部分-------------------//
char buf[REC_LEN];
void zykProcessData(void);
void ctrlUartRxCpltCallback()
{
	rx_free = 0;
	if((__HAL_UART_GET_FLAG(&UPPER_UART,UART_FLAG_IDLE) != RESET))  
	{
		__HAL_UART_CLEAR_IDLEFLAG(&UPPER_UART);  
		HAL_UART_DMAStop(&UPPER_UART);
		uint32_t rx_len =  REC_LEN - UPPER_UART.hdmarx->Instance->NDTR;
		buf[rx_len]='\0';
		zykProcessData();
	}
	if(HAL_UART_Receive_DMA(&UPPER_UART, (uint8_t *)buf, REC_LEN) != HAL_OK)
	{
		Error_Handler();
		printf( "CtrlUart error" );
	}
}

void ctrlUartInit(){
	if(HAL_UART_Receive_DMA(&UPPER_UART,(uint8_t *)buf, REC_LEN) != HAL_OK){
		Error_Handler();
		printf( "InitCtrlUart error" );
	} 
}

void zykProcessData()
{
	if(RX_DONE)
	{
		char data[15];
		/////////// GM CONTROL ////////////////
		if(strcmp(buf,"U")==0)
		{
			fw_printf("UP\r\n");
			GMP.TargetAngle+=20;
		}
		else if(strcmp(buf,"D")==0)
		{
			fw_printf("DOWN\r\n");
			GMP.TargetAngle-=20;
		}
		if(strcmp(buf,"L")==0)
		{
			fw_printf("LEFT\r\n");
			GMY.TargetAngle+=20;
		}
		else if(strcmp(buf,"R")==0)
		{
			fw_printf("RIGHT\r\n");
			GMY.TargetAngle-=20;
		}
//		GM PID yaw
		else if(ComProtocal(buf,"#GMYPP","$","@",data))
		{
			float p=atof(data);
			GMY.positionPID.kp = p;
			fw_printf("Yaw position P change to %f\r\n",p);
		}
		else if(ComProtocal(buf,"#GMYPI","$","@",data))
		{
			float i=atof(data);
			GMY.positionPID.ki = i;
			fw_printf("Yaw position I change to %f\r\n",i);
		}
		else if(ComProtocal(buf,"#GMYPD","$","@",data))
		{
			float d=atof(data);
			GMY.positionPID.kd = d;
			fw_printf("Yaw position D change to %f\r\n",d);
		}
		else if(ComProtocal(buf,"#GMYSP","$","@",data))
		{
			float p=atof(data);
			GMY.speedPID.kp = p;
			fw_printf("Yaw speed P change to %f\r\n",p);
		}
		else if(ComProtocal(buf,"#GMYSI","$","@",data))
		{
			float i=atof(data);
			GMY.speedPID.ki = i;
			fw_printf("Yaw speed I change to %f\r\n",i);
		}
		else if(ComProtocal(buf,"#GMYSD","$","@",data))
		{
			float d=atof(data);
			GMY.speedPID.kd = d;
			fw_printf("Yaw speed D change to %f\r\n",d);
		}
//		GM PID pitch
		else if(ComProtocal(buf,"#GMPPP","$","@",data))
		{
			float p=atof(data);
			GMP.positionPID.kp = p;
			fw_printf("Pitch position P change to %f\r\n",p);
		}
		else if(ComProtocal(buf,"#GMPPI","$","@",data))
		{
			float i=atof(data);
			GMP.positionPID.ki = i;
			fw_printf("Pitch position I change to %f\r\n",i);
		}
		else if(ComProtocal(buf,"#GMPPD","$","@",data))
		{
			float d=atof(data);
			GMP.positionPID.kd = d;
			fw_printf("Pitch position D change to %f\r\n",d);
		}
		else if(ComProtocal(buf,"#GMPSP","$","@",data))
		{
			float p=atof(data);
			GMP.speedPID.kp = p;
			fw_printf("Pitch speed P change to %f\r\n",p);
		}
		else if(ComProtocal(buf,"#GMPSI","$","@",data))
		{
			float i=atof(data);
			GMP.speedPID.ki = i;
			fw_printf("Pitch speed I change to %f\r\n",i);
		}
		else if(ComProtocal(buf,"#GMPSD","$","@",data))
		{
			float d=atof(data);
			GMP.speedPID.kd = d;
			fw_printf("Pitch speed D change to %f\r\n",d);
		}

		else if(strcmp(buf,"RD1")==0)
		{
			float realSpeed2=-gyroZspeed/(float)(32.8);
			fw_printf("#DATA%.2f@%.2f@%.2f$",GMY.positionPID.output,realSpeed2,GMY.RxMsg6623.angle*360/8192.0);
		}
		else if(strcmp(buf,"RD2")==0)
		{
			//speed
			float realSpeed2=-gyroXspeed/(float)(32.8);
			fw_printf("#DATA%.2f@%.2f@%.2f$",GMP.positionPID.output,realSpeed2,GMP.RxMsg6623.angle*360/8192.0);

		}
		strcpy(buf,"\0");
		RX_STA=0;
	}
}


//--------------------数据解析协议部分-------------------//
uint8_t ComProtocal(char*rxbuf,char*head,char*end,char* separater,char dataout[15])
{
    uint8_t headlength,endlength,datalength,totallength;
    uint8_t i=0;
    char temp[50]="";
    char*splitchar;
    headlength=strlen(head);
    endlength=strlen(end);
    totallength=strlen(rxbuf);
	datalength=totallength-headlength-endlength;
    strncpy(temp,rxbuf,headlength);
    temp[headlength]='\0';
    if(strcmp(temp,head))
    {
        return 0;
    }
    strncpy(temp,rxbuf+totallength-endlength,endlength);
    temp[endlength]='\0';
    if(strcmp(temp,end))
    {
        return 0;
    }
    strncpy(temp,rxbuf+headlength,datalength);
    temp[datalength]='\0';

    splitchar=strtok((char*)temp,separater);
    while(splitchar!=NULL)
    {
        sprintf(dataout,"%s",splitchar);
        splitchar=strtok(NULL,separater);
    }
    return i;
}

//--------------------任务循环部分-------------------//
//debug监测变量
void dataCallBack()
{
	static uint16_t pcnt = 0;
	if(pcnt>100)
		{
			pcnt = 0;
		}
		else pcnt++;
}
#else
void ctrlUartRxCpltCallback()
{
	
}
#endif
