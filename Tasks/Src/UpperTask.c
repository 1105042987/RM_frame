/**
  ******************************************************************************
  * File Name       : UpperTask.c
  * Description     : 上位机处理任务，进行串口调试
  * Author			: 秦绍飞
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#include "includes.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#define fw_printf(...) printf(__VA_ARGS__)

#ifdef __GNUC__
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
	#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif 
PUTCHAR_PROTOTYPE
{
 	HAL_UART_Transmit(&DEBUG_UART , (uint8_t *)&ch, 1, 0xFFFF);
	return ch;
}

#ifdef DEBUG_MODE
//--------------------底层接收驱动部分-------------------//
char buf[REC_LEN];
void zykProcessData(void);
void ctrlUartRxCpltCallback()
{
	if((__HAL_UART_GET_FLAG(&DEBUG_UART,UART_FLAG_IDLE) != RESET))  
	{
		__HAL_UART_CLEAR_IDLEFLAG(&DEBUG_UART);  
		HAL_UART_DMAStop(&DEBUG_UART);
		uint32_t rx_len =  REC_LEN - DEBUG_UART.hdmarx->Instance->NDTR;
		buf[rx_len]='\0';
		zykProcessData();
	}
	if(HAL_UART_Receive_DMA(&DEBUG_UART, (uint8_t *)buf, REC_LEN) != HAL_OK)
	{
		Error_Handler();
		printf( "CtrlUart error" );
	}
}

void ctrlUartInit(){
	if(HAL_UART_Receive_DMA(&DEBUG_UART,(uint8_t *)buf, REC_LEN) != HAL_OK){
		Error_Handler();
		printf( "InitCtrlUart error" );
	} 
}
extern MotorINFO* GimbalMotorGroup[2];
#define GMP (*GimbalMotorGroup[0])
#define GMY (*GimbalMotorGroup[1])
void zykProcessData()
{
    char data[15];
    //00 00 		00 		00
    //空 P=10Y=11  P=10S=11 P=01I=10D=11
    uint8_t Index=0;
    
    /////////// GM CONTROL ////////////////
    if(strncmp(buf,"U",1)==0)
    {
    	fw_printf("UP\r\n");
    	GMP.Target+=20;
    }
    else if(strncmp(buf,"D",1)==0)
    {
    	fw_printf("DOWN\r\n");
    	GMP.Target-=20;
    }
    else if(strncmp(buf,"SHOWYS",6)==0)
    {	//clolor order: R:target G:real B:adjust
    	float realSpeed2=-gyro_data.wz/(float)(32.8);
    	fw_printf("#DATA%.2f@%.2f@%.2f$",GMY.positionPID.output,realSpeed2,GMY.speedPID.output);
    }
    else if(strncmp(buf,"SHOWYP",6)==0)
    {	//clolor order: R:target G:real B:adjust
    	fw_printf("#DATA%.2f@%.2f@%.2f$",GMY.Target,GMY.Real,GMY.positionPID.output);
    }
    else if(strncmp(buf,"SHOWPS",6)==0)
    {	//clolor order: R:target G:real B:adjust
    	float realSpeed2=-gyro_data.wx/(float)(32.8);
    	fw_printf("#DATA%.2f@%.2f@%.2f$",GMP.positionPID.output,realSpeed2,GMP.speedPID.output);
    }
    else if(strncmp(buf,"SHOWPP",6)==0)
    {	//clolor order: R:target G:real B:adjust
    	fw_printf("#DATA%.2f@%.2f@%.2f$",GMP.Target,GMP.Real,GMP.positionPID.output);
    }
    else if(strncmp(buf,"L",1)==0)
    {
    	fw_printf("LEFT\r\n");
    	GMY.Target+=20;
    }
    else if(strncmp(buf,"R",1)==0)
    {
    	fw_printf("RIGHT\r\n");
    	GMY.Target-=20;
    }
    //GM PID yaw
    else if(strncmp(buf,"#",1)==0)
    {
    	switch(buf[3])
    	{
    		case'P':Index|=0x20;break;
    		case'Y':Index|=0x30;break;
    	}
    	switch(buf[4])
    	{
    		case'P':Index|=0x08;break;
    		case'S':Index|=0x0c;break;
    	}
    	switch(buf[5])
    	{
    		case'P':Index|=0x01;break;
    		case'I':Index|=0x02;break;
    		case'D':Index|=0x03;break;
    	}
    	int iii=0;
    	while(buf[6+iii]!='@'){
    		data[iii]=buf[6+iii];
    		iii++;
    	}
    	if(iii!=15)
    	{
    		float data_num=atof(data);
    		fw_printf("%c%c%c change to %f\r\n",buf[3],buf[4],buf[5],data_num);
    		switch(Index)
    		{
    			case 0x29:GMP.positionPID.kp = data_num;break;
    			case 0x2a:GMP.positionPID.ki = data_num;break;
    			case 0x2b:GMP.positionPID.kd = data_num;break;
    			case 0x2d:GMP.speedPID.kp = data_num;break;
    			case 0x2e:GMP.speedPID.ki = data_num;break;
    			case 0x2f:GMP.speedPID.kd = data_num;break;
    
    			case 0x39:GMY.positionPID.kp = data_num;break;
    			case 0x3a:GMY.positionPID.ki = data_num;break;
    			case 0x3b:GMY.positionPID.kd = data_num;break;
    			case 0x3d:GMY.speedPID.kp = data_num;break;
    			case 0x3e:GMY.speedPID.ki = data_num;break;
    			case 0x3f:GMY.speedPID.kd = data_num;break;
    		}
    	}
    }		
    strcpy(buf,"\0");
}


//--------------------数据解析协议部分-------------------//
/*uint8_t ComProtocal(char*rxbuf,char*head,char*end,char* separater,char dataout[15])
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
}*/




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
#endif
