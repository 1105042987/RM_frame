/**
  ******************************************************************************
  * File Name          : JudgeTask.c
  * Description        : 裁判系统处理任务，得到裁判系统信息
  ******************************************************************************
  *
  * Copyright (c) 2019 Team JiaoLong-ShanghaiJiaoTong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#include "includes.h"
#include "offical_Judge_Handler.h"
#include <stdlib.h>
#include <string.h>

uint8_t tmp_judge;
void InitJudgeUart(void){
	tx_free = 1;
	Send_User_Data();
	if(HAL_UART_Receive_DMA(&JUDGE_UART, &tmp_judge, 1) != HAL_OK){
			Error_Handler();
	}
}
uint8_t receiving = 0;
uint8_t received = 0;
uint8_t buffer[80] = {0}; 
uint8_t buffercnt = 0;
uint16_t cmdID;

void judgeUartRxCpltCallback(void)
{
//	fw_printfln("judge receive");
		if(receiving) 
		{
			if(buffercnt >40)buffercnt = 4;
			buffer[buffercnt] = tmp_judge;
			buffercnt++;
			
			if(buffercnt == 5)
			{
				if (myVerify_CRC8_Check_Sum(buffer, 5)==0) 
				{
					receiving = 0;
					buffercnt = 0;
				}
			}
			if(buffercnt == 7) cmdID = (0x0000 | buffer[5]) | (buffer[6] << 8);
			
			if(buffercnt == 29 && cmdID == 0x0004)
			{
				if (myVerify_CRC16_Check_Sum(buffer, 29)) 
				{
					Judge_Refresh_Power();
				}
				receiving = 0;
				buffercnt = 0;
			}
			if(buffercnt == 17 && cmdID == 0x0001)
			{
				if (myVerify_CRC16_Check_Sum(buffer, 17)) 
				{
					Judge_Refresh_State();
				}
				receiving = 0;
				buffercnt = 0;
			}
			if(buffercnt == 15 && cmdID == 0x0003)
			{
				if (myVerify_CRC16_Check_Sum(buffer, 15)) 
				{
					Judge_Refresh_Shoot();
				}
				receiving = 0;
				buffercnt = 0;
			}

		
			if(buffercnt == 25 && cmdID == 0x0008)
			{
				if (myVerify_CRC16_Check_Sum(buffer, 25)) 
				{
					Judge_Refresh_Position();
				}
				receiving = 0;
				buffercnt = 0;
			}
			if(buffercnt == 10 && cmdID == 0x0002)
			{
				if (myVerify_CRC16_Check_Sum(buffer, 10)) 
				{
					Judge_Refresh_Hit();
				}
				receiving = 0;
				buffercnt = 0;
			}
			if(buffercnt == 11 && cmdID == 0x0005)
			{
				if (myVerify_CRC16_Check_Sum(buffer, 11)) 
				{
					Judge_Refresh_Interact();
				}
				receiving = 0;
				buffercnt = 0;
			}
			if(buffercnt == 10 && cmdID == 0x0006)
			{
				if (myVerify_CRC16_Check_Sum(buffer, 10)) 
				{
					Judge_Refresh_Result();
				}
				receiving = 0;
				buffercnt = 0;
			}
			if(buffercnt == 11 && cmdID == 0x0007)
			{
				if (myVerify_CRC16_Check_Sum(buffer, 11)) 
				{
					Judge_Refresh_Buff();
				}
				receiving = 0;
				buffercnt = 0;
			}
		}	
		else 
		{
			if(tmp_judge == 0xA5)
			{
				receiving = 1;
				buffercnt = 0;
				buffer[0] = tmp_judge;
				buffercnt++;
			}
		}
		if(HAL_UART_Receive_DMA(&JUDGE_UART, &tmp_judge, 1) != HAL_OK)
		{
			Error_Handler();
	  }
}


uint8_t JUDGE_Received = 0;
JudgeState_e JUDGE_State = OFFLINE;

extGameRobotState_t RobotState;
extPowerHeatData_t PowerHeatData;
extShootData_t ShootData0;
extShootData_t ShootData1;

uint16_t maxHP = 1500;
uint16_t remainHP;
uint16_t maxHeat0 = 480;
uint16_t remainHeat0 = 480;
uint16_t maxHeat1 = 480;
uint16_t remainHeat1 = 480;
uint16_t realHeat0 = 0;
float fakeHeat0 = 0;
float realBulletSpeed0 = 22;
float cooldown0 = 72;
uint8_t shoot0Cnt = 0;
uint8_t shoot1Cnt = 0;
uint8_t syncCnt0 = 0;

void Judge_Refresh_Power()
{
	//printf("verify OK\r\n");

	unsigned char * bs = (unsigned char*)&PowerHeatData.chassisPower;
	for(int i = 0; i<4; i++){
		bs[i] = (unsigned char)buffer[i+15];
	}
	
	bs = (unsigned char*)&PowerHeatData.chassisPowerBuffer;
	for(int i = 0; i<4; i++){
		bs[i] = (unsigned char)buffer[i+19];
	}
	
	bs = (unsigned char*)&PowerHeatData.shooterHeat0;
	for(int i = 0; i<2; i++){
		bs[i] = (unsigned char)buffer[i+23];
	}
	
	bs = (unsigned char*)&PowerHeatData.shooterHeat1;
	for(int i = 0; i<2; i++){
		bs[i] = (unsigned char)buffer[i+25];
	}

	realHeat0 = PowerHeatData.shooterHeat0;
	remainHeat0 = maxHeat0 - PowerHeatData.shooterHeat0;
	remainHeat1 = maxHeat1 - PowerHeatData.shooterHeat1;
	JUDGE_Received = 1;
	
}

void Judge_Refresh_State()
{
	unsigned char * bss = (unsigned char*)&RobotState.robotLevel;
	char cs[1] = {buffer[10]};
	bss[0] = (unsigned char)cs[0];
	unsigned char * bss1 = (unsigned char*)&RobotState.maxHP;
	char cs1[2] = {buffer[13],buffer[14]};
	for(int i = 0; i<2; i++){
		bss1[i] = (unsigned char)cs1[i];
	}
	unsigned char * bss2 = (unsigned char*)&RobotState.remainHP;
	char cs2[2] = {buffer[11],buffer[12]};
	for(int i = 0; i<2; i++){
		bss2[i] = (unsigned char)cs2[i];
	}

	maxHP = RobotState.maxHP;
	remainHP = RobotState.remainHP;
	switch(maxHP)
	{
		case 1000:{maxHeat0 = 120;cooldown0 = 18;}break;
		case 1250:{maxHeat0 = 240;cooldown0 = 36;}break;
		case 1500:{maxHeat0 = 480;cooldown0 = 72;}break;
		default:{maxHeat0 = 480;cooldown0 = 72;}break;
	}
	JUDGE_Received = 1;
}

void Judge_Refresh_Shoot()
{
	unsigned char * bsd;
	if(buffer[7]==1)
	{
		bsd = (unsigned char*)&ShootData0.bulletSpeed;
		shoot0Cnt++;
	}
	else if(buffer[7]==2)
	{
		bsd = (unsigned char*)&ShootData1.bulletSpeed;
		shoot1Cnt++;
	}
	else return;
	
	char cs[4] = {buffer[9],buffer[10],buffer[11],buffer[12]};
	for(int i = 0; i<4; i++){
		bsd[i] = (unsigned char)cs[i];
	}
	
	if(buffer[7]==1)realBulletSpeed0 = ShootData0.bulletSpeed;
	JUDGE_Received = 1;
}

void Judge_Refresh_Position()
{
	JUDGE_Received = 1;
}

void Judge_Refresh_Hit()
{
	JUDGE_Received = 1;
}

void Judge_Refresh_Interact()
{
	JUDGE_Received = 1;
}

void Judge_Refresh_Result()
{
	JUDGE_Received = 1;
}

void Judge_Refresh_Buff()
{
	JUDGE_Received = 1;
}

void getJudgeState(void)
{
	static int s_count_judge = 0;
	if(JUDGE_Received==1)
	{
		s_count_judge = 0;
		JUDGE_State = ONLINE;
		JUDGE_Received = 0;
	}
	else
	{
		s_count_judge++;
		if(s_count_judge > 150)
		{//300ms
			JUDGE_State = OFFLINE;
		}
	}
}
extShowData_t user_data;

void Send_User_Data()
{
	uint8_t buffer[22]={0};
	/*test
	user_data.data1 = tmpx;
	user_data.data2 += 0.2f;
	user_data.data3 += 0.3f;
	user_data.mask = 0xC0;
	*/
	unsigned char * bs1 = (unsigned char*)&user_data.data1;
	unsigned char * bs2 = (unsigned char*)&user_data.data2;
	unsigned char * bs3 = (unsigned char*)&user_data.data3;
	buffer[0] = 0xA5;
	buffer[1]  = 13;
	buffer[3] = 1;
	buffer[4] = myGet_CRC8_Check_Sum(&buffer[0], 5-1, myCRC8_INIT);
	buffer[5] = 0x00;
	buffer[6] = 0x01;
	for(int i=7;i<11;i++) buffer[i] = bs1[i-7];
	for(int i=11;i<15;i++) buffer[i] = bs2[i-11];
	for(int i=15;i<19;i++) buffer[i] = bs3[i-15];
	buffer[19] = user_data.mask;
	static uint16_t CRC16=0;
	CRC16 = myGet_CRC16_Check_Sum(buffer, 20, myCRC16_INIT);
	buffer[20] = CRC16 & 0xff;
	buffer[21] = (CRC16 >> 8) & 0xff;

	tx_free = 0;
	while(HAL_UART_Transmit_DMA(&JUDGE_UART,(uint8_t *)&buffer,22)!=HAL_OK);
}
