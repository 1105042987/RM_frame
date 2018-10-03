/**
  ******************************************************************************
  * File Name          : JudgeTask.h
  * Description        : 裁判系统处理任务，得到裁判系统信息
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#ifndef __JUDGETASK_H
#define __JUDGETASK_H

#include "includes.h"

typedef struct 
{
    uint32_t remainTime;
    uint16_t remainLifeValue;
    float realChassisOutV;
    float realChassisOutA;
    float remainPower;
}tGameInfo;

typedef struct
{
	uint16_t stageRemianTime;
	uint8_t gameProgress;
	uint8_t robotLevel;
	uint16_t remainHP;	
	uint16_t maxHP;
}extGameRobotState_t;

typedef struct
{
	uint8_t bulletType;
	uint8_t bulletFreq;
	float bulletSpeed;
}extShootData_t;

typedef struct
{
	float chassisVolt;
	float chassisCurrent;
	float chassisPower;
	float chassisPowerBuffer;
	uint16_t shooterHeat0;
	uint16_t shooterHeat1;
}extPowerHeatData_t;

typedef struct 
{
	float data1;
	float data2;
	float data3;
	uint8_t mask;
}tUserData;

typedef struct 
{
	uint8_t sof;
	uint16_t data_length;
	uint8_t seq;
	uint8_t crc8;
}frame_header_t;

typedef struct
{
	frame_header_t head;
	uint16_t cmdID;
	tUserData data;
	uint16_t CRC16;
}SendData_t;

typedef enum
{
	ONLINE,
	OFFLINE
}JudgeState_e;

void judgeUartRxCpltCallback(void);
void InitJudgeUart(void);
void Judge_Refresh(void);
void Judge_Refresh_ShootData(void);
void Judge_Refresh_Power(void);
void Judge_Refresh_State(void);
void Judge_Refresh_Position(void);

#endif /*__ JUDGETASK_H */
