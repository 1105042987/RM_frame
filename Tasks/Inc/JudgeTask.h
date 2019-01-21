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

typedef __packed struct
{
uint16_t stageRemianTime;
uint8_t gameProgress;
uint8_t robotLevel;
uint16_t remainHP;
uint16_t maxHP;
}extGameRobotState_t;
typedef __packed struct
{
uint8_t armorType : 4;
uint8_t hurtType : 4;
}extRobotHurt_t;
typedef __packed struct
{
uint8_t bulletType;
uint8_t bulletFreq;
float bulletSpeed;
}extShootData_t;
typedef __packed struct
{
float chassisVolt;
float chassisCurrent;
float chassisPower;
float chassisPowerBuffer;
uint16_t shooterHeat0;
uint16_t shooterHeat1;
}extPowerHeatData_t;
typedef __packed struct
{
uint8_t cardType;
uint8_t cardIdx;
}extRfidDetect_t;
typedef __packed struct
{
uint8_t winner;
}extGameResult_t;
typedef __packed struct
{
uint8_t buffType;
uint8_t buffAddition;
} extGetBuff_t;
typedef __packed struct
{
float x;
float y;
float z;
float yaw;
}extGameRobotPos_t;
typedef __packed struct
{
float data1;
float data2;
float data3;
uint8_t mask;
}extShowData_t;


typedef enum
{
	ONLINE = 0,
	OFFLINE = 1,
}JudgeState_e;

void judgeUartRxCpltCallback(void);
void Send_User_Data(void);
void InitJudgeUart(void);
void Judge_Refresh_Power(void);
void Judge_Refresh_State(void);
void Judge_Refresh_Position(void);
void Judge_Refresh_Shoot(void);
void Judge_Refresh_Hit(void);
void Judge_Refresh_Interact(void);
void Judge_Refresh_Result(void);
void Judge_Refresh_Buff(void);
extern void getJudgeState(void);
extern extGameRobotState_t RobotState;
extern extPowerHeatData_t PowerHeatData;
extern extShootData_t ShootData0;
extern extShootData_t ShootData1;
extern JudgeState_e JUDGE_State;
extern extShowData_t user_data;
extern float fakeHeat0;
extern float realBulletSpeed0;
extern uint16_t realHeat0;
extern uint8_t syncCnt0;
extern float cooldown0;
extern uint16_t maxHeat0;

#endif /*__ JUDGETASK_H */
