/**
  ******************************************************************************
  * File Name          : AutoGetTask.h
  * Description        : �Զ�ȡ����������ͷ�ļ�
  ******************************************************************************
  *
  * Copyright (c) 2019 Team JiaoLong-ShanghaiJiaoTong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#include "includes.h"
#ifndef __AUTOGETTASK_H
#define __AUTOGETTASK_H


#define CLAWOUT   HAL_GPIO_WritePin(GPIOH,1<<2,GPIO_PIN_SET)//צ�ӵ���
#define CLAWIN    HAL_GPIO_WritePin(GPIOH,1<<2,GPIO_PIN_RESET)//צ�ӽ���
#define CLAWTIGHT HAL_GPIO_WritePin(GPIOH,1<<5,GPIO_PIN_SET)//צ��ץ��
#define CLAWLOOSE HAL_GPIO_WritePin(GPIOH,1<<5,GPIO_PIN_RESET)//צ���ɿ�  //������ ���� ǰ�� ץ��  4 2 5
#define LAUNCH    HAL_GPIO_WritePin(GPIOH,1<<4,GPIO_PIN_SET)//�������
#define LAND      HAL_GPIO_WritePin(GPIOH,1<<4,GPIO_PIN_RESET)//���������λ

#define CLAW_IS_UP NMUDL.RealAngle<=-(UPLEVEL-30)
#define CLAW_IS_DOWN NMUDL.RealAngle>=-50

#define UPLEVEL 440

#define CLAW_INSPECT_SUCCEED Claw_SelfInspecting==3
extern uint32_t AutoGet_Start;  
extern uint32_t AutoGet_TotalStep;
extern uint32_t AutoGet_Alreadywaited;
extern uint32_t AutoGet_Error;
extern uint32_t Claw_AlreadyRollOut;
extern uint32_t Claw_AlreadyWaited;
extern uint32_t Claw_AlreadyTight;
extern uint32_t Claw_UpToPosition;
extern uint32_t Claw_DownToPosition;
extern uint16_t Claw_TruePosition[5];
extern int32_t Claw_UpAngle;
extern int Claw_TakeThisBox;
extern uint32_t Claw_SelfInspecting;
extern uint32_t Claw_FindingNextBox_Lower_Forward;
extern uint32_t Claw_FindingNextBox_Lower_Backward;
extern uint32_t Claw_FindingNextBox_Upper_Forward;
extern uint32_t Claw_FindingNextBox_Upper_Backward;
extern uint8_t CM_AutoRotate90;
//�洢���⴫��������ֵ
extern uint32_t ADC_value[160];
extern uint32_t ADC2_value[10];
extern uint32_t adfl,adfr,adbl,adbr,addf,addb;
extern uint32_t adgl,adgr;
extern uint32_t disfl,disfr,disbl,disbr,disdf,disdb;
extern uint32_t disgl,disgr;
//�����ñ���
extern uint32_t Sensor_Tmp[2];
extern uint16_t Sensor_Count[2];
extern uint32_t Sensor_Ready[2];
extern uint32_t Sensor_a;
extern uint32_t Sensor_b;
extern uint32_t Sensor_LongPush;
extern uint32_t Sensor_Lock;


extern int32_t auto_counter;		//����׼ȷ��ʱ�����ĳ�¼�
extern int32_t auto_waiter;
extern int32_t cnt_clk;
extern int32_t auto_wait;
extern int16_t cnt;
extern uint32_t ifset;//�����Լ�

extern uint32_t Yaw_Reset_Flag;
extern uint32_t Yaw_Reset_Cnt;
extern uint32_t Yaw_Set_Flag;
extern uint32_t Yaw_Set_Cnt;


uint8_t hasReach(MotorINFO* id, double distance);//�����жϵ���Ƿ�λ
uint8_t canMovePositive(MotorINFO* id, int16_t stuckMoment);
uint8_t canMoveNegetive(MotorINFO* id, int16_t stuckMoment);
void RefreshADC(void);     //ˢ�º��⴫������ֵ
void Sensor_Read_Lower(void);//���ڼ����⴫�����Ƿ��⵽������϶
void Claw_Rollout(void);//צ��ת����ת��
void Claw_Rollin(void);
void Claw_Tight(void);//צ��ץ�����ɿ�
void Claw_Loose(void);
void Box_Launch(void);//��ҩ�䵯��װ��
void Box_Land(void);
void Claw_GoTo(int a);//צ���ߵ���a�����ӵ�λ��
void Claw_GetaBox(void);
void Box_Fire(void);
void AutoGet_Stop_And_Clear(void);
void Box_ThrowForward(void);
void AutoGet_Lower(void);
void AutoGet_LowerANDThrow(void);
void AutoGet_Upper(void);
void Claw_GetSpecifiedBox(void);
void Claw_SelfInspect(void);
void Claw_GoToNextBox_lower(void);
void Claw_GoToNextBox_upper(void);
void AutoGet_SensorControl(void);
void Claw_Up(void);
void AutoGet_SwitchState(void);
void ClawUpDown_SwitchState(void);
void Claw_Protect(void);
void Claw_AutoIn(void);
void Claw_AutoInTest(void);
void State_AutoGet(void);
void State_Common(void);
void Yaw_Check(void);
void Rotate_Check(void);
void AutoGet_AutoDown(void);
void Claw_AutoBack(void);
void AutoGet_Enqueue(int);
void Claw_Wait(void);
void AutoGet_FillQueue(void);
#endif /*__ AUTOGETTASK_H */
