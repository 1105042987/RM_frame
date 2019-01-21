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


#define CLAWOUT   HAL_GPIO_WritePin(GPIOH,1<<2,1)//צ�ӵ���
#define CLAWIN    HAL_GPIO_WritePin(GPIOH,1<<2,0)//צ�ӽ���
#define CLAWTIGHT HAL_GPIO_WritePin(GPIOI,1<<5,1)//צ��ץ��
#define CLAWLOOSE HAL_GPIO_WritePin(GPIOI,1<<5,0)//צ���ɿ�
#define LAUNCH    HAL_GPIO_WritePin(GPIOH,1<<4,1)//�������
#define LAND      HAL_GPIO_WritePin(GPIOH,1<<4,0)//���������λ


extern uint32_t AutoGet_Start;  
extern uint32_t AutoGet_TotalStep;
extern uint32_t AutoGet_Alreadywaited;
extern uint32_t Claw_AlreadyRollOut;
extern uint32_t Claw_AlreadyWaited;
extern uint32_t Claw_AlreadyTight;
extern uint32_t Claw_UpToPosition;
extern uint16_t Claw_TruePosition[5];
extern int32_t Claw_UpAngle;
extern uint32_t Claw_TakeThisBox;
extern uint32_t Claw_SelfInspecting;
extern uint32_t Claw_FindingNextBox;
//�洢���⴫��������ֵ
extern uint32_t ADC_value[10];
extern uint32_t ADC2_value[10];
//�����ñ���
extern uint32_t Sensor_Tmp[2];
extern uint16_t Sensor_Count[2];
extern uint32_t Sensor_Ready[2];
extern uint32_t Sensor_a;
extern uint32_t Sensor_b;


extern int32_t auto_counter;		//����׼ȷ��ʱ�����ĳ�¼�
extern int32_t auto_waiter;
extern int32_t cnt_clk;
extern int32_t auto_wait;
extern int16_t cnt;
extern uint32_t ifset;//�����Լ�



uint32_t average(uint32_t a[]);//���ڼ�����⴫�ؾ������ݵ�ƽ��ֵ
uint8_t hasReach(MotorINFO* id, double distance);//�����жϵ���Ƿ�λ
void Sensor_Read_Lower();//���ڼ����⴫�����Ƿ��⵽������϶
void Claw_Rollout();//צ��ת����ת��
void Claw_Rollin();
void Claw_Tight();//צ��ץ�����ɿ�
void Claw_Loose();
void Box_Launch();//��ҩ�䵯��װ��
void Box_Land();
void Claw_GoTo(int a);//צ���ߵ���a�����ӵ�λ��
void Claw_GetaBox();
void Box_Fire();
void AutoGet_Stop_And_Clear();
void Box_ThrowForward();
void AutoGet_Lower();
void AutoGet_Upper();
void Claw_GetSpecifiedBox();
void Claw_SelfInspect();
void Claw_GoToNextBox_lower();
void Claw_Up();
void AutoGet_SwitchState();
#endif /*__ AUTOGETTASK_H */