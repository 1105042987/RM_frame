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
#define CLAWTIGHT HAL_GPIO_WritePin(GPIOH,1<<5,1)//צ��ץ��
#define CLAWLOOSE HAL_GPIO_WritePin(GPIOH,1<<5,0)//צ���ɿ�  //������ ���� ǰ�� ץ��  4 2 5
#define LAUNCH    HAL_GPIO_WritePin(GPIOH,1<<4,1)//�������
#define LAND      HAL_GPIO_WritePin(GPIOH,1<<4,0)//���������λ

#define UPLEVEL 440

extern uint32_t AutoGet_Start;  
extern uint32_t AutoGet_TotalStep;
extern uint32_t AutoGet_Alreadywaited;
extern uint32_t Claw_AlreadyRollOut;
extern uint32_t Claw_AlreadyWaited;
extern uint32_t Claw_AlreadyTight;
extern uint32_t Claw_UpToPosition;
extern uint32_t Claw_DownToPosition;
extern uint16_t Claw_TruePosition[5];
extern int32_t Claw_UpAngle;
extern uint32_t Claw_TakeThisBox;
extern uint32_t Claw_SelfInspecting;
extern uint32_t Claw_FindingNextBox_Lower;
extern uint32_t Claw_FindingNextBox_Upper;
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


extern int32_t auto_counter;		//����׼ȷ��ʱ�����ĳ�¼�
extern int32_t auto_waiter;
extern int32_t cnt_clk;
extern int32_t auto_wait;
extern int16_t cnt;
extern uint32_t ifset;//�����Լ�




uint8_t hasReach(MotorINFO* id, double distance);//�����жϵ���Ƿ�λ
uint8_t canMovePositive(MotorINFO* id, int16_t stuckMoment);
uint8_t canMoveNegetive(MotorINFO* id, int16_t stuckMoment);
void RefreshADC();     //ˢ�º��⴫������ֵ
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
void AutoGet_LowerANDThrow();
void AutoGet_Upper();
void Claw_GetSpecifiedBox();
void Claw_SelfInspect();
void Claw_GoToNextBox_lower();
void Claw_GoToNextBox_upper();
void Claw_Up();
void AutoGet_SwitchState();
void ClawUpDown_SwitchState();
void Claw_Protect();
void Claw_AutoIn();
void Claw_AutoInTest();
#endif /*__ AUTOGETTASK_H */