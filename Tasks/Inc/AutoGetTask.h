/**
  ******************************************************************************
  * File Name          : AutoGetTask.h
  * Description        : 自动取弹控制任务头文件
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


#define CLAWOUT   HAL_GPIO_WritePin(GPIOH,1<<2,1)//爪子弹出
#define CLAWIN    HAL_GPIO_WritePin(GPIOH,1<<2,0)//爪子进入
#define CLAWTIGHT HAL_GPIO_WritePin(GPIOH,1<<5,1)//爪子抓紧
#define CLAWLOOSE HAL_GPIO_WritePin(GPIOH,1<<5,0)//爪子松开  //左中右 弹箱 前后 抓紧  4 2 5
#define LAUNCH    HAL_GPIO_WritePin(GPIOH,1<<4,1)//弹射起飞
#define LAND      HAL_GPIO_WritePin(GPIOH,1<<4,0)//弹射机构归位

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
extern uint32_t Claw_FindingNextBox_Lower_Forward;
extern uint32_t Claw_FindingNextBox_Lower_Backward;
extern uint32_t Claw_FindingNextBox_Upper_Forward;
extern uint32_t Claw_FindingNextBox_Upper_Backward;
extern uint8_t CM_AutoRotate90;
//存储红外传感器的数值
extern uint32_t ADC_value[160];
extern uint32_t ADC2_value[10];
extern uint32_t adfl,adfr,adbl,adbr,addf,addb;
extern uint32_t adgl,adgr;
extern uint32_t disfl,disfr,disbl,disbr,disdf,disdb;
extern uint32_t disgl,disgr;
//消抖用变量
extern uint32_t Sensor_Tmp[2];
extern uint16_t Sensor_Count[2];
extern uint32_t Sensor_Ready[2];
extern uint32_t Sensor_a;
extern uint32_t Sensor_b;
extern uint32_t Sensor_LongPush;
extern uint32_t Sensor_Lock;


extern int32_t auto_counter;		//用于准确延时的完成某事件
extern int32_t auto_waiter;
extern int32_t cnt_clk;
extern int32_t auto_wait;
extern int16_t cnt;
extern uint32_t ifset;//用于自检




uint8_t hasReach(MotorINFO* id, double distance);//用于判断电机是否到位
uint8_t canMovePositive(MotorINFO* id, int16_t stuckMoment);
uint8_t canMoveNegetive(MotorINFO* id, int16_t stuckMoment);
void RefreshADC();     //刷新红外传感器数值
void Sensor_Read_Lower();//用于检测红外传感器是否检测到两个空隙
void Claw_Rollout();//爪子转出与转回
void Claw_Rollin();
void Claw_Tight();//爪子抓紧与松开
void Claw_Loose();
void Box_Launch();//弹药箱弹射装置
void Box_Land();
void Claw_GoTo(int a);//爪子走到第a个箱子的位置
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
void AutoGet_SensorControl();
void Claw_Up();
void AutoGet_SwitchState();
void ClawUpDown_SwitchState();
void Claw_Protect();
void Claw_AutoIn();
void Claw_AutoInTest();
void State_AutoGet();
void State_Common();
void Yaw_Check();
void Rotate_Check();
#endif /*__ AUTOGETTASK_H */
