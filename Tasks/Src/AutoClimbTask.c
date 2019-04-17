/**
  ******************************************************************************
  * File Name          : AutoGetTask.c
  * Description        : 自动上岛控制任务
  ******************************************************************************
  *
  * Copyright (c) 2019 Team JiaoLong-ShanghaiJiaoTong University
  * All rights reserved.
  *
  ******************************************************************************
  */
	#include "includes.h"
extern Distance_Couple_t distance_couple;
extern Engineer_State_e EngineerState;

extern int8_t Test_UD_Direction;
uint32_t adfl=0,adfr=0,adbl=0,adbr=0,addf=0,addb=0;
uint32_t disfl=0,disfr=0,disbl=0,disbr=0,disdf=0,disdb=0;
uint32_t AutoClimb_ComeToTop=0;
uint32_t AutoClimb_AlreadyTop=0;
uint8_t signal1=0;
uint8_t signal2=0;

uint8_t AlreadyClimbed=0;
uint8_t AlreadyDowned=0;

uint32_t AutoClimbing=0;
uint32_t AutoClimb_Level=0;  //用于指示现在车在第几层
//上 10000 8000
//disfl>2000 disfr>2000
//电机-8000 -6000
//disbr >3500 到1300下岛

void Chassis_Choose(uint8_t flag,uint8_t ensure)
{
	CM1.RealAngle=0;
	CM2.RealAngle=0;
	if(ChassisSpeedRef.forward_back_ref>0)
	{
	CM1.TargetAngle=ChassisSpeedRef.forward_back_ref*5;
	CM2.TargetAngle=-ChassisSpeedRef.forward_back_ref*5;
	}
	if(ChassisSpeedRef.forward_back_ref<0&&AlreadyDowned==0)
	{
	CM1.TargetAngle=ChassisSpeedRef.forward_back_ref*1.2;
	CM2.TargetAngle=-ChassisSpeedRef.forward_back_ref*1.2;
	}
	if(ChassisSpeedRef.forward_back_ref<0&&AlreadyDowned==1)
	{
	CM1.TargetAngle=ChassisSpeedRef.forward_back_ref*0.25;
	CM2.TargetAngle=-ChassisSpeedRef.forward_back_ref*0.25;
	}
	if(ChassisSpeedRef.forward_back_ref==0)
	{
		CM1.TargetAngle=0;
	  CM2.TargetAngle=0;
	}
	if(flag)
	{		
		if(ChassisSpeedRef.forward_back_ref>0)
		ChassisSpeedRef.forward_back_ref/=5;
		if(ChassisSpeedRef.forward_back_ref<0&&AlreadyDowned==0)
		ChassisSpeedRef.forward_back_ref/=10;
    if(ChassisSpeedRef.forward_back_ref<0&&AlreadyDowned==1)
		ChassisSpeedRef.forward_back_ref/=20;			
	}
	if(NMCDL.RealAngle<-950 )
	{//small chassis
		signal1=0;
		if((distance_couple.move_flags&0x0006)==0)      //下岛之后抬架子让车落地
		{
			if (ChassisSpeedRef.forward_back_ref < 0) {
					ChassisSpeedRef.forward_back_ref=0;
				  ChassisSpeedRef.rotate_ref=0;
					CM1.TargetAngle=0;
					CM2.TargetAngle=0;
					if(signal2==0){
							NMCDL.TargetAngle = UD_TOP;
						  NMCDR.TargetAngle = UD_TOP;
						signal2=1;
						if(AlreadyDowned==1)
						{
							AlreadyDowned=0;
							AutoClimb_Level--;
							State_Common();
						}
					}	
				}
		}
		if(flag==1)
		switch(distance_couple.move_flags&0x000f)                //上岛之后抬架子让车完全上岛
		{
			case 15:
				if(ChassisSpeedRef.forward_back_ref > 0){
					ChassisSpeedRef.forward_back_ref=0;
					ChassisSpeedRef.rotate_ref=0;
					CM1.TargetAngle=0;
					CM2.TargetAngle=0;
					if(signal2==0){
							NMCDL.TargetAngle = UD_TOP;
						  NMCDR.TargetAngle = UD_TOP;
						signal2=1;
						if(AlreadyClimbed==1)
						{
							AlreadyClimbed=0;
							AutoClimb_Level++;
							State_Common();
						}
					}
				}
				break;
			default:break;
		}
	}
	else if(NMCDL.RealAngle>-230)
	{// chassis
		signal2=0;
		if(flag==1)
		switch(distance_couple.move_flags&0xf)
		{
			case 6:                                                  //下岛时候降架子把车架住往后走
				if (ChassisSpeedRef.forward_back_ref < 0) { 
					//ChassisSpeedRef.forward_back_ref=0;
					CM1.TargetAngle=0;
					CM2.TargetAngle=0;
					switch((distance_couple.move_flags>>4)&0x3)
					{
						case 1://右转
							if(AlreadyDowned==0)
							{
							ChassisSpeedRef.rotate_ref=ChassisSpeedRef.forward_back_ref/8;
							ChassisSpeedRef.forward_back_ref=5;
							}
							break;
						case 2://左转
							if(AlreadyDowned==0)
							{
							ChassisSpeedRef.rotate_ref=ChassisSpeedRef.forward_back_ref/-8;
							ChassisSpeedRef.forward_back_ref=5;
							}
							break;
						case 0:
							ChassisSpeedRef.forward_back_ref=0;
						  ChassisSpeedRef.rotate_ref=0;
							if(signal1==0 && ensure==1)
							{
									NMCDL.TargetAngle = UD_BOTTOM;
								  NMCDR.TargetAngle = UD_BOTTOM;
								signal1=1;
								if(AlreadyDowned==0)
									AlreadyDowned=1;
							}break;
					}	
				}
				break;				
			case 15:                                             //上岛时候降架子把车抬起来
				if(ChassisSpeedRef.forward_back_ref>0){
					ChassisSpeedRef.forward_back_ref/=2;
					CM1.TargetAngle=0;
					CM2.TargetAngle=0;
					ChassisSpeedRef.rotate_ref=0;
					if(signal1==0 && ensure==1){
							NMCDL.TargetAngle = UD_BOTTOM;
						  NMCDR.TargetAngle = UD_BOTTOM;
						signal1=1;
						if(AlreadyClimbed==0)
						AlreadyClimbed=1;
					}
				}
				break;
			default:break;
		}
	}
	else{
		ChassisSpeedRef.forward_back_ref=0;
		CM1.TargetAngle=0;
		CM2.TargetAngle=0;
	}
}

void ComeToTop()
{
	if(AutoClimb_AlreadyTop==0)
		AutoClimb_ComeToTop=1;
	if(AutoClimb_ComeToTop==1)
	{
		if(NMCDL.RxMsgC6x0.moment<5000)
		{	
			NMCDL.TargetAngle+=2;
		}
		if(NMCDR.RxMsgC6x0.moment<5000)
		{
			NMCDR.TargetAngle+=2;
		}
		if(NMCDL.RxMsgC6x0.moment>5000&&NMCDR.RxMsgC6x0.moment>5000)
		{
			AutoClimb_AlreadyTop=1;
			AutoClimb_ComeToTop=0;
			NMCDL.RealAngle=0;
			NMCDR.RealAngle=0;
		}
	}
		if(NMCDL.RxMsgC6x0.moment>7000)//保护
			NMCDL.TargetAngle-=10;
		if(NMCDR.RxMsgC6x0.moment>7000)
			NMCDR.TargetAngle-=5;
	
}

void AutoClimb_SwitchState()
{
	if(AutoClimbing==1)
		Chassis_Choose(1,1);
}

void State_AutoClimb()
{
	EngineerState=CLIMB_STATE;
	AutoClimbing=1;
}

