/**
  ******************************************************************************
  * File Name          : AutoGetTask.c
  * Description        : �Զ�ȡ����������
  ******************************************************************************
  *
  * Copyright (c) 2019 Team JiaoLong-ShanghaiJiaoTong University
  * All rights reserved.
  *
  ******************************************************************************
  */
	#include "includes.h"
	
uint32_t AutoGet_Start=0;  
uint32_t AutoGet_TotalStep=1;
uint32_t AutoGet_Alreadywaited=0;
uint32_t Claw_AlreadyRollOut=0;
uint32_t Claw_AlreadyWaited=0;
uint32_t Claw_AlreadyTight=0;
uint32_t Claw_UpToPosition=0;
uint16_t Claw_TruePosition[5] = {0, 820, 1600, 400, 1200};
int32_t  Claw_UpAngle=0;
uint32_t Claw_TakeThisBox=0;
uint32_t Claw_SelfInspecting=0;
uint32_t Claw_FindingNextBox=0;
//�洢���⴫��������ֵ
extern uint32_t ADC_value[10];
extern uint32_t ADC2_value[10];
//�����ñ���
uint32_t Sensor_Tmp[2];
uint16_t Sensor_Count[2];
uint32_t Sensor_Ready[2]={0,0};
uint32_t Sensor_a;
uint32_t Sensor_b;


int32_t auto_counter=0;		//����׼ȷ��ʱ�����ĳ�¼�
int32_t auto_waiter=0;
int32_t cnt_clk;
int32_t auto_wait=0;
int16_t cnt = 0;
uint32_t ifset=-5;//�����Լ�

uint32_t average(uint32_t a[])//���ڼ�����⴫�ؾ������ݵ�ƽ��ֵ
{
	uint32_t ave;
	uint32_t sum=0;
	for(int j=0;j<sizeof(a);j++)
	{
		sum+=a[j];
	}
	ave=sum/sizeof(a);
	
	return ave;
}
uint8_t hasReach(MotorINFO* id, double distance)//�����жϵ���Ƿ�λ
{
	if(fabs(id->RealAngle - id->TargetAngle) < distance)return 1;
	else return 0;
}
void Sensor_Read_Lower()//���ڼ����⴫�����Ƿ��⵽������϶
{
	if(Sensor_Tmp[0]<2000&&average(ADC_value)<2000)
		Sensor_Count[0]=1;
	else
	{Sensor_Tmp[0]=average(ADC_value);Sensor_Count[0]=0;}
	if(Sensor_Count[0]==1&&Sensor_Ready[0]==0)
		Sensor_Ready[0]=1;
	if(Sensor_Tmp[1]<2000&&average(ADC2_value)<2000)
		Sensor_Count[1]=1;
	else
	{Sensor_Tmp[1]=average(ADC2_value);Sensor_Count[1]=0;}
	if(Sensor_Count[1]==1&&Sensor_Ready[0]==1)
		Sensor_Ready[0]=2;
		
}
void Claw_Rollout()//צ��ת����ת��
{
	if(auto_counter==0&&Claw_AlreadyRollOut==0)
	{
		UM1.TargetAngle=185;
		UM2.TargetAngle=-185;
	}
	if((hasReach(&UM1, 10) || hasReach(&UM2, 10))&&Claw_AlreadyRollOut==0&&UM1.TargetAngle==185)
	{Claw_AlreadyRollOut=1;}
}

void Claw_Rollin()
{
	if(auto_counter==0&&Claw_AlreadyRollOut==1)
	{
		UM1.TargetAngle=34;
    UM2.TargetAngle=-34;
		Claw_AlreadyRollOut=2;
	}
	if((hasReach(&UM1, 5) || hasReach(&UM2, 5))&&Claw_AlreadyRollOut==2)
	{
		AutoGet_TotalStep++;
		Claw_AlreadyRollOut=0;
		Claw_AlreadyTight=0;
		//��ɾ������� ���Զ����ν� auto_counter=500;
	}
}

void Claw_Tight()//צ��ץ�����ɿ�
{
	if(Claw_AlreadyRollOut==1&&Claw_AlreadyTight==0)
	{
		HAL_GPIO_WritePin(GPIOI,1<<5,1);
		auto_counter=300;
		Claw_AlreadyTight=1;
	}
}

void Claw_Loose()
{
	if(auto_counter==0&&auto_waiter==0)
	   {
	HAL_GPIO_WritePin(GPIOI,1<<5,0);
	if(Claw_AlreadyWaited==0)
	{
		auto_wait=100;
		Claw_AlreadyWaited=1;
	}
     }
}
void Box_launch()//��ҩ�䵯��װ��
{
	if(auto_wait==0&&Claw_AlreadyWaited==1)
	{
	  HAL_GPIO_WritePin(GPIOH,1<<4,1);
	  auto_wait=300;
		Claw_AlreadyWaited=2;
		AutoGet_TotalStep++;//�¼ӵ�
	}
}

void Box_Land()
{
	if(auto_wait==0&&Claw_AlreadyWaited==2)
	{
	  HAL_GPIO_WritePin(GPIOH,1<<4,0);
		//AutoGet_TotalStep++;//��ɾ����
		Claw_AlreadyWaited=0;
	}
}

void Claw_GoTo(int a)//צ���ߵ���a�����ӵ�λ��
{
	if(auto_counter==0)
	{
	switch(a)
	{
		case 1:{UFM.TargetAngle=0;
		        if(hasReach(&UFM,15))
		           AutoGet_TotalStep++;
		        break;}
		case 2:{UFM.TargetAngle=820;
		        if(hasReach(&UFM,5))
		           AutoGet_TotalStep++;
		        break;}
		case 3:{UFM.TargetAngle=1570;
		        if(hasReach(&UFM,20))
		           AutoGet_TotalStep++;
		        break;}
		case 4:{UFM.TargetAngle=1200;
		        if(hasReach(&UFM,5))
		           AutoGet_TotalStep++;
		        break;}
		case 5:{UFM.TargetAngle=400;
		        if(hasReach(&UFM,5))
		           AutoGet_TotalStep++;
		        break;}
	}
  }
}
void Claw_GetaBox()//ȡһ�����ӵ��������
{
	Claw_Rollout();
	Claw_Tight();
	Claw_Rollin();
}

void Box_Fire()//����һ�����ӵ���������
{
	Claw_Loose();
	Box_launch();
	//Box_Land();//ɾ����
}
void AutoGet_Stop_And_Clear()//״̬���� צ��ת�� ���Ƶ��ͣת�������쳣״�������һ��ȡ��������
{
	AutoGet_Start=0;
	AutoGet_TotalStep=1;
	AutoGet_Alreadywaited=0;
	UM1.TargetAngle=0;
	UM2.TargetAngle=0;
	Claw_AlreadyRollOut=0;
	Claw_AlreadyTight=0;
	CLAWLOOSE;
	Claw_TakeThisBox=0;
	Claw_SelfInspecting=0;
	Claw_FindingNextBox=0;
	Claw_UpToPosition=0;
	Claw_UpAngle=0;
	Sensor_Ready[0]=0;
}
void Box_ThrowForward()//��ǰ�ӳ�����
{ 
	if(auto_counter==0&&auto_waiter==0){
	UM1.TargetAngle=180;
	UM2.TargetAngle=-180;
	if(fabs(UM1.RealAngle-60)<=10||fabs(UM2.RealAngle-(-60))<=10)
	{
		CLAWLOOSE;
	}
	if(hasReach(&UM1,5)||hasReach(&UM2,5))
	{
		AutoGet_TotalStep++;
	}
                     }
}
void AutoGet_Lower()//�Զ�ȡ���������������
{
	switch(AutoGet_TotalStep)
	{
		case 1:{Claw_GoTo(1);break;}
		case 2:{Claw_GetaBox();  break;}
		case 3:{Claw_GoTo(2);break;}
		case 4:{Box_Fire();     break;}
		case 5:{Claw_GetaBox();  break;}
		case 6:{Claw_GoTo(3);break;}
		case 7:{Box_Fire();     break;}
		case 8:{Claw_GetaBox();  break;}
		case 9:{Claw_GoTo(4);break;}
		case 10:{Box_Fire();    break;}
		case 11:{CLAWOUT;AutoGet_TotalStep++;auto_counter=500;break;}  
		case 12:{Claw_GetaBox(); break;}
		case 13:{Claw_GoTo(5);break;}
		case 14:{Box_Fire();    break;}
		case 15:{Claw_GetaBox(); break;}
		case 16:{Claw_GoTo(1);break;}
		case 17:{Box_Fire();    break;}
		case 18:{CLAWIN;AutoGet_TotalStep++;break;}
		default:{AutoGet_Stop_And_Clear();}
	}
}
void AutoGet_Upper()//�Զ�ȡ����������������
{
	switch(AutoGet_TotalStep)
	{
		case 1:{//CLAWOUT;
		AutoGet_TotalStep++;break;}
		case 2:{Claw_GoTo(1);break;}
		case 3:{Claw_GetaBox();  break;}
		case 4:{Claw_GoTo(2);break;}
		case 5:{Box_ThrowForward();     break;}
		case 6:{Claw_GetaBox();  break;}
		case 7:{Claw_GoTo(3);break;}
		case 8:{Box_ThrowForward();     break;}
		case 9:{Claw_GetaBox();  break;}
		case 10:{Claw_GoTo(1);break;}
		case 11:{Box_ThrowForward();    break;}
		case 12:{UM1.TargetAngle=0;UM2.TargetAngle=0;AutoGet_TotalStep++;break;}
		case 13:{//CLAWIN;
		AutoGet_TotalStep++;break;}
		default:{AutoGet_Stop_And_Clear();break;}
	}
}
void Claw_GetSpecifiedBox()//�������ȡ����λ�õ�
{
	switch(Claw_TakeThisBox)
	{
		case 1:{
			switch(AutoGet_TotalStep)
			{
				case 1:{Claw_GoTo(1);break;}
				case 2:{
					  Claw_GetaBox(); 
					if(AutoGet_Alreadywaited==0)
					{auto_waiter=2000;AutoGet_Alreadywaited=1;}  
					  break;}
				case 3:{Box_ThrowForward();     break;}
				default:{AutoGet_Stop_And_Clear();   break;}
			}
		}break;
		case 2:{
			switch(AutoGet_TotalStep)
			{
				case 1:{Claw_GoTo(2);break;}
				case 2:{
					  Claw_GetaBox(); 
					if(AutoGet_Alreadywaited==0)
					{auto_waiter=2000;AutoGet_Alreadywaited=1;}  
					  break;}
				case 3:{Box_Fire();     break;}
				default:{AutoGet_Stop_And_Clear();   break;}
			}
		}break;
		case 3:{
			switch(AutoGet_TotalStep)
			{
				case 1:{Claw_GoTo(3);break;}
				case 2:{
					  Claw_GetaBox(); 
					if(AutoGet_Alreadywaited==0)
					{auto_waiter=2000;AutoGet_Alreadywaited=1;}  
					  break;}
				case 3:{Box_Fire();     break;}
				default:{AutoGet_Stop_And_Clear();   break;}
			}
		}break;
		case 4:{
			switch(AutoGet_TotalStep)
			{
				case 1:{Claw_GoTo(4);break;}
				case 2:{
					  Claw_GetaBox(); 
					if(AutoGet_Alreadywaited==0)
					{auto_waiter=2000;AutoGet_Alreadywaited=1;}  
					  break;}
				case 3:{Box_Fire();     break;}
				default:{AutoGet_Stop_And_Clear();   break;}
			}
		}break;
		case 5:{
			switch(AutoGet_TotalStep)
			{
				case 1:{Claw_GoTo(5);break;}
				case 2:{
					  Claw_GetaBox(); 
					if(AutoGet_Alreadywaited==0)
					{auto_waiter=2000;AutoGet_Alreadywaited=1;}  
					  break;}
				case 3:{Box_Fire();     break;}
				default:{AutoGet_Stop_And_Clear();   break;}
			}
		}break;
		default:break;
	}
}
void Claw_SelfInspect()//צ�Ӻ����Զ���λ���
{
	if(UFM.RxMsgC6x0.moment>-4000&&NMUDL.RealAngle>500&&Claw_SelfInspecting==1)
		UFM.TargetAngle-=8;
	if(UFM.RxMsgC6x0.moment<-4000&&Claw_SelfInspecting==1)
	{
		UFM.RealAngle=0;
		UFM.TargetAngle=0;
		Claw_SelfInspecting=0;
		Claw_UpToPosition=0;
	}
}
void Claw_GoToNextBox_lower()//���⴫��������צ�ӵ�����һ�����Ӵ�
{
	if(Claw_FindingNextBox==1)
{
	Sensor_Read_Lower();
	if(Sensor_Ready[0]!=2)
	{
		ChassisSpeedRef.forward_back_ref = -50 * RC_CHASSIS_SPEED_REF;
	}
	if(Sensor_Ready[0]==2)
	{
		ChassisSpeedRef.forward_back_ref=0.0f;
		Claw_FindingNextBox=0;
	}
}
}
void Claw_Up()//����������̧����̧�����צ���Զ���λ
{
			if(Claw_UpToPosition==1&&Claw_UpAngle<=864&&auto_counter==0)//860
			{
				Claw_UpAngle+=4;
				NMUDL.TargetAngle=Claw_UpAngle;
				NMUDR.TargetAngle=-Claw_UpAngle;
				auto_counter=1;
			}
			if(Claw_UpToPosition==1&&hasReach(&NMUDL,10)&&NMUDL.RealAngle>800)
				Claw_SelfInspecting=1;
}
void AutoGet_SwitchState()//ִ������ȡ��ģʽ ����/����
{
	if(AutoGet_Start==1)
		AutoGet_Lower();
	if(AutoGet_Start==2)
		AutoGet_Upper();
}

