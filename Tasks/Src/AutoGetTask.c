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
	
#define FIRSTBOX 80
#define SECONDBOX 820
#define THIRDBOX 1550             //�����������λ��
#define FOURTHBOX 400
#define FIFTHBOX 1120

#define LOWERCRITICIAL 2000      //�����ٽ�ֵ
#define UPPERCRITICIAL 800 //������

//#define UPLEVEL 432    //̧��ʱ�ĺ��ʸ߶� ���뱻4����
#define UPPROTECT 400  //̧�����ٽ籣��ֵ

#define OUTANGLE 190  //ץ���ӵĽǶ�ֵ
#define INANGLE  50   //�������ӻ����ĽǶ�ֵ

#define THROWANGLE 180 //�ӵ�����ʱ������Ƕ���צ��

Distance_Couple_t distance_couple;
uint32_t AutoGet_Start=0;  
uint32_t AutoGet_TotalStep=1;
uint32_t AutoGet_Alreadywaited=0;
uint32_t Claw_AlreadyRollOut=0;
uint32_t Claw_AlreadyWaited=0;
uint32_t Claw_AlreadyTight=0;
uint32_t Claw_UpToPosition=0;
uint32_t Claw_DownToPosition=0;
uint16_t Claw_TruePosition[5] = {0, 820, 1600, 400, 1100};
int32_t  Claw_UpAngle=0;
uint32_t Claw_TakeThisBox=0;
uint32_t Claw_SelfInspecting=0;
uint32_t Claw_FirstSelfInspect=0;
uint32_t Claw_FindingNextBox_Lower=0;
uint32_t Claw_FindingNextBox_Upper=0;
uint32_t Claw_SetZero=0;
uint32_t Claw_Zero_Counting=0;
uint32_t Claw_Zero_Count=0;
//�洢���⴫��������ֵ
extern uint32_t ADC_value[160];
extern uint32_t ADC2_value[10];
uint32_t adgl=0,adgr=0;
uint32_t disgl=0,disgr=0;

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
uint32_t warning_cnt=0;
uint32_t claw_warning=0;
int16_t cnt = 0;
uint32_t ifset=-5;//�����Լ�

void RefreshADC()
{
	for(uint16_t i=0;i<160;i++)
	{
		if(i%8==0)adfl+=ADC_value[i];
		if(i%8==1)adfr+=ADC_value[i];
		if(i%8==2)adbl+=ADC_value[i];
		if(i%8==3)adbr+=ADC_value[i];
		if(i%8==4)addf+=ADC_value[i];
		if(i%8==5)addb+=ADC_value[i];
		if(i%8==6)adgl+=ADC_value[i];
		if(i%8==7)adgr+=ADC_value[i];
	}
	adfl=adfl/21;
	adfr=adfr/21;
	adbl=adbl/21;
	adbr=adbr/21;
	addf=addf/21;
	addb=addb/21;
	adgl=adgl/21;
	adgr=adgr/21;
	
	
	disfl=adfl;
	disfr=adfr;
	disbl=adbl;
	disbr=adbr;
	disdf=addf;
	disdb=addb;
	disgl=adgl;
	disgr=adgr;
	
	distance_couple.frontl.val_ref	= adfl;
	distance_couple.frontr.val_ref	= adfr;
	distance_couple.frontf.val_ref	= addf;
	distance_couple.backl.val_ref	= adbl;
	distance_couple.backr.val_ref	= adbr;
	distance_couple.backb.val_ref	= addb;
	distance_couple.left.val_ref	= adgl;
	distance_couple.right.val_ref	= adgr;
		
		
	FLAG_SET(distance_couple.frontf);
	FLAG_SETr(distance_couple.frontr);
	FLAG_SET(distance_couple.frontl);
	FLAG_SET(distance_couple.backb);
	FLAG_SETbr(distance_couple.backr);
	FLAG_SETbl(distance_couple.backl);
	FLAG_SET(distance_couple.left);
	FLAG_SET(distance_couple.right);
	
	distance_couple.move_flags = 0;
	distance_couple.move_flags = ((distance_couple.left.flag)								*512) +
								 ((distance_couple.right.flag) 								*256) +
								 ((distance_couple.frontl.flag) 							*128) +
								 ((distance_couple.frontr.flag) 							* 64) +
								 ((distance_couple.backl.flag) 								* 32) +
								 ((distance_couple.backr.flag) 								* 16) +
								 ((distance_couple.frontr.flag&distance_couple.frontl.flag) *  8) +
								 ((distance_couple.frontf.flag) 							*  4) +
								 ((distance_couple.backb.flag)								*  2) +
								 ((distance_couple.backr.flag&distance_couple.backl.flag)	*  1);
		//�Ͱ�λ������λ �����ж�������λ ��ϸ�ж���ת��
		//�߰�λ������λ ץȡ�ж�
}

uint8_t hasReach(MotorINFO* id, double distance)//�����жϵ���Ƿ�λ
{
	if(fabs(id->RealAngle - id->TargetAngle) < distance)return 1;
	else return 0;
}

uint8_t canMovePositive(MotorINFO* id, int16_t stuckMoment)
{
	if(id->RxMsgC6x0.moment < stuckMoment)return 1;
	else return 0;
}

uint8_t canMoveNegetive(MotorINFO* id, int16_t stuckMoment)
{
	if(id->RxMsgC6x0.moment > stuckMoment)return 1;
	else return 0;
}

uint8_t stuckPositive(MotorINFO* id)
{
	if(id->warningDir == 1 && id->warningCount > 1000)return 1;
	else return 0;
}

uint8_t stuckNegetive(MotorINFO* id)
{
	if(id->warningDir == -1 && id->warningCount > 1000)return 1;
	else return 0;
}

void Sensor_Read_Lower()//���ڼ����⴫�����Ƿ��⵽������϶
{
	if(Sensor_Tmp[0]<LOWERCRITICIAL&&adgl<LOWERCRITICIAL)
		Sensor_Count[0]=1;
	else
	{Sensor_Tmp[0]=adgl;Sensor_Count[0]=0;}
	if(Sensor_Count[0]==1&&Sensor_Ready[0]==0)
		Sensor_Ready[0]=1;
	if(Sensor_Tmp[1]<LOWERCRITICIAL&&adgr<LOWERCRITICIAL)
		Sensor_Count[1]=1;
	else
	{Sensor_Tmp[1]=adgr;Sensor_Count[1]=0;}
	if(Sensor_Count[1]==1&&Sensor_Ready[0]==1)
		Sensor_Ready[0]=2;
		
}
void Sensor_Read_Upper()//���ڼ����⴫�����Ƿ��⵽������϶
{
	if(Sensor_Tmp[0]<UPPERCRITICIAL&&adgl<UPPERCRITICIAL)
		Sensor_Count[0]=1;
	else
	{Sensor_Tmp[0]=adgl;Sensor_Count[0]=0;}
	if(Sensor_Count[0]==1&&Sensor_Ready[0]==0)
		Sensor_Ready[0]=1;
	if(Sensor_Tmp[1]<UPPERCRITICIAL&&adgr<UPPERCRITICIAL)
		Sensor_Count[1]=1;
	else
	{Sensor_Tmp[1]=adgr;Sensor_Count[1]=0;}
	if(Sensor_Count[1]==1&&Sensor_Ready[0]==1)
		Sensor_Ready[0]=2;
		
}
void Claw_Rollout()//צ��ת����ת��
{
	if(auto_counter==0&&Claw_AlreadyRollOut==0)
	{
		UM1.TargetAngle=-OUTANGLE;
		UM2.TargetAngle=OUTANGLE;
	}
	if((hasReach(&UM1, 10) || hasReach(&UM2, 10))&&Claw_AlreadyRollOut==0&&UM1.TargetAngle==-OUTANGLE)
	{Claw_AlreadyRollOut=1;}
}

void Claw_Rollin()
{
	if(auto_counter==0&&Claw_AlreadyRollOut==1)
	{
		UM1.TargetAngle=-INANGLE;
    UM2.TargetAngle=INANGLE;
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
		CLAWTIGHT;
		auto_counter=300;
		Claw_AlreadyTight=1;
	}
}

void Claw_Loose()
{
	if(auto_counter==0&&auto_waiter==0)
	   {
	CLAWLOOSE;
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
	  LAUNCH;
	  auto_wait=300;
		Claw_AlreadyWaited=2;
		AutoGet_TotalStep++;//�¼ӵ�
	}
}

void Box_Land()
{
	if(auto_wait==0&&Claw_AlreadyWaited==2)
	{
	  LAND;
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
		case 1:{UFM.TargetAngle=FIRSTBOX;
		        if(hasReach(&UFM,15))
		           AutoGet_TotalStep++;
		        break;}
		case 2:{UFM.TargetAngle=SECONDBOX;
		        if(hasReach(&UFM,5))
		           AutoGet_TotalStep++;
		        break;}
		case 3:{UFM.TargetAngle=THIRDBOX;
		        if(hasReach(&UFM,20))
		           AutoGet_TotalStep++;
		        break;}
		case 4:{UFM.TargetAngle=FOURTHBOX;
		        if(hasReach(&UFM,5))
		           AutoGet_TotalStep++;
		        break;}
		case 5:{UFM.TargetAngle=FIFTHBOX;
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
	CLAWLOOSE;
	CLAWIN;
	AutoGet_Start=0;
	AutoGet_TotalStep=1;
	AutoGet_Alreadywaited=0;
	UM1.TargetAngle=-INANGLE;
  UM2.TargetAngle=INANGLE;
	Claw_AlreadyRollOut=0;
	Claw_AlreadyTight=0;
	
	Claw_TakeThisBox=0;
	Claw_FindingNextBox_Lower=0;
	Claw_FindingNextBox_Upper=0;
	Claw_UpToPosition=0;
	Sensor_Ready[0]=0;
}
void Box_ThrowForward()//��ǰ�ӳ�����
{ 
	if(auto_counter==0&&auto_waiter==0){
	UM1.TargetAngle=-OUTANGLE;
	UM2.TargetAngle=OUTANGLE;
	if(fabs(UM1.RealAngle+75)<=10||fabs(UM2.RealAngle+(-75))<=10)
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
		case 9:{Claw_GoTo(5);break;}
		case 10:{Box_Fire();    break;}
		case 11:{CLAWOUT;AutoGet_TotalStep++;auto_counter=500;break;}  
		case 12:{Claw_GetaBox(); break;}
		case 13:{Claw_GoTo(4);break;}
		case 14:{Box_Fire();    break;}
		case 15:{Claw_GetaBox(); break;}
		case 16:{auto_counter=300;AutoGet_TotalStep++;break;}
		case 17:{Box_Fire();    break;}
		case 18:{CLAWIN;AutoGet_TotalStep++;break;}
		default:{AutoGet_Stop_And_Clear();}
	}
}
void AutoGet_LowerANDThrow()//�Զ�ȡ���������������
{
	switch(AutoGet_TotalStep)
	{
		case 1:{Claw_GoTo(1);break;}
		case 2:{Claw_GetaBox();  break;}
		case 3:{Claw_GoTo(2);break;}
		case 4:{Box_ThrowForward();     break;}
		case 5:{Claw_GetaBox();  break;}
		case 6:{Claw_GoTo(3);break;}
		case 7:{Box_ThrowForward();     break;}
		case 8:{Claw_GetaBox();  break;}
		case 9:{Claw_GoTo(5);break;}
		case 10:{Box_ThrowForward();    break;}
		case 11:{CLAWOUT;AutoGet_TotalStep++;auto_counter=500;break;}  
		case 12:{Claw_GetaBox(); break;}
		case 13:{Claw_GoTo(4);break;}
		case 14:{Box_ThrowForward();    break;}
		case 15:{Claw_GetaBox(); break;}
		case 16:{auto_counter=300;AutoGet_TotalStep++;break;}
		case 17:{Box_ThrowForward();    break;}
		case 18:{CLAWIN;AutoGet_TotalStep++;break;}
		default:{AutoGet_Stop_And_Clear();}
	}
}
void AutoGet_Upper()//�Զ�ȡ����������������
{
	switch(AutoGet_TotalStep)
	{
		case 1:{CLAWOUT;auto_counter=500;
		AutoGet_TotalStep++;break;}
		case 2:{Claw_GoTo(1);break;}
		case 3:{Claw_GetaBox();  break;}
		case 4:{Claw_GoTo(2);break;}
		case 5:{Box_ThrowForward();     break;}
		case 6:{Claw_GetaBox();  break;}
		case 7:{Claw_GoTo(3);break;}
		case 8:{Box_ThrowForward();     break;}
		case 9:{Claw_GetaBox();  break;}
		case 10:{Claw_GoTo(4);break;}
		case 11:{Box_ThrowForward();    break;}
		case 12:{UM1.TargetAngle=0;UM2.TargetAngle=0;AutoGet_TotalStep++;break;}
		case 13:{CLAWIN;auto_counter=500;
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
				case 1:{CLAWIN;auto_counter=500;AutoGet_TotalStep++;break;}
				case 2:{Claw_GoTo(1);break;}
				case 3:{
					  Claw_GetaBox(); 
					if(AutoGet_Alreadywaited==0)
					{auto_waiter=2000;AutoGet_Alreadywaited=1;}  
					  break;}
				case 4:{Box_Fire();     break;}
				default:{AutoGet_Stop_And_Clear();   break;}
			}
		}break;
		case 2:{
			switch(AutoGet_TotalStep)
			{
				case 1:{CLAWIN;auto_counter=500;AutoGet_TotalStep++;break;}
				case 2:{Claw_GoTo(2);break;}
				case 3:{
					  Claw_GetaBox(); 
					if(AutoGet_Alreadywaited==0)
					{auto_waiter=2000;AutoGet_Alreadywaited=1;}  
					  break;}
				case 4:{Box_Fire();     break;}
				default:{AutoGet_Stop_And_Clear();   break;}
			}
		}break;
		case 3:{
			switch(AutoGet_TotalStep)
			{
				case 1:{CLAWIN;auto_counter=500;AutoGet_TotalStep++;break;}
				case 2:{Claw_GoTo(3);break;}
				case 3:{
					  Claw_GetaBox(); 
					if(AutoGet_Alreadywaited==0)
					{auto_waiter=2000;AutoGet_Alreadywaited=1;}  
					  break;}
				case 4:{Box_Fire();     break;}
				default:{AutoGet_Stop_And_Clear();   break;}
			}
		}break;
		case 4:{
			switch(AutoGet_TotalStep)
			{
				case 1:{CLAWOUT;auto_counter=500;AutoGet_TotalStep++;break;}
				case 2:{Claw_GoTo(4);break;}
				case 3:{
					  Claw_GetaBox(); 
					if(AutoGet_Alreadywaited==0)
					{auto_waiter=2000;AutoGet_Alreadywaited=1;}  
					  break;}
				case 4:{Box_Fire();CLAWIN;     break;}
				default:{AutoGet_Stop_And_Clear();   break;}
			}
		}break;
		case 5:{
			switch(AutoGet_TotalStep)
			{
				case 1:{CLAWOUT;auto_counter=500;AutoGet_TotalStep++;break;}
				case 2:{Claw_GoTo(5);break;}
				case 3:{
					  Claw_GetaBox(); 
					if(AutoGet_Alreadywaited==0)
					{auto_waiter=2000;AutoGet_Alreadywaited=1;}  
					  break;}
				case 4:{Box_Fire();CLAWIN;     break;}
				default:{AutoGet_Stop_And_Clear();   break;}
			}
		}break;
		default:break;
	}
}
void Claw_SelfInspect()//צ�Ӻ����Զ���λ���
{
	if(Claw_FirstSelfInspect==0)
	{
		Claw_SelfInspecting=1;
		Claw_FirstSelfInspect=1;
	}
	if(Claw_SetZero==1)
	{
	if(UFM.RxMsgC6x0.moment>-4000&&Claw_SelfInspecting==1)
		UFM.TargetAngle-=8;
	if(UFM.RxMsgC6x0.moment<-4000&&Claw_SelfInspecting==1)
	{
		UFM.RealAngle=0;
		UFM.TargetAngle=FOURTHBOX;
		Claw_SelfInspecting=2;
		Claw_UpToPosition=0;
	}
  }
}
void Claw_GoToNextBox_lower()//���⴫��������צ�ӵ�����һ�����Ӵ�
{
	if(Claw_FindingNextBox_Lower==1)
{
	Sensor_Read_Lower();
	if(Sensor_Ready[0]!=2)
	{
		ChassisSpeedRef.forward_back_ref = 50 * RC_CHASSIS_SPEED_REF;
	}
	if(Sensor_Ready[0]==2)
	{
		ChassisSpeedRef.forward_back_ref=0.0f;
		Claw_FindingNextBox_Lower=0;
	}
}
}
void Claw_GoToNextBox_upper()//���⴫��������צ�ӵ�����һ�����Ӵ�
{
	if(Claw_FindingNextBox_Upper==1)
{
	Sensor_Read_Upper();
	if(Sensor_Ready[0]!=2)
	{
		ChassisSpeedRef.forward_back_ref = 150 * RC_CHASSIS_SPEED_REF;
	}
	if(Sensor_Ready[0]==2)
	{
		ChassisSpeedRef.forward_back_ref=0.0f;
		Claw_FindingNextBox_Upper=0;
	}
}
}
void Claw_Up()//����������̧����̧�����צ���Զ���λ
{
			if(Claw_UpToPosition==1&&Claw_UpAngle<=UPLEVEL&&auto_counter==0)//-480
			{
				Claw_UpAngle+=12;
				NMUDL.TargetAngle=-Claw_UpAngle;
				NMUDR.TargetAngle=-Claw_UpAngle;
				auto_counter=1;
			}
			//if(Claw_UpToPosition==1&&hasReach(&NMUDL,10)&&NMUDL.RealAngle<-UPPROTECT)
			//{Claw_SelfInspecting=1;}
			if(Claw_UpToPosition==1&&NMUDL.RealAngle<=(-UPLEVEL+5)&&NMUDR.RealAngle<=(-UPLEVEL+5))
			{
				Claw_UpToPosition=0;
				Claw_GoTo(1);
			}
}

void Claw_Down()
{
	 if(Claw_DownToPosition==1&&auto_counter==0)
	 {
		 Claw_UpAngle-=12;
		 NMUDL.TargetAngle=-Claw_UpAngle;
		 NMUDR.TargetAngle=-Claw_UpAngle;
		 auto_counter=1;
	 }
	 if(Claw_DownToPosition==1&&NMUDL.RealAngle>=-5&&NMUDR.RealAngle>=-5)
	 {
		 Claw_DownToPosition=0;
	 }
}

void AutoGet_SwitchState()//ִ������ȡ��ģʽ ����/����
{
	if(AutoGet_Start==1)
		AutoGet_Lower();
	if(AutoGet_Start==2)
		AutoGet_Upper();
	if(AutoGet_Start==3)
		AutoGet_LowerANDThrow();
}

void ClawUpDown_SwitchState()
{
	if(Claw_DownToPosition==1)
	{
		Claw_Down();
		Claw_UpToPosition = 0;
	}
	else if(Claw_UpToPosition==1)
	{
		Claw_Up();
		Claw_DownToPosition = 0;
	}
}

void Claw_Protect()
{
	if(UFM.RxMsgC6x0.moment>10000||UFM.RxMsgC6x0.moment<-10000)
		claw_warning=1;
	if(UFM.RxMsgC6x0.moment>-10000&&UFM.RxMsgC6x0.moment<10000)
		claw_warning=0;
	
	if(warning_cnt>1000)
	{
		UFM.TargetAngle=UFM.RealAngle;
		if(UFM.RxMsgC6x0.moment>10000)
			UFM.TargetAngle-=20;
		else if(UFM.RxMsgC6x0.moment<-10000)
			UFM.TargetAngle+=20;
		
	}
}

void Claw_AutoIn()
{
	if(Claw_SetZero==0)
	{
		if(UM1.RxMsgC6x0.moment<4000||UM2.RxMsgC6x0.moment>-4000)
		{
			Claw_Zero_Counting=0;
	    UM1.TargetAngle+=3;
	    UM2.TargetAngle-=3;
		}
		if(UM1.RxMsgC6x0.moment>4000&&UM2.RxMsgC6x0.moment<-4000)
		{
			Claw_Zero_Counting=1;
		}
		if(Claw_Zero_Count>=1000)
		{
			UM1.RealAngle=0;
			UM1.TargetAngle=0;
			UM2.RealAngle=0;
			UM2.TargetAngle=0;
			Claw_SetZero=1;
		}
	}
}

void Claw_AutoInTest()
{
	if(Claw_SetZero==0)
	{
		if(canMovePositive(&UM1,4000)||canMoveNegetive(&UM2,-4000))
		{
	    UM1.TargetAngle+=3;
	    UM2.TargetAngle-=3;
		}
		if(stuckPositive(&UM1) && stuckNegetive(&UM2))
		{
			UM1.RealAngle=0;
			UM1.TargetAngle=0;
			UM2.RealAngle=0;
			UM2.TargetAngle=0;
			Claw_SetZero=1;
		}
	}
}
