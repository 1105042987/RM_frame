/**
  ******************************************************************************
  * File Name          : AutoGetTask.c
  * Description        : 自动取弹控制任务
  ******************************************************************************
  *
  * Copyright (c) 2019 Team JiaoLong-ShanghaiJiaoTong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#include "includes.h"
	
#define FIRSTBOX -40
#define SECONDBOX -815
#define THIRDBOX -1590            //这五个是箱子位置
#define FOURTHBOX -415
#define FIFTHBOX -1215
#define SIXTHBOX -820

#define LOWERCRITICIAL 2000      //岛下临界值
#define UPPERCRITICIAL_LEFT 1100 // 岛上临界值    1000-1100
#define UPPERCRITICIAL_RIGHT 1300  //             1200-1300


//#define UPLEVEL 432    //抬升时的合适高度 必须被4整除                            已写到.h里
#define UPPROTECT 600  //抬升的临界保护值

#define OUTANGLE 187  //抓箱子的角度值
#define INANGLE  50   //带着箱子回来的角度值

#define THROWANGLE 180 //扔掉箱子时在这个角度松爪子


Distance_Couple_t distance_couple;
uint32_t AutoGet_Start=0;  
uint32_t AutoGet_TotalStep=1;
uint32_t AutoGet_Alreadywaited=0;
uint32_t AutoGet_Success=0;
uint32_t AutoGet_Error=0;
uint32_t AutoGet_Skill=0;
uint32_t AutoGet_Bullet_S=0;
uint32_t AutoGet_Bullet_B=0;
uint32_t Claw_AlreadyRollOut=0;
uint32_t Claw_AlreadyWaited=0;
uint32_t Claw_AlreadyTight=0;
uint32_t Claw_RollBack=0;
uint32_t Claw_UpToPosition=0;
uint32_t Claw_DownToPosition=0;
uint16_t Claw_TruePosition[5] = {0, 820, 1600, 400, 1100};
int32_t  Claw_UpAngle=0;
int Claw_TakeThisBox=0;
uint32_t Claw_SelfInspecting=0;
uint32_t Claw_FirstSelfInspect=0;
uint32_t Claw_FindingNextBox_Lower_Forward=0;
uint32_t Claw_FindingNextBox_Lower_Backward=0;
uint32_t Claw_FindingNextBox_Upper_Forward=0;
uint32_t Claw_FindingNextBox_Upper_Backward=0;
uint32_t Claw_SetZero=0;
uint32_t Claw_Zero_Counting=0;
uint32_t Claw_Zero_Count=0;
uint32_t Claw_SelfInspect_cnt=0;
uint32_t Claw_FirstInspect=0;
uint32_t Claw_Out=0;
uint32_t Box_Clearing=0;
uint32_t Box_Tight=0;
extern uint32_t ClawBack_Locker;
extern uint32_t Z_count;
extern uint32_t Z_State;

uint8_t CM_AutoRotate90=0;				//底盘自动转90度，1——转，0——到位
//存储红外传感器的数值
extern uint32_t ADC_value[160];
extern uint32_t ADC2_value[10];
uint32_t adgl=0,adgr=0;
uint32_t disgl=0,disgr=0;

//消抖用变量
uint32_t Sensor_Tmp[2];
uint16_t Sensor_Count[2];
uint32_t Sensor_Ready[2]={0,0};
uint32_t Sensor_a;
uint32_t Sensor_b;
uint32_t Sensor_LongPush=0;
uint32_t Sensor_Lock=0;


int32_t auto_counter=0;		//用于准确延时的完成某事件
uint32_t rotate_waiter=0;
int32_t auto_waiter=0;
int32_t cnt_clk;
int32_t auto_wait=0;
uint32_t warning_cnt=0;
uint32_t claw_warning=0;
uint32_t sensorlock_cnt=0;
int16_t cnt = 0;
uint32_t clawback=0;
uint32_t clawback_cnt=0;

extern Engineer_State_e EngineerState;//用于处理车辆模式
extern uint32_t Direction_Indicator;
extern uint32_t OnePush_Locker;
extern View_State_e Viewstate;
extern SlaveMode_e Slave;
extern uint32_t Slave_Commoning;
extern uint32_t Chassis_locker;
extern ManualClimb_State_e ManualClimb_State;

uint32_t Yaw_Reset_Flag=0;
uint32_t Yaw_Reset_Cnt=0;
uint32_t Yaw_Set_Flag=0;
uint32_t Yaw_Set_Cnt=0;
int32_t  imu_start_angle=0;

void RefreshADC()
{
	for(uint16_t i=0;i<160;i++)
	{
		if(i%8==0)adfl+=ADC_value[i];
		if(i%8==1)adfr+=ADC_value[i];
		if(i%8==2)adbr+=ADC_value[i];
		if(i%8==3)adbl+=ADC_value[i];
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
		
		
	FLAG_SET_TRICK(distance_couple.frontl);
	FLAG_SET_TRICK(distance_couple.frontr);
	FLAG_SET(distance_couple.frontf);
	FLAG_SETbl(distance_couple.backl);
	FLAG_SETbr(distance_couple.backr);
	FLAG_SETdb(distance_couple.backb);
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
		//低八位：低四位 基础判定，高四位 精细判定（转向）
		//高八位：低四位 抓取判定
}

uint8_t hasReach(MotorINFO* id, double distance)//用于判断电机是否到位
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

void Sensor_Read_Lower()//用于检测红外传感器是否检测到两个空隙
{
//	if(Sensor_Tmp[0]>LOWERCRITICIAL&&adgl>LOWERCRITICIAL)
//		Sensor_Count[0]=1;
//	else
//	{Sensor_Tmp[0]=adgl;Sensor_Count[0]=0;}
//	if(Sensor_Count[0]==1&&Sensor_Ready[0]==0)
//		Sensor_Ready[0]=1;
//	if(Sensor_Tmp[1]>LOWERCRITICIAL&&adgr>LOWERCRITICIAL)
//		Sensor_Count[1]=1;
//	else
//	{Sensor_Tmp[1]=adgr;Sensor_Count[1]=0;}
//	if(Sensor_Count[1]==1&&Sensor_Ready[0]==1)
//		Sensor_Ready[0]=2;
//	
	if(adgl>LOWERCRITICIAL&&adgr>LOWERCRITICIAL)
		Sensor_Ready[0]=2;
		
}
void Sensor_Read_Upper()//用于检测红外传感器是否检测到两个空隙
{
//	if(Sensor_Tmp[0]>UPPERCRITICIAL_LEFT&&adgl>UPPERCRITICIAL_LEFT)
//		Sensor_Count[0]=1;
//	else
//	{Sensor_Tmp[0]=adgl;Sensor_Count[0]=0;Sensor_Ready[0]=0;}
//	if(Sensor_Count[0]==1&&Sensor_Ready[0]==0)
//		Sensor_Ready[0]=1;
//	if(Sensor_Tmp[1]>UPPERCRITICIAL_RIGHT&&adgr>UPPERCRITICIAL_RIGHT)
//		Sensor_Count[1]=1;
//	else
//	{Sensor_Tmp[1]=adgr;Sensor_Count[1]=0;Sensor_Ready[0]=0;}
//	if(Sensor_Count[1]==1&&Sensor_Ready[0]==1)
//		Sensor_Ready[0]=2;
	
	if(adgl>UPPERCRITICIAL_LEFT&&adgr>UPPERCRITICIAL_RIGHT)
		Sensor_Ready[0]=2;
		
}
void Claw_Rollout()//爪子转出与转回
{
	if(auto_counter==0&&Claw_AlreadyRollOut==0)
	{
		if(CLAW_INSPECT_SUCCEED)
		{
		UM1.TargetAngle=-OUTANGLE;
		UM2.TargetAngle=OUTANGLE;
		}
		else
		{
		UM1.TargetAngle=-(OUTANGLE*0.5+1);
		UM2.TargetAngle=OUTANGLE*0.5+1;
		}
			
	}
	if((hasReach(&UM1, 10) || hasReach(&UM2, 10))&&Claw_AlreadyRollOut==0&&((UM1.TargetAngle==-OUTANGLE&&CLAW_INSPECT_SUCCEED)||(UM1.TargetAngle==-(OUTANGLE*0.5+1))))
	{Claw_AlreadyRollOut=1;}
}

void Claw_Rollin()
{
	if(auto_counter==0&&Claw_AlreadyRollOut==1)
	{
		if(CLAW_INSPECT_SUCCEED)
		{
		UM1.TargetAngle=-INANGLE;
    UM2.TargetAngle=INANGLE;
		}
		Claw_AlreadyRollOut=2;
	}
//	if((fabs(UM1.RealAngle+65)<=10||fabs(UM2.RealAngle+(-65))<=10)&&ON_THE_GROUND&&auto_counter==0&&Claw_AlreadyRollOut==2)   
//	{
//		CLAWLOOSE;
//	}
	if((hasReach(&UM1, 5) || hasReach(&UM2, 5))&&Claw_AlreadyRollOut==2)
	{
		AutoGet_TotalStep++;
		Claw_AlreadyRollOut=0;
		Claw_AlreadyTight=0;
		//被删除的语句 测试动作衔接 auto_counter=500;
	}
}

void Claw_Stay()
{
	if(auto_counter==0&&Claw_AlreadyRollOut==1)
	{
		Claw_AlreadyRollOut=2;
	}
	if((hasReach(&UM1, 5) || hasReach(&UM2, 5))&&Claw_AlreadyRollOut==2)
	{
		AutoGet_TotalStep++;
		Claw_AlreadyRollOut=0;
		Claw_AlreadyTight=0;
	}
}
void Claw_Tight()//爪子抓紧与松开
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
void Box_launch()//弹药箱弹射装置
{
	if(auto_wait==0&&Claw_AlreadyWaited==1)
	{
	  LAUNCH;
	  auto_wait=300;
		Claw_AlreadyWaited=2;
		AutoGet_TotalStep++;//新加的
	}
}

void Box_Land()
{
	if(auto_wait==0&&Claw_AlreadyWaited==2)
	{
	  LAND;
		//AutoGet_TotalStep++;//刚删除的
		Claw_AlreadyWaited=0;
	}
}

void Claw_GoTo(int a)//爪子走到第a个箱子的位置
{
	clawback=0;
	if(auto_counter==0)
	{
	switch(a)
	{
		case 1:{UFM.TargetAngle=FIRSTBOX;
		        if(hasReach(&UFM,20))
		           AutoGet_TotalStep++;
		        break;}
		case 2:{UFM.TargetAngle=SECONDBOX;
		        if(hasReach(&UFM,5))
		           AutoGet_TotalStep++;
		        break;}
		case 3:{UFM.TargetAngle=THIRDBOX;
		        if(hasReach(&UFM,50))
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
		case 6:{UFM.TargetAngle=SIXTHBOX;
		        if(hasReach(&UFM,5))
		           AutoGet_TotalStep++;
		        break;}
	}
  }
}
void Claw_GetaBox()//取一个箱子的完成流程
{
	Claw_Rollout();
	Claw_Tight();
	Claw_Rollin();
}
void Claw_FetchaBox()//很骚的取一个箱子的流程
{
	Claw_Rollout();
	Claw_Tight();
	Claw_Stay();
}
void Box_Fire()//弹射一个箱子的完整流程
{
	Claw_Loose();
	Box_launch();
	//Box_Land();//删除的
}
void AutoGet_Stop_And_Clear()//状态清零 爪子转回 横移电机停转（用于异常状况处理和一次取弹结束）
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
	Claw_FindingNextBox_Lower_Forward=0;
	Claw_FindingNextBox_Upper_Forward=0;
	Claw_FindingNextBox_Lower_Backward=0;
	Claw_FindingNextBox_Upper_Backward=0;
	Claw_UpToPosition=0;
	Sensor_Ready[0]=0;
	
	if(AutoGet_Error==0)
	{
		if(AutoClimb_Level!=1)
		{
			if(ClawBack_Locker==0&&ON_THE_GROUND)
	    {
			 clawback=1;
	     clawback_cnt=1000;
			}
			else if(ClawBack_Locker==1)
				ClawBack_Locker=0;
		}
	}
	else if(AutoGet_Error==1)
	{
		AutoGet_Error=0;
	}
	
}
void Box_ThrowForward()//向前扔出箱子
{ 
	if(auto_counter==0&&auto_waiter==0)
 {
	UM1.TargetAngle=-OUTANGLE;
	UM2.TargetAngle=OUTANGLE;
	if(fabs(UM1.RealAngle+70)<=10||fabs(UM2.RealAngle+(-70))<=10)   //变大是晚扔，变小是早扔 最远为65
	{
		CLAWLOOSE;
	}
	if(hasReach(&UM1,5)||hasReach(&UM2,5))
	{
		AutoGet_TotalStep++;
	}
 }
}

void Box_ThrowBackward()//向后扔出箱子
{
	if(auto_counter==0&&auto_waiter==0&&Claw_AlreadyRollOut==0)
	{
	UM1.TargetAngle=-OUTANGLE;
	UM2.TargetAngle=OUTANGLE;
	Claw_AlreadyRollOut=1;
	}
	if(fabs(UM1.RealAngle+140)<=10||fabs(UM2.RealAngle+(-140))<=10)   //变大是晚扔，变小是早扔
	{
	UM1.TargetAngle=-INANGLE;
  UM2.TargetAngle=INANGLE;
	Claw_RollBack=1;
	}
	if(Claw_RollBack==1&&(fabs(UM1.RealAngle+115)<=10||fabs(UM2.RealAngle+(-115))<=10))
	{
		Claw_RollBack=0;
		CLAWLOOSE;
	}
	if(hasReach(&UM1,5)||hasReach(&UM2,5))
	{
		AutoGet_TotalStep++;
		Claw_AlreadyRollOut=0;
	}
	
}
void Box_Clear()
{
	if(Box_Clearing==1)
	{
		if(Box_Tight==0)
		{
			CLAWTIGHT;
			Box_Tight=1;
			auto_counter=300;
		}
	  if(auto_counter==0&&Claw_AlreadyRollOut==0)
	  {
	    UM1.TargetAngle=-OUTANGLE;
	    UM2.TargetAngle=OUTANGLE;
    	Claw_AlreadyRollOut=1;
	  }
		if(fabs(UM1.RealAngle+130)<=10||fabs(UM2.RealAngle+(-130))<=10)   //变大是晚扔，变小是早扔 最远为65
	  {
		  CLAWLOOSE;
			UM1.TargetAngle=-INANGLE;
	    UM2.TargetAngle=INANGLE;
			Claw_AlreadyRollOut=0;
			Box_Tight=0;
			Box_Clearing=0;
	  }
	}
}
//void AutoGet_Lower()//自动取弹（岛下五个弹）
//{
//	switch(AutoGet_TotalStep)
//	{
//		case 1:{Claw_GoTo(1);break;}
//		case 2:{Claw_GetaBox();  break;}
//		case 3:{Claw_GoTo(2);break;}
//		case 4:{Box_Fire();     break;}
//		case 5:{Claw_GetaBox();  break;}
//		case 6:{Claw_GoTo(3);break;}
//		case 7:{Box_Fire();     break;}
//		case 8:{Claw_GetaBox();  break;}
//		case 9:{Claw_GoTo(5);break;}
//		case 10:{Box_Fire();    break;}
//		case 11:{CLAWOUT;AutoGet_TotalStep++;auto_counter=800;break;}  
//		case 12:{Claw_GetaBox(); break;}
//		case 13:{Claw_GoTo(4);break;}
//		case 14:{Box_Fire();    break;}
//		case 15:{Claw_GetaBox(); break;}
//		case 16:{auto_counter=300;AutoGet_TotalStep++;break;}
//		case 17:{Box_Fire();    break;}
//		case 18:{CLAWIN;AutoGet_TotalStep++;break;}
//		default:{AutoGet_Stop_And_Clear();}
//	}
//}
void AutoGet_LowerANDThrow()//自动取弹（岛下五个弹）
{
	switch(AutoGet_TotalStep)
	{
		case 1:{Claw_GoTo(1);break;}
		case 2:{Claw_GetaBox();  break;}
		case 3:{Claw_GoTo(2);break;}
		case 4:{Box_ThrowForward();     AutoGet_Bullet_S+=1;    break;}
		case 5:{Claw_GetaBox();  break;}
		case 6:{Claw_GoTo(3);break;}
		case 7:{Box_ThrowForward();     AutoGet_Bullet_S+=1;  break;}
		case 8:{Claw_GetaBox();  break;}
		case 9:{Claw_GoTo(5);break;}
		case 10:{CLAWOUT;AutoGet_TotalStep++;auto_counter=500;  break;}
		case 11:{Box_ThrowForward();    AutoGet_Bullet_S+=1;  break;}  
		case 12:{Claw_GetaBox(); break;}
		case 13:{Claw_GoTo(4);break;}
		case 14:{Box_ThrowForward();    AutoGet_Bullet_S+=1;  break;}
		case 15:{Claw_GetaBox(); break;}
		case 16:{auto_counter=300;AutoGet_TotalStep++;break;}
		case 17:{Box_ThrowForward();    AutoGet_Bullet_S+=1;  break;}
		case 18:{CLAWIN;AutoGet_TotalStep++;break;}
		default:{AutoGet_Stop_And_Clear();AutoGet_Success=1;break;}
	}
}
void Claw_Wait()
{
	CLAWOUT;
	Claw_GoTo(2);
	UM1.TargetAngle=-(OUTANGLE/2+2);
	UM2.TargetAngle=(OUTANGLE+2);
	
}

void AutoGet_Upper()//自动取弹（岛上三个弹）
{
	switch(AutoGet_TotalStep)
	{
		case 1:{CLAWOUT;auto_counter=500;
		AutoGet_TotalStep++;break;}
		case 2:{Claw_GoTo(1);break;}
		case 3:{Claw_GetaBox();  break;}
		case 4:{Claw_GoTo(2);break;}
		case 5:{Box_ThrowForward();   AutoGet_Bullet_B+=1;  break;}
		case 6:{Claw_GetaBox();  break;}
		case 7:{Claw_GoTo(3);break;}
		case 8:{Box_ThrowForward();   AutoGet_Bullet_B+=1;  break;}
		case 9:{Claw_GetaBox();  break;}
		case 10:{Claw_GoTo(4);break;}
		case 11:{Box_ThrowForward();  AutoGet_Bullet_B+=1;  break;}
		case 12:{UM1.TargetAngle=-INANGLE;UM2.TargetAngle=INANGLE;AutoGet_TotalStep++;break;}
		case 13:{CLAWIN;auto_counter=500;
		AutoGet_TotalStep++;break;}
		default:{AutoGet_Stop_And_Clear();AutoGet_Success=1;break;}
	}
}
void Claw_Go_and_Get(int position)
{ if(CLAW_INSPECT_SUCCEED)
	{
	if(Claw_TakeThisBox!=0)
{
	if(AutoGet_Skill==0){
	switch(AutoGet_TotalStep)
			{
				case 1:
				{
				  if(ON_THE_GROUND&&position<=3)
				  {AutoGet_TotalStep++;break;}
				  if(ON_THE_FLOOR||position>=4)
				  {CLAWOUT;auto_counter=500;AutoGet_TotalStep++;break;}
				}
				case 2:{Claw_GoTo(position);break;}
				case 3:{
					  Claw_GetaBox(); 
					if(AutoGet_Alreadywaited==0)
					{auto_waiter=0;AutoGet_Alreadywaited=1;}  
					  break;}
				case 4:{
				if(ON_THE_GROUND)    Box_ThrowForward();
        else if(ON_THE_FLOOR)Box_ThrowForward();				break;}
				default:{
					if(ON_THE_GROUND)
						AutoGet_Bullet_S+=1;
					if(ON_THE_FLOOR)
						AutoGet_Bullet_B+=1;
				AutoGet_Stop_And_Clear(); 
				AutoGet_Success=1;	
        custom_data.masks=0x00;					
				break;}
			}
		}
	if(AutoGet_Skill==1)  //将箱子取回来
	{
		switch(AutoGet_TotalStep)
			{
				case 1:
				{
				  if(ON_THE_FLOOR)
				  {CLAWOUT;auto_counter=500;AutoGet_TotalStep++;break;}
					if(ON_THE_GROUND)
					{AutoGet_Stop_And_Clear();}
				}
				case 2:{Claw_GoTo(position);break;}
				case 3:{
					  Claw_FetchaBox(); 
					if(AutoGet_Alreadywaited==0)
					{auto_waiter=0;AutoGet_Alreadywaited=1;}  
					  break;}
				case 4:{CLAWIN;auto_waiter=1000;AutoGet_TotalStep++;break;}
				case 5:{if(auto_waiter==0){CLAWLOOSE;auto_counter=200;AutoGet_TotalStep++;break;}}
				default:
				{
				if(auto_waiter==0&&auto_counter==0)
				  {
				  AutoGet_Stop_And_Clear(); 
					UFM.TargetAngle=SECONDBOX;
				  AutoGet_Success=1;  
				  AutoGet_Skill=0;
				  break;
				  }
			  }
	    }
  }
}
}
}
void AutoGet_Enqueue(int position)//入队列
{
	if(OnePush_Locker==0)
		{
		if(EngineerState==GET_STATE)
		if(CLAW_INSPECT_SUCCEED&&CLAW_IS_UP)
		{
		queue_push(&AutoGet_Queue,position);
		}
		OnePush_Locker++;
		}
}
void AutoGet_FillQueue()
{
	if(OnePush_Locker==0)
	{
		if(CLAW_INSPECT_SUCCEED&&CLAW_IS_UP&&ON_THE_GROUND)
		{
			queue_push(&AutoGet_Queue,1);
			queue_push(&AutoGet_Queue,2);
			queue_push(&AutoGet_Queue,3);
			queue_push(&AutoGet_Queue,5);
			queue_push(&AutoGet_Queue,4);
		}
		if(CLAW_INSPECT_SUCCEED&&CLAW_IS_UP&&ON_THE_FLOOR)
		{
			queue_push(&AutoGet_Queue,1);
			queue_push(&AutoGet_Queue,2);
			queue_push(&AutoGet_Queue,3);
		}
		OnePush_Locker++;
	}
}
void AutoGet_Fillstream()
{
		if(CLAW_INSPECT_SUCCEED&&CLAW_IS_UP&&ON_THE_GROUND)
		{
			AutoGet_Start=1;
		}
		if(CLAW_INSPECT_SUCCEED&&CLAW_IS_UP&&ON_THE_FLOOR)
		{
		  AutoGet_Start=2;
		}
}
void AutoGet_Dequeue()
{
	if(AutoGet_TotalStep==1&&Claw_TakeThisBox==0&&!queue_empty(&AutoGet_Queue)&&CLAW_INSPECT_SUCCEED)
		queue_pop(&AutoGet_Queue,&Claw_TakeThisBox);
}
void AutoGet_QueueCheck()
{
	int now_searching=0;
	if(AutoGet_Queue.tail-AutoGet_Queue.head>1)
{
	for(int i=AutoGet_Queue.head;i<AutoGet_Queue.tail;i++)
	    for(int j=i+1;j<AutoGet_Queue.tail;j++)
	{
		now_searching=AutoGet_Queue.arr[i];
		if(AutoGet_Queue.arr[j]==now_searching)
		{
			queue_deinit(&AutoGet_Queue);
			queue_push(&AutoGet_Queue,now_searching);
			now_searching=0;
			return;
		}
		
	}
}
}
void Claw_GetSpecifiedBox()//键鼠控制取任意位置弹
{
	AutoGet_QueueCheck();
	AutoGet_Dequeue();
  Claw_Go_and_Get(Claw_TakeThisBox);
}

void Claw_SelfInspect()//爪子横移自动对位零点
{
	if(Claw_FirstSelfInspect==0)
	{
		Claw_SelfInspecting=1;
		Claw_FirstSelfInspect=1;
	}
	if(Claw_SetZero==1)
	{
	if(UFM.RxMsgC6x0.moment<=4000&&Claw_SelfInspecting==1)
	{
		UFM.TargetAngle+=20;
		Claw_SelfInspect_cnt=0;
	}
	if(UFM.RxMsgC6x0.moment>4000&&Claw_SelfInspecting==1)
	{
		Claw_SelfInspect_cnt++;
	}
	if(Claw_SelfInspect_cnt>20)
	{
		UFM.RealAngle=0;
		if(CLAW_IS_UP)
		UFM.TargetAngle=SECONDBOX;
		else if(CLAW_IS_DOWN)
		UFM.TargetAngle=FOURTHBOX;
		Claw_SelfInspecting=2;
		Claw_UpToPosition=0;
		Claw_SelfInspect_cnt=0;
	}
  }
	if(Claw_SelfInspecting==2)
	{
		if(Claw_FirstInspect==1)
		{
			  AutoGet_Error=1;
				AutoGet_Stop_And_Clear();
				Claw_SelfInspecting=3;
				UM1.TargetAngle=-INANGLE;
        UM2.TargetAngle=INANGLE;
		}
		if(Claw_FirstInspect==0)
		{
		switch(AutoGet_TotalStep)
		{
			case 1:{Claw_GetaBox();break;}
			case 2:{Box_Fire();break;}
			default:{
				AutoGet_Error=1;
				AutoGet_Stop_And_Clear();
				Claw_SelfInspecting=3;
				UM1.TargetAngle=-INANGLE;
        UM2.TargetAngle=INANGLE;
			  Claw_FirstInspect=1;break;}
		}
	  }
		
	}
}
void Claw_GoToNextBox_lower()//红外传感器控制爪子向前到达下一个箱子处
{
	if(Claw_FindingNextBox_Lower_Forward==1||Claw_FindingNextBox_Lower_Backward==1)
{
	if(sensorlock_cnt==0)
	Sensor_Read_Lower();
	else
	{
		Sensor_Ready[0]=0;
	}
	if(Sensor_Ready[0]!=2)
	{
		if(Claw_FindingNextBox_Lower_Forward==1)
		ChassisSpeedRef.forward_back_ref = -100 * RC_CHASSIS_SPEED_REF;
		else if(Claw_FindingNextBox_Lower_Backward==1)
		ChassisSpeedRef.forward_back_ref = 100 * RC_CHASSIS_SPEED_REF;
	}
	if(Sensor_Ready[0]==2)
	{
		Chassis_locker=1;
		Sensor_Ready[0]=0;
		ChassisSpeedRef.forward_back_ref=0.0f;
		Claw_FindingNextBox_Lower_Forward=0;
		Claw_FindingNextBox_Lower_Backward=0;
	}
}
}
void Claw_GoToNextBox_upper()//红外传感器控制爪子到达下一个箱子处
{
	if(Claw_FindingNextBox_Upper_Forward==1||Claw_FindingNextBox_Upper_Backward==1)
{
	if(sensorlock_cnt==0)
	Sensor_Read_Upper();
	else
	{
		Sensor_Ready[0]=0;
	}
	if(Sensor_Ready[0]!=2)
	{
		if(Claw_FindingNextBox_Upper_Forward==1)
		ChassisSpeedRef.forward_back_ref = -100 * RC_CHASSIS_SPEED_REF;
		else if(Claw_FindingNextBox_Upper_Backward==1)
		ChassisSpeedRef.forward_back_ref = 100 * RC_CHASSIS_SPEED_REF;	
	}
	if(Sensor_Ready[0]==2)
	{
		Chassis_locker=1;
		Sensor_Ready[0]=0;
		ChassisSpeedRef.forward_back_ref=0.0f;
		Claw_FindingNextBox_Upper_Forward=0;
		Claw_FindingNextBox_Upper_Backward=0;
	}
}

}

void AutoGet_SensorLock()      //在长按对位键时锁定传感器来实现车的强行移动
{
	if(Sensor_Lock==1)
		sensorlock_cnt=150;
}
void AutoGet_SensorControl()
{
	AutoGet_SensorLock();
	if(Claw_FindingNextBox_Lower_Forward==1||Claw_FindingNextBox_Lower_Backward==1)
			Claw_GoToNextBox_lower();
	if(Claw_FindingNextBox_Upper_Forward==1||Claw_FindingNextBox_Upper_Backward==1)
			Claw_GoToNextBox_upper();	
}
void Claw_Up()//整个机构的抬升  往上走时角度下降
{
			if(Claw_UpToPosition==1&&Claw_UpAngle<UPLEVEL&&auto_counter==0)//-480
			{
				UFM.TargetAngle=SECONDBOX;
				Claw_UpAngle+=10;
				NMUDL.TargetAngle=Claw_UpAngle;
				NMUDR.TargetAngle=Claw_UpAngle;
				auto_counter=1;
			}
			if(Claw_UpToPosition==1&&CLAW_IS_UP)
			{
				Claw_UpToPosition=0;
				NMUDL.TargetAngle=UPLEVEL;
				NMUDR.TargetAngle=UPLEVEL;
			}
}
uint32_t Down_stucking=0;
void Claw_Down()   //往下走时角度增加
{
	 if(Claw_DownToPosition==1&&auto_counter==0)
	 {
		 Claw_UpAngle-=10;
		 NMUDL.TargetAngle=Claw_UpAngle;
		 NMUDR.TargetAngle=Claw_UpAngle;
		 auto_counter=1;
	 }
	 if(NMUDL.TargetAngle)
	 if(Claw_DownToPosition==1&&CLAW_IS_DOWN)
	 {
		 Claw_DownToPosition=0;
		 NMUDL.TargetAngle=15;
		 NMUDR.TargetAngle=15;
		 UFM.TargetAngle=FOURTHBOX;
		 Down_stucking=0;
		 custom_data.masks=0x00;
	 }
}


void ClawUpDown_Protect()
{
	if(Claw_DownToPosition==1&& NMUDL.RxMsgC6x0.moment<-1000&&NMUDR.RxMsgC6x0.moment<-1000)
		Down_stucking+=1;
	if(Down_stucking>=10)
	{
		 Down_stucking=0;
		 Claw_DownToPosition=0;
		 NMUDL.TargetAngle=NMUDL.RealAngle+50;
		 NMUDR.TargetAngle=NMUDR.RealAngle+50;
		 custom_data.masks=0x01;
	}
}

void AutoGet_SwitchState()//执行哪种取弹模式 岛下/岛上
{
	if(AutoGet_Start==1)
		AutoGet_LowerANDThrow();
	if(AutoGet_Start==2)
		AutoGet_Upper();
		Box_Clear();
	 if(Z_State==1&&Z_count==0)
	 {
		 Z_State=2;
	 }
	 if(Z_State==2)
	 {
	   Z_State=0;
	   Claw_DownToPosition=1;
		 State_Common();
	 }
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
	if(UFM.RxMsgC6x0.moment<-10000||UFM.RxMsgC6x0.moment>10000)
		claw_warning=1;
	if(UFM.RxMsgC6x0.moment<10000&&UFM.RxMsgC6x0.moment>-10000)
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
void AutoGet_AutoDown()
{
//	if(((abs(ChassisSpeedRef.forward_back_ref)>=200 * RC_CHASSIS_SPEED_REF)||(abs(ChassisSpeedRef.rotate_ref)>=15* MOUSE_TO_YAW_ANGLE_INC_FACT*15))
//		&&AutoGet_Success==1)
	if(abs(CMFL.RxMsgC6x0.RotateSpeed)>2500 && abs(CMFR.RxMsgC6x0.RotateSpeed)>2500 && abs(CMBL.RxMsgC6x0.RotateSpeed)>2500 && abs(CMBR.RxMsgC6x0.RotateSpeed)>2500			//to be modified
		&&AutoGet_Success==1&&ON_THE_GROUND)
	{
		Claw_DownToPosition=1;
		State_Common();
	}
}
void Claw_AutoBack()
{
	if(clawback==1&&clawback_cnt==0)
	{
		clawback=0;
		Claw_SelfInspecting=1;
	}
}
void State_AutoGet()
{
	InitialSave();
	EngineerState=GET_STATE;
	Direction_Indicator=2;
	//For Debug
	UM1.TargetAngle=-OUTANGLE/2;
	UM2.TargetAngle=OUTANGLE/2;
	///////
	YTP.TargetAngle = YTP_NORMAL;
		if(Yaw_Set_Flag==0)
		{
			Yaw_Set_Flag=1;
			Yaw_Set_Cnt=150;
		}
		Viewstate=GET_VIEW;
	if(Claw_UpToPosition==0)
		Claw_UpToPosition=1;
	if(CM_AutoRotate90==0)
	{
		CM_AutoRotate90 = 1;
		imu_start_angle=imu.now_yaw;
		imu.target_yaw=imu.now_yaw;
	}
}
void Rotate_Check()
{
	if(CM_AutoRotate90==1&&rotate_waiter==0&&imu.target_yaw>=(imu_start_angle-114))
	{
		imu.target_yaw -= 3;
		rotate_waiter=1;
	}
	if(imu.target_yaw<(imu_start_angle-114))
	{
		CM_AutoRotate90 = 0;
	}
}
void State_Common()  
{
	AutoGet_Stop_And_Clear(); 
	UM1.TargetAngle=-INANGLE;
  UM2.TargetAngle=INANGLE;
	AutoClimbing=0;
	ManualClimbing=0;
	AutoGet_Success=0;
	if(ON_THE_FLOOR||ON_THE_GROUND||AutoClimb_Level==1)
	imu_pause=1;
	if(EngineerState==GET_STATE)
	{
		Direction_Indicator=0;
		YTP.TargetAngle = YTP_NORMAL;
		if(Yaw_Reset_Flag==0)
		{
			Yaw_Reset_Flag=1;
			Yaw_Reset_Cnt=300;
		}
	}
		EngineerState=COMMON_STATE;
	  Slave_Commoning=1;;
	  Viewstate=NORMAL_VIEW;
}

void Yaw_Reset_Check()
{
	if(Yaw_Reset_Flag==1&&Yaw_Reset_Cnt==0)
	{
		YTY.TargetAngle=0;
		Yaw_Reset_Flag=0;
	}
}
void Yaw_Set_Check()
{
	if(Yaw_Set_Flag==1&&Yaw_Set_Cnt==0)
	{
		YTY.TargetAngle = SCREEN_POSITION;;
		Yaw_Set_Flag=0;
	}
	
	if(Yaw_Set_Flag==2&&Yaw_Set_Cnt==0)
	{
		YTY.TargetAngle = BACK_POSITION;;
		Yaw_Set_Flag=0;
	}
}
void Yaw_Check()
{
	Yaw_Reset_Check();
	Yaw_Set_Check();
}

void Chassis_Check()
{
	if(AutoGet_TotalStep!=1)
	{
		if(adgl<LOWERCRITICIAL)
			ChassisSpeedRef.forward_back_ref = -20 * RC_CHASSIS_SPEED_REF;
		if(adgr<LOWERCRITICIAL)
			ChassisSpeedRef.forward_back_ref = 20 * RC_CHASSIS_SPEED_REF;
	}
//	if(CLAW_IS_OUT)
//		 // ChassisSpeedRef.left_right_ref = 20 * RC_CHASSIS_SPEED_REF;
}
