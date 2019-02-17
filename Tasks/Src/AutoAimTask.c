/**
  ******************************************************************************
  *FileName				: AutoAimTask.c
  *Description		: �������
  *Author					: ���׺�
  ******************************************************************************
  *
  * Copyright (c) 2019 Team JiaoLong-ShanghaiJiaoTong University
  * All rights reserved.
  *
  ******************************************************************************
*/

#include "includes.h"

#ifndef DEBUG_MODE
#ifdef	USE_AUTOAIM


//*****************************************��������******************************************//

GMINFO_t aim,aim_rcd;																												//Ŀ��Ƕ�
Coordinate_t enemy_gun,enemy_scope,scope_gun;																//����
uint8_t Enemy_INFO[8];																											//����
uint8_t find_enemy=0,aim_mode=0;																						//aim_mode����ѡ����׼ģʽ��0Ϊ����׼��1Ϊ�������飬2Ϊ�����3���ޣ����䣿��
uint16_t aim_cnt=0;																													//�����Ƶ��ʱ����
int16_t current_yaw=0,current_pitch=0;																			//��ǰ��̨�Ƕ�
int16_t receive_cnt=0,receive_rcd=0;																				//�����λ���ź�֡��
double bullet_speed=10.0,bullet_speed_adjust=0,yaw_adjust=0,pitch_adjust=0;	//У׼�������

//********************************************************************************************//


//********************************�����ʼ��********************************//

void InitAutoAim()
{
	//����AUTO_AIM_UART��DMA����
	if(HAL_UART_Receive_DMA(&AUTOAIM_UART,(uint8_t *)&Enemy_INFO,8)!= HAL_OK)
	{
		Error_Handler();
	}
	
	//���������ʼ��������Ҫ�޸ģ�
	enemy_scope.x=0;	enemy_scope.y=0;	enemy_scope.z=200;
	enemy_gun.x=0;		enemy_gun.y=0;		enemy_gun.z=200;
	
	//���������ʼֵ�����ݲ�ͬ��װ���������3��������
	scope_gun.x=0;		scope_gun.y=-8;		scope_gun.z=0;
}

//**************************************************************************//


//*******************************UART�ص�����********************************//

void AutoAimUartRxCpltCallback()
{
	//�������ݽ���
	if(!find_enemy&&RX_ENEMY_START=='s'&&RX_ENEMY_END=='e')
	{
		enemy_scope.x=(float)((RX_ENEMY_X1<<8)|RX_ENEMY_X2)*k_coordinate;
		enemy_scope.y=(float)((RX_ENEMY_Y1<<8)|RX_ENEMY_Y2)*k_coordinate;
		enemy_scope.z=(float)((RX_ENEMY_Z1<<8)|RX_ENEMY_Z2)*k_distance;
		enemy_scope.x=(enemy_scope.x>coordinate_max)?(enemy_scope.x-2*coordinate_max):enemy_scope.x;
		enemy_scope.y=(enemy_scope.y>coordinate_max)?(enemy_scope.y-2*coordinate_max):enemy_scope.y;
		find_enemy=1;
		receive_cnt++;
	}
	
	HAL_UART_Receive_DMA(&AUTOAIM_UART,(uint8_t *)&Enemy_INFO,8);
}
//***************************************************************************//


//****************************************����Ƕ�ת������*************************************//

//��ʱ���ж��з�Ƶ����øú���
void EnemyINFOProcess()
{
	//����ת��
	enemy_gun.x=enemy_scope.x+scope_gun.x;
	enemy_gun.y=enemy_scope.y+scope_gun.y;
	enemy_gun.z=enemy_scope.z+scope_gun.z;
	
	//�Ƕȼ��㣨���������ڴ�϶࣬���ܷ���2ms���µ�ʱ���ж���ִ�У�
	aim.y=atan(enemy_gun.x/(enemy_gun.z*cos(GMP_ANGLE)-enemy_gun.y*sin(GMP_ANGLE)))/const_pi*180.0-yaw_adjust;
	aim.p=atan(enemy_gun.y/enemy_gun.z)/const_pi*180.0+pitch_adjust;
}

//*********************************************************************************************//


//**************************��ͨģʽ������ƺ���****************************//

void AutoAimNormal()
{
	MINMAX(aim.y,-4.0f,4.0f);
	MINMAX(aim.p,-2.0f,2.0f);
	if(find_enemy)
	{
		if(aim_cnt<1)
		{
			GMY.Target+=(aim.y+aim_rcd.y)/2;
			GMP.Target+=(aim.p+aim_rcd.p)/2;
			aim_cnt++;
		}
		else
		{
			find_enemy=0;
			aim_cnt=0;
			aim_rcd.y=aim.y;
			aim_rcd.p=aim.p;
		}
	}
}

//**************************************************************************//


//**************************���ģʽ������ƺ���****************************//

void AutoAimBuff()
{
	if(find_enemy)
	{
		//��̨ת��Ŀ�귽��
		if(aim_cnt==0)
		{
			aim_rcd.y=aim.y;
			aim_rcd.p=aim.p;
			GMY.Target+=aim_rcd.y/20;
			GMP.Target+=aim_rcd.p/20;
			aim_cnt++;
		}
		else if(aim_cnt>=1 && aim_cnt<20)
		{
			GMY.Target+=aim_rcd.y/20;
			GMP.Target+=aim_rcd.p/20;
			aim_cnt++;
		}
		//�ȴ���̨�ȶ�
		else if(aim_cnt>=20 && aim_cnt<30)
		{
			aim_cnt++;
		}
		//����
		else if(aim_cnt==30)
		{
			//ShootOneBullet();
			aim_cnt++;
		}
		//�ȴ���̨�ȶ�
		else if(aim_cnt>30 && aim_cnt<50)
		{
			aim_cnt++;
		}
		//����ָ������
		else if(aim_cnt>=50 && aim_cnt<70)
		{
			GMY.Target-=aim_rcd.y/20;
			GMP.Target-=aim_rcd.p/20;
			aim_cnt++;
		}
		else
		{
			aim_cnt=0;
			find_enemy=0;
		}
	}
}

//**************************************************************************//


//***********************************�������*******************************//

void AutoAimGMCTRL()
{
	switch(aim_mode)
	{
		case 1: AutoAimNormal(); break;		//����
		case 2: AutoAimBuff(); break;			//���
		case 3: break;										//���䣿
		default: break;
	}
	
	//************���֡��*************
	if(receive_cnt==0)
		auto_counter=1000;
	if(auto_counter==0)
	{
		receive_rcd=receive_cnt;
		receive_cnt=0;
		auto_counter=1000;
	}
	//*********************************
}

//**************************************************************************//


#endif /*USE_AUTOAIM*/
#endif /*DEBUG_MODE*/
