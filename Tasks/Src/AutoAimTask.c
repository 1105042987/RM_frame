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
//#define AUTOAIM_DEBUG
#ifndef DEBUG_MODE
#ifdef	USE_AUTOAIM
#define USE_AUTOAIM_ANGLE
//*****************************************��������******************************************//
GMAngle_t aim,abt,opt,jst0,jst1;									//Ŀ��Ƕ�
uint8_t Enemy_INFO[8],Tx_INFO[8];								//����
uint8_t find_enemy=0,upper_mode,aim_mode;
uint16_t aim_cnt=0;															//�����Ƶ��ʱ����
int8_t track_cnt=60;														//׷�ٱ���
double rcd_yaw=0, rcd_pitch=0;
uint8_t Pre_aim=0, clb=0,numState=1,colorAim;
float attack_mark;
int16_t AimTic=1;
GMAngle_t aimProcess(float yaw,float pit,int16_t *tic);
//********************************�����ʼ��********************************//
void InitAutoAim(){
	//����AUTO_AIM_UART��DMA����
	if(HAL_UART_Receive_DMA(&AUTOAIM_UART,(uint8_t *)&Enemy_INFO,8)!= HAL_OK){
		Error_Handler();
	}
	//�Ƕȱ�����ʼ��������Ҫ�޸ģ�
	aim.yaw=0;				aim.pit=0;
	jst0.yaw=1;			jst0.pit=1;
	jst1.yaw=1;			jst1.pit=3;
}

//*******************************UART�ص�����********************************//
float rate1=10,rate2=12;
void AutoAimUartRxCpltCallback(){
	if(RX_ENEMY_START=='s'&&RX_ENEMY_END=='e'){
		aim.yaw=-(int16_t)((RX_ENEMY_YAW1<<8)|RX_ENEMY_YAW2)*k_angle;
		aim.pit=(int16_t)((RX_ENEMY_PITCH1<<8)|RX_ENEMY_PITCH2)*k_angle;
		aim.yaw/=rate1*2;
		aim.pit/=rate2*2;
		abt.yaw=GMY.RealAngle+aim.yaw;
		abt.pit=GMP.RealAngle+aim.pit;
		opt=aimProcess(GMY.RealAngle+aim.yaw, GMP.RealAngle+aim.pit, &AimTic);
		find_enemy=1;
	}
	HAL_UART_Receive_DMA(&AUTOAIM_UART,(uint8_t *)&Enemy_INFO,8);
}

//**************************��ͨģʽ������ƺ���****************************//
float py=0.5, iy=0.0, dy=0.0;
float yawAj=1,pitAj=1;
void AutoAim(){
	if(find_enemy){
//		static float yi = 0, lasty = 0;
//		yi += aim.yaw;
//		//auto attack
//		attack_mark = aim.yaw * py + yi * iy + (aim.yaw-lasty) * dy;
//		GMY.TargetAngle = GMY.RealAngle + attack_mark;  
////		GMY.TargetAngle += aim.yaw * py + yi * iy + (aim.yaw-lasty) * dy;
//		lasty = aim.yaw;
//		GMP.TargetAngle = GMP.RealAngle + aim.pit * 0.3f;
//		GMY.TargetAngle=abt.yaw;
//		GMP.TargetAngle=abt.pit;
		GMY.TargetAngle=opt.yaw;
		GMP.TargetAngle=opt.pit;
		if(AimArmor){
			GMY.TargetAngle += jst0.yaw;
			GMP.TargetAngle += jst0.pit;
		}else{
			GMY.TargetAngle += jst1.yaw;
			GMP.TargetAngle += jst1.pit;
		}
		
		find_enemy=0;
	}
}
//***************************��λ������ģʽ�л�*****************************//
uint8_t AimArmor,msgArmor[]="3\n",msgBase[]="4\n";
void UpperStateFSM(){
	static int16_t cnt;
	if(cnt>500){
//		if(GameRobotState.robot_id==7)
		if(AimArmor){HAL_UART_Transmit_IT(&AUTOAIM_UART,(uint8_t*)msgArmor,2);}
		else{HAL_UART_Transmit_IT(&AUTOAIM_UART,(uint8_t*)msgBase,2);}
		cnt=0;
	}
	cnt++;
}
float wySum;
GMAngle_t aimProcess(float yaw,float pit,int16_t *tic){
/*@������������Ԥ�⼰��׹����
	���������ԽǶ�yaw��pit����ʱ����ַ
	����˼�룺
	1.�Ӿ�������Ҫ����ʵ�Ƕȱ궨���������ΪĿ����ԽǶȣ���ֳ��ٶ�
	2.�Ӿ����ݴ�������һ��ʱ��������(��������ʵ��ѧ���ģ��߿��ؿ�����)
	3.��ѭ�����ٶ������ۼ���ָ��˥���Կ�
	4.����pitch��������׹����
*/
	#define amt 5	//��������amount������amtʹʱ������ԼΪ50ms���� amt=50ms/1000ms*fps
	static int8_t i,lock;				//���������״ν��뱣����
	static float 	y[amt],p[amt],//yaw,pit��ʷ
								tSum,t[amt],	//���ʱ��,tic��ʷ
								wy,wp,				//yaw,pit���ٶ�
								//wySum,
	wpSum;	//���ٶ��ۼӶԿ�
	static GMAngle_t in,out;		//��һ��ֵ������ֵ�Ƕ�
	tSum+=*tic-t[i];	//��pid��i�������һ�ޣ����ϱ��β���ȥamt����ǰ��ʱ�������õ���Ƶ��ļ��
	if(*tic>150){			//if��������ʱ��������150*2ms�������ʷ�����뱣����
		lock=amt;
		wy=0;wp=0;
		wySum=0;wpSum=0;
		in.yaw=yaw;in.pit=pit;
		out.yaw=yaw;out.pit=pit;
	}
	in.yaw=(yaw+in.yaw)/2;//����ֵ�˲�
	in.pit=(pit+in.pit)/2;
	if(lock){lock--;}			//�����״ν��뱣����ֻ��¼���ݲ�Ԥ��
	else{
		wy=(wy+(in.yaw-y[i])/tSum)/2;	//�ٶ��˲�
		wp=(wp+(in.pit-p[i])/tSum)/2;
		wySum+=wy;	//���ٶ��ۼ���ָ��˥���Կ�
		wpSum+=wp;
		wySum*=0.9;	//ָ��˥�������ۼ�,ʧȥ��������
		wpSum*=0.9;
	}
	y[i]=in.yaw;	//yaw��ʷ
	p[i]=in.pit;	//pit��ʷ
	t[i]=*tic;		//tic��ʷ
	i=(i+1)%amt;	//amt��֮��ѭ��
	out.yaw=(in.yaw+wySum*18+out.yaw)/2;		//ʵ��Ԥ�⣬�˲�
	out.pit=(in.pit+wpSum*8+out.pit)/2;
//	angle.pit-=40/angle.pit-0.4;//������׹����
	*tic=1;			//ʱ���жϼ�ʱ�����¿�ʼ
	return out;
}
#endif /*USE_AUTOAIM*/
#endif /*DEBUG_MODE*/
