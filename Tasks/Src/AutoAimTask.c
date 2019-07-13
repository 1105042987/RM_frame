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
#define USE_AUTOAIM_ANGLE
//*****************************************��������******************************************//
GMAngle_t aim,opt,adjust;										//У׼�������
uint8_t Enemy_INFO[8],Tx_INFO[8];						//����
uint8_t findEnemy=0,aimMode=0,upper_mode;		//aimMode����ѡ����׼ģʽ��0Ϊ�ֶ���׼��1Ϊ�������飬2Ϊ�����3���ޣ����䣿��
int16_t AimTic=1;
GMAngle_t aimProcess(float yaw,float pit,int16_t *tic);
//********************************�����ʼ��********************************//
void InitAutoAim(){
	//����AUTO_AIM_UART��DMA����
	if(HAL_UART_Receive_DMA(&AUTOAIM_UART,(uint8_t *)&Enemy_INFO,8)!= HAL_OK){Error_Handler();}
	//�Ƕȱ�����ʼ��������Ҫ�޸ģ�
	aim.yaw=0;				aim.pit=0;
	adjust.yaw=0;			adjust.pit=0;
}
//*******************************UART�ص�����********************************//
//float rate1=4.1,rate2=4.49;
float rate1=2.7,rate2=2.85;
void AutoAimUartRxCpltCallback(){
	if(RX_ENEMY_START=='s'&&RX_ENEMY_END=='e'){
		onLed(5);
		aim.yaw=(int16_t)((RX_ENEMY_YAW1<<8)|RX_ENEMY_YAW2)*kAngle;
		aim.pit=-(int16_t)((RX_ENEMY_PITCH1<<8)|RX_ENEMY_PITCH2)*kAngle;
		aim.dis=(int16_t)((RX_ENEMY_DIS1<<8)|RX_ENEMY_DIS2);
		aim.yaw/=rate1;
		aim.pit/=rate2;
		if(GMP.Real+aim.pit<5){
//			opt.yaw=opt.yaw*0.9+tmp.yaw*0.1;
//			opt.pit=opt.pit*0.9+tmp.pit*0.1;
			opt=aimProcess(GMY.Real+aim.yaw, GMP.Real+aim.pit, &AimTic);
			findEnemy=1;
		}
	}
	HAL_UART_Receive_DMA(&AUTOAIM_UART,(uint8_t*)&Enemy_INFO,8);
}

//**************************��ͨģʽ������ƺ���****************************//
void autoAim(){
	GMY.Target=opt.yaw-2;
//	GMP.Target=GMP.Real+aim.pit*0.65-aimLast.pit*0.2;
	GMP.Target=opt.pit-1;
	findEnemy=0;
}

//***************************��λ������ģʽ�л�*****************************//
void UpperStateFSM(){
	if(upper_mode != aimMode && upper_mode != 0){
		Tx_INFO[0] = 'c';
		Tx_INFO[1] = aimMode;
		Tx_INFO[2] = 'e';
		HAL_UART_Transmit(&AUTOAIM_UART,Tx_INFO,3,0xff);
		upper_mode = aimMode;
	}
}
float wySum;
GMAngle_t aimProcess(float yaw,float pit,int16_t *tic){
/*@������������Ԥ�⣬��׹�����������ݣ��˶���������functionTask��
	���������ԽǶ�yaw��pit����ʱ����ַ
	����˼�룺
	1.�Ӿ�������Ҫ����ʵ�Ƕȱ궨���������ΪĿ����ԽǶȣ���ֳ��ٶ�
	2.�Ӿ����ݴ�������һ��ʱ��������(��������ʵ��ѧ���ģ��߿��ؿ�����)
	3.��ѭ�����ٶ������ۼ���ָ��˥���Կ�
	4.����pitch��������׹����
	5.�ж����ݣ��ٶ���aim��С��һ���������ƴγ���3
	6.�����ݣ���Ȩƽ��
	7.�����˶���������functionTask
*/
	#define amt 5	//��������amount������amtʹʱ������ԼΪ50ms���� amt=50ms/1000ms*fps
	static int8_t i,lock,whipCnt,cnt;	//index���������״ν��뱣�����������ж����������ü���
	static float 	y[amt],p[amt],			//yaw,pit��ʷ
								tSum,t[amt],				//���ʱ��,tic��ʷ
								wy,wp,							//yaw,pit���ٶ�
	//							wySum,
	wpSum;				//���ٶ��ۼӶԿ�
	static GMAngle_t in,out;					//��һ��ֵ������ֵ�Ƕ�
	tSum+=*tic-t[i];	//��pid��i�������һ�ޣ����ϱ��β���ȥamt����ǰ��ʱ�������õ���Ƶ��ļ��
	cnt++;
	if(*tic>160){			//if��������ʱ��������150*2ms�������ʷ�����뱣����
		lock=amt;
		wy=0;wp=0;
		whipCnt=0;
		in.yaw=yaw;in.pit=pit;
		out.yaw=yaw;out.pit=pit;
	}
	in.yaw=(yaw+in.yaw)/2;//����ֵ�˲�
	in.pit=(pit+in.pit)/2;
	if(lock){lock--;}			//�����״ν��뱣����ֻ��¼���ݲ�Ԥ��
	else{
		//�ж����ݣ�Ӧ���ڱ��μ���wySum֮ǰ
		float tmpD=in.yaw-out.yaw;
		if(tmpD*wySum<-1.5 && fabs(tmpD)>8){
			if(whipCnt<20){whipCnt++;}
			cnt=0;
		}else if(cnt>120){
			cnt=0;
			whipCnt=0;
		}
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
	*tic=1;				//ʱ���жϼ�ʱ�����¿�ʼ
//	angle.pit-=40/angle.pit-0.4;//������׹����
	
	if(whipCnt<5){//Ԥ�⣬�˲�
		out.yaw=(in.yaw+wySum*18+out.yaw+out.yaw)/3;
		out.pit=(in.pit+wpSum*8+out.pit+out.pit)/3;
		out.dis=1;
	}else{//�����ݣ���Ȩֵ�˲�������Ԥ��
		out.yaw=out.yaw*0.9+in.yaw*0.1-wySum*2;
		out.pit=out.pit*0.9+in.pit*0.1-wpSum;
		out.dis=2;
	}
	return out;
}
#endif /*USE_AUTOAIM*/
#endif /*DEBUG_MODE*/

