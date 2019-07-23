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
GMAngle_t aim,abt,opt,jst;										//У׼�������
uint8_t Enemy_INFO[8],Tx_INFO[8];						//����
uint8_t FindEnemy=0;
int16_t AimTic=1;
GMAngle_t aimProcess(float yaw,float pit,int16_t *tic);
//********************************�����ʼ��********************************//
void InitAutoAim(){
	//����AUTO_AIM_UART��DMA����
	if(HAL_UART_Receive_DMA(&AUTOAIM_UART,(uint8_t *)&Enemy_INFO,8)!= HAL_OK){Error_Handler();}
	//�Ƕȱ�����ʼ��������Ҫ�޸ģ�
	aim.yaw=0;				aim.pit=0;
	jst.yaw=2;			jst.pit=0.5;
}
//*******************************UART�ص�����********************************/
//float rate1=2.7,rate2=2.85;
void AutoAimUartRxCpltCallback(){
	if(RX_ENEMY_START=='s'&&RX_ENEMY_END=='e'){
		onLed(5);
		aim.yaw=(int16_t)((RX_ENEMY_YAW1<<8)|RX_ENEMY_YAW2)*kAngle;
		aim.pit=-(int16_t)((RX_ENEMY_PITCH1<<8)|RX_ENEMY_PITCH2)*kAngle;
		aim.dis=(int16_t)((RX_ENEMY_DIS1<<8)|RX_ENEMY_DIS2);
		aim.yaw/=2.7;
		aim.pit/=2.85;
		abt.yaw=GMY.Real+aim.yaw;
		abt.pit=GMP.Real+aim.pit;
		//if(abt.pit<0){
			if(aim.dis==2000){//��������װ��
				opt.yaw=opt.yaw*0.6+abt.yaw*0.4;
				opt.pit=opt.pit*0.6+abt.pit*0.4;
//				opt.yaw-=sgn(GMY.encoderAngle)*(((uint16_t)(receiveData[0].data[0])>>8) -127)/200.0;
			}else if(aim.dis==3000){//���ݲ��װ��
				opt.yaw=opt.yaw*0.6+(GMY.Real+sgn(aim.yaw)*(fabs(aim.yaw)-3))*0.4;
				opt.pit=opt.pit*0.6+abt.pit*0.4;
//				opt.yaw-=sgn(GMY.encoderAngle)*(((uint16_t)(receiveData[0].data[0])>>8) -127)/200.0;
			}
			else{opt=aimProcess(abt.yaw,abt.pit,&AimTic);}
			FindEnemy=1;
	//	}
	}
	HAL_UART_Receive_DMA(&AUTOAIM_UART,(uint8_t*)&Enemy_INFO,8);
}

//**************************��ͨģʽ������ƺ���****************************//
float test=2;
void autoAim(){
	GMY.Target=opt.yaw-jst.yaw;
//	GMP.Target=GMP.Real+aim.pit*0.65-aimLast.pit*0.2;
	GMP.Target=opt.pit+jst.pit;
	
	GMY.Target-=((int16_t)((uint16_t)(receiveData[0].data[0])>>8) -127)/557.0 * fabs(sin((GMY.encoderAngle-aim.yaw)/180*3.14))*sgn(GMY.encoderAngle);
	GMP.Target+=((int16_t)((uint16_t)(receiveData[0].data[0])>>8) -127)/800 * fabs(cos((GMY.encoderAngle-aim.yaw)/180*3.14))*sgn(GMY.encoderAngle);

	if(GMP.Target>-5){GMP.Target+=test;}
	else{GMP.Target-=15/GMP.Target+0.5;}

	FindEnemy=0;
}


float wySum,dy;
GMAngle_t aimProcess(float yaw,float pit,int16_t *tic){
/*@������������Ԥ�⣬�����ݣ��˶�����
	���������ԽǶ�yaw��pit����ʱ����ַ
	����˼�룺
	1.�Ӿ�������Ҫ����ʵ�Ƕȱ궨���������ΪĿ����ԽǶȣ���ֳ��ٶ�
	2.�Ӿ����ݴ�������һ��ʱ��������(��������ʵ��ѧ���ģ��߿��ؿ�����)
	3.��ѭ�����ٶ������ۼ���ָ��˥���Կ�
	4.�ж����ݣ��ٶ���aim��С��һ���������ƴγ���3
	5.�����ݣ���Ȩƽ��
	6.�����˶���������functionTask
*/
	#define amt 5	//��������amount������amtʹʱ������ԼΪ50ms���� amt=50ms/1000ms*fps
	static int8_t i,lock,whipCnt,cnt;	//index���������״ν��뱣�����������ж����������ü���
	static float 	y[amt],p[amt],			//yaw,pit��ʷ
								tSum,t[amt],				//���ʱ��,tic��ʷ
								wy,wp,							//yaw,pit���ٶ�
	//							wySum,
	wpSum,				//���ٶ��ۼӶԿ�
								dYaw;								//����yaw��ֵ���ж�������
	static GMAngle_t in,out;					//��һ��ֵ������ֵ�Ƕ�
	tSum+=*tic-t[i];	//��pid��i�������һ�ޣ����ϱ��β���ȥamt����ǰ��ʱ�������õ���Ƶ��ļ��
	cnt++;
	if(*tic>160){			//if��������ʱ��������150*2ms�������ʷ�����뱣����
		lock=amt;
		wy=0;wp=0;dYaw=yaw;
		wySum=0;wpSum=0;
		whipCnt=0;
		in.yaw=yaw;in.pit=pit;
		out=in;
	}
	in.yaw=(yaw+in.yaw)/2;//����ֵ�˲�
	in.pit=(pit+in.pit)/2;
	if(lock){lock--;}			//�����״ν��뱣����ֻ��¼���ݲ�Ԥ��
	else{
		//�ж����ݣ�Ӧ���ڱ��μ���wySum֮ǰ
		dYaw=in.yaw-dYaw;
		dy=dYaw;
		if(dYaw*wySum<-0.35){
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
	dYaw=in.yaw;	//�ݴ��ϴ�yaw�������´μ���yaw��ֵ
	y[i]=in.yaw;	//yaw��ʷ
	p[i]=in.pit;	//pit��ʷ
	t[i]=*tic;		//tic��ʷ
	i=(i+1)%amt;	//amt��֮��ѭ��
	*tic=1;				//ʱ���жϼ�ʱ�����¿�ʼ
	
	if(whipCnt<5){//Ԥ�⣬�˲�
		out.yaw=(in.yaw+wySum*25+out.yaw+out.yaw)/3;
		out.pit=(in.pit+wpSum*15+out.pit+out.pit)/3;
		out.dis=1;
	}else{//�����ݣ���Ȩֵ�˲�
		out.yaw=out.yaw*0.8+in.yaw*0.2;
		out.pit=out.pit*0.8+in.pit*0.2;
//		out.yaw-=sgn(GMY.encoderAngle)*(((uint16_t)(receiveData[0].data[0])>>8) -127)/800.0;
		out.dis=2;
	}
	//3.14x8x0.09x8/60/21= 1/557
	return out;
}
#endif /*USE_AUTOAIM*/
#endif /*DEBUG_MODE*/

