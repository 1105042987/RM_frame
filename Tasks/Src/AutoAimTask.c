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
GMAngle_t aimProcess(float yaw,float pit,int16_t *tic);
void fallCompsition();
typedef struct{
	float x,v,k1,k2,t,R;
	float q1,q2,p11,p12,p21,p22;
	float t11,t12,t21,t22;//tmp
}kalman_t;
#define kalmanInit \
{ 0,0,0,0,1,1,\
	0.1,0.1,1,0,0,1,\
	0,0,0,0 \
}
kalman_t kFilter=kalmanInit;
float kalmanCalc(kalman_t *f,float z,int t){
	//x=A*x0
	f->x += f->v * f->t;
	//P=APA'+Q
	f->t11 = f->p11+ (f->p12+ f->p21+ f->p22* f->t)* f->t+ f->q1;
	f->t12 = f->p12+ f->p22* f->t;
	f->t21 = f->p21+ f->p22* f->t;
	f->t22 = f->p22+ f->q2;
	//K=PH/(HPH'+R)
	f->k1= f->t11/ (f->t11+ f->R);
	f->k2= f->t12/ (f->t11+ f->R);
	//x=x+K(z-Hx)
	f->x += f->k1*(z- f->x);
	f->v += f->k2*(z- f->x);
	//P=(1-KH)PH
	f->p11=(1- f->k1)* f->t11;
	f->p12=(1- f->k1)* f->t12;
	f->p21=f->t21- f->k2* f->t11;
	f->p22=f->t22- f->k2* f->t12;
	return f->x + f->v *t;
}
//*****************************************��������******************************************//

GMAngle_t aim,aimLast,opt,tmp;//optimize					//Ŀ��Ƕ�
GMAngle_t adjust;															//У׼�������
uint8_t Enemy_INFO[8],Tx_INFO[8];							//����
uint8_t findEnemy=0,aimMode=0,upper_mode;			//aimMode����ѡ����׼ģʽ��0Ϊ�ֶ���׼��1Ϊ�������飬2Ϊ�����3���ޣ����䣿��

uint16_t aimCnt=0;														//�����Ƶ��ʱ����
int16_t receiveCnt=0,receiveFps=0,AimTic=1;		//�����λ���ź�֡��
extern int16_t receiveCnt,receiveFps;
int8_t trackCnt=0;														//׷�ٱ���

//********************************�����ʼ��********************************//

void InitAutoAim(){
	//����AUTO_AIM_UART��DMA����
	if(HAL_UART_Receive_DMA(&AUTOAIM_UART,(uint8_t *)&Enemy_INFO,8)!= HAL_OK){Error_Handler();}
	//�Ƕȱ�����ʼ��������Ҫ�޸ģ�
	aim.yaw=0;				aim.pit=0;
	adjust.yaw=0;			adjust.pit=0;
}
//*******************************UART�ص�����********************************//

void AutoAimUartRxCpltCallback(){
	if(RX_ENEMY_START=='s'&&RX_ENEMY_END=='e'){
		onLed(6);
		aim.yaw=-(int16_t)((RX_ENEMY_YAW1<<8)|RX_ENEMY_YAW2)*kAngle;
		aim.pit=(int16_t)((RX_ENEMY_PITCH1<<8)|RX_ENEMY_PITCH2)*kAngle;
		aim.dis=(int16_t)((RX_ENEMY_DIS1<<8)|RX_ENEMY_DIS2);
		aim.yaw/=2.7;
		aim.pit/=2.6;
//		aim.yaw=(GMY.Real+aim.yaw+aimLast.yaw)/2;
//		aim.pit=(GMP.Real+aim.pit+aimLast.pit)/2;
//		aimLast=aim;
//		tmp.yaw=(GMY.Real+aim.yaw+aimLast.yaw)/2;
//		tmp.pit=(GMP.Real+aim.pit+aimLast.pit)/2;
//		aimLast=tmp;
		tmp.yaw=GMY.Real+aim.yaw;
		tmp.pit=GMP.Real+aim.pit;
		if(GMP.Real+aim.pit<-15){
			opt=aimProcess(tmp.yaw,tmp.pit,&AimTic);
			findEnemy=1;
		}
	}
	HAL_UART_Receive_DMA(&AUTOAIM_UART,(uint8_t*)&Enemy_INFO,8);
}

//**************************��ͨģʽ������ƺ���****************************//
void autoAim(){
	GMY.Target=opt.yaw+2;
//	GMP.Target=GMP.Real+aim.pit*0.65-aimLast.pit*0.2;
	GMP.Target=opt.pit+3;
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

float test1,test2,test3;
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
								wySum,wpSum;	//���ٶ��ۼӶԿ�
	static GMAngle_t in,out;		//��һ��ֵ������ֵ�Ƕ�
	tSum+=*tic-t[i];	//��pid��i�������һ�ޣ����ϱ��β���ȥamt����ǰ��ʱ�������õ���Ƶ��ļ��
	if(*tic>150){			//if��������ʱ��������100*2ms�������ʷ�����뱣����
		lock=amt;
		wy=0;wp=0;
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
	test1=in.pit;
	test2=out.pit;
	test3=wpSum;
	out.yaw=(in.yaw+wySum*20+out.yaw)/2;		//ʵ��Ԥ��
	out.pit=(in.pit+wpSum*10+out.pit)/2;
//	angle.pit-=40/angle.pit-0.4;//������׹����
	*tic=1;			//ʱ���жϼ�ʱ�����¿�ʼ
	return out;
}
#endif /*USE_AUTOAIM*/
#endif /*DEBUG_MODE*/

