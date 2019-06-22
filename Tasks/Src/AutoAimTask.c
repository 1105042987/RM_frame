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

GMAngle_t aim,aimLast,opt;//optimize													//Ŀ��Ƕ�
GMAngle_t adjust;																							//У׼�������
Coordinate_t enemy_gun,enemyScope,scopeGun;										//����
uint8_t Enemy_INFO[8],Tx_INFO[8];															//����
uint8_t findEnemy=0,aimMode=0,upper_mode;											//aimMode����ѡ����׼ģʽ��0Ϊ�ֶ���׼��1Ϊ�������飬2Ϊ�����3���ޣ����䣿��

uint16_t aimCnt=0;																						//�����Ƶ��ʱ����
int16_t receiveCnt=0,receiveFps=0,AimTic=1;											//�����λ���ź�֡��
extern int16_t receiveCnt,receiveFps;
int8_t trackCnt=0;																						//׷�ٱ���

//********************************�����ʼ��********************************//

void InitAutoAim(){
	//����AUTO_AIM_UART��DMA����
	if(HAL_UART_Receive_DMA(&AUTOAIM_UART,(uint8_t *)&Enemy_INFO,8)!= HAL_OK){Error_Handler();}
	//���������ʼ��������Ҫ�޸ģ�
	enemyScope.x=0;	enemyScope.y=0;	enemyScope.z=200;
	enemy_gun.x=0;		enemy_gun.y=0;		enemy_gun.z=200;
	//�Ƕȱ�����ʼ��������Ҫ�޸ģ�
	aim.yaw=0;				aim.pit=0;
	adjust.yaw=0;			adjust.pit=0;
	//���������ʼֵ�����ݲ�ͬ��װ���������3��������
	scopeGun.x=0;		scopeGun.y=-10;		scopeGun.z=0;
}
//*******************************UART�ص�����********************************//
void AutoAimUartRxCpltCallback(){
	#ifndef USE_AUTOAIM_ANGLE
	//�������ݽ���
	if(RX_ENEMY_START=='s'&&RX_ENEMY_END=='e')
	{
		enemyScope.x=(float)((RX_ENEMY_X1<<8)|RX_ENEMY_X2)*k_coordinate;
		enemyScope.y=(float)((RX_ENEMY_Y1<<8)|RX_ENEMY_Y2)*k_coordinate;
//		enemyScope.z=(float)((RX_ENEMY_Z1<<8)|RX_ENEMY_Z2)*k_distance;
		enemyScope.z=350;
		enemyScope.x=(enemyScope.x>coordinate_max)?(enemyScope.x-2*coordinate_max):enemyScope.x;
		enemyScope.y=(enemyScope.y>coordinate_max)?(enemyScope.y-2*coordinate_max):enemyScope.y;
		findEnemy=1;
		receiveCnt++;
	}
	#else
	if(RX_ENEMY_START=='s'&&RX_ENEMY_END=='e'){
		onLed(6);
//		aim.yaw=(float)(( (((RX_ENEMY_YAW1<<8)|RX_ENEMY_YAW2)>0x7fff) ? (((RX_ENEMY_YAW1<<8)|RX_ENEMY_YAW2)-0xffff) : (RX_ENEMY_YAW1<<8)|RX_ENEMY_YAW2 )*kAngle);
//		aim.pit=-9-(float)(( (((RX_ENEMY_PITCH1<<8)|RX_ENEMY_PITCH2)>0x7fff) ? (((RX_ENEMY_PITCH1<<8)|RX_ENEMY_PITCH2)-0xffff) : (RX_ENEMY_PITCH1<<8)|RX_ENEMY_PITCH2 )*kAngle);
//		aim.dis=(float)(( (((RX_ENEMY_DIS1<<8)|RX_ENEMY_DIS2)>0x7fff) ? (((RX_ENEMY_DIS1<<8)|RX_ENEMY_DIS2)-0xffff) : (RX_ENEMY_DIS1<<8)|RX_ENEMY_DIS2 ));
		aim.yaw=(int16_t)((RX_ENEMY_YAW1<<8)|RX_ENEMY_YAW2)*kAngle;
		aim.pit=-10-(int16_t)((RX_ENEMY_PITCH1<<8)|RX_ENEMY_PITCH2)*kAngle;
		aim.dis=(int16_t)((RX_ENEMY_DIS1<<8)|RX_ENEMY_DIS2);
		aim.yaw/=1.6975;
		aim.pit/=1.3976;
		
		aim.abs=(GMY.Real+aim.yaw+aim.absLast)/2;
//		aim.abs=(GMY.Real+aim.yaw);
		aim.absLast=aim.abs;
//		adjust.pit=GMP.Real*0.16+0.5;
		if(GMP.Real+aim.pit<3){findEnemy=1;}
//		MINMAX(aim.yaw,-10,10);
//		MINMAX(aim.pit,-8,8);
		if(AimTic<50){findEnemy=0;}
		else if(AimTic<100){
			opt.wz=(opt.wz+(aim.abs-opt.abs)/AimTic)/2;
			opt.abs=(opt.abs+opt.wz*AimTic+aim.abs)/2;
			AimTic=1;
		}else{
			opt.abs=aim.abs;
			opt.wz*=0.8;
			AimTic=1;
		}
	}
	#endif
	HAL_UART_Receive_DMA(&AUTOAIM_UART,(uint8_t*)&Enemy_INFO,8);
}
//****************************************����Ƕ�ת������*************************************//

//��ʱ���ж��з�Ƶ����øú���
void EnemyINFOProcess(){
	#ifndef USE_AUTOAIM_ANGLE
	//����ת��
	enemy_gun.x=enemyScope.x+scopeGun.x;
	enemy_gun.x=enemyScope.y+scopeGun.y;
	enemy_gun.z=enemyScope.z+scopeGun.z;
	
	//�Ƕȼ��㣨���������ڴ�϶࣬���ܷ���2ms���µ�ʱ���ж���ִ�У�
	aim.yaw=atan(enemy_gun.x/(enemy_gun.z*cos(GMP_ANGLE)-enemy_gun.y*sin(GMP_ANGLE)))/constPi*180.0-adjust.yaw;
	aim.pit=atan(enemy_gun.y/enemy_gun.z)/constPi*180.0+adjust.pit;
	#endif
	
	//׷��
	if(((aim.yaw>0 && aimLast.yaw>0) || (aim.yaw<0 && aimLast.yaw<0))){
		trackCnt++;
		MINMAX(trackCnt,0,100);
	}
	else{
		trackCnt=0;
	}
}

//**************************��ͨģʽ������ƺ���****************************//
float tmp=30;
float wzSum,wzLast;
void autoAim(){
//	GMY.Target=GMY.Real+(double)aim.yaw*0.5;//sgn(aim.yaw)*0.5*sqrt(fabs(aim.yaw));//(aim.yaw+aimLast.yaw)/kk;//8;
//	GMP.Target=GMP.Real+(double)aim.pit*0.4;//(sgn(aim.pit)*(aim.pit*aim.pit)+aim.pit)*0.3;//(aim.pitch+aimLast.pitch)/kk;//8;

	wzSum+=opt.wz;
	wzSum*=0.85;
//	GMY.Target=opt.abs+opt.wz*tmp;
	GMY.Target=opt.abs+wzSum*tmp;
	wzLast=opt.wz;
	GMP.Target=GMP.Real+aim.pit*0.65-aimLast.pit*0.2;

	aimLast.yaw=aim.yaw;
	aimLast.pit=aim.pit;
	
//	wy=GMY.Real+aim.yaw;
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

#endif /*USE_AUTOAIM*/
#endif /*DEBUG_MODE*/

