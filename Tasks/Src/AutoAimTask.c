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
GMAngle_t aim,abt,abtLast,opt,jst;
uint8_t Enemy_INFO[8],Tx_INFO[8];						//����
uint8_t FindEnemy,AimMode;
int16_t AimTic=1;
GMAngle_t aimProcess(float yaw,float pit,int16_t *tic);
//********************************�����ʼ��********************************//
void InitAutoAim(){
	//����AUTO_AIM_UART��DMA����
	if(HAL_UART_Receive_DMA(&AUTOAIM_UART,(uint8_t *)&Enemy_INFO,8)!= HAL_OK){Error_Handler();}
	//�Ƕȱ�����ʼ��������Ҫ�޸ģ�
	aim.yaw=0;				aim.pit=0;
	jst.yaw=1;			jst.pit=0;
}
//*******************************UART�ص�����********************************/
//float rate1=2.7,rate2=2.85;
float tmpY,tmpP;
void AutoAimUartRxCpltCallback(){
	if(RX_ENEMY_START=='s'&&RX_ENEMY_END=='e'){
		onLed(5);
		aim.yaw=(int16_t)((RX_ENEMY_YAW1<<8)|RX_ENEMY_YAW2)*kAngle;
		aim.pit=-(int16_t)((RX_ENEMY_PITCH1<<8)|RX_ENEMY_PITCH2)*kAngle;
		aim.dis=(int16_t)((RX_ENEMY_DIS1<<8)|RX_ENEMY_DIS2);//0�ޣ�500��ʶ��1000������2000������װ�ף�3000������װ�ף�4000������װ�ף�5000�����⣬6000����
		if(aim.dis==0){
			HAL_UART_Receive_DMA(&AUTOAIM_UART,(uint8_t*)&Enemy_INFO,8);
			return;
		}
		aim.yaw/=2.7;
		aim.pit/=2.83;
		abt.yaw=GMY.Real+aim.yaw-jst.yaw;
		abt.pit=GMP.Real+aim.pit+jst.pit;
		if(abt.pit>-12 && GMY.encoderAngle>0){AimMode=5;}//���Զ��
		if(abt.pit<-13){AimMode=0;}
		if(abt.pit<-2){
//=========Ԥ��===========
			if(aim.dis==500 || aim.dis==1000 || aim.dis==6000){opt=aimProcess(abt.yaw,abt.pit,&AimTic);}
//========================
			else{
				if(aim.dis==2000){//��������װ��
					abt.yaw=abtLast.yaw*0.6+abt.yaw*0.4;
					abt.pit=abtLast.pit*0.7+abt.pit*0.3;
				}else if(aim.dis==3000){//�����Ҳ��װ��,yawΪ��
					abt.pit=abtLast.pit*0.7+abt.pit*0.3;
					abt.yaw=abtLast.yaw*0.6+(abt.yaw-5*sin(abt.pit))*0.4;
				}else if(aim.dis==4000){//��������װ��
					abt.pit=abtLast.pit*0.7+abt.pit*0.3;
					abt.yaw=abtLast.yaw*0.7+(abt.yaw+5*sin(abt.pit))*0.3;
				}
//========�ٶȲ���========
				if(AimMode){
					opt.yaw=abt.yaw-ChaSpdSin /19;//19.3,20
					opt.pit=abt.pit-ChaSpdCos /18;//18
				}else{
					opt.yaw=abt.yaw-ChaSpdSin /14.3;//19.3
					opt.pit=abt.pit-ChaSpdCos /14.3;
				}
//========================
			}
			FindEnemy=1;
		}
		abtLast=abt;
	}
	HAL_UART_Receive_DMA(&AUTOAIM_UART,(uint8_t*)&Enemy_INFO,8);
}

//**************************��ͨģʽ������ƺ���****************************//
void autoAim(){
	if(AimMode){opt.pit-=12/opt.pit+0.5;}//5m
	else if(GMY.encoderAngle<0){opt.pit-=10/opt.pit-0.5;}
	else{opt.pit-=30/opt.pit+0.5;}//��׹����
	GMY.Target=opt.yaw;
	GMP.Target=opt.pit;
	FindEnemy=0;
}
//float wy,wp;
GMAngle_t aimProcess(float yaw,float pit,int16_t *tic){
/*@������������Ԥ�⣬�����ݣ��˶�����
	���������ԽǶ�yaw��pit����ʱ����ַ
	����˼�룺
	1.�Ӿ�������Ҫ����ʵ�Ƕȱ궨���������ΪĿ����ԽǶȣ���ֳ��ٶ�
	2.�Ӿ����ݴ�������һ��ʱ��������
	3.�ٶ��ۼ���ָ��˥���Կ�
	4.�ж����ݣ��ٶ���aim��С��һ���������ƴγ���3
	5.�����ݣ���Ȩƽ��
	6.�����˶���������functionTask
*/
	#define amt 6	//��������amount������amtʹʱ������ԼΪ50ms���� amt=50ms/1000ms*fps
	static int8_t i,lock,whipCnt,cnt;	//index���������״ν��뱣�����������ж����������ü���
	static float 	y[amt],p[amt],			//yaw,pit��ʷ
								tSum,t[amt],				//���ʱ��,tic��ʷ
								wy,wp,							//yaw,pit���ٶ�
								wySum,wpSum,				//���ٶ��ۼӶԿ�
								dYaw;								//����yaw��ֵ���ж�������
	static GMAngle_t in,out;					//��һ��ֵ������ֵ�Ƕ�
	tSum+=*tic-t[i];	//��pid��i�������һ�ޣ����ϱ��β���ȥamt����ǰ��ʱ�������õ���Ƶ��ļ��
	cnt++;
	if(*tic>180){			//if��������ʱ��������XXms�������ʷ�����뱣����
		lock=amt;
		wy=0;wp=0;dYaw=yaw;
		wySum=0;wpSum=0;
		whipCnt=0;
		in.yaw=yaw;in.pit=pit;in.dis=pit;
		out=in;
	}
	in.yaw=in.yaw*0.7+yaw*0.3;//����ֵ�˲�
	in.pit=in.pit*0.7+pit*0.3;
	if(lock){			//�����״ν��뱣����ֻ��¼���ݲ�Ԥ��
		lock--;
		if(AimMode){
			wySum=-ChaSpdSin /610;
			wpSum=-ChaSpdCos /327;
		}else{
			wySum=-ChaSpdSin /434;
			wpSum=-ChaSpdCos /222;
		}
	}
	else{
		//�ж����ݣ�Ӧ���ڱ��μ���wySum֮ǰ
		dYaw=in.yaw-dYaw;
		if(dYaw*wySum<-0.35){
			if(whipCnt<20){whipCnt++;}
			cnt=0;
		}else if(cnt>120){
			cnt=0;
			whipCnt=0;
		}
//==============
//		wy=(wy+(in.yaw-y[i])*50/tSum)/2;	//�ٶ��˲�,*45������Ԥ��ʱ�䣬�ŵ�������߳�������
//		wp=(wp+(in.pit-p[i])*50/tSum)/2;
//==============	
		wy=(wy+(in.yaw-y[i])/tSum)/2;	//�ٶ��˲�
		wp=0.7*wp+0.3*(in.pit-p[i])/tSum;
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
//==============
//		out.yaw=(in.yaw-wy/sin(in.pit/57.3)+out.yaw)/2;
//		out.pit=(in.pit+wp+out.pit)/2;
//==============	
		out.yaw=0.4*(in.yaw+wySum*26)+ 0.6*out.yaw;
		out.dis=0.3*(in.pit+wpSum*10)+ 0.7*out.pit;
		out.yaw-=ChaSpdSin/100;
		out.pit=out.dis-ChaSpdCos/40;
	}else{//�����ݣ���Ȩֵ�˲�
		out.yaw=0.8*out.yaw+0.2*(in.yaw-ChaSpdSin /19);//19.3
		out.pit=0.8*out.pit+0.2*(in.pit-ChaSpdCos /18);
	}
//3.14x8x0.09x8/60/21= 1/557
	return out;
}
//��¼��̨�Ƕ���ʷ
GMAngle_t GMAngleRcd(){
	#define rcdAmt 30
	static GMAngle_t GMAngleRcd[rcdAmt];
	static uint8_t i = 0;
	GMAngleRcd[i].yaw = GMY.Real;
	GMAngleRcd[i].pit = GMP.Real;
	i=(i+1)%rcdAmt;
	return GMAngleRcd[i];
}

#endif /*USE_AUTOAIM*/
#endif /*DEBUG_MODE*/

