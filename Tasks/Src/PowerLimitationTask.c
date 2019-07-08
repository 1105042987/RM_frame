#include "includes.h"
#include "math.h"

//底盘功率限制
#ifdef USE_POWER_LIMIT
int LimitCnt=500;
int8_t LimitRate=1;
void PowerLimitation(){
	static float limitTgt=120;
	float rate;
	if(LimitCnt){
		LimitCnt--;
		limitTgt=70;
	}
	else if(limitTgt<120){
		limitTgt+=0.05;
	}
	float tmp=(float)PowerHeat.chassis_power_buffer-limitTgt;
	if(tmp<180){rate=tmp/(180-limitTgt);}
	else{rate=1;}
	if(rate<0.1){rate=0.1;}
//	ChassisSpeed*=rate;
	if(LimitRate==0){
		CML.offical_speedPID.outputMax=0;
		CMR.offical_speedPID.outputMax=0;
	}
	else{
		CML.offical_speedPID.outputMax=3500*rate;
		CMR.offical_speedPID.outputMax=3500*rate;
	}
}

#endif

