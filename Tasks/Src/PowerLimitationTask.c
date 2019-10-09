#include "includes.h"
#include "math.h"

//底盘功率限制
int powLmtCnt=500;
int8_t LimitRate=1;
#ifdef USE_POWER_LIMIT
void PowerLimitation(){
	static float limitTgt=120;
	float rate;
	if(powLmtCnt){
		powLmtCnt--;
		limitTgt=90;
	}
	else if(limitTgt<120){
		limitTgt+=0.05;
	}
	rate=((float)PowerHeat.chassis_power_buffer-limitTgt)/80;
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

