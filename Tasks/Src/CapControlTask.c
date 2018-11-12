/**
  ******************************************************************************
  * File Name   : CapControlTask.c
  * Description : 超级电容开关控制程序
  * Author		: 林炳辉，赵振宇
  ******************************************************************************
  *
  * Copyright (c) 2019 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
	
#include "includes.h"
#include "math.h"

uint8_t can_power_in = 0;
CapControl_t Control_SuperCap = {0,0};
	
	
void Cap_Control(void)
{
	if(RobotState.remainHP < 1)
	{
		FUNC__CAP__CLOSE();
	}
	else
	{
		if (PowerHeatData.chassisPower < 80 && PowerHeatData.chassisPowerBuffer == 60.0f)
		{
			can_power_in = 1;
		}
		else 
		{
			if (PowerHeatData.chassisPowerBuffer < 40.0f)
			{
				can_power_in = 0;
			}
		}
		if (Control_SuperCap.stop_power)
		{
			  FUNC__CAP__CLOSE();
		}
		else
		{
		    if (Control_SuperCap.release_power)
		    {
			    FUNC__CAP__OUTPUT();
		    }
		    else
		    {
		       if(fabs(CMFL.offical_speedPID.fdb) < 2000 && fabs(CMFR.offical_speedPID.fdb) < 2000 && fabs(CMBL.offical_speedPID.fdb) < 2000 && fabs(CMBR.offical_speedPID.fdb) < 2000 && can_power_in==1) 
			     {
				     FUNC__CAP__RECHARGE();
			     }
			     else
			     {
				     FUNC__CAP__CLOSE();
			     }
	      }
		
		}

	}
}
/**/
