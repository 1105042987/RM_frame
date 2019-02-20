#ifndef CONFIGURATION
void ControlNM(MotorINFO *id);
void ControlCM(MotorINFO *id);
void ControlANTI_CM(MotorINFO *id);
void ControlGMY(MotorINFO *id);
void ControlGMP(MotorINFO *id);

//***********************************************************************************************************
//											INFANTRY
//***********************************************************************************************************
#ifdef INFANTRY
//电机信息：
MotorINFO CMFL = SpeedBased_MOTORINFO_Init(&ControlCM,CHASSIS_MOTOR_SPEED_PID_DEFAULT);
MotorINFO CMFR = SpeedBased_MOTORINFO_Init(&ControlCM,CHASSIS_MOTOR_SPEED_PID_DEFAULT);
MotorINFO CMBL = SpeedBased_MOTORINFO_Init(&ControlCM,CHASSIS_MOTOR_SPEED_PID_DEFAULT);
MotorINFO CMBR = SpeedBased_MOTORINFO_Init(&ControlCM,CHASSIS_MOTOR_SPEED_PID_DEFAULT);
MotorINFO FRICL = SpeedBased_MOTORINFO_Init(&ControlCM,CHASSIS_MOTOR_SPEED_PID_DEFAULT);
MotorINFO FRICR = SpeedBased_MOTORINFO_Init(&ControlCM,CHASSIS_MOTOR_SPEED_PID_DEFAULT);

#if INFANTRY == 2
MotorINFO GMP  = Gimbal_MOTORINFO_Init(2.0,&ControlGMP, 3740,0,20,
										fw_PID_INIT_EASY(0.5,0,0.9,100),
										fw_PID_INIT_EASY(920,30,0,5000));
MotorINFO GMY  = Gimbal_MOTORINFO_Init(1.0,&ControlGMY, 1310,0,40,
										fw_PID_INIT_EASY(0.6,0,0.5,100),
										fw_PID_INIT_EASY(2500,100,0,5000) );
#else
#if INFANTRY == 4
MotorINFO GMP  = Gimbal_MOTORINFO_Init(2.0,&ControlGMP, 7788,800,15,
										fw_PID_INIT_EASY(0.5, 0,0.9,100),
										fw_PID_INIT_EASY(920,30,0, 5000));
MotorINFO GMY  = Gimbal_MOTORINFO_Init(1.0,&ControlGMY,4640,0,30,
										fw_PID_INIT_EASY(0.6, 0,0.5,100),
										fw_PID_INIT_EASY(2500,100,0, 5000));
#else
#if INFANTRY == 5
MotorINFO GMP  = Gimbal_MOTORINFO_Init(2.0,&ControlGMP,4416,0,20,
										fw_PID_INIT_EASY(0.5, 0,0.9,100),
										fw_PID_INIT_EASY(820,30,0, 5000));
MotorINFO GMY  = Gimbal_MOTORINFO_Init(1.0,&ControlGMY,5577,0,40,
										fw_PID_INIT_EASY(0.6, 0,	0.5,100),
										fw_PID_INIT_EASY(2000,20,0, 5000));
#endif
#endif
#endif
MotorINFO STIR = AngleBased_MOTORINFO_Init(36.0,&ControlNM,
								fw_PID_INIT_EASY(10.0, 0.0, 0.0, 1080.0),
								fw_PID_INIT_EASY(30, 0.0, 0.0,	 15000.0));
MotorINFO* can1[8]={&FRICL,&FRICR,0,0,&GMY,&GMP,&STIR,0};
MotorINFO* can2[8]={&CMFL,&CMFR,&CMBL,&CMBR,0,0,0,0};

MotorINFO* ChassisMotorGroup[4]={&CMFL,&CMFR,&CMBL,&CMBR};
MotorINFO* GimbalMotorGroup[2]={&GMP,&GMY};
#endif
//***********************************************************************************************************
//											GUARD
//***********************************************************************************************************
#ifdef GUARD
//电机信息：
MotorINFO FRICL = SpeedBased_MOTORINFO_Init(&ControlANTI_CM,CHASSIS_MOTOR_SPEED_PID_DEFAULT);
MotorINFO FRICR = SpeedBased_MOTORINFO_Init(&ControlANTI_CM,CHASSIS_MOTOR_SPEED_PID_DEFAULT);
MotorINFO GMP  = Gimbal_MOTORINFO_Init(2.0,&ControlGMP, 2495 , 0 , 20 ,
									   fw_PID_INIT_EASY(0.5,0,0.9, 	100.0),
									   fw_PID_INIT_EASY(920,30,0, 	5000.0));
MotorINFO GMY  = Gimbal_MOTORINFO_Init(1.0,&ControlGMY, 4490 , 0 , 40 ,
									   fw_PID_INIT_EASY(0.6,0,0.5, 	100.0),
									   fw_PID_INIT_EASY(2500,100,0, 5000.0));
									   
MotorINFO STIR = AngleBased_MOTORINFO_Init(36.0,&ControlNM,
								fw_PID_INIT_EASY(10.0, 0.0, 0.0, 1080.0),
								fw_PID_INIT_EASY(30, 0.0, 0.0,	 15000.0));
MotorINFO CML = AngleBased_MOTORINFO_Init(19.0,&ControlNM,
								fw_PID_INIT_EASY(10.0, 0.0, 0.0, 1500.0),
								fw_PID_INIT_EASY(40, 0.0, 5.0,	 15000.0));
MotorINFO CMR = AngleBased_MOTORINFO_Init(19.0,&ControlNM,
								fw_PID_INIT_EASY(10.0, 0.0, 0.0, 1500.0),
								fw_PID_INIT_EASY(40, 0.0, 5.0,	 15000.0));
MotorINFO* can2[8]={&FRICL,&FRICR,0,0,0,0,0,0};
MotorINFO* can1[8]={&CML,&CMR,0,0,&GMY,&GMP,&STIR,0};
MotorINFO* GimbalMotorGroup[2]={&GMP,&GMY};
MotorINFO* ChassisMotorGroup[4]={&CML,&CMR,0,0};
#endif
//***********************************************************************************************************
//											HERO
//***********************************************************************************************************
#ifdef HERO
MotorINFO CMFL = SpeedBased_MOTORINFO_Init(&ControlCM,CHASSIS_MOTOR_SPEED_PID_DEFAULT);
MotorINFO CMFR = SpeedBased_MOTORINFO_Init(&ControlCM,CHASSIS_MOTOR_SPEED_PID_DEFAULT);
MotorINFO CMBL = SpeedBased_MOTORINFO_Init(&ControlCM,CHASSIS_MOTOR_SPEED_PID_DEFAULT);
MotorINFO CMBR = SpeedBased_MOTORINFO_Init(&ControlCM,CHASSIS_MOTOR_SPEED_PID_DEFAULT);
MotorINFO GMP  = Gimbal_MOTORINFO_Init(2.0,&ControlGMP, 2495 , 0 , 20 ,
									   fw_PID_INIT_EASY(0.5,0,0.9, 	100.0),
									   fw_PID_INIT_EASY(920,30,0, 	5000.0));
MotorINFO GMY  = Gimbal_MOTORINFO_Init(1.0,&ControlGMY, 4490 , 0 , 40 ,
									   fw_PID_INIT_EASY(0.6,0,0.5, 	100.0),
									   fw_PID_INIT_EASY(2500,100,0, 5000.0));


MotorINFO* can1[8]={0};
MotorINFO* can2[8]={0};

MotorINFO* ChassisMotorGroup[4]={&CMFL,&CMFR,&CMBL,&CMBR};
MotorINFO* GimbalMotorGroup[2]={&GMP,&GMY};
#endif
//***********************************************************************************************************
//											ENGINEER
//***********************************************************************************************************
#ifdef ENGINEER

MotorINFO CMFL = SpeedBased_MOTORINFO_Init(&ControlCM,CHASSIS_MOTOR_SPEED_PID_DEFAULT);
MotorINFO CMFR = SpeedBased_MOTORINFO_Init(&ControlCM,CHASSIS_MOTOR_SPEED_PID_DEFAULT);
MotorINFO CMBL = SpeedBased_MOTORINFO_Init(&ControlCM,CHASSIS_MOTOR_SPEED_PID_DEFAULT);
MotorINFO CMBR = SpeedBased_MOTORINFO_Init(&ControlCM,CHASSIS_MOTOR_SPEED_PID_DEFAULT);

MotorINFO* can1[8]={0};
MotorINFO* can2[8]={0};

MotorINFO* ChassisMotorGroup[4]={&CMFL,&CMFR,&CMBL,&CMBR};
#endif
#endif
