#ifndef ROBOOTDETAIL_H
#define ROBOOTDETAIL_H
#ifndef CONFIGURATION
//***********************************************************************************************************
//											INFANTRY
//***********************************************************************************************************
#ifdef INFANTRY

#define USE_IMU
#define USE_CHASSIS_FOLLOW
#define USE_SUPER_CAP
#define USE_POWER_LIMIT 	80
#define USE_AUTOAIM
#define USE_HEAT_LIMIT_INFANTRY

#define CAN11
#define CAN12
#define CAN21
#endif
//***********************************************************************************************************
//											GUARD
//***********************************************************************************************************
#ifdef GUARD
#if GUARD == 'U'
	//#define USE_POWER_LIMIT 	20
	
	//#define CAN11
	#define CAN13 	1
#else
	#define SLAVE_MODE
	//#define USE_IMU
	//#define USE_AUTOAIM
	
	//#define CAN21
	//#define CAN22
	#define CAN13	1
#endif
#endif
//***********************************************************************************************************
//											HERO
//***********************************************************************************************************
#ifdef HERO
#define USE_IMU
#define USE_AUTOAIM
#define USE_CHASSIS_FOLLOW
#define USE_POWER_LIMIT 	80

#define CAN11
#define CAN12
#define CAN21
#define CAN22
#define CAN23	1
#endif
//***********************************************************************************************************
//											ENGINEER
//***********************************************************************************************************
#ifdef ENGINEER
#define CAN11
#define CAN12
#define CAN21
#define CAN22
#endif

#endif
#endif
