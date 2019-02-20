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

#define MAXHP1 750
#define MAXHP2 1000
#define MAXHP3 1500

#define COOLDOWN01 18
#define COOLDOWN02 36
#define COOLDOWN03 72

#define MAXHEAT01 120
#define MAXHEAT02 240
#define MAXHEAT03 480
#endif
//***********************************************************************************************************
//											GUARD
//***********************************************************************************************************
#ifdef GUARD
#define USE_IMU
#define USE_AUTOAIM
#define USE_POWER_LIMIT 	20

#define CAN11
//#define CAN21
//#define CAN22
//#define CAN13	1
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
