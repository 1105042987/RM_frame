#ifndef CONFIGURATION
//******************INFANTRY*************************
#ifdef INFANTRY
#define USE_IMU
#define USE_CHASSIS_FOLLOW
#define USE_SUPER_CAP
#define USE_POWER_LIMIT 	80
#define USE_AUTOAIM

#define CAN11
#define CAN12
#define CAN21
#endif
//*******************GUARD****************************
#ifdef GUARD
#define USE_IMU
#define USE_AUTOAIM
#define USE_POWER_LIMIT 	20

#define CAN11
#define CAN12
#define CAN21
#define CAN22
#define CAN23	1
#endif











































#endif
