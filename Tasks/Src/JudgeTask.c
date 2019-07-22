/******************************************************************************
  * File Name          : JudgeTask.h
  * Description        : Deal with referee system to obtain referee data
	* Author						 : Junhao Lin, Xinyu Jian
  ******************************************************************************
  *
  * Copyright (c) 2019 Team Jiao Long - Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************/
#include "includes.h"
#include "offical_Judge_Handler.h"
#include <stdlib.h>
#include <string.h>

static void Judge_Refresh_Hit(void);
static void Judge_Refresh_Result(void);
static void Referee_Update_RobotState(void);
static void Referee_Update_PowerHeatData(void);
static void Referee_Update_BuffMask(void);
static void Referee_Update_ShootData(void);

uint8_t tmp_judge;
void InitJudgeUart(void){
	//tx_free = 1;
//	Send_User_Data();
	//Referee_Transmit_UserData();
	if(HAL_UART_Receive_DMA(&JUDGE_UART, &tmp_judge, 1) != HAL_OK){
			Error_Handler();
	}
}
uint8_t receiving = 0;
uint8_t received = 0;
uint8_t buffer[80] = {0}; 
uint8_t buffercnt = 0;
uint16_t cmdID;
uint8_t tmpBuffer[80] = {0};
uint8_t coooool = 0;

uint32_t testJudgeFreq = 0; //jxy test
void judgeUartRxCpltCallback(void)
{
//	fw_printfln("judge receive");
		if(receiving) 
		{
			if(buffercnt >40)buffercnt = 4;
			buffer[buffercnt] = tmp_judge;
			buffercnt++;
			
			if(buffercnt == 5)
			{
				if (myVerify_CRC8_Check_Sum(buffer, 5)==0) 
				{
					receiving = 0;
					buffercnt = 0;
				}
			}
			
			if(buffercnt == 7) cmdID = (0x0000 | buffer[5]) | (buffer[6] << 8);
		
			if(buffercnt == 12 && cmdID == 0x0001)
			{
				if (myVerify_CRC16_Check_Sum(buffer, 12))
				{
					Judge_Refresh_Result();
					receiving = 0;
					buffercnt = 0;
				}					
			}
			
			if(buffercnt == 10 && cmdID == 0x0002)
			{
				if (myVerify_CRC16_Check_Sum(buffer, 10))
				{
					Judge_Refresh_Result();
					receiving = 0;
					buffercnt = 0;
				}					
			}
			
			if(buffercnt == 11 && cmdID == 0x0003)
			{
				if (myVerify_CRC16_Check_Sum(buffer, 11))
				{
					Judge_Refresh_Result();
					receiving = 0;
					buffercnt = 0;
				}					
			}
			
			if(buffercnt == 13 && cmdID == 0x0101)
			{
				if (myVerify_CRC16_Check_Sum(buffer, 13))
				{
					Judge_Refresh_Result();
					receiving = 0;
					buffercnt = 0;
				}					
			}
			
			if(buffercnt == 12 && cmdID == 0x0102)
			{
				if (myVerify_CRC16_Check_Sum(buffer, 12))
				{
					Judge_Refresh_Result();
					receiving = 0;
					buffercnt = 0;
				}					
			}
			
			if(buffercnt == 11 && cmdID == 0x0103)
			{
				if (myVerify_CRC16_Check_Sum(buffer, 11))
				{
					Judge_Refresh_Result();
					receiving = 0;
					buffercnt = 0;
				}					
			}

			if(buffercnt == 24 && cmdID == 0x0201)
			{
				if (myVerify_CRC16_Check_Sum(buffer, 24))
				{
					Referee_Update_RobotState();
					receiving = 0;
					buffercnt = 0;
				}					
			}
			
			if(buffercnt == 23 && cmdID == 0x0202)
			{
				if (myVerify_CRC16_Check_Sum(buffer, 23))
				{
					Referee_Update_PowerHeatData();
					testJudgeFreq++;
					receiving = 0;
					buffercnt = 0;
				}					
			}

 			if(buffercnt == 25 && cmdID == 0x0203)
			{
				if (myVerify_CRC16_Check_Sum(buffer, 25))
				{
					Judge_Refresh_Result();
					receiving = 0;
					buffercnt = 0;
				}					
			}
			
			if(buffercnt == 10 && cmdID == 0x0204)
			{
				if (myVerify_CRC16_Check_Sum(buffer, 10))
				{
					Referee_Update_BuffMask();
					receiving = 0;
					buffercnt = 0;
				}					
			}
			
			if(buffercnt == 12 && cmdID == 0x0205)
			{
				if (myVerify_CRC16_Check_Sum(buffer, 12))
				{
					Judge_Refresh_Result();
					receiving = 0;
					buffercnt = 0;
				}					
			}

 			if(buffercnt == 10 && cmdID == 0x0206)
			{
				if (myVerify_CRC16_Check_Sum(buffer, 10)) 
				{
					Judge_Refresh_Hit();
					receiving = 0;
					buffercnt = 0;
				}
			}

			if(buffercnt == 15 && cmdID == 0x0207)
			{
				if (myVerify_CRC16_Check_Sum(buffer, 15))
				{
					Referee_Update_ShootData();
					receiving = 0;
					buffercnt = 0;
				}					
			}
		}	
		
		else 
		{
			if(tmp_judge == 0xA5)
			{
				receiving = 1;
				buffercnt = 0;
				buffer[0] = tmp_judge;
				buffercnt++;
			}
		}
		
		if(HAL_UART_Receive_DMA(&JUDGE_UART, &tmp_judge, 1) != HAL_OK)
		{
			Error_Handler();
	  }
}


uint8_t JUDGE_Received = 0;
JudgeState_e JUDGE_State = OFFLINE;


ext_robot_hurt_t HitData;
AAAA_Data detectingAAAA;
AAAA_Act reactingAAAA;

uint16_t maxHP = 300;
uint16_t remainHP;
uint16_t maxHeat0 = 480;
uint16_t remainHeat0 = 480;
uint16_t maxHeat1 = 400;
uint16_t remainHeat1 = 400;
uint16_t realHeat0 = 0;
float fakeHeat0 = 0;
float realBulletSpeed0 = 22;
float cooldown0 = 72;
uint8_t shoot0Cnt = 0;
uint8_t shoot1Cnt = 0;

uint8_t syncCnt0 = 0;
uint8_t ArmorHitted = 0;
uint8_t AL,AH;
uint8_t HitedNum[40] = {6};
float HitedAngle[40] = {180};
uint8_t HitedCnt = 0;
//extern uint8_t AAAAmode;
//extern float ChassisTwistGapAngle;
//extern uint8_t colorAim;

uint8_t HitCnt=0;
void Judge_Refresh_Hit()
{
	HitCnt++;
	ArmorHitted = buffer[7];
	AL = ArmorHitted & 0x7;
	AH = ArmorHitted >> 4;
	if(AH == 0)
	{
		if(detectingAAAA.Delay == 0)detectingAAAA.Armor = AL;
		if(AL == 0 || AL == 1)
		{
			HitedAngle[HitedCnt] = (GMY.RxMsgC6x0.angle - GM_YAW_ZERO) * 360 / 8192.0f;
			NORMALIZE_ANGLE180(HitedAngle[HitedCnt]);
			HitedNum[HitedCnt] = AL;
			if(HitedNum[HitedCnt] == 0 )
			{
				if(reactingAAAA.Dir == 1 && reactingAAAA.inPosition)
				{
					//reactingAAAA.Angle0 += 5;
				}
				reactingAAAA.Dir = 1;
				//ChassisTwistGapAngle = reactingAAAA.Angle0;
				if(reactingAAAA.counting0)
				{
					reactingAAAA.redetect0 = reactingAAAA.redC0;
				}
				reactingAAAA.counting0 = 0;
				reactingAAAA.redC0 = 0;
//				if(HitedAngle[HitedCnt] > reactingAAAA.Angle0)
//				{
//					if(HitedAngle[HitedCnt] < 90 && HitedAngle[HitedCnt] > -90)
//					{
//						float angleAdjust0 = HitedAngle[HitedCnt] - reactingAAAA.Angle0;
////						reactingAAAA.Angle0 += angleAdjust0 + 1;
////						reactingAAAA.Angle1 += angleAdjust0 + 1;
//					}
//				}
			}

			else if(HitedNum[HitedCnt] == 1 )
			{
				if(reactingAAAA.Dir == -1 && reactingAAAA.inPosition)
				{
					//reactingAAAA.Angle1 -= 5;
				}
				reactingAAAA.Dir = -1;
				//ChassisTwistGapAngle = reactingAAAA.Angle1;
				if(reactingAAAA.counting1)
				{
					reactingAAAA.redetect1 = reactingAAAA.redC1;
				}
				reactingAAAA.counting1 = 0;
				reactingAAAA.redC1 = 0;
//				if(HitedAngle[HitedCnt] < reactingAAAA.Angle1)
//				{
//					if(HitedAngle[HitedCnt] < 90 && HitedAngle[HitedCnt] > -90)
//					{
//						float angleAdjust1 = reactingAAAA.Angle1 - HitedAngle[HitedCnt];
////						reactingAAAA.Angle0 -= angleAdjust1 + 1;
////						reactingAAAA.Angle1 -= angleAdjust1 + 1;
//					}
//				}
			}
			HitedCnt++;
			if(HitedCnt > 35)HitedCnt = 0;
		}
	}
	JUDGE_Received = 1;
}


void Judge_Refresh_Result()
{
	JUDGE_Received = 1;
}

void getJudgeState(void)
{
	static int s_count_judge = 0;
	if(JUDGE_Received==1)
	{
		s_count_judge = 0;
		JUDGE_State = ONLINE;
		JUDGE_Received = 0;
	}
	else
	{
		s_count_judge++;
		if(s_count_judge > 150)
		{//300ms
			JUDGE_State = OFFLINE;
		}
	}
}


ext_game_robot_state_t GameRobotState;
ext_power_heat_data_t PowerHeat;
ext_buff_musk_t BuffMask;
ext_shoot_data_t ShootData;
client_custom_data_t custom_data;

void Referee_Update_RobotState()
{
	unsigned char* grs0 = (unsigned char*)&GameRobotState.robot_id;
	char tmp0[1] = {buffer[7]};
	grs0[0] = (unsigned char)tmp0[0];
	
	unsigned char* grs1 = (unsigned char*)&GameRobotState.robot_level;
	char tmp1[1] = {buffer[8]};
	grs1[0] = (unsigned char)tmp1[0];
	
	unsigned char* grs2 = (unsigned char*)&GameRobotState.remain_HP;
	char tmp2[2] = {buffer[9],buffer[10]};
	for(int i = 0; i<2; i++){
		grs2[i] = (unsigned char)tmp2[i];
	}
	
	unsigned char* grs3 = (unsigned char*)&GameRobotState.max_HP;
	char tmp3[2] = {buffer[11], buffer[12]};
	for(int i = 0; i<2; i++){
		grs3[i] = (unsigned char)tmp3[i];
	}
	
	unsigned char* grs4 = (unsigned char*)&GameRobotState.shooter_heat0_cooling_rate;
	char tmp4[2] = {buffer[13], buffer[14]};
	for(int i = 0; i<2; i++){
		grs4[i] = (unsigned char)tmp4[i];
	}
//	if(GameRobotState.shooter_heat0_cooling_rate == 200
//		|| GameRobotState.shooter_heat0_cooling_rate == 300 || GameRobotState.shooter_heat0_cooling_rate == 400)
//		coooool = 1;
	unsigned char* grs5 = (unsigned char*)&GameRobotState.shooter_heat0_cooling_limit;
	char tmp5[2] = {buffer[15], buffer[16]};
	for(int i = 0; i<2; i++){
		grs5[i] = (unsigned char)tmp5[i];
	}
	
	unsigned char* grs6 = (unsigned char*)&GameRobotState.shooter_heat1_cooling_rate;
	char tmp6[2] = {buffer[17], buffer[18]};
	for(int i = 0; i<2; i++){
		grs6[i] = (unsigned char)tmp6[i];
	}	
	
	unsigned char* grs7 = (unsigned char*)&GameRobotState.shooter_heat1_cooling_limit;
	char tmp7[2] = {buffer[19], buffer[20]};
	for(int i = 0; i<2; i++){
		grs7[i] = (unsigned char)tmp7[i];
	}
	
	maxHP = GameRobotState.max_HP;
	remainHP = GameRobotState.remain_HP;
	switch(maxHP)
	{
		case 200:{maxHeat0 = 240;cooldown0 = 40;}break;
		case 250:{maxHeat0 = 360;cooldown0 = 60;}break;
		case 300:{maxHeat0 = 480;cooldown0 = 80;}break;
		default:{maxHeat0 = 480;cooldown0 = 80;}break;
//		case 200:{maxHeat0 = 240;}break;
//		case 250:{maxHeat0 = 360;}break;
//		case 300:{maxHeat0 = 480;}break;
//		default:{maxHeat0 = 480;}break;
	}
//	if(coooool)cooldown0 *= 5;
	JUDGE_Received = 1;
	
//	if(GameRobotState.robot_id > 9)colorAim = 1;//blue
	//else colorAim = 0;
	//电源输出情况TBD
	
//	unsigned char* grs = (unsigned char*)&GameRobotState;
//	char tmp[15];
//	for(int i = 0; i<15; i++){
//		 //strcpy(tmp, buffer[i+7]);
////		sprintf(tmp, "%s", (char *)buffer[i+7]);
//	}
}

void Referee_Update_PowerHeatData()
{
	//电压tbd
	//电流tbd
	unsigned char * ph0 = (unsigned char*)&PowerHeat.chassis_power;
	for(int i = 0; i<4; i++){
	ph0[i] = (unsigned char)buffer[i+11];
	}
	
	unsigned char * ph1 = (unsigned char*)&PowerHeat.chassis_power_buffer;
	for(int i = 0; i<2; i++){
	ph1[i] = (unsigned char)buffer[i+15];
	}
	
	unsigned char * ph2 = (unsigned char*)&PowerHeat.shooter_heat0;
	for(int i = 0; i<2; i++){
	ph2[i] = (unsigned char)buffer[i+17];
	}
	
	unsigned char * ph3 = (unsigned char*)&PowerHeat.shooter_heat1;
	for(int i = 0; i<2; i++){
	ph3[i] = (unsigned char)buffer[i+19];
	}
	realHeat0 = PowerHeat.shooter_heat0;
	remainHeat0 = maxHeat0 - PowerHeat.shooter_heat0;
	remainHeat1 = maxHeat1 - PowerHeat.shooter_heat1;
	JUDGE_Received = 1;
}

void Referee_Update_BuffMask()
{
	unsigned char * bm = (unsigned char*)&BuffMask.power_rune_buff;
	for(int i = 0; i<1; i++){
	bm[i] = (unsigned char)buffer[i+7];
	}
	JUDGE_Received = 1;
}

void Referee_Update_ShootData()
{
	unsigned char * sd0 = (unsigned char*)&ShootData.bullet_type;
	sd0[0] = (unsigned char)buffer[7];
	
	JUDGE_Received = 1;unsigned char * sd1 = (unsigned char*)&ShootData.bullet_freq;
	sd1[0] = (unsigned char)buffer[8];
	
	unsigned char * sd2 = (unsigned char*)&ShootData.bullet_speed;
	for(int i = 0; i<4; i++){
	sd2[i] = (unsigned char)buffer[i+9];
	}
	
	realBulletSpeed0 = ShootData.bullet_speed;
	JUDGE_Received = 1;
}
extern Distance_Couple_t distance_couple;
uint16_t State_Flag;
void Referee_Transmit_UserData()//数据上传可以限制在10hz
{
	uint8_t buffer[28]={0};
	//test
	custom_data.data1 = AutoClimb_Level;
	custom_data.data2 = AutoGet_Bullet_S*3+AutoGet_Bullet_B*15;
	custom_data.data3 = 111111;
	
	State_Flag=(distance_couple.left.flag)*128+(distance_couple.right.flag)*4;
	
	custom_data.masks = State_Flag;//二进制最左位对应灯条最右灯
	
	unsigned char * bs1 = (unsigned char*)&custom_data.data1;
	unsigned char * bs2 = (unsigned char*)&custom_data.data2;
	unsigned char * bs3 = (unsigned char*)&custom_data.data3;
	buffer[0] = 0xA5;//数据帧起始字节，固定值为 0xA5
	buffer[1] = 19;//数据帧中 data 的长度,占两个字节
	
	buffer[3] = 1;//包序号
	buffer[4] = myGet_CRC8_Check_Sum(&buffer[0], 5-1, myCRC8_INIT);//帧头 CRC8 校验
	buffer[5] = 0x01;
	buffer[6] = 0x03;
	
	buffer[7] = 0x80;//数据的内容 ID:0xD180  ,占两个字节
	buffer[8] = 0xD1;
	buffer[9] = GameRobotState.robot_id;//发送者的 ID, 占两个字节
	buffer[10] = 0;
	buffer[11] = GameRobotState.robot_id;//客户端的 ID, 只能为发送者机器人对应的客户端,  占两个字节
	if(buffer[11]>9&&buffer[11]<16)buffer[11]+=6;
	buffer[12] = 0x01;//注意哨兵没有这个ID
	for(int i=13;i<17;i++) buffer[i] = bs1[i-13];
	for(int i=17;i<21;i++) buffer[i] = bs2[i-17];
	for(int i=21;i<25;i++) buffer[i] = bs3[i-21];
	buffer[25] = custom_data.masks;
	static uint16_t CRC16=0;
	CRC16 = myGet_CRC16_Check_Sum(buffer, 26, myCRC16_INIT);
	buffer[26] = CRC16 & 0x00ff;//0xff
	buffer[27] = (CRC16 >> 8) & 0xff;

	//tx_free = 0;
	while(HAL_UART_Transmit_IT(&JUDGE_UART,(uint8_t *)&buffer,28)!=HAL_OK);
}
int check=0;
void Referee_Transmit_Image()//数据上传可以限制在10hz 这个函数用于绘制自定义图形
{
	uint8_t buffer[71]={0};
	
	buffer[0] = 0xA5;//数据帧起始字节，固定值为 0xA5
	buffer[1] = 62;//数据帧中 data 的长度,占两个字节
	
	buffer[3] = 1;//包序号
	buffer[4] = myGet_CRC8_Check_Sum(&buffer[0], 5-1, myCRC8_INIT);//帧头 CRC8 校验
	buffer[5] = 0x01;
	buffer[6] = 0x03;
	buffer[7] = 0x00;//数据的内容 ID:0x0100  ,占两个字节
	buffer[8] = 0x01;
	buffer[9] = GameRobotState.robot_id;//发送者的 ID, 占两个字节
	buffer[10] = 0;
	buffer[11] = GameRobotState.robot_id;//客户端的 ID, 只能为发送者机器人对应的客户端,  占两个字节
	buffer[12] = 0x01;//注意哨兵没有这个ID
	  check=1;
	  buffer[13] = 1;//增加图形
		buffer[14] = 3;//类型
	  buffer[15] = 't';//下五个字节是图形名
		buffer[16] = 'e';
		buffer[17] = 's';
		buffer[18] = 't';
		buffer[19] = 'p';
    buffer[20] = 1;//图层数
    buffer[21] = 0;//颜色 0为队伍主颜色
    buffer[22] = 10;//线宽
    buffer[23] = 0xc0;//起点x坐标 占两字节
    buffer[24] = 0x03;	
    buffer[25] = 0x1c;//起点y坐标 占两字节
    buffer[26] = 0x02;
    buffer[28] = 0x01;//字体大小或者圆的半径   	
		buffer[29] = 0x00;
		buffer[30] = 0xb0;    //终点x和y	
		buffer[31] = 0x04;
		buffer[32] = 0xbc;
		buffer[33] = 0x02;  
		buffer[34] = 0;  //圆弧的起始角度和终止角度
		buffer[35] = 0;
		buffer[36] = 0;
		buffer[37] = 0;
		buffer[38] = 4;  //文本信息的长度
		buffer[39] = 't';//文本信息
		buffer[40] = 'e';
		buffer[41] = 's';
		buffer[42] = 't';
		for(int i=43;i<=68;i++)
		buffer[i] = 0;
	static uint16_t CRC16=0;
	CRC16 = myGet_CRC16_Check_Sum(buffer, 69, myCRC16_INIT);
	buffer[69] = CRC16 & 0x00ff;//0xff 校验位
	buffer[70] = (CRC16 >> 8) & 0xff;
		
	while(HAL_UART_Transmit_IT(&JUDGE_UART,(uint8_t *)&buffer,71)!=HAL_OK);
}

uint8_t flying = 0;
void Referee_Transmit_RobotData()
{
	uint8_t buffer[28]={0};

	buffer[0] = 0xA5;//数据帧起始字节，固定值为 0xA5
	buffer[1] = 7;//数据帧中 data 的长度,占两个字节
	
	buffer[3] = 1;//包序号
	buffer[4] = myGet_CRC8_Check_Sum(&buffer[0], 5-1, myCRC8_INIT);//帧头 CRC8 校验
	buffer[5] = 0x01;
	buffer[6] = 0x03;
	
	buffer[7] = 0x03;//数据的内容 ID:0x200-0x2FF  ,占两个字节
	buffer[8] = 0x02;
	if(GameRobotState.robot_id < 9)buffer[9] = 3;//发送者的 ID, 占两个字节
	else buffer[9] = 13;
	buffer[10] = 0;
//	buffer[11] = GameRobotState.robot_id;//客户端的 ID, 只能为发送者机器人对应的客户端,  占两个字节
//	if(buffer[11]>9&&buffer[11]<16)buffer[11]+=6;
//	if(colorAim == 1)
//	{
//		buffer[11] = 17;
//	}
//	else 
//	{
//		buffer[11] = 7;
//		buffer[12] = 0;
//	}
	buffer[13] = flying;
	static uint16_t CRC16=0;
	CRC16 = myGet_CRC16_Check_Sum(buffer, 14, myCRC16_INIT);
	buffer[14] = CRC16 & 0x00ff;//0xff
	buffer[15] = (CRC16 >> 8) & 0xff;
	//tx_free = 0;
	while(HAL_UART_Transmit_IT(&JUDGE_UART,(uint8_t *)&buffer,28)!=HAL_OK);
}

//void Send_User_Data()
//{
//	uint8_t buffer[22]={0};
//	//test
//	user_data.data1 += 0.1f;
//	user_data.data2 += 0.2f;
//	user_data.data3 += 0.3f;
//	//user_data.mask = 0xE0;//二进制最左位对应灯条最右灯
//	
//	unsigned char * bs1 = (unsigned char*)&user_data.data1;
//	unsigned char * bs2 = (unsigned char*)&user_data.data2;
//	unsigned char * bs3 = (unsigned char*)&user_data.data3;
//	buffer[0] = 0xA5;
//	buffer[1]  = 13;
//	buffer[3] = 1;
//	buffer[4] = myGet_CRC8_Check_Sum(&buffer[0], 5-1, myCRC8_INIT);
//	buffer[5] = 0x00;
//	buffer[6] = 0x01;
//	for(int i=7;i<11;i++) buffer[i] = bs1[i-7];
//	for(int i=11;i<15;i++) buffer[i] = bs2[i-11];
//	for(int i=15;i<19;i++) buffer[i] = bs3[i-15];
//	buffer[19] = user_data.mask;
//	static uint16_t CRC16=0;
//	CRC16 = myGet_CRC16_Check_Sum(buffer, 20, myCRC16_INIT);
//	buffer[20] = CRC16 & 0x00ff;//0xff
//	buffer[21] = (CRC16 >> 8) & 0xff;

//	tx_free = 0;
//	while(HAL_UART_Transmit_DMA(&JUDGE_UART,(uint8_t *)&buffer,22)!=HAL_OK);
//}
