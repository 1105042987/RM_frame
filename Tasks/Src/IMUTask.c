/**
  ******************************************************************************
  * File Name          : IMUTask.c
  * Description        : IMU数据刷新任务
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#include "includes.h"

#define MPU6500_NSS_Low() HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET)
#define MPU6500_NSS_High() HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET)

IMUDataTypedef imu_data = {0,0,0,0,0,0,0,0,0,0};
IMUDataTypedef imu_data_offest = {0,0,0,0,0,0,0,0,0,0};
float gYroXs, gYroYs, gYroZs;


//Read a register from MPU6500
uint8_t MPU6500_Read_Reg(uint8_t const reg)
{
  static uint8_t MPU_Rx, MPU_Tx;
  
  MPU6500_NSS_Low();
  
  MPU_Tx = reg|0x80;
  HAL_SPI_TransmitReceive(&hspi5, &MPU_Tx, &MPU_Rx, 1, 55);
  HAL_SPI_TransmitReceive(&hspi5, &MPU_Tx, &MPU_Rx, 1, 55);
  
  MPU6500_NSS_High();
  return MPU_Rx;
}

//Write a register to MPU6500
uint8_t MPU6500_Write_Reg(uint8_t const reg, uint8_t const data)
{
  static uint8_t MPU_Rx, MPU_Tx;
  
  MPU6500_NSS_Low();
  
  MPU_Tx = reg&0x7f;
  HAL_SPI_TransmitReceive(&hspi5, &MPU_Tx, &MPU_Rx, 1, 55);
  MPU_Tx = data;
  HAL_SPI_TransmitReceive(&hspi5, &MPU_Tx, &MPU_Rx, 1, 55);
  
  MPU6500_NSS_High();
  return 0;
}

//Initialize the MPU6500
uint8_t MPU_id = 0;
uint8_t InitMPU6500(void)
{
  uint8_t index = 0;
  uint8_t MPU6500_Init_Data[10][2] = 
  {
    {MPU6500_PWR_MGMT_1,    0x80},      // Reset Device
    {MPU6500_PWR_MGMT_1,    0x03},      // Clock Source - Gyro-Z
    {MPU6500_PWR_MGMT_2,    0x00},      // Enable Acc & Gyro
    {MPU6500_CONFIG,        0x02},      // LPF 98Hz
    {MPU6500_GYRO_CONFIG,   0x18},      // +-2000dps
    {MPU6500_ACCEL_CONFIG,  0x10},      // +-8G
    {MPU6500_ACCEL_CONFIG_2,0x02},      // enable LowPassFilter  Set Acc LPF
    
		//{MPU6500_INT_PIN_CFG,   0x06},//Enable INT ==
		//{MPU6500_INT_ENABLE,    0x00},//Enable INT ==
		
		{MPU6500_USER_CTRL,     0x20},      // Enable AUX
		
		{MPU6500_INT_ENABLE,    0x01}//Enable INT ==
  };
  
  HAL_Delay(100);
  MPU_id = MPU6500_Read_Reg(MPU6500_WHO_AM_I);  //read id of device,check if MPU6500 or not
  //fw_printfln("MPU_id %x", MPU_id);
	
  for(index = 0; index < 10; index++)
  {
    MPU6500_Write_Reg(MPU6500_Init_Data[index][0], MPU6500_Init_Data[index][1]);
    HAL_Delay(1);
  }

  return 0;
}

//IMU数据刷新函数，放在main函数主循环中
void IMURefresh()
{
	MPU6500_NSS_Low();
	uint8_t MPU_Tx = MPU6500_ACCEL_XOUT_H | 0x80;
	HAL_SPI_Transmit(&hspi5, &MPU_Tx, 1, 55);
	uint8_t mpu_buff[22];
	HAL_SPI_Receive(&hspi5, mpu_buff, 22, 55);
	
	MPU6500_NSS_High();

	imu_data.ax = mpu_buff[0]<<8 |mpu_buff[1];
	imu_data.ay = mpu_buff[2]<<8 |mpu_buff[3];
	imu_data.az = mpu_buff[4]<<8 |mpu_buff[5];

	imu_data.temp = mpu_buff[6]<<8 |mpu_buff[7];

	imu_data.gx = mpu_buff[8]<<8 |mpu_buff[9] - imu_data_offest.gx;
	imu_data.gy = mpu_buff[10]<<8 |mpu_buff[11] - imu_data_offest.gy;
	imu_data.gz = mpu_buff[12]<<8 |mpu_buff[13] - imu_data_offest.gz;

	imu_data.mx=mpu_buff[15]<<8 |mpu_buff[14];
	imu_data.my=mpu_buff[17]<<8 |mpu_buff[16];
	imu_data.mz=mpu_buff[19]<<8 |mpu_buff[18];
		
	gYroXs = imu_data.gx / 32.8f;
	gYroYs = imu_data.gy / 32.8f;
	gYroZs = imu_data.gz / 32.8f;
}
