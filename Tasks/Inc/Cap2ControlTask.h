/**
  ******************************************************************************
  * File Name          : Cap2ControlTask.h
  * Description        : The header file of Cap2ControlTask.c
  ******************************************************************************
  *
  * Copyright (c) 2019 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */

#ifndef __CAP_2_CONTROL_TASK__
#define __CAP_2_CONTROL_TASK__

#include "stm32f4xx_hal.h"

typedef enum {
	CAP_STATE_STOP,
	CAP_STATE_RECHARGE,
	CAP_STATE_RELEASE
}cap_state;

typedef __packed struct
{
	uint16_t P_voltage;
	uint16_t P_Power;
	uint16_t C_voltage;
}CapControl_t;

extern CapControl_t Control_SuperCap;

/*********************************************************
  * @brief  Initialize the capacitance controller.
  * @param  None
  * @retval None
  */
void Cap_Init(void);

/*********************************************************
  * @brief  Run the capacitance controller. (called with period of 2 ms)
  * @param  None
  * @retval None
  */
void Cap_Run(void);

/*********************************************************
  * @brief  Switch the state of the capacitance controller
			into a specific state manually.
  * @param  The aim state of the capacitance controller.
  * @retval None
  */
void Cap_State_Switch(cap_state);

/*********************************************************
  * @brief  Get the current voltage of capacitance.
  * @param  None
  * @retval The current voltage of capacitance.(typical value: 11.0-22.5)
  */
double Cap_Get_Cap_Voltage(void);

//by zzy
double Cap_Get_Power_Voltage(void);
double Cap_Get_Power_CURR(void);

/*********************************************************
  * @brief  Get the current state of the capacitance controller.
  * @param  None
  * @retval The current state of the capacitance controller.
  */
cap_state Cap_Get_Cap_State(void);

void LED_Show_SuperCap_Voltage(uint8_t flag);

#endif /* __CAP_2_CONTROL_TASK__ */
