/**
  ******************************************************************************
  *FileName     : Cap2ControlTask.c                                            *
  *Description  : 超级电容2开关控制程序                                        *
  *Author       : 唐欣阳                                                       *
  ******************************************************************************
  *                                                                            *
  * Copyright (c) 2019 Team JiaoLong-ShanghaiJiaoTong University               *
  * All rights reserved.                                                       *
  *                                                                            *
  ******************************************************************************
  */

#include <math.h>
#include <string.h>
#include "includes.h"

#define CAP_USE_CURR
//#define CAP_AUTO_RECHARGE
#define CAP_DEBUG

#define ADC_CHANNALS            (4)
#define ADC_HITS                (50)
#define PWM_CMP_MAX             (42000-1)
#define VREFINT                 (0)
#define ICOUT                   (1)       //PB0 Channel 8
#define ICIN                    (2)       //PB1 Channel 9
#define UCK1                    (3)       //PC0 Channel 10
#define RECHARGE_POWER_MAX      (72)
#define RELEASE_POWER_MAX       (68)//70
#define RECHARGE_CURR_MAX       (3.0)
#define RELEASE_CURR_MAX        (3.0)
#define RELEASE_POW_RATE        (1.4)//1.3
#define RECHARGE_POW_RATE       ((PowerHeatData.chassisPower > RECHARGE_POWER_MAX)?\
                                ((RECHARGE_VOLTAGE_MAX - VAL__CAP_VOLTAGE) / (RECHARGE_VOLTAGE_MAX - RELEASE_VOLTAGE_MIN) * (1.4-0.8) + 0.8):\
                                ((RECHARGE_VOLTAGE_MAX - VAL__CAP_VOLTAGE) / (RECHARGE_VOLTAGE_MAX - RELEASE_VOLTAGE_MIN) * (1.4-1.0) + 1.0))
#define INPUT_PWM_PERCENT_MAX   (100)
#define OUTPUT_PWM_PERCENT_MAX  (100)
#define RECHARGE_VOLTAGE_MAX    (21.5)
#define RELEASE_VOLTAGE_MIN     (8.0)//6
#define RE_RECHARGE_VOLTAGE     (19.0)
#define OUT_VOL_PWM_TIM         htim8
#define OUT_VOL_PWM_CHANNEL     TIM_CHANNEL_2
#define IN_VOL_PWM_TIM          htim8
#define IN_VOL_PWM_CHANNEL      TIM_CHANNEL_3

#define VAL__INPUT_PWM_MAX      (INPUT_PWM_PERCENT_MAX * PWM_CMP_MAX / 100)
#define VAL__OUTPUT_PWM_MAX     (OUTPUT_PWM_PERCENT_MAX * PWM_CMP_MAX / 100)
#define VAL__OUTPUT_PWM_PERCENT (output_pwm_cmp * 100.0 / PWM_CMP_MAX)
#define VAL__INPUT_PWM_PERCENT  (input_pwm_cmp * 100.0 / PWM_CMP_MAX)
#define VAL__CAP_VOLTAGE        (FUNC__Get_Voltage(UCK1)  * 9.2 * 13.33 / 11.01)
#define VAL__CAP_Power_CURR     (FUNC__Get_Voltage(ICIN)  * 2.48 / 1.08)
#define VAL__CAP_Power_Voltage    (FUNC__Get_Voltage(ICOUT)  * 9.2 * 13.33 / 11.01) //Power Voltage

#define FUNC__Get_Voltage(dev)              (1.2 * ADC_val[dev] / ADC_val[VREFINT])
#define FUNC__ADD_INPUT_PWM_PERCENT(rate)   (input_pwm_cmp += (rate)*PWM_CMP_MAX/100)
#define FUNC__ADD_OUTPUT_PWM_PERCENT(rate)  (output_pwm_cmp += (rate)*PWM_CMP_MAX/100)
#define FUNC__Cap_Set_Output_Percent(rate)  (output_pwm_cmp = PWM_CMP_MAX*rate/100)
#define FUNC__Cap_Set_Input_Percent(rate)   (input_pwm_cmp  = PWM_CMP_MAX*rate/100)

#ifdef CAP_USE_CURR
#define FUNC__RECAL_INPUT_PWM(rate)       FUNC__ADD_INPUT_PWM_PERCENT(rate*CAL_RECHARGE(RECHARGE_POWER_MAX, \
                                              Cap_Get_Power_Voltage()*Cap_Get_Power_CURR() + (60 - PowerHeatData.chassisPowerBuffer) / 2));
#define FUNC__RECAL_OUTPUT_PWM(rate)      FUNC__ADD_OUTPUT_PWM_PERCENT(-rate*CAL_RELEASE(RELEASE_POWER_MAX, \
                                              Cap_Get_Power_Voltage()*Cap_Get_Power_CURR() + (60 - PowerHeatData.chassisPowerBuffer) / 2));
#else
#define FUNC__RECAL_INPUT_PWM(rate)       FUNC__ADD_INPUT_PWM_PERCENT(rate*CAL_RECHARGE(RECHARGE_POWER_MAX, \
                                              PowerHeatData.chassisPower + (60 - PowerHeatData.chassisPowerBuffer) / 2));
#define FUNC__RECAL_OUTPUT_PWM(rate)      FUNC__ADD_OUTPUT_PWM_PERCENT(-rate*CAL_RELEASE(RELEASE_POWER_MAX, \
                                              PowerHeatData.chassisPower + (60 - PowerHeatData.chassisPowerBuffer) / 2));
#endif /* CAP_USE_CURR */

#define CAL_RELEASE(max, x) (((max)>(x))?(pow((max)-(x), RELEASE_POW_RATE)):(-pow((x)-(max), RELEASE_POW_RATE)))
#define CAL_RECHARGE(max, x) (((max)>(x))?(pow((max)-(x), RECHARGE_POW_RATE)):(-pow((x)-(max), RECHARGE_POW_RATE)))

static int16_t ADC_hits_val[ADC_HITS][ADC_CHANNALS];
static int32_t ADC_tmp[ADC_CHANNALS];
static int16_t ADC_val[ADC_CHANNALS];
static int32_t input_pwm_cmp = 0;
static int32_t output_pwm_cmp = 0;
static int16_t recharge_cnt = 0;
static cap_state CapState = CAP_STATE_STOP;
CapControl_t Control_SuperCap = { 0,0 };

static void Cap_State(void);
static void Cap_Ctr(void);

#ifdef CAP_DEBUG
#define DEBUG_HITS    (4000)
#define DEBUG_AMOUNT  (4)
static uint8_t cnt=0;
int32_t cps[DEBUG_AMOUNT][DEBUG_HITS];
static uint32_t ncnt=0;
#if DEBUG_HITS * DEBUG_AMOUNT * 4 > 65535
  #error Cap debug info bytes must below 65535, try to change the DEBUG_AMOUNT and DEBUG_HITS
#endif
#endif /* CAP_DEBUG */

void Cap_Init(void) {
	memset(ADC_hits_val, 0, sizeof(ADC_hits_val));
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_hits_val, ADC_CHANNALS*ADC_HITS);
	HAL_TIM_PWM_Start(&OUT_VOL_PWM_TIM, OUT_VOL_PWM_CHANNEL);
	__HAL_TIM_SET_COMPARE(&OUT_VOL_PWM_TIM, OUT_VOL_PWM_CHANNEL, 0);
	HAL_TIM_PWM_Start(&IN_VOL_PWM_TIM, IN_VOL_PWM_CHANNEL);
	__HAL_TIM_SET_COMPARE(&IN_VOL_PWM_TIM, IN_VOL_PWM_CHANNEL, 0);
}

void Cap_Run(void) {
	Control_SuperCap.C_voltage = 100*Cap_Get_Cap_Voltage();
	Control_SuperCap.P_voltage = 100*Cap_Get_Power_Voltage();
	Control_SuperCap.P_Power = 100*Cap_Get_Power_Voltage()*Cap_Get_Power_CURR();
	
	Cap_Ctr();
	Cap_State();
	user_data.mask = 0xC0 | ((1 << ((int)(((VAL__CAP_VOLTAGE - RELEASE_VOLTAGE_MIN) / (RECHARGE_VOLTAGE_MAX - RELEASE_VOLTAGE_MIN)) * 6 + 1))) - 1);
}

void Cap_State_Switch(cap_state State) {
	switch (State) {
	case CAP_STATE_STOP:
		CapState = CAP_STATE_STOP, output_pwm_cmp = 0, input_pwm_cmp = 0;
		HAL_GPIO_WritePin(Cap_In_GPIO_Port, Cap_In_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Cap_Out_GPIO_Port, Cap_Out_Pin, GPIO_PIN_RESET);
		break;
	case CAP_STATE_RELEASE:
		CapState = CAP_STATE_RELEASE, input_pwm_cmp = 0;
		HAL_GPIO_WritePin(Cap_In_GPIO_Port, Cap_In_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Cap_Out_GPIO_Port, Cap_Out_Pin, GPIO_PIN_SET);
		break;
	case CAP_STATE_RECHARGE:
		CapState = CAP_STATE_RECHARGE, output_pwm_cmp = 0, recharge_cnt = 0;
		HAL_GPIO_WritePin(Cap_In_GPIO_Port, Cap_In_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(Cap_Out_GPIO_Port, Cap_Out_Pin, GPIO_PIN_RESET);
		break;
	}
}

double Cap_Get_Cap_Voltage(void) {
	return VAL__CAP_VOLTAGE;
}

//by zzy
double Cap_Get_Power_Voltage(void){
	return VAL__CAP_Power_Voltage;
}

double Cap_Get_Power_CURR(void){
	return VAL__CAP_Power_CURR;
}

cap_state Cap_Get_Cap_State(void) {
	return CapState;
}

/**
  * @brief  Set the pwm compare according to the state.
  * @param  None
  * @retval None
  */
void Cap_State() { // called with period of 2 ms
	switch (CapState) {
	case CAP_STATE_STOP:
		__HAL_TIM_SET_COMPARE(&OUT_VOL_PWM_TIM, OUT_VOL_PWM_CHANNEL, 0);
		__HAL_TIM_SET_COMPARE(&IN_VOL_PWM_TIM, IN_VOL_PWM_CHANNEL, 0);
		break;
	case CAP_STATE_RECHARGE:
		__HAL_TIM_SET_COMPARE(&OUT_VOL_PWM_TIM, OUT_VOL_PWM_CHANNEL, 0);
		__HAL_TIM_SET_COMPARE(&IN_VOL_PWM_TIM, IN_VOL_PWM_CHANNEL,input_pwm_cmp);
		break;
	case CAP_STATE_RELEASE:
		__HAL_TIM_SET_COMPARE(&OUT_VOL_PWM_TIM, OUT_VOL_PWM_CHANNEL, output_pwm_cmp);
		__HAL_TIM_SET_COMPARE(&IN_VOL_PWM_TIM, IN_VOL_PWM_CHANNEL, 0);
		break;
	}
}

/**
  * @brief  The detailed capacitance control logic.
  * @param  None
  * @retval None
  */
static void Cap_Ctr_STOP() {
#ifdef CAP_AUTO_RECHARGE
	if (VAL__CAP_VOLTAGE < RE_RECHARGE_VOLTAGE) {
		Cap_State_Switch(CAP_STATE_RECHARGE);
	}
#else
	(void)0;
	#ifdef CAP_DEBUG
		/*
			Control_SuperCap.C_voltage = 100*Cap_Get_Cap_Voltage();
	Control_SuperCap.P_voltage = 100*Cap_Get_Power_Voltage();
	Control_SuperCap.P_Power = 100*Cap_Get_Power_Voltage()*Cap_Get_Power_CURR();*/
    if(++cnt >=5 && ncnt < DEBUG_HITS){
      cps[0][ncnt] = PowerHeatData.chassisPower * 100;
      cps[1][ncnt] = PowerHeatData.chassisPowerBuffer  * 100;
      cps[2][ncnt] = Cap_Get_Power_Voltage()*Cap_Get_Power_CURR()*100;
      cps[3][ncnt++] = VAL__INPUT_PWM_PERCENT * 100;
      cnt = 0;
    }else if(ncnt >= DEBUG_HITS){
      HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
    }
#endif
#endif
	return;
}

static void Cap_Ctr_RECHARGE() {
	if (VAL__CAP_VOLTAGE > RECHARGE_VOLTAGE_MAX) {
		Cap_State_Switch(CAP_STATE_STOP);
	}
	else {
#ifdef CAP_DEBUG
		/*
			Control_SuperCap.C_voltage = 100*Cap_Get_Cap_Voltage();
	Control_SuperCap.P_voltage = 100*Cap_Get_Power_Voltage();
	Control_SuperCap.P_Power = 100*Cap_Get_Power_Voltage()*Cap_Get_Power_CURR();*/
    if(++cnt >=5 && ncnt < DEBUG_HITS){
      cps[0][ncnt] = PowerHeatData.chassisPower * 100;
      cps[1][ncnt] = PowerHeatData.chassisPowerBuffer  * 100;
      cps[2][ncnt] = Cap_Get_Power_Voltage()*Cap_Get_Power_CURR()*100;
      cps[3][ncnt++] = VAL__INPUT_PWM_PERCENT * 100;
      cnt = 0;
    }else if(ncnt >= DEBUG_HITS){
      HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
    }
#endif /* CAP_DEBUG */
		if (recharge_cnt < 250) {
			recharge_cnt++;
			FUNC__RECAL_INPUT_PWM(0.0005f);//0.0005f
		}
		else {
			FUNC__RECAL_INPUT_PWM(0.0015f);//0.002f
		}
		if (input_pwm_cmp > VAL__INPUT_PWM_MAX) {
			input_pwm_cmp = VAL__INPUT_PWM_MAX;
		}
		else if (input_pwm_cmp < 0) {
			input_pwm_cmp = 0;
		}
	}
}

static void Cap_Ctr_RELEASE() {
	if (VAL__CAP_VOLTAGE < RELEASE_VOLTAGE_MIN) {
		Cap_State_Switch(CAP_STATE_STOP);
	}
	else {

#ifdef CAP_DEBUG
    if(++cnt>=5 && ncnt<DEBUG_HITS){
      cps[0][ncnt] = PowerHeatData.chassisPower * 100;
      cps[1][ncnt] = PowerHeatData.chassisPowerBuffer  * 100;
      cps[2][ncnt] = Cap_Get_Power_Voltage()*Cap_Get_Power_CURR()*100;
      cps[3][ncnt++] = VAL__OUTPUT_PWM_PERCENT * 100;
      cnt = 0;
    }else if(ncnt >= DEBUG_HITS){
      HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
    }
#endif /* CAP_DEBUG */
		FUNC__RECAL_OUTPUT_PWM(0.0015f);//0.002f
		if (output_pwm_cmp > VAL__OUTPUT_PWM_MAX) {
			output_pwm_cmp = VAL__OUTPUT_PWM_MAX;
		}
		else if (output_pwm_cmp < 0) {
			output_pwm_cmp = 0;
		}
	}
}

/**
  * @brief  Control the release and recharge progress.
  * @param  None
  * @retval None
  */
void Cap_Ctr() { // called with period of 2 ms
	if (RobotState.remainHP < 1 || WorkState == STOP_STATE) {
		Cap_State_Switch(CAP_STATE_STOP);
	}
	else {
		switch (CapState) {
		case CAP_STATE_STOP:
			Cap_Ctr_STOP();
			break;
		case CAP_STATE_RECHARGE:
			Cap_Ctr_RECHARGE();
			break;
		case CAP_STATE_RELEASE:
			Cap_Ctr_RELEASE();
			break;
		}
	}
}

/**
  * @brief  Regular conversion complete callback in non blocking mode
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	int cnt1, cnt2;
	memset(ADC_tmp, 0, sizeof(ADC_tmp));
	for (cnt1 = 0; cnt1 < ADC_CHANNALS; cnt1++) {
		for (cnt2 = 0; cnt2 < ADC_HITS; cnt2++) {
			ADC_tmp[cnt1] += ADC_hits_val[cnt2][cnt1];
		}
		ADC_val[cnt1] = ADC_tmp[cnt1] / ADC_HITS;
	}
}
#ifdef CAP_DEBUG
uint8_t sendfinish=1;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
  if(sendfinish){
    sendfinish = 0;
    HAL_UART_Transmit_DMA(&huart8, (uint8_t*)cps, sizeof(cps));
  }
}
#endif /* CAP_DEBUG */

void LED_Show_SuperCap_Voltage(uint8_t flag)
{
	if (flag == 0)
	{
		HAL_GPIO_WritePin(GPIOG, 0x1fe, GPIO_PIN_SET);
		return;
	}
	if (Control_SuperCap.C_voltage < 1100)
		HAL_GPIO_WritePin(GPIOG, 0x1fe, GPIO_PIN_SET);
	else {
		HAL_GPIO_WritePin(GPIOG, 0x1fe, GPIO_PIN_SET);
		int unlight = 7 - (Control_SuperCap.C_voltage - 1100) / 143;
		if (unlight < 0) unlight = 0;
		HAL_GPIO_WritePin(GPIOG, 0x1fe >> unlight, GPIO_PIN_RESET);
	}
}
