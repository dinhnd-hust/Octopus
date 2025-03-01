/*
 * Octopus.c
 *
 *  Created on: Dec 19, 2024
 *      Author: dinhbkvn
 */

#include "_BMS.h"
#include "_HMI.h"
#include "_Battery.h"
#include "_Speaker.h"
#include "_Fan.h"
#include "_NumberofCharge.h"
#include "main.h"
#include "globalDefine.h"
#include "stdio.h"
#include "myADC.h"
#include "myUART.h"
#include "_CellBalancing.h"
#include "myBQ76942_I2C.h"
#include "myI2CLCD.h"
#include "myPWM.h"
#include "_controller.h"

extern Battery_TypeDef	thisBattery;
extern HMI_TypeDef		thisHMI;
extern Speaker_TypeDef	thisSpeaker;
extern CellBalancing_TypeDef	thisCellBalancing;

BMS_TypeDef			thisBMS;
AdcTypeDef			thisADC;
PwmConfig			thisPWM;
FAN_TypeDef			thisFAN;
NumCharge_TypeDef	thisPriMCU;
Controller_FixPoint_TypeDef	CC_Controller;
Controller_FixPoint_TypeDef	CV_Controller;
ChargerTypeDef		thisCharger;

volatile _Bool flgCharging_Button_Pressed 	= false;
volatile _Bool flgBalancing_Button_Pressed 	= false;
volatile _Bool flgStop_Button_Pressed 		= false;
volatile _Bool flgChargeCount_Increased		= false;

extern char BufTxUart[UART_TX_BUFFER_SIZE];
extern char BufRxUart[UART_RX_BUFFER_SIZE];
extern uint16_t 	g_cnt_stateSW;
extern uint32_t ISR_Duty;
extern uint16_t g_starttime;
extern uint16_t g_endtime;
extern uint16_t g_elapsetime;
void _system_initialization(){
	/*
	 * Initializing user's definition
	 */
	adcInit(&thisADC);
	Battery_Init(&thisBattery);
	HMI_Init(&thisHMI);
	Speaker_Init(&thisSpeaker);
	BMS_Init();
	CellBalancing_Init(&thisCellBalancing);
	_fan_Init(&thisFAN);
	numchargeInit(&thisPriMCU);
	_statechargerInit(&thisCharger);
	_Controller_Init(&CC_Controller, &CV_Controller);
	HMI_Update_Display(&thisHMI);
	BQ769x2_Init();
	pwmInit(&thisPWM);
	HAL_Delay(1000);

	/*
	 * Initializing peripherals
	 */
	HAL_ADCEx_Calibration_Start(&hadc);
	HAL_ADC_Start_DMA(&hadc, (uint32_t*) &thisADC.BufAdc, ADC_BUF_MAX_ELEMENTS);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_UART_Transmit_DMA(&huart, (uint8_t *) &BufTxUart, UART_TX_BUFFER_SIZE);
	HAL_UART_Receive_DMA(&huart, (uint8_t *) &BufRxUart, UART_RX_BUFFER_SIZE);
	HAL_TIM_Base_Start(&htim_PWM);
	HAL_TIM_Base_Start(&htim_FAN);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
	HAL_TIM_Base_Start(&htim_timcal);
	/*
	 * Initializng number of charge from primary MCU' Memory
	 */
	return;
}

void _ext_interrupt_service(uint16_t GPIO_Pin) {
	if(GPIO_Pin == userSW_Pin){
		g_cnt_stateSW++;
	}
    return;
}

void _main_interrupt_service(){	//100us = 1.8 degrees
	static uint16_t cnt_1ms 			= 0;
	/* 1ms clocking tasks */
	/*
	 * Time calculation: Tcy = 248 ~ 248/64e6 = 3,875 us
	 */
	if (cnt_1ms++ >= 10U) { //1ms clock
	  cnt_1ms = 1;

	  if (cnt_Led_Update++ >= blinkPeriod) {
		  cnt_Led_Update = 0;
		  HAL_GPIO_TogglePin(userLED_GPIO_Port, userLED_Pin);
	   }
	  if (thisSpeaker.updateCounter++ > thisSpeaker.updatePeriod) {
		  thisSpeaker.updateCounter  	= 0;
		  thisSpeaker.updateFlag	= true;
	  }
	  if (thisCellBalancing.updateCounter++ > thisCellBalancing.updatePeriod){
		  thisCellBalancing.updateCounter  	= 0;
		  thisCellBalancing.updateFlag	= true;
	  }
	  if (thisPriMCU.updateCounter++ > thisPriMCU.updatePeriod){
		  thisPriMCU.updateCounter  	= 0;
		  thisPriMCU.updateFlag	= true;
	  }
	  if (thisHMI.updateCounter++ > thisHMI.updatePeriod) {
		  thisHMI.updateCounter  	= 0;
		  thisHMI.updateFlag	= true;
	  }
	  if (thisFAN.updateCounter++ > thisFAN.updatePeriod) {
		  thisFAN.updateCounter  	= 0;
		  thisFAN.updateFlag	= true;
	  }
 }

//---------------------------------------------------------
 	 /*
 	  * Time calculation: Tcy = 214 ~ 214/64e6 = 3,34375 us
 	  */
	/*-------------------------- Read ADC here -------------------------------*/
 	//adcRead(&thisADC);
	adcRead(&thisADC);
 	adcWaitforconversion();

	/*-------------------------- Update BMS and Charger ----------------------*/
	 /*
	  * Time calculation: Tcy = 620 ~ 620/64e6 = 9,6875 us
	  */
	BMS_Status_Update();
	_statechargerUpdate(&thisCharger, thisADC, thisBattery.batVoltage);
	if(thisCharger.ChargerState == STATE_PROTECTION){
//		MY_PWM_STOP();
//		MY_FAN_STOP();
	}
	//FIXME: Erase this code after debugging
//	flgBalancing_Button_Pressed = true;
 	/*
 	 * TODO: Debug for PWM
 	 */
#ifdef OPEN_LOOP_TEST

    static uint16_t first_time = 1;
    static uint32_t Duty = 0;
    if(first_time){
    	pwmSetpfm(&thisPWM);
    	MY_PWM_START();
        first_time = 0;
    }
    if(ISR_Duty>=100){
    	_Controller_RampUp(392, 1, &Duty);
    	thisPWM.duty = Duty;
//    	_Charging_OpenLoop(2016, thisADC.BufAdcBattVoltage, &thisPWM);
    	pwmSetpfm(&thisPWM);
    	ISR_Duty = 0;
    }
    else {ISR_Duty++;}

#elif defined(CLOSE_LOOP_TEST)
  static uint16_t first_time = 1;
//    static uint16_t cnt_ISR = 0;
//    static uint16_t SS = 0;

  if(first_time){
	  MY_PWM_START();
	  pwmSetpfm(&thisPWM);
      first_time = 0;
  }
    if(cnt_ISR>=20000 && SS == 0){
    	_Charging_OpenLoop(2016, 1020, &thisPWM);
    	MY_PWM_START();
    	if (thisADC.BufAdcBattVoltage > 778){
    		SS = 1;
    	}
    }
    else {cnt_ISR++;}

    if (SS == 1){
    _Controller_CC_OnlyRunning(&CC_Controller, thisADC, &thisPWM);
    }


	/*
	 * TODO: Debug for controller
	 */
	/*-------------------------- Charger's controller CC-CV ------------------*/
	 /*
	  * Time calculation - DISABLE: Tcy = 58 ~ 58/64e6 = 0,90625 us
	  * Time calculation - ENABLE : Tcy = 544 ~ 544/64e6 = 8,5 us
	  */

#else
  switch (thisCharger.ChargerState) {
  	  case STATE_BATTERY_CHECK:
  		  _Charging_State_BatteryCheck();
  		  break;
  	  case STATE_SOFT_START:
  		  _Charging_OpenLoop(thisBattery.batVoltage, thisADC.BufAdcBattVoltage, &thisPWM);
  		  break;
  	  case STATE_HAND_SHAKE:
  		  _Charging_State_HandShake();
  		  break;
  	  case STATE_CHARGING:
  		  if (cnt_interrupt >= DELAY_RELAY){
  			  _LED1_ON();
  		      myPWM_Start();
  		      //_Controller_CV_CC(&CV_Controller, &CC_Controller, thisADC, &thisPWM);
  		      _Controller_CC_OnlyRunning(&CC_Controller, thisADC, &thisPWM);
  		  }
  		  else cnt_interrupt++;
  		      break;
  	  case STATE_FULCHARGED:
  		  _Charging_State_FulCharged();
  		      break;
  	  case STATE_PROTECTION:
  		  _Charging_State_Protection();
  			  break;
  			  default:
  				  break;
  		    }
#endif



 	/*
 	 * Debug for FAN
 	 */
	/*-------------------------- Charger's controller CC-CV ------------------*/
	 /*
	  * Time calculation: Tcy = 74 ~ 74/64e6 = 1,15625 us
	  */
	adcResumeconversion();
 	g_starttime = TIM2->CNT;
	g_endtime	= TIM2->CNT;
	g_elapsetime = g_endtime - g_starttime;
	return;
}

void _main_inf_loop(){
	BQ769x2_ReadFETStatus();
	if(g_cnt_stateSW == 1){
		flgCharging_Button_Pressed = true;
		thisCharger.State	= ENABLE;
//		BMS_Increase_NumofCharge();
	}
	if(g_cnt_stateSW == 2){
		flgBalancing_Button_Pressed = true;
	}
	if(g_cnt_stateSW == 3){
		flgStop_Button_Pressed = true;
		thisCharger.State	= DISABLE;
		g_cnt_stateSW = 0;
	}

	if (thisHMI.updateFlag) {
		thisHMI.updateFlag = false;
		HMI_Update_Message(&thisHMI);
		HMI_Update_Display(&thisHMI);
	}

	if (thisFAN.updateFlag) {
		thisFAN.updateFlag = false;
		//_fan_Updatespeed(&thisFAN);
		_fan_Setspeed(thisFAN);
	}

	if (thisPriMCU.updateFlag) {
		thisPriMCU.updateFlag = false;
//		BMS_Read_NumofCharge();
		// FIXME: Erase this code after testing
//		if ( (thisCharger.UpdateChargeCount == true) && (flgChargeCount_Increased == false)){
//			BMS_Increase_NumofCharge();
//			flgChargeCount_Increased = true;
//		}
	}

	if (thisCellBalancing.updateFlag){
		thisHMI.updateFlag = false;
		BQ769x2_Measure_Variables();
		_Battery_Reading_cellVoltage(&thisBattery);
		_Battery_Reading_chargerVoltage(&thisBattery);
		_Battery_Reading_batVoltage(&thisBattery);
		_Battery_Reading_batCurrent(&thisBattery);
		_Battery_Reading_batTemperature(&thisBattery);
		_Battery_Reading_cellVoltageMaxMin(&thisBattery);
		_Battery_Reading_deltacellVoltageMax(&thisBattery);
		// Fixme: Erase this code after completing debug
		thisBMS.adaptorVoltage = 38000;
	}

	if (thisSpeaker.updateFlag) {
		thisSpeaker.updateFlag = false;
		Speaker_Update(&thisSpeaker);
		_speakerOff();
	}

//	snprintf(BufTxUart, sizeof(BufTxUart), "Vpack: %d, Temp: %d, Vbat: %d, Ibat: %d, I: %d, Vbat: %d, c1: %d, c2: %d, c3: %d, c4: %d, c5: %d, c6: %d, c7: %d, c8: %d, c9: %f\r\n" ,thisADC.BufAdc[0], thisADC.BufAdc[1], thisADC.BufAdc[2], thisADC.BufAdc[3],
//						thisBattery.batCurrent, thisBattery.batVoltage, thisBattery.cellVoltage[0], thisBattery.cellVoltage[1], thisBattery.cellVoltage[2],
//						thisBattery.cellVoltage[3], thisBattery.cellVoltage[4], thisBattery.cellVoltage[5], thisBattery.cellVoltage[6], thisBattery.cellVoltage[7], thisBattery.batTemperature[0]);
//	snprintf(BufTxUart, sizeof(BufTxUart), "Vpack: %d, Temp: %d, Vbat: %d, Ibat: %d\r\n" ,thisADC.BufAdc[0], thisADC.BufAdc[1], thisADC.BufAdc[2], thisADC.BufAdc[3]);
	return;
}

void BMS_Status_Update(){
	if (thisBMS.Status.Shadow != thisBMS.Status.Value) {
		thisBMS.Status.Shadow = thisBMS.Status.Value;
		thisBMS.Status.flgStatusChanged = true;
	}
	if (thisBMS.Status1.Shadow != thisBMS.Status1.Value) {
		thisBMS.Status1.Shadow = thisBMS.Status1.Value;
		thisBMS.Status1.flgStatusChanged = true;
	}
	/*
	 * Check for system status first
	 */
	 // Under voltage error
	if (thisBMS.adaptorVoltage < ADAPTOR_VOLTAGE_MIN)
		thisBMS.Status.Value |= BMS_STT_ERROR_100;
	else
		thisBMS.Status.Value &= ~BMS_STT_ERROR_100;
	// Over voltage error
	if (thisBMS.adaptorVoltage > ADAPTOR_VOLTAGE_MAX)
		thisBMS.Status.Value |= BMS_STT_ERROR_101;
	else
		thisBMS.Status.Value &= ~BMS_STT_ERROR_101;
	// pack voltage too low
	if (thisBattery.batVoltage < BATTERY_VOLTAGE_MIN)
		thisBMS.Status.Value |= BMS_STT_ERROR_200;
	else
		thisBMS.Status.Value &= ~BMS_STT_ERROR_200;
	// pack voltage too high
	if (thisBattery.batVoltage > BATTERY_VOLTAGE_MAX)
		thisBMS.Status.Value |= BMS_STT_ERROR_201;
	else
		thisBMS.Status.Value &= ~BMS_STT_ERROR_201;
	// limit reached
	if (thisBattery.numberOfCharge > BATTERY_CHARGE_TIME_LIMIT)
		thisBMS.Status.Value |= BMS_STT_ERROR_202;
	else
		thisBMS.Status.Value &= ~BMS_STT_ERROR_202;
	// cell voltage too small
	if (thisBattery.minCellVoltage < CELL_VOLTAGE_MIN)
		thisBMS.Status.Value |= BMS_STT_ERROR_203;
	else
		thisBMS.Status.Value &= ~BMS_STT_ERROR_203;
	// cell voltage too large
	if (thisBattery.maxCellVoltage > CELL_VOLTAGE_MAX)
		thisBMS.Status.Value |= BMS_STT_ERROR_204;
	else
		thisBMS.Status.Value &= ~BMS_STT_ERROR_204;
	// unbalance too much
	if (thisBattery.cellUnbalance > CELL_VOLTAGE_DEVIATION_MAX)
		thisBMS.Status.Value |= BMS_STT_ERROR_205;
	else
		thisBMS.Status.Value &= ~BMS_STT_ERROR_205;

	// About to reach the charge time limitation
	if (thisBattery.numberOfCharge > BATTERY_CHARGE_TIME_WARNING)
		thisBMS.Status.Value |= BMS_STT_WARNING_300;
	else
		thisBMS.Status.Value &= ~BMS_STT_WARNING_300;
	// Significant unbalance
	if (thisBattery.cellUnbalance > CELL_VOLTAGE_DEVIATION_MIN)
		thisBMS.Status.Value |= BMS_STT_WARNING_301;			// user should run balancing first
	else
		thisBMS.Status.Value &= ~BMS_STT_WARNING_301;


	/*
	 * battery's charging status
	 */
	// Battery is not connected
	if (thisBattery.batVoltage <= BATTERY_NOT_PRESENCE)
		thisBMS.Status1.Value |= BMS_STT_NO_BATTERY_DETECTED;
	else
		thisBMS.Status1.Value &= ~BMS_STT_NO_BATTERY_DETECTED;
	// Battery is already full
	if (thisBattery.batVoltage >= BATTERY_VOLTAGE_FULL)
		thisBMS.Status1.Value |= BMS_STT_BATTERY_FULL;
	else
		thisBMS.Status1.Value &= ~BMS_STT_BATTERY_FULL;

	if (flgStop_Button_Pressed) {
		flgStop_Button_Pressed = false; //acknowledge the button
		thisBMS.Status1.Value |= BMS_STT_STOP_BUTTON_PRESSED;
//		thisBMS.Status1.Value |= BMS_STT_IS_BALANCING;
	}

	if (flgCharging_Button_Pressed) {
		flgCharging_Button_Pressed = false; //acknowledge the button
		thisBMS.Status1.Value |= BMS_STT_CHARGE_BUTTON_PRESSED;
	}
	if (flgBalancing_Button_Pressed) {
		flgBalancing_Button_Pressed = false; //acknowledge the button
		thisBMS.Status1.Value |= BMS_STT_BALANCE_BUTTON_PRESSED;
	}


	/*
	 * Now let us check for operating condition
	 */

	return;
}

void BMS_Init(){
	thisBMS.Status.Value		= BMS_STT_SYSTEM_READY;
	thisBMS.fanSpeed			= 0;		// FAN is OFF.
	thisBMS.adaptorVoltage		= 38000U;
}

void BMS_Read_NumofCharge(){
	snprintf(BufTxUart, sizeof(BufTxUart), "3");
	snprintf(BufTxUart, sizeof(BufTxUart), "4");
	sscanf(BufRxUart, "%hu", &thisBattery.numberOfCharge);
	return;
}

void BMS_Increase_NumofCharge(){
	snprintf(BufTxUart, sizeof(BufTxUart), "1");
	snprintf(BufTxUart, sizeof(BufTxUart), "2");
	sscanf(BufRxUart, "%hu", &thisBattery.numberOfCharge);
	return;
}

void _Charging_State_BatteryCheck(){
	MY_PWM_STOP();
	MY_FAN_STOP();
	pwmSetpfm(&thisPWM);
}

void _Charging_State_HandShake(){
	MY_PWM_STOP();
	CommandSubcommands(FET_ENABLE);
	CommandSubcommands(ALL_FETS_ON);
}

void _Charging_State_Protection(){
	MY_PWM_STOP();
	MY_FAN_STOP();
	CommandSubcommands(ALL_FETS_OFF);
}

void _Charging_State_FulCharged(){
	MY_PWM_STOP();
	CommandSubcommands(ALL_FETS_OFF);
	MY_FAN_STOP();
	thisFAN.duty = 1600;
	_fan_Setspeed(thisFAN);
}
