/*
 * __Octopus.h
 *
 *  Created on: Dec 19, 2024
 *      Author: dinhbkvn
 */

#ifndef SRC__BMS_H_
#define SRC__BMS_H_


#include "stdint.h"

#define ADAPTOR_VOLTAGE_MIN	34200U
#define ADAPTOR_VOLTAGE_MAX	41580U

#define	BMS_STT_SYSTEM_READY 	 0x00000000
#define	BMS_STT_ERROR_100		 ((uint32_t) 1U << 0)	//
#define	BMS_STT_ERROR_101		 ((uint32_t) 1U << 1)
#define	BMS_STT_ERROR_102		 ((uint32_t) 1U << 2)
#define	BMS_STT_ERROR_103		 ((uint32_t) 1U << 3)
#define	BMS_STT_ERROR_104		 ((uint32_t) 1U << 4)
#define	BMS_STT_ERROR_105		 ((uint32_t) 1U << 5)
#define	BMS_STT_ERROR_106		 ((uint32_t) 1U << 6)	// reserve
#define	BMS_STT_ERROR_107		 ((uint32_t) 1U << 7)
#define	BMS_STT_ERROR_200		 ((uint32_t) 1U << 8)
#define	BMS_STT_ERROR_201		 ((uint32_t) 1U << 9)
#define	BMS_STT_ERROR_202		 ((uint32_t) 1U << 10)
#define	BMS_STT_ERROR_203		 ((uint32_t) 1U << 11)
#define	BMS_STT_ERROR_204		 ((uint32_t) 1U << 12)
#define	BMS_STT_ERROR_205		 ((uint32_t) 1U << 13)
#define	BMS_STT_ERROR_206		 ((uint32_t) 1U << 14)
#define	BMS_STT_ERROR_207		 ((uint32_t) 1U << 15)
#define	BMS_STT_WARNING_300		 ((uint32_t) 1U << 16)
#define	BMS_STT_WARNING_301		 ((uint32_t) 1U << 17)
#define	BMS_STT_WARNING_302		 ((uint32_t) 1U << 18)
#define	BMS_STT_WARNING_303		 ((uint32_t) 1U << 19)
#define	BMS_STT_WARNING_304		 ((uint32_t) 1U << 20)
#define	BMS_STT_WARNING_305		 ((uint32_t) 1U << 21)
#define	BMS_STT_WARNING_306		 ((uint32_t) 1U << 22)
#define	BMS_STT_WARNING_307		 ((uint32_t) 1U << 23)
#define	BMS_STT_WARNING_400		 ((uint32_t) 1U << 24)
#define	BMS_STT_WARNING_401		 ((uint32_t) 1U << 25)
#define	BMS_STT_WARNING_402		 ((uint32_t) 1U << 26)
#define	BMS_STT_WARNING_403		 ((uint32_t) 1U << 27)
#define	BMS_STT_WARNING_404		 ((uint32_t) 1U << 28)
#define	BMS_STT_WARNING_405		 ((uint32_t) 1U << 29)
#define	BMS_STT_WARNING_406		 ((uint32_t) 1U << 30)
#define	BMS_STT_WARNING_407		 ((uint32_t) 1U << 31)

#define BMS_STT_IDLE					 ((uint32_t) 1U << 0)
#define	BMS_STT_NO_BATTERY_DETECTED		 ((uint32_t) 1U << 1)
#define	BMS_STT_BATTERY_FULL			 ((uint32_t) 1U << 2)
#define	BMS_STT_CHARGE_BUTTON_PRESSED 	 ((uint32_t) 1U << 3)
#define	BMS_STT_BALANCE_BUTTON_PRESSED 	 ((uint32_t) 1U << 4)
#define BMS_STT_IS_BALANCING			 ((uint32_t) 1U << 5)
#define	BMS_STT_STOP_BUTTON_PRESSED		 ((uint32_t) 1U << 6)
#define BMS_STT_IS_CHARGING			     ((uint32_t) 1U << 7)

#define OPEN_LOOP_TEST
//#define CLOSE_LOOP_TEST

typedef struct {
	uint32_t 	Value;
	uint32_t 	Shadow;
	_Bool		flgStatusChanged;
} BMS_Status_TypeDef;

typedef struct {
	BMS_Status_TypeDef Status;
	BMS_Status_TypeDef Status1;
	uint16_t	temp[3];
	_Bool		pushButton[3];
	uint16_t	fanSpeed;
	uint16_t	adaptorVoltage;
} BMS_TypeDef;

void _main_interrupt_service();
void _ext_interrupt_service(uint16_t GPIO_Pin);
void _system_initialization();
void _main_inf_loop();

void BMS_Init();
void BMS_Status_Update();
void BMS_Read_NumofCharge();
void BMS_Increase_NumofCharge();
#endif /* SRC__BMS_H_ */
