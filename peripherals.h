/***********************************************************************************************//**
 * \file   peripherals.h
 * \brief  All the Hardware Peripheral Functions
 * \author Claudio Filho
 ***************************************************************************************************
 * <b> (C) Copyright 2017 Silicon Labs, http://www.silabs.com</b>
 ***************************************************************************************************
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 **************************************************************************************************/

#ifndef PERIPHERALS_H_
#define PERIPHERALS_H_


/***********************************************************************************************//**
 * @addtogroup Peripheral Definitions
 * @{
 **************************************************************************************************/

/* PWM frequency */
#define PWM_FREQ 20000

#define USART0_DATA_AVAILABLE 0xf5

/***********************************************************************************************//**
 * @addtogroup Application
 * @{
 **************************************************************************************************/
void InitUSART0(void);

void InitLEUART0(void);
void UART_Tx(uint8_t *buffer, uint16_t size);


void InitPWM1(void);
void UpdatePWM1(uint32_t desiredDutyCycle);


#endif /* PERIPHERALS_H_ */

