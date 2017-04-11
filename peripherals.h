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


#define UARTBUFFERSIZE 64
#define UART_SOF '$'
#define UART_EOF '\r'
#define USART0_DATA_AVAILABLE 0xf5

typedef union
        {

	struct{

		uint8_t			TXready:1;		//For UART Only - It indicates when to Transmit
	  	uint8_t			RXready:1;		//For UART Only - It indicates when data is available
	  	uint8_t			Unused2:1;
	  	uint8_t			Unused3:1;
	  	uint8_t			Unused4:1;
	  	uint8_t			Enabled:1;       // Is the task enabled or not?
	  	uint8_t			Finished:1;
	  	uint8_t			Status:1;		//Does it need attention?
	}bits;

	uint8_t all;
    } OBJFLAGS;



/***********************************************************************************************//**
 * @addtogroup Application
 * @{
 **************************************************************************************************/


void InitPWM1(void);
void UpdatePWM1(uint32_t DutyCycle);
void PWMHandler(void);


void InitLEUART0(void);
void UART_Tx(uint8_t *buffer, uint16_t size);
void UART_RXHandler(void);
void UART_TXHandler(void);
void ClearSOFReceived();



#endif /* PERIPHERALS_H_ */

