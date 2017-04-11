/***********************************************************************************************//**
 * \file   peripherals.c
 * \brief  All the Hardware Peripheral Functions
 * \author Claudio Filho
 ***************************************************************************************************
 * <b> (C) Copyright 2017 Silicon Labs, http://www.silabs.com</b>
 ***************************************************************************************************
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 **************************************************************************************************/

/*Application Specific Code*/
#include "em_timer.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "native_gecko.h"
#include "peripherals.h"
#include "em_usart.h"
#include "em_leuart.h"
#include <string.h>

OBJFLAGS PWMObj;
OBJFLAGS UARTObj;
GPIOS UserGPIOs;

/**************************************************************************//**
 * @brief GPIO Related Functions  and variables
 *
 *****************************************************************************/


void InitGPIO(void) {

	// $[Port A Configuration]

	/*UART Function Related GPIOs*/

	//LE UART TX - PIN P9 on WSTK
	GPIO_PinModeSet(gpioPortA, 0, gpioModePushPull, 1);

	//LE UART RX - PIN P37 on WSTK
	GPIO_PinModeSet(gpioPortD, 9, gpioModeInput, 0);


	/*PWM Function Related GPIOs*/

	//PWM Output - PIN P7 on WSTK
	GPIO_PinModeSet(gpioPortC, 9, gpioModePushPull, 0);


	/*General Purpose Function Related GPIOs*/

	//UIF LED0 - PIN P30 on WSTK
	/* Pin PF4 is configured to Push-pull */
	GPIO_PinModeSet(gpioPortF, 4, gpioModePushPull, 0);
	UserGPIOs.bits.LED0 = 0;	//Logic Definition

	//UIF LED1 - PIN P32 on WSTK
	/* Pin PF5 is configured to Push-pull */
	GPIO_PinModeSet(gpioPortF, 5, gpioModePushPull, 0);
	UserGPIOs.bits.LED1 = 0;	//Logic Definition

	//UIF PB0 - PIN P34 on WSTK
	/* Pin PF6 is configured to Input enabled with filter */
	GPIO_PinModeSet(gpioPortF, 6, gpioModeInput, 1);
	UserGPIOs.bits.PB0 = 0;	//Logic Definition

	//UIF PB1 - PIN P36 on WSTK
	/* Pin PF7 is configured to Input enabled with filter */
	GPIO_PinModeSet(gpioPortF, 7, gpioModeInput, 1);
	UserGPIOs.bits.PB1 = 0;	//Logic Definition

	// [Port F Configuration]$

	/*Other GPIOs*/

	/* Pin PA1 is configured to Input enabled */
	GPIO_PinModeSet(gpioPortA, 1, gpioModeInput, 0);

	/* Pin PA3 is configured to Push-pull */
	GPIO_PinModeSet(gpioPortA, 3, gpioModePushPull, 0);

	/* Pin PA5 is configured to Push-pull */
	GPIO_PinModeSet(gpioPortA, 5, gpioModePushPull, 1);
	// [Port A Configuration]$



}

void GPIOHandler(void)
{
	if (UserGPIOs.bits.LED0 != GPIO_PinOutGet(gpioPortF, 4))
		{
			if (UserGPIOs.bits.LED0==1)  GPIO_PinOutSet(gpioPortF, 4);
			else  GPIO_PinOutClear(gpioPortF, 4);
		}

	if (UserGPIOs.bits.LED1 != GPIO_PinOutGet(gpioPortF, 5))
		{
			if (UserGPIOs.bits.LED1==1)  GPIO_PinOutSet(gpioPortF, 5);
			else  GPIO_PinOutClear(gpioPortF, 5);
		}

	if(GPIO_PinInGet(gpioPortF,6)==0) UserGPIOs.bits.PB0=1;
	else UserGPIOs.bits.PB0=0;

	if(GPIO_PinInGet(gpioPortF,7)==0) UserGPIOs.bits.PB1=1;
	else UserGPIOs.bits.PB1=0;

}

/**************************************************************************//**
 * @brief PWM Related Functions  and variables
 *
 *****************************************************************************/
static uint32_t desiredDutyCycle;

void InitPWM1()
{
#if ClockNotdefinedonInit_c
	/* Enable clock for HF peripherals */
	CMU_ClockEnable(cmuClock_HFPER, true);

	 /* run the HFPERCLK at divide-by-1 */
	 CMU_ClockDivSet(cmuClock_HFPER, cmuClkDiv_1);
#endif

	/* Enable clock for TIMER1 */
	CMU_ClockEnable(cmuClock_TIMER1, true);


	// $[TIMER1 I/O setup]
	/* Set up CC0 */
	TIMER1->ROUTELOC0 |= TIMER_ROUTELOC0_CC0LOC_LOC14;
	TIMER1->ROUTEPEN |= TIMER_ROUTEPEN_CC0PEN;

	TIMER_Init_TypeDef init = TIMER_INIT_DEFAULT;

	init.enable = 0;
	init.debugRun = 1;
	init.dmaClrAct = 0;
	init.sync = 0;
	init.clkSel = timerClkSelHFPerClk;
	init.prescale = timerPrescale1;
	init.fallAction = timerInputActionNone;
	init.riseAction = timerInputActionNone;
	init.mode = timerModeUp;
	init.quadModeX4 = 0;
	init.oneShot = 0;
	init.count2x = 0;
	init.ati = 0;
	TIMER_Init(TIMER1, &init);
	// [TIMER1 initialization]$

	// $[TIMER1 CC0 init]
	TIMER_InitCC_TypeDef initCC0 = TIMER_INITCC_DEFAULT;

	initCC0.prsInput = false;
	initCC0.prsSel = timerPRSSELCh0;
	initCC0.edge = timerEdgeRising;
	initCC0.mode = timerCCModePWM;
	initCC0.eventCtrl = timerEventEveryEdge;
	initCC0.filter = 0;
	initCC0.cofoa = timerOutputActionNone;
	initCC0.cufoa = timerOutputActionNone;
	initCC0.cmoa = timerOutputActionNone;
	initCC0.coist = 0;
	initCC0.outInvert = 0;
	TIMER_InitCC(TIMER1, 0, &initCC0);
	// [TIMER1 CC0 init]$0

	//PWM Output - PIN P7 on WSTK
	GPIO_PinModeSet (gpioPortC, 9, gpioModePushPull, 0);

	PWMObj.all=0;
	PWMObj.bits.Enabled=1;

}

void UpdatePWM1(uint32_t DutyCycle) //desiredDutyCycle varies from 0-100;
{
	desiredDutyCycle = DutyCycle;
	PWMObj.bits.Status=1;
}

void ChangePWMoutput() //desiredDutyCycle varies from 0-100;
{
	uint32_t PWMFrequency;
	uint32_t CC1DutyCycle;


	PWMFrequency = CMU_ClockFreqGet (cmuClock_HFPER) / PWM_FREQ;

	if (desiredDutyCycle>0)
	{

		CC1DutyCycle = (desiredDutyCycle*PWMFrequency)/100;
		CC1DutyCycle =CC1DutyCycle ;

	}
	else  {CC1DutyCycle=0;}

	TIMER_Enable(TIMER1,0);	//Disables Timer

	/* set PWM period */
	TIMER_TopSet (TIMER1, PWMFrequency);

	/* Set PWM duty cycle to 50% */
	TIMER_CompareSet (TIMER1, 0, CC1DutyCycle);

	CC1DutyCycle = CC1DutyCycle;

	TIMER_Enable(TIMER1,1);

}

void PWMHandler(void)
{
	if( (PWMObj.bits.Enabled==1) && (PWMObj.bits.Status==1))
	{
		ChangePWMoutput();
		PWMObj.bits.Status=0;
	}

}


/**************************************************************************//**
 * @brief UART Related Functions and variables
 *
 *****************************************************************************/

/* UART VARIABLES */



static uint32_t leuartif;
uint8_t UARTbuffer[UARTBUFFERSIZE];
static uint8_t UARTbufferctr;


uint32_t SOFReceived=0;


void InitLEUART0(void) {


	// $[LEUART0 initialization]
	LEUART_Init_TypeDef initleuart = LEUART_INIT_DEFAULT;


	/* Enable clock for LEUART0 */
	CMU_ClockEnable(cmuClock_LEUART0, true);

	initleuart.enable = leuartEnable;
	initleuart.baudrate = 9600;
	initleuart.databits = leuartDatabits8;
	initleuart.parity = leuartNoParity;
	initleuart.stopbits = leuartStopbits1;
	LEUART_Init(LEUART0, &initleuart);

	/* Configuring non-standard properties */
	//LEUART_TxDmaInEM2Enable(LEUART0, 1);
	//LEUART_RxDmaInEM2Enable(LEUART0, 1);


	/* Set up RX pin */
	LEUART0->ROUTELOC0 = (LEUART0->ROUTELOC0 & (~_LEUART_ROUTELOC0_RXLOC_MASK))
			| LEUART_ROUTELOC0_RXLOC_LOC16; //PD9
	LEUART0->ROUTEPEN = LEUART0->ROUTEPEN | LEUART_ROUTEPEN_RXPEN;

	/* Set up TX pin */
	LEUART0->ROUTELOC0 = (LEUART0->ROUTELOC0 & (~_LEUART_ROUTELOC0_TXLOC_MASK))
			| LEUART_ROUTELOC0_TXLOC_LOC0; //PA0
	LEUART0->ROUTEPEN = LEUART0->ROUTEPEN | LEUART_ROUTEPEN_TXPEN;

	  /* Set LEUART signal frame */
	  LEUART0->STARTFRAME = UART_SOF;

	  /* Set LEUART signal frame */
	  LEUART0->SIGFRAME = UART_EOF;

	  //Start-Frame Unblock set - Clears RXBLOCK when the start-frame is found in the incoming data. The start-frame is loaded into the receive buffer.
		LEUART0->CTRL |= 0x100;

	  // leuartif = LEUART_IntGet(LEUART0);
	  LEUART_IntClear(LEUART0, 0);

	  //LEUART_IEN_SIGF
	  /* Enable LEUART Start Frame Interrupt */
	  	  	  LEUART_IntEnable(LEUART0, LEUART_IEN_STARTF );

	 /* Enable LEUART any byte received Interrupt */
	  	  LEUART_IntEnable(LEUART0, LEUART_IEN_RXDATAV );
	  		  /* Enable LEUART Signal Frame Interrupt */
	  		//  LEUART_IntEnable(LEUART0, LEUART_IEN_SIGF);

	  /* Enable LEUART1 interrupt vector */
	  //NVIC_EnableIRQ(LEUART0_IRQn);


	// [LEUART0 initialization]$

	  	ClearSOFReceived();
		UARTObj.all=0;
		UARTObj.bits.Enabled=1;



}

void UART_Tx(uint8_t * buffer, uint16_t size)
{
	uint16_t counter;

	for(counter=0; counter<size; counter++)
	{
		UARTbuffer[counter] = * buffer++;
	}

	UARTObj.bits.TXready=1;

}



void UART_TxOut(uint8_t * buffer, uint16_t size)
{
	uint16_t counter;

	for(counter=0; counter<size; counter++)
	{
		LEUART_Tx(LEUART0, *buffer++);

	}

}




/**************************************************************************//**
 * @brief LEUART IRQ handler
 *
 * When the signal frame is detected by the LEUART, this interrupt routine will
 * zero-terminate the char array, write the received string the to the LCD, and
 * reset the DMA for new data.
 *
 *****************************************************************************/
void LEUART0_IRQHandler(void)
{
  /* Store and reset pending interupts */
  leuartif = LEUART_IntGet(LEUART0);
  LEUART_IntClear(LEUART0, leuartif);
  gecko_external_signal(USART0_DATA_AVAILABLE);

#if 0
  /* Signal frame found. */
  if (leuartif & LEUART_IF_SIGF)
  {
    /* Zero-terminate rx buffer */
    len            = BUF_MAX - 1 - ((dmaControlBlock->CTRL >> 4) & 0x3FF);
    rxbuf[len - 1] = 0;

    /* Reactivate DMA */
    DMA_ActivateBasic(DMA_CHANNEL,     /* Activate DMA channel 0 */
                      true,            /* Activate using primary descriptor */
                      false,           /* No DMA burst */
                      NULL,            /* Keep source */
                      NULL,            /* Keep destination */
                      BUF_MAX - 1);    /* Number of DMA transfer elements (minus 1) */


  }
#endif

}



void ClearSOFReceived()
{
	SOFReceived=0;
}


void UART_TXHandler(void)
{
	if(UARTObj.bits.Enabled==1)
	{
		if(UARTObj.bits.TXready==1)
		{
			UART_TxOut((uint8_t *)UARTbuffer, UARTBUFFERSIZE);
			UARTObj.bits.TXready=0;
		}
	}
}


void UART_RXHandler(void)
{

	if(UARTObj.bits.Enabled==1)
	{
		  if((LEUART0->IF & 0x200) ==0x200)
		  {
			  UARTbufferctr=0;
			  LEUART_IntClear(LEUART0, 0x200);


			  SOFReceived =1;


		  }

		  if (((LEUART0->IF & 0x4) ==0x4) && (SOFReceived==1))
		  {
			  UARTbuffer[UARTbufferctr] = LEUART0->RXDATA;
			  LEUART_Tx(LEUART0, UARTbuffer[UARTbufferctr]);
			  if((UARTbuffer[UARTbufferctr]=='\r') || (UARTbufferctr++==63)) SOFReceived =0;

		  }
	}

}




/** @} (end addtogroup app) */
/** @} (end addtogroup Application) */
