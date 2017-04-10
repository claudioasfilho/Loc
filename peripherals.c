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


void InitUSART0(void)
{
	/* Enable clock for USART0 */
	CMU_ClockEnable(cmuClock_USART0, true);

	// $[USART_InitAsync]
	USART_InitAsync_TypeDef initasync = USART_INITASYNC_DEFAULT;

	initasync.enable = usartDisable;
	initasync.baudrate = 9600;
	initasync.databits = usartDatabits8;
	initasync.parity = usartNoParity;
	initasync.stopbits = usartStopbits1;
	initasync.oversampling = usartOVS4;
#if defined( USART_INPUT_RXPRS ) && defined( USART_CTRL_MVDIS )
	initasync.mvdis = 0;
	initasync.prsRxEnable = 0;
	initasync.prsRxCh = 0;
#endif

	USART_InitAsync(USART0, &initasync);
	// [USART_InitAsync]$

	// $[USART_InitSync]
	// [USART_InitSync]$

	// $[USART_InitPrsTrigger]
	USART_PrsTriggerInit_TypeDef initprs = USART_INITPRSTRIGGER_DEFAULT;

	initprs.rxTriggerEnable = 0;
	initprs.txTriggerEnable = 0;
	initprs.prsTriggerChannel = usartPrsTriggerCh0;

	USART_InitPrsTrigger(USART0, &initprs);
	// [USART_InitPrsTrigger]$

	// $[USART_InitIO]
	/* Disable CLK pin */
	USART0->ROUTELOC0 = (USART0->ROUTELOC0 & (~_USART_ROUTELOC0_CLKLOC_MASK))
			| USART_ROUTELOC0_CLKLOC_LOC0;
	USART0->ROUTEPEN = USART0->ROUTEPEN & (~USART_ROUTEPEN_CLKPEN);

	/* Disable CS pin */
	USART0->ROUTELOC0 = (USART0->ROUTELOC0 & (~_USART_ROUTELOC0_CSLOC_MASK))
			| USART_ROUTELOC0_CSLOC_LOC0;
	USART0->ROUTEPEN = USART0->ROUTEPEN & (~USART_ROUTEPEN_CSPEN);

	/* Disable CTS pin */
	USART0->ROUTELOC1 = (USART0->ROUTELOC1 & (~_USART_ROUTELOC1_CTSLOC_MASK))
			| USART_ROUTELOC1_CTSLOC_LOC0;
	USART0->ROUTEPEN = USART0->ROUTEPEN & (~USART_ROUTEPEN_CTSPEN);

	/* Disable RTS pin */
	USART0->ROUTELOC1 = (USART0->ROUTELOC1 & (~_USART_ROUTELOC1_RTSLOC_MASK))
			| USART_ROUTELOC1_RTSLOC_LOC0;
	USART0->ROUTEPEN = USART0->ROUTEPEN & (~USART_ROUTEPEN_RTSPEN);

	/* Set up RX pin */
	USART0->ROUTELOC0 = (USART0->ROUTELOC1 & (~_USART_ROUTELOC0_RXLOC_MASK))
			| USART_ROUTELOC0_RXLOC_LOC17;
	USART0->ROUTEPEN = USART0->ROUTEPEN | USART_ROUTEPEN_RXPEN;

	/* Set up TX pin */
	USART0->ROUTELOC0 = (USART0->ROUTELOC0 & (~_USART_ROUTELOC0_TXLOC_MASK))
			| USART_ROUTELOC0_TXLOC_LOC0;
	USART0->ROUTEPEN = USART0->ROUTEPEN | USART_ROUTEPEN_TXPEN;

	// [USART_InitIO]$

	// $[USART_Misc]
	/* Disable CTS */
	USART0->CTRLX = USART0->CTRLX & (~USART_CTRLX_CTSEN);
	/* Set CTS active low */
	USART0->CTRLX = USART0->CTRLX & (~USART_CTRLX_CTSINV);
	/* Set RTS active low */
	USART0->CTRLX = USART0->CTRLX & (~USART_CTRLX_RTSINV);
	/* Set CS active low */
	USART0->CTRL = USART0->CTRL & (~USART_CTRL_CSINV);
	/* Set TX active high */
	USART0->CTRL = USART0->CTRL & (~USART_CTRL_TXINV);
	/* Set RX active high */
	USART0->CTRL = USART0->CTRL & (~USART_CTRL_RXINV);
	// [USART_Misc]$

	// $[USART_Enable]


	  /* Clear previous RX interrupts */
	  USART_IntClear(USART0, USART_IF_RXDATAV);
	  NVIC_ClearPendingIRQ(USART0_RX_IRQn);

	  /* Enable RX interrupts */
	  USART_IntEnable(USART0, USART_IF_RXDATAV);
	  NVIC_EnableIRQ(USART0_RX_IRQn);



	/* Enable USART if opted by user */
	USART_Enable(USART0, usartEnable);

}

void InitLEUART0(void) {

	uint32_t leuartif;

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
	  LEUART0->STARTFRAME = '$';

	  /* Set LEUART signal frame */
	  LEUART0->SIGFRAME = '\r';

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

}

void UART_Tx(uint8_t* buffer, uint16_t size)
{
	uint16_t counter;

	for(counter=0; counter<size; counter++)
	{
		LEUART_Tx(LEUART0, *buffer++);
	//	UART0_Tx (LEUART0, *buffer++);
	}

}


/**
* Handle GPIO interrupts and trigger system_external_signal event
*/
void USART0_RX_IRQHandler()
{
  static bool radioHalted = false;
  uint32_t flags = USART_IntGet(USART1);
  USART_IntClear(USART1, flags);
  //Send gecko_evt_system_external_signal_id event to the main loop
  gecko_external_signal(USART0_DATA_AVAILABLE);
  }

/* GLOBAL VARIABLES */

//char rxbuf[BUF_MAX];
uint32_t leuartif;
uint32_t len;

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

}


void UpdatePWM1(uint32_t desiredDutyCycle) //desiredDutyCycle varies from 0-100;
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




/** @} (end addtogroup app) */
/** @} (end addtogroup Application) */
