/***********************************************************************************************//**
 * \file   main.c
 * \brief  Silicon Labs Empty Example Project
 *
 * This example demonstrates the bare minimum needed for a Blue Gecko C application
 * that allows Over-the-Air Device Firmware Upgrading (OTA DFU). The application
 * starts advertising after boot and restarts advertising after a connection is closed.
 ***************************************************************************************************
 * <b> (C) Copyright 2016 Silicon Labs, http://www.silabs.com</b>
 ***************************************************************************************************
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 **************************************************************************************************/

#ifndef GENERATION_DONE
#error You must run generate first!
#endif

/* Board headers */
#include "boards.h"
#include "ble-configuration.h"
#include "board_features.h"

/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"
#include "aat.h"

/* Libraries containing default Gecko configuration values */
#include "em_emu.h"
#include "em_cmu.h"
#ifdef FEATURE_BOARD_DETECTED
#include "bspconfig.h"
#include "pti.h"
#endif

/* Device initialization header */
#include "InitDevice.h"

#ifdef FEATURE_SPI_FLASH
#include "em_usart.h"
#include "mx25flash_spi.h"
#endif /* FEATURE_SPI_FLASH */


/*Application Specific Code*/
#include "em_timer.h"
#include "em_gpio.h"
#include "peripherals.h"
#include "em_usart.h"
#include "em_leuart.h"



/***********************************************************************************************//**
 * @addtogroup app
 * @{
 **************************************************************************************************/

#ifndef MAX_CONNECTIONS
#define MAX_CONNECTIONS 4
#endif
uint8_t bluetooth_stack_heap[DEFAULT_BLUETOOTH_HEAP(MAX_CONNECTIONS)];

#ifdef FEATURE_PTI_SUPPORT
static const RADIO_PTIInit_t ptiInit = RADIO_PTI_INIT;
#endif

/* Gecko configuration parameters (see gecko_configuration.h) */
static const gecko_configuration_t config = {
  .config_flags=0,
  .sleep.flags=SLEEP_FLAGS_ACTIVE_HIGH,//SLEEP_FLAGS_DEEP_SLEEP_ENABLE,
  .bluetooth.max_connections=MAX_CONNECTIONS,
  .bluetooth.heap=bluetooth_stack_heap,
  .bluetooth.heap_size=sizeof(bluetooth_stack_heap),
  .bluetooth.sleep_clock_accuracy = 100, // ppm
  .gattdb=&bg_gattdb_data,
  .ota.flags=0,
  .ota.device_name_len=3,
  .ota.device_name_ptr="OTA",
  #ifdef FEATURE_PTI_SUPPORT
  .pti = &ptiInit,
  #endif
};

/* Flag for indicating DFU Reset must be performed */
uint8_t boot_to_dfu = 0;

uint8_t UARTbuffer[UARTBUFFERSIZE]= "UART test 123 hhh\n\r";

/**
 * @brief  Main function
 */
int main(void)
{
#ifndef FEATURE_SPI_FLASH
  /* Put the SPI flash into Deep Power Down mode for those radio boards where it is available */
  MX25_init();
  MX25_DP();
  /* We must disable SPI communication */
  USART_Reset(USART1);

#endif /* FEATURE_SPI_FLASH */

  /* Initialize peripherals */
  enter_DefaultMode_from_RESET();

  InitPWM1();
  InitLEUART0();
  UpdatePWM1(15);
 // ClearSOFReceived();


#ifndef TESTING


  while (1)
  {
	  UART_Tx((uint8_t *)UARTbuffer, UARTBUFFERSIZE);
	  UART_TXHandler();
	  UART_RXHandler();
	  PWMHandler();

	  for (int counterr=0; counterr<0x3fff; counterr++);

	// UART_Tx(&UARTbuffer, 19);
  }

#endif

  /* Initialize stack */
  gecko_init(&config);

  while (1) {
    /* Event pointer for handling events */
    struct gecko_cmd_packet* evt;
    
    /* Check for stack event. */
    evt = gecko_wait_event();

    /* Handle events */
    switch (BGLIB_MSG_ID(evt->header)) {

      /* This boot event is generated when the system boots up after reset.
       * Here the system is set to start advertising immediately after boot procedure. */
      case gecko_evt_system_boot_id:


      	gecko_cmd_system_set_tx_power(0);   // set TX power to 0 dBm

        /* Set advertising parameters. 100ms advertisement interval. All channels used.
         * The first two parameters are minimum and maximum advertising interval, both in
         * units of (milliseconds * 1.6). The third parameter '7' sets advertising on all channels. */
        gecko_cmd_le_gap_set_adv_parameters(160,160,7);

        /* Start general advertising and enable connections. */
        gecko_cmd_le_gap_set_mode(le_gap_general_discoverable, le_gap_undirected_connectable);
        break;

      case gecko_evt_le_connection_closed_id:
	  
        /* Check if need to boot to dfu mode */
        if (boot_to_dfu) {
          /* Enter to DFU OTA mode */
          gecko_cmd_system_reset(2);
        }
        else {
          /* Restart advertising after client has disconnected */
          gecko_cmd_le_gap_set_mode(le_gap_general_discoverable, le_gap_undirected_connectable);
        }
        break;

      case gecko_evt_system_external_signal_id:

      if (evt->data.evt_system_external_signal.extsignals == USART0_DATA_AVAILABLE)

      {

      //your code

      }

      break;



      /* Events related to OTA upgrading
      ----------------------------------------------------------------------------- */

      /* Check if the user-type OTA Control Characteristic was written.
       * If ota_control was written, boot the device into Device Firmware Upgrade (DFU) mode. */
      case gecko_evt_gatt_server_user_write_request_id:
      
        if(evt->data.evt_gatt_server_user_write_request.characteristic==gattdb_ota_control)
        {
          /* Set flag to enter to OTA mode */
          boot_to_dfu = 1; 
          /* Send response to Write Request */
          gecko_cmd_gatt_server_send_user_write_response(
            evt->data.evt_gatt_server_user_write_request.connection, 
            gattdb_ota_control, 
            bg_err_success);
         
          /* Close connection to enter to DFU OTA mode */        
          gecko_cmd_endpoint_close(evt->data.evt_gatt_server_user_write_request.connection);
        }

        if (evt->data.evt_gatt_server_user_write_request.characteristic==gattdb_xgatt_gpio_PWM1)
        {
        		if (evt->data.evt_gatt_server_user_write_request.value.data[0] == 1)
        		{

        		}


        }

        break;

      default:
        break;
    }
  }
}




/** @} (end addtogroup app) */
/** @} (end addtogroup Application) */
