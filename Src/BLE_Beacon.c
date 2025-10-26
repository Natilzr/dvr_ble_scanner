/******************** (C) COPYRIGHT 2015 STMicroelectronics ********************
* File Name          : BLE_Beacon.c
* Author             : AMS - VMA Division
* Version            : V1.2.0
* Date               : 25-June-2015
* Description        : BlueNRG/BlueNRG-MS main file for beacon device
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/
/**
 * @file  BLE_Beacon.c
 * @brief This is a BLE beacon demo that shows how to configure a BlueNRG/BlueNRG-MS device 
 * in order to advertise specific manufacturing data and allow another BLE device to
 * know if it is in the range of the BlueNRG/BlueNRG-MS beacon device. 
 *
 * <!-- Copyright 2015 by STMicroelectronics.  All rights reserved.       *80*-->

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# <b>BlueNRG device</b>: Open the IAR project
     <tt> ...\\Projects\\Projects_STD_Library\\BLE_Beacon\\EWARM\\BLE_Beacon.eww </tt> or
     <tt> ...\\Projects\\Projects_Cube\\BLE_Beacon\\EWARM\\BLE_Beacon.eww </tt>
  -# <b>BlueNRG-MS device</b>: Open the IAR project
     <tt> ...\\Projects\\Projects_STD_Library\\BLE_Beacon\\EWARM_BlueNRG-MS\\BLE_Beacon.eww </tt> or
     <tt> ...\\Projects\\Projects_Cube\\BLE_Beacon\\EWARM_BlueNRG-MS\\BLE_Beacon.eww </tt>
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect STLink to JTAG connector  in your board (if available).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the BlueNRG GUI, put the board in DFU mode and download the built binary image.

* \subsection IAR_project_configurations IAR project configurations

  - \c Release - BLE beacon release configuration (BlueNRG, BlueNRG-MS Kits platforms).
  - \c Release_Nucleo - BLE beacon release configuration for NUCLEO-L152RE platform with X-NUCLEO-IDB04V1 (BlueNRG) or X-NUCLEO-IDB05A1 (BlueNRG-MS).

  - IAR configurations for NUCLEO-L152RE + with X-NUCLEO-IDB04V1 (BlueNRG) or X-NUCLEO-IDB05A1 (BlueNRG-MS) are available only on STM32L1 Cube framework.
                        
* \section Prebuilt_images Prebuilt images
  - None
 
* \section Jumper_settings Jumper settings
@table
------------------------------------------------------
| Jumper name       |  Description                   | 
------------------------------------------------------
| JP1, if available | USB or Battery supply position | 

@endtable 


* \section Board_supported Boards supported
@table
| Board name (Order Code)                       | Description                           | 
-----------------------------------------------------------------------------------------
| STEVAL-IDB002V1                               | BlueNRG Development Platform          | 
| STEVAL-IDB003V1                               | BlueNRG  USB Dongle                   | 
| STEVAL-IDB005V1                               | BlueNRG-MS Development Platform       | 
| STEVAL-IDB005V1D                              | BlueNRG-MS Daughter Board             |  
| STEVAL-IDB006V1                               | BlueNRG-MS  USB Dongle                | 
| NUCLEO-L152RE + X-NUCLEO-IDB04/5A1            | STM32L152 Nucleo + BlueNRG/BlueNRG-MS |
@endtable

* \section Serial_IO Serial I/O
 - Not Applicable;

* \section LEDs_description LEDs description
@table                    
| LED name         | STEVAL-IDB002V1/5V1 | STEVAL-IDB003V1/6V1 | NUCLEO-L152RE + X-NUCLEO-IDB04/5A1 |    
-----------------------------------------------------------------------------------------------------
| D1               | Activity led        | NA                  | NA                                 |                    
| D2               | Error led           | Activity led        | Error led                          |    
| D3               | Not used            | Error led           | NA                                 |    
| D4               | Not used            | NA                  | NA                                 |     
| D5               | Not used            | NA                  | NA                                 |   
@endtable
 - NA : Not Applicable;

* \section Buttons_description Buttons description
@table                
| BUTTON name      | STEVAL-IDB002V1/5V1 | STEVAL-IDB003V1/6V1 | NUCLEO-L152RE + X-NUCLEO-IDB04/5A1 |     
-----------------------------------------------------------------------------------------------------
| RESET            | X                   | NA                  | X                                  |     
| Push Button      | Not used            | NA                  | Not used                           |       
| Jostick Sel      | Not used            | NA                  | NA                                 |        
| SW1              | NA                  | Not used            | NA                                 |         
| SW2              | NA                  | Not used            | NA                                 |           
@endtable
 - NA : Not Applicable;

* \section DFU_Activation  DFU activation
BlueNRG/BlueNRG-MS boards are preprogrammed with a DFU application which allows to upload the 
STM32L micro with a selected binary image through USB. Follow list of actions 
for activating DFU on each supported platforms
@table
-------------------------------------------------------------------------------------------------
| Board  name          | Event                                             | Note               |
-------------------------------------------------------------------------------------------------
| STEVAL-IDB002V1/5V1  | Press RESET and Push Button. Release RESET button | LED D2 is toggling |  
| STEVAL-IDB003V1/6V1  | Press SW1 button and plug USB dongle on a PC port | LED D3 is toggling |  
@endtable
 - Note: No DFU support on NUCLEO-L152RE + with X-NUCLEO-IDB04V1 (BlueNRG) or X-NUCLEO-IDB05A1 (BlueNRG-MS).

* \section Usage Usage
The Beacon demo configures a BlueNRG/BlueNRG-MS device in advertising mode (non-connectable mode) with specific manufacturing data.
It transmits advertisement packets at regular intervals which contain the following manufacturing data:
@table   
------------------------------------------------------------------------------------------------------------------------
| Data field              | Description                       | Notes                                                  |
------------------------------------------------------------------------------------------------------------------------
| Company identifier code | SIG company identifier (1)        | Default is 0x0030 (STMicroelectronics)                 |
| ID                      | Beacon ID                         | Fixed value                                            |
| Length                  | Length of the remaining payload   | NA                                                     |
| Location UUID           | Beacons UUID                      | It is used to distinguish specific beacons from others |
| Major number            | Identifier for a group of beacons | It is used to group a related set of beacons           |                                              
| Minor number            | Identifier for a single beacon    | It is used to identify a single beacon                 |                                       
| Tx Power                | 2's complement of the Tx power    | It is used to establish how far you are from device    |                                       
@endtable

 - (1): SIG company identifiers are available on https://www.bluetooth.org/en-us/specification/assigned-numbers/company-identifiers
 - NA : Not Applicable;

**/
/** @addtogroup BlueNRG_and_BlueNRG_MS_demonstrations_applications
 * BlueNRG/BlueNRG-MS Beacon demo \see BLE_Beacon.c for documentation.
 *
 *@{
 */

/** @} */
/** \cond DOXYGEN_SHOULD_SKIP_THIS
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "hal_types.h"
#include "hci.h"
#include "bluenrg_aci.h"
#include "gp_timer.h"
#include "hal.h"
#include "osal.h"
#include "bluenrg_gatt_server.h"
#include "hci_const.h"
#include "bluenrg_gap.h"
#include "sm.h"
#include "app_state.h"
#include <stdio.h>

#include "SDK_EVAL_Config.h"

/* External variables --------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/** 
  * @brief  Enable febug printf's
  */ 
#ifdef DEBUG
#define DEBUG 0
#endif

/* Private macros ------------------------------------------------------------*/
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/* Private variables ---------------------------------------------------------*/
volatile int app_flags = SET_DISCOVERABLE;
static uint16_t led_blinking_rate = 500;

/* Private function prototypes -----------------------------------------------*/
void Make_Discoverable(void);
void User_Process(void);

/* Private functions ---------------------------------------------------------*/

/*  User Function where serial received data should be processed */
void processInputData(uint8_t * rx_data, uint16_t data_size)
{
}

/*******************************************************************************
* Function Name  : main.
* Description    : Main routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int main(void)
{
  int ret;
  
  /* Device Initialization */
  Init_Device();
  
  /* Identify BlueNRG/BlueNRG-MS platform */
  SdkEvalIdentification();
  
  SdkEvalLedInit(LED1);//activity led
  SdkEvalLedInit(LED2);//error led
  
  HCI_Init();
  
  /* Init SPI interface */
  SdkEvalSpiInit(SPI_MODE_EXTI);
  BlueNRG_RST(); 
  
  {
    uint8_t bdaddr[] = {0xff, 0x00, 0x00, 0xE1, 0x80, 0x02};

    ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET, CONFIG_DATA_PUBADDR_LEN,
                                    bdaddr);
    if(ret){
        SdkEvalLedOn(LED2);
        return ret;
    }
  }
  
  ret = aci_gatt_init();    
  if(ret){
    SdkEvalLedOn(LED2);
    return ret;
  }
  
  {
    uint16_t service_handle, dev_name_char_handle, appearance_char_handle;

#if BLUENRG_MS
        ret = aci_gap_init(GAP_PERIPHERAL_ROLE, 0, 0x07, &service_handle, &dev_name_char_handle, &appearance_char_handle);
#else
        ret = aci_gap_init(GAP_PERIPHERAL_ROLE, &service_handle, &dev_name_char_handle, &appearance_char_handle);
#endif
    if(ret){
        SdkEvalLedOn(LED2);
        return ret;
    }
  }

  /* -2 dBm output power */
  ret = aci_hal_set_tx_power_level(1,4);
  
  while(1)
  { 
    static tClockTime startTime = 0;
    HCI_Process();
    User_Process();
    
    if (Clock_Time() - startTime >led_blinking_rate)
    {    
      SdkEvalLedToggle(LED1);
      startTime = Clock_Time();
    }
  }
}

/**
* @brief  Make the device discoverable
* @param  None 
* @retval None
*/
void Make_Discoverable(void)
{  
  tBleStatus ret;
  
   const uint8_t manuf_data[] = {26, AD_TYPE_MANUFACTURER_SPECIFIC_DATA, 
      0x30, 0x00, //Company identifier code (Default is 0x0030 - STMicroelectronics: To be customized for specific identifier)
      0x02,       // ID
      0x15,       //Length of the remaining payload
      0xE2, 0x0A, 0x39, 0xF4, 0x73, 0xF5, 0x4B, 0xC4, //Location UUID
      0xA1, 0x2F, 0x17, 0xD1, 0xAD, 0x07, 0xA9, 0x61,
      0x00, 0x00, // Major number 
      0x00, 0x00, // Minor number 
      0xC8        //2's complement of the Tx power (-56dB)};      
   };
   
  /* disable scan response */
  hci_le_set_scan_resp_data(0,NULL);
  
  /* put device in non connectable mode */
  ret = aci_gap_set_discoverable(ADV_NONCONN_IND, 160, 160, PUBLIC_ADDR, NO_WHITE_LIST_USE,
                                 0, NULL, 0, NULL, 0, 0); 
  if (ret != BLE_STATUS_SUCCESS)
  {
   SdkEvalLedOn(LED2);
   return;
  }
  
  ret = aci_gap_delete_ad_type(AD_TYPE_TX_POWER_LEVEL); 
  if (ret != BLE_STATUS_SUCCESS)
  {
   SdkEvalLedOn(LED2);
   return;
  }
  
  ret = aci_gap_update_adv_data(27, manuf_data);  
  if (ret != BLE_STATUS_SUCCESS)
  {
   SdkEvalLedOn(LED2);
   return;
  }
}


void User_Process(void)
{
  if(APP_FLAG(SET_DISCOVERABLE)){
    Make_Discoverable();
    APP_FLAG_CLEAR(SET_DISCOVERABLE);
  } 
}

/**
  * @brief  This function is called whenever there is an ACI event to be processed.
  * @note   Inside this function each event must be identified and correctly
  *         parsed.
  * @param  pckt  Pointer to the ACI packet
  * @retval None
  */
void HCI_Event_CB(void *pckt)
{
  hci_uart_pckt *hci_pckt = pckt;
  hci_event_pckt *event_pckt = (hci_event_pckt*)hci_pckt->data;
  
  if(hci_pckt->type != HCI_EVENT_PKT)
    return;
  
  switch(event_pckt->evt){
    
  case EVT_DISCONN_COMPLETE:
    {
    }
    break;
    
  case EVT_LE_META_EVENT:
    {
      evt_le_meta_event *evt = (void *)event_pckt->data;
      
      switch(evt->subevent){
      case EVT_LE_CONN_COMPLETE:
        {
          //evt_le_connection_complete *cc = (void *)evt->data;
        }
        break;
      }
    }
    break;
    
  case EVT_VENDOR:
    {
    }
    break;
  } 
}


#ifdef USE_FULL_ASSERT
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert_param error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert_param error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {}
}
#endif

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
/** \endcond
 */
