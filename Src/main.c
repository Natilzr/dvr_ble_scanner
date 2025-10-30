/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "iwdg.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "gp_timer.h"
#include "hal.h"
#include "hci.h"
#include "bluenrg_aci.h"
#include "osal.h"
#include "bluenrg_gatt_server.h"
#include "hci_const.h"
#include "bluenrg_gap.h"
//#include "SDK_EVAL_Config.h"
#include "stm32_bluenrg_ble.h"
#include "string.h"
#include "wli_ble.h"

#include "blefunc.h"
#include "show.h"
#include "serialdrivers.h"
#include "printfunc.h"
#include "ConnFunc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void User_Process(void);
  void ShowResult(ReadStruct Readbuf);
  void PrintRet(int ret_v);
#ifdef TRACE_VERSION
extern BOOL trace_flag;
#include <stdarg.h>
void Trace_Error_New(char* pBuffer,char* format,...);
extern char DebugBuffer[];

#endif
TestDevices ScanDevices[SCAN_TABLE_SIZE];
TestDevices* FindDeviceMAXRssi(TestDevices* pScanDevices, int No );
#ifdef TRACE_VERSION
int
#else
void 
#endif
ProcessReceivedTags(TestDevices* pScannedDevices, uint16_t Counter);
//void ProcessReceivedTags2(TestDevices* pScannedDevices, uint16_t Counter);
void  SortTable(TestDevices* pScannedDevices, uint16_t Counter);
uint32_t u32Alder( uint8_t *pu8Data, uint16_t u32Len,uint32_t *a ,uint32_t *b);
void CleanTable(void);
//uint16_t  PrintMsg(uint16_t cnt);
void CheckCR(void);
uint16_t ProcessRX(void);
void CleanRec(void);
/* USER CODE END PFP */
uint32_t Address = 0, PageError = 0;
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
TestDevices ScanDevices[SCAN_TABLE_SIZE];
DevicesSort DeviceTable[DEVICE_TABLE_SIZE];
uint8_t g_BDdaddr[BD_ADDR_SIZE];
uint16_t DeviceCount = 0;
//extern ReadStruct Readbuf;
ReadStruct Readbuf;
//ReadStruct confbuf;
uint8_t show_cnt;
//uint8_t hexbuffer[100];
uint32_t Len = 0;
uint8_t textbuf[60];
uint8_t Rbuf[RBUF_SIZE];
uint8_t table_l=0;
uint8_t Resend,LEDDelay;
uint32_t LoopDelayCnt;
uint16_t PrintCnt;
uint16_t DelayCnt;
uint16_t TickCnt;
extern uint32_t TickCount;
extern uint16_t Flags;
uint8_t rcnt,SecCnt;
uint8_t UUidFilter[4];
uint8_t testbuf[40];
uint32_t TempAge;
uint8_t TempMac[6];
uint8_t ConState,LastConstate,RetConstate;
uint16_t My_Con_Handle;
uint8_t Att_ReadLengh;
uint8_t  Att_ReadVal[13];
processt t_process;
uint16_t Temp_SrviceAtt;
  uint8_t UuidOffset;
  uint8_t CopyOffset;
  uint8_t copySize;    
  
  ProtocolS   PrPars[6];
  uint8_t DevName[19];
  uint16_t Protocol;
uint8_t Mode;
uint16_t DeviceRespCount = 0;
uint8_t RunState;
uint16_t MsgCnt;
uint16_t DateTime[6];
uint64_t UNIT_ID;
ERROR_STATUS    S_Error,St_Error;

volatile int wli_app_flags = 0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
//	  uint8_t ret;
//	  uint8_t q;
          uint16_t MsgLen = 0;
          uint16_t SND=0;
	   TestStatus = BtInit;
	    Flags &= ~DeviceChoos;
	    Resend = 0;
            rcnt = 0;
#if defined(OLD_TAG)  
     UuidOffset =  9;
     CopyOffset  =  4;
     copySize =  26;            
            UUidFilter[0] = 0xfd;//default
            UUidFilter[1] = 0xa5;
            UUidFilter[2] = 0x06;
            UUidFilter[3] = 0x93;

#elif defined(TLM)
     UuidOffset =   3;
     CopyOffset  =  13;
     copySize =  4;
            UUidFilter[0] = 0x3;//default
            UUidFilter[1] = 0x3;
            UUidFilter[2] = 0xAA;
            UUidFilter[3] = 0xfE;
#elif defined(BRACELET)
     UuidOffset  =  3;
     CopyOffset  =  7;
     copySize =  7;
            UUidFilter[0] = 0x05;//default
            UUidFilter[1] = 0x16;
            UUidFilter[2] = 0x09;
            UUidFilter[3] = 0x18;
#endif
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  InitTables();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  //MX_USART1_UART_Init();
  MX_TIM2_Init();
  #ifdef WDG
  MX_IWDG_Init();
#endif
  /* USER CODE BEGIN 2 */
   HCI_Init();
  /* Init SPI interface */
    LoadName();
    Protocol = 1;
  HAL_GPIO_WritePin(BNRG_SPI_RESET_PORT,BNRG_SPI_RESET_PIN, GPIO_PIN_SET);//reset bt
  HAL_GPIO_WritePin(ExtMem_CS_GPIO_Port,ExtMem_CS_Pin, GPIO_PIN_SET);//eeprom cs
  HAL_GPIO_WritePin(Buzzer_GPIO_Port,Buzzer_Pin, GPIO_PIN_RESET);// relay
  HAL_GPIO_WritePin(LED_Red_GPIO_Port,LED_Red_Pin,GPIO_PIN_SET);
 // HAL_Delay(600);
  HAL_GPIO_WritePin(LED_Red_GPIO_Port,LED_Red_Pin,GPIO_PIN_RESET);
 // HAL_Delay(600);
  HAL_GPIO_WritePin(LED_Red_GPIO_Port,LED_Red_Pin,GPIO_PIN_SET);
  HAL_TIM_Base_Start(&htim2);
  //SdkEvalSpiInit(SPI_MODE_EXTI);
  BNRG_SPI_Init();
  HAL_Delay(400);
#ifdef TRACE_VERSION
  OpenSerial(COM1,PC_BAUDRATE,FALSE);
#endif
  if(StartBT(GAP_OBSERVER_ROLE_IDB05A1) == 0) //init stack
  {
    HAL_GPIO_WritePin(LED_Red_GPIO_Port,LED_Red_Pin,GPIO_PIN_RESET);
    HAL_Delay(600);
 
 //     WriteSerial(COM1,(char*)textbuf,MsgLen);
  }
  else
  {
            SND=EncodeCommandStatus(STATUS1_ERROR);
  }
//  UNIT_ID = Flash_ReadValue();
#ifndef TRACE_VERSION
  OpenSerial(COM1,PC_BAUDRATE,FALSE);
      memset(textbuf,'\0',sizeof(textbuf));
      sprintf (textbuf,"START HOST");
      SND = EncodeTrace((uint8_t*)textbuf,strlen(textbuf));
  if (SND)
  {
    WriteSerial(COM1,(char*)textbuf,SND);
  }
#endif
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      HCI_Process();
      User_Process();
      /******   every Tick   *************/


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_RTC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void User_Process(void)
{
  uint16_t MsgLen = 0;
    char tBuff[20];
#ifdef TRACE_VERSION
  int FilteredCntr;
#endif
#ifdef WDG
  HAL_IWDG_Refresh(&hiwdg);
#endif
  if (Flags & Tick_flag)//every 1ms
  {
      //DelayCnt--;
      if(DelayCnt-- == 0 && WAPP_FLAG(DELAYF))
      {
        WAPP_FLAG_SET(STATE_UPDATE);
        WAPP_FLAG_CLEAR(DELAYF);
      }
    //DelayCnt--;
    LoopDelayCnt--;
    Flags &= ~Tick_flag;
    if((Flags & CR_flag) && (!(Flags & TX_flag)) )
    { 
      StoredStat = TestStatus;
      MsgLen = ProcessRX();

      if(MsgLen)
      {
        WriteSerial(COM1,(char*)textbuf,MsgLen);
        TestStatus = StoredStat;
      }
      memset(Rbuf,'\0',sizeof(Rbuf));
      Flags &= ~CR_flag;
  
    }
    if(WAPP_FLAG(MASTER_C))
    {
      ConnFunc();
    }
    else
    {
      switch(TestStatus)
      {
      case BtInit:
          TestStatus=Searching;
          TestType = 0;
          DeviceCount = 0;
          CleanTable();
          Flags &= ~ScanEnd;
          
        break;
      case Searching:
          //sprintf((char*)textbuf,"\033[2J");
          memset(textbuf,'\0',sizeof(textbuf));
          TestStatus=Scan_wait;
          if(ScanBT() == 0)
          {
             HAL_GPIO_WritePin(LED_Red_GPIO_Port,LED_Red_Pin,GPIO_PIN_RESET);
             LEDDelay = 5;
             Flags |= LedFlag;
             Flags &= ~ScanEnd;
             TIM2->CNT = 0;
             TIM2->SR &= ~0x01;
             HAL_TIM_Base_Start_IT(&htim2);
          }
          else
          {

            TestStatus=Searching;
          }
        break;
      case Scan_wait:
        break;
      case Scan_end:// Process the data after Scan
        if(Flags & ScanEnd)
        {
 //         HAL_GPIO_WritePin(LED_Red_GPIO_Port,LED_Red_Pin,GPIO_PIN_SET);
          aci_gap_terminate_gap_procedure(GAP_OBSERVATION_PROC_IDB05A1);
        }
#ifdef TRACE_VERSION
        FilteredCntr = 
#endif
        
        ProcessReceivedTags(ScanDevices,DeviceCount);
#ifdef TRACE_VERSION
        Trace_Error_New(DebugBuffer,"Processed: %d, filtered %d",DeviceCount,FilteredCntr);
#endif
        TestStatus = Wait_Bot;
        break;
      case Wait_Bot:
 //         DeviceCount = 0;  //clean scan devices
 //         CleanTable();
          if(Flags & Sec_10)
          {
            TestStatus=SendSer;
  //          DeviceCount = table_l;
#ifndef SENDT_TEST
            table_l = DeviceCount ;
#else
            table_l = 1000;
#endif
            HAL_GPIO_WritePin(LED_Green_GPIO_Port,LED_Green_Pin, GPIO_PIN_RESET);
            Flags &= ~Sec_10;
            Flags |= TX_flag;//set tx flag
          }
          else
          {
            TestStatus=SendEnd;
          }
        break;
      case SendSer:
        if(PrintCnt>table_l-1)
        {
          TestStatus = SendEnd;
          CleanRec();
          HAL_GPIO_WritePin(LED_Green_GPIO_Port,LED_Green_Pin, GPIO_PIN_SET);
          Flags &= ~TX_flag;
          break;
        }
        memset(textbuf,'\0',sizeof(textbuf));
        //MsgLen = PrintDVRMsg(PrintCnt);//changes inside
        MsgLen = PrintMsg(PrintCnt);//changes inside
#ifdef TRACE_VERSION
        if (!trace_flag)
#endif
        WriteSerial(COM1,(char*)textbuf,MsgLen);
        PrintCnt++;
        DelayCnt = 2000;
        TestStatus = SendDelay;
        break;
      case SendDelay:
        if(DelayCnt == 0)
        {
          TestStatus=SendSer;
        }
        break;
      case SendEnd:
        TestStatus=Scan_wait;
        PrintCnt = 0;
        memset(textbuf,'\0',sizeof(textbuf));
          sprintf (tBuff,"FinishScan %d",DeviceCount);//,10);
          MsgLen = EncodeTrace((uint8_t*)tBuff,strlen(tBuff));//10);
          WriteSerial(COM1,(char*)textbuf,MsgLen);
          DeviceCount = 0;  //clean scan devices
          CleanTable();
        if(ScanBT() == 0)//search again
          {
             LEDDelay = 10;
             Flags |= LedFlag;
             Flags &= ~ScanEnd;
             HAL_GPIO_WritePin(LED_Red_GPIO_Port,LED_Red_Pin,GPIO_PIN_RESET);
             TIM2->CNT = 0;
             TIM2->SR &= ~0x01;
             HAL_TIM_Base_Start_IT(&htim2);
          }
        else
        {
          TestStatus=SendEnd;
        }

        break;
      case LoopDelay:
        if(LoopDelayCnt == 0x00)
        {
          TestStatus = StoredStat;
        }
        break;
      default:
        break;
      }
    }
  }  //end of tick flag
  if (Flags & mSec_100)
  {
      CheckCR();
      if(Flags & LedFlag)
      {
        if (LEDDelay-- == 0)
        {
          HAL_GPIO_WritePin(LED_Red_GPIO_Port,LED_Red_Pin,GPIO_PIN_SET);
          Flags &= ~LedFlag;
        }
      }
      Flags &= ~mSec_100;
  }  //end of 100 mS
  if(Flags & Sec_10)//10 sec
  {

     // HAL_GPIO_WritePin(LED_Green_GPIO_Port,LED_Green_Pin, GPIO_PIN_RESET);
      //Flags &= ~Sec_10;
  }   //end of 10 Sec

}


  uint8_t bdAddrTest[12];

  static void GAP_AdvertizingReport_CB( le_advertising_info *pAdvInfo)
  {
    memcpy(bdAddrTest,&pAdvInfo->bdaddr,12);
  }

  TestDevices* FindDeviceMAXRssi(TestDevices* pScanDevices, int No )
  {
    uint8_t RssiMin = 0x00;
    int idxmin = 0;
    int i;
    if (No == 0)
      return NULL;
    for (i = 0; i < No; i++)
    {
      if (pScanDevices[i].Rssi > RssiMin)
      {
        RssiMin = pScanDevices[i].Rssi;
        idxmin = i;

      }
    }
    return pScanDevices+idxmin;
  }




  void PrintRet(int ret_v)
  {
          memset(textbuf,'\0',sizeof(textbuf));
          sprintf((char*)textbuf,"Ret = %X   ,%d\r\n",ret_v,TestStatus);
          //VCP_write(textbuf,strlen((char*)textbuf));
          HAL_UART_Transmit_IT(&huart1,textbuf,strlen((char*)textbuf));
    while(1)
    {
    }
  }




#ifdef TRACE_VERSION
  int
#else
  void ProcessReceivedTags(TestDevices* pScannedDevices, uint16_t Counter)
  {}
#endif
    
    

  

void CheckCR(void)
//void CheckCR(void)
{
  uint16_t Res;

  uint8_t status=0;

  while(status != REC_BUFFER_EMPTY)
  {
    Res = ReadSerial(COM1);
    status =(uint8_t)(Res>>8);
    if(status != REC_BUFFER_EMPTY)
    {
      Rbuf[rcnt] = (uint8_t)Res&0xff;

      if(rcnt++ > RBUF_SIZE)
      {
        rcnt = 0;
      }
      if(Rbuf[rcnt-1]=='#')
      {
        Flags |= CR_flag;
        rcnt = 0;
      }
    }
  }

}

uint16_t ProcessRX(void)
{
  uint8_t SPos;
  uint32_t TempInt;
  uint16_t SndLen = 0;
    uint8_t Protocol_x=0;
  uint8_t mode_x=0;
    uint8_t sx = 0;
      uint8_t s;
        uint8_t Idx = 0;
        uint8_t Serial[12]={0};
        
        uint16_t Tyear;
        uint8_t Tmonth;
        uint8_t Tday;
  if((Rbuf[0] == '$') && (Rbuf[1] == '$') )
  {
    //keep alive msg($$hb,1#)
  }
  else if(Rbuf[0] == '@') 
  {
      if(Rbuf[1] == '0') 
      {
        if(Rbuf[2] == '1')//set time
        {
          if (Rbuf[3] == ',' )
          {
            int n;
            uint8_t w = 0;
            // First call: pass string to initialize
            n = getNextNumber(&Rbuf[4]);
            DateTime[w] = n;
            while (n != -1)
            {
              n = getNextNumber(NULL); // subsequent calls
              w++;
              DateTime[w] = n;
            }
            UpdateDateTime(DateTime[0], DateTime[1], DateTime[2],
                    DateTime[3],DateTime[4],DateTime[5]);
          
          }
        }
        if(Rbuf[2] == '2')
        {
          if (Rbuf[3] == ',' )
          {
            UNIT_ID = 0;
            UNIT_ID = Ascii12ToUint64(&Rbuf[4]);
            //Flash_WriteValue(UNIT_ID);
            SaveFlash(UUID_P);
            
            
          }
        }
      }

    
  
  }
  return SndLen;
}

void CleanRec(void)
{
  table_l = 0;
}


void CleanTable(void)
{
  uint16_t q;
    for(q = 0;q<SCAN_TABLE_SIZE-1;q++)

    {

    ScanDevices[q].i64Addr = 0;
    ScanDevices[q].length = 0;
    ScanDevices[q].Rssi  = 0;
    ScanDevices[q].type  = 0;
    ScanDevices[q].lengthEx = 0x0;

    ScanDevices[q].dataEx[0] = 0xFF;
    ScanDevices[q].dataEx[1] = 0xFF;
    ScanDevices[q].data[25] = 0;
    ScanDevices[q].data[26] = 0;
    ScanDevices[q].data[27] = 0;
    ScanDevices[q].data[28] = 0;
    ScanDevices[q].data[29] = 0;
  }
}




void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  tBleStatus ret;
    if (htim->Instance == TIM2)
    {
        // 10-second tick
 

      HAL_TIM_Base_Stop(&htim2);
      Flags |= ScanEnd;
      TestStatus = Scan_end;
 
    }
}


void LoadName(void)
{
  uint8_t UUtmp[4];
  uint8_t P_t[4] = {0,0,0,0};
  uint64_t *ptr;
  memcpy(UUtmp,(void*)FLASH_UUID_ADRESS,4);
  if(UUtmp[0] == 0xff)// empty flash
     {
          PrPars[0].UUidFilter[0] = 0xfd;    
          PrPars[0].UUidFilter[1] = 0xa5; 
          PrPars[0].UUidFilter[2] = 0x06; 
          PrPars[0].UUidFilter[3] = 0x93;
     }
     else
     {
        PrPars[0].UUidFilter[3] = UUtmp[3];
        PrPars[0].UUidFilter[2] = UUtmp[2];
        PrPars[0].UUidFilter[1] = UUtmp[1];
        PrPars[0].UUidFilter[0] = UUtmp[0];
     }
  memcpy(P_t,(void*)FLASH_PROTOCOL_ADRESS,4);
  if(P_t[0] == 0xff)
  {
    Protocol = 0;
  }
  else
  {
  Protocol = P_t[0];
  }
  if(P_t[1] == 0xff)
  {
    Mode = ModeSCAN;//scan by default
  }
  else
  {
    Mode = P_t[1];
  }
/*
  //memcpy(DevName,(void*)FLASH_DEV_NAME,18);
  strncpy((char*)DevName,(void*)FLASH_DEV_NAME,18);
  DevName[18] = '\0';
  // hci_sscan_set(handle,DedviceName,strlan());

*/
  ptr = (uint64_t *)FLASH_USER_UUID_ADDR;
 UNIT_ID = *ptr;
}

void  InitTables(void)
{
  PrPars[0].UuidOffset = 9;
  PrPars[0].CopyOffset = 4;
  PrPars[0].copySize = 26;
  PrPars[0].UUidFilter[0] = 0xfd;    
  PrPars[0].UUidFilter[1] = 0xa5; 
  PrPars[0].UUidFilter[2] = 0x06; 
  PrPars[0].UUidFilter[3] = 0x93; 
  
  PrPars[1].UuidOffset = 3;
  PrPars[1].CopyOffset = 12;//was 13
  PrPars[1].copySize = 19;//was 18//4;
  PrPars[1].UUidFilter[0] = 0x03;//default
  PrPars[1].UUidFilter[1] = 0x03;
  PrPars[1].UUidFilter[2] = 0xaa;
  PrPars[1].UUidFilter[3] = 0xfe;
  
  PrPars[2].UuidOffset = 3;
  PrPars[2].CopyOffset = 7;
  PrPars[2].copySize = 7;
  PrPars[2].UUidFilter[0] = 0x05;//default;//default
  PrPars[2].UUidFilter[1] = 0x16;
  PrPars[2].UUidFilter[2] = 0x09;
  PrPars[2].UUidFilter[3] = 0x18;
  
  PrPars[3].UuidOffset = 9;
  PrPars[3].CopyOffset = 18;
  PrPars[3].copySize = 7;
  PrPars[3].UUidFilter[0] = 0x5f;    
  PrPars[3].UUidFilter[1] = 0xda; 
  PrPars[3].UUidFilter[2] = 0x3b; 
  PrPars[3].UUidFilter[3] = 0x11;
  
  PrPars[4].UuidOffset = 9;
  PrPars[4].CopyOffset = 18;
  PrPars[4].copySize = 7;
  PrPars[4].UUidFilter[0] = 0x5f;    
  PrPars[4].UUidFilter[1] = 0xda; 
  PrPars[4].UUidFilter[2] = 0x3b; 
  PrPars[4].UUidFilter[3] = 0x11;
  
  PrPars[5].UuidOffset = 9;
  PrPars[5].CopyOffset = 18;
  PrPars[5].copySize = 7;
  PrPars[5].UUidFilter[0] = 0x5f;    
  PrPars[5].UUidFilter[1] = 0xda; 
  PrPars[5].UUidFilter[2] = 0x3b; 
  PrPars[5].UUidFilter[3] = 0x11;
}



void SaveFlash(uint8_t params)
{
  
    uint8_t k;
    uint32_t addr3      =    FLASH_UUID_ADRESS;
    uint32_t bck[14];
/*Variable used for Erase procedure*/
    uint32_t Address = 0, PageError = 0;
      static FLASH_EraseInitTypeDef EraseInitStruct;

      uint32_t DATA_32   = 0x12345678;
      Address = ADDR_FLASH_PAGE_63;
    /* Unlock the Flash */
  HAL_FLASH_Unlock();
 /* Fill EraseInit structure*/
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.PageAddress = FLASH_UUID_ADRESS;
  EraseInitStruct.NbPages = (FLASH_USER_END_ADDR - FLASH_USER_START_ADDR) / FLASH_PAGE_SIZE;

  if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
  {
    /*
      Error occurred while page erase.
    */
    /* Infinite loop */
    while (1)
    {
      /* Make LED2 blink (100ms on, 2s off) to indicate error in Erase operation */
    }
  }

    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, (((uint32_t) PrPars[0].UUidFilter[3]<< 24) + ((uint32_t)PrPars[0].UUidFilter[2]<< 16) + ((uint32_t)PrPars[0].UUidFilter[1]<< 8) + (uint32_t)PrPars[0].UUidFilter[0])) == HAL_OK)
    {
      Address = Address + 4;
    }
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, ((uint32_t)Mode << 8)+((uint32_t)Protocol & 0x000000ff)) == HAL_OK)
    {
      Address = Address + 4;
    }    
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, UNIT_ID)== HAL_OK)
    {
      Address = Address + 4;
    } 

      HAL_FLASH_Lock();


    //save again stack
    /* Lock the Flash */
 //   flash_sw_lock = FLASH_LOCK_WORD;
    
}

int ConvertBcdToBin(unsigned char* pBcdStr,unsigned char* pBin,uint8_t clen)
{
      int j,i,c;

      int iLen = strlen((char*)pBcdStr);
      if((iLen == 13) && (pBcdStr[12] == 0x0d))
      {
        iLen = 12;
      }
      if (iLen % 2)
            return 0;
      iLen /= 2;
      //c = 5;
      c = clen-1;
      for (j = 0; j < iLen; j++)
      {
            unsigned char Val[2];
            Val[0] = pBcdStr[j*2];
            Val[1] = pBcdStr[j*2+1];
            for (i = 0; i < 2; i++)
            {
                  if (Val[i] >= '0' && Val[i] <= '9')
                        Val[i] -= '0';
                  else if (Val[i] >= 'A' && Val[i] <= 'F')
                  {
                        Val[i] -= 'A';
                        Val[i] += 10;
                  }
                  else if (Val[i] >= 'a' && Val[i] <= 'f')
                  {
                        Val[i] -= 'a';
                        Val[i] += 10;
                  }
                  else
                        return 0;

            }
            pBin[c] = (Val[0] << 4) | Val[1];
            c--;
      }
      return iLen;
}

int getNextNumber(const char *str) {
    static const char *p = NULL;  // static pointer to remember position
    char numbuf[20];
    int i = 0;

    if (str != NULL) {  // first call: initialize pointer
        p = str;
    }

    if (p == NULL) return -1; // not initialized

    // Skip non-digit characters
    while (*p && !isdigit((unsigned char)*p)) {
        p++;
    }

    // If end of string reached
    if (*p == '\0') return -1;

    // Collect digits into buffer
    while (*p && isdigit((unsigned char)*p)) {
        numbuf[i++] = *p++;
    }
    numbuf[i] = '\0';

    return atoi(numbuf);
}

void UpdateDateTime(uint16_t year, uint16_t month, uint16_t day,
                    uint16_t hour, uint16_t minute, uint16_t second)
{
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};

    sTime.Hours   = hour;
    sTime.Minutes = minute;
    sTime.Seconds = second;
    HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

    sDate.Year  = (uint8_t)(year - 2000);
    sDate.Month = month;
    sDate.Date  = day;
    HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
}


void ReadDateTimeFromRTC(uint16_t *year, uint16_t *month, uint16_t *day,
                         uint16_t *hour, uint16_t *minute, uint16_t *second)
{
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};

    // Read current RTC time and date
    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

    if (year)   *year   = 2000 + sDate.Year;  // RTC year is 0–99
    if (month)  *month  = sDate.Month;
    if (day)    *day    = sDate.Date;
    if (hour)   *hour   = sTime.Hours;
    if (minute) *minute = sTime.Minutes;
    if (second) *second = sTime.Seconds;
}


uint64_t Ascii12ToUint64(const char *str)
{
    uint64_t value = 0;
    int i;

    if (str == NULL)
        return 0;

    for (i = 0; i < 12; i++) {
        char c = str[i];

        // Stop if string ends early
        if (c == '\0')
            break;

        // Ensure it's a digit
        if (!isdigit((unsigned char)c))
            return 0;  // or handle error as you wish

        value = value * 10ULL + (uint64_t)(c - '0');
    }

    return value;
}

// Write a uint64_t value to Flash
HAL_StatusTypeDef Flash_WriteValue(uint64_t value)
{
    HAL_StatusTypeDef status;
    uint32_t address = FLASH_USER_UUID_ADDR;

    // Unlock Flash for write access
    HAL_FLASH_Unlock();

    // Erase the page first
    FLASH_EraseInitTypeDef eraseInit;
    uint32_t pageError = 0;

    eraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
    eraseInit.PageAddress = FLASH_USER_START_ADDR;
    eraseInit.NbPages = 1;

    status = HAL_FLASHEx_Erase(&eraseInit, &pageError);
    if (status != HAL_OK) {
        HAL_FLASH_Lock();
        return status;
    }

    // Program 64-bit data (STM32F0 supports double word programming)
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, value);

    // Lock Flash again
    HAL_FLASH_Lock();

    return status;
}

// Read back the value from Flash
uint64_t Flash_ReadValue(void)
{
    uint64_t *ptr = (uint64_t *)FLASH_USER_UUID_ADDR;
    return *ptr;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
