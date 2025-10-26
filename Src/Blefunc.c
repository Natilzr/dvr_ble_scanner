//BLE functions

#include "main.h"
#include "stm32f0xx_hal.h"
#include "gp_timer.h"
#include "hal.h"
#include "hci.h"
#include "bluenrg_aci.h"
#include "osal.h"
#include "bluenrg_gatt_server.h"
#include "hci_const.h"
#include "bluenrg_gap.h"
#include "wli_ble.h"
#include "stm32_bluenrg_ble.h"
#include "blefunc.h"
#include "string.h"
#ifdef TRACE_VERSION
#include <stdarg.h>
void Trace_Error_New(char* pBuffer,char* format,...);
extern char DebugBuffer[];
char* PresentBinDataAsTextHexComplex(char* Buf,unsigned char* Bin,int Len,char Delimiter,BOOL bHighFirst);
int gCounter_AdvReport = 0;
int gCounter_TagsAdv = 0;
int StartErrCntr = 0;
tClockTime	ScanStartTick;

char szBdAddrText[6*3];
char szDataText[48*3];
#endif

extern DevicesSort DeviceTable[DEVICE_TABLE_SIZE];
extern uint8_t UUidFilter[4];
extern uint8_t testbuf[40];
uint16_t BT_handle;
extern   ProtocolS   PrPars[6];
extern     uint16_t Protocol;
extern uint16_t DeviceRespCount;
extern uint8_t ConState,LastConstate,RetConstate;
extern uint16_t Group_End_Handle;
extern uint16_t Found_Attribute_Handle;
extern uint16_t Temp_SrviceAtt;
extern uint8_t  Att_ReadVal[13];
extern uint8_t Att_ReadLengh;
//uint8_t UUIDs[16]={0xFD,0XA5,0X06,0X93,0X00,0X01,0X00,0X02,0XD8,0X2E,0X00,0X00,0X00,0X00,0X00,0X00};
uint8_t UUIDs[16] = { 0xFD, 0XA5, 0X06, 0X93, 0xA4, 0xE2, 0x4F, 0xB1, 0XD8,
		0X2E, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00 };

uint8_t StartBT(uint8_t M_ROLE) {
	int ret;
	uint8_t AddrLen;
	uint8_t g_BDdaddr[BD_ADDR_SIZE];
	const uint8_t mode = 2;
	BlueNRG_RST();
#if 0
#endif
	ret = aci_hal_read_config_data((uint8_t) CONFIG_DATA_RANDOM_ADDRESS,
			CONFIG_DATA_PUBADDR_LEN, &AddrLen, g_BDdaddr);
	if (ret) {
//		HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, GPIO_PIN_RESET); //turn on error led
		return ret;
	}
	ret = aci_hal_write_config_data(CONFIG_DATA_ROLE, CONFIG_DATA_ROLE_LEN,
			&mode);
	if (ret) {
//		HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, GPIO_PIN_RESET); //turn on error led
		return ret;
	}
	uint8_t bdaddr[] = { 0xff, 0x00, 0x00, 0xE1, 0x80, 0x02 };
	ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET,
			CONFIG_DATA_PUBADDR_LEN, bdaddr);
	if (ret) {
//		HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, GPIO_PIN_RESET); //turn on error led
		return ret;
	}
	ret = aci_hal_set_tx_power_level(1, 7);
	if (ret) {
//		HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, GPIO_PIN_RESET); //turn on error led
		return ret;
	}
	ret = aci_gatt_init();
	if (ret) {
//    SdkEvalLedOn(LED2);
		return ret;
	}
	uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
//	ret = aci_gap_init_IDB05A1(GAP_CENTRAL_ROLE_IDB05A1, 0, 0x07,
//			&service_handle, &dev_name_char_handle, &appearance_char_handle);
//        	ret = aci_gap_init_IDB05A1(GAP_OBSERVER_ROLE_IDB05A1, 0, 0x07,
//			&service_handle, &dev_name_char_handle, &appearance_char_handle);
        	ret = aci_gap_init_IDB05A1(M_ROLE, 0, 0x07,
			&service_handle, &dev_name_char_handle, &appearance_char_handle);        
	if (ret) {
		return ret;
	}

	const char *name = "BlueNRG";

	ret = aci_gatt_update_char_value(service_handle, dev_name_char_handle, 0,
			strlen(name), (uint8_t *) name);

	if (ret) {

		return ret;
	}
#if 0
	ret = hci_le_read_local_version(&hci_version,&hci_revision,&lmp_pal_version,
			&manufacturer_name,&lmp_pal_subversion);
	if(ret) {

		return ret;
	}
#endif
#ifdef TRACE_VERSION
    ret = getMyBlueNRGVersion(&gBlueNRGVersion);
    if (ret == BLE_STATUS_SUCCESS)
    {
          Trace_Error_New(DebugBuffer,"BLE %s",gBlueNRGVersion.szVersion);
    }
    else
          Trace_Error_New(DebugBuffer,"BLE Ver. get falied");
#endif
	return 0;
}

int8_t ScanBT(void) {
	int ret;
	// Refer D:\STMicroelectronics\BlueNRG DK 1.9.0\Docs\Downloads\en.DM00162667.pdf
	uint16_t scanInterval = 0x7d0; //was 7d0 2000msec
/*
scanInterval is a Time interval from when the controller started its last LE scan
until it begins the subsequent LE scan. The scan interval should
be a number in the range 0x0004 to 0x4000. This corresponds to
a time range 2.5 msec to 10240 msec. For a number N, Time = N
* 0.625 msec.
So, value 0x7d0 = 2000 dec * 0.625 = 1250 ms
*/
	uint16_t scanWindow = 0x7d0;
/*
Amount of time for the duration of the LE scan. Scan_Window
shall be less than or equal to Scan_Interval. The scan window
should be a number in the range 0x0004 to 0x4000. This
corresponds to a time range 2.5 msec to 10240 msec. For a
number N, Time = N * 0.625 msec
*/
	uint8_t own_address_type = 0;
/*
0x00: Public device address (default)
0x01: Random device address
*/
	uint8_t filterDuplicates = 0x01;
#ifdef TRACE_VERSION
	gCounter_AdvReport = 0;
	gCounter_TagsAdv = 0;
#endif
        
#if new_fix        
	ret = aci_gap_start_general_discovery_proc(scanInterval, scanWindow,
			own_address_type, filterDuplicates);
#endif
        
    
        
        ret = aci_gap_start_observation_procedure(scanInterval, scanWindow,ACTIVE_SCAN,
			own_address_type, filterDuplicates);
	{
          if (ret) {
                  
#ifdef TRACE_VERSION
			  StartErrCntr++;
#endif
                  return ret;
		} 
          HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, GPIO_PIN_RESET); //turn on error led
#ifdef TRACE_VERSION
		  ScanStartTick = HAL_GetTick();
		  Trace_Error_New(DebugBuffer,"BT scan started");
#endif

	}
	return 0;
}

int8_t ConnectBT(tBDAddr peer_bdaddr) {
	int ret;
	Flags &= ~DeviceChoos;
	uint16_t scanInterval = 0x7d0;
	uint16_t scanWindow = 0x7d0;
	uint8_t peer_bdaddr_type = 0x00;
	uint8_t own_bdaddr_type = 0x00;
	uint16_t conn_min_interval = 0x014;
	uint16_t conn_max_interval = 0x028;
	uint16_t conn_latency = 0x00;
	uint16_t supervision_timeout = 0x64;
	uint16_t min_conn_length = 0x02;
	uint16_t max_conn_length = 0x02;
	ret = aci_gap_create_connection(scanInterval, scanWindow, peer_bdaddr_type,
			peer_bdaddr, own_bdaddr_type, conn_min_interval, conn_max_interval,
			conn_latency, supervision_timeout, min_conn_length,
			max_conn_length);
	if (ret) {
//		HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, GPIO_PIN_RESET); //turn on error led
		return ret;
	}
	return 0;
}

void HCI_Event_CB(void *pckt) {
//	uint8_t *pUUID;
	uint8_t bt_status;
	hci_uart_pckt *hci_pckt = pckt;
	hci_event_pckt *event_pckt = (hci_event_pckt*) hci_pckt->data;
	evt_att_read_resp *popo;
	evt_att_read_blob_resp *ww;
	evt_att_prepare_write_resp *qq;
        evt_gatt_procedure_complete *evt;
        evt_gatt_disc_read_char_by_uuid_resp *resp1;
	//evt_gatt_procedure_complete *vv;
        evt_gap_procedure_complete *vv;
        evt_att_find_by_type_val_resp *res;
	if (hci_pckt->type != HCI_EVENT_PKT)
		return;
	switch (event_pckt->evt) 
        {
        case EVT_DISCONN_COMPLETE: 
          {
              WAPP_FLAG_SET(STATE_UPDATE);
              ConState = DisconnectedEnd;
          }
          break;
	case EVT_LE_META_EVENT: 
          {
		evt_le_meta_event *evt = (void *) event_pckt->data;
		switch (evt->subevent) 
                {
		case EVT_LE_CONN_COMPLETE: 
                  {
			evt_le_connection_complete *ee =
					(evt_le_connection_complete *) evt->data;
			BT_handle = ee->handle;
			bt_status = ee->status;
                        WAPP_FLAG_SET(WCONNECTED);
                        WAPP_FLAG_SET(STATE_UPDATE);
                        break;
                  }
		case EVT_LE_ADVERTISING_REPORT: 
                  {
			le_advertising_info *cc = (le_advertising_info *) (evt->data + 1); //delete here -1
                        advertising_report_event_handler(cc);
                        // Advertising report structure: refer Bluetooth Core v4.1 specification, section 7.7.65.2 LE Advertising Report Event on page 938-939
                        // Here the Num_Reports is always 1
                        // data_length includes only length of Data[] and don't include RSSI snt after Data[]
                        // Note: for RDL iBeacon sensors the standard RSSI is always 0
                        break;
                  }
		case EVT_LE_READ_REMOTE_USED_FEATURES_COMPLETE: 
                  {
                  }
			break;
		}// evt->subevent
	}//end EVT_LE_META_EVENT
		break;

	case EVT_VENDOR: 
          {
		evt_blue_aci *blue_evt = (void*) event_pckt->data;
		switch (blue_evt->ecode) 
                {
                case EVT_BLUE_ATT_FIND_BY_TYPE_VAL_RESP:
                  {
                      res = (evt_att_find_by_type_val_resp*)blue_evt->data;
                      for (uint8_t i = 0; i < res->event_data_length; i++) 
                      {
                        Readbuf.data[i] = res->handles_info_list[i];
			//           sprintf((char*)textbuf+(3*i),"%02x ",popo->attribute_value[i]);
                      }
                      WAPP_FLAG_SET(STATE_UPDATE);
                      Group_End_Handle = (Readbuf.data[3] << 8) | Readbuf.data[2];
                      Found_Attribute_Handle = (Readbuf.data[1] << 8) | Readbuf.data[0];
                      // ConState = CharAttDiscoverySearch ;
                       break;
                  }
                  case EVT_BLUE_GATT_DISC_READ_CHAR_BY_UUID_RESP: 
                    {
                      resp1 = (evt_gatt_disc_read_char_by_uuid_resp*)blue_evt->data;
                      for (uint8_t i = 0; i < resp1->event_data_length; i++) 
                      {
                        Readbuf.data[i] = resp1->attr_value[i];
			//           sprintf((char*)textbuf+(3*i),"%02x ",popo->attribute_value[i]);
                      }

                      WAPP_FLAG_SET(STATE_UPDATE);
                      Temp_SrviceAtt = resp1->attr_handle;
                      ConState = CharAttDiscovery;
  
                      break;
                    }
		case EVT_BLUE_ATT_READ_RESP: 
                  {
			popo = (evt_att_read_resp*) blue_evt->data;
			for (uint8_t i = 0; i < popo->event_data_length; i++) 
                        {
				Readbuf.data[i] = popo->attribute_value[i];
                                Att_ReadVal[i] =  popo->attribute_value[i];
			//           sprintf((char*)textbuf+(3*i),"%02x ",popo->attribute_value[i]);
			}
			Readbuf.length = popo->event_data_length;

                            Att_ReadLengh = popo->event_data_length;
                            WAPP_FLAG_SET(ATT_DATA);  
                    
                            break;
		}
		case EVT_BLUE_ATT_READ_BLOB_RESP: 
                  {
			ww = (evt_att_read_blob_resp*) blue_evt->data;
			switch (TestStatus) {
			default:
				break;

			}
                  }
			break;
		case EVT_BLUE_GAP_PROCEDURE_COMPLETE: {
			vv = (evt_gap_procedure_complete*) blue_evt->data;
                        memcpy(testbuf+3, &event_pckt->data, 10);
                        testbuf[0]=vv->procedure_code;
                        testbuf[1]=vv->status;
			if (vv->procedure_code == 0x02)          //finish searching
					{
				Flags |= ScanEnd;
				TestStatus = Scan_end;
			}

		}
			break;
		case EVT_BLUE_ATT_PREPARE_WRITE_RESP:
			qq = (evt_att_prepare_write_resp*) blue_evt->data;
			for (uint8_t i = 0; i < qq->event_data_length; i++) {
				Readbuf.data[i] = qq->part_attr_value[i];
				//sprintf((char*)textbuf+(3*i),"%02x ",ww->part_attribute_value[i]);
			}

			printf("VenEvent %u\r\n", qq->conn_handle);
			break;
		case EVT_BLUE_ATT_EXEC_WRITE_RESP:

			break;
		case EVT_BLUE_GATT_PROCEDURE_COMPLETE:
                        evt = (evt_gatt_procedure_complete*)blue_evt->data;
                        proc_complete_event(evt);
			break;
		default:
			break;
		}

		//printf("evnt %x %x %x %x \n\r",event_pckt->data[0],event_pckt->data[1],event_pckt->data[2],event_pckt->data[3]);
	}
		break;
	}

}

#ifdef TRACE_VERSION
BLUENGR_VERSION_INFO gBlueNRGVersion;
static char* GetHCIVersionDescription(uint8_t hci_version);

uint8_t getMyBlueNRGVersion(BLUENGR_VERSION_INFO* pVer)
{
	 // See: UM1865- BlueNrg MS BLE app.comamnd interface.pdf, HCI commands - p.19
	 // Bleetooth core specification, Vol.2, PArt E, Chapter 7
	 // Returned values:
	 //		see https://www.bluetooth.com/specifications/assigned-numbers/host-controller-interface
	 // hci_version = 7 (Bluetooth Core Specification 4.1)
	 // hci_revision = 0x3107 (Revision of current HCI in the BR/EDR controller
	 // lmp_pal_version = 0x07 - Current LMP(Link Manager Protocol) or PAL(Protocol Adaptation Level) in the controller (Bluetooth Core Specification 4.1)
	 // manufacturer_name = 0x30 - Manufacturer name of BR/EDR controller 
	 // lmp_pal_subversion = 0x13 - subversion of LMP or PAL in the controller (implementation delendent)
	char szFwDescriptor[48];
	int Offset;
	char* pCoreDesc;
  pVer->Status = hci_le_read_local_version(&pVer->hci_version, &pVer->hci_revision, &pVer->lmp_pal_version, 
				     &pVer->manufacturer_name, &pVer->lmp_pal_subversion);

  if (pVer->Status == BLE_STATUS_SUCCESS) {
    pVer->hwVersion = pVer->hci_revision >> 8;
    pVer->fwVersion = (pVer->hci_revision & 0xFF) << 8;              // Major Version Number
    pVer->fwVersion |= ((pVer->lmp_pal_subversion >> 4) & 0xF) << 4; // Minor Version Number
    pVer->fwVersion |= pVer->lmp_pal_subversion & 0xF;               // Patch Version Number
		Offset = sprintf(szFwDescriptor,"Core ");
		pCoreDesc = GetHCIVersionDescription(pVer->hci_version);
		if (pCoreDesc)
			Offset += sprintf(szFwDescriptor+Offset,"%s",pCoreDesc); 
		else
			Offset += sprintf(szFwDescriptor+Offset,"ID#%u",pVer->hci_version); 
		Offset += sprintf(szFwDescriptor+Offset," HW%d.%c;FW%d.%d%c", 
						BLUNRG_HW_VERSION_HIGH(pVer->hwVersion),
						BLUNRG_HW_VERSION_LOW_CHAR(pVer->hwVersion),
						BLUNRG_FW_VERSION_HIGH(pVer->fwVersion),
						BLUNRG_FW_VERSION_LOW(pVer->fwVersion),
						BLUNRG_FW_VERSION_PATCH_CHAR(pVer->fwVersion));
		strncpy(pVer->szVersion,szFwDescriptor,sizeof(pVer->szVersion)-1);
		pVer->szVersion[sizeof(pVer->szVersion)-1] = '\0';

  }

  HCI_Process(); // To receive the BlueNRG EVT

  return pVer->Status;
}
static char* GetHCIVersionDescription(uint8_t hci_version)
{
	// see https://www.bluetooth.com/specifications/assigned-numbers/host-controller-interface
  switch(hci_version)
  {
	case 0:
		return "1.0b";
	case 1:
		return "1.1";
	case 2:
		return "1.2";
	case 3:
		return "2.0";
	case 4:
		return "2.1";
	case 5:
		return "3.0";
	case 6:
		return "4.0";
	case 7:
		return "4.1";
	case 8:
		return "4.2";
	default:
    break;
  }
		return NULL;
}

#endif



BOOL IsDataAcceptable(le_advertising_info* pReport)
{
 //     uint8_t *pUUID;
     // pUUID = Advertising_Report[0].Data + PrPars[Protocol].UuidOffset;
  //    pUUID = pReport[0].Data + PrPars[Protocol].UuidOffset;
      //pUUID = data + PrPars[Protocol].UuidOffset;//3 tlm for beacon 9;
    //  if (memcmp(pUUID,PrPars[Protocol].UUidFilter, 4) == 0)//was 16
        if(pReport[0].data_RSSI[PrPars[Protocol].UuidOffset]==PrPars[Protocol].UUidFilter[0] &&
           pReport[0].data_RSSI[PrPars[Protocol].UuidOffset+1]==PrPars[Protocol].UUidFilter[1] &&
           pReport[0].data_RSSI[PrPars[Protocol].UuidOffset+2]==PrPars[Protocol].UUidFilter[2] &&
           pReport[0].data_RSSI[PrPars[Protocol].UuidOffset+3]==PrPars[Protocol].UUidFilter[3])
      {
        if(pReport[0].data_length > 13)
        {
                return TRUE;
        }else
        {
          return FALSE;
        }
      }
      else
      {
          return FALSE;      
      }
}

BOOL IsScanResponseDataAcceptable(le_advertising_info* pReport)
{
     if(Protocol == 0)//ibeacon
    {
      /*IBEACON ?BEACON BATTERY
      0X09 LENGTH ,0X16 TYPE , BATTERY PRESENT
      WE COPY 10 BYTES
      */
      if((pReport[0].data_RSSI[0] == 0x09) && (pReport[0].data_RSSI[1]== 0x16) && (pReport[0].data_RSSI[2]== 0x80))
      {
                return TRUE;
      }
      else
      {
                return FALSE;
      }
    }
    else if(Protocol == 1)
    {
      if((pReport[0].data_RSSI[0] == 0x09) && (pReport[0].data_RSSI[1]== 0xFF))
      {
                return TRUE;
      }
      else if((pReport[0].data_RSSI[0] == 0x0B) &&
           (pReport[0].data_RSSI[1]== 0xFF) &&
           (pReport[0].data_RSSI[2]== 0xFF) && 
           (pReport[0].data_RSSI[3]== 0xFE))
           {
                return TRUE;
           }
      else  
      {
                return FALSE;
      }
              
    }
    return FALSE;
}




BOOL AddTagId(le_advertising_info* pReport,TestDevices* pDevices, int idxsToAdd, uint16_t *pCounter)
{
          int idx1;
          uint8_t y=0;
         int64_t  i64Addr;

         if(*pCounter >= SCAN_TABLE_SIZE)
                                return FALSE;
        for (idx1 =  *pCounter; idx1  >  idxsToAdd; idx1--)
                                memcpy(&pDevices[idx1],&pDevices[idx1-1],sizeof(TestDevices));
                // add new

        memcpy(&i64Addr,pReport->bdaddr,6);
        i64Addr &= 0xFFFFFFFFFFFFL;
                    if (i64Addr == 0x571900160024)
            {
             y++; 
            }
        pDevices[idxsToAdd].i64Addr = i64Addr;
                pDevices[idxsToAdd].length = pReport->data_length;
                if (pReport->data_length > (BLE_ADV_LEN_MAX-1))
                  pReport->data_length  = BLE_ADV_LEN_MAX-1;//-1 because RSSI
        pDevices[idxsToAdd].lengthEx = 0;
                memcpy(pDevices[idxsToAdd].data,pReport->data_RSSI,pReport->data_length);
                pDevices[idxsToAdd].Rssi = pReport->data_RSSI[(pReport->data_length)];
                          if(pDevices[idxsToAdd].Rssi < 0x80)
                            pDevices[idxsToAdd].Rssi = pReport->data_RSSI[(pReport->data_length)+1];
                //ScanDevices[DeviceCount].Rssi = cc->data_RSSI[cc->data_length];
                // pDevices[*pCounter].type ???
                (*pCounter)++;
                return TRUE;
}

// Check the tag bdaddr is prersent in thew devices list
//            If the tag is not fiund, *pIdxToAdd is a index where the new record must be added (as sorted in low-To-High order by address)
//                            - in this case to add this new entry: - move all the following records to next index, then add the new in *pIdxToAdd
//            If the tag is found the *pIdxToAdd is the index of the found enrfty - you can update its data if need
BOOL IsTagIdPresent(uint8_t Address[6],TestDevices* pDevices, int Counter,int* pIdxToAdd)
{
    int left = 0;
    int8_t y;
    int right = Counter - 1;
    int64_t  i64Addr;
    memcpy(&i64Addr,Address,6);
    i64Addr &= 0xFFFFFFFFFFFFL;
    while (left <= right)
    {
            int mid =  (left + right)/2;
            int64_t  i64MidAddr = pDevices[mid].i64Addr;
            i64MidAddr &= 0xFFFFFFFFFFFFL;

            if (i64Addr == i64MidAddr)
            {
                if (pIdxToAdd)
                  *pIdxToAdd = mid;
                return TRUE;
            }
            if (i64MidAddr > i64Addr)
                    right = mid - 1;
            else
                    left = mid + 1;

    }
    if (pIdxToAdd)
    *pIdxToAdd = left;
    return FALSE;

}


void advertising_report_event_handler(le_advertising_info *Advertising_Report)
{
    int addidx;
//  uint8_t *pUUID;
  if(Advertising_Report[0].evt_type == SCAN_RSP) 
   {
//     endcountRes++;
        if (IsScanResponseDataAcceptable(&Advertising_Report[0]))
        {
          if(IsTagIdPresent(Advertising_Report[0].bdaddr,ScanDevices,DeviceCount,&addidx))
          {
            if (DeviceRespCount < SCAN_TABLE_SIZE)
            {
              DeviceRespCount++;
              // Update addidx entry (ScanDevices[addidx].lengthEx will be > 0
              if(Protocol == 0)//IBEACON
              {
                ScanDevices[addidx].lengthEx = Advertising_Report[0].data_length;
                if (ScanDevices[addidx].lengthEx > BLE_ADV_LEN_MAX)
                ScanDevices[addidx].lengthEx = BLE_ADV_LEN_MAX;
                memcpy(ScanDevices[addidx].dataEx,Advertising_Report[0].data_RSSI,ScanDevices[addidx].lengthEx);
              }
              else if(Protocol == 1)//TLM
              {
                if(Advertising_Report[0].data_RSSI[0] == 0x09)
                {
                  ScanDevices[addidx].dataEx[0] = 0xff;
                  ScanDevices[addidx].dataEx[1] = 0xff;
                }
                else // WLI_TLM
                {
                  ScanDevices[addidx].dataEx[0] = Advertising_Report[0].data_RSSI[10];
                  ScanDevices[addidx].dataEx[1] = Advertising_Report[0].data_RSSI[11];
                }
                ScanDevices[addidx].lengthEx = Advertising_Report[0].data_length;
                Advertising_Report[0].data_RSSI[0] = 0;
              }
          }
         }
        // MAC already present in ScanDevciesTable - update data from Scan Responce report        
      }
   }
   else // advertizing
   {
 //    endcountAdv++;
     if (IsDataAcceptable(&Advertising_Report[0]))
     {          
        if(IsTagIdPresent(Advertising_Report[0].bdaddr,ScanDevices,DeviceCount,&addidx) == FALSE)
        {
          if (DeviceCount < SCAN_TABLE_SIZE)
          {
            AddTagId(&Advertising_Report[0],ScanDevices,addidx,&DeviceCount);
          }
        }
      }
   }
}


void proc_complete_event(evt_gatt_procedure_complete *pevt)
{
      switch(ConState)
    {
    case DeviceConnected:
      break;
    case WriteChar:
      ConState = ReadMem;
      WAPP_FLAG_SET(STATE_UPDATE);
      break;
    case MainServiceDiscovery:
      ConState = CharAttDiscoverySearch;// finish main service search for services
      WAPP_FLAG_SET(STATE_UPDATE);
      break;
    case CharAttDiscovery:
        ConState = WriteChar;
        WAPP_FLAG_SET(STATE_UPDATE); 
      break;
    case ReadMem:
       WAPP_FLAG_SET(STATE_UPDATE);
       WAPP_FLAG_CLEAR(ATT_DATA);  
       ConState = DataRead;
      break;
    default:
      break;
    }
}