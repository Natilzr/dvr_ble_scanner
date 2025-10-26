
#include "main.h"
#include "wli_ble.h"
#include "gp_timer.h"
#include "hci.h"
#include "bluenrg_aci.h"
#include "bluenrg_gap.h"
#include "osal.h"
#include "serialdrivers.h"
#include "printfunc.h"
#include "Connfunc.h"

 const uint8_t  charUuidmsg[16] = {0x00,0x00,0x12,0xac,0x42,0x02,0x1d,0x86,0xed,0x11,0xad,0x23,0x64,0x58,0x0f,0xdd};
 const uint8_t charUuidmsg1[16] = {0x01,0x00,0x12,0xac,0x42,0x02,0x1d,0x86,0xed,0x11,0xad,0x23,0x64,0x58,0x0f,0xdd};

 UUID_t myservice;
uint16_t Group_End_Handle;
uint16_t Found_Attribute_Handle;
 
extern uint8_t ConState,LastConstate,RetConstate;
extern uint16_t DelayCnt;
extern ERROR_STATUS    S_Error,St_Error; 
extern uint16_t MsgCnt;
extern uint8_t RunState;
extern processt t_process;
extern uint8_t textbuf[50];
extern  uint8_t TempMac[6];
extern uint16_t My_Con_Handle;
extern uint32_t TempAge;
extern uint8_t Att_ReadLengh;
extern uint8_t  Att_ReadVal[13];
extern uint16_t BT_handle;
extern uint16_t Temp_SrviceAtt;
extern uint8_t  Att_ReadVal[13];
extern uint8_t Att_ReadLengh;
void ConnFunc(void)
{
    uint8_t ret;
    uint8_t Att_Val[8] ;
    uint16_t MsgLen = 0;
    char tBuff[20];
    Att_Val[0]=0xff;
    Att_Val[1]=0xff;
    Att_Val[2]=0xff;
    Att_Val[3]=0xff;
    if(WAPP_FLAG(STATE_UPDATE))
    {
      WAPP_FLAG_CLEAR(STATE_UPDATE);
      switch(ConState)
      {
      case IdleC:
        break;
      case KillScan:
              S_Error = NOT_DEFINED;
             if(!(Flags & ScanEnd))
             {

               ret = aci_gap_terminate_gap_procedure(GAP_OBSERVATION_PROC_IDB05A1);
               ConState =  SwitchCentral;
             }
             else
             {
               ConState =  SwitchCentral;
             }
             WAPP_FLAG_SET(STATE_UPDATE);
             MsgCnt = 1;
             
        break;
      case SwitchCentral:
        StartBT(GAP_CENTRAL_ROLE_IDB05A1);
        ConState =  StartCon;
        WAPP_FLAG_SET(STATE_UPDATE);
        break;
      case StartCon:
        //ret = deviceConnection();//no stack timout

        ret = aci_gap_create_connection(

                                    0x4000,//LE_Scan_Interval
                                    0x4000,//LE_Scan_Window
                                    //0x00,//Initiator_Filter_Policy
                                    RANDOM_ADDR,//Peer_Address_Type
                                    TempMac,//Peer_Address[6]
                                    0,//Own_Address_Type
                                    0x0006,//Conn_Interval_Min
                                    0x0028,//Conn_Interval_Max
                                    0,//Conn_Latency
                                    0x03E8,//Supervision_Timeout
                                    0x0000,//Minimum_CE_Length
                                    0x03E8);//Maximum_CE_Length 

#if 0
    
        uint16_t scanInterval   =       0x7d0;
        uint16_t scanWindow     =       0x7d0;
        uint8_t peer_bdaddr_type=       0x00;
        uint8_t own_bdaddr_type =       0x00;
        uint16_t conn_min_interval =    0x014;
        uint16_t conn_max_interval =    0x028;
        uint16_t conn_latency =         0x00;
        uint16_t supervision_timeout =  0x64;
        uint16_t min_conn_length =      0x02;
        uint16_t max_conn_length =      0x02;
        ret = aci_gap_create_connection(scanInterval,scanWindow,
				     peer_bdaddr_type,TempMac,	
				     own_bdaddr_type, conn_min_interval,	
				     conn_max_interval,conn_latency,	
				     supervision_timeout, min_conn_length, 
				     max_conn_length);       
 
        #endif
        
        if (ret == BLE_STATUS_SUCCESS) 
        {
          HAL_GPIO_WritePin(LED_Red_GPIO_Port,LED_Red_Pin, GPIO_PIN_SET);//turn on error led 
          RetConstate = WaitForCon;
          DelayCnt = 10000;//60000;
          ConState = ConDelay;
          WAPP_FLAG_SET(STATE_UPDATE | DELAYF);
        }
        else if(ret == ERR_INVALID_HCI_CMD_PARAMS  )
        {
 //        WAPP_FLAG_CLEAR(MASTER_C);
//         WAPP_FLAG_SET(STATE_UPDATE);
         t_process.on_going = BtInit;
         RunState = BtInit;
         //BLE_Init();
         
        }
        else if(ret == ERR_COMMAND_DISALLOWED)
        {
          //MsgLen = EncodeConnectStatus(NOT_WLI,NOT_DEFINED);
          //WriteSerial(COM1,(char*)textbuf,MsgLen);
 //         WAPP_FLAG_CLEAR(MASTER_C);
 //        WAPP_FLAG_SET(STATE_UPDATE);
         t_process.on_going = BtInit;
         RunState = BtInit;
         //BLE_Init();
        }
//        WAPP_FLAG_SET(STATE_UPDATE);
        break;
      case WaitForCon:
        if(WAPP_FLAG(WCONNECTED))
        {
             WAPP_FLAG_SET(STATE_UPDATE);
             ConState = DeviceConnected;
             memset(textbuf,'\0',TEXT_BUFFER_SIZE);
             sprintf ((char*)tBuff,"Connectedx");//,9);
             MsgLen = EncodeTrace((uint8_t*)tBuff,10);
             WriteSerial(COM1,(char*)textbuf,MsgLen);

        }
        else
        {
          ConState = KillScan;
          WAPP_FLAG_SET(STATE_UPDATE);
          MsgLen = EncodeConnectStatus(NOT_CONNECTABLE);
          WriteSerial(COM1,(char*)textbuf,MsgLen);
        }
        break;
      case DeviceConnected:     
        Osal_MemCpy(&myservice.UUID_128, charUuidmsg, 16);//start main service by UUID
        // ret = aci_gatt_disc_all_primary_services(My_Con_Handle);
       // ret = aci_gatt_disc_primary_service_by_uuid(My_Con_Handle,0x02,&myservice);
        My_Con_Handle = BT_handle;
        ret = aci_gatt_disc_prim_service_by_uuid(My_Con_Handle,UUID_TYPE_128,myservice.UUID_128);
        if (ret == BLE_STATUS_SUCCESS) 
        {
             WAPP_FLAG_SET(STATE_UPDATE);
             ConState = MainServiceDiscovery;
             memset(textbuf,'\0',TEXT_BUFFER_SIZE);
             sprintf (tBuff,"Service ");
             MsgLen = EncodeTrace((uint8_t*)tBuff,10);
             WriteSerial(COM1,(char*)textbuf,MsgLen);             
        }  
        else
        {
          LastConstate = ConState;
          ConState = ConError;
          S_Error = SERVICE_UUID_ERROR;

          WAPP_FLAG_SET(STATE_UPDATE);

        }
        break;
      case MainServiceDiscovery:
        //wait for service discovery results after aci_gatt_proc_complete_event
        //think timeout
        break;

      case CharAttDiscoverySearch:
        Osal_MemCpy(&myservice.UUID_128, charUuidmsg1, 16);
        /*
        ret = aci_gatt_disc_char_by_uuid(My_Con_Handle,
                                      Temp_MainSrviceAtt.Found_Attribute_Handle,
                                      Temp_MainSrviceAtt.Group_End_Handle,
                                      0x02,
                                      &myservice);
        */
        ret = aci_gatt_disc_charac_by_uuid(My_Con_Handle, Found_Attribute_Handle,
				                     Group_End_Handle,0x02,
                                                     myservice.UUID_128);
        if (ret == BLE_STATUS_SUCCESS) 
        {
          WAPP_FLAG_SET(STATE_UPDATE);
          ConState = CharAttDiscovery;
             memset(textbuf,'\0',TEXT_BUFFER_SIZE);
             sprintf (tBuff,"Char");//,10);
             MsgLen = EncodeTrace((uint8_t*)tBuff,10);
             WriteSerial(COM1,(char*)textbuf,MsgLen);
        }  
        else
        {
          LastConstate = ConState;
          ConState = ConError;
          S_Error = ERROR_DISC_CHAR;
          WAPP_FLAG_SET(STATE_UPDATE);
        }
        break;
      case CharAttDiscovery:
        //wait for service discovery results after aci_gatt_proc_complete_event
        //think timeout
        break;
      case WriteChar:
//             MsgLen = EncodeConnectStatus(CONNECTED_S);
//             WriteSerial(COM1,(char*)textbuf,MsgLen);
             memset(textbuf,'\0',TEXT_BUFFER_SIZE);
             sprintf (tBuff,"Write");//,10);
             MsgLen = EncodeTrace((uint8_t*)tBuff,10);
             WriteSerial(COM1,(char*)textbuf,MsgLen);
        //write ff to read history
      Att_Val[3] = (uint8_t)( TempAge        & 0xff);
      Att_Val[2] = (uint8_t)((TempAge >>  8) & 0xff);
      Att_Val[1] = (uint8_t)((TempAge >> 16) & 0xff);
      Att_Val[0] = (uint8_t)((TempAge >> 24) & 0xff);

 //       WAPP_FLAG_SET(WRITE_CHAR);

//        ret = aci_gatt_write_char_value(My_Con_Handle,
//                                     Temp_SrviceAtt+1,//0x12,
//                                     0x04,
//                                     Att_Val);

ret = aci_gatt_write_charac_value(My_Con_Handle, 
                            Temp_SrviceAtt+1,//0x12,, 
				       0x04,
                                       Att_Val);       

        if (ret == BLE_STATUS_SUCCESS) 
        {
               
        }
        else
        {
          LastConstate = ConState;
          ConState = ConError;
          S_Error = ERROR_WRITE_CHAR;
           WAPP_FLAG_SET(STATE_UPDATE);
        }
        break;
      case ReadMem:
#if 0
        ret = aci_gatt_read_char_value(My_Con_Handle,
                                    Temp_SrviceAtt+1);//0x12);
#endif
        ret = aci_gatt_read_charac_val(My_Con_Handle,Temp_SrviceAtt+1);//0x12);
        
        if (ret == BLE_STATUS_SUCCESS) 
        {
          
        }
        else
        {
                    LastConstate = ConState;
                    ConState = ConError;
                    S_Error = ERROR_READ_CHAR;
                  WAPP_FLAG_SET(STATE_UPDATE);        
        }
        break;
      case DataRead:

          if((Att_ReadLengh == 1) && (Att_ReadVal[0] == 0))
          {
            WAPP_FLAG_SET(STATE_UPDATE);
            WAPP_FLAG_SET(END_RX);
            ConState = SendEndRXChar;
          }
          else if(Att_ReadVal[0] == 0x72)
          {
             WAPP_FLAG_SET(STATE_UPDATE);
            ConState = SendRXChar;           
      
          }
          else if(Att_ReadVal[0] == 0x71)
          {
             WAPP_FLAG_SET(STATE_UPDATE);
            ConState = SendRXChar;           
      
          }
          else
          {
             WAPP_FLAG_SET(STATE_UPDATE);
            WAPP_FLAG_SET(END_RX);
            ConState = SendEndRXChar;     
          }
        
        break;
      case SendRXChar:
        MsgLen = PrintMemMsg(MsgCnt);
        WriteSerial(COM1,(char*)textbuf,MsgLen);
        MsgCnt++;
        DelayCnt = 100;//was 1000
        ConState = ConDelay;
        RetConstate = ReadMem;
        WAPP_FLAG_SET(STATE_UPDATE | DELAYF);

        break;
      case SendEndRXChar:
        MsgLen = EncodeConnectStatus(END_OF_DATA);
        WriteSerial(COM1,(char*)textbuf,MsgLen);
        WAPP_FLAG_SET(STATE_UPDATE);       
        ConState = DisconnectRX;
        break;
      case DisconnectRX:
        ret = hci_disconnect(My_Con_Handle,
                          0x13);//0x13: Remote User Terminated Connection
        if (ret == BLE_STATUS_SUCCESS) 
        {
          MsgLen = EncodeConnectStatus(DISCONNECTED__S);
          WriteSerial(COM1,(char*)textbuf,MsgLen);    
        }
        break;
      case ConDelay:

        if(!WAPP_FLAG(DELAYF))
        {
          ConState = RetConstate;
          WAPP_FLAG_SET(STATE_UPDATE);
        }
        break;
      case ConError:
         MsgLen = EncodeConnectStatus(ERROR_P);
         WriteSerial(COM1,(char*)textbuf,MsgLen);
         WAPP_FLAG_CLEAR(MASTER_C);
         WAPP_FLAG_SET(STATE_UPDATE);
         t_process.on_going = BtInit;
         RunState = BtInit;
         //BLE_Init();
         StartBT(GAP_OBSERVER_ROLE_IDB05A1);
         ConState = KillScan  ;    
        break;
      case DisconnectedEnd:
        WAPP_FLAG_CLEAR(MASTER_C);
        WAPP_FLAG_SET(STATE_UPDATE);
        t_process.on_going = BtInit;
        RunState = BtInit;
        //BLE_Init();
        StartBT(GAP_OBSERVER_ROLE_IDB05A1);
        ConState = KillScan  ;  
       break;
       case FailEnd:
        MsgLen = EncodeConnectStatus(NOT_CONNECTABLE);
        WriteSerial(COM1,(char*)textbuf,MsgLen);
        WAPP_FLAG_SET(STATE_UPDATE);       
        ConState = DisconnectedEnd;
       break;
      case DiscError:
        MsgLen = EncodeConnectStatus(NOT_CONNECTABLE);
        WriteSerial(COM1,(char*)textbuf,MsgLen);
        WAPP_FLAG_SET(STATE_UPDATE);       
        ConState = DisconnectedEnd;
        break;
      case NoService:
          ConState = DisconnectedEnd;
          WAPP_FLAG_SET(STATE_UPDATE);
          MsgLen = EncodeConnectStatus(NOT_WLI);
          WriteSerial(COM1,(char*)textbuf,MsgLen);
        break;
      default:
        break;
      }
    }
}