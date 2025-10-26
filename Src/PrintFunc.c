/* print functions */

#include "main.h"
#include "stm32f0xx_hal.h"
#include "hci_const.h"
#include "wli_ble.h"
#include "string.h"
#include "printfunc.h"


#define STX 0x02
#define ETX 0x03
#define CDPD_SLIP_ESCAPED_STX		0xd2
#define CDPD_SLIP_ESCAPED_ETX		0xd3
#define CDPD_SLIP_ESCAPE		0xdb
#define CDPD_SLIP_ESCAPED_ESCAPE	0xdd

//#define MinTemp 3150
#define MinTemp 3070
#define MaxTemp 4200
const uint16_t TempTable[]={
3530 ,3535 ,3540 ,3545 ,3550 ,3555 ,3560 ,3565 ,3570 ,3575 ,
3580 ,3588 ,3596 ,3604 ,3612 ,3620 ,3623 ,3625 ,3628 ,3631 ,
3634 ,3636 ,3639 ,3642 ,3644 ,3647 ,3650 ,3652 ,3655 ,3658 ,
3661 ,3663 ,3666 ,3669 ,3671 ,3674 ,3677 ,3679 ,3682 ,3685 ,
3688 ,3690 ,3693 ,3696 ,3698 ,3700 ,3706 ,3711 ,3717 ,3722 ,
3728 ,3733 ,3739 ,3744 ,3750 ,3758 ,3766 ,3774 ,3783 ,3791 ,
3800 ,3809 ,3818 ,3827 ,3836 ,3845 ,3854 ,3863 ,3872 ,3881 ,
3890 ,3900 ,3909 ,3918 ,3927 ,3936 ,3945 ,3954 ,3963 ,3972 ,
3981 ,3990 ,4000 ,4009 ,4018 ,4027 ,4036 ,4045 ,4054 ,4063 ,
4072 ,4081 ,4090 ,4100 ,4109 ,4118 ,4127 ,4136 ,4145 ,4154 ,
4163 ,4172 ,4181 ,4190 ,4200 ,4209 ,4218 ,4227 ,4236 ,4245 ,
4254 ,4263 ,4272 ,4281 ,4290 ,4300 };

extern TestDevices ScanDevices[SCAN_TABLE_SIZE];
extern     uint16_t Protocol;
extern DevicesSort DeviceTable[DEVICE_TABLE_SIZE];
extern uint8_t Mode;
extern   uint8_t DevName[19];
extern uint32_t TempAge;
extern  uint8_t TempMac[6];
extern ERROR_STATUS    S_Error,St_Error;
extern uint8_t  Att_ReadVal[13];
extern uint16_t DateTime[6];
extern uint64_t UNIT_ID;
uint16_t  PrintMsg(uint16_t cnt)
  {
        uint8_t		        au8Data[45];
	uint8_t 		u8Data = 0;
	uint16_t 		u8Len = 0;
	uint8_t                 u8Pos;
	uint32_t 		u32Checksum;
	uint32_t 		a = 0;
	uint32_t 		b = 0;
        uint16_t                batt_d;
        uint16_t                T_send;
        float                   Temp;
        uint8_t                 T1,T2;
        int16_t                 Temp16;
        uint16_t                batt;
        uint16_t                pres;
        uint16_t                   ConvT,DeltaT;
        int8_t                  temp1,temp2;
        int16_t                 shTemp;
        uint16_t                TagMask;
        SensorData              kbprodata;
        uint8_t                 Hy[2];
        au8Data[u8Data++] = 34;//lenght low
        au8Data[u8Data++] = 0x00;//lenght high
        au8Data[u8Data++] = cnt;//seq  
        au8Data[u8Data++] = PACKET_TYPE_BEACON;
 
        au8Data[u8Data++] = (uint8_t)((ScanDevices[cnt].i64Addr >>  40) & 0xff);
        au8Data[u8Data++] = (uint8_t)((ScanDevices[cnt].i64Addr >>  32) & 0xff);
        au8Data[u8Data++] = (uint8_t)((ScanDevices[cnt].i64Addr >>  24) & 0xff);
        au8Data[u8Data++] = (uint8_t)((ScanDevices[cnt].i64Addr >>  16) & 0xff);
        au8Data[u8Data++] = (uint8_t)((ScanDevices[cnt].i64Addr >>  8) & 0xff);
        au8Data[u8Data++] = (uint8_t)( ScanDevices[cnt].i64Addr        & 0xff);

        au8Data[u8Data++] = ScanDevices[cnt].Rssi;
        
 //       if(DeviceTable[cnt].Baddr==243211181426525)   
 //       {
 //         TagMask = DeviceTable[cnt].data[1]&0xff; 
 //       }
        if(Protocol == 0)//i Beacon
        {
 //         for (u8Pos = 0; u8Pos < 26; u8Pos++)//was 26 places
          for (u8Pos = 4; u8Pos < 30; u8Pos++)//was 26 places
          {
            au8Data[u8Data++] = ScanDevices[cnt].data[u8Pos];    //uuid
          }
        }

        if(Protocol == 1 || Protocol == 2 || Protocol == 3 || Protocol == 4)//TLM or DS
        {
            if(Protocol == 1)//TLM ONLY
            {
              if(ScanDevices[cnt].data[11] == 0x21) 
                //if(ScanDevices[cnt].type == 0xA4)
                {
                  ScanDevices[cnt].type = 0xA4;/////K6     BATTERY
                  TagMask = ScanDevices[cnt].data[1]&0xff; 
                  TagMask += ScanDevices[cnt].data[0]<<8 & 0xff00;                 
                  kbprodata = parseSensorData(ScanDevices[cnt].data);
                  batt = kbprodata.voltage;
                }
              else if(ScanDevices[cnt].data[11] == 0x22) 
                //if(ScanDevices[cnt].type == 0xA4)
                {
                  return 0;
                }
                else
                {
                  batt = ScanDevices[cnt].data[14]&0xff; 
                  batt += ScanDevices[cnt].data[13]<<8 & 0xff00;
                  kbprodata.acc_x = 0;
                  kbprodata.acc_y = 0;
                  kbprodata.acc_z = 0;                 
                }
                batt_d = batt/20;
                pres = 239;    
            if(ScanDevices[cnt].type == 0xA4)//K6 frame   TEMPERATURE
            {    
                shTemp = kbprodata.temperature;
            } 
            else
            {
                temp1 = ScanDevices[cnt].data[15];
                temp2 = ScanDevices[cnt].data[16];
                shTemp  = MAKEWORD(temp2,temp1);
            }              
            if((uint16_t)shTemp == 0x8000)      
            {
              Temp = -45;
            }
            else
            {
              Temp = shTemp/256.;
            }
           }
           else//not TLM
           {
                pres = ScanDevices[cnt].data[6];//battery 
                Temp16 =  (int16_t)ScanDevices[cnt].data[0] | ((int16_t)(ScanDevices[cnt].data[1])<<8);
                if(Temp16 < MinTemp) Temp16 = 2200;
                else if(Temp16 > MaxTemp) Temp16 = MaxTemp;
                else
                {                  
                  DeltaT = (Temp16 - MinTemp)/10;//temperature fixing
                  ConvT = TempTable[DeltaT];
                  Temp16 = ConvT;
                  //
                }
                Temp = Temp16/100.;
            }
// original formula from white tags ?	T = (val/ 0xffff) * 175 – 45 
//eddiston reports with 2 signed bytes
//  T + 45 = (val/65535)*175
//  (T + 45)*65535/175         
             Temp += 45;
             Temp *= 374.5; 
             T_send = (uint16_t)Temp; 
             T1=(T_send>>8)&0xff;//temp
             T2=T_send&0xff;
             //end temperature conversion
            au8Data[u8Data++]=0xff;
            au8Data[u8Data++]=0x59;
//after 0x59  
            //MEMORY WLI                       on scan resp
            if((ScanDevices[cnt].data[25] == 0x05) && (ScanDevices[cnt].data[26] == 0x09) && (ScanDevices[cnt].data[27] == 'W') && (ScanDevices[cnt].length > 26))
            {
              //05 09 57 54 53 48 – Device Name “WTSH”
              au8Data[u8Data++]=0x57;       //wli memory tag       
            }
#if FUEL
            else  
            if(( ScanDevices[cnt].data[18] == 0x33) && ( ScanDevices[cnt].data[19] == 0x00) && ( ScanDevices[cnt].data[20] == 0x53))//fuel sensor
              //0601 1A-> lengh 26  FF-> prpritery  
              //4C 00 02 15 
              //5f DA 3B 11 99 9C 4D 6A 9F -> fix
              //33-> protocol version  
              //00-> hardware version
              //53-> software version 
              //00 01-> device ID 
              //03 C4 -> output level(0.1mm)
              //00 0A 00 07 32
            {
              
                   au8Data[u8Data++]=0x58;  //fuel
            }
#endif
            else
            {
              //normal TLM
              au8Data[u8Data++]=0x00;
            }
            au8Data[u8Data++]=0x02;
            //ff590002  normal
            //ff595702  memory
            //ff595802  fuel
 
            au8Data[u8Data++]=0x15;//vendor
            au8Data[u8Data++]=0xfd;
            au8Data[u8Data++]=0xa5;
            au8Data[u8Data++]=0x06;
            au8Data[u8Data++]=0x93;//uuid
            au8Data[u8Data++]=0x00;
            au8Data[u8Data++]=0x01;
            au8Data[u8Data++]=0x00;
            au8Data[u8Data++]=0x02;
            au8Data[u8Data++]=0xd8;
            au8Data[u8Data++]=pres;//batt presenteg
            //XXXX YYYY ZZZZ
//#ifdef ALEX_TLM_STRONG_CHECK
//            char* pPattern = "\x05\x09WTSH";
//            if (memcmp(DeviceTable[cnt].data+13,"\x05\x09WTSH",6)==0)
//#else
            if((ScanDevices[cnt].data[25] == 0x05) && (ScanDevices[cnt].data[26] == 0x09) && (ScanDevices[cnt].data[27] == 'W'))//memory tag?
//#endif
            {
              au8Data[u8Data++]=0x00;
              au8Data[u8Data++]=0x00;
              au8Data[u8Data++]=0x00;  
              au8Data[u8Data++]=0x00;
              au8Data[u8Data++]=0xff;
              au8Data[u8Data++]=0xff;
            }
#if FUEL
            else if(( ScanDevices[cnt].data[18] == 0x33) && ( ScanDevices[cnt].data[19] == 0x00) && ( ScanDevices[cnt].data[20] == 0x53))//fuel sensor
            {
                au8Data[u8Data++]=ScanDevices[cnt].data[23];  
                au8Data[u8Data++]=ScanDevices[cnt].data[24];
                au8Data[u8Data++]=0x00;  
                au8Data[u8Data++]=0x00;
                au8Data[u8Data++]=0x00;
                au8Data[u8Data++]=0x00;
            }
#endif
            else
            {// normal TLM
              if(kbprodata.acc_x != 0) 
              {
                Temp += 45;
              }
         au8Data[u8Data++] = (uint8_t)((kbprodata.acc_x >>  8) & 0xff);
         au8Data[u8Data++] = (uint8_t)( kbprodata.acc_x        & 0xff);     
         au8Data[u8Data++] = (uint8_t)((kbprodata.acc_y >>  8) & 0xff);
         au8Data[u8Data++] = (uint8_t)( kbprodata.acc_y        & 0xff);     
         au8Data[u8Data++] = (uint8_t)((kbprodata.acc_z >>  8) & 0xff);
         au8Data[u8Data++] = (uint8_t)( kbprodata.acc_z        & 0xff);     
#if 0
              au8Data[u8Data++]=0x00;
              au8Data[u8Data++]=0x00;
              au8Data[u8Data++]=0x00;  
              au8Data[u8Data++]=0x00;
              au8Data[u8Data++]=0x00;
              au8Data[u8Data++]=0x00;
#endif
            }
            //TTTT HHHH
#if FUEL
            if(( ScanDevices[cnt].data[18] == 0x33) && ( ScanDevices[cnt].data[19] == 0x00) && ( ScanDevices[cnt].data[20] == 0x53))//fuel sensor
            {
              au8Data[u8Data++]=0x00;//temp
              au8Data[u8Data++]=0x00;
              au8Data[u8Data++]=0x00;//humid
              au8Data[u8Data++]=0x00;
            }
            else
#endif
            {
              
              au8Data[u8Data++]=T1;//temp
              au8Data[u8Data++]=T2;
              if(ScanDevices[cnt].type == 0xA4)//K6
              {
                if(kbprodata.humidity == 0)
                {
                    if(kbprodata.sensor_mask & 0x10)
                    {
                      au8Data[u8Data++]=0XA0 | kbprodata.cutoff;//humid
                      au8Data[u8Data++]=0x00;
                    }
                      else
                    {
                      au8Data[u8Data++]=0x00;//humid
                      au8Data[u8Data++]=0x00;
                    }
                }
                else 
                {
                  Hy[1] = (uint8_t)(kbprodata.humidity & 0xFF);        // Low byte
                  Hy[0] = (uint8_t)((kbprodata.humidity >> 8) & 0xFF); // High byte
                  au8Data[u8Data++]=Hy[0];//humid
                  au8Data[u8Data++]=Hy[1];
                  //au8Data[u8Data++]=DeviceTable[cnt].data[5];//humid
                  //au8Data[u8Data++]=DeviceTable[cnt].data[6];
                }
                

              }
              else
              {
                au8Data[u8Data++]=0x00;//humid
                au8Data[u8Data++]=0x00;
              }
            }
            //battery voltage / 20
            au8Data[u8Data++]=(uint8_t)batt_d;//0x00;
            if((ScanDevices[cnt].data[25] == 0x05) && (ScanDevices[cnt].data[26] == 0x09) && (ScanDevices[cnt].data[27] == 'W'))//memory tag?
            {
              //au8Data[u8Data++]=ScanDevices[cnt].data[19];//0x00;  UUID 
             // au8Data[u8Data++]=ScanDevices[cnt].data[18];//0x00;  UUID
              au8Data[u8Data++]=ScanDevices[cnt].dataEx[10];//0x00;  UUID 
              au8Data[u8Data++]=ScanDevices[cnt].dataEx[9];//0x00;  UUID
            }
            else
            {
              au8Data[u8Data++]=0xff;           //NO UUID
              au8Data[u8Data++]=0xff;
            }
        }

        au8Data[0] = u8Data - 3 ;//2 bytes len + 00 - checksum// PacketType
      	u32Checksum = u32Alder(&au8Data[3],
				au8Data[0],
				&a,&b);
//endcountAdv = 0;
//endcountRes = 0;
	/* Add checksum to message */
	au8Data[u8Data++] = (uint8_t)( u32Checksum        & 0xff);		/* Checksum */
	au8Data[u8Data++] = (uint8_t)((u32Checksum >>  8) & 0xff);
	au8Data[u8Data++] = (uint8_t)((u32Checksum >> 16) & 0xff);
	au8Data[u8Data++] = (uint8_t)((u32Checksum >> 24) & 0xff);

	// cerate output frame
	textbuf[u8Len++] = STX;
	for (u8Pos = 0; u8Pos < u8Data; u8Pos++)
	{
		switch(au8Data[u8Pos])
		{
		case STX:
			textbuf[u8Len++] = CDPD_SLIP_ESCAPE;
			textbuf[u8Len++] = CDPD_SLIP_ESCAPED_STX;
			break;
		case ETX:
			textbuf[u8Len++] = CDPD_SLIP_ESCAPE;
			textbuf[u8Len++] = CDPD_SLIP_ESCAPED_ETX;
			break;
		case CDPD_SLIP_ESCAPE:
			textbuf[u8Len++] = CDPD_SLIP_ESCAPE;
			textbuf[u8Len++] = CDPD_SLIP_ESCAPED_ESCAPE;
			break;
		default:
			textbuf[u8Len++] = au8Data[u8Pos];
			break;

		}
	}
	textbuf[u8Len++] = ETX;
	return u8Len;    
  }
  
  

uint16_t  PrintDVRMsg(uint16_t cnt)
  {
        uint8_t		        au8Data[100];
	uint8_t 		u8Data = 0;
	uint16_t 		u8Len = 0;
	uint8_t                 u8Pos;

        float                   Temp;
        int8_t                  temp1,temp2;
        int16_t                 shTemp;
        uint16_t                TagMask;
        SensorData              kbprodata;
        uint8_t                 Hy[2];
        char                    tbuffer[20];
        char                    UUbuffer[12];
        ReadDateTimeFromRTC(&DateTime[0],&DateTime[1], &DateTime[2],
                        &DateTime[3],&DateTime[4],&DateTime[5]);
            // Format: YYYY/MM/DDThh:mm:ss

        uint64_to_ascii12_manual(UNIT_ID,UUbuffer);
        SData                   SDData;
         SDData.mac[0] = (uint8_t)((ScanDevices[cnt].i64Addr >>  40) & 0xff);
         SDData.mac[1] = (uint8_t)((ScanDevices[cnt].i64Addr >>  32) & 0xff);
         SDData.mac[2] = (uint8_t)((ScanDevices[cnt].i64Addr >>  24) & 0xff);
         SDData.mac[3] = (uint8_t)((ScanDevices[cnt].i64Addr >>  16) & 0xff);
         SDData.mac[4] = (uint8_t)((ScanDevices[cnt].i64Addr >>  8) & 0xff);
         SDData.mac[5] = (uint8_t)( ScanDevices[cnt].i64Addr        & 0xff);
        au8Data[u8Data++]='@';
        au8Data[u8Data++]='W';
        au8Data[u8Data++]='L';
        au8Data[u8Data++]='I';
        au8Data[u8Data++]=',';
        au8Data[u8Data++]='0';
        au8Data[u8Data++]='3';
        au8Data[u8Data++]=',';
        // Append 12 ASCII characters
        for (uint8_t i = 0; i < 12; i++) {
            au8Data[u8Data++] = (uint8_t)UUbuffer[i];
        }
         au8Data[u8Data++] = ',';
                snprintf(tbuffer, 20, "%04d/%02d/%02dT%02d:%02d:%02d",
             DateTime[0], DateTime[1],DateTime[2],
             DateTime[3], DateTime[4],DateTime[5]);
                for (uint8_t i = 0; i < 19; i++) {
            au8Data[u8Data++] = (uint8_t)tbuffer[i];
        }
         au8Data[u8Data++] = ',';
         
        if(Protocol == 0)//i Beacon
        {
          for (u8Pos = 4; u8Pos < 30; u8Pos++)//was 26 places
          {
            au8Data[u8Data++] = ScanDevices[cnt].data[u8Pos];    //uuid
          }
        }
        if(Protocol == 1 || Protocol == 2 || Protocol == 3 || Protocol == 4)//TLM or DS
        {
            if(Protocol == 1)//TLM ONLY
            {
              if(ScanDevices[cnt].data[11] == 0x21) 
                {
                  ScanDevices[cnt].type = 0xA4;/////K6     BATTERY
                  TagMask = ScanDevices[cnt].data[1]&0xff; 
                  TagMask += ScanDevices[cnt].data[0]<<8 & 0xff00;                 
                  kbprodata = parseSensorData(ScanDevices[cnt].data);
 
                  
                  if (kbprodata.voltage != 0)
                  {
                    SDData.batt = kbprodata.voltage;
                  }
                  else 
                  {
                    SDData.batt = 0;
                  }
                  if (kbprodata.temperature != 0)
                  {
                    SDData.temperature = (float)kbprodata.temperature/256;
                  }
                  else 
                  {
                    kbprodata.temperature = 0;
                  }
                  if (kbprodata.humidity != 0)
                  {
                    SDData.humidity = (float)kbprodata.humidity/256;
                  }
                  else 
                  {
                    SDData.humidity = 0;
                  }
                  if (kbprodata.acc_x != 0)
                  {
                    SDData.acc_x = kbprodata.acc_x;
                  }
                  else 
                  {
                    SDData.acc_x = 0;
                  }
                    if (kbprodata.acc_y != 0)
                  {
                    SDData.acc_y = kbprodata.acc_y;
                  }
                  else 
                  {
                    SDData.acc_y = 0;
                  }
                  if (kbprodata.acc_z != 0)
                  {
                    SDData.acc_z = kbprodata.acc_z;
                  }
                  else 
                  {
                    SDData.acc_z = 0;
                  }
                  if (kbprodata.cutoff != 0)
                  {
                    SDData.ios = kbprodata.cutoff;
                  }
                  else 
                  {
                    SDData.ios = 0;
                  }
                }
              else if(ScanDevices[cnt].data[11] == 0x22) 
                {

             
                }
                else
                {
                    SDData.batt = (ScanDevices[cnt].data[14]&0xff) + (ScanDevices[cnt].data[13]<<8 & 0xff00);
                    SDData.temperature =  (float)((ScanDevices[cnt].data[16]&0xff) + (ScanDevices[cnt].data[15]<<8 & 0xff00))/256;
                    SDData.humidity = 0;
                    SDData.ios = 0;
                    SDData.acc_x = 0;
                    SDData.acc_y = 0;
                    SDData.acc_z = 0;
                    
                } 
            
    
//           prepareAsciiString(&SDData,cnt,(char*)au8Data,10);
           u8Len = prepareAsciiString(&SDData,4,(char*)au8Data+u8Data,10);
           u8Len = u8Len + u8Data;
//           au8Data[u8Len++] = ',';
           memset(tbuffer,0,10);
            temp1 = ScanDevices[cnt].Rssi;
            sprintf(tbuffer, "%d", temp1);
              for (uint8_t i = 0; i < 4 && tbuffer[i]!= 0; i++) {
            au8Data[u8Len++] = (uint8_t)tbuffer[i];
        }

           au8Data[u8Len++] = '#';
           }  
#if 0
            if((ScanDevices[cnt].data[25] == 0x05) && (ScanDevices[cnt].data[26] == 0x09) && (ScanDevices[cnt].data[27] == 'W') && (ScanDevices[cnt].length > 26))
            {
              //05 09 57 54 53 48 – Device Name “WTSH”
              au8Data[u8Data++]=0x57;       //wli memory tag       
            }
            else
            {
              //normal TLM
              
            }
            if((ScanDevices[cnt].data[25] == 0x05) && (ScanDevices[cnt].data[26] == 0x09) && (ScanDevices[cnt].data[27] == 'W'))//memory tag?
            {
 
            }

            else
            {// normal TLM
     

            }
 
            if((ScanDevices[cnt].data[25] == 0x05) && (ScanDevices[cnt].data[26] == 0x09) && (ScanDevices[cnt].data[27] == 'W'))//memory tag?
            {
           
            }
            else
            {

            }
            #endif
        }

        memcpy(textbuf,au8Data,u8Len);
  //      au8Data[0] = u8Data - 3 ;//2 bytes len + 00 - checksum// PacketType
#if 0        
	for (u8Pos = 0; u8Pos < u8Data; u8Pos++)
	{

		switch(au8Data[u8Pos])
		{
		case STX:
			textbuf[u8Len++] = CDPD_SLIP_ESCAPE;
			textbuf[u8Len++] = CDPD_SLIP_ESCAPED_STX;
			break;
		case ETX:
			textbuf[u8Len++] = CDPD_SLIP_ESCAPE;
			textbuf[u8Len++] = CDPD_SLIP_ESCAPED_ETX;
			break;
		case CDPD_SLIP_ESCAPE:
			textbuf[u8Len++] = CDPD_SLIP_ESCAPE;
			textbuf[u8Len++] = CDPD_SLIP_ESCAPED_ESCAPE;
			break;
		default:

			textbuf[u8Len++] = au8Data[u8Pos];
			break;

		}
	}
	textbuf[u8Len++] = ETX;
#endif
	return u8Len;    


  }



uint16_t EncodeTrace(uint8_t* buf,uint8_t lengh)
{
        uint8_t		     au8Data[40];
	uint8_t 		 u8Data = 0;
	uint16_t 		 u8Len = 0;
	uint8_t            u8Pos;
	uint32_t 		u32Checksum;
	uint32_t 		   a = 0;
	uint32_t 		   b = 0;
        uint8_t                         i;

	/* Format for output (legacy format) */
	au8Data[u8Data++] = 0x02; 							// Length, low byte size of packet PING_Report
	au8Data[u8Data++] = 0x00; 							// Length, high byte size of packet PING_Report
	au8Data[u8Data++] = 0x00;     							// Output sequence
	// Start to fill PING_Report structure
	au8Data[u8Data++] = PACKET_TYPE_TRACE; 					// PacketType, 0xA3
        for(i = 0;i < lengh;i++)
        {
 	au8Data[u8Data++] = buf[i];         
        }
	au8Data[u8Data - i - i] = i;	
        au8Data[0] = u8Data - 3 ;//2 bytes len + 00 - checksum// PacketType
        
	u32Checksum = u32Alder(&au8Data[3],
				au8Data[0],
				&a,&b);

	/* Add checksum to message */
	au8Data[u8Data++] = (uint8_t)( u32Checksum        & 0xff);		/* Checksum */
	au8Data[u8Data++] = (uint8_t)((u32Checksum >>  8) & 0xff);
	au8Data[u8Data++] = (uint8_t)((u32Checksum >> 16) & 0xff);
	au8Data[u8Data++] = (uint8_t)((u32Checksum >> 24) & 0xff);
	// Create output frame
	textbuf[u8Len++] = STX;
	for (u8Pos = 0; u8Pos < u8Data; u8Pos++)
	{
		switch(au8Data[u8Pos])
		{
		case STX:
			textbuf[u8Len++] = CDPD_SLIP_ESCAPE;
			textbuf[u8Len++] = CDPD_SLIP_ESCAPED_STX;
			break;
		case ETX:
			textbuf[u8Len++] = CDPD_SLIP_ESCAPE;
			textbuf[u8Len++] = CDPD_SLIP_ESCAPED_ETX;
			break;
		case CDPD_SLIP_ESCAPE:
			textbuf[u8Len++] = CDPD_SLIP_ESCAPE;
			textbuf[u8Len++] = CDPD_SLIP_ESCAPED_ESCAPE;
			break;
		default:
			textbuf[u8Len++] = au8Data[u8Pos];
			break;

		}
	}
	textbuf[u8Len++] = ETX;
	return u8Len;
}

uint16_t EncodeConnectStatus(CON_STATUS Status)
{
        uint8_t		     au8Data[40];
	uint8_t 		 u8Data = 0;
	uint16_t 		 u8Len = 0;
	uint8_t            u8Pos;
	uint32_t 		u32Checksum;
	uint32_t 		   a = 0;
	uint32_t 		   b = 0;
        uint8_t t = 0;
       memset(au8Data,'\0',sizeof(au8Data));
	/* Format for output (legacy format) */
	au8Data[u8Data++] = 0x02; 							// Length, low byte size of packet PING_Report
	au8Data[u8Data++] = 0x00; 							// Length, high byte size of packet PING_Report
	au8Data[u8Data++] = 0x00;     							// Output sequence
	// Start to fill PING_Report structure
	au8Data[u8Data++] = PACKET_TYPE_BLE_CONNECTION_STATUS; 					// PacketType, 0xA3
	au8Data[u8Data++] = Status; 	// PacketType
       // au8Data[u8Data++] = CError; 
        if( Status == ERROR_P)
        {
          au8Data[u8Data++] = S_Error;
        }
        else if(( Status == DISCONNECTED__S) || ( Status == HOST_RESETART) || ( Status == ATH_ERROR) ||  ( Status == WD) || ( Status == BLE_CHAT_ERR) || ( Status == BLE_STUCK_ERR))
        {
          au8Data[u8Data++] = 0;
        }
        else
        {
 //         au8Data[u8Data++] = St_Error;
          au8Data[u8Data++] = TempMac[5];
          au8Data[u8Data++] = TempMac[4];
          au8Data[u8Data++] = TempMac[3];
          au8Data[u8Data++] = TempMac[2];
          au8Data[u8Data++] = TempMac[1];
          au8Data[u8Data++] = TempMac[0];       
        }
        	au8Data[u8Data - t ] = t;
	 au8Data[0] = u8Data - 3 ;//2 bytes len + 00 - checksum// PacketType
	u32Checksum = u32Alder(&au8Data[3],
				au8Data[0],
				&a,&b);

	/* Add checksum to message */
	au8Data[u8Data++] = (uint8_t)( u32Checksum        & 0xff);		/* Checksum */
	au8Data[u8Data++] = (uint8_t)((u32Checksum >>  8) & 0xff);
	au8Data[u8Data++] = (uint8_t)((u32Checksum >> 16) & 0xff);
	au8Data[u8Data++] = (uint8_t)((u32Checksum >> 24) & 0xff);
	// Create output frame
	textbuf[u8Len++] = STX;
	for (u8Pos = 0; u8Pos < u8Data; u8Pos++)
	{
		switch(au8Data[u8Pos])
		{
		case STX:
			textbuf[u8Len++] = CDPD_SLIP_ESCAPE;
			textbuf[u8Len++] = CDPD_SLIP_ESCAPED_STX;
			break;
		case ETX:
			textbuf[u8Len++] = CDPD_SLIP_ESCAPE;
			textbuf[u8Len++] = CDPD_SLIP_ESCAPED_ETX;
			break;
		case CDPD_SLIP_ESCAPE:
			textbuf[u8Len++] = CDPD_SLIP_ESCAPE;
			textbuf[u8Len++] = CDPD_SLIP_ESCAPED_ESCAPE;
			break;
		default:
			textbuf[u8Len++] = au8Data[u8Pos];
			break;

		}
	}
	textbuf[u8Len++] = ETX;

	return u8Len;
	//return u8Len;
}

uint16_t EncodeCommandStatus(STATUSES1_E Status)
{
        uint8_t		     au8Data[40];
	uint8_t 		 u8Data = 0;
	uint16_t 		 u8Len = 0;
	uint8_t            u8Pos;
	uint32_t 		u32Checksum;
	uint32_t 		   a = 0;
	uint32_t 		   b = 0;

	/* Format for output (legacy format) */
	au8Data[u8Data++] = 0x02; 							// Length, low byte size of packet PING_Report
	au8Data[u8Data++] = 0x00; 							// Length, high byte size of packet PING_Report
	au8Data[u8Data++] = 0x00;     							// Output sequence
	// Start to fill PING_Report structure
	au8Data[u8Data++] = PACKET_TYPE_COMMAND_STATUS; 					// PacketType, 0xA3
	au8Data[u8Data++] = Status; 										// PacketType
	u32Checksum = u32Alder(&au8Data[3],
				au8Data[0],
				&a,&b);

	/* Add checksum to message */
	au8Data[u8Data++] = (uint8_t)( u32Checksum        & 0xff);		/* Checksum */
	au8Data[u8Data++] = (uint8_t)((u32Checksum >>  8) & 0xff);
	au8Data[u8Data++] = (uint8_t)((u32Checksum >> 16) & 0xff);
	au8Data[u8Data++] = (uint8_t)((u32Checksum >> 24) & 0xff);
	// Create output frame
	textbuf[u8Len++] = STX;
	for (u8Pos = 0; u8Pos < u8Data; u8Pos++)
	{
		switch(au8Data[u8Pos])
		{
		case STX:
			textbuf[u8Len++] = CDPD_SLIP_ESCAPE;
			textbuf[u8Len++] = CDPD_SLIP_ESCAPED_STX;
			break;
		case ETX:
			textbuf[u8Len++] = CDPD_SLIP_ESCAPE;
			textbuf[u8Len++] = CDPD_SLIP_ESCAPED_ETX;
			break;
		case CDPD_SLIP_ESCAPE:
			textbuf[u8Len++] = CDPD_SLIP_ESCAPE;
			textbuf[u8Len++] = CDPD_SLIP_ESCAPED_ESCAPE;
			break;
		default:
			textbuf[u8Len++] = au8Data[u8Pos];
			break;

		}
	}
	textbuf[u8Len++] = ETX;
	return u8Len;
}
  
  uint16_t  EncodeUUId(uint32_t UUid,uint16_t Ver)
  {
        uint8_t		        au8Data[40];
	uint8_t                 u8Data = 0;
	uint16_t 		u8Len = 0;
	uint8_t                 u8Pos;
	uint32_t 		u32Checksum;
	uint32_t 		a = 0;
	uint32_t 		b = 0;
        uint8_t                 t = 0;
	au8Data[u8Data++] = 0x07; 							// Length, low byte size of packet PING_Report
	au8Data[u8Data++] = 0x00; 							// Length, high byte size of packet PING_Report
	au8Data[u8Data++] = 0x00;     							// Output sequence
	// Start to fill PING_Report structure
	au8Data[u8Data++] = PACKET_TYPE_APP_ID; // PacketType 0xA2

        au8Data[u8Data++] = (uint8_t)((UUid >> 24) & 0xff);
	au8Data[u8Data++] = (uint8_t)((UUid >> 16) & 0xff);
	au8Data[u8Data++] = (uint8_t)((UUid >>  8) & 0xff);
	au8Data[u8Data++] = (uint8_t)( UUid        & 0xff);
        au8Data[u8Data++] = (uint8_t)((Ver >> 8)   & 0xff);
	au8Data[u8Data++] = (uint8_t)( Ver         & 0xff);
  	au8Data[u8Data++] = (uint8_t)(FLAGS_T);
	au8Data[u8Data++] = (uint8_t)(P_FLAGS);      
  	au8Data[u8Data++] = (uint8_t)(Protocol); 
 // 	au8Data[u8Data++] = (uint8_t)(Mode);
//device name length
 //       au8Data[u8Data++] = 0;//ready 
        for(t = 0;(t < 10) && (DevName[10+t] != 0) && (DevName[10+t] != 0xff);t++)
        {
          au8Data[u8Data++] = DevName[10+t];
        }
//	au8Data[u8Data - t - 1] = t;	
        au8Data[0] = u8Data - 3 ;//2 bytes len + 00 - checksum// PacketType
	u32Checksum = u32Alder(&au8Data[3],
				au8Data[0],
				&a,&b);

	/* Add checksum to message */
	au8Data[u8Data++] = (uint8_t)( u32Checksum        & 0xff);		/* Checksum */
	au8Data[u8Data++] = (uint8_t)((u32Checksum >>  8) & 0xff);
	au8Data[u8Data++] = (uint8_t)((u32Checksum >> 16) & 0xff);
	au8Data[u8Data++] = (uint8_t)((u32Checksum >> 24) & 0xff);
        
	// cerate output frame
	textbuf[u8Len++] = STX;
	for (u8Pos = 0; u8Pos < u8Data; u8Pos++)
	{
		switch(au8Data[u8Pos])
		{
		case STX:
			textbuf[u8Len++] = CDPD_SLIP_ESCAPE;
			textbuf[u8Len++] = CDPD_SLIP_ESCAPED_STX;
			break;
		case ETX:
			textbuf[u8Len++] = CDPD_SLIP_ESCAPE;
			textbuf[u8Len++] = CDPD_SLIP_ESCAPED_ETX;
			break;
		case CDPD_SLIP_ESCAPE:
			textbuf[u8Len++] = CDPD_SLIP_ESCAPE;
			textbuf[u8Len++] = CDPD_SLIP_ESCAPED_ESCAPE;
			break;
		default:
			textbuf[u8Len++] = au8Data[u8Pos];
			break;

		}
	}
	textbuf[u8Len++] = ETX;
	return u8Len;    
  }
  
uint32_t u32Alder( uint8_t *pu8Data, uint16_t u32Len,uint32_t *a ,uint32_t *b)
{
    #define MOD_ADLER 65521

        *a = 1;
        *b = 0;
    
    while (u32Len)
    {
        uint16_t tlen = u32Len > 5550 ? 5550 : u32Len;
        u32Len -= tlen;
        do
        {
            *a += *pu8Data++;
            *b += *a;
        } while (--tlen);

        *a %= MOD_ADLER;
        *b %= MOD_ADLER;
    }

    return (*b << 16) | *a;
}

SensorData parseSensorData(const uint8_t *rawData) {
    SensorData data;
    uint8_t i = 14;
    if(rawData[12] == 0x01)//old format
    {
      data.sensor_mask = rawData[13] & 0x00ff;
    }
    else
    {
      data.sensor_mask = (rawData[12] << 8) | rawData[13];
    }
    if(data.sensor_mask & 0x01)
    {
      data.voltage = (rawData[i] << 8) | rawData[i+1];
      i = i+2;
    }else {
        data.voltage = 0; // Or any other default value
    }
    if (data.sensor_mask & 0x02) {
        // Temperature is present
        data.temperature = (rawData[i] << 8) | rawData[i+1];
        i = i+2;
    } else {
        data.temperature = 0; // Or any other default value
    }
    
    if (data.sensor_mask & 0x04) {
        // Humidity is present
        data.humidity = (rawData[i] << 8) | rawData[i+1];
        i = i+2;
    } else {
        data.humidity = 0; // Or any other default value
    }
    
    if (data.sensor_mask & 0x08) {
        // Accelerometer data is present
        data.acc_x = (rawData[i] << 8) | rawData[i+1];
        i = i+2;
        data.acc_y = (rawData[i] << 8) | rawData[i+1];
        i = i+2;
        data.acc_z = (rawData[i] << 8) | rawData[i+1];
         i = i+2;
    } else {
        data.acc_x = 0; // Or any other default value
        data.acc_y = 0; // Or any other default value
        data.acc_z = 0; // Or any other default value
    }
    
    if (data.sensor_mask & 0x10) {
        // cutoff is present
        data.cutoff =  0x80 | rawData[i];
        i = i+1;
    } else {
        data.cutoff = 0; // Or any other default value
    }
    
    // Parse other fields similarly
    
    return data;
}


uint16_t  PrintMemMsg(uint16_t cnt)
  {
        uint8_t		        au8Data[45];
	uint8_t 		u8Data = 0;
	uint16_t 		u8Len = 0;
	uint8_t                 u8Pos;
	uint32_t 		u32Checksum;
	uint32_t 		a = 0;
	uint32_t 		b = 0;
        uint16_t                batt_d;
        uint16_t                T_send;
        float                   Temp;
        uint8_t                 T1,T2;
        uint16_t                batt;
        uint16_t                pres;
        int8_t                  temp1,temp2;
        int16_t shTemp;
        

        
        au8Data[u8Data++] = 34;//lenght low
        au8Data[u8Data++] = 0x00;//lenght high
        au8Data[u8Data++] = cnt;//seq  
        au8Data[u8Data++] = PACKET_TYPE_BEACON;
   
        au8Data[u8Data++] = TempMac[5];
	au8Data[u8Data++] = TempMac[4];
	au8Data[u8Data++] = TempMac[3];
	au8Data[u8Data++] = TempMac[2];
        au8Data[u8Data++] = TempMac[1];
	au8Data[u8Data++] = TempMac[0];

        au8Data[u8Data++] = 0;//DeviceTable[cnt].Rssi;
//                batt = Att_ReadVal[2];
//                batt = ((uint16_t)Att_ReadVal[1]) << 8) | Att_ReadVal[2];
        batt = MAKEWORD(Att_ReadVal[2],Att_ReadVal[1]);
                batt_d = batt/20;
                pres = 239;
                
                temp1 = Att_ReadVal[3];
                temp2 = Att_ReadVal[4];           
                shTemp  = MAKEWORD(temp2,temp1);
                Temp = shTemp/256.;
                  Temp += 45;
                  Temp *= 374.5;
                  T_send = (uint16_t)Temp;
                  T1=(T_send>>8)&0xff;//temp
                  T2=T_send&0xff;
                        
            au8Data[u8Data++]=0xff;
            au8Data[u8Data++]=0x59;
            au8Data[u8Data++]=0x57;              
            au8Data[u8Data++]=0x02;
            au8Data[u8Data++]=0x15;//vendor
            au8Data[u8Data++]=0xfd;
            au8Data[u8Data++]=0xa5;
            au8Data[u8Data++]=0x06;
            au8Data[u8Data++]=0x93;//uuid
            au8Data[u8Data++]=0x00;
            au8Data[u8Data++]=0x01;
            au8Data[u8Data++]=0x00;
            au8Data[u8Data++]=0x02;
            au8Data[u8Data++]=0xd8;
            au8Data[u8Data++] = pres;//batt
            au8Data[u8Data++] = Att_ReadVal[5];
            au8Data[u8Data++] = Att_ReadVal[6];
            au8Data[u8Data++] = Att_ReadVal[7];
            au8Data[u8Data++] = Att_ReadVal[8];
            au8Data[u8Data++] = Att_ReadVal[9];
            au8Data[u8Data++] = Att_ReadVal[10];
            au8Data[u8Data++] = T1;//temp
            au8Data[u8Data++] = T2;
            au8Data[u8Data++]=0x00;//humid
            au8Data[u8Data++]=0x00;
            au8Data[u8Data++]=(uint8_t)(batt/20 & 0xff);//0x00;

        
      	u32Checksum = u32Alder(&au8Data[3],
				au8Data[0],
				&a,&b);

	/* Add checksum to message */
	au8Data[u8Data++] = (uint8_t)( u32Checksum        & 0xff);		/* Checksum */
	au8Data[u8Data++] = (uint8_t)((u32Checksum >>  8) & 0xff);
	au8Data[u8Data++] = (uint8_t)((u32Checksum >> 16) & 0xff);
	au8Data[u8Data++] = (uint8_t)((u32Checksum >> 24) & 0xff);

	// cerate output frame
	textbuf[u8Len++] = STX;
	for (u8Pos = 0; u8Pos < u8Data; u8Pos++)
	{
		switch(au8Data[u8Pos])
		{
		case STX:
			textbuf[u8Len++] = CDPD_SLIP_ESCAPE;
			textbuf[u8Len++] = CDPD_SLIP_ESCAPED_STX;
			break;
		case ETX:
			textbuf[u8Len++] = CDPD_SLIP_ESCAPE;
			textbuf[u8Len++] = CDPD_SLIP_ESCAPED_ETX;
			break;
		case CDPD_SLIP_ESCAPE:
			textbuf[u8Len++] = CDPD_SLIP_ESCAPE;
			textbuf[u8Len++] = CDPD_SLIP_ESCAPED_ESCAPE;
			break;
		default:
			textbuf[u8Len++] = au8Data[u8Pos];
			break;

		}
	}
	textbuf[u8Len++] = ETX;
	return u8Len;    
  }


#ifdef TRACE_VERSION
#include <stdarg.h>
char DebugBuffer[256];
BOOL trace_flag = TRUE;
#define SOB  0x5E
#define EOB  0x0D

char szSOB[2] = { SOB,0};
char szEOB[2] = { EOB,0};
unsigned char WriteSerial(unsigned char uchLine,char* message, short Len);

void Trace_Error_New(char* pBuffer,char* format,...)
{
   if (trace_flag == TRUE)
   {
   		char szTick[20];
		int TickLen;

		short LengthOfString;
		va_list arglist;
		va_start(arglist, format);
		vsprintf(pBuffer,format,arglist);
		va_end(arglist);
		TickLen = sprintf(szTick,"%05lu: ",HAL_GetTick());

		LengthOfString=strlen(pBuffer);
		if (LengthOfString > (sizeof(DebugBuffer)-1))
			LengthOfString = sizeof(DebugBuffer)-1;
		if (pBuffer != DebugBuffer)
		{
			strncpy(DebugBuffer,pBuffer,LengthOfString);
		}
		DebugBuffer[sizeof(DebugBuffer)-1] = '\0';
        WriteSerial(0,szSOB,1);
		WriteSerial(0,szTick,(short)TickLen);
        WriteSerial(0,DebugBuffer,(short)LengthOfString);
		WriteSerial(0,szEOB,1);
   }
}

char* PresentBinDataAsTextHexComplex(char* Buf,unsigned char* Bin,int Len,char Delimiter,BOOL bHighFirst)
{
	char* pOut = Buf;
	int i;
        Buf[0] = '\0';
	if (bHighFirst)
	{
		for (i = Len; i > 0; i--)
		{
			pOut += sprintf(pOut,"%02X",Bin[i-1]);
			if (Delimiter && (i != 1))
				*pOut++ = Delimiter;
		}
		*pOut = '\0';
	}
	else
	{
		for (i = 0; i < Len; i++)
		{
			pOut += sprintf(pOut,"%02X",Bin[i]);
			if (Delimiter && (i < (Len -1)))
				*pOut++ = Delimiter;

		}
		*pOut = '\0';
	}
	return Buf;
}




#endif // TRACE_VERSION

uint8_t prepareAsciiString(SData *data, uint8_t count, char *output, size_t maxLen) {
    char buffer[64];  // temporary per device
    output[0] = '\0'; // clear output
    uint8_t i = 0;
    uint8_t c = 0;
  //  for (uint8_t i = 0; i < count; i++) {
        char tempStr[16] = "";
        char humStr[16] = "";
        char battStr[16] = "";
        char ioStr[16] = "";
        char AxStr[16] = "";
        char AyStr[16] = "";
        char AzStr[16] = "";
  
        // Format MAC
        snprintf(buffer, sizeof(buffer), "%02X%02X%02X%02X%02X%02X,",
                 data[i].mac[0], data[i].mac[1], data[i].mac[2],
                 data[i].mac[3], data[i].mac[4], data[i].mac[5]);

        // Temperature
        if (data[i].temperature != 0.0f) {
            snprintf(tempStr, sizeof(tempStr), "%.1f", data[i].temperature);
        }

        // Humidity
        if (data[i].humidity != 0.0f) {
            snprintf(humStr, sizeof(humStr), "%.1f", data[i].humidity);
        }
        // battery
        if (data[i].batt != 0) {
            snprintf(battStr, sizeof(humStr), "%d", data[i].batt);
        }
        // ios
        if (data[i].ios != 0) {
            snprintf(ioStr, sizeof(humStr), "%u", data[i].ios &~ 0x80);
        }
        // ACCx
        if((data[i].acc_x != 0) || (data[i].acc_y != 0) || (data[i].acc_z != 0))
        {
          snprintf(AxStr, sizeof(humStr), "%d", data[i].acc_x);
          snprintf(AyStr, sizeof(humStr), "%d", data[i].acc_y);
          snprintf(AzStr, sizeof(humStr), "%d", data[i].acc_z);
        }


        // Build full line: MAC|temp|hum|ios
        c = 12 + strlen(battStr) + 1;
        strncat(buffer, battStr, sizeof(buffer) - strlen(buffer) - 1);
        strncat(buffer, ",", sizeof(buffer) - strlen(buffer) - 1);
        c += strlen(tempStr) + 1;
        strncat(buffer, tempStr, sizeof(buffer) - strlen(buffer) - 1);
        strncat(buffer, ",", sizeof(buffer) - strlen(buffer) - 1);
        c += strlen(humStr) + 1;
        strncat(buffer, humStr, sizeof(buffer) - strlen(buffer) - 1);
        strncat(buffer, ",", sizeof(buffer) - strlen(buffer) - 1);
        c += strlen(ioStr) + 1;
        strncat(buffer, ioStr, sizeof(buffer) - strlen(buffer) - 1);  
        strncat(buffer, ",", sizeof(buffer) - strlen(buffer) - 1);
        if((data[i].acc_x != 0) || (data[i].acc_y != 0) || (data[i].acc_z != 0))
        {
          c += strlen(AxStr) + 1;
          strncat(buffer, AxStr, sizeof(buffer) - strlen(buffer) - 1);
          strncat(buffer, "|", sizeof(buffer) - strlen(buffer) - 1);
          c += strlen(AyStr) + 1;
          strncat(buffer, AyStr, sizeof(buffer) - strlen(buffer) - 1);   
          strncat(buffer, "|", sizeof(buffer) - strlen(buffer) - 1);
          c += strlen(AzStr) + 2;
          strncat(buffer, AzStr, sizeof(buffer) - strlen(buffer) - 1);
        }
        else
        {
          c += 2;
        }
        strncat(buffer, ",", sizeof(buffer) - strlen(buffer) - 1);

        strncat(output, buffer, strlen(buffer));

        // Add comma if not last
 //       if (i < count - 1) {
 //           strncat(output, ",", maxLen - strlen(output) - 1);
  //      }
   // }
        return c;
}

void uint64_to_ascii12_manual(uint64_t value, char *output)
{
    for (int i = 11; i >= 0; i--) {
        output[i] = (value % 10) + '0';
        value /= 10;
    }
    output[12] = '\0';
}