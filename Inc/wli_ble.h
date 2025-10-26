#ifndef WLI_BLE
#define WLI_BLE




#include "compiler.h"
#include <stdio.h>
#include <string.h>
#include "gp_timer.h"
//#include "ble_const.h" 
//#include "bluefunc.h"


extern volatile int wli_app_flags;
#define WAPP_FLAG(flag) (wli_app_flags & flag)

#define WAPP_FLAG_SET(flag) (wli_app_flags |= flag)
#define WAPP_FLAG_CLEAR(flag) (wli_app_flags &= ~flag)

#define Tick_flag       0x00001
#define CR_flag         0x00002
#define ScanEnd         0x00004
#define BinStart        0x00008
#define wait_flag       0x00010
#define mSec_100        0x00020
#define Sec_10          0x00040
#define ScanStoped      0x00080
#define TX_flag         0x00100
#define  BinMsg         0x00200
#define ADV_ON          0x00400
#define SLEEP_EN        0x00800
#define GP_IRQ          0x01000
#define UPDATE_ADV      0x02000
#define WasConnected    0x04000
#define ConFlag         0x08000
#define SLEEP_MODE      0x10000
#define STATE_UPDATE    0x20000
#define DELAYF          0x40000
#define WAIT_SCAN_ADV   0x80000
#define WCONNECTED      0x100000
#define PULSED          0x200000
#define MASTER_C        0x400000
#define WRITE_CHAR      0x800000
#define END_RX          0x1000000
#define ATT_DATA        0x2000000
#define DeviceChoos     0x4000000
#define LedFlag         0x8000000
//connapp flags


typedef enum {
  BtInit = 0,
  Searching,
  Scan,  
  Scan2,
  Scan_end,
  Scan_wait,
  Wait_Bot,
  Connecting,//5
  Connectingw,
  Connected,
  Idle,
  SleepT,
  SendSer,      //a
  SendEnd,      //b
  SendDelay,    //c
  Done_w,       //d
  Done,         //e
  StopScan,     //f
  StopScanw,
  LoopDelay,
  WaitScan
}TestStat;

  
  typedef enum{
KillScan= 0,
StartCon ,
SwitchCentral,
WaitForCon,
DeviceConnected,
MainServiceDiscovery,
CharAttDiscoverySearch,
CharAttDiscovery,
WriteChar,
ReadMem,
DataRead,
SendEndRXChar,  //a
SendRXChar,     //b
DisconnectRX,   //c
ConDelay,       //d
ConError,       //e
DisconnectedEnd,//f
FailEnd,
DiscError,
NoService,
IdleC
  }ConnStat;
  
  



	typedef enum
	{
		STATUS1_OK = '0',
		STATUS1_ERROR = '3',
	} STATUSES1_E;
      
typedef enum
{
    CONNECTED_S = 0x00, //–Connected event (when connected, and as answer on get status command ats?)	
    NOT_CONNECTABLE = 0x01, //– Cannot connect to tag – not appears or not connectable
    NOT_WLI = 0x02, // No WLI TLM History tag
    ERROR_P = 0x09, //Any error in not final state od during the reading
    END_OF_DATA = 0x10, //– End of data
    DISCONNECTED__S = 0x04, //– disconnected (periodic scan tags mode) – answer on get status (ats?)
    HOST_RESETART = 0x05, //– Started (sent automatically when the Host is restarted from any reason (via pin or itself)
    ATH_OK =0x06,
    ATH_ERROR = 0x07,
    ERROR_S = 0x08,
    BLE_STUCK_ERR = 0x09,
    BLE_CHAT_ERR = 0x0A,
    WD = 0x0B
} CON_STATUS;
        
typedef enum
{
    NOT_DEFINED = 0x00, //–Connected event 
    SERVICE_UUID_ERROR = 0x01,
    ERROR_DISC_CHAR = 0x02,
    ERROR_WRITE_CHAR = 0x03,
    ERROR_READ_CHAR = 0x04,
    STACK_ERROR = 0x05
} ERROR_STATUS;      






        typedef struct process{
  uint16_t on_going;
  uint16_t next_status;
}processt;
        

    

                
#define DISCOVERY_TIMEOUT 3000 /* at least 3 seconds */

#define DISCOVERY_PROC_SCAN_INT 0x4000 
#define DISCOVERY_PROC_SCAN_WIN 0x4000
#define ADV_INT_MIN 0X1F40//0x90
#define ADV_INT_MAX 0X1F40//0x90
        


#define PACKET_TYPE_BEACON              0xb1
#define PACKET_TYPE_APP_ID	        0xb2
#define  PACKET_TYPE_COMMAND_STATUS	0xb3
#define PACKET_TYPE_BLE_CONNECTION_STATUS 	        0xb4
#define PACKET_TYPE_VAL_STATUS	        0xb5    
#define PACKET_TYPE_ERROR_STATUS	        0xb6  
#define PACKET_TYPE_TRACE               0xb7
        
#define SCAN_TABLE_SIZE 100//150//250
#define DEVICE_TABLE_SIZE 70//65
#define RBUF_SIZE 45


#define JBUS_STATUS     0x0
#define ODOMETER        0x1
#define BT_ADC          0x2
#define J_CONFIG        0x3
#define DEVELOPER       0x04

#define BLE_ADV_LEN_MAX 31
typedef struct _TestDevices{
  int64_t i64Addr; // instead bdaddr[6]
  uint8_t length;
  uint8_t data[BLE_ADV_LEN_MAX];
  uint8_t lengthEx; // for scan response
  uint8_t dataEx[BLE_ADV_LEN_MAX]; // for scan response
  uint8_t Rssi;
  uint8_t type;
}TestDevices;


//typedef PACKED(struct) _TestScanResp{
typedef __packed struct _TestScanResp{
  uint8_t bdaddr[6];
  uint8_t length;
  uint8_t data[10];
}TestScanResp;


  typedef struct ProtocolS{

  uint8_t UuidOffset;
  uint8_t CopyOffset;
  uint8_t copySize;
  uint8_t UUidFilter[4];
}ProtocolS;


typedef __packed struct _ReadStruct{
  
  uint8_t MsgType ;
  uint8_t bdaddr[6];
  uint8_t length;
  uint8_t data[44];
  uint8_t Rssi;
} ReadStruct;


typedef __packed struct _DevicesSort{
  uint8_t data[26];
  //uint8_t data1[4];
  //uint8_t data2[2];
  //uint8_t data3[2];
  //uint8_t data4[2];
  uint8_t Rssi;
  uint8_t type;
  uint64_t Baddr;
}  DevicesSort;

#endif

typedef union UUID_t_s {
  /** 16-bit UUID
   */
  uint16_t UUID_16;
  /** 128-bit UUID
   */
  uint8_t UUID_128[16];
} UUID_t;


#ifdef TRACE_VERSION
typedef struct
{
	int Status; // BLE_STATUS_SUCCESS
	// Local version HCI info (refer Bleetooth core specification, Vol.2, PArt E, Chapter 7.4.1)
  uint8_t hci_version;	
  uint16_t hci_revision;
  uint8_t lmp_pal_version;
  uint16_t manufacturer_name;
  uint16_t lmp_pal_subversion;
	// - refer D:\AckPlatforms\CortexPlat\WliDongle\Middlewares\ST\STM32_BlueNRG\SimpleBlueNRG_HCI\hci\controller\bluenrg_utils.c
  uint8_t hwVersion;
  uint16_t fwVersion;
	char szVersion[24];
  } BLUENGR_VERSION_INFO;





extern BLUENGR_VERSION_INFO gBlueNRGVersion;
#define BLUNRG_HW_VERSION_HIGH(hwVersion) ((hwVersion >> 4) & 0xff)
#define BLUNRG_HW_VERSION_LOW_CHAR(hwVersion) (hwVersion & 0xff)
#define BLUNRG_FW_VERSION_HIGH(fwVersion) ((fwVersion >> 8) & 0xff)
#define BLUNRG_FW_VERSION_LOW(fwVersion) ((fwVersion >> 4) & 0x0f)
#define BLUNRG_FW_VERSION_PATCH_CHAR(fwVersion) ((fwVersion  & 0x0f)+'a'-1)
uint8_t getMyBlueNRGVersion(BLUENGR_VERSION_INFO* pVer);

#endif


#define VERSION         0x0502

#define JBUS_STATUS     0x0
#define ODOMETER        0x1
#define BT_ADC          0x2
#define J_CONFIG        0x3
#define DEVELOPER       0x04
#if defined(OLD_TAG)
#define VERSION         0x0202
#elif defined(TLM)
  //#define VERSION         0x0302  //initial
  #define VERSION         0x0402  //fixed - sign problem 
//#define VERSION         0x0203  //for reading TLM
#elif defined(BRACELET)
#define VERSION         0x0x0303
#endif
#if 0
typedef __packed struct _TestDevices{

  uint8_t bdaddr[6];
  uint8_t length;
  uint8_t data[30];
  uint8_t Rssi;
  uint8_t type;
  //uint8_t addrestype;
  //uint64_t Baddr;
} PACKED TestDevices;
#endif










#ifdef TRACE_VERSION
typedef struct
{
	int Status; // BLE_STATUS_SUCCESS
	// Local version HCI info (refer Bleetooth core specification, Vol.2, PArt E, Chapter 7.4.1)
  uint8_t hci_version;	
  uint16_t hci_revision;
  uint8_t lmp_pal_version;
  uint16_t manufacturer_name;
  uint16_t lmp_pal_subversion;
	// - refer D:\AckPlatforms\CortexPlat\WliDongle\Middlewares\ST\STM32_BlueNRG\SimpleBlueNRG_HCI\hci\controller\bluenrg_utils.c
  uint8_t hwVersion;
  uint16_t fwVersion;
	char szVersion[24];
  } BLUENGR_VERSION_INFO;

extern BLUENGR_VERSION_INFO gBlueNRGVersion;
#define BLUNRG_HW_VERSION_HIGH(hwVersion) ((hwVersion >> 4) & 0xff)
#define BLUNRG_HW_VERSION_LOW_CHAR(hwVersion) (hwVersion & 0xff)
#define BLUNRG_FW_VERSION_HIGH(fwVersion) ((fwVersion >> 8) & 0xff)
#define BLUNRG_FW_VERSION_LOW(fwVersion) ((fwVersion >> 4) & 0x0f)
#define BLUNRG_FW_VERSION_PATCH_CHAR(fwVersion) ((fwVersion  & 0x0f)+'a'-1)
uint8_t getMyBlueNRGVersion(BLUENGR_VERSION_INFO* pVer);

#endif