#ifndef BLE_FUNC
#define  BLE_FUNC

extern uint16_t BT_handle;
extern ReadStruct Readbuf;
//extern ReadStruct confbuf;
extern TestDevices ScanDevices[SCAN_TABLE_SIZE];
extern DevicesSort DeviceTable[DEVICE_TABLE_SIZE];
extern uint16_t DeviceCount;


#define Developer_Handle  0x17
#define JBus_Config_Status_Handle  0x12
#define JBus_GPS_Status_Handle  0x14
#define System_Event_ID_Handle  0x23
#define System_Adc_ID_Handle  0x27
#define Version_Handle  0x10
/* Private macros ------------------------------------------------------------*/
#define COPY_UUID_128(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
do {\
    uuid_struct[0] = uuid_0; uuid_struct[1] = uuid_1; uuid_struct[2] = uuid_2; uuid_struct[3] = uuid_3; \
        uuid_struct[4] = uuid_4; uuid_struct[5] = uuid_5; uuid_struct[6] = uuid_6; uuid_struct[7] = uuid_7; \
            uuid_struct[8] = uuid_8; uuid_struct[9] = uuid_9; uuid_struct[10] = uuid_10; uuid_struct[11] = uuid_11; \
                uuid_struct[12] = uuid_12; uuid_struct[13] = uuid_13; uuid_struct[14] = uuid_14; uuid_struct[15] = uuid_15; \
}while(0)
#define COPY_CONFIG_SERVICE_UUID(uuid_struct)         COPY_UUID_128(uuid_struct,0x00,0x00,0x01,0x03, 0x86,0xd6, 0x11,0xe5 ,0xaf,0x63 ,0xfe,0xff,0x81,0x9c,0xdc,0x9f)
#define COPY_CONFIRM_SERVICE_UUID(uuid_struct)         COPY_UUID_128(uuid_struct,0x00,0x00,0x01,0x06, 0x86,0xd6, 0x11,0xe5 ,0xaf,0x63 ,0xfe,0xff,0x81,0x9c,0xdc,0x9f)
#define COPY_ENGINE_SERVICE_UUID(uuid_struct)            COPY_UUID_128(uuid_struct,0x00,0x00,0x10,0x02, 0xcb,0x73, 0x43,0x7d ,0x8f,0xad ,0x84,0x2c,0x16,0xc7,0xaa,0x6f)
#define COPY_ADC_SERVICE_UUID(uuid_struct)            COPY_UUID_128(uuid_struct,0x00,0x00,0x1a,0x07, 0xcb,0x73, 0x43,0x7d ,0x8f,0xad ,0x84,0x2c,0x16,0xc7,0xaa,0x6f)
                                                                                     //00001a02-cb73-437d-8fad-842c16c7aa6f
                                                                                      //00001a02-cb73-437d-8fad-842c16c7aa6f
uint8_t StartBT(uint8_t M_ROLE);

int8_t ScanBT(void);
int8_t ConnectBT(tBDAddr peer_bdaddr);
void HCI_Event_CB(void *pckt);
void proc_complete_event(evt_gatt_procedure_complete *pevt);

//BOOL IsScanResponseDataAcceptable(Advertising_Report_t* pReport);
void advertising_report_event_handler(le_advertising_info *Advertising_Report);
BOOL IsScanResponseDataAcceptable(le_advertising_info* pReport);
BOOL IsDataAcceptable(le_advertising_info* pReport);
BOOL AddTagId(le_advertising_info* pReport,TestDevices* pDevices, int idxsToAdd, uint16_t *pCounter);
BOOL IsTagIdPresent(uint8_t Address[6],TestDevices* pDevices, int Counter,int* pIdxToAdd);
//BOOL IsTagIdPresent(uint8_t Address[6],TestDevices* pDevices, int Counter,int* pIdxToAdd);
//BOOL IsDataAcceptable(Advertising_Report_t* pReport);
//BOOL AddTagId(Advertising_Report_t* pReport,TestDevices* pDevices, int idxsToAdd, int16_t* pCounter);
//BOOL AddTagId(Advertising_Report_t* pReport,TestDevices* pDevices, int idxsToAdd, uint16_t *pCounter);
//BOOL IsTagIdPresent(uint8_t Address[6],TestDevices* pDevices, int Counter,int* pIdxToAdd);
#endif
