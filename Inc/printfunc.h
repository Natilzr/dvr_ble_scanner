#ifndef PRINTFUNC_H
#define PRINTFUNC_H

#include "stm32f0xx_hal.h"
#include "main.h"


//PROTOCOL Flags
#define IbeaconP        0x01 //ibeacon
#define TLM_P           0x02 //TLM
#define BRACELET1_P     0x04 //bracelet1
#define FUEL_HYUMY      0x08 //fuel + hyumidity

#define P_FLAGS IbeaconP | TLM_P //| FUEL_HYUMY///| BRACELET1_P  bracelet is not supported

#define AdvSupport      0x01    //Advertising support
#define ProCFG          0x02    //protocols configuration support
#define SleepSupported  0x04    //sleep supported
#define HistorySupported 0x10   
//#define WakeupSupported 0x08
#define FLAGS_T   ProCFG   

#define ModeSCAN        0x02
#define ModeADV         0x01

#define MAKELONG(a, b)      ((long)(((unsigned short)(a)) | ((unsigned long)((unsigned short)(b))) << 16))
#define MAKEWORD(a, b)      ((unsigned short)(((unsigned char)(a)) | ((unsigned short)((unsigned char)(b))) << 8))

typedef struct {
    uint16_t sensor_mask;      // Sensor mask indicating which readings are present
    uint16_t voltage;         // Battery voltage in mV
    int16_t temperature;      // Temperature in Fixed Point 8.8 format
    int16_t humidity;         // Humidity in Fixed Point 8.8 format (if valid)
    int16_t acc_x;           // Accelerometer X-axis position in mg (if valid)
    int16_t acc_y;           // Accelerometer Y-axis position in mg (if valid)
    int16_t acc_z;        // Add more fields for other sensor readings if needed
    uint8_t cutoff;
} SensorData;

typedef struct {
    uint16_t Pversion;      // Sensor mask indicating which readings are present
    uint16_t voltage;         // Battery voltage in mV
    int16_t temperature;      // Temperature in Fixed Point 8.8 format
} TlmData;

#define MAX_DEVICES 10   // adjust as needed

typedef struct {
    uint8_t mac[6];
    float temperature;
    float humidity;
    uint16_t batt;
    uint8_t ios;
    int16_t acc_x;           // Accelerometer X-axis position in mg (if valid)
    int16_t acc_y;           // Accelerometer Y-axis position in mg (if valid)
    int16_t acc_z;        // Add more fields for other sensor readings if needed
} SData;


#define TEXT_BUFFER_SIZE 50

extern uint8_t textbuf[60];
extern DevicesSort DeviceTable[DEVICE_TABLE_SIZE];

uint16_t  PrintMsg(uint16_t cnt);
uint16_t  PrintDVRMsg(uint16_t cnt);
uint16_t EncodeCommandStatus(STATUSES1_E Status);
uint16_t  EncodeUUId(uint32_t UUid,uint16_t Ver);
uint32_t u32Alder( uint8_t *pu8Data, uint16_t u32Len,uint32_t *a ,uint32_t *b);
uint16_t EncodeConnectStatus(CON_STATUS Status);
SensorData parseSensorData(const uint8_t *rawData);
uint16_t  PrintMemMsg(uint16_t cnt);
void uint64_to_ascii12_manual(uint64_t value, char *output);
uint8_t prepareAsciiString(SData *data, uint8_t count, char *output, size_t maxLen);
#endif