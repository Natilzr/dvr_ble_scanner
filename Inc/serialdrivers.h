#ifndef _SERIAL_DRIVERS_H
#define _SERIAL_DRIVERS_H

#include "hal_types.h"

// internal for drivers using
#ifdef TRACE_VERSION
#define TERMINAL_TX_BUFFER_SIZE     256
#else
#define TERMINAL_TX_BUFFER_SIZE     100
#endif

#define TERMINAL_RX_BUFFER_SIZE     64
//#define TERMINAL_RX_LOW_WATER_MARK 100
//#define TERMINAL_RX_HIGH_WATER_MARK 30

#ifdef STM32F072xB
#define DISABLE_INTERRUPT_LOCAL(n)  uint32_t localInterruptStateBefore = NVIC->ISER[n >> 0x05]; \
        NVIC->ICER[n >> 0x05] = (uint32_t)0x01 << (n & (uint8_t)0x1F);

#define RESTORE_INTERRUPT_LOCAL(n)  if (localInterruptStateBefore &  (uint32_t)0x01 << (n & (uint8_t)0x1F)) NVIC->ISER[n >> 0x05] = (uint32_t)0x01 << (n & (uint8_t)0x1F);
#endif



#define REC_BUFFER_EMPTY		0x80
#define UART_OVERRUN_ERROR		0x10
#define UART_FRAMING_ERROR		0x20
#define UART_BREAK_CONDITION	        0x40

#define PC_BAUDRATE 9600//115200

typedef enum
{
	EMPTY_BUF   = 0,
	NORMAL_BUF	= 1,
	FULL_BUF   = 0xff
} TERMINAL_BUFFER_STATES;

// from D:\AckPlatforms\CortexPlat\Samples\arm2\examples\ST\STM32F10x\stm32f10x_stdperiph_lib\Utilities\STM32_EVAL\stm32100b_eval\stm32100b_eval.h
typedef enum 
{
  COM1 = 0,
//  COM2 = 1,
//  COM3 = 2,
//  COM4 = 3,
  COM_FIRST_UNRESOLVED
} COM_TypeDef;   


typedef struct TerminalDataDesc 
{
    uint8_t          rx_buf[TERMINAL_RX_BUFFER_SIZE] ;
    uint8_t          tx_buf[TERMINAL_TX_BUFFER_SIZE] ;
    uint8_t          *rx_in_pt ;
    uint8_t         *rx_out_pt ;
    uint32_t           rx_status ;
    uint32_t           rx_error ;
    uint8_t          *tx_in_pt ;
    uint8_t          *tx_out_pt ;
    uint32_t           tx_status ;
    uint32_t           tx_write_enabled;
} TerminalDataDesc;


// Intrenal UARTs functions
BOOL OpenSerial(COM_TypeDef Port,  unsigned int baudrate, unsigned char FlowControl);
BOOL CloseSerial(COM_TypeDef Port);
BOOL IsSerialOpened(COM_TypeDef Port);

unsigned char WriteSerial(unsigned char uchLine,char* message, short Len);
unsigned char SetBaudrateInternal(unsigned char InternalPort,  unsigned int baudrate);
unsigned char GetBaudrateInternal(unsigned char InternalPort);
// SPI UART functions
// External Uart functions
unsigned char OpenSerialMemUART(unsigned char baudrate, unsigned char mode,unsigned char FlowControl, unsigned char uchInitOutputState);
void CloseSerialMemUART(void);
unsigned char WriteUartMemMessage(char* message, short shMsgLen);
void WriteMemUartBlocked(char* pData,int counter);


unsigned short ReadSerial(unsigned char Port);

unsigned short read_uart_mem(void);

unsigned char GetUartStatusInternal(unsigned char Port);
unsigned char GetExtUartStatus(void);
unsigned char GetTransmitStatusInternal(unsigned char Port);
unsigned char GetTransmitStatusExternal(void);
unsigned char GetRxStatusInternal(unsigned char Port);
#ifdef WAVECOM_GPRS_SUPPORT
unsigned char GetRxStatusExternal(void);
#endif
unsigned char SetMCRinternal(unsigned char Port,unsigned char Mask,unsigned char SetClr);
void FlowControlModem(unsigned char OnOff);
void ModemIrqTrig(void);
unsigned char SetMCRexternal(unsigned char Mask,unsigned char SetClr);
unsigned char GetMCRexternal(void);
unsigned char GetMCRinternal(unsigned char Port);

void SetBreakInternal(unsigned char Port);

void SetBreakExternal(unsigned char State);

unsigned int GetSerialInterruptsNoInternal(unsigned char uchPort);



#endif // _SERIAL_DRIVERS_H
