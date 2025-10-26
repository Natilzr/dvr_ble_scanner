
#include "usart.h"
#include "serialdrivers.h"
#include "gpio.h"
#include "hal_types.h"
#include "STM32_BLUENRG_BLE.h"
#include "string.h"
typedef struct
{
  UART_HandleTypeDef* phuart;
	IRQn_Type				nIrq;
	uint8_t					UartNo;
  TerminalDataDesc        *term_data ;
  uint32_t                   BaudIndex ;
//  u_int                   format ;
//	u_int                   FlowControlActivated ;
//	u_int					MSR;
//	u_int					MCR;
	uint32_t					bOpened;
} TerminalDesc ;

/* Private variables ---------------------------------------------------------*/
TerminalDataDesc    terminal_data_0_pc;
TerminalDesc terminal_0_pc = 
{
  &huart1,            //UART_HandleTypeDef* phuart;
  USART1_IRQn,        // nIrq
	COM1,             // UartNo
	&terminal_data_0_pc, // term_data
	115200,               // BaudIndex
//	0,0,0,0,
	FALSE,

};
TerminalDesc /*const*/*  pTerminals[COM_FIRST_UNRESOLVED] = 
{      

	&terminal_0_pc
};

/* Private function prototypes -----------------------------------------------*/
void WMX_USART1_UART_Init(uint32_t baudrate);

/* USER CODE BEGIN 0 */
// Terminal functions

void InitTerminalData(TerminalDataDesc* pTermData)
{
	pTermData->rx_in_pt  = pTermData->rx_buf;
	pTermData->rx_out_pt = pTermData->rx_buf;
	pTermData->rx_status = EMPTY_BUF;
	pTermData->rx_error = 0;
	pTermData->tx_in_pt  = pTermData->tx_buf;
	pTermData->tx_out_pt = pTermData->tx_buf;
	pTermData->tx_status = EMPTY_BUF;
	pTermData->tx_write_enabled   = TRUE;
	memset(pTermData->rx_buf,0,sizeof(pTermData->rx_buf));
	memset(pTermData->tx_buf,0,sizeof(pTermData->tx_buf));
}

//*----------------------------------------------------------------------------
//* Function Name       : OpenSerial
//* Object              :
//* Input Parameters    : ...
//* Output Parameters   : boolean
//* Functions called    :
//*----------------------------------------------------------------------------

BOOL OpenSerial(COM_TypeDef Port,  unsigned int baudrate, unsigned char FlowControl)
{
  TerminalDesc* pTerm;
	if (Port >= COM_FIRST_UNRESOLVED)
		return FALSE;
  pTerm = pTerminals[Port];
  if (pTerm == NULL)
     return FALSE;
	if (pTerm->bOpened)
		return FALSE;
	InitTerminalData(pTerm->term_data);
	switch(Port)
	{
		case COM1:
			WMX_USART1_UART_Init(baudrate);
			break;
		default:
			// not implemented
			return FALSE;
	}

	// Init UART interrupts
  /* Process Locked */
  __HAL_LOCK(pTerm->phuart);
  /* Enable the UART Parity Error Interrupt */
  __HAL_UART_ENABLE_IT(pTerm->phuart, UART_IT_PE);
  
  /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
  __HAL_UART_ENABLE_IT(pTerm->phuart, UART_IT_ERR);
  __HAL_UNLOCK(pTerm->phuart);
  /* Enable the UART Data Register not empty Interrupt */
  __HAL_UART_ENABLE_IT(pTerm->phuart, UART_IT_RXNE);

	pTerm->bOpened = TRUE;
	return TRUE;
}

BOOL CloseSerial(COM_TypeDef Port)
{
	if (Port < COM_FIRST_UNRESOLVED)
	{
		TerminalDesc* pTerm = pTerminals[Port];
		if (pTerm)
		{
			TerminalDataDesc* pTermData = pTerm->term_data;
			// disable UART interrupts
			if (!pTerm->bOpened)
				return FALSE;
			switch(Port)
			{
				case COM1:
					HAL_UART_DeInit(&huart1);
					break;
				default:
					// not implemented
					return FALSE;
			}
			InitTerminalData(pTermData);
			pTerm->bOpened = FALSE;
			return TRUE;
		}
	}
	return FALSE;
}


BOOL IsSerialOpened(COM_TypeDef Port)
{
	if (Port < COM_FIRST_UNRESOLVED)
	{
		if (pTerminals[Port])
			return pTerminals[Port]->bOpened;
	}
  return FALSE;
}


//*----------------------------------------------------------------------------
//* Function Name       : terminal_write
//* Object              :
//* Input Parameters    : terminal_descritor, string to write
//* Output Parameters   : None
//* Functions called    :
//*----------------------------------------------------------------------------
unsigned char terminal_write (TerminalDesc *pTerm, unsigned char data)
{
//	StructLPCUART * pUart;
	unsigned char* pNext;
//	u_int InterruptState = VICIntEnable;
    TerminalDataDesc* pTermData = pTerm->term_data;
//	pUart = (StructLPCUART *)pTerm->pLPCUart;

//	VICIntEnClr = (1 << pTerm->periph_id);
	DISABLE_INTERRUPT_LOCAL(pTerm->nIrq);

	if(pTermData->tx_status  == FULL_BUF)
	{
		// restore interrupt
//		if (InterruptState & (1 << pTerm->periph_id))
	//		VICIntEnable = (1 << pTerm->periph_id);
    RESTORE_INTERRUPT_LOCAL(pTerm->nIrq);
		return 1;
	}
	pNext = pTermData->tx_in_pt+1;
	if ((pNext - pTermData->tx_buf) >= TERMINAL_TX_BUFFER_SIZE)
		pNext = pTermData->tx_buf;
	if (pNext == pTermData->tx_out_pt)
	{
		pTermData->tx_status  = FULL_BUF;
		// restore interrupt
		//if (InterruptState & (1 << pTerm->periph_id))
		//	VICIntEnable = (1 << pTerm->periph_id);
    RESTORE_INTERRUPT_LOCAL(pTerm->nIrq);
		return 1;
	}


	*pTermData->tx_in_pt = data;
	pTermData->tx_in_pt = pNext;
	
	if(pTermData->tx_status  == NORMAL_BUF) // previous data waits for interrupt
	{
		// restore interrupt
		//if (InterruptState & (1 << pTerm->periph_id))
//			VICIntEnable = (1 << pTerm->periph_id);
    RESTORE_INTERRUPT_LOCAL(pTerm->nIrq);
		return 0;
	}

	// here pTermData->tx_status  == EMPTY_BUF
	if (!pTermData->tx_write_enabled )				
	{
		// empty buffer, but direct write to uart is not enabled
		// because previous byte was writen but there was not received interrupt
		// that THR is empty
		pTermData->tx_status  = NORMAL_BUF;
		// restore interrupt
		//if (InterruptState & (1 << pTerm->periph_id))
//			VICIntEnable = (1 << pTerm->periph_id);
    RESTORE_INTERRUPT_LOCAL(pTerm->nIrq);
		return 0;
	}

	// Now all previously sent symbols are transmitted.
	// we can write data directly to UART

	// set out pointer == in pointer, because out data deirectly
	pTermData->tx_status  = NORMAL_BUF;
	pTermData->tx_write_enabled = FALSE; 
   /* Enable the UART Transmit Complete Interrupt */
   __HAL_UART_ENABLE_IT(pTerm->phuart, UART_IT_TXE);

   RESTORE_INTERRUPT_LOCAL(pTerm->nIrq);

	return 0;
}

unsigned char WriteSerial(unsigned char Port,char* message, short Len)
{
	if (Port < COM_FIRST_UNRESOLVED)
	{
		while(Len--)
		{
			if (terminal_write(pTerminals[Port],*message++))
				return 1;
		}
		return 0;
	}
	else
		return 1;
}


//*----------------------------------------------------------------------------
//* Function Name       : terminal_read
//* Object              :
//* Input Parameters    : terminal_descritor, string to write
//* Output Parameters   : None
//* Functions called    :
//*----------------------------------------------------------------------------

unsigned short terminal_read ( TerminalDesc *pTerm)
//* Begin
{

  TerminalDataDesc    *pTermData = pTerm->term_data ;
	unsigned char ch;
	unsigned char status;
  DISABLE_INTERRUPT_LOCAL(pTerm->nIrq);


	if (pTermData->rx_out_pt != pTermData->rx_in_pt)
	{
		ch = *pTermData->rx_out_pt++;
		if (pTermData->rx_out_pt - pTermData->rx_buf >= TERMINAL_RX_BUFFER_SIZE)
			pTermData->rx_out_pt = pTermData->rx_buf;
		if (pTermData->rx_out_pt == pTermData->rx_in_pt)
			pTermData->rx_status = EMPTY_BUF;
		else
			pTermData->rx_status = NORMAL_BUF;
		status = 0;
	}
	else
	{

		status = REC_BUFFER_EMPTY;
		ch = 0;

	}
	status |= pTermData->rx_error;
	pTermData->rx_error = 0;

  RESTORE_INTERRUPT_LOCAL(pTerm->nIrq);

	return MAKEWORD(ch,status);

}

unsigned short ReadSerial(unsigned char Port)
{
	return terminal_read (pTerminals[Port]);

}

unsigned char GetUartStatusInternal(unsigned char Port)
{
	unsigned char uchStatus = 0;
#if 0
#ifdef UART1_IN_USE
	if (Port == UART1_INTERNAL) // only this port have DTR
	{
	    TerminalDesc *term_desc = &terminal_1_modem;
		StructLPCUART * pUart = (StructLPCUART *)term_desc->pLPCUart;
		uchStatus = (pUart->MSR >> 4) & 0x0f;
	}
	else
#endif
		uchStatus = pTerminals[Port]->MSR;

	uchStatus |= pTerminals[Port]->term_data->rx_error;
	//terminal_data_modem_2.rx_error = 0;
	pTerminals[Port]->term_data->rx_error = 0;
#endif
	return uchStatus;
}

/* USER CODE END 0 */

/* USER CODE BEGIN 1 */
// Init and Interrurts related functions
/* USART1 init function - aas generated by STM32Cube*/
void WMX_USART1_UART_Init(uint32_t baudrate)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = baudrate;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE ;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&huart1);

}

/**
* @brief This function handles UART_IT_RXNE interrupt for specific USARTx global interrupt using passed terminal description 
*/
void WLI_UART_Receive_IT(TerminalDesc *term_desc)
{
  		int shMaxSymbolsForInput;
		unsigned char  uchReadSym;
	TerminalDataDesc* pTermData = term_desc->term_data;

		shMaxSymbolsForInput = (int)(pTermData->rx_out_pt - pTermData->rx_in_pt) - 1;
		if (shMaxSymbolsForInput < 0)
			shMaxSymbolsForInput+=TERMINAL_RX_BUFFER_SIZE;
//		uchReadSym =pUart->RBR_THR_DLL; 
		//uchReadSym = USART_ReceiveData(term_desc->pUartDesc->pUsart);
     uchReadSym = (uint16_t)(term_desc->phuart->Instance->RDR & (uint16_t)0x00FF);

		if (shMaxSymbolsForInput == 0)
			 pTermData->rx_status = FULL_BUF;

		else 
		{
			*pTermData->rx_in_pt++ = uchReadSym;
			shMaxSymbolsForInput--;
			if (pTermData->rx_in_pt - pTermData->rx_buf >= TERMINAL_RX_BUFFER_SIZE)
				pTermData->rx_in_pt = pTermData->rx_buf;
			pTermData->rx_status = NORMAL_BUF;
		}

}
/**
* @brief This function handles UART_IT_TXE interrupt for specific USARTx global interrupt using passed terminal description 
*/
void WLI_UART_Transmit_IT(TerminalDesc *term_desc)
{
	TerminalDataDesc* pTermData = term_desc->term_data;
 		if (pTermData->tx_in_pt == pTermData->tx_out_pt)
		{
			pTermData->tx_status = EMPTY_BUF;
			pTermData->tx_write_enabled = TRUE;
      /* Disable the EVAL_COM1 Transmit interrupt */
//      USART_ITConfig(term_desc->pUartDesc->pUsart, USART_IT_TXE, DISABLE);
     __HAL_UART_DISABLE_IT(term_desc->phuart, UART_IT_TXE);

		}

		else 
//			if (!term_desc->FlowControlActivated || (pUart->MSR & UART_MSR_CTS_STATE)  || (term_desc->MSR & UART_CTS))
		{
			// Think about flow control
	    //USART_SendData(term_desc->pUartDesc->pUsart, *pTermData->tx_out_pt++);
        term_desc->phuart->Instance->TDR = (uint8_t)(*pTermData->tx_out_pt++);

//			pUart->RBR_THR_DLL = *pTermData->tx_out_pt++;
			if ((pTermData->tx_out_pt - pTermData->tx_buf) >= TERMINAL_TX_BUFFER_SIZE)
					pTermData->tx_out_pt = pTermData->tx_buf;
			pTermData->tx_status  = NORMAL_BUF;
			pTermData->tx_write_enabled = FALSE;
		}

}

/**
* @brief This function handles specific USARTx global interrupt using passed terminal description 
*/
void USARTx_IRQHandler(TerminalDesc *term_desc)
{
   uint32_t tmp1 = 0, tmp2 = 0;
   UART_HandleTypeDef * huart = term_desc->phuart;
# if 1

  tmp1 = __HAL_UART_GET_FLAG(huart, UART_FLAG_PE);
  tmp2 = __HAL_UART_GET_IT_SOURCE(huart, UART_IT_PE);  
  /* UART parity error interrupt occurred ------------------------------------*/
  if((tmp1 != RESET) && (tmp2 != RESET))
  { 
    __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_PE);
    
    huart->ErrorCode |= HAL_UART_ERROR_PE;
  }
  
  tmp1 = __HAL_UART_GET_FLAG(huart, UART_FLAG_FE);
  tmp2 = __HAL_UART_GET_IT_SOURCE(huart, UART_IT_ERR);
  /* UART frame error interrupt occurred -------------------------------------*/
  if((tmp1 != RESET) && (tmp2 != RESET))
  { 
    __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_FE);
    
    huart->ErrorCode |= HAL_UART_ERROR_FE;
  }
  
  tmp1 = __HAL_UART_GET_FLAG(huart, UART_FLAG_NE);
  tmp2 = __HAL_UART_GET_IT_SOURCE(huart, UART_IT_ERR);
  /* UART noise error interrupt occurred -------------------------------------*/
  if((tmp1 != RESET) && (tmp2 != RESET))
  { 
    __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_NE);
    
    huart->ErrorCode |= HAL_UART_ERROR_NE;
  }
  
  tmp1 = __HAL_UART_GET_FLAG(huart, UART_FLAG_ORE);
  tmp2 = __HAL_UART_GET_IT_SOURCE(huart, UART_IT_ERR);
  /* UART Over-Run interrupt occurred ----------------------------------------*/
  if((tmp1 != RESET) && (tmp2 != RESET))
  { 
    __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_ORE);
    
    huart->ErrorCode |= HAL_UART_ERROR_ORE;
  }
  
  tmp1 = __HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE);
  tmp2 = __HAL_UART_GET_IT_SOURCE(huart, UART_IT_RXNE);
  /* UART in mode Receiver ---------------------------------------------------*/
  if((tmp1 != RESET) && (tmp2 != RESET))
  { 
    //UART_Receive_IT(huart);
    WLI_UART_Receive_IT(term_desc);
    __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_RXNE);
  }
  
  tmp1 = __HAL_UART_GET_FLAG(huart, UART_FLAG_TXE);
  tmp2 = __HAL_UART_GET_IT_SOURCE(huart, UART_IT_TXE);
  /* UART in mode Transmitter ------------------------------------------------*/
  if((tmp1 != RESET) && (tmp2 != RESET))
  {
//    UART_Transmit_IT(huart);
    WLI_UART_Transmit_IT(term_desc);
    __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_TXE);
  }
  
  if(huart->ErrorCode != HAL_UART_ERROR_NONE)
  {
    /* Set the UART state ready to be able to start again the process */
    huart->gState = HAL_UART_STATE_READY;
    
    HAL_UART_ErrorCallback(huart);
  }  

# endif // 1
}

/**
* @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
  USART1_IRQHandler() function is moved to UartDrv.c
*/
void USART1_IRQHandler(void)
{
   USARTx_IRQHandler(pTerminals[COM1]);
}






/* USER CODE END 1 */

