/**
 * @file    SDK_EVAL_Spi_Driver.h
 * @author  AMS - AAS Division
 * @version V1.0.1
 * @date    March, 10 2014
 * @brief   This file contains definitions for Software Development Kit eval SPI devices
 * @details
 *
 * In this module there are API for the management of the SPI devices on the SDK Eval
 * motherboards.
 *
 * <b>Example:</b>
 * @code
 *
 *   SdkEvalSpiInit();
 *
 *   ...
 *
 * @endcode
 *
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
 * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
 * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
 * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
 * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * THIS SOURCE CODE IS PROTECTED BY A LICENSE.
 * FOR MORE INFORMATION PLEASE CAREFULLY READ THE LICENSE AGREEMENT FILE LOCATED
 * IN THE ROOT DIRECTORY OF THIS FIRMWARE PACKAGE.
 *
 * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SDK_EVAL_DRIVER_H
#define __SDK_EVAL_DRIVER_H

/* Includes ------------------------------------------------------------------*/
#include "stm32F0xx.h"
#include "stm32f0xx_hal.h"
#ifdef __cplusplus
 extern "C" {
#endif


/** @addtogroup SDK_EVAL_STM32L
* @{
*/
   
   
/** @defgroup SPI_Private_TypesDefinitions
* @{
*/
#if BLUENRG_MS
#undef  ENABLE_SPI_FIX
#define ENABLE_SPI_FIX 0
#else
#ifndef ENABLE_SPI_FIX
#define ENABLE_SPI_FIX 1
#endif 
#endif

/**
* @}
*/


/** @addtogroup SDK_EVAL_Spi                    SDK EVAL Spi
* @brief SPI functions implementation.
* @details This file implements the BlueNRG Library SPI interface functions. 
* @{
*/

   /** @addtogroup SDK_EVAL_SPI_Exported_Types          SDK EVAL Spi Exported Types
 * @{
 */

/**
 * @brief  SPI Mode for SDK EVAL enumeration
 */
typedef enum
{
  SPI_MODE_GPIO = 0,
  SPI_MODE_EXTI = 1

} SdkEvalSpiMode;

/**
* @}
*/

/** @defgroup SPI_Private_Defines
* @{
*/


#define CS_TO_SCLK_DELAY  0x0001
#define CLK_TO_CS_DELAY   0x0001


/**
* @}
*/


/** @defgroup SDK_EVAL_Spi_Peripheral_Gpio
* @{
*/


/**
* @brief BlueNRG Eval Board (STEVAL-IDB002V1) & USB Dongle: 
 *       common SPI and GPIO lines
 */

/* BlueNRG SPI Port */
#define SDK_EVAL_V3_SPI_PERIPH_INSTANCE               SPI2
/* BlueNRG SPI Clock */
#define SDK_EVAL_V3_SPI_PERIPH_CLK_ENABLE()	      __SPI2_CLK_ENABLE()

// SPI Configuration
#define SDK_EVAL_V3_SPI_PERIPH_MODE		      SPI_MODE_MASTER
#define SDK_EVAL_V3_SPI_PERIPH_DIRECTION	      SPI_DIRECTION_2LINES
#define SDK_EVAL_V3_SPI_PERIPH_DATASIZE	              SPI_DATASIZE_8BIT
#define SDK_EVAL_V3_SPI_PERIPH_CLKPOLARITY	      SPI_POLARITY_LOW
#define SDK_EVAL_V3_SPI_PERIPH_CLKPHASE	              SPI_PHASE_1EDGE
#define SDK_EVAL_V3_SPI_PERIPH_NSS		      SPI_NSS_SOFT
#define SDK_EVAL_V3_SPI_PERIPH_FIRSTBIT	              SPI_FIRSTBIT_MSB
#define SDK_EVAL_V3_SPI_PERIPH_TIMODE		      SPI_TIMODE_DISABLED
#define SDK_EVAL_V3_SPI_PERIPH_CRCPOLYNOMIAL	      7
#define SDK_EVAL_V3_SPI_PERIPH_BAUDRATEPRESCALER      SPI_BAUDRATEPRESCALER_4 
#define SDK_EVAL_V3_SPI_PERIPH_CRCCALCULATION	      SPI_CRCCALCULATION_DISABLED


/* BlueNRG SPI SCLK define values */
#define SDK_EVAL_V3_SPI_PERIPH_SCLK_PIN               GPIO_PIN_13                 /* PB.13 */
#define SDK_EVAL_V3_SPI_PERIPH_SCLK_MODE	      GPIO_MODE_AF_PP
#define SDK_EVAL_V3_SPI_PERIPH_SCLK_PULL	      GPIO_PULLUP
#define SDK_EVAL_V3_SPI_PERIPH_SCLK_SPEED	      GPIO_SPEED_HIGH             /* GPIOB */
#define SDK_EVAL_V3_SPI_PERIPH_SCLK_ALTERNATE	      GPIO_AF5_SPI2
#define SDK_EVAL_V3_SPI_PERIPH_SCLK_PORT	      GPIOB
#define SDK_EVAL_V3_SPI_PERIPH_SCLK_CLK_ENABLE()      __GPIOB_CLK_ENABLE()
/* Defines for MISO pin */
#define SDK_EVAL_V3_SPI_PERIPH_MISO_PIN               GPIO_PIN_14                 /* PB.14 */
#define SDK_EVAL_V3_SPI_PERIPH_MISO_MODE	      GPIO_MODE_AF_PP
#define SDK_EVAL_V3_SPI_PERIPH_MISO_PULL	      GPIO_PULLDOWN 
#define SDK_EVAL_V3_SPI_PERIPH_MISO_SPEED	      GPIO_SPEED_HIGH
#define SDK_EVAL_V3_SPI_PERIPH_MISO_ALTERNATE	      GPIO_AF5_SPI2
#define SDK_EVAL_V3_SPI_PERIPH_MISO_PORT	      GPIOB
#define SDK_EVAL_V3_SPI_PERIPH_MISO_CLK_ENABLE()      __GPIOB_CLK_ENABLE()
/* Defines for MOSI pin */
#define SDK_EVAL_V3_SPI_PERIPH_MOSI_PIN               GPIO_PIN_15                 /* PB.15 */
#define SDK_EVAL_V3_SPI_PERIPH_MOSI_MODE	      GPIO_MODE_AF_PP
#define SDK_EVAL_V3_SPI_PERIPH_MOSI_PULL	      GPIO_PULLUP 
#define SDK_EVAL_V3_SPI_PERIPH_MOSI_SPEED	      GPIO_SPEED_HIGH
#define SDK_EVAL_V3_SPI_PERIPH_MOSI_ALTERNATE	      GPIO_AF5_SPI2
#define SDK_EVAL_V3_SPI_PERIPH_MOSI_PORT              GPIOB                       /* GPIOB */
#define SDK_EVAL_V3_SPI_PERIPH_MOSI_CLK_ENABLE()      __GPIOB_CLK_ENABLE()
/* BlueNRG SPI CS define values */
#define SDK_EVAL_V3_SPI_PERIPH_CS_PIN                 GPIO_PIN_12                 /* PB.12 */
#define SDK_EVAL_V3_SPI_PERIPH_CS_MODE	              GPIO_MODE_OUTPUT_PP
#define SDK_EVAL_V3_SPI_PERIPH_CS_PULL	              GPIO_NOPULL 
#define SDK_EVAL_V3_SPI_PERIPH_CS_SPEED	              GPIO_SPEED_HIGH
#define SDK_EVAL_V3_SPI_PERIPH_CS_ALTERNATE	      0
#define SDK_EVAL_V3_SPI_PERIPH_CS_PORT                GPIOB                       /* GPIOB */
#define SDK_EVAL_V3_SPI_PERIPH_CS_CLK_ENABLE()        __GPIOB_CLK_ENABLE()
  
/* BlueNRG SW reset line define values */
#define SDK_EVAL_V3_SPI_PERIPH_RESET_PIN              GPIO_PIN_13                 /* PC.13 */
#define SDK_EVAL_V3_SPI_PERIPH_RESET_MODE	      GPIO_MODE_OUTPUT_PP
#define SDK_EVAL_V3_SPI_PERIPH_RESET_PULL	      GPIO_NOPULL 
#define SDK_EVAL_V3_SPI_PERIPH_RESET_SPEED	      GPIO_SPEED_LOW 
#define SDK_EVAL_V3_SPI_PERIPH_RESET_ALTERNATE	      0
#define SDK_EVAL_V3_SPI_PERIPH_RESET_PORT             GPIOC
#define SDK_EVAL_V3_SPI_PERIPH_RESET_CLK_ENABLE()     __GPIOC_CLK_ENABLE()

/* Defines for SPI IRQ pin */ 

/**
 * @brief BlueNRG Eval Board (STEVAL-IDB002V1) SPI IRQ line: EXTI15_10_IRQHandler() 
 */
#define SDK_EVAL_V3_SPI_PERIPH_IRQ_PIN              GPIO_PIN_10                  /* PC.10 */
#define SDK_EVAL_V3_SPI_PERIPH_IRQ_MODE	            GPIO_MODE_IT_RISING
#define SDK_EVAL_V3_SPI_PERIPH_IRQ_PULL	            GPIO_PULLDOWN 
#define SDK_EVAL_V3_SPI_PERIPH_IRQ_SPEED	    GPIO_SPEED_HIGH
#define SDK_EVAL_V3_SPI_PERIPH_IRQ_ALTERNATE	    0
#define SDK_EVAL_V3_SPI_PERIPH_IRQ_PORT	            GPIOC
#define SDK_EVAL_V3_SPI_PERIPH_IRQ_CLK_ENABLE()     __GPIOC_CLK_ENABLE()

#define SDK_EVAL_V3_SPI_PERIPH_EXTI_IRQn            EXTI15_10_IRQn
#define SDK_EVAL_V3_SPI_PERIPH_EXTI_IRQHandler      EXTI15_10_IRQHandler
#define SDK_EVAL_V3_SPI_PERIPH_EXTI_PIN             SDK_EVAL_V3_SPI_PERIPH_IRQ_PIN 
#define SDK_EVAL_V3_SPI_PERIPH_EXTI_PORT            SDK_EVAL_V3_SPI_PERIPH_IRQ_PORT 

/**
 * @brief USB Dongle SPI IRQ line: EXTI15_10_IRQHandler() 
 */
#define SDK_EVAL_VD1_SPI_PERIPH_IRQ_PIN             GPIO_PIN_10                  /* PB.10 */
#define SDK_EVAL_VD1_SPI_PERIPH_IRQ_MODE	    GPIO_MODE_IT_RISING
#define SDK_EVAL_VD1_SPI_PERIPH_IRQ_PULL	    GPIO_PULLDOWN 
#define SDK_EVAL_VD1_SPI_PERIPH_IRQ_SPEED	    GPIO_SPEED_HIGH
#define SDK_EVAL_VD1_SPI_PERIPH_IRQ_ALTERNATE	    0
#define SDK_EVAL_VD1_SPI_PERIPH_IRQ_PORT	    GPIOB
#define SDK_EVAL_VD1_SPI_PERIPH_IRQ_CLK_ENABLE()    __GPIOB_CLK_ENABLE()

#define SDK_EVAL_VD1_SPI_PERIPH_EXTI_IRQn           EXTI15_10_IRQn
#define SDK_EVAL_VD1_SPI_PERIPH_EXTI_IRQHandler     EXTI15_10_IRQHandler
#define SDK_EVAL_VD1_SPI_PERIPH_EXTI_PIN            SDK_EVAL_VD1_SPI_PERIPH_IRQ_PIN 
#define SDK_EVAL_VD1_SPI_PERIPH_EXTI_PORT           SDK_EVAL_VD1_SPI_PERIPH_IRQ_PORT 


/* BlueNRG Eval Board (STEVAL-IDB002V1) & USB Dongle STEVAL-IDB003V1 IRQ EXTI handler */
#ifndef BNRG_SPI_EXTI_IRQHandler
#define	BNRG_SPI_EXTI_IRQHandler	            EXTI15_10_IRQHandler 
#endif 

#define RTC_WAKEUP_IRQHandler                       RTC_IRQHandler /* ---??? */

/* end EVAL_V3 */

/**
* @}
*/

/**
* @}
*/


/**
* @brief  Initializes the SPI for BlueNRG module
* @param  xSpiMode : Spi mode (Polling or IRQ)
* @retval Status
*/
int8_t SdkEvalSpiInit(SdkEvalSpiMode xSpiMode);
uint8_t SdkEvalSPI_Irq_Pin(void);

void Clear_SPI_EXTI_Flag();

void Enable_SPI_IRQ(void); 
void Disable_SPI_IRQ(void); 

void BlueNRG_RST(void);
void BlueNRG_HW_Bootloader(void);
uint8_t BlueNRG_SPI_Read_All(uint8_t *buffer, uint8_t buff_size); 
int16_t BlueNRG_SPI_Write(uint8_t* data1, uint8_t* data2, uint16_t Nb_bytes1, uint16_t Nb_bytes2); 
void BlueNRG_Power_Down(void);
void BlueNRG_Power_Up(void);
uint8_t BlueNRG_DataPresent(void);
void BlueNRG_IRQ_High(void);
void BlueNRG_Release_IRQ(void);
/**
* @brief  This function fixes a bug on the HW bootloader activation. This bug appears when the BlueNRG *only*  when all the condition below are satisfiled:
*         \li Device is *BlueNRG*
*         \li 32 MHz XTAL is used
*         \li Device *originally* delivered with firmware *6.3*
*
*         The bug does *not* appear when at least one of the following condiion is satisfied, so there is no need to call this patch function:
*         \li Device is *BlueNRG-MS*
*         \li 16 MHz XTAL is used
*         \li Device originally delivered with firmware 6.4
*
*         Running this function on a BlueNRG part originally delivered 
*         with firmware 6.4 will not cause any harm, since a check is done before patching the bootloader
*
*         The synptoms of the bug are that modification of the flash (e.g. IFR update or firmware update) will fail with return value 0x4A
*         and this is due to violation of flash programming timing when using a 32MHz.
*         Due to the nature of the bug (timing violation), its occurence is not deterministic, i.e. a specific BlueNRG part originally delivered
*         with firmware 6.3 and using a 32 MHz can never show the problem. In other words, the occurence of this problem can cause yield issue
*         during production.
* @note   This function exploit a very tight timing condition to patch the bootloader and must be executed with SPI clock at 8MHz and with interrupts
*         disabled.
* @retval Nothing
*/
void BlueNRG_BootloaderPatch(void);

/**
* @}
*/


/**
* @}
*/


/**
* @}
*/


#ifdef __cplusplus
}
#endif

#endif

/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
