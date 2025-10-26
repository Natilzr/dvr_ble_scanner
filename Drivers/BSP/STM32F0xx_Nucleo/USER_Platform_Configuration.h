/**
 * @file    USER_Platform_Configuration_auto.h
 * @author  AMS VMA RF application team
 * @version V1.0.1
 * @date    May, 15 2015
 * @brief   This file contains definitions for BlueNRG, BlueNRG-MS kits platforms with auto identification.
 *
 * In this module there the define values for the automatic configuration  management 
 * of the BlueNRG, BlueNRG-MS kits platforms at runtime.
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
#ifndef __USER_PLATFORM_CONFIGURATION_H
#define __USER_PLATFORM_CONFIGURATION_H

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx.h"

#ifdef __cplusplus
 extern "C" {
#endif
   
/* No OTA support */  
#if !defined(ST_OTA_BTL) && !defined(ST_OTA_BASIC_APPLICATION)
#define VECTOR_TABLE_BASE_ADDRESS  (0x0000)
#endif 

/* When using OTA service, the base address of Reset Manager is the same as the one
   used for BlueNRG, BlueNRG-MS kits for keeping same reference base addresses */
#define VECTOR_TABLE_BASE_DFU_OFFSET (0x3000)
   
/**
 * @brief BlueNRG User Platform EEPROM SPI & GPIO lines: to be customized for addressing
 *        user platform. EEPROM is used only for storing prduction data related to the Eval kit.
 *        It is not needed by BlueNRG
 */
#define EEPROM_SPI_PERIPH_NB              SPI1
#define EEPROM_SPI_PERIPH_RCC_APB2()     __SPI1_CLK_ENABLE()
  
/*  EEPROM MOSI */
#define EEPROM_SPI_PERIPH_MOSI_PORT       GPIOA                      /* PA.7 */
#define EEPROM_SPI_PERIPH_MOSI_PIN        GPIO_PIN_7
#define EEPROM_SPI_PERIPH_MOSI_AF         GPIO_AF5_SPI1
#define EEPROM_SPI_PERIPH_MOSI_RCC()      __GPIOA_CLK_ENABLE()

/* EEPROM MISO */
#define EEPROM_SPI_PERIPH_MISO_PORT       GPIOA                      /* PA.6 */
#define EEPROM_SPI_PERIPH_MISO_PIN        GPIO_PIN_6
#define EEPROM_SPI_PERIPH_MISO_AF         GPIO_AF5_SPI1
#define EEPROM_SPI_PERIPH_MISO_RCC()      __GPIOA_CLK_ENABLE()
   
/* Defines for SCLK pin */
#define EEPROM_SPI_PERIPH_SCLK_PORT       GPIOB                      /* PB.3 */
#define EEPROM_SPI_PERIPH_SCLK_PIN        GPIO_PIN_3
#define EEPROM_SPI_PERIPH_SCLK_AF         GPIO_AF5_SPI1
#define EEPROM_SPI_PERIPH_SCLK_RCC()      __GPIOB_CLK_ENABLE()
   
/* Defines for chip select pin  */ 
#define EEPROM_SPI_PERIPH_CS_PORT         GPIOA                     /* PA.9 */
#define EEPROM_SPI_PERIPH_CS_PIN          GPIO_PIN_9
   
/**
 * @brief BlueNRG User Platform Leds GPIO lines: to be customized for addressing
 *        user platform.
 *        Reference configuration: Nucleo-L152RE (only 1 led)
 */
#define LED_ON                            GPIO_PIN_SET  //TBR
#define LED_OFF                           GPIO_PIN_RESET 
     
#define USER_LED1_GPIO_PIN                GPIO_PIN_5
#define USER_LED1_GPIO_PORT               GPIOA

#define USER_LED2_GPIO_PIN                GPIO_PIN_5
#define USER_LED2_GPIO_PORT               GPIOA

#define USER_LED3_GPIO_PIN                GPIO_PIN_5
#define USER_LED3_GPIO_PORT               GPIOA

#define USER_LED4_GPIO_PIN                GPIO_PIN_5
#define USER_LED4_GPIO_PORT               GPIOA

#define USER_LED5_GPIO_PIN                GPIO_PIN_5
#define USER_LED5_GPIO_PORT               GPIOA

/**
 * @brief BlueNRG User Platform Buttons GPIO lines: to be customized for addressing
 *        user platform.
 *        Reference configuration: Nucleo-L152RE (only 1 button): 
 *        IRQ handler:  EXTI15_10_IRQHandler()
 */
#define	USER_BUTTON1_GPIO_PIN	         GPIO_PIN_13
#define	USER_BUTTON1_GPIO_PORT	         GPIOC
#define	USER_BUTTON1_EXTI_PORT_SOURCE    USER_BUTTON1_GPIO_PORT 
#define	USER_BUTTON1_EXTI_PIN_SOURCE     USER_BUTTON1_GPIO_PIN 
#define	USER_BUTTON1_EXTI_IRQn	         EXTI15_10_IRQn
   
#define	USER_BUTTON2_GPIO_PIN	         GPIO_PIN_13
#define	USER_BUTTON2_GPIO_PORT	         GPIOC
#define	USER_BUTTON2_EXTI_PORT_SOURCE    USER_BUTTON2_GPIO_PORT 
#define	USER_BUTTON2_EXTI_PIN_SOURCE     USER_BUTTON2_GPIO_PIN 
#define	USER_BUTTON2_EXTI_IRQn	         EXTI15_10_IRQn
   
   
/**
 * @brief Definition for Serial COM1 connected to USART2
 */
#define EVAL_COM1                        USART2
#define EVAL_COM1_USART2                 1
#ifndef EVAL_COM1_BAUD_RATE
#define EVAL_COM1_BAUD_RATE              115200 //921600 is used only for throughput tests
#endif 
#define EVAL_COM1_CLK()                  __HAL_RCC_USART2_CLK_ENABLE()
#define EVAL_COM1_TX_PIN                 GPIO_PIN_2
#define EVAL_COM1_TX_GPIO_PORT           GPIOA
#define EVAL_COM1_TX_GPIO_CLK()          __HAL_RCC_GPIOA_CLK_ENABLE()
#define EVAL_COM1_TX_AF                  GPIO_AF7_USART2
#define EVAL_COM1_RX_PIN                 GPIO_PIN_3
#define EVAL_COM1_RX_GPIO_PORT           GPIOA
#define EVAL_COM1_RX_GPIO_CLK()          __HAL_RCC_GPIOA_CLK_ENABLE()
#define EVAL_COM1_RX_AF                  GPIO_AF7_USART2
#define EVAL_COM1_IRQn                   USART2_IRQn

#define SdkEval_IO_Uart_Irq_Handler      USART2_IRQHandler
  
   
#ifdef __cplusplus
}
#endif

#endif /* __USER_PLATFORM_CONFIGURATION_H */

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
