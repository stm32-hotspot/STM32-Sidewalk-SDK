/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32wlxx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#ifdef FREERTOS
#  include "FreeRTOSConfig.h"
#endif
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
#if SIDEWALK_RADIO_USE_ARD_CONN_STACKING
#  define INTER_MCU_BUS_SPI1
#else
#  define INTER_MCU_BUS_SPI2
#endif /* SIDEWALK_RADIO_USE_ARD_CONN_STACKING */
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
#if defined(INTER_MCU_BUS_SPI1)
void MX_SPI1_Init(void);
#endif
#if defined(INTER_MCU_BUS_SPI2)
void MX_SPI2_Init(void);
#endif

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RTC_N_PREDIV_S 15
#define RTC_PREDIV_S ((1<<RTC_N_PREDIV_S)-1)
#define RTC_PREDIV_A ((1<<(15-RTC_N_PREDIV_S))-1)
#define LED1_Pin GPIO_PIN_15
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_9
#define LED2_GPIO_Port GPIOB
#define BUT1_Pin GPIO_PIN_0
#define BUT1_GPIO_Port GPIOA
#define BUT1_EXTI_IRQn EXTI0_IRQn
#define BUT3_Pin GPIO_PIN_6
#define BUT3_GPIO_Port GPIOC
#define BUT3_EXTI_IRQn EXTI9_5_IRQn
#define BUT2_Pin GPIO_PIN_1
#define BUT2_GPIO_Port GPIOA
#define BUT2_EXTI_IRQn EXTI1_IRQn
#define LED3_Pin GPIO_PIN_11
#define LED3_GPIO_Port GPIOB
#define USARTx_RX_Pin GPIO_PIN_3
#define USARTx_RX_GPIO_Port GPIOA
#define USARTx_TX_Pin GPIO_PIN_2
#define USARTx_TX_GPIO_Port GPIOA

#define APP_LOG_DMA_TX_CHANNEL_IRQn DMA1_Channel5_IRQn
#define APP_LOG_UART_IRQn           USART2_IRQn

#define SIDEWALK_RADIO_SPI_IRQ_EXTI_IRQn_PRIORITY configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY
#if defined(INTER_MCU_BUS_SPI1)
#define SIDEWALK_RADIO_SPI hspi1
#define SIDEWALK_RADIO_SPI_INSTANCE_BASE SPI1_BASE
#define SIDEWALK_RADIO_SPI_SCK_Pin GPIO_PIN_5
#define SIDEWALK_RADIO_SPI_SCK_GPIO_Port GPIOA
#define SIDEWALK_RADIO_SPI_MOSI_Pin GPIO_PIN_7
#define SIDEWALK_RADIO_SPI_MOSI_GPIO_Port GPIOA
#define SIDEWALK_RADIO_SPI_MISO_Pin GPIO_PIN_6
#define SIDEWALK_RADIO_SPI_MISO_GPIO_Port GPIOA
#define SIDEWALK_RADIO_SPI_NSS_Pin GPIO_PIN_4
#define SIDEWALK_RADIO_SPI_NSS_GPIO_Port GPIOA
#define SIDEWALK_RADIO_SPI_IRQ_Pin GPIO_PIN_2
#define SIDEWALK_RADIO_SPI_IRQ_GPIO_Port GPIOC
#define SIDEWALK_RADIO_SPI_IRQ_EXTI_IRQn EXTI2_IRQn
#define SIDEWALK_RADIO_SPI_DMA_TX hdma_spi1_tx
#define SIDEWALK_RADIO_SPI_DMA_RX hdma_spi1_rx
#elif defined(INTER_MCU_BUS_SPI2)
#define SIDEWALK_RADIO_SPI hspi2
#define SIDEWALK_RADIO_SPI_INSTANCE_BASE SPI2_BASE
#define SIDEWALK_RADIO_SPI_SCK_Pin GPIO_PIN_8
#define SIDEWALK_RADIO_SPI_SCK_GPIO_Port GPIOA
#define SIDEWALK_RADIO_SPI_MOSI_Pin GPIO_PIN_10
#define SIDEWALK_RADIO_SPI_MOSI_GPIO_Port GPIOA
#define SIDEWALK_RADIO_SPI_MISO_Pin GPIO_PIN_5
#define SIDEWALK_RADIO_SPI_MISO_GPIO_Port GPIOA
#define SIDEWALK_RADIO_SPI_NSS_Pin GPIO_PIN_9
#define SIDEWALK_RADIO_SPI_NSS_GPIO_Port GPIOA
#define SIDEWALK_RADIO_SPI_IRQ_Pin GPIO_PIN_4
#define SIDEWALK_RADIO_SPI_IRQ_GPIO_Port GPIOA
#define SIDEWALK_RADIO_SPI_IRQ_EXTI_IRQn EXTI4_IRQn
#define SIDEWALK_RADIO_SPI_DMA_TX hdma_spi2_tx
#define SIDEWALK_RADIO_SPI_DMA_RX hdma_spi2_rx
#else
#error "INTER_MCU_BUS_SPI1 or INTER_MCU_BUS_SPI2 should be defined"
#endif

#define SIDEWALK_RADIO_SPI_DMA_NSS_DETECT hdma_dma_generator0
#define SIDEWALK_RADIO_SPI_DMA_SCK_CTRL   hdma_dma_generator1

/* USER CODE BEGIN Private defines */

/**
 * User interaction
 * When CFG_LED_SUPPORTED is set, LEDS are activated if requested
 * When CFG_BUTTON_SUPPORTED is set, the push button are activated if requested
 * When CFG_DBG_SUPPORTED is set, the debugger is activated
 */

#if defined(NUCLEO_WL55_BOARD)
#  ifndef CFG_LED_SUPPORTED
#    define CFG_LED_SUPPORTED                    (1)
#  endif /* CFG_LED_SUPPORTED */
#  ifndef CFG_BUTTON_SUPPORTED
#    define CFG_BUTTON_SUPPORTED                 (1)
#endif /* CFG_BUTTON_SUPPORTED */
#  ifndef CFG_DBG_SUPPORTED
#    define CFG_DBG_SUPPORTED                    (1)
#  endif /* CFG_DBG_SUPPORTED */
#elif defined(GENERIC_WL55_BOARD)
# define CFG_LED_SUPPORTED                       (0)
# define CFG_BUTTON_SUPPORTED                    (0)
# define CFG_DBG_SUPPORTED                       (0)
#endif

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
