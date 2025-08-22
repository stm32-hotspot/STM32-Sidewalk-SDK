/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023-2025 STMicroelectronics.
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
#include "stm32wbaxx_hal.h"
#include "app_conf.h"
#include "app_entry.h"
#include "app_common.h"
#include "app_debug.h"

#include "stm32wbaxx_ll_icache.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
#define FLASH_ECC_TEST_SINGLE_ERROR_ADDRESS (0x0BFA1F00u)
#define FLASH_ECC_TEST_DOUBLE_ERROR_ADDRESS (0x0BFA1F80u)
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
void Fatal_Error_Report_Handler(void);
void MX_GPIO_Init(void);
void MX_ICACHE_Init(void);
void MX_RAMCFG_Init(void);
void MX_RTC_Init(void);
#if (CFG_LOG_SUPPORTED)
void MX_USART1_UART_Init(void);
void MX_GPDMA1_Init(void);
#endif /* CFG_LOG_SUPPORTED */

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#if defined(NUCLEO_WBA52_BOARD) || defined(NUCLEO_WBA55_BOARD) || defined(NUCLEO_WBA65_BOARD)
#  define LED_GREEN_Pin                            (GPIO_PIN_11)
#  define LED_GREEN_GPIO_Port                      (GPIOB)
#  define LED_RED_Pin                              (GPIO_PIN_8)
#  define LED_RED_GPIO_Port                        (GPIOB)
#  define LED_BLUE_Pin                             (GPIO_PIN_4)
#  define LED_BLUE_GPIO_Port                       (GPIOB)
#endif /* NUCLEO_WBA52_BOARD || NUCLEO_WBA55_BOARD || NUCLEO_WBA65_BOARD */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
