/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32_rtos.h
  * @author  MCD Application Team
  * @brief   Include file for all RTOS/Sequencer can be used on WBA
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024-2025 STMicroelectronics.
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
#ifndef STM32_RTOS_H
#define STM32_RTOS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <cmsis_os.h>
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* FreeRTOS priorities by default  */
#define TASK_PRIO_LINK_LAYER                    osPriorityISR
#define TASK_PRIO_TEMP_MEAS_LL                  osPriorityRealtime2 /* FIXME: does it really need that high priority? */
#define TASK_PRIO_AMM                           osPriorityBelowNormal
#define TASK_PRIO_RNG                           osPriorityHigh
#define TASK_PRIO_FLASH_MANAGER                 osPriorityBelowNormal
#define TASK_PRIO_BLE_HOST                      osPriorityNormal
#define TASK_PRIO_HCI_ASYNC_EVENT               osPriorityAboveNormal
#define TASK_PRIO_BLE_TIMER                     osPriorityNormal
#define TASK_PRIO_BPKA                          osPriorityNormal

/* USER CODE BEGIN TASK_Priority_Define */

/* USER CODE END TASK_Priority_Define */

#define RTOS_MAX_THREAD                         (20u)

#define RTOS_STACK_SIZE_LARGE                   ( 1024u * 3u )
#define RTOS_STACK_SIZE_MODERATE                ( 2048u )
#define RTOS_STACK_SIZE_NORMAL                  ( 1024u )
#define RTOS_STACK_SIZE_REDUCED                 ( 512u )
#define RTOS_STACK_SIZE_BELOW_REDUCED           ( 384u )
#define RTOS_STACK_SIZE_SMALL                   ( 256u )
#define RTOS_STACK_SIZE_TINY                    ( configMINIMAL_STACK_SIZE )

/* Tasks stack sizes by default  */
#define TASK_STACK_SIZE_LINK_LAYER              RTOS_STACK_SIZE_LARGE
#define TASK_STACK_SIZE_TEMP_MEAS_LL            RTOS_STACK_SIZE_SMALL
#define TASK_STACK_SIZE_AMM                     RTOS_STACK_SIZE_REDUCED
#define TASK_STACK_SIZE_RNG                     RTOS_STACK_SIZE_BELOW_REDUCED
#define TASK_STACK_SIZE_FLASH_MANAGER           (RTOS_STACK_SIZE_NORMAL + RTOS_STACK_SIZE_REDUCED)
#define TASK_STACK_SIZE_BLE_HOST                RTOS_STACK_SIZE_MODERATE
#define TASK_STACK_SIZE_HCI_ASYNC_EVENT         RTOS_STACK_SIZE_MODERATE
#define TASK_STACK_SIZE_BLE_TIMER               RTOS_STACK_SIZE_SMALL
#define TASK_STACK_SIZE_BPKA                    RTOS_STACK_SIZE_BELOW_REDUCED
/* USER CODE BEGIN TASK_Size_Define */

/* USER CODE END TASK_Size_Define */

/* Attributes needed by CMSIS */
#define TASK_DEFAULT_ATTR_BITS                  ( 0u )
#define TASK_DEFAULT_CB_MEM                     ( 0u )
#define TASK_DEFAULT_CB_SIZE                    ( 0u )
#define TASK_DEFAULT_STACK_MEM                  ( 0u )

#define SEMAPHORE_DEFAULT_ATTR_BITS             ( 0u )
#define SEMAPHORE_DEFAULT_CB_MEM                ( 0u )
#define SEMAPHORE_DEFAULT_CB_SIZE               ( 0u )

#define MUTEX_DEFAULT_ATTR_BITS                 ( 0u )
#define MUTEX_DEFAULT_CB_MEM                    ( 0u )
#define MUTEX_DEFAULT_CB_SIZE                   ( 0u )

#define QUEUE_DEFAULT_ATTR_BITS                 ( 0u )
#define QUEUE_DEFAULT_CB_MEM                    ( 0u )
#define QUEUE_DEFAULT_CB_SIZE                   ( 0u )
#define QUEUE_DEFAULT_MQ_MEM                    ( 0u )
#define QUEUE_DEFAULT_MQ_SIZE                   ( 0u )

/* USER CODE BEGIN Attributes_Define */
#define MUTEX_ATTR_BITS_LINK_LAYER              ( osMutexRecursive ) /* A recursive mutex is required for Sidewalk BLE driver to work properly */
/* USER CODE END Attributes_Define */

/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macros -----------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Exported functions prototypes ---------------------------------------------*/
/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif // STM32_RTOS_H
