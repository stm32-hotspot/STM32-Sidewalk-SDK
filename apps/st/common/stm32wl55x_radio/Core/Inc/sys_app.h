/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    sys_app.h
  * @author  MCD Application Team
  * @brief   Function prototypes for sys_app.c file
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
#ifndef __SYS_APP_H__
#define __SYS_APP_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "sys_conf.h"
#include "stm32_adv_trace.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported defines ----------------------------------------------------------*/
/* USER CODE BEGIN ED */

/* USER CODE END ED */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define STM32WLxx_MCU_REV_Z (0x1001u) /* Revision ID of the die Rev Z */
#define STM32WLxx_MCU_REV_Y (0x1003u) /* Revision ID of the die Rev Y */
/* USER CODE END EC */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Exported macros -----------------------------------------------------------*/
#define APP_PPRINTF(...)  do{ } while( UTIL_ADV_TRACE_OK \
                              != UTIL_ADV_TRACE_COND_FSend(VLEVEL_ALWAYS, T_REG_OFF, TS_OFF, __VA_ARGS__) ) /* Polling Mode */
#define APP_TPRINTF(...)   do{ {UTIL_ADV_TRACE_COND_FSend(VLEVEL_ALWAYS, T_REG_OFF, TS_ON, __VA_ARGS__);} }while(0); /* with timestamp */
#define APP_PRINTF(...)   do{ {UTIL_ADV_TRACE_COND_FSend(VLEVEL_ALWAYS, T_REG_OFF, TS_OFF, __VA_ARGS__);} }while(0);

#if defined (APP_LOG_ENABLED) && (APP_LOG_ENABLED == 1)
#define APP_LOG(TS,VL,...)   do{ {UTIL_ADV_TRACE_COND_FSend(VL, T_REG_OFF, TS, __VA_ARGS__);} }while(0);
#elif defined (APP_LOG_ENABLED) && (APP_LOG_ENABLED == 0) /* APP_LOG disabled */
#define APP_LOG(TS,VL,...)
#else
#error "APP_LOG_ENABLED not defined or out of range <0,1>"
#endif /* APP_LOG_ENABLED */

/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief initialize the system (dbg pins, trace, mbmux, sys timer, LPM, ...)
  */
void SystemApp_Init(void);

/**
  * @brief  callback to get the battery level in % of full charge (254 full charge, 0 no charge)
  * @retval battery level
  */
uint8_t GetBatteryLevel(void);

/**
  * @brief  callback to get the current temperature in the MCU
  * @retval temperature level
  */
int16_t GetTemperatureLevel(void);

/**
  * @brief  callback to get the board 64 bits unique ID
  * @param  id unique ID
  */
void GetUniqueId(uint8_t *id);

/**
  * @brief  callback to get the board 32 bits unique ID (LSB)
  * @param  devAddr Device Address
  */
void GetDevAddr(uint32_t *devAddr);

/* USER CODE BEGIN EFP */
/**
 * Implements a busy wait delay safe to be used in SWI context.
 *
 * Due to busy wait implementation, should not be used for long durations.
 * If you need delay >1ms then use sid_pal_scheduler_delay_ms instead.
 *
 * @param[in]   delay          Time in us to delay
 *
 * @retval none
 */
void sid_pal_delay_us(uint32_t delay);

/**
 * Implements a delay function using RTOS API call.
 * This function will block the calling thread, do not use in ISR context.
 * This will allow the RTOS scheduler to run other tasks or switch to an
 * IDLE state while the delay is pending.
 *
 * If you need sub-millisecond delays then use sid_pal_delay_us instead.
 *
 * Do not use this function for internal Sidewalk stack delays - use the
 * sid_pal_timer API instead.
 *
 * @param[in]   delay          Time in ms to delay
 *
 * @retval none
 */
void sid_pal_scheduler_delay_ms(uint32_t delay);
/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __SYS_APP_H__ */
