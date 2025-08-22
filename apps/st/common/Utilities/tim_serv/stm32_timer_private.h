/**
  ******************************************************************************
  * @file           : stm32_timer_private.h
  * @brief          : Private APIs and definitions of the time server utility
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#ifndef UTIL_TIME_SERVER_PRIVATE_H__
#define UTIL_TIME_SERVER_PRIVATE_H__

/* Includes ------------------------------------------------------------------*/

#include "stm32_timer.h"
#include "utilities_conf.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup TIMER_SERVER
  * @{
  */

/* Private macro -----------------------------------------------------------*/
/**
 * @defgroup TIMER_SERVER_private_macro TIMER_SERVER private macros
 *  @{
 */
/**
  * @brief macro definition to initialize a critical section.
  *
  */
#ifndef UTIL_TIMER_INIT_CRITICAL_SECTION
  #define UTIL_TIMER_INIT_CRITICAL_SECTION( )
#endif

/**
  * @brief macro definition to enter a critical section.
  *
  */
#ifndef UTIL_TIMER_ENTER_CRITICAL_SECTION
  #define UTIL_TIMER_ENTER_CRITICAL_SECTION( )   UTILS_ENTER_CRITICAL_SECTION( )
#endif

/**
  * @brief macro definition to exit a critical section.
  *
  */
#ifndef UTIL_TIMER_EXIT_CRITICAL_SECTION
  #define UTIL_TIMER_EXIT_CRITICAL_SECTION( )    UTILS_EXIT_CRITICAL_SECTION( )
#endif
/**
  *  @}
  */

/**************************** Private functions *******************************/

/**
  *  @addtogroup TIMER_SERVER_private_function
  *
  *  @{
  */
/**
 * @brief Check if the Object to be added is not already in the list
 *
 * @param TimerObject Structure containing the timer object parameters
 * @retval 1 (the object is already in the list) or 0
 */
uint32_t TimerExists(const UTIL_TIMER_Object_t * const TimerObject);

/**
  *  @}
  */

/**
  *  @}
  */

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* UTIL_TIME_SERVER_PRIVATE_H__ */
