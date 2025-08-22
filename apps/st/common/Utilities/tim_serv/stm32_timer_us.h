/**
  ******************************************************************************
  * @file    stm32_timer_us.h
  * @brief   Time server utility extension to support microsecond resolution
  * 
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef UTIL_TIME_SERVER_US_H__
#define UTIL_TIME_SERVER_US_H__

#ifdef __cplusplus

 extern "C" {
#endif

 /** @defgroup TIMER_SERVER timer server
   * @{
   */

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>   
#include <cmsis_compiler.h>
   
/* Exported types ------------------------------------------------------------*/
/** @defgroup TIMER_SERVER_exported_TypeDef TIMER_SERVER exported Typedef
  *  @{
  */

/**
  * @brief Timer driver definition
  */
typedef struct
{    
    uint64_t              (* us2Tick)(const uint64_t uSecond );    /*!< convert us to tick */
    uint64_t              (* Tick2us)(const uint64_t ticks );      /*!< convert tick into us */
} UTIL_TIMER_Driver_Us_s;


/**
  *  @}
  */

/* Exported variables ------------------------------------------------------------*/
/** @defgroup TIMER_SERVER_exported_Variable TIMER_SERVER exported Variable
  *  @{
  */
/**
 * @brief low layer interface to handle timing execution
 *
 * @remark This structure is defined and initialized in the specific platform
 *         timer implementation
 */
extern const UTIL_TIMER_Driver_Us_s UTIL_TimerDriverUs;

/**
  *  @}
  */

/* Exported constants --------------------------------------------------------*/
#define UTIL_TIMER_US_INFINITY (UINT64_MAX)
#define UTIL_TIMER_US_MAX_TICKS (UINT32_MAX)
/* External variables --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */ 

/** @defgroup TIMER_SERVER_exported_function TIMER_SERVER exported function
  *  @{
  */

/**
  * @brief Create the timer object
  *
  * @remark TimerSetValue function must be called before starting the timer.
  *         this function initializes timestamp and reload value at 0.
  *
  * @param TimerObject Structure containing the timer object parameters
  * @param PeriodValueUs Period value of the timer in us
  * @param Mode @ref UTIL_TIMER_Mode_t
  * @param Callback Function callback called at the end of the timeout
  * @param Argument argument for the callback function
  * @retval Status based on @ref UTIL_TIMER_Status_t
  */
UTIL_TIMER_Status_t UTIL_TIMER_CreateUs(UTIL_TIMER_Object_t * const TimerObject, const uint64_t PeriodValueUs, const UTIL_TIMER_Mode_t Mode, void ( * const Callback )( void *) , void * const Argument);

/**
  * @brief Start and adds the timer object to the list of timer events
  *
  * @param TimerObject Structure containing the timer object parameters
  * @param PeriodValueUs period value of the timer in us
  * @retval Status based on @ref UTIL_TIMER_Status_t
  */
UTIL_TIMER_Status_t UTIL_TIMER_StartWithPeriodUs(UTIL_TIMER_Object_t * const TimerObject, const uint64_t PeriodValueUs);

/**
  * @brief update the period and start the timer
  *
  * @param TimerObject Structure containing the timer object parameters
  * @param NewPeriodValueUs new period value of the timer in us
  * @retval Status based on @ref UTIL_TIMER_Status_t
  */
UTIL_TIMER_Status_t UTIL_TIMER_SetPeriodUs(UTIL_TIMER_Object_t * const TimerObject, const uint64_t NewPeriodValueUs);

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* UTIL_TIME_SERVER_US_H__*/

