/**
  ******************************************************************************
  * @file    stm32_timer_us.c
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

/* Includes ------------------------------------------------------------------*/
#include "stm32_timer.h"
#include "stm32_timer_us.h"
#include "stm32_timer_private.h"
#include <sid_stm32_common_utils.h>

/** @addtogroup TIMER_SERVER
  * @{
  */

/* Private macro -----------------------------------------------------------*/
/**
 * @defgroup TIMER_SERVER_private_macro TIMER_SERVER private macros
 *  @{
 */
 
/* Private variables -----------------------------------------------------------*/
/**
 * @defgroup TIMER_SERVER_private_varaible TIMER_SERVER private variable
 *  @{
 */

/**
  *  @}
  */

/* Functions Definition ------------------------------------------------------*/
/**
  * @addtogroup TIMER_SERVER_exported_function
  *  @{
  */
SID_STM32_SPEED_OPTIMIZED UTIL_TIMER_Status_t UTIL_TIMER_CreateUs(UTIL_TIMER_Object_t * const TimerObject, const uint64_t PeriodValueUs, const UTIL_TIMER_Mode_t Mode, void ( * const Callback )( void *) , void * const Argument)
{
  UTIL_TIMER_Status_t ret;

  UTIL_TIMER_ENTER_CRITICAL_SECTION();

  /* Create the timer using the default method */
  ret = UTIL_TIMER_Create(TimerObject, 0u, Mode, Callback, Argument);

  /* Update reload value if the timer was created successfully */
  if (UTIL_TIMER_OK == ret)
  {
    TimerObject->ReloadValue = (UTIL_TIMER_US_INFINITY == PeriodValueUs) ? UTIL_TIMER_US_MAX_TICKS : UTIL_TimerDriverUs.us2Tick(PeriodValueUs);
  }

  UTIL_TIMER_EXIT_CRITICAL_SECTION();

  return ret;
}

SID_STM32_SPEED_OPTIMIZED UTIL_TIMER_Status_t UTIL_TIMER_StartWithPeriodUs( UTIL_TIMER_Object_t * const TimerObject, const uint64_t PeriodValueUs)
{
  UTIL_TIMER_Status_t ret;

  UTIL_TIMER_ENTER_CRITICAL_SECTION();

  if(NULL == TimerObject)
  {
    ret = UTIL_TIMER_INVALID_PARAM;
  }
  else
  {
    if(UTIL_TIMER_IsRunning(TimerObject) != FALSE)
    {
      (void)UTIL_TIMER_Stop(TimerObject);
    }
    __COMPILER_BARRIER();
    TimerObject->ReloadValue = (UTIL_TIMER_US_INFINITY == PeriodValueUs) ? UTIL_TIMER_US_MAX_TICKS : UTIL_TimerDriverUs.us2Tick(PeriodValueUs);
    ret = UTIL_TIMER_Start(TimerObject);
  }

  UTIL_TIMER_EXIT_CRITICAL_SECTION();

  return ret;
}

SID_STM32_SPEED_OPTIMIZED UTIL_TIMER_Status_t UTIL_TIMER_SetPeriodUs(UTIL_TIMER_Object_t * const TimerObject, const uint64_t NewPeriodValueUs)
{
  UTIL_TIMER_Status_t ret = UTIL_TIMER_OK;

  UTIL_TIMER_ENTER_CRITICAL_SECTION();

  if(NULL == TimerObject)
  {
    ret = UTIL_TIMER_INVALID_PARAM;
  }
  else
  {
    uint32_t need_restart = FALSE;
    if(UTIL_TIMER_IsRunning(TimerObject) != FALSE)
    {
      (void)UTIL_TIMER_Stop(TimerObject);
      need_restart = TRUE;
    }
    TimerObject->ReloadValue = (UTIL_TIMER_US_INFINITY == NewPeriodValueUs) ? UTIL_TIMER_US_MAX_TICKS : UTIL_TimerDriverUs.us2Tick(NewPeriodValueUs);
    if(need_restart != FALSE)
    {
      ret = UTIL_TIMER_Start(TimerObject);
    }
  }

  UTIL_TIMER_EXIT_CRITICAL_SECTION();

  return ret;
}

/**
  *  @}
  */

/**
  *  @}
  */
