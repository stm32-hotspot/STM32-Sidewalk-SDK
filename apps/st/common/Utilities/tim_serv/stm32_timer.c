/*!
 * \file      timer.c
 *
 * \brief     Timer objects and scheduling management implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */

/******************************************************************************
 * @file    stm32_timer.c
 * @author  MCD Application Team
 * @brief   Time server utility
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32_timer.h"
#include "stm32_timer_private.h"
#include <sid_stm32_common_utils.h>
 
/** @addtogroup TIMER_SERVER
  * @{
  */

/* Private variables -----------------------------------------------------------*/
/**
 * @defgroup TIMER_SERVER_private_varaible TIMER_SERVER private variable
 *  @{
 */

/**
  * @brief Timers list head pointer
  *
  */
static UTIL_TIMER_Object_t *TimerListHead = NULL;

static uint32_t TimerInitCounter = 0u;

/**
  *  @}
  */

/**
 * @defgroup TIMER_SERVER_private_function TIMER_SERVER private function
 *  @{
 */

static inline void TimerInsertNewHeadTimer(UTIL_TIMER_Object_t * const TimerObject);
static inline void TimerInsertTimer(UTIL_TIMER_Object_t * const TimerObject);
static inline void TimerSetTimeout(UTIL_TIMER_Object_t * const TimerObject);

/**
  *  @}
  */

/* Functions Definition ------------------------------------------------------*/
/**
  * @addtogroup TIMER_SERVER_exported_function
  *  @{
  */

UTIL_TIMER_Status_t UTIL_TIMER_Init(void)
{
  UTIL_TIMER_Status_t status;

  UTIL_TIMER_INIT_CRITICAL_SECTION();

  UTIL_TIMER_ENTER_CRITICAL_SECTION();

  if (0u == TimerInitCounter)
  {
    /* Timer is not initialized yet */
    TimerListHead = NULL;
    status = UTIL_TimerDriver.InitTimer();
    if (UTIL_TIMER_OK == status)
    {
      /* Set initial timer context on the first initialization */
      (void)UTIL_TimerDriver.SetTimerContext();
    }
  }
  else
  {
    /* Timer is initialized already */
    status = UTIL_TIMER_OK;
  }

  /* Increment timer reference counter if initialization is successful */
  if (UTIL_TIMER_OK == status)
  {
    TimerInitCounter++;
  }

  UTIL_TIMER_EXIT_CRITICAL_SECTION();

  return status;
}

UTIL_TIMER_Status_t UTIL_TIMER_DeInit(void)
{
  UTIL_TIMER_Status_t status;

  UTIL_TIMER_ENTER_CRITICAL_SECTION();

  if (0u == TimerInitCounter)
  {
    /* Timer is de-initialized already */
    status = UTIL_TIMER_OK;
  }
  else if (1u == TimerInitCounter)
  {
    status = UTIL_TimerDriver.DeInitTimer();
    if (UTIL_TIMER_OK == status)
    {
      /* Timer was de-initialized successfully */
      TimerInitCounter = 0u;
    }
  }
  else /* if (TimerInitCounter > 1u) */
  {
    /* There's more than one user of the timer - just decrement the reference counter */
    TimerInitCounter--;
    status = UTIL_TIMER_OK;
  }

  UTIL_TIMER_EXIT_CRITICAL_SECTION();

  return status;
}

SID_STM32_SPEED_OPTIMIZED UTIL_TIMER_Status_t UTIL_TIMER_Create(UTIL_TIMER_Object_t * const TimerObject, const uint32_t PeriodValue, const UTIL_TIMER_Mode_t Mode, void ( * const Callback )( void *) , void * const Argument)
{
  UTIL_TIMER_Status_t ret;

  UTIL_TIMER_ENTER_CRITICAL_SECTION();

  if((TimerObject != NULL) && (Callback != NULL) && ( TimerExists( TimerObject ) == FALSE ))
  {
    TimerObject->Timestamp = 0U;
    TimerObject->ReloadValue = UTIL_TimerDriver.ms2Tick(PeriodValue);
    TimerObject->IsPending = FALSE;
    TimerObject->IsRunning = FALSE;
    TimerObject->IsReloadStopped = TRUE;
    TimerObject->Callback = Callback;
    TimerObject->argument = Argument;
    TimerObject->Mode = Mode;
    TimerObject->Next = NULL;
    ret = UTIL_TIMER_OK;
  }
  else
  {
    ret = UTIL_TIMER_INVALID_PARAM;
  }

  UTIL_TIMER_EXIT_CRITICAL_SECTION();

  return ret;
}

SID_STM32_SPEED_OPTIMIZED UTIL_TIMER_Status_t UTIL_TIMER_Start(UTIL_TIMER_Object_t * const TimerObject)
{
  UTIL_TIMER_Status_t ret = UTIL_TIMER_OK;
  register uint32_t elapsedTime;
  register uint32_t minValue;
  register uint32_t ticks;

  UTIL_TIMER_ENTER_CRITICAL_SECTION();

  if(( TimerObject != NULL ) && ( TimerExists( TimerObject ) == FALSE ) && (UTIL_TIMER_IsRunning(TimerObject) == FALSE))
  {
    ticks = TimerObject->ReloadValue;
    minValue = UTIL_TimerDriver.GetMinimumTimeout( );

    if( ticks < minValue )
    {
      ticks = minValue;
    }

    TimerObject->Timestamp = ticks;
    TimerObject->IsPending = FALSE;
    TimerObject->IsRunning = TRUE;
    if (UTIL_TIMER_ONESHOT == TimerObject->Mode)
    {
      TimerObject->IsReloadStopped = TRUE;
    }
    else
    {
      TimerObject->IsReloadStopped = FALSE;
    }

    /* Do not update timer context from here because this function may be called from anywhere, including a timer callback
     * invoked in the timer IRQ context while the timer list update is ongoing. Modifying the context here will affect the
     * other timers and will ruin the scheduling.
     */
    elapsedTime = UTIL_TimerDriver.GetTimerElapsedTime( );
    TimerObject->Timestamp += elapsedTime;

    if (( TimerListHead == NULL ) || ( TimerObject->Timestamp < TimerListHead->Timestamp ))
    {
      TimerInsertNewHeadTimer( TimerObject);
    }
    else
    {
      TimerInsertTimer( TimerObject);
    }
  }
  else
  {
    ret =  UTIL_TIMER_INVALID_PARAM;
  }

  UTIL_TIMER_EXIT_CRITICAL_SECTION();

  return ret;
}

SID_STM32_SPEED_OPTIMIZED UTIL_TIMER_Status_t UTIL_TIMER_StartWithPeriod(UTIL_TIMER_Object_t * const TimerObject, const uint32_t PeriodValue)
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
    TimerObject->ReloadValue = UTIL_TimerDriver.ms2Tick(PeriodValue);
    ret = UTIL_TIMER_Start(TimerObject);
  }

  UTIL_TIMER_EXIT_CRITICAL_SECTION();

  return ret;
}

SID_STM32_SPEED_OPTIMIZED UTIL_TIMER_Status_t UTIL_TIMER_Stop(UTIL_TIMER_Object_t * const TimerObject)
{
  UTIL_TIMER_Status_t ret;

  UTIL_TIMER_ENTER_CRITICAL_SECTION();

  if (TimerObject != NULL)
  {
    TimerObject->IsReloadStopped = TRUE;
    TimerObject->IsRunning = FALSE;

    if (TimerExists(TimerObject) != FALSE)
    {
      UTIL_TIMER_Object_t* prev = TimerListHead;
      UTIL_TIMER_Object_t* cur = TimerListHead;

      if( TimerObject == TimerListHead ) /* Stop the Head */
      {
        TimerListHead->IsPending = FALSE;
        if( TimerListHead->Next != NULL )
        {
          TimerListHead = TimerListHead->Next;
          TimerSetTimeout( TimerListHead );
        }
        else
        {
          UTIL_TimerDriver.StopTimerEvt( );
          TimerListHead = NULL;
        }
      }
      else /* Stop an object within the list */
      {
        while( cur != NULL )
        {
          if( TimerObject == cur )
          {
            cur = cur->Next;
            prev->Next = cur;
            break;
          }
          else
          {
            prev = cur;
            cur = cur->Next;
          }
        }
      }
    }

    ret = UTIL_TIMER_OK;
  }
  else
  {
    /* List is empty or the Obj to stop does not exist */
    ret = UTIL_TIMER_INVALID_PARAM;
  }

  UTIL_TIMER_EXIT_CRITICAL_SECTION();

  return ret;
}

SID_STM32_SPEED_OPTIMIZED UTIL_TIMER_Status_t UTIL_TIMER_SetPeriod(UTIL_TIMER_Object_t * const TimerObject, const uint32_t NewPeriodValue)
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
    TimerObject->ReloadValue = UTIL_TimerDriver.ms2Tick(NewPeriodValue);
    if(need_restart != FALSE)
    {
      ret = UTIL_TIMER_Start(TimerObject);
    }
  }

  UTIL_TIMER_EXIT_CRITICAL_SECTION();

  return ret;
}

SID_STM32_SPEED_OPTIMIZED UTIL_TIMER_Status_t UTIL_TIMER_SetReloadMode(UTIL_TIMER_Object_t * const TimerObject, const UTIL_TIMER_Mode_t ReloadMode)
{
  UTIL_TIMER_Status_t ret = UTIL_TIMER_OK;

  UTIL_TIMER_ENTER_CRITICAL_SECTION();

  if(NULL == TimerObject)
  {
    ret = UTIL_TIMER_INVALID_PARAM;
  }
  else
  {
    TimerObject->Mode = ReloadMode;
  }

  UTIL_TIMER_EXIT_CRITICAL_SECTION();

  return ret;
}

SID_STM32_SPEED_OPTIMIZED UTIL_TIMER_Status_t UTIL_TIMER_GetRemainingTime(UTIL_TIMER_Object_t *TimerObject, uint32_t *ElapsedTime)
{
  UTIL_TIMER_Status_t ret = UTIL_TIMER_OK;
  if(TimerExists(TimerObject))
  {
    uint32_t time = UTIL_TimerDriver.GetTimerElapsedTime();
    if (TimerObject->Timestamp < time )
    {
      *ElapsedTime = 0u;
    }
    else
    {
      *ElapsedTime = TimerObject->Timestamp - time;
    }
  }
  else
  {
    ret = UTIL_TIMER_INVALID_PARAM;
  }
  return ret;
}

SID_STM32_SPEED_OPTIMIZED uint32_t UTIL_TIMER_IsRunning(const UTIL_TIMER_Object_t * const TimerObject)
{
  register uint32_t IsRunning;

  UTIL_TIMER_ENTER_CRITICAL_SECTION();

  if( TimerObject != NULL )
  {
    if (UTIL_TIMER_ONESHOT == TimerObject->Mode)
    {
      IsRunning = TimerObject->IsRunning;
    }
    else
    {
      /* For periodic timers TimerObject->IsReloadStopped is the source of the state. TimerObject->IsRunning may
       * intermittently indicate FALSE during timer IRQ processing
       */
      IsRunning = TimerObject->IsReloadStopped != FALSE ? FALSE : TRUE;
    }
  }
  else
  {
    IsRunning = FALSE;
  }

  UTIL_TIMER_EXIT_CRITICAL_SECTION();

  return IsRunning;
}

SID_STM32_SPEED_OPTIMIZED uint32_t UTIL_TIMER_GetFirstRemainingTime(void)
{
  uint32_t NextTimer = 0xFFFFFFFFU;

  UTIL_TIMER_ENTER_CRITICAL_SECTION();

  if(TimerListHead != NULL)
  {
    (void)UTIL_TIMER_GetRemainingTime(TimerListHead, &NextTimer);
  }

  UTIL_TIMER_EXIT_CRITICAL_SECTION();

  return NextTimer;
}

SID_STM32_SPEED_OPTIMIZED void UTIL_TIMER_IRQ_Handler( void )
{
  UTIL_TIMER_ENTER_CRITICAL_SECTION();

  /* Update timer context ASAP */
  register const uint32_t old = UTIL_TimerDriver.GetTimerContext( );
  register const uint32_t now = UTIL_TimerDriver.SetTimerContext( );
  __COMPILER_BARRIER();

  register const uint32_t minTicks = UTIL_TimerDriver.GetMinimumTimeout( );
  register const uint32_t deltaContext = now - old; /*intentional wrap around */

  register uint32_t firstNonexpiredFound = FALSE;

  /* update timeStamp based upon new Time Reference*/
  /* because delta context should never exceed 2^32*/
  UTIL_TIMER_Object_t * cur = TimerListHead;
  UTIL_TIMER_Object_t * invokeListHead = NULL;
  UTIL_TIMER_Object_t * invokeListTail;
  while (cur != NULL)
  {
    /* Store the link to the next element of the timer list since current element may be edited in here */
    UTIL_TIMER_Object_t * const next_timer = cur->Next;

    if ((firstNonexpiredFound != FALSE) || (cur->Timestamp > (deltaContext + (minTicks >> 1))))
    {
      /* If the expiration time is still in the future or if rescheduling the timer in minTicks
       * will give lower deviation than invoking the callback now - just update the timestamp */
      cur->Timestamp -= deltaContext;
      firstNonexpiredFound = TRUE;
    }
    else
    {
      /* Timer has expired */
      cur->Timestamp = 0u;

      /* Since timer list is ordered, this implicitly means cur is always pointing to the TimerListHead when the code gets here.
       * In other words, we are always dropping the TimerListHead and adding it to the invocation list until the first non-expired
       * timer is found
       */
      TimerListHead = cur->Next; /* Advance the head */
      cur->Next = NULL; /* Cut ties with the head */

      /* Update timer status */
      cur->IsPending = FALSE;
      cur->IsRunning = FALSE;

      /* Add the timer to the invocation list */
      if (NULL == invokeListHead)
      {
        /* Init invocation list head */
        invokeListHead = cur;
        invokeListTail = cur;
      }
      else
      {
        /* Head is initialized already, insert new timer to the tail of the list */
        invokeListTail->Next = cur;
        invokeListTail = cur;
      }
    }

    /* Proceed to the next timer */
    cur = next_timer;
  }

  /* Now the timer list is updated with all new timestamps and invocation list is populated with the expired timer pending callback invocation */

  /* Call user callbacks. Do it within the critical section since user callbacks may reorganize timer list */
  cur = invokeListHead;
  while (cur != NULL)
  {
    /* Store the link to the next element of the timer list since current element may be edited by user callback */
    UTIL_TIMER_Object_t * const next_timer = cur->Next;

    /* Invoke user callback */
    if (cur->Callback != NULL)
    {
      cur->Callback(cur->argument);
    }

    /* Restart timer if needed and not (re)started already within the user callback */
    if ((UTIL_TIMER_PERIODIC == cur->Mode) && (FALSE == cur->IsRunning) && (FALSE == cur->IsReloadStopped))
    {
      cur->IsReloadStopped = TRUE; /* Set stop flag to allow UTIL_TIMER_Start() restart ther timer */
      (void)UTIL_TIMER_Start(cur);
    }

    /* Proceed to the next timer */
    cur = next_timer;
  }

  /* Unconditionally restart list head */
  if (TimerListHead != NULL)
  {
    TimerSetTimeout( TimerListHead );
  }

  UTIL_TIMER_EXIT_CRITICAL_SECTION();
}

SID_STM32_SPEED_OPTIMIZED UTIL_TIMER_Time_t UTIL_TIMER_GetCurrentTime(void)
{
  register const uint32_t nowInTicks = UTIL_TimerDriver.GetTimerValue( );
  register const uint32_t now = UTIL_TimerDriver.Tick2ms(nowInTicks);
  return now;
}

SID_STM32_SPEED_OPTIMIZED UTIL_TIMER_Time_t UTIL_TIMER_GetElapsedTime(const UTIL_TIMER_Time_t past)
{
  register const uint32_t nowInTicks = UTIL_TimerDriver.GetTimerValue( );
  register const uint32_t pastInTicks = UTIL_TimerDriver.ms2Tick( past );
  /* intentional wrap around. Works Ok if tick duration below 1ms */
  register const uint32_t elapsedTime = UTIL_TimerDriver.Tick2ms( nowInTicks - pastInTicks );
  return elapsedTime;
}

SID_STM32_SPEED_OPTIMIZED const UTIL_TIMER_Object_t * UTIL_TIMER_GetTimerList(void)
{
  return TimerListHead;
}

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
SID_STM32_SPEED_OPTIMIZED uint32_t TimerExists(const UTIL_TIMER_Object_t * const TimerObject)
{
  UTIL_TIMER_ENTER_CRITICAL_SECTION();

  uint32_t exists = FALSE;
  const UTIL_TIMER_Object_t * cur = TimerListHead;

  while( cur != NULL )
  {
    if( TimerObject == cur )
    {
      exists = TRUE;
      break;
    }
    cur = cur->Next;
  }

  UTIL_TIMER_EXIT_CRITICAL_SECTION();

  return exists;
}

/**
 * @brief Sets a timeout with the duration "timestamp"
 *
 * @param TimerObject Structure containing the timer object parameters
 */
SID_STM32_SPEED_OPTIMIZED static inline void TimerSetTimeout(UTIL_TIMER_Object_t * const TimerObject)
{
  register uint32_t minTicks = UTIL_TimerDriver.GetMinimumTimeout( );
  register uint32_t elapsedTime = UTIL_TimerDriver.GetTimerElapsedTime( );
  TimerObject->IsPending = TRUE;

  /* In case deadline too soon */
  if(TimerObject->Timestamp  < (elapsedTime + minTicks) )
  {
    TimerObject->Timestamp = elapsedTime + minTicks;
  }
  UTIL_TimerDriver.StartTimerEvt( TimerObject->Timestamp );
}

/**
 * @brief Adds a timer to the list.
 *
 * @remark The list is automatically sorted. The list head always contains the
 *     next timer to expire.
 *
 * @param TimerObject Structure containing the timer object parameters
 */
SID_STM32_SPEED_OPTIMIZED static inline void TimerInsertTimer(UTIL_TIMER_Object_t * const TimerObject)
{
  UTIL_TIMER_Object_t* cur = TimerListHead;
  UTIL_TIMER_Object_t* next = TimerListHead->Next;

  while (cur->Next != NULL )
  {  
    if( TimerObject->Timestamp  > next->Timestamp )
    {
        cur = next;
        next = next->Next;
    }
    else
    {
        cur->Next = TimerObject;
        TimerObject->Next = next;
        return;

    }
  }
  cur->Next = TimerObject;
  TimerObject->Next = NULL;
}

/**
 * @brief Adds or replace the head timer of the list.
 *
 * @param TimerObject Structure containing the timer object parameters
 *
 * @remark The list is automatically sorted. The list head always contains the
 *         next timer to expire.
 */
SID_STM32_SPEED_OPTIMIZED static inline void TimerInsertNewHeadTimer(UTIL_TIMER_Object_t * const TimerObject)
{
  UTIL_TIMER_Object_t* cur = TimerListHead;

  if( cur != NULL )
  {
    cur->IsPending = FALSE;
  }

  TimerObject->Next = cur;
  TimerListHead = TimerObject;
  TimerSetTimeout( TimerListHead );
}

/**
  *  @}
  */

/**
  *  @}
  */
