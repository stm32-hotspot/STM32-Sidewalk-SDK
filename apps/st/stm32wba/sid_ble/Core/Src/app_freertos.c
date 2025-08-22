/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022-2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "app_conf.h"
#include "app_freertos.h"
#include "timer_if.h"
#include <sid_pal_log_ifc.h>
#include <sid_stm32_common_utils.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
#define portNVIC_SYSTICK_CTRL_REG             ( *( ( volatile uint32_t * ) 0xe000e010 ) )
#define portNVIC_SYSTICK_LOAD_REG             ( *( ( volatile uint32_t * ) 0xe000e014 ) )
#define portNVIC_SYSTICK_CURRENT_VALUE_REG    ( *( ( volatile uint32_t * ) 0xe000e018 ) )
#define portNVIC_SYSTICK_ENABLE_BIT           ( 1UL << 0UL )
#define portNVIC_SYSTICK_INT_BIT              ( 1UL << 1UL )
#define portNVIC_SYSTICK_CLK_BIT              ( 1UL << 2UL )
#define portMISSED_COUNTS_FACTOR              ( 94UL )
#ifndef configSYSTICK_CLOCK_HZ
  #define configSYSTICK_CLOCK_HZ              ( configCPU_CLOCK_HZ )
  #define portNVIC_SYSTICK_CLK_BIT_CONFIG     ( portNVIC_SYSTICK_CLK_BIT )
#else
  #define portNVIC_SYSTICK_CLK_BIT_CONFIG     ( 0 )
#endif
#define CORE_TICK_RATE                        (( configSYSTICK_CLOCK_HZ ) / ( configTICK_RATE_HZ ))

/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if (STACK_USAGE_DIAGNOSTICS_ENABLED)
volatile uint32_t userStackFreeSpaceMinimum = UINT32_MAX;
#endif /* STACK_USAGE_DIAGNOSTICS_ENABLED */

#if (HEAP_USAGE_DIAGNOSTICS_ENABLED)
volatile uint32_t userHeapFreeSpaceMinimum = UINT32_MAX;
#endif /* HEAP_USAGE_DIAGNOSTICS_ENABLED */

#if (SYS_INFO_DIAGNOSTICS_ENABLED)
#define TASK_STATUS_ARRAY_SIZE  25
volatile TaskStatus_t pxTaskStatusArray[TASK_STATUS_ARRAY_SIZE];
volatile UBaseType_t uxArraySize = TASK_STATUS_ARRAY_SIZE;
unsigned long ulTotalRunTime, ulStatsAsPercentage;
#endif /* SYS_INFO_DIAGNOSTICS_ENABLED */

/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* USER CODE BEGIN 4 */
#if (configCHECK_FOR_STACK_OVERFLOW > 0)
void vApplicationStackOverflowHook(TaskHandle_t xTask, char * pcTaskName)
{
  __disable_irq();
  SID_PAL_LOG_ERROR("Stack overflow in %s thread", pcTaskName);
  Error_Handler();
}
#endif /* (configCHECK_FOR_STACK_OVERFLOW > 0) */
/* USER CODE END 4 */

/* USER CODE BEGIN 1 */
#if configUSE_IDLE_HOOK
void vApplicationIdleHook( void )
{
#  if (SYS_INFO_DIAGNOSTICS_ENABLED)
  uxArraySize = uxTaskGetNumberOfTasks();
    /* Generate raw status information about each task. */
  uxArraySize = uxTaskGetSystemState( pxTaskStatusArray,
                              uxArraySize,
                              &ulTotalRunTime );
#  endif /* SYS_INFO_DIAGNOSTICS_ENABLED */

#  if (STACK_USAGE_DIAGNOSTICS_ENABLED)
#    define USER_STACK_GUARD_WATERMARK (0xDEADBEEFu)
#    define USER_STACK_FREE_RAM_WATERMARK (0xAA55AA55u)
  uint32_t * stackGuardPtr = (uint32_t *)((void *)0 + APP_CONFIG_USER_STACK_GUARD_START);
  const uint32_t * const stackGuardEnd = (uint32_t *)((void *)0 + APP_CONFIG_USER_STACK_GUARD_END);
  while(stackGuardPtr < stackGuardEnd)
  {
    const uint32_t guardData = *stackGuardPtr;
    if (guardData != USER_STACK_GUARD_WATERMARK)
    {
      /* Stack underflow detected */
      Error_Handler();
    }
    stackGuardPtr++;
  }

  uint32_t * stackPtr = (uint32_t *)((void *)0 + APP_CONFIG_USER_STACK_START);
  const uint32_t * const stackEnd = (uint32_t *)((void *)0 + APP_CONFIG_USER_STACK_END);
  uint32_t stackFreeSpace = 0u;
  while(stackPtr < stackEnd)
  {
    const uint32_t stackData = *stackPtr;
    if (stackData == USER_STACK_FREE_RAM_WATERMARK)
    {
      stackFreeSpace += sizeof(stackData);
      stackPtr++;
    }
    else
    {
      break;
    }
  }

  /* Update stack free space minimums */
  if (stackFreeSpace <= userStackFreeSpaceMinimum)
  {
    userStackFreeSpaceMinimum = stackFreeSpace;
  }
#  endif /* STACK_USAGE_DIAGNOSTICS_ENABLED */

#  if (HEAP_USAGE_DIAGNOSTICS_ENABLED)
#    define USER_HEAP_FREE_RAM_WATERMARK (0x55AA55AAu)
  uint32_t * heapPtr = (uint32_t *)((void *)0 + APP_CONFIG_USER_HEAP_END/* Address of the first byte after the heap */ - sizeof(uint32_t)); 
  const uint32_t * const heapStart = (uint32_t *)((void *)0 + APP_CONFIG_USER_HEAP_START);
  uint32_t heapFreeSpace = 0u;
  while(heapPtr >= heapStart)
  {
    const uint32_t heapData = *heapPtr;
    if (heapData == USER_HEAP_FREE_RAM_WATERMARK)
    {
      heapFreeSpace += sizeof(heapData);
      heapPtr--;
    }
    else
    {
      break;
    }
  }

  /* Update stack free space minimums */
  if (heapFreeSpace <= userHeapFreeSpaceMinimum)
  {
    userHeapFreeSpaceMinimum = heapFreeSpace;
  }
#  endif /* HEAP_USAGE_DIAGNOSTICS_ENABLED */
}
#endif /* configUSE_IDLE_HOOK */
/* USER CODE END 1 */

/* USER CODE BEGIN VPORT_SUPPORT_TICKS_AND_SLEEP */
#if ( CFG_LPM_LEVEL != 0)
/* return current time since boot, continue to count in standby low power mode */
SID_STM32_SPEED_OPTIMIZED static uint32_t getCurrentTime(void)
{
  return TIMER_IF_GetTimerValue();
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED uint32_t checkExpectedRtcIdleTime(uint32_t ulExpectedIdleTimeIn)
{
  uint32_t ulExpectedIdleTimeOut;
  const uint32_t ulSleepThresholdMs = portTICK_PERIOD_MS * configEXPECTED_IDLE_TIME_BEFORE_SLEEP;
  const uint32_t ulNextRtcEventMs = TIMER_IF_Convert_Tick2ms(UTIL_TIMER_GetFirstRemainingTime());
  uint32_t ulNextBleEventMs = UINT32_MAX;

  if (ll_sys_is_initialized() != FALSE)
  {
    ble_stat_t cmd_status;

    cmd_status = ll_intf_le_get_remaining_time_for_next_event(&ulNextBleEventMs);
    UNUSED(cmd_status);
    assert_param(cmd_status == SUCCESS);

    ulNextBleEventMs /= 1000u; /* LL report stime in microseconds */
  }

  if ((ulNextRtcEventMs < ulSleepThresholdMs) || (ulNextBleEventMs < ulSleepThresholdMs))
  {
    /* Next RTC or 2.4GHz radio event is too close, skip entering LPM */
    ulExpectedIdleTimeOut = 0u;
  }
  else
  {
    /* Next RTC event is far enough, enter the planned LPM */
    ulExpectedIdleTimeOut = ulExpectedIdleTimeIn;
  }

  return ulExpectedIdleTimeOut;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED void vPortSuppressTicksAndSleep(uint32_t xExpectedIdleTime)
{
  uint32_t ulReloadValue, ulStoppedTimerCompensation;
  uint32_t lowPowerTimeBeforeSleep, lowPowerTimeAfterSleep, lowPowerStepTicks;
  uint32_t expectedSleepDurationMs;
  eSleepModeStatus eSleepStatus;

  /* Enter a critical section but don't use the taskENTER_CRITICAL()
  * method as that will mask interrupts that should exit sleep mode. */
  __asm volatile ( "cpsid i" ::: "memory" );
  __asm volatile ( "dsb" );
  __asm volatile ( "isb" );
  __COMPILER_BARRIER();

  /* Stop the SysTick momentarily.  The time the SysTick is stopped for
  is accounted for as best it can be, but using the tickless mode will
  inevitably result in some tiny drift of the time maintained by the
  kernel with respect to calendar time. */
  portNVIC_SYSTICK_CTRL_REG = ( portNVIC_SYSTICK_CLK_BIT_CONFIG | portNVIC_SYSTICK_INT_BIT );
  __COMPILER_BARRIER();

  eSleepStatus = eTaskConfirmSleepModeStatus();
  /* If a context switch is pending or a task is waiting for the scheduler
   * to be unsuspended then abandon the low power entry. */
  if( eSleepStatus == eAbortSleep )
  {
    /* Restart from whatever is left in the count register to complete
     * this tick period. */
    portNVIC_SYSTICK_LOAD_REG = portNVIC_SYSTICK_CURRENT_VALUE_REG;

    /* Restart SysTick so it runs from portNVIC_SYSTICK_LOAD_REG again,
     * then set portNVIC_SYSTICK_LOAD_REG back to its standard value.  If
     * the SysTick is not using the core clock, temporarily configure it to
     * use the core clock.  This configuration forces the SysTick to load
     * from portNVIC_SYSTICK_LOAD_REG immediately instead of at the next
     * cycle of the other clock.  Then portNVIC_SYSTICK_LOAD_REG is ready
     * to receive the standard value immediately. */
    portNVIC_SYSTICK_CURRENT_VALUE_REG = 0UL;
    portNVIC_SYSTICK_CTRL_REG = portNVIC_SYSTICK_CLK_BIT | portNVIC_SYSTICK_INT_BIT | portNVIC_SYSTICK_ENABLE_BIT;
#if ( portNVIC_SYSTICK_CLK_BIT_CONFIG == portNVIC_SYSTICK_CLK_BIT )
    portNVIC_SYSTICK_LOAD_REG = CORE_TICK_RATE - 1UL;
#else
    /* The temporary usage of the core clock has served its purpose,
     * as described above.  Resume usage of the other clock. */
    portNVIC_SYSTICK_CTRL_REG = portNVIC_SYSTICK_CLK_BIT | portNVIC_SYSTICK_INT_BIT;
    if( ( portNVIC_SYSTICK_CTRL_REG & portNVIC_SYSTICK_COUNT_FLAG_BIT ) != 0 )
    {
      /* The partial tick period already ended.  Be sure the SysTick
       * counts it only once. */
      portNVIC_SYSTICK_CURRENT_VALUE_REG = 0;
    }
    portNVIC_SYSTICK_LOAD_REG = CORE_TICK_RATE - 1UL;
    portNVIC_SYSTICK_CTRL_REG = portNVIC_SYSTICK_CLK_BIT_CONFIG | portNVIC_SYSTICK_INT_BIT | portNVIC_SYSTICK_ENABLE_BIT;
#endif /* portNVIC_SYSTICK_CLK_BIT_CONFIG */

    /* Re-enable interrupts - see comments above the cpsid instruction above. */
    __asm volatile ( "cpsie i" ::: "memory" );
  }
  else
  {
    /* Read the current time from RTC, maintained in standby */
    lowPowerTimeBeforeSleep = getCurrentTime();
    if( eSleepStatus == eNoTasksWaitingTimeout )
    {
      /* It is not necessary to configure an interrupt to bring the
        microcontroller out of its low power state at a fixed time in the
        future. */
      expectedSleepDurationMs = APPE_OS_SLEEP_NO_WAKEUP;
    }
    else
    {
      /* Configure an interrupt to bring the microcontroller out of its low
        power state at the time the kernel next needs to execute. The
        interrupt must be generated from a source that remains operational
        when the microcontroller is in a low power state. */
      expectedSleepDurationMs = (xExpectedIdleTime - 1u) * portTICK_PERIOD_MS;
    }
    /* Enter the low power state. */
    MX_APPE_EnterLPM(expectedSleepDurationMs); /* WFI instruction call is inside this API */
    MX_APPE_LeaveLPM();
    /* Determine how long the microcontroller was actually in a low power
      state for, which will be less than xExpectedIdleTime if the
      microcontroller was brought out of low power mode by an interrupt
      other than that configured by the vSetWakeTimeInterrupt() call.
      Note that the scheduler is suspended before
      portSUPPRESS_TICKS_AND_SLEEP() is called, and resumed when
      portSUPPRESS_TICKS_AND_SLEEP() returns.  Therefore no other tasks will
      execute until this function completes. */
    lowPowerTimeAfterSleep = getCurrentTime();
    lowPowerStepTicks = TIMER_IF_Convert_Tick2ms(lowPowerTimeAfterSleep - lowPowerTimeBeforeSleep) / portTICK_PERIOD_MS;

    /* Restart from whatever is left in the count register to complete
    this tick period. */
    ulReloadValue = portNVIC_SYSTICK_CURRENT_VALUE_REG;
    ulStoppedTimerCompensation = portMISSED_COUNTS_FACTOR / ( configCPU_CLOCK_HZ / configSYSTICK_CLOCK_HZ );
    if (ulReloadValue < ulStoppedTimerCompensation)
    {
      /* LPM processing compensation triggers flip over - compensate one tick and calculate the new reload value */
      lowPowerStepTicks++;
      ulReloadValue = (CORE_TICK_RATE - 1UL) - (ulStoppedTimerCompensation - ulReloadValue);
    }
    portNVIC_SYSTICK_LOAD_REG = ulReloadValue;

    /* Ensure kernel tick correction does not exceed the limit, otherwise it will trigger an assertion */
    if (lowPowerStepTicks > (xExpectedIdleTime - 1u))
    {
      lowPowerStepTicks = xExpectedIdleTime - 1u;
    }
    /* Correct the kernel tick count to account for the time spent in its low power state. */
    vTaskStepTick(lowPowerStepTicks);

    /* Restart SysTick so it runs from portNVIC_SYSTICK_LOAD_REG again,
     * then set portNVIC_SYSTICK_LOAD_REG back to its standard value.  If
     * the SysTick is not using the core clock, temporarily configure it to
     * use the core clock.  This configuration forces the SysTick to load
     * from portNVIC_SYSTICK_LOAD_REG immediately instead of at the next
     * cycle of the other clock.  Then portNVIC_SYSTICK_LOAD_REG is ready
     * to receive the standard value immediately. */
    portNVIC_SYSTICK_CURRENT_VALUE_REG = 0UL;
    portNVIC_SYSTICK_CTRL_REG = portNVIC_SYSTICK_CLK_BIT | portNVIC_SYSTICK_INT_BIT | portNVIC_SYSTICK_ENABLE_BIT;
#if ( portNVIC_SYSTICK_CLK_BIT_CONFIG == portNVIC_SYSTICK_CLK_BIT )
    portNVIC_SYSTICK_LOAD_REG = CORE_TICK_RATE - 1UL;
#else
    /* The temporary usage of the core clock has served its purpose,
     * as described above.  Resume usage of the other clock. */
    portNVIC_SYSTICK_CTRL_REG = portNVIC_SYSTICK_CLK_BIT | portNVIC_SYSTICK_INT_BIT;
    if( ( portNVIC_SYSTICK_CTRL_REG & portNVIC_SYSTICK_COUNT_FLAG_BIT ) != 0 )
    {
      /* The partial tick period already ended.  Be sure the SysTick
       * counts it only once. */
      portNVIC_SYSTICK_CURRENT_VALUE_REG = 0;
    }
    portNVIC_SYSTICK_LOAD_REG = CORE_TICK_RATE - 1UL;
    portNVIC_SYSTICK_CTRL_REG = portNVIC_SYSTICK_CLK_BIT_CONFIG | portNVIC_SYSTICK_INT_BIT | portNVIC_SYSTICK_ENABLE_BIT;
#endif /* portNVIC_SYSTICK_CLK_BIT_CONFIG */

    /* Re-enable interrupts to allow the interrupt that brought the MCU
    * out of sleep mode to execute immediately.  See comments above
    * the cpsid instruction above. */
    __asm volatile ( "cpsie i" ::: "memory" );
    __asm volatile ( "dsb" );
    __asm volatile ( "isb" );
  }
  return;
}
#else
uint32_t checkRtcWakeupTime(uint32_t xExpectedIdleTimeIn)
{
  return xExpectedIdleTimeIn;
}

/*----------------------------------------------------------------------------*/

void vPortSuppressTicksAndSleep(uint32_t xExpectedIdleTime)
{
  return;
}
#endif /* ( CFG_LPM_LEVEL != 0) */
/* USER CODE END VPORT_SUPPORT_TICKS_AND_SLEEP */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */
  /* creation of advLowPowerTimer */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
