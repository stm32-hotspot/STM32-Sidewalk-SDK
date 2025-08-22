/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021-2025 STMicroelectronics.
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
#include <FreeRTOS.h>
#include <task.h>
#include "main.h"
#include <cmsis_os2.h>

#include "host_comm.h"
#include "serial_bus_spi_pal.h"
#include "sid_pal_log_like.h"
#include <stm32_mcu_info.h>
#include "stm32wlxx_hal.h"
#include "sys_app.h"
#include "sys_conf.h"
#include <timer_if.h>
#include "utilities_def.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#if defined(NUCLEO_WL55_BOARD)
#  include <stm32wlxx_nucleo.h>
#endif /* NUCLEO_WL55_BOARD */

#include "stm32_timer.h"
#include "stm32_lpm.h"

#include SID_APP_VERSION_HEADER
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef __PACKED_UNION {
    __PACKED_STRUCT {
        uint8_t build;
        uint8_t patch;
        uint8_t minor;
        uint8_t major;
    };
    uint32_t raw;
} hal_version_info_t;

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define configTICK_RATE_HZ_1MS                 1000

#define portNVIC_SYSTICK_CTRL_REG               ( * ( ( volatile uint32_t * ) 0xe000e010 ) )
#define portNVIC_SYSTICK_LOAD_REG               ( * ( ( volatile uint32_t * ) 0xe000e014 ) )
#define portNVIC_SYSTICK_CURRENT_VALUE_REG      ( * ( ( volatile uint32_t * ) 0xe000e018 ) )
#define portNVIC_SYSTICK_ENABLE_BIT             ( 1UL << 0UL )
#define portMISSED_COUNTS_FACTOR                ( 45UL )
#ifndef configSYSTICK_CLOCK_HZ
  #define configSYSTICK_CLOCK_HZ                ( configCPU_CLOCK_HZ )
#endif
#define CORE_TICK_RATE                          (( configSYSTICK_CLOCK_HZ ) / ( configTICK_RATE_HZ ))

#if (STACK_USAGE_DIAGNOSTICS_ENABLED == 1)
extern uint32_t __start_STACK;
extern uint32_t __end_STACK;
extern uint32_t __start_STACK_GUARD;
extern uint32_t __end_STACK_GUARD;
#define APP_CONFIG_USER_STACK_START       ( (uint32_t)&__start_STACK)
#define APP_CONFIG_USER_STACK_END         ( (uint32_t)&__end_STACK)
#define APP_CONFIG_USER_STACK_GUARD_START ( (uint32_t)&__start_STACK_GUARD)
#define APP_CONFIG_USER_STACK_GUARD_END   ( (uint32_t)&__end_STACK_GUARD)
#endif
#if (HEAP_USAGE_DIAGNOSTICS_ENABLED == 1)
extern uint32_t __start_HEAP;
extern uint32_t __end_HEAP;
#define APP_CONFIG_USER_HEAP_START       ( (uint32_t)&__start_HEAP)
#define APP_CONFIG_USER_HEAP_END         ( (uint32_t)&__end_HEAP)
#endif

#ifndef STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER
#  error "STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER is not defined"
#endif /* STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static UTIL_TIMER_Object_t WakeUpTimer;
static volatile uint32_t Time_BeforeSleep;
#if (STACK_USAGE_DIAGNOSTICS_ENABLED == 1)
volatile uint32_t userStackFreeSpaceMinimum = UINT32_MAX;
#endif
#if (HEAP_USAGE_DIAGNOSTICS_ENABLED == 1)
volatile uint32_t userHeapFreeSpaceMinimum = UINT32_MAX;
#endif

extern SPI_HandleTypeDef SIDEWALK_RADIO_SPI;
/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
/**
  * @brief  Callback when wakeup: do nothing
  * @param  None
  * @retval None
  */
static void  WakeUpTimer_Cb(void *context);
/* USER CODE END FunctionPrototypes */

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN PREPOSTSLEEP */
#if defined (LOW_POWER_DISABLE) && (LOW_POWER_DISABLE == 0)
SID_STM32_SPEED_OPTIMIZED static inline void PreSleepAction(void)
{
  /* Called by the kernel before it places the MCU into a sleep mode because
  configPRE_SLEEP_PROCESSING() is #defined to PreSleepProcessing().

  NOTE:  Additional actions can be taken here to get the power consumption
  even lower.  For example, peripherals can be turned off here, and then back
  on again in the post sleep processing function.  For maximum power saving
  ensure all unused pins are in their lowest power state. */

  /* Check if NSS line is active now */
#if SERIAL_BUS_SPI_PAL_NSS_ACTIVATED_GPIO_PIN_STATE == GPIO_PIN_RESET
  /* NSS is active when GPIO state is low */
  if ((SIDEWALK_RADIO_SPI_NSS_GPIO_Port->IDR & SIDEWALK_RADIO_SPI_NSS_Pin) == 0u)
#else
  /* NSS is active when GPIO state is high */
  if ((SIDEWALK_RADIO_SPI_NSS_GPIO_Port->IDR & SIDEWALK_RADIO_SPI_NSS_Pin) != 0u)
#endif /* SERIAL_BUS_SPI_PAL_NSS_ACTIVATED_GPIO_PIN_STATE */
  {
    /* NSS is active - Stop mode is prohibited since SPI and DMA shall keep running */
    UTIL_LPM_SetStopMode((1u << CFG_LPM_HOST_COMM_SPI), UTIL_LPM_DISABLE);
  }
  else
  {
    register const uint32_t mcu_rev = LL_DBGMCU_GetRevisionID();
    if (mcu_rev > STM32WLxx_MCU_REV_Z)
    {
      /* Temporarily enable NSS activation to serve as a wakeup source */
#if SERIAL_BUS_SPI_PAL_NSS_ACTIVATED_GPIO_PIN_STATE == GPIO_PIN_RESET
      /* NSS is active when GPIO state is low */
      SET_BIT(EXTI->FTSR1, SIDEWALK_RADIO_SPI_NSS_Pin);
#else
      /* NSS is active when GPIO state is high */
      SET_BIT(EXTI->RTSR1, SIDEWALK_RADIO_SPI_NSS_Pin);
#endif /* SERIAL_BUS_SPI_PAL_NSS_ACTIVATED_GPIO_PIN_STATE */
    }
    else
    {
      /* For Rev Z both edges on NSS pin are permanently enabled as part of the workaround for errata 2.2.1 */
    }
  }

  UTIL_LPM_EnterLowPower();
}

SID_STM32_SPEED_OPTIMIZED static inline void PostSleepAction(void)
{
  /* Called by the kernel when the MCU exits a sleep mode because
  configPOST_SLEEP_PROCESSING is #defined to PostSleepProcessing(). */

  register const uint32_t mcu_rev = LL_DBGMCU_GetRevisionID();
  if (mcu_rev > STM32WLxx_MCU_REV_Z)
  {
    /* Check if NSS falling edge was enabled as a wakeup source */
#if SERIAL_BUS_SPI_PAL_NSS_ACTIVATED_GPIO_PIN_STATE == GPIO_PIN_RESET
    /* NSS is active when GPIO state is low */
    if (READ_BIT(EXTI->FTSR1, SIDEWALK_RADIO_SPI_NSS_Pin) != 0u)
#else
    /* NSS is active when GPIO state is high */
    if (READ_BIT(EXTI->RTSR1, SIDEWALK_RADIO_SPI_NSS_Pin) != 0u)
#endif /* SERIAL_BUS_SPI_PAL_NSS_ACTIVATED_GPIO_PIN_STATE */
    {
      /* Disable falling edge trigger and clear pending IRQ since we don't need to process NSS activation for SPI transactions */
#if SERIAL_BUS_SPI_PAL_NSS_ACTIVATED_GPIO_PIN_STATE == GPIO_PIN_RESET
      /* NSS is active when GPIO state is low */
      CLEAR_BIT( EXTI->FTSR1, SIDEWALK_RADIO_SPI_NSS_Pin);
#else
      /* NSS is active when GPIO state is high */
      CLEAR_BIT( EXTI->RTSR1, SIDEWALK_RADIO_SPI_NSS_Pin);
#endif /* SERIAL_BUS_SPI_PAL_NSS_ACTIVATED_GPIO_PIN_STATE */
      __HAL_GPIO_EXTI_CLEAR_IT(SIDEWALK_RADIO_SPI_NSS_Pin);
    }
  }
  else
  {
    /* For Rev Z both edges on NSS pin are permanently enabled as part of the workaround for errata 2.2.1 */
  }
}

SID_STM32_SPEED_OPTIMIZED uint32_t checkExpectedRtcIdleTime(uint32_t ulExpectedIdleTimeIn)
{
  uint32_t ulExpectedIdleTimeOut;
  const uint32_t ulSleepThresholdMs = portTICK_PERIOD_MS * configEXPECTED_IDLE_TIME_BEFORE_SLEEP;
  const uint32_t ulNextRtcEventMs = TIMER_IF_Convert_Tick2ms(UTIL_TIMER_GetFirstRemainingTime());

  if (ulNextRtcEventMs < ulSleepThresholdMs)
  {
    /* Next RTC event is too close, skip entering LPM */
    ulExpectedIdleTimeOut = 0u;
  }
  else
  {
    /* Next RTC event is far enough, enter the planned LPM */
    ulExpectedIdleTimeOut = ulExpectedIdleTimeIn;
  }

  return ulExpectedIdleTimeOut;
}

SID_STM32_SPEED_OPTIMIZED void vPortSuppressTicksAndSleep(TickType_t xExpectedIdleTime)
{
  uint32_t ulReloadValue, ulStoppedTimerCompensation;
  uint32_t lowPowerTimeBeforeSleep, lowPowerTimeAfterSleep, lowPowerStepTicks;

  /* Enter a critical section but don't use the taskENTER_CRITICAL()
  method as that will mask interrupts that should exit sleep mode. */
  __asm volatile( "cpsid i" ::: "memory" );
  __asm volatile( "dsb" );
  __asm volatile( "isb" );
  __COMPILER_BARRIER();

  /* Stop the SysTick momentarily.  The time the SysTick is stopped for
  is accounted for as best it can be, but using the tickless mode will
  inevitably result in some tiny drift of the time maintained by the
  kernel with respect to calendar time. */
  portNVIC_SYSTICK_CTRL_REG &= ~portNVIC_SYSTICK_ENABLE_BIT;
  __COMPILER_BARRIER();

  /* If a context switch is pending or a task is waiting for the scheduler
  to be unsuspended then abandon the low power entry. */
  const eSleepModeStatus eSleepStatus = eTaskConfirmSleepModeStatus();
  if( eSleepStatus == eAbortSleep )
  {
    /* Restart from whatever is left in the count register to complete
    this tick period. */
    portNVIC_SYSTICK_LOAD_REG = portNVIC_SYSTICK_CURRENT_VALUE_REG;

    /* Restart SysTick. */
    portNVIC_SYSTICK_CTRL_REG |= portNVIC_SYSTICK_ENABLE_BIT;

    /* Reset the reload register to the value required for normal tick
    periods. */
    portNVIC_SYSTICK_LOAD_REG = CORE_TICK_RATE - 1UL;

    /* Re-enable interrupts - see comments above the cpsid instruction()
    above. */
    __asm volatile( "cpsie i" ::: "memory" );
  }
  else
  {
    /* Read the current time from RTC, maintainned in standby */
    lowPowerTimeBeforeSleep = TIMER_IF_GetTimerValue();
    if( eSleepStatus == eNoTasksWaitingTimeout )
    {
      /* It is not necessary to configure an interrupt to bring the
      microcontroller out of its low power state at a fixed time in the
      future. */
    }
    else
    {
      /* Configure an interrupt to bring the microcontroller out of its low
      power state at the time the kernel next needs to execute. The
      interrupt must be generated from a source that remains operational
      when the microcontroller is in a low power state. */
      UTIL_TIMER_StartWithPeriod(&WakeUpTimer, (xExpectedIdleTime - 1u) * portTICK_PERIOD_MS);
    }
    /* Sleep until something happens. */
    PreSleepAction();
    PostSleepAction();

    /* Determine how long the microcontroller was actually in a low power
    state for, which will be less than xExpectedIdleTime if the
    microcontroller was brought out of low power mode by an interrupt
    other than that configured by the vSetWakeTimeInterrupt() call.
    Note that the scheduler is suspended before
    portSUPPRESS_TICKS_AND_SLEEP() is called, and resumed when
    portSUPPRESS_TICKS_AND_SLEEP() returns.  Therefore no other tasks will
    execute until this function completes. */
    lowPowerTimeAfterSleep = TIMER_IF_GetTimerValue();
    lowPowerStepTicks = TIMER_IF_Convert_Tick2ms(lowPowerTimeAfterSleep - lowPowerTimeBeforeSleep) / portTICK_PERIOD_MS;

    /* Restart from whatever is left in the count register to complete
    this tick period. */
    ulReloadValue = portNVIC_SYSTICK_CURRENT_VALUE_REG;
    ulStoppedTimerCompensation = portMISSED_COUNTS_FACTOR / ( configCPU_CLOCK_HZ / configSYSTICK_CLOCK_HZ );
    if (ulReloadValue < ulStoppedTimerCompensation)
    {
      /* LPM processing compensation triggers flip over - compensate one tick and calculate the new reload value */
      vTaskStepTick(1u);
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

    /* Restart SysTick. */
    portNVIC_SYSTICK_CTRL_REG |= portNVIC_SYSTICK_ENABLE_BIT;

    /* Reset the reload register to the value required for normal tick
    periods. */
    portNVIC_SYSTICK_LOAD_REG = CORE_TICK_RATE - 1UL;

    /* Re-enable interrupts to allow the interrupt that brought the MCU
    out of sleep mode to execute immediately.  see comments above
    __disable_interrupt() call above. */
    __asm volatile( "cpsie i" ::: "memory" );
    __asm volatile( "dsb" );
    __asm volatile( "isb" );
  }
}
#elif (LOW_POWER_DISABLE != 0)
uint32_t checkExpectedRtcIdleTime(uint32_t ulExpectedIdleTimeIn)
{
  return ulExpectedIdleTimeIn;
}

void vPortSuppressTicksAndSleep(TickType_t xExpectedIdleTime)
{
  (void)xExpectedIdleTime;
  return;
}
#elif !defined (LOW_POWER_DISABLE)
  #error LOW_POWER_DISABLE not defined
#endif /* LOW_POWER_DISABLE */
/* USER CODE END PREPOSTSLEEP */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  UTIL_TIMER_Create(&WakeUpTimer, 0, UTIL_TIMER_ONESHOT, WakeUpTimer_Cb, NULL);

  /* init code for LoRaWAN */
  SystemApp_Init();

  /* Adjust log level to use sid_pal_log-like functions */
  if (SID_PAL_LOG_LEVEL > SID_PAL_LOG_SEVERITY_ERROR)
  {
    /* Errors are always printed out and require no adjustments */
    UTIL_ADV_TRACE_SetVerboseLevel(SID_PAL_LOG_LEVEL);
  }

  /* Printout application version info */
  SID_PAL_LOG_INFO("Application name: %s", SID_APP_PROJECT_NAME);
  SID_PAL_LOG_INFO("Application version %s", SID_APP_PROJECT_VERSION_STRING);
  SID_PAL_LOG_INFO("Application build type: %s", SID_APP_PROJECT_BUILD_TYPE);
  SID_PAL_LOG_INFO("Application commit hash: %s", SID_APP_PROJECT_COMMIT_HASH_STRING);
  SID_PAL_LOG_INFO("Application commit description: %s", SID_APP_PROJECT_COMMIT_DESCRIPTION);
  SID_PAL_LOG_INFO("FreeRTOS Kernel: %s", tskKERNEL_VERSION_NUMBER);

  /* CubeMX pack version */
  const hal_version_info_t cubemx_fw_pack_ver = {
    .raw = HAL_GetHalVersion(),
  };
  SID_PAL_LOG_INFO("STM32CubeWL: %u.%u.%u", cubemx_fw_pack_ver.major, cubemx_fw_pack_ver.minor, cubemx_fw_pack_ver.patch);

  /* Printout MCU details */
  stm32_mcu_info_t mcu_info = stm32_mcu_info_describe_host();
  SID_PAL_LOG_INFO("Host MCU: %s (0x%X), revision: %s (0x%X)", mcu_info.device_name, mcu_info.device_id, mcu_info.rev_name, mcu_info.rev_id);

  /* Ensure SPI configuration is consistent */
  if ((uint32_t)SIDEWALK_RADIO_SPI_INSTANCE_BASE != (uint32_t)(void *)SIDEWALK_RADIO_SPI.Instance)
  {
    SID_PAL_LOG_ERROR("Mismatch between SIDEWALK_RADIO_SPI_INSTANCE_BASE and SIDEWALK_RADIO_SPI.Instance. Please check inter-MCU SPI configuration");
    Error_Handler();
  }

#if CFG_LED_SUPPORTED
    /* Indicate the processing has started */
    BSP_LED_On(LED_GREEN);
#endif /* CFG_LED_SUPPORTED */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  sid_host_comm_error_t sid_hc_err = sid_host_comm_init();
  if (sid_hc_err != SID_HOST_COMM_ERROR_NONE)
  {
    SID_PAL_LOG_ERROR("STM32WLxx Radio App init failed. Error %u", (uint32_t)sid_hc_err);
    Error_Handler();
  }

#if STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER
  sid_hc_err = sid_host_comm_udt_user_init();
  if (sid_hc_err != SID_HOST_COMM_ERROR_NONE)
  {
    SID_PAL_LOG_ERROR("Failed to initialize user part of UDT. Error %u", (uint32_t)sid_hc_err);
    Error_Handler();
  }
#endif /* STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
static void WakeUpTimer_Cb(void *context)
{
  /*Nothing to do*/
  UNUSED(context);
}

void vApplicationIdleHook( void )
{
#if (STACK_USAGE_DIAGNOSTICS_ENABLED == 1)
#  define USER_STACK_GUARD_WATERMARK (0xDEADBEEFu)
#  define USER_STACK_FREE_RAM_WATERMARK (0xAA55AA55u)
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
#endif

#if (HEAP_USAGE_DIAGNOSTICS_ENABLED == 1)
#  define USER_HEAP_FREE_RAM_WATERMARK (0x55AA55AAu)
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
#endif

  const uint32_t fifo_level = serial_bus_spi_get_rx_fifo_level();
  if (fifo_level > 0u)
  {
    /* There's some unprocessed data in the Host Comm SPI buffer, process it */
    sid_host_comm_on_spi_frame_received();
  }
  else
  {
    /* Nothing to do */
  }
}
/* USER CODE END Application */
