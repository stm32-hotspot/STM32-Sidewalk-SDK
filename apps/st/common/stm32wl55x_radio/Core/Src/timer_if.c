/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    timer_if.c
  * @author  MCD Application Team
  * @brief   Configure RTC Alarm, Tick and Calendar manager
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

/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include "timer_if.h"
#include "main.h" /*for STM32CubeMX generated RTC_N_PREDIV_S and RTC_N_PREDIV_A*/
#include "rtc.h"
#include "utilities_def.h"
#include "stm32wlxx_ll_rtc.h"
#include <sid_stm32_common_utils.h>
#include "utilities_conf.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/**
  * @brief RTC handle
  */
extern RTC_HandleTypeDef hrtc;

/**
  * @brief Timer driver callbacks handler
  */
const UTIL_TIMER_Driver_s UTIL_TimerDriver =
{
  TIMER_IF_Init,
  TIMER_IF_DeInit,

  TIMER_IF_StartTimer,
  TIMER_IF_StopTimer,

  TIMER_IF_SetTimerContext,
  TIMER_IF_GetTimerContext,

  TIMER_IF_GetTimerElapsedTime,
  TIMER_IF_GetTimerValue,
  TIMER_IF_GetMinimumTimeout,

  TIMER_IF_Convert_ms2Tick,
  TIMER_IF_Convert_Tick2ms,
};

/**
  * @brief SysTime driver callbacks handler
  */
const UTIL_SYSTIM_Driver_s UTIL_SYSTIMDriver =
{
  TIMER_IF_BkUp_Write_Seconds,
  TIMER_IF_BkUp_Read_Seconds,
  TIMER_IF_BkUp_Write_SubSeconds,
  TIMER_IF_BkUp_Read_SubSeconds,
  TIMER_IF_GetTime,
};

/* USER CODE BEGIN EV */
const UTIL_TIMER_Driver_Us_s UTIL_TimerDriverUs = 
{
  TIMER_IF_Convert_us2Tick,
  TIMER_IF_Convert_Tick2us,
};

/* USER CODE END EV */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/**
  * @brief Minimum timeout delay of Alarm in ticks
  */
#define MIN_ALARM_DELAY    3

/**
  * @brief Backup seconds register
  */
#define RTC_BKP_SECONDS    RTC_BKP_DR0

/**
  * @brief Backup subseconds register
  */
#define RTC_BKP_SUBSECONDS RTC_BKP_DR1

/**
  * @brief Backup msbticks register
  */
#define RTC_BKP_MSBTICKS   RTC_BKP_DR2

/* #define RTIF_DEBUG */

/**
  * @brief Map UTIL_TIMER_IRQ can be overridden in utilities_conf.h to Map on Task rather then Isr
  */
#ifndef UTIL_TIMER_IRQ_MAP_INIT
#define UTIL_TIMER_IRQ_MAP_INIT()
#endif /* UTIL_TIMER_IRQ_MAP_INIT */

#ifndef UTIL_TIMER_IRQ_MAP_PROCESS
#define UTIL_TIMER_IRQ_MAP_PROCESS() UTIL_TIMER_IRQ_Handler()
#endif /* UTIL_TIMER_IRQ_MAP_PROCESS */

/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
#ifdef RTIF_DEBUG
#include "sys_app.h" /*for app_log*/
/**
  * @brief Post the RTC log string format to the circular queue for printing in using the polling mode
  */
#define TIMER_IF_DBG_PRINTF(...) do{ {UTIL_ADV_TRACE_COND_FSend(VLEVEL_ALWAYS, T_REG_OFF, TS_OFF, __VA_ARGS__);} }while(0);
#else
/**
  * @brief not used
  */
#define TIMER_IF_DBG_PRINTF(...)
#endif /* RTIF_DEBUG */

/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/**
  * @brief Indicates if the RTC is already Initialized or not
  */
static uint32_t RTC_Initialized = FALSE;

/**
  * @brief RtcTimerContext
  */
static volatile uint32_t RtcTimerContext = 0u;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/**
  * @brief Get rtc timer Value in rtc tick
  * @return val the rtc timer value (upcounting)
  */
static inline uint32_t GetTimerTicks(void);

/**
  * @brief Writes MSBticks to backup register
  * Absolute RTC time in tick is (MSBticks)<<32 + (32bits binary counter)
  * @note MSBticks incremented every time the 32bits RTC timer wraps around (~44days)
  * @param[in] MSBticks
  */
static inline void TIMER_IF_BkUp_Write_MSBticks(uint32_t MSBticks);

/**
  * @brief Reads MSBticks from backup register
  * Absolute RTC time in tick is (MSBticks)<<32 + (32bits binary counter)
  * @note MSBticks incremented every time the 32bits RTC timer wraps around (~44days)
  * @retval MSBticks
  */
static inline uint32_t TIMER_IF_BkUp_Read_MSBticks(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Exported functions ---------------------------------------------------------*/
UTIL_TIMER_Status_t TIMER_IF_Init(void)
{
  UTIL_TIMER_Status_t ret = UTIL_TIMER_OK;
  /* USER CODE BEGIN TIMER_IF_Init */

  /* USER CODE END TIMER_IF_Init */
  if (FALSE == RTC_Initialized)
  {
    hrtc.IsEnabled.RtcFeatures = UINT32_MAX;
    /*Init RTC*/
    MX_RTC_Init();
    /*Stop Timer */
    TIMER_IF_StopTimer();
    /*overload RTC feature enable*/
    hrtc.IsEnabled.RtcFeatures = UINT32_MAX;

    /*Enable Direct Read of the calendar registers (not through Shadow) */
    HAL_RTCEx_EnableBypassShadow(&hrtc);
    /*Initialize MSB ticks*/
    TIMER_IF_BkUp_Write_MSBticks(0);

    TIMER_IF_SetTimerContext();

    /* Register a task to associate to UTIL_TIMER_Irq() interrupt */
    UTIL_TIMER_IRQ_MAP_INIT();

    RTC_Initialized = TRUE;
  }

  /* USER CODE BEGIN TIMER_IF_Init_Last */

  /* USER CODE END TIMER_IF_Init_Last */
  return ret;
}

SID_STM32_SPEED_OPTIMIZED UTIL_TIMER_Status_t TIMER_IF_StartTimer(uint32_t timeout)
{
  UTIL_TIMER_Status_t ret = UTIL_TIMER_OK;

  UTILS_ENTER_CRITICAL_SECTION();
  /* USER CODE BEGIN TIMER_IF_StartTimer */

  /* USER CODE END TIMER_IF_StartTimer */
  RTC_AlarmTypeDef sAlarm = {0};
  /*Stop timer if one is already started*/
  TIMER_IF_StopTimer();
  timeout += RtcTimerContext;

  TIMER_IF_DBG_PRINTF("Start timer: time=%d, alarm=%d\n\r",  GetTimerTicks(), timeout);
  /* starts timer*/
  sAlarm.BinaryAutoClr = RTC_ALARMSUBSECONDBIN_AUTOCLR_NO;
  sAlarm.AlarmTime.SubSeconds = UINT32_MAX - timeout;
  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDBINMASK_NONE;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIMER_IF_StartTimer_Last */

  /* USER CODE END TIMER_IF_StartTimer_Last */
  UTILS_EXIT_CRITICAL_SECTION();
  return ret;
}

SID_STM32_SPEED_OPTIMIZED UTIL_TIMER_Status_t TIMER_IF_StopTimer(void)
{
  UTIL_TIMER_Status_t ret = UTIL_TIMER_OK;

  UTILS_ENTER_CRITICAL_SECTION();
  /* USER CODE BEGIN TIMER_IF_StopTimer */

  /* USER CODE END TIMER_IF_StopTimer */
  /* Disable the Alarm A interrupt */
  HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);
  /* Ensure RTC and NVIC are updated before proceeding */
  __DSB();
  __ISB();
  /* Clear RTC Alarm Flag */
  __HAL_RTC_ALARM_CLEAR_FLAG(&hrtc, RTC_FLAG_ALRAF);
  /*overload RTC feature enable*/
  hrtc.IsEnabled.RtcFeatures = UINT32_MAX;
  /* USER CODE BEGIN TIMER_IF_StopTimer_Last */

  /* USER CODE END TIMER_IF_StopTimer_Last */
  UTILS_EXIT_CRITICAL_SECTION();
  return ret;
}

SID_STM32_SPEED_OPTIMIZED uint32_t TIMER_IF_SetTimerContext(void)
{
  /* Store time context */
  UTILS_ENTER_CRITICAL_SECTION();
  register const uint32_t TimerContextCopy = GetTimerTicks();
  RtcTimerContext = TimerContextCopy;
  __DSB();
  __ISB();
  UTILS_EXIT_CRITICAL_SECTION();

  /* USER CODE BEGIN TIMER_IF_SetTimerContext */

  /* USER CODE END TIMER_IF_SetTimerContext */

  TIMER_IF_DBG_PRINTF("TIMER_IF_SetTimerContext=%d\n\r", TimerContextCopy);
  /*return time context*/
  return TimerContextCopy;
}

SID_STM32_SPEED_OPTIMIZED uint32_t TIMER_IF_GetTimerContext(void)
{
  /* USER CODE BEGIN TIMER_IF_GetTimerContext */

  /* USER CODE END TIMER_IF_GetTimerContext */

  UTILS_ENTER_CRITICAL_SECTION();
  register const uint32_t TimerContextCopy = RtcTimerContext;
  __DSB();
  __ISB();
  UTILS_EXIT_CRITICAL_SECTION();

  TIMER_IF_DBG_PRINTF("TIMER_IF_GetTimerContext=%d\n\r", RtcTimerContext);
  /*return time context*/
  return TimerContextCopy;
}

SID_STM32_SPEED_OPTIMIZED uint32_t TIMER_IF_GetTimerElapsedTime(void)
{
  UTILS_ENTER_CRITICAL_SECTION();
  /* USER CODE BEGIN TIMER_IF_GetTimerElapsedTime */

  /* USER CODE END TIMER_IF_GetTimerElapsedTime */
  register const uint32_t TimerContextCopy = RtcTimerContext;
  register const uint32_t TimerTicks = GetTimerTicks();
  __DSB();
  __ISB();
  /* USER CODE BEGIN TIMER_IF_GetTimerElapsedTime_Last */

  /* USER CODE END TIMER_IF_GetTimerElapsedTime_Last */
  UTILS_EXIT_CRITICAL_SECTION();
  return (TimerTicks - TimerContextCopy);
}

SID_STM32_SPEED_OPTIMIZED uint32_t TIMER_IF_GetTimerValue(void)
{
  register uint32_t TimerTicks;

  UTILS_ENTER_CRITICAL_SECTION();
  /* USER CODE BEGIN TIMER_IF_GetTimerValue */

  /* USER CODE END TIMER_IF_GetTimerValue */
  if (RTC_Initialized != FALSE)
  {
    TimerTicks = GetTimerTicks();
  }
  else
  {
    TimerTicks = 0u;
  }
  /* USER CODE BEGIN TIMER_IF_GetTimerValue_Last */

  /* USER CODE END TIMER_IF_GetTimerValue_Last */
  UTILS_EXIT_CRITICAL_SECTION();
  return TimerTicks;
}

SID_STM32_SPEED_OPTIMIZED uint32_t TIMER_IF_GetMinimumTimeout(void)
{
  register uint32_t ret;
  /* USER CODE BEGIN TIMER_IF_GetMinimumTimeout */

  /* USER CODE END TIMER_IF_GetMinimumTimeout */
  ret = (MIN_ALARM_DELAY);
  /* USER CODE BEGIN TIMER_IF_GetMinimumTimeout_Last */

  /* USER CODE END TIMER_IF_GetMinimumTimeout_Last */
  return ret;
}

SID_STM32_SPEED_OPTIMIZED uint32_t TIMER_IF_Convert_ms2Tick(uint32_t timeMilliSec)
{
  uint32_t ret = 0;
  /* USER CODE BEGIN TIMER_IF_Convert_ms2Tick */

  /* USER CODE END TIMER_IF_Convert_ms2Tick */
  ret = ((uint32_t)((((uint64_t) timeMilliSec) << RTC_N_PREDIV_S) / 1000u));
  /* USER CODE BEGIN TIMER_IF_Convert_ms2Tick_Last */

  /* USER CODE END TIMER_IF_Convert_ms2Tick_Last */
  return ret;
}

SID_STM32_SPEED_OPTIMIZED uint32_t TIMER_IF_Convert_Tick2ms(uint32_t tick)
{
  uint32_t ret = 0;
  /* USER CODE BEGIN TIMER_IF_Convert_Tick2ms */

  /* USER CODE END TIMER_IF_Convert_Tick2ms */
  ret = ((uint32_t)((((uint64_t)(tick)) * 1000u) >> RTC_N_PREDIV_S));
  /* USER CODE BEGIN TIMER_IF_Convert_Tick2ms_Last */

  /* USER CODE END TIMER_IF_Convert_Tick2ms_Last */
  return ret;
}

void TIMER_IF_DelayMs(uint32_t delay)
{
  /* USER CODE BEGIN TIMER_IF_DelayMs */

  /* USER CODE END TIMER_IF_DelayMs */
  uint32_t delayTicks = TIMER_IF_Convert_ms2Tick(delay);
  uint32_t timeout = GetTimerTicks();

  /* Wait delay ms */
  while (((GetTimerTicks() - timeout)) < delayTicks)
  {
    __NOP();
  }
  /* USER CODE BEGIN TIMER_IF_DelayMs_Last */

  /* USER CODE END TIMER_IF_DelayMs_Last */
}

SID_STM32_SPEED_OPTIMIZED void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
  /* USER CODE BEGIN HAL_RTC_AlarmAEventCallback */

  /* USER CODE END HAL_RTC_AlarmAEventCallback */
  UTIL_TIMER_IRQ_MAP_PROCESS();
  /* USER CODE BEGIN HAL_RTC_AlarmAEventCallback_Last */

  /* USER CODE END HAL_RTC_AlarmAEventCallback_Last */
}

SID_STM32_SPEED_OPTIMIZED void HAL_RTCEx_SSRUEventCallback(RTC_HandleTypeDef *hrtc)
{
  /* USER CODE BEGIN HAL_RTCEx_SSRUEventCallback */

  /* USER CODE END HAL_RTCEx_SSRUEventCallback */
  /*called every 48 days with 1024 ticks per seconds*/
  TIMER_IF_DBG_PRINTF(">>Handler SSRUnderflow at %d\n\r", GetTimerTicks());
  /*Increment MSBticks*/
  uint32_t MSB_ticks = TIMER_IF_BkUp_Read_MSBticks();
  TIMER_IF_BkUp_Write_MSBticks(MSB_ticks + 1);
  /* USER CODE BEGIN HAL_RTCEx_SSRUEventCallback_Last */

  /* USER CODE END HAL_RTCEx_SSRUEventCallback_Last */
}

SID_STM32_SPEED_OPTIMIZED uint32_t TIMER_IF_GetTime(uint16_t *mSeconds)
{
  uint32_t seconds = 0;
  /* USER CODE BEGIN TIMER_IF_GetTime */

  /* USER CODE END TIMER_IF_GetTime */

  UTILS_ENTER_CRITICAL_SECTION();
  uint64_t ticks;
  uint32_t timerValueLsb = GetTimerTicks();
  uint32_t timerValueMSB = TIMER_IF_BkUp_Read_MSBticks();
  UTILS_EXIT_CRITICAL_SECTION();

  ticks = (((uint64_t) timerValueMSB) << 32) + timerValueLsb;

  seconds = (uint32_t)(ticks >> RTC_N_PREDIV_S);

  ticks = (uint32_t) ticks & RTC_PREDIV_S;

  *mSeconds = TIMER_IF_Convert_Tick2ms(ticks);

  /* USER CODE BEGIN TIMER_IF_GetTime_Last */

  /* USER CODE END TIMER_IF_GetTime_Last */
  return seconds;
}

SID_STM32_SPEED_OPTIMIZED void TIMER_IF_BkUp_Write_Seconds(uint32_t Seconds)
{
  /* USER CODE BEGIN TIMER_IF_BkUp_Write_Seconds */

  /* USER CODE END TIMER_IF_BkUp_Write_Seconds */
  UTILS_ENTER_CRITICAL_SECTION();
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_SECONDS, Seconds);
  UTILS_EXIT_CRITICAL_SECTION();
  /* USER CODE BEGIN TIMER_IF_BkUp_Write_Seconds_Last */

  /* USER CODE END TIMER_IF_BkUp_Write_Seconds_Last */
}

SID_STM32_SPEED_OPTIMIZED void TIMER_IF_BkUp_Write_SubSeconds(uint32_t SubSeconds)
{
  /* USER CODE BEGIN TIMER_IF_BkUp_Write_SubSeconds */

  /* USER CODE END TIMER_IF_BkUp_Write_SubSeconds */
  UTILS_ENTER_CRITICAL_SECTION();
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_SUBSECONDS, SubSeconds);
  UTILS_EXIT_CRITICAL_SECTION();
  /* USER CODE BEGIN TIMER_IF_BkUp_Write_SubSeconds_Last */

  /* USER CODE END TIMER_IF_BkUp_Write_SubSeconds_Last */
}

SID_STM32_SPEED_OPTIMIZED uint32_t TIMER_IF_BkUp_Read_Seconds(void)
{
  uint32_t ret = 0;
  /* USER CODE BEGIN TIMER_IF_BkUp_Read_Seconds */

  /* USER CODE END TIMER_IF_BkUp_Read_Seconds */
  if (FALSE == RTC_Initialized)
  {
    ret = 0u;
  }
  else
  {
    ret = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_SECONDS);
  }
  /* USER CODE BEGIN TIMER_IF_BkUp_Read_Seconds_Last */

  /* USER CODE END TIMER_IF_BkUp_Read_Seconds_Last */
  return ret;
}

SID_STM32_SPEED_OPTIMIZED uint32_t TIMER_IF_BkUp_Read_SubSeconds(void)
{
  uint32_t ret = 0;
  /* USER CODE BEGIN TIMER_IF_BkUp_Read_SubSeconds */

  /* USER CODE END TIMER_IF_BkUp_Read_SubSeconds */
  if (FALSE == RTC_Initialized)
  {
    ret = 0u;
  }
  else
  {
    ret = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_SUBSECONDS);
  }
  /* USER CODE BEGIN TIMER_IF_BkUp_Read_SubSeconds_Last */

  /* USER CODE END TIMER_IF_BkUp_Read_SubSeconds_Last */
  return ret;
}

/* USER CODE BEGIN EF */
SID_STM32_SPEED_OPTIMIZED uint32_t TIMER_IF_GetTimeUs(uint32_t * const uSeconds)
{
  UTILS_ENTER_CRITICAL_SECTION();
  const uint32_t timerValueLsb = GetTimerTicks();
  const uint32_t timerValueMSB = TIMER_IF_BkUp_Read_MSBticks();
  UTILS_EXIT_CRITICAL_SECTION();

  const uint64_t ticks = (((uint64_t)timerValueMSB) << 32) + (uint64_t)timerValueLsb;
  const uint32_t seconds = (uint32_t)(ticks >> RTC_N_PREDIV_S);
  const uint64_t subsecond_ticks = ticks & RTC_PREDIV_S;

  *uSeconds = (uint32_t)TIMER_IF_Convert_Tick2us(subsecond_ticks);
  return seconds;
}

SID_STM32_SPEED_OPTIMIZED uint64_t TIMER_IF_Convert_us2Tick(const uint64_t uSeconds)
{
  const uint64_t ticks = (uSeconds << RTC_N_PREDIV_S) / (uint64_t)1000000u;
  return ticks;
}

SID_STM32_SPEED_OPTIMIZED uint64_t TIMER_IF_Convert_Tick2us(const uint64_t ticks)
{
  const uint64_t uSeconds = (ticks * (uint64_t)1000000u) >> RTC_N_PREDIV_S;
  return uSeconds;
}

UTIL_TIMER_Status_t TIMER_IF_DeInit(void)
{
  UTIL_TIMER_Status_t ret = UTIL_TIMER_OK;

  /* USER CODE BEGIN TIMER_IF_Init_1 */

  /* USER CODE END TIMER_IF_Init_1 */

  if (RTC_Initialized != FALSE)
  {
    /* Stop Timer */
    TIMER_IF_StopTimer();

    /* Disable Direct Read of the calendar registers (not through Shadow) */
    HAL_RTCEx_DisableBypassShadow(&hrtc);

    TIMER_IF_SetTimerContext();

    RTC_Initialized = FALSE;
  }

  /* USER CODE BEGIN TIMER_IF_Init_2 */

  /* USER CODE END TIMER_IF_Init_2 */

  return ret;

  /* USER CODE BEGIN TIMER_IF_Init_3 */

  /* USER CODE END TIMER_IF_Init_3 */
}
/* USER CODE END EF */

/* Private functions ---------------------------------------------------------*/
SID_STM32_SPEED_OPTIMIZED static inline void TIMER_IF_BkUp_Write_MSBticks(uint32_t MSBticks)
{
  /* USER CODE BEGIN TIMER_IF_BkUp_Write_MSBticks */

  /* USER CODE END TIMER_IF_BkUp_Write_MSBticks */
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_MSBTICKS, MSBticks);
  /* USER CODE BEGIN TIMER_IF_BkUp_Write_MSBticks_Last */

  /* USER CODE END TIMER_IF_BkUp_Write_MSBticks_Last */
}

SID_STM32_SPEED_OPTIMIZED static inline uint32_t TIMER_IF_BkUp_Read_MSBticks(void)
{
  uint32_t MSBticks;
  /* USER CODE BEGIN TIMER_IF_BkUp_Read_MSBticks */

  /* USER CODE END TIMER_IF_BkUp_Read_MSBticks */
  if (FALSE == RTC_Initialized)
  {
    MSBticks = 0u;
  }
  else
  {
    MSBticks = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_MSBTICKS);
  }
  /* USER CODE BEGIN TIMER_IF_BkUp_Read_MSBticks_Last */

  /* USER CODE END TIMER_IF_BkUp_Read_MSBticks_Last */
  return MSBticks;
}

SID_STM32_SPEED_OPTIMIZED static inline uint32_t GetTimerTicks(void)
{
  register uint32_t ticks;
  /* USER CODE BEGIN GetTimerTicks */

  /* USER CODE END GetTimerTicks */
  if (FALSE == RTC_Initialized)
  {
    ticks = 0u;
  }
  else
  {
    register uint32_t ssr = LL_RTC_TIME_GetSubSecond(RTC);

    /* Read SSR until two consecutive readings return the same value - this is required to avoid synchronization issues when shadow registers are bypassed */
    while (ssr != LL_RTC_TIME_GetSubSecond(RTC))
    {
      ssr = LL_RTC_TIME_GetSubSecond(RTC);
    }

    ticks = UINT32_MAX - ssr;
  }
  /* USER CODE BEGIN GetTimerTicks_Last */

  /* USER CODE END GetTimerTicks_Last */
  return ticks;
}

/* USER CODE BEGIN PrFD */

/* USER CODE END PrFD */
