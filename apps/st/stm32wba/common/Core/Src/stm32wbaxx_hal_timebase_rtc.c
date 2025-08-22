/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32wbaxx_hal_timebase_rtc.c
  * @brief   HAL time base based on the RTC.
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

/* Includes ------------------------------------------------------------------*/
#include "stm32wbaxx_hal.h"
#include "timer_if.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  This function configures the RTC as a time base source.
  *         The time source is configured  to have 1ms time base with a dedicated
  *         Tick interrupt priority.
  * @note   This function is called  automatically at the beginning of program after
  *         reset by HAL_Init() or at any time when clock is configured, by HAL_RCC_ClockConfig().
  * @note This function overwrites the __weak one from HAL
  * @param  TickPriority: Tick interrupt priority.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
  HAL_StatusTypeDef ret = HAL_OK;

  UNUSED(TickPriority);

  /* USER CODE BEGIN HAL_InitTick_1 */

  /* USER CODE END HAL_InitTick_1 */

#ifdef FREERTOS
  /* Configure SysTick IRQ to have the lowest priority - as required by FreeRTOS */
  HAL_NVIC_SetPriority(SysTick_IRQn, (1U << __NVIC_PRIO_BITS) - 1U, 0U);
#endif

  /* USER CODE BEGIN HAL_InitTick_2 */

  /* USER CODE END HAL_InitTick_2 */

  return ret;
}

/**
  * @brief  Suspend Tick increment.
  * @note   This function overwrites the __weak one from HAL
  * @param  None
  * @retval None
  */
void HAL_SuspendTick(void)
{
  /* Nothing to do here */
  return;
}

/**
  * @brief  Resume Tick increment.
  * @note   This function overwrites the __weak one from HAL
  * @param  None
  * @retval None
  */
void HAL_ResumeTick(void)
{
  /* Nothing to do here */
  return;
}

/**
  * @note This function overwrites the __weak one from HAL
  */

/**
  * @brief Provide a tick value in millisecond.
  * @note This function overwrites the __weak one from HAL
  * @retval tick value
  */
uint32_t HAL_GetTick(void)
{
  uint32_t ret;
  uint32_t seconds;
  uint32_t us;

  /* USER CODE BEGIN HAL_GetTick_1 */

  /* USER CODE END HAL_GetTick_1 */
  seconds = TIMER_IF_GetTimeUs(&us);
  ret = (uint32_t)(((uint64_t)seconds * (uint64_t)1000u) + ((uint64_t)us / (uint64_t)1000u));
  /* USER CODE BEGIN HAL_GetTick_2 */

  /* USER CODE END HAL_GetTick_2 */
  return ret;
}

/**
  * @brief This function provides minimum delay (in milliseconds) based
  *        on variable incremented.
  * @note This function overwrites the __weak one from HAL
  * @param Delay  specifies the delay time length, in milliseconds.
  * @retval None
  */
void HAL_Delay(uint32_t Delay)
{
  /* USER CODE BEGIN HAL_Delay_1 */

  /* USER CODE END HAL_Delay_1 */
  TIMER_IF_DelayMs(Delay);
  /* USER CODE BEGIN HAL_Delay_2 */

  /* USER CODE END HAL_Delay_2 */
}
