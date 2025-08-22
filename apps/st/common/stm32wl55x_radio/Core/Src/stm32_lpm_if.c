/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32_lpm_if.c
  * @author  MCD Application Team
  * @brief   Low layer function to enter/exit low power modes (stop, sleep)
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
#include "platform.h"
#include "stm32_lpm.h"
#include "stm32_lpm_if.h"
#include "sys_app.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Private typedef -----------------------------------------------------------*/
/**
  * @brief Power driver callbacks handler
  */
const struct UTIL_LPM_Driver_s UTIL_PowerDriver =
{
  PWR_EnterSleepMode,
  PWR_ExitSleepMode,

  PWR_EnterStopMode,
  PWR_ExitStopMode,

  PWR_EnterOffMode,
  PWR_ExitOffMode,
};

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
static uint32_t sys_clk_src = UINT32_MAX; /* Use init value that is invalid to detect data loss during LPM or reset */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Exported functions --------------------------------------------------------*/

void PWR_EnterOffMode(void)
{
  /* USER CODE BEGIN EnterOffMode_1 */
  const register uint32_t mcu_rev  = LL_DBGMCU_GetRevisionID();

  if (STM32WLxx_MCU_REV_Z == mcu_rev)
  {
    /* Workaround for errata 2.2.12 - wait for LDO to report its readiness */
    while (LL_PWR_IsActiveFlag_LDORDY() == 0u)
    {
      __NOP();
    }
  }
  /* USER CODE END EnterOffMode_1 */
}

void PWR_ExitOffMode(void)
{
  /* USER CODE BEGIN ExitOffMode_1 */

  /* USER CODE END ExitOffMode_1 */
}

void PWR_EnterStopMode(void)
{
  /* USER CODE BEGIN EnterStopMode_1 */
  const register uint32_t mcu_rev  = LL_DBGMCU_GetRevisionID();

  if (STM32WLxx_MCU_REV_Z == mcu_rev)
  {
    /* Workaround for errata 2.2.12 - wait for LDO to report its readiness */
    while (LL_PWR_IsActiveFlag_LDORDY() == 0u)
    {
      __NOP();
    }
  }

  /* Store the system clock source before entry */
  sys_clk_src = LL_RCC_GetSysClkSource();

  /* USER CODE END EnterStopMode_1 */
  HAL_SuspendTick();
  /* Clear Status Flag before entering STOP/STANDBY Mode */
  LL_PWR_ClearFlag_C1STOP_C1STB();

  /* USER CODE BEGIN EnterStopMode_2 */

  /* USER CODE END EnterStopMode_2 */
  HAL_PWREx_EnterSTOP1Mode(PWR_STOPENTRY_WFI);
  /* USER CODE BEGIN EnterStopMode_3 */

  /* USER CODE END EnterStopMode_3 */
}

void PWR_ExitStopMode(void)
{
  /* USER CODE BEGIN ExitStopMode_1 */

  /* Restore system clock source */
  switch (sys_clk_src)
  {
    case LL_RCC_SYS_CLKSOURCE_STATUS_MSI:
      /* Ensure MSI is enabled */
      LL_RCC_MSI_Enable();
      /* Wait till MSI is ready */
      while (LL_RCC_MSI_IsReady() == 0U);
      /* Switch System Clock on MSI */
      LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_MSI);
      /* Wait for system clock switch */
      while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_MSI);
      /* Disable HSI to save power */
      LL_RCC_HSI_Disable();
      break;

    case LL_RCC_SYS_CLKSOURCE_STATUS_HSI:
      /* Ensure HSI is enabled */
      LL_RCC_HSI_Enable();
      /* Wait till HSI is ready */
      while (LL_RCC_HSI_IsReady() == 0U);
      /* Switch System Clock on HSI */
      LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);
      /* Wait for system clock switch */
      while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI);
      /* Disable MSI to save power */
      LL_RCC_MSI_Disable();
      break;

    case LL_RCC_SYS_CLKSOURCE_STATUS_HSE:
      /* Ensure HSE is enabled */
      LL_RCC_HSE_Enable();
      /* Wait till HSE is ready */
      while (LL_RCC_HSE_IsReady() == 0U);
      /* Switch System Clock on HSE */
      LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSE);
      /* Wait for system clock switch */
      while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSE);
      /* Disable MSI and HSI to save power */
      LL_RCC_MSI_Disable();
      LL_RCC_HSI_Disable();
      break;

    case LL_RCC_SYS_CLKSOURCE_STATUS_PLL:
      /* Ensure HSE is enabled */
      LL_RCC_HSE_Enable();
      /* Wait till HSE is ready */
      while (LL_RCC_HSE_IsReady() == 0U);
      /* Enable the main PLL. */
      LL_RCC_PLL_Enable();
      /* Wait till PLL is ready */
      while (LL_RCC_PLL_IsReady() == 0U);
      /* Switch System Clock on PLL */
      LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
      /* Wait for system clock switch */
      while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL);
      /* Disable MSI and HSI to save power */
      LL_RCC_MSI_Disable();
      LL_RCC_HSI_Disable();
      break;

    default:
      /* Unexpected sys clock selection */
      Error_Handler();
      return; /* Normally this point should not be reached, but just in case - terminate */
  }

  /* Ensure time base clock coherency */
  SystemCoreClockUpdate();

  /* USER CODE END ExitStopMode_1 */
  /* Resume sysTick : work around for debugger problem in dual core */
  HAL_ResumeTick();
  /* USER CODE BEGIN ExitStopMode_2 */

  /* USER CODE END ExitStopMode_2 */
}

void PWR_EnterSleepMode(void)
{
  /* USER CODE BEGIN EnterSleepMode_1 */

  /* USER CODE END EnterSleepMode_1 */
  /* Suspend sysTick */
  HAL_SuspendTick();
  /* USER CODE BEGIN EnterSleepMode_2 */

  /* USER CODE END EnterSleepMode_2 */
  HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
  /* USER CODE BEGIN EnterSleepMode_3 */

  /* USER CODE END EnterSleepMode_3 */
}

void PWR_ExitSleepMode(void)
{
  /* USER CODE BEGIN ExitSleepMode_1 */

  /* USER CODE END ExitSleepMode_1 */
  /* Resume sysTick */
  HAL_ResumeTick();

  /* USER CODE BEGIN ExitSleepMode_2 */

  /* USER CODE END ExitSleepMode_2 */
}

/* USER CODE BEGIN EF */

/* USER CODE END EF */

/* Private Functions Definition -----------------------------------------------*/
/* USER CODE BEGIN PrFD */

/* USER CODE END PrFD */
