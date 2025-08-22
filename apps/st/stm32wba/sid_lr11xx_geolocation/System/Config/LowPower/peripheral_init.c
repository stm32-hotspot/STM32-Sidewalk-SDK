/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    peripheral_init.c
  * @author  MCD Application Team
  * @brief   peripheral init module
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "peripheral_init.h"
#include "main.h"
#include "crc_ctrl.h"
#if (USE_TEMPERATURE_BASED_RADIO_CALIBRATION == 1)
#include "adc_ctrl.h"
#endif /* USE_TEMPERATURE_BASED_RADIO_CALIBRATION */
#if (CFG_LPM_WAKEUP_TIME_PROFILING == 1)
#include "stm32_lpm_if.h"
#endif /* CFG_LPM_WAKEUP_TIME_PROFILING */
/* Private includes -----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* External variables --------------------------------------------------------*/

extern RAMCFG_HandleTypeDef hramcfg_SRAM1;
extern RAMCFG_HandleTypeDef hramcfg_SRAM2;

extern DMA_HandleTypeDef handle_GPDMA1_Channel0;
extern DMA_HandleTypeDef handle_GPDMA1_Channel1;
extern DMA_HandleTypeDef handle_GPDMA1_Channel2;
#if defined(STM32WBA5x)
extern SPI_HandleTypeDef hspi1;
#elif defined(STM32WBA6x)
extern SPI_HandleTypeDef hspi2;
#endif /* STM32WBAxx */
extern LPTIM_HandleTypeDef hlptim1;

#if (CFG_LOG_SUPPORTED)
extern DMA_HandleTypeDef handle_GPDMA1_Channel4;
extern DMA_HandleTypeDef handle_GPDMA1_Channel5;
extern UART_HandleTypeDef huart1;
#endif /* CFG_LOG_SUPPORTED */

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Functions Definition ------------------------------------------------------*/

/**
  * @brief  Configure the SoC peripherals at Standby mode exit.
  * @param  None
  * @retval None
  */
void MX_StandbyExit_PeripheralInit(void)
{
  /* USER CODE BEGIN MX_STANDBY_EXIT_PERIPHERAL_INIT_1 */

  /* USER CODE END MX_STANDBY_EXIT_PERIPHERAL_INIT_1 */

#if (CFG_LPM_WAKEUP_TIME_PROFILING == 1)
#if (CFG_LPM_STDBY_SUPPORTED == 1)
  /* Do not configure sysTick if currently used by wakeup time profiling mechanism */
  if(LPM_is_wakeup_time_profiling_done() != 0)
  {
    /* Select SysTick source clock */
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_LSE);

    /* Initialize SysTick */
    if (HAL_InitTick(TICK_INT_PRIORITY) != HAL_OK)
    {
      assert_param(0);
    }
  }
#endif /* CFG_LPM_STDBY_SUPPORTED */
#else
  /* Select SysTick source clock */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_LSE);

  /* Initialize SysTick */
  if (HAL_InitTick(TICK_INT_PRIORITY) != HAL_OK)
  {
    assert_param(0);
  }
#endif /* CFG_LPM_WAKEUP_TIME_PROFILING */

#if (CFG_DEBUGGER_LEVEL == 0)
/* Setup GPIOA 13, 14, 15 in Analog no pull */
  GPIO_InitTypeDef DbgIOsInit = {0};
  DbgIOsInit.Mode = GPIO_MODE_ANALOG;
  DbgIOsInit.Pull = GPIO_NOPULL;
  DbgIOsInit.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  __HAL_RCC_GPIOA_CLK_ENABLE();
  HAL_GPIO_Init(GPIOA, &DbgIOsInit);

  /* Setup GPIOB 3, 4 in Analog no pull */
  DbgIOsInit.Mode = GPIO_MODE_ANALOG;
  DbgIOsInit.Pull = GPIO_NOPULL;
  DbgIOsInit.Pin = GPIO_PIN_3|GPIO_PIN_4;
  __HAL_RCC_GPIOB_CLK_ENABLE();
  HAL_GPIO_Init(GPIOB, &DbgIOsInit);
#endif /* CFG_DEBUGGER_LEVEL */

  memset(&hramcfg_SRAM1, 0u, sizeof(hramcfg_SRAM1));
  memset(&hramcfg_SRAM2, 0u, sizeof(hramcfg_SRAM2));

  memset(&handle_GPDMA1_Channel0, 0u, sizeof(handle_GPDMA1_Channel0));
  memset(&handle_GPDMA1_Channel1, 0u, sizeof(handle_GPDMA1_Channel1));
  memset(&handle_GPDMA1_Channel2, 0u, sizeof(handle_GPDMA1_Channel2));
#if defined(STM32WBA5x)
  memset(&hspi1, 0u, sizeof(hspi1));
#elif defined(STM32WBA6x)
  memset(&hspi2, 0u, sizeof(hspi2));
#endif /* STM32WBAxx */
  memset(&hlptim1, 0u, sizeof(hlptim1));

  #if (CFG_LOG_SUPPORTED)
  memset(&handle_GPDMA1_Channel5, 0u, sizeof(handle_GPDMA1_Channel5));
  memset(&handle_GPDMA1_Channel4, 0u, sizeof(handle_GPDMA1_Channel4));
  memset(&huart1, 0u, sizeof(huart1));
#endif

  MX_GPIO_Init();
  MX_GPDMA1_Init();
  MX_ICACHE_Init();
  MX_RAMCFG_Init();
  MX_RTC_Init();
#if (CFG_LOG_SUPPORTED)
  MX_USART1_UART_Init();
#endif /* CFG_LOG_SUPPORTED */
  CRCCTRL_Init();
#if (USE_TEMPERATURE_BASED_RADIO_CALIBRATION == 1)
  ADCCTRL_Init();
#endif /* USE_TEMPERATURE_BASED_RADIO_CALIBRATION */
  /* USER CODE BEGIN MX_STANDBY_EXIT_PERIPHERAL_INIT_2 */

  /*Enable NVIC for Wkup pins*/
  HAL_NVIC_SetPriority(WKUP_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(WKUP_IRQn);
  /* USER CODE END MX_STANDBY_EXIT_PERIPHERAL_INIT_2 */
}
