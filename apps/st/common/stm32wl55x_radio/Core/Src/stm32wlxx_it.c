/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32wlxx_it.c
  * @brief   Interrupt Service Routines.
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
#include "main.h"
#include "stm32wlxx_it.h"
#include "stm32wlxx_ll_exti.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <assert.h>

#include "host_comm.h"
#include "serial_bus_spi_pal.h"
#include "sys_conf.h"
#include "utilities_conf.h"
#include "utilities_def.h"

#include <sid_stm32_common_utils.h>
#include <stm32_lpm_if.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern RTC_HandleTypeDef hrtc;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern UART_HandleTypeDef huart2;
extern SPI_HandleTypeDef SIDEWALK_RADIO_SPI;
extern DMA_HandleTypeDef SIDEWALK_RADIO_SPI_DMA_TX;
extern DMA_HandleTypeDef SIDEWALK_RADIO_SPI_DMA_RX;
extern SUBGHZ_HandleTypeDef hsubghz;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32WLxx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32wlxx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles RTC Tamper, RTC TimeStamp, LSECSS and RTC SSRU Interrupts.
  */
void TAMP_STAMP_LSECSS_SSRU_IRQHandler(void)
{
  /* USER CODE BEGIN TAMP_STAMP_LSECSS_SSRU_IRQn 0 */

  /* USER CODE END TAMP_STAMP_LSECSS_SSRU_IRQn 0 */
  HAL_RTCEx_SSRUIRQHandler(&hrtc);
  /* USER CODE BEGIN TAMP_STAMP_LSECSS_SSRU_IRQn 1 */

  /* USER CODE END TAMP_STAMP_LSECSS_SSRU_IRQn 1 */
}

/**
  * @brief This function handles EXTI Line 0 Interrupt.
  */
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(BUT1_Pin);

  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles EXTI Line 1 Interrupt.
  */
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */

  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(BUT2_Pin);
  /* USER CODE BEGIN EXTI1_IRQn 1 */

  /* USER CODE END EXTI1_IRQn 1 */
}

/**
  * @brief This function handles EXTI Line 2 Interrupt.
  */
void EXTI2_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_IRQn 0 */
#if defined(INTER_MCU_BUS_SPI1)
  static_assert(SIDEWALK_RADIO_SPI_IRQ_Pin == GPIO_PIN_2);
  if (__HAL_GPIO_EXTI_GET_IT(SIDEWALK_RADIO_SPI_IRQ_Pin) != 0u)
  {
    /* Notify about the GPIO Handshake request */
    sid_host_comm_notify_handshake_requested();

    /* IMPORTANT: do not clear the IRQ flag or re-enable the IRQ here,
     * Host Comm task will take care of that as per Handshake procedure
    */
  }
#else
  /* USER CODE END EXTI2_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
  /* USER CODE BEGIN EXTI2_IRQn 1 */
#endif /* INTER_MCU_BUS_SPI1 */
  /* USER CODE END EXTI2_IRQn 1 */
}

/**
  * @brief This function handles EXTI Line 4 Interrupt.
  */
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */
#if defined(INTER_MCU_BUS_SPI1)
  static_assert(SIDEWALK_RADIO_SPI_NSS_Pin == GPIO_PIN_4);
  if (__HAL_GPIO_EXTI_GET_IT(SIDEWALK_RADIO_SPI_NSS_Pin) != 0u)
  {
    /* Clear IRQ flag ASAP for proper DMAMUX operation if workaround for errata 2.2.1 is applied */
    __HAL_GPIO_EXTI_CLEAR_IT(SIDEWALK_RADIO_SPI_NSS_Pin);
    __COMPILER_BARRIER();

#if SERIAL_BUS_SPI_PAL_NSS_ACTIVATED_GPIO_PIN_STATE == GPIO_PIN_RESET
    /* NSS is active when GPIO state is low */
    if ((SIDEWALK_RADIO_SPI_NSS_GPIO_Port->IDR & SIDEWALK_RADIO_SPI_NSS_Pin) == 0u)
#else
    /* NSS is active when GPIO state is high */
    if ((SIDEWALK_RADIO_SPI_NSS_GPIO_Port->IDR & SIDEWALK_RADIO_SPI_NSS_Pin) != 0u)
#endif /* SERIAL_BUS_SPI_PAL_NSS_ACTIVATED_GPIO_PIN_STATE */
    {
      /**
       * This is SPI activation on frame start. This is part of workaround for errata 2.2.1
       * Don't check for MCU revision in here to save on runtime. If workaround is not active
       * this code branch is unreachable unless there's a systematic failure in the software
       */
      goto exit;
    }

#if (USE_SPI_CRC != 0U)
    /* Check if CRC error occurred */
    if (__HAL_SPI_GET_FLAG(&SIDEWALK_RADIO_SPI, SPI_FLAG_CRCERR))
    {
      /* Call the corresponding SPI IRQ handler since CRC error flag does not generate IRQ trigger on its own */
      SPI1_IRQHandler();
    }
    else
#endif
    {
      /* Notify driver that a new frame is available */
      serial_bus_spi_on_frame_received();
    }

#if defined (LOW_POWER_DISABLE) && (LOW_POWER_DISABLE == 0)
    /* SPI transaction is finished - allow the system to enter LPM */
    UTIL_LPM_SetStopMode((1u << CFG_LPM_HOST_COMM_SPI), UTIL_LPM_ENABLE);
#elif !defined (LOW_POWER_DISABLE)
  #error LOW_POWER_DISABLE not defined
#endif /* LOW_POWER_DISABLE */

exit:
  }
#elif defined(INTER_MCU_BUS_SPI2)
  static_assert(SIDEWALK_RADIO_SPI_IRQ_Pin == GPIO_PIN_4);
  if (__HAL_GPIO_EXTI_GET_IT(SIDEWALK_RADIO_SPI_IRQ_Pin) != 0u)
  {
    /* Notify about the GPIO Handshake request */
    sid_host_comm_notify_handshake_requested();

    /* IMPORTANT: do not clear the IRQ flag or re-enable the IRQ here,
     * Host Comm task will take care of that as per Handshake procedure
    */
  }
#else
  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
  /* USER CODE BEGIN EXTI4_IRQn 1 */
#endif /* INTER_MCU_BUS_SPI1 */
  /* USER CODE END EXTI4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 Channel 5 Interrupt.
  */
void DMA1_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel5_IRQn 0 */

  /* USER CODE END DMA1_Channel5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
  /* USER CODE BEGIN DMA1_Channel5_IRQn 1 */

  /* USER CODE END DMA1_Channel5_IRQn 1 */
}

/**
  * @brief This function handles DMA1 Channel 2 Interrupt.
  */
void DMA1_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_IRQn 0 */

  /* USER CODE END DMA1_Channel2_IRQn 0 */
  HAL_DMA_IRQHandler(&SIDEWALK_RADIO_SPI_DMA_TX);
  /* USER CODE BEGIN DMA1_Channel2_IRQn 1 */

  /* USER CODE END DMA1_Channel2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 Channel 3 Interrupt.
  */
void DMA1_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel3_IRQn 0 */

  /* USER CODE END DMA1_Channel3_IRQn 0 */
  HAL_DMA_IRQHandler(&SIDEWALK_RADIO_SPI_DMA_RX);
  /* USER CODE BEGIN DMA1_Channel3_IRQn 1 */

  /* USER CODE END DMA1_Channel3_IRQn 1 */
}

/**
  * @brief This function handles EXTI Lines [9:5] Interrupt.
  */
SID_STM32_SPEED_OPTIMIZED void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */
#if defined(INTER_MCU_BUS_SPI2)
  static_assert(SIDEWALK_RADIO_SPI_NSS_Pin == GPIO_PIN_9);
  if (__HAL_GPIO_EXTI_GET_IT(SIDEWALK_RADIO_SPI_NSS_Pin) != 0u)
  {
    /* Clear IRQ flag ASAP for proper DMAMUX operation if workaround for errata 2.2.1 is applied */
    __HAL_GPIO_EXTI_CLEAR_IT(SIDEWALK_RADIO_SPI_NSS_Pin);
    __COMPILER_BARRIER();

#if SERIAL_BUS_SPI_PAL_NSS_ACTIVATED_GPIO_PIN_STATE == GPIO_PIN_RESET
    /* NSS is active when GPIO state is low */
    if ((SIDEWALK_RADIO_SPI_NSS_GPIO_Port->IDR & SIDEWALK_RADIO_SPI_NSS_Pin) == 0u)
#else
    /* NSS is active when GPIO state is high */
    if ((SIDEWALK_RADIO_SPI_NSS_GPIO_Port->IDR & SIDEWALK_RADIO_SPI_NSS_Pin) != 0u)
#endif /* SERIAL_BUS_SPI_PAL_NSS_ACTIVATED_GPIO_PIN_STATE */
    {
      /**
       * This is SPI activation on frame start. This is part of workaround for errata 2.2.1
       * Don't check for MCU revision in here to save on runtime. If workaround is not active
       * this code branch is unreachable unless there's a systematic failure in the software
       */
      goto exit;
    }

#if (USE_SPI_CRC != 0U)
    /* Check if CRC error occurred */
    if (__HAL_SPI_GET_FLAG(&SIDEWALK_RADIO_SPI, SPI_FLAG_CRCERR))
    {
      /* Call the corresponding SPI IRQ handler since CRC error flag does not generate IRQ trigger on its own */
      SPI2_IRQHandler();
    }
    else
#endif
    {
      /* Notify driver that a new frame is available */
      serial_bus_spi_on_frame_received();
    }

#if defined (LOW_POWER_DISABLE) && (LOW_POWER_DISABLE == 0)
    /* SPI transaction is finished - allow the system to enter LPM */
    UTIL_LPM_SetStopMode((1u << CFG_LPM_HOST_COMM_SPI), UTIL_LPM_ENABLE);
#elif !defined (LOW_POWER_DISABLE)
  #error LOW_POWER_DISABLE not defined
#endif /* LOW_POWER_DISABLE */

exit:
  }
  else
#endif /* INTER_MCU_BUS_SPI2 */
  {
  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(BUT3_Pin);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */
  }
  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles USART2 Interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles RTC Alarms (A and B) Interrupt.
  */
void RTC_Alarm_IRQHandler(void)
{
  /* USER CODE BEGIN RTC_Alarm_IRQn 0 */

  /* USER CODE END RTC_Alarm_IRQn 0 */
  HAL_RTC_AlarmIRQHandler(&hrtc);
  /* USER CODE BEGIN RTC_Alarm_IRQn 1 */

  /* USER CODE END RTC_Alarm_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
