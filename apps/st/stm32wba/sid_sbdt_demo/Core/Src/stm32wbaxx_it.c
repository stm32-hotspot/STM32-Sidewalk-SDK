/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32wbaxx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023-2025 STMicroelectronics.
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
#include "stm32wbaxx_it.h"
#include "ll_sys.h"
#include "stm32wbaxx_hal.h"
#include <flash_driver.h>
#include <sid_pal_log_ifc.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* External functions --------------------------------------------------------*/
extern void (*radio_callback)(void);
extern void (*low_isr_callback)(void);
extern void sid_pal_gpio_exti_irq_handler(const uint32_t st_pin);

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
extern volatile uint8_t radio_sw_low_isr_is_running_high_prio;
extern RTC_HandleTypeDef hrtc;

/* Log UART Rx and Tx channels */
extern DMA_HandleTypeDef handle_GPDMA1_Channel5;
extern DMA_HandleTypeDef handle_GPDMA1_Channel4;
extern UART_HandleTypeDef huart1;

extern uint32_t littlefs_ecc_error_address;

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
  /* Check if NMI is due to flash ECCD (error detection) */
  if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_ECCD))
  {
#if defined(FLASH_DBANK_SUPPORT)
    const uint32_t physical_bank_id = READ_BIT(FLASH->ECCR, FLASH_ECCR_BK_ECC);
    const uint32_t bank_swap_active = READ_BIT (FLASH->OPTR, FLASH_OPTR_SWAP_BANK_Msk);
    const uint32_t flash_base_addr = FLASH_BASE + (
      (((physical_bank_id == 0u) && (bank_swap_active == OB_SWAP_BANK_DISABLE)) || ((physical_bank_id != 0u) && (bank_swap_active != OB_SWAP_BANK_DISABLE)))
      ? 0u /* Physical bank 1 and no bank swap OR physical bank 2 and bank swap active */
      : (FLASH_SIZE >> 1u) /* Physical bank 2 and no bank swap OR physical bank 1 and bank swap active */
    );
#else
    const uint32_t flash_base_addr = FLASH_BASE;
#endif /* FLASH_DBANK_SUPPORT */
    const uint32_t error_address = ((READ_BIT(FLASH->ECCR, FLASH_ECCR_SYSF_ECC) == 0u) ? flash_base_addr : SYSTEM_FLASH_BASE_NS)
      + ((FLASH->ECCR & FLASH_ECCR_ADDR_ECC_Msk) >> FLASH_ECCR_ADDR_ECC_Pos);

    if ((error_address >= APP_CONFIG_FLASH_START) && (error_address < APP_CONFIG_FLASH_END))
    {
      /* ECC error detected in the LittleFS flash area - store the address of the problematic quad-word */
      littlefs_ecc_error_address = error_address;

      /* Clear ECC detection flag to allow the program to proceed */
      __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ECCD);

      /* Return from here and let the LittleFS port to handle the ECC error */
      return;
    }
#ifdef BLE
    else if ((error_address >= APP_CONFIG_SNVMA_FLASH_START) && (error_address < APP_CONFIG_SNVMA_FLASH_END))
    {
      FD_FlashOp_Status_t fd_status = FD_FLASHOP_SUCCESS;

      /* ECC error detected in the BLE NVM area */
      SID_PAL_LOG_ERROR("Uncorrectable ECC error detected in BLE NVM area at address 0x%08X", error_address);

      /* Erase BLE NVM area to try to get rid of the ECC error */
      (void)HAL_FLASH_Unlock();

      /* Page erase loop */
      for (uint32_t sector_id = CFG_SNVMA_START_SECTOR_ID; sector_id <= CFG_SNVMA_END_SECTOR_ID; sector_id++)
      {
        fd_status = FD_EraseSectors(sector_id);
        if (fd_status != FD_FLASHOP_SUCCESS)
        {
          SID_PAL_LOG_ERROR("Failed to erase BLE NVM page %u", sector_id);
          break;
        }
      }

      (void)HAL_FLASH_Lock();

      if (FD_FLASHOP_SUCCESS == fd_status)
      {
        /* Clear ECC Detection flag on successful erasure to allow the program to proceed */
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ECCD);
        return;
      }
    }
#endif /* BLE */
    else if (FLASH_ECC_TEST_DOUBLE_ERROR_ADDRESS == error_address)
    {
      SID_PAL_LOG_INFO("Double ECC error detection test succeeded");
      /* Clear ECC Detection flag to allow the program to proceed */
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ECCD);
        return;
    }
    else
    {
      // TODO: define your app-specific logic here
      SID_PAL_LOG_ERROR("Uncorrectable ECC error detected in flash at address 0x%08X", error_address);
      Error_Handler();
    }
  }
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
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
/* STM32WBAxx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32wbaxx.s).                    */
/******************************************************************************/

/**
  * @brief  This function handles Flash interrupt request.
  * @param  None
  * @retval None
  */
void FLASH_IRQHandler(void)
{
  /* USER CODE BEGIN FLASH_IRQn 0 */

  /* USER CODE END FLASH_IRQn 0 */
  /* Note: ECC correction flag (ECCC) must be cleared after every correction to allow proper address reporting for any subsequent ECC corrections and uncorrectable error reports */
  HAL_FLASH_IRQHandler();
  /* USER CODE BEGIN FLASH_IRQn 1 */

  /* USER CODE END FLASH_IRQn 1 */
}

/**
  * @brief This function handles RTC non-secure interrupt.
  */
void RTC_IRQHandler(void)
{
  /* Get RTC interrupt status */
  const register uint32_t rtc_misr = READ_REG(RTC->MISR);

  if((rtc_misr & RTC_MISR_ALRAMF) || (rtc_misr & RTC_MISR_ALRBMF))
  {
    HAL_RTC_AlarmIRQHandler(&hrtc);
  }
  else if(rtc_misr & RTC_MISR_SSRUMF)
  {
    HAL_RTCEx_SSRUIRQHandler(&hrtc);
  }
}

/**
* @brief This function handles EXTI Line0 interrupt.
*/
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

  /* USER CODE END EXTI0_IRQn 0 */
  sid_pal_gpio_exti_irq_handler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
* @brief This function handles EXTI Line1 interrupt.
*/
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */

  /* USER CODE END EXTI1_IRQn 0 */
  sid_pal_gpio_exti_irq_handler(GPIO_PIN_1);
  /* USER CODE BEGIN EXTI1_IRQn 1 */

  /* USER CODE END EXTI1_IRQn 1 */
}

/**
  * @brief This function handles EXTI Line2 interrupt.
*/
void EXTI2_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_IRQn 0 */

  /* USER CODE END EXTI2_IRQn 0 */
  sid_pal_gpio_exti_irq_handler(GPIO_PIN_2);
  /* USER CODE BEGIN EXTI2_IRQn 1 */

  /* USER CODE END EXTI2_IRQn 1 */
}

/**
* @brief This function handles EXTI Line3 interrupt.
*/
void EXTI3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI3_IRQn 0 */

  /* USER CODE END EXTI3_IRQn 0 */
  sid_pal_gpio_exti_irq_handler(GPIO_PIN_3);
  /* USER CODE BEGIN EXTI3_IRQn 1 */

  /* USER CODE END EXTI3_IRQn 1 */
}

/**
* @brief This function handles EXTI Line4 interrupt.
*/
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */

  /* USER CODE END EXTI4_IRQn 0 */
  sid_pal_gpio_exti_irq_handler(GPIO_PIN_4);
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  /* USER CODE END EXTI4_IRQn 1 */
}

/**
* @brief This function handles EXTI Line5 interrupt.
*/
void EXTI5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI5_IRQn 0 */

  /* USER CODE END EXTI5_IRQn 0 */
  sid_pal_gpio_exti_irq_handler(GPIO_PIN_5);
  /* USER CODE BEGIN EXTI5_IRQn 1 */

  /* USER CODE END EXTI5_IRQn 1 */
}

/**
* @brief This function handles EXTI Line6 interrupt.
*/
void EXTI6_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI6_IRQn 0 */

  /* USER CODE END EXTI6_IRQn 0 */
  sid_pal_gpio_exti_irq_handler(GPIO_PIN_6);
  /* USER CODE BEGIN EXTI6_IRQn 1 */

  /* USER CODE END EXTI6_IRQn 1 */
}

/**
* @brief This function handles EXTI Line7 interrupt.
*/
void EXTI7_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI7_IRQn 0 */

  /* USER CODE END EXTI7_IRQn 0 */
  sid_pal_gpio_exti_irq_handler(GPIO_PIN_7);
  /* USER CODE BEGIN EXTI7_IRQn 1 */

  /* USER CODE END EXTI7_IRQn 1 */
}

/**
* @brief This function handles EXTI Line8 interrupt.
*/
void EXTI8_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI8_IRQn 0 */

  /* USER CODE END EXTI8_IRQn 0 */
  sid_pal_gpio_exti_irq_handler(GPIO_PIN_8);
  /* USER CODE BEGIN EXTI8_IRQn 1 */

  /* USER CODE END EXTI8_IRQn 1 */
}

/**
* @brief This function handles EXTI Line9 interrupt.
*/
void EXTI9_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_IRQn 0 */

  /* USER CODE END EXTI9_IRQn 0 */
  sid_pal_gpio_exti_irq_handler(GPIO_PIN_9);
  /* USER CODE BEGIN EXTI9_IRQn 1 */

  /* USER CODE END EXTI9_IRQn 1 */
}

/**
* @brief This function handles EXTI Line10 interrupt.
*/
void EXTI10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI10_IRQn 0 */

  /* USER CODE END EXTI10_IRQn 0 */
  sid_pal_gpio_exti_irq_handler(GPIO_PIN_10);
  /* USER CODE BEGIN EXTI10_IRQn 1 */

  /* USER CODE END EXTI10_IRQn 1 */
}

/**
* @brief This function handles EXTI Line11 interrupt.
*/
void EXTI11_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI11_IRQn 0 */

  /* USER CODE END EXTI11_IRQn 0 */
  sid_pal_gpio_exti_irq_handler(GPIO_PIN_11);
  /* USER CODE BEGIN EXTI11_IRQn 1 */

  /* USER CODE END EXTI11_IRQn 1 */
}

/**
* @brief This function handles EXTI Line12 interrupt.
*/
void EXTI12_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI12_IRQn 0 */

  /* USER CODE END EXTI12_IRQn 0 */
  sid_pal_gpio_exti_irq_handler(GPIO_PIN_12);
}

/**
* @brief This function handles EXTI Line13 interrupt.
*/
void EXTI13_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI13_IRQn 0 */

  /* USER CODE END EXTI13_IRQn 0 */
  sid_pal_gpio_exti_irq_handler(GPIO_PIN_13);
  /* USER CODE BEGIN EXTI12_IRQn 1 */

  /* USER CODE END EXTI12_IRQn 1 */
}

/**
* @brief This function handles EXTI Line14 interrupt.
*/
void EXTI14_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI14_IRQn 0 */

  /* USER CODE END EXTI14_IRQn 0 */
  sid_pal_gpio_exti_irq_handler(GPIO_PIN_14);
  /* USER CODE BEGIN EXTI14_IRQn 1 */

  /* USER CODE END EXTI14_IRQn 1 */
}

/**
* @brief This function handles EXTI Line15 interrupt.
*/
void EXTI15_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_IRQn 0 */

  /* USER CODE END EXTI15_IRQn 0 */
  sid_pal_gpio_exti_irq_handler(GPIO_PIN_15);
  /* USER CODE BEGIN EXTI15_IRQn 1 */

  /* USER CODE END EXTI15_IRQn 1 */
}

/**
  * @brief This function handles GPDMA1 Channel 4 global interrupt.
  */
void GPDMA1_Channel4_IRQHandler(void)
{
  /* USER CODE BEGIN GPDMA1_Channel4_IRQn 0 */

  /* USER CODE END GPDMA1_Channel4_IRQn 0 */
  HAL_DMA_IRQHandler(&handle_GPDMA1_Channel4);
  /* USER CODE BEGIN GPDMA1_Channel4_IRQn 1 */

  /* USER CODE END GPDMA1_Channel4_IRQn 1 */
}

/**
  * @brief This function handles GPDMA1 Channel 5 global interrupt.
  */
void GPDMA1_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN GPDMA1_Channel5_IRQn 0 */

  /* USER CODE END GPDMA1_Channel5_IRQn 0 */
  HAL_DMA_IRQHandler(&handle_GPDMA1_Channel5);
  /* USER CODE BEGIN GPDMA1_Channel5_IRQn 1 */

  /* USER CODE END GPDMA1_Channel5_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles 2.4GHz RADIO global interrupt.
  */
void RADIO_IRQHandler(void)
{
  /* USER CODE BEGIN RADIO_IRQn 0 */

  /* USER CODE END RADIO_IRQn 0 */

  if(NULL != radio_callback)
  {
    radio_callback();
  }

  LL_RCC_RADIO_DisableSleepTimerClock();
  __ISB();

  /* USER CODE BEGIN RADIO_IRQn 1 */

  /* USER CODE END RADIO_IRQn 1 */
}

/**
  * @brief This function handles PWR global WKUP pin interrupt.
  */
void WKUP_IRQHandler(void)
{
  /* USER CODE BEGIN WKUP_IRQn 0 */

  /* USER CODE END WKUP_IRQn 0 */
  HAL_PWR_WKUP_IRQHandler();
  /* USER CODE BEGIN WKUP_IRQn 1 */

  /* USER CODE END WKUP_IRQn 1 */
}

/**
  * @brief This function handles HASH global interrupt.
  */
void HASH_IRQHandler(void)
{
  /* USER CODE BEGIN HASH_IRQn 0 */

  /* USER CODE END HASH_IRQn 0 */

  /* Disable SW radio low interrupt to prevent nested calls */
  NVIC_DisableIRQ(RADIO_SW_LOW_INTR_NUM);

  if(NULL != low_isr_callback) {
    low_isr_callback();
  }

  /* Check if nested SW radio low interrupt has been requested*/
  if(radio_sw_low_isr_is_running_high_prio != 0) {
    HAL_NVIC_SetPriority((IRQn_Type) RADIO_SW_LOW_INTR_NUM, RADIO_INTR_PRIO_LOW, 0);
    radio_sw_low_isr_is_running_high_prio = 0;
  }

  /* Re-enable SW radio low interrupt */
  NVIC_EnableIRQ(RADIO_SW_LOW_INTR_NUM);

  /* USER CODE BEGIN HASH_IRQn 1 */

  /* USER CODE END HASH_IRQn 1 */
}
