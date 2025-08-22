/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "cmsis_os2.h"
#include "gpio.h"
#include "dma.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sid_pal_log_like.h"

#if defined(NUCLEO_WL55_BOARD)
#  include <stm32wlxx_nucleo.h>
#endif /* NUCLEO_WL55_BOARD */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef SIDEWALK_RADIO_SPI;
DMA_HandleTypeDef SIDEWALK_RADIO_SPI_DMA_TX;
DMA_HandleTypeDef SIDEWALK_RADIO_SPI_DMA_RX;
DMA_HandleTypeDef SIDEWALK_RADIO_SPI_DMA_NSS_DETECT;
DMA_HandleTypeDef SIDEWALK_RADIO_SPI_DMA_SCK_CTRL;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
#if defined(INTER_MCU_BUS_SPI1)
  MX_SPI1_Init();
#elif defined(INTER_MCU_BUS_SPI2)
  MX_SPI2_Init();
#else
#  error "Unsupported host MCU SPI bus"
#endif /* INTER_MCU_BUS_SPI1 */
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Configure the main internal regulator output voltage
  */

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS_PWR;
  RCC_OscInitStruct.HSEDiv = RCC_HSE_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 6;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

#if defined(INTER_MCU_BUS_SPI1)
/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
void MX_SPI1_Init(void)
#elif defined(INTER_MCU_BUS_SPI2)
/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
void MX_SPI2_Init(void)
#endif
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  SIDEWALK_RADIO_SPI.Instance = ((SPI_TypeDef *)SIDEWALK_RADIO_SPI_INSTANCE_BASE);
  SIDEWALK_RADIO_SPI.Init.Mode = SPI_MODE_SLAVE;
  SIDEWALK_RADIO_SPI.Init.Direction = SPI_DIRECTION_2LINES;
  SIDEWALK_RADIO_SPI.Init.DataSize = SPI_DATASIZE_8BIT;
  SIDEWALK_RADIO_SPI.Init.CLKPolarity = SPI_POLARITY_LOW;
  SIDEWALK_RADIO_SPI.Init.CLKPhase = SPI_PHASE_1EDGE;
  SIDEWALK_RADIO_SPI.Init.NSS = SPI_NSS_SOFT;
  SIDEWALK_RADIO_SPI.Init.FirstBit = SPI_FIRSTBIT_MSB;
  SIDEWALK_RADIO_SPI.Init.TIMode = SPI_TIMODE_DISABLE;
  SIDEWALK_RADIO_SPI.Init.CRCCalculation = SPI_CRCCALCULATION_ENABLE;
  SIDEWALK_RADIO_SPI.Init.CRCPolynomial = 0x1021;
  SIDEWALK_RADIO_SPI.Init.CRCLength = SPI_CRC_LENGTH_16BIT;
  SIDEWALK_RADIO_SPI.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&SIDEWALK_RADIO_SPI) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* Disable all interrupts */
  __disable_irq();

#if APP_LOG_ENABLED
  /* Temporarily re-enable DMA interrupts that are relevant for logging to flush the buffer */

  /* Disable all NVIC interrupts except the ones that are relevant to log flushing */
#ifdef CORE_CM0PLUS
  const register uint32_t disable_mask = 0xFFFFFFFFu & (~(1u << APP_LOG_DMA_TX_CHANNEL_IRQn) & ~(1u << APP_LOG_UART_IRQn));
  NVIC->ICER[0] = disable_mask;
#elif defined (CORE_CM4)
  register uint32_t disable_mask[2] = {0xFFFFFFFFu, 0xFFFFFFFFu};
  disable_mask[APP_LOG_DMA_TX_CHANNEL_IRQn >> 5] &= ~(1 << (APP_LOG_DMA_TX_CHANNEL_IRQn & 0x1Fu));
  disable_mask[APP_LOG_UART_IRQn >> 5]           &= ~(1 << (APP_LOG_UART_IRQn & 0x1Fu));
  NVIC->ICER[0] = disable_mask[0];
  NVIC->ICER[1] = disable_mask[1];
#else
#  error "Unknown CPU core architecture, the implementation above may not be compatible with it"
#endif /* CORE_CM0PLUS */
  __DSB();
  __ISB();

  /* Unmask interrupts with priorities */
  __set_PRIMASK(0u);

  /* Wait for log buffer flush */
  while (UTIL_ADV_TRACE_IsBufferEmpty() == 0u)
  {
    __NOP();
  }

  /* Finally disable all interrupts */
  __disable_irq();
#endif

  /* Provide visual error indication if possible */
#if CFG_LED_SUPPORTED
  BSP_LED_Off(LED_GREEN);
  BSP_LED_Off(LED_BLUE);
  BSP_LED_On(LED_RED);
#endif

  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  __disable_irq();
  SID_PAL_LOG_ERROR("\e[1;35m" "ASSERT" "\e[0m" " line %u %s", line, file);
  Error_Handler();
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
