/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_entry.c
  * @author  MCD Application Team
  * @brief   Entry point of the application
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
#include "app_common.h"
#include "log_module.h"
#include "app_conf.h"
#include "main.h"
#include "app_entry.h"
#include "stm32_rtos.h"
#include "stm32_timer.h"
#include "advanced_memory_manager.h"
#include "stm32_mm.h"

#include "stm32_adv_trace.h"
#include "app_ble.h"
#include "ll_sys_if.h"
#include "app_sys.h"
#include "otp.h"
#include "scm.h"
#include "bpka.h"
#include "ll_sys.h"
#include "flash_driver.h"
#include "flash_manager.h"
#include "simple_nvm_arbiter.h"
#include "app_debug.h"
#include "adc_ctrl.h"
#include "crc_ctrl.h"
#if (USE_TEMPERATURE_BASED_RADIO_CALIBRATION)
#include "temp_measurement.h"
#endif /* USE_TEMPERATURE_BASED_RADIO_CALIBRATION */

/* Private includes -----------------------------------------------------------*/

#if defined(NUCLEO_WBA52_BOARD) || defined(NUCLEO_WBA55_BOARD) || defined (NUCLEO_WBA65_BOARD)
#  include "stm32wbaxx_nucleo.h"
#endif
#include "adv_trace_usart_if.h"
#include "app_sidewalk.h"
#include "common_memory_symbols.h"
#include <sid_pal_critical_region_ifc.h>
#include <sid_stm32_common_utils.h>

#if (CFG_LPM_LEVEL != 0)
#include "stm32_lpm.h"
#include "timer_if.h"
#endif /* (CFG_LPM_LEVEL != 0) */

#if (configCHECK_FOR_STACK_OVERFLOW > 0)
#include <sid_pal_log_ifc.h>
#endif /* (configCHECK_FOR_STACK_OVERFLOW > 0) */

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private defines -----------------------------------------------------------*/

#define STM32WBAXX_SI_CUT1_0_DBGMCU_REVID_REG_VAL (0x1000u)

/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macros ------------------------------------------------------------*/

/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if ( CFG_LPM_LEVEL != 0)
static uint8_t  system_startup_done = FALSE;
static UTIL_TIMER_Object_t TimerOSwakeup_Id;
#endif /* ( CFG_LPM_LEVEL != 0) */

#if (CFG_LOG_SUPPORTED != 0)
/* Log configuration
 * .verbose_level can be any value of the Log_Verbose_Level_t enum.
 * .region_mask can either be :
 * - LOG_REGION_ALL_REGIONS to enable all regions
 * or
 * - One or several specific regions (any value except LOG_REGION_ALL_REGIONS)
 *   from the Log_Region_t enum and matching the mask value.
 *
 *   For example, to enable both LOG_REGION_BLE and LOG_REGION_APP,
 *   the value assigned to the define is :
 *   (1U << LOG_REGION_BLE | 1U << LOG_REGION_APP)
 */
static Log_Module_t Log_Module_Config = { .verbose_level = APPLI_CONFIG_LOG_LEVEL, .region_mask = APPLI_CONFIG_LOG_REGION };
#endif /* (CFG_LOG_SUPPORTED != 0) */

/* AMM configuration */
static uint32_t AMM_Pool[CFG_AMM_POOL_SIZE];
static AMM_VirtualMemoryConfig_t vmConfig[CFG_AMM_VIRTUAL_MEMORY_NUMBER] =
{
  /* Virtual Memory #1 */
  {
    .Id = CFG_AMM_VIRTUAL_STACK_BLE,
    .BufferSize = CFG_AMM_VIRTUAL_STACK_BLE_BUFFER_SIZE
  },
  /* Virtual Memory #2 */
  {
    .Id = CFG_AMM_VIRTUAL_APP_BLE,
    .BufferSize = CFG_AMM_VIRTUAL_APP_BLE_BUFFER_SIZE
  },
};

static AMM_InitParameters_t ammInitConfig =
{
  .p_PoolAddr = AMM_Pool,
  .PoolSize = CFG_AMM_POOL_SIZE,
  .VirtualMemoryNumber = CFG_AMM_VIRTUAL_MEMORY_NUMBER,
  .p_VirtualMemoryConfigList = vmConfig
};

/* RTOS objects declaration */

static osThreadId_t     AmmTaskHandle;
static osSemaphoreId_t  AmmSemaphore;

static const osThreadAttr_t AmmTask_attributes = {
  .name         = "AMM Task",
  .priority     = TASK_PRIO_AMM,
  .stack_size   = TASK_STACK_SIZE_AMM,
  .attr_bits    = TASK_DEFAULT_ATTR_BITS,
  .cb_mem       = TASK_DEFAULT_CB_MEM,
  .cb_size      = TASK_DEFAULT_CB_SIZE,
  .stack_mem    = TASK_DEFAULT_STACK_MEM,
};

static const osSemaphoreAttr_t AmmSemaphore_attributes = {
  .name         = "AMM Semaphore",
  .attr_bits    = SEMAPHORE_DEFAULT_ATTR_BITS,
  .cb_mem       = SEMAPHORE_DEFAULT_CB_MEM,
  .cb_size      = SEMAPHORE_DEFAULT_CB_SIZE,
};

static osThreadId_t     RngTaskHandle;
static osSemaphoreId_t  RngSemaphore;
static osSemaphoreId_t  RngPoolSemaphore;

static const osThreadAttr_t RngTask_attributes = {
  .name         = "Random Number Generator Task",
  .priority     = TASK_PRIO_RNG,
  .stack_size   = TASK_STACK_SIZE_RNG,
  .attr_bits    = TASK_DEFAULT_ATTR_BITS,
  .cb_mem       = TASK_DEFAULT_CB_MEM,
  .cb_size      = TASK_DEFAULT_CB_SIZE,
  .stack_mem    = TASK_DEFAULT_STACK_MEM,
};

static const osSemaphoreAttr_t RngSemaphore_attributes = {
  .name         = "Random Number Generator Semaphore",
  .attr_bits    = SEMAPHORE_DEFAULT_ATTR_BITS,
  .cb_mem       = SEMAPHORE_DEFAULT_CB_MEM,
  .cb_size      = SEMAPHORE_DEFAULT_CB_SIZE,
};

static const osSemaphoreAttr_t RngPoolSemaphore_attributes = {
  .name = "Random Number Generator Pool Semaphore",
  .attr_bits    = SEMAPHORE_DEFAULT_ATTR_BITS,
  .cb_mem       = SEMAPHORE_DEFAULT_CB_MEM,
  .cb_size      = SEMAPHORE_DEFAULT_CB_SIZE,
};

static osThreadId_t     FlashManagerTaskHandle;
static osSemaphoreId_t  FlashManagerSemaphore;

static const osThreadAttr_t FlashManagerTask_attributes = {
  .name         = "FLASH Manager Task",
  .priority     = TASK_PRIO_FLASH_MANAGER,
  .stack_size   = TASK_STACK_SIZE_FLASH_MANAGER,
  .attr_bits    = TASK_DEFAULT_ATTR_BITS,
  .cb_mem       = TASK_DEFAULT_CB_MEM,
  .cb_size      = TASK_DEFAULT_CB_SIZE,
  .stack_mem    = TASK_DEFAULT_STACK_MEM
};

static const osSemaphoreAttr_t FlashManagerSemaphore_attributes = {
  .name         = "FLASH Manager Semaphore",
  .attr_bits    = SEMAPHORE_DEFAULT_ATTR_BITS,
  .cb_mem       = SEMAPHORE_DEFAULT_CB_MEM,
  .cb_size      = SEMAPHORE_DEFAULT_CB_SIZE,
};

static osThreadId_t     BpkaTaskHandle;
static osSemaphoreId_t  BpkaSemaphore;

static const osThreadAttr_t BpkaTask_attributes = {
  .name         = "BPKA Task",
  .priority     = TASK_PRIO_BPKA,
  .stack_size   = TASK_STACK_SIZE_BPKA,
  .attr_bits    = TASK_DEFAULT_ATTR_BITS,
  .cb_mem       = TASK_DEFAULT_CB_MEM,
  .cb_size      = TASK_DEFAULT_CB_SIZE,
  .stack_mem    = TASK_DEFAULT_STACK_MEM
};

static const osSemaphoreAttr_t BpkaSemaphore_attributes = {
  .name         = "BPKA Semaphore",
  .attr_bits    = TASK_DEFAULT_ATTR_BITS,
  .cb_mem       = TASK_DEFAULT_CB_MEM,
  .cb_size      = TASK_DEFAULT_CB_SIZE
};

static osMutexId_t      crcCtrlMutex;

static const osMutexAttr_t crcCtrlMutex_attributes = {
  .name         = "CRC CTRL Mutex",
  .attr_bits    = MUTEX_DEFAULT_ATTR_BITS,
  .cb_mem       = MUTEX_DEFAULT_CB_MEM,
  .cb_size      = MUTEX_DEFAULT_CB_SIZE,
};

static osMutexId_t      adcCtrlMutex;

static const osMutexAttr_t adcCtrlMutex_attributes = {
  .name         = "ADC CTRL Mutex",
  .attr_bits    = osMutexRecursive, /* The use of a recursive mutex allows to lock ADC for a series of individual measurements */
  .cb_mem       = MUTEX_DEFAULT_CB_MEM,
  .cb_size      = MUTEX_DEFAULT_CB_SIZE,
};

static osMutexId_t      advTraceMutex;

static const osMutexAttr_t advTraceMutex_attributes = {
  .name         = "Adv Trace Mutex",
  .attr_bits    = osMutexRecursive,
  .cb_mem       = MUTEX_DEFAULT_CB_MEM,
  .cb_size      = MUTEX_DEFAULT_CB_SIZE,
};

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Global variables ----------------------------------------------------------*/

/* USER CODE BEGIN GV */

/* USER CODE END GV */

/* Private functions prototypes-----------------------------------------------*/

static void Config_HSE(void);
static void System_Init(void);
static void SystemPower_Config(void);

static void RNG_Task_Entry(void * argument);
static void APPE_RNG_Init(void);

static void APPE_AMM_Init(void);
static void AMM_Task_Entry(void * argument);
static void AMM_WrapperInit(uint32_t * const p_PoolAddr, const uint32_t PoolSize);
static uint32_t * AMM_WrapperAllocate(const uint32_t BufferSize);
static void AMM_WrapperFree(uint32_t * const p_BufferAddr);

static void APPE_FLASH_MANAGER_Init(void);
static void FLASH_Manager_Task_Entry(void * argument);

static void APPE_BPKA_Init(void);
static void BPKA_Task_Entry(void * argument);

/* USER CODE BEGIN PFP */
#if (CFG_LED_SUPPORTED)
static void Led_Init(void);
#endif
#if ( CFG_LPM_LEVEL != 0)
static void TimerOSwakeupCB(void *arg);
#endif /* ( CFG_LPM_LEVEL != 0) */

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Functions Definition ------------------------------------------------------*/

void MX_APPE_Config(void)
{
  /* Configure HSE Tuning */
  Config_HSE();
}

uint32_t MX_APPE_Init(void * p_param)
{
  APP_DEBUG_SIGNAL_SET(APP_APPE_INIT);

  UNUSED(p_param);

  /* System initialization */
  System_Init();

  /* Configure the system Power Mode */
  SystemPower_Config();

#if (USE_TEMPERATURE_BASED_RADIO_CALIBRATION)
  /* Initialize the Temperature measurement */
  TEMPMEAS_Init ();
#endif /* (USE_TEMPERATURE_BASED_RADIO_CALIBRATION) */

  /* Initialize the Advance Memory Manager module */
  APPE_AMM_Init();

  /* Initialize the Random Number Generator module */
  APPE_RNG_Init();

  /* Initialize the Flash Manager module */
  APPE_FLASH_MANAGER_Init();

  /* USER CODE BEGIN APPE_Init_1 */
#if (CFG_LED_SUPPORTED)
  Led_Init();
#endif

#if (CFG_SCM_SUPPORTED)
  /* Inform SCM that we want to run the App on PLL clock */
  scm_setsystemclock(SCM_USER_APP, SYS_PLL);
#endif /* CFG_SCM_SUPPORTED */
  /* USER CODE END APPE_Init_1 */

  /* Initialize the Ble Public Key Accelerator module */
  APPE_BPKA_Init();

  /* Initialize the Simple NVM Arbiter */
  SNVMA_Init ((uint32_t *)CFG_SNVMA_START_ADDRESS);

  /* USER CODE BEGIN APPE_Init_2 */
#ifdef STM32WBA5x
  /* Check MCU revision */ 
  const uint32_t mcu_rev_id = HAL_GetREVID();
#  ifdef STM32WBAXX_SI_CUT1_0
  if(mcu_rev_id != STM32WBAXX_SI_CUT1_0_DBGMCU_REVID_REG_VAL)
  {
    APP_DBG_MSG("Project is built for STM32WBA5x Rev.A, but detected revision is newer than Rev.A\n");
    Error_Handler();
  }
#  else
  if(mcu_rev_id == STM32WBAXX_SI_CUT1_0_DBGMCU_REVID_REG_VAL)
  {
    APP_DBG_MSG("Project is built for STM32WBA5x post Rev.A, but detected revision is Rev.A\n");
    Error_Handler();
  }
#  endif /* STM32WBAXX_SI_CUT1_0 */
#endif /* STM32WBA5x */

#if ( CFG_LPM_LEVEL != 0)
  /* create a SW timer to wakeup system from low power */
  UTIL_TIMER_Create(&TimerOSwakeup_Id,
                    0u, /* TImer period is set dynamically on activation */
                    UTIL_TIMER_ONESHOT,
                    &TimerOSwakeupCB, NULL);
#endif /* ( CFG_LPM_LEVEL != 0) */

  SID_APP_Init();

  /* Keep  RFTS Bypass (FD_FLASHACCESS_RFTS_BYPASS) enabled, it is managed by Sidewalk BLE driver */

  /* USER CODE END APPE_Init_2 */

  APP_DEBUG_SIGNAL_RESET(APP_APPE_INIT);

  return WPAN_SUCCESS;
}

/* USER CODE BEGIN FD */

/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

static void Config_HSE(void)
{
  OTP_Data_s* otp_ptr = NULL;

  /**
   * Read HSE_Tuning from OTP
   */
  if (OTP_Read(DEFAULT_OTP_IDX, &otp_ptr) != HAL_OK) {
    /* OTP no present in flash, apply default gain */
    HAL_RCCEx_HSESetTrimming(0x0C);
  }
  else
  {
    HAL_RCCEx_HSESetTrimming(otp_ptr->hsetune);
  }
}

/*----------------------------------------------------------------------------*/

static void System_Init( void )
{
  /* Clear RCC RESET flag */
  LL_RCC_ClearResetFlags();

  UTIL_TIMER_Init();

#if (CFG_LOG_SUPPORTED != 0)
  /* Initialize the logs ( using the USART ) */
  Log_Module_Init( Log_Module_Config );

  /* Process any errors that triggered a reset */
  Fatal_Error_Report_Handler();
#endif  /* (CFG_LOG_SUPPORTED != 0) */

#ifdef BLE
  ll_sys_platform_init();
#endif /* BLE */

  adcCtrlMutex = osMutexNew(&adcCtrlMutex_attributes);
  if (NULL == adcCtrlMutex)
  {
    /* Can't proceed without mutex for ADC access management */
    Error_Handler();
  }
  ADCCTRL_Init();

  crcCtrlMutex = osMutexNew(&crcCtrlMutex_attributes);
  if (NULL == crcCtrlMutex)
  {
    /* Can't proceed without mutex for CRC access management */
    Error_Handler();
  }
  CRCCTRL_Init();

#if(CFG_RT_DEBUG_DTB)
  /* DTB initialization and configuration */
  RT_DEBUG_DTBInit();
  RT_DEBUG_DTBConfig();
#endif /* CFG_RT_DEBUG_DTB */
#if(CFG_RT_DEBUG_GPIO_MODULE)
  /* RT DEBUG GPIO_Init */
  RT_DEBUG_GPIO_Init();
#endif /* (CFG_RT_DEBUG_GPIO_MODULE) */

#if (CFG_DEBUGGER_LEVEL > 1)
  if ((CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) != 0u)
  {
    /* Keep debug facilities up & running in Stop and Standby modes if a debugger is attached at startup */
    LL_DBGMCU_EnableDBGStopMode();
    LL_DBGMCU_EnableDBGStandbyMode();
  }
  else
#endif /* CFG_DEBUGGER_LEVEL */
  {
    /* Disable debug facilities in Stop and Standby modes to allow more accurate power consumption measurements in LPM */
    LL_DBGMCU_DisableDBGStopMode();
    LL_DBGMCU_DisableDBGStandbyMode();
  }

#if (CFG_LPM_LEVEL != 0)
  system_startup_done = TRUE;
#endif /* ( CFG_LPM_LEVEL != 0) */

  return;
}

/*----------------------------------------------------------------------------*/

/**
 * @brief  Configure the system for power optimization
 *
 * @note  This API configures the system to be ready for low power mode
 *
 * @param  None
 * @retval None
 */
static void SystemPower_Config(void)
{
#if (CFG_SCM_SUPPORTED)
  /* Initialize System Clock Manager */
  scm_init();
#endif /* CFG_SCM_SUPPORTED */

#if (CFG_LPM_LEVEL != 0)
  /* Initialize low power manager */
  UTIL_LPM_Init();
  /* Disable Stand-by mode */
  UTIL_LPM_SetOffMode((1 << CFG_LPM_APP), UTIL_LPM_DISABLE);
#endif /* (CFG_LPM_LEVEL != 0)  */
}

/*----------------------------------------------------------------------------*/

static void RNG_Task_Entry(void * argument)
{
  UNUSED(argument);

  for(;;)
  {
    int hw_rng_status;

    osSemaphoreAcquire(RngSemaphore, osWaitForever);
    hw_rng_status = HW_RNG_Process();
    if (HW_OK == hw_rng_status)
    {
      osSemaphoreRelease(RngPoolSemaphore);
    }
  }
}

/*----------------------------------------------------------------------------*/

/**
 * @brief Initialize Random Number Generator module
 */
static void APPE_RNG_Init(void)
{
  HW_RNG_SetPoolThreshold(CFG_HW_RNG_POOL_THRESHOLD);
  HW_RNG_Init();
  HW_RNG_Start();

  /* Create Random Number Generator RTOS objects */

  RngSemaphore = osSemaphoreNew(1U, 0U, &RngSemaphore_attributes);

  RngPoolSemaphore = osSemaphoreNew(1U, 1U, &RngPoolSemaphore_attributes);

  RngTaskHandle = osThreadNew(RNG_Task_Entry, NULL, &RngTask_attributes);

  if ((RngTaskHandle == NULL) || (RngSemaphore == NULL) || (RngPoolSemaphore == NULL))
  {
    LOG_ERROR_APP("RNG RTOS objects creation FAILED");
    Error_Handler();
  }
}

/*----------------------------------------------------------------------------*/

static void APPE_AMM_Init(void)
{
  /* Initialize the Advance Memory Manager */
  if( AMM_Init(&ammInitConfig) != AMM_ERROR_OK )
  {
    Error_Handler();
  }

  /* Create Advance Memory Manager RTOS objects */

  AmmSemaphore = osSemaphoreNew(1U, 0U, &AmmSemaphore_attributes);

  AmmTaskHandle = osThreadNew(AMM_Task_Entry, NULL, &AmmTask_attributes);

  if ((AmmTaskHandle == NULL) || (AmmSemaphore == NULL))
  {
    LOG_ERROR_APP("AMM RTOS objects creation FAILED");
    Error_Handler();
  }
}

/*----------------------------------------------------------------------------*/

static void AMM_Task_Entry(void * argument)
{
  UNUSED(argument);

  for(;;)
  {
    osSemaphoreAcquire(AmmSemaphore, osWaitForever);
    AMM_BackgroundProcess();
  }
}

/*----------------------------------------------------------------------------*/

static void AMM_WrapperInit(uint32_t * const p_PoolAddr, const uint32_t PoolSize)
{
  UTIL_MM_Init ((uint8_t *)p_PoolAddr, ((size_t)PoolSize * sizeof(uint32_t)));
}

/*----------------------------------------------------------------------------*/

static uint32_t * AMM_WrapperAllocate (const uint32_t BufferSize)
{
  return (uint32_t *)UTIL_MM_GetBuffer(((size_t)BufferSize * sizeof(uint32_t)));
}

/*----------------------------------------------------------------------------*/

static void AMM_WrapperFree (uint32_t * const p_BufferAddr)
{
  UTIL_MM_ReleaseBuffer((void *)p_BufferAddr);
}

/*----------------------------------------------------------------------------*/

/**
 * @brief Initialize Flash Manager module
 */
static void APPE_FLASH_MANAGER_Init(void)
{
  /* Create Flash Manager RTOS objects */

  FlashManagerSemaphore = osSemaphoreNew(1U, 0U, &FlashManagerSemaphore_attributes);

  FlashManagerTaskHandle = osThreadNew(FLASH_Manager_Task_Entry, NULL, &FlashManagerTask_attributes);

  if ((FlashManagerTaskHandle == NULL) || (FlashManagerSemaphore == NULL))
  {
    LOG_ERROR_APP("FLASH RTOS objects creation FAILED");
    Error_Handler();
  }

  /* Enable and set FLASH Interrupt priority */
  /* FLASH interrupt is used to handle ECC corrections */
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ECCC);
  __HAL_FLASH_ENABLE_IT(FLASH_IT_ECCC);
  HAL_NVIC_SetPriority(FLASH_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(FLASH_IRQn);

  /* Disable flash before any use - RFTS */
  FD_SetStatus (FD_FLASHACCESS_RFTS, LL_FLASH_DISABLE);
  /* Enable RFTS Bypass for flash operation - Since LL has not started yet */
  FD_SetStatus (FD_FLASHACCESS_RFTS_BYPASS, LL_FLASH_ENABLE);
  /* Enable flash system flag */
  FD_SetStatus (FD_FLASHACCESS_SYSTEM, LL_FLASH_ENABLE);

  return;
}

/*----------------------------------------------------------------------------*/

static void FLASH_Manager_Task_Entry(void * argument)
{
  UNUSED(argument);

  for(;;)
  {
    osSemaphoreAcquire(FlashManagerSemaphore, osWaitForever);
    FM_BackgroundProcess();
  }
}

/*----------------------------------------------------------------------------*/

/**
 * @brief Initialize Ble Public Key Accelerator module
 */
static void APPE_BPKA_Init(void)
{
  /* Create Ble Public Key Accelerator RTOS objects */

  BpkaSemaphore = osSemaphoreNew(1U, 0U, &BpkaSemaphore_attributes);

  BpkaTaskHandle = osThreadNew(BPKA_Task_Entry, NULL, &BpkaTask_attributes);

  if ((BpkaTaskHandle == NULL) || (BpkaSemaphore == NULL))
  {
    LOG_ERROR_APP("BPKA RTOS objects creation FAILED");
    Error_Handler();
  }

}

/*----------------------------------------------------------------------------*/

static void BPKA_Task_Entry(void * argument)
{
  UNUSED(argument);

  for(;;)
  {
    osSemaphoreAcquire(BpkaSemaphore, osWaitForever);
    BPKA_BG_Process();
  }
}

/*----------------------------------------------------------------------------*/

#if (CFG_LPM_LEVEL != 0)
/* OS wakeup callback */
static void TimerOSwakeupCB(void *arg)
{
  /* USER CODE BEGIN TimerOSwakeupCB */
  (void)arg;
  /* USER CODE END TimerOSwakeupCB */
  return;
}
#endif /* (CFG_LPM_LEVEL != 0) */

/*----------------------------------------------------------------------------*/

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS */
#if (CFG_LED_SUPPORTED)
static void Led_Init(void)
{
  /* Leds Initialization */
#if defined(NUCLEO_WBA65_BOARD)
  BSP_LED_Init(LED_BLUE);
#endif /* NUCLEO_WBA65_BOARD */
#if defined(NUCLEO_WBA52_BOARD) || defined(NUCLEO_WBA55_BOARD) || defined(NUCLEO_WBA65_BOARD)
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_RED);

  BSP_LED_On(LED_GREEN);
#else
#warning "Unknown board, usable LEDs are not defined"
#endif /* NUCLEO_WBA52_BOARD || NUCLEO_WBA55_BOARD || NUCLEO_WBA65_BOARD */

  return;
}
#endif /* CFG_LED_SUPPORTED */

/* USER CODE END FD_LOCAL_FUNCTIONS */

/*************************************************************
 *
 * WRAP FUNCTIONS
 *
 *************************************************************/

void BPKACB_Process(void)
{
  osSemaphoreRelease(BpkaSemaphore);
}

/*----------------------------------------------------------------------------*/

/**
 * @brief Callback used by Random Number Generator to launch Task to generate Random Numbers
 */
void HWCB_RNG_Process( void )
{
  osSemaphoreRelease(RngSemaphore);
}

/*----------------------------------------------------------------------------*/

void AMM_RegisterBasicMemoryManager(AMM_BasicMemoryManagerFunctions_t * const p_BasicMemoryManagerFunctions)
{
  /* Fulfill the function handle */
  p_BasicMemoryManagerFunctions->Init = AMM_WrapperInit;
  p_BasicMemoryManagerFunctions->Allocate = AMM_WrapperAllocate;
  p_BasicMemoryManagerFunctions->Free = AMM_WrapperFree;
}

/*----------------------------------------------------------------------------*/

void AMM_ProcessRequest(void)
{
  /* Trigger to call Advance Memory Manager process function */
  osSemaphoreRelease(AmmSemaphore);
}

/*----------------------------------------------------------------------------*/

void FM_ProcessRequest(void)
{
  /* Trigger to call Flash Manager process function */
  osSemaphoreRelease(FlashManagerSemaphore);
}

/*----------------------------------------------------------------------------*/

#if (CFG_LOG_SUPPORTED != 0)
SID_STM32_SPEED_OPTIMIZED void UTIL_ADV_TRACE_PreSendHook(void)
{
  /* USER CODE BEGIN UTIL_ADV_TRACE_PreSendHook_0 */

  /* USER CODE END UTIL_ADV_TRACE_PreSendHook_0 */
#if (CFG_LPM_LEVEL != 0)
  UTIL_LPM_SetStopMode((1 << CFG_LPM_LOG), UTIL_LPM_DISABLE);
#endif /* (CFG_LPM_LEVEL != 0) */
  /* USER CODE BEGIN UTIL_ADV_TRACE_PreSendHook_1 */

  /* USER CODE END UTIL_ADV_TRACE_PreSendHook_1 */
}
#endif /* (CFG_LOG_SUPPORTED != 0) */

/*----------------------------------------------------------------------------*/

#if (CFG_LOG_SUPPORTED != 0)
SID_STM32_SPEED_OPTIMIZED void UTIL_ADV_TRACE_PostSendHook(void)
{
  /* USER CODE BEGIN UTIL_ADV_TRACE_PostSendHook_0 */

  /* USER CODE END UTIL_ADV_TRACE_PostSendHook_0 */
#if (CFG_LPM_LEVEL != 0)
  UTIL_LPM_SetStopMode((1 << CFG_LPM_LOG), UTIL_LPM_ENABLE);
#endif /* (CFG_LPM_LEVEL != 0) */
  /* USER CODE BEGIN UTIL_ADV_TRACE_PostSendHook_1 */

  /* USER CODE END UTIL_ADV_TRACE_PostSendHook_1 */
}
#endif /* (CFG_LOG_SUPPORTED != 0) */

/*----------------------------------------------------------------------------*/

/* USER CODE BEGIN FD_WRAP_FUNCTIONS */
#if ((CFG_LOG_SUPPORTED == 0) && (CFG_LPM_LEVEL != 0) && (CFG_SCM_SUPPORTED == 0))
/* RNG module turn off HSI clock when traces are not used and low power used */
SID_STM32_SPEED_OPTIMIZED void RNG_KERNEL_CLK_OFF(void)
{
  /* USER CODE BEGIN RNG_KERNEL_CLK_OFF_1 */

  /* USER CODE END RNG_KERNEL_CLK_OFF_1 */
  LL_RCC_HSI_Disable();
  /* USER CODE BEGIN RNG_KERNEL_CLK_OFF_2 */

  /* USER CODE END RNG_KERNEL_CLK_OFF_2 */
}
#endif /* ((CFG_LOG_SUPPORTED == 0) && (CFG_LPM_LEVEL != 0)  && (CFG_SCM_SUPPORTED == 0)) */

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED void HW_RNG_WaitPoolIsFull(void)
{
  /* USER CODE BEGIN HW_RNG_WaitPoolIsFull_0 */

  /* USER CODE END HW_RNG_WaitPoolIsFull_0 */
  (void)osSemaphoreAcquire(RngPoolSemaphore, osWaitForever);
  /* USER CODE BEGIN HW_RNG_WaitPoolIsFull_1 */

  /* USER CODE END HW_RNG_WaitPoolIsFull_1 */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED CRCCTRL_Cmd_Status_t CRCCTRL_MutexTake(void)
{
  osStatus_t os_status;
  CRCCTRL_Cmd_Status_t crc_status;
  /* USER CODE BEGIN CRCCTRL_MutexTake_0 */

  /* USER CODE END CRCCTRL_MutexTake_0 */
  os_status = osMutexAcquire(crcCtrlMutex, osWaitForever);
  crc_status = (osOK == os_status) ? CRCCTRL_OK : CRCCTRL_NOK;
  /* USER CODE BEGIN CRCCTRL_MutexTake_1 */

  /* USER CODE END CRCCTRL_MutexTake_1 */
  return crc_status;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED CRCCTRL_Cmd_Status_t CRCCTRL_MutexRelease(void)
{
  osStatus_t os_status;
  CRCCTRL_Cmd_Status_t crc_status;
  /* USER CODE BEGIN CRCCTRL_MutexRelease_0 */

  /* USER CODE END CRCCTRL_MutexRelease_0 */
  os_status = osMutexRelease(crcCtrlMutex);
  crc_status = (osOK == os_status) ? CRCCTRL_OK : CRCCTRL_NOK;
  /* USER CODE BEGIN CRCCTRL_MutexRelease_1 */

  /* USER CODE END CRCCTRL_MutexRelease_1 */
  return crc_status;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED ADCCTRL_Cmd_Status_t ADCCTRL_MutexTake(void)
{
  osStatus_t os_status;
  ADCCTRL_Cmd_Status_t adc_status;
  /* USER CODE BEGIN ADCCTRL_MutexTake_0 */

  /* USER CODE END ADCCTRL_MutexTake_0 */
  os_status = osMutexAcquire(adcCtrlMutex, osWaitForever);
  adc_status = (osOK == os_status) ? ADCCTRL_OK : ADCCTRL_NOK;
  /* USER CODE BEGIN ADCCTRL_MutexTake_1 */

  /* USER CODE END ADCCTRL_MutexTake_1 */
  return adc_status;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED ADCCTRL_Cmd_Status_t ADCCTRL_MutexRelease(void)
{
  osStatus_t os_status;
  ADCCTRL_Cmd_Status_t adc_status;
  /* USER CODE BEGIN ADCCTRL_MutexRelease_0 */

  /* USER CODE END ADCCTRL_MutexRelease_0 */
  os_status = osMutexRelease(adcCtrlMutex);
  adc_status = (osOK == os_status) ? ADCCTRL_OK : ADCCTRL_NOK;
  /* USER CODE BEGIN ADCCTRL_MutexRelease_1 */

  /* USER CODE END ADCCTRL_MutexRelease_1 */
  return adc_status;
}

/*----------------------------------------------------------------------------*/

void UTIL_ADV_TRACE_CriticalSectionInit(void)
{
  /* USER CODE BEGIN UTIL_ADV_TRACE_CriticalSectionInit_0 */

  /* USER CODE END UTIL_ADV_TRACE_CriticalSectionInit_0 */
  if (NULL == advTraceMutex)
  {
    advTraceMutex = osMutexNew(&advTraceMutex_attributes);
    if (NULL == advTraceMutex)
    {
      /* Can't proceed without mutex for ADV_TRACE access management */
      Error_Handler();
    }
  }
  /* USER CODE BEGIN UTIL_ADV_TRACE_CriticalSectionInit_1 */

  /* USER CODE END UTIL_ADV_TRACE_CriticalSectionInit_1 */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED uint32_t UTIL_ADV_TRACE_EnterCriticalSection(void)
{
  osStatus_t os_status;
  uint32_t mutex_acquired;

  /* USER CODE BEGIN UTIL_ADV_TRACE_EnterCriticalSection_0 */

  /* USER CODE END UTIL_ADV_TRACE_EnterCriticalSection_0 */
  os_status = osMutexAcquire(advTraceMutex, osWaitForever);
  if (osOK == os_status)
  {
    mutex_acquired = TRUE;
  }
  else
  {
    mutex_acquired = FALSE;
    sid_pal_enter_critical_region();
  }
  /* USER CODE BEGIN UTIL_ADV_TRACE_EnterCriticalSection_1 */

  /* USER CODE END UTIL_ADV_TRACE_EnterCriticalSection_1 */

  return mutex_acquired;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED void UTIL_ADV_TRACE_ExitCriticalSection(const uint32_t mutext_used)
{
  /* USER CODE BEGIN UTIL_ADV_TRACE_ExitCriticalSection_0 */

  /* USER CODE END UTIL_ADV_TRACE_ExitCriticalSection_0 */
  if (mutext_used != FALSE)
  {
    (void)osMutexRelease(advTraceMutex);
  }
  else
  {
    sid_pal_exit_critical_region();
  }
  /* USER CODE BEGIN UTIL_ADV_TRACE_ExitCriticalSection_1 */

  /* USER CODE END UTIL_ADV_TRACE_ExitCriticalSection_1 */
}

/* USER CODE END FD_WRAP_FUNCTIONS */

/*----------------------------------------------------------------------------*/

#if (CFG_LPM_LEVEL != 0)
SID_STM32_SPEED_OPTIMIZED void MX_APPE_EnterLPM(uint32_t expectedSleepDurationMs)
{
  if (APPE_OS_SLEEP_NO_WAKEUP == expectedSleepDurationMs)
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
    (void)UTIL_TIMER_StartWithPeriod(&TimerOSwakeup_Id, expectedSleepDurationMs);
  }

  LL_PWR_ClearFlag_STOP();

  if ((system_startup_done != FALSE) && (UTIL_LPM_GetMode() == UTIL_LPM_OFFMODE))
  {
    APP_SYS_BLE_EnterDeepSleep();
  }

  LL_RCC_ClearResetFlags();

#  if defined(STM32WBA5x) && defined(STM32WBAXX_SI_CUT1_0)
  /* Wait until HSE is ready */
  while (LL_RCC_HSE_IsReady() == 0);

  UTILS_ENTER_LIMITED_CRITICAL_SECTION(RCC_INTR_PRIO << 4U);
  // FIXME: SCM may not be in use, scm_hserdy_isr may be empty
  scm_hserdy_isr();
  UTILS_EXIT_LIMITED_CRITICAL_SECTION();
#  endif /* STM32WBA5x && STM32WBAXX_SI_CUT1_0 */

  UTIL_LPM_EnterLowPower(); /* WFI instruction call is inside this API */
}
#endif /* (CFG_LPM_LEVEL != 0) */

/*----------------------------------------------------------------------------*/

#if (CFG_LPM_LEVEL != 0)
SID_STM32_SPEED_OPTIMIZED void MX_APPE_LeaveLPM(void)
{
  (void)UTIL_TIMER_Stop(&TimerOSwakeup_Id);

  /* Put the radio in active state */
  if ((system_startup_done != FALSE) && (UTIL_LPM_GetMode() == UTIL_LPM_OFFMODE))
  {
    LL_AHB5_GRP1_EnableClock(LL_AHB5_GRP1_PERIPH_RADIO);
    ll_sys_dp_slp_exit();
    UTIL_LPM_SetOffMode(1U << CFG_LPM_LL_DEEPSLEEP, UTIL_LPM_ENABLE);
  }
}
#endif /* (CFG_LPM_LEVEL != 0) */
