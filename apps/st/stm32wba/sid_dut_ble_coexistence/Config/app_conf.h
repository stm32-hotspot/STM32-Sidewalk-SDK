/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_conf.h
  * @author  MCD Application Team
  * @brief   Application configuration file for STM32WPAN Middleware.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef APP_CONF_H
#define APP_CONF_H

/* Includes ------------------------------------------------------------------*/
#include "hw_if.h"
#include "stm32_adv_trace.h"
#include <common_memory_symbols.h>

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/******************************************************************************
 * Application Config
 ******************************************************************************/
/**< generic parameters ******************************************************/

#if (defined(STM32WBA50xx) || defined(STM32WBA52xx) || defined(STM32WBA54xx) || defined(STM32WBA55xx) || defined(STM32WBA5Mxx)) && !defined(STM32WBA5x)
#  define STM32WBA5x
#elif (defined(STM32WBA62xx) || defined(STM32WBA63xx) || defined(STM32WBA64xx) || defined(STM32WBA65xx) || defined(STM32WBA6Mxx)) && !defined(STM32WBA6x)
#  define STM32WBA6x
#else
#  error "This app supports STM32WBA5x and STM32WBA6x targets only"
#endif

/**
 * Define maximum BLE Tx Power
 * @note Individual connections may set Tx power lower than this threshold
 */
#define CFG_BLE_MAX_TX_POWER              (0x19) /* 0x19 <=> -0.3 dBm */

/**
 * Definition of public BD Address,
 * when CFG_BD_ADDRESS = 0x000000000000 the BD address is generated based on Unique Device Number.
 */
#define CFG_BD_ADDRESS                    (0x000000000000)

/**
 * Define IO Authentication
 * @note This configuration is shared across all the connections that require authentication
 */
#define CFG_BONDING_MODE                  (0)
#define CFG_ENABLE_FIXED_PIN_PAIRING      (0)
#define CFG_FIXED_PIN                     (111111)
#define CFG_ENCRYPTION_KEY_SIZE_MAX       (16)
#define CFG_ENCRYPTION_KEY_SIZE_MIN       (8)

/**
 * Define Input Output capabilities
 * @note This configuration is shared across all the connections that require authentication.
 *       When BLE is used in Sidewalk-only mode, IO_CAP_NO_INPUT_NO_OUTPUT is enforced
 *       regardless of selection in CFG_IO_CAPABILITY
 */
#define CFG_IO_CAPABILITY                 (IO_CAP_DISPLAY_ONLY)

/**
 * Define Man In The Middle modes
 * @note This configuration is shared across all the connections that require authentication.
 *       When BLE is used in Sidewalk-only mode, MITM_PROTECTION_NOT_REQUIRED is enforced
 *       regardless of selection in CFG_MITM_PROTECTION
 */
#define CFG_MITM_PROTECTION               (MITM_PROTECTION_REQUIRED)

/**
 * Define Secure Connections Support
 * @note This configuration is shared across all the connections.
 *       When BLE is used in Sidewalk-only mode, SC_PAIRING_UNSUPPORTED is enforced
 *       regardless of selection in CFG_SC_SUPPORT
 * @attention When the BLE driver is configured to run Sidewalk BLE and user BLE concurrently
 *            and CFG_SC_SUPPORT is set to SC_PAIRING_ONLY, the actually applied config is
 *            SC_PAIRING_OPTIONAL because Sidewalk connections can't be established if pairing
 *            is mandatory. The user mode can still benefit from pairing though as the driver
 *            will ensure that secure BLE profiles cannot be accessed without paring. The only
 *            difference is that pairing in this mode is enforced on the driver level rather
 *            than on the BLE stack level. If this is unacceptable for your use case, switch to
 *            the interleaved BLE mode, where Sidewalk and user-defined BLE profiles do not run
 *            concurrently and the secure connection configuration is applied and respected by
 *            the BLE stack directly.
 */
#define CFG_SC_SUPPORT                    (SC_PAIRING_OPTIONAL)

/**
 * Define Keypress Notification Support
 * @note This configuration is shared across all the connections that require authentication.
 *       When BLE is used in Sidewalk-only mode, KEYPRESS_NOT_SUPPORTED is enforced
 *       regardless of selection in CFG_KEYPRESS_NOTIFICATION_SUPPORT
 */
#define CFG_KEYPRESS_NOTIFICATION_SUPPORT (KEYPRESS_NOT_SUPPORTED)

/**
*   Identity root key used to derive IRK and DHK(Legacy)
*/
#define CFG_BLE_IRK     {0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0, 0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0}

/**
* Encryption root key used to derive LTK(Legacy) and CSRK
*/
#define CFG_BLE_ERK     {0xFE, 0xDC, 0xBA, 0x09, 0x87, 0x65, 0x43, 0x21, 0xFE, 0xDC, 0xBA, 0x09, 0x87, 0x65, 0x43, 0x21}

/* USER CODE BEGIN Generic_Parameters */

/* USER CODE END Generic_Parameters */

/**< specific parameters */
/*****************************************************/

/* USER CODE BEGIN Specific_Parameters */

/* USER CODE END Specific_Parameters */

/******************************************************************************
 * BLE Stack
 ******************************************************************************/

/**
 * Maximum number of simultaneous connections and advertising that the device will support.
 * This setting should not exceed the number of BLE connection supported by BLE host stack.
 */
#define CFG_BLE_NUM_LINK            (3)

/**
 * Maximum number of Services that can be stored in the GATT database.
 * Note that the GAP and GATT services are automatically added so this parameter should be 2 plus the number of user services
 */
#define CFG_BLE_NUM_GATT_SERVICES   (3)

/**
 * Maximum number of Attributes
 * (i.e. the number of characteristic + the number of characteristic values + the number of descriptors, excluding the services)
 * that can be stored in the GATT database.
 * Note that certain characteristics and relative descriptors are added automatically during device initialization
 * so this parameters should be 9 plus the number of user Attributes
 */
#define CFG_BLE_NUM_GATT_ATTRIBUTES (30)

/**
 * Maximum supported ATT_MTU size
 * This setting should be aligned with ATT_MTU value configured in the ble host
 */
#define CFG_BLE_ATT_MTU_MAX         (512u)

/**
 * Size of the storage area for Attribute values
 *  This value depends on the number of attributes used by application. In particular the sum of the following quantities (in octets) should be made for each attribute:
 *  - attribute value length
 *  - 5, if UUID is 16 bit; 19, if UUID is 128 bit
 *  - 2, if server configuration descriptor is used
 *  - 2*DTM_NUM_LINK, if client configuration descriptor is used
 *  - 2, if extended properties is used
 *  The total amount of memory needed is the sum of the above quantities for each attribute.
 */
#define CFG_BLE_ATT_VALUE_ARRAY_SIZE    (1344)

/**
 * Maximum numbers of bearers that can be created for Enhanced ATT per ACL links
 */
#define CFG_BLE_EATT_BEARER_PER_LINK    (0)

/**
 * depth of the PREPARE WRITE queue when PREPARE WRITE REQUEST
 */
#define CFG_BLE_ATTR_PREPARE_WRITE_VALUE_SIZE       (30)

#define CFG_BLE_MBLOCK_COUNT_MARGIN                 (0x15)

#define PREP_WRITE_LIST_SIZE                        (BLE_DEFAULT_PREP_WRITE_LIST_SIZE)

/**
 * Appearance of device set into BLE GAP
 */
#define CFG_GAP_APPEARANCE            (GAP_APPEARANCE_UNKNOWN)

/**
 * Connection Oriented Channel parameters
 */
#define CFG_BLE_COC_NBR_MAX           (64)
#define CFG_BLE_COC_MPS_MAX           (248)
#define CFG_BLE_COC_INITIATOR_NBR_MAX (32)

/* USER CODE BEGIN BLE_Stack */

/* USER CODE END BLE_Stack */

/******************************************************************************
 * Low Power
 *
 *  When CFG_LPM_LEVEL is set to:
 *   - 0 : Low Power Mode is not activated, RUN mode will be used.
 *   - 1 : Low power active, mode selected with CFG_LPM_STDBY_SUPPORTED
 *   - 2 : In addition log and debug are disabled to reach lowest power figures.
 *
 * When CFG_LPM_STDBY_SUPPORTED is set to:
 *   - 2 : Stop mode 2 is used as low power mode (if supported by target)
 *   - 1 : Standby is used as low power mode.
 *   - 0 : Stop mode 1 is used as low power mode.
 *
 ******************************************************************************/
#define CFG_LPM_LEVEL            (1)
#define CFG_LPM_STDBY_SUPPORTED  (0)

/* Defines time to wake up from standby before radio event to meet timings */
#define CFG_LPM_STDBY_WAKEUP_TIME (1500)

/* Wait time (in microseconds) right after the HSE startup to allow the oscillator frequency to stabilize */
#define CFG_LPM_HSE_STABILIZATION_DELAY_US (200)

/* USER CODE BEGIN Low_Power 0 */

/* USER CODE END Low_Power 0 */

/**
 * Supported requester to the MCU Low Power Manager - can be increased up  to 32
 * It list a bit mapping of all user of the Low Power Manager
 */
typedef enum
{
  CFG_LPM_APP            = 0,
  CFG_LPM_LOG            = 1,
  CFG_LPM_LL_DEEPSLEEP   = 2,
  CFG_LPM_LL_HW_RCO_CLBR = 3,
  CFG_LPM_SIDEWALK       = 4,
  /* USER CODE BEGIN CFG_LPM_Id_t */

  /* USER CODE END CFG_LPM_Id_t */
} CFG_LPM_Id_t;

/* USER CODE BEGIN Low_Power 1 */

/* USER CODE END Low_Power 1 */

/******************************************************************************
 * RTC
 ******************************************************************************/

/* USER CODE BEGIN RTC */

/* USER CODE END RTC */

/*****************************************************************************
 * Logs
 *
 * Applications must call LOG_INFO_APP for logs.
 * By default, CFG_LOG_INSERT_TIME_STAMP_INSIDE_THE_TRACE is set to 0.
 * As a result, there is no time stamp insertion inside the logs.
 *
 * For advanced log use cases, see the log_module.h file.
 * This file is customizable, you can create new verbose levels and log regions.
 *****************************************************************************/
/**
 * Enable or disable LOG over UART in the application.
 * Low power level(CFG_LPM_LEVEL) above 1 will disable LOG.
 */
#define CFG_LOG_SUPPORTED           (1U)

/* Usart used by LOG */
extern UART_HandleTypeDef            huart1;
#define LOG_UART_HANDLER             huart1
#define LOG_UART_HANDLER_IRQn        USART1_IRQn
#define LOG_UART_HANDLER_TX_DMA_IRQn GPDMA1_Channel4_IRQn

/* Configure Log display settings */
#define CFG_LOG_INSERT_COLOR_INSIDE_THE_TRACE       (0U)
#define CFG_LOG_INSERT_TIME_STAMP_INSIDE_THE_TRACE  (0U)
#define CFG_LOG_INSERT_EOL_INSIDE_THE_TRACE         (0U)

#define CFG_LOG_TRACE_FIFO_SIZE     (1280U)
#define CFG_LOG_TRACE_BUF_SIZE      ((SID_PAL_LOG_MSG_LENGTH_MAX) + 30U)

/* macro ensuring retrocompatibility with old applications */
#define APP_DBG                     LOG_INFO_APP
#define APP_DBG_MSG                 LOG_INFO_APP

/* USER CODE BEGIN Logs */

/* USER CODE END Logs */

/******************************************************************************
 * Configure Log level for Application
 *
 * APPLI_CONFIG_LOG_LEVEL can be any value of the Log_Verbose_Level_t enum.
 *
 * APPLI_CONFIG_LOG_REGION can either be :
 * - LOG_REGION_ALL_REGIONS to enable all regions
 * or
 * - One or several specific regions (any value except LOG_REGION_ALL_REGIONS)
 *   from the Log_Region_t enum and matching the mask value.
 *
 *   For example, to enable both LOG_REGION_BLE and LOG_REGION_APP,
 *   the value assigned to the define is :
 *   (1U << LOG_REGION_BLE | 1U << LOG_REGION_APP)
 ******************************************************************************/
#define APPLI_CONFIG_LOG_LEVEL      LOG_VERBOSE_INFO
#define APPLI_CONFIG_LOG_REGION     (LOG_REGION_ALL_REGIONS)
/* USER CODE BEGIN Log_level */

/* USER CODE END Log_level */

/******************************************************************************
 * NVM configuration
 ******************************************************************************/

#define CFG_SNVMA_START_SECTOR_ID     ((APP_CONFIG_SNVMA_FLASH_START - FLASH_BASE) / FLASH_PAGE_SIZE)

#define CFG_SNVMA_END_SECTOR_ID       (((APP_CONFIG_SNVMA_FLASH_END - 1u) - FLASH_BASE) / FLASH_PAGE_SIZE)

#define CFG_SNVMA_START_ADDRESS       (APP_CONFIG_SNVMA_FLASH_START)

/* Number of 64-bit words in NVM flash area */
#define CFG_BLE_NVM_SIZE_MAX          ((2048/8)-4)

/* USER CODE BEGIN NVM_Configuration */

/* USER CODE END NVM_Configuration */

/******************************************************************************
 * Debugger
 *
 *  When CFG_DEBUGGER_LEVEL is set to:
 *   - 0 : No Debugger available, SWD/JTAG pins are disabled.
 *   - 1 : Debugger available in RUN mode only.
 *   - 2 : Debugger available in low power mode.
 *
 ******************************************************************************/
#define CFG_DEBUGGER_LEVEL                  (1)

/******************************************************************************
 * RealTime GPIO debug module configuration
 ******************************************************************************/

#define CFG_RT_DEBUG_GPIO_MODULE            (0)
#define CFG_RT_DEBUG_DTB                    (0)

/******************************************************************************
 * System Clock Manager module configuration
 ******************************************************************************/

#define CFG_SCM_SUPPORTED                   (1)

/******************************************************************************
 * HW RADIO configuration
 ******************************************************************************/
/* Link Layer uses temperature based calibration (0 --> NO ; 1 --> YES) */
#define USE_TEMPERATURE_BASED_RADIO_CALIBRATION  (1)

#define RADIO_INTR_NUM                      RADIO_IRQn     /* 2.4GHz RADIO global interrupt */
#define RADIO_INTR_PRIO_HIGH                (0)            /* 2.4GHz RADIO interrupt priority when radio is Active */
#define RADIO_INTR_PRIO_LOW                 (5)            /* 2.4GHz RADIO interrupt priority when radio is Not Active - Sleep Timer Only */

#define RADIO_SW_LOW_INTR_NUM               HASH_IRQn      /* Selected interrupt vector for 2.4GHz RADIO low ISR */
#define RADIO_SW_LOW_INTR_PRIO              (14)           /* 2.4GHz RADIO low ISR priority */

#define RCC_INTR_PRIO                       (1)           /* HSERDY and PLL1RDY */

/* RF TX power table ID selection:
 *   0 -> RF TX output level from -20 dBm to +10 dBm
 *   1 -> RF TX output level from -20 dBm to +3 dBm
 */
#define CFG_RF_TX_POWER_TABLE_ID            (1)

/* Radio sleep clock LSE accuracy configuration */
#define CFG_RADIO_LSE_SLEEP_TIMER_CUSTOM_SCA_RANGE (0x04)

/* USER CODE BEGIN Radio_Configuration */

/* USER CODE END Radio_Configuration */

/******************************************************************************
 * HW_RNG configuration
 ******************************************************************************/

/* Number of 32-bit random numbers stored in internal pool */
#define CFG_HW_RNG_POOL_SIZE                (32)

/* Threshold of random numbers available before triggering pool refill */
#define CFG_HW_RNG_POOL_THRESHOLD           (16)

/* USER CODE BEGIN HW_RNG_Configuration */

/* USER CODE END HW_RNG_Configuration */

/******************************************************************************
 * MEMORY MANAGER
 ******************************************************************************/

#define CFG_MM_POOL_SIZE                                  (3000U)  /* bytes */
#define CFG_AMM_VIRTUAL_MEMORY_NUMBER                     (2U)
#define CFG_AMM_VIRTUAL_STACK_BLE                         (1U)
#define CFG_AMM_VIRTUAL_STACK_BLE_BUFFER_SIZE     (400U)  /* words (32 bits) */
#define CFG_AMM_VIRTUAL_APP_BLE                           (2U)
#define CFG_AMM_VIRTUAL_APP_BLE_BUFFER_SIZE     (200U)  /* words (32 bits) */
#define CFG_AMM_POOL_SIZE                                 ( DIVC(CFG_MM_POOL_SIZE, sizeof (uint32_t)) \
                                                          + (AMM_VIRTUAL_INFO_ELEMENT_SIZE * CFG_AMM_VIRTUAL_MEMORY_NUMBER) )

/* USER CODE BEGIN MEMORY_MANAGER_Configuration */

/* USER CODE END MEMORY_MANAGER_Configuration */

/* USER CODE BEGIN Defines */
/**
 * User interaction
 * When CFG_LED_SUPPORTED is set, LEDS are activated if requested
 * When CFG_BUTTON_SUPPORTED is set, the push button are activated if requested
 */
#if defined(NUCLEO_WBA52_BOARD) || defined(NUCLEO_WBA55_BOARD) || defined (NUCLEO_WBA65_BOARD)
# define CFG_LED_SUPPORTED                       (1)
# define CFG_BUTTON_SUPPORTED                    (1)
#elif defined(GENERIC_WBA5x_BOARD) || defined(GENERIC_WBA6x_BOARD)
# define CFG_LED_SUPPORTED                       (0)
# define CFG_BUTTON_SUPPORTED                    (0)
#endif

/**
 * Overwrite some configuration imposed by Low Power level selected.
 */
#if (CFG_LPM_LEVEL > 1)
  #if CFG_LED_SUPPORTED
    #undef  CFG_LED_SUPPORTED
    #define CFG_LED_SUPPORTED      (0)
  #endif /* CFG_LED_SUPPORTED */
#endif /* CFG_LPM_LEVEL */

/* USER CODE END Defines */

/**
 * Overwrite some configuration imposed by Low Power level selected.
 */
#if (CFG_LPM_LEVEL > 1)
  #if CFG_LOG_SUPPORTED
    #undef  CFG_LOG_SUPPORTED
    #define CFG_LOG_SUPPORTED       (0)
  #endif /* CFG_LOG_SUPPORTED */
  #if CFG_DEBUGGER_LEVEL
    #undef  CFG_DEBUGGER_LEVEL
    #define CFG_DEBUGGER_LEVEL      (0)
  #endif /* CFG_DEBUGGER_LEVEL */
#endif /* CFG_LPM_LEVEL */

/* USER CODE BEGIN Defines_2 */

/* USER CODE END Defines_2 */

#endif /*APP_CONF_H */
