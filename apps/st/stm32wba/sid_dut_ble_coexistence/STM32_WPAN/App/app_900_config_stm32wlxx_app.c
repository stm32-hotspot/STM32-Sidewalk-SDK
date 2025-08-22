/**
  ******************************************************************************
  * @file    app_900_config_stm32wlxx_app.c
  * @brief   Sub-GHz radio configuration for Sidewalk application
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

/* Includes ------------------------------------------------------------------*/

#include <sid_pal_log_ifc.h>
#include <sid_pal_radio_ifc.h>
#include <sid_pal_serial_bus_ifc.h>
#include <sid_pal_serial_bus_spi_config.h>
#include <sid_stm32_common_utils.h>
#include <stm32wlxx_app_radio_config.h>
#include <comm_def.h>

#include "app_900_config.h"
#include "main.h"
#include "sid_pal_gpio_ext_ifc.h"

/* Private defines -----------------------------------------------------------*/

#define RADIO_MAX_TX_POWER_NA                                      (20)
#define RADIO_MAX_TX_POWER_EU                                      (14)

#if defined (REGION_ALL)
#  define RADIO_REGION                                             (RADIO_REGION_NONE)
#elif defined (REGION_US915)
#  define RADIO_REGION                                             (RADIO_REGION_NA)
#elif defined (REGION_EU868)
#  define RADIO_REGION                                             (RADIO_REGION_EU)
#else
#  error "SubGHz configuration error. RADIO_REGION is not defined"
#endif

#define RADIO_STM32WLxx_SPI_BUFFER_SIZE                            (STM32WLxx_RADIO_COMM_MTU_SIZE)

#define RADIO_RX_LNA_GAIN                                          (0u)
#define RADIO_MAX_CAD_SYMBOL                                       (SID_PAL_RADIO_LORA_CAD_04_SYMBOL)

#ifndef SIDEWALK_RADIO_USE_ARD_CONN_STACKING
#  error "Define SIDEWALK_RADIO_USE_ARD_CONN_STACKING value explicitly to define the connection scheme. Set a non-zero value to use Arduino UNO v3 connector to stack NUCLEO-WBA and NUCLEO-WL boards. WARNING: strictly follow the connection guide to avoid electrical damage to the boards"
#endif /* SIDEWALK_RADIO_USE_ARD_CONN_STACKING */

#if SIDEWALK_RADIO_USE_ARD_CONN_STACKING
/* This option applies to board stacking connection of WBA and WL */
#  if defined(STM32WBA5x)
#    define SIDEWALK_RADIO_SPI_IRQ_Pin GPIO_PIN_15
#    define SIDEWALK_RADIO_SPI_IRQ_GPIO_Port GPIOB
#  elif defined( STM32WBA6x)
#    define SIDEWALK_RADIO_SPI_IRQ_Pin GPIO_PIN_10
#    define SIDEWALK_RADIO_SPI_IRQ_GPIO_Port GPIOA
#  else
#    error "This configuration supports only STM32WBA5x and STM32WBA6x MCU families"
#  endif /* STM32WBA5x || STM32WBA6x */
#else
/* This option applies to the original jumper-wire connection scheme */
#  define SIDEWALK_RADIO_SPI_IRQ_Pin GPIO_PIN_1
#  define SIDEWALK_RADIO_SPI_IRQ_GPIO_Port GPIOA
#endif /* SIDEWALK_RADIO_USE_ARD_CONN_STACKING */

/* External variables --------------------------------------------------------*/

#ifdef STM32WBA5x
extern SPI_HandleTypeDef hspi1;
extern void MX_SPI1_Init(void);
#endif /* STM32WBA5x */

#ifdef STM32WBA6x
extern SPI_HandleTypeDef hspi2;
extern void MX_SPI2_Init(void);
#endif /* STM32WBA6x */

extern LPTIM_HandleTypeDef hlptim1;
extern void MX_LPTIM1_Init(void);

/* Private function prototypes -----------------------------------------------*/

static int32_t radio_stm32wlxx_pa_cfg(const int32_t desired_tx_power, const stm32wlxx_radio_regional_param_t * const regional_params, stm32wlxx_radio_pa_cfg_t * const out_pa_cfg);

/* Private variables ---------------------------------------------------------*/

SID_STM32_ALIGN_4BYTES(static uint8_t stm32wlxx_processing_buffer[RADIO_STM32WLxx_SPI_BUFFER_SIZE]);

/* Private constants ---------------------------------------------------------*/

/**
 * @brief Links to the SPI peripherals that are required for the SPI driver of
 * the host MCU (STM32WBAxx)
 */
static const sid_pal_serial_bus_stm32wbaxx_factory_config_t radio_spi_factory_config = {
#if defined(STM32WBA5x)
    .hspi              = &hspi1,
    .mx_spi_init       = MX_SPI1_Init,
#elif defined(STM32WBA6x)
    .hspi              = &hspi2,
    .mx_spi_init       = MX_SPI2_Init,
#else
#  error "This configuration supports only STM32WBA5x and STM32WBA6x MCU families"
#endif /* STM32WBAxx */
    .hlptim            = &hlptim1,
    .mx_lptim_init     = MX_LPTIM1_Init,
};

/**
 * @brief SPI transaction configuration for the driver
 */
static const sid_pal_serial_bus_stm32wbaxx_transaction_config_t radio_spi_transaction_config = {
    .crc_polynomial    = 0x11021u,
    .crc_length        = SPI_CRC_LENGTH_16BIT,
    .crc_init_pattern  = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN,

    .nss_to_sck_cycles = 12u,
    .sck_to_nss_cycles = 700u, /* Approx. 60us is required here to ensure STM32WLxx's SPI DMA Tx Done IRQ is processed before NSS pin is raised */
    .nss_to_nss_cycles = 700u, /* Approx. 60us is required here for STM32WLxx Rev Z to work properly due to errata 2.2.1 workaround */
};

/**
 * @brief SPI driver configuration for SX126X communication
 */
static const sid_pal_serial_bus_client_t spi_client_config = {
#if defined(STM32WBA5x)
    .client_selector           = GPIO_PORT_PIN_TO_NUM(GPIOA, GPIO_PIN_12),
#elif defined(STM32WBA6x)
    .client_selector           = GPIO_PORT_PIN_TO_NUM(GPIOB, GPIO_PIN_9),
#else
#  error "This configuration supports only STM32WBA5x and STM32WBA6x MCU families"
#endif /* STM32WBAxx */
    .speed_hz                  = 12000000u,
    .bit_order                 = SID_PAL_SERIAL_BUS_BIT_ORDER_MSB_FIRST,
    .mode                      = SID_PAL_SERIAL_BUS_STM32WBAxx_SPI_MODE_0,
    .client_selector_extension = &radio_spi_transaction_config,
};

/**
 * @brief Aggregator object to pass to the SPI driver
 */
static const struct sid_pal_serial_bus_factory radio_spi_factory = {
    .create = sid_pal_serial_bus_stm32wbaxx_create,
    .config = &radio_spi_factory_config,
};

/**
 * @brief Sidewalk sub-GHz link network layer parameters
 */
static const struct sid_sub_ghz_links_config sub_ghz_link_config = {
    .enable_link_metrics = true,
    .metrics_msg_retries = 3u,
    .sar_dcr = 100u,
    .registration_config = {
        .enable = true,
        .periodicity_s = UINT32_MAX,
    },
    .link2_max_tx_power_in_dbm = RADIO_MAX_TX_POWER_NA,
    .link3_max_tx_power_in_dbm = RADIO_MAX_TX_POWER_NA,
};

/**
 * @brief Regional parameters and limitations for the physical layer
 */
static const stm32wlxx_radio_regional_param_t radio_stm32wlxx_regional_param[] =
{
#if defined (REGION_ALL) || defined (REGION_US915)
    {
        .param_region     = RADIO_REGION_NA,
        .max_tx_power     = { RADIO_MAX_TX_POWER_NA, RADIO_MAX_TX_POWER_NA, RADIO_MAX_TX_POWER_NA,
                              RADIO_MAX_TX_POWER_NA, RADIO_MAX_TX_POWER_NA, RADIO_MAX_TX_POWER_NA },
        .cca_level_adjust = { 0, 0, 0, 0, 0, 0 },
        .ant_dbi          = CONFIG_RADIO_GAIN_DB_TO_INT(2.0),
    },
#endif
#if defined (REGION_ALL) || defined (REGION_EU868)
    {
        .param_region     = RADIO_REGION_EU,
        .max_tx_power     = { RADIO_MAX_TX_POWER_EU, RADIO_MAX_TX_POWER_EU, RADIO_MAX_TX_POWER_EU,
                              RADIO_MAX_TX_POWER_EU, RADIO_MAX_TX_POWER_EU, RADIO_MAX_TX_POWER_EU },
        .cca_level_adjust = { 0, 0, 0, 0, 0, 0 },
        .ant_dbi          = CONFIG_RADIO_GAIN_DB_TO_INT(2.0),
    },
#endif
};

/**
 * @brief Board-specific SX126x driver configuration
 */
static const stm32wlxx_app_radio_device_config_t radio_stm32wlxx_cfg = {
    .rx_boost                   = false,
    .lna_gain                   = RADIO_RX_LNA_GAIN,
    .bus_factory                = &radio_spi_factory,

    .gpio = {
        .radio_irq              = GPIO_PORT_PIN_TO_NUM(SIDEWALK_RADIO_SPI_IRQ_GPIO_Port, SIDEWALK_RADIO_SPI_IRQ_Pin),
        .radio_irq_prio_high    = RADIO_INTR_PRIO_HIGH, /* Keep the same priority as high priority for BLE radio - both need elevated priority to respect radio timings */
        .radio_irq_prio_low     = RADIO_INTR_PRIO_LOW,  /* Keep aligned with BLE config. The priority should be low enough to allow RTOS API calls, but we still want to process radio events ASAP */
#if STM32WLxx_RADIO_CFG_USE_STATUS_LED
#  if defined NUCLEO_WBA52_BOARD || defined NUCLEO_WBA55_BOARD
        .tx_led                 = GPIO_PORT_PIN_TO_NUM(GPIOB, GPIO_PIN_8),
        .tx_led_on_gpio_state   = 0u,
        .rx_led                 = GPIO_PORT_PIN_TO_NUM(GPIOB, GPIO_PIN_8),
        .rx_led_on_gpio_state   = 0u,
#  elif defined NUCLEO_WBA65_BOARD
        .tx_led                 = GPIO_PORT_PIN_TO_NUM(GPIOD, GPIO_PIN_8),
        .tx_led_on_gpio_state   = 0u,
        .rx_led                 = GPIO_PORT_PIN_TO_NUM(GPIOD, GPIO_PIN_8),
        .rx_led_on_gpio_state   = 0u,
#  else
#    warning "This sample config supports only NUCLEO-WBA52, NUCLEO-WBA55, and NUCLEO-WBA65 boards. Please define the pin assignment here if you are using another (e.g. custom) board"
        .tx_led                 = HALO_GPIO_NOT_CONNECTED,
        .tx_led_on_gpio_state   = 0u,
        .rx_led                 = HALO_GPIO_NOT_CONNECTED,
        .rx_led_on_gpio_state   = 0u,
#  endif /* NUCLEO_WBA52_BOARD || NUCLEO_WBA55_BOARD || NUCLEO_WBA65_BOARD */
#endif /* STM32WLxx_RADIO_CFG_USE_STATUS_LED */
    },

    .pa_cfg_callback            = radio_stm32wlxx_pa_cfg,
    .enable_lpa                 = true,
    .enable_hpa                 = true,

    .spi_client_cfg             = &spi_client_config,

    .regional_config = {
        .radio_region           = RADIO_REGION,
        .reg_param_table_size   = SID_STM32_UTIL_ARRAY_SIZE(radio_stm32wlxx_regional_param),
        .reg_param_table        = radio_stm32wlxx_regional_param,
    },

    .state_timings = {
#if STM32WLxx_RADIO_APP_LPM_SUPPORT
        .sleep_to_full_power_us = 1675u, /* Time to reach the STDBY_XOSC state */
#else
        .sleep_to_full_power_us = 820u, /* Time to reach the STDBY_XOSC state */
#endif /* STM32WLxx_RADIO_APP_LPM_SUPPORT */
        .full_power_to_sleep_us = 0u,   /* Currently not used by Sidewalk SDK */
        .rx_to_tx_us            = 0u,   /* Currently not used by Sidewalk SDK */
        .tx_to_rx_us            = 0u,   /* Currently not used by Sidewalk SDK */
        .tcxo_delay_us          = STM32WLxx_SUBGHZ_TICKS_TO_US(STM32WLxx_RADIO_COMM_TCXO_TIMEOUT_DURATION_TUS), /* While the TCXO is controlled by STM32WLxx, this value shall be aligned with STM32WLxx operation to accommodate the delays cause by TCXO startup */
    },

    .processing_timings         = {
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
        .lora = {
            .tx_process_delay_us = STM32WLxx_RADIO_CFG_USE_DEFAULT_TIMINGS,
            .rx_process_delay_us = STM32WLxx_RADIO_CFG_USE_DEFAULT_TIMINGS,
        },
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 */
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_2
        .fsk = {
            .tx_process_delay_us = STM32WLxx_RADIO_CFG_USE_DEFAULT_TIMINGS,
            .rx_process_delay_us = STM32WLxx_RADIO_CFG_USE_DEFAULT_TIMINGS,
        },
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 */
    },

    .internal_buffer = {
        .p    = stm32wlxx_processing_buffer,
        .size = sizeof(stm32wlxx_processing_buffer),
    },
};

/* Private function definitions ----------------------------------------------*/

static int32_t radio_stm32wlxx_pa_cfg(const int32_t desired_tx_power, const stm32wlxx_radio_regional_param_t * const regional_params, stm32wlxx_radio_pa_cfg_t * const out_pa_cfg)
{
    static int32_t last_desired_pwr = INT32_MAX;
    int32_t pwr, max_pwr, min_pwr;

    (void)regional_params;

    if ((false == radio_stm32wlxx_cfg.enable_lpa) && (false == radio_stm32wlxx_cfg.enable_hpa))
    {
        SID_PAL_LOG_ERROR("Invalid radio config. At least one of power amplifiers (LPA or HPA) shall be enabled");
        return RADIO_ERROR_INVALID_PARAMS;
    }

    min_pwr = radio_stm32wlxx_cfg.enable_lpa != false ? RADIO_STM32WLxx_LPA_LOWER_LIMIT_DBM : RADIO_STM32WLxx_HPA_LOWER_LIMIT_DBM;
    max_pwr = radio_stm32wlxx_cfg.enable_hpa != false ? RADIO_STM32WLxx_HPA_UPPER_LIMIT_DBM : RADIO_STM32WLxx_LPA_UPPER_LIMIT_DBM;

    if (desired_tx_power > max_pwr)
    {
        pwr = max_pwr;
    }

    if (desired_tx_power < min_pwr)
    {
        pwr = min_pwr;
    }
    else
    {
        /* Power selection is within the boundaries, no override required */
        pwr = desired_tx_power;
    }

    if ((desired_tx_power != last_desired_pwr) && (pwr != desired_tx_power))
    {
        SID_PAL_LOG_WARNING("SubGHz Tx power set to %s%ddBm instead of %s%ddBm due to the HW limits",
                            pwr >= 0 ? "+" : "", pwr, desired_tx_power >= 0 ? "+" : "", desired_tx_power); /* tiny_vsnprintf() may not support %+d format */
    }

    /* Set the desired Tx power */
    out_pa_cfg->target_tx_power = pwr;

    /* Don't touch the other PA parameters here - the driver will automatically apply matching settings */

    /**
     * NOTE: you may override out_pa_cfg values here if needed and then the driver will keep the overrides supplied via this callback. While
     *       normally you should be ok without any overrides, it may be useful to address certain scenarios or hardware configurations. For
     *       example, if you PCB has different filter tuning for high-power and low-power amplifiers (e.g. HPA tuned to 915MHz band and LPA
     *       tuned to 868MHz) you may explicitly set the power amplifier to use (HPA or LPA) in out_pa_cfg->device_sel depending on the
     *       selected regional settings (can be accessed via regional_params parameter)
     */

    /* Remember the latest request to avoid repeated log messages */
    last_desired_pwr = desired_tx_power;

    return RADIO_ERROR_NONE;
}

/* Global function definitions -----------------------------------------------*/

const stm32wlxx_app_radio_device_config_t* get_radio_cfg(void)
{
    return &radio_stm32wlxx_cfg;
}

/*----------------------------------------------------------------------------*/

const struct sid_sub_ghz_links_config* app_get_sub_ghz_config(void)
{
    return &sub_ghz_link_config;
}
