/*
 * Copyright 2021-2023 Amazon.com, Inc. or its affiliates. All rights reserved.
 *
 * AMAZON PROPRIETARY/CONFIDENTIAL
 *
 * You may not use this file except in compliance with the terms and
 * conditions set forth in the accompanying LICENSE.TXT file. This file is a
 * Modifiable File, as defined in the accompanying LICENSE.TXT file.
 *
 * THESE MATERIALS ARE PROVIDED ON AN "AS IS" BASIS. AMAZON SPECIFICALLY
 * DISCLAIMS, WITH RESPECT TO THESE MATERIALS, ALL WARRANTIES, EXPRESS,
 * IMPLIED, OR STATUTORY, INCLUDING THE IMPLIED WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE, AND NON-INFRINGEMENT.
 */
/**
  ******************************************************************************
  * @file    app_900_config_lr11xx.c
  * @brief   Sub-GHz radio configuration for Sidewalk application
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

#include <sid_pal_log_ifc.h>
#include <sid_pal_radio_ifc.h>
#include <sid_pal_serial_bus_ifc.h>
#include <sid_pal_serial_bus_spi_config.h>
#include <sid_stm32_common_utils.h>
#include <lr11xx_radio_config.h>

#include "app_900_config.h"
#include "main.h"
#include "sid_pal_gpio_ext_ifc.h"

/* Private defines -----------------------------------------------------------*/

/* This product has no external PA and LR11xx can support max of 22dBm*/
#define RADIO_LR11XX_MAX_TX_POWER                                  (22)
#define RADIO_LR11XX_MIN_TX_POWER                                  (-9)

#define RADIO_MAX_TX_POWER_NA                                      (20)
#define RADIO_MAX_TX_POWER_EU                                      (14)

#if defined (REGION_ALL)
#define RADIO_REGION                                               (RADIO_REGION_NONE)
#elif defined (REGION_US915)
#define RADIO_REGION                                               (RADIO_REGION_NA)
#elif defined (REGION_EU868)
#define RADIO_REGION                                               (RADIO_REGION_EU)
#endif

#define RADIO_LR11XX_SPI_BUFFER_SIZE                               (256u)

#define RADIO_LR11XX_PA_DUTY_CYCLE                                 (0x04)
#define RADIO_LR11XX_HP_MAX                                        (0x07)
#define RADIO_LR11XX_DEVICE_SEL                                    (0x00)
#define RADIO_LR11XX_PA_LUT                                        (0x01)

#define RADIO_MAX_CAD_SYMBOL                                       (SID_PAL_RADIO_LORA_CAD_04_SYMBOL)
#define RADIO_ANT_GAIN(X)                                          ((X) * 100)

/* DIO mapping for RF switch operation - this is specific to LR1110MB1LxKS devkits, adjust to your hardware design as needed */
#define LR1110MB1LxKS_SUBGHZ_RF_SW_V1                              (LR11XX_SYSTEM_RFSW0_HIGH)
#define LR1110MB1LxKS_SUBGHZ_RF_SW_V2                              (LR11XX_SYSTEM_RFSW1_HIGH)
#define LR1110MB1LxKS_GNSS_LNA_EN_SW                               (LR11XX_SYSTEM_RFSW2_HIGH)
#define LR1110MB1LxKS_RF_SW_ALL_OFF                                (0u)

#define RADIO_LR11XX_TCXO_START_TIME_US                            (2000u)  /* Typical TCXO start time is about 2ms */
#define RADIO_LR11XX_TCXO_GNSS_START_TIME_US                       (15000u) /* Allow longer TCXO startup time for better frequency stability */

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

static int32_t radio_lr11xx_pa_cfg(const int32_t desired_tx_power, const radio_lr11xx_regional_param_t * const regional_params, radio_lr11xx_pa_cfg_t * const out_pa_cfg);

/* Private variables ---------------------------------------------------------*/

SID_STM32_ALIGN_4BYTES(static uint8_t radio_lr11xx_buffer[RADIO_LR11XX_SPI_BUFFER_SIZE]);

/* Private constants ---------------------------------------------------------*/

/**
 * @brief Links to the peripherals that are required for the SPI driver of the host MCU (STM32WBAxx)
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
    .crc_polynomial    = SID_PAL_SERIAL_BUS_STM32WBAxx_CRC_NOT_USED,
    .crc_length        = 0u,

    .nss_to_sck_cycles = 40u,
    .sck_to_nss_cycles = 0u,
    .nss_to_nss_cycles = 6u,
};

/**
 * @brief SPI driver configuration for LR11x0 communication
 */
static const sid_pal_serial_bus_client_t spi_client_config = {
#if defined(STM32WBA5x)
    .client_selector           = GPIO_PORT_PIN_TO_NUM(GPIOB, GPIO_PIN_9),
#elif defined(STM32WBA6x)
    .client_selector           = GPIO_PORT_PIN_TO_NUM(GPIOD, GPIO_PIN_14),
#else
#  error "This configuration supports only STM32WBA5x and STM32WBA6x MCU families"
#endif /* STM32WBAxx */
    .speed_hz                  = 12000000u, /* 16MHz is maximum for LR11xx */
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
const radio_lr11xx_regional_param_t radio_lr11xx_regional_param[] =
{
    #if defined (REGION_ALL) || defined (REGION_US915)
    {
        .param_region = RADIO_REGION_NA,
        .max_tx_power = { RADIO_MAX_TX_POWER_NA, RADIO_MAX_TX_POWER_NA, RADIO_MAX_TX_POWER_NA,
                            RADIO_MAX_TX_POWER_NA, RADIO_MAX_TX_POWER_NA, RADIO_MAX_TX_POWER_NA },
        .cca_level_adjust = { 0, 0, 0, 0, 0, 0 },
        .ant_dbi = RADIO_ANT_GAIN(2.15)
    },
    #endif
    #if defined (REGION_ALL) || defined (REGION_EU868)
    {
        .param_region = RADIO_REGION_EU,
        .max_tx_power = { RADIO_MAX_TX_POWER_EU, RADIO_MAX_TX_POWER_EU, RADIO_MAX_TX_POWER_EU,
                            RADIO_MAX_TX_POWER_EU, RADIO_MAX_TX_POWER_EU, RADIO_MAX_TX_POWER_EU },
        .cca_level_adjust = { 0, 0, 0, 0, 0, 0 },
        .ant_dbi = RADIO_ANT_GAIN(2.15)
    },
    #endif
};

/**
 * @brief Board-specific LR11xx driver configuration
 */
const lr11xx_radio_device_config_t radio_lr11xx_cfg = {
    .regulator_mode             = LR11XX_SYSTEM_REG_MODE_DCDC,
    .rx_boost                   = false,
    .bus_factory                = &radio_spi_factory,
    .gpio = {
#if defined(STM32WBA5x)
        .radio_reset            = GPIO_PORT_PIN_TO_NUM(GPIOA, GPIO_PIN_7),  /* LR11xx NRESET */
        .radio_irq              = GPIO_PORT_PIN_TO_NUM(GPIOB, GPIO_PIN_14), /* LR11xx IRQ */
        .radio_busy             = GPIO_PORT_PIN_TO_NUM(GPIOB, GPIO_PIN_6),  /* LR11xx BUSY - conflict with B2 on NUCLEO-WBA5x boards */
#elif defined(STM32WBA6x)
        .radio_reset            = GPIO_PORT_PIN_TO_NUM(GPIOA, GPIO_PIN_4),  /* LR11xx NRESET */
        .radio_irq              = GPIO_PORT_PIN_TO_NUM(GPIOB, GPIO_PIN_14), /* LR11xx IRQ */
        .radio_busy             = GPIO_PORT_PIN_TO_NUM(GPIOB, GPIO_PIN_13), /* LR11xx BUSY */
#else
#  error "This configuration supports only STM32WBA5x and STM32WBA6x MCU families"
#endif /* STM32WBAxx */

        .radio_irq_prio_high    = RADIO_INTR_PRIO_HIGH, /* Keep the same priority as high priority for BLE radio - both need elevated priority to respect radio timings */
        .radio_irq_prio_low     = RADIO_INTR_PRIO_LOW,  /* Keep aligned with BLE config. The priority should be low enough to allow RTOS API calls, but we still want to process radio events ASAP */

#if LR11XX_RADIO_CFG_USE_FEM_POW_ENA
#  if defined NUCLEO_WBA52_BOARD || defined NUCLEO_WBA55_BOARD || defined NUCLEO_WBA65_BOARD
        .fem_power              = HALO_GPIO_NOT_CONNECTED,
        .fem_power_en_gpio_state   = 1u,
#  else
#    warning "This sample config supports only NUCLEO-WBA52, NUCLEO-WBA55, and NUCLEO-WBA65 boards. Please define the pin assignment here if you are using another (e.g. custom) board"
        .fem_power                 = HALO_GPIO_NOT_CONNECTED,
        .fem_power_en_gpio_state   = 1u,
#  endif /* NUCLEO_WBA52_BOARD || NUCLEO_WBA55_BOARD || NUCLEO_WBA65_BOARD */
#endif /* LR11XX_RADIO_CFG_USE_FEM_POW_ENA */

#if LR11XX_RADIO_CFG_USE_TX_RX_SW
#  if defined NUCLEO_WBA52_BOARD || defined NUCLEO_WBA55_BOARD || defined NUCLEO_WBA65_BOARD
        .fem_tx_rx_mode                   = HALO_GPIO_NOT_CONNECTED,
        .fem_tx_mode_sel_gpio_state     = 1u,
#  else
#    warning "This sample config supports only NUCLEO-WBA52, NUCLEO-WBA55, and NUCLEO-WBA65 boards. Please define the pin assignment here if you are using another (e.g. custom) board"
        .fem_tx_rx_mode                   = HALO_GPIO_NOT_CONNECTED,
        .fem_tx_mode_sel_gpio_state     = 1u,
#  endif /* NUCLEO_WBA52_BOARD || NUCLEO_WBA55_BOARD || NUCLEO_WBA65_BOARD */
#endif /* LR11XX_RADIO_CFG_USE_TX_RX_SW */

#if LR11XX_RADIO_CFG_USE_HPA_LPA_SW
#  if defined NUCLEO_WBA52_BOARD || defined NUCLEO_WBA55_BOARD || defined NUCLEO_WBA65_BOARD
        .hpa_lpa_sw                = HALO_GPIO_NOT_CONNECTED,
        .hpa_lpa_sw_on_gpio_state  = 1u,
#  else
#    warning "This sample config supports only NUCLEO-WBA52, NUCLEO-WBA55, and NUCLEO-WBA65 boards. Please define the pin assignment here if you are using another (e.g. custom) board"
        .hpa_lpa_sw                = HALO_GPIO_NOT_CONNECTED,
        .hpa_lpa_sw_on_gpio_state  = 1u,
#  endif /* NUCLEO_WBA52_BOARD || NUCLEO_WBA55_BOARD || NUCLEO_WBA65_BOARD */
#endif /* LR11XX_RADIO_CFG_USE_HPA_LPA_SW */

#if LR11XX_RADIO_CFG_USE_EXTERNAL_PA && LR11XX_RADIO_CFG_USE_FEM_BYPASS
#  if defined NUCLEO_WBA52_BOARD || defined NUCLEO_WBA55_BOARD || defined NUCLEO_WBA65_BOARD
        .fem_bypass                   = HALO_GPIO_NOT_CONNECTED,
        .fem_bypass_en_gpio_state     = 1u,
#  else
#    warning "This sample config supports only NUCLEO-WBA52, NUCLEO-WBA55, and NUCLEO-WBA65 boards. Please define the pin assignment here if you are using another (e.g. custom) board"
        .fem_bypass                   = HALO_GPIO_NOT_CONNECTED,
        .fem_bypass_en_gpio_state     = 1u,
#  endif /* NUCLEO_WBA52_BOARD || NUCLEO_WBA55_BOARD || NUCLEO_WBA65_BOARD */
#endif /* LR11XX_RADIO_CFG_USE_EXTERNAL_PA */

#if LR11XX_RADIO_CFG_GEOLOCATION_SUPPORT
#  if defined NUCLEO_WBA52_BOARD || defined NUCLEO_WBA55_BOARD || defined NUCLEO_WBA65_BOARD
            .gnss_lna_ctrl               = GPIO_PORT_PIN_TO_NUM(GPIOA, GPIO_PIN_1), /*!< Some older Semtech shield PCB revisions used MCU pin to control the LNA power, newer revisions use DIO7 of LR11xx. This setting applies to the older shields */
            .gnss_lna_ctrl_en_gpio_state = 1u,
#  else
#    warning "This sample config supports only NUCLEO-WBA52, NUCLEO-WBA55, and NUCLEO-WBA65 boards. Please define the pin assignment here if you are using another (e.g. custom) board"
            .gnss_lna_ctrl               = HALO_GPIO_NOT_CONNECTED,
            .gnss_lna_ctrl_en_gpio_state = 1u,
#  endif /* NUCLEO_WBA52_BOARD || NUCLEO_WBA55_BOARD || NUCLEO_WBA65_BOARD */
# endif /* LR11XX_RADIO_CFG_GEOLOCATION_SUPPORT */

#if LR11XX_RADIO_CFG_USE_STATUS_LED
#  if defined NUCLEO_WBA52_BOARD || defined NUCLEO_WBA55_BOARD || defined NUCLEO_WBA65_BOARD
        .tx_led                      = GPIO_PORT_PIN_TO_NUM(GPIOA, GPIO_PIN_5),
        .tx_led_on_gpio_state        = 1u,
        .rx_led                      = GPIO_PORT_PIN_TO_NUM(GPIOA, GPIO_PIN_0),
        .rx_led_on_gpio_state        = 1u,
#    if LR11XX_RADIO_CFG_GEOLOCATION_SUPPORT
#      if defined(STM32WBA5x)
        .gnss_scan_led               = GPIO_PORT_PIN_TO_NUM(GPIOA, GPIO_PIN_11),
        .gnss_scan_led_on_gpio_state = 1u,
        .wifi_scan_led               = GPIO_PORT_PIN_TO_NUM(GPIOA, GPIO_PIN_11),
        .wifi_scan_led_on_gpio_state = 1u,
#      elif defined(STM32WBA6x)
        .gnss_scan_led               = GPIO_PORT_PIN_TO_NUM(GPIOA, GPIO_PIN_3),
        .gnss_scan_led_on_gpio_state = 1u,
        .wifi_scan_led               = GPIO_PORT_PIN_TO_NUM(GPIOA, GPIO_PIN_3),
        .wifi_scan_led_on_gpio_state = 1u,
#      else
#        error "This configuration supports only STM32WBA5x and STM32WBA6x MCU families"
#      endif /* STM32WBAxx */
#    endif /* LR11XX_RADIO_CFG_GEOLOCATION_SUPPORT */
#  else
#    warning "This sample config supports only NUCLEO-WBA52, NUCLEO-WBA55, and NUCLEO-WBA65 boards. Please define the pin assignment here if you are using another (e.g. custom) board"
        .tx_led                      = HALO_GPIO_NOT_CONNECTED,
        .tx_led_on_gpio_state        = 0u,
        .rx_led                      = HALO_GPIO_NOT_CONNECTED,
        .rx_led_on_gpio_state        = 0u,
        .gnss_scan_led               = HALO_GPIO_NOT_CONNECTED,
        .gnss_scan_led_on_gpio_state = 0u,
        .wifi_scan_led               = HALO_GPIO_NOT_CONNECTED,
        .wifi_scan_led_on_gpio_state = 0u,
#  endif /* NUCLEO_WBA52_BOARD || NUCLEO_WBA55_BOARD || NUCLEO_WBA65_BOARD */
#endif /* LR11XX_RADIO_CFG_USE_STATUS_LED */
    },

    .pa_config = {
#if LR11XX_RADIO_CFG_USE_EXTERNAL_PA
        .tx_gain_dbi = CONFIG_RADIO_GAIN_DB_TO_INT(16.0),
        .tx_bypass_loss = CONFIG_RADIO_GAIN_DB_TO_INT(1.5),
        .rx_gain_dbi = CONFIG_RADIO_GAIN_DB_TO_INT(18.0),
#else
        .rf_sw_insertion_loss = CONFIG_RADIO_GAIN_DB_TO_INT(0.35),
#endif /* LR11XX_RADIO_CFG_USE_EXTERNAL_PA */
    },

    .pa_cfg_callback            = radio_lr11xx_pa_cfg,

    .bus_selector               = spi_client_config,

    .lfclock_cfg                = LR11XX_SYSTEM_LFCLK_XTAL,

    .enable_lpa                 = true,
    .enable_hpa                 = true,

    .tcxo_config = {
        .ctrl                   = LR11XX_TCXO_CTRL_VTCXO,
        .ctrl_voltage           = LR11XX_SYSTEM_TCXO_CTRL_3_0V,
        .timeout_us             = RADIO_LR11XX_TCXO_START_TIME_US,
#if LR11XX_RADIO_CFG_GEOLOCATION_SUPPORT
        .gnss_timeout_us        = RADIO_LR11XX_TCXO_GNSS_START_TIME_US,
# endif /* LR11XX_RADIO_CFG_GEOLOCATION_SUPPORT */
    },

    .rfswitch = {
        .enable                 = (LR1110MB1LxKS_SUBGHZ_RF_SW_V1 | LR1110MB1LxKS_SUBGHZ_RF_SW_V2 | LR1110MB1LxKS_GNSS_LNA_EN_SW),
        .standby                = LR1110MB1LxKS_RF_SW_ALL_OFF,
        .rx                     = (LR1110MB1LxKS_SUBGHZ_RF_SW_V1),
        .tx                     = (LR1110MB1LxKS_SUBGHZ_RF_SW_V1 | LR1110MB1LxKS_SUBGHZ_RF_SW_V2),
        .tx_hp                  = (LR1110MB1LxKS_SUBGHZ_RF_SW_V2),
        .tx_hf                  = LR1110MB1LxKS_RF_SW_ALL_OFF,
        .gnss                   = (LR1110MB1LxKS_GNSS_LNA_EN_SW), /*!< Newer Semtech shield PCB revisions use DIO7 of LR11xx to control the LNA power, but some older ones use MCU pin. This setting applies to the newer shields */
        .wifi                   = LR1110MB1LxKS_RF_SW_ALL_OFF,
    },

    .regional_config = {
        .radio_region           = RADIO_REGION,
        .reg_param_table_size   = SID_STM32_UTIL_ARRAY_SIZE(radio_lr11xx_regional_param),
        .reg_param_table        = radio_lr11xx_regional_param,
    },

    .state_timings = {
        .sleep_to_full_power_us = 820u, /*!< Time to wake up the radio and reach STDBY_XOSC state. IMPORTANT: this setting should not account for TCXO startup delay as this delay is added automatically by the driver */
        .full_power_to_sleep_us = 0u,   /*!< Currently not used by Sidewalk SDK */
        .rx_to_tx_us            = 0u,   /*!< Currently not used by Sidewalk SDK */
        .tx_to_rx_us            = 0u,   /*!< Currently not used by Sidewalk SDK */
        /* .tcxo_delay_us is automatically set to .tcxo_config.timeout_us by the driver whenever TCXO is used */
    },

    .processing_timings         = {
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
        .lora = {
            .tx_process_delay_us = LR11XX_RADIO_CFG_USE_DEFAULT_TIMINGS,
            .rx_process_delay_us = LR11XX_RADIO_CFG_USE_DEFAULT_TIMINGS,
        },
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 */
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_2
        .fsk = {
            .tx_process_delay_us = LR11XX_RADIO_CFG_USE_DEFAULT_TIMINGS,
            .rx_process_delay_us = LR11XX_RADIO_CFG_USE_DEFAULT_TIMINGS,
        },
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 */
    },

    .internal_buffer = {
        .p                      = radio_lr11xx_buffer,
        .size                   = sizeof(radio_lr11xx_buffer),
    },
};

/* Private function definitions ----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static int32_t radio_lr11xx_pa_cfg(const int32_t desired_tx_power, const radio_lr11xx_regional_param_t * const regional_params, radio_lr11xx_pa_cfg_t * const out_pa_cfg)
{
    static int32_t last_desired_pwr = INT32_MAX;
    int32_t pwr;

    (void)regional_params;

    if (desired_tx_power > CONFIG_RADIO_GAIN_DB_TO_INT(RADIO_LR11XX_MAX_TX_POWER))
    {
        pwr = CONFIG_RADIO_GAIN_DB_TO_INT(RADIO_LR11XX_MAX_TX_POWER);
    }
    else if (desired_tx_power < CONFIG_RADIO_GAIN_DB_TO_INT(RADIO_LR11XX_MIN_TX_POWER))
    {
        pwr = CONFIG_RADIO_GAIN_DB_TO_INT(RADIO_LR11XX_MIN_TX_POWER);
    }
    else
    {
        /* Power selection is within the boundaries, no override required */
        pwr = desired_tx_power;
    }

    if ((desired_tx_power != last_desired_pwr) && (pwr != desired_tx_power))
    {
        /* Compute integer and fractional parts since tiny_vsnprintf may not support %f printout */
        const uint32_t applied_pwr_int_db   = pwr > 0 ? (uint32_t)(pwr / LR11XX_PA_CFG_DB_MULT) : (uint32_t)((-pwr) / LR11XX_PA_CFG_DB_MULT);
        const uint32_t applied_pwr_fract_db = pwr > 0 ? (uint32_t)(pwr % LR11XX_PA_CFG_DB_MULT) : (uint32_t)((-pwr) % LR11XX_PA_CFG_DB_MULT);
        const uint32_t desired_pwr_int_db   = desired_tx_power > 0 ? (uint32_t)(desired_tx_power / LR11XX_PA_CFG_DB_MULT) : (uint32_t)((-desired_tx_power) / LR11XX_PA_CFG_DB_MULT);
        const uint32_t desired_pwr_fract_db = desired_tx_power > 0 ? (uint32_t)(desired_tx_power % LR11XX_PA_CFG_DB_MULT) : (uint32_t)((-desired_tx_power) % LR11XX_PA_CFG_DB_MULT);

        SID_PAL_LOG_WARNING("SubGHz Tx power set to %s%u.%02udBm instead of %s%u.%02udBm due to the HW limits",
                            pwr < 0 ? "-" : "+", applied_pwr_int_db, applied_pwr_fract_db,               /* tiny_vsnprintf() may not support %f format */
                            desired_tx_power < 0 ? "-" : "+", desired_pwr_int_db, desired_pwr_fract_db); /* tiny_vsnprintf() may not support %+d format */
    }

    /* Set the desired Tx power */
    out_pa_cfg->target_tx_power = (int16_t)pwr;

    /* Don't touch the other PA parameters here - the driver will automatically apply matching settings */

    /**
     * NOTE: you may override out_pa_cfg values here if needed and then the driver will keep the overrides supplied via this callback. While
     *       normally you should be ok without any overrides, it may be useful to address certain scenarios or hardware configurations. For
     *       example, if you PCB has different filter tuning for high-power and low-power amplifiers (e.g. HPA tuned to 915MHz band and LPA
     *       tuned to 868MHz) you may explicitly set the power amplifier to use (HPA or LPA) in out_pa_cfg->pa_cfg.pa_sel depending on the
     *       selected regional settings (can be accessed via regional_params parameter)
     */

    /* Remember the latest request to avoid repeated log messages */
    last_desired_pwr = desired_tx_power;

    return RADIO_ERROR_NONE;
}

/* Global function definitions -----------------------------------------------*/

const lr11xx_radio_device_config_t * get_radio_cfg(void)
{
    return &radio_lr11xx_cfg;
}

/*----------------------------------------------------------------------------*/

const struct sid_sub_ghz_links_config * app_get_sub_ghz_config(void)
{
    return &sub_ghz_link_config;
}
