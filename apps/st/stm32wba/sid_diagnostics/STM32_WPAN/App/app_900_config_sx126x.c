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
  * @file    app_900_config_sx126x.c
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
#include <sx126x_radio_config.h>

#include "app_900_config.h"
#include "main.h"
#include "sid_pal_gpio_ext_ifc.h"

/* Private defines -----------------------------------------------------------*/

#define RADIO_MAX_TX_POWER_NA                                      (20)
#define RADIO_MAX_TX_POWER_EU                                      (14)

#if defined (REGION_ALL)
#define RADIO_REGION                                               (RADIO_REGION_NONE)
#elif defined (REGION_US915)
#define RADIO_REGION                                               (RADIO_REGION_NA)
#elif defined (REGION_EU868)
#define RADIO_REGION                                               (RADIO_REGION_EU)
#endif

#define RADIO_SX126X_SPI_BUFFER_SIZE                               (256u)

#define RADIO_SX126X_TCXO_START_TIME_US                            (2000u) /* Typical TCXO start time is about 2ms */

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

static int32_t radio_sx126x_pa_cfg(const int32_t desired_tx_power, const radio_sx126x_regional_param_t * const regional_params, radio_sx126x_pa_dynamic_cfg_t * const out_pa_cfg);

/* Private variables ---------------------------------------------------------*/

SID_STM32_ALIGN_4BYTES(static uint8_t radio_sx1262_buffer[RADIO_SX126X_SPI_BUFFER_SIZE]);

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
    .crc_polynomial    = SID_PAL_SERIAL_BUS_STM32WBAxx_CRC_NOT_USED,
    .crc_length        = 0u,

    .nss_to_sck_cycles = 12u,
    .sck_to_nss_cycles = 0u,
    .nss_to_nss_cycles = 6u,
};

/**
 * @brief SPI driver configuration for SX126X communication
 */
static const sid_pal_serial_bus_client_t spi_client_config = {
#if defined(STM32WBA5x)
    .client_selector           = GPIO_PORT_PIN_TO_NUM(GPIOB, GPIO_PIN_9),
#elif defined(STM32WBA6x)
    .client_selector           = GPIO_PORT_PIN_TO_NUM(GPIOD, GPIO_PIN_14),
#else
#  error "This configuration supports only STM32WBA5x and STM32WBA6x MCU families"
#endif /* STM32WBAxx */
    .speed_hz                  = 12000000u, /* 16MHz is maximum for SX126x */
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
const radio_sx126x_regional_param_t radio_sx126x_regional_param[] =
{
    #if defined (REGION_ALL) || defined (REGION_US915)
    {
        .param_region     = RADIO_REGION_NA,
        .max_tx_power     = { RADIO_MAX_TX_POWER_NA, RADIO_MAX_TX_POWER_NA, RADIO_MAX_TX_POWER_NA,
                              RADIO_MAX_TX_POWER_NA, RADIO_MAX_TX_POWER_NA, RADIO_MAX_TX_POWER_NA },
        .cca_level_adjust = { 0, 0, 0, 0, 0, 0 },
        .ant_dbi          = CONFIG_RADIO_GAIN_DB_TO_INT(2.15),
    },
    #endif
    #if defined (REGION_ALL) || defined (REGION_EU868)
    {
        .param_region     = RADIO_REGION_EU,
        .max_tx_power     = { RADIO_MAX_TX_POWER_EU, RADIO_MAX_TX_POWER_EU, RADIO_MAX_TX_POWER_EU,
                              RADIO_MAX_TX_POWER_EU, RADIO_MAX_TX_POWER_EU, RADIO_MAX_TX_POWER_EU },
        .cca_level_adjust = { 0, 0, 0, 0, 0, 0 },
        .ant_dbi          = CONFIG_RADIO_GAIN_DB_TO_INT(2.15),
    },
    #endif
};

/**
 * @brief Board-specific SX126x driver configuration
 */
const sx126x_radio_device_config_t radio_sx1262_cfg = {
    .id                             = SEMTECH_ID_SX1262,                        /* Chip iID register is not present in SX126x, specify the target chip here. IMPORTANT: putting wrong configuration here may damage the transceiver */

    .regulator_mode                 = SX126X_REG_MODE_DCDC,

    .gpio = {
#if defined(STM32WBA5x)
        .radio_reset            = GPIO_PORT_PIN_TO_NUM(GPIOA, GPIO_PIN_7),  /* SX126xMB2xAS NRESET */
        .radio_irq              = GPIO_PORT_PIN_TO_NUM(GPIOB, GPIO_PIN_14), /* SX126xMB2xAS DIO1 (IRQ) */
        .radio_busy             = GPIO_PORT_PIN_TO_NUM(GPIOB, GPIO_PIN_6),  /* SX126xMB2xAS BUSY - conflict with B2 on NUCLEO-WBA5x boards */
#elif defined(STM32WBA6x)
        .radio_reset            = GPIO_PORT_PIN_TO_NUM(GPIOA, GPIO_PIN_4),  /* SX126xMB2xAS NRESET */
        .radio_irq              = GPIO_PORT_PIN_TO_NUM(GPIOB, GPIO_PIN_14), /* SX126xMB2xAS DIO1 (IRQ) */
        .radio_busy             = GPIO_PORT_PIN_TO_NUM(GPIOB, GPIO_PIN_13), /* SX126xMB2xAS BUSY */
#else
#  error "This configuration supports only STM32WBA5x and STM32WBA6x MCU families"
#endif /* STM32WBAxx */

        .radio_irq_prio_high    = RADIO_INTR_PRIO_HIGH, /* Keep the same priority as high priority for BLE radio - both need elevated priority to respect radio timings */
        .radio_irq_prio_low     = RADIO_INTR_PRIO_LOW,  /* Keep aligned with BLE config. The priority should be low enough to allow RTOS API calls, but we still want to process radio events ASAP */

#if SX126X_RADIO_CFG_USE_FEM_PWR_CTRL
#  if defined(STM32WBA5x)
        .fem_power                  = GPIO_PORT_PIN_TO_NUM(GPIOB, GPIO_PIN_15), /* SX126xMB2xAS ANT_SW - use this pin to power on and shutdown FEM/RF switch */
#  elif defined(STM32WBA6x)
        .fem_power                  = GPIO_PORT_PIN_TO_NUM(GPIOA, GPIO_PIN_10), /* SX126xMB2xAS ANT_SW - use this pin to power on and shutdown FEM/RF switch */
#  else
#    error "This configuration supports only STM32WBA5x and STM32WBA6x MCU families"
#  endif /* STM32WBAxx */
        .fem_power_en_gpio_state    = 1u,                                       /*!< State of the MCU GPIO pin to turn FEM/RF switch on */
#endif /* SX126X_RADIO_CFG_USE_FEM_PWR_CTRL */
#if SX126X_RADIO_CFG_USE_TX_RX_CTRL
        .fem_tx_rx_mode             = HALO_GPIO_NOT_CONNECTED,                  /*!< Select signal direction in FEM/RF switch (Tx or Rx) */
        .fem_tx_mode_sel_gpio_state = 1u,                                       /*!< State of the MCU GPIO pin to select Tx mode in FEM/RF switch */
#endif /* SX126X_RADIO_CFG_USE_TX_RX_CTRL */
#if SX126X_RADIO_CFG_USE_EXTERNAL_PA && SX126X_RADIO_CFG_USE_FEM_BYPASS_CTRL
        .fem_bypass                 = HALO_GPIO_NOT_CONNECTED,                  /*!< Enables Tx bypass on FEM/PA, connecting SX126x's built-in PA to the antenna directly */
        .fem_bypass_en_gpio_state   = 1u,                                       /*!< State of the MCU GPIO pin to activate bypass mode */
#endif /* SX126X_RADIO_CFG_USE_EXTERNAL_PA && SX126X_RADIO_CFG_USE_FEM_BYPASS_CTRL */

#if SX126X_RADIO_CFG_USE_STATUS_LED
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
#endif
    },

    .pa_config = {
        .rx_boost_en            = FALSE,                             /*!< Enables Rx boost in SX126x. This results in approx. +2dB Rx gain increase */
        .dio2_ctrl_en           = TRUE,                              /*!< Use DIO2 pin of SX126x to control Rx/Tx mode selection on RF switch/FEM. DIO2 is high for Tx and low for Rx */
        .pa_cfg_callback        = radio_sx126x_pa_cfg,               /*!< User-defined callback that is triggered by the driver when PA or Tx power is being configured */
#if SX126X_RADIO_CFG_USE_EXTERNAL_PA                                 /*!< This part is relevant only when an external RF Power Amplifier (PA) is mounted on PCB. Reference config is for SKY66420 FEM */
        .tx_gain_dbi            = CONFIG_RADIO_GAIN_DB_TO_INT(16.0), /*!< Tx gain of the external PA, expressed in cdBi (dBi * 100) */
        .tx_bypass_loss         = CONFIG_RADIO_GAIN_DB_TO_INT(1.5),  /*!< Insertion loss when Tx PA is bypassed, expressed in cdBi (dBi * 100) */
        .rx_gain_dbi            = CONFIG_RADIO_GAIN_DB_TO_INT(18.0), /*!< Rx gain of the external LNA, expressed in cdBi (dBi * 100) */
#else                                                                /*!< This part is valid when just an RF switch with no active amplifiers is used */
        .rf_sw_insertion_loss   = CONFIG_RADIO_GAIN_DB_TO_INT(0.35), /*!< RF switch insertion loss - applies when no external PA is used, just an RF switch, expressed in cdBi (dBi * 100). Reference config corresponds to PE4259 RF switch */
#endif /* SX126X_RADIO_CFG_USE_EXTERNAL_PA */
    },

    .bus_factory                = &radio_spi_factory,
    .bus_selector               = spi_client_config,

    .tcxo_config = {
        .ctrl                   = SX126X_TCXO_CTRL_NONE,           /* SX126x-MB2xAS boards use XTAL. If your board uses TCXO configure it here */
        .ctrl_voltage           = SX126X_TCXO_CTRL_3_0V,           /*!< Supply voltage level. Valid only when TCXO is powered via SX126x's DIO3 pin */
        .timeout_us             = RADIO_SX126X_TCXO_START_TIME_US, /*!< TCXO start time in microseconds */
    },

    .regional_config = {
        .radio_region = RADIO_REGION,
        .reg_param_table_size = sizeof(radio_sx126x_regional_param) / sizeof(radio_sx126x_regional_param[0]),
        .reg_param_table = radio_sx126x_regional_param,
    },

    .state_timings = {
        .sleep_to_full_power_us = 570u, /*!< Time to wake up the radio and reach STBY_XOSC state. IMPORTANT: this setting should not account for TCXO startup delay as this delay is added automatically by the driver */
        .full_power_to_sleep_us = 0u,   /*!< Currently not used by Sidewalk SDK */
        .rx_to_tx_us            = 0u,   /*!< Currently not used by Sidewalk SDK */
        .tx_to_rx_us            = 0u,   /*!< Currently not used by Sidewalk SDK */
        /* .tcxo_delay_us is automatically set to .tcxo_config.timeout_us by the driver whenever TCXO is used */
    },

    .processing_timings         = {
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
        .lora = {
            .tx_process_delay_us = SX126X_RADIO_CFG_USE_DEFAULT_TIMINGS,
            .rx_process_delay_us = SX126X_RADIO_CFG_USE_DEFAULT_TIMINGS,
        },
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 */
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_2
        .fsk = {
            .tx_process_delay_us = SX126X_RADIO_CFG_USE_DEFAULT_TIMINGS,
            .rx_process_delay_us = SX126X_RADIO_CFG_USE_DEFAULT_TIMINGS,
        },
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 */
    },

    .internal_buffer = {
        .p    = radio_sx1262_buffer,
        .size = sizeof(radio_sx1262_buffer),
    },
};

/* Private function definitions ----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static int32_t radio_sx126x_pa_cfg(const int32_t desired_tx_power, const radio_sx126x_regional_param_t * const regional_params, radio_sx126x_pa_dynamic_cfg_t * const out_pa_cfg)
{
    static int32_t last_desired_pwr = INT32_MAX;
    int32_t pwr, max_pwr, min_pwr;

    (void)regional_params;

    switch (radio_sx1262_cfg.id)
    {
        case SEMTECH_ID_SX1261:
            max_pwr = CONFIG_RADIO_GAIN_DB_TO_INT(SX1261_MAX_TX_POWER);
            min_pwr = CONFIG_RADIO_GAIN_DB_TO_INT(SX1261_MIN_TX_POWER);
            break;

        case SEMTECH_ID_SX1262:
            max_pwr = CONFIG_RADIO_GAIN_DB_TO_INT(SX1262_MAX_TX_POWER);
            min_pwr = CONFIG_RADIO_GAIN_DB_TO_INT(SX1262_MIN_TX_POWER);
            break;

        default:
            SID_PAL_LOG_ERROR("Invalid chip ID in radio config (0x%02X)", radio_sx1262_cfg.id);
            return RADIO_ERROR_INVALID_PARAMS;
    }

    if (desired_tx_power > max_pwr)
    {
        pwr = max_pwr;
    }
    else if (desired_tx_power < min_pwr)
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
        /* Compute integer and fractional parts since tiny_vsnprintf may not support %f printout */
        const uint32_t applied_pwr_int_db   = pwr > 0 ? (uint32_t)(pwr / 100) : (uint32_t)((-pwr) / 100);
        const uint32_t applied_pwr_fract_db = pwr > 0 ? (uint32_t)(pwr % 100) : (uint32_t)((-pwr) % 100);
        const uint32_t desired_pwr_int_db   = desired_tx_power > 0 ? (uint32_t)(desired_tx_power / 100) : (uint32_t)((-desired_tx_power) / 100);
        const uint32_t desired_pwr_fract_db = desired_tx_power > 0 ? (uint32_t)(desired_tx_power % 100) : (uint32_t)((-desired_tx_power) % 100);

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
     *       example, you can customize pa_lut, pa_duty_cycle or ramp_time to better match the RF front end on your PCB to achieve better
     *       power efficiency
     */

    /* Remember the latest request to avoid repeated log messages */
    last_desired_pwr = desired_tx_power;

    return RADIO_ERROR_NONE;
}

/* Global function definitions -----------------------------------------------*/

const sx126x_radio_device_config_t * get_radio_cfg(void)
{
    return &radio_sx1262_cfg;
}

/*----------------------------------------------------------------------------*/

const struct sid_sub_ghz_links_config * app_get_sub_ghz_config(void)
{
    return &sub_ghz_link_config;
}
