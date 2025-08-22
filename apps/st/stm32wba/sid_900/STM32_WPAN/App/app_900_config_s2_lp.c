/**
  ******************************************************************************
  * @file    app_900_config_s2_lp.c
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

/* Includes ------------------------------------------------------------------*/

#include <sid_pal_log_ifc.h>
#include <sid_pal_radio_ifc.h>
#include <sid_pal_serial_bus_ifc.h>
#include <sid_pal_serial_bus_spi_config.h>
#include <sid_stm32_common_utils.h>
#include <s2_lp_radio_config.h>

#include "app_900_config.h"
#include "main.h"
#include "sid_pal_gpio_ext_ifc.h"

/* Private defines -----------------------------------------------------------*/

#define RADIO_MAX_TX_POWER_NA                                      (20)
#define RADIO_MAX_TX_POWER_EU                                      (14)

#if S2LP_RADIO_CFG_USE_EXTERNAL_PA
#  define EXT_PA_MAX_POWER_DBM                                     (27) /*!< Maximum Tx power of the external PA - this setting corresponds to the X-NUCLEO-S2915A1/SKY66420, modify it to match the actual PA/FEM used */
#endif /* S2LP_RADIO_CFG_USE_EXTERNAL_PA */

#if defined (REGION_ALL)
#  define RADIO_REGION                                             (RADIO_REGION_NONE)
#elif defined (REGION_US915)
#  define RADIO_REGION                                             (RADIO_REGION_NA)
#elif defined (REGION_EU868)
#  define RADIO_REGION                                             (RADIO_REGION_EU)
#else
#  error "SubGHz configuration error. RADIO_REGION is not defined"
#endif

#define RADIO_S2LP_SPI_BUFFER_SIZE                                 (256u)

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

static int32_t radio_s2_lp_pa_cfg(const int32_t desired_tx_power, const s2_lp_radio_regional_params_t * const regional_params, s2_lp_radio_pa_dynamic_cfg_t * const out_pa_cfg);

/* Private variables ---------------------------------------------------------*/

SID_STM32_ALIGN_4BYTES(static uint8_t s2lp_processing_buffer[RADIO_S2LP_SPI_BUFFER_SIZE]);

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
    .crc_init_pattern  = 0u,

    .nss_to_sck_cycles = 6u,
    .sck_to_nss_cycles = 0u,
    .nss_to_nss_cycles = 2u,
};

/**
 * @brief SPI driver configuration for S2-LP communication
 */
static const sid_pal_serial_bus_client_t spi_client_config = {
#if defined(STM32WBA5x)
    .client_selector           = GPIO_PORT_PIN_TO_NUM(GPIOA, GPIO_PIN_12),
#elif defined(STM32WBA6x)
    .client_selector           = GPIO_PORT_PIN_TO_NUM(GPIOB, GPIO_PIN_9),
#else
#  error "This configuration supports only STM32WBA5x and STM32WBA6x MCU families"
#endif /* STM32WBAxx */
    .speed_hz                  = 6000000u, /* 10MHz is maximum for S2-LP */
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
static const s2_lp_radio_regional_params_t radio_s2_lp_regional_params[] =
{
#if defined (REGION_ALL) || defined (REGION_US915)
    {
        .param_region       = RADIO_REGION_NA,
        .max_tx_power       = { RADIO_MAX_TX_POWER_NA, RADIO_MAX_TX_POWER_NA, RADIO_MAX_TX_POWER_NA,
                                RADIO_MAX_TX_POWER_NA, RADIO_MAX_TX_POWER_NA, RADIO_MAX_TX_POWER_NA },
        .cca_level_adjust   = { 0, 0, 0, 0, 0, 0 },
        .ant_dbi            = CONFIG_RADIO_GAIN_DB_TO_INT(3.0),
        .base_frequency_hz  = 902200000u,
        .channel_spacing_hz = 200000u,
    },
#endif
#if defined (REGION_ALL) || defined (REGION_EU868)
    {
        .param_region       = RADIO_REGION_EU,
        .max_tx_power       = { RADIO_MAX_TX_POWER_EU, RADIO_MAX_TX_POWER_EU, RADIO_MAX_TX_POWER_EU,
                                RADIO_MAX_TX_POWER_EU, RADIO_MAX_TX_POWER_EU, RADIO_MAX_TX_POWER_EU },
        .cca_level_adjust   = { 0, 0, 0, 0, 0, 0 },
        .ant_dbi            = CONFIG_RADIO_GAIN_DB_TO_INT(3.0),
        .base_frequency_hz  = 863125000u,
        .channel_spacing_hz = 200000u,
    },
#endif
};

/**
 * @brief Board-specific S2-LP driver configuration
 */
static const s2_lp_radio_device_config_t radio_s2lp_cfg = {
    .pa_cfg_callback              = radio_s2_lp_pa_cfg,
    .bus_factory                  = &radio_spi_factory,
    .spi_client_cfg               = &spi_client_config,
    .gpio = {
        .radio_irq                = {
            .mcu_pin              = GPIO_PORT_PIN_TO_NUM(GPIOA, GPIO_PIN_0),
            .s2lp_pin             = S2_LP_GPIO_3,
            .prio_high            = RADIO_INTR_PRIO_HIGH, /* Keep the same priority as high priority for BLE radio - both need elevated priority to respect radio timings */
            .prio_low             = RADIO_INTR_PRIO_LOW,  /* Keep aligned with BLE config. The priority should be low enough to allow RTOS API calls, but we still want to process radio events ASAP */
        },
        .radio_shutdown           = GPIO_PORT_PIN_TO_NUM(GPIOB, GPIO_PIN_0),
#if S2LP_RADIO_CFG_USE_EXTERNAL_PA /*!< This part is relevant only when an external RF Power Amplifier (PA) is mounted on PCB */
#  if defined(STM32WBA5x)
        .rf_fem_csd               = GPIO_PORT_PIN_TO_NUM(GPIOA, GPIO_PIN_7),
#  elif defined(STM32WBA6x)
        .rf_fem_csd               = GPIO_PORT_PIN_TO_NUM(GPIOA, GPIO_PIN_4),
#  else
#    error "This configuration supports only STM32WBA5x and STM32WBA6x MCU families"
#  endif /* STM32WBAxx */
        .rf_fem_ctx               = GPIO_PORT_PIN_TO_NUM(GPIOA, GPIO_PIN_1),
        .rf_fem_cps               = GPIO_PORT_PIN_TO_NUM(GPIOA, GPIO_PIN_2),
#endif /* S2LP_RADIO_CFG_USE_EXTERNAL_PA */
#if S2LP_RADIO_CFG_USE_STATUS_LED  /*!< This part is relevant only if there's an LED available on  PCB to be controlled by the driver */
#  if defined NUCLEO_WBA52_BOARD
        .tx_led                   = GPIO_PORT_PIN_TO_NUM(GPIOA, GPIO_PIN_9),
        .tx_led_on_gpio_state     = 1u,
        .rx_led                   = GPIO_PORT_PIN_TO_NUM(GPIOA, GPIO_PIN_9),
        .rx_led_on_gpio_state     = 1u,
#  elif defined NUCLEO_WBA55_BOARD
        .tx_led                   = GPIO_PORT_PIN_TO_NUM(GPIOB, GPIO_PIN_15),
        .tx_led_on_gpio_state     = 1u,
        .rx_led                   = GPIO_PORT_PIN_TO_NUM(GPIOB, GPIO_PIN_15),
        .rx_led_on_gpio_state     = 1u,
#  elif defined NUCLEO_WBA65_BOARD
        .tx_led                   = GPIO_PORT_PIN_TO_NUM(GPIOB, GPIO_PIN_11),
        .tx_led_on_gpio_state     = 1u,
        .rx_led                   = GPIO_PORT_PIN_TO_NUM(GPIOB, GPIO_PIN_11),
        .rx_led_on_gpio_state     = 1u,
#  else
#    warning "This sample config supports only NUCLEO-WBA52, NUCLEO-WBA55, and NUCLEO-WBA65 boards. Please define the pin assignment here if you are using another (e.g. custom) board"
        .tx_led                   = HALO_GPIO_NOT_CONNECTED,
        .tx_led_on_gpio_state     = 0u,
        .rx_led                   = HALO_GPIO_NOT_CONNECTED,
        .rx_led_on_gpio_state     = 0u,
#  endif /* NUCLEO_WBA52_BOARD || NUCLEO_WBA55_BOARD || NUCLEO_WBA65_BOARD */
#endif /* S2LP_RADIO_CFG_USE_STATUS_LED */
    },

    .use_external_clock           = FALSE,     /*!< Set to TRUE if S2-LP is clocked from an external source (e.g. TCXO, MCU, etc.) - this disables built-in oscillator of S2-LP. Set to FALSE when using XTAL to clock the S2-LP */
    .xin_freq                     = 50000000u, /*!< S2-LP clock speed in Hz, whether it comes from XTAL or an external source. Valid options are 24MHz, 25MH, 26Mhz and 48MHz, 50Mhz, 52MHz. Other frequencies are not supported */

    .pa_config                    = {
#if S2LP_RADIO_CFG_USE_EXTERNAL_PA             /*!< This part is relevant only when an external RF Power Amplifier (PA) is mounted on PCB. Reference config corresponds to the X-NUCLEO-S2915A1 board/SKY66420 FEM */
        .tx_gain_dbi              = CONFIG_RADIO_GAIN_DB_TO_INT(16.0),
        .tx_bypass_loss           = CONFIG_RADIO_GAIN_DB_TO_INT(1.5),
        .rx_gain_dbi              = CONFIG_RADIO_GAIN_DB_TO_INT(18.0),
#else                                          /*!< When no external PA is used the driver compensate only for the RF switch insertion loss. Reference config applies to the X-NUCLEO-S2868A2/BALF-SPI2-01D3 */
        .rf_sw_insertion_loss     = CONFIG_RADIO_GAIN_DB_TO_INT(2.0),
#endif /* S2LP_RADIO_CFG_USE_EXTERNAL_PA */
    },

    .regional_config = {
        .radio_region             = RADIO_REGION,
        .reg_param_table_size     = SID_STM32_UTIL_ARRAY_SIZE(radio_s2_lp_regional_params),
        .reg_param_table          = radio_s2_lp_regional_params,
    },

    .state_timings = {
        .sleep_to_full_power_us   = 288u, /*!< Wake-up time without RCO calibration */
        .full_power_to_sleep_us   = 0u,   /*!< Currently not used by Sidewalk SDK */
        .rx_to_tx_us              = 0u,   /*!< Currently not used by Sidewalk SDK */
        .tx_to_rx_us              = 0u,   /*!< Currently not used by Sidewalk SDK */
        .tcxo_delay_us            = 0u,   /*!< Currently not used by Sidewalk SDK */
    },

    .processing_timings = {
        .tx_process_delay_us      = S2LP_RADIO_CFG_USE_DEFAULT_TIMINGS,
        .rx_process_delay_us      = S2LP_RADIO_CFG_USE_DEFAULT_TIMINGS,
    },

    .internal_buffer = {
        .p    = s2lp_processing_buffer,
        .size = sizeof(s2lp_processing_buffer),
    },
};

/* Private function definitions ----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static int32_t radio_s2_lp_pa_cfg(const int32_t desired_tx_power, const s2_lp_radio_regional_params_t * const regional_params, s2_lp_radio_pa_dynamic_cfg_t * const out_pa_cfg)
{
    static int32_t last_desired_pwr = INT32_MAX;
    int32_t pwr;

    /* Ensure the requested power does not exceed the maximum power of the PA */
    const int32_t max_pwr =
#if S2LP_RADIO_CFG_USE_EXTERNAL_PA
        EXT_PA_MAX_POWER_DBM;
#else
        S2_LP_IC_MAX_PA_POWER_DBM;
#endif /* S2LP_RADIO_CFG_USE_EXTERNAL_PA */

    if (desired_tx_power > max_pwr)
    {
        pwr = max_pwr;
    }
    else if (desired_tx_power < S2_LP_IC_MIN_PA_POWER_DBM)
    {
        pwr = S2_LP_IC_MIN_PA_POWER_DBM;
    }
    else
    {
        /* Power selection is within the boundaries, no override required */
        pwr = desired_tx_power;
    }

    if ((desired_tx_power != last_desired_pwr) && (pwr != desired_tx_power))
    {
        SID_PAL_LOG_WARNING("S2-LP Tx power set to %s%ddBm instead of %s%ddBm due to the HW limits",
                            pwr >= 0 ? "+" : "", pwr, desired_tx_power >= 0 ? "+" : "", desired_tx_power); /* tiny_vsnprintf() may not support %+d format */
    }

    out_pa_cfg->tx_power     = pwr;
    out_pa_cfg->ramp_time_us = 40u; /* Use 40us ramp time - user may adjust this dynamically in this callback (e.g. dpending on the Tx power) */

    /**
     * NOTE: you may override out_pa_cfg values here if needed and then the driver will keep the overrides supplied via this callback. While
     *       normally you should be ok without any overrides, it may be useful to address certain scenarios or hardware configurations. For
     *       example, you can customize ramp_time to better match the RF front end on your PCB to achieve better power efficiency
     */

    /* Remember the latest request to avoid repeated log messages */
    last_desired_pwr         = desired_tx_power;

    return RADIO_ERROR_NONE;
}

/* Global function definitions -----------------------------------------------*/

const s2_lp_radio_device_config_t* get_radio_cfg(void)
{
    return &radio_s2lp_cfg;
}

/*----------------------------------------------------------------------------*/

const struct sid_sub_ghz_links_config* app_get_sub_ghz_config(void)
{
    return &sub_ghz_link_config;
}
