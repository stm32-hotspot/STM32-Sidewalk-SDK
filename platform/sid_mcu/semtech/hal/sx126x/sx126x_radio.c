/*
 * Copyright (c) 2019-2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * This file supports radio HAL interface
 */
/**
  ******************************************************************************
  * @file    sx126x_radio.c
  * @brief   Semtech SX126x radio driver for Sidewalk running on STM32 platform
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

#include "sx126x_radio.h"
#include "sx126x_radio_config.h"

/* Sidewalk interfaces */
#include <sid_clock_ifc.h>
#include <sid_time_ops.h>
#include <sid_pal_assert_ifc.h>
#include <sid_pal_critical_region_ifc.h>
#include <sid_pal_delay_ifc.h>
#include <sid_pal_log_ifc.h>

#if SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION
/* LoRa Basics Modem interfaces */
#  include <smtc_modem_hal.h>

/* CMSIS OS interfaces */
#  include <cmsis_os2.h>
#endif /* SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION */

/* Utilities and helpers */
#include <sid_stm32_common_utils.h>
#include <cmsis_compiler.h>

#if SID_SDK_CONFIG_RF_TIMINGS_MEASUREMENT
#  include "main.h"
#endif

/* LPM handling */
#include "app_conf.h"
#if (CFG_LPM_LEVEL != 0)
#  include <stm32_lpm.h>
#endif /* (CFG_LPM_LEVEL != 0) */

/* Private defines -----------------------------------------------------------*/

#ifndef SX126X_RADIO_EXTRA_LOGGING
/* Set SX126X_RADIO_EXTRA_LOGGING to 1 to enable extended logs */
#  define SX126X_RADIO_EXTRA_LOGGING (0)
#endif

#if SX126X_RADIO_EXTRA_LOGGING
#  define SX126X_RADIO_LOG_ERROR(...)         SID_PAL_LOG_ERROR(__VA_ARGS__)
#  define SX126X_RADIO_LOG_WARNING(...)       SID_PAL_LOG_WARNING(__VA_ARGS__)
#  define SX126X_RADIO_LOG_INFO(...)          SID_PAL_LOG_INFO(__VA_ARGS__)
#  define SX126X_RADIO_LOG_DEBUG(...)         SID_PAL_LOG_DEBUG(__VA_ARGS__)
#  define SX126X_RADIO_LOG_TRACE(...)         SID_PAL_LOG_TRACE(__VA_ARGS__)
#else
#  define SX126X_RADIO_LOG_ERROR(...)         ((void)0u)
#  define SX126X_RADIO_LOG_WARNING(...)       ((void)0u)
#  define SX126X_RADIO_LOG_INFO(...)          ((void)0u)
#  define SX126X_RADIO_LOG_DEBUG(...)         ((void)0u)
#  define SX126X_RADIO_LOG_TRACE(...)         ((void)0u)
#endif

#ifndef RADIO_SX126X_TXPWR_WORKAROUND
#  define RADIO_SX126X_TXPWR_WORKAROUND       (1u)
#endif /* RADIO_SX126X_TXPWR_WORKAROUND */

#if RADIO_SX126X_TXPWR_WORKAROUND
#define SX1262_BAND_EDGE_LIMIT_FREQ           (903000000u)
#define SX1262_REG_VAL_FREQ_LOW               (0x1Fu)
#define SX1262_REG_VAL_FREQ_HIGH              (0x18u)
#endif /* RADIO_SX126X_TXPWR_WORKAROUND */

#define SX126X_RADIO_MODEM_MODE_INVALID       (0xFFu)

#define SX126X_DEFAULT_LORA_IRQ_MASK          ( SX126X_IRQ_TX_DONE           \
                                              | SX126X_IRQ_RX_DONE           \
                                              | SX126X_IRQ_PREAMBLE_DETECTED \
                                              | SX126X_IRQ_HEADER_ERROR      \
                                              | SX126X_IRQ_CRC_ERROR         \
                                              )

#define SX126X_LORA_CAD_IRQ_MASK              ( SX126X_DEFAULT_LORA_IRQ_MASK \
                                              | SX126X_IRQ_CAD_DONE          \
                                              | SX126X_IRQ_CAD_DETECTED      \
                                              )

#define SX126X_DEFAULT_FSK_IRQ_MASK           ( SX126X_IRQ_TX_DONE           \
                                              | SX126X_IRQ_RX_DONE           \
                                              | SX126X_IRQ_PREAMBLE_DETECTED \
                                              | SX126X_IRQ_SYNC_WORD_VALID   \
                                              )

#define SX126X_FSK_CARRIER_SENSE_IRQ_MASK     ( SX126X_IRQ_PREAMBLE_DETECTED )

#define SX126X_RADIO_IRQ_PIN_INACTIVE_STATE   (0u)       /*!< State of the radio IRQ pin when there's no IRQ indication from the radio */

#define SX126X_DEFAULT_TRIM_CAP_VAL           (0x1212u)

#define SX126X_MIN_CHANNEL_FREE_DELAY_US      (1u)
#define SX126X_MIN_CHANNEL_NOISE_DELAY_US     (30u)
#define SX126X_NOISE_SAMPLE_SIZE              (32u)

/**
  * @brief drive value used anytime radio is NOT in TX low power mode
  * @note override the default configuration of radio_driver.c
  */
#define SX126X_SMPS_DRIVE_SETTING_DEFAULT     (SX126X_SMPS_DRV_40)

/**
  * @brief drive value used anytime radio is in TX low power mode
  *        TX low power mode is the worst case because the PA sinks from SMPS
  *        while in high power mode, current is sunk directly from the battery
  * @note override the default configuration of radio_driver.c
  */
#define SX126X_SMPS_DRIVE_SETTING_MAX         (SX126X_SMPS_DRV_60)

#define SX126X_CAD_DEFAULT_TX_TIMEOUT         (0u)                  /* Disable Tx timeout for CAD */

#define SX126X_PA_CFG_DB_MULT                 (100)                 /*!< Multiplier to conduct integer calculations with more precision */

#define SX126X_ANT_GAIN_CAP_REGION_NA         ((int32_t)(6 * SX126X_PA_CFG_DB_MULT)) /*!< Antenna gain (in dBi * 100) upper limit as per FCC requirements */

#define SX126X_PA_CFG_LPA_UPPER_LIMIT_DBM     (SX1261_MAX_TX_POWER) /*!< The upper limit of the built-in LPA in dBm */
#define SX126X_PA_CFG_LPA_OPTIMIZATION_14_DBM (14)                  /*!< LPA Tx power level for which an optimized PA config exists */
#define SX126X_PA_CFG_LPA_OPTIMIZATION_10_DBM (10)                  /*!< LPA Tx power level for which an optimized PA config exists */
#define SX126X_PA_CFG_LPA_LOWER_LIMIT_DBM     (SX1261_MIN_TX_POWER) /*!< The lower limit of the built-in LPA in dBm */
#define SX126X_PA_CFG_HPA_UPPER_LIMIT_DBM     (SX1262_MAX_TX_POWER) /*!< The upper limit of the built-in HPA in dBm */
#define SX126X_PA_CFG_HPA_OPTIMIZATION_20_DBM (20)                  /*!< HPA Tx power level for which an optimized PA config exists */
#define SX126X_PA_CFG_HPA_OPTIMIZATION_17_DBM (17)                  /*!< HPA Tx power level for which an optimized PA config exists */
#define SX126X_PA_CFG_HPA_OPTIMIZATION_14_DBM (14)                  /*!< HPA Tx power level for which an optimized PA config exists */
#define SX126X_PA_CFG_HPA_LOWER_LIMIT_DBM     (SX1262_MIN_TX_POWER) /*!< The lower limit of the built-in HPA in dBm */

/* Private macro -------------------------------------------------------------*/

#define SX126X_RADIO_GET_FREQ_BAND_STR(_FB_)  (SX126X_BAND_900M == (_FB_) ? "902-928 MHz" :\
                                               SX126X_BAND_850M == (_FB_) ? "863-870 MHz" :\
                                               SX126X_BAND_770M == (_FB_) ? "779-787 MHz" :\
                                               SX126X_BAND_460M == (_FB_) ? "470-510 MHz" :\
                                               SX126X_BAND_430M == (_FB_) ? "430-440 MHz" :\
                                               "<invalid>")

/* Private types -------------------------------------------------------------*/

typedef struct {
    uint8_t freq_border_low;
    uint8_t freq_border_high;
} sx126x_cal_img_freq_range_t;

/* Private variables ---------------------------------------------------------*/

static halo_drv_semtech_ctx_t drv_ctx = {0};

/* Private constants ---------------------------------------------------------*/

static const sx126x_cal_img_freq_range_t cal_img_bands[] = {
    {
        /* Frequency band 902-928 MHz */
        .freq_border_low  = SX126X_IMAGE_CAL_GET_FREQ1_VAL(902u),
        .freq_border_high = SX126X_IMAGE_CAL_GET_FREQ2_VAL(928u),
    },
    {
        /* Frequency band 863-870 MHz */
        .freq_border_low  = SX126X_IMAGE_CAL_GET_FREQ1_VAL(863u),
        .freq_border_high = SX126X_IMAGE_CAL_GET_FREQ2_VAL(875u), /* Give it a bit more room because some channels are extremely close to the 870 MHz edge (e.g., 869.525 MHz) */
    },
    {
        /* Frequency band 779-787 MHz */
        .freq_border_low  = SX126X_IMAGE_CAL_GET_FREQ1_VAL(774u), /* Give it a bit more room because some channels are extremely close to the 779 MHz edge (e.g., 779.5 MHz) */
        .freq_border_high = SX126X_IMAGE_CAL_GET_FREQ2_VAL(787u),
    },
    {
        /* Frequency band 470-510 MHz */
        .freq_border_low  = SX126X_IMAGE_CAL_GET_FREQ1_VAL(470u),
        .freq_border_high = SX126X_IMAGE_CAL_GET_FREQ2_VAL(515u), /* Give it a bit more room because some channels are extremely close to the 510 MHz edge (e.g., 509.7 MHz) */
    },
    {
        /* Frequency band 430-440 MHz */
        .freq_border_low  = SX126X_IMAGE_CAL_GET_FREQ1_VAL(430u),
        .freq_border_high = SX126X_IMAGE_CAL_GET_FREQ1_VAL(440u),
    },
};

#if SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION
static const osSemaphoreAttr_t sidewalk_radio_access_lock_attr = {
    .name = "sid_radio_lock",
};
#endif /* SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION */

/* Private function prototypes -----------------------------------------------*/

static        int32_t            radio_sx126x_configure_clocks(void);
static        int32_t            radio_sx126x_configure_rx_boost(void);
static        int32_t            radio_sx126x_platform_init(void);
static        int32_t            radio_sx126x_platform_deinit(void);
static inline int32_t            radio_set_irq_mask(const sx126x_irq_mask_t irq_mask);
static inline int32_t            radio_set_modem_to_lora_mode(void);
static inline int32_t            radio_set_modem_to_fsk_mode(void);
static inline int32_t            radio_clear_irq_status_all(void);
static inline int32_t            sx126x_radio_smps_set(const uint8_t level);
static inline sx126x_freq_band_t radio_sx126x_get_freq_band(const uint32_t freq_hz);
static        void               radio_sx126x_on_radio_timeout_event(void * arg, sid_pal_timer_t * originator);
static inline int32_t            radio_sx126x_start_radio_timeout_mon(const uint32_t timeout_us);
static inline int32_t            radio_sx126x_convert_to_db(const int32_t multiplied_db);

/* Private function definitions ----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static int32_t radio_sx126x_configure_clocks(void)
{
    int32_t err;
    sx126x_status_t sys_err;

    do
    {
        /* Bring up TCXO (if used) */
        if (drv_ctx.config->tcxo_config.ctrl != SX126X_TCXO_CTRL_NONE)
        {
            uint32_t tcxo_start_timeout;

            SID_PAL_ASSERT((drv_ctx.config->tcxo_config.ctrl_voltage >= SX126X_TCXO_CTRL_1_6V) && (drv_ctx.config->tcxo_config.ctrl_voltage <= SX126X_TCXO_CTRL_3_3V));

            if (SX126X_TCXO_CTRL_DIO3 == drv_ctx.config->tcxo_config.ctrl)
            {
                tcxo_start_timeout = SX126X_US_TO_TUS(drv_ctx.config->tcxo_config.timeout_us);
            }
            else /* if (SX126X_TCXO_CTRL_VDD == drv_ctx.config->tcxo_config.ctrl) */
            {
                /**
                 * Since TCXO is powered directly from VDD in this config we don't need a start timeout for it. However,
                 * we can't set the timeout to 0 in SX126x because this will switch it to XOSC mode, so we have to use
                 * the minimum timeout of 1 tick (15.625us)
                 */
                tcxo_start_timeout = 1u;
            }

            /* Enable TCXO */
            sys_err = sx126x_set_dio3_as_tcxo_ctrl(&drv_ctx, drv_ctx.config->tcxo_config.ctrl_voltage, tcxo_start_timeout);
            if (sys_err != SX126X_STATUS_OK)
            {
                SID_PAL_LOG_ERROR("Failed to configure SX126x TCXO. Error %d", (int32_t)sys_err);
                err = RADIO_ERROR_IO_ERROR;
                break;
            }

            /* TCXO usage automatically generates error on boot because HF clock cannot be started. Clear it */
            sys_err = sx126x_clear_device_errors(&drv_ctx);
            if (sys_err != SX126X_STATUS_OK)
            {
                err = RADIO_ERROR_IO_ERROR;
                break;
            }

            /* Re-calibrate XOSC since startup calibration is not valid (TCXO is off on boot) */
            sys_err = sx126x_cal(&drv_ctx, SX126X_CAL_ALL);
            if (sys_err != SX126X_STATUS_OK)
            {
                SID_PAL_LOG_ERROR("Failed to re-calibrate SX126x after switching to TCXO. Error %d", (int32_t)sys_err);
                err = RADIO_ERROR_IO_ERROR;
                break;
            }
        }
        else
        {
            /* XTAL is used */

            if (drv_ctx.config->trim_cap_val_callback != NULL)
            {
                /* User-supplied XTAL trim selection function provided - call it to get XTAL trim (e.g. load from OTP, manufacturing data, etc. - this allows end-of-line calibrations for individual devices) */
                err = drv_ctx.config->trim_cap_val_callback(&drv_ctx.trim);
                if (err != RADIO_ERROR_NONE)
                {
                    break;
                }
            }
            else
            {
                /* Use a default value */
                drv_ctx.trim = SX126X_DEFAULT_TRIM_CAP_VAL;
            }
        }

        err = RADIO_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static int32_t radio_sx126x_configure_rx_boost(void)
{
    int32_t err;
    sx126x_status_t sys_err;
    sx126x_hal_status_t hal_err;

    do
    {
        if (FALSE == drv_ctx.config->pa_config.rx_boost_en)
        {
            /* Rx boost is not used */
            err = RADIO_ERROR_NONE;
            break;
        }

        /* Enable Rx boost mode in the SX126x */
        sys_err = sx126x_cfg_rx_boosted(&drv_ctx, TRUE);
        if (sys_err != SX126X_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to configure Rx boost mode. Error %d", (int32_t)sys_err);
            err = RADIO_ERROR_IO_ERROR;
            break;
        }

        /** By default Rx Boost setting is not part of the RAM retention domain - apply the workaround:
         * • Set register 0x029F to 0x01
         * • Set register 0x02A0 to 0x08
         * • Set register 0x02A1 to 0xAC
         */
        uint8_t rx_boot_retention_cfg[] = {
            0x01u,
            (uint8_t)(SX126X_REG_RXGAIN >> 8),
            (uint8_t)(SX126X_REG_RXGAIN & 0xFFu),
        };

        hal_err = sx126x_write_register(&drv_ctx, 0x029Fu, rx_boot_retention_cfg, sizeof(rx_boot_retention_cfg));
        if (hal_err != SX126X_HAL_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to configure Rx boost mode retention. Error %d", (int32_t)hal_err);
            err = RADIO_ERROR_IO_ERROR;
            break;
        }

        /* Done */
        err = RADIO_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static int32_t radio_sx126x_platform_init(void)
{
    int32_t err = RADIO_ERROR_GENERIC;
    sid_error_t sid_err;
    sx126x_hal_status_t hal_err;

    do
    {
        /* Validate inputs */
        if (NULL == drv_ctx.config->pa_config.pa_cfg_callback)
        {
            SID_PAL_LOG_ERROR("SX126x PA configuration callback must be set");
            err = RADIO_ERROR_INVALID_PARAMS;
            break;
        }

        /* Configure GPIO pins */
        hal_err = sx126x_hal_init_gpio(&drv_ctx);
        if (hal_err != SX126X_HAL_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to initialize GPIO to drive SX126x. HAL error %d", (int32_t)hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Create SPI bus */
        sid_err = drv_ctx.config->bus_factory->create(&drv_ctx.bus_iface, drv_ctx.config->bus_factory->config);
        if (sid_err != SID_ERROR_NONE)
        {
            SX126X_RADIO_LOG_ERROR("Failed to create SPI bus");
            err = RADIO_ERROR_IO_ERROR;
            break;
        }

        /* Done */
        err = RADIO_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static int32_t radio_sx126x_platform_deinit(void)
{
    int32_t err = RADIO_ERROR_GENERIC;
    sx126x_hal_status_t hal_err;
    sid_error_t sid_err;

    SX126X_RADIO_LOG_DEBUG("radio_sx126x_platform_deinit");

    do
    {
        /* Deactivate IRQ pin */
        hal_err = sx126x_hal_disarm_irq(&drv_ctx);
        if (hal_err != SX126X_HAL_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("radio_sx126x_platform_deinit - error in sx126x_hal_disarm_irq(). Error %u", (uint32_t)hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Bring down SPI bus */
        const struct sid_pal_serial_bus_iface * const spi_bus_iface = drv_ctx.bus_iface;
        if (spi_bus_iface != NULL)
        {
            if (NULL == spi_bus_iface->destroy)
            {
                SID_PAL_LOG_WARNING("radio_sx126x_platform_deinit - spi_bus_iface has no destroy() method");
                err = RADIO_ERROR_NOT_SUPPORTED;
                break;
            }

            sid_err = spi_bus_iface->destroy(spi_bus_iface);
            if (sid_err != SID_ERROR_NONE)
            {
                SID_PAL_LOG_ERROR("radio_sx126x_platform_deinit - error in spi_bus_iface->destroy(). Error %d", (int32_t)sid_err);
                err = RADIO_ERROR_IO_ERROR;
                break;
            }
        }
        else
        {
            SID_PAL_LOG_WARNING("radio_sx126x_platform_deinit - SPI bus interface is null, bus deinitialization skipped");
        }

        hal_err = sx126x_hal_deinit_gpio(&drv_ctx);
        if (hal_err != SX126X_HAL_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("radio_sx126x_platform_deinit - error in sx126x_hal_init_gpio(). Error %u", (uint32_t)hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Deinitialization is competed */
        SX126X_RADIO_LOG_DEBUG("radio_sx126x_platform_deinit - done");
        err = RADIO_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline int32_t radio_set_irq_mask(const sx126x_irq_mask_t irq_mask)
{
    int32_t err;
    sx126x_status_t sys_err;

    sid_pal_enter_critical_region();
    sys_err = sx126x_set_dio_irq_params(&drv_ctx, irq_mask, irq_mask, RADIO_IRQ_NONE, RADIO_IRQ_NONE);

    if (sys_err != SX126X_STATUS_OK)
    {
        err = RADIO_ERROR_IO_ERROR;
    }
    else
    {
        drv_ctx.irq_mask = irq_mask;
        err = RADIO_ERROR_NONE;
    }
    sid_pal_exit_critical_region();

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline int32_t radio_set_modem_to_lora_mode(void)
{
    int32_t err;
    sx126x_status_t sys_err;

    do
    {
        /* Set modem mode in the transceiver */
        sys_err = sx126x_set_pkt_type(&drv_ctx, SX126X_PKT_TYPE_LORA);
        if (sys_err != SX126X_STATUS_OK)
        {
            SX126X_RADIO_LOG_ERROR("Failed to set packet type to LoRa");
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        err = radio_set_irq_mask(SX126X_DEFAULT_LORA_IRQ_MASK);
        if (err != RADIO_ERROR_NONE)
        {
            SX126X_RADIO_LOG_ERROR("Failed to set IRQ mask for LoRa mode");
            break;
        }

        /* Done */
        drv_ctx.modem = SID_PAL_RADIO_MODEM_MODE_LORA;
        err = RADIO_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline int32_t radio_set_modem_to_fsk_mode(void)
{
    int32_t err;
    sx126x_status_t sys_err;

    do
    {
        /* Set modem mode in the transceiver */
        sys_err = sx126x_set_pkt_type(&drv_ctx, SX126X_PKT_TYPE_GFSK);
        if (sys_err != SX126X_STATUS_OK)
        {
            SX126X_RADIO_LOG_ERROR("Failed to set packet type to FSK");
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        err = radio_set_irq_mask(SX126X_DEFAULT_FSK_IRQ_MASK);
        if (err != RADIO_ERROR_NONE)
        {
            SX126X_RADIO_LOG_ERROR("Failed to set IRQ mask for FSK mode");
            break;
        }

        /* Done */
        drv_ctx.modem = SID_PAL_RADIO_MODEM_MODE_FSK;
        err = RADIO_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline int32_t radio_clear_irq_status_all(void)
{
    int32_t err;
    sx126x_status_t hal_err;

    hal_err = sx126x_clear_irq_status(&drv_ctx, SX126X_IRQ_ALL);
    if (hal_err != SX126X_STATUS_OK)
    {
        err = RADIO_ERROR_HARDWARE_ERROR;
    }
    else
    {
        err = RADIO_ERROR_NONE;
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline int32_t sx126x_radio_smps_set(const uint8_t level)
{
    int32_t err;
    uint8_t reg_value;
    sx126x_hal_status_t hal_err;

    do
    {
        hal_err = sx126x_read_register(&drv_ctx, SX126X_REG_SMPSC2R, &reg_value, sizeof(reg_value));
        if (hal_err != SX126X_HAL_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to read SubGHz reg. Reg: 0x%04X, HAL_SUBGHZ err: 0x%x", SX126X_REG_SMPSC2R, hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        reg_value &= (~SX126X_SMPS_DRV_MASK);
        reg_value |= level;

        hal_err = sx126x_write_register(&drv_ctx, SX126X_REG_SMPSC2R, &reg_value, sizeof(reg_value));
        if (hal_err != SX126X_HAL_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to write SubGHz reg. Reg: 0x%04X, HAL_SUBGHZ err: 0x%x", SX126X_REG_SMPSC2R, hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        err = RADIO_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

/*!
 * @brief return the calibration band of the given frequency
 * @param [in] frequency in hz
 * @return calibration band for the frequency
 */
SID_STM32_SPEED_OPTIMIZED static inline sx126x_freq_band_t radio_sx126x_get_freq_band(const uint32_t freq_hz)
{
    sx126x_freq_band_t band;

    if (freq_hz > 900000000u)
    {
        band = SX126X_BAND_900M;
    }
    else if (freq_hz > 850000000u)
    {
        band = SX126X_BAND_850M;
    }
    else if (freq_hz > 770000000u)
    {
        band = SX126X_BAND_770M;
    }
    else if (freq_hz > 460000000u)
    {
        band = SX126X_BAND_460M;
    }
    else if (freq_hz > 430000000u)
    {
        band = SX126X_BAND_430M;
    }
    else
    {
        band = SX126X_BAND_INVALID;
    }

    return band;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static void radio_sx126x_on_radio_timeout_event(void * arg, sid_pal_timer_t * originator)
{
    int32_t err;
    sx126x_status_t sys_err;
    sid_pal_radio_events_t radio_event = SID_PAL_RADIO_EVENT_UNKNOWN;
    uint32_t irq_disarmed = FALSE;
    int16_t rssi_now = 0u;
    uint32_t radio_state_on_entry;

    (void)arg;
    (void)originator;

    do
    {
        SX126X_RADIO_LOG_DEBUG("SX126X_SIMULATED_IRQ_TIMEOUT");

        if ((drv_ctx.radio_state != SID_PAL_RADIO_RX) && (drv_ctx.radio_state != SID_PAL_RADIO_TX))
        {
            err = RADIO_ERROR_INVALID_STATE;
            break;
        }

        /* Run in a critical section to avoid interference with radio IRQ processing */
        sid_pal_enter_critical_region();
        {
            sid_error_t sid_err;
            uint8_t irq_pin_state;

            /* Read out current state of the radio IRQ pin */
            sid_err = sid_pal_gpio_read(drv_ctx.config->gpio.radio_irq, &irq_pin_state);
            if (sid_err != SID_ERROR_NONE)
            {
                sid_pal_exit_critical_region();
                err = RADIO_ERROR_IO_ERROR;
                break;
            }

            /* Check if IRQ pin indicates an active IRQ */
            if (irq_pin_state != SX126X_RADIO_IRQ_PIN_INACTIVE_STATE)
            {
                /* There's an active IRQ, terminate processing */
                sid_pal_exit_critical_region();
                err = RADIO_ERROR_INVALID_STATE;
                break;
            }

            /* There's no active IRQ at the moment - temporarily deactivate IRQ pin */
            (void)sx126x_hal_disarm_irq(&drv_ctx);
            irq_disarmed = TRUE;
        }
        /* Done with the critical processing */
        sid_pal_exit_critical_region();

        /* Capture real-time RSSI before putting the radio to Standby */
        if (drv_ctx.cad_exit_mode != SID_PAL_RADIO_CAD_EXIT_MODE_NONE)
        {
            rssi_now = sid_pal_radio_rssi();
        }
        __COMPILER_BARRIER(); /* Ensure RSSI readout takes place before the radio is put to Standby */

        /* Put the radio to Standby state */
        sys_err = sx126x_set_standby(&drv_ctx, SX126X_STANDBY_CFG_XOSC); /* Use STDBY_XOSC to save on oscillator restart time (e.g. when Tx follows right after CS) */
        if (sys_err != SX126X_STATUS_OK)
        {
            drv_ctx.radio_state = SID_PAL_RADIO_UNKNOWN;
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Turn off FEM as we don't need it any longer */
        (void)radio_sx126x_set_fem_mode(SX126X_RADIO_FRONTEND_MODE_OFF);

        radio_state_on_entry = drv_ctx.radio_state;
        drv_ctx.radio_state  = SID_PAL_RADIO_STANDBY_XOSC;
        __COMPILER_BARRIER();

        if (SID_PAL_RADIO_RX == radio_state_on_entry)
        {
            if ((drv_ctx.cad_exit_mode != SID_PAL_RADIO_CAD_EXIT_MODE_NONE) && (SID_PAL_RADIO_MODEM_MODE_FSK == drv_ctx.modem))
            {
                /* Restore the default IRQ mask for FSK after any type of Carrier Sense */
                err = radio_set_irq_mask(SX126X_DEFAULT_FSK_IRQ_MASK);
                if (err != RADIO_ERROR_NONE)
                {
                    SX126X_RADIO_LOG_ERROR("Failed to set IRQ mask for FSK mode");
                    break;
                }
            }

            /* Process Carrier Sense/CAD events */
            if ((SID_PAL_RADIO_CAD_EXIT_MODE_CS_LBT == drv_ctx.cad_exit_mode) && (SID_PAL_RADIO_MODEM_MODE_FSK == drv_ctx.modem))
            {
                drv_ctx.cad_exit_mode = SID_PAL_RADIO_CAD_EXIT_MODE_NONE;

                if (rssi_now >= SX126X_SIDEWALK_FSK_CS_RSSI_THRESHOLD)
                {
                    /* RSSI is above the threshold - radio channel is busy */
                    radio_event = SID_PAL_RADIO_EVENT_CS_DONE;
                    err = RADIO_ERROR_NONE;

                    /* Store the RSSI on the channel */
                    drv_ctx.radio_rx_packet->fsk_rx_packet_status.rssi_sync = (int8_t)rssi_now;
                }
                else
                {
                    /* CS detected no activity - selected radio channel is free */
#  if HALO_ENABLE_DIAGNOSTICS
                    radio_event = SID_PAL_RADIO_EVENT_CS_TIMEOUT;
                    err = RADIO_ERROR_NONE;
#  else
                    /* Carrier Sense for FSK ended with timeout - radio channel is free and we can proceed with Tx */
                    radio_event = SID_PAL_RADIO_EVENT_UNKNOWN;

                    /* Start Tx */
                    err = sid_pal_radio_start_tx(SX126X_CAD_DEFAULT_TX_TIMEOUT);
                    if (err != RADIO_ERROR_NONE)
                    {
                        /* Logs provided by sid_pal_radio_start_tx() */
                        break;
                    }
#  endif /* HALO_ENABLE_DIAGNOSTICS */
                }
            }
            else if (SID_PAL_RADIO_CAD_EXIT_MODE_NONE == drv_ctx.cad_exit_mode)
            {
                /* This was a normal Rx, not CS */
                radio_event = SID_PAL_RADIO_EVENT_RX_TIMEOUT;
                err = RADIO_ERROR_NONE;
            }
            else
            {
                /* Nothing to do here - some sort of CS we're not interested in */
                err = RADIO_ERROR_NONE;
            }
        }
        else /* if (SID_PAL_RADIO_TX == radio_state_on_entry) */
        {
            radio_event = SID_PAL_RADIO_EVENT_TX_TIMEOUT;
            err = RADIO_ERROR_NONE;
        }
    } while (0);

    /* Restore IRQ pin settings */
    if (irq_disarmed != FALSE)
    {
        (void)sx126x_hal_arm_irq(&drv_ctx);
    }

    if ((RADIO_ERROR_NONE == err) && (SID_PAL_RADIO_EVENT_UNKNOWN != radio_event))
    {
#if SID_SDK_CONFIG_RF_TIMINGS_MEASUREMENT
        /* Clear Rx/Tx starting point indication */
        SIDEWALK_RF_TIMINGS_RXTX_START_GPIO_Port->BRR = SIDEWALK_RF_TIMINGS_RXTX_START_Pin;
#endif /* SID_SDK_CONFIG_RF_TIMINGS_MEASUREMENT */

#if SX126X_RADIO_CFG_USE_STATUS_LED
        /* Turn off activity LEDs - any event means activities are finished */
        sx126x_radio_hal_tx_led_off(&drv_ctx);
        sx126x_radio_hal_rx_led_off(&drv_ctx);
#endif /* SX126X_RADIO_CFG_USE_STATUS_LED */

#if (CFG_LPM_LEVEL != 0)
        /* Allow Stop mode after Rx/CS is finished */
        UTIL_LPM_SetStopMode((1u << CFG_LPM_SIDEWALK), UTIL_LPM_ENABLE);
#  if (CFG_LPM_STDBY_SUPPORTED != 0)
        UTIL_LPM_SetOffMode((1u << CFG_LPM_SIDEWALK), UTIL_LPM_ENABLE);
#  endif /* CFG_LPM_STDBY_SUPPORTED */
#endif /* (CFG_LPM_LEVEL != 0) */

        /* Notify the radio about the event - make sure this is the last action after all GPIO handling and LPM management since drv_ctx.report_radio_event() may modify GPIO and LPM states */
        drv_ctx.report_radio_event(radio_event);
    }
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline int32_t radio_sx126x_start_radio_timeout_mon(const uint32_t timeout_us)
{
    int32_t err;

    do
    {
        sid_error_t sid_err;
        struct sid_timespec rx_timeout_ts;

        if ((0u == timeout_us) || (SX126X_RADIO_INFINITE_TIME == timeout_us))
        {
            /* No need to start the countdown */
            err = RADIO_ERROR_NONE;
            break;
        }

        /* Calculate timeout timestamp */
        sid_err = sid_clock_now(SID_CLOCK_SOURCE_UPTIME, &rx_timeout_ts, NULL);
        if (sid_err != SID_ERROR_NONE)
        {
            err = RADIO_ERROR_IO_ERROR;
            break;
        }

        /* Compensate for the timer setup processing delay */
        rx_timeout_ts.tv_nsec += ((timeout_us + SX126X_TUS_TO_US(1u)) * 1000u);
        sid_time_normalize(&rx_timeout_ts);

        sid_err = sid_pal_timer_arm(&drv_ctx.radio_timeout_mon, SID_PAL_TIMER_PRIO_CLASS_PRECISE, &rx_timeout_ts, NULL);
        if (sid_err != SID_ERROR_NONE)
        {
            err = RADIO_ERROR_IO_ERROR;
            break;
        }

        err = RADIO_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline int32_t radio_sx126x_convert_to_db(const int32_t multiplied_db)
{
    int32_t result;

    if (multiplied_db > 0)
    {
        result = (multiplied_db + (SX126X_PA_CFG_DB_MULT / 2)) / SX126X_PA_CFG_DB_MULT;
    }
    else
    {
        result = (multiplied_db - (SX126X_PA_CFG_DB_MULT / 2)) / SX126X_PA_CFG_DB_MULT;
    }

    return result;
}

/* Global function definitions -----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED halo_drv_semtech_ctx_t * sx126x_get_drv_ctx(void)
{
    return &drv_ctx;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t radio_sx126x_compute_tx_power_config(const int8_t power)
{
    int32_t err;

    if (drv_ctx.radio_state != SID_PAL_RADIO_STANDBY_XOSC)
    {
        return RADIO_ERROR_INVALID_STATE;
    }

    SID_PAL_ASSERT(drv_ctx.regional_radio_param != NULL);

#if HALO_ENABLE_DIAGNOSTICS
    if (FALSE == drv_ctx.pa_cfg_override)
#endif
    {
        /* Compute the target Tx power taking into account the antenna gain */
        int32_t requested_pwr = power * SX126X_PA_CFG_DB_MULT; /* Multiply by 100 for more accurate calculations */
        switch (drv_ctx.regional_radio_param->param_region)
        {
            case RADIO_REGION_NA:
                if (drv_ctx.regional_radio_param->ant_dbi > SX126X_ANT_GAIN_CAP_REGION_NA)
                {
                    /* As per the FCC requirements PA gain shall be reduced by the difference between antenna gain and the antenna gain cap value */
                    requested_pwr -= ((int32_t)drv_ctx.regional_radio_param->ant_dbi - SX126X_ANT_GAIN_CAP_REGION_NA);
                }
                break;

            case RADIO_REGION_EU:
                /* EU requirements put the limit on ERP - PA gain shall be reduced by antenna gain */
                requested_pwr -= (int32_t)drv_ctx.regional_radio_param->ant_dbi;
                break;

            case RADIO_REGION_NONE:
            default:
                /* Apply PA gain as is */
                break;
        }

        /* Fill-in drv_ctx.pa_cfg with invalid values */
        SID_STM32_UTIL_fast_memset(&drv_ctx.pa_cfg, SX126X_PA_CFG_PARAM_INVALID, sizeof(drv_ctx.pa_cfg));

        /* Call app callback to populate PA config - this allows the app to override the default values */
        err = drv_ctx.config->pa_config.pa_cfg_callback(requested_pwr, drv_ctx.regional_radio_param, &drv_ctx.pa_cfg);
        if (err != RADIO_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Unable to select SX126x PA settings. Error %d", err);
            return RADIO_ERROR_INVALID_PARAMS;
        }

        /* Check if the callback applied custom PA config */
        if ((SX126X_PA_CFG_PARAM_INVALID == drv_ctx.pa_cfg.pa_duty_cycle)
         || (SX126X_PA_CFG_PARAM_INVALID == drv_ctx.pa_cfg.hp_max)
         || (SX126X_PA_CFG_PARAM_INVALID == drv_ctx.pa_cfg.pa_lut)
         || (SX126X_PA_CFG_PARAM_INVALID == (uint8_t)drv_ctx.pa_cfg.tx_power_reg)
         || (((SX126X_PA_CFG_PARAM_INVALID << 8) | SX126X_PA_CFG_PARAM_INVALID) == (uint16_t)drv_ctx.pa_cfg.target_tx_power)
#if SX126X_RADIO_CFG_USE_EXTERNAL_PA
         || (SX126X_PA_CFG_PARAM_INVALID == drv_ctx.pa_cfg.use_ext_pa)
#endif /* SX126X_RADIO_CFG_USE_EXTERNAL_PA */
        )
        {
            /* At least one PA config param was not set - apply default config */
            /* Check if the callback changed the target Tx power */
            if (((SX126X_PA_CFG_PARAM_INVALID << 8) | SX126X_PA_CFG_PARAM_INVALID) == drv_ctx.pa_cfg.target_tx_power)
            {
                drv_ctx.pa_cfg.target_tx_power = (int16_t)requested_pwr;
            }
            else
            {
                requested_pwr = drv_ctx.pa_cfg.target_tx_power;
            }

            /* Define the limits for the built-in PA */
            int32_t builtin_pa_upper_lim;
            if (SEMTECH_ID_SX1262 == drv_ctx.config->id)
            {
                builtin_pa_upper_lim = (SX126X_PA_CFG_HPA_UPPER_LIMIT_DBM * SX126X_PA_CFG_DB_MULT);
            }
            else if (SEMTECH_ID_SX1261 == drv_ctx.config->id)
            {
                builtin_pa_upper_lim = (SX126X_PA_CFG_LPA_UPPER_LIMIT_DBM * SX126X_PA_CFG_DB_MULT);
            }
            else
            {
                return RADIO_ERROR_NOT_SUPPORTED;
            }

#if SX126X_RADIO_CFG_USE_EXTERNAL_PA
            /* Check if need external PA to achieve the desired Tx power */
            if ((requested_pwr + drv_ctx.config->pa_config.tx_bypass_loss) <= builtin_pa_upper_lim)
            {
                /* The requested power level can be achieved with HPA alone, even compensating for Tx bypass loss */
                drv_ctx.pa_cfg.use_ext_pa = FALSE;
                requested_pwr += drv_ctx.config->pa_config.tx_bypass_loss; /* Compensate for bypass loss */
            }
            else
            {
                /* Requested power level cannot be achieved by the built-in PA only, external PA shall be engaged */
                drv_ctx.pa_cfg.use_ext_pa = TRUE;
                requested_pwr -= drv_ctx.config->pa_config.tx_gain_dbi; /* Take into account the gain of the external PA */
            }
#else
            /* Compensate for RF switch losses */
            requested_pwr += drv_ctx.config->pa_config.rf_sw_insertion_loss;
#endif /* SX126X_RADIO_CFG_USE_EXTERNAL_PA */

            /* Saturate by upper limit for built-in PA */
            if (requested_pwr > builtin_pa_upper_lim)
            {
                requested_pwr = builtin_pa_upper_lim;
            }

            /* Go back into physical dB representation as we are about to configure the PA */
            requested_pwr = radio_sx126x_convert_to_db(requested_pwr);

            /* Configure PA parameters based on the selected PA */
            if (SEMTECH_ID_SX1262 == drv_ctx.config->id)
            {
                /* Search for the closest optimized config - configure as per AN5457 */
                if (requested_pwr <= SX126X_PA_CFG_HPA_OPTIMIZATION_14_DBM)
                {
                    /* +14dBm optimization */
                    drv_ctx.pa_cfg.pa_duty_cycle = 0x02u;
                    drv_ctx.pa_cfg.hp_max        = 0x02u;
                    drv_ctx.pa_cfg.pa_lut        = 0x01u;
                    drv_ctx.pa_cfg.tx_power_reg  = requested_pwr;
                }
                else if (requested_pwr <= SX126X_PA_CFG_HPA_OPTIMIZATION_17_DBM)
                {
                    /* +17dBm optimization */
                    drv_ctx.pa_cfg.pa_duty_cycle = 0x02u;
                    drv_ctx.pa_cfg.hp_max        = 0x03u;
                    drv_ctx.pa_cfg.pa_lut        = 0x01u;
                    drv_ctx.pa_cfg.tx_power_reg  = SX126X_PA_CFG_HPA_UPPER_LIMIT_DBM - (SX126X_PA_CFG_HPA_OPTIMIZATION_17_DBM - requested_pwr);
                }
                else if (requested_pwr <= SX126X_PA_CFG_HPA_OPTIMIZATION_20_DBM)
                {
                    /* +20dBm optimization */
                    drv_ctx.pa_cfg.pa_duty_cycle = 0x03u;
                    drv_ctx.pa_cfg.hp_max        = 0x05u;
                    drv_ctx.pa_cfg.pa_lut        = 0x01u;
                    drv_ctx.pa_cfg.tx_power_reg  = SX126X_PA_CFG_HPA_UPPER_LIMIT_DBM - (SX126X_PA_CFG_HPA_OPTIMIZATION_20_DBM - requested_pwr);
                }
                else
                {
                    /* +22dBm optimization */
                    drv_ctx.pa_cfg.pa_duty_cycle = 0x04u;
                    drv_ctx.pa_cfg.hp_max        = 0x07u;
                    drv_ctx.pa_cfg.pa_lut        = 0x01u;
                    drv_ctx.pa_cfg.tx_power_reg  = requested_pwr;
                }

                /* Saturate by lower limit for HPA */
                if (drv_ctx.pa_cfg.tx_power_reg < SX126X_PA_CFG_HPA_LOWER_LIMIT_DBM)
                {
                    drv_ctx.pa_cfg.tx_power_reg = SX126X_PA_CFG_HPA_LOWER_LIMIT_DBM;
                }
            }
            else /* if(SEMTECH_ID_SX1261 == drv_ctx.config->id) */
            {
                /* Saturate by upper limit for LPA */
                if (requested_pwr > SX126X_PA_CFG_LPA_UPPER_LIMIT_DBM)
                {
                    requested_pwr = SX126X_PA_CFG_LPA_UPPER_LIMIT_DBM;
                }

                /* Search for the closest optimized config - configure as per AN5457 */
                if (requested_pwr <= SX126X_PA_CFG_LPA_OPTIMIZATION_10_DBM)
                {
                    /* +10dBm optimization */
                    drv_ctx.pa_cfg.pa_duty_cycle = 0x01u;
                    drv_ctx.pa_cfg.hp_max        = 0x00u;
                    drv_ctx.pa_cfg.pa_lut        = 0x01u;
                    drv_ctx.pa_cfg.tx_power_reg  = 13 - (SX126X_PA_CFG_LPA_OPTIMIZATION_10_DBM - requested_pwr);
                }
                else if (requested_pwr <= SX126X_PA_CFG_LPA_OPTIMIZATION_14_DBM)
                {
                    /* +14dBm optimization */
                    drv_ctx.pa_cfg.pa_duty_cycle = 0x04u;
                    drv_ctx.pa_cfg.hp_max        = 0x00u;
                    drv_ctx.pa_cfg.pa_lut        = 0x01u;
                    drv_ctx.pa_cfg.tx_power_reg  = requested_pwr;
                }
                else
                {
                    /* +15dBm optimization */
                    drv_ctx.pa_cfg.pa_duty_cycle = 0x07u;
                    drv_ctx.pa_cfg.hp_max        = 0x00u;
                    drv_ctx.pa_cfg.pa_lut        = 0x01u;
                    drv_ctx.pa_cfg.tx_power_reg  = SX126X_PA_CFG_LPA_OPTIMIZATION_14_DBM - (SX126X_PA_CFG_LPA_UPPER_LIMIT_DBM - requested_pwr);
                }

                /* Saturate by lower limit for HPA */
                if (drv_ctx.pa_cfg.tx_power_reg < SX126X_PA_CFG_LPA_LOWER_LIMIT_DBM)
                {
                    drv_ctx.pa_cfg.tx_power_reg = SX126X_PA_CFG_LPA_LOWER_LIMIT_DBM;
                }
            }
        }
        else
        {
            /* PA configuration is provided by the user callback - do a few sanity checks and use it */
            if (SEMTECH_ID_SX1262 == drv_ctx.config->id)
            {
                if ((drv_ctx.pa_cfg.tx_power_reg > SX126X_PA_CFG_HPA_UPPER_LIMIT_DBM)
                || (drv_ctx.pa_cfg.tx_power_reg < SX126X_PA_CFG_HPA_LOWER_LIMIT_DBM)
                || (drv_ctx.pa_cfg.pa_duty_cycle > 0x07u)
                || (drv_ctx.pa_cfg.pa_lut != 0x01u))
                {
                    return RADIO_ERROR_INVALID_PARAMS;
                }
            }
            else if (SEMTECH_ID_SX1261 == drv_ctx.config->id)
            {
                if ((drv_ctx.pa_cfg.tx_power_reg > SX126X_PA_CFG_LPA_UPPER_LIMIT_DBM)
                || (drv_ctx.pa_cfg.tx_power_reg < SX126X_PA_CFG_LPA_LOWER_LIMIT_DBM)
                || (drv_ctx.pa_cfg.pa_lut != 0x01u))
                {
                    return RADIO_ERROR_INVALID_PARAMS;
                }
            }
            else
            {
                return RADIO_ERROR_NOT_SUPPORTED;
            }
        }

        /* Check if the callback specified the ramp up time */
        if (SX126X_PA_CFG_PARAM_INVALID == drv_ctx.pa_cfg.ramp_time)
        {
            /* Use the default value */
            drv_ctx.pa_cfg.ramp_time = SX126X_RAMP_40_US;
        }
    }

    return RADIO_ERROR_NONE;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED void sx126x_radio_set_lora_exit_mode(const sid_pal_radio_cad_param_exit_mode_t cad_exit_mode)
{
    drv_ctx.cad_exit_mode = cad_exit_mode;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t radio_sx126x_set_fem_mode(const sx126x_subghz_fe_mode_t frontend_mode)
{
    int32_t     err = RADIO_ERROR_NONE;
    sid_error_t sid_err;
    uint8_t     smps_drive_lvl;

    do
    {
        switch (frontend_mode)
        {
            case SX126X_RADIO_FRONTEND_MODE_OFF:
                smps_drive_lvl = SX126X_SMPS_DRIVE_SETTING_DEFAULT;

#if SX126X_RADIO_CFG_USE_TX_RX_CTRL
                if (drv_ctx.config->gpio.fem_tx_rx_mode != HALO_GPIO_NOT_CONNECTED)
                {
                    sid_err = sid_pal_gpio_write(drv_ctx.config->gpio.fem_tx_rx_mode, 0u); /* Do not provide any voltage to FEM when it is powered off */
                    if (sid_err != SID_ERROR_NONE)
                    {
                        err = RADIO_ERROR_IO_ERROR;
                        break;
                    }
                }
#endif /* SX126X_RADIO_CFG_USE_TX_RX_CTRL */

#if SX126X_RADIO_CFG_USE_EXTERNAL_PA && SX126X_RADIO_CFG_USE_FEM_BYPASS_CTRL
                if (drv_ctx.config->gpio.fem_bypass != HALO_GPIO_NOT_CONNECTED)
                {
                    sid_err = sid_pal_gpio_write(drv_ctx.config->gpio.fem_bypass, 0u); /* Do not provide any voltage to FEM when it is powered off */
                    if (sid_err != SID_ERROR_NONE)
                    {
                        err = RADIO_ERROR_IO_ERROR;
                        break;
                    }
                }
#endif /* SX126X_RADIO_CFG_USE_EXTERNAL_PA && SX126X_RADIO_CFG_USE_FEM_BYPASS_CTRL */

#if SX126X_RADIO_CFG_USE_FEM_PWR_CTRL
                if (drv_ctx.config->gpio.fem_power != HALO_GPIO_NOT_CONNECTED)
                {
                    uint8_t write_val = drv_ctx.config->gpio.fem_power_en_gpio_state != 0u ? 0u : 1u;
                    sid_err = sid_pal_gpio_write(drv_ctx.config->gpio.fem_power, write_val);
                    if (sid_err != SID_ERROR_NONE)
                    {
                        err = RADIO_ERROR_IO_ERROR;
                        break;
                    }
                }
#endif /* SX126X_RADIO_CFG_USE_FEM_PWR_CTRL */
                break;
                /*----------------------------------------------------------------------------*/

            case SX126X_RADIO_FRONTEND_MODE_TX:
                if (SEMTECH_ID_SX1261 == drv_ctx.config->id)
                {
                    /* If SMPS is used we need to increase its output power since LPA draws current from SMPS */
                    smps_drive_lvl = SX126X_SMPS_DRIVE_SETTING_MAX;
                }
                else
                {
                    /* For SX1262 the amplifier is powered directly from Vdd */
                    smps_drive_lvl = SX126X_SMPS_DRIVE_SETTING_DEFAULT;
                }

#if SX126X_RADIO_CFG_USE_TX_RX_CTRL
                if (drv_ctx.config->gpio.fem_tx_rx_mode != HALO_GPIO_NOT_CONNECTED)
                {
                    uint8_t write_val = drv_ctx.config->gpio.fem_tx_mode_sel_gpio_state != 0u ? 1u : 0u;
                    sid_err = sid_pal_gpio_write(drv_ctx.config->gpio.fem_tx_rx_mode, write_val);
                    if (sid_err != SID_ERROR_NONE)
                    {
                        err = RADIO_ERROR_IO_ERROR;
                        break;
                    }
                }
#endif /* SX126X_RADIO_CFG_USE_TX_RX_CTRL */

#if SX126X_RADIO_CFG_USE_EXTERNAL_PA && SX126X_RADIO_CFG_USE_FEM_BYPASS_CTRL
                if (drv_ctx.config->gpio.fem_bypass != HALO_GPIO_NOT_CONNECTED)
                {
                    uint8_t write_val;

                    if (drv_ctx.pa_cfg.use_ext_pa != FALSE)
                    {
                        /* Use external PA */
                        write_val = drv_ctx.config->gpio.fem_bypass_en_gpio_state != 0u ? 0u : 1u;
                    }
                    else
                    {
                        /* Use Tx bypass mode */
                        write_val = drv_ctx.config->gpio.fem_bypass_en_gpio_state != 0u ? 1u : 0u;
                    }

                    sid_err = sid_pal_gpio_write(drv_ctx.config->gpio.fem_bypass, write_val);
                    if (sid_err != SID_ERROR_NONE)
                    {
                        err = RADIO_ERROR_IO_ERROR;
                        break;
                    }
                }
#endif /* SX126X_RADIO_CFG_USE_EXTERNAL_PA && SX126X_RADIO_CFG_USE_FEM_BYPASS_CTRL */

#if SX126X_RADIO_CFG_USE_FEM_PWR_CTRL
                if (drv_ctx.config->gpio.fem_power != HALO_GPIO_NOT_CONNECTED)
                {
                    uint8_t write_val = drv_ctx.config->gpio.fem_power_en_gpio_state != 0u ? 1u : 0u;
                    sid_err = sid_pal_gpio_write(drv_ctx.config->gpio.fem_power, write_val);
                    if (sid_err != SID_ERROR_NONE)
                    {
                        err = RADIO_ERROR_IO_ERROR;
                        break;
                    }
                }
#endif /* SX126X_RADIO_CFG_USE_FEM_PWR_CTRL */
                break;
                /*----------------------------------------------------------------------------*/

            case SX126X_RADIO_FRONTEND_MODE_RX:
                smps_drive_lvl = SX126X_SMPS_DRIVE_SETTING_DEFAULT;

#if SX126X_RADIO_CFG_USE_TX_RX_CTRL
                if (drv_ctx.config->gpio.fem_tx_rx_mode != HALO_GPIO_NOT_CONNECTED)
                {
                    uint8_t write_val = drv_ctx.config->gpio.fem_tx_mode_sel_gpio_state != 0u ? 0u : 1u;
                    sid_err = sid_pal_gpio_write(drv_ctx.config->gpio.fem_tx_rx_mode, write_val);
                    if (sid_err != SID_ERROR_NONE)
                    {
                        err = RADIO_ERROR_IO_ERROR;
                        break;
                    }
                }
#endif /* SX126X_RADIO_CFG_USE_TX_RX_CTRL */

#if SX126X_RADIO_CFG_USE_EXTERNAL_PA && SX126X_RADIO_CFG_USE_FEM_BYPASS_CTRL
                if (drv_ctx.config->gpio.fem_bypass != HALO_GPIO_NOT_CONNECTED)
                {
                    uint8_t write_val = drv_ctx.config->gpio.fem_bypass_en_gpio_state != 0u ? 1u : 0u;
                    sid_err = sid_pal_gpio_write(drv_ctx.config->gpio.fem_bypass, write_val);
                    if (sid_err != SID_ERROR_NONE)
                    {
                        err = RADIO_ERROR_IO_ERROR;
                        break;
                    }
                }
#endif /* SX126X_RADIO_CFG_USE_EXTERNAL_PA && SX126X_RADIO_CFG_USE_FEM_BYPASS_CTRL */

#if SX126X_RADIO_CFG_USE_FEM_PWR_CTRL
                if (drv_ctx.config->gpio.fem_power != HALO_GPIO_NOT_CONNECTED)
                {
                    uint8_t write_val = drv_ctx.config->gpio.fem_power_en_gpio_state != 0u ? 1u : 0u;
                    sid_err = sid_pal_gpio_write(drv_ctx.config->gpio.fem_power, write_val);
                    if (sid_err != SID_ERROR_NONE)
                    {
                        err = RADIO_ERROR_IO_ERROR;
                        break;
                    }
                }
#endif /* SX126X_RADIO_CFG_USE_FEM_PWR_CTRL */
                break;
                /*----------------------------------------------------------------------------*/

            default:
                SID_PAL_LOG_ERROR("radio_sx126x_set_fem_mode invalid params");
                err = RADIO_ERROR_INVALID_PARAMS;
                break;
        }

        /* Check the switch above completed normally */
        if (err != RADIO_ERROR_NONE)
        {
            break;
        }

        /* Adjust SMPS settings if it is used */
        if ((drv_ctx.config->regulator_mode == SX126X_REG_MODE_DCDC) && ((SID_PAL_RADIO_STANDBY == drv_ctx.radio_state) || (SID_PAL_RADIO_STANDBY_XOSC == drv_ctx.radio_state)))
        {
            err = sx126x_radio_smps_set(smps_drive_lvl);
            if (err != RADIO_ERROR_NONE)
            {
                SID_PAL_LOG_ERROR("Failed to set SMPS current drive to 0x%02X. Error %d", smps_drive_lvl, err);
                break;
            }
        }

        err = RADIO_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED void sx126x_radio_set_device_config(const sx126x_radio_device_config_t * const cfg)
{
    drv_ctx.config = cfg;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED uint8_t sid_pal_radio_get_status(void)
{
    uint8_t state_to_report;

    sid_pal_enter_critical_region();

    if (drv_ctx.sid_busy_indication != FALSE)
    {
        /* Tell Sidewalk stack the radio is not available now */
        state_to_report = SID_PAL_RADIO_BUSY;
    }
    else
    {
        switch (drv_ctx.radio_state)
        {
            case SID_PAL_RADIO_STANDBY_XOSC:
                /* Sidewalk stack does not recognize SID_PAL_RADIO_STANDBY_XOSC, report SID_PAL_RADIO_STANDBY instead */
                state_to_report = SID_PAL_RADIO_STANDBY;
                break;

            case SID_PAL_RADIO_STANDBY:
                /**
                 * Sidewalk stack assumes the radio to be fully prepared for Tx/Rx when SID_PAL_RADIO_STANDBY is reported. This translates into
                 * the actual state of STDBY_XOSC. However, in some case (e.g. SPI error) this driver may leave the radio in STDBY_RC. To address
                 * such case we report SID_PAL_RADIO_SLEEP here - this will force Sidewalk stack to call sid_pal_radio_standby() and the radio will
                 * reach STDBY_XOSC state in the end.
                 */
                state_to_report = SID_PAL_RADIO_SLEEP;
                break;

            default:
                /* Forward radio state as is */
                state_to_report = drv_ctx.radio_state;
                break;
        }
    }

    sid_pal_exit_critical_region();

    return state_to_report;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_pal_radio_modem_mode_t sid_pal_radio_get_modem_mode(void)
{
    return drv_ctx.modem;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_set_modem_mode(sid_pal_radio_modem_mode_t mode)
{
    int32_t err;
    const char * mode_str;
    const sid_pal_radio_modem_mode_t mode_on_entry = drv_ctx.modem;

#  if SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION
    if (RADIO_SX126X_LBM_BRIDGE_STATE_EXCLUSIVE_MODE == drv_ctx.lbm.bridge_state)
    {
        /* Don't allow Sidewalk operations */
        return RADIO_ERROR_BUSY;
    }
#  endif /* SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION */

    switch(mode)
    {
        case SID_PAL_RADIO_MODEM_MODE_LORA:
            err = radio_set_modem_to_lora_mode();
            mode_str = "LoRa";
            break;

        case SID_PAL_RADIO_MODEM_MODE_FSK:
            err = radio_set_modem_to_fsk_mode();
            mode_str = "FSK";
            break;

        default:
            err = RADIO_ERROR_NOT_SUPPORTED;
            mode_str = "Undefined";
            break;
    }


    if ((RADIO_ERROR_NONE == err) & (drv_ctx.modem != mode_on_entry))
    {
        SID_PAL_LOG_INFO("SX126x modem set to %s mode", mode_str);
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_irq_process(void)
{
    int32_t                  err         = RADIO_ERROR_GENERIC;
    sx126x_hal_status_t      hal_err;
    sx126x_status_t          sys_err;
    sid_pal_radio_events_t   radio_event = SID_PAL_RADIO_EVENT_UNKNOWN;
    sx126x_irq_mask_t        reported_irqs;
    sx126x_irq_mask_t        remaining_irqs = SX126X_IRQ_NONE;

    do
    {
        if (SID_PAL_RADIO_SLEEP == drv_ctx.radio_state)
        {
            /* This may happen if radio IRQ was pre-emptied by timeout monitoring software timer */
            SX126X_RADIO_LOG_DEBUG("Radio IRQ detected while SX126x is in Sleep - ignored");
            err = RADIO_ERROR_INVALID_STATE;
            break;
        }

        /* Read out IRQ status and clear IRQ flags */
        sys_err = sx126x_get_and_clear_irq_status(&drv_ctx, &reported_irqs);
        if (sys_err != SX126X_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to read out SX126x IRQ status. Error %d", (int32_t)sys_err);
            err =  RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Store reported IRQs for diagnostic purposes */
        /* SX126x sets IRQ status flags whenever the related event occurs, regardless of the IRQ mask settings.
         * IRQ mask just disables IRQ indication via a GPIO pin. This means we need to clean up the status
         * reported by sx126x_get_and_clear_irq_status() from any irrelevant flags
         */
        reported_irqs = (reported_irqs & drv_ctx.irq_mask);
        if (SX126X_IRQ_NONE == reported_irqs)
        {
            /* All IRQs are masked out */
            err = RADIO_ERROR_NONE;
            break;
        }

        /* Store the list of unhandled IRQs for diagnostic purposes */
        remaining_irqs = reported_irqs;
        /*----------------------------------------------------------------------------*/

        /* Handle Tx Done IRQ --------------------------------------------------------*/
        if ((reported_irqs & SX126X_IRQ_TX_DONE) != SX126X_IRQ_NONE)
        {
            SX126X_RADIO_LOG_DEBUG("SX126X_IRQ_TX_DONE");

            /* Radio automatically enters Standby state after Tx/Rx completion or error (e.g. timeout) */
            drv_ctx.radio_state = SID_PAL_RADIO_STANDBY_XOSC;
            /* Turn off FEM as we don't need it any longer */
            (void)radio_sx126x_set_fem_mode(SX126X_RADIO_FRONTEND_MODE_OFF);

            /* Tx Done IRQ is processed successfully */
            remaining_irqs &= ~SX126X_IRQ_TX_DONE;
            radio_event = SID_PAL_RADIO_EVENT_TX_DONE;
            err = RADIO_ERROR_NONE;
            break;
        }
        /*----------------------------------------------------------------------------*/

        /* Sync Word Valid IRQ -------------------------------------------------------*/
        if ((reported_irqs & SX126X_IRQ_SYNC_WORD_VALID) != SX126X_IRQ_NONE)
        {
            SX126X_RADIO_LOG_DEBUG("SX126X_IRQ_SYNC_WORD_VALID");

            const uint32_t rx_done = (reported_irqs & SX126X_IRQ_RX_DONE) != SX126X_IRQ_NONE ? TRUE : FALSE;

            if (SID_PAL_RADIO_MODEM_MODE_FSK == drv_ctx.modem)
            {
                err = radio_fsk_process_sync_word_detected(&drv_ctx, rx_done);
                if (err != RADIO_ERROR_NONE)
                {
                    /* Logs provided by radio_fsk_process_sync_word_detected() */
                    radio_event = SID_PAL_RADIO_EVENT_RX_TIMEOUT;
                    break;
                }
            }
            else
            {
                SX126X_RADIO_LOG_ERROR("Sync Word Valid IRQ is expected only for FSK mode, but modem is in %u mode", (uint32_t)drv_ctx.modem);
                err = RADIO_ERROR_INVALID_STATE;
                break;
            }

            /* Sync Word Valid IRQ is processed successfully */
            remaining_irqs &= ~SX126X_IRQ_SYNC_WORD_VALID;

            /* Terminate IRQ processing if no more IRQs are indicated, otherwise proceed (e.g. with Preamble Detected, Rx Done, etc.) */
            if (SX126X_IRQ_NONE == remaining_irqs)
            {
                break;
            }
        }
        /*----------------------------------------------------------------------------*/

        /* Preamble Detect IRQ -------------------------------------------------------*/
        if ((reported_irqs & SX126X_IRQ_PREAMBLE_DETECTED) != SX126X_IRQ_NONE)
        {
            SX126X_RADIO_LOG_DEBUG("SX126X_IRQ_PREAMBLE_DETECTED");

#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_2
            if (SID_PAL_RADIO_MODEM_MODE_FSK == drv_ctx.modem)
            {
                /* Restore the default IRQ mask for FSK */
                if (drv_ctx.cad_exit_mode != SID_PAL_RADIO_CAD_EXIT_MODE_NONE)
                {
                    err = radio_set_irq_mask(SX126X_DEFAULT_FSK_IRQ_MASK);
                    if (err != RADIO_ERROR_NONE)
                    {
                        SX126X_RADIO_LOG_ERROR("Failed to set IRQ mask for FSK mode");
                        break;
                    }
                }

                if (SID_PAL_RADIO_CAD_EXIT_MODE_CS_LBT == drv_ctx.cad_exit_mode)
                {
                    /* Preamble detected in FSK pre-Tx Carrier Sense - channel is busy */
                    drv_ctx.cad_exit_mode = SID_PAL_RADIO_CAD_EXIT_MODE_NONE;

                    /* Store the RSSI on the channel */
                    drv_ctx.radio_rx_packet->fsk_rx_packet_status.rssi_sync = (int8_t)sid_pal_radio_rssi();

                    /* Abort CS */
                    err = sid_pal_radio_standby();
                    if (err != RADIO_ERROR_NONE)
                    {
                        SID_PAL_LOG_ERROR("Failed to abort ongoing carrier sense. Error %d", err);
                        break;
                    }

                    radio_event = SID_PAL_RADIO_EVENT_CS_DONE;
                    err = RADIO_ERROR_NONE;
                }
                else
                {
                    /* Nothing to do - e.g. preamble detected during the normal Rx process */
                    err = RADIO_ERROR_NONE;
                }
            }
            else
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 */
            {
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
                /* This IRQ is used to stop timeout timer only */
                err = RADIO_ERROR_NONE;
#else
                SX126X_RADIO_LOG_ERROR("Preamble Detect IRQ is expected only for FSK mode, but modem is in %u mode", (uint32_t)drv_ctx.modem);
                err = RADIO_ERROR_INVALID_STATE;
                break;
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 */
            }

            /* Preamble Detect IRQ is processed successfully */
            remaining_irqs &= ~SX126X_IRQ_PREAMBLE_DETECTED;

            /* Terminate IRQ processing if no more IRQs are indicated, otherwise proceed (e.g. with Rx Done, etc.) */
            if (SX126X_IRQ_NONE == remaining_irqs)
            {
                break;
            }
        }
        /*----------------------------------------------------------------------------*/

        /* CAD Done IRQ --------------------------------------------------------------*/
        if ((reported_irqs & SX126X_IRQ_CAD_DONE) != SX126X_IRQ_NONE)
        {
            SX126X_RADIO_LOG_DEBUG("SX126X_IRQ_CAD_DONE");

            if (SID_PAL_RADIO_MODEM_MODE_LORA == drv_ctx.modem)
            {
#if HALO_ENABLE_DIAGNOSTICS
                radio_event = ((reported_irqs & SX126X_IRQ_CAD_DETECTED) != SX126X_IRQ_NONE) ? SID_PAL_RADIO_EVENT_CAD_DONE : SID_PAL_RADIO_EVENT_CAD_TIMEOUT;

                if ((SID_PAL_RADIO_CAD_EXIT_MODE_CS_LBT == drv_ctx.cad_exit_mode) && (SID_PAL_RADIO_EVENT_CAD_TIMEOUT == radio_event))
                {
                    /* Radio starts Tx automatically in CAD-LBT mode */
                    radio_event = SID_PAL_RADIO_EVENT_UNKNOWN;
                    drv_ctx.radio_state = SID_PAL_RADIO_TX;

                }
                if ((SID_PAL_RADIO_CAD_EXIT_MODE_CS_RX == drv_ctx.cad_exit_mode) && (SID_PAL_RADIO_EVENT_CAD_DONE == radio_event))
                {
                    /* Radio starts Rx automatically in CAD-Rx mode */
                    radio_event = SID_PAL_RADIO_EVENT_UNKNOWN;
                    drv_ctx.radio_state = SID_PAL_RADIO_RX;
                }
                else
                {
                    /* For the other modes report CAD Done/ CAD Timeout event */
                }

                drv_ctx.cad_exit_mode = SID_PAL_RADIO_CAD_EXIT_MODE_NONE;

                /* Restore default LoRa IRQ mask */
                err = radio_set_irq_mask(SX126X_DEFAULT_LORA_IRQ_MASK);
                if (err != RADIO_ERROR_NONE)
                {
                    SX126X_RADIO_LOG_ERROR("Failed to restore default IRQ mask for LoRa");
                }
#endif /* HALO_ENABLE_DIAGNOSTICS */
                err = RADIO_ERROR_NONE;
            }
            else
            {
                SID_PAL_LOG_ERROR("CAD Done IRQ is expected only for LoRa mode, but modem is in %u mode", (uint32_t)drv_ctx.modem);
                err = RADIO_ERROR_INVALID_STATE;
                break;
            }

            /* CAD Done IRQ is processed successfully */
            remaining_irqs &= ~(SX126X_IRQ_CAD_DONE | SX126X_IRQ_CAD_DETECTED);
            break;
        }
        /*----------------------------------------------------------------------------*/

        /* CAD Detected IRQ ----------------------------------------------------------*/
        if (((reported_irqs & SX126X_IRQ_CAD_DETECTED) != SX126X_IRQ_NONE) && ((reported_irqs & SX126X_IRQ_CAD_DONE) == SX126X_IRQ_NONE))
        {
            SX126X_RADIO_LOG_DEBUG("SX126X_IRQ_CAD_DETECTED");

            /**
             * SX126x may sporadically report orphaned SX126X_IRQ_CAD_DETECTED in LoRa Rx mode even when CAD was not initiated
             * and without accompanying SX126X_IRQ_CAD_DONE IRQ , which theoretically should never happen. As a workaround,
             * just clear it and proceed as it has no effect on the Rx operation.
             */
            remaining_irqs &= ~(SX126X_IRQ_CAD_DETECTED);
        }
        /*----------------------------------------------------------------------------*/

        /* CRC Error IRQ -------------------------------------------------------------*/
        if ((reported_irqs & SX126X_IRQ_CRC_ERROR) != SX126X_IRQ_NONE)
        {
            SX126X_RADIO_LOG_DEBUG("SX126X_IRQ_CRC_ERROR");
            err = RADIO_ERROR_NONE;
            radio_event = SID_PAL_RADIO_EVENT_RX_ERROR;
            remaining_irqs &= ~(SX126X_IRQ_CRC_ERROR | SX126X_IRQ_RX_DONE);
            break;
        }
        /*----------------------------------------------------------------------------*/

        /* Header Error IRQ ----------------------------------------------------------*/
        if ((reported_irqs & SX126X_IRQ_HEADER_ERROR) != SX126X_IRQ_NONE)
        {
            SX126X_RADIO_LOG_DEBUG("SX126X_IRQ_HEADER_ERROR");
            err = RADIO_ERROR_NONE;
            radio_event = SID_PAL_RADIO_EVENT_HEADER_ERROR;
            remaining_irqs &= ~SX126X_IRQ_HEADER_ERROR;
            break;
        }
        /*----------------------------------------------------------------------------*/

        /* Rx Done IRQ ---------------------------------------------------------------*/
        if ((reported_irqs & SX126X_IRQ_RX_DONE) != SX126X_IRQ_NONE)
        {
            /* There's no need to check for CRC and other errors here since all error IRQs are handled before we get here */
            SX126X_RADIO_LOG_DEBUG("SX126X_IRQ_RX_DONE");

            /* Radio automatically enters Standby state after Tx/Rx completion or error (e.g. timeout) */
            drv_ctx.radio_state = SID_PAL_RADIO_STANDBY_XOSC;
            /* Turn off FEM as we don't need it any longer */
            (void)radio_sx126x_set_fem_mode(SX126X_RADIO_FRONTEND_MODE_OFF);

            /* Process FSK first as it's more time sensitive */
            if (SID_PAL_RADIO_MODEM_MODE_FSK == drv_ctx.modem)
            {
                radio_fsk_rx_done_status_t fsk_rx_done_status;

                err = radio_fsk_process_rx_done(&drv_ctx, &fsk_rx_done_status);
                if (err != RADIO_ERROR_NONE)
                {
                    SX126X_RADIO_LOG_ERROR("FSK Rx completed with error. Error: %d, rx status: %u", err, (uint32_t)fsk_rx_done_status);
                    radio_event = SID_PAL_RADIO_EVENT_RX_ERROR;
                    err = RADIO_ERROR_NONE;
                }
                else
                {
                    radio_event = SID_PAL_RADIO_EVENT_RX_DONE;
                }
            }
            else if (SID_PAL_RADIO_MODEM_MODE_LORA == drv_ctx.modem)
            {
                err = radio_lora_process_rx_done(&drv_ctx);
                if (err != RADIO_ERROR_NONE)
                {
                    SX126X_RADIO_LOG_ERROR("Lora Rx completed with error. Error: %d", err);
                    radio_event = SID_PAL_RADIO_EVENT_RX_ERROR;
                    err = RADIO_ERROR_NONE;
                }
                else
                {
                    radio_event = SID_PAL_RADIO_EVENT_RX_DONE;
                }
            }
            else
            {
                SID_PAL_LOG_ERROR("Received SubGHz Rx Done IRQ, but active modem mode (%u) is invalid", (uint32_t)drv_ctx.modem);
                err = RADIO_ERROR_INVALID_STATE;
                break;
            }

            /* Rx Done IRQ is processed successfully */
            remaining_irqs &= ~SX126X_IRQ_RX_DONE;
            break;
        }

        /* If we've got here it means none of the above blocks took care of the IRQ. Not an error, but... */
        SID_PAL_LOG_WARNING("Unexpected SubGHz IRQ detected: 0x%04X. No handlers assigned to it. All reported: 0x%04X", remaining_irqs, reported_irqs);
        err = RADIO_ERROR_NONE;
    } while (0);

    /* Report the event to the Sidewalk stack */
    if (RADIO_ERROR_NONE == err)
    {
        if (remaining_irqs != SX126X_IRQ_NONE)
        {
            SID_PAL_LOG_WARNING("Unhandled SubGHz IRQs identified. Reported: 0x%04X, unhandled: 0x%04X", reported_irqs, remaining_irqs);
        }

        /* Radio is transitioned to Standby state for the events below */
        if ((SID_PAL_RADIO_EVENT_RX_TIMEOUT == radio_event) || (SID_PAL_RADIO_EVENT_RX_DONE == radio_event) || (SID_PAL_RADIO_EVENT_TX_DONE == radio_event) || (SID_PAL_RADIO_EVENT_TX_TIMEOUT == radio_event)
          || (SID_PAL_RADIO_EVENT_RX_ERROR == radio_event) || (SID_PAL_RADIO_EVENT_HEADER_ERROR == radio_event) || (SID_PAL_RADIO_EVENT_CS_DONE == radio_event) || (SID_PAL_RADIO_EVENT_CAD_DONE == radio_event)
          || (SID_PAL_RADIO_EVENT_CAD_TIMEOUT == radio_event))
        {
#if (CFG_LPM_LEVEL != 0)
            /* Allow Stop mode after Rx/Tx is finished */
            UTIL_LPM_SetStopMode((1u << CFG_LPM_SIDEWALK), UTIL_LPM_ENABLE);
#  if (CFG_LPM_STDBY_SUPPORTED != 0)
            UTIL_LPM_SetOffMode((1u << CFG_LPM_SIDEWALK), UTIL_LPM_ENABLE);
#  endif /* CFG_LPM_STDBY_SUPPORTED */
#endif /* (CFG_LPM_LEVEL != 0) */
        }

        if (SID_PAL_RADIO_EVENT_UNKNOWN != radio_event)
        {
#if SID_SDK_CONFIG_RF_TIMINGS_MEASUREMENT
            /* Clear Rx/Tx starting point indication */
            SIDEWALK_RF_TIMINGS_RXTX_START_GPIO_Port->BRR = SIDEWALK_RF_TIMINGS_RXTX_START_Pin;
#endif /* SID_SDK_CONFIG_RF_TIMINGS_MEASUREMENT */

#if SX126X_RADIO_CFG_USE_STATUS_LED
            /* Turn off activity LEDs - any event means activities are finished */
            sx126x_radio_hal_tx_led_off(&drv_ctx);
            sx126x_radio_hal_rx_led_off(&drv_ctx);
#endif /* SX126X_RADIO_CFG_USE_STATUS_LED */

            /* Notify the radio about the event - make sure this is the last action after all GPIO handling and LPM management since drv_ctx.report_radio_event() may modify GPIO and LPM states */
            drv_ctx.report_radio_event(radio_event);
        }
    }

    /* Re-enable radio IRQ detection if the radio is not in Sleep state - this check is mandatory because drv_ctx.report_radio_event() may change the state of the radio */
    if (drv_ctx.radio_state != SID_PAL_RADIO_SLEEP)
    {
        hal_err = sx126x_hal_arm_irq(&drv_ctx);
        if (hal_err != SX126X_HAL_STATUS_OK)
        {
            /* Logs are provided by the sx126x_hal_arm_irq() */
            err = RADIO_ERROR_HARDWARE_ERROR;
        }
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t set_radio_sx126x_trim_cap_val(const uint16_t trim)
{
    int32_t err = RADIO_ERROR_NONE;
    sx126x_status_t sys_err;
    const uint8_t xta = (uint8_t)(trim >> 8);
    const uint8_t xtb = (uint8_t)(trim & 0xFFu);

    do
    {
#if SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION
        if (RADIO_SX126X_LBM_BRIDGE_STATE_EXCLUSIVE_MODE == drv_ctx.lbm.bridge_state)
        {
            /* Don't allow Sidewalk operations */
            return RADIO_ERROR_BUSY;
        }
#endif /* SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION */

        /* Trim capacitors shall be set in STDBY_XOSC state because transitions into STDBY_XOSC overwrite the settings */
        if (drv_ctx.radio_state != SID_PAL_RADIO_STANDBY_XOSC)
        {
            err = RADIO_ERROR_INVALID_STATE;
            break;
        }

        /* Trim value is forced by SX126x in TCXO mode */
        if (drv_ctx.config->tcxo_config.ctrl != SX126X_TCXO_CTRL_NONE)
        {
            err = RADIO_ERROR_NONE;
            break;
        }

        sys_err = sx126x_set_trimming_capacitor_values(&drv_ctx, xta, xtb);
        if (sys_err != SX126X_STATUS_OK)
        {
            err = RADIO_ERROR_IO_ERROR;
            break;
        }

        drv_ctx.trim = trim;
        err = RADIO_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED uint16_t get_radio_sx126x_trim_cap_val(void)
{
    return drv_ctx.trim;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_set_frequency(uint32_t freq)
{
    int32_t err = RADIO_ERROR_NONE;
    sx126x_status_t sys_err;
    sx126x_hal_status_t hal_err;

    do
    {
#if SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION
        if (RADIO_SX126X_LBM_BRIDGE_STATE_EXCLUSIVE_MODE == drv_ctx.lbm.bridge_state)
        {
            /* Don't allow Sidewalk operations */
            err = RADIO_ERROR_BUSY;
            break;
        }
#endif /* SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION */

       if (drv_ctx.radio_state != SID_PAL_RADIO_STANDBY_XOSC)
       {
           err =  RADIO_ERROR_INVALID_STATE;
           break;
       }

       const sx126x_freq_band_t freq_band = radio_sx126x_get_freq_band(freq);

       if (SX126X_BAND_INVALID == freq_band)
       {
            err = RADIO_ERROR_INVALID_PARAMS;
            break;
       }

       /* Image calibration is required if frequency band has changed */
        if (drv_ctx.radio_freq_band != freq_band)
        {
            sys_err = sx126x_cal_img(&drv_ctx, cal_img_bands[(uint32_t)freq_band - 1u].freq_border_low, cal_img_bands[(uint32_t)freq_band - 1u].freq_border_high);
            if (sys_err != SX126X_STATUS_OK)
            {
                err = RADIO_ERROR_HARDWARE_ERROR;
                break;
            }

            SID_PAL_LOG_INFO("SubGHz frequency band set to %s", SX126X_RADIO_GET_FREQ_BAND_STR(freq_band));
        }

#if RADIO_SX126X_TXPWR_WORKAROUND
       if (SX126X_BAND_900M == freq_band)
       {
            uint8_t reg_val;

            if (freq <= SX1262_BAND_EDGE_LIMIT_FREQ)
            {
                reg_val = SX1262_REG_VAL_FREQ_LOW;
            }
            else
            {
                reg_val = SX1262_REG_VAL_FREQ_HIGH;
            }

            hal_err = sx126x_write_register(&drv_ctx, SX126X_REG_LR_CFG_FREQ, &reg_val, sizeof(reg_val));
            if (hal_err != SX126X_HAL_STATUS_OK)
            {
                err = RADIO_ERROR_IO_ERROR;
                break;
            }
       }
#endif /* RADIO_SX126X_TXPWR_WORKAROUND */

       sys_err = sx126x_set_rf_freq(&drv_ctx, freq);
       if (sys_err != SX126X_STATUS_OK)
       {
           err = RADIO_ERROR_HARDWARE_ERROR;
           break;
       }

       /* Update context */
        drv_ctx.radio_freq_hz   = freq;
        drv_ctx.radio_freq_band = freq_band;

        err = RADIO_ERROR_NONE;
    } while(0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_get_max_tx_power(sid_pal_radio_data_rate_t data_rate, int8_t * tx_power)
{
    int32_t err;

    SID_PAL_ASSERT(drv_ctx.regional_radio_param != NULL);

    do
    {
        if ((data_rate <= SID_PAL_RADIO_DATA_RATE_INVALID) || (data_rate > SID_PAL_RADIO_DATA_RATE_MAX_NUM))
        {
            err = RADIO_ERROR_INVALID_PARAMS;
            break;
        }

        *tx_power = drv_ctx.regional_radio_param->max_tx_power[data_rate - 1];
        err = RADIO_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_set_region(sid_pal_radio_region_code_t region)
{
    int32_t err = RADIO_ERROR_GENERIC;

    do
    {
        if ((region <= SID_PAL_RADIO_RC_NONE) || (region >= SID_PAL_RADIO_RC_MAX))
        {
            err = RADIO_ERROR_INVALID_PARAMS;
            break;
        }

        /* Search for region in param table */
        err = RADIO_ERROR_NOT_SUPPORTED;
        for (uint32_t i = 0u; i < drv_ctx.config->regional_config.reg_param_table_size; i++)
        {
            if (drv_ctx.config->regional_config.reg_param_table[i].param_region == region)
            {
                /* Found */
                drv_ctx.regional_radio_param = &drv_ctx.config->regional_config.reg_param_table[i];
                err = RADIO_ERROR_NONE;
                break;
            }
        }
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

#if HALO_ENABLE_DIAGNOSTICS
SID_STM32_SPEED_OPTIMIZED int32_t semtech_radio_set_sx126x_pa_config(semtech_radio_pa_cfg_t * cfg)
{
    int32_t err;

    do
    {
        if (NULL == cfg)
        {
            err = RADIO_ERROR_INVALID_PARAMS;
            break;
        }

        radio_sx126x_pa_dynamic_cfg_t cur_cfg;

        cur_cfg.pa_duty_cycle   = cfg->pa_duty_cycle;
        cur_cfg.hp_max          = cfg->hp_max;
        cur_cfg.pa_lut          = cfg->pa_lut;
        cur_cfg.tx_power_reg    = cfg->tx_power;
        cur_cfg.target_tx_power = cfg->tx_power;
        cur_cfg.ramp_time       = cfg->ramp_time;
#if SX126X_RADIO_CFG_USE_EXTERNAL_PA
        cur_cfg.use_ext_pa      = cfg->enable_ext_pa;
#endif /* SX126X_RADIO_CFG_USE_EXTERNAL_PA */

        drv_ctx.pa_cfg          = cur_cfg;
        drv_ctx.pa_cfg_override = TRUE;
        err = RADIO_ERROR_NONE;
    } while (0);

    return err;
}
#endif /* HALO_ENABLE_DIAGNOSTICS */

/*----------------------------------------------------------------------------*/

#if HALO_ENABLE_DIAGNOSTICS
SID_STM32_SPEED_OPTIMIZED int32_t get_radio_sx126x_pa_config(radio_sx126x_pa_dynamic_cfg_t * const cfg)
{
    int32_t err;

    do
    {
        if (NULL == cfg)
        {
            err = RADIO_ERROR_INVALID_PARAMS;
            break;
        }

        *cfg = drv_ctx.pa_cfg;
        err = RADIO_ERROR_NONE;
    } while (0);

    return err;
}
#endif /* HALO_ENABLE_DIAGNOSTICS */

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_set_tx_power(int8_t power)
{
    int32_t                err = RADIO_ERROR_GENERIC;
    sx126x_hal_status_t    hal_err;
    sx126x_status_t        sys_err;
    sx126x_pa_cfg_params_t pa_cfg_params;
    uint8_t                ocp_reg;

   SX126X_RADIO_LOG_DEBUG("sid_pal_radio_set_tx_power - set pwr: %s%ddB", power > 0 ? "+" : "", power);

    do
    {
#if SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION
        if (RADIO_SX126X_LBM_BRIDGE_STATE_EXCLUSIVE_MODE == drv_ctx.lbm.bridge_state)
        {
            /* Don't allow Sidewalk operations */
            err = RADIO_ERROR_BUSY;
            break;
        }
#endif /* SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION */

        if (drv_ctx.radio_state != SID_PAL_RADIO_STANDBY_XOSC)
        {
            err = RADIO_ERROR_INVALID_STATE;
            break;
        }

        /* Store the currently used value */
        const radio_sx126x_pa_dynamic_cfg_t previous_pa_cfg = drv_ctx.pa_cfg;

#if HALO_ENABLE_DIAGNOSTICS
        if (FALSE == drv_ctx.pa_cfg_override)
#endif /* HALO_ENABLE_DIAGNOSTICS */
        {
            /* Process config update */
            err = radio_sx126x_compute_tx_power_config(power);
            if (err != RADIO_ERROR_NONE)
            {
                SID_PAL_LOG_ERROR("Failed to set SX126x SubGHz Tx power to %ddB. Error %d", power, err);
                drv_ctx.pa_cfg = previous_pa_cfg; /* restore the state if we've failed to apply the new setting */
                break;
            }
        }

        /* Select transceiver-specific params */
        if (SEMTECH_ID_SX1261 == drv_ctx.config->id)
        {
            ocp_reg = SX1261_DEFAULT_OCP_VAL;
            pa_cfg_params.device_sel = SX126X_PA_CFG_USE_LPA;
        }
        else if (SEMTECH_ID_SX1262 == drv_ctx.config->id)
        {
            ocp_reg = SX1262_DEFAULT_OCP_VAL;
            pa_cfg_params.device_sel = SX126X_PA_CFG_USE_HPA;
        }
        else
        {
            err = RADIO_ERROR_NOT_SUPPORTED;
            break;
        }

        /* Apply PA config */
        pa_cfg_params.pa_duty_cycle = drv_ctx.pa_cfg.pa_duty_cycle;
        pa_cfg_params.hp_max        = drv_ctx.pa_cfg.hp_max;
        pa_cfg_params.pa_lut        = drv_ctx.pa_cfg.pa_lut;

        sys_err = sx126x_set_pa_cfg(&drv_ctx, &pa_cfg_params);
        if (sys_err != SX126X_STATUS_OK)
        {
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Adjust overcurrent protection */
        hal_err = sx126x_write_register(&drv_ctx, SX126X_REG_OCP, &ocp_reg, sizeof(ocp_reg));
        if (hal_err != SX126X_HAL_STATUS_OK)
        {
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

#if RADIO_SX126X_TXPWR_WORKAROUND
        /* Adjust overvoltage protection */
        {
            uint8_t ovp_reg;
            hal_err = sx126x_read_register(&drv_ctx, SX126X_REG_OVP, &ovp_reg, sizeof(ovp_reg));
            if (hal_err != SX126X_HAL_STATUS_OK)
            {
                err = RADIO_ERROR_HARDWARE_ERROR;
                break;
            }

            ovp_reg &= 0xF9u; /* Clear bits 1 and 2 */
            hal_err = sx126x_write_register(&drv_ctx, SX126X_REG_OVP, &ovp_reg, sizeof(ovp_reg));
            if (hal_err != SX126X_HAL_STATUS_OK)
            {
                err = RADIO_ERROR_HARDWARE_ERROR;
                break;
            }
        }
#endif /* RADIO_SX126X_TXPWR_WORKAROUND */

        /* Set Tx power */
        sys_err = sx126x_set_tx_params(&drv_ctx, drv_ctx.pa_cfg.tx_power_reg, drv_ctx.pa_cfg.ramp_time);
        if (sys_err != SX126X_STATUS_OK)
        {
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Done */
        if (previous_pa_cfg.target_tx_power != drv_ctx.pa_cfg.target_tx_power)
        {
            /* Compute integer and fractional parts since tiny_vsnprintf may not support %f printout */
            const uint32_t pwr_int_db   = drv_ctx.pa_cfg.target_tx_power > 0 ? (uint32_t)(drv_ctx.pa_cfg.target_tx_power / 100) : (uint32_t)((-drv_ctx.pa_cfg.target_tx_power) / 100);
            const uint32_t pwr_fract_db = drv_ctx.pa_cfg.target_tx_power > 0 ? (uint32_t)(drv_ctx.pa_cfg.target_tx_power % 100) : (uint32_t)((-drv_ctx.pa_cfg.target_tx_power) % 100);

            SID_PAL_LOG_INFO("SX126x Tx power set to %s%u.%02udB", drv_ctx.pa_cfg.target_tx_power < 0 ? "-" : "+", pwr_int_db, pwr_fract_db); /* tiny_vsnprintf() may not support %+d */
        }
        err = RADIO_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_sleep(uint32_t sleep_us)
{
    int32_t err;
    sx126x_hal_status_t hal_err;
    sx126x_status_t sys_err;

    do
    {
        /* Immediately stop software timer for Tx/Rx/CS/CAD timeout */
        (void)sid_pal_timer_cancel(&drv_ctx.radio_timeout_mon);
        __COMPILER_BARRIER();

#if SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION
        if (RADIO_SX126X_LBM_BRIDGE_STATE_EXCLUSIVE_MODE == drv_ctx.lbm.bridge_state)
        {
            /* Don't allow Sidewalk operations */
            err = RADIO_ERROR_BUSY;
            break;
        }
#endif /* SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION */

        /* No actions required if the radio is already in sleep mode */
        if (drv_ctx.radio_state == SID_PAL_RADIO_SLEEP)
        {
            err = RADIO_ERROR_NONE;
            break;
        }

        /* Ensure the radio is in Standby, otherwise Sleep state cannot be entered */
        err = sid_pal_radio_standby();
        if (err != RADIO_ERROR_NONE)
        {
            break;
        }

        if (UINT32_MAX == sleep_us)
        {
            /* This is a special case, keep the radio in Standby and the stack will call sid_pal_radio_sleep() again with the valid sleep duration */
            err = RADIO_ERROR_NONE;
            break;
        }

        /* Disable IRQ pin on the MCU side to avoid reaction on any spikes while the radio is in sleep and does not actively drive the IRQ line */
        hal_err = sx126x_hal_disarm_irq(&drv_ctx);
        if (hal_err != SX126X_HAL_STATUS_OK)
        {
            SX126X_RADIO_LOG_ERROR("Failed to disarm radio IRQ before sleep entry. Error %d", (int32_t)hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        sys_err = sx126x_set_sleep(&drv_ctx, SX126X_SLEEP_CFG_WARM_START);
        if (sys_err != SX126X_STATUS_OK)
        {
            SX126X_RADIO_LOG_ERROR("Failed to put the radio to sleep. Error %d", (int32_t)sys_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Ensure the radio rises Busy pin as indication of entering Sleep state */
        hal_err = sx126x_hal_wait_busy_indicated(&drv_ctx);
        if (hal_err != SX126X_HAL_STATUS_OK)
        {
            /* Logs provided by sx126x_hal_wait_busy_indicated() */
            drv_ctx.radio_state = SID_PAL_RADIO_UNKNOWN;
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }
        else
        {
            /* drv_ctx.radio_state is set to SID_PAL_RADIO_SLEEP by successful sx126x_set_sleep() invocation */
        }
        __COMPILER_BARRIER();

        /* Update status in the context if sleep entry was successful */
        sid_pal_enter_critical_region();
        drv_ctx.sid_busy_indication = FALSE;
#if SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION
        /* Activate LBM bridge if requested. Always start in exclusive mode, app may adjust this to blanking mode later if necessary */
        if (RADIO_SX126X_LBM_BRIDGE_STATE_ACTIVATION_PENDING == drv_ctx.lbm.bridge_state)
        {
            drv_ctx.lbm.bridge_state = RADIO_SX126X_LBM_BRIDGE_STATE_EXCLUSIVE_MODE;
        }

        /* Release radio lock by Sidewalk stack */
        SID_PAL_ASSERT(drv_ctx.lbm.sidewalk_radio_access_lock != NULL);
        (void)osSemaphoreRelease(drv_ctx.lbm.sidewalk_radio_access_lock);
#endif /* SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION */
        sid_pal_exit_critical_region();

#if (CFG_LPM_LEVEL != 0)
        /* Allow Stop mode after the radio was put into Sleep */
        UTIL_LPM_SetStopMode((1u << CFG_LPM_SIDEWALK), UTIL_LPM_ENABLE);
#  if (CFG_LPM_STDBY_SUPPORTED != 0)
        UTIL_LPM_SetOffMode((1u << CFG_LPM_SIDEWALK), UTIL_LPM_ENABLE);
#  endif /* CFG_LPM_STDBY_SUPPORTED */
#endif /* (CFG_LPM_LEVEL != 0) */

        err = RADIO_ERROR_NONE;
    } while(0);

    if (err != RADIO_ERROR_NONE)
    {
        /* Re-enable IRQ line */
        hal_err = sx126x_hal_arm_irq(&drv_ctx);
        if (hal_err != SX126X_HAL_STATUS_OK)
        {
            SX126X_RADIO_LOG_ERROR("Failed to re-arm radio IRQ on sleep entry failure. Error %d", (int32_t)hal_err);
        }
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_standby(void)
{
    int32_t err = RADIO_ERROR_NONE;
    sx126x_hal_status_t hal_err;
    sx126x_status_t sys_err;

    do
    {
        /* Immediately stop software timer for Tx/Rx/CS/CAD timeout */
        (void)sid_pal_timer_cancel(&drv_ctx.radio_timeout_mon);
        __COMPILER_BARRIER();

#if SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION
        if (RADIO_SX126X_LBM_BRIDGE_STATE_EXCLUSIVE_MODE == drv_ctx.lbm.bridge_state)
        {
            /* Don't allow Sidewalk operations */
            return RADIO_ERROR_BUSY;
        }

        /* Activate radio lock by Sidewalk. We don't care if osSemaphoreAcquire() fails due to semaphore is locked already */
        SID_PAL_ASSERT(drv_ctx.lbm.sidewalk_radio_access_lock != NULL);
        (void)osSemaphoreAcquire(drv_ctx.lbm.sidewalk_radio_access_lock, 0u);
#endif /* SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION */

        if (SID_PAL_RADIO_STANDBY_XOSC == drv_ctx.radio_state)
        {
            /* No need to do anything, already in Standby */
            break;
        }

        /* Put radio into Standby to stop any potentially ongoing operations - wakeup is performed automatically by radio HAL layer */
        sys_err = sx126x_set_standby(&drv_ctx, SX126X_STANDBY_CFG_XOSC); /* Use STDBY_XOSC to save on oscillator restart time (e.g. when the radio performs Tx right after Rx) */
        if (sys_err != SX126X_STATUS_OK)
        {
            drv_ctx.radio_state = SID_PAL_RADIO_UNKNOWN;
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Set radio state in driver context after STDBY_XOSC is reached */
        drv_ctx.radio_state = SID_PAL_RADIO_STANDBY_XOSC;

        /* Disable FEM - this should be done after the radio is in Standby, otherwise this may lead to HW damage (e.g. if the radio was actively transmitting and FEM went off) */
        err = radio_sx126x_set_fem_mode(SX126X_RADIO_FRONTEND_MODE_OFF);
        if (err != RADIO_ERROR_NONE)
        {
            break;
        }

        /* Clear residual IRQs */
        err = radio_clear_irq_status_all();
        if (err != RADIO_ERROR_NONE)
        {
            break;
        }

        /* Ensure IRQs will be cleared before the GPIO IRQ on the MCU side is activated */
        __COMPILER_BARRIER();

        /* Restore XOSC trim as it will be overwritten when entering STDBY_XOSC from STDBY_RC */
        err = set_radio_sx126x_trim_cap_val(drv_ctx.trim);
        if (err != RADIO_ERROR_NONE)
        {
            break;
        }

        /* Re-enable radio IRQ pin on MCU side */
        hal_err = sx126x_hal_arm_irq(&drv_ctx);
        if (hal_err != SX126X_HAL_STATUS_OK)
        {
            err = RADIO_ERROR_IO_ERROR;
            break;
        }

#if SX126X_RADIO_CFG_USE_STATUS_LED
        /* Turn off status LEDs after the radio has reached Standby state */
        sx126x_radio_hal_tx_led_off(&drv_ctx);
        sx126x_radio_hal_rx_led_off(&drv_ctx);
#endif /* SX126X_RADIO_CFG_USE_STATUS_LED */

        err = RADIO_ERROR_NONE;
    } while(0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_set_radio_busy(void)
{
    int32_t err;

    sid_pal_enter_critical_region();
    do
    {
        if (drv_ctx.radio_state != SID_PAL_RADIO_SLEEP)
        {
            SID_PAL_LOG_ERROR("Cannot mark radio as busy because it's currently in an active state (%d)", drv_ctx.radio_state);
            err = RADIO_ERROR_INVALID_STATE;
            break;
        }

        /* Everything is ok, mark radio as busy */
        drv_ctx.sid_busy_indication = TRUE;

        err = RADIO_ERROR_NONE;
    } while (0);
    sid_pal_exit_critical_region();

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_set_tx_payload(const uint8_t * buffer, uint8_t size)
{
    int32_t err;
    sx126x_hal_status_t hal_err;

    do
    {
#if SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION
        if (RADIO_SX126X_LBM_BRIDGE_STATE_EXCLUSIVE_MODE == drv_ctx.lbm.bridge_state)
        {
            /* Don't allow Sidewalk operations */
            err = RADIO_ERROR_BUSY;
            break;
        }
#endif /* SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION */

        /* Validate inputs */
        if ((NULL == buffer) || (0u == size))
        {
            SX126X_RADIO_LOG_ERROR("Set Tx payload aborted - invalid args (buf 0x%08x, sz %u)", (uint32_t)buffer, size);
            err = RADIO_ERROR_INVALID_PARAMS;
            break;
        }

        hal_err = sx126x_write_buffer(&drv_ctx, 0x00u, buffer, size);
        if (hal_err != SX126X_HAL_STATUS_OK)
        {
            SX126X_RADIO_LOG_ERROR("Failed to upload data to SX126x Tx buffer. Error %d", (int32_t)hal_err);
            err = RADIO_ERROR_IO_ERROR;
        }

        /* Done */
        err = RADIO_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_start_tx(uint32_t timeout)
{
    int32_t err;
    sx126x_status_t sys_err;

    do
    {
#if SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION
        if (RADIO_SX126X_LBM_BRIDGE_STATE_EXCLUSIVE_MODE == drv_ctx.lbm.bridge_state)
        {
            /* Don't allow Sidewalk operations */
            SX126X_RADIO_LOG_DEBUG("SX126x rejected Tx start");
            err = RADIO_ERROR_BUSY;
            break;
        }
#endif /* SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION */

        /* Configure FEM for Tx */
        err = radio_sx126x_set_fem_mode(SX126X_RADIO_FRONTEND_MODE_TX);
        if (err != RADIO_ERROR_NONE)
        {
            SX126X_RADIO_LOG_ERROR("Failed to configure FEM for Tx. Error %d", err);
            break;
        }

        /* Update status in the driver */
        drv_ctx.radio_state = SID_PAL_RADIO_TX;
        __COMPILER_BARRIER();

        /* Initiate Tx */
        sys_err = sx126x_set_tx_with_timeout_in_rtc_step(&drv_ctx, SX126X_RADIO_RXTX_NO_TIMEOUT_VAL); /* We use a software timer to avoid the radio falling back to STDBY_RC mode on Tx timeout */
        if (sys_err != SX126X_STATUS_OK)
        {
            SX126X_RADIO_LOG_ERROR("Failed to initiate Tx. Error %d", (int32_t)sys_err);
            drv_ctx.radio_state = SID_PAL_RADIO_UNKNOWN;
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Start the software timer if needed */
        err = radio_sx126x_start_radio_timeout_mon(timeout);
        if (err != RADIO_ERROR_NONE)
        {
            SX126X_RADIO_LOG_ERROR("Failed to start Tx timeout timer. Error %d", err);
            break;
        }

        /* Ensure LEDs will be operated only after the Tx is initiated */
        __COMPILER_BARRIER();

#if (CFG_LPM_LEVEL != 0)
        if (SID_PAL_RADIO_MODEM_MODE_FSK == drv_ctx.modem)
        {
            /* Prohibit Stop mode until Tx is done as we need fast response on Tx Done IRQ */
            UTIL_LPM_SetStopMode((1u << CFG_LPM_SIDEWALK), UTIL_LPM_DISABLE);
#  if (CFG_LPM_STDBY_SUPPORTED != 0)
            UTIL_LPM_SetOffMode((1u << CFG_LPM_SIDEWALK), UTIL_LPM_DISABLE);
#  endif /* CFG_LPM_STDBY_SUPPORTED */
        }
        else
        {
            /* For LoRa link we don't need such rapid response. Additional 200-250us added by the wakeup from Stop LPM won't affect the link performance and we can save some power */
        }
#endif /* (CFG_LPM_LEVEL != 0) */

#if SX126X_RADIO_CFG_USE_STATUS_LED
        /* Drive the status LED after the actual start of Tx to avoid impact on timings */
        sx126x_radio_hal_tx_led_on(&drv_ctx);
#endif /* SX126X_RADIO_CFG_USE_STATUS_LED */
    } while(0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_set_tx_continuous_wave(uint32_t freq, int8_t power)
{
#if HALO_ENABLE_DIAGNOSTICS
    int32_t err;
    sx126x_status_t sys_err;

    do
    {
#if SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION
        if (RADIO_SX126X_LBM_BRIDGE_STATE_EXCLUSIVE_MODE == drv_ctx.lbm.bridge_state)
        {
            /* Don't allow Sidewalk operations */
            err = RADIO_ERROR_BUSY;
            break;
        }
#endif /* SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION */

        /* Ensure the radio is in Standby */
        sys_err = sx126x_set_standby(&drv_ctx, SX126X_STANDBY_CFG_XOSC);
        if (sys_err != SX126X_STATUS_OK)
        {
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Update frequency */
        err = sid_pal_radio_set_frequency(freq);
        if (err != RADIO_ERROR_NONE)
        {
            break;
        }

        /* Set desired Tx power */
        err = sid_pal_radio_set_tx_power(power);
        if (err != RADIO_ERROR_NONE)
        {
            break;
        }

        /* Clear any residual IRQs */
        err = radio_clear_irq_status_all();
        if (err != RADIO_ERROR_NONE)
        {
            SX126X_RADIO_LOG_ERROR("Failed to clear IRQs before starting Tx. Error %d", err);
            break;
        }

        /* Configure FEM for Tx */
        err = radio_sx126x_set_fem_mode(SX126X_RADIO_FRONTEND_MODE_TX);
        if (err != RADIO_ERROR_NONE)
        {
            SX126X_RADIO_LOG_ERROR("Failed to configure FEM for Tx. Error %d", err);
            break;
        }

        /* Update status in the driver */
        drv_ctx.radio_state = SID_PAL_RADIO_TX;
        __COMPILER_BARRIER();

        /* Initiate Tx */
        sys_err = sx126x_set_tx_cw(&drv_ctx);
        if (sys_err != SX126X_STATUS_OK)
        {
            SX126X_RADIO_LOG_ERROR("Failed to initiate CW Tx. Error %d", (int32_t)sys_err);
            drv_ctx.radio_state = SID_PAL_RADIO_UNKNOWN;
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Ensure LEDs will be operated only after the Tx is initiated */
        __COMPILER_BARRIER();

#if SX126X_RADIO_CFG_USE_STATUS_LED
        /* Drive the status LED after the actual start of Tx to avoid impact on timings */
        sx126x_radio_hal_tx_led_on(&drv_ctx);
#endif /* SX126X_RADIO_CFG_USE_STATUS_LED */
    } while(0);

    return err;
#else
    return RADIO_ERROR_NOT_SUPPORTED;
#endif /* HALO_ENABLE_DIAGNOSTICS */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_start_rx(uint32_t timeout)
{
    int32_t err;
    sx126x_status_t sys_err;

    do
    {
        /* Ensure CS/CAD exit mode is clear */
        drv_ctx.cad_exit_mode = SID_PAL_RADIO_CAD_EXIT_MODE_NONE;

#if SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION
        if (RADIO_SX126X_LBM_BRIDGE_STATE_EXCLUSIVE_MODE == drv_ctx.lbm.bridge_state)
        {
            /* Don't allow Sidewalk operations */
            SX126X_RADIO_LOG_DEBUG("SX126x rejected Rx start");
            err = RADIO_ERROR_BUSY;
            break;
        }
#endif /* SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION */

        /* Configure FEM for Rx */
        err = radio_sx126x_set_fem_mode(SX126X_RADIO_FRONTEND_MODE_RX);
        if (err != RADIO_ERROR_NONE)
        {
            SX126X_RADIO_LOG_ERROR("Failed to configure FEM for Rx. Error %d", err);
            break;
        }

#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
        /* Take into account LoRa symbol timeout if it is set */
        if ((SID_PAL_RADIO_MODEM_MODE_LORA == drv_ctx.modem) && (drv_ctx.settings_cache.lora_sync_timeout > 0u))
        {
            const uint32_t lora_sync_timeout_us = sid_pal_radio_get_lora_symbol_timeout_us(&drv_ctx.settings_cache.lora_mod_params, (uint8_t)drv_ctx.settings_cache.lora_sync_timeout);
            if ((lora_sync_timeout_us < timeout) || (0u == timeout)) /* If symbol time is shorter than Rx timeout or Rx timeout is disabled */
            {
                timeout = lora_sync_timeout_us;
            }
        }
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 */

        /* Update status in the driver before the actual start of Rx - to be able to react on IRQ properly */
        drv_ctx.radio_state = SID_PAL_RADIO_RX;
        __COMPILER_BARRIER();

        /* Initiate Rx */
        sys_err = sx126x_set_rx_with_timeout_in_rtc_step(&drv_ctx, SX126X_RADIO_INFINITE_TIME == timeout ? SX126X_RADIO_RX_CONTINUOUS_VAL : SX126X_RADIO_RXTX_NO_TIMEOUT_VAL); /* We use a software timer to avoid the radio falling back to STDBY_RC mode on Rx timeout */
        if (sys_err != SX126X_STATUS_OK)
        {
            SX126X_RADIO_LOG_ERROR("Failed to initiate Rx. Error %d", (int32_t)sys_err);
            drv_ctx.radio_state = SID_PAL_RADIO_UNKNOWN;
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Start the software timer if needed */
        err = radio_sx126x_start_radio_timeout_mon(timeout);
        if (err != RADIO_ERROR_NONE)
        {
            break;
        }

        /* Ensure LEDs will be operated only after the Rx is initiated */
        __COMPILER_BARRIER();

#if (CFG_LPM_LEVEL != 0)
        if (SID_PAL_RADIO_MODEM_MODE_FSK == drv_ctx.modem)
        {
            /* Prohibit Stop mode until Rx is done as we need fast response on Rx Done IRQ  */
            UTIL_LPM_SetStopMode((1u << CFG_LPM_SIDEWALK), UTIL_LPM_DISABLE);
#  if (CFG_LPM_STDBY_SUPPORTED != 0)
            UTIL_LPM_SetOffMode((1u << CFG_LPM_SIDEWALK), UTIL_LPM_DISABLE);
#  endif /* CFG_LPM_STDBY_SUPPORTED */
        }
        else
        {
            /* For LoRa link we don't need such rapid response. Additional 200-250us added by the wakeup from Stop LPM won't affect the link performance and we can save some power */
        }
#endif /* (CFG_LPM_LEVEL != 0) */

#if SX126X_RADIO_CFG_USE_STATUS_LED
        /* Drive the status LED after the actual start of Rx to avoid impact on timings */
        sx126x_radio_hal_rx_led_on(&drv_ctx);
#endif /* SX126X_RADIO_CFG_USE_STATUS_LED */
     } while(0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_is_cad_exit_mode(sid_pal_radio_cad_param_exit_mode_t exit_mode)
{
    int32_t err;

    if ( (exit_mode != SID_PAL_RADIO_CAD_EXIT_MODE_CS_ONLY)
      && (exit_mode != SID_PAL_RADIO_CAD_EXIT_MODE_CS_RX)
      && (exit_mode != SID_PAL_RADIO_CAD_EXIT_MODE_CS_LBT)
      && (exit_mode != SID_PAL_RADIO_CAD_EXIT_MODE_ED_ONLY)
      && (exit_mode != SID_PAL_RADIO_CAD_EXIT_MODE_ED_RX)
      && (exit_mode != SID_PAL_RADIO_CAD_EXIT_MODE_ED_LBT))
    {
        err = RADIO_ERROR_INVALID_PARAMS;
    }
    else
    {
        err = RADIO_ERROR_NONE;
    }

    return err;
}
/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_start_carrier_sense(const sid_pal_radio_fsk_cad_params_t * cad_params, sid_pal_radio_cad_param_exit_mode_t exit_mode)
{
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_2
    int32_t err;
    sx126x_status_t sys_err;

    do
    {
        /* This operation applies to FSK mode only */
        if (drv_ctx.modem != SID_PAL_RADIO_MODEM_MODE_FSK)
        {
            err = RADIO_ERROR_INVALID_STATE;
            break;
        }

        /* Validate params */
        err = sid_pal_radio_is_cad_exit_mode(exit_mode);
        if (err != RADIO_ERROR_NONE)
        {
            break;
        }

#if SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION
        if (RADIO_SX126X_LBM_BRIDGE_STATE_EXCLUSIVE_MODE == drv_ctx.lbm.bridge_state)
        {
            /* Don't allow Sidewalk operations */
            SX126X_RADIO_LOG_DEBUG("SX126x rejected CS start");
            err = RADIO_ERROR_BUSY;
            break;
        }
#endif /* SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION */

        /* Configure FEM for Rx */
        err = radio_sx126x_set_fem_mode(SX126X_RADIO_FRONTEND_MODE_RX);
        if (err != RADIO_ERROR_NONE)
        {
            break;
        }

        /* Enable IRQs relevant to CS */
        err = radio_set_irq_mask(SX126X_FSK_CARRIER_SENSE_IRQ_MASK);
        if (err != RADIO_ERROR_NONE)
        {
            break;
        }

        /* Update status in the driver */
        drv_ctx.radio_state = SID_PAL_RADIO_RX;
        drv_ctx.cad_exit_mode = exit_mode;
        __COMPILER_BARRIER();

        /* Start Rx to check for radio channel availability */
        sys_err = sx126x_set_rx_with_timeout_in_rtc_step(&drv_ctx, SX126X_RADIO_RXTX_NO_TIMEOUT_VAL); /* We use a software timer to avoid the radio falling back to STDBY_RC mode on Rx timeout */
        if (sys_err != SX126X_STATUS_OK)
        {
            SX126X_RADIO_LOG_ERROR("Failed to initiate Rx. Error %d", (int32_t)sys_err);
            drv_ctx.radio_state = SID_PAL_RADIO_UNKNOWN;
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Start the software timer for FSK if needed */
        err = radio_sx126x_start_radio_timeout_mon(cad_params->fsk_cs_duration_us);
        if (err != RADIO_ERROR_NONE)
        {
            break;
        }

        /* Ensure LEDs will be operated only after the CS is initiated */
        __COMPILER_BARRIER();

#  if (CFG_LPM_LEVEL != 0)
        /* Prohibit Stop mode until CS is done as we need fast response on the radio IRQs */
        UTIL_LPM_SetStopMode((1u << CFG_LPM_SIDEWALK), UTIL_LPM_DISABLE);
#    if (CFG_LPM_STDBY_SUPPORTED != 0)
        UTIL_LPM_SetOffMode((1u << CFG_LPM_SIDEWALK), UTIL_LPM_DISABLE);
#    endif /* CFG_LPM_STDBY_SUPPORTED */
#  endif /* (CFG_LPM_LEVEL != 0) */

#  if SX126X_RADIO_CFG_USE_STATUS_LED
        /* Drive the status LEDs after the actual start of CS to avoid impact on timings */
#    if (HALO_ENABLE_DIAGNOSTICS == 0)
        if (SID_PAL_RADIO_CAD_EXIT_MODE_CS_LBT == exit_mode)
        {
            /* This is carrier sense for FSK Tx */
            sx126x_radio_hal_tx_led_on(&drv_ctx);
        }
        else
#    endif /* HALO_ENABLE_DIAGNOSTICS */
        {
            sx126x_radio_hal_rx_led_on(&drv_ctx);
        }
#  endif /* SX126X_RADIO_CFG_USE_STATUS_LED */
     } while(0);

    return err;
#else
    (void)cad_params;
    (void)exit_mode;

    return RADIO_ERROR_NOT_SUPPORTED;
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_start_continuous_rx(void)
{
#if HALO_ENABLE_DIAGNOSTICS
    int32_t err = sid_pal_radio_start_rx(SX126X_RADIO_INFINITE_TIME);
    return err;
#else
    return RADIO_ERROR_NOT_SUPPORTED;
#endif /* HALO_ENABLE_DIAGNOSTICS */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_set_rx_duty_cycle(uint32_t rx_time, uint32_t sleep_time)
{
#if HALO_ENABLE_DIAGNOSTICS
    int32_t err;
    sx126x_status_t sys_err;

    do
    {
#  if SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION
        if (RADIO_SX126X_LBM_BRIDGE_STATE_EXCLUSIVE_MODE == drv_ctx.lbm.bridge_state)
        {
            /* Don't allow Sidewalk operations */
            err = RADIO_ERROR_BUSY;
            break;
        }
#  endif /* SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION */

        /* Validate inputs */
        if ((0u == rx_time) || (0u == sleep_time))
        {
            err = RADIO_ERROR_INVALID_PARAMS;
            break;
        }

        /* Configure FEM for Rx */
        err = radio_sx126x_set_fem_mode(SX126X_RADIO_FRONTEND_MODE_RX);
        if (err != RADIO_ERROR_NONE)
        {
            break;
        }

        /* Clear any residual IRQs */
        err = radio_clear_irq_status_all();
        if (err != RADIO_ERROR_NONE)
        {
            SX126X_RADIO_LOG_ERROR("Failed to clear IRQs before starting Rx. Error %d", err);
            break;
        }

        /* Update status in the driver */
        drv_ctx.radio_state = SID_PAL_RADIO_RX_DC;
        __COMPILER_BARRIER();

        /* Start Rx Duty Cycle mode */
        sys_err = sx126x_set_rx_duty_cycle(&drv_ctx, rx_time, sleep_time);
        if (sys_err != SX126X_STATUS_OK)
        {
            drv_ctx.radio_state = SID_PAL_RADIO_UNKNOWN;
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Ensure LEDs will be operated only after the CS is initiated */
        __COMPILER_BARRIER();

#  if SX126X_RADIO_CFG_USE_STATUS_LED
        /* Drive the status LEDs after the actual start of Rx to avoid impact on timings */
        sx126x_radio_hal_rx_led_on(&drv_ctx);
#  endif /* SX126X_RADIO_CFG_USE_STATUS_LED */

        err = RADIO_ERROR_NONE;
     } while(0);

    return err;
#else
    (void)rx_time;
    (void)sleep_time;

    return RADIO_ERROR_NOT_SUPPORTED;
#endif /* HALO_ENABLE_DIAGNOSTICS */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_lora_start_cad(void)
{
#if HALO_ENABLE_DIAGNOSTICS
    int32_t err;
    sx126x_status_t sys_err;

    do
    {
#  if SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION
        if (RADIO_SX126X_LBM_BRIDGE_STATE_EXCLUSIVE_MODE == drv_ctx.lbm.bridge_state)
        {
            /* Don't allow Sidewalk operations */
            err = RADIO_ERROR_BUSY;
            break;
        }
#  endif /* SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION */

        /* Configure FEM for CAD */
        if (SID_PAL_RADIO_CAD_EXIT_MODE_CS_LBT == drv_ctx.cad_exit_mode)
        {
            if (FALSE == drv_ctx.config->pa_config.dio2_ctrl_en)
            {
                SID_PAL_LOG_ERROR("LoRa CAD with subsequent Tx requires DIO2 to be used to switch FEM to Tx mode after CAD is finished");
                err = RADIO_ERROR_NOT_SUPPORTED;
                break;
            }

            /**
             * Preconfigure FEM for Tx. Since Tx/Rx direction is controlled by SX126x via DIO2, this will still result in FEM configured for Rx,
             * but once CAD is finished, SX126x will toggle ther DIO2 and put FEM to Tx without software interaction
             */
            err = radio_sx126x_set_fem_mode(SX126X_RADIO_FRONTEND_MODE_TX);
        }
        else
        {
            /* For CAD-only and CAD followed by Rx use Rx configuration of FEM */
            err = radio_sx126x_set_fem_mode(SX126X_RADIO_FRONTEND_MODE_RX);
        }

        if (err != RADIO_ERROR_NONE)
        {
            break;
        }

        /* Clear any residual IRQs */
        err = radio_clear_irq_status_all();
        if (err != RADIO_ERROR_NONE)
        {
            break;
        }

        /* Set CAD-specific IRQ mask */
        err = radio_set_irq_mask(SX126X_LORA_CAD_IRQ_MASK);
        if (err != RADIO_ERROR_NONE)
        {
            SX126X_RADIO_LOG_ERROR("Failed to set IRQ mask for LoRa CAD mode");
            break;
        }

        /* Update status in the driver */
        drv_ctx.radio_state = SID_PAL_RADIO_CAD;
        __COMPILER_BARRIER();

        /* Start CAD */
        sys_err = sx126x_set_cad(&drv_ctx);
        if (sys_err != SX126X_STATUS_OK)
        {
            drv_ctx.radio_state = SID_PAL_RADIO_UNKNOWN;
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Ensure LEDs will be operated only after the CS is initiated */
        __COMPILER_BARRIER();

#  if SX126X_RADIO_CFG_USE_STATUS_LED
        /* Drive the status LEDs after the actual start of Rx to avoid impact on timings */
        sx126x_radio_hal_rx_led_on(&drv_ctx);
#  endif /* SX126X_RADIO_CFG_USE_STATUS_LED */

        err = RADIO_ERROR_NONE;
     } while(0);

    return err;
#else
    return RADIO_ERROR_NOT_SUPPORTED;
#endif /* HALO_ENABLE_DIAGNOSTICS */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int16_t sid_pal_radio_rssi(void)
{
    int16_t rssi = 0;
    sx126x_status_t sys_err;

#if SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION
    if (RADIO_SX126X_LBM_BRIDGE_STATE_EXCLUSIVE_MODE == drv_ctx.lbm.bridge_state)
    {
        /* Report a default value to Sidewalk */
        return INT16_MAX;
    }
#endif /* SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION */

    do
    {
        sys_err = sx126x_get_rssi_inst(&drv_ctx, &rssi);
        if (sys_err != SX126X_STATUS_OK)
        {
            SX126X_RADIO_LOG_ERROR("Failed to read realtime RSSI. Error %d", (int32_t)sys_err);
            rssi = INT16_MAX;
            break;
        }

        /* Compensate for external LNA, Rx boost, antenna gain, etc. */
        rssi = sx126x_hal_get_adjusted_rssi(&drv_ctx, rssi);
    } while (0);

    return rssi;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_is_channel_free(uint32_t freq, int16_t threshold, uint32_t delay_us, bool * is_channel_free)
{
#if HALO_ENABLE_DIAGNOSTICS
    int32_t err, irq_err;
    sid_error_t sid_err;
    sx126x_hal_status_t hal_err;

    SID_PAL_ASSERT(is_channel_free != NULL);

    *is_channel_free = false;

    do
    {
#  if SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION
        if (RADIO_SX126X_LBM_BRIDGE_STATE_EXCLUSIVE_MODE == drv_ctx.lbm.bridge_state)
        {
            /* Don't allow Sidewalk operations */
            err = RADIO_ERROR_BUSY;
            break;
        }
#  endif /* SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION */

        /* Disable radio IRQ reaction */
        hal_err = sx126x_hal_disarm_irq(&drv_ctx);
        if (hal_err != SX126X_HAL_STATUS_OK)
        {
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Ensure the radio is in Standby */
        err = sid_pal_radio_standby();
        if (err != RADIO_ERROR_NONE)
        {
            break;
        }

        /* Set target frequency */
        err = sid_pal_radio_set_frequency(freq);
        if (err != RADIO_ERROR_NONE)
        {
            break;
        }

        /* Start continuous Rx to conduct RSSI measurements */
        err = sid_pal_radio_start_continuous_rx();
        if (err != RADIO_ERROR_NONE)
        {
            goto enable_irq;
        }

        struct sid_timespec t_start, t_cur, t_threshold;
        int16_t rssi;

        if (delay_us < SX126X_MIN_CHANNEL_FREE_DELAY_US)
        {
            delay_us = SX126X_MIN_CHANNEL_FREE_DELAY_US;
        }

        sid_us_to_timespec(delay_us, &t_threshold);

        sid_err = sid_clock_now(SID_CLOCK_SOURCE_UPTIME, &t_start, NULL);
        if (sid_err != SID_ERROR_NONE)
        {
            err = RADIO_ERROR_GENERIC;
            goto enable_irq;
        }
        __COMPILER_BARRIER();

        /* Measurement loop */
        do
        {
            /* Keep inter-measurement delay */
            sid_pal_delay_us(SX126X_MIN_CHANNEL_FREE_DELAY_US);

            /* Readout realtime RSSI */
            rssi = sid_pal_radio_rssi();
            if (rssi > threshold)
            {
                goto enable_irq;
            }

            sid_err = sid_clock_now(SID_CLOCK_SOURCE_UPTIME, &t_cur, NULL);
            if (sid_err != SID_ERROR_NONE)
            {
                err = RADIO_ERROR_GENERIC;
                goto enable_irq;
            }
            sid_time_sub(&t_cur, &t_start);
        } while(sid_time_gt(&t_threshold, &t_cur));

        /* Channel is free if the measurement loop terminated due to timeout */
        *is_channel_free = true;

enable_irq:
        /* Do not update err on success of the function calls below */

        /* Put radio to Standby to stop continuous Rx */
        irq_err = sid_pal_radio_standby();
        if (irq_err != RADIO_ERROR_NONE)
        {
            err = irq_err;
        }

        /* Clear any accumulated IRQ flags */
        irq_err = radio_clear_irq_status_all();
        if (irq_err != RADIO_ERROR_NONE)
        {
            err = irq_err;
        }

        /* Re-enabled IRQ pin on MCU side */
        hal_err = sx126x_hal_arm_irq(&drv_ctx);
        if (hal_err != SX126X_HAL_STATUS_OK)
        {
            err = RADIO_ERROR_HARDWARE_ERROR;
        }
    } while (0);

    return err;
#else
    (void)freq;
    (void)threshold;
    (void)delay_us;
    (void)is_channel_free;

    return RADIO_ERROR_NOT_SUPPORTED;
#endif /* HALO_ENABLE_DIAGNOSTICS */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_random(uint32_t * random)
{
#if HALO_ENABLE_DIAGNOSTICS
    int32_t err, irq_err;
    sx126x_hal_status_t hal_err;
    sx126x_status_t sys_err;

    SID_PAL_ASSERT(random != NULL);

#  if SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION
    if (RADIO_SX126X_LBM_BRIDGE_STATE_EXCLUSIVE_MODE == drv_ctx.lbm.bridge_state)
    {
        /* Don't allow Sidewalk operations */
        return RADIO_ERROR_BUSY;
    }
#  endif /* SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION */

    *random = UINT32_MAX;

    do
    {
        /* Disable IRQ pin from MCU side */
        hal_err = sx126x_hal_disarm_irq(&drv_ctx);
        if (hal_err != SX126X_HAL_STATUS_OK)
        {
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Run RNG */
        sys_err = sx126x_get_random_numbers(&drv_ctx, random, 1u);
        if (sys_err != SX126X_STATUS_OK)
        {
            err = RADIO_ERROR_HARDWARE_ERROR;
        }

        /* Clear any accumulated IRQ flags */
        irq_err = radio_clear_irq_status_all();
        if (irq_err != RADIO_ERROR_NONE)
        {
            err = irq_err;
        }

        /* Re-enabled IRQ pin on MCU side */
        hal_err = sx126x_hal_arm_irq(&drv_ctx);
        if (hal_err != SX126X_HAL_STATUS_OK)
        {
            err = RADIO_ERROR_HARDWARE_ERROR;
        }
    } while (0);

    return err;
#else
    (void)random;

    return RADIO_ERROR_NOT_SUPPORTED;
#endif /* HALO_ENABLE_DIAGNOSTICS */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int16_t sid_pal_radio_get_ant_dbi(void)
{
    SID_PAL_ASSERT(drv_ctx.regional_radio_param != NULL);
    return drv_ctx.regional_radio_param->ant_dbi;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_get_cca_level_adjust(sid_pal_radio_data_rate_t data_rate, int8_t * adj_level)
{
    int32_t err;

    SID_PAL_ASSERT(drv_ctx.regional_radio_param != NULL);

    do
    {
        if ((data_rate <= SID_PAL_RADIO_DATA_RATE_INVALID) || (data_rate > SID_PAL_RADIO_DATA_RATE_MAX_NUM))
        {
            err = RADIO_ERROR_INVALID_PARAMS;
            break;
        }

        *adj_level = drv_ctx.regional_radio_param->cca_level_adjust[data_rate - 1];
        err = RADIO_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_get_chan_noise(uint32_t freq, int16_t * noise)
{
#if HALO_ENABLE_DIAGNOSTICS
    int32_t err, irq_err;
    sx126x_hal_status_t hal_err;

#  if SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION
    if (RADIO_SX126X_LBM_BRIDGE_STATE_EXCLUSIVE_MODE == drv_ctx.lbm.bridge_state)
    {
        /* Don't allow Sidewalk operations */
        return RADIO_ERROR_BUSY;
    }
#  endif /* SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION */

    *noise = 0;

    do
    {
        /* Disable radio IRQ reaction */
        hal_err = sx126x_hal_disarm_irq(&drv_ctx);
        if (hal_err != SX126X_HAL_STATUS_OK)
        {
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Ensure the radio is in Standby */
        err = sid_pal_radio_standby();
        if (err != RADIO_ERROR_NONE)
        {
            break;
        }

        /* Set target frequency */
        err = sid_pal_radio_set_frequency(freq);
        if (err != RADIO_ERROR_NONE)
        {
            break;
        }

        /* Start continuous Rx to conduct RSSI measurements */
        err = sid_pal_radio_start_continuous_rx();
        if (err != RADIO_ERROR_NONE)
        {
            goto enable_irq;
        }

        for (uint32_t i = 0u; i < SX126X_NOISE_SAMPLE_SIZE; i++)
        {
            *noise += sid_pal_radio_rssi();
            sid_pal_delay_us(SX126X_MIN_CHANNEL_NOISE_DELAY_US);
        }

        *noise /= SX126X_NOISE_SAMPLE_SIZE;

enable_irq:
        /* Do not update err on success of the function calls below */

        /* Put radio to Standby to stop continuous Rx */
        irq_err = sid_pal_radio_standby();
        if (irq_err != RADIO_ERROR_NONE)
        {
            err = irq_err;
        }

        /* Clear any accumulated IRQ flags */
        irq_err = radio_clear_irq_status_all();
        if (irq_err != RADIO_ERROR_NONE)
        {
            err = irq_err;
        }

        /* Re-enabled IRQ pin on MCU side */
        hal_err = sx126x_hal_arm_irq(&drv_ctx);
        if (hal_err != SX126X_HAL_STATUS_OK)
        {
            err = RADIO_ERROR_HARDWARE_ERROR;
        }
    } while (0);

    return err;
#else
    (void)freq;
    (void)noise;

    return RADIO_ERROR_NOT_SUPPORTED;
#endif /* HALO_ENABLE_DIAGNOSTICS */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_get_radio_state_transition_delays(sid_pal_radio_state_transition_timings_t * state_delay)
{
    SID_PAL_ASSERT(state_delay != NULL);
    SID_PAL_ASSERT(drv_ctx.config != NULL);

    /* Copy the base info */
    SID_STM32_UTIL_fast_memcpy(state_delay, &drv_ctx.config->state_timings, sizeof(*state_delay));

    /* Adjust timings based on the TCXO configuration */
    if (SX126X_TCXO_CTRL_DIO3 == drv_ctx.config->tcxo_config.ctrl)
    {
        /* TCXO is powered from SX126x's DIO3 pin and will require full wait time on each start (e.g. on every transition to STDBY_XOSC, TX, RX, etc. from STDBY_RC state) */
        state_delay->tcxo_delay_us = drv_ctx.config->tcxo_config.timeout_us;
    }
    else if (SX126X_TCXO_CTRL_VDD == drv_ctx.config->tcxo_config.ctrl)
    {
        /**
         * Since TCXO is powered directly from VDD in this config we don't need a start timeout for it. However,
         * we can't set the timeout to 0 in SX126x because this will switch it to XOSC mode, so we have to use
         * the minimum timeout of 1 tick (15.625us)
         */
         state_delay->tcxo_delay_us = SX126X_TUS_TO_US(1u);
    }
    else
    {
        /* XOSC is used instead of TCXO */
        state_delay->tcxo_delay_us = 0u;
    }

    /* Adjust base wake-up time based on the TCXO start time */
    state_delay->sleep_to_full_power_us += state_delay->tcxo_delay_us;

    return RADIO_ERROR_NONE;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_init(sid_pal_radio_event_notify_t notify, sid_pal_radio_irq_handler_t dio_irq_handler, sid_pal_radio_rx_packet_t * rx_packet)
{
    int32_t err;
    sid_error_t sid_err;
    sx126x_hal_status_t hal_err;
    sx126x_status_t sys_err;
    sid_pal_radio_modem_mode_t initial_modem_mode;
    int8_t tx_power;

    SID_PAL_ASSERT(drv_ctx.config != NULL);

    do
    {
        /* Validate inputs */
        if ((NULL == notify) || (NULL == dio_irq_handler) || (NULL == rx_packet))
        {
            err = RADIO_ERROR_INVALID_PARAMS;
            break;
        }

        /*----------------------------------------------------------------------------*/

        /* Check if the driver is initialized already */
        if (drv_ctx.init_done != FALSE)
        {
#if SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION
            /* Ensure LoRa Basics Modem bridge is not active */
            if (RADIO_SX126X_LBM_BRIDGE_STATE_EXCLUSIVE_MODE == drv_ctx.lbm.bridge_state)
            {
                /* Don't allow Sidewalk operations */
                SID_PAL_LOG_ERROR("Can't reinitialize Sidewalk radio driver - LBM bridge mode is active");
                return RADIO_ERROR_BUSY;
            }
#endif /* SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION */
            SID_PAL_LOG_WARNING("SX126x driver is initialized already. Requesting deinitialization");

            /* Deinitialize any hardware that may have been partially initialized to bring it to the known state */
            err = sid_pal_radio_deinit();
            if (err != RADIO_ERROR_NONE)
            {
                SID_PAL_LOG_ERROR("Unable to initialize SX126x radio - driver is initialized already and deinitialization failed. Error %d", err);
                break;
            }
        }

        /*----------------------------------------------------------------------------*/

        /* Ensure SX126x won't trigger any interrupt if we are re-initializing ------*/
        hal_err = sx126x_hal_disarm_irq(&drv_ctx);
        if (hal_err != SX126X_HAL_STATUS_OK)
        {
            /* Logs are provided by the sx126x_hal_disarm_irq() */
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }
        __COMPILER_BARRIER(); /* Ensure IRQ is disarmed right now before any further actions take place */
        /*----------------------------------------------------------------------------*/

        /* Initialize context */
        drv_ctx.radio_rx_packet    = rx_packet;
        drv_ctx.report_radio_event = notify;
        drv_ctx.irq_handler        = dio_irq_handler;
        drv_ctx.modem              = SX126X_RADIO_MODEM_MODE_INVALID;
        drv_ctx.radio_state        = SID_PAL_RADIO_UNKNOWN;
#if SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION
        drv_ctx.lbm.bridge_state   = RADIO_SX126X_LBM_BRIDGE_STATE_INACTIVE;
        if (NULL == drv_ctx.lbm.sidewalk_radio_access_lock)
            {
            drv_ctx.lbm.sidewalk_radio_access_lock = osSemaphoreNew(1u, 0u, &sidewalk_radio_access_lock_attr);
            /* Check if the semaphore was actually created */
            if (NULL == drv_ctx.lbm.sidewalk_radio_access_lock)
            {
                SID_PAL_LOG_ERROR("Failed to create semaphore for Sidewalk and LBM radio access arbitration");
                err = RADIO_ERROR_NOMEM;
                break;
            }
        }
#endif /* SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION */

#if ((SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 != 0) && (SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 == 0))
        /* FSK-only SubGHz link, default to FSK modem mode for initialization */
        initial_modem_mode         = SID_PAL_RADIO_MODEM_MODE_FSK;
#else
        /* LoRa-only or FSK-LoRa SubGHz link - default to LoRa modem mode for initialization */
        initial_modem_mode         = SID_PAL_RADIO_MODEM_MODE_LORA;
#endif

        /* Load regional params */
        err = sid_pal_radio_set_region(drv_ctx.config->regional_config.radio_region);
        if (err != RADIO_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to set radio regional params. Error %d", err);
            break;
        }

        /* Create the timer to process Rx,Tx, and CS/CAD timeouts */
        sid_err = sid_pal_timer_init(&drv_ctx.radio_timeout_mon, radio_sx126x_on_radio_timeout_event, NULL);
        if (sid_err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to create radio timeout monitor. Error %d", (int32_t)sid_err);
        }
        /*----------------------------------------------------------------------------*/

        /* Base init */
        err = radio_sx126x_platform_init();
        if (err != RADIO_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Low-level radio init failed. Error %d", err);
            break;
        }

        /* Ensure FEM is off (if present) */
        err = radio_sx126x_set_fem_mode(SX126X_RADIO_FRONTEND_MODE_OFF);
        if (err != RADIO_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to turn off FEM. Error %d", err);
            break;
        }

        /* Reset SX126x to ensure its consistent state */
        sys_err = sx126x_reset(&drv_ctx);
        if (sys_err != SX126X_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to reset SX126x. Error %d", (int32_t)sys_err);
            err = RADIO_ERROR_IO_ERROR;
            break;
        }

        /* Assume the radio automatically reaches STDBY_RC state after reset */
        drv_ctx.radio_state = SID_PAL_RADIO_STANDBY;

        /* Ensure Standby RC mode - this is a mandatory step before configuring the regulator mode */
        sys_err = sx126x_set_standby(&drv_ctx, SX126X_STANDBY_CFG_RC);
        if (sys_err != SX126X_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to put the radio into STDBY_RC state. Error %d", (int32_t)sys_err);
            err = RADIO_ERROR_IO_ERROR;
            break;
        };

        /* Configure radio DIO2 to operate the RF switch */
        if (drv_ctx.config->pa_config.dio2_ctrl_en != FALSE)
        {
            sys_err = sx126x_set_dio2_as_rf_sw_ctrl(&drv_ctx, true);
            if (sys_err != SX126X_STATUS_OK)
            {
                SID_PAL_LOG_ERROR("Failed to configure DIO2 to drive RF switch. Error %d", (int32_t)sys_err);
                err =  RADIO_ERROR_IO_ERROR;
                break;
            }
        }

        /* Allow user-specific config for DIO3 if it is not used to drive TCXO */
        if ((drv_ctx.config->dio3_cfg_callback != NULL) && (drv_ctx.config->tcxo_config.ctrl != SX126X_TCXO_CTRL_DIO3))
        {
            err = drv_ctx.config->dio3_cfg_callback();
            if (err != RADIO_ERROR_NONE)
            {
                SID_PAL_LOG_ERROR("User-defined DIO3 configuration failed. Error %d", err);
                break;
            }
        }

        /* Apply power regulator settings */
        sys_err = sx126x_set_reg_mode(&drv_ctx, drv_ctx.config->regulator_mode);
        if (sys_err != SX126X_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to configure radio power regulator. Error %d", (int32_t)sys_err);
            err = RADIO_ERROR_IO_ERROR;
            break;
        };

        /* Now configure the radio clocks */
        err = radio_sx126x_configure_clocks();
        if (err != RADIO_ERROR_NONE)
        {
            /* Logs provided by radio_sx126x_configure_clocks() */;
            break;
        }

        /* Set fallback mode to STDBY_XOSC to save on oscillator restart time */
        sys_err = sx126x_set_rx_tx_fallback_mode(&drv_ctx, SX126X_FALLBACK_STDBY_XOSC);
        if (sys_err != SX126X_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to set radio fallback mode to STDBY_XOSC. Error %d", (int32_t)sys_err);
            err = RADIO_ERROR_IO_ERROR;
            break;
        }

        /* Bring the radio to STDBY_XOSC state now */
        sys_err = sx126x_set_standby(&drv_ctx, SX126X_STANDBY_CFG_XOSC);
        if (sys_err != SX126X_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to put the radio into STDBY_XOSC state. Error %d", (int32_t)sys_err);
            err = RADIO_ERROR_IO_ERROR;
            break;
        };
        drv_ctx.radio_state = SID_PAL_RADIO_STANDBY_XOSC;

        /* Configure Tx/Rx buffer base address - this config is static and never changes during the operation */
        sys_err = sx126x_set_buffer_base_address(&drv_ctx, SX126X_RADIO_TX_BUFFER_BASE_OFFSET, SX126X_RADIO_RX_BUFFER_BASE_OFFSET);
        if (sys_err != SX126X_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to configure SX126x Rx/Tx buffer base address. Error %d", (int32_t)sys_err);
            err = RADIO_ERROR_IO_ERROR;
            break;
        }

        /* Apply PA and LNA related settings */
        err = radio_sx126x_configure_rx_boost();
        if (err != RADIO_ERROR_NONE)
        {
            /* Logs provided by radio_sx126x_configure_rx_boost */
            break;
        }

        err = sid_pal_radio_get_max_tx_power(SID_PAL_RADIO_DATA_RATE_50KBPS, &tx_power);
        if (err != RADIO_ERROR_NONE)
        {
            break;
        }

        err = sid_pal_radio_set_tx_power(tx_power);
        if (err != RADIO_ERROR_NONE)
        {
            break;
        }

        /* Apply settings specific to the selected modem mode */
        err = sid_pal_radio_set_modem_mode(initial_modem_mode);
        if (err != RADIO_ERROR_NONE)
        {
            break;
        }

        /* Clear any potentially present IRQ flags */
        err = radio_clear_irq_status_all();
        if (err != RADIO_ERROR_NONE)
        {
            break;
        }

        /* Ensure all the above operations are completed before the radio IRQ is enabled */
        __COMPILER_BARRIER();

        /* Enable IRQ from the MCU side */
        hal_err = sx126x_hal_arm_irq(&drv_ctx);
        if (hal_err != SX126X_HAL_STATUS_OK)
        {
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Done */
        SID_PAL_LOG_INFO("SX126x radio initialized");
        drv_ctx.init_done = TRUE;
        err = RADIO_ERROR_NONE;
    } while (0);

    if (err != RADIO_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("Unable to initialize SX126x radio. Error %d", err);

        /* Ensure we deinitialize any hardware that may have been partially initialized */
        (void)sid_pal_radio_deinit();
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_deinit(void)
{
    int32_t err = RADIO_ERROR_GENERIC;

    SX126X_RADIO_LOG_DEBUG("sid_pal_radio_deinit... ");

    do
    {
        (void)sid_pal_timer_deinit(&drv_ctx.radio_timeout_mon);
        __COMPILER_BARRIER();

        /* Bring down the underlying hardware */
        err = radio_sx126x_platform_deinit();
        if (err != RADIO_ERROR_NONE)
        {
            SX126X_RADIO_LOG_ERROR("radio_sx126x_platform_deinit() failed. Error %d", err);
            break;
        }

        /* From here radio IRQs won't be processed */

        /* Invalidate the context */
        drv_ctx.init_done                   = FALSE;
        drv_ctx.irq_handler                 = NULL;
        drv_ctx.report_radio_event          = NULL;
        drv_ctx.radio_freq_hz               = 0u;
        drv_ctx.radio_freq_band             = SX126X_BAND_INVALID;
        drv_ctx.radio_rx_packet             = NULL;
        drv_ctx.radio_state                 = SID_PAL_RADIO_UNKNOWN;
#if SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION
        drv_ctx.lbm.bridge_state            = RADIO_SX126X_LBM_BRIDGE_STATE_INACTIVE;

        /* Ensure radio lock is released */
        if (osSemaphoreGetCount(drv_ctx.lbm.sidewalk_radio_access_lock) == 0u)
        {
            osStatus_t os_err = osSemaphoreRelease(drv_ctx.lbm.sidewalk_radio_access_lock);
            SID_PAL_ASSERT(osOK == os_err); /* osSemaphoreRelease() can fail only on systematic issues, so assertion is better and more efficient here */
        }
#endif /* SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION */
#if HALO_ENABLE_DIAGNOSTICS
        drv_ctx.pa_cfg_override             = FALSE;
#endif /* HALO_ENABLE_DIAGNOSTICS */
        SID_STM32_UTIL_fast_memset(&drv_ctx.settings_cache, 0u, sizeof(drv_ctx.settings_cache));

        err = RADIO_ERROR_NONE;
    } while (0);

    if (err != RADIO_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("Unable to deinitialize SX126x radio. Error %d", err);
    }
    else
    {
        SID_PAL_LOG_INFO("SX126x radio deinitialized");
    }

#if (CFG_LPM_LEVEL != 0)
    /* Allow Stop mode after the radio driver is deinitialized */
    UTIL_LPM_SetStopMode((1u << CFG_LPM_SIDEWALK), UTIL_LPM_ENABLE);
#  if (CFG_LPM_STDBY_SUPPORTED != 0)
    UTIL_LPM_SetOffMode((1u << CFG_LPM_SIDEWALK), UTIL_LPM_ENABLE);
#  endif /* CFG_LPM_STDBY_SUPPORTED */
#endif /* (CFG_LPM_LEVEL != 0) */

    return err;
}

/*----------------------------------------------------------------------------*/

#if SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION
SID_STM32_SPEED_OPTIMIZED sid_error_t sid_pal_radio_sx126x_start_lbm_bridge_mode(const radio_sx126x_lbm_bridge_state_t desired_mode)
{
    sid_error_t err;
    osStatus_t os_status;

    sid_pal_enter_critical_region();

    do
    {
        /* Validate inputs */
        if (RADIO_SX126X_LBM_BRIDGE_STATE_EXCLUSIVE_MODE == desired_mode)
        {
            /* Supported mode is selected, proceed */
        }
        else
        {
            /* Unsupported mode requested, terminate */
            err = RADIO_ERROR_INVALID_PARAMS;
            break;
        }

        /* Precondition #1 - radio driver should be initialized */
        if (FALSE == drv_ctx.init_done)
        {
            err = RADIO_ERROR_INVALID_STATE;
            break;
        }

        /* Precondition #2 - LBM bridge mode should not be activated already */
        if (drv_ctx.lbm.bridge_state > RADIO_SX126X_LBM_BRIDGE_STATE_ACTIVATION_PENDING)
        {
            /* Not an issue, just report RADIO_ERROR_NONE without any further actions */
            drv_ctx.lbm.bridge_state = desired_mode;
            err = RADIO_ERROR_NONE;
            break;
        }

        /* Precondition #3 - radio should be in Sleep state and not performing any Sidewalk-related actions */
        drv_ctx.lbm.bridge_state = RADIO_SX126X_LBM_BRIDGE_STATE_ACTIVATION_PENDING;

        /* Try to acquire radio lock in  the critical section without wait */
        os_status = osSemaphoreAcquire(drv_ctx.lbm.sidewalk_radio_access_lock, 0u);
        if (os_status != osOK)
        {
            /* Try waiting out of critical section for radio availability */
            sid_pal_exit_critical_region();
            os_status = osSemaphoreAcquire(drv_ctx.lbm.sidewalk_radio_access_lock, osWaitForever);
            sid_pal_enter_critical_region();
        }

        /* Terminate if the radio lock is not acquired by now */
        if (os_status != osOK)
        {
            err = RADIO_ERROR_BUSY;
            break;
        }

        err = sid_pal_set_radio_busy(); /* This call will atomically check the radio state and set SID_PAL_RADIO_BUSY indication for Sidewalk stack */
        if (err != RADIO_ERROR_NONE)
        {
            err = RADIO_ERROR_BUSY;
            break;
        }

        /* Switch on LBM bridge mode */
        drv_ctx.lbm.bridge_state = desired_mode;
        err = RADIO_ERROR_NONE;
    } while (0);

    sid_pal_exit_critical_region();

    return err;
}
#endif /* SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION */

/*----------------------------------------------------------------------------*/

#if SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION
SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_sx126x_stop_lbm_bridge_mode(void)
{
    int32_t err;
    sx126x_status_t sys_err;
    const radio_sx126x_lbm_bridge_state_t lbm_bridge_state_backup = drv_ctx.lbm.bridge_state;

    sid_pal_enter_critical_region();

    do
    {
        /* Precondition #1 - radio driver should be initialized */
        if (FALSE == drv_ctx.init_done)
        {
            err = RADIO_ERROR_INVALID_STATE;
            break;
        }

        /* Precondition #2 - LBM bridge mode should be activate */
        if (drv_ctx.lbm.bridge_state <= RADIO_SX126X_LBM_BRIDGE_STATE_ACTIVATION_PENDING)
        {
            err = RADIO_ERROR_INVALID_STATE;
            break;
        }

        /* Precondition #3 - radio should be in Sleep state and not performing any LBM-related actions */
        if (drv_ctx.radio_state != SID_PAL_RADIO_SLEEP)
        {
            /**
             * Note: This check intends to catch a situation when LBM bridge gets deactivated when LoRa Basics Modem
             *       is actively using the radio. Make sure that all relevant LBM radio tasks are terminated or
             *       smtc_modem_suspend_radio_communications() call is made before stopping the LBM bridge mode. Normally
             *       the code below should not be reached and if it is - this typically means some issues with the app
             *       logic flow
             */
            SID_PAL_LOG_ERROR("LBM bridge mode exit error - radio is not in sleep state");
            err = RADIO_ERROR_BUSY;
            break;
        }

        /* Clear any remaining IRQ flags. This command will also transition the radio into STDBY_RC state */
        err = radio_clear_irq_status_all();
        if (err != RADIO_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("LBM bridge mode exit failed to clear radio IRQs. Error %d", err);
            break;
        }

        /* Switch off LBM bridge mode indication to allow Sidewalk radio API calls */
        drv_ctx.lbm.bridge_state = RADIO_SX126X_LBM_BRIDGE_STATE_INACTIVE;

        /* Move to normal standby state */
        err = sid_pal_radio_standby();
        if (err != RADIO_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("LBM bridge mode exit failed to put radio into standby. Error %d", err);
            break;
        }

        /* Restore modem mode selection */
        err = sid_pal_radio_set_modem_mode(drv_ctx.modem);
        if (err != RADIO_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("LBM bridge mode exit failed to restore modem mode. Error %d", err);
            break;
        }

#  if SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
        if (SID_PAL_RADIO_MODEM_MODE_LORA == drv_ctx.modem)
        {
            /* Restore LoRa packet and modulation parameters from the last known state */
            err = sid_pal_radio_set_lora_modulation_params(&drv_ctx.settings_cache.lora_mod_params);
            if (err != RADIO_ERROR_NONE)
            {
                SID_PAL_LOG_ERROR("LBM bridge mode exit failed to restore LoRa modulation parameters. Error %d", err);
                break;
            }

            err = sid_pal_radio_set_lora_packet_params(&drv_ctx.settings_cache.lora_pkt_params);
            if (err != RADIO_ERROR_NONE)
            {
                SID_PAL_LOG_ERROR("LBM bridge mode exit failed to restore LoRa packet parameters. Error %d", err);
                break;
            }
        }
#  endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 */

#  if SID_SDK_CONFIG_ENABLE_LINK_TYPE_2
        if (SID_PAL_RADIO_MODEM_MODE_FSK == drv_ctx.modem)
        {
            /* Restore FSK packet and modulation parameters from the last known state */
            err = sid_pal_radio_set_fsk_modulation_params(&drv_ctx.settings_cache.fsk_mod_params);
            if (err != RADIO_ERROR_NONE)
            {
                SID_PAL_LOG_ERROR("LBM bridge mode exit failed to restore FSK modulation parameters. Error %d", err);
                break;
            }

            err = sid_pal_radio_set_fsk_packet_params(&drv_ctx.settings_cache.fsk_pkt_params);
            if (err != RADIO_ERROR_NONE)
            {
                SID_PAL_LOG_ERROR("LBM bridge mode exit failed to restore FSK packet parameters. Error %d", err);
                break;
            }
        }
#  endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 */

        err = sid_pal_radio_set_frequency(drv_ctx.radio_freq_hz);
        if (err != RADIO_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("LBM bridge mode exit failed to restore RF frequency setting. Error %d", err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Put the radio back to sleep - this will also clear SID_PAL_RADIO_BUSY indication to Sidewalk stack */
        err = sid_pal_radio_sleep(0u);
        if (err != RADIO_ERROR_NONE)
        {
            SX126X_RADIO_LOG_ERROR("Failed to put the radio to sleep. Error %d", err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        err = RADIO_ERROR_NONE;
    } while (0);

    if (err != RADIO_ERROR_NONE)
    {
        /* Restore LBM bridge state */
        drv_ctx.lbm.bridge_state = lbm_bridge_state_backup;
    }

    sid_pal_exit_critical_region();

    return err;
}
#endif /* SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION */

/*----------------------------------------------------------------------------*/

#if SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION
SID_STM32_SPEED_OPTIMIZED radio_sx126x_lbm_bridge_state_t sid_pal_radio_sx126x_get_lbm_bridge_mode(void)
{
    radio_sx126x_lbm_bridge_state_t current_state = drv_ctx.lbm.bridge_state;
    return current_state;
}
#endif /* SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION */

/*----------------------------------------------------------------------------*/

#if SID_SDK_CONFIG_RF_TIMINGS_MEASUREMENT
SID_STM32_SPEED_OPTIMIZED void sid_pal_radio_rxtx_start_cb(void)
{
    /* Keep this function as short as possible to minimize its contribution to the Rx/Tx processing delays */
    SIDEWALK_RF_TIMINGS_RXTX_START_GPIO_Port->BSRR = SIDEWALK_RF_TIMINGS_RXTX_START_Pin;
}
#endif /* SID_SDK_CONFIG_RF_TIMINGS_MEASUREMENT */
