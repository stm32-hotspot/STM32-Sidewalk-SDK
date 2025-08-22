/*
 * Copyright (c) 2019-2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * This file supports radio HAL interface
 */
/**
  ******************************************************************************
  * @file    lr11xx_radio.c
  * @brief   Semtech LR11xx radio driver for Sidewalk running on STM32 platform
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

#include "halo_lr11xx_radio.h"

#include "lr11xx_hal.h"
#include "lr11xx_radio.h"
#include "lr11xx_system.h"
#include "lr11xx_regmem.h"

/* Sidewalk interfaces */
#include <sid_time_ops.h>
#include <sid_pal_assert_ifc.h>
#include <sid_pal_critical_region_ifc.h>
#include <sid_pal_delay_ifc.h>
#include <sid_pal_log_ifc.h>
#include <sid_pal_uptime_ifc.h>

#if LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION
/* LoRa Basics Modem interfaces */
#  include <smtc_modem_hal.h>

/* CMSIS OS interfaces */
#  include <cmsis_os2.h>
#endif /* LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION */

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

#ifndef LR11XX_RADIO_EXTRA_LOGGING
/* Set LR11XX_RADIO_EXTRA_LOGGING to 1 to enable extended logs */
#  define LR11XX_RADIO_EXTRA_LOGGING (0)
#endif

#if LR11XX_RADIO_EXTRA_LOGGING
#  define LR11XX_RADIO_LOG_ERROR(...)         SID_PAL_LOG_ERROR(__VA_ARGS__)
#  define LR11XX_RADIO_LOG_WARNING(...)       SID_PAL_LOG_WARNING(__VA_ARGS__)
#  define LR11XX_RADIO_LOG_INFO(...)          SID_PAL_LOG_INFO(__VA_ARGS__)
#  define LR11XX_RADIO_LOG_DEBUG(...)         SID_PAL_LOG_DEBUG(__VA_ARGS__)
#  define LR11XX_RADIO_LOG_TRACE(...)         SID_PAL_LOG_TRACE(__VA_ARGS__)
#else
#  define LR11XX_RADIO_LOG_ERROR(...)         ((void)0u)
#  define LR11XX_RADIO_LOG_WARNING(...)       ((void)0u)
#  define LR11XX_RADIO_LOG_INFO(...)          ((void)0u)
#  define LR11XX_RADIO_LOG_DEBUG(...)         ((void)0u)
#  define LR11XX_RADIO_LOG_TRACE(...)         ((void)0u)
#endif

#define LR11XX_RADIO_MODEM_MODE_INVALID       (0xFFu)

#define LR11XX_DEFAULT_LORA_IRQ_MASK          ( LR11XX_SYSTEM_IRQ_TX_DONE                \
                                              | LR11XX_SYSTEM_IRQ_RX_DONE                \
                                              | LR11XX_SYSTEM_IRQ_PREAMBLE_DETECTED      \
                                              | LR11XX_SYSTEM_IRQ_HEADER_ERROR           \
                                              | LR11XX_SYSTEM_IRQ_CRC_ERROR              \
                                              | LR11XX_SYSTEM_IRQ_CAD_DONE               \
                                              | LR11XX_SYSTEM_IRQ_CAD_DETECTED           \
                                              | LR11XX_SYSTEM_IRQ_ERROR                  \
                                              )

#define LR11XX_DEFAULT_FSK_IRQ_MASK           ( LR11XX_SYSTEM_IRQ_TX_DONE                \
                                              | LR11XX_SYSTEM_IRQ_RX_DONE                \
                                              | LR11XX_SYSTEM_IRQ_PREAMBLE_DETECTED      \
                                              | LR11XX_SYSTEM_IRQ_SYNC_WORD_HEADER_VALID \
                                              | LR11XX_SYSTEM_IRQ_ERROR                  \
                                              )

#define LR11XX_FSK_CARRIER_SENSE_IRQ_MASK     ( LR11XX_SYSTEM_IRQ_PREAMBLE_DETECTED \
                                              | LR11XX_SYSTEM_IRQ_ERROR             \
                                              )

#define LR11XX_MIN_CHANNEL_FREE_DELAY_US      (300u) /* The minimum time needed in between measurements is about 300us */
#define LR11XX_NOISE_SAMPLE_SIZE              (32u)

#ifndef RADIO_LR11XX_TXPWR_WORKAROUND
#  define RADIO_LR11XX_TXPWR_WORKAROUND       (1u)
#endif

#if RADIO_LR11XX_TXPWR_WORKAROUND
#  define LR11XX_BAND_EDGE_BAND_START_FREQ    (902000000u)
#  define LR11XX_BAND_EDGE_LIMIT_FREQ         (903000000u)
#  define LR11XX_REG_ADDR_BAND_EDGE_FIX       (0x00F20420u)
#  define LR11XX_REG_MASK_BAND_EDGE_FIX       (0x00070000u) /* bits 16-18 */

#  ifndef LR11XX_REG_VALUE_BAND_EDGE_FIX_LO  /* flag to change other register value. */
#    define LR11XX_REG_VALUE_BAND_EDGE_FIX_LO (0x00050000u) /* 0b101 at mask */
#  endif /* LR11XX_REG_VALUE_BAND_EDGE_FIX_LO */
#  define LR11XX_REG_VALUE_BAND_EDGE_FIX_HI   (0x00040000u) /* 0b100 at mask */

#endif /* RADIO_LR11XX_TXPWR_WORKAROUND */

#define LR11XX_CAD_DEFAULT_TX_TIMEOUT         (0u)      /* Disable Tx timeout for CAD */

#define LR11XX_RADIO_FREQ_STEP                (4000000u) /*!< 4MHz step for frequency band selection */

#define LR11XX_RADIO_IRQ_PIN_INACTIVE_STATE   (0u)       /*!< State of the radio IRQ pin when there's no IRQ indication from the radio */

#define LR11XX_RADIO_VBAT_TO_VTCXO_DIFF_MIN   (0.2f)     /*!< Minimum difference (in Volts) required between the selected VTCXO output voltage and the Vbat power input */

#define LR11XX_RADIO_ANT_GAIN_CAP_REGION_NA             ((int32_t)(6 * 100)) /* Antenna gain (in dBi * 100) upper limit as per FCC requirements */

#define LR11XX_RADIO_PA_CFG_PARAM_INVALID               (0xFFu) /*!< Indicates that PA config parameter is invalid */
#define LR11XX_RADIO_PA_CFG_USE_LPA                     (0x01u) /*!< Use Low-Power Amplifier for Tx */
#define LR11XX_RADIO_PA_CFG_USE_HPA                     (0x00u) /*!< Use High-Power Amplifier for Tx */
#define LR11XX_RADIO_PA_CFG_LPA_UPPER_LIMIT_DBM         (15)    /*!< The upper limit of the built-in LPA in dBm */
#define LR11XX_RADIO_PA_CFG_LPA_OPTIMIZATION_14_DBM     (14)    /*!< LPA Tx power level for which an optimized PA config exists */
#define LR11XX_RADIO_PA_CFG_LPA_OPTIMIZATION_10_DBM     (10)    /*!< LPA Tx power level for which an optimized PA config exists */
#define LR11XX_RADIO_PA_CFG_LPA_LOWER_LIMIT_DBM         (-17)   /*!< The lower limit of the built-in LPA in dBm */
#define LR11XX_RADIO_PA_CFG_HPA_UPPER_LIMIT_DBM         (22)    /*!< The upper limit of the built-in HPA in dBm */
#define LR11XX_RADIO_PA_CFG_HPA_OPTIMIZATION_20_DBM     (20)    /*!< HPA Tx power level for which an optimized PA config exists */
#define LR11XX_RADIO_PA_CFG_HPA_OPTIMIZATION_17_DBM     (17)    /*!< HPA Tx power level for which an optimized PA config exists */
#define LR11XX_RADIO_PA_CFG_HPA_OPTIMIZATION_14_DBM     (14)    /*!< HPA Tx power level for which an optimized PA config exists */
#define LR11XX_RADIO_PA_CFG_HPA_LOWER_LIMIT_DBM         (-9)    /*!< The lower limit of the built-in HPA in dBm */
/* Private macro -------------------------------------------------------------*/

#define LR11XX_RADIO_VBAT_PHYS_TO_INTERNAL(V) ((uint8_t)((((uint32_t)((float)(V) * 680.f) + 9u) / 18u) + 51u)) /*!< Convert physical voltage to the internal representation for Vbat measurements */

/* Private variables ---------------------------------------------------------*/

static halo_drv_semtech_ctx_t drv_ctx = {0};

/* Private constants ---------------------------------------------------------*/

static const lr11xx_freq_band_t bands[] = {
    { 430000000u,  440000000u}, /* 430-440 MHz */
    { 470000000u,  510000000u}, /* 470-510 MHz */
    { 779000000u,  787000000u}, /* 779-787 MHz */
    { 863000000u,  879000000u}, /* 863-879 MHz */
    { 902000000u,  928000000u}, /* 902-928 MHz */
    {2400000000u, 2480000000u}, /* 2.4 GHz */
};

#if LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION
static const osSemaphoreAttr_t sidewalk_radio_access_lock_attr = {
    .name = "sid_radio_lock",
};
#endif /* LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION */

/* Private function prototypes -----------------------------------------------*/

static        int32_t  radio_lr11xx_validate_version(const lr11xx_system_version_t * const ver);
static        int32_t  radio_lr11xx_configure_clocks(void);
static        int32_t  radio_lr11xx_platform_init(void);
static        int32_t  radio_lr11xx_platform_deinit(void);
static inline int32_t  radio_set_irq_mask(const lr11xx_system_irq_mask_t irq_mask);
static inline int32_t  radio_clear_irq_status_all(void);
static inline int32_t  sid_pal_radio_set_modem_to_lora_mode(void);
static inline int32_t  sid_pal_radio_set_modem_to_fsk_mode(void);
static inline const lr11xx_freq_band_t * lr11xx_get_freq_band(const uint32_t freq);
static        void     radio_lr11xx_on_radio_timeout_event(void * arg, sid_pal_timer_t * originator);
static inline int32_t  radio_lr11xx_start_radio_timeout_mon(const uint32_t timeout_us);
static inline uint8_t  radio_lr11xx_get_vbat_raw_threshold_for_tcxo(const lr11xx_system_tcxo_supply_voltage_t tcxo_ctrl_voltage);
static inline int32_t radio_lr11xx_convert_to_db(const int32_t multiplied_db);

/* Private function definitions ----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static int32_t radio_lr11xx_validate_version(const lr11xx_system_version_t * const ver)
{
    int32_t err;

    do
    {
        const char * printable_name;
        uint16_t min_supported_version;

        /* Check for bootloader mode */
        if (LR11XX_RADIO_BOOTLOADER_MODE_ID == ver->type)
        {
            SID_PAL_LOG_WARNING("LR11xx is in bootloader mode, Sidewalk operation is not possible");
            err = RADIO_ERROR_INVALID_STATE;
            break;
        }

        /* Search for a printable name for known devices */
        switch (ver->type)
        {
            case LR11XX_RADIO_LR11XX_IC_ID:
                printable_name        = "LR1110";
                min_supported_version = LR11XX_RADIO_MIN_SUPPORTED_FW_VER_LR1110;
                break;

            case LR11XX_RADIO_LR1120_IC_ID:
                printable_name        = "LR1120";
                min_supported_version = LR11XX_RADIO_MIN_SUPPORTED_FW_VER_LR1120;
                break;

            case LR11XX_RADIO_LR1121_IC_ID:
                printable_name        = "LR1121";
                min_supported_version = LR11XX_RADIO_MIN_SUPPORTED_FW_VER_LR1121;
                break;

            default:
                printable_name = NULL;
                break;
        }

        /* Check if the transceiver is recognized */
        if (NULL == printable_name)
        {
            SID_PAL_LOG_ERROR("Unrecognized LR11xx type reported (type: 0x%02X, HW: 0x%02X, FW: 0x%04X). Can't proceed", ver->type, ver->hw, ver->fw);
            err = RADIO_ERROR_IO_ERROR;
            break;
        }

        /* Printout version information */
        SID_PAL_LOG_INFO("Detected %s device. HW: 0x%02X, FW: %02X.%02X", printable_name, ver->hw, ((ver->fw >> 8) & 0xFFu), (ver->fw & 0xFFu));

        if (drv_ctx.ver.fw < min_supported_version)
        {
            SID_PAL_LOG_WARNING("Minimum required FW version is %02X.%02X, but current is %02X.%02X. Please update to avoid issues",
                                ((min_supported_version >> 8) & 0xFFu), (min_supported_version & 0xFFu),
                                ((ver->fw >> 8) & 0xFFu), (ver->fw & 0xFFu));
        }

        err = RADIO_ERROR_NONE;
    } while (0u);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static int32_t radio_lr11xx_configure_clocks(void)
{
    int32_t err;
    lr11xx_status_t sys_err;

    do
    {
        /* Set LF clock source and wait for the LF clock readiness */
        sys_err = lr11xx_system_cfg_lfclk(&drv_ctx, drv_ctx.config->lfclock_cfg, true);
        if (sys_err != LR11XX_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to configure LR11xx LF clock. Error %d", (int32_t)sys_err);
            err = RADIO_ERROR_IO_ERROR;
            break;
        }

        /* Bring up TCXO (if used) */
        if (drv_ctx.config->tcxo_config.ctrl != LR11XX_TCXO_CTRL_NONE)
        {
            uint32_t tcxo_start_timeout;

            SID_PAL_ASSERT((drv_ctx.config->tcxo_config.ctrl_voltage >= LR11XX_SYSTEM_TCXO_CTRL_1_6V) && (drv_ctx.config->tcxo_config.ctrl_voltage <= LR11XX_SYSTEM_TCXO_CTRL_3_3V));

            if (LR11XX_TCXO_CTRL_VTCXO == drv_ctx.config->tcxo_config.ctrl)
            {
                tcxo_start_timeout = LR11XX_US_TO_TUS(drv_ctx.config->tcxo_config.timeout_us);
            }
            else /* if (LR11XX_TCXO_CTRL_VDD == drv_ctx.config->tcxo_config.ctrl) */
            {
                /**
                 * Since TCXO is powered directly from VDD in this config we don't need a start timeout for it. However,
                 * we can't set the timeout to 0 in LR11xx because this will switch it to XOSC mode, so we have to use
                 * the minimum timeout of 1 tick (approx. 30.52us)
                 */
                tcxo_start_timeout = 1u;
            }

            /* Enable TCXO */
            sys_err = lr11xx_system_set_tcxo_mode(&drv_ctx, drv_ctx.config->tcxo_config.ctrl_voltage, tcxo_start_timeout);
            if (sys_err != LR11XX_STATUS_OK)
            {
                SID_PAL_LOG_ERROR("Failed to configure LR11xx TCXO. Error %d", (int32_t)sys_err);
                err = RADIO_ERROR_IO_ERROR;
                break;
            }

            /* TCXO usage automatically generates error on boot because HF clock cannot be started. Clear it */
            sys_err = lr11xx_system_clear_errors(&drv_ctx);
            if (sys_err != LR11XX_STATUS_OK)
            {
                err = RADIO_ERROR_IO_ERROR;
                break;
            }

            /* Now when TCXO is configured we can perform Vbat measurement, doing it earlier is not possible and will result in an error */
            if (LR11XX_TCXO_CTRL_VTCXO == drv_ctx.config->tcxo_config.ctrl)
            {
                /* In this mode TCXO control voltage shall be at least 200mV below Vbat for correct operation */
                uint8_t vbat_raw;
                const uint8_t tcxo_threshold_raw = radio_lr11xx_get_vbat_raw_threshold_for_tcxo(drv_ctx.config->tcxo_config.ctrl_voltage);

                sys_err = lr11xx_system_get_vbat(&drv_ctx, &vbat_raw);
                if (sys_err != LR11XX_STATUS_OK)
                {
                    SID_PAL_LOG_ERROR("Failed to read LR11xx Vbat. Error %d", (int32_t)sys_err);
                    err = RADIO_ERROR_IO_ERROR;
                    break;
                }

                if (vbat_raw < tcxo_threshold_raw)
                {
                    SID_PAL_LOG_ERROR("VBAT is too low for TCXO operation. VTCXO config shall be at least 200mV below VBAT");
                    SID_PAL_LOG_ERROR("VBAT voltage (raw): %u, threshold (raw): %u", vbat_raw, tcxo_threshold_raw);
                    err = RADIO_ERROR_HARDWARE_ERROR;
                    break;
                }
            }

            /* Re-calibrate HF_XOSC since startup calibration is not valid (TCXO is off on boot) */
            const uint8_t callibrate_all = LR11XX_SYSTEM_CALIB_LF_RC_MASK | LR11XX_SYSTEM_CALIB_HF_RC_MASK |
                                           LR11XX_SYSTEM_CALIB_PLL_MASK | LR11XX_SYSTEM_CALIB_ADC_MASK |
                                           LR11XX_SYSTEM_CALIB_IMG_MASK | LR11XX_SYSTEM_CALIB_PLL_TX_MASK;
            sys_err = lr11xx_system_calibrate(&drv_ctx, callibrate_all);
            if (sys_err != LR11XX_STATUS_OK)
            {
                SID_PAL_LOG_ERROR("Failed to re-calibrate LR11xx after switching to TCXO. Error %d", (int32_t)sys_err);
                err = RADIO_ERROR_IO_ERROR;
                break;
            }
        }
#if LR11XX_RADIO_CFG_GEOLOCATION_SUPPORT
        else
        {
            /**
             * GNSS scanning is extremely sensitive to clock stability. It's essential to use TCXO instead of XOSC for this reason.
             * However, if you are only interested in WiFi scanning capabilities and not the GNSS ones feel free to comment out
             * the error below
             */
            SID_PAL_LOG_ERROR("GNSS applications require LR11xx to run on TCXO, XOSC cannot be used.")
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }
#endif /* LR11XX_RADIO_CFG_GEOLOCATION_SUPPORT */

        err = RADIO_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static int32_t radio_lr11xx_platform_init(void)
{
    int32_t err = RADIO_ERROR_GENERIC;
    sid_error_t sid_err;
    lr11xx_hal_status_t hal_err;

    do
    {
        /* Validate inputs */
        if (NULL == drv_ctx.config->pa_cfg_callback)
        {
            SID_PAL_LOG_ERROR("LR11xx PA configuration callback must be set");
            err = RADIO_ERROR_INVALID_PARAMS;
            break;
        }

        /* Configure GPIO pins */
        hal_err = lr11xx_hal_gpio_init(&drv_ctx);
        if (hal_err != LR11XX_HAL_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to initialize GPIO to drive LR11xx. HAL error %d", (int32_t)hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Create SPI bus */
        sid_err = drv_ctx.config->bus_factory->create(&drv_ctx.bus_iface, drv_ctx.config->bus_factory->config);
        if (sid_err != SID_ERROR_NONE)
        {
            LR11XX_RADIO_LOG_ERROR("Failed to create SPI bus");
            err = RADIO_ERROR_IO_ERROR;
            break;
        }

        /* Done */
        err = RADIO_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static int32_t radio_lr11xx_platform_deinit(void)
{
    int32_t err = RADIO_ERROR_GENERIC;
    sid_error_t sid_err;
    lr11xx_hal_status_t hal_err;

    do
    {
        /* Deactivate IRQ pin */
        hal_err = lr11xx_hal_disarm_irq(&drv_ctx);
        if (hal_err != LR11XX_HAL_STATUS_OK)
        {
           LR11XX_RADIO_LOG_ERROR("radio_lr11xx_platform_deinit - unable to deactivate radio IRQ pin. Error %u", (uint32_t)hal_err);
           err = RADIO_ERROR_HARDWARE_ERROR;
           break;
        }

        /* Bring down SPI bus */
        const struct sid_pal_serial_bus_iface * const spi_bus_iface = drv_ctx.bus_iface;
        if (spi_bus_iface != NULL)
        {
            if (NULL == spi_bus_iface->destroy)
            {
                LR11XX_RADIO_LOG_WARNING("radio_lr11xx_platform_deinit - spi_bus_iface has no destroy() method");
                err = RADIO_ERROR_NOT_SUPPORTED;
                break;
            }

            sid_err = spi_bus_iface->destroy(spi_bus_iface);
            if (sid_err != SID_ERROR_NONE)
            {
                LR11XX_RADIO_LOG_ERROR("radio_lr11xx_platform_deinit - error in spi_bus_iface->destroy(). Error %d", (int32_t)sid_err);
                err = RADIO_ERROR_IO_ERROR;
                break;
            }
        }
        else
        {
            SID_PAL_LOG_WARNING("radio_lr11xx_platform_deinit - SPI bus interface is null, bus deinitialization skipped");
        }

        /* Release GPIO pins */
        hal_err = lr11xx_hal_gpio_deinit(&drv_ctx);
        if (hal_err != LR11XX_HAL_STATUS_OK)
        {
           LR11XX_RADIO_LOG_ERROR("radio_lr11xx_platform_deinit - unable to de-initialize radio GPIO. Error %u", (uint32_t)hal_err);
           err = RADIO_ERROR_HARDWARE_ERROR;
           break;
        }

        /* Deinitialization is competed */
        LR11XX_RADIO_LOG_DEBUG("radio_lr11xx_platform_deinit - done");
        err = RADIO_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline int32_t radio_set_irq_mask(const lr11xx_system_irq_mask_t irq_mask)
{
    int32_t err;
    lr11xx_status_t hal_err;

    sid_pal_enter_critical_region();
    hal_err = lr11xx_system_set_dio_irq_params(&drv_ctx, irq_mask, LR11XX_SYSTEM_IRQ_NONE);

    if (hal_err != LR11XX_STATUS_OK)
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

SID_STM32_SPEED_OPTIMIZED static inline int32_t radio_clear_irq_status_all(void)
{
    int32_t err;
    lr11xx_status_t hal_err;

    hal_err = lr11xx_system_clear_irq_status(&drv_ctx, LR11XX_SYSTEM_IRQ_ALL_MASK);
    if (hal_err != LR11XX_STATUS_OK)
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

SID_STM32_SPEED_OPTIMIZED static inline int32_t sid_pal_radio_set_modem_to_lora_mode(void)
{
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
    int32_t err;
    lr11xx_status_t sys_err;

    do
    {
        /* Set modem mode in the transceiver */
        sys_err = lr11xx_radio_set_pkt_type(&drv_ctx, LR11XX_RADIO_PKT_TYPE_LORA);
        if (sys_err != LR11XX_STATUS_OK)
        {
            LR11XX_RADIO_LOG_ERROR("Failed to set packet type to LoRa");
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        err = radio_set_irq_mask(LR11XX_DEFAULT_LORA_IRQ_MASK);
        if (err != RADIO_ERROR_NONE)
        {
            LR11XX_RADIO_LOG_ERROR("Failed to set IRQ mask for LoRa mode");
            break;
        }

        /* Done */
        drv_ctx.modem = SID_PAL_RADIO_MODEM_MODE_LORA;
        err = RADIO_ERROR_NONE;
    } while (0);

    return err;
#else
    return RADIO_ERROR_NOT_SUPPORTED;
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline int32_t sid_pal_radio_set_modem_to_fsk_mode(void)
{
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_2
    int32_t err;
    lr11xx_status_t sys_err;

    do
    {
        /* Set modem mode in the transceiver */
        sys_err = lr11xx_radio_set_pkt_type(&drv_ctx, LR11XX_RADIO_PKT_TYPE_GFSK);
        if (sys_err != LR11XX_STATUS_OK)
        {
            LR11XX_RADIO_LOG_ERROR("Failed to set packet type to FSK");
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        err = radio_set_irq_mask(LR11XX_DEFAULT_FSK_IRQ_MASK);
        if (err != RADIO_ERROR_NONE)
        {
            LR11XX_RADIO_LOG_ERROR("Failed to set IRQ mask for FSK mode");
            break;
        }

        /* Done */
        drv_ctx.modem = SID_PAL_RADIO_MODEM_MODE_FSK;
        err = RADIO_ERROR_NONE;
    } while (0);

    return err;
#else
    return RADIO_ERROR_NOT_SUPPORTED;
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline const lr11xx_freq_band_t * lr11xx_get_freq_band(const uint32_t freq)
{
    const lr11xx_freq_band_t * freq_band = NULL;

    for (uint32_t i = 0u; i < SID_STM32_UTIL_ARRAY_SIZE(bands); i++)
    {
        if ((freq <= bands[i].stop) && (freq >= bands[i].start))
        {
            /* Found */
            freq_band = &bands[i];
            break;
        }
    }

    return freq_band;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static void radio_lr11xx_on_radio_timeout_event(void * arg, sid_pal_timer_t * originator)
{
    int32_t err;
    lr11xx_status_t sys_err;
    sid_pal_radio_events_t radio_event = SID_PAL_RADIO_EVENT_UNKNOWN;
    uint32_t irq_disarmed = FALSE;
    int16_t rssi_now = 0u;
    uint32_t radio_state_on_entry;

    (void)arg;
    (void)originator;

    do
    {
        LR11XX_RADIO_LOG_DEBUG("LR11XX_SIMULATED_IRQ_TIMEOUT");

#if LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION && LR11XX_RADIO_CFG_LBM_CONCURRENCY_SUPPORT
        if (RADIO_LR11XX_LBM_BRIDGE_STATE_SIDEWALK_BLANKING_MODE == drv_ctx.lbm.bridge_state)
        {
            if (SID_PAL_RADIO_RX == drv_ctx.lbm.simulated_radio_state)
            {
                /* Process Carrier Sense/CAD events */
                if ((SID_PAL_RADIO_CAD_EXIT_MODE_CS_LBT == drv_ctx.cad_exit_mode) && (SID_PAL_RADIO_MODEM_MODE_FSK == drv_ctx.modem))
                {
                    drv_ctx.cad_exit_mode = SID_PAL_RADIO_CAD_EXIT_MODE_NONE;

#  if HALO_ENABLE_DIAGNOSTICS
                    radio_event = SID_PAL_RADIO_EVENT_CS_TIMEOUT;
#  else
                    /* Simulate Tx Done event for FSK - since every Tx requries ACK, this will lead to a simulated no-ACK from the gateway */
                    LR11XX_RADIO_LOG_DEBUG("LR11xx simulated Tx Done after CS");
                    radio_event = SID_PAL_RADIO_EVENT_TX_DONE;
#  endif /* HALO_ENABLE_DIAGNOSTICS */
                }
                else /* if (SID_PAL_RADIO_CAD_EXIT_MODE_NONE == drv_ctx.cad_exit_mode) */
                {
                    /* This was a normal Rx, not CS */
                    LR11XX_RADIO_LOG_DEBUG("LR11xx simulated Rx Timeout");
                    radio_event = SID_PAL_RADIO_EVENT_RX_TIMEOUT;
                }
            }
            else
            {
                /* Normally should never happen */
                LR11XX_RADIO_LOG_ERROR("Unexpected radio state %u in simulated Tx/Rx timeout event", drv_ctx.lbm.simulated_radio_state);
                SID_PAL_ASSERT(0);
            }

            /* Simulate radio is in standby after Tx/Rx */
            drv_ctx.lbm.simulated_radio_state = SID_PAL_RADIO_STANDBY;

            SID_PAL_ASSERT(radio_event != SID_PAL_RADIO_EVENT_UNKNOWN); /* It is expected that simulation always generates a valid event */
            drv_ctx.report_radio_event(radio_event);
            return;
        }
#endif /* LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION && LR11XX_RADIO_CFG_LBM_CONCURRENCY_SUPPORT */

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
            if (irq_pin_state != LR11XX_RADIO_IRQ_PIN_INACTIVE_STATE)
            {
                /* There's an active IRQ, terminate processing */
                sid_pal_exit_critical_region();
                err = RADIO_ERROR_INVALID_STATE;
                break;
            }

            /* There's no active IRQ at the moment - temporarily deactivate IRQ pin */
            (void)lr11xx_hal_disarm_irq(&drv_ctx);
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
        sys_err = lr11xx_system_set_standby(&drv_ctx, LR11XX_SYSTEM_STANDBY_CFG_XOSC); /* Use STDBY_XOSC to save on oscillator restart time (e.g. when Tx follows right after CS) */
        if (sys_err != LR11XX_STATUS_OK)
        {
            drv_ctx.radio_state = SID_PAL_RADIO_UNKNOWN;
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Turn off FEM as we don't need it any longer */
        (void)radio_lr11xx_set_subghz_fem_mode(LR11XX_RADIO_FRONTEND_MODE_OFF);

        radio_state_on_entry = drv_ctx.radio_state;
        drv_ctx.radio_state  = SID_PAL_RADIO_STANDBY_XOSC;
        __COMPILER_BARRIER();

        if (SID_PAL_RADIO_RX == radio_state_on_entry)
        {
            if ((drv_ctx.cad_exit_mode != SID_PAL_RADIO_CAD_EXIT_MODE_NONE) && (SID_PAL_RADIO_MODEM_MODE_FSK == drv_ctx.modem))
            {
                /* Restore the default IRQ mask for FSK after any type of Carrier Sense */
                err = radio_set_irq_mask(LR11XX_DEFAULT_FSK_IRQ_MASK);
                if (err != RADIO_ERROR_NONE)
                {
                    LR11XX_RADIO_LOG_ERROR("Failed to set IRQ mask for FSK mode");
                    break;
                }
            }

            /* Process Carrier Sense/CAD events */
            if ((SID_PAL_RADIO_CAD_EXIT_MODE_CS_LBT == drv_ctx.cad_exit_mode) && (SID_PAL_RADIO_MODEM_MODE_FSK == drv_ctx.modem))
            {
                drv_ctx.cad_exit_mode = SID_PAL_RADIO_CAD_EXIT_MODE_NONE;

                if (rssi_now >= LR11XX_SIDEWALK_FSK_CS_RSSI_THRESHOLD)
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
                    err = sid_pal_radio_start_tx(LR11XX_CAD_DEFAULT_TX_TIMEOUT);
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
        (void)lr11xx_hal_arm_irq(&drv_ctx);
    }

    if ((RADIO_ERROR_NONE == err) && (SID_PAL_RADIO_EVENT_UNKNOWN != radio_event))
    {
#if SID_SDK_CONFIG_RF_TIMINGS_MEASUREMENT
            /* Clear Rx/Tx starting point indication */
            SIDEWALK_RF_TIMINGS_RXTX_START_GPIO_Port->BRR = SIDEWALK_RF_TIMINGS_RXTX_START_Pin;
#endif /* SID_SDK_CONFIG_RF_TIMINGS_MEASUREMENT */

#if LR11XX_RADIO_CFG_USE_STATUS_LED
        /* Turn off activity LEDs - any event means activities are finished */
        lr11xx_radio_hal_tx_led_off(&drv_ctx);
        lr11xx_radio_hal_rx_led_off(&drv_ctx);
#endif /* LR11XX_RADIO_CFG_USE_STATUS_LED */

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

SID_STM32_SPEED_OPTIMIZED static inline int32_t radio_lr11xx_start_radio_timeout_mon(const uint32_t timeout_us)
{
    int32_t err;

    do
    {
        sid_error_t sid_err;
        lr11xx_hal_status_t hal_err;
        struct sid_timespec rx_timeout_ts;

        if ((0u == timeout_us) || (LR11XX_RADIO_INFINITE_TIME == timeout_us))
        {
            /* No need to start the countdown */
            err = RADIO_ERROR_NONE;
            break;
        }

        /* Wait for Rx/Tx to actually start */
        hal_err = lr11xx_hal_wait_readiness(&drv_ctx);
        if (hal_err != LR11XX_HAL_STATUS_OK)
        {
            err = RADIO_ERROR_IO_ERROR;
            break;
        }

        /* Ensure the timestamp will be acquired exactly after this point */
        __COMPILER_BARRIER();

        /* Calculate timeout timestamp */
        sid_err = sid_pal_uptime_now(&rx_timeout_ts);
        if (sid_err != SID_ERROR_NONE)
        {
            err = RADIO_ERROR_IO_ERROR;
            break;
        }

        /* Compensate for the timer setup processing delay */
        rx_timeout_ts.tv_nsec += ((timeout_us + LR11XX_TUS_TO_US(1u)) * 1000u);
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

SID_STM32_SPEED_OPTIMIZED static inline uint8_t radio_lr11xx_get_vbat_raw_threshold_for_tcxo(const lr11xx_system_tcxo_supply_voltage_t tcxo_ctrl_voltage)
{
    uint8_t vbat_raw_theshold;

    switch (tcxo_ctrl_voltage)
    {
        case LR11XX_SYSTEM_TCXO_CTRL_1_6V:
            vbat_raw_theshold = LR11XX_RADIO_VBAT_PHYS_TO_INTERNAL(1.6 + LR11XX_RADIO_VBAT_TO_VTCXO_DIFF_MIN);
            break;

        case LR11XX_SYSTEM_TCXO_CTRL_1_7V:
            vbat_raw_theshold = LR11XX_RADIO_VBAT_PHYS_TO_INTERNAL(1.7 + LR11XX_RADIO_VBAT_TO_VTCXO_DIFF_MIN);
            break;

        case LR11XX_SYSTEM_TCXO_CTRL_1_8V:
            vbat_raw_theshold = LR11XX_RADIO_VBAT_PHYS_TO_INTERNAL(1.8 + LR11XX_RADIO_VBAT_TO_VTCXO_DIFF_MIN);
            break;

        case LR11XX_SYSTEM_TCXO_CTRL_2_2V:
            vbat_raw_theshold = LR11XX_RADIO_VBAT_PHYS_TO_INTERNAL(2.2 + LR11XX_RADIO_VBAT_TO_VTCXO_DIFF_MIN);
            break;

        case LR11XX_SYSTEM_TCXO_CTRL_2_4V:
            vbat_raw_theshold = LR11XX_RADIO_VBAT_PHYS_TO_INTERNAL(2.4 + LR11XX_RADIO_VBAT_TO_VTCXO_DIFF_MIN);
            break;

        case LR11XX_SYSTEM_TCXO_CTRL_2_7V:
            vbat_raw_theshold = LR11XX_RADIO_VBAT_PHYS_TO_INTERNAL(2.7 + LR11XX_RADIO_VBAT_TO_VTCXO_DIFF_MIN);
            break;

        case LR11XX_SYSTEM_TCXO_CTRL_3_0V:
            vbat_raw_theshold = LR11XX_RADIO_VBAT_PHYS_TO_INTERNAL(3.0 + LR11XX_RADIO_VBAT_TO_VTCXO_DIFF_MIN);
            break;

        case LR11XX_SYSTEM_TCXO_CTRL_3_3V:
            vbat_raw_theshold = LR11XX_RADIO_VBAT_PHYS_TO_INTERNAL(3.3 + LR11XX_RADIO_VBAT_TO_VTCXO_DIFF_MIN);
            break;

        default:
            /* Use physically unreachable value to indicate the error */
            vbat_raw_theshold = UINT8_MAX;
            break;
    }

    return vbat_raw_theshold;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline int32_t radio_lr11xx_convert_to_db(const int32_t multiplied_db)
{
    int32_t result;

    if (multiplied_db > 0)
    {
        result = (multiplied_db + (LR11XX_PA_CFG_DB_MULT / 2)) / LR11XX_PA_CFG_DB_MULT;
    }
    else
    {
        result = (multiplied_db - (LR11XX_PA_CFG_DB_MULT / 2)) / LR11XX_PA_CFG_DB_MULT;
    }

    return result;
}

/* Global function definitions -----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED halo_drv_semtech_ctx_t * lr11xx_get_drv_ctx(void)
{
    return &drv_ctx;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t radio_lr11xx_compute_tx_power_config(const int8_t power)
{
    int32_t err;

    SID_PAL_ASSERT(drv_ctx.regional_radio_param != NULL);

    /* Compute the target Tx power taking into account the antenna gain */
    int32_t target_pwr = power * LR11XX_PA_CFG_DB_MULT; /* Multiply by 100 for more accurate calculations */
    switch (drv_ctx.regional_radio_param->param_region)
    {
        case RADIO_REGION_NA:
            if (drv_ctx.regional_radio_param->ant_dbi > LR11XX_RADIO_ANT_GAIN_CAP_REGION_NA)
            {
                /* As per the FCC requirements PA gain shall be reduced by the difference between antenna gain and the antenna gain cap value */
                target_pwr -= ((int32_t)drv_ctx.regional_radio_param->ant_dbi - LR11XX_RADIO_ANT_GAIN_CAP_REGION_NA);
            }
            break;

        case RADIO_REGION_EU:
            /* EU requirements put the limit on ERP - PA gain shall be reduced by antenna gain */
            target_pwr -= ((int32_t)drv_ctx.regional_radio_param->ant_dbi);
            break;

        case RADIO_REGION_NONE:
        default:
            /* Apply PA gain as is */
            break;
    }

    /* Fill-in drv_ctx.pa_cfg with invalid values */
    SID_STM32_UTIL_fast_memset(&drv_ctx.pa_cfg, LR11XX_RADIO_PA_CFG_PARAM_INVALID, sizeof(drv_ctx.pa_cfg));

    /* Call app callback to populate PA config - this allows the app to override the default values */
    err = drv_ctx.config->pa_cfg_callback(target_pwr, drv_ctx.regional_radio_param, &drv_ctx.pa_cfg);
    if (err != RADIO_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("Unable to select LR11xx PA settings. Error %d", err);
        return RADIO_ERROR_INVALID_PARAMS;
    }

    /* Check if the callback applied custom PA config */
    if ((LR11XX_RADIO_PA_CFG_PARAM_INVALID == drv_ctx.pa_cfg.pa_cfg.pa_duty_cycle)
     || (LR11XX_RADIO_PA_CFG_PARAM_INVALID == drv_ctx.pa_cfg.pa_cfg.pa_hp_sel)
     || (LR11XX_RADIO_PA_CFG_PARAM_INVALID == drv_ctx.pa_cfg.pa_cfg.pa_sel)
     || (LR11XX_RADIO_PA_CFG_PARAM_INVALID == drv_ctx.pa_cfg.pa_cfg.pa_reg_supply)
     || (((LR11XX_RADIO_PA_CFG_PARAM_INVALID << 8) | LR11XX_RADIO_PA_CFG_PARAM_INVALID) == (uint16_t)drv_ctx.pa_cfg.target_tx_power)
     || (LR11XX_RADIO_PA_CFG_PARAM_INVALID == (uint8_t)drv_ctx.pa_cfg.tx_power_reg)
#if LR11XX_RADIO_CFG_USE_EXTERNAL_PA && LR11XX_RADIO_CFG_USE_FEM_BYPASS
     || (LR11XX_RADIO_PA_CFG_PARAM_INVALID == drv_ctx.pa_cfg.enable_ext_pa)
#endif /* LR11XX_RADIO_CFG_USE_EXTERNAL_PA && LR11XX_RADIO_CFG_USE_FEM_BYPASS */
     || (LR11XX_RADIO_PA_CFG_PARAM_INVALID == drv_ctx.pa_cfg.ramp_time))
    {
        /* At least one PA config param was not set - apply default config */
        /* Check if the callback changed the target Tx power */
        if (((LR11XX_RADIO_PA_CFG_PARAM_INVALID << 8) | LR11XX_RADIO_PA_CFG_PARAM_INVALID) == (uint16_t)drv_ctx.pa_cfg.target_tx_power)
        {
            drv_ctx.pa_cfg.target_tx_power = (int16_t)target_pwr;
        }
        else
        {
            target_pwr = drv_ctx.pa_cfg.target_tx_power;
        }

#if LR11XX_RADIO_CFG_USE_EXTERNAL_PA
#  if LR11XX_RADIO_CFG_USE_FEM_BYPASS
        /* Check if need external PA to achieve the desired Tx power */
        if ( (drv_ctx.config->enable_lpa != false)
                && (target_pwr + drv_ctx.config->pa_config.tx_bypass_loss) <= (LR11XX_RADIO_PA_CFG_LPA_UPPER_LIMIT_DBM * LR11XX_PA_CFG_DB_MULT))
        {
            /* The requested power level can be achieved with LPA alone, even compensating for Tx bypass loss */
            drv_ctx.pa_cfg.enable_ext_pa = FALSE;
            target_pwr += drv_ctx.config->pa_config.tx_bypass_loss; /* Compensate for bypass loss */
            /* Requested Tx power can be achieved with LPA */
            drv_ctx.pa_cfg.pa_cfg.pa_sel = LR11XX_RADIO_PA_SEL_LP;
        }
        else if ( (drv_ctx.config->enable_hpa != false)
                    && (target_pwr + drv_ctx.config->pa_config.tx_bypass_loss) <= (LR11XX_RADIO_PA_CFG_HPA_UPPER_LIMIT_DBM * LR11XX_PA_CFG_DB_MULT))
        {
            /* The requested power level can be achieved with HPA alone, even compensating for Tx bypass loss */
            drv_ctx.pa_cfg.enable_ext_pa = FALSE;
            target_pwr += drv_ctx.config->pa_config.tx_bypass_loss; /* Compensate for bypass loss */
            /* Requested Tx power can be achieved with HPA */
            drv_ctx.pa_cfg.pa_cfg.pa_sel = LR11XX_RADIO_PA_SEL_HP;
        }
        else
#  endif /* LR11XX_RADIO_CFG_USE_FEM_BYPASS */
        if ( (drv_ctx.config->enable_lpa != false)
                    && (target_pwr + drv_ctx.config->pa_config.tx_gain_dbi) <= (LR11XX_RADIO_PA_CFG_LPA_UPPER_LIMIT_DBM * LR11XX_PA_CFG_DB_MULT))
        {
#  if LR11XX_RADIO_CFG_USE_FEM_BYPASS
            /* Requested power level cannot be achieved by the built-in PA only, external PA shall be engaged */
            drv_ctx.pa_cfg.enable_ext_pa = TRUE;
#  endif
            target_pwr -= drv_ctx.config->pa_config.tx_gain_dbi; /* Take into account the gain of the external PA */
            /* Requested Tx power can be achieved with LPA */
            drv_ctx.pa_cfg.pa_cfg.pa_sel = LR11XX_RADIO_PA_SEL_LP;
        }
        else if ( (drv_ctx.config->enable_hpa != false)
                    && (target_pwr + drv_ctx.config->pa_config.tx_gain_dbi) <= (LR11XX_RADIO_PA_CFG_HPA_UPPER_LIMIT_DBM * LR11XX_PA_CFG_DB_MULT))
        {
#  if LR11XX_RADIO_CFG_USE_FEM_BYPASS
            /* Requested power level cannot be achieved by the built-in PA only, external PA shall be engaged */
            drv_ctx.pa_cfg.enable_ext_pa = TRUE;
#  endif
            target_pwr -= drv_ctx.config->pa_config.tx_gain_dbi; /* Take into account the gain of the external PA */
            /* Requested Tx power can be achieved with HPA */
            drv_ctx.pa_cfg.pa_cfg.pa_sel = LR11XX_RADIO_PA_SEL_HP;
        }
        else
        {
            /* Requested power level cannot be achieved */
            return RADIO_ERROR_INVALID_PARAMS;
        }
#else
        /* Compensate for RF switch losses */
        target_pwr += drv_ctx.config->pa_config.rf_sw_insertion_loss;
        /* Apply the optimized PA settings as per the application note to achieve the lower power consumption */
        if ((drv_ctx.config->enable_lpa != false)  && (drv_ctx.config->enable_hpa != false))
        {
            /* Both HPA and LPA are enabled - select the PA to use based on the targeted Tx power */
            if (drv_ctx.pa_cfg.target_tx_power > (LR11XX_RADIO_PA_CFG_LPA_UPPER_LIMIT_DBM * LR11XX_PA_CFG_DB_MULT))
            {
                /* Requested Tx power exceeds the capabilities of LPA - use HPA for it */
                drv_ctx.pa_cfg.pa_cfg.pa_sel = LR11XX_RADIO_PA_SEL_HP;
            }
            else
            {
                /* Requested Tx power can be achieved with LPA */
                drv_ctx.pa_cfg.pa_cfg.pa_sel = LR11XX_RADIO_PA_SEL_LP;
            }
        }
        else if (drv_ctx.config->enable_hpa != false)
        {
            /* Only HPA is enabled */
            drv_ctx.pa_cfg.pa_cfg.pa_sel = LR11XX_RADIO_PA_SEL_HP;
        }
        else if (drv_ctx.config->enable_lpa != false)
        {
            /* Only LPA is enabled */
            drv_ctx.pa_cfg.pa_cfg.pa_sel = LR11XX_RADIO_PA_SEL_LP;
        }
        else
        {
            return RADIO_ERROR_INVALID_PARAMS;
        }
#endif /* LR11XX_RADIO_CFG_USE_EXTERNAL_PA */

        /* Go back into physical dB representation as we are about to configure the PA */
        target_pwr = radio_lr11xx_convert_to_db(target_pwr);

        /* Configure PA parameters based on the selected PA */
        if (LR11XX_RADIO_PA_SEL_HP == drv_ctx.pa_cfg.pa_cfg.pa_sel)
        {
            /* Saturate by upper limit for HPA */
            if (target_pwr > LR11XX_RADIO_PA_CFG_HPA_UPPER_LIMIT_DBM)
            {
                target_pwr = LR11XX_RADIO_PA_CFG_HPA_UPPER_LIMIT_DBM;
            }

            /* Search for the closest optimized config - configure as per AN5457 */
            if (target_pwr <= LR11XX_RADIO_PA_CFG_HPA_OPTIMIZATION_14_DBM)
            {
                /* +14dBm optimization */
                drv_ctx.pa_cfg.pa_cfg.pa_duty_cycle = 0x01u;
                drv_ctx.pa_cfg.pa_cfg.pa_hp_sel     = 0x03u;
                drv_ctx.pa_cfg.pa_cfg.pa_reg_supply = 0x01u;
                drv_ctx.pa_cfg.tx_power_reg  = LR11XX_RADIO_PA_CFG_HPA_UPPER_LIMIT_DBM - (LR11XX_RADIO_PA_CFG_HPA_OPTIMIZATION_14_DBM - target_pwr);
            }
            else if (target_pwr <= LR11XX_RADIO_PA_CFG_HPA_OPTIMIZATION_17_DBM)
            {
                /* +17dBm optimization */
                drv_ctx.pa_cfg.pa_cfg.pa_duty_cycle = 0x01u;
                drv_ctx.pa_cfg.pa_cfg.pa_hp_sel     = 0x05u;
                drv_ctx.pa_cfg.pa_cfg.pa_reg_supply = 0x01u;
                drv_ctx.pa_cfg.tx_power_reg  = LR11XX_RADIO_PA_CFG_HPA_UPPER_LIMIT_DBM - (LR11XX_RADIO_PA_CFG_HPA_OPTIMIZATION_17_DBM - target_pwr);
            }
            else if (target_pwr <= LR11XX_RADIO_PA_CFG_HPA_OPTIMIZATION_20_DBM)
            {
                /* +20dBm optimization */
                drv_ctx.pa_cfg.pa_cfg.pa_duty_cycle = 0x03u;
                drv_ctx.pa_cfg.pa_cfg.pa_hp_sel     = 0x07u;
                drv_ctx.pa_cfg.pa_cfg.pa_reg_supply = 0x01u;
                drv_ctx.pa_cfg.tx_power_reg  = LR11XX_RADIO_PA_CFG_HPA_UPPER_LIMIT_DBM - (LR11XX_RADIO_PA_CFG_HPA_OPTIMIZATION_20_DBM - target_pwr);
            }
            else
            {
                /* +22dBm optimization */
                drv_ctx.pa_cfg.pa_cfg.pa_duty_cycle = 0x04u;
                drv_ctx.pa_cfg.pa_cfg.pa_hp_sel     = 0x07u;
                drv_ctx.pa_cfg.pa_cfg.pa_reg_supply = 0x01u;
                drv_ctx.pa_cfg.tx_power_reg  = target_pwr;
            }

            /* Saturate by lower limit for HPA */
            if (drv_ctx.pa_cfg.tx_power_reg < LR11XX_RADIO_PA_CFG_HPA_LOWER_LIMIT_DBM)
            {
                drv_ctx.pa_cfg.tx_power_reg = LR11XX_RADIO_PA_CFG_HPA_LOWER_LIMIT_DBM;
            }
        }
        else
        {
            /* Saturate by upper limit for LPA */
            if (target_pwr > LR11XX_RADIO_PA_CFG_LPA_UPPER_LIMIT_DBM)
            {
                target_pwr = LR11XX_RADIO_PA_CFG_LPA_UPPER_LIMIT_DBM;
            }

            /* Search for the closest optimized config - configure as per AN5457 */
            if (target_pwr <= LR11XX_RADIO_PA_CFG_LPA_OPTIMIZATION_10_DBM)
            {
                /* +10dBm optimization */
                drv_ctx.pa_cfg.pa_cfg.pa_duty_cycle = 0x00u;
                drv_ctx.pa_cfg.pa_cfg.pa_hp_sel     = 0x00u;
                drv_ctx.pa_cfg.pa_cfg.pa_reg_supply = 0x00u;
                drv_ctx.pa_cfg.tx_power_reg  = LR11XX_RADIO_PA_CFG_LPA_OPTIMIZATION_14_DBM - (LR11XX_RADIO_PA_CFG_LPA_OPTIMIZATION_10_DBM - target_pwr);
            }
            else if (target_pwr <= LR11XX_RADIO_PA_CFG_LPA_OPTIMIZATION_14_DBM)
            {
                /* +14dBm optimization */
                drv_ctx.pa_cfg.pa_cfg.pa_duty_cycle = 0x04u;
                drv_ctx.pa_cfg.pa_cfg.pa_hp_sel     = 0x00u;
                drv_ctx.pa_cfg.pa_cfg.pa_reg_supply = 0x00u;
                drv_ctx.pa_cfg.tx_power_reg  = target_pwr;
            }
            else
            {
                /* +15dBm optimization */
                drv_ctx.pa_cfg.pa_cfg.pa_duty_cycle = 0x07u;
                drv_ctx.pa_cfg.pa_cfg.pa_hp_sel     = 0x00u;
                drv_ctx.pa_cfg.pa_cfg.pa_reg_supply = 0x00u;
                drv_ctx.pa_cfg.tx_power_reg  = LR11XX_RADIO_PA_CFG_LPA_OPTIMIZATION_14_DBM - (LR11XX_RADIO_PA_CFG_LPA_UPPER_LIMIT_DBM - target_pwr);
            }

            /* Saturate by lower limit for HPA */
            if (drv_ctx.pa_cfg.tx_power_reg < LR11XX_RADIO_PA_CFG_LPA_LOWER_LIMIT_DBM)
            {
                drv_ctx.pa_cfg.tx_power_reg = LR11XX_RADIO_PA_CFG_LPA_LOWER_LIMIT_DBM;
            }
        }
    }
    else if( (drv_ctx.config->enable_hpa == false) && (drv_ctx.config->enable_lpa == false) )
    {
        return RADIO_ERROR_INVALID_PARAMS;
    }

    /* Check if the callback specified the ramp up time */
    if (LR11XX_RADIO_PA_CFG_PARAM_INVALID == drv_ctx.pa_cfg.ramp_time)
    {
        /* Use the default value */
        drv_ctx.pa_cfg.ramp_time = LR11XX_RADIO_RAMP_48_US;
    }

    return RADIO_ERROR_NONE;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED void lr11xx_radio_set_lora_exit_mode(sid_pal_radio_cad_param_exit_mode_t cad_exit_mode)
{
    drv_ctx.cad_exit_mode = cad_exit_mode;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t radio_lr11xx_set_subghz_fem_mode(const lr11xx_subghz_fe_mode_t frontend_mode)
{
    int32_t err = RADIO_ERROR_NONE;
    uint8_t fem_power_en = 0u;
    uint8_t fem_tx_mode_sel = 0u;
    uint8_t hpa_lpa_switch = 0u;
    uint8_t fem_bypass = 0u;

    do
    {
        switch (frontend_mode)
        {
            case LR11XX_RADIO_FRONTEND_MODE_OFF:
                fem_power_en = drv_ctx.config->gpio.fem_power_en_gpio_state != 0u ? 0u : 1u;
                fem_tx_mode_sel = drv_ctx.config->gpio.fem_tx_mode_sel_gpio_state != 0u ? 0u : 1u;
                hpa_lpa_switch = drv_ctx.config->gpio.hpa_lpa_sw_on_gpio_state != 0u ? 0u : 1u;
                fem_bypass = drv_ctx.config->gpio.fem_bypass_en_gpio_state != 0u ? 0u : 1u;
                break;

            case LR11XX_RADIO_FRONTEND_MODE_RX:
                fem_power_en = drv_ctx.config->gpio.fem_power_en_gpio_state != 0u ? 1u : 0u;
                fem_tx_mode_sel = drv_ctx.config->gpio.fem_tx_mode_sel_gpio_state != 0u ? 0u : 1u;
                hpa_lpa_switch = drv_ctx.config->gpio.hpa_lpa_sw_on_gpio_state != 0u ? 0u : 1u;
                fem_bypass = drv_ctx.config->gpio.fem_bypass_en_gpio_state != 0u ? 0u : 1u;
                break;

            case LR11XX_RADIO_FRONTEND_MODE_TX:
                fem_power_en = drv_ctx.config->gpio.fem_power_en_gpio_state != 0u ? 1u : 0u;
                fem_tx_mode_sel = drv_ctx.config->gpio.fem_tx_mode_sel_gpio_state != 0u ? 1u : 0u;
                if(LR11XX_RADIO_PA_SEL_HP == drv_ctx.pa_cfg.pa_cfg.pa_sel)
                {
                    /* Enable HPA */
                    hpa_lpa_switch = drv_ctx.config->gpio.hpa_lpa_sw_on_gpio_state != 0u ? 1u : 0u;
                }
                else if(LR11XX_RADIO_PA_SEL_LP == drv_ctx.pa_cfg.pa_cfg.pa_sel)
                {
                    /* Disable HPA */
                    hpa_lpa_switch = drv_ctx.config->gpio.hpa_lpa_sw_on_gpio_state != 0u ? 0u : 1u;
                }
#if LR11XX_RADIO_CFG_USE_EXTERNAL_PA && LR11XX_RADIO_CFG_USE_FEM_BYPASS
                if(drv_ctx.pa_cfg.enable_ext_pa == FALSE)
                {
                    /* If EXT PA disabled, enable tx bypass */
                    fem_bypass = drv_ctx.config->gpio.fem_bypass_en_gpio_state != 0u ? 1u : 0u;
                }
                else
                {
                    /* Otherwise disable tx bypass */
                    fem_bypass = drv_ctx.config->gpio.fem_bypass_en_gpio_state != 0u ? 0u : 1u;
                }
#endif /* LR11XX_RADIO_CFG_USE_EXTERNAL_PA && LR11XX_RADIO_CFG_USE_FEM_BYPASS */
                break;

            default:
                fem_power_en = drv_ctx.config->gpio.fem_power_en_gpio_state != 0u ? 0u : 1u;
                fem_tx_mode_sel = drv_ctx.config->gpio.fem_tx_mode_sel_gpio_state != 0u ? 0u : 1u;
                hpa_lpa_switch = drv_ctx.config->gpio.hpa_lpa_sw_on_gpio_state != 0u ? 0u : 1u;
                fem_bypass = drv_ctx.config->gpio.fem_bypass_en_gpio_state != 0u ? 0u : 1u;
                SID_PAL_LOG_ERROR("radio_lr11xx_set_subghz_fem_mode invalid params");
                return RADIO_ERROR_INVALID_PARAMS;
        }

#if LR11XX_RADIO_CFG_USE_FEM_POW_ENA
        if (drv_ctx.config->gpio.fem_power != HALO_GPIO_NOT_CONNECTED)
        {
            if (sid_pal_gpio_write(drv_ctx.config->gpio.fem_power, fem_power_en) != SID_ERROR_NONE)
            {
                err = RADIO_ERROR_IO_ERROR;
                break;
            }
        }
#else
    (void) fem_power_en;
#endif /* LR11XX_RADIO_CFG_USE_FEM_POW_ENA */

#if LR11XX_RADIO_CFG_USE_EXTERNAL_PA && LR11XX_RADIO_CFG_USE_FEM_BYPASS
        if (drv_ctx.config->gpio.fem_bypass != HALO_GPIO_NOT_CONNECTED)
        {
            if (sid_pal_gpio_write(drv_ctx.config->gpio.fem_bypass, fem_bypass) != SID_ERROR_NONE)
            {
                err = RADIO_ERROR_IO_ERROR;
                break;
            }
        }
#else
    (void) fem_bypass;
#endif /* LR11XX_RADIO_CFG_USE_EXTERNAL_PA && LR11XX_RADIO_CFG_USE_FEM_BYPASS */

#if LR11XX_RADIO_CFG_USE_TX_RX_SW
        if (drv_ctx.config->gpio.fem_tx_rx_mode != HALO_GPIO_NOT_CONNECTED)
        {
            if (sid_pal_gpio_write(drv_ctx.config->gpio.fem_tx_rx_mode, fem_tx_mode_sel) != SID_ERROR_NONE)
            {
                err = RADIO_ERROR_IO_ERROR;
                break;
            }
        }
#else
    (void) fem_tx_mode_sel;
#endif /* LR11XX_RADIO_CFG_USE_TX_RX_SW */

#if LR11XX_RADIO_CFG_USE_HPA_LPA_SW
        if (drv_ctx.config->gpio.hpa_lpa_sw != HALO_GPIO_NOT_CONNECTED)
        {
            if (sid_pal_gpio_write(drv_ctx.config->gpio.hpa_lpa_sw, hpa_lpa_switch) != SID_ERROR_NONE)
            {
                err = RADIO_ERROR_IO_ERROR;
                break;
            }
        }
#else
    (void) hpa_lpa_switch;
#endif /* LR11XX_RADIO_CFG_USE_HPA_LPA_SW */
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

#if LR11XX_RADIO_CFG_GEOLOCATION_SUPPORT
SID_STM32_SPEED_OPTIMIZED int32_t radio_lr11xx_set_gnss_fem_mode(const lr11xx_gnss_fe_mode_t frontend_mode)
{
    int32_t err;
    sid_error_t sid_err;

    do
    {
        if (drv_ctx.config->gpio.gnss_lna_ctrl != HALO_GPIO_NOT_CONNECTED)
        {
            uint8_t gnss_lna_en;

            switch (frontend_mode)
            {
                case LR11XX_GNSS_FRONTEND_MODE_OFF:
                    gnss_lna_en = drv_ctx.config->gpio.gnss_lna_ctrl_en_gpio_state != 0u ? 0u : 1u;
                    break;

                case LR11XX_GNSS_FRONTEND_MODE_ON:
                    gnss_lna_en = drv_ctx.config->gpio.gnss_lna_ctrl_en_gpio_state != 0u ? 1u : 0u;
                    break;

                default:
                    gnss_lna_en = drv_ctx.config->gpio.gnss_lna_ctrl_en_gpio_state != 0u ? 0u : 1u;
                    SID_PAL_LOG_ERROR("radio_lr11xx_set_gnss_fem_mode invalid params");
                    return RADIO_ERROR_INVALID_PARAMS;
                    break;
            }

            sid_err = sid_pal_gpio_write(drv_ctx.config->gpio.gnss_lna_ctrl, gnss_lna_en);
            if (sid_err != SID_ERROR_NONE)
            {
                err = RADIO_ERROR_IO_ERROR;
                break;
            }
        }

        /* Done */
        err = RADIO_ERROR_NONE;
    } while (0);

    return err;
}
#endif /* LR11XX_RADIO_CFG_GEOLOCATION_SUPPORT */

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED void lr11xx_radio_set_device_config(const lr11xx_radio_device_config_t * const cfg)
{
    drv_ctx.config = cfg;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED uint8_t sid_pal_radio_get_status(void)
{
    uint8_t state_to_report;

    sid_pal_enter_critical_region();

#if LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION && LR11XX_RADIO_CFG_LBM_CONCURRENCY_SUPPORT
    if (RADIO_LR11XX_LBM_BRIDGE_STATE_SIDEWALK_BLANKING_MODE == drv_ctx.lbm.bridge_state)
    {
        state_to_report = drv_ctx.lbm.simulated_radio_state;
    }
    else
#endif /* LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION && LR11XX_RADIO_CFG_LBM_CONCURRENCY_SUPPORT */
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

#if LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION
    if (RADIO_LR11XX_LBM_BRIDGE_STATE_EXCLUSIVE_MODE == drv_ctx.lbm.bridge_state)
    {
        /* Don't allow Sidewalk operations */
        return RADIO_ERROR_BUSY;
    }
#  if LR11XX_RADIO_CFG_LBM_CONCURRENCY_SUPPORT
    else if (RADIO_LR11XX_LBM_BRIDGE_STATE_SIDEWALK_BLANKING_MODE == drv_ctx.lbm.bridge_state)
    {
        /* Report success to Sidewalk without doing anything */
        drv_ctx.modem = mode;
        return RADIO_ERROR_NONE;
    }
#  endif /* LR11XX_RADIO_CFG_LBM_CONCURRENCY_SUPPORT */
#endif /* LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION */

    switch(mode)
    {
        case SID_PAL_RADIO_MODEM_MODE_LORA:
            err = sid_pal_radio_set_modem_to_lora_mode();
            mode_str = "LoRa";
            break;

        case SID_PAL_RADIO_MODEM_MODE_FSK:
            err = sid_pal_radio_set_modem_to_fsk_mode();
            mode_str = "FSK";
            break;

        default:
            err = RADIO_ERROR_INVALID_PARAMS;
            mode_str = "Undefined";
            break;
    }

    if (RADIO_ERROR_NONE == err)
    {
        if (drv_ctx.modem != mode_on_entry)
        {
            SID_PAL_LOG_INFO("LR11xx modem set to %s mode", mode_str);
        }
    }
    else
    {
        SID_PAL_LOG_ERROR("Failed to set LR11xx modem mode to %s. Error %d", mode_str, err);
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_irq_process(void)
{
    int32_t                  err            = RADIO_ERROR_GENERIC;
    lr11xx_hal_status_t      hal_err;
    lr11xx_status_t          sys_err;
    sid_pal_radio_events_t   radio_event    = SID_PAL_RADIO_EVENT_UNKNOWN;
    lr11xx_system_irq_mask_t reported_irqs;
    lr11xx_system_irq_mask_t remaining_irqs = LR11XX_SYSTEM_IRQ_NONE;

    do
    {
        if (SID_PAL_RADIO_SLEEP == drv_ctx.radio_state)
        {
            /* This may happen if radio IRQ was pre-empted by timeout monitoring software timer */
            LR11XX_RADIO_LOG_DEBUG("Radio IRQ detected while LR11xx is in Sleep - ignored");
            err = RADIO_ERROR_INVALID_STATE;
            break;
        }

        /* Read out IRQ status and clear IRQ flags */
        sys_err = lr11xx_system_get_and_clear_irq_status(&drv_ctx, &reported_irqs);
        if (sys_err != LR11XX_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to read out LR11xx IRQ status. Error %d", (int32_t)sys_err);
            err =  RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Store reported IRQs for diagnostic purposes */
        /* LR11xx sets IRQ status flags whenever the related event occurs, regardless of the IRQ mask settings.
         * IRQ mask just disables IRQ indication via a GPIO pin. This means we need to clean up the status
         * reported by lr11xx_system_get_and_clear_irq_status() from any irrelevant flags
         */
        reported_irqs = (reported_irqs & drv_ctx.irq_mask);
        if (LR11XX_SYSTEM_IRQ_NONE == reported_irqs)
        {
            /* All IRQs are masked out */
            err = RADIO_ERROR_NONE;
            break;
        }

        /* Store the list of unhandled IRQs for diagnostic purposes */
        remaining_irqs = reported_irqs;

        /* Error IRQ -----------------------------------------------------------------*/
        if ((reported_irqs & LR11XX_SYSTEM_IRQ_ERROR) != LR11XX_SYSTEM_IRQ_NONE)
        {
            lr11xx_system_errors_t errors;

            sys_err = lr11xx_system_get_errors(&drv_ctx, &errors);
            if (LR11XX_STATUS_OK == sys_err)
            {
                SID_PAL_LOG_WARNING("LR11xx: Errors 0x%04X", errors);
            }
            (void)lr11xx_system_clear_errors(&drv_ctx);

            remaining_irqs &= ~LR11XX_SYSTEM_IRQ_ERROR;
            err = RADIO_ERROR_NONE;

            /* Proceed with processing */
        }
        /*----------------------------------------------------------------------------*/

        /* Handle Tx Done IRQ --------------------------------------------------------*/
        if ((reported_irqs & LR11XX_SYSTEM_IRQ_TX_DONE) != LR11XX_SYSTEM_IRQ_NONE)
        {
            LR11XX_RADIO_LOG_DEBUG("LR11XX_SYSTEM_IRQ_TX_DONE");

            /* Radio automatically enters Standby state after Tx/Rx completion or error (e.g. timeout) */
            drv_ctx.radio_state = SID_PAL_RADIO_STANDBY_XOSC;
            /* Turn off FEM as we don't need it any longer */
            (void)radio_lr11xx_set_subghz_fem_mode(LR11XX_RADIO_FRONTEND_MODE_OFF);

            remaining_irqs &= ~LR11XX_SYSTEM_IRQ_TX_DONE;
            radio_event = SID_PAL_RADIO_EVENT_TX_DONE;
            err = RADIO_ERROR_NONE;
            break;
        }
        /*----------------------------------------------------------------------------*/

        /* Sync Word Valid IRQ -------------------------------------------------------*/
        if ((reported_irqs & LR11XX_SYSTEM_IRQ_SYNC_WORD_HEADER_VALID) != LR11XX_SYSTEM_IRQ_NONE)
        {
            LR11XX_RADIO_LOG_DEBUG("LR11XX_SYSTEM_IRQ_SYNC_WORD_HEADER_VALID");

            const uint32_t rx_done = (reported_irqs & LR11XX_SYSTEM_IRQ_RX_DONE) != LR11XX_SYSTEM_IRQ_NONE ? TRUE : FALSE;

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
                LR11XX_RADIO_LOG_ERROR("Sync Word Valid IRQ is expected only for FSK mode, but modem is in %u mode", (uint32_t)drv_ctx.modem);
                err = RADIO_ERROR_INVALID_STATE;
                break;
            }

            remaining_irqs &= ~LR11XX_SYSTEM_IRQ_SYNC_WORD_HEADER_VALID;

            /* Terminate IRQ processing if no more IRQs are indicated, otherwise proceed (e.g. with Preamble Detected, Rx Done, etc.) */
            if (LR11XX_SYSTEM_IRQ_NONE == remaining_irqs)
            {
                break;
            }
        }
        /*----------------------------------------------------------------------------*/

        /* Preamble Detect IRQ -------------------------------------------------------*/
        if ((reported_irqs & LR11XX_SYSTEM_IRQ_PREAMBLE_DETECTED) != LR11XX_SYSTEM_IRQ_NONE)
        {
            LR11XX_RADIO_LOG_DEBUG("LR11XX_SYSTEM_IRQ_PREAMBLE_DETECTED");

#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_2
            if (SID_PAL_RADIO_MODEM_MODE_FSK == drv_ctx.modem)
            {
                /* Restore the default IRQ mask for FSK */
                if (drv_ctx.cad_exit_mode != SID_PAL_RADIO_CAD_EXIT_MODE_NONE)
                {
                    err = radio_set_irq_mask(LR11XX_DEFAULT_FSK_IRQ_MASK);
                    if (err != RADIO_ERROR_NONE)
                    {
                        LR11XX_RADIO_LOG_ERROR("Failed to set IRQ mask for FSK mode");
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
                LR11XX_RADIO_LOG_ERROR("Preamble Detect IRQ is expected only for FSK mode, but modem is in %u mode", (uint32_t)drv_ctx.modem);
                err = RADIO_ERROR_INVALID_STATE;
                break;
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 */
            }

            remaining_irqs &= ~LR11XX_SYSTEM_IRQ_PREAMBLE_DETECTED;

            /* Terminate IRQ processing if no more IRQs are indicated, otherwise proceed (e.g. with Rx Done, etc.) */
            if (LR11XX_SYSTEM_IRQ_NONE == remaining_irqs)
            {
                break;
            }
        }
        /*----------------------------------------------------------------------------*/

        /* CAD Done IRQ --------------------------------------------------------------*/
        if ((reported_irqs & LR11XX_SYSTEM_IRQ_CAD_DONE) != LR11XX_SYSTEM_IRQ_NONE)
        {
            if (SID_PAL_RADIO_MODEM_MODE_LORA == drv_ctx.modem)
            {
                radio_event = ((reported_irqs & LR11XX_SYSTEM_IRQ_CAD_DETECTED) != LR11XX_SYSTEM_IRQ_NONE) ? SID_PAL_RADIO_EVENT_CAD_DONE : SID_PAL_RADIO_EVENT_CAD_TIMEOUT;

                if (((SID_PAL_RADIO_CAD_EXIT_MODE_CS_LBT == drv_ctx.cad_exit_mode) || (SID_PAL_RADIO_CAD_EXIT_MODE_CS_RX == drv_ctx.cad_exit_mode))
                  && (SID_PAL_RADIO_EVENT_CAD_TIMEOUT == radio_event))
                {
                    /* LoRa CAD discovered no activities - radio channel is free, the radio proceeds with Tx or Rx automatically */
                    radio_event = SID_PAL_RADIO_EVENT_UNKNOWN;
                    drv_ctx.cad_exit_mode = SID_PAL_RADIO_CAD_EXIT_MODE_NONE;

                    if (SID_PAL_RADIO_CAD_EXIT_MODE_CS_LBT == drv_ctx.cad_exit_mode)
                    {
                        /* Configure FEM for Tx */
                        err = radio_lr11xx_set_subghz_fem_mode(LR11XX_RADIO_FRONTEND_MODE_TX);
                        if (err != RADIO_ERROR_NONE)
                        {
                            LR11XX_RADIO_LOG_ERROR("Failed to configure FEM for Tx. Error %d", err);
                            break;
                        }

                        /* Update status in the driver */
                        drv_ctx.radio_state = SID_PAL_RADIO_TX;
                        __COMPILER_BARRIER();

#if LR11XX_RADIO_CFG_USE_STATUS_LED
                        /* Drive the status LED */
                        lr11xx_radio_hal_tx_led_on(&drv_ctx);
#endif /* LR11XX_RADIO_CFG_USE_STATUS_LED */
                    }
                    else
                    {
                        /* FEM is already configured for Rx, Rx LED is active */
                        err = RADIO_ERROR_NONE;

                        /* Update status in the driver */
                        drv_ctx.radio_state = SID_PAL_RADIO_RX;
                        __COMPILER_BARRIER();
                    }
                }
                else
                {
                    /* Put the radio to Standby - by defaut CAD moves out to STDBY_RC, but the driver uses STDBY_XOSC */
                    err = sid_pal_radio_standby();
                    if (err != RADIO_ERROR_NONE)
                    {
                        SID_PAL_LOG_ERROR("Failed to put radio to Standby after CAD. Error %d", err);
                        break;
                    }
                }
            }
            else
            {
                LR11XX_RADIO_LOG_ERROR("CAD Done IRQ is expected only for LoRa mode, but modem is in %u mode", (uint32_t)drv_ctx.modem);
                err = RADIO_ERROR_INVALID_STATE;
                break;
            }

            /* CAD Done IRQ is processed successfully */
            remaining_irqs &= ~(LR11XX_SYSTEM_IRQ_CAD_DONE | LR11XX_SYSTEM_IRQ_CAD_DETECTED);
            break;
        }
        /*----------------------------------------------------------------------------*/

        /* CRC Error IRQ -------------------------------------------------------------*/
        if ((reported_irqs & LR11XX_SYSTEM_IRQ_CRC_ERROR) != LR11XX_SYSTEM_IRQ_NONE)
        {
            LR11XX_RADIO_LOG_DEBUG("LR11XX_SYSTEM_IRQ_CRC_ERROR");;
            remaining_irqs &= ~(LR11XX_SYSTEM_IRQ_CRC_ERROR | LR11XX_SYSTEM_IRQ_RX_DONE);
            radio_event = SID_PAL_RADIO_EVENT_RX_ERROR;
            err = RADIO_ERROR_NONE;
            break;
        }
        /*----------------------------------------------------------------------------*/

        /* Header Error IRQ ----------------------------------------------------------*/
        if ((reported_irqs & LR11XX_SYSTEM_IRQ_HEADER_ERROR) != LR11XX_SYSTEM_IRQ_NONE)
        {
            LR11XX_RADIO_LOG_DEBUG("LR11XX_SYSTEM_IRQ_HEADER_ERROR");
            err = RADIO_ERROR_NONE;
            radio_event = SID_PAL_RADIO_EVENT_RX_TIMEOUT;
            remaining_irqs &= ~LR11XX_SYSTEM_IRQ_HEADER_ERROR;
            break;
        }
        /*----------------------------------------------------------------------------*/

        /* Rx Done IRQ ---------------------------------------------------------------*/
        if ((reported_irqs & LR11XX_SYSTEM_IRQ_RX_DONE) != LR11XX_SYSTEM_IRQ_NONE)
        {
            LR11XX_RADIO_LOG_DEBUG("LR11XX_SYSTEM_IRQ_RX_DONE");

            /* Radio automatically enters Standby state after Tx/Rx completion or error (e.g. timeout) */
            drv_ctx.radio_state = SID_PAL_RADIO_STANDBY_XOSC;
            /* Turn off FEM as we don't need it any longer */
            (void)radio_lr11xx_set_subghz_fem_mode(LR11XX_RADIO_FRONTEND_MODE_OFF);

            if (SID_PAL_RADIO_MODEM_MODE_FSK == drv_ctx.modem)
            {
                radio_fsk_rx_done_status_t fsk_rx_done_status;

                err = radio_fsk_process_rx_done(&drv_ctx, &fsk_rx_done_status);
                if (err != RADIO_ERROR_NONE)
                {
                    LR11XX_RADIO_LOG_ERROR("FSK Rx completed with error. Error: %d, rx status: %u", err, (uint32_t)fsk_rx_done_status);
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
                    LR11XX_RADIO_LOG_ERROR("LoRa Rx completed with error %d", err);
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
            remaining_irqs &= ~LR11XX_SYSTEM_IRQ_RX_DONE;
            break;
        }
        /*----------------------------------------------------------------------------*/

        /* If we've got here it means none of the above blocks took care of the IRQ. Not an error, but... */
        SID_PAL_LOG_WARNING("Unexpected SubGHz IRQ detected: 0x%08X. No handlers assigned to it. All reported: 0x%08X", remaining_irqs, reported_irqs);
        err = RADIO_ERROR_NONE;
    } while (0);

    /* Report the event to the Sidewalk stack */
    if (RADIO_ERROR_NONE == err)
    {
        if (remaining_irqs != LR11XX_SYSTEM_IRQ_NONE)
        {
            SID_PAL_LOG_WARNING("Unhandled SubGHz IRQs identified. Reported: 0x%08X, unhandled: 0x%08X", reported_irqs, remaining_irqs);
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

#if LR11XX_RADIO_CFG_USE_STATUS_LED
            /* Turn off activity LEDs - any event means activities are finished */
            lr11xx_radio_hal_tx_led_off(&drv_ctx);
            lr11xx_radio_hal_rx_led_off(&drv_ctx);
#endif /* LR11XX_RADIO_CFG_USE_STATUS_LED */

            /* Notify the radio about the event - make sure this is the last action after all GPIO handling and LPM management since drv_ctx.report_radio_event() may modify GPIO and LPM states */
            drv_ctx.report_radio_event(radio_event);
        }
    }

    /* Re-enable radio IRQ detection if the radio is not in Sleep state - this check is mandatory because drv_ctx.report_radio_event() may change the state of the radio */
    if (drv_ctx.radio_state != SID_PAL_RADIO_SLEEP)
    {
        hal_err = lr11xx_hal_arm_irq(&drv_ctx);
        if (hal_err != LR11XX_HAL_STATUS_OK)
        {
            /* Logs are provided by the lr11xx_hal_arm_irq() */
            err = RADIO_ERROR_HARDWARE_ERROR;
        }
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_set_frequency(uint32_t freq)
{
    int32_t err = RADIO_ERROR_NONE;
    lr11xx_status_t sys_err;

    do
    {
#if LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION
        if (RADIO_LR11XX_LBM_BRIDGE_STATE_EXCLUSIVE_MODE == drv_ctx.lbm.bridge_state)
        {
            /* Don't allow Sidewalk operations */
            err = RADIO_ERROR_BUSY;
            break;
        }
#  if LR11XX_RADIO_CFG_LBM_CONCURRENCY_SUPPORT
        else if (RADIO_LR11XX_LBM_BRIDGE_STATE_SIDEWALK_BLANKING_MODE == drv_ctx.lbm.bridge_state)
        {
            /* Report success to Sidewalk without doing anything */
            err = RADIO_ERROR_NONE;
            break;
        }
#  endif /* LR11XX_RADIO_CFG_LBM_CONCURRENCY_SUPPORT */
#endif /* LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION */

        if (drv_ctx.radio_state != SID_PAL_RADIO_STANDBY_XOSC)
        {
            err =  RADIO_ERROR_INVALID_STATE;
            break;
        }

        const lr11xx_freq_band_t * const freq_band = lr11xx_get_freq_band(freq);

        if (freq_band == NULL)
        {
            err = RADIO_ERROR_INVALID_PARAMS;
            break;
        }

        /* Image calibration is required if frequency band has changed */
        if (drv_ctx.radio_freq_band != freq_band)
        {
            sys_err = lr11xx_system_calibrate_image(&drv_ctx,
                                                    (freq_band->start + (LR11XX_RADIO_FREQ_STEP / 2u)) / LR11XX_RADIO_FREQ_STEP,
                                                    (freq_band->stop  + (LR11XX_RADIO_FREQ_STEP / 2u)) / LR11XX_RADIO_FREQ_STEP);
            if (sys_err != LR11XX_STATUS_OK)
            {
                err = RADIO_ERROR_HARDWARE_ERROR;
                break;
            }

            SID_PAL_LOG_INFO("SubGHz frequency band set to %u-%u MHz", freq_band->start / 1000000u, freq_band->stop / 1000000u);
        }

#if RADIO_LR11XX_TXPWR_WORKAROUND
        if ((LR11XX_BAND_EDGE_BAND_START_FREQ == freq_band->start)
            && (SID_PAL_RADIO_MODEM_MODE_LORA == drv_ctx.modem))
        {
            uint32_t reg_val;
            if (freq <= LR11XX_BAND_EDGE_LIMIT_FREQ)
            {
                reg_val = LR11XX_REG_VALUE_BAND_EDGE_FIX_LO;
            }
            else
            {
                reg_val = LR11XX_REG_VALUE_BAND_EDGE_FIX_HI;
            }

            sys_err = lr11xx_regmem_write_regmem32_mask(&drv_ctx, LR11XX_REG_ADDR_BAND_EDGE_FIX, LR11XX_REG_MASK_BAND_EDGE_FIX, reg_val);
            if (sys_err != LR11XX_STATUS_OK)
            {
                SID_PAL_LOG_WARNING("LR11xx: Band edge fix was not applied.");
            }
        }
#endif /* RADIO_LR11XX_TXPWR_WORKAROUND */

        sys_err = lr11xx_radio_set_rf_freq(&drv_ctx, freq);
        if (sys_err != LR11XX_STATUS_OK)
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
SID_STM32_SPEED_OPTIMIZED int32_t semtech_radio_set_sx126x_pa_config(semtech_radio_pa_cfg_t *cfg)
{
    int32_t err;

    do
    {
        if (NULL == cfg)
        {
            err = RADIO_ERROR_INVALID_PARAMS;
            break;
        }

        radio_lr11xx_pa_cfg_t cur_cfg;

        cur_cfg.pa_cfg.pa_duty_cycle = cfg->pa_duty_cycle;
        cur_cfg.pa_cfg.pa_hp_sel     = cfg->hp_max;
        cur_cfg.pa_cfg.pa_sel        = (lr11xx_radio_pa_selection_t)cfg->device_sel;
        cur_cfg.pa_cfg.pa_reg_supply = (lr11xx_radio_pa_reg_supply_t)cfg->pa_lut;
        cur_cfg.target_tx_power      = cfg->tx_power;
        cur_cfg.ramp_time            = (lr11xx_radio_ramp_time_t)cfg->ramp_time;
#if LR11XX_RADIO_CFG_USE_EXTERNAL_PA && LR11XX_RADIO_CFG_USE_FEM_BYPASS
        cur_cfg.enable_ext_pa        = cfg->enable_ext_pa;
#endif

        drv_ctx.pa_cfg          = cur_cfg;
        drv_ctx.pa_cfg_override = TRUE;
err = RADIO_ERROR_NONE;
    } while (0);

    return err;
}
#endif /* HALO_ENABLE_DIAGNOSTICS */

/*----------------------------------------------------------------------------*/

#if HALO_ENABLE_DIAGNOSTICS
SID_STM32_SPEED_OPTIMIZED int32_t get_radio_lr11xx_pa_config(radio_lr11xx_pa_cfg_t * cfg)
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

int32_t sid_pal_radio_set_tx_power(int8_t power)
{
    int32_t err = RADIO_ERROR_NONE;

#if LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION
    if (RADIO_LR11XX_LBM_BRIDGE_STATE_EXCLUSIVE_MODE == drv_ctx.lbm.bridge_state)
    {
        /* Don't allow Sidewalk operations */
        return RADIO_ERROR_BUSY;
    }
#  if LR11XX_RADIO_CFG_LBM_CONCURRENCY_SUPPORT
    else if (RADIO_LR11XX_LBM_BRIDGE_STATE_SIDEWALK_BLANKING_MODE == drv_ctx.lbm.bridge_state)
    {
        /* Report success to Sidewalk without doing anything */
        return RADIO_ERROR_NONE;
    }
#  endif /* LR11XX_RADIO_CFG_LBM_CONCURRENCY_SUPPORT */
#endif /* LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION */

    if (drv_ctx.radio_state != SID_PAL_RADIO_STANDBY_XOSC)
    {
        return RADIO_ERROR_INVALID_STATE;
    }

#if HALO_ENABLE_DIAGNOSTICS
    if (FALSE == drv_ctx.pa_cfg_override)
#endif /* HALO_ENABLE_DIAGNOSTICS */
    {
        /* Process config update */
        err = radio_lr11xx_compute_tx_power_config(power);
        if (err != RADIO_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to set LR11xx SubGHz Tx power to %ddB. Error %d", power, err);
            return RADIO_ERROR_INVALID_PARAMS;
        }
    }

    // TODO: Add over current config when the API is added to the Driver

    if (RADIO_ERROR_NONE == err)
    {
        /* Set new config in the radio */
        do {
            lr11xx_status_t sys_err;

            sys_err = lr11xx_radio_set_pa_cfg(&drv_ctx, (const lr11xx_radio_pa_cfg_t*)&drv_ctx.pa_cfg.pa_cfg);
            if (sys_err != LR11XX_STATUS_OK)
            {
                err = RADIO_ERROR_HARDWARE_ERROR;
                break;
            }

            sys_err = lr11xx_radio_set_tx_params(&drv_ctx, drv_ctx.pa_cfg.tx_power_reg, drv_ctx.pa_cfg.ramp_time);
            if (sys_err != LR11XX_STATUS_OK)
            {
                err = RADIO_ERROR_HARDWARE_ERROR;
                break;
            }

            /* Done */
            err = RADIO_ERROR_NONE;
        } while (0);

        err = RADIO_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_hal_set_sleep_start_notify_cb(sid_pal_radio_sleep_start_notify_handler_t callback)
{
    drv_ctx.sleep_start_notify_cb = callback;
    return RADIO_ERROR_NONE;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_sleep(uint32_t sleep_us)
{
    int32_t err;
    lr11xx_hal_status_t hal_err;
    lr11xx_status_t sys_err;

    do
    {
        /* Immediately stop software timer for Tx/Rx/CS/CAD timeout */
        (void)sid_pal_timer_cancel(&drv_ctx.radio_timeout_mon);
        __COMPILER_BARRIER();

#if LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION
        if (RADIO_LR11XX_LBM_BRIDGE_STATE_EXCLUSIVE_MODE == drv_ctx.lbm.bridge_state)
        {
            /* Don't allow Sidewalk operations */
            return RADIO_ERROR_BUSY;
        }
#  if LR11XX_RADIO_CFG_LBM_CONCURRENCY_SUPPORT
        else if (RADIO_LR11XX_LBM_BRIDGE_STATE_SIDEWALK_BLANKING_MODE == drv_ctx.lbm.bridge_state)
        {
            /* Report success to Sidewalk without doing anything */
            drv_ctx.lbm.simulated_radio_state = SID_PAL_RADIO_SLEEP;
            return RADIO_ERROR_NONE;
        }
#  endif /* LR11XX_RADIO_CFG_LBM_CONCURRENCY_SUPPORT */
#endif /* LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION */

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
            break;
        }

        /**
         * IMPORTANT: Radio IRQ line gets disabled from the MCU side by the LR11xx HAL layer as part of the lr11xx_system_set_sleep() call.
         *            This is a workaround for accidental spikes on the IRQ line while the radio is in sleep and does not actively drive the IRQ line.
         *            IRQ reaction is restored automatically by the LR11xx HAL layer upon calling lr11xx_system_wakeup()
         */

        /* Put the radio into warm sleep to retain the configuration */
        lr11xx_system_sleep_cfg_t cfg = {
            .is_warm_start  = true,
            .is_rtc_timeout = true,
        };

        sys_err = lr11xx_system_set_sleep(&drv_ctx, cfg, LR11XX_RADIO_INFINITE_TIME);
        if (sys_err != LR11XX_STATUS_OK)
        {
            LR11XX_RADIO_LOG_ERROR("Failed to put the radio to sleep. Error %d", (int32_t)sys_err);
            err = RADIO_ERROR_HARDWARE_ERROR;

            /* Re-enable IRQ line */
            hal_err = lr11xx_hal_arm_irq(&drv_ctx);
            if (hal_err != LR11XX_HAL_STATUS_OK)
            {
                LR11XX_RADIO_LOG_ERROR("Failed to re-arm radio IRQ on sleep entry failure. Error %d", (int32_t)hal_err);
                err = RADIO_ERROR_HARDWARE_ERROR;
            }
            break;
        }
        else
        {
            /* drv_ctx.radio_state is set to SID_PAL_RADIO_SLEEP by successful lr11xx_system_set_sleep() invocation */
        }
        __COMPILER_BARRIER();

        /* Update status in the context if sleep entry was successful */
        sid_pal_enter_critical_region();
        drv_ctx.sid_busy_indication = FALSE;
#if LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION
        /* Activate LBM bridge if requested. Always start in exclusive mode, app may adjust this to blanking mode later if necessary */
        if (RADIO_LR11XX_LBM_BRIDGE_STATE_ACTIVATION_PENDING == drv_ctx.lbm.bridge_state)
        {
            drv_ctx.lbm.bridge_state = RADIO_LR11XX_LBM_BRIDGE_STATE_EXCLUSIVE_MODE;
        }

        /* Release radio lock by Sidewalk stack */
        SID_PAL_ASSERT(drv_ctx.lbm.sidewalk_radio_access_lock != NULL);
        (void)osSemaphoreRelease(drv_ctx.lbm.sidewalk_radio_access_lock);
#endif /* LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION */
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

    if (RADIO_ERROR_NONE == err)
    {
        /* Use critical section to avoid drv_ctx.sleep_start_notify_cb updates mid-flight */
        sid_pal_enter_critical_region();
        if ((drv_ctx.sleep_start_notify_cb != NULL) && (sleep_us != UINT32_MAX))
        {
            struct sid_timespec delta_time, wakeup_time;

            if (sleep_us != 0u)
            {
                /* Calculate wakeup time */
                sid_pal_uptime_now(&wakeup_time);
                sid_ms_to_timespec(sleep_us, &delta_time);
                sid_time_add(&wakeup_time, &delta_time);
            }
            else
            {
                /* Wakeup point is unknown, the radio may remain in sleep indefinitely until Sidewalk stack decides to send out something */
                wakeup_time = SID_TIME_INFINITY;
            }

            /* Call the user callback */
            drv_ctx.sleep_start_notify_cb(&wakeup_time);
        }
        sid_pal_exit_critical_region();
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_standby(void)
{
    int32_t err = RADIO_ERROR_NONE;
    lr11xx_status_t sys_err;

    do
    {
        /* Immediately stop software timer for Tx/Rx/CS/CAD timeout */
        (void)sid_pal_timer_cancel(&drv_ctx.radio_timeout_mon);
        __COMPILER_BARRIER();

#if LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION
        if (RADIO_LR11XX_LBM_BRIDGE_STATE_EXCLUSIVE_MODE == drv_ctx.lbm.bridge_state)
        {
            /* Don't allow Sidewalk operations */
            return RADIO_ERROR_BUSY;
        }
#  if LR11XX_RADIO_CFG_LBM_CONCURRENCY_SUPPORT
        else if (RADIO_LR11XX_LBM_BRIDGE_STATE_SIDEWALK_BLANKING_MODE == drv_ctx.lbm.bridge_state)
        {
            /* Report success to Sidewalk without doing anything */
            drv_ctx.lbm.simulated_radio_state = SID_PAL_RADIO_STANDBY;
            return RADIO_ERROR_NONE;
        }
#  endif /* LR11XX_RADIO_CFG_LBM_CONCURRENCY_SUPPORT */

        /* Activate radio lock by Sidewalk. We don't care if osSemaphoreAcquire() fails due to semaphore is locked already */
        SID_PAL_ASSERT(drv_ctx.lbm.sidewalk_radio_access_lock != NULL);
        (void)osSemaphoreAcquire(drv_ctx.lbm.sidewalk_radio_access_lock, 0u);
#endif /* LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION */

        if (SID_PAL_RADIO_STANDBY_XOSC == drv_ctx.radio_state)
        {
            /* No need to do anything, already in Standby */
            break;
        }

        /* Put radio into Standby to stop any potentially ongoing operations - wakeup is performed automatically by radio HAL layer */
        sys_err = lr11xx_system_set_standby(&drv_ctx, LR11XX_SYSTEM_STANDBY_CFG_XOSC); /* Use STDBY_XOSC to save on oscillator restart time (e.g. when the radio performs Tx right after Rx) */
        if (sys_err != LR11XX_STATUS_OK)
        {
            drv_ctx.radio_state = SID_PAL_RADIO_UNKNOWN;
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* After wake up LR11xx will be in STDBY_RC mode */

        /* Disable FEM - this should be done after the radio is in Standby, otherwise this may lead to HW damage (e.g. if the radio was actively transmitting and FEM went off) */
        err = radio_lr11xx_set_subghz_fem_mode(LR11XX_RADIO_FRONTEND_MODE_OFF);
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

#if LR11XX_RADIO_CFG_USE_STATUS_LED
        /* Turn off status LEDs after the radio has reached Standby state */
        lr11xx_radio_hal_tx_led_off(&drv_ctx);
        lr11xx_radio_hal_rx_led_off(&drv_ctx);
#endif /* LR11XX_RADIO_CFG_USE_STATUS_LED */

        drv_ctx.radio_state = SID_PAL_RADIO_STANDBY_XOSC;
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
    lr11xx_status_t sys_err;

    do
    {
#if LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION
    if (RADIO_LR11XX_LBM_BRIDGE_STATE_EXCLUSIVE_MODE == drv_ctx.lbm.bridge_state)
    {
        /* Don't allow Sidewalk operations */
        err = RADIO_ERROR_BUSY;
        break;
    }
#  if LR11XX_RADIO_CFG_LBM_CONCURRENCY_SUPPORT
    else if (RADIO_LR11XX_LBM_BRIDGE_STATE_SIDEWALK_BLANKING_MODE == drv_ctx.lbm.bridge_state)
    {
        /* Report success to Sidewalk without doing anything */
        err = RADIO_ERROR_NONE;
        break;
    }
#  endif /* LR11XX_RADIO_CFG_LBM_CONCURRENCY_SUPPORT */
#endif /* LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION */

        /* Validate inputs */
        if ((NULL == buffer) || (0u == size))
        {
            LR11XX_RADIO_LOG_ERROR("Set Tx payload aborted - invalid args (buf 0x%08x, sz %u)", (uint32_t)buffer, size);
            err = RADIO_ERROR_INVALID_PARAMS;
            break;
        }

        sys_err = lr11xx_regmem_write_buffer8(&drv_ctx, buffer, size);
        if (sys_err != LR11XX_STATUS_OK)
        {
            LR11XX_RADIO_LOG_ERROR("Failed to upload data to LR11xx Tx buffer. Error %d", (int32_t)sys_err);
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
    lr11xx_status_t sys_err;

    do
    {
#if LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION
        if (RADIO_LR11XX_LBM_BRIDGE_STATE_EXCLUSIVE_MODE == drv_ctx.lbm.bridge_state)
        {
            /* Don't allow Sidewalk operations */
            LR11XX_RADIO_LOG_DEBUG("LR11xx rejected Tx start");
            err = RADIO_ERROR_BUSY;
            break;
        }
#  if LR11XX_RADIO_CFG_LBM_CONCURRENCY_SUPPORT
        else if (RADIO_LR11XX_LBM_BRIDGE_STATE_SIDEWALK_BLANKING_MODE == drv_ctx.lbm.bridge_state)
        {
            /* Still report RADIO_ERROR_BUSY to Sidewalk stack */
            LR11XX_RADIO_LOG_DEBUG("LR11xx rejected Tx start");
            err = RADIO_ERROR_BUSY;
            break;
        }
    #  endif /* LR11XX_RADIO_CFG_LBM_CONCURRENCY_SUPPORT */
    #endif /* LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION */

        /* Configure FEM for Tx */
        err = radio_lr11xx_set_subghz_fem_mode(LR11XX_RADIO_FRONTEND_MODE_TX);
        if (err != RADIO_ERROR_NONE)
        {
            LR11XX_RADIO_LOG_ERROR("Failed to configure FEM for Tx. Error %d", err);
            break;
        }

        /* Update status in the driver */
        drv_ctx.radio_state = SID_PAL_RADIO_TX;
        __COMPILER_BARRIER();

        /* Initiate Tx */
        sys_err = lr11xx_radio_set_tx_with_timeout_in_rtc_step(&drv_ctx, LR11XX_RADIO_RXTX_NO_TIMEOUT_VAL); /* We use a software timer to avoid the radio falling back to STDBY_RC mode on Tx timeout */
        if (sys_err != LR11XX_STATUS_OK)
        {
            LR11XX_RADIO_LOG_ERROR("Failed to initiate Tx. Error %d", (int32_t)sys_err);
            drv_ctx.radio_state = SID_PAL_RADIO_UNKNOWN;
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Start the software timer if needed */
        err = radio_lr11xx_start_radio_timeout_mon(timeout);
        if (err != RADIO_ERROR_NONE)
        {
            LR11XX_RADIO_LOG_ERROR("Failed to start Tx timeout timer. Error %d", err);
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

#if LR11XX_RADIO_CFG_USE_STATUS_LED
        /* Drive the status LED after the actual start of Tx to avoid impact on timings */
        lr11xx_radio_hal_tx_led_on(&drv_ctx);
#endif /* LR11XX_RADIO_CFG_USE_STATUS_LED */
    } while(0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_set_tx_continuous_wave(uint32_t freq, int8_t power)
{
#if HALO_ENABLE_DIAGNOSTICS
    int32_t err;
    lr11xx_status_t sys_err;

#if LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION
    if (RADIO_LR11XX_LBM_BRIDGE_STATE_EXCLUSIVE_MODE == drv_ctx.lbm.bridge_state)
    {
        /* Don't allow Sidewalk operations */
        return RADIO_ERROR_BUSY;
    }
#  if LR11XX_RADIO_CFG_LBM_CONCURRENCY_SUPPORT
    else if (RADIO_LR11XX_LBM_BRIDGE_STATE_SIDEWALK_BLANKING_MODE == drv_ctx.lbm.bridge_state)
    {
        /* Still report RADIO_ERROR_BUSY to Sidewalk stack */
        return RADIO_ERROR_BUSY;
    }
#  endif /* LR11XX_RADIO_CFG_LBM_CONCURRENCY_SUPPORT */
#endif /* LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION */

    do
    {
        /* Ensure the radio is in Standby */
        sys_err = lr11xx_system_set_standby(&drv_ctx, LR11XX_SYSTEM_STANDBY_CFG_XOSC);
        if (sys_err != LR11XX_STATUS_OK)
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
            LR11XX_RADIO_LOG_ERROR("Failed to clear IRQs before starting Tx. Error %d", err);
            break;
        }

        /* Configure FEM for Tx */
        err = radio_lr11xx_set_subghz_fem_mode(LR11XX_RADIO_FRONTEND_MODE_TX);
        if (err != RADIO_ERROR_NONE)
        {
            LR11XX_RADIO_LOG_ERROR("Failed to configure FEM for Tx. Error %d", err);
            break;
        }

        /* Update status in the driver */
        drv_ctx.radio_state = SID_PAL_RADIO_TX;
        __COMPILER_BARRIER();

        /* Initiate Tx */
        sys_err = lr11xx_radio_set_tx_cw(&drv_ctx);
        if (sys_err != LR11XX_STATUS_OK)
        {
            LR11XX_RADIO_LOG_ERROR("Failed to initiate CW Tx. Error %d", (int32_t)sys_err);
            drv_ctx.radio_state = SID_PAL_RADIO_UNKNOWN;
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Ensure LEDs will be operated only after the Tx is initiated */
        __COMPILER_BARRIER();

#if LR11XX_RADIO_CFG_USE_STATUS_LED
        /* Drive the status LED after the actual start of Tx to avoid impact on timings */
        lr11xx_radio_hal_tx_led_on(&drv_ctx);
#endif /* LR11XX_RADIO_CFG_USE_STATUS_LED */
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
    lr11xx_status_t sys_err;

    do
    {
        /* Ensure CS/CAD exit mode is clear */
        drv_ctx.cad_exit_mode = SID_PAL_RADIO_CAD_EXIT_MODE_NONE;

#if LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION
        if (RADIO_LR11XX_LBM_BRIDGE_STATE_EXCLUSIVE_MODE == drv_ctx.lbm.bridge_state)
        {
            /* Don't allow Sidewalk operations */
            LR11XX_RADIO_LOG_DEBUG("LR11xx rejected Rx start");
            err = RADIO_ERROR_BUSY;
            break;
        }
#  if LR11XX_RADIO_CFG_LBM_CONCURRENCY_SUPPORT
        else if (RADIO_LR11XX_LBM_BRIDGE_STATE_SIDEWALK_BLANKING_MODE == drv_ctx.lbm.bridge_state)
        {
            if (LR11XX_RADIO_INFINITE_TIME == timeout)
            {
                err = RADIO_ERROR_BUSY;
                break;
            }

            /* Simulate Rx timeout */
            sid_error_t sid_err;
            struct sid_timespec rx_timeout_ts;

            /* Calculate timeout timestamp */
            sid_err = sid_pal_uptime_now(&rx_timeout_ts);
            if (sid_err != SID_ERROR_NONE)
            {
                err = RADIO_ERROR_IO_ERROR;
                break;
            }

            /* Compensate for the timer setup processing delay */
            rx_timeout_ts.tv_nsec += ((timeout + LR11XX_TUS_TO_US(1u)) * 1000u);
            sid_time_normalize(&rx_timeout_ts);

            LR11XX_RADIO_LOG_DEBUG("LR11xx simulated Rx start");
            drv_ctx.lbm.simulated_radio_state = SID_PAL_RADIO_RX;

            (void)sid_pal_timer_cancel(&drv_ctx.radio_timeout_mon);
            sid_err = sid_pal_timer_arm(&drv_ctx.radio_timeout_mon, SID_PAL_TIMER_PRIO_CLASS_PRECISE, &rx_timeout_ts, NULL);
            if (sid_err != SID_ERROR_NONE)
            {
                err = RADIO_ERROR_IO_ERROR;
                break;
            }

            err = RADIO_ERROR_NONE;
            break;
        }
#  endif /* LR11XX_RADIO_CFG_LBM_CONCURRENCY_SUPPORT */
#endif /* LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION */

        /* Configure FEM for Rx */
        err = radio_lr11xx_set_subghz_fem_mode(LR11XX_RADIO_FRONTEND_MODE_RX);
        if (err != RADIO_ERROR_NONE)
        {
            LR11XX_RADIO_LOG_ERROR("Failed to configure FEM for Rx. Error %d", err);
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
        sys_err = lr11xx_radio_set_rx_with_timeout_in_rtc_step(&drv_ctx, LR11XX_RADIO_INFINITE_TIME == timeout ? LR11XX_RADIO_RX_CONTINUOUS_VAL : LR11XX_RADIO_RXTX_NO_TIMEOUT_VAL); /* We use a software timer to avoid the radio falling back to STDBY_RC mode on Rx timeout */
        if (sys_err != LR11XX_STATUS_OK)
        {
            LR11XX_RADIO_LOG_ERROR("Failed to initiate Rx. Error %d", (int32_t)sys_err);
            drv_ctx.radio_state = SID_PAL_RADIO_UNKNOWN;
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Start the software timer if needed */
        err = radio_lr11xx_start_radio_timeout_mon(timeout);
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

#if LR11XX_RADIO_CFG_USE_STATUS_LED
        /* Drive the status LED after the actual start of Rx to avoid impact on timings */
        lr11xx_radio_hal_rx_led_on(&drv_ctx);
#endif /* LR11XX_RADIO_CFG_USE_STATUS_LED */
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
    lr11xx_status_t sys_err;

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

#  if LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION
        if (RADIO_LR11XX_LBM_BRIDGE_STATE_EXCLUSIVE_MODE == drv_ctx.lbm.bridge_state)
        {
            /* Don't allow Sidewalk operations */
            LR11XX_RADIO_LOG_DEBUG("LR11xx rejected CS start");
            err = RADIO_ERROR_BUSY;
            break;
        }
#    if LR11XX_RADIO_CFG_LBM_CONCURRENCY_SUPPORT
        else if (RADIO_LR11XX_LBM_BRIDGE_STATE_SIDEWALK_BLANKING_MODE == drv_ctx.lbm.bridge_state)
        {
            /* Simulate Rx timeout */
            sid_error_t sid_err;
            struct sid_timespec rx_timeout_ts;

            /* Calculate timeout timestamp */
            sid_err = sid_pal_uptime_now(&rx_timeout_ts);
            if (sid_err != SID_ERROR_NONE)
            {
                err = RADIO_ERROR_IO_ERROR;
                break;
            }

            /* Update status in the driver */
            drv_ctx.cad_exit_mode = exit_mode;

            /* Compensate for the timer setup processing delay */
            rx_timeout_ts.tv_nsec += ((cad_params->fsk_cs_duration_us + LR11XX_TUS_TO_US(1u)) * 1000u);
            sid_time_normalize(&rx_timeout_ts);

            LR11XX_RADIO_LOG_DEBUG("LR11xx simulated CS start");
            drv_ctx.lbm.simulated_radio_state = SID_PAL_RADIO_RX;

            (void)sid_pal_timer_cancel(&drv_ctx.radio_timeout_mon);
            sid_err = sid_pal_timer_arm(&drv_ctx.radio_timeout_mon, SID_PAL_TIMER_PRIO_CLASS_PRECISE, &rx_timeout_ts, NULL);
            if (sid_err != SID_ERROR_NONE)
            {
                err = RADIO_ERROR_IO_ERROR;
                break;
            }

            err = RADIO_ERROR_NONE;
            break;
        }
#    endif /* LR11XX_RADIO_CFG_LBM_CONCURRENCY_SUPPORT */
#  endif /* LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION */

        /* Configure FEM for Rx */
        err = radio_lr11xx_set_subghz_fem_mode(LR11XX_RADIO_FRONTEND_MODE_RX);
        if (err != RADIO_ERROR_NONE)
        {
            break;
        }

        /* Enable IRQs relevant to CS */
        err = radio_set_irq_mask(LR11XX_FSK_CARRIER_SENSE_IRQ_MASK);
        if (err != RADIO_ERROR_NONE)
        {
            break;
        }

        /* Update status in the driver */
        drv_ctx.radio_state = SID_PAL_RADIO_RX;
        drv_ctx.cad_exit_mode = exit_mode;
        __COMPILER_BARRIER();

        /* Start Rx to check for radio channel availability */
        sys_err = lr11xx_radio_set_rx_with_timeout_in_rtc_step(&drv_ctx, LR11XX_RADIO_RXTX_NO_TIMEOUT_VAL); /* We use a software timer to avoid the radio falling back to STDBY_RC mode on Rx timeout */
        if (sys_err != LR11XX_STATUS_OK)
        {
            LR11XX_RADIO_LOG_ERROR("Failed to initiate Rx. Error %d", (int32_t)sys_err);
            drv_ctx.radio_state = SID_PAL_RADIO_UNKNOWN;
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Start the software timer for FSK if needed */
        err = radio_lr11xx_start_radio_timeout_mon(cad_params->fsk_cs_duration_us);
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

#  if LR11XX_RADIO_CFG_USE_STATUS_LED
        /* Drive the status LEDs after the actual start of CS to avoid impact on timings */
#    if (HALO_ENABLE_DIAGNOSTICS == 0)
        if (SID_PAL_RADIO_CAD_EXIT_MODE_CS_LBT == exit_mode)
        {
            /* This is carrier sense for FSK Tx */
            lr11xx_radio_hal_tx_led_on(&drv_ctx);
        }
        else
#    endif /* HALO_ENABLE_DIAGNOSTICS */
        {
            lr11xx_radio_hal_rx_led_on(&drv_ctx);
        }
#  endif /* LR11XX_RADIO_CFG_USE_STATUS_LED */
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
    int32_t err = sid_pal_radio_start_rx(LR11XX_RADIO_INFINITE_TIME);
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
    lr11xx_status_t sys_err;

    do
    {
#  if LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION
    if (RADIO_LR11XX_LBM_BRIDGE_STATE_EXCLUSIVE_MODE == drv_ctx.lbm.bridge_state)
    {
        /* Don't allow Sidewalk operations */
        err = RADIO_ERROR_BUSY;
        break;
    }
#    if LR11XX_RADIO_CFG_LBM_CONCURRENCY_SUPPORT
    else if (RADIO_LR11XX_LBM_BRIDGE_STATE_SIDEWALK_BLANKING_MODE == drv_ctx.lbm.bridge_state)
    {
        /* Still report RADIO_ERROR_BUSY to Sidewalk stack */
        err = RADIO_ERROR_BUSY;
        break;
    }
#    endif /* LR11XX_RADIO_CFG_LBM_CONCURRENCY_SUPPORT */
#  endif /* LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION */

        /* Validate inputs */
        if ((0u == rx_time) || (0u == sleep_time))
        {
            err = RADIO_ERROR_INVALID_PARAMS;
            break;
        }

        /* Configure FEM for Rx */
        err = radio_lr11xx_set_subghz_fem_mode(LR11XX_RADIO_FRONTEND_MODE_RX);
        if (err != RADIO_ERROR_NONE)
        {
            break;
        }

        /* Clear any residual IRQs */
        err = radio_clear_irq_status_all();
        if (err != RADIO_ERROR_NONE)
        {
            LR11XX_RADIO_LOG_ERROR("Failed to clear IRQs before starting Rx. Error %d", err);
            break;
        }

        /* Update status in the driver */
        drv_ctx.radio_state = SID_PAL_RADIO_RX_DC;
        __COMPILER_BARRIER();

        /* Start Rx Duty Cycle mode */
        sys_err = lr11xx_radio_set_rx_duty_cycle(&drv_ctx, rx_time, sleep_time, LR11XX_RADIO_RX_DUTY_CYCLE_MODE_CAD);
        if (sys_err != LR11XX_STATUS_OK)
        {
            drv_ctx.radio_state = SID_PAL_RADIO_UNKNOWN;
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Ensure LEDs will be operated only after the CS is initiated */
        __COMPILER_BARRIER();

#  if LR11XX_RADIO_CFG_USE_STATUS_LED
        /* Drive the status LEDs after the actual start of Rx to avoid impact on timings */
        lr11xx_radio_hal_rx_led_on(&drv_ctx);
#  endif /* LR11XX_RADIO_CFG_USE_STATUS_LED */

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
    lr11xx_status_t sys_err;

    do
    {
#  if LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION
    if (RADIO_LR11XX_LBM_BRIDGE_STATE_EXCLUSIVE_MODE == drv_ctx.lbm.bridge_state)
    {
        /* Don't allow Sidewalk operations */
        err = RADIO_ERROR_BUSY;
        break;
    }
#    if LR11XX_RADIO_CFG_LBM_CONCURRENCY_SUPPORT
    else if (RADIO_LR11XX_LBM_BRIDGE_STATE_SIDEWALK_BLANKING_MODE == drv_ctx.lbm.bridge_state)
    {
        /* Still report RADIO_ERROR_BUSY to Sidewalk stack */
        err = RADIO_ERROR_BUSY;
        break;
    }
#    endif /* LR11XX_RADIO_CFG_LBM_CONCURRENCY_SUPPORT */
#  endif /* LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION */

        /* Configure FEM for Tx */
        err = radio_lr11xx_set_subghz_fem_mode(LR11XX_RADIO_FRONTEND_MODE_TX);
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

        /* Update status in the driver */
        drv_ctx.radio_state = SID_PAL_RADIO_CAD;
        __COMPILER_BARRIER();

        /* Start CAD */
        sys_err = lr11xx_radio_set_cad(&drv_ctx);
        if (sys_err != LR11XX_STATUS_OK)
        {
            drv_ctx.radio_state = SID_PAL_RADIO_UNKNOWN;
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Ensure LEDs will be operated only after the CS is initiated */
        __COMPILER_BARRIER();

#  if LR11XX_RADIO_CFG_USE_STATUS_LED
        /* Drive the status LEDs after the actual start of Rx to avoid impact on timings */
        lr11xx_radio_hal_rx_led_on(&drv_ctx);
#  endif /* LR11XX_RADIO_CFG_USE_STATUS_LED */

        err = RADIO_ERROR_NONE;
     } while(0);

    return err;
#else
    return RADIO_ERROR_NOT_SUPPORTED;
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 && HALO_ENABLE_DIAGNOSTICS */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int16_t sid_pal_radio_rssi(void)
{
    int16_t rssi = 0;
    lr11xx_status_t sys_err;

#if LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION
    if (RADIO_LR11XX_LBM_BRIDGE_STATE_EXCLUSIVE_MODE == drv_ctx.lbm.bridge_state)
    {
        /* Report a defualt value to Sidewalk */
        return INT16_MAX;
    }
#  if LR11XX_RADIO_CFG_LBM_CONCURRENCY_SUPPORT
    else if (RADIO_LR11XX_LBM_BRIDGE_STATE_SIDEWALK_BLANKING_MODE == drv_ctx.lbm.bridge_state)
    {
        /* Still report a default value to Sidewalk stack */
        return INT16_MAX;
    }
#  endif /* LR11XX_RADIO_CFG_LBM_CONCURRENCY_SUPPORT */
#endif /* LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION */

    do
    {
        int8_t raw_rssi;

        sys_err = lr11xx_radio_get_rssi_inst(&drv_ctx, &raw_rssi);
        if (sys_err != LR11XX_STATUS_OK)
        {
            LR11XX_RADIO_LOG_ERROR("Failed to read realtime RSSI. Error %d", (int32_t)sys_err);
            rssi = INT16_MAX;
            break;
        }

        /* Compensate for external LNA, Rx boost, antenna gain, etc. */
        rssi = lr11xx_radio_hal_get_adjusted_rssi(&drv_ctx, raw_rssi);
    } while (0);

    return rssi;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_is_channel_free(uint32_t freq, int16_t threshold, uint32_t delay_us, bool * is_channel_free)
{
#if HALO_ENABLE_DIAGNOSTICS
    int32_t err, irq_err;
    sid_error_t sid_err;
    lr11xx_hal_status_t hal_err;

    SID_PAL_ASSERT(is_channel_free != NULL);

    *is_channel_free = false;

    do
    {
#  if LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION
    if (RADIO_LR11XX_LBM_BRIDGE_STATE_EXCLUSIVE_MODE == drv_ctx.lbm.bridge_state)
    {
        /* Don't allow Sidewalk operations */
        err = RADIO_ERROR_BUSY;
        break;
    }
#    if LR11XX_RADIO_CFG_LBM_CONCURRENCY_SUPPORT
    else if (RADIO_LR11XX_LBM_BRIDGE_STATE_SIDEWALK_BLANKING_MODE == drv_ctx.lbm.bridge_state)
    {
        /* Report to Sidewalk stack the channel is busy */
        err = RADIO_ERROR_NONE;
        break;
    }
#    endif /* LR11XX_RADIO_CFG_LBM_CONCURRENCY_SUPPORT */
#  endif /* LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION */

        /* Disable radio IRQ reaction */
        hal_err = lr11xx_hal_disarm_irq(&drv_ctx);
        if (hal_err != LR11XX_HAL_STATUS_OK)
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

        if (delay_us < LR11XX_MIN_CHANNEL_FREE_DELAY_US)
        {
            delay_us = LR11XX_MIN_CHANNEL_FREE_DELAY_US;
        }

        sid_us_to_timespec(delay_us, &t_threshold);

        sid_err = sid_pal_uptime_now(&t_start);
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
            sid_pal_delay_us(LR11XX_MIN_CHANNEL_FREE_DELAY_US);

            /* Readout realtime RSSI */
            rssi = sid_pal_radio_rssi();
            if (rssi > threshold)
            {
                goto enable_irq;
            }

            sid_err = sid_pal_uptime_now(&t_cur);
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
        hal_err = lr11xx_hal_arm_irq(&drv_ctx);
        if (hal_err != LR11XX_HAL_STATUS_OK)
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
    lr11xx_hal_status_t hal_err;
    lr11xx_status_t sys_err;

    SID_PAL_ASSERT(random != NULL);

#  if LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION
    if (RADIO_LR11XX_LBM_BRIDGE_STATE_EXCLUSIVE_MODE == drv_ctx.lbm.bridge_state)
    {
        /* Don't allow Sidewalk operations */
        return RADIO_ERROR_BUSY;
    }
#    if LR11XX_RADIO_CFG_LBM_CONCURRENCY_SUPPORT
    else if (RADIO_LR11XX_LBM_BRIDGE_STATE_SIDEWALK_BLANKING_MODE == drv_ctx.lbm.bridge_state)
    {
        /* Still report RADIO_ERROR_BUSY to Sidewalk stack */
        return RADIO_ERROR_BUSY;
    }
#    endif /* LR11XX_RADIO_CFG_LBM_CONCURRENCY_SUPPORT */
#  endif /* LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION */

    *random = UINT32_MAX;

    do
    {
        /* Disable IRQ pin from MCU side */
        hal_err = lr11xx_hal_disarm_irq(&drv_ctx);
        if (hal_err != LR11XX_HAL_STATUS_OK)
        {
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Run RNG */
        sys_err = lr11xx_system_get_random_number(&drv_ctx, random);
        if (sys_err != LR11XX_STATUS_OK)
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
        hal_err = lr11xx_hal_arm_irq(&drv_ctx);
        if (hal_err != LR11XX_HAL_STATUS_OK)
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
    lr11xx_hal_status_t hal_err;

#  if LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION
    if (RADIO_LR11XX_LBM_BRIDGE_STATE_EXCLUSIVE_MODE == drv_ctx.lbm.bridge_state)
    {
        /* Don't allow Sidewalk operations */
        return RADIO_ERROR_BUSY;
    }
#    if LR11XX_RADIO_CFG_LBM_CONCURRENCY_SUPPORT
    else if (RADIO_LR11XX_LBM_BRIDGE_STATE_SIDEWALK_BLANKING_MODE == drv_ctx.lbm.bridge_state)
    {
        /* Still report RADIO_ERROR_BUSY to Sidewalk stack */
        return RADIO_ERROR_BUSY;
    }
#    endif /* LR11XX_RADIO_CFG_LBM_CONCURRENCY_SUPPORT */
#  endif /* LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION */

    *noise = 0;

    do
    {
        /* Disable radio IRQ reaction */
        hal_err = lr11xx_hal_disarm_irq(&drv_ctx);
        if (hal_err != LR11XX_HAL_STATUS_OK)
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

        for (uint32_t i = 0u; i < LR11XX_NOISE_SAMPLE_SIZE; i++)
        {
            *noise += sid_pal_radio_rssi();
            sid_pal_delay_us(LR11XX_MIN_CHANNEL_FREE_DELAY_US);
        }

        *noise /= LR11XX_NOISE_SAMPLE_SIZE;

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
        hal_err = lr11xx_hal_arm_irq(&drv_ctx);
        if (hal_err != LR11XX_HAL_STATUS_OK)
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
    if (LR11XX_TCXO_CTRL_VTCXO == drv_ctx.config->tcxo_config.ctrl)
    {
        /* TCXO is powered from LR11xx's VTCXO pin and will require full wait time on each start (e.g. on every transition to STDBY_XOSC, TX, RX, etc. from STDBY_RC state) */
        state_delay->tcxo_delay_us = drv_ctx.config->tcxo_config.timeout_us;
    }
    else if (LR11XX_TCXO_CTRL_VDD == drv_ctx.config->tcxo_config.ctrl)
    {
        /**
         * Since TCXO is powered directly from VDD in this config we don't need a start timeout for it. However,
         * we can't set the timeout to 0 in LR11xx because this will switch it to XOSC mode, so we have to use
         * the minimum timeout of 1 tick (approx. 30.52us)
         */
         state_delay->tcxo_delay_us = LR11XX_TUS_TO_US(1u);
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
    lr11xx_hal_status_t hal_err;
    lr11xx_status_t sys_err;
    sid_error_t sid_err;
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
#if LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION
            /* Ensure LoRa Basics Modem bridge is not active */
            if (RADIO_LR11XX_LBM_BRIDGE_STATE_EXCLUSIVE_MODE == drv_ctx.lbm.bridge_state)
            {
                /* Don't allow Sidewalk operations */
                SID_PAL_LOG_ERROR("Can't reinitialize Sidewalk radio driver - LBM bridge mode is active");
                return RADIO_ERROR_BUSY;
            }
#  if LR11XX_RADIO_CFG_LBM_CONCURRENCY_SUPPORT
            else if (RADIO_LR11XX_LBM_BRIDGE_STATE_SIDEWALK_BLANKING_MODE == drv_ctx.lbm.bridge_state)
            {
                /* Still report RADIO_ERROR_BUSY to Sidewalk stack */
                SID_PAL_LOG_ERROR("Can't reinitialize Sidewalk radio driver - LBM bridge mode is active");
                return RADIO_ERROR_BUSY;
            }
#  endif /* LR11XX_RADIO_CFG_LBM_CONCURRENCY_SUPPORT */
#endif /* LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION */

            SID_PAL_LOG_WARNING("LR11xx driver is initialized already. Requesting deinitialization");

            /* Deinitialize any hardware that may have been partially initialized to bring it to the known state */
            err = sid_pal_radio_deinit();
            if (err != RADIO_ERROR_NONE)
            {
                SID_PAL_LOG_ERROR("Unable to initialize LR11xx radio - driver is initialized already and deinitialization failed. Error %d", err);
                break;
            }
        }
        /*----------------------------------------------------------------------------*/

        /* Ensure LR11xx won't trigger any interrupt if we are re-initializing -------*/
        hal_err = lr11xx_hal_disarm_irq(&drv_ctx);
        if (hal_err != LR11XX_HAL_STATUS_OK)
        {
            /* Logs are provided by the lr11xx_hal_disarm_irq() */
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }
        __COMPILER_BARRIER(); /* Ensure IRQ is disarmed right now before any further actions take place */
        /*----------------------------------------------------------------------------*/

        /* Initialize context --------------------------------------------------------*/
        drv_ctx.radio_rx_packet    = rx_packet;
        drv_ctx.report_radio_event = notify;
        drv_ctx.irq_handler        = dio_irq_handler;
        drv_ctx.modem              = LR11XX_RADIO_MODEM_MODE_INVALID;
        drv_ctx.radio_state        = SID_PAL_RADIO_UNKNOWN;
#if LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION
        drv_ctx.lbm.bridge_state   = RADIO_LR11XX_LBM_BRIDGE_STATE_INACTIVE;
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
#endif /* LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION */

#if ((SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 != 0) && (SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 == 0))
        /* FSK-only SubGHz link, default to FSK modem mode for initialization */
        initial_modem_mode         = SID_PAL_RADIO_MODEM_MODE_FSK;
#else
        /* LoRa-only or FSK-LoRa SubGHz link - default to LoRa modem mode for initialization */
        initial_modem_mode         = SID_PAL_RADIO_MODEM_MODE_LORA;
#endif

        /* Load regional params */
        sid_pal_radio_set_region(drv_ctx.config->regional_config.radio_region);

        /* Create the timer to process Rx,Tx, and CS/CAD timeouts */
        sid_err = sid_pal_timer_init(&drv_ctx.radio_timeout_mon, radio_lr11xx_on_radio_timeout_event, NULL);
        if (sid_err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to create radio timeout monitor. Error %d", (int32_t)sid_err);
        }
        /*----------------------------------------------------------------------------*/

        /* Base init */
        err = radio_lr11xx_platform_init();
        if (err != RADIO_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Low-level radio init failed. Error %d", err);
            break;
        }

        /* Ensure FEM is off (if present) */
        err = radio_lr11xx_set_subghz_fem_mode(LR11XX_RADIO_FRONTEND_MODE_OFF);
        if (err != RADIO_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to turn off FEM. Error %d", err);
            break;
        }

        /* Reset LR11xx to ensure its consistent state */
        sys_err = lr11xx_system_reset(&drv_ctx);
        if (sys_err != LR11XX_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to reset LR11xx. Error %d", (int32_t)sys_err);
            err = RADIO_ERROR_IO_ERROR;
            break;
        }

        /* Assume the radio automatically reaches STDBY_RC state after reset */
        drv_ctx.radio_state = SID_PAL_RADIO_STANDBY;

        /* Readout and store version information */
        sys_err = lr11xx_system_get_version(&drv_ctx, &drv_ctx.ver);
        if (sys_err != LR11XX_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to read LR11xx version info. Error %d", (int32_t)sys_err);
            err = RADIO_ERROR_IO_ERROR;
            break;
        };

        /* Cehck if reported version is valid and supported by this driver */
        err = radio_lr11xx_validate_version(&drv_ctx.ver);
        if (err != RADIO_ERROR_NONE)
        {
            /* Logs provided by radio_lr11xx_validate_version() */;
            if ((RADIO_ERROR_INVALID_STATE == err) && (LR11XX_RADIO_BOOTLOADER_MODE_ID == drv_ctx.ver.type))
            {
                /* Special case - LR11xx is in bootloader mode. While transceiver cannot be used normally we still need to proceed with SPI initilaization to allow LR11xx firmware flashing */
                err = RADIO_ERROR_NONE;
                goto finalize_init;
            }
            else
            {
                /* Fatal error, cannot proceed */
                break;
            }
        }

        /* Ensure Standby RC mode - this is a mandatory step before configuring the regulator mode */
        sys_err = lr11xx_system_set_standby(&drv_ctx, LR11XX_SYSTEM_STANDBY_CFG_RC);
        if (sys_err != LR11XX_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to put the radio into STDBY_RC state. Error %d", (int32_t)sys_err);
            err = RADIO_ERROR_IO_ERROR;
            break;
        };

        /* Configure radio DIOs to operate the RF switches properly */
        if (drv_ctx.config->rfswitch.enable != 0u)
        {
            sys_err = lr11xx_system_set_dio_as_rf_switch(&drv_ctx, &drv_ctx.config->rfswitch);
            if (sys_err != LR11XX_STATUS_OK)
            {
                SID_PAL_LOG_ERROR("Failed to configure DIO to drive RF switch. Error %d", (int32_t)sys_err);
                err =  RADIO_ERROR_IO_ERROR;
                break;
            }
        }

        /* Apply power regulator settings */
        sys_err = lr11xx_system_set_reg_mode(&drv_ctx, drv_ctx.config->regulator_mode);
        if (sys_err != LR11XX_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to configure radio power regulator. Error %d", (int32_t)sys_err);
            err = RADIO_ERROR_IO_ERROR;
            break;
        };

        /* Now configure the radio clocks */
        err = radio_lr11xx_configure_clocks();
        if (err != RADIO_ERROR_NONE)
        {
            /* Logs provided by radio_lr11xx_configure_clocks() */;
            break;
        }

        /* Set fallback mode to STDBY_XOSC to save on oscillator restart time */
        sys_err = lr11xx_radio_set_rx_tx_fallback_mode(&drv_ctx, LR11XX_RADIO_FALLBACK_STDBY_XOSC);
        if (sys_err != LR11XX_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to set radio fallback mode to STDBY_XOSC. Error %d", (int32_t)sys_err);
            err = RADIO_ERROR_IO_ERROR;
            break;
        }

        /* Bring the radio to STDBY_XOSC state now */
        err = sid_pal_radio_standby();
        if (err != RADIO_ERROR_NONE)
        {
            /* Logs provided by sid_pal_radio_standby() */;
            break;
        }

        /* Apply PA and LNA related settings */
        sys_err = lr11xx_radio_cfg_rx_boosted(&drv_ctx, drv_ctx.config->rx_boost);
        if (sys_err != LR11XX_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to configure Rx boost mode. Error %d", (int32_t)sys_err);
            err = RADIO_ERROR_IO_ERROR;
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

#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 || SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
        /* Apply settings specific to the selected modem mode */
        err = sid_pal_radio_set_modem_mode(initial_modem_mode);
        if (err != RADIO_ERROR_NONE)
        {
            break;
        }
#else
        (void)initial_modem_mode;
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 || SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 */

        /* Clear any potentially present IRQ flags */
        err = radio_clear_irq_status_all();
        if (err != RADIO_ERROR_NONE)
        {
            break;
        }

        /* Ensure all the above operations are complited before the radio IRQ is enabled */
        __COMPILER_BARRIER();

        /* Enable IRQ from the MCU side */
        hal_err = lr11xx_hal_arm_irq(&drv_ctx);
        if (hal_err != LR11XX_HAL_STATUS_OK)
        {
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

finalize_init:
        /* Done */
        SID_PAL_LOG_INFO("LR11xx radio initialized");
        drv_ctx.init_done = TRUE;
        err = RADIO_ERROR_NONE;
    } while (0);

    if (err != RADIO_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("Unable to initialize LR11xx radio. Error %d. LR11xx state: 0x%02X%02X", err, drv_ctx.last.stat1, drv_ctx.last.stat2);

        /* Ensure we deinitialize any hardware that may have been partially initialized */
        (void)sid_pal_radio_deinit();
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_deinit(void)
{
    int32_t err = RADIO_ERROR_GENERIC;

    LR11XX_RADIO_LOG_DEBUG("sid_pal_radio_deinit... ");

    do
    {
        (void)sid_pal_timer_deinit(&drv_ctx.radio_timeout_mon);
        __COMPILER_BARRIER();

        /* Bring down the underlying hardware */
        err = radio_lr11xx_platform_deinit();
        if (err != RADIO_ERROR_NONE)
        {
            LR11XX_RADIO_LOG_ERROR("radio_lr11xx_platform_deinit() failed. Error %d", err);
            break;
        }

        /* From here radio IRQs won't be processed */

        /* Invalidate the context */
        drv_ctx.init_done               = FALSE;
        drv_ctx.irq_handler             = NULL;
        drv_ctx.report_radio_event      = NULL;
        drv_ctx.radio_freq_hz           = 0u;
        drv_ctx.radio_freq_band         = NULL;
        drv_ctx.radio_rx_packet         = NULL;
        drv_ctx.radio_state             = SID_PAL_RADIO_UNKNOWN;
        drv_ctx.sleep_start_notify_cb   = NULL;
        drv_ctx.sid_busy_indication     = FALSE;
#if LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION
        drv_ctx.lbm.bridge_state        = RADIO_LR11XX_LBM_BRIDGE_STATE_INACTIVE;
        drv_ctx.lbm.gnss_scan_active    = FALSE;
        drv_ctx.lbm.wifi_scan_active    = FALSE;
        drv_ctx.lbm.radio_sleep_pending = FALSE;

        /* Ensure radio lock is released */
        if (osSemaphoreGetCount(drv_ctx.lbm.sidewalk_radio_access_lock) == 0u)
        {
            osStatus_t os_err = osSemaphoreRelease(drv_ctx.lbm.sidewalk_radio_access_lock);
            SID_PAL_ASSERT(osOK == os_err); /* osSemaphoreRelease() can fail only on systematic issues, so assertion is better and more efficient here */
        }
#endif /* LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION */
#if HALO_ENABLE_DIAGNOSTICS
        drv_ctx.pa_cfg_override       = FALSE;
#endif /* HALO_ENABLE_DIAGNOSTICS */
        SID_STM32_UTIL_fast_memset(&drv_ctx.last, 0u, sizeof(drv_ctx.last));
        SID_STM32_UTIL_fast_memset(&drv_ctx.settings_cache, 0u, sizeof(drv_ctx.settings_cache));

        err = RADIO_ERROR_NONE;
    } while (0);

    if (err != RADIO_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("Unable to deinitialize LR11xx radio. Error %d", err);
    }
    else
    {
        SID_PAL_LOG_INFO("LR11xx radio deinitialized");
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

#if LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION
SID_STM32_SPEED_OPTIMIZED sid_error_t sid_pal_radio_lr11xx_start_lbm_bridge_mode(const radio_lr11xx_lbm_bridge_state_t desired_mode)
{
    sid_error_t err;
    osStatus_t os_status;

    sid_pal_enter_critical_region();

    do
    {
        /* Validate inputs */
        if (RADIO_LR11XX_LBM_BRIDGE_STATE_EXCLUSIVE_MODE == desired_mode)
        {
            /* Supported mode is selected, proceed */
        }
#  if LR11XX_RADIO_CFG_LBM_CONCURRENCY_SUPPORT
        else if (RADIO_LR11XX_LBM_BRIDGE_STATE_SIDEWALK_BLANKING_MODE == desired_mode)
        {
            /* Supported mode is selected, proceed */
        }
#  endif /* LR11XX_RADIO_CFG_LBM_CONCURRENCY_SUPPORT */
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
        if (drv_ctx.lbm.bridge_state > RADIO_LR11XX_LBM_BRIDGE_STATE_ACTIVATION_PENDING)
        {
            /* Not an issue, just report RADIO_ERROR_NONE without any further actions */
            drv_ctx.lbm.bridge_state = desired_mode;
            err = RADIO_ERROR_NONE;
            break;
        }

        /* Precondition #3 - radio should be in Sleep state and not performing any Sidewalk-related actions */
        drv_ctx.lbm.bridge_state = RADIO_LR11XX_LBM_BRIDGE_STATE_ACTIVATION_PENDING;

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
#  if LR11XX_RADIO_CFG_LBM_CONCURRENCY_SUPPORT
        drv_ctx.lbm.simulated_radio_state = drv_ctx.radio_state;
#  endif /* LR11XX_RADIO_CFG_LBM_CONCURRENCY_SUPPORT */
        err = RADIO_ERROR_NONE;
    } while (0);

    sid_pal_exit_critical_region();

    return err;
}
#endif /* LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION */

/*----------------------------------------------------------------------------*/

#if LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION
SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_lr11xx_stop_lbm_bridge_mode(void)
{
    int32_t err;
    lr11xx_status_t sys_err;
    const radio_lr11xx_lbm_bridge_state_t lbm_bridge_state_backup = drv_ctx.lbm.bridge_state;

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
        if (drv_ctx.lbm.bridge_state <= RADIO_LR11XX_LBM_BRIDGE_STATE_ACTIVATION_PENDING)
        {
            err = RADIO_ERROR_INVALID_STATE;
            break;
        }

        /* Precondition #3 - radio should be in Sleep state and not performing any LBM-related actions */
        if ((drv_ctx.radio_state != SID_PAL_RADIO_SLEEP) && (FALSE == drv_ctx.lbm.radio_sleep_pending))
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

        /* Ensure the radio is free of any residual errors. This command will also transition the radio into STDBY_RC state */
        sys_err = lr11xx_system_clear_errors(&drv_ctx);
        if (sys_err != LR11XX_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("LBM bridge mode exit failed to clear radio errors. Error %d", (int32_t)sys_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Clear any remaining IRQ flags */
        err = radio_clear_irq_status_all();
        if (err != RADIO_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("LBM bridge mode exit failed to clear radio IRQs. Error %d", err);
            break;
        }

        /* Switch off LBM bridge mode indication to allow Sidewalk radio API calls */
        drv_ctx.lbm.bridge_state = RADIO_LR11XX_LBM_BRIDGE_STATE_INACTIVE;

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
            LR11XX_RADIO_LOG_ERROR("Failed to put the radio to sleep. Error %d", err);
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
#endif /* LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION */

/*----------------------------------------------------------------------------*/

#if LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION
SID_STM32_SPEED_OPTIMIZED radio_lr11xx_lbm_bridge_state_t sid_pal_radio_lr11xx_get_lbm_bridge_mode(void)
{
    radio_lr11xx_lbm_bridge_state_t current_state = drv_ctx.lbm.bridge_state;
    return current_state;
}
#endif /* LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION */

/*----------------------------------------------------------------------------*/

#if SID_SDK_CONFIG_RF_TIMINGS_MEASUREMENT
SID_STM32_SPEED_OPTIMIZED void sid_pal_radio_rxtx_start_cb(void)
{
    /* Keep this function as short as possible to minimize its contribution to the Rx/Tx processing delays */
    SIDEWALK_RF_TIMINGS_RXTX_START_GPIO_Port->BSRR = SIDEWALK_RF_TIMINGS_RXTX_START_Pin;
}
#endif /* SID_SDK_CONFIG_RF_TIMINGS_MEASUREMENT */
