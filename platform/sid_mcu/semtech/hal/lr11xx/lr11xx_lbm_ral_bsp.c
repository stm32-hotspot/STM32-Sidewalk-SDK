/**
  ******************************************************************************
  * @file    lr11xx_lbm_ral_bsp.c
  * @brief   Hardware abstractions for LoRa Basics Modem leveraging Sidewalk API
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

/* STD headers */
#include <stdbool.h>

/* Sidewalk interfaces */
#include <sid_pal_assert_ifc.h>
#include <sid_pal_critical_region_ifc.h>
#include <sid_pal_log_ifc.h>
#include <sid_pal_serial_bus_spi_config.h>

/* LoRa Basics Modem (LBM) interfaces */
#include <smtc_modem_api.h>
#include <smtc_modem_utilities.h>
#include <smtc_ralf.h>
#include <smtc_ralf_drv.h>
#include <smtc_ralf_lr11xx.h>
#include <smtc_ral_lr11xx_bsp.h>

/* Sidewalk LR11xx driver */
#include "halo_lr11xx_radio.h"
#include "lr11xx_hal.h"
#include "lr11xx_radio_config.h"
#include "lr11xx_radio_ext_ifc.h"

/* Platform-specific includes */
#include <sid_stm32_common_utils.h>

/* Private defines -----------------------------------------------------------*/

#ifndef LR11XX_LBM_RAL_BSP_EXTRA_LOGGING
/* Set LR11XX_LBM_RAL_BSP_EXTRA_LOGGING to 1 to enable extended logs */
#  define LR11XX_LBM_RAL_BSP_EXTRA_LOGGING (0)
#endif

#if LR11XX_LBM_RAL_BSP_EXTRA_LOGGING
#  define LR11XX_RAL_BSP_LOG_ERROR(...)    SID_PAL_LOG_ERROR(__VA_ARGS__)
#  define LR11XX_RAL_BSP_LOG_WARNING(...)  SID_PAL_LOG_WARNING(__VA_ARGS__)
#  define LR11XX_RAL_BSP_LOG_INFO(...)     SID_PAL_LOG_INFO(__VA_ARGS__)
#  define LR11XX_RAL_BSP_LOG_DEBUG(...)    SID_PAL_LOG_DEBUG(__VA_ARGS__)
#  define LR11XX_RAL_BSP_LOG_TRACE(...)    SID_PAL_LOG_TRACE(__VA_ARGS__)
#else
#  define LR11XX_RAL_BSP_LOG_ERROR(...)    ((void)0u)
#  define LR11XX_RAL_BSP_LOG_WARNING(...)  ((void)0u)
#  define LR11XX_RAL_BSP_LOG_INFO(...)     ((void)0u)
#  define LR11XX_RAL_BSP_LOG_DEBUG(...)    ((void)0u)
#  define LR11XX_RAL_BSP_LOG_TRACE(...)    ((void)0u)
#endif /* LR11XX_LBM_RAL_BSP_EXTRA_LOGGING */

/* Private variables ---------------------------------------------------------*/

#if LR11XX_RADIO_CFG_GNSS_SCAN_USE_BLANKING
static radio_lr11xx_lbm_bridge_state_t lbm_bridge_mode_on_gnss_scan_entry;
#endif /* LR11XX_RADIO_CFG_GNSS_SCAN_USE_BLANKING */
#if LR11XX_RADIO_CFG_WIFI_SCAN_USE_BLANKING
static radio_lr11xx_lbm_bridge_state_t lbm_bridge_mode_on_wifi_scan_entry;
#endif /* LR11XX_RADIO_CFG_WIFI_SCAN_USE_BLANKING */

/* Global function definitions -----------------------------------------------*/

__WEAK SID_STM32_SPEED_OPTIMIZED sid_error_t smtc_modem_ext_ral_bsp_app_specific_init(const ralf_t * const ralf_instance)
{
    /* This is a default implementation of the app-specific initialization. Feel free to override it */
    (void)ralf_instance;
    return SID_ERROR_NONE;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED const ralf_t * smtc_modem_ext_ral_bsp_init(void)
{
    sid_error_t sid_err;
    const halo_drv_semtech_ctx_t * const drv_ctx = lr11xx_get_drv_ctx();
    const ralf_t * ralf_instance = NULL;

    do
    {
        /* Validate inputs */
        if ((NULL == drv_ctx) || (FALSE == drv_ctx->init_done))
        {
            /* Driver is not initialized properly */
            SID_PAL_LOG_ERROR("Can't init RAL BSP. LR11xx Sidewalk driver is not initialized");
            break;
        }

        /* Store LR11xx driver context to LBM */
        smtc_modem_set_radio_context((const void *)drv_ctx);

        /* Get pointer to RALF instance from LBM */
        ralf_instance = smtc_modem_get_radio_ralf();
        if (NULL == ralf_instance)
        {
            SID_PAL_LOG_ERROR("Can't init RAL BSP. LBM RALF instance is null");
            break;
        }

        /* Call app-specific hook for BSP initialization */
        sid_err = smtc_modem_ext_ral_bsp_app_specific_init(ralf_instance);
        if (sid_err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Can't init RAL BSP. Application-specific initialization failed with error %d", (int32_t)sid_err);
            ralf_instance = NULL;
            break;
        }

        /* Done */
    } while (0);

    return ralf_instance;
}

/*----------------------------------------------------------------------------*/

__WEAK SID_STM32_SPEED_OPTIMIZED void ral_lr11xx_bsp_get_rssi_calibration_table(const void * context, const uint32_t freq_in_hz, lr11xx_radio_rssi_calibration_table_t * rssi_calibration_table)
{
    (void)context;

    if (freq_in_hz <= 600000000u)
    {
        rssi_calibration_table->gain_offset      =  0u;
        rssi_calibration_table->gain_tune.g4     = 12u;
        rssi_calibration_table->gain_tune.g5     = 12u;
        rssi_calibration_table->gain_tune.g6     = 14u;
        rssi_calibration_table->gain_tune.g7     =  0u;
        rssi_calibration_table->gain_tune.g8     =  1u;
        rssi_calibration_table->gain_tune.g9     =  3u;
        rssi_calibration_table->gain_tune.g10    =  4u;
        rssi_calibration_table->gain_tune.g11    =  4u;
        rssi_calibration_table->gain_tune.g12    =  3u;
        rssi_calibration_table->gain_tune.g13    =  6u;
        rssi_calibration_table->gain_tune.g13hp1 =  6u;
        rssi_calibration_table->gain_tune.g13hp2 =  6u;
        rssi_calibration_table->gain_tune.g13hp3 =  6u;
        rssi_calibration_table->gain_tune.g13hp4 =  6u;
        rssi_calibration_table->gain_tune.g13hp5 =  6u;
        rssi_calibration_table->gain_tune.g13hp6 =  6u;
        rssi_calibration_table->gain_tune.g13hp7 =  6u;
    }
    else if (freq_in_hz <= 2000000000u)
    {
        rssi_calibration_table->gain_offset      = 0u;
        rssi_calibration_table->gain_tune.g4     = 2u;
        rssi_calibration_table->gain_tune.g5     = 2u;
        rssi_calibration_table->gain_tune.g6     = 2u;
        rssi_calibration_table->gain_tune.g7     = 3u;
        rssi_calibration_table->gain_tune.g8     = 3u;
        rssi_calibration_table->gain_tune.g9     = 4u;
        rssi_calibration_table->gain_tune.g10    = 5u;
        rssi_calibration_table->gain_tune.g11    = 4u;
        rssi_calibration_table->gain_tune.g12    = 4u;
        rssi_calibration_table->gain_tune.g13    = 6u;
        rssi_calibration_table->gain_tune.g13hp1 = 5u;
        rssi_calibration_table->gain_tune.g13hp2 = 5u;
        rssi_calibration_table->gain_tune.g13hp3 = 6u;
        rssi_calibration_table->gain_tune.g13hp4 = 6u;
        rssi_calibration_table->gain_tune.g13hp5 = 6u;
        rssi_calibration_table->gain_tune.g13hp6 = 7u;
        rssi_calibration_table->gain_tune.g13hp7 = 6u;
    }
    else /* if (freq_in_hz > 2000000000u) */
    {
        rssi_calibration_table->gain_offset      = 2030u;
        rssi_calibration_table->gain_tune.g4     =    6u;
        rssi_calibration_table->gain_tune.g5     =    7u;
        rssi_calibration_table->gain_tune.g6     =    6u;
        rssi_calibration_table->gain_tune.g7     =    4u;
        rssi_calibration_table->gain_tune.g8     =    3u;
        rssi_calibration_table->gain_tune.g9     =    4u;
        rssi_calibration_table->gain_tune.g10    =   14u;
        rssi_calibration_table->gain_tune.g11    =   12u;
        rssi_calibration_table->gain_tune.g12    =   14u;
        rssi_calibration_table->gain_tune.g13    =   12u;
        rssi_calibration_table->gain_tune.g13hp1 =   12u;
        rssi_calibration_table->gain_tune.g13hp2 =   12u;
        rssi_calibration_table->gain_tune.g13hp3 =   12u;
        rssi_calibration_table->gain_tune.g13hp4 =    8u;
        rssi_calibration_table->gain_tune.g13hp5 =    8u;
        rssi_calibration_table->gain_tune.g13hp6 =    9u;
        rssi_calibration_table->gain_tune.g13hp7 =    9u;
    }
}

/*----------------------------------------------------------------------------*/

__WEAK SID_STM32_SPEED_OPTIMIZED void ral_lr11xx_bsp_get_lora_cad_det_peak(const void * context, ral_lora_sf_t sf, ral_lora_bw_t bw, ral_lora_cad_symbs_t nb_symbol, uint8_t * in_out_cad_det_peak)
{
    /* Function used to fine tune the cad detection peak, update if needed */
    (void)context;
    (void)sf;
    (void)bw;
    (void)nb_symbol;
    (void)in_out_cad_det_peak;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED void ral_lr11xx_bsp_get_tx_cfg(const void * context, const ral_lr11xx_bsp_tx_cfg_input_params_t * input_params, ral_lr11xx_bsp_tx_cfg_output_params_t * output_params)
{
    const halo_drv_semtech_ctx_t * const drv_ctx = (const halo_drv_semtech_ctx_t *)context;
    int32_t sid_radio_err;

    /* Validate inputs */
    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->config != NULL);
    SID_PAL_ASSERT(drv_ctx->config->pa_cfg_callback != NULL);

    do
    {
        /* Use PA configuration resolver from Sidewalk driver */
        sid_radio_err = radio_lr11xx_compute_tx_power_config(input_params->system_output_pwr_in_dbm);
        if (sid_radio_err != RADIO_ERROR_NONE)
        {
            LR11XX_RAL_BSP_LOG_ERROR("Failed to set LR11xx SubGHz Tx power to %ddB. Error %d", input_params->system_output_pwr_in_dbm, sid_radio_err);
            
            /* Fill-in drv_ctx.pa_cfg with invalid values */
            SID_STM32_UTIL_fast_memset(output_params, 0xFFu, sizeof(*output_params));
            break;
        }

        output_params->pa_cfg.pa_sel                     = drv_ctx->pa_cfg.pa_cfg.pa_sel;
        output_params->pa_cfg.pa_reg_supply              = drv_ctx->pa_cfg.pa_cfg.pa_reg_supply;
        output_params->pa_cfg.pa_duty_cycle              = drv_ctx->pa_cfg.pa_cfg.pa_duty_cycle;
        output_params->pa_cfg.pa_hp_sel                  = drv_ctx->pa_cfg.pa_cfg.pa_hp_sel;
        output_params->pa_ramp_time                      = drv_ctx->pa_cfg.ramp_time;
        output_params->chip_output_pwr_in_dbm_expected   = drv_ctx->pa_cfg.target_tx_power;
        output_params->chip_output_pwr_in_dbm_configured = drv_ctx->pa_cfg.tx_power_reg;
    } while (0);
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED void ral_lr11xx_bsp_get_lfclk_cfg_in_sleep(const void * context, bool * lfclk_is_running)
{
    (void)context;

    /* Sidewalk implementation always keeps RTC running in sleep state */
    *lfclk_is_running = (bool)0x01u; /* Explicitly use 0x01 here since `true` may mean anything non-zero, but LR11xx firmware accepts only 0x01 as valid value */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED void ral_lr11xx_bsp_get_rx_boost_cfg(const void * context, bool * rx_boost_is_activated)
{
    const halo_drv_semtech_ctx_t * const drv_ctx = (const halo_drv_semtech_ctx_t *)context;

    /* Validate inputs */
    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->config != NULL);

    /* Get Rx Boost mode configuration from from Sidewalk radio config */
    *rx_boost_is_activated = drv_ctx->config->rx_boost;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED void ral_lr11xx_bsp_get_xosc_cfg(const void * context, ral_xosc_cfg_t * xosc_cfg, lr11xx_system_tcxo_supply_voltage_t * supply_voltage, uint32_t * startup_time_in_tick)
{
    const halo_drv_semtech_ctx_t * const drv_ctx = (const halo_drv_semtech_ctx_t *)context;

    /* Validate inputs */
    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->config != NULL);

    /* Map Sidewalk radio configuration settings for TCXO control to LBM RAL */
    switch (drv_ctx->config->tcxo_config.ctrl)
    {
        case LR11XX_TCXO_CTRL_VDD:
            *xosc_cfg = RAL_XOSC_CFG_TCXO_EXT_CTRL;
            break;

        case LR11XX_TCXO_CTRL_VTCXO:
            *xosc_cfg = RAL_XOSC_CFG_TCXO_RADIO_CTRL;
            break;

        case LR11XX_TCXO_CTRL_NONE:
        default:
            *xosc_cfg = RAL_XOSC_CFG_XTAL;
            break;
    }

    /* Copy other settings from Sidewalk radio config */
    *supply_voltage       = drv_ctx->config->tcxo_config.ctrl_voltage;
    *startup_time_in_tick = LR11XX_US_TO_TUS(drv_ctx->config->tcxo_config.timeout_us);
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED void ral_lr11xx_bsp_get_rf_switch_cfg(const void * context, lr11xx_system_rfswitch_cfg_t * rf_switch_cfg)
{
    const halo_drv_semtech_ctx_t * const drv_ctx = (const halo_drv_semtech_ctx_t *)context;

    /* Validate inputs */
    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->config != NULL);

    /* Copy LR11xx DIO config for RF switch control from Sidewalk radio config */
    SID_STM32_UTIL_fast_memcpy(rf_switch_cfg, &drv_ctx->config->rfswitch, sizeof(*rf_switch_cfg));
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED void ral_lr11xx_bsp_get_reg_mode(const void * context, lr11xx_system_reg_mode_t * reg_mode)
{
    const halo_drv_semtech_ctx_t * const drv_ctx = (const halo_drv_semtech_ctx_t *)context;

    /* Validate inputs */
    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->config != NULL);

    /* Get regulator mode from Sidewalk radio config */
    *reg_mode = drv_ctx->config->regulator_mode;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED void ral_lr11xx_bsp_get_crc_state(const void * context, bool * crc_is_activated)
{
    const halo_drv_semtech_ctx_t * const drv_ctx = (const halo_drv_semtech_ctx_t *)context;

    /* Validate inputs */
    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->config != NULL);
    SID_PAL_ASSERT(drv_ctx->config->bus_selector.client_selector_extension != NULL);

    /* Pop SPI CRC selection from the SPI bus config */
    const sid_pal_serial_bus_stm32wbaxx_transaction_config_t * const radio_spi_transaction_config = (const sid_pal_serial_bus_stm32wbaxx_transaction_config_t *)drv_ctx->config->bus_selector.client_selector_extension;
    *crc_is_activated = (SID_PAL_SERIAL_BUS_STM32WBAxx_CRC_NOT_USED == radio_spi_transaction_config->crc_polynomial) ? false : true;
}

/*----------------------------------------------------------------------------*/

__WEAK SID_STM32_SPEED_OPTIMIZED ral_status_t ral_lr11xx_bsp_get_instantaneous_tx_power_consumption(const void * context, const ral_lr11xx_bsp_tx_cfg_output_params_t * tx_cfg, lr11xx_system_reg_mode_t radio_reg_mode, uint32_t * pwr_consumption_in_ua)
{
    (void)context;
    (void)tx_cfg;
    (void)radio_reg_mode;
    *pwr_consumption_in_ua = 0u;

    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

/*----------------------------------------------------------------------------*/

__WEAK SID_STM32_SPEED_OPTIMIZED ral_status_t ral_lr11xx_bsp_get_instantaneous_gfsk_rx_power_consumption(const void * context, lr11xx_system_reg_mode_t radio_reg_mode, bool rx_boosted, uint32_t * pwr_consumption_in_ua)
{
    (void)context;
    (void)radio_reg_mode;
    (void)rx_boosted;
    *pwr_consumption_in_ua = 0u;

    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

/*----------------------------------------------------------------------------*/

__WEAK SID_STM32_SPEED_OPTIMIZED ral_status_t ral_lr11xx_bsp_get_instantaneous_lora_rx_power_consumption(const void * context, lr11xx_system_reg_mode_t radio_reg_mode, bool rx_boosted, uint32_t * pwr_consumption_in_ua)
{
    (void)context;
    (void)radio_reg_mode;
    (void)rx_boosted;
    *pwr_consumption_in_ua = 0u;

    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

/*----------------------------------------------------------------------------*/

#if LR11XX_RADIO_CFG_GEOLOCATION_SUPPORT
__WEAK SID_STM32_SPEED_OPTIMIZED void geolocation_bsp_gnss_get_consumption(lr11xx_gnss_instantaneous_power_consumption_ua_t * instantaneous_power_consumption_ua)
{
    /* These value are for EVK board in DC DC mode with Xtal 32KHz and a TCXO 32MHz*/
    instantaneous_power_consumption_ua->board_voltage_mv            =  3300u;
    instantaneous_power_consumption_ua->init_ua                     =  3150u;
    instantaneous_power_consumption_ua->phase1_gps_capture_ua       = 11900u;
    instantaneous_power_consumption_ua->phase1_gps_process_ua       =  3340u;
    instantaneous_power_consumption_ua->multiscan_gps_capture_ua    = 10700u;
    instantaneous_power_consumption_ua->multiscan_gps_process_ua    =  4180u;
    instantaneous_power_consumption_ua->phase1_beidou_capture_ua    = 13500u;
    instantaneous_power_consumption_ua->phase1_beidou_process_ua    =  3190u;
    instantaneous_power_consumption_ua->multiscan_beidou_capture_ua = 12600u;
    instantaneous_power_consumption_ua->multiscan_beidou_process_ua =  3430u;
    instantaneous_power_consumption_ua->sleep_32k_ua                =  1210u;
    instantaneous_power_consumption_ua->demod_sleep_32m_ua          =  2530u;
}
#endif /* LR11XX_RADIO_CFG_GEOLOCATION_SUPPORT */

/*----------------------------------------------------------------------------*/

#if LR11XX_RADIO_CFG_GEOLOCATION_SUPPORT
SID_STM32_SPEED_OPTIMIZED void geolocation_bsp_gnss_prescan_actions(void)
{
    halo_drv_semtech_ctx_t * const drv_ctx = lr11xx_get_drv_ctx();
    sid_error_t err;
    int32_t radio_err;
    lr11xx_status_t sys_err;
    uint32_t tcxo_start_timeout;

    /* Validate inputs */
    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->config != NULL);

    do
    {
        if (drv_ctx->lbm.gnss_scan_active != FALSE)
        {
            /* Configuration is applied already, nothing to do here */
            LR11XX_RAL_BSP_LOG_DEBUG("LBM_RAL GNSS prescan action skipped");
            err = SID_ERROR_NONE;
            break;
        }
        else
        {
            LR11XX_RAL_BSP_LOG_DEBUG("LBM_RAL GNSS prescan action");
        }

        /* Perform prescan actions */
#  if LR11XX_RADIO_CFG_GNSS_SCAN_USE_BLANKING
        /* Store current LBM bridge operating mode */
        lbm_bridge_mode_on_gnss_scan_entry = sid_pal_radio_lr11xx_get_lbm_bridge_mode();

        /* If LBM bridge is not in exclusive mode request the blanking mode */
        if (lbm_bridge_mode_on_gnss_scan_entry != RADIO_LR11XX_LBM_BRIDGE_STATE_EXCLUSIVE_MODE)
        {
            err = sid_pal_radio_lr11xx_start_lbm_bridge_mode(RADIO_LR11XX_LBM_BRIDGE_STATE_SIDEWALK_BLANKING_MODE);
            if (err != SID_ERROR_NONE)
            {
                LR11XX_RAL_BSP_LOG_ERROR("Failed to activate LBM bridge in Sidewalk blanking mode. Error %d", (int32_t)err);
                break;
            }
        }
        else
        {
            /* Just keep using exclusive mode of LBM */
        }
#  endif /* LR11XX_RADIO_CFG_GNSS_SCAN_USE_BLANKING */

        /* Indicate config changes from here since we might have to revert all or some of the actions below even in case of an error */
        drv_ctx->lbm.gnss_scan_active = TRUE;

#  if LR11XX_RADIO_CFG_USE_STATUS_LED
        /* Turn on GNSS scan LED to signalize the start of scanning */
        (void)lr11xx_radio_hal_gnss_scan_led_on(drv_ctx);
#  endif /* LR11XX_RADIO_CFG_USE_STATUS_LED */

        /* Power on LNA / active antenna */
        radio_err = radio_lr11xx_set_gnss_fem_mode(LR11XX_GNSS_FRONTEND_MODE_ON);
        if (radio_err != RADIO_ERROR_NONE)
        {
            LR11XX_RAL_BSP_LOG_ERROR("Failed to power on GNSS FEM. Error %d", radio_err);
            err = SID_ERROR_IO_ERROR;
            break;
        }

        /* Put the radio into STDBY_RC to configure TCXO settings */
        sys_err = lr11xx_system_set_standby(drv_ctx, LR11XX_SYSTEM_STANDBY_CFG_RC);
        if (sys_err != LR11XX_STATUS_OK)
        {
            SID_PAL_LOG_WARNING("Failed to reconfigure TCXO startup delay for GNSS. Location performance may be degraded");
            err = SID_ERROR_IO_ERROR;
            break;
        }

        if (LR11XX_TCXO_CTRL_VTCXO == drv_ctx->config->tcxo_config.ctrl)
        {
            tcxo_start_timeout = LR11XX_US_TO_TUS(drv_ctx->config->tcxo_config.gnss_timeout_us);
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

        /* Adjust TCXO startup time for better frequency stability */
        sys_err = lr11xx_system_set_tcxo_mode(drv_ctx, drv_ctx->config->tcxo_config.ctrl_voltage, tcxo_start_timeout);
        if (sys_err != LR11XX_STATUS_OK)
        {
            SID_PAL_LOG_WARNING("Failed to reconfigure TCXO startup delay for GNSS. Location performance may be degraded");
            err = SID_ERROR_IO_ERROR;
            break;
        }

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    if (err != SID_ERROR_NONE)
    {
        SID_PAL_LOG_WARNING("LBM: GNSS prescan actions failed. Error %d", (int32_t)err);
    }
}
#endif /* LR11XX_RADIO_CFG_GEOLOCATION_SUPPORT */

/*----------------------------------------------------------------------------*/

#if LR11XX_RADIO_CFG_GEOLOCATION_SUPPORT
SID_STM32_SPEED_OPTIMIZED void geolocation_bsp_gnss_postscan_actions(void)
{
    halo_drv_semtech_ctx_t * const drv_ctx = lr11xx_get_drv_ctx();
    sid_error_t err;
    int32_t radio_err;
    lr11xx_status_t sys_err;
    uint32_t tcxo_start_timeout;

    /* Validate inputs */
    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->config != NULL);

    do
    {
        if (FALSE == drv_ctx->lbm.gnss_scan_active)
        {
            /* Configuration is reverted already, nothing to do here */
            LR11XX_RAL_BSP_LOG_DEBUG("LBM_RAL GNSS postscan action skipped");
            err = SID_ERROR_NONE;
            break;
        }
        else
        {
            LR11XX_RAL_BSP_LOG_DEBUG("LBM_RAL GNSS postscan action");
            drv_ctx->lbm.gnss_scan_active = FALSE;
        }

        /* Power off LNA / active antenna */
        radio_err = radio_lr11xx_set_gnss_fem_mode(LR11XX_GNSS_FRONTEND_MODE_OFF);
        if (radio_err != RADIO_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to power off GNSS FEM. Error %d", radio_err);
            /* Do not terminate here, proceed with the teardown even if FEM power off failed */
        }

        /* Put the radio into STDBY_RC to configure TCXO settings */
        sys_err = lr11xx_system_set_standby(drv_ctx, LR11XX_SYSTEM_STANDBY_CFG_RC);
        if (sys_err != LR11XX_STATUS_OK)
        {
            SID_PAL_LOG_WARNING("Failed to restore TCXO startup delay after GNSS scan. Rx/Tx timings may be affected");
            err = SID_ERROR_IO_ERROR;
            break;
        }

        if (LR11XX_TCXO_CTRL_VTCXO == drv_ctx->config->tcxo_config.ctrl)
        {
            tcxo_start_timeout = LR11XX_US_TO_TUS(drv_ctx->config->tcxo_config.timeout_us);
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

        /* Reconfigure TCXO startup time back to normal */
        sys_err = lr11xx_system_set_tcxo_mode(drv_ctx, drv_ctx->config->tcxo_config.ctrl_voltage, tcxo_start_timeout);
        if (sys_err != LR11XX_STATUS_OK)
        {
            SID_PAL_LOG_WARNING("Failed to restore TCXO startup delay after GNSS scan. Rx/Tx timings may be affected");
        }

#  if LR11XX_RADIO_CFG_GNSS_SCAN_USE_BLANKING
        /* Get current LBM bridge operating mode */
        radio_lr11xx_lbm_bridge_state_t lbm_bridge_mode_on_gnss_scan_exit = sid_pal_radio_lr11xx_get_lbm_bridge_mode();

        /* If LBM bridge was inactive on GNSS scan entry and exclusive mode was not activated during the scan return back to inactive state */
        if ((RADIO_LR11XX_LBM_BRIDGE_STATE_INACTIVE == lbm_bridge_mode_on_gnss_scan_entry) && (lbm_bridge_mode_on_gnss_scan_exit != RADIO_LR11XX_LBM_BRIDGE_STATE_EXCLUSIVE_MODE))
        {
            err = sid_pal_radio_lr11xx_stop_lbm_bridge_mode();
            if (err != SID_ERROR_NONE)
            {
                LR11XX_RAL_BSP_LOG_ERROR("LBM_RAL failed to stop LBM bridge mode after GNSS scan. Error %d", (int32_t)err);
                break;
            }

            /* IMPORTANT: do not perform any radio access after this point as Sidewalk stack may take over the radio control immediately */
        }
        else
#  endif /* LR11XX_RADIO_CFG_GNSS_SCAN_USE_BLANKING */
        {
            /* Perform some cleanup and keep LBM bridge in the same state as it was prior to GNSS scan */

            /* Ensure the radio is free of any residual errors */
            sys_err = lr11xx_system_clear_errors(drv_ctx);
            if (sys_err != LR11XX_STATUS_OK)
            {
                LR11XX_RAL_BSP_LOG_ERROR("GNSS postscan failed to clear radio errors. Error %d", (int32_t)sys_err);
                err = SID_ERROR_IO_ERROR;
                break;
            }

            /* Clear any remaining IRQ flags */
            sys_err = lr11xx_system_clear_irq_status(drv_ctx, LR11XX_SYSTEM_IRQ_ALL_MASK);
            if (sys_err != LR11XX_STATUS_OK)
            {
                LR11XX_RAL_BSP_LOG_ERROR("GNSS postscan failed to clear radio IRQs. Error %d", (int32_t)sys_err);
                err = SID_ERROR_IO_ERROR;
                break;
            }

            /* Put the radio back to sleep mode if required */
            if ((drv_ctx->lbm.radio_sleep_pending != FALSE) && (drv_ctx->radio_state != SID_PAL_RADIO_SLEEP))
            {
                lr11xx_system_sleep_cfg_t cfg = {
                    .is_warm_start   = true,
                    .is_rtc_timeout  = true,
                };

                sys_err = lr11xx_system_set_sleep(drv_ctx, cfg, LR11XX_RADIO_INFINITE_TIME);
                if (sys_err != LR11XX_STATUS_OK)
                {
                    LR11XX_RAL_BSP_LOG_ERROR("Failed to put the radio to sleep. Error %d", (int32_t)sys_err);
                    err = SID_ERROR_IO_ERROR;
                    break;
                }
            }
        }

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    if (err != SID_ERROR_NONE)
    {
        SID_PAL_LOG_WARNING("LBM: GNSS postscan actions failed. Error %d", (int32_t)err);
    }

#  if LR11XX_RADIO_CFG_USE_STATUS_LED
    /* Unconditionally switch off GNSS scan LED */
    (void)lr11xx_radio_hal_gnss_scan_led_off(drv_ctx);
#  endif /* LR11XX_RADIO_CFG_USE_STATUS_LED */
}
#endif /* LR11XX_RADIO_CFG_GEOLOCATION_SUPPORT */

/*----------------------------------------------------------------------------*/

#if LR11XX_RADIO_CFG_GEOLOCATION_SUPPORT
SID_STM32_SPEED_OPTIMIZED void geolocation_bsp_wifi_prescan_actions(void)
{
    halo_drv_semtech_ctx_t * const drv_ctx = lr11xx_get_drv_ctx();
    sid_error_t err;

    /* Validate inputs */
    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->config != NULL);

    do
    {
        /* Configuration is applied already, nothing to do here */
        if (drv_ctx->lbm.wifi_scan_active != FALSE)
        {
            /* Configuration is applied already, nothing to do here */
            LR11XX_RAL_BSP_LOG_DEBUG("LBM_RAL WiFi prescan action skipped");
            err = SID_ERROR_NONE;
            break;
        }
        else
        {
            LR11XX_RAL_BSP_LOG_DEBUG("LBM_RAL WiFi prescan action");
        }

        /* Perform prescan actions */
#  if LR11XX_RADIO_CFG_WIFI_SCAN_USE_BLANKING
        /* Store current LBM bridge operating mode */
        lbm_bridge_mode_on_wifi_scan_entry = sid_pal_radio_lr11xx_get_lbm_bridge_mode();

        /* If LBM bridge is not in exclusive mode request the blanking mode */
        if (lbm_bridge_mode_on_wifi_scan_entry != RADIO_LR11XX_LBM_BRIDGE_STATE_EXCLUSIVE_MODE)
        {
            err = sid_pal_radio_lr11xx_start_lbm_bridge_mode(RADIO_LR11XX_LBM_BRIDGE_STATE_SIDEWALK_BLANKING_MODE);
            if (err != SID_ERROR_NONE)
            {
                LR11XX_RAL_BSP_LOG_ERROR("Failed to activate LBM bridge in Sidewalk blanking mode. Error %d", (int32_t)err);
                break;
            }
        }
        else
        {
            /* Just keep using exclusive mode of LBM */
        }
#  endif /* LR11XX_RADIO_CFG_WIFI_SCAN_USE_BLANKING */

        /* Indicate config changes from here since we might have to revert all or some of the actions below even in case of an error */
        drv_ctx->lbm.wifi_scan_active = TRUE;

#  if LR11XX_RADIO_CFG_USE_STATUS_LED
        /* Turn on WiFi scan LED to signalize the start of scanning */
        (void)lr11xx_radio_hal_wifi_scan_led_on(drv_ctx);
#  endif /* LR11XX_RADIO_CFG_USE_STATUS_LED */

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    if (err != SID_ERROR_NONE)
    {
        SID_PAL_LOG_WARNING("LBM: WiFi prescan actions failed. Error %d", (int32_t)err);
    }
}
#endif /* LR11XX_RADIO_CFG_GEOLOCATION_SUPPORT */

/*----------------------------------------------------------------------------*/

#if LR11XX_RADIO_CFG_GEOLOCATION_SUPPORT
SID_STM32_SPEED_OPTIMIZED void geolocation_bsp_wifi_postscan_actions(void)
{
    halo_drv_semtech_ctx_t * const drv_ctx = lr11xx_get_drv_ctx();
    sid_error_t err;
    lr11xx_status_t sys_err;

    /* Validate inputs */
    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->config != NULL);

    do
    {
        if (FALSE == drv_ctx->lbm.wifi_scan_active)
        {
            /* Configuration is reverted already, nothing to do here */
            LR11XX_RAL_BSP_LOG_DEBUG("LBM_RAL WiFi postscan action skipped");
            err = SID_ERROR_NONE;
            break;
        }
        else
        {
            LR11XX_RAL_BSP_LOG_DEBUG("LBM_RAL WiFi postscan action");
            drv_ctx->lbm.wifi_scan_active = FALSE;
        }

#  if LR11XX_RADIO_CFG_WIFI_SCAN_USE_BLANKING
        /* Get current LBM bridge operating mode */
        radio_lr11xx_lbm_bridge_state_t lbm_bridge_mode_on_wifi_scan_exit = sid_pal_radio_lr11xx_get_lbm_bridge_mode();

        /* If LBM bridge was inactive on GNSS scan entry and exclusive mode was not activated during the scan return back to inactive state */
        if ((RADIO_LR11XX_LBM_BRIDGE_STATE_INACTIVE == lbm_bridge_mode_on_wifi_scan_entry) && (lbm_bridge_mode_on_wifi_scan_exit != RADIO_LR11XX_LBM_BRIDGE_STATE_EXCLUSIVE_MODE))
        {
            err = sid_pal_radio_lr11xx_stop_lbm_bridge_mode();
            if (err != SID_ERROR_NONE)
            {
                LR11XX_RAL_BSP_LOG_ERROR("LBM_RAL failed to stop LBM bridge mode after WiFi scan. Error %d", (int32_t)err);
                break;
            }

            /* IMPORTANT: do not perform any radio access after this point as Sidewalk stack may take over the radio control immediately */
        }
        else
#  endif /* LR11XX_RADIO_CFG_WIFI_SCAN_USE_BLANKING */
        {
            /* Perform some cleanup and keep LBM bridge in the same state as it was prior to WiFi scan */

            /* Ensure the radio is free of any residual errors. This command will also transition the radio into STDBY_RC state */
            sys_err = lr11xx_system_clear_errors(drv_ctx);
            if (sys_err != LR11XX_STATUS_OK)
            {
                LR11XX_RAL_BSP_LOG_ERROR("WiFi postscan failed to clear radio errors. Error %d", (int32_t)sys_err);
                err = SID_ERROR_IO_ERROR;
                break;
            }

            /* Clear any remaining IRQ flags */
            sys_err = lr11xx_system_clear_irq_status(drv_ctx, LR11XX_SYSTEM_IRQ_ALL_MASK);
            if (sys_err != LR11XX_STATUS_OK)
            {
                LR11XX_RAL_BSP_LOG_ERROR("WiFi postscan failed to clear radio IRQs. Error %d", (int32_t)sys_err);
                err = SID_ERROR_IO_ERROR;
                break;
            }

            /* Put the radio back to sleep mode if required */
            if ((drv_ctx->lbm.radio_sleep_pending != FALSE) && (drv_ctx->radio_state != SID_PAL_RADIO_SLEEP))
            {
                lr11xx_system_sleep_cfg_t cfg = {
                    .is_warm_start   = true,
                    .is_rtc_timeout  = true,
                };

                sys_err = lr11xx_system_set_sleep(drv_ctx, cfg, LR11XX_RADIO_INFINITE_TIME);
                if (sys_err != LR11XX_STATUS_OK)
                {
                    LR11XX_RAL_BSP_LOG_ERROR("Failed to put the radio to sleep. Error %d", (int32_t)sys_err);
                    err = SID_ERROR_IO_ERROR;
                    break;
                }
            }
        }

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    if (err != SID_ERROR_NONE)
    {
        SID_PAL_LOG_WARNING("LBM: WiFi postscan actions failed. Error %d", (int32_t)err);
    }

#  if LR11XX_RADIO_CFG_USE_STATUS_LED
    /* Unconditionally switch off WiFi scan LED */
    (void)lr11xx_radio_hal_wifi_scan_led_off(drv_ctx);
#  endif /* LR11XX_RADIO_CFG_USE_STATUS_LED */
}
#endif /* LR11XX_RADIO_CFG_GEOLOCATION_SUPPORT */

/*----------------------------------------------------------------------------*/

#if LR11XX_RADIO_CFG_GEOLOCATION_SUPPORT
SID_STM32_SPEED_OPTIMIZED void geolocation_bsp_get_lr11xx_reg_mode(const void * context, lr11xx_system_reg_mode_t * reg_mode)
{
    const halo_drv_semtech_ctx_t * const drv_ctx = (const halo_drv_semtech_ctx_t *)context;

    /* Validate inputs */
    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->config != NULL);

    *reg_mode = drv_ctx->config->regulator_mode;
}
#endif /* LR11XX_RADIO_CFG_GEOLOCATION_SUPPORT */

/*----------------------------------------------------------------------------*/

#if LR11XX_RADIO_CFG_GEOLOCATION_SUPPORT
SID_STM32_SPEED_OPTIMIZED lr11xx_system_lfclk_cfg_t geolocation_bsp_get_lr11xx_lf_clock_cfg(void)
{
    const halo_drv_semtech_ctx_t * const drv_ctx = lr11xx_get_drv_ctx();

    /* Validate inputs */
    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->config != NULL);

    return drv_ctx->config->lfclock_cfg;
}
#endif /* LR11XX_RADIO_CFG_GEOLOCATION_SUPPORT */
