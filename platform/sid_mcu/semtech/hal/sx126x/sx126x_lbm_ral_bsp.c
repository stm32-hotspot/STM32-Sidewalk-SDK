/**
  ******************************************************************************
  * @file    sx126x_lbm_ral_bsp.c
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
#include <smtc_ralf_sx126x.h>
#include <smtc_ral_sx126x_bsp.h>

/* Sidewalk SX126x driver */
#include "sx126x_radio.h"
#include "sx126x_hal.h"
#include "sx126x_radio_config.h"

/* Platform-specific includes */
#include <sid_stm32_common_utils.h>

/* Private defines -----------------------------------------------------------*/

#ifndef SX126XLBM_RAL_BSP_EXTRA_LOGGING
/* Set SX126XLBM_RAL_BSP_EXTRA_LOGGING to 1 to enable extended logs */
#  define SX126XLBM_RAL_BSP_EXTRA_LOGGING (0)
#endif

#if SX126XLBM_RAL_BSP_EXTRA_LOGGING
#  define SX126XRAL_BSP_LOG_ERROR(...)    SID_PAL_LOG_ERROR(__VA_ARGS__)
#  define SX126XRAL_BSP_LOG_WARNING(...)  SID_PAL_LOG_WARNING(__VA_ARGS__)
#  define SX126XRAL_BSP_LOG_INFO(...)     SID_PAL_LOG_INFO(__VA_ARGS__)
#  define SX126XRAL_BSP_LOG_DEBUG(...)    SID_PAL_LOG_DEBUG(__VA_ARGS__)
#  define SX126XRAL_BSP_LOG_TRACE(...)    SID_PAL_LOG_TRACE(__VA_ARGS__)
#else
#  define SX126XRAL_BSP_LOG_ERROR(...)    ((void)0u)
#  define SX126XRAL_BSP_LOG_WARNING(...)  ((void)0u)
#  define SX126XRAL_BSP_LOG_INFO(...)     ((void)0u)
#  define SX126XRAL_BSP_LOG_DEBUG(...)    ((void)0u)
#  define SX126XRAL_BSP_LOG_TRACE(...)    ((void)0u)
#endif /* SX126XLBM_RAL_BSP_EXTRA_LOGGING */

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
    const halo_drv_semtech_ctx_t * const drv_ctx = sx126x_get_drv_ctx();
    const ralf_t * ralf_instance = NULL;

    do
    {
        /* Validate inputs */
        if ((NULL == drv_ctx) || (FALSE == drv_ctx->init_done))
        {
            /* Driver is not initialized properly */
            SID_PAL_LOG_ERROR("Can't init RAL BSP. SX126x Sidewalk driver is not initialized");
            break;
        }

        /* Store SX126x driver context to LBM */
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

SID_STM32_SPEED_OPTIMIZED void ral_sx126x_bsp_get_tx_cfg(const void * context, const ral_sx126x_bsp_tx_cfg_input_params_t * input_params, ral_sx126x_bsp_tx_cfg_output_params_t * output_params)
{
    const halo_drv_semtech_ctx_t * const drv_ctx = (const halo_drv_semtech_ctx_t *)context;
    int32_t sid_radio_err;

    /* Validate inputs */
    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->config != NULL);
    SID_PAL_ASSERT(drv_ctx->config->pa_config.pa_cfg_callback != NULL);

    do
    {
        /* Use PA configuration resolver from Sidewalk driver */
        sid_radio_err = radio_sx126x_compute_tx_power_config(input_params->system_output_pwr_in_dbm);
        if (sid_radio_err != RADIO_ERROR_NONE)
        {
            SX126XRAL_BSP_LOG_ERROR("Failed to set SX126x Tx power to %ddB. Error %d", input_params->system_output_pwr_in_dbm, sid_radio_err);

            /* Fill-in drv_ctx.pa_cfg with invalid values */
            SID_STM32_UTIL_fast_memset(output_params, SX126X_PA_CFG_PARAM_INVALID, sizeof(*output_params));
            break;
        }

        output_params->pa_cfg.pa_duty_cycle              = drv_ctx->pa_cfg.pa_duty_cycle;
        output_params->pa_cfg.hp_max                     = drv_ctx->pa_cfg.hp_max;
        output_params->pa_cfg.pa_lut                     = drv_ctx->pa_cfg.pa_lut;
        output_params->pa_ramp_time                      = drv_ctx->pa_cfg.ramp_time;
        output_params->chip_output_pwr_in_dbm_expected   = drv_ctx->pa_cfg.target_tx_power;
        output_params->chip_output_pwr_in_dbm_configured = drv_ctx->pa_cfg.tx_power_reg;

        switch (drv_ctx->config->id)
        {
            case SEMTECH_ID_SX1261:
                output_params->pa_cfg.device_sel = SX126X_PA_CFG_USE_LPA;
                break;

            case SEMTECH_ID_SX1262:
                output_params->pa_cfg.device_sel = SX126X_PA_CFG_USE_HPA;
                break;

            default:
                output_params->pa_cfg.device_sel = SX126X_PA_CFG_PARAM_INVALID;
                SX126XRAL_BSP_LOG_ERROR("0x%02X is not a valid SX126x transceiver version specification", drv_ctx->config->id);
                SID_PAL_ASSERT(0);
                break;
        }
    } while (0);
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED void ral_sx126x_bsp_get_xosc_cfg(const void * context, ral_xosc_cfg_t * xosc_cfg, sx126x_tcxo_ctrl_voltages_t * supply_voltage, uint32_t * startup_time_in_tick)
{
    const halo_drv_semtech_ctx_t * const drv_ctx = (const halo_drv_semtech_ctx_t *)context;

    /* Validate inputs */
    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->config != NULL);

    /* Map Sidewalk radio configuration settings for TCXO control to LBM RAL */
    switch (drv_ctx->config->tcxo_config.ctrl)
    {
        case SX126X_TCXO_CTRL_VDD:
            *xosc_cfg = RAL_XOSC_CFG_TCXO_EXT_CTRL;
            break;

        case SX126X_TCXO_CTRL_DIO3:
            *xosc_cfg = RAL_XOSC_CFG_TCXO_RADIO_CTRL;
            break;

        case SX126X_TCXO_CTRL_NONE:
        default:
            *xosc_cfg = RAL_XOSC_CFG_XTAL;
            break;
    }

    /* Copy other settings from Sidewalk radio config */
    *supply_voltage       = drv_ctx->config->tcxo_config.ctrl_voltage;
    *startup_time_in_tick = SX126XUS_TO_TUS(drv_ctx->config->tcxo_config.timeout_us);
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED void ral_sx126x_bsp_get_trim_cap(const void * context, uint8_t * trimming_cap_xta, uint8_t * trimming_cap_xtb)
{
    const halo_drv_semtech_ctx_t * const drv_ctx = (const halo_drv_semtech_ctx_t *)context;

    /* Validate inputs */
    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->config != NULL);

    *trimming_cap_xta = (uint8_t)(drv_ctx->trim >> 8);
    *trimming_cap_xtb = (uint8_t)(drv_ctx->trim & 0xFFu);
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED void ral_sx126x_bsp_get_rx_boost_cfg(const void * context, bool * rx_boost_is_activated)
{
    const halo_drv_semtech_ctx_t * const drv_ctx = (const halo_drv_semtech_ctx_t *)context;

    /* Validate inputs */
    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->config != NULL);

    /* Get Rx Boost mode configuration from from Sidewalk radio config */
    *rx_boost_is_activated = (drv_ctx->config->pa_config.rx_boost_en != FALSE);
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED void ral_sx126x_bsp_get_ocp_value(const void * context, uint8_t * ocp_in_step_of_2_5_ma)
{
    const halo_drv_semtech_ctx_t * const drv_ctx = (const halo_drv_semtech_ctx_t *)context;

    /* Validate inputs */
    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->config != NULL);

    /* Select transceiver-specific params - Sidewalk radio driver configures OCP at initialization so we don't need to read the actual OCP register here */
    if (SEMTECH_ID_SX1261 == drv_ctx->config->id)
    {
        *ocp_in_step_of_2_5_ma = SX1261_DEFAULT_OCP_VAL;
    }
    else if (SEMTECH_ID_SX1262 == drv_ctx->config->id)
    {
        *ocp_in_step_of_2_5_ma = SX1262_DEFAULT_OCP_VAL;
    }
    else
    {
        /* Normally this should never happen. Set OCP value to 0 to force LBM to overwrite the register in SX126x */
        *ocp_in_step_of_2_5_ma = 0u;
    }
}

/*----------------------------------------------------------------------------*/

__WEAK SID_STM32_SPEED_OPTIMIZED void ral_sx126x_bsp_get_lora_cad_det_peak(const void * context, ral_lora_sf_t sf, ral_lora_bw_t bw, ral_lora_cad_symbs_t nb_symbol, uint8_t * in_out_cad_det_peak)
{
    /* Accept the proposed in_out_cad_det_peak value by default */
    (void)context;
    (void)sf;
    (void)bw;
    (void)nb_symbol;
    (void)in_out_cad_det_peak;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED void ral_sx126x_bsp_get_rf_switch_cfg(const void * context, bool * dio2_is_set_as_rf_switch)
{
    const halo_drv_semtech_ctx_t * const drv_ctx = (const halo_drv_semtech_ctx_t *)context;

    /* Validate inputs */
    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->config != NULL);

    /* Copy SX126x DIO config for RF switch control from Sidewalk radio config */
    *dio2_is_set_as_rf_switch = (drv_ctx->config->pa_config.dio2_ctrl_en != FALSE);
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED void ral_sx126x_bsp_get_reg_mode(const void * context, sx126x_reg_mod_t * reg_mode)
{
    const halo_drv_semtech_ctx_t * const drv_ctx = (const halo_drv_semtech_ctx_t *)context;

    /* Validate inputs */
    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->config != NULL);

    /* Get regulator mode from Sidewalk radio config */
    *reg_mode = drv_ctx->config->regulator_mode;
}

/*----------------------------------------------------------------------------*/

__WEAK SID_STM32_SPEED_OPTIMIZED ral_status_t ral_sx126x_bsp_get_instantaneous_tx_power_consumption(const void * context, const ral_sx126x_bsp_tx_cfg_output_params_t * tx_cfg_output_params_local, sx126x_reg_mod_t radio_reg_mode, uint32_t * pwr_consumption_in_ua)
{
    (void)context;
    (void)tx_cfg_output_params_local;
    (void)radio_reg_mode;
    *pwr_consumption_in_ua = 0u;

    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

/*----------------------------------------------------------------------------*/

__WEAK SID_STM32_SPEED_OPTIMIZED ral_status_t ral_sx126x_bsp_get_instantaneous_gfsk_rx_power_consumption(const void * context, sx126x_reg_mod_t radio_reg_mode, bool rx_boosted, uint32_t* pwr_consumption_in_ua)
{
    (void)context;
    (void)radio_reg_mode;
    (void)rx_boosted;
    *pwr_consumption_in_ua = 0u;

    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

/*----------------------------------------------------------------------------*/

__WEAK SID_STM32_SPEED_OPTIMIZED ral_status_t ral_sx126x_bsp_get_instantaneous_lora_rx_power_consumption(const void * context, sx126x_reg_mod_t radio_reg_mode, bool rx_boosted, uint32_t * pwr_consumption_in_ua)
{
    (void)context;
    (void)radio_reg_mode;
    (void)rx_boosted;
    *pwr_consumption_in_ua = 0u;

    return RAL_STATUS_UNSUPPORTED_FEATURE;
}
