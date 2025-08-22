/**
  ******************************************************************************
  * @file  stm32wlxx_radio_lora.c
  * @brief LoRa-specific portion of the radio driver for the STM32WLxx Radio App
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

#include "stm32wlxx_radio.h"
#include "stm32wlxx_radio_hal.h"
#include "stm32wlxx_phy.h"
#include "stm32wlxx_phy_timings.h"

#include <sid_lora_phy_cfg.h>
#include <sid_pal_assert_ifc.h>
#include <sid_pal_log_ifc.h>

#include <sid_stm32_common_utils.h>

/* Private defines -----------------------------------------------------------*/

#define STM32WLxx_LORA_RX_PROCESS_SAFETY_GAP_US  (50u) /*!< Rx window will be opened earlier by this amount of microseconds to mitigate any SW execution delays and jitter */

/* Private macros ------------------------------------------------------------*/

#ifndef STM32WLxx_RADIO_LORA_EXTRA_LOGGING
/* Set STM32WLxx_RADIO_LORA_EXTRA_LOGGING to 1 to enable extended logs */
#  define STM32WLxx_RADIO_LORA_EXTRA_LOGGING (0)
#endif

#if STM32WLxx_RADIO_LORA_EXTRA_LOGGING
#  define STM32WLxx_LORA_LOG_ERROR(...)   SID_PAL_LOG_ERROR(__VA_ARGS__)
#  define STM32WLxx_LORA_LOG_WARNING(...) SID_PAL_LOG_WARNING(__VA_ARGS__)
#  define STM32WLxx_LORA_LOG_INFO(...)    SID_PAL_LOG_INFO(__VA_ARGS__)
#  define STM32WLxx_LORA_LOG_DEBUG(...)   SID_PAL_LOG_DEBUG(__VA_ARGS__)
#  define STM32WLxx_LORA_LOG_TRACE(...)   SID_PAL_LOG_TRACE(__VA_ARGS__)
#else
#  define STM32WLxx_LORA_LOG_ERROR(...)   ((void)0u)
#  define STM32WLxx_LORA_LOG_WARNING(...) ((void)0u)
#  define STM32WLxx_LORA_LOG_INFO(...)    ((void)0u)
#  define STM32WLxx_LORA_LOG_DEBUG(...)   ((void)0u)
#  define STM32WLxx_LORA_LOG_TRACE(...)   ((void)0u)
#endif

/* Private constants ---------------------------------------------------------*/

#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
static const uint32_t radio_lora_symb_time[3][8] = {
                                                        { 32768u, 16384u, 8192u, 4096u, 2048u, 1024u, 512u, 256u }, /* 125 KHz */
                                                        { 16384u,  8192u, 4096u, 2048u, 1024u,  512u, 256u, 128u }, /* 250 KHz */
                                                        {  8192u,  4096u, 2048u, 1024u,  512u,  256u, 128u,  64u }, /* 500 KHz */
                                                    };
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 */

/* Private function prototypes -----------------------------------------------*/

static inline uint32_t _lora_need_low_data_rate_optimize(const stm32wlxx_lora_sf_t sf, const stm32wlxx_lora_bw_t bw);
static inline void     _sid_radio_convert_to_stm32wlxx_lora_mod_params(stm32wlxx_mod_params_lora_t * const sx_m, const sid_pal_radio_lora_modulation_params_t * const rd_m);
static inline void     _sid_radio_convert_to_stm32wlxx_lora_pkt_params(stm32wlxx_pkt_params_lora_t * const sx_p, const sid_pal_radio_lora_packet_params_t * const rd_p);

/* Private function definitions ----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline uint32_t _lora_need_low_data_rate_optimize(const stm32wlxx_lora_sf_t sf, const stm32wlxx_lora_bw_t bw)
{
    uint32_t enable_ldr_optimizations;

    if (((bw == STM32WLxx_LORA_BW_125) && ((sf == STM32WLxx_LORA_SF11) || (sf == STM32WLxx_LORA_SF12)))
     || ((bw == STM32WLxx_LORA_BW_250) &&  (sf == STM32WLxx_LORA_SF12)))
    {
        enable_ldr_optimizations = TRUE;
    }
    else
    {
        enable_ldr_optimizations = FALSE;
    }

    return enable_ldr_optimizations;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline void _sid_radio_convert_to_stm32wlxx_lora_mod_params(stm32wlxx_mod_params_lora_t * const sx_m, const sid_pal_radio_lora_modulation_params_t * const rd_m)
{
    sx_m->sf   = (stm32wlxx_lora_sf_t)rd_m->spreading_factor;
    sx_m->bw   = (stm32wlxx_lora_bw_t)rd_m->bandwidth;
    sx_m->ldro = _lora_need_low_data_rate_optimize(rd_m->spreading_factor, rd_m->bandwidth) == FALSE ? 0u : 1u;

    /* unfortunately long interleaved mode is not supported by WL55, so changing it to modes without LI */
    switch (rd_m->coding_rate)
    {
        case STM32WLxx_LORA_CR_4_5:
        case STM32WLxx_LORA_CR_4_5_LI:
            sx_m->cr = STM32WLxx_LORA_CR_4_5;
            break;

        case STM32WLxx_LORA_CR_4_6:
        case STM32WLxx_LORA_CR_4_6_LI:
            sx_m->cr = STM32WLxx_LORA_CR_4_6;
            break;

        case STM32WLxx_LORA_CR_4_7:
            sx_m->cr = STM32WLxx_LORA_CR_4_7;
            break;

        case STM32WLxx_LORA_CR_4_8:
        case STM32WLxx_LORA_CR_4_8_LI:
            sx_m->cr = STM32WLxx_LORA_CR_4_8;
            break;

        default:
            SID_PAL_LOG_ERROR("_sid_radio_convert_to_stm32wlxx_lora_mod_params: coding rate selector is not supported, requested CR is 0x%x", (uint32_t)rd_m->coding_rate);
            break;
    }
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline void _sid_radio_convert_to_stm32wlxx_lora_pkt_params(stm32wlxx_pkt_params_lora_t * const sx_p, const sid_pal_radio_lora_packet_params_t * const rd_p)
{
    sx_p->pbl_len_in_symb = rd_p->preamble_length;
    sx_p->hdr_type =  (stm32wlxx_lora_pkt_len_modes_t)rd_p->header_type;
    sx_p->pld_len_in_bytes = rd_p->payload_length;
    sx_p->crc_is_on = rd_p->crc_mode;
    sx_p->invert_iq_is_on = rd_p->invert_IQ;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_pal_radio_data_rate_t sid_pal_radio_lora_mod_params_to_data_rate(const sid_pal_radio_lora_modulation_params_t * mod_params)
{
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
    sid_pal_radio_data_rate_t dr = SID_PAL_RADIO_DATA_RATE_INVALID;

    if (SID_PAL_RADIO_LORA_BW_500KHZ == mod_params->bandwidth)
    {
        if ((SID_PAL_RADIO_LORA_CODING_RATE_4_5 == mod_params->coding_rate) || (SID_PAL_RADIO_LORA_CODING_RATE_4_5_LI == mod_params->coding_rate))
        {
            if (SID_PAL_RADIO_LORA_SF11 == mod_params->spreading_factor)
            {
                dr = SID_PAL_RADIO_DATA_RATE_2KBPS;
            }
            else if (SID_PAL_RADIO_LORA_SF8 == mod_params->spreading_factor)
            {
                dr = SID_PAL_RADIO_DATA_RATE_12_5KBPS;
            }
            else
            {
                /* Invalid */
            }
        }
        else if (SID_PAL_RADIO_LORA_CODING_RATE_4_6 == mod_params->coding_rate)
        {
            if (SID_PAL_RADIO_LORA_SF7 == mod_params->spreading_factor)
            {
                dr = SID_PAL_RADIO_DATA_RATE_22KBPS;
            }
        }
        else
        {
            /* Invalid */
        }
    }

    return dr;
#else
    (void)mod_params;

    return SID_PAL_RADIO_DATA_RATE_INVALID;
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_lora_data_rate_to_mod_params(sid_pal_radio_lora_modulation_params_t * mod_params, sid_pal_radio_data_rate_t data_rate, uint8_t li_enable)
{
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
    int err = RADIO_ERROR_NONE;

    /* STM32WLxx radio does not support Long Interleaver, so fall back to non-interleaved CR */
    switch (data_rate)
    {
        case SID_PAL_RADIO_DATA_RATE_22KBPS:
            mod_params->spreading_factor = SID_PAL_RADIO_LORA_SF7;
            mod_params->bandwidth        = SID_PAL_RADIO_LORA_BW_500KHZ;
            mod_params->coding_rate      = SID_PAL_RADIO_LORA_CODING_RATE_4_6;
            break;

        case SID_PAL_RADIO_DATA_RATE_12_5KBPS:
            mod_params->spreading_factor = SID_PAL_RADIO_LORA_SF8;
            mod_params->bandwidth        = SID_PAL_RADIO_LORA_BW_500KHZ;
            mod_params->coding_rate      = SID_PAL_RADIO_LORA_CODING_RATE_4_5; /* CR 4/5-LI is not supported */
            break;

        case SID_PAL_RADIO_DATA_RATE_2KBPS:
            mod_params->spreading_factor = SID_PAL_RADIO_LORA_SF11;
            mod_params->bandwidth        = SID_PAL_RADIO_LORA_BW_500KHZ;
            mod_params->coding_rate      = SID_PAL_RADIO_LORA_CODING_RATE_4_5; /* CR 4/5-LI is not supported */
            break;

        default:
            err = RADIO_ERROR_NOT_SUPPORTED;
            break;
    }

    return err;
#else
    (void)mod_params;
    (void)data_rate;
    (void)li_enable;

    return RADIO_ERROR_NOT_SUPPORTED;
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_set_lora_sync_word(uint16_t sync_word)
{
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
    int32_t                          err     = RADIO_ERROR_GENERIC;
    stm32wlxx_hal_status_t           hal_err;
    halo_drv_stm32wlxx_ctx_t * const drv_ctx = (halo_drv_stm32wlxx_ctx_t *)stm32wlxx_radio_get_drv_ctx_ctx();

    do
    {
        /* Check if we have any changes */
        if (drv_ctx->applied_lora_phy_cfg.sync_word == sync_word)
        {
            STM32WLxx_LORA_LOG_DEBUG("Set LoRa sync word - skipped, no changes");
            err = RADIO_ERROR_NONE;
            break;
        }

        hal_err = stm32wlxx_hal_radio_set_syncword(drv_ctx, STM32WLxx_SUBGHZ_RADIO_MODEM_MODE_LORA, (void *)&sync_word, sizeof(sync_word));
        if (hal_err != STM32WLxx_HAL_STATUS_OK)
        {
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Everything is fine, store the update */
        drv_ctx->applied_lora_phy_cfg.sync_word = sync_word;
        err = RADIO_ERROR_NONE;
    } while (0);

    if (err != RADIO_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("Failed to apply LoRa sync word. Error %d", (int32_t)err);
    }

    return err;
#else
    (void)sync_word;

    return RADIO_ERROR_NOT_SUPPORTED;
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_set_lora_symbol_timeout(uint8_t num_of_symbols)
{
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
    int32_t                          err     = RADIO_ERROR_GENERIC;
    stm32wlxx_hal_status_t           hal_err;
    halo_drv_stm32wlxx_ctx_t * const drv_ctx = (halo_drv_stm32wlxx_ctx_t *)stm32wlxx_radio_get_drv_ctx_ctx();

    do
    {
        /* Check if we have any changes */
        if (drv_ctx->applied_lora_phy_cfg.symbol_timeout == num_of_symbols)
        {
            STM32WLxx_LORA_LOG_DEBUG("Set LoRa symbol timeout - skipped, no changes");
            err = RADIO_ERROR_NONE;
            break;
        }

        /* Update is needed */
        hal_err = stm32wlxx_hal_set_lora_symbol_timeout(stm32wlxx_radio_get_drv_ctx_ctx(), num_of_symbols);
        if (hal_err != STM32WLxx_HAL_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to set LoRa symbol timeout. HAL error %u", (uint32_t)hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Everything is fine, store the update */
        drv_ctx->applied_lora_phy_cfg.symbol_timeout = num_of_symbols;
        err = RADIO_ERROR_NONE;
    } while (0);

    if (err != RADIO_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("Failed to apply LoRa symbol timeout. Error %d", (int32_t)err);
    }

    return err;
#else
    (void)num_of_symbols;

    return RADIO_ERROR_NOT_SUPPORTED;
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_set_lora_modulation_params(const sid_pal_radio_lora_modulation_params_t * mod_params)
{
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
    int32_t                                        err               = RADIO_ERROR_GENERIC;
    stm32wlxx_hal_status_t                         hal_err;
    const halo_drv_stm32wlxx_ctx_t * const         drv_ctx           = stm32wlxx_radio_get_drv_ctx_ctx();
    sid_pal_radio_lora_modulation_params_t * const active_mod_params = (sid_pal_radio_lora_modulation_params_t *)&drv_ctx->applied_lora_phy_cfg.lora_modulation_params;

    do
    {
        /* Note: always send out modulation parameters even if there are no changes to keep Rx/Tx setup timings consistent */
        stm32wlxx_radio_lora_phy_mod_params_t hal_mod_params = {
            .bandwidth        = mod_params->bandwidth,
            .coding_rate      = mod_params->coding_rate,
            .spreading_factor = mod_params->spreading_factor,
        };

        hal_err = stm32wlxx_hal_set_lora_modulation_params(drv_ctx, &hal_mod_params);
        if (hal_err != STM32WLxx_HAL_STATUS_OK)
        {
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Everything is fine, store the update */
        SID_STM32_UTIL_fast_memcpy((void *)active_mod_params, mod_params, sizeof(*active_mod_params));
        err = RADIO_ERROR_NONE;
    } while (0);

    if (err != RADIO_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("Failed to apply LoRa modulation params. Error %d", (int32_t)err);
    }

    return err;
#else
    (void)mod_params;

    return RADIO_ERROR_NOT_SUPPORTED;
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_set_lora_packet_params(const sid_pal_radio_lora_packet_params_t * packet_params)
{
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
    int32_t                                    err               = RADIO_ERROR_GENERIC;
    stm32wlxx_hal_status_t                     hal_err;
    const halo_drv_stm32wlxx_ctx_t * const     drv_ctx           = stm32wlxx_radio_get_drv_ctx_ctx();
    sid_pal_radio_lora_packet_params_t * const active_pkt_params = (sid_pal_radio_lora_packet_params_t *)&drv_ctx->applied_lora_phy_cfg.lora_packet_params;

    do
    {
        /* Note: always send out packet parameters even if there are no changes to keep Rx/Tx setup timings consistent */
        stm32wlxx_radio_lora_phy_pkt_params_t hal_pkt_params = {
            .crc_mode        = packet_params->crc_mode,
            .header_type     = packet_params->header_type,
            .invert_IQ       = packet_params->invert_IQ,
            .payload_length  = packet_params->payload_length,
            .preamble_length = packet_params->preamble_length,
        };

        hal_err = stm32wlxx_hal_radio_set_lora_pkt_params(drv_ctx, hal_pkt_params);
        if (hal_err != STM32WLxx_HAL_STATUS_OK)
        {
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Everything is fine, store the update */
        SID_STM32_UTIL_fast_memcpy((void *)active_pkt_params, packet_params, sizeof(*active_pkt_params));
        err = RADIO_ERROR_NONE;
    } while (0);

    if (err != RADIO_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("Failed to apply LoRa packet params. Error %d", (int32_t)err);
    }

    return err;
#else
    (void)packet_params;

    return RADIO_ERROR_NOT_SUPPORTED;
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_set_lora_cad_params(const sid_pal_radio_lora_cad_params_t *cad_params)
{
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
    stm32wlxx_hal_status_t hal_err;
    halo_drv_stm32wlxx_ctx_t * const drv_ctx = (halo_drv_stm32wlxx_ctx_t *)stm32wlxx_radio_get_drv_ctx_ctx();

    drv_ctx->cad_exit_mode = cad_params->cad_exit_mode;

    // FIXME: 1. send actual CAD params, 2. skip sending if there's no change
    hal_err = stm32wlxx_hal_radio_set_lora_cad_params(drv_ctx);
    if (hal_err != STM32WLxx_HAL_STATUS_OK)
    {
        return RADIO_ERROR_HARDWARE_ERROR;
    }

    return RADIO_ERROR_NONE;
#else
    (void)cad_params;

    return RADIO_ERROR_NOT_SUPPORTED;
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED uint32_t sid_pal_radio_lora_cad_duration(uint8_t symbol, const sid_pal_radio_lora_modulation_params_t * mod_params)
{
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
    uint32_t cad_duration;

    if (mod_params != NULL)
    {
        SID_PAL_ASSERT(SID_PAL_RADIO_LORA_BW_125KHZ == 0x04u);
        SID_PAL_ASSERT(mod_params->bandwidth >= SID_PAL_RADIO_LORA_BW_125KHZ);

        SID_PAL_ASSERT(SID_PAL_RADIO_LORA_SF12 == 0x0Cu);
        SID_PAL_ASSERT(mod_params->spreading_factor <= SID_PAL_RADIO_LORA_SF12);

        const uint32_t bw_idx = mod_params->bandwidth - SID_PAL_RADIO_LORA_BW_125KHZ;
        const uint32_t sf_idx = SID_PAL_RADIO_LORA_SF12 - mod_params->spreading_factor;

        SID_PAL_ASSERT(bw_idx < SID_STM32_UTIL_ARRAY_SIZE(radio_lora_symb_time));
        SID_PAL_ASSERT(sf_idx < SID_STM32_UTIL_ARRAY_SIZE(radio_lora_symb_time[0]));

        /* Compute single symbol duration in us for the given modulation params */
        const uint32_t symbol_time_us = radio_lora_symb_time[bw_idx][sf_idx];

        /* Compute CAD timeout based on the CAD duration in symbols */
        cad_duration = symbol_time_us * symbol;
    }
    else
    {
        cad_duration = 0u;
    }

    return cad_duration;
#else
    (void)symbol;
    (void)mod_params;

    return 0u;
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED uint32_t sid_pal_radio_lora_time_on_air(const sid_pal_radio_lora_modulation_params_t * mod_params, const sid_pal_radio_lora_packet_params_t * packet_params, uint8_t packet_len)
{
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
    uint32_t time_on_air;

    if ((NULL == mod_params) || (NULL == packet_params))
    {
        time_on_air = 0u;
    }
    else
    {
        stm32wlxx_pkt_params_lora_t lora_packet_params;
        stm32wlxx_mod_params_lora_t lora_mod_params;

        _sid_radio_convert_to_stm32wlxx_lora_mod_params(&lora_mod_params, mod_params);
        _sid_radio_convert_to_stm32wlxx_lora_pkt_params(&lora_packet_params, packet_params);
        lora_packet_params.pld_len_in_bytes = packet_len;

        time_on_air = stm32wlxx_phy_get_lora_time_on_air_in_ms(&lora_packet_params, &lora_mod_params);
    }

    return time_on_air;
#else
    (void)mod_params;
    (void)packet_params;
    (void)packet_len;

    return 0u;
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED uint32_t sid_pal_radio_lora_get_lora_number_of_symbols(const sid_pal_radio_lora_modulation_params_t * mod_params, uint32_t delay_micro_sec)
{
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
    uint32_t numsymbols;
    if (mod_params != NULL)
    {
        SID_PAL_ASSERT(SID_PAL_RADIO_LORA_BW_125KHZ == 0x04u);
        SID_PAL_ASSERT(mod_params->bandwidth >= SID_PAL_RADIO_LORA_BW_125KHZ);

        SID_PAL_ASSERT(SID_PAL_RADIO_LORA_SF12 == 0x0Cu);
        SID_PAL_ASSERT(mod_params->spreading_factor <= SID_PAL_RADIO_LORA_SF12);

        const uint32_t bw_idx = mod_params->bandwidth - SID_PAL_RADIO_LORA_BW_125KHZ;
        const uint32_t sf_idx = SID_PAL_RADIO_LORA_SF12 - mod_params->spreading_factor;

        SID_PAL_ASSERT(bw_idx < SID_STM32_UTIL_ARRAY_SIZE(radio_lora_symb_time));
        SID_PAL_ASSERT(sf_idx < SID_STM32_UTIL_ARRAY_SIZE(radio_lora_symb_time[0]));

        const uint32_t ts = radio_lora_symb_time[bw_idx][sf_idx];
        numsymbols = (delay_micro_sec + (ts - 1u)) / ts; /* Round up */
    }
    else
    {
        numsymbols = 0u;
    }

    return numsymbols;
#else
    (void)mod_params;
    (void)delay_micro_sec;

    return 0u;
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED uint32_t sid_pal_radio_get_lora_rx_done_delay(const sid_pal_radio_lora_modulation_params_t * mod_params, const sid_pal_radio_lora_packet_params_t * pkt_params)
{
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
    stm32wlxx_pkt_params_lora_t lora_packet_params;
    stm32wlxx_mod_params_lora_t lora_mod_params;

    _sid_radio_convert_to_stm32wlxx_lora_mod_params(&lora_mod_params, mod_params);
    _sid_radio_convert_to_stm32wlxx_lora_pkt_params(&lora_packet_params, pkt_params);

    return stm32wlxx_phy_timings_get_delay_between_last_bit_sent_and_rx_done_in_us(&lora_mod_params, &lora_packet_params);
#else
    (void)mod_params;
    (void)pkt_params;

    return 0u;
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED uint32_t sid_pal_radio_get_lora_tx_process_delay(void)
{
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
    const halo_drv_stm32wlxx_ctx_t * const drv_ctx = stm32wlxx_radio_get_drv_ctx_ctx();
    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->config != NULL);

    const uint32_t lora_tx_process_delay =
        (STM32WLxx_RADIO_CFG_USE_DEFAULT_TIMINGS == drv_ctx->config->processing_timings.lora.tx_process_delay_us) ?
        STM32WLxx_LORA_TX_DEFAULT_PROCESS_DELAY_US : drv_ctx->config->processing_timings.lora.tx_process_delay_us;

    const uint32_t process_delay = lora_tx_process_delay + drv_ctx->config->state_timings.tcxo_delay_us;
    return process_delay;
#else
    return 0u;
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED uint32_t sid_pal_radio_get_lora_rx_process_delay(void)
{
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
    const halo_drv_stm32wlxx_ctx_t * const drv_ctx = stm32wlxx_radio_get_drv_ctx_ctx();
    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->config != NULL);

    const uint32_t lora_rx_process_delay =
        (STM32WLxx_RADIO_CFG_USE_DEFAULT_TIMINGS == drv_ctx->config->processing_timings.lora.rx_process_delay_us) ?
        STM32WLxx_LORA_RX_DEFAULT_PROCESS_DELAY_US : drv_ctx->config->processing_timings.lora.rx_process_delay_us;

    const uint32_t process_delay = lora_rx_process_delay + drv_ctx->config->state_timings.tcxo_delay_us + STM32WLxx_LORA_RX_PROCESS_SAFETY_GAP_US;
    return process_delay;
#else
    return 0u;
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED uint32_t sid_pal_radio_get_lora_symbol_timeout_us(sid_pal_radio_lora_modulation_params_t * mod_params, uint8_t number_of_symbol)
{
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
    uint32_t symbol_timeout;

    if (mod_params != NULL)
    {
        SID_PAL_ASSERT(SID_PAL_RADIO_LORA_BW_125KHZ == 0x04u);
        SID_PAL_ASSERT(mod_params->bandwidth >= SID_PAL_RADIO_LORA_BW_125KHZ);

        SID_PAL_ASSERT(SID_PAL_RADIO_LORA_SF12 == 0x0Cu);
        SID_PAL_ASSERT(mod_params->spreading_factor <= SID_PAL_RADIO_LORA_SF12);

        const uint32_t bw_idx = mod_params->bandwidth - SID_PAL_RADIO_LORA_BW_125KHZ;
        const uint32_t sf_idx = SID_PAL_RADIO_LORA_SF12 - mod_params->spreading_factor;

        SID_PAL_ASSERT(bw_idx < SID_STM32_UTIL_ARRAY_SIZE(radio_lora_symb_time));
        SID_PAL_ASSERT(sf_idx < SID_STM32_UTIL_ARRAY_SIZE(radio_lora_symb_time[0]));

        symbol_timeout = radio_lora_symb_time[bw_idx][sf_idx] * number_of_symbol;
    }
    else
    {
        symbol_timeout = 0u;
    }

    return symbol_timeout;
#else
    (void)mod_params;
    (void)number_of_symbol;

    return 0u;
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 */
}
