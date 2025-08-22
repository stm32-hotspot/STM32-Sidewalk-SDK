/**
  ******************************************************************************
  * @file  stm32wlxx_radio_fsk.c
  * @brief FSK-specific portion of the radio driver for the STM32WLxx Radio App
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

#include <assert.h>
#include <string.h>

#include "stm32wlxx_phy.h"
#include "stm32wlxx_radio.h"
#include "stm32wlxx_radio_hal.h"

#include <sid_fsk_phy_cfg.h>
#include <sid_pal_assert_ifc.h>
#include <sid_pal_log_ifc.h>

#include <sid_stm32_common_utils.h>

/* Private defines -----------------------------------------------------------*/

#define  STM32WLxx_FSK_RX_PROCESS_SAFETY_GAP_US (50u)   /*!< Rx window will be opened earlier by this amount of microseconds to mitigate any SW execution delays and jitter */

#define STM32WLxx_SIDEWALK_FSK_CS_DURATION_US   (1200u) /*!< Carrier Sense timeout for FSK link as defined by Sidewalk specification */

#define WL55_FSK_WHITENING_SEED                 (0x01FFu)
#define WL55_FSK_MAX_PAYLOAD_LENGTH             (255u)
#define WL55_FSK_SYNC_WORD_LENGTH_IN_RX         (3u)
#define MAX_PAYLOAD_LENGTH_WITH_FCS_TYPE_0      (251u)
#define MAX_PAYLOAD_LENGTH_WITH_FCS_TYPE_1      (253u)

#define STM32WLxx_FSK_POLYNOMIAL_CRC16          (0x1021u)
#define STM32WLxx_FSK_POLYNOMIAL_CRC32          (0x04C11DB7u)

/* Private macros ------------------------------------------------------------*/

#ifndef STM32WLxx_RADIO_FSK_EXTRA_LOGGING
/* Set STM32WLxx_RADIO_FSK_EXTRA_LOGGING to 1 to enable extended logs */
#  define STM32WLxx_RADIO_FSK_EXTRA_LOGGING (0)
#endif

#if STM32WLxx_RADIO_FSK_EXTRA_LOGGING
#  define STM32WLxx_FSK_LOG_ERROR(...)   SID_PAL_LOG_ERROR(__VA_ARGS__)
#  define STM32WLxx_FSK_LOG_WARNING(...) SID_PAL_LOG_WARNING(__VA_ARGS__)
#  define STM32WLxx_FSK_LOG_INFO(...)    SID_PAL_LOG_INFO(__VA_ARGS__)
#  define STM32WLxx_FSK_LOG_DEBUG(...)   SID_PAL_LOG_DEBUG(__VA_ARGS__)
#  define STM32WLxx_FSK_LOG_TRACE(...)   SID_PAL_LOG_TRACE(__VA_ARGS__)
#else
#  define STM32WLxx_FSK_LOG_ERROR(...)   ((void)0u)
#  define STM32WLxx_FSK_LOG_WARNING(...) ((void)0u)
#  define STM32WLxx_FSK_LOG_INFO(...)    ((void)0u)
#  define STM32WLxx_FSK_LOG_DEBUG(...)   ((void)0u)
#  define STM32WLxx_FSK_LOG_TRACE(...)   ((void)0u)
#endif

/* Private function prototypes -----------------------------------------------*/

static inline void     _perform_data_whitening(uint16_t seed, const uint8_t * const buffer_in, uint8_t * const buffer_out, const uint16_t length);
static inline uint16_t _compute_crc16(const uint8_t * const buffer, const uint32_t length);
static inline uint32_t _compute_crc32(const uint8_t * const buffer, const uint32_t length);
static inline void     radio_modparams_to_stm32wlxx_modparams(stm32wlxx_radio_fsk_phy_mod_params_t * const fsk_mp, const sid_pal_radio_fsk_modulation_params_t * const mod_params);
static inline void     radio_pktparams_to_stm32wlxx_pktparams(stm32wlxx_pkt_params_gfsk_t *fsk_pp, const sid_pal_radio_fsk_packet_params_t *packet_params);

/* Private function definitions ----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline void _perform_data_whitening(uint16_t seed, const uint8_t * const buffer_in, uint8_t * const buffer_out, const uint16_t length)
{
    uint16_t lfsr    = seed;
    uint16_t xor_out = 0u;
    uint8_t  ret     = 0u;
    uint8_t  result  = 0u;

    for (uint32_t index = 0u; index < length; index++)
    {
        xor_out = 0;
        ret     = 0;
        result  = 0;

        xor_out = ( ( lfsr >> 5 ) & 0x0Fu ) ^ ( lfsr & 0x0Fu );
        lfsr    = (   lfsr >> 4 ) | ( xor_out << 5 );
        ret    |= (   lfsr >> 5 ) & 0x0Fu;

        xor_out = ( ( lfsr >> 5 ) & 0x0Fu ) ^ ( lfsr & 0x0Fu );
        lfsr    = (   lfsr >> 4 ) | ( xor_out << 5 );
        ret    |= ( ( lfsr >> 1 ) & 0xF0u );

        result |= ( ret & 0x80u ) >> 7;
        result |= ( ret & 0x40u ) >> 5;
        result |= ( ret & 0x20u ) >> 3;
        result |= ( ret & 0x10u ) >> 1;
        result |= ( ret & 0x08u ) << 1;
        result |= ( ret & 0x04u ) << 3;
        result |= ( ret & 0x02u ) << 5;
        result |= ( ret & 0x01u ) << 7;

        buffer_out[index] = buffer_in[index] ^ result;
    }
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline uint16_t _compute_crc16(const uint8_t * const buffer, const uint32_t length)
{
    if ((NULL == buffer) || (0u == length))
    {
        return 0u;
    }

    register uint16_t crc16 = 0x0000u;

    for (uint32_t index_buffer = 0u; index_buffer < length; index_buffer++)
    {
        crc16 ^= ( ( uint16_t ) buffer[index_buffer] << 8 );

        for (uint32_t i = 0u; i < 8u; i++ )
        {
            crc16 = (uint16_t)(( crc16 & 0x8000u ) ? ( crc16 << 1 ) ^ STM32WLxx_FSK_POLYNOMIAL_CRC16 : ( crc16 << 1 ));
        }
    }

    return crc16;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline uint32_t _compute_crc32(const uint8_t * const buffer, const uint32_t length)
{
    if ((NULL == buffer) || (0u == length))
    {
        return 0u;
    }

    uint8_t           temp_buffer[sizeof(uint32_t)] = {0};
    register uint32_t crc32                         = 0xFFFFFFFFu;
    const uint8_t *   buffer_local;
    uint32_t          length_local;

    if (length < sizeof(uint32_t))
    {
        SID_STM32_UTIL_fast_memcpy(temp_buffer, buffer, length);
        length_local = sizeof(uint32_t);
        buffer_local = temp_buffer;
    }
    else
    {
        length_local = length;
        buffer_local = buffer;
    }

    for (uint32_t index_buffer = 0u; index_buffer < length_local; index_buffer++)
    {
        crc32 ^= ( index_buffer < length ) ? ( ( uint32_t ) buffer_local[index_buffer] << 24 ) : 0x00000000u;

        for (uint32_t i = 0u; i < 8u; i++ )
        {
            crc32 = ( crc32 & 0x80000000u ) ? ( crc32 << 1 ) ^ STM32WLxx_FSK_POLYNOMIAL_CRC32 : ( crc32 << 1 );
        }
    }

    return ~crc32;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline void radio_modparams_to_stm32wlxx_modparams(stm32wlxx_radio_fsk_phy_mod_params_t * const fsk_mp, const sid_pal_radio_fsk_modulation_params_t * const mod_params)
{
    fsk_mp->bandwidth = (stm32wlxx_gfsk_bw_t)mod_params->bandwidth;
    fsk_mp->bit_rate  = mod_params->bit_rate;
    fsk_mp->freq_dev  = mod_params->freq_dev;
    fsk_mp->shaping   = (stm32wlxx_gfsk_mod_shapes_t)mod_params->mod_shaping;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline void radio_pktparams_to_stm32wlxx_pktparams(stm32wlxx_pkt_params_gfsk_t *fsk_pp, const sid_pal_radio_fsk_packet_params_t *packet_params)
{
    if(packet_params->preamble_length > 1u)
    {
        fsk_pp->pbl_len_in_bits       = (uint16_t)((packet_params->preamble_length - 1u) << 3);
    }
    else
    {
        fsk_pp->pbl_len_in_bits       = 0u;
    }
    fsk_pp->pbl_min_det           = (stm32wlxx_gfsk_pbl_det_t)packet_params->preamble_min_detect;
    fsk_pp->sync_word_len_in_bits = (uint8_t)(packet_params->sync_word_length << 3);
    fsk_pp->addr_cmp              = (stm32wlxx_gfsk_addr_cmp_t)packet_params->addr_comp;
    fsk_pp->hdr_type              = (stm32wlxx_gfsk_pkt_len_modes_t)packet_params->header_type;
    fsk_pp->pld_len_in_bytes      = packet_params->payload_length;
    fsk_pp->crc_type              = (stm32wlxx_gfsk_crc_types_t)packet_params->crc_type;
    fsk_pp->dc_free               = (stm32wlxx_gfsk_dc_free_t)packet_params->radio_whitening_mode;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_pal_radio_data_rate_t sid_pal_radio_fsk_mod_params_to_data_rate(const sid_pal_radio_fsk_modulation_params_t * mp)
{
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_2
    sid_pal_radio_data_rate_t data_rate;

    if (       (RADIO_FSK_BR_50KBPS  == mp->bit_rate) && (RADIO_FSK_FDEV_25KHZ  == mp->freq_dev) && ((uint8_t)SID_PAL_RADIO_FSK_BW_125KHZ == mp->bandwidth))
    {
        data_rate =  SID_PAL_RADIO_DATA_RATE_50KBPS;
    } else if ((RADIO_FSK_BR_150KBPS == mp->bit_rate) && (RADIO_FSK_FDEV_37_5KHZ == mp->freq_dev) && ((uint8_t)SID_PAL_RADIO_FSK_BW_250KHZ == mp->bandwidth))
    {
        data_rate = SID_PAL_RADIO_DATA_RATE_150KBPS;
    } else if ((RADIO_FSK_BR_250KBPS == mp->bit_rate) && (RADIO_FSK_FDEV_62_5KHZ == mp->freq_dev) && ((uint8_t)SID_PAL_RADIO_FSK_BW_500KHZ == mp->bandwidth))
    {
        data_rate = SID_PAL_RADIO_DATA_RATE_250KBPS;
    }
    else
    {
        data_rate = SID_PAL_RADIO_DATA_RATE_INVALID;
    }

    return data_rate;
#else
    (void)mp;

    return SID_PAL_RADIO_DATA_RATE_INVALID;
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_fsk_data_rate_to_mod_params(sid_pal_radio_fsk_modulation_params_t *mod_params, sid_pal_radio_data_rate_t data_rate)
{
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_2
    if (NULL == mod_params)
    {
       return RADIO_ERROR_INVALID_PARAMS;
    }

    switch (data_rate)
    {
        case SID_PAL_RADIO_DATA_RATE_50KBPS:
            mod_params->bit_rate     = RADIO_FSK_BR_50KBPS;
            mod_params->freq_dev     = RADIO_FSK_FDEV_25KHZ;
            mod_params->bandwidth    = (uint8_t)SID_PAL_RADIO_FSK_BW_125KHZ;
            mod_params->mod_shaping  = (uint8_t)STM32WLxx_GFSK_MOD_SHAPE_BT_1;
            break;

        case SID_PAL_RADIO_DATA_RATE_150KBPS:
            mod_params->bit_rate     = RADIO_FSK_BR_150KBPS;
            mod_params->freq_dev     = RADIO_FSK_FDEV_37_5KHZ;
            mod_params->bandwidth    = (uint8_t)SID_PAL_RADIO_FSK_BW_250KHZ;
            mod_params->mod_shaping  = (uint8_t)STM32WLxx_GFSK_MOD_SHAPE_BT_05;
            break;

        case SID_PAL_RADIO_DATA_RATE_250KBPS:
            mod_params->bit_rate     = RADIO_FSK_BR_250KBPS;
            mod_params->freq_dev     = RADIO_FSK_FDEV_62_5KHZ;
            mod_params->bandwidth    = (uint8_t)SID_PAL_RADIO_FSK_BW_500KHZ;
            mod_params->mod_shaping  = (uint8_t)STM32WLxx_GFSK_MOD_SHAPE_BT_05;
            break;

        default:
            return RADIO_ERROR_INVALID_PARAMS;
    }

    return RADIO_ERROR_NONE;
#else
    (void)mod_params;
    (void)data_rate;

    return RADIO_ERROR_NOT_SUPPORTED;
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_prepare_fsk_for_rx(sid_pal_radio_fsk_pkt_cfg_t * rx_pkt_cfg)
{
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_2
    int32_t err = RADIO_ERROR_INVALID_PARAMS;

    do
    {
        if (NULL == rx_pkt_cfg)
        {
            break;
        }

        if ((NULL == rx_pkt_cfg->phy_hdr) || (NULL == rx_pkt_cfg->packet_params) || (NULL == rx_pkt_cfg->sync_word))
        {
            break;
        }

        sid_pal_radio_fsk_packet_params_t *f_pp  =  rx_pkt_cfg->packet_params;
        sid_pal_radio_fsk_phy_hdr_t       *phr   =  rx_pkt_cfg->phy_hdr;
        uint8_t                           *sw    =  rx_pkt_cfg->sync_word;

        f_pp->preamble_min_detect       = (uint8_t)STM32WLxx_GFSK_PBL_DET_16_BITS;
        f_pp->sync_word_length          = WL55_FSK_SYNC_WORD_LENGTH_IN_RX;
        f_pp->addr_comp                 = (uint8_t)STM32WLxx_GFSK_ADDR_CMP_FILT_OFF;
        f_pp->header_type               = (uint8_t)STM32WLxx_GFSK_PKT_FIX_LEN;
        f_pp->payload_length            = WL55_FSK_MAX_PAYLOAD_LENGTH;
        f_pp->crc_type                  = (uint8_t)STM32WLxx_GFSK_CRC_OFF;
        f_pp->radio_whitening_mode      = (uint8_t)STMWLxx_GFSK_DC_FREE_OFF;

        /* The sync word length must be greater than preamble min detect.
         * As preamble min detect is 2 bytes set sync word length to 3 bytes.
         */
        sw[0]                           = 0x55u;
        sw[1]                           = phr->is_fec_enabled ? 0x6Fu : 0x90u;
        sw[2]                           = 0x4Eu;

        err = RADIO_ERROR_NONE;
    } while(0);

    return err;
#else
    (void)rx_pkt_cfg;

    return RADIO_ERROR_NOT_SUPPORTED;
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_prepare_fsk_for_tx(sid_pal_radio_fsk_pkt_cfg_t * tx_pkt_cfg)
{
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_2
    int32_t err = RADIO_ERROR_INVALID_PARAMS;

    do
    {
        if (NULL == tx_pkt_cfg)
        {
            break;
        }

        if ((NULL == tx_pkt_cfg->phy_hdr) || (NULL == tx_pkt_cfg->packet_params) || (NULL == tx_pkt_cfg->sync_word) || (NULL == tx_pkt_cfg->payload))
        {
            break;
        }

        sid_pal_radio_fsk_packet_params_t *f_pp  =  tx_pkt_cfg->packet_params;
        if ((0u == f_pp->payload_length) || (0u == f_pp->preamble_length))
        {
            err = RADIO_ERROR_INVALID_PARAMS;
            break;
        }

        sid_pal_radio_fsk_phy_hdr_t  *phr  = tx_pkt_cfg->phy_hdr;
        if ( ((RADIO_FSK_FCS_TYPE_0 == phr->fcs_type) && (f_pp->payload_length > MAX_PAYLOAD_LENGTH_WITH_FCS_TYPE_0))
          || ((RADIO_FSK_FCS_TYPE_1 == phr->fcs_type) && (f_pp->payload_length > MAX_PAYLOAD_LENGTH_WITH_FCS_TYPE_1)) )
        {
            err = RADIO_ERROR_INVALID_PARAMS;
            break;
        }

        uint8_t   psdu_length      = f_pp->payload_length;
        uint8_t   *tx_buffer       = tx_pkt_cfg->payload;
        uint32_t  crc              = 0x00000000u;

        if (RADIO_FSK_FCS_TYPE_0 == phr->fcs_type)
        {
            crc = _compute_crc32(tx_pkt_cfg->payload, f_pp->payload_length);

            tx_buffer[STM32WLxx_RADIO_COMM_FSK_PHY_HEADER_LENGTH + psdu_length++] = (uint8_t)(crc >> 24);
            tx_buffer[STM32WLxx_RADIO_COMM_FSK_PHY_HEADER_LENGTH + psdu_length++] = (uint8_t)(crc >> 16);
            tx_buffer[STM32WLxx_RADIO_COMM_FSK_PHY_HEADER_LENGTH + psdu_length++] = (uint8_t)(crc >> 8);
            tx_buffer[STM32WLxx_RADIO_COMM_FSK_PHY_HEADER_LENGTH + psdu_length++] = (uint8_t)(crc >> 0);
        }
        else if (RADIO_FSK_FCS_TYPE_1 == phr->fcs_type)
        {
            crc = _compute_crc16(tx_pkt_cfg->payload, f_pp->payload_length);

            tx_buffer[STM32WLxx_RADIO_COMM_FSK_PHY_HEADER_LENGTH + psdu_length++] = (uint8_t)(crc >> 8);
            tx_buffer[STM32WLxx_RADIO_COMM_FSK_PHY_HEADER_LENGTH + psdu_length++] = (uint8_t)(crc >> 0);
        }
        else
        {
            err = RADIO_ERROR_NOT_SUPPORTED;
            break;
        }

        uint8_t  *sync_word               = tx_pkt_cfg->sync_word;
        uint8_t  sync_word_length_in_byte = 0u;
        memmove(tx_buffer + STM32WLxx_RADIO_COMM_FSK_PHY_HEADER_LENGTH, tx_buffer, f_pp->payload_length);
        /* Build the PHR and put it in the Tx buffer */
        tx_buffer[0] = (phr->fcs_type << 4);
        tx_buffer[0] += ((phr->is_data_whitening_enabled == true ) ? 1 : 0) << 3;
        tx_buffer[1] = psdu_length;

        if (phr->is_data_whitening_enabled != false)
        {
            _perform_data_whitening(WL55_FSK_WHITENING_SEED, tx_buffer + STM32WLxx_RADIO_COMM_FSK_PHY_HEADER_LENGTH,
                                    tx_buffer + STM32WLxx_RADIO_COMM_FSK_PHY_HEADER_LENGTH, psdu_length);
        }

        /* Build the syncword */
        sync_word[sync_word_length_in_byte++] = 0x55u;  /* Added to force the preamble polarity to a real "0x55" */
        sync_word[sync_word_length_in_byte++] = (phr->is_fec_enabled == true) ? 0x6Fu : 0x90u;
        sync_word[sync_word_length_in_byte++] = 0x4Eu;

        f_pp->sync_word_length     = sync_word_length_in_byte;
        f_pp->addr_comp            = (uint8_t)STM32WLxx_GFSK_ADDR_CMP_FILT_OFF;
        f_pp->header_type          = (uint8_t)STM32WLxx_GFSK_PKT_FIX_LEN;
        f_pp->payload_length       = STM32WLxx_RADIO_COMM_FSK_PHY_HEADER_LENGTH + psdu_length;
        f_pp->crc_type             = (uint8_t)STM32WLxx_GFSK_CRC_OFF;
        f_pp->radio_whitening_mode = (uint8_t)STMWLxx_GFSK_DC_FREE_OFF;

        err = RADIO_ERROR_NONE;
    } while(0);

    return err;
#else
    (void)tx_pkt_cfg;

    return RADIO_ERROR_NOT_SUPPORTED;
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_set_fsk_sync_word(const uint8_t * sync_word, uint8_t sync_word_length)
{
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_2
    int32_t                          err     = RADIO_ERROR_GENERIC;
    stm32wlxx_hal_status_t           hal_err;
    halo_drv_stm32wlxx_ctx_t * const drv_ctx = (halo_drv_stm32wlxx_ctx_t *)stm32wlxx_radio_get_drv_ctx_ctx();

    do
    {
        /* Check if we have any changes */
        if ((drv_ctx->applied_fsk_phy_cfg.sync_word_len == sync_word_length)
          && (SID_STM32_UTIL_fast_memcmp(drv_ctx->applied_fsk_phy_cfg.sync_word, sync_word, sync_word_length) == 0u))
        {
            STM32WLxx_FSK_LOG_DEBUG("Set FSK sync word - skipped, no changes");
            err = RADIO_ERROR_NONE;
            break;
        }

        hal_err = stm32wlxx_hal_radio_set_syncword(drv_ctx, STM32WLxx_SUBGHZ_RADIO_MODEM_MODE_FSK, sync_word, sync_word_length);
        if (hal_err != STM32WLxx_HAL_STATUS_OK)
        {
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Everything is fine, store the update */
        drv_ctx->applied_fsk_phy_cfg.sync_word_len = sync_word_length;
        SID_STM32_UTIL_fast_memcpy(drv_ctx->applied_fsk_phy_cfg.sync_word, sync_word, sync_word_length);
        err = RADIO_ERROR_NONE;
    } while (0);

    if (err != RADIO_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("Failed to apply FSK sync word. Error %d", (int32_t)err);
    }

    return err;
#else
    (void)sync_word;
    (void)sync_word_length;

    return RADIO_ERROR_NOT_SUPPORTED;
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_set_fsk_whitening_seed(uint16_t seed)
{
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_2
    //FIXME: at least check it did not change. Or better send an update to WLxx
    return RADIO_ERROR_NONE;
#else
    (void)seed;

    return RADIO_ERROR_NOT_SUPPORTED;
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_set_fsk_modulation_params(const sid_pal_radio_fsk_modulation_params_t * mod_params)
{
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_2
    int32_t                                       err               = RADIO_ERROR_GENERIC;
    stm32wlxx_hal_status_t                        hal_err;
    const halo_drv_stm32wlxx_ctx_t * const        drv_ctx           = stm32wlxx_radio_get_drv_ctx_ctx();
    sid_pal_radio_fsk_modulation_params_t * const active_mod_params = (sid_pal_radio_fsk_modulation_params_t *)&drv_ctx->applied_fsk_phy_cfg.fsk_modulation_params;

    do
    {
        /* Note: always send out modulation parameters even if there are no changes to keep Rx/Tx setup timings consistent */
        stm32wlxx_radio_fsk_phy_mod_params_t hal_mod_params = {
            .bit_rate  = mod_params->bit_rate,
            .freq_dev  = mod_params->freq_dev,
            .shaping   = mod_params->mod_shaping,
        };

        _Static_assert(STM32WLxx_GFSK_BW_4800   == SID_PAL_RADIO_FSK_BW_4800   , "Invalid conversion");
        /* STM32WLxx_GFSK_BW_5800   != SID_PAL_RADIO_FSK_BW_5800 */
        _Static_assert(STM32WLxx_GFSK_BW_7300   == SID_PAL_RADIO_FSK_BW_7300   , "Invalid conversion");
        _Static_assert(STM32WLxx_GFSK_BW_9700   == SID_PAL_RADIO_FSK_BW_9700   , "Invalid conversion");
        _Static_assert(STM32WLxx_GFSK_BW_11700  == SID_PAL_RADIO_FSK_BW_11700  , "Invalid conversion");
        _Static_assert(STM32WLxx_GFSK_BW_14600  == SID_PAL_RADIO_FSK_BW_14600  , "Invalid conversion");
        _Static_assert(STM32WLxx_GFSK_BW_19500  == SID_PAL_RADIO_FSK_BW_19500  , "Invalid conversion");
        _Static_assert(STM32WLxx_GFSK_BW_23400  == SID_PAL_RADIO_FSK_BW_23400  , "Invalid conversion");
        _Static_assert(STM32WLxx_GFSK_BW_29300  == SID_PAL_RADIO_FSK_BW_29300  , "Invalid conversion");
        _Static_assert(STM32WLxx_GFSK_BW_39000  == SID_PAL_RADIO_FSK_BW_39000  , "Invalid conversion");
        _Static_assert(STM32WLxx_GFSK_BW_46900  == SID_PAL_RADIO_FSK_BW_46900  , "Invalid conversion");
        _Static_assert(STM32WLxx_GFSK_BW_58600  == SID_PAL_RADIO_FSK_BW_58600  , "Invalid conversion");
        _Static_assert(STM32WLxx_GFSK_BW_78200  == SID_PAL_RADIO_FSK_BW_78200  , "Invalid conversion");
        _Static_assert(STM32WLxx_GFSK_BW_93800  == SID_PAL_RADIO_FSK_BW_93800  , "Invalid conversion");
        _Static_assert(STM32WLxx_GFSK_BW_117300 == SID_PAL_RADIO_FSK_BW_117300 , "Invalid conversion");
        _Static_assert(STM32WLxx_GFSK_BW_156200 == SID_PAL_RADIO_FSK_BW_156200 , "Invalid conversion");
        _Static_assert(STM32WLxx_GFSK_BW_187200 == SID_PAL_RADIO_FSK_BW_187200 , "Invalid conversion");
        _Static_assert(STM32WLxx_GFSK_BW_234300 == SID_PAL_RADIO_FSK_BW_234300 , "Invalid conversion");
        _Static_assert(STM32WLxx_GFSK_BW_312000 == SID_PAL_RADIO_FSK_BW_312000 , "Invalid conversion");
        _Static_assert(STM32WLxx_GFSK_BW_373600 == SID_PAL_RADIO_FSK_BW_373600 , "Invalid conversion");
        _Static_assert(STM32WLxx_GFSK_BW_467000 == SID_PAL_RADIO_FSK_BW_467000 , "Invalid conversion");

        switch (mod_params->bandwidth)
        {
            case SID_PAL_RADIO_FSK_BW_5800:
                hal_mod_params.bandwidth = STM32WLxx_GFSK_BW_5800;
                break;

            case SID_PAL_RADIO_FSK_BW_125KHZ:
                hal_mod_params.bandwidth = STM32WLxx_GFSK_BW_117300;
                break;

            case SID_PAL_RADIO_FSK_BW_250KHZ:
                hal_mod_params.bandwidth = STM32WLxx_GFSK_BW_234300;
                break;

            case SID_PAL_RADIO_FSK_BW_500KHZ:
                hal_mod_params.bandwidth = STM32WLxx_GFSK_BW_467000;
                break;

            default:
                hal_mod_params.bandwidth = mod_params->bandwidth;
                break;
        }

        hal_err = stm32wlxx_hal_set_fsk_modulation_params(drv_ctx, &hal_mod_params);
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
        SID_PAL_LOG_ERROR("Failed to apply FSK modulation params. Error %d", (int32_t)err);
    }

    return err;
#else
    (void)mod_params;

    return RADIO_ERROR_NOT_SUPPORTED;
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_set_fsk_packet_params(const sid_pal_radio_fsk_packet_params_t * packet_params)
{
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_2
    int32_t                                   err               = RADIO_ERROR_GENERIC;
    stm32wlxx_hal_status_t                    hal_err;
    const halo_drv_stm32wlxx_ctx_t * const    drv_ctx           = stm32wlxx_radio_get_drv_ctx_ctx();
    sid_pal_radio_fsk_packet_params_t * const active_pkt_params = (sid_pal_radio_fsk_packet_params_t *)&drv_ctx->applied_fsk_phy_cfg.fsk_packet_params;

    do
    {
        /* Note: always send out packet parameters even if there are no changes to keep Rx/Tx setup timings consistent */
        stm32wlxx_pal_radio_fsk_pkt_params_t hal_pkt_params = {
            .preamble_length      = packet_params->preamble_length,
            .preamble_min_detect  = packet_params->preamble_min_detect,
            .sync_word_length     = packet_params->sync_word_length,
            .addr_comp            = packet_params->addr_comp,
            .header_type          = packet_params->header_type,
            .payload_length       = packet_params->payload_length,
            .crc_type             = packet_params->crc_type,
            .radio_whitening_mode = packet_params->radio_whitening_mode,
        };

        hal_err = stm32wlxx_hal_radio_set_fsk_pkt_params(drv_ctx, hal_pkt_params);
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
        SID_PAL_LOG_ERROR("Failed to apply FSK packet params. Error %d", (int32_t)err);
    }

    return err;
#else
    (void)packet_params;

    return RADIO_ERROR_NOT_SUPPORTED;
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED uint32_t sid_pal_radio_fsk_time_on_air(const sid_pal_radio_fsk_modulation_params_t * mod_params, const sid_pal_radio_fsk_packet_params_t * packet_params,
                                                                 uint8_t packetLen)
{
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_2
    uint32_t time_on_air;

    if ((NULL == mod_params) || (NULL == packet_params))
    {
        time_on_air = 0u;
    }
    else
    {
        stm32wlxx_pkt_params_gfsk_t          fsk_pp;
        stm32wlxx_radio_fsk_phy_mod_params_t fsk_mp;

        fsk_pp.pld_len_in_bytes = packetLen;
        radio_pktparams_to_stm32wlxx_pktparams(&fsk_pp, packet_params);
        radio_modparams_to_stm32wlxx_modparams(&fsk_mp, mod_params);

        time_on_air = stm32wlxx_phy_get_gfsk_time_on_air_in_ms(&fsk_pp, &fsk_mp);
    }

    return time_on_air;
#else
    (void)mod_params;
    (void)packet_params;
    (void)packetLen;

    return 0u;
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED uint32_t sid_pal_radio_fsk_get_fsk_number_of_symbols(const sid_pal_radio_fsk_modulation_params_t * mod_params, uint32_t delay_micro_secs)
{
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_2
    uint32_t num_symb = STM32WLxx_SUBGHZ_US_TO_SYMBOLS(delay_micro_secs, mod_params->bit_rate);
    return num_symb;
#else
    (void)mod_params;
    (void)delay_micro_secs;

    return 0u;
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED uint32_t sid_pal_radio_get_fsk_tx_process_delay(void)
{
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_2
    const halo_drv_stm32wlxx_ctx_t * const drv_ctx = stm32wlxx_radio_get_drv_ctx_ctx();
    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->config != NULL);

    const uint32_t fsk_tx_process_delay =
        (STM32WLxx_RADIO_CFG_USE_DEFAULT_TIMINGS == drv_ctx->config->processing_timings.fsk.tx_process_delay_us) ?
        STM32WLxx_FSK_TX_DEFAULT_PROCESS_DELAY_US : drv_ctx->config->processing_timings.fsk.tx_process_delay_us;

    const uint32_t process_delay = fsk_tx_process_delay + drv_ctx->config->state_timings.tcxo_delay_us - STM32WLxx_SIDEWALK_FSK_CS_DURATION_US;
    return process_delay;
#else
    return 0u;
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED uint32_t sid_pal_radio_get_fsk_rx_process_delay(void)
{
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_2
    const halo_drv_stm32wlxx_ctx_t * const drv_ctx = stm32wlxx_radio_get_drv_ctx_ctx();
    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->config != NULL);

    const uint32_t fsk_rx_process_delay =
        (STM32WLxx_RADIO_CFG_USE_DEFAULT_TIMINGS == drv_ctx->config->processing_timings.fsk.rx_process_delay_us) ?
        STM32WLxx_FSK_RX_DEFAULT_PROCESS_DELAY_US : drv_ctx->config->processing_timings.fsk.rx_process_delay_us;

    const uint32_t process_delay = fsk_rx_process_delay + drv_ctx->config->state_timings.tcxo_delay_us + STM32WLxx_FSK_RX_PROCESS_SAFETY_GAP_US;
    return process_delay;
#else
    return 0u;
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 */
}
