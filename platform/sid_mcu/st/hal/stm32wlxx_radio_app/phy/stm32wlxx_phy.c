/**
 * @file      stm32wlxx_phy.c
 *
 * @brief     SX126x radio driver implementation
 *
 * Revised BSD License
 * Copyright Semtech Corporation 2020. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/**
  ******************************************************************************
  * @file    stm32wlxx_phy.c
  * @brief   STM32WLxx Sub-GHz PHY helper functions implementation
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

#include <string.h>

#include "stm32wlxx_phy.h"
#include "comm_def.h"

#include <sid_stm32_common_utils.h>

/* Private function prototypes -----------------------------------------------*/

static inline uint32_t _get_gfsk_crc_len_in_bytes(const stm32wlxx_gfsk_crc_types_t crc_type);
static inline uint32_t _get_lora_bw_in_hz(const stm32wlxx_lora_bw_t bw);
static inline uint32_t _get_lora_time_on_air_numerator(const stm32wlxx_pkt_params_lora_t * const pkt_p, const stm32wlxx_mod_params_lora_t * const mod_p);
static inline uint32_t _get_gfsk_time_on_air_numerator(const stm32wlxx_pkt_params_gfsk_t * const pkt_p);

/* Private function definitions ----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline uint32_t _get_gfsk_crc_len_in_bytes(const stm32wlxx_gfsk_crc_types_t crc_type)
{
    uint32_t crc_len_bytes;

    switch (crc_type)
    {
        case STM32WLxx_GFSK_CRC_1_BYTE:
        case STM32WLxx_GFSK_CRC_1_BYTE_INV:
            crc_len_bytes = 1u;
            break;

        case STM32WLxx_GFSK_CRC_2_BYTES:
        case STM32WLxx_GFSK_CRC_2_BYTES_INV:
            crc_len_bytes = 2u;
            break;

        case STM32WLxx_GFSK_CRC_OFF:
        default:
            crc_len_bytes = 0u;
            break;
    }

    return crc_len_bytes;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline uint32_t _get_lora_bw_in_hz(const stm32wlxx_lora_bw_t bw)
{
    uint32_t bw_in_hz;

    switch (bw)
    {
        case STM32WLxx_LORA_BW_007:
            bw_in_hz = 7812u;
            break;

        case STM32WLxx_LORA_BW_010:
            bw_in_hz = 10417u;
            break;

        case STM32WLxx_LORA_BW_015:
            bw_in_hz = 15625u;
            break;

        case STM32WLxx_LORA_BW_020:
            bw_in_hz = 20833u;
            break;

        case STM32WLxx_LORA_BW_031:
            bw_in_hz = 31250u;
            break;

        case STM32WLxx_LORA_BW_041:
            bw_in_hz = 41667u;
            break;

        case STM32WLxx_LORA_BW_062:
            bw_in_hz = 62500u;
            break;

        case STM32WLxx_LORA_BW_125:
            bw_in_hz = 125000u;
            break;

        case STM32WLxx_LORA_BW_250:
            bw_in_hz = 250000u;
            break;

        case STM32WLxx_LORA_BW_500:
            bw_in_hz = 500000u;
            break;

        default:
            bw_in_hz = 0u;
            break;
    }

    return bw_in_hz;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline uint32_t _get_lora_time_on_air_numerator(const stm32wlxx_pkt_params_lora_t * const pkt_p, const stm32wlxx_mod_params_lora_t * const mod_p)
{
    const int32_t pld_len_in_bytes = pkt_p->pld_len_in_bytes;
    const int32_t sf               = mod_p->sf;
    const bool    pld_is_fix       = pkt_p->hdr_type == STM32WLxx_LORA_PKT_IMPLICIT;

    int32_t  fine_synch        = ( sf <= 6 ) ? 1 : 0;
    bool     long_interleaving = ( mod_p->cr > 4 );

    int32_t total_bytes_nb = pld_len_in_bytes + ( pkt_p->crc_is_on ? 2 : 0 );
    int32_t tx_bits_symbol = sf - 2 * ( mod_p->ldro != 0 ? 1 : 0 );

    int32_t ceil_numerator;
    int32_t ceil_denominator;

    int32_t intermed;

    int32_t symbols_nb_data;
    int32_t  tx_infobits_header;
    int32_t  tx_infobits_payload;

    if( long_interleaving )
    {
        const int32_t fec_rate_numerator   = 4;
        const int32_t fec_rate_denominator = ( mod_p->cr + ( mod_p->cr == 7 ? 1 : 0 ) );

        if( pld_is_fix )
        {
            int32_t tx_bits_symbol_start = sf - 2 + 2 * fine_synch;
            if( 8 * total_bytes_nb * fec_rate_denominator <= 7 * fec_rate_numerator * tx_bits_symbol_start )
            {
                ceil_numerator   = 8 * total_bytes_nb * fec_rate_denominator;
                ceil_denominator = fec_rate_numerator * tx_bits_symbol_start;
            }
            else
            {
                int32_t tx_codedbits_header = tx_bits_symbol_start * 8;
                ceil_numerator = 8 * fec_rate_numerator * tx_bits_symbol + 8 * total_bytes_nb * fec_rate_denominator -
                                 fec_rate_numerator * tx_codedbits_header;
                ceil_denominator = fec_rate_numerator * tx_bits_symbol;
            }
        }
        else
        {
            tx_infobits_header = ( sf * 4 + fine_synch * 8 - 28 ) & ~0x07;
            if( tx_infobits_header < 8 * total_bytes_nb )
            {
                if( tx_infobits_header > 8 * pld_len_in_bytes )
                {
                    tx_infobits_header = 8 * pld_len_in_bytes;
                }
            }
            tx_infobits_payload = 8 * total_bytes_nb - tx_infobits_header;
            if( tx_infobits_payload < 0 )
            {
                tx_infobits_payload = 0;
            }

            ceil_numerator   = tx_infobits_payload * fec_rate_denominator + 8 * fec_rate_numerator * tx_bits_symbol;
            ceil_denominator = fec_rate_numerator * tx_bits_symbol;
        }
    }
    else
    {
        tx_infobits_header = sf * 4 + fine_synch * 8 - 8;

        if( !pld_is_fix )
        {
            tx_infobits_header -= 20;
        }

        tx_infobits_payload = 8 * total_bytes_nb - tx_infobits_header;

        if( tx_infobits_payload < 0 )
            tx_infobits_payload = 0;

        ceil_numerator   = tx_infobits_payload;
        ceil_denominator = 4 * tx_bits_symbol;
    }

    symbols_nb_data = ( ( ceil_numerator + ceil_denominator - 1 ) / ceil_denominator );
    if( !long_interleaving )
    {
        symbols_nb_data = symbols_nb_data * ( mod_p->cr + 4 ) + 8;
    }
    intermed = pkt_p->pbl_len_in_symb + 4 + 2 * fine_synch + symbols_nb_data;

    return ( uint32_t )( ( 4 * intermed + 1 ) * ( 1 << ( sf - 2 ) ) ) - 1;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline uint32_t _get_gfsk_time_on_air_numerator(const stm32wlxx_pkt_params_gfsk_t * const pkt_p)
{
    const uint32_t numerator = pkt_p->pbl_len_in_bits + (pkt_p->hdr_type == STM32WLxx_GFSK_PKT_VAR_LEN ? 8u : 0u) +
                               pkt_p->sync_word_len_in_bits +
                               ( ( pkt_p->pld_len_in_bytes + (pkt_p->addr_cmp == STM32WLxx_GFSK_ADDR_CMP_FILT_OFF ? 0u : 1u) +
                                   _get_gfsk_crc_len_in_bytes(pkt_p->crc_type) )
                                 << 3 );

    return numerator;
}

/* Global function definitions -----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED uint32_t stm32wlxx_phy_get_lora_time_on_air_in_ms(const stm32wlxx_pkt_params_lora_t * const pkt_p, const stm32wlxx_mod_params_lora_t * const mod_p)
{
    const uint32_t numerator   = 1000u * _get_lora_time_on_air_numerator(pkt_p, mod_p);
    const uint32_t denominator = _get_lora_bw_in_hz(mod_p->bw);

    /* Perform integral ceil() */
    const uint32_t time_on_air = (numerator + denominator - 1u) / denominator;

    return time_on_air;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED uint32_t stm32wlxx_phy_get_gfsk_time_on_air_in_ms(const stm32wlxx_pkt_params_gfsk_t * const pkt_p, const stm32wlxx_radio_fsk_phy_mod_params_t * const mod_p)
{
    const uint32_t numerator   = 1000u * _get_gfsk_time_on_air_numerator(pkt_p);
    const uint32_t denominator = mod_p->bit_rate;

    /* Perform integral ceil() */
    const uint32_t time_on_air = (numerator + (denominator - 1u)) / denominator;

    return time_on_air;
}
