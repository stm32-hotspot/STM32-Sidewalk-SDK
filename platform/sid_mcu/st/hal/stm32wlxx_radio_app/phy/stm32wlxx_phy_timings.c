/**
 * @file      stm32wlxx_phy_timings.c
 *
 * @brief     SX126x timing helper functions implementation
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
  * @file    stm32wlxx_phy_timings.c
  * @brief   STM32WLxx Sub-GHz PHY timing helper functions implementation
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

#include "stm32wlxx_phy_timings.h"
#include <sid_stm32_common_utils.h>

/* Private defines -----------------------------------------------------------*/

#define RX_DONE_IRQ_PROCESSING_TIME_IN_US (54u)
#define TX_DONE_IRQ_PROCESSING_TIME_IN_US (53u)

#define LORA_OVERSAMPLING_RATE            (32u)

/* Private function prototypes -----------------------------------------------*/

/**
 * @brief Get the LoRa clock frequency
 *
 * @param [in] bw LoRa bandwidth
 *
 * @returns LoRa clock frequency in Hz
 */
static inline uint32_t _get_lora_clock_freq_in_hz(const stm32wlxx_lora_bw_t bw);

/**
 * @brief Get the LoRa reception input delay
 *
 * @param [in] bw LoRa bandwidth
 *
 * @returns LoRa reception input delay in microsecond
 */
static inline uint32_t _get_lora_rx_input_delay_in_us(const stm32wlxx_lora_bw_t bw);

/**
 * @brief Get the LoRa reception base delay
 *
 * @param [in] sf LoRa spreading factor
 *
 * @returns LoRa reception base delay in tick
 */
static inline uint32_t _get_lora_rx_base_delay_in_tick(const stm32wlxx_lora_sf_t sf);

/**
 * @brief Get the LoRa reception symbol delay
 *
 * @param [in] sf LoRa spreading factor
 *
 * @returns LoRa reception symbol delay in tick
 */
static inline uint32_t _get_lora_rx_symb_delay_in_tick(const stm32wlxx_lora_sf_t sf);

/**
 * @brief Get the LoRa reception data delay
 *
 * @param [in] mod_params Pointer to a structure holding the LoRa modulation parameters used for the computation
 * @param [in] pkt_params Pointer to a structure holding the LoRa packet parameters used for the computation
 *
 * @returns LoRa reception data delay in tick
 */
static inline uint32_t _get_lora_rx_data_delay_in_tick(const stm32wlxx_mod_params_lora_t * const mod_params, const stm32wlxx_pkt_params_lora_t * const pkt_params );

/**
 * @brief Get the LoRa physical layer delay
 *
 * @param [in] mod_params Pointer to a structure holding the LoRa modulation parameters used for the computation
 * @param [in] pkt_params Pointer to a structure holding the LoRa packet parameters used for the computation
 *
 * @returns LoRa physical layer delay in microsecond
 */
static inline uint32_t _get_lora_rx_phy_delay_in_us(const stm32wlxx_mod_params_lora_t * const mod_params, const stm32wlxx_pkt_params_lora_t * const pkt_params);

/* Private function definitions ----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline uint32_t _get_lora_clock_freq_in_hz(const stm32wlxx_lora_bw_t bw)
{
    uint32_t freq;

    switch (bw)
    {
        case STM32WLxx_LORA_BW_500:
            freq = LORA_OVERSAMPLING_RATE * 500000u;
            break;

        case STM32WLxx_LORA_BW_250:
            freq = LORA_OVERSAMPLING_RATE * 250000u;
            break;

        case STM32WLxx_LORA_BW_125:
            freq = LORA_OVERSAMPLING_RATE * 125000u;
            break;

        case STM32WLxx_LORA_BW_062:
            freq = LORA_OVERSAMPLING_RATE * 62500u;
            break;

        case STM32WLxx_LORA_BW_041:
            freq = LORA_OVERSAMPLING_RATE * 41670u;
            break;

        case STM32WLxx_LORA_BW_031:
            freq = LORA_OVERSAMPLING_RATE * 31250u;
            break;

        case STM32WLxx_LORA_BW_020:
            freq = LORA_OVERSAMPLING_RATE * 20830u;
            break;

        case STM32WLxx_LORA_BW_015:
            freq = LORA_OVERSAMPLING_RATE * 15630u;
            break;

        case STM32WLxx_LORA_BW_010:
            freq = LORA_OVERSAMPLING_RATE * 10420u;
            break;

        case STM32WLxx_LORA_BW_007:
            freq = LORA_OVERSAMPLING_RATE * 7810u;
            break;

        default:
            freq = 0u;
            break;
    }

    return freq;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline uint32_t _get_lora_rx_input_delay_in_us(const stm32wlxx_lora_bw_t bw)
{
    uint32_t delay;

    switch (bw)
    {
        case STM32WLxx_LORA_BW_500:
            delay = 16u;
            break;

        case STM32WLxx_LORA_BW_250:
            delay = 31u;
            break;

        case STM32WLxx_LORA_BW_125:
            delay = 57u;
            break;

        default:
            delay = 0u;
            break;
    }

    return delay;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline uint32_t _get_lora_rx_base_delay_in_tick(const stm32wlxx_lora_sf_t sf)
{
    return ((uint32_t)sf + 1u) << ((uint32_t)sf + 1u);
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline uint32_t _get_lora_rx_symb_delay_in_tick(const stm32wlxx_lora_sf_t sf)
{
    return ((uint32_t)1u << (uint32_t)sf);
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline uint32_t _get_lora_rx_data_delay_in_tick(const stm32wlxx_mod_params_lora_t * const mod_params, const stm32wlxx_pkt_params_lora_t * const pkt_params)
{
    const uint32_t fine_sync =
        ( ( mod_params->sf == STM32WLxx_LORA_SF5 ) || ( mod_params->sf == STM32WLxx_LORA_SF6 ) ) ? 1u : 0u;
    const uint32_t cr_len  = ( ( (uint32_t)mod_params->cr == 7u ) ? ( (uint32_t)mod_params->cr + 1u ) : (uint32_t)mod_params->cr );
    const uint32_t crc_len = ( ( pkt_params->crc_is_on == true ) ? 1u : 0u );

    if( ( mod_params->cr == STM32WLxx_LORA_CR_4_5 ) || ( mod_params->cr == STM32WLxx_LORA_CR_4_6 ) ||
        ( mod_params->cr == STM32WLxx_LORA_CR_4_7 ) || ( mod_params->cr == STM32WLxx_LORA_CR_4_8 ) )
    {
        return ( 2u * ( (uint32_t)pkt_params->pld_len_in_bytes + 2u * crc_len ) - ( (uint32_t)mod_params->sf + 2u * fine_sync - 7u ) ) %
                   (uint32_t)mod_params->sf * 16u * ( 4u + cr_len ) +
               5u;
    }
    else
    {
        uint32_t pld_len = (uint32_t)pkt_params->pld_len_in_bytes + 2 * crc_len - ( (uint32_t)mod_params->sf + 2u * fine_sync - 7u ) / 2u;

        return ( 2u * pld_len * ( 5u + cr_len ) ) + ( 2u * pld_len * cr_len % (uint32_t)mod_params->sf ) + 4u;
    }
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline uint32_t _get_lora_rx_phy_delay_in_us(const stm32wlxx_mod_params_lora_t * const mod_params, const stm32wlxx_pkt_params_lora_t * const pkt_params)
{
    uint32_t rx_delay_in_clock_cycle = _get_lora_rx_base_delay_in_tick(mod_params->sf) +
                                       _get_lora_rx_symb_delay_in_tick(mod_params->sf) +
                                       _get_lora_rx_data_delay_in_tick(mod_params, pkt_params);
    uint32_t clock_freq_in_mhz = _get_lora_clock_freq_in_hz(mod_params->bw) / 1000000u;

    if( clock_freq_in_mhz != 0u )
    {
        /* Perform integral ceil() */
        return ( rx_delay_in_clock_cycle + clock_freq_in_mhz - 1u ) / clock_freq_in_mhz;
    }
    else
    {
        return 0u;
    }
}

/* Global function definitions -----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED uint32_t stm32wlxx_phy_timings_get_delay_between_last_bit_sent_and_rx_done_in_us(const stm32wlxx_mod_params_lora_t * const mod_params, const stm32wlxx_pkt_params_lora_t * const pkt_params)
{
    return _get_lora_rx_input_delay_in_us(mod_params->bw) +
           _get_lora_rx_phy_delay_in_us(mod_params, pkt_params) + RX_DONE_IRQ_PROCESSING_TIME_IN_US;
}
