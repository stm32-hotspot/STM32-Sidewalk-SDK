/**
 * @file      stm32wlxx_phy_timings.h
 *
 * @brief     SX126x timing helper functions definition
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
  * @file    stm32wlxx_phy_timings.h
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

#ifndef __STM32WLXX_PHY_TIMINGS_H_
#define __STM32WLXX_PHY_TIMINGS_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include <stm32wlxx_phy.h>
#include <comm_def.h>

/* Exported functions --------------------------------------------------------*/

/**
 * @brief Get the time between the last bit sent (on Tx side) and the Rx done event (on Rx side)
 *
 * @param [in] mod_params Pointer to a structure holding the LoRa modulation parameters used for the computation
 * @param [in] pkt_params Pointer to a structure holding the LoRa packet parameters used for the computation
 *
 * @returns Delay in microsecond
 */
uint32_t stm32wlxx_phy_timings_get_delay_between_last_bit_sent_and_rx_done_in_us(const stm32wlxx_mod_params_lora_t * const mod_params, const stm32wlxx_pkt_params_lora_t * const pkt_params);

#ifdef __cplusplus
}
#endif

#endif  /* __STM32WLXX_PHY_TIMINGS_H_ */
