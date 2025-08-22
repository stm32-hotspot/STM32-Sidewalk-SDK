/**
 * @file      sx126x_hal.h
 *
 * @brief     Hardware Abstraction Layer for SX126x
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2021. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/**
  ******************************************************************************
  * @file    sx126x_hal.h
  * @brief   Semtech SX126x radio driver for Sidewalk running on STM32 platform
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

#ifndef SX126X_HAL_H
#define SX126X_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include <stdbool.h>

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/**
 * @brief Write this to SPI bus while reading data, or as a dummy/placeholder
 */
#define SX126X_NOP ( 0x00 )

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

typedef enum sx126x_hal_status_e
{
    SX126X_HAL_STATUS_OK    = 0,
    SX126X_HAL_STATUS_ERROR = 3,
} sx126x_hal_status_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/**
 * Radio data transfer - write
 *
 * @remark Shall be implemented by the user
 *
 * @param [in] context          Radio implementation parameters
 * @param [in] command          Pointer to the buffer to be transmitted
 * @param [in] command_length   Buffer size to be transmitted
 * @param [in] data             Pointer to the buffer to be transmitted
 * @param [in] data_length      Buffer size to be transmitted
 *
 * @returns Operation status
 */
sx126x_hal_status_t sx126x_hal_write( const void* context, const uint8_t* command, const uint16_t command_length,
                                      const uint8_t* data, const uint16_t data_length );

/**
 * Radio data transfer - read
 *
 * @remark Shall be implemented by the user
 *
 * @param [in] context          Radio implementation parameters
 * @param [in] command          Pointer to the buffer to be transmitted
 * @param [in] command_length   Buffer size to be transmitted
 * @param [in] data             Pointer to the buffer to be received
 * @param [in] data_length      Buffer size to be received
 *
 * @returns Operation status
 */
sx126x_hal_status_t sx126x_hal_read( const void* context, const uint8_t* command, const uint16_t command_length,
                                     uint8_t* data, const uint16_t data_length );

/**
 * Reset the radio
 *
 * @remark Shall be implemented by the user
 *
 * @param [in] context Radio implementation parameters
 *
 * @returns Operation status
 */
sx126x_hal_status_t sx126x_hal_reset( const void* context );

/**
 * Wake the radio up.
 *
 * @remark Shall be implemented by the user
 *
 * @param [in] context Radio implementation parameters
 *
 * @returns Operation status
 */
sx126x_hal_status_t sx126x_hal_wakeup( const void* context );

/*vvvvvvvvvvvvvvvvvvvv Start of STMicroelectronics patch vvvvvvvvvvvvvvvvvvvvv*/
/**
 * Wait for busy pin high level
 *
 * @remark Shall be implemented by the user
 *
 * @param [in] context Radio implementation parameters
 *
 * @retval status     Operation status
 */
sx126x_hal_status_t sx126x_hal_wait_busy_indicated(const void * const context);

/**
 * Init GPIOs.
 *
 * @remark Shall be implemented by the user
 *
 * @param [in] context Radio implementation parameters
 *
 * @retval status    Operation status
 */
sx126x_hal_status_t sx126x_hal_init_gpio(const void * const context);

/**
 * Deinit GPIOs.
 *
 * @remark Shall be implemented by the user
 *
 * @param [in] context Radio implementation parameters
 *
 * @retval status    Operation status
 */
sx126x_hal_status_t sx126x_hal_deinit_gpio(const void * const context);

/**
 * Disable irq on int1 pin
 *
 * @remark Shall be implemented by the user
 *
 * @param [in] context Radio implementation parameters
 *
 * @retval status    Operation status
 */
sx126x_hal_status_t sx126x_hal_disarm_irq(const void * const context);

/**
 * Enable irq on int1 pin
 *
 * @remark Shall be implemented by the user
 *
 * @param [in] context Radio implementation parameters
 *
 * @retval status    Operation status
 */
sx126x_hal_status_t sx126x_hal_arm_irq(void * const context);

/**
 * @brief Waits till SX126x becomes ready after last SPI command
 *
 * @remark Shall be implemented by the user
 *
 * @param [in] ctx Radio implementation parameters
 *
 * @returns Operation status
 */
sx126x_hal_status_t sx126x_hal_wait_readiness(const void * const context);

/**
 * @brief Compensates the raw RSSI value provided by SX126x for antenna gain, external LNA gain (if used), and RF switch insertion loss
 *
 * @remark Shall be implemented by the user
 *
 * @param [in] context  Radio implementation parameters
 * @param [in] raw_rssi Raw RSSI reading reported by SX126x
 *
 * @returns Compensated RSSI reading in dBm
 */
int8_t sx126x_hal_get_adjusted_rssi(const void * const context, const int8_t raw_rssi);

#if SX126X_RADIO_CFG_USE_STATUS_LED
/**
 * @brief Turn Tx LED on (if LED is present)
 *
 * @remark Shall be implemented by the user
 *
 * @param [in] context Radio implementation parameters
 *
 * @returns None
 */
void sx126x_radio_hal_tx_led_on(const void * const ctx);

/**
 * @brief Turn Tx LED off (if LED is present)
 *
 * @remark Shall be implemented by the user
 *
 * @param [in] context Radio implementation parameters
 *
 * @returns None
 */
void sx126x_radio_hal_tx_led_off(const void * const ctx);

/**
 * @brief Turn Rx LED on (if LED is present)
 *
 * @remark Shall be implemented by the user
 *
 * @param [in] context Radio implementation parameters
 *
 * @returns None
 */
void sx126x_radio_hal_rx_led_on(const void * const ctx);

/**
 * @brief Turn Rx LED off (if LED is present)
 *
 * @remark Shall be implemented by the user
 *
 * @param [in] context Radio implementation parameters
 *
 * @returns None
 */
void sx126x_radio_hal_rx_led_off(const void * const ctx);
#endif /* SX126X_RADIO_CFG_USE_STATUS_LED */
/*^^^^^^^^^^^^^^^^^^^^^ End of STMicroelectronics patch ^^^^^^^^^^^^^^^^^^^^^^*/

#ifdef __cplusplus
}
#endif

#endif  // SX126X_HAL_H

/* --- EOF ------------------------------------------------------------------ */
