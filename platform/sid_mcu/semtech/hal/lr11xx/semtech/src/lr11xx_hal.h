/*!
 * @file      lr11xx_hal.h
 *
 * @brief     Hardware Abstraction Layer (HAL) interface for LR11XX
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
  * @file    lr11xx_hal.h
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

#ifndef LR11XX_HAL_H
#define LR11XX_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include <stdbool.h>

#include "halo_lr11xx_radio.h"

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
#define LR11XX_NOP ( 0x00 )

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*!
 * @brief LR11XX HAL status
 */
typedef enum lr11xx_hal_status_e
{
    LR11XX_HAL_STATUS_OK    = 0,
    LR11XX_HAL_STATUS_ERROR = 3,
} lr11xx_hal_status_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * @brief Radio data transfer - write
 *
 * @param [in] context          Radio implementation parameters
 * @param [in] command          Pointer to the buffer to be transmitted
 * @param [in] command_length   Buffer size to be transmitted
 * @param [in] data             Pointer to the buffer to be transmitted
 * @param [in] data_length      Buffer size to be transmitted
 *
 * @returns Operation status
 */
lr11xx_hal_status_t lr11xx_hal_write( const void* context, const uint8_t* command, const uint16_t command_length,
                                      const uint8_t* data, const uint16_t data_length );

/*!
 * @brief Radio data transfer - read
 *
 * @remark This is a two-step radio read operation. It consists of writing the command, releasing then re-asserting the
 * NSS line, then reading a discarded dummy byte followed by data_length bytes of response data from the transceiver.
 * While reading the dummy bytes and the response data, the implementation of this function must ensure that only zero
 * bytes (NOP) are written to the SPI bus.
 *
 * @param [in] context          Radio implementation parameters
 * @param [in] command          Pointer to the buffer to be transmitted
 * @param [in] command_length   Buffer size to be transmitted
 * @param [out] data            Pointer to the buffer to be received
 * @param [in] data_length      Buffer size to be received
 *
 * @returns Operation status
 *
 * @remark Some hardware SPI implementations write arbitary values on the MOSI line while reading. If this is done on
 * the LR11XX, non-zero values may be interpreted as commands. This driver does not exploit this functionality, and
 * expects that zeros be sent on the MOSI line when this command is reading the command response data.
 */
lr11xx_hal_status_t lr11xx_hal_read( const void* context, const uint8_t* command, const uint16_t command_length,
                                     uint8_t* data, const uint16_t data_length );

/*!
 * @brief  Direct read from the SPI bus
 *
 * @remark Unlike @ref lr11xx_hal_read, this is a simple direct SPI bus SS/read/nSS operation. While reading the
 * response data, the implementation of this function must ensure that only zero bytes (NOP) are written to the SPI bus.
 *
 * @remark Formerly, that function depended on a lr11xx_hal_write_read API function, which required bidirectional SPI
 * communication. Given that all other radio functionality can be implemented with unidirectional SPI, it has been
 * decided to make this HAL API change to simplify implementation requirements.
 *
 * @remark Only required by the @ref lr11xx_system_get_status and @ref lr11xx_bootloader_get_status commands
 *
 * @param [in]  context      Radio implementation parameters
 * @param [out] data         Pointer to the buffer to be received
 * @param [in]  data_length  Buffer size to be received
 *
 * @returns Operation status
 */
lr11xx_hal_status_t lr11xx_hal_direct_read( const void* context, uint8_t* data, const uint16_t data_length );

/*!
 * @brief Reset the radio
 *
 * @param [in] context Radio implementation parameters
 *
 * @returns Operation status
 */
lr11xx_hal_status_t lr11xx_hal_reset( const void* context );

/*!
 * @brief Wake the radio up.
 *
 * @param [in] context Radio implementation parameters
 *
 * @returns Operation status
 */
lr11xx_hal_status_t lr11xx_hal_wakeup( const void* context );

/*!
 * @brief Abort a blocking command
 *
 * @param [in] context Radio implementation parameters
 *
 * @returns Operation status
 */
lr11xx_hal_status_t lr11xx_hal_abort_blocking_cmd( const void* context );

/*!
 * @brief Return the computed CRC
 *
 * @param [in] initial_value initial value of the CRC
 * @param [in] buffer Buffer containing data used to compute the CRC
 * @param [in] length Length of buffer
 *
 * @returns CRC value
 */
inline static uint8_t lr11xx_hal_compute_crc( const uint8_t initial_value, const uint8_t* buffer, uint16_t length )
{
    uint8_t crc = initial_value;

    for( uint16_t i = 0; i < length; i++ )
    {
        uint8_t extract = buffer[i];
        uint8_t sum;

        for( uint8_t j = 8; j > 0; j-- )
        {
            sum = ( crc ^ extract ) & 0x01;
            crc >>= 1;

            if( sum != 0 )
            {
                crc ^= 0x65;
            }

            extract >>= 1;
        }
    }

    return crc;
}

/*vvvvvvvvvvvvvvvvvvvv Start of STMicroelectronics patch vvvvvvvvvvvvvvvvvvvvv*/
/**
 * @brief GPIO initialization to drive the LR11xx transceiver
 *
 * @param [in] drv_ctx Radio implementation parameters
 *
 * @returns Operation status
 */
lr11xx_hal_status_t lr11xx_hal_gpio_init(const halo_drv_semtech_ctx_t * const drv_ctx);

/**
 * @brief GPIO de-initialization to drive the LR11xx transceiver
 *
 * @param [in] drv_ctx Radio implementation parameters
 *
 * @returns Operation status
 */
lr11xx_hal_status_t lr11xx_hal_gpio_deinit(const halo_drv_semtech_ctx_t * const drv_ctx);

/**
 * @brief Disables MCU reaction on radio IRQ pin
 *
 * @note This method does not affect how LR11xx generates IRQs, it just disables the MCU reaction on IRQ line transitions
 *
 * @param [in] drv_ctx Radio implementation parameters
 *
 * @returns Operation status
 */
lr11xx_hal_status_t lr11xx_hal_disarm_irq(const halo_drv_semtech_ctx_t * const drv_ctx);

/**
 * @brief Enables MCU reaction on radio IRQ pin
 *
 * @note This method does not affect how LR11xx generates IRQs, it just enables the MCU reaction on IRQ line transitions
 *
 * @param [in] drv_ctx Radio implementation parameters
 *
 * @returns Operation status
 */
lr11xx_hal_status_t lr11xx_hal_arm_irq(halo_drv_semtech_ctx_t * const drv_ctx);

/**
 * @brief Waits till LR11xx becomes ready after last SPI command
 *
 * @param [in] drv_ctx Radio implementation parameters
 *
 * @returns Operation status
 */
lr11xx_hal_status_t lr11xx_hal_wait_readiness(const halo_drv_semtech_ctx_t * const drv_ctx);

/**
 * @brief Compensates the raw RSSI value provided by LR11xx for antenna gain, external LNA gain (if used), and RF switch insertion loss
 *
 * @param [in] drv_ctx  Radio implementation parameters
 * @param [in] raw_rssi Raw RSSI reading reported by LR11xx
 *
 * @returns Compensated RSSI reading in dBm
 */
int16_t lr11xx_radio_hal_get_adjusted_rssi(const halo_drv_semtech_ctx_t * const drv_ctx, const int8_t raw_rssi);

#if LR11XX_RADIO_CFG_USE_STATUS_LED
/**
 * @brief Turn Tx LED on (if LED is present)
 *
 * @param [in] drv_ctx Radio implementation parameters
 *
 * @returns None
 */
void lr11xx_radio_hal_tx_led_on(const halo_drv_semtech_ctx_t * const drv_ctx);

/**
 * @brief Turn Tx LED off (if LED is present)
 *
 * @param [in] drv_ctx Radio implementation parameters
 *
 * @returns None
 */
void lr11xx_radio_hal_tx_led_off(const halo_drv_semtech_ctx_t * const drv_ctx);

/**
 * @brief Turn Rx LED on (if LED is present)
 *
 * @param [in] drv_ctx Radio implementation parameters
 *
 * @returns None
 */
void lr11xx_radio_hal_rx_led_on(const halo_drv_semtech_ctx_t * const drv_ctx);

/**
 * @brief Turn Rx LED off (if LED is present)
 *
 * @param [in] drv_ctx Radio implementation parameters
 *
 * @returns None
 */
void lr11xx_radio_hal_rx_led_off(const halo_drv_semtech_ctx_t * const drv_ctx);

#  if LR11XX_RADIO_CFG_GEOLOCATION_SUPPORT
/**
 * @brief Turn GNSS Scan LED on (if LED is present)
 *
 * @param [in] drv_ctx Radio implementation parameters
 *
 * @returns None
 */
void lr11xx_radio_hal_gnss_scan_led_on(const halo_drv_semtech_ctx_t * const drv_ctx);

/**
 * @brief Turn GNSS Scan LED off (if LED is present)
 *
 * @param [in] drv_ctx Radio implementation parameters
 *
 * @returns None
 */
void lr11xx_radio_hal_gnss_scan_led_off(const halo_drv_semtech_ctx_t * const drv_ctx);

/**
 * @brief Turn WiFi Scan LED on (if LED is present)
 *
 * @param [in] drv_ctx Radio implementation parameters
 *
 * @returns None
 */
void lr11xx_radio_hal_wifi_scan_led_on(const halo_drv_semtech_ctx_t * const drv_ctx);

/**
 * @brief Turn WiFi Scan LED off (if LED is present)
 *
 * @param [in] drv_ctx Radio implementation parameters
 *
 * @returns None
 */
void lr11xx_radio_hal_wifi_scan_led_off(const halo_drv_semtech_ctx_t * const drv_ctx);
#  endif /* LR11XX_RADIO_CFG_GEOLOCATION_SUPPORT */
#endif /* LR11XX_RADIO_CFG_USE_STATUS_LED */
/*^^^^^^^^^^^^^^^^^^^^^ End of STMicroelectronics patch ^^^^^^^^^^^^^^^^^^^^^^*/

#ifdef __cplusplus
}
#endif

#endif  // LR11XX_HAL_H
