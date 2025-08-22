/*
 * Copyright (c) 2019-2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * This file supports radio hal interface  specific to FSK
 * Some parts of the the code are based on Semtech reference driver
 */
/**
  ******************************************************************************
  * @file  lr11xx_radio_fsk.c
  * @brief FSK-specific portion of the LR11xx driver for STM32 platform
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

#include <assert.h>
#include <string.h>

#include "halo_lr11xx_radio.h"
#include "lr11xx_radio.h"
#include "lr11xx_regmem.h"
#include "lr11xx_hal.h"
#include "lr11xx_halo.h"
#include "lr11xx_radio_config.h"

/* Sidewalk interfaces */
#include <sid_pal_assert_ifc.h>
#include <sid_pal_log_ifc.h>
#include <sid_pal_radio_ifc.h>
#include <sid_pal_uptime_ifc.h>
#include <sid_time_ops.h>

/* Utilities and helpers */
#include <cmsis_compiler.h>
#include <sid_stm32_common_utils.h>

/* Private defines -----------------------------------------------------------*/

#define LR11XX_FSK_RX_PROCESS_SAFETY_GAP_US     (50u)   /*!< Rx window will be opened earlier by this amount of microseconds to mitigate any SW execution delays and jitter */

#define FSK_MICRO_SECS_PER_SYMBOL               (250u)

#define RADIO_FSK_SYNC_WORD_VALID_MARKER        (0xABBACAFEu)
#define RADIO_FSK_PACKET_TYPE_OFFSET            (0u)
#define RADIO_FSK_PACKET_LENGTH_OFFSET          (1u)
#define RADIO_FSK_SYNC_WORD_VALID_MARKER_OFFSET (2u)

#define LR11XX_FSK_PHY_HEADER_LENGTH            (2u)
#define LR11XX_FSK_PHY_HEADER_MIN_RX_LENGTH     (((LR11XX_FSK_PHY_HEADER_LENGTH + sizeof(uint32_t) - 1u) & ~(sizeof(uint32_t) - 1u)) + 1u) /* Round up to 4-byte boundary and add 1 byte. This is required because LR11xx internal Rx buffer is populated by 4-byte words until Rx is finished */

#define LR11XX_FSK_WHITENING_SEED               (0x01FFu)

#define LR11XX_FSK_SYNC_WORD_LENGTH_IN_RX       (3u)

#define LR11XX_FSK_MAX_PAYLOAD_LENGTH           (SID_PAL_RADIO_RX_PAYLOAD_MAX_SIZE)
#define LR11XX_FSK_FCS_TYPE_0_CRC_LENGTH        (4u)
#define LR11XX_FSK_FCS_TYPE_1_CRC_LENGTH        (2u)
#define MAX_PAYLOAD_LENGTH_WITH_FCS_TYPE_0      (LR11XX_FSK_MAX_PAYLOAD_LENGTH - LR11XX_FSK_FCS_TYPE_0_CRC_LENGTH)
#define MAX_PAYLOAD_LENGTH_WITH_FCS_TYPE_1      (LR11XX_FSK_MAX_PAYLOAD_LENGTH - LR11XX_FSK_FCS_TYPE_1_CRC_LENGTH)

/* Private function prototypes -----------------------------------------------*/

static inline void radio_mp_to_lr11xx_mp(lr11xx_radio_mod_params_gfsk_t * const fsk_mp, const sid_pal_radio_fsk_modulation_params_t * const mod_params);
static inline void radio_pp_to_lr11xx_pp(lr11xx_radio_pkt_params_gfsk_t * const fsk_pp, const sid_pal_radio_fsk_packet_params_t * const packet_params);

/* Private function definitions ----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline void radio_mp_to_lr11xx_mp(lr11xx_radio_mod_params_gfsk_t * const fsk_mp, const sid_pal_radio_fsk_modulation_params_t * const mod_params)
{
    fsk_mp->br_in_bps    = mod_params->bit_rate;
    fsk_mp->fdev_in_hz   = mod_params->freq_dev;

    _Static_assert(LR11XX_RADIO_GFSK_BW_4800   == SID_PAL_RADIO_FSK_BW_4800   , "Invalid conversion");
                /* LR11XX_RADIO_GFSK_BW_5800   != SID_PAL_RADIO_FSK_BW_5800 */
    _Static_assert(LR11XX_RADIO_GFSK_BW_7300   == SID_PAL_RADIO_FSK_BW_7300   , "Invalid conversion");
    _Static_assert(LR11XX_RADIO_GFSK_BW_9700   == SID_PAL_RADIO_FSK_BW_9700   , "Invalid conversion");
    _Static_assert(LR11XX_RADIO_GFSK_BW_11700  == SID_PAL_RADIO_FSK_BW_11700  , "Invalid conversion");
    _Static_assert(LR11XX_RADIO_GFSK_BW_14600  == SID_PAL_RADIO_FSK_BW_14600  , "Invalid conversion");
    _Static_assert(LR11XX_RADIO_GFSK_BW_19500  == SID_PAL_RADIO_FSK_BW_19500  , "Invalid conversion");
    _Static_assert(LR11XX_RADIO_GFSK_BW_23400  == SID_PAL_RADIO_FSK_BW_23400  , "Invalid conversion");
    _Static_assert(LR11XX_RADIO_GFSK_BW_29300  == SID_PAL_RADIO_FSK_BW_29300  , "Invalid conversion");
    _Static_assert(LR11XX_RADIO_GFSK_BW_39000  == SID_PAL_RADIO_FSK_BW_39000  , "Invalid conversion");
    _Static_assert(LR11XX_RADIO_GFSK_BW_46900  == SID_PAL_RADIO_FSK_BW_46900  , "Invalid conversion");
    _Static_assert(LR11XX_RADIO_GFSK_BW_58600  == SID_PAL_RADIO_FSK_BW_58600  , "Invalid conversion");
    _Static_assert(LR11XX_RADIO_GFSK_BW_78200  == SID_PAL_RADIO_FSK_BW_78200  , "Invalid conversion");
    _Static_assert(LR11XX_RADIO_GFSK_BW_93800  == SID_PAL_RADIO_FSK_BW_93800  , "Invalid conversion");
    _Static_assert(LR11XX_RADIO_GFSK_BW_117300 == SID_PAL_RADIO_FSK_BW_117300 , "Invalid conversion");
    _Static_assert(LR11XX_RADIO_GFSK_BW_156200 == SID_PAL_RADIO_FSK_BW_156200 , "Invalid conversion");
    _Static_assert(LR11XX_RADIO_GFSK_BW_187200 == SID_PAL_RADIO_FSK_BW_187200 , "Invalid conversion");
    _Static_assert(LR11XX_RADIO_GFSK_BW_234300 == SID_PAL_RADIO_FSK_BW_234300 , "Invalid conversion");
    _Static_assert(LR11XX_RADIO_GFSK_BW_312000 == SID_PAL_RADIO_FSK_BW_312000 , "Invalid conversion");
    _Static_assert(LR11XX_RADIO_GFSK_BW_373600 == SID_PAL_RADIO_FSK_BW_373600 , "Invalid conversion");
    _Static_assert(LR11XX_RADIO_GFSK_BW_467000 == SID_PAL_RADIO_FSK_BW_467000 , "Invalid conversion");
    switch (mod_params->bandwidth)
    {
        case SID_PAL_RADIO_FSK_BW_5800:
            fsk_mp->bw_dsb_param = LR11XX_RADIO_GFSK_BW_5800;
            break;

        case SID_PAL_RADIO_FSK_BW_125KHZ:
            fsk_mp->bw_dsb_param = LR11XX_RADIO_GFSK_BW_117300;
            break;

        case SID_PAL_RADIO_FSK_BW_250KHZ:
            fsk_mp->bw_dsb_param = LR11XX_RADIO_GFSK_BW_234300;
            break;

        case SID_PAL_RADIO_FSK_BW_500KHZ:
            fsk_mp->bw_dsb_param = LR11XX_RADIO_GFSK_BW_467000;
            break;

        default:
            fsk_mp->bw_dsb_param = (lr11xx_radio_gfsk_bw_t)mod_params->bandwidth;
            break;
    }

    _Static_assert(SID_PAL_RADIO_FSK_MOD_SHAPING_OFF     == LR11XX_RADIO_GFSK_PULSE_SHAPE_OFF  , "Invalid conversion");
    _Static_assert(SID_PAL_RADIO_FSK_MOD_SHAPING_G_BT_03 == LR11XX_RADIO_GFSK_PULSE_SHAPE_BT_03, "Invalid conversion");
    _Static_assert(SID_PAL_RADIO_FSK_MOD_SHAPING_G_BT_05 == LR11XX_RADIO_GFSK_PULSE_SHAPE_BT_05, "Invalid conversion");
    _Static_assert(SID_PAL_RADIO_FSK_MOD_SHAPING_G_BT_07 == LR11XX_RADIO_GFSK_PULSE_SHAPE_BT_07, "Invalid conversion");
    _Static_assert(SID_PAL_RADIO_FSK_MOD_SHAPING_G_BT_1  == LR11XX_RADIO_GFSK_PULSE_SHAPE_BT_1 , "Invalid conversion");
    fsk_mp->pulse_shape  = (lr11xx_radio_gfsk_pulse_shape_t)mod_params->mod_shaping;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline void radio_pp_to_lr11xx_pp(lr11xx_radio_pkt_params_gfsk_t * const fsk_pp, const sid_pal_radio_fsk_packet_params_t * const packet_params)
{
    if (packet_params->preamble_length > 1u)
    {
        fsk_pp->preamble_len_in_bits = (packet_params->preamble_length - 1u) << 3;
    }
    else
    {
        fsk_pp->preamble_len_in_bits = 0u;
    }

    _Static_assert(LR11XX_RADIO_GFSK_PREAMBLE_DETECTOR_OFF        == SID_PAL_RADIO_FSK_PREAMBLE_DETECTOR_OFF, "Invalid conversion");
    _Static_assert(LR11XX_RADIO_GFSK_PREAMBLE_DETECTOR_MIN_8BITS  == SID_PAL_RADIO_FSK_PREAMBLE_DETECTOR_08_BITS, "Invalid conversion");
    _Static_assert(LR11XX_RADIO_GFSK_PREAMBLE_DETECTOR_MIN_16BITS == SID_PAL_RADIO_FSK_PREAMBLE_DETECTOR_16_BITS, "Invalid conversion");
    _Static_assert(LR11XX_RADIO_GFSK_PREAMBLE_DETECTOR_MIN_24BITS == SID_PAL_RADIO_FSK_PREAMBLE_DETECTOR_24_BITS, "Invalid conversion");
    _Static_assert(LR11XX_RADIO_GFSK_PREAMBLE_DETECTOR_MIN_32BITS == SID_PAL_RADIO_FSK_PREAMBLE_DETECTOR_32_BITS, "Invalid conversion");
    fsk_pp->preamble_detector     = (lr11xx_radio_gfsk_preamble_detector_t)packet_params->preamble_min_detect;
    fsk_pp->sync_word_len_in_bits = packet_params->sync_word_length << 3;

    _Static_assert(LR11XX_RADIO_GFSK_ADDRESS_FILTERING_DISABLE      == SID_PAL_RADIO_FSK_ADDRESSCOMP_FILT_OFF, "Invalid conversion");
    _Static_assert(LR11XX_RADIO_GFSK_ADDRESS_FILTERING_NODE_ADDRESS == SID_PAL_RADIO_FSK_ADDRESSCOMP_FILT_NODE, "Invalid conversion");
    _Static_assert(LR11XX_RADIO_GFSK_ADDRESS_FILTERING_NODE_AND_BROADCAST_ADDRESSES == SID_PAL_RADIO_FSK_ADDRESSCOMP_FILT_NODE_BROAD, "Invalid conversion");
    fsk_pp->address_filtering     = (lr11xx_radio_gfsk_address_filtering_t)packet_params->addr_comp;

    _Static_assert(LR11XX_RADIO_GFSK_PKT_FIX_LEN == SID_PAL_RADIO_FSK_RADIO_PACKET_FIXED_LENGTH, "Invalid conversion");
    _Static_assert(LR11XX_RADIO_GFSK_PKT_VAR_LEN == SID_PAL_RADIO_FSK_RADIO_PACKET_VARIABLE_LENGTH, "Invalid conversion");
    fsk_pp->header_type           = (lr11xx_radio_gfsk_pkt_len_modes_t)packet_params->header_type;
    fsk_pp->pld_len_in_bytes      = packet_params->payload_length;

    _Static_assert(LR11XX_RADIO_GFSK_CRC_OFF         == SID_PAL_RADIO_FSK_CRC_OFF,         "Invalid conversion");
    _Static_assert(LR11XX_RADIO_GFSK_CRC_1_BYTE      == SID_PAL_RADIO_FSK_CRC_1_BYTES,     "Invalid conversion");
    _Static_assert(LR11XX_RADIO_GFSK_CRC_2_BYTES     == SID_PAL_RADIO_FSK_CRC_2_BYTES,     "Invalid conversion");
    _Static_assert(LR11XX_RADIO_GFSK_CRC_1_BYTE_INV  == SID_PAL_RADIO_FSK_CRC_1_BYTES_INV, "Invalid conversion");
    _Static_assert(LR11XX_RADIO_GFSK_CRC_2_BYTES_INV == SID_PAL_RADIO_FSK_CRC_2_BYTES_INV, "Invalid conversion");
    SID_PAL_ASSERT(packet_params->crc_type != SID_PAL_RADIO_FSK_CRC_2_BYTES_IBM);  /* Not supported */
    SID_PAL_ASSERT(packet_params->crc_type != SID_PAL_RADIO_FSK_CRC_2_BYTES_CCIT); /* Not supported */
    fsk_pp->crc_type              = (lr11xx_radio_gfsk_crc_type_t)packet_params->crc_type;

    _Static_assert(LR11XX_RADIO_GFSK_DC_FREE_OFF       == SID_PAL_RADIO_FSK_DC_FREE_OFF,      "Invalid conversion");
    _Static_assert(LR11XX_RADIO_GFSK_DC_FREE_WHITENING == SID_PAL_RADIO_FSK_DC_FREEWHITENING, "Invalid conversion");
    fsk_pp->dc_free               = (lr11xx_radio_gfsk_dc_free_t)packet_params->radio_whitening_mode;
}

/* Global function definitions -----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t radio_fsk_process_sync_word_detected(const halo_drv_semtech_ctx_t * const drv_ctx, const uint32_t rx_completed)
{
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_2
    int32_t err =  RADIO_ERROR_NONE;
    lr11xx_status_t sys_err;
    sid_pal_radio_rx_packet_t * radio_rx_packet = drv_ctx->radio_rx_packet;
    uint8_t * buffer = radio_rx_packet->rcv_payload;
    lr11xx_radio_rx_buffer_status_t rx_buffer_status;

    do
    {
        uint32_t rx_buffer_ptr_position_raw;
        uint32_t bytes_received;
        uint32_t expected_fsk_frame_length;

        /* Semtech has no 802.15.4 packet engine for FSK - wait till the 2-byte header is received and parse it here to reconfigure the radio */
        if (FALSE == rx_completed)
        {
            /* Rx is still ongoing, let's modify Rx length on the fly */
            struct sid_timespec start, elapsed;
            uint32_t elapsed_time_ms;
            const uint32_t timeout_ms = sid_pal_radio_fsk_time_on_air(&drv_ctx->settings_cache.fsk_mod_params, &drv_ctx->settings_cache.fsk_pkt_params, LR11XX_FSK_PHY_HEADER_MIN_RX_LENGTH) * 2u;

            (void)sid_pal_uptime_now(&start);

            do
            {
                /* Magic numbers are LR11xx internal register addresses */

                /* Readout current Rx position from the radio */
                sys_err = lr11xx_regmem_read_regmem32(drv_ctx, 0x00F20384u, &rx_buffer_ptr_position_raw, sizeof(rx_buffer_ptr_position_raw) / sizeof(uint32_t));

                /* Convert raw value into the position */
                bytes_received = (rx_buffer_ptr_position_raw >> 16) & 0x0FFFu;

                /* Compute the elapsed time */
                (void)sid_pal_uptime_now(&elapsed);
                sid_time_sub(&elapsed, &start);
                elapsed_time_ms = sid_timespec_to_ms(&elapsed);
            } while ((LR11XX_STATUS_OK == sys_err) && (bytes_received < LR11XX_FSK_PHY_HEADER_MIN_RX_LENGTH) && (elapsed_time_ms <= timeout_ms));

            if (sys_err != LR11XX_STATUS_OK)
            {
                SID_PAL_LOG_ERROR("Failed to read LR11xx reg 0x00F20384, err: %d", (int32_t)sys_err);
                err = RADIO_ERROR_HARDWARE_ERROR;
                break;
            }
            else if (elapsed_time_ms > timeout_ms)
            {
                /* Failed to receive header in time, terminating */
                err = RADIO_ERROR_HARDWARE_ERROR;
                break;
            }
            else
            {
                /* All good, proceed with processing */
            }
        }

        /* 802.15.4 header received - read it out */
#  if HALO_ENABLE_DIAGNOSTICS
        /* Continuous Rx is enabled in diagnostics mode, meaning buffer start pointer may move - we have to read it out */
        sys_err = lr11xx_radio_get_rx_buffer_status(drv_ctx, &rx_buffer_status);
        if (sys_err != LR11XX_STATUS_OK)
        {
            err = RADIO_ERROR_IO_ERROR;
            break;
        }
#  else
        /* In the normal mode only the single Rx operations are allowed, meaning the Rx buffer start pointer is fixed and ther's no need to waste time on reading it out */
        rx_buffer_status.buffer_start_pointer = LR11XX_RADIO_RX_BUFFER_BASE_OFFSET;
#  endif /* HALO_ENABLE_DIAGNOSTICS */

        sys_err = lr11xx_regmem_read_buffer8(drv_ctx, buffer, rx_buffer_status.buffer_start_pointer, LR11XX_FSK_PHY_HEADER_LENGTH);
        if (sys_err != LR11XX_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to read %u bytes from LR11xx buffer.err: %d", LR11XX_FSK_PHY_HEADER_LENGTH, (int32_t)sys_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Extract frame length from the header */
        expected_fsk_frame_length = (((uint32_t)buffer[0] & 0x07u) << 8) | (uint32_t)buffer[1]; /* 802.15.4 allocates 11 bits for the frame length */

        /* Header length shall be accounted for as well */
        expected_fsk_frame_length += LR11XX_FSK_PHY_HEADER_LENGTH;

        /* Check the expected length matches Sidewalk constraints */
        if (expected_fsk_frame_length > LR11XX_FSK_MAX_PAYLOAD_LENGTH)
        {
            SID_PAL_LOG_WARNING("Oversized FSK packet: %u bytes. Limit: %u bytes", expected_fsk_frame_length, LR11XX_FSK_MAX_PAYLOAD_LENGTH);
            err = RADIO_ERROR_NOT_SUPPORTED;
            break;
        }

        /* Update the expected FSK packet length on the fly */
        if (FALSE == rx_completed)
        {
            sys_err = lr11xx_regmem_write_regmem32_mask(drv_ctx, 0x00F20368u, 0xFFF00000u, (expected_fsk_frame_length << 20));
            if (sys_err != LR11XX_STATUS_OK)
            {
                SID_PAL_LOG_ERROR("Failed to write LR11xx reg 0x00F20368, err: %d", (int32_t)sys_err);
                err = RADIO_ERROR_HARDWARE_ERROR;
                break;
            }
        }

        /* Ensure the Rx packet size is updated ASAP */
        __COMPILER_BARRIER();

        /* Now it is safe to proceed with local arrangements, LR11xx is reconfigured */

        /* Store validity marker into the local Rx buffer to indicate header data is valid */
        uint32_t * const validity_marker_ptr = (uint32_t *)(void *)&buffer[RADIO_FSK_SYNC_WORD_VALID_MARKER_OFFSET];
        *validity_marker_ptr                 = RADIO_FSK_SYNC_WORD_VALID_MARKER;

        err = RADIO_ERROR_NONE;
    } while(0);

    return err;
#else
    (void)drv_ctx;

    return RADIO_ERROR_NOT_SUPPORTED;
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t radio_fsk_process_rx_done(halo_drv_semtech_ctx_t * const drv_ctx, radio_fsk_rx_done_status_t * const rx_done_status)
{
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_2
    sid_pal_radio_rx_packet_t       *radio_rx_packet      = drv_ctx->radio_rx_packet;
    sid_pal_radio_fsk_rx_packet_status_t     *fsk_rx_packet_status = &radio_rx_packet->fsk_rx_packet_status;
    sid_pal_radio_fsk_phy_hdr_t             phy_hdr;
    lr11xx_radio_pkt_status_gfsk_t  pkt_status            = {0};
    lr11xx_status_t                 sys_err;
    int32_t                         err                   = RADIO_ERROR_NONE;
    uint8_t                         *buffer               = radio_rx_packet->rcv_payload;
    uint32_t                        length_temp           = 0u;
    uint32_t                        crc                   = 0x00000000u;
    uint32_t                        crc_length            = 0u;
    lr11xx_radio_rx_buffer_status_t rx_buffer_status;
    int16_t                         tmp_rssi;

    do
    {
        /* Check of FSK packet header validity marker is present in the buffer */
        uint32_t * const validity_marker_ptr = (uint32_t *)(void *)&buffer[RADIO_FSK_SYNC_WORD_VALID_MARKER_OFFSET];
        if (RADIO_FSK_SYNC_WORD_VALID_MARKER != *validity_marker_ptr)
        {
            /* Header was never received or buffer is corrupted */
            *rx_done_status = RADIO_FSK_RX_DONE_STATUS_SW_MARK_NOT_PRESENT;
            err = RADIO_ERROR_GENERIC;
            break;
        }

        /* Clear the validity marker */
        *validity_marker_ptr = 0u;

        /* Parse 802.15.4 header */
        phy_hdr.fcs_type                  = buffer[RADIO_FSK_PACKET_TYPE_OFFSET] >> 4;
        phy_hdr.is_data_whitening_enabled = (buffer[RADIO_FSK_PACKET_TYPE_OFFSET] & (1 << 3)) ? true : false;
        length_temp                       = (((uint32_t)buffer[RADIO_FSK_PACKET_TYPE_OFFSET] & 0x07u) << 8) | buffer[RADIO_FSK_PACKET_LENGTH_OFFSET];

        /* Check that expected packet length at least covers the CRC part + 1 byte */
        if ( ((RADIO_FSK_FCS_TYPE_0 == phy_hdr.fcs_type) && (length_temp <= LR11XX_FSK_FCS_TYPE_0_CRC_LENGTH))
          || ((RADIO_FSK_FCS_TYPE_1 == phy_hdr.fcs_type) && (length_temp <= LR11XX_FSK_FCS_TYPE_1_CRC_LENGTH)))
        {
            *rx_done_status =  RADIO_FSK_RX_DONE_STATUS_BAD_CRC;
            err = RADIO_ERROR_IO_ERROR;
            break;
        }

        /* Readout Rx packet status from LR11xx */
        sys_err = lr11xx_radio_get_rx_buffer_status(drv_ctx, &rx_buffer_status);
        if (sys_err != LR11XX_STATUS_OK)
        {
            *rx_done_status = RADIO_FSK_RX_DONE_STATUS_UNKNOWN_ERROR;
            err = RADIO_ERROR_IO_ERROR;
            break;
        }

        /**
         * Validity check: actually received data length should be not less than info from the header. Normally these two values are equal,
         * but we may receive excessive bytes if radio_fsk_process_sync_word_detected() failed to reconfigure Rx length in time
         */
        if (rx_buffer_status.pld_len_in_bytes < (LR11XX_FSK_PHY_HEADER_LENGTH + length_temp))
        {
            *rx_done_status = RADIO_FSK_RX_DONE_STATUS_INVALID_LENGTH;
            err = RADIO_ERROR_IO_ERROR;
            break;
        }

        /* Ensure the received frame won't overflow the buffer */
        if (length_temp > LR11XX_FSK_MAX_PAYLOAD_LENGTH)
        {
            *rx_done_status = RADIO_FSK_RX_DONE_STATUS_INVALID_LENGTH;
            err = RADIO_ERROR_NOMEM;
            break;
        }

        /* Readout received data */
        sys_err = lr11xx_regmem_read_buffer8(drv_ctx, buffer, rx_buffer_status.buffer_start_pointer + LR11XX_FSK_PHY_HEADER_LENGTH, length_temp);
        if (sys_err != LR11XX_STATUS_OK)
        {
            *rx_done_status = RADIO_FSK_RX_DONE_STATUS_TIMEOUT;
            err = RADIO_ERROR_IO_ERROR;
            break;
        }

        /* Do data de-whitening if required */
        if (phy_hdr.is_data_whitening_enabled != false)
        {
            perform_data_whitening(LR11XX_FSK_WHITENING_SEED, buffer, buffer, length_temp);
        }

        /* Compute CRC of the received payload */
        switch (phy_hdr.fcs_type)
        {
            case RADIO_FSK_FCS_TYPE_0:
                crc_length = LR11XX_FSK_FCS_TYPE_0_CRC_LENGTH;
                crc        = compute_crc32(buffer, length_temp - crc_length);
                break;

            case RADIO_FSK_FCS_TYPE_1:
                crc_length = LR11XX_FSK_FCS_TYPE_1_CRC_LENGTH;
                crc        = compute_crc16(buffer, length_temp - crc_length);
                break;

            default:
                radio_rx_packet->payload_len = 0u;
                *rx_done_status = RADIO_FSK_RX_DONE_STATUS_UNKNOWN_ERROR;
                err = RADIO_ERROR_GENERIC;
                break;
        }

        /* Jump out of loop if switch() ended with an error */
        if (err != RADIO_ERROR_NONE)
        {
            break;
        }

        /* Store the amount of the payload we've got without the CRC part */
        radio_rx_packet->payload_len = length_temp - crc_length;

        /* Verify CRC */
        for (uint32_t i = 0u; i < crc_length; i++)
        {
            if (buffer[radio_rx_packet->payload_len + i] != (uint8_t)(crc >> (8u * (crc_length - i - 1u))))
            {
                *rx_done_status =  RADIO_FSK_RX_DONE_STATUS_BAD_CRC;
                err = RADIO_ERROR_GENERIC;
                break;
            }
        }
        /* Check if the for loop terminated with error */
        if (err != RADIO_ERROR_NONE)
        {
            break;
        }

        sys_err = lr11xx_radio_get_gfsk_pkt_status(drv_ctx, &pkt_status);
        if (sys_err != LR11XX_STATUS_OK)
        {
            err = RADIO_ERROR_IO_ERROR;
            *rx_done_status = RADIO_FSK_RX_DONE_STATUS_TIMEOUT;
            break;
        }

        /* Populate packet status data */
        tmp_rssi                        = lr11xx_radio_hal_get_adjusted_rssi(drv_ctx, pkt_status.rssi_sync_in_dbm);
        fsk_rx_packet_status->rssi_sync = tmp_rssi < INT8_MIN ? INT8_MIN : (int8_t)tmp_rssi;
        tmp_rssi                        = lr11xx_radio_hal_get_adjusted_rssi(drv_ctx, pkt_status.rssi_avg_in_dbm);
        fsk_rx_packet_status->rssi_avg  = tmp_rssi < INT8_MIN ? INT8_MIN : (int8_t)tmp_rssi;
        fsk_rx_packet_status->snr       = 0; /* Measurement not available */

        err = RADIO_ERROR_NONE;
    } while(0);

    return err;
#else
    (void)drv_ctx;
    (void)rx_done_status;

    return RADIO_ERROR_NOT_SUPPORTED;
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_pal_radio_data_rate_t sid_pal_radio_fsk_mod_params_to_data_rate(const sid_pal_radio_fsk_modulation_params_t * mp)
{
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_2
    sid_pal_radio_data_rate_t data_rate;

    if (     (RADIO_FSK_BR_50KBPS  == mp->bit_rate) && (RADIO_FSK_FDEV_25KHZ   == mp->freq_dev) && ((uint8_t)SID_PAL_RADIO_FSK_BW_125KHZ == mp->bandwidth))
    {
        data_rate =  SID_PAL_RADIO_DATA_RATE_50KBPS;
    }
    else if ((RADIO_FSK_BR_150KBPS == mp->bit_rate) && (RADIO_FSK_FDEV_37_5KHZ == mp->freq_dev) && ((uint8_t)SID_PAL_RADIO_FSK_BW_250KHZ == mp->bandwidth))
    {
        data_rate = SID_PAL_RADIO_DATA_RATE_150KBPS;
    }
    else if ((RADIO_FSK_BR_250KBPS == mp->bit_rate) && (RADIO_FSK_FDEV_62_5KHZ == mp->freq_dev) && ((uint8_t)SID_PAL_RADIO_FSK_BW_500KHZ == mp->bandwidth))
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

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_fsk_data_rate_to_mod_params(sid_pal_radio_fsk_modulation_params_t * mod_params, sid_pal_radio_data_rate_t data_rate)
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
            mod_params->bandwidth    = SID_PAL_RADIO_FSK_BW_125KHZ;
            mod_params->mod_shaping  = SID_PAL_RADIO_FSK_MOD_SHAPING_G_BT_1;
            break;

        case SID_PAL_RADIO_DATA_RATE_150KBPS:
            mod_params->bit_rate     = RADIO_FSK_BR_150KBPS;
            mod_params->freq_dev     = RADIO_FSK_FDEV_37_5KHZ;
            mod_params->bandwidth    = SID_PAL_RADIO_FSK_BW_250KHZ;
            mod_params->mod_shaping  = SID_PAL_RADIO_FSK_MOD_SHAPING_G_BT_05;
            break;

        case SID_PAL_RADIO_DATA_RATE_250KBPS:
            mod_params->bit_rate     = RADIO_FSK_BR_250KBPS;
            mod_params->freq_dev     = RADIO_FSK_FDEV_62_5KHZ;
            mod_params->bandwidth    = SID_PAL_RADIO_FSK_BW_500KHZ;
            mod_params->mod_shaping  = SID_PAL_RADIO_FSK_MOD_SHAPING_G_BT_05;
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

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_prepare_fsk_for_rx(sid_pal_radio_fsk_pkt_cfg_t *rx_pkt_cfg)
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

        f_pp->preamble_min_detect       = (uint8_t)LR11XX_RADIO_GFSK_PREAMBLE_DETECTOR_MIN_16BITS;
        f_pp->sync_word_length          = LR11XX_FSK_SYNC_WORD_LENGTH_IN_RX;
        f_pp->addr_comp                 = (uint8_t)LR11XX_RADIO_GFSK_ADDRESS_FILTERING_DISABLE;
        f_pp->header_type               = (uint8_t)LR11XX_RADIO_GFSK_PKT_FIX_LEN;
        f_pp->payload_length            = LR11XX_FSK_MAX_PAYLOAD_LENGTH; /* This will be reconfigured dynamically when 802.15.4 header is received, but we need to ensure the packet is received even if header processing is delayed */
        f_pp->crc_type                  = (uint8_t)LR11XX_RADIO_GFSK_CRC_OFF;
        f_pp->radio_whitening_mode      = (uint8_t)LR11XX_RADIO_GFSK_DC_FREE_OFF;

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

        sid_pal_radio_fsk_packet_params_t * f_pp  =  tx_pkt_cfg->packet_params;
        if ((0u == f_pp->payload_length) || (0u == f_pp->preamble_length))
        {
            err = RADIO_ERROR_INVALID_PARAMS;
            break;
        }

        sid_pal_radio_fsk_phy_hdr_t * phr  = tx_pkt_cfg->phy_hdr;
        if ( ((RADIO_FSK_FCS_TYPE_0 == phr->fcs_type) && (f_pp->payload_length > MAX_PAYLOAD_LENGTH_WITH_FCS_TYPE_0))
          || ((RADIO_FSK_FCS_TYPE_1 == phr->fcs_type) && (f_pp->payload_length > MAX_PAYLOAD_LENGTH_WITH_FCS_TYPE_1)) )
        {
            err = RADIO_ERROR_INVALID_PARAMS;
            break;
        }

        uint16_t  psdu_length      = f_pp->payload_length;
        uint8_t   *tx_buffer       = tx_pkt_cfg->payload;
        uint32_t  crc              = 0x00000000u;

        /* Compute CRC and place it in the end of the buffer */
        if (RADIO_FSK_FCS_TYPE_0 == phr->fcs_type)
        {
            crc = compute_crc32(tx_pkt_cfg->payload, f_pp->payload_length);

            tx_buffer[LR11XX_FSK_PHY_HEADER_LENGTH + psdu_length++] = (uint8_t)(crc >> 24);
            tx_buffer[LR11XX_FSK_PHY_HEADER_LENGTH + psdu_length++] = (uint8_t)(crc >> 16);
            tx_buffer[LR11XX_FSK_PHY_HEADER_LENGTH + psdu_length++] = (uint8_t)(crc >> 8);
            tx_buffer[LR11XX_FSK_PHY_HEADER_LENGTH + psdu_length++] = (uint8_t)(crc >> 0);
        }
        else if (RADIO_FSK_FCS_TYPE_1 == phr->fcs_type)
        {
            crc = compute_crc16(tx_pkt_cfg->payload, f_pp->payload_length);

            tx_buffer[LR11XX_FSK_PHY_HEADER_LENGTH + psdu_length++] = (uint8_t)(crc >> 8);
            tx_buffer[LR11XX_FSK_PHY_HEADER_LENGTH + psdu_length++] = (uint8_t)(crc >> 0);
        }
        else
        {
            err = RADIO_ERROR_NOT_SUPPORTED;
            break;
        }

        /* Free up space for the packet header */
        memmove(tx_buffer + LR11XX_FSK_PHY_HEADER_LENGTH, tx_buffer, f_pp->payload_length);
        /* Build the PHR and put it in the Tx buffer */
        tx_buffer[0]  = (uint8_t)((phr->fcs_type << 4) & (0x10u));
        tx_buffer[0] |= (uint8_t)((phr->is_data_whitening_enabled == true ) ? 1u : 0u) << 3;
        tx_buffer[0] |= (uint8_t)((psdu_length >> 8) & 0x07u);
        tx_buffer[1]  = (uint8_t)(psdu_length & 0xFFu);

        if (phr->is_data_whitening_enabled != false)
        {
            perform_data_whitening(LR11XX_FSK_WHITENING_SEED, tx_buffer + LR11XX_FSK_PHY_HEADER_LENGTH,
                                   tx_buffer + LR11XX_FSK_PHY_HEADER_LENGTH, psdu_length);
        }

        /* Build the syncword */
        uint8_t  *sync_word               = tx_pkt_cfg->sync_word;
        uint8_t  sync_word_length_in_byte = 0;
        sync_word[sync_word_length_in_byte++] = 0x55u;  // Added to force the preamble polarity to a real "0x55"
        sync_word[sync_word_length_in_byte++] = (phr->is_fec_enabled != false) ? 0x6Fu : 0x90u;
        sync_word[sync_word_length_in_byte++] = 0x4Eu;

        f_pp->sync_word_length     = sync_word_length_in_byte;
        f_pp->addr_comp            = (uint8_t)LR11XX_RADIO_GFSK_ADDRESS_FILTERING_DISABLE;
        f_pp->header_type          = (uint8_t)LR11XX_RADIO_GFSK_PKT_FIX_LEN;
        f_pp->payload_length       = LR11XX_FSK_PHY_HEADER_LENGTH + psdu_length;
        f_pp->crc_type             = (uint8_t)LR11XX_RADIO_GFSK_CRC_OFF;
        f_pp->radio_whitening_mode = (uint8_t)LR11XX_RADIO_GFSK_DC_FREE_OFF;

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
    int32_t err;
    lr11xx_status_t sys_err;
    halo_drv_semtech_ctx_t * const drv_ctx = lr11xx_get_drv_ctx();

    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(sync_word != NULL);
    SID_PAL_ASSERT(sync_word_length <= LR11XX_RADIO_GFSK_SYNC_WORD_LENGTH);

    do
    {
#  if LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION
        if (RADIO_LR11XX_LBM_BRIDGE_STATE_EXCLUSIVE_MODE == drv_ctx->lbm.bridge_state)
        {
            /* Don't allow Sidewalk operations */
            err = RADIO_ERROR_BUSY;
            break;
        }
#    if LR11XX_RADIO_CFG_LBM_CONCURRENCY_SUPPORT
        else if (RADIO_LR11XX_LBM_BRIDGE_STATE_SIDEWALK_BLANKING_MODE == drv_ctx->lbm.bridge_state)
        {
            /* Report success to Sidewalk without doing anything */
            err = RADIO_ERROR_NONE;
            break;
        }
#    endif /* LR11XX_RADIO_CFG_LBM_CONCURRENCY_SUPPORT */
#  endif /* LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION */

        /* LR11xx requires to send 8 bytes in the command regardless of the actual length of the sync word */
        uint8_t sync_word_buf[LR11XX_RADIO_GFSK_SYNC_WORD_LENGTH];

        SID_STM32_UTIL_fast_memset(sync_word_buf, 0u, sizeof(sync_word_buf));
        SID_STM32_UTIL_fast_memcpy(sync_word_buf, sync_word, sync_word_length);

        sys_err = lr11xx_radio_set_gfsk_sync_word(drv_ctx, sync_word_buf);
        if (sys_err != LR11XX_STATUS_OK)
        {
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        err = RADIO_ERROR_NONE;
    } while (0);

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
    (void)seed;
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_2
    return RADIO_ERROR_NONE;
#else
    return RADIO_ERROR_NOT_SUPPORTED;
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_set_fsk_modulation_params(const sid_pal_radio_fsk_modulation_params_t * mod_params)
{
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_2
    int32_t err;
    lr11xx_status_t sys_err;
    halo_drv_semtech_ctx_t * const drv_ctx = lr11xx_get_drv_ctx();

    SID_PAL_ASSERT(drv_ctx != NULL);

    do
    {
        if (NULL == mod_params)
        {
            err = RADIO_ERROR_INVALID_PARAMS;
            break;
        }

        lr11xx_radio_mod_params_gfsk_t fsk_mp;
        radio_mp_to_lr11xx_mp(&fsk_mp, mod_params);

#  if LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION
        if (RADIO_LR11XX_LBM_BRIDGE_STATE_EXCLUSIVE_MODE == drv_ctx->lbm.bridge_state)
        {
            /* Don't allow Sidewalk operations */
            err = RADIO_ERROR_BUSY;
            break;
        }
#    if LR11XX_RADIO_CFG_LBM_CONCURRENCY_SUPPORT
        else if (RADIO_LR11XX_LBM_BRIDGE_STATE_SIDEWALK_BLANKING_MODE == drv_ctx->lbm.bridge_state)
        {
            /* Report success to Sidewalk without doing anything */
            drv_ctx->settings_cache.fsk_mod_params = *mod_params;
            err = RADIO_ERROR_NONE;
            break;
        }
#    endif /* LR11XX_RADIO_CFG_LBM_CONCURRENCY_SUPPORT */
#  endif /* LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION */

        sys_err = lr11xx_radio_set_gfsk_mod_params(drv_ctx, &fsk_mp);
        if (sys_err != LR11XX_STATUS_OK)
        {
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        drv_ctx->settings_cache.fsk_mod_params = *mod_params;
        err = RADIO_ERROR_NONE;
    } while (0);

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
    int32_t err;
    lr11xx_status_t sys_err;
    halo_drv_semtech_ctx_t * const drv_ctx = lr11xx_get_drv_ctx();

    SID_PAL_ASSERT(drv_ctx != NULL);

    do
    {
        if (NULL == packet_params)
        {
            err = RADIO_ERROR_INVALID_PARAMS;
            break;
        }

        lr11xx_radio_pkt_params_gfsk_t fsk_pp;
        radio_pp_to_lr11xx_pp(&fsk_pp, packet_params);

#  if LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION
        if (RADIO_LR11XX_LBM_BRIDGE_STATE_EXCLUSIVE_MODE == drv_ctx->lbm.bridge_state)
        {
            /* Don't allow Sidewalk operations */
            err = RADIO_ERROR_BUSY;
            break;
        }
#    if LR11XX_RADIO_CFG_LBM_CONCURRENCY_SUPPORT
        else if (RADIO_LR11XX_LBM_BRIDGE_STATE_SIDEWALK_BLANKING_MODE == drv_ctx->lbm.bridge_state)
        {
            /* Report success to Sidewalk without doing anything */
            drv_ctx->settings_cache.fsk_pkt_params = *packet_params;
            err = RADIO_ERROR_NONE;
            break;
        }
#    endif /* LR11XX_RADIO_CFG_LBM_CONCURRENCY_SUPPORT */
#  endif /* LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION */

        sys_err = lr11xx_radio_set_gfsk_pkt_params(drv_ctx, &fsk_pp);
        if (sys_err != LR11XX_STATUS_OK)
        {
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        drv_ctx->settings_cache.fsk_pkt_params = *packet_params;
        err = RADIO_ERROR_NONE;
    } while (0);

    return err;
#else
    (void)packet_params;

    return RADIO_ERROR_NOT_SUPPORTED;
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED uint32_t sid_pal_radio_fsk_time_on_air(const sid_pal_radio_fsk_modulation_params_t * mod_params,
                                                                 const sid_pal_radio_fsk_packet_params_t * packet_params,
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
        lr11xx_radio_pkt_params_gfsk_t fsk_pp;
        lr11xx_radio_mod_params_gfsk_t fsk_mp;

        fsk_pp.pld_len_in_bytes = packetLen;
        radio_mp_to_lr11xx_mp(&fsk_mp, mod_params);
        radio_pp_to_lr11xx_pp(&fsk_pp, packet_params);

        time_on_air = lr11xx_radio_get_gfsk_time_on_air_in_ms(&fsk_pp, &fsk_mp) + LR11XX_RADIO_CFG_IRQ_PROCESS_GUARD_TIME_MS;
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
    SID_PAL_ASSERT(mod_params != NULL);

    const uint32_t num_symb = LR11XX_US_TO_SYMBOLS(delay_micro_secs, mod_params->bit_rate);
    return num_symb;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED uint32_t sid_pal_radio_get_fsk_tx_process_delay(void)
{
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_2
    const halo_drv_semtech_ctx_t * const drv_ctx = lr11xx_get_drv_ctx();
    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->config != NULL);

    const uint32_t fsk_tx_process_delay =
        (LR11XX_RADIO_CFG_USE_DEFAULT_TIMINGS == drv_ctx->config->processing_timings.fsk.tx_process_delay_us) ?
        LR11XX_FSK_TX_DEFAULT_PROCESS_DELAY_US : drv_ctx->config->processing_timings.fsk.tx_process_delay_us;

    const uint32_t process_delay = fsk_tx_process_delay - LR11XX_SIDEWALK_FSK_CS_DURATION_US;
    return process_delay;
#else
    return 0u;
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED uint32_t sid_pal_radio_get_fsk_rx_process_delay(void)
{
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_2
    const halo_drv_semtech_ctx_t * const drv_ctx = lr11xx_get_drv_ctx();
    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->config != NULL);

    const uint32_t fsk_rx_process_delay =
        (LR11XX_RADIO_CFG_USE_DEFAULT_TIMINGS == drv_ctx->config->processing_timings.fsk.rx_process_delay_us) ?
        LR11XX_FSK_RX_DEFAULT_PROCESS_DELAY_US : drv_ctx->config->processing_timings.fsk.rx_process_delay_us;

    const uint32_t process_delay = fsk_rx_process_delay + LR11XX_FSK_RX_PROCESS_SAFETY_GAP_US;
    return process_delay;
#else
    return 0u;
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 */
}
