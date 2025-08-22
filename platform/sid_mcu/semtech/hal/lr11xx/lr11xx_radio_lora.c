/*
 * Copyright (c) 2019-2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * This file supports radio HAL interface specific to LoRa
 */
/**
  ******************************************************************************
  * @file  lr11xx_radio_lora.c
  * @brief LoRa-specific portion of the LR11xx driver for STM32 platform
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

#include "lr11xx_hal.h"
#include "lr11xx_radio.h"
#include "lr11xx_regmem.h"
#include "lr11xx_radio_timings.h"
#include "halo_lr11xx_radio.h"


/* Sidewalk interfaces */
#include <sid_pal_assert_ifc.h>
#include <sid_pal_log_ifc.h>
#include <sid_pal_radio_ifc.h>
#include <sid_pal_radio_lora_defs.h>

/* Utilities and helpers */
#include <cmsis_compiler.h>
#include <sid_stm32_common_utils.h>

/* Private defines -----------------------------------------------------------*/

#define LR11XX_LORA_RX_PROCESS_SAFETY_GAP_US  (50u) /*!< Rx window will be opened earlier by this amount of microseconds to mitigate any SW execution delays and jitter */

#define LR11XX_LORA_NUM_USED_BW               (3u)    /* BW125, BW250, BW500 */
#define LR11XX_LORA_NUM_USED_SF               (8u)    /* SF12~SF5 */

#define LR11XX_LORA_IRQ_PROCESS_TIME_US       (74u)

/* Private constants ---------------------------------------------------------*/

#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
/*
* From Semtech: For symbol-based timeout, LR11xx requires an extra-delay to process if there is a LoRa symbol.
* This extra-delay in the LR11xx can be computed as follows: Rx base delay + Rx input delay + IRQ processing time
* Rx base delay: symbol_time * ((SF + 1) >> 1)+ 1) / 16 * BW / 1000000
* Rx input delay: 57us in BW125; 31us in BW250; 16us in BW500
* IRQ processing time: 74us
*/
static const uint32_t radio_lora_rx_base_delay_us[LR11XX_LORA_NUM_USED_SF]  = {1792u, 896u, 384u, 192u, 80u, 40u, 16u, 8u}; /* SF12~SF5 */
static const uint32_t radio_lora_rx_input_delay_us[LR11XX_LORA_NUM_USED_BW] = {57u, 31u, 16u}; /* BW125, BW250, BW500 */

/* LUT for LoRa symbol duration for various bandwidth and spreading factor combinations */
static const uint32_t radio_lora_symb_time[LR11XX_LORA_NUM_USED_BW][LR11XX_LORA_NUM_USED_SF] = {
                                                        { 32768u, 16384u, 8192u, 4096u, 2048u, 1024u, 512u, 256u }, /* 125 KHz, SF12~SF5 */
                                                        { 16384u,  8192u, 4096u, 2048u, 1024u,  512u, 256u, 128u }, /* 250 KHz, SF12~SF5 */
                                                        {  8192u,  4096u, 2048u, 1024u,  512u,  256u, 128u,  64u }, /* 500 KHz, SF12~SF5 */
                                                    };
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 */

/* Private function prototypes -----------------------------------------------*/

static inline uint32_t lora_need_low_data_rate_optimize(const lr11xx_radio_lora_sf_t sf, const lr11xx_radio_lora_bw_t bw);
static inline void     radio_to_lr11xx_lora_modulation_params(lr11xx_radio_mod_params_lora_t * const sx_m, const sid_pal_radio_lora_modulation_params_t * const rd_m);
static inline void     radio_to_lr11xx_lora_packet_params(lr11xx_radio_pkt_params_lora_t * const sx_p, const sid_pal_radio_lora_packet_params_t * const rd_p);
static inline int32_t  get_payload(const halo_drv_semtech_ctx_t * const drv_ctx, uint8_t * const buffer, uint8_t * const size, const uint32_t max_size);
static inline int32_t  get_lora_crc_present_in_header(halo_drv_semtech_ctx_t *drv_ctx, sid_pal_radio_lora_crc_present_t *is_crc_present);
static inline int32_t  get_lora_crc_present_in_header(halo_drv_semtech_ctx_t * const drv_ctx, sid_pal_radio_lora_crc_present_t * const is_crc_present);

/* Private function definitions ----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline uint32_t lora_need_low_data_rate_optimize(const lr11xx_radio_lora_sf_t sf, const lr11xx_radio_lora_bw_t bw)
{
    uint32_t enable_ldr_optimizations;

    if (((bw == LR11XX_RADIO_LORA_BW_125) && ((sf == LR11XX_RADIO_LORA_SF11) || (sf == LR11XX_RADIO_LORA_SF12)))
     || ((bw == LR11XX_RADIO_LORA_BW_250) &&  (sf == LR11XX_RADIO_LORA_SF12)))
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

SID_STM32_SPEED_OPTIMIZED static inline void radio_to_lr11xx_lora_modulation_params(lr11xx_radio_mod_params_lora_t * const sx_m, const sid_pal_radio_lora_modulation_params_t * const rd_m)
{
    _Static_assert(LR11XX_RADIO_LORA_SF5  == SID_PAL_RADIO_LORA_SF5, "Invalid conversion");
    _Static_assert(LR11XX_RADIO_LORA_SF6  == SID_PAL_RADIO_LORA_SF6, "Invalid conversion");
    _Static_assert(LR11XX_RADIO_LORA_SF7  == SID_PAL_RADIO_LORA_SF7, "Invalid conversion");
    _Static_assert(LR11XX_RADIO_LORA_SF8  == SID_PAL_RADIO_LORA_SF8, "Invalid conversion");
    _Static_assert(LR11XX_RADIO_LORA_SF9  == SID_PAL_RADIO_LORA_SF9, "Invalid conversion");
    _Static_assert(LR11XX_RADIO_LORA_SF10 == SID_PAL_RADIO_LORA_SF10, "Invalid conversion");
    _Static_assert(LR11XX_RADIO_LORA_SF11 == SID_PAL_RADIO_LORA_SF11, "Invalid conversion");
    _Static_assert(LR11XX_RADIO_LORA_SF12 == SID_PAL_RADIO_LORA_SF12, "Invalid conversion");
    sx_m->sf   = (lr11xx_radio_lora_sf_t)rd_m->spreading_factor;

    _Static_assert(LR11XX_RADIO_LORA_BW_10  == SID_PAL_RADIO_LORA_BW_10KHZ,  "Invalid conversion");
    _Static_assert(LR11XX_RADIO_LORA_BW_15  == SID_PAL_RADIO_LORA_BW_15KHZ,  "Invalid conversion");
    _Static_assert(LR11XX_RADIO_LORA_BW_20  == SID_PAL_RADIO_LORA_BW_20KHZ,  "Invalid conversion");
    _Static_assert(LR11XX_RADIO_LORA_BW_31  == SID_PAL_RADIO_LORA_BW_31KHZ,  "Invalid conversion");
    _Static_assert(LR11XX_RADIO_LORA_BW_41  == SID_PAL_RADIO_LORA_BW_41KHZ,  "Invalid conversion");
    _Static_assert(LR11XX_RADIO_LORA_BW_62  == SID_PAL_RADIO_LORA_BW_62KHZ,  "Invalid conversion");
    _Static_assert(LR11XX_RADIO_LORA_BW_125 == SID_PAL_RADIO_LORA_BW_125KHZ, "Invalid conversion");
    _Static_assert(LR11XX_RADIO_LORA_BW_250 == SID_PAL_RADIO_LORA_BW_250KHZ, "Invalid conversion");
    _Static_assert(LR11XX_RADIO_LORA_BW_500 == SID_PAL_RADIO_LORA_BW_500KHZ, "Invalid conversion");
    assert(rd_m->bandwidth != SID_PAL_RADIO_LORA_BW_7KHZ); /* Not supported */
    sx_m->bw   = (lr11xx_radio_lora_bw_t)rd_m->bandwidth;

    _Static_assert(LR11XX_RADIO_LORA_CR_4_5 == SID_PAL_RADIO_LORA_CODING_RATE_4_5, "Invalid conversion");
    _Static_assert(LR11XX_RADIO_LORA_CR_4_6 == SID_PAL_RADIO_LORA_CODING_RATE_4_6, "Invalid conversion");
    _Static_assert(LR11XX_RADIO_LORA_CR_4_7 == SID_PAL_RADIO_LORA_CODING_RATE_4_7, "Invalid conversion");
    _Static_assert(LR11XX_RADIO_LORA_CR_4_8 == SID_PAL_RADIO_LORA_CODING_RATE_4_8, "Invalid conversion");
    _Static_assert(LR11XX_RADIO_LORA_CR_LI_4_5 == SID_PAL_RADIO_LORA_CODING_RATE_4_5_LI, "Invalid conversion");
    _Static_assert(LR11XX_RADIO_LORA_CR_LI_4_6 == SID_PAL_RADIO_LORA_CODING_RATE_4_6_LI, "Invalid conversion");
    _Static_assert(LR11XX_RADIO_LORA_CR_LI_4_8 == SID_PAL_RADIO_LORA_CODING_RATE_4_8_LI, "Invalid conversion");

    sx_m->cr   = (lr11xx_radio_lora_cr_t)rd_m->coding_rate;

    sx_m->ldro = lora_need_low_data_rate_optimize(rd_m->spreading_factor, rd_m->bandwidth);
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline void radio_to_lr11xx_lora_packet_params(lr11xx_radio_pkt_params_lora_t * const sx_p, const sid_pal_radio_lora_packet_params_t * const rd_p)
{
    sx_p->preamble_len_in_symb = rd_p->preamble_length;

    _Static_assert(LR11XX_RADIO_LORA_PKT_EXPLICIT == SID_PAL_RADIO_LORA_HEADER_TYPE_VARIABLE_LENGTH, "Invalid conversion");
    _Static_assert(LR11XX_RADIO_LORA_PKT_IMPLICIT == SID_PAL_RADIO_LORA_HEADER_TYPE_FIXED_LENGTH, "Invalid conversion");
    sx_p->header_type =  (lr11xx_radio_lora_pkt_len_modes_t)rd_p->header_type;
    sx_p->pld_len_in_bytes = rd_p->payload_length;

    _Static_assert(LR11XX_RADIO_LORA_CRC_OFF == SID_PAL_RADIO_LORA_CRC_OFF, "Invalid conversion");
    _Static_assert(LR11XX_RADIO_LORA_CRC_ON  == SID_PAL_RADIO_LORA_CRC_ON , "Invalid conversion");
    sx_p->crc = (lr11xx_radio_lora_crc_t)rd_p->crc_mode;

    _Static_assert(LR11XX_RADIO_LORA_IQ_STANDARD  == SID_PAL_RADIO_LORA_IQ_NORMAL,    "Invalid conversion");
    _Static_assert(LR11XX_RADIO_LORA_IQ_INVERTED  == SID_PAL_RADIO_LORA_IQ_INVERTED , "Invalid conversion");
    sx_p->iq = (lr11xx_radio_lora_iq_t)rd_p->invert_IQ;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline int32_t get_payload(const halo_drv_semtech_ctx_t * const drv_ctx, uint8_t * const buffer, uint8_t * const size, const uint32_t max_size)
{
    int32_t err;
    lr11xx_status_t sys_err;
    lr11xx_radio_rx_buffer_status_t rx_buffer_status = {0};

    do
    {
        sys_err = lr11xx_radio_get_rx_buffer_status(drv_ctx, &rx_buffer_status);
        if (sys_err != LR11XX_STATUS_OK)
        {
            err = RADIO_ERROR_IO_ERROR;
            break;
        }

        if (rx_buffer_status.pld_len_in_bytes > max_size)
        {
            err = RADIO_ERROR_NOMEM;
            break;
        }

        sys_err = lr11xx_regmem_read_buffer8(drv_ctx, buffer, rx_buffer_status.buffer_start_pointer, rx_buffer_status.pld_len_in_bytes);
        if (sys_err != LR11XX_STATUS_OK)
        {
            err = RADIO_ERROR_IO_ERROR;
            break;
        }

        *size = rx_buffer_status.pld_len_in_bytes;
        err = RADIO_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline int32_t get_lora_crc_present_in_header(halo_drv_semtech_ctx_t * const drv_ctx, sid_pal_radio_lora_crc_present_t * const is_crc_present)
{
    int32_t err;

    do
    {
        lr11xx_status_t sys_err;
        lr11xx_radio_lora_cr_t cr;
        bool is_crc_on;

        if (drv_ctx->ver.fw < LR11XX_GET_LORA_RX_INFO_CMD_VERSION_MIN)
        {
            /* Command GetLoRaRxHeaderInfos was added in FW 03.07. It's not available on eralier versions. Use substitution values */
            *is_crc_present = false;
            err = RADIO_ERROR_NONE;
            break;
        }

        sys_err = lr11xx_radio_get_lora_rx_info(drv_ctx, &is_crc_on, &cr);
        if (sys_err != LR11XX_STATUS_OK)
        {
            *is_crc_present = SID_PAL_RADIO_CRC_PRESENT_INVALID;
            err = RADIO_ERROR_IO_ERROR;
            break;
        }

        *is_crc_present = (is_crc_on) ? SID_PAL_RADIO_CRC_PRESENT_ON : SID_PAL_RADIO_CRC_PRESENT_OFF;
        err = RADIO_ERROR_NONE;
    } while (0);

    return err;
}

/* Global function definitions -----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t radio_lora_process_rx_done(halo_drv_semtech_ctx_t * const drv_ctx)
{
    int32_t err;
    lr11xx_status_t sys_err;
    sid_pal_radio_rx_packet_t * const radio_rx_packet = drv_ctx->radio_rx_packet;
    sid_pal_radio_lora_rx_packet_status_t * const lora_rx_packet_status = &radio_rx_packet->lora_rx_packet_status;
    int16_t tmp_rssi;

    do
    {
        err = get_payload(drv_ctx, radio_rx_packet->rcv_payload, &radio_rx_packet->payload_len, SID_PAL_RADIO_RX_PAYLOAD_MAX_SIZE);
        if (err != RADIO_ERROR_NONE)
        {
            break;
        }

        lr11xx_radio_pkt_status_lora_t lora_pkt_status;
        sys_err = lr11xx_radio_get_lora_pkt_status(drv_ctx, &lora_pkt_status);
        if (sys_err != LR11XX_STATUS_OK)
        {
            err = RADIO_ERROR_IO_ERROR;
            break;
        }

        lora_rx_packet_status->rssi        = lr11xx_radio_hal_get_adjusted_rssi(drv_ctx, lora_pkt_status.rssi_pkt_in_dbm);
        lora_rx_packet_status->snr         = lora_pkt_status.snr_pkt_in_db;
        tmp_rssi                           = lr11xx_radio_hal_get_adjusted_rssi(drv_ctx, lora_pkt_status.signal_rssi_pkt_in_dbm);
        lora_rx_packet_status->signal_rssi = tmp_rssi < INT8_MIN ? INT8_MIN : (int8_t)tmp_rssi;

        /*
         * NOTE: By default sx126x radio chip's rssi saturates at -106 dBm (or some cases -107 dBm)
         * After that the signal deterioration is reflected on the snr change.
         * to make the rssi carry more significance to the upper layer consumers, we need to figure out
         * a derived rssi value combining the rssi and snr in a meaningful way.
         *
         * Based on the empirical data, the formula is: <adjusted rssi = raw rssi + snr (when snr is negative)>.
         * This adjusted value is reported and used in all the ring products.
         */
        if (lora_rx_packet_status->snr < 0)
        {
            lora_rx_packet_status->rssi += lora_rx_packet_status->snr;
        }

        err = get_lora_crc_present_in_header(drv_ctx, &lora_rx_packet_status->is_crc_present);
        if (err != RADIO_ERROR_NONE)
        {
            break;
        }
    } while(0);

    return err;
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

    /**
     * IMPORTANT: Long Interleaver (CR = 4/5  LI, 4/6 LI, and 4/8 LI) is supported for packets of minimum Payload length of 8 bytes,
     *            and maximum Payload length of 253 bytes if the CRC is activated (255 bytes if the CRC is deactivated).
     */
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
            mod_params->coding_rate      = (li_enable != 0u) ? SID_PAL_RADIO_LORA_CODING_RATE_4_5_LI : SID_PAL_RADIO_LORA_CODING_RATE_4_5;
            break;

    case SID_PAL_RADIO_DATA_RATE_2KBPS:
            mod_params->spreading_factor = SID_PAL_RADIO_LORA_SF11;
            mod_params->bandwidth        = SID_PAL_RADIO_LORA_BW_500KHZ;
            mod_params->coding_rate      = (li_enable != 0u) ? SID_PAL_RADIO_LORA_CODING_RATE_4_5_LI : SID_PAL_RADIO_LORA_CODING_RATE_4_5;
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
    int32_t err;
    lr11xx_status_t sys_err;
    const halo_drv_semtech_ctx_t * const drv_ctx = lr11xx_get_drv_ctx();
    uint8_t syncword_cfg;

    SID_PAL_ASSERT(drv_ctx != NULL);

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

        /* Select syncword configuratin value based on the requested sync word */
        if (0x1424u == sync_word)
        {
            syncword_cfg = 0x12u;
        }
        else
        {
            syncword_cfg = 0x34u;
        }

        if (drv_ctx->ver.fw >= LR11XX_SET_LORA_SYNC_WORD_CMD_VERSION_MIN)
        {
            /* SetLoRaSyncWord command is supported */
            sys_err = lr11xx_radio_set_lora_sync_word(drv_ctx, syncword_cfg);
            if (sys_err != LR11XX_STATUS_OK)
            {
                err = RADIO_ERROR_HARDWARE_ERROR;
                break;
            }
        }
        else
        {
            /* We are running on an old firmware, use deprected API */
            sys_err = lr11xx_radio_set_lora_public_network(drv_ctx, syncword_cfg);
            if (sys_err != LR11XX_STATUS_OK)
            {
                err = RADIO_ERROR_HARDWARE_ERROR;
                break;
            }
        }

        /* Done */
        err = RADIO_ERROR_NONE;
    } while (0);

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
    halo_drv_semtech_ctx_t * const drv_ctx = lr11xx_get_drv_ctx();

    SID_PAL_ASSERT(drv_ctx != NULL);

    /* This drvier uses software timer to control Tx/Rx timeouts to avoid the radio falling back to STBY_RC state on timeout. STBY_XOSC is used instead */
    drv_ctx->settings_cache.lora_sync_timeout = num_of_symbols;

    return RADIO_ERROR_NONE;
#else
    (void)num_of_symbols;

    return RADIO_ERROR_NOT_SUPPORTED;
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_set_lora_modulation_params(const sid_pal_radio_lora_modulation_params_t * mod_params)
{
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
    int32_t err;
    lr11xx_status_t sys_err;
    lr11xx_radio_mod_params_lora_t lora_mod_params;
    halo_drv_semtech_ctx_t * const drv_ctx = lr11xx_get_drv_ctx();

    SID_PAL_ASSERT(drv_ctx != NULL);

    radio_to_lr11xx_lora_modulation_params(&lora_mod_params, mod_params);

#  if LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION
    if (RADIO_LR11XX_LBM_BRIDGE_STATE_EXCLUSIVE_MODE == drv_ctx->lbm.bridge_state)
    {
        /* Don't allow Sidewalk operations */
        return RADIO_ERROR_BUSY;
    }
#    if LR11XX_RADIO_CFG_LBM_CONCURRENCY_SUPPORT
    else if (RADIO_LR11XX_LBM_BRIDGE_STATE_SIDEWALK_BLANKING_MODE == drv_ctx->lbm.bridge_state)
    {
        /* Report success to Sidewalk without doing anything */
        return RADIO_ERROR_NONE;
    }
#    endif /* LR11XX_RADIO_CFG_LBM_CONCURRENCY_SUPPORT */
#  endif /* LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION */

    sys_err = lr11xx_radio_set_lora_mod_params(drv_ctx, &lora_mod_params);
    if (sys_err != LR11XX_STATUS_OK)
    {
        err = RADIO_ERROR_HARDWARE_ERROR;
    }
    else
    {
        drv_ctx->settings_cache.lora_mod_params = *mod_params;
        err = RADIO_ERROR_NONE;
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
    int32_t err;
    lr11xx_status_t sys_err;
    lr11xx_radio_pkt_params_lora_t lora_packet_params;
    halo_drv_semtech_ctx_t * const drv_ctx = lr11xx_get_drv_ctx();

    SID_PAL_ASSERT(drv_ctx != NULL);

    radio_to_lr11xx_lora_packet_params(&lora_packet_params, packet_params);

#  if LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION
    if (RADIO_LR11XX_LBM_BRIDGE_STATE_EXCLUSIVE_MODE == drv_ctx->lbm.bridge_state)
    {
        /* Don't allow Sidewalk operations */
        return RADIO_ERROR_BUSY;
    }
#    if LR11XX_RADIO_CFG_LBM_CONCURRENCY_SUPPORT
    else if (RADIO_LR11XX_LBM_BRIDGE_STATE_SIDEWALK_BLANKING_MODE == drv_ctx->lbm.bridge_state)
    {
        /* Report success to Sidewalk without doing anything */
        return RADIO_ERROR_NONE;
    }
#    endif /* LR11XX_RADIO_CFG_LBM_CONCURRENCY_SUPPORT */
#  endif /* LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION */

    sys_err = lr11xx_radio_set_lora_pkt_params(drv_ctx, &lora_packet_params);
    if (sys_err != LR11XX_STATUS_OK)
    {
        err = RADIO_ERROR_HARDWARE_ERROR;
    }
    else
    {
        drv_ctx->settings_cache.lora_pkt_params = *packet_params;
        err = RADIO_ERROR_NONE;
    }

    return err;
#else
    (void)packet_params;

    return RADIO_ERROR_NOT_SUPPORTED;
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_set_lora_cad_params(const sid_pal_radio_lora_cad_params_t * cad_params)
{
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
    int32_t err;
    lr11xx_status_t sys_err;
    lr11xx_radio_cad_params_t lora_cad_params;
    halo_drv_semtech_ctx_t * const drv_ctx = lr11xx_get_drv_ctx();

    SID_PAL_ASSERT(drv_ctx != NULL);

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

        lora_cad_params.cad_symb_nb     = cad_params->cad_symbol_num;
        lora_cad_params.cad_detect_peak = cad_params->cad_detect_peak;
        lora_cad_params.cad_detect_min  = cad_params->cad_detect_min;

        switch (cad_params->cad_exit_mode)
        {
            case SID_PAL_RADIO_CAD_EXIT_MODE_CS_ONLY:
                lora_cad_params.cad_exit_mode = LR11XX_RADIO_CAD_EXIT_MODE_STANDBYRC;
                err = RADIO_ERROR_NONE;
                break;

            case SID_PAL_RADIO_CAD_EXIT_MODE_CS_RX:
                lora_cad_params.cad_exit_mode = LR11XX_RADIO_CAD_EXIT_MODE_RX;
                err = RADIO_ERROR_NONE;
                break;

            case SID_PAL_RADIO_CAD_EXIT_MODE_CS_LBT:
                lora_cad_params.cad_exit_mode = LR11XX_RADIO_CAD_EXIT_MODE_TX;
                err = RADIO_ERROR_NONE;
                break;

            default:
                err = RADIO_ERROR_INVALID_PARAMS;
                break;
        }

        if (err != RADIO_ERROR_NONE)
        {
            break;
        }

        lr11xx_radio_set_lora_exit_mode(cad_params->cad_exit_mode);
        lora_cad_params.cad_timeout = LR11XX_RX_TX_TIMEOUT_US_TO_TUS(cad_params->cad_timeout);

        sys_err = lr11xx_radio_set_cad_params(drv_ctx, &lora_cad_params);
        if (sys_err != LR11XX_STATUS_OK)
        {
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        err = RADIO_ERROR_NONE;
    } while (0);

    return err;
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

        /* Add overhead delays */
        cad_duration += radio_lora_rx_base_delay_us[sf_idx];
        cad_duration += radio_lora_rx_input_delay_us[bw_idx];
        cad_duration += LR11XX_LORA_IRQ_PROCESS_TIME_US;
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
        lr11xx_radio_pkt_params_lora_t lora_packet_params;
        lr11xx_radio_mod_params_lora_t lora_mod_params;

        radio_to_lr11xx_lora_modulation_params(&lora_mod_params, mod_params);
        radio_to_lr11xx_lora_packet_params(&lora_packet_params, packet_params);
        lora_packet_params.pld_len_in_bytes = packet_len;

        time_on_air = lr11xx_radio_get_lora_time_on_air_in_ms(&lora_packet_params, &lora_mod_params) + LR11XX_RADIO_CFG_IRQ_PROCESS_GUARD_TIME_MS;
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

/**
 * @brief Calculates the minimum number of symbols that takes more than delay_micro_sec of air time.
 *
 * In the case of a fractional number of symbols, the return value is rounded up to the next integer.
 * Does not affect the radio state and can be executed without radio ownership.
 * In the case of an error, 0 is returned.
 *
 *  @param[in] mod_params Current modulation parameters that phy uses. If null, zero will be returned.
 *  @param[in] delay_micro_sec Input amount of time that will be translated to number of symbols.
 *  @return number of symbols
 *
 */
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
    lr11xx_radio_mod_params_lora_t lora_mod_params;

    radio_to_lr11xx_lora_modulation_params(&lora_mod_params, mod_params);
    return lr11xx_radio_timings_get_delay_between_last_bit_sent_and_rx_done_in_us(&lora_mod_params);
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
    const halo_drv_semtech_ctx_t * const drv_ctx = lr11xx_get_drv_ctx();
    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->config != NULL);

    const uint32_t lora_tx_process_delay =
        (LR11XX_RADIO_CFG_USE_DEFAULT_TIMINGS == drv_ctx->config->processing_timings.lora.tx_process_delay_us) ?
        LR11XX_LORA_TX_DEFAULT_PROCESS_DELAY_US : drv_ctx->config->processing_timings.lora.tx_process_delay_us;

    const uint32_t process_delay = lora_tx_process_delay;
    return process_delay;
#else
    return 0u;
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED uint32_t sid_pal_radio_get_lora_rx_process_delay(void)
{
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
    const halo_drv_semtech_ctx_t * const drv_ctx = lr11xx_get_drv_ctx();
    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->config != NULL);

    const uint32_t lora_rx_process_delay =
        (LR11XX_RADIO_CFG_USE_DEFAULT_TIMINGS == drv_ctx->config->processing_timings.lora.rx_process_delay_us) ?
        LR11XX_LORA_RX_DEFAULT_PROCESS_DELAY_US : drv_ctx->config->processing_timings.lora.rx_process_delay_us;

    register uint32_t process_delay = lora_rx_process_delay + LR11XX_LORA_RX_PROCESS_SAFETY_GAP_US;
    return process_delay;
#else
    return 0u;
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 */
}

/*----------------------------------------------------------------------------*/

/**
 * @brief Calculates the lora symbol timeout in us.
 *
 * In the case of an error, 0 is returned.
 *
 *  @param[in] mod_params Current modulation parameters that phy uses. If null, zero will be returned.
 *  @param[in] number_of_symbol Input number of symbol .
 *  @return lora symbol timeout in us
 *
 */
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

        /* Compute generic symbol duration */
        symbol_timeout = radio_lora_symb_time[bw_idx][sf_idx] * number_of_symbol;

        /* Add overhead delays */
        symbol_timeout += radio_lora_rx_base_delay_us[sf_idx];
        symbol_timeout += radio_lora_rx_input_delay_us[bw_idx];
        symbol_timeout += LR11XX_LORA_IRQ_PROCESS_TIME_US;
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
