/*
 * Copyright (c) 2019-2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * This file has private functions needed by the driver
 */
/**
  ******************************************************************************
  * @file    sx126x_radio.h
  * @brief   Semtech SX126x radio driver for Sidewalk running on STM32 platform
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

#ifndef __SX126X_RADIO_H_
#define __SX126X_RADIO_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>

#include <sx126x_radio_config.h>
#include <sid_pal_radio_ifc.h>
#include <sid_pal_gpio_ifc.h>
#include <sid_pal_timer_ifc.h>

#include <sx126x.h>
#include <sx126x_hal.h>
#include <sx126x_halo.h>
#include <sx126x_regs.h>
#include <sx126x_timings.h>

#if SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION
#include <cmsis_os2.h>
#endif /* SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION */

/* Exported constants --------------------------------------------------------*/

#define SX126X_US_TO_TUS_COMMON_DIVIDER             (8000UL)

#define SX126X_RADIO_RXTX_NO_TIMEOUT_VAL            (0x000000u)
#define SX126X_RADIO_RX_CONTINUOUS_VAL              (0xFFFFFFu)
#define SX126X_RADIO_INFINITE_TIME                  (0xFFFFFFFFu)

#define SX126X_PA_CFG_PARAM_INVALID                 (0xFFu)       /*!< Indicates that PA config parameter is invalid */
#define SX126X_PA_CFG_USE_LPA                       (0x01u)       /*!< Use Low-Power Amplifier for Tx */
#define SX126X_PA_CFG_USE_HPA                       (0x00u)       /*!< Use High-Power Amplifier for Tx */

#define SX126X_RADIO_TX_BUFFER_BASE_OFFSET          (0x00u)       /*!< The Tx buffer start offset is fixed to 0x00 to simplify Tx buffer uploading */
#define SX126X_RADIO_RX_BUFFER_BASE_OFFSET          (0x00u)       /*!< The Rx buffer start offset is fixed to 0x00 to simplify readout */

/* Exported macro ------------------------------------------------------------*/

#define SX126X_US_TO_SYMBOLS(time_in_us, bit_rate)  ((uint64_t)(((uint64_t)time_in_us * (uint64_t)bit_rate) + (uint64_t)(SX126X_US_IN_SEC / 2u)) / SX126X_US_IN_SEC)

#define SX126X_RX_TX_TIMEOUT_US_TO_TUS(_US_)        ((0u == (_US_)) ? 0u :                                                                                                                   \
                                                     (SX126X_RADIO_INFINITE_TIME == (_US_)) ? SX126X_RADIO_RXTX_NO_TIMEOUT_VAL :                                                             \
                                                     (((_US_) * (SX126X_TUS_IN_SEC / SX126X_US_TO_TUS_COMMON_DIVIDER)) + (((SX126X_US_IN_SEC / SX126X_US_TO_TUS_COMMON_DIVIDER) + 1u) / 2u)) \
                                                     / (SX126X_US_IN_SEC / SX126X_US_TO_TUS_COMMON_DIVIDER)                                                                                  \
                                                    )

/* Exported types ------------------------------------------------------------*/

/*
 * @brief RADIO FSK status enumeration definition
 */
typedef enum {
    RADIO_FSK_RX_DONE_STATUS_OK                  = 0,
    RADIO_FSK_RX_DONE_STATUS_INVALID_PARAMETER   = 1,
    RADIO_FSK_RX_DONE_STATUS_INVALID_LENGTH      = 2,
    RADIO_FSK_RX_DONE_STATUS_BAD_CRC             = 3,
    RADIO_FSK_RX_DONE_STATUS_TIMEOUT             = 4,
    RADIO_FSK_RX_DONE_STATUS_UNKNOWN_ERROR       = 5,
    RADIO_FSK_RX_DONE_STATUS_SW_MARK_NOT_PRESENT = 6,
} radio_fsk_rx_done_status_t;

/* enum for calibration bands in semtech radio */
typedef enum {
    SX126X_BAND_INVALID = 0,
    SX126X_BAND_900M    = 1,
    SX126X_BAND_850M    = 2,
    SX126X_BAND_770M    = 3,
    SX126X_BAND_460M    = 4,
    SX126X_BAND_430M    = 5,
} sx126x_freq_band_t;

typedef enum {
    SX126X_RADIO_FRONTEND_MODE_OFF = 0,
    SX126X_RADIO_FRONTEND_MODE_TX  = 1,
    SX126X_RADIO_FRONTEND_MODE_RX  = 2,
} sx126x_subghz_fe_mode_t;

#if SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION
typedef enum {
    RADIO_SX126X_LBM_BRIDGE_STATE_INACTIVE               = 0, /*!< LBM bridge is deactivated, radio driver is in Sidewalk mode */
    RADIO_SX126X_LBM_BRIDGE_STATE_ACTIVATION_PENDING     = 1, /*!< LBM bridge mode is requested, radio driver is still in Sidewalk mode, but will switch immediately after current radio operation is finished */
    RADIO_SX126X_LBM_BRIDGE_STATE_EXCLUSIVE_MODE         = 2, /*!< LBM bridge is active and in exclusive mode. Sidewalk stack does not have access to the radio (RADIO_ERROR_BUSY is reported on any Sidewalk-related request) */
} radio_sx126x_lbm_bridge_state_t;
#endif /* SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION */

typedef struct {
    uint8_t                                      init_done;
    uint8_t                                      sid_busy_indication; /*!< true: report SID_RADIO_BUSY state via Sidewalk APIs, false: report actual radio state via Sidewalk APIs */
#if (__ARM_ARCH_6M__ == 0)
    volatile uint8_t                             radio_is_running_high_prio; /*!< Indicates whether the radio interrupt priority is elevated and RTOS API calls cannot be made */
#endif /* (__ARM_ARCH_6M__ == 0) */
#if SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION
    struct {
        radio_sx126x_lbm_bridge_state_t          bridge_state;               /*!< Indicates whether the driver currently serves as a bridge for LoRa Basics Modem (LBM) */
        osSemaphoreId_t                          sidewalk_radio_access_lock; /*!< Semaphore is release every time Sidewalk stack finalizes a radio operation and puts the radio into sleep */
    }                                            lbm;
#endif /* SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION */

    const sx126x_radio_device_config_t           *config;
    const struct sid_pal_serial_bus_iface        *bus_iface;

    sid_pal_radio_modem_mode_t                   modem;
    sid_pal_radio_rx_packet_t                    *radio_rx_packet;
    sid_pal_radio_event_notify_t                 report_radio_event;
    uint8_t                                      radio_state;
    sid_pal_radio_irq_handler_t                  irq_handler;
    sid_pal_timer_t                              radio_timeout_mon;

    sid_pal_radio_cad_param_exit_mode_t          cad_exit_mode;
    radio_sx126x_pa_dynamic_cfg_t                pa_cfg;
#if HALO_ENABLE_DIAGNOSTICS
    uint32_t                                     pa_cfg_override;
#endif

struct {
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
        uint32_t                                 lora_sync_timeout; /*!< Timeout in LoRa symbols for Sync Word detection */
        sid_pal_radio_lora_packet_params_t       lora_pkt_params;
        sid_pal_radio_lora_modulation_params_t   lora_mod_params;
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 */
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_2
        sid_pal_radio_fsk_packet_params_t        fsk_pkt_params;
        sid_pal_radio_fsk_modulation_params_t    fsk_mod_params;
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 */
    }                                            settings_cache;

    uint16_t                                     irq_mask;
    uint16_t                                     trim;
    uint32_t                                     radio_freq_hz;
    sx126x_freq_band_t                           radio_freq_band;

    const radio_sx126x_regional_param_t *        regional_radio_param;
} halo_drv_semtech_ctx_t;

/* Exported functions --------------------------------------------------------*/

halo_drv_semtech_ctx_t * sx126x_get_drv_ctx(void);

int32_t radio_sx126x_set_fem_mode(const sx126x_subghz_fe_mode_t frontend_mode);

int32_t radio_lora_process_rx_done(halo_drv_semtech_ctx_t * const drv_ctx);

int32_t radio_fsk_process_sync_word_detected(const halo_drv_semtech_ctx_t * const drv_ctx, const uint32_t rx_completed);

int32_t radio_fsk_process_rx_done(halo_drv_semtech_ctx_t * const drv_ctx, radio_fsk_rx_done_status_t * const rx_done_status);

void sx126x_radio_set_lora_exit_mode(const sid_pal_radio_cad_param_exit_mode_t cad_exit_mode);

int32_t set_radio_sx126x_trim_cap_val(const uint16_t trim);

int32_t get_radio_sx126x_pa_config(radio_sx126x_pa_dynamic_cfg_t * const cfg);

int32_t radio_sx126x_compute_tx_power_config(const int8_t power);

#ifdef __cplusplus
}
#endif

#endif /* __SX126X_RADIO_H_ */
