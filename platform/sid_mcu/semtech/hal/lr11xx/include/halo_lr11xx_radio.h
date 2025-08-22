/*
 * Copyright (c) 2019-2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * This file has private functions needed by the driver
 */
/**
  ******************************************************************************
  * @file    halo_lr11xx_radio.h
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

#ifndef __HALO_LR11XX_RADIO_H_
#define __HALO_LR11XX_RADIO_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>

#include <sid_pal_serial_bus_ifc.h>
#include <lr11xx_radio_config.h>
#include <sid_pal_radio_ifc.h>
#include <sid_pal_gpio_ifc.h>
#include <sid_pal_timer_ifc.h>

#include "lr11xx_system.h"

#if LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION
#include <cmsis_os2.h>
#endif /* LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION */

/* Exported constants --------------------------------------------------------*/

#define LP_MAX_POWER                                ( 14)
#define LP_MIN_POWER                                (-17)
#define HP_MAX_POWER                                ( 22)
#define HP_MIN_POWER                                ( -9)

#define LR11XX_SET_LORA_SYNC_WORD_CMD_VERSION_MIN   (0x0303u)     /*!< Minimum FW version where SetLoRaSyncWord command is available */
#define LR11XX_GET_LORA_RX_INFO_CMD_VERSION_MIN     (0x0307u)     /*!< Minimum FW version where GetLoRaRxHeaderInfo command is available */

#define LR11XX_RADIO_MIN_SUPPORTED_FW_VER_LR1110    (0x0401u)     /*!< Minimum required firmware version of LR1110 IC to work properly without any workarounds and substitutions */
#define LR11XX_RADIO_MIN_SUPPORTED_FW_VER_LR1120    (0x0201u)     /*!< Minimum required firmware version of LR1120 IC to work properly without any workarounds and substitutions */
#define LR11XX_RADIO_MIN_SUPPORTED_FW_VER_LR1121    (0x0103u)     /*!< Minimum required firmware version of LR1121 IC to work properly without any workarounds and substitutions */

#define LR11XX_RADIO_RXTX_NO_TIMEOUT_VAL            (0x000000u)
#define LR11XX_RADIO_RX_CONTINUOUS_VAL              (0xFFFFFFu)
#define LR11XX_RADIO_INFINITE_TIME                  (0xFFFFFFFFu)

#define LR11XX_US_TO_TUS_COMMON_DIVIDER             (64UL)

#define LR11XX_RADIO_LR11XX_IC_ID                   (0x01u)       /*!< ID reported by the GetVersion command for LR11xx transceiver */
#define LR11XX_RADIO_LR1120_IC_ID                   (0x02u)       /*!< ID reported by the GetVersion command for LR11xx transceiver */
#define LR11XX_RADIO_LR1121_IC_ID                   (0x03u)       /*!< ID reported by the GetVersion command for LR11xx transceiver */
#define LR11XX_RADIO_BOOTLOADER_MODE_ID             (0xDFu)       /*!< ID reported by the GetVersion command when LR11xx transceiver is in the bootloader mode */

#define LR11XX_RADIO_TX_BUFFER_BASE_OFFSET          (0x00u)       /*!< The Tx buffer start offset is fixed to 0x00 to simplify Tx buffer uploading */
#define LR11XX_RADIO_RX_BUFFER_BASE_OFFSET          (0x00u)       /*!< The Rx buffer start offset is fixed to 0x00 to simplify readout */

#define LR11XX_DISABLE_WARNINGS                     (1)           /*!< Suppress  compile-time warnings in Semtech SWDR001 since they do not apply for this driver */

/* Exported macro ------------------------------------------------------------*/

#define LR11XX_US_TO_SYMBOLS(time_in_us, bit_rate)  ((uint64_t)(((uint64_t)time_in_us * (uint64_t)bit_rate) + (uint64_t)(LR11XX_US_IN_SEC / 2u)) / LR11XX_US_IN_SEC)

#define LR11XX_RX_TX_TIMEOUT_US_TO_TUS(_US_)        ((0u == (_US_)) ? 0u :                                                                                                                  \
                                                     (LR11XX_RADIO_INFINITE_TIME == (_US_)) ? LR11XX_RADIO_RXTX_NO_TIMEOUT_VAL :                                                            \
                                                     (((_US_) * (LR11XX_TUS_IN_SEC / LR11XX_US_TO_TUS_COMMON_DIVIDER)) + (((LR11XX_US_IN_SEC / LR11XX_US_TO_TUS_COMMON_DIVIDER) +1u) / 2u)) \
                                                     / (LR11XX_US_IN_SEC / LR11XX_US_TO_TUS_COMMON_DIVIDER)                                                                                 \
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

typedef struct {
    uint32_t start;
    uint32_t stop;
} lr11xx_freq_band_t;

#if LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION
typedef enum {
    RADIO_LR11XX_LBM_BRIDGE_STATE_INACTIVE               = 0, /*!< LBM bridge is deactivated, radio driver is in Sidewalk mode */
    RADIO_LR11XX_LBM_BRIDGE_STATE_ACTIVATION_PENDING     = 1, /*!< LBM bridge mode is requested, radio driver is still in Sidewalk mode, but will switch immediately after current radio operation is finished */
    RADIO_LR11XX_LBM_BRIDGE_STATE_EXCLUSIVE_MODE         = 2, /*!< LBM bridge is active and in exclusive mode. Sidewalk stack does not have access to the radio (RADIO_ERROR_BUSY is reported on any Sidewalk-related request) */
#  if LR11XX_RADIO_CFG_LBM_CONCURRENCY_SUPPORT
    RADIO_LR11XX_LBM_BRIDGE_STATE_SIDEWALK_BLANKING_MODE = 3, /*!< LBM bridge is active and in Sidewalk Blanking mode - whenever Sidewalk stack tries to access the radio an absence of radio signal is simulated (e.g. Rx timeout) */
#  endif /* LR11XX_RADIO_CFG_LBM_CONCURRENCY_SUPPORT */
} radio_lr11xx_lbm_bridge_state_t;
#endif /* LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION */

typedef struct {
    uint8_t                                      init_done;
    uint8_t                                      radio_state;
    uint8_t                                      sid_busy_indication; /*!< true: report SID_RADIO_BUSY state via Sidewalk APIs, false: report actual radio state via Sidewalk APIs */
#if (__ARM_ARCH_6M__ == 0)
    volatile uint8_t                             radio_is_running_high_prio; /*!< Indicates whether the radio interrupt priority is elevated and RTOS API calls cannot be made */
#endif /* (__ARM_ARCH_6M__ == 0) */
#if LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION
    struct {
        radio_lr11xx_lbm_bridge_state_t          bridge_state;               /*!< Indicates whether the driver currently serves as a bridge for LoRa Basics Modem (LBM) */
#  if LR11XX_RADIO_CFG_LBM_CONCURRENCY_SUPPORT
        uint8_t                                  simulated_radio_state;      /*!< Simulated radio state to be reported to Sidewalk stack when Sidewalk Blanking bridge mode is used */
#  endif /* LR11XX_RADIO_CFG_LBM_CONCURRENCY_SUPPORT */
#  if LR11XX_RADIO_CFG_GEOLOCATION_SUPPORT
        uint8_t                                  gnss_scan_active;           /*!< LBM runs a GNSS scan session */
        uint8_t                                  wifi_scan_active;           /*!< LBM runs a WiFi scan session */
#endif /* LR11XX_RADIO_CFG_GEOLOCATION_SUPPORT */
        uint8_t                                  radio_sleep_pending;        /*!< LBM requested radio to enter Sleep mode, but the driver postponed that */
        osSemaphoreId_t                          sidewalk_radio_access_lock; /*!< Semaphore is release every time Sidewalk stack finalizes a radio operation and puts the radio into sleep */
    }                                            lbm;
#endif /* LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION */

    const lr11xx_radio_device_config_t           *config;
    const struct sid_pal_serial_bus_iface        *bus_iface;

    sid_pal_radio_modem_mode_t                   modem;
    sid_pal_radio_rx_packet_t                    *radio_rx_packet;
    sid_pal_radio_event_notify_t                 report_radio_event;
    sid_pal_radio_irq_handler_t                  irq_handler;
    sid_pal_timer_t                              radio_timeout_mon;

    sid_pal_radio_cad_param_exit_mode_t          cad_exit_mode;
    radio_lr11xx_pa_cfg_t                        pa_cfg;
#if HALO_ENABLE_DIAGNOSTICS
    uint32_t                                     pa_cfg_override;
#endif

    lr11xx_system_irq_mask_t                     irq_mask;
    uint32_t                                     radio_freq_hz;
    const lr11xx_freq_band_t *                   radio_freq_band;

    lr11xx_system_version_t                      ver;

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

    struct {
        uint8_t                                  stat1;
        uint8_t                                  stat2;
        uint16_t                                 command;
    }                                            last;
    const radio_lr11xx_regional_param_t *        regional_radio_param;

    sid_pal_radio_sleep_start_notify_handler_t   sleep_start_notify_cb; /*!< This callback is triggered when the radio enters sleep state. User app may use this time to perform other activities, e.g. GPS or WiFi scan */
} halo_drv_semtech_ctx_t;

typedef enum {
    LR11XX_RADIO_FRONTEND_MODE_OFF = 0,
    LR11XX_RADIO_FRONTEND_MODE_TX  = 1,
    LR11XX_RADIO_FRONTEND_MODE_RX  = 2,
} lr11xx_subghz_fe_mode_t;

#if LR11XX_RADIO_CFG_GEOLOCATION_SUPPORT
typedef enum {
    LR11XX_GNSS_FRONTEND_MODE_OFF = 0,
    LR11XX_GNSS_FRONTEND_MODE_ON  = 1,
} lr11xx_gnss_fe_mode_t;
#endif /* LR11XX_RADIO_CFG_GEOLOCATION_SUPPORT */

/* Exported functions --------------------------------------------------------*/

halo_drv_semtech_ctx_t* lr11xx_get_drv_ctx(void);

int32_t radio_lora_process_rx_done(halo_drv_semtech_ctx_t * const drv_ctx);

int32_t radio_fsk_process_sync_word_detected(const halo_drv_semtech_ctx_t * const drv_ctx, const uint32_t rx_completed);

int32_t radio_fsk_process_rx_done(halo_drv_semtech_ctx_t * const drv_ctx, radio_fsk_rx_done_status_t * const rx_done_status);

void lr11xx_radio_set_lora_exit_mode(sid_pal_radio_cad_param_exit_mode_t cad_exit_mode);

int32_t radio_lr11xx_set_subghz_fem_mode(const lr11xx_subghz_fe_mode_t frontend_mode);

int32_t radio_lr11xx_compute_tx_power_config(const int8_t power);

#if LR11XX_RADIO_CFG_GEOLOCATION_SUPPORT
int32_t radio_lr11xx_set_gnss_fem_mode(const lr11xx_gnss_fe_mode_t frontend_mode);
#endif /* LR11XX_RADIO_CFG_GEOLOCATION_SUPPORT */

#if HALO_ENABLE_DIAGNOSTICS
int32_t get_radio_lr11xx_pa_config(radio_lr11xx_pa_cfg_t *cfg);
#endif /* HALO_ENABLE_DIAGNOSTICS */

#ifdef __cplusplus
}
#endif

#endif /* __HALO_LR11XX_RADIO_H_ */
