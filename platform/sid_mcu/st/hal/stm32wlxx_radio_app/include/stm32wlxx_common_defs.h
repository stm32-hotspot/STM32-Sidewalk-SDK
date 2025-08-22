/**
  ******************************************************************************
  * @file    stm32wlxx_common_defs.h
  * @brief   Common definitions for the STM32WLxx Sidewalk Radio App driver
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

#ifndef __STM32WLXX_RADIO_APP_COMMON_DEFS_H_
#define __STM32WLXX_RADIO_APP_COMMON_DEFS_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include <cmsis_os2.h>
#include <sid_pal_serial_bus_ifc.h>

#include "comm_def.h"
#include "stm32wlxx_app_radio_config.h"
#include "stm32wlxx_radio_ext_ifc.h"

/* Exported types ------------------------------------------------------------*/

typedef struct {
    const stm32wlxx_app_radio_device_config_t * config;
    const struct sid_pal_serial_bus_iface *     bus_iface;

    sid_pal_radio_rx_packet_t *                 radio_rx_packet;
    sid_pal_radio_event_notify_t                report_radio_event;
    sid_pal_radio_irq_handler_t                 irq_handler;
    stm32wlxx_pal_radio_modem_mode_t            modem;
    uint8_t                                     radio_state;

    uint16_t                                    subghz_irq_mask;
    uint16_t                                    app_irq_mask;
    stm32wlxx_rcp_irq_status_t                  last_irq_status;
    osSemaphoreId_t                             irq_detected_wait_lock;
    osSemaphoreId_t                             req_completed_wait_lock;

#if STM32WLxx_RADIO_APP_LPM_SUPPORT
    uint32_t                                    ldt_ongoing; /*!< Indicates that Long Data Transfer (LDT) SPI transaction is ongoing */
#endif /* STM32WLxx_RADIO_APP_LPM_SUPPORT */

#if STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER
    struct {
        osThreadId_t                            task;
        osMessageQueueId_t                      outbound_msg_queue;
        struct sid_timespec                     cut_off_time;
        volatile uint32_t                       udt_enabled;
    } udt_ctx;
#endif /* STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER */

    sid_pal_radio_cad_param_exit_mode_t         cad_exit_mode;
    stm32wlxx_radio_pa_cfg_t                    pa_cfg;
#if HALO_ENABLE_DIAGNOSTICS
    bool                                        pa_cfg_configured;
#endif

    uint32_t                                    radio_freq_hz;
    const stm32wlxx_radio_regional_param_t *    regional_radio_param;

#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_2
    sid_pal_radio_fsk_phy_settings_t            applied_fsk_phy_cfg;
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 */

#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
    sid_pal_radio_lora_phy_settings_t           applied_lora_phy_cfg;
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 */

    /* variables below are used to check against double set of syncword */
    uint8_t                                     sync_word[SID_PAL_RADIO_FSK_SYNC_WORD_LENGTH];
    uint8_t                                     sync_word_length;

    uint8_t                                     init_done;
    uint8_t                                     irq_enabled;
    uint8_t                                     req_with_ack_ongoing;
#if (__ARM_ARCH_6M__ == 0)
    volatile uint8_t                            radio_is_running_high_prio; /*!< Indicates whether the radio interrupt priority is elevated and RTOS API calls cannot be made */
#endif /* (__ARM_ARCH_6M__ == 0) */

    /* Extended driver user API */
    stm32wlxx_ext_ifc_on_incoming_user_data     on_user_data_rx;

    struct {
        uint32_t                                    drv_err_cntr;        /*!< Counter of the critical radio driver errors */
        struct sid_timespec                         last_err_timestamp;  /*!< Timestamp of the last detected error */
    }                                           error_monitor;           /*!< Error management context - used to store error-related info and manage recovery */
} halo_drv_stm32wlxx_ctx_t;

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* __STM32WLXX_RADIO_APP_COMMON_DEFS_H_ */
