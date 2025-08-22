/**
  ******************************************************************************
  * @file    s2_lp_radio.h
  * @brief   Handling of the S2-LP transceiver for Sidewalk application
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

#ifndef __S2_LP_RADIO_H_
#define __S2_LP_RADIO_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "s2_lp_ic_definitions.h"
#include "s2_lp_radio_config.h"

#include <sid_pal_radio_fsk_defs.h>
#include <sid_pal_timer_ifc.h>
#include <sid_time_types.h>

/* Exported types ------------------------------------------------------------*/

typedef struct {
    const s2_lp_radio_device_config_t     * config;
    const struct sid_pal_serial_bus_iface * bus_iface;

    /* Sidewalk SDK callbacks and buffers */
    sid_pal_radio_rx_packet_t *             radio_rx_packet;
    sid_pal_radio_event_notify_t            report_radio_event;
    sid_pal_radio_irq_handler_t             radio_irq_handler;
    uint8_t                                 radio_state;

    struct {
        sid_pal_timer_t                     timer;
        struct sid_timespec                 start_ts;
        struct sid_timespec                 timeout_ts;
    } rco_calibration;

    const s2_lp_radio_regional_params_t *   regional_radio_params; /*!< Pointer to the set of the regional radio params that is currently active */

    sid_pal_radio_cad_param_exit_mode_t     cad_exit_mode;

    struct {
        s2_lp_radio_pa_dynamic_cfg_t            pa_dyn_cfg;       /*!< Dynamic config options - can be modified by the user app on the fly */
        uint32_t                                pa_ramp_bit_rate; /*!< Bit rate for which PA ramping was configured - S2-LP counts ramp time in terms of the bit time, so we need to adjust the settings each time the modulation parameters are changed */
#if S2LP_RADIO_CFG_USE_EXTERNAL_PA
        uint8_t                                 ext_pa_en;        /*!< Specifies if the external PA shall be enabled to reach the desired power level */
#endif /* S2LP_RADIO_CFG_USE_EXTERNAL_PA */
#if HALO_ENABLE_DIAGNOSTICS
        uint8_t                                 pa_cfg_configured;
#endif /* HALO_ENABLE_DIAGNOSTICS */
    }                                       pa_params;

    /* Cached hardware settings and states */
    sl2_lp_ic_irq_mask_t                    radio_irq_mask;
    uint32_t                                ref_clk_div_enabled;                           /*!< Determines if the reference clock divider is active */
    uint32_t                                ref_freq_hz;                                   /*!< Actual reference clock frequency in Hz */
    uint32_t                                dig_freq_hz;                                   /*!< Actual digital domain clock frequency in Hz */
    uint32_t                                dig_freq_mhz;                                  /*!< Digital domain frequency expressed in MHz - to avoid repetitive divisions in runtime */
    uint32_t                                radio_freq_hz;                                 /*!< RF frequency that's currently applied to the radio */
    uint32_t                                radio_freq_band_factor;                        /*!< S2-LP frequency band factor for the currently active RF frequency */
    uint32_t                                smps_krm_tx;                                   /*!< KRM multiplier for Tx mode - used to adjust SMPS frequency for Tx operation */
    uint32_t                                smps_krm_rx;                                   /*!< KRM multiplier for Rx mode - used to adjust SMPS frequency for Rx operation */
    uint32_t                                current_bit_rate;                              /*!< Bit rate that is currently configured in the radio */

    /* Cache for the register values written to the S2-LP IC */
    struct {
        s2_lp_ic_status_t                   mc_state;
        sl2_lp_ic_reg_pa_cfg_t              pa_mode_cfg_regs;
        sl2_lp_ic_reg_pckt_params_t         pckt_cfg_regs;
        sl2_lp_ic_reg_pckt_flt_options_t    pckt_flt_options_reg;
        sl2_lp_ic_reg_pm_conf0_t            pm_conf0_reg;
        sl2_lp_ic_reg_protocol_t            protocol_regs;
        sl2_lp_ic_reg_xo_rco_conf0_t        xo_rco_conf0_reg;
    } regs_cache;

    /* Driver states and statuses */
    uint8_t                                 init_done;
#if (__ARM_ARCH_6M__ == 0)
    volatile uint8_t                        radio_is_running_high_prio; /*!< Indicates whether the radio interrupt priority is elevated and RTOS API calls cannot be made */
#endif /* (__ARM_ARCH_6M__ == 0) */
    uint32_t                                processing_buf_offset;      /*!< Current read/write position in the internal processing buffer */
    uint32_t                                processing_buf_total_bytes; /*!< Total amount of bytes available in the internal processing buffer */
    uint32_t                                last_tx_payload_size;       /*!< The size (in bytes) of the last Tx payload set by the sid_pal_radio_set_tx_payload(). Does not include packet header bytes */
    uint32_t                                rco_cal_init_err_cnt;       /*!< Counter for RCO Calibration initiation faults */

    struct {
        uint32_t                            drv_err_cntr;               /*!< Counter of the critical radio driver errors */
        struct sid_timespec                 last_err_timestamp;         /*!< Timestamp of the last detected error */
    } error_monitor;                                                    /*!< Error management context - used to store error-related info and manage recovery */
} halo_drv_s2_lp_ctx_t;

#ifdef __cplusplus
}
#endif

#endif /* __S2_LP_RADIO_H_ */
