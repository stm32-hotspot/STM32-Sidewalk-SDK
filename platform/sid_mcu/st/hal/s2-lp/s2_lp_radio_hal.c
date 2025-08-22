/**
  ******************************************************************************
  * @file    s2_lp_radio_hal.c
  * @brief   Hardware abstraction layer for the S2-LP Sidewalk radio driver
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

#include "s2_lp_radio_config.h"
#include "s2_lp_radio_hal.h"

/* Sidewalk SDK headers */
#include <sid_fsk_phy_cfg.h>
#include <sid_pal_assert_ifc.h>
#include <sid_pal_critical_region_ifc.h>
#include <sid_pal_delay_ifc.h>
#include <sid_pal_gpio_ifc.h>
#include <sid_pal_gpio_ext_ifc.h>
#include <sid_pal_log_ifc.h>
#include <sid_pal_radio_ifc.h>
#include <sid_pal_serial_bus_ext_ifc.h>
#include <sid_pal_timer_ifc.h>
#include <sid_pal_uptime_ifc.h>
#include <sid_time_ops.h>

/* ST platform headers */
#include <sid_stm32_common_utils.h>

#if SID_SDK_CONFIG_RF_TIMINGS_MEASUREMENT
#  include "main.h"
#endif

/* Private defines -----------------------------------------------------------*/

#define S2LP_RADIO_HAL_SHUTDOWN_HOLD_TIME_US               (100u)     /*!< Time is microseconds to hold SDN pin high - shall be at least as specified in S2-LP datasheet (1us) */
#define S2LP_RADIO_HAL_INTERNAL_RESET_PULSE_WIDTH_MS       (3u)       /*!< Duration of the internal reset pulse of the S2-LP IC after Shutdown pin is release. According to the specification this pulse is 2ms maximum */

#define S2LP_RADIO_HAL_SPI_HEADER_WRITE_MASK               (0x00u)    /*!< Write mask for header byte */
#define S2LP_RADIO_HAL_SPI_HEADER_READ_MASK                (0x01u)    /*!< Read mask for header byte */
#define S2LP_RADIO_HAL_SPI_HEADER_ADDRESS_MASK             (0x00u)    /*!< Address mask for header byte */
#define S2LP_RADIO_HAL_SPI_HEADER_COMMAND_MASK             (0x80u)    /*!< Command mask for header byte */

#define S2LP_RADIO_HAL_SPI_HEADER_LENGTH                   (2u)       /*!< Size of the header which prepends any SPI transaction */
#define S2LP_RADIO_HAL_SPI_FIFO_SIZE                       (128u)     /*!< Size of the FIFO inside S2-LP IC */
#define S2LP_RADIO_HAL_SPI_WORKING_BUF_SIZE                (S2LP_RADIO_HAL_SPI_HEADER_LENGTH + S2LP_RADIO_HAL_SPI_FIFO_SIZE) /*!< Maximum size of the SPI buffer for a single transaction) */

#define S2LP_RADIO_HAL_INTERMEDIATE_FREQUENCY_HZ           (300000u)  /*!< As per the datasheet the intermediate frequency shall be set at 300kHz */

#define S2LP_RADIO_HAL_READY_STATE_CLK_CFG_TIMEOUT_US      (50000u)   /*!< Timeout (in microseconds) for the S2-LP to transition to the READY after clocking configuration performed in STANDBY mode - may require additional time for RCO and XTAL to restart */
#define S2LP_RADIO_HAL_STATE_TRANSITION_DEFAULT_TIMEOUT_US (1000u)    /*!< Timeout (in microseconds) for the S2-LP to transition to the desired internal state (e.g. STANDBY, READY, LOCK, etc.) */
#define S2LP_RADIO_HAL_STATE_TRANSITION_PROBE_PERIOD_US    (50u)      /*!< The driver will check for the successful transition with this period (in microseconds) */

#define S2LP_RADIO_HAL_RCO_CALIBRATION_PROBE_PERIOD_MS     (1u)       /*!< The driver will check for the RCO calibration status with this period (in milliseconds) */
#define S2LP_RADIO_HAL_RCO_CALIBRATION_START_WAIT_LIMIT    (100u)     /*!< The driver is polling the RC_CAL_OK bi in MC_STATE1 reg to become zero as indication of the calibration start. It will poll up to this amount of times */
#define S2LP_RADIO_HAL_RCO_CAL_INIT_ERROR_LIMIT            (5u)       /*!< Limit for the missed RCO calibration start events */

#define S2LP_RADIO_HAL_RX_CH_BW_LUT_REF_FREQ_MHZ           (26u)      /*!< Reference frequency for the rx_ch_bandwidth_lut_26mhz - if the actual Fdig is different an adjustment shall be made */

#define S2LP_RADIO_HAL_AFC_ANTICIPATED_MIN_PREAMBLE_LEN    (16u)      /*!< Anticipated minimum preamble length in bits. Used to configure AFC, does not affect actual preamble length */
#define S2LP_RADIO_HAL_RSSI_THRESHOLD_DBM                  (RNET_FSK_DEFAULT_CAD_RSSI_THRESHOLD) /*!< RSSI threshold in dBm for various adjustments (e.g. AFC, CSMA, packet filtering, etc.) - used to avoid unwanted reactions on noise signals */

#define S2LP_RADIO_HAL_SMPS_FREQ_FOR_TX                    (5480000u) /*!< Set the SMPS switching frequency to 5.46MHz on Tx operations for ETSI regulation compliance */
#define S2LP_RADIO_HAL_SMPS_FREQ_FOR_RX                    (3120000u) /*!< Set the SMPS switching frequency to 3.12MHz on Rx operations for ETSI regulation compliance */

#define S2LP_RADIO_HAL_SPI_NSS_DELAY_IN_LPM_US             (40u)      /*!< Additional delay between activating the SPI NSS line and the first SCK edge when S2-LP is in one of the low power modes */

#define S2LP_RADIO_HAL_MAX_PA_RAMP_TICKS_MAX               (32u)      /*!< Maximum number of ticks for power ramping (8 slots up to 4 ticks each = 32 ticks maximum) */

#define S2LP_RADIO_HAL_PM_START_COUNTER                    (75u)      /*!< Radio will wait for this time after PM start before proceeding with Tx/Rx. This is timer counter value (1 tick = 2us approx. */

#define S2LP_RADIO_HAL_ANT_GAIN_CAP_REGION_NA              ((int32_t)(6 * 100)) /* Antenna gain (in dBi * 100) upper limit as per FCC requirements */

/* Private macros ------------------------------------------------------------*/

#ifndef S2LP_RADIO_HAL_EXTRA_LOGGING
/* Set S2LP_RADIO_HAL_EXTRA_LOGGING to 1 to enable extended logs */
#  define S2LP_RADIO_HAL_EXTRA_LOGGING (0)
#endif

#if S2LP_RADIO_HAL_EXTRA_LOGGING
#  define S2LP_RADIO_HAL_LOG_ERROR(...)   SID_PAL_LOG_ERROR(__VA_ARGS__)
#  define S2LP_RADIO_HAL_LOG_WARNING(...) SID_PAL_LOG_WARNING(__VA_ARGS__)
#  define S2LP_RADIO_HAL_LOG_INFO(...)    SID_PAL_LOG_INFO(__VA_ARGS__)
#  define S2LP_RADIO_HAL_LOG_DEBUG(...)   SID_PAL_LOG_DEBUG(__VA_ARGS__)
#  define S2LP_RADIO_HAL_LOG_TRACE(...)   SID_PAL_LOG_TRACE(__VA_ARGS__)
#else
#  define S2LP_RADIO_HAL_LOG_ERROR(...)   ((void)0u)
#  define S2LP_RADIO_HAL_LOG_WARNING(...) ((void)0u)
#  define S2LP_RADIO_HAL_LOG_INFO(...)    ((void)0u)
#  define S2LP_RADIO_HAL_LOG_DEBUG(...)   ((void)0u)
#  define S2LP_RADIO_HAL_LOG_TRACE(...)   ((void)0u)
#endif

#define S2LP_RADIO_HAL_SPI_BUILD_HEADER(__addr_comm__, __w_r_selector__) ((uint8_t)(((__addr_comm__) & ~0x01u) | (__w_r_selector__)))                                                  /*!< macro to build the header byte*/
#define S2LP_RADIO_HAL_SPI_WRITE_HEADER                                  S2LP_RADIO_HAL_SPI_BUILD_HEADER(S2LP_RADIO_HAL_SPI_HEADER_ADDRESS_MASK, S2LP_RADIO_HAL_SPI_HEADER_WRITE_MASK) /*!< macro to build the write header byte*/
#define S2LP_RADIO_HAL_SPI_READ_HEADER                                   S2LP_RADIO_HAL_SPI_BUILD_HEADER(S2LP_RADIO_HAL_SPI_HEADER_ADDRESS_MASK, S2LP_RADIO_HAL_SPI_HEADER_READ_MASK)  /*!< macro to build the read header byte*/
#define S2LP_RADIO_HAL_SPI_COMMAND_HEADER                                S2LP_RADIO_HAL_SPI_BUILD_HEADER(S2LP_RADIO_HAL_SPI_HEADER_COMMAND_MASK, S2LP_RADIO_HAL_SPI_HEADER_WRITE_MASK) /*!< macro to build the command header byte*/

#define S2LP_RADIO_HAL_SMPS_FREQ_TO_KRM(__smps_freq, __fdig__)           ((uint32_t)((((uint64_t)(__smps_freq) << 15) + (uint64_t)((__fdig__) >> 1)) / (uint64_t)(__fdig__)))

/* Private types -------------------------------------------------------------*/

typedef __PACKED_UNION {
    __PACKED_STRUCT {
        sl2_lp_ic_reg_afc_corr_t     afc_corr;
        sl2_lp_ic_reg_link_qualif2_t link_qualif2;
        sl2_lp_ic_reg_link_qualif1_t link_qualif1;
        uint8_t                      __reserved0;
        uint8_t                      rssi_level;
        uint8_t                      __reserved1;
        uint8_t                      rx_pckt_len1;
        uint8_t                      rx_pckt_len0;
        uint8_t                      crc_field3;
        uint8_t                      crc_field2;
        uint8_t                      crc_field1;
        uint8_t                      crc_field0;
    };
    uint8_t raw[12];
} s2_lp_hal_rx_packet_info_regs_t;
static_assert(sizeof(s2_lp_hal_rx_packet_info_regs_t) == 12u * sizeof(uint8_t));

typedef __PACKED_UNION {
    __PACKED_STRUCT {
        sl2_lp_ic_reg_pm_conf3_t pm_conf3;
        sl2_lp_ic_reg_pm_conf2_t pm_conf2;
    };
    uint8_t raw[2];
} s2_lp_hal_smps_krm_regs_t;
static_assert(sizeof(s2_lp_hal_smps_krm_regs_t) == 2u * sizeof(uint8_t));

typedef __PACKED_UNION {
    __PACKED_STRUCT {
        sl2_lp_ic_reg_pm_conf1_t pm_conf1;
        sl2_lp_ic_reg_pm_conf0_t pm_conf0;
    };
    uint8_t raw[2];
} s2_lp_hal_smps_cfg_regs_t;
static_assert(sizeof(s2_lp_hal_smps_cfg_regs_t) == 2u * sizeof(uint8_t));

typedef __PACKED_UNION {
    __PACKED_STRUCT {
        sl2_lp_ic_reg_pa_powerx_t pa_levels[8];
        sl2_lp_ic_reg_pa_power0_t pa_power0;
    };
    uint8_t raw[9];
} s2_lp_hal_pa_power_regs_t;
static_assert(sizeof(s2_lp_hal_pa_power_regs_t) == 9u * sizeof(uint8_t));

typedef struct {
    uint8_t step_cnt;
    uint8_t step_len;
} s2_lp_hal_pa_ramp_cfg_t;

/* Private variables ---------------------------------------------------------*/

SID_STM32_ALIGN_4BYTES(static uint8_t spi_tx_working_buf[S2LP_RADIO_HAL_SPI_WORKING_BUF_SIZE]);
SID_STM32_ALIGN_4BYTES(static uint8_t spi_rx_working_buf[S2LP_RADIO_HAL_SPI_WORKING_BUF_SIZE]);

/* Private constants ---------------------------------------------------------*/

/**
* @brief  Rx channel bandwidth lookup table for 26MHz XTAL. See Table 44 in the datasheet
* @note   The channel bandwidth for others xtal frequencies can be computed since this table
*         multiplying the current table by a factor xtal_frequency/26e6.
*/
static const uint32_t rx_ch_bandwidth_lut_26mhz[10][9] = {
    { 800100u, 795100u, 768400u, 736800u, 705100u, 670900u, 642300u, 586700u, 541400u },
    { 450900u, 425900u, 403200u, 380800u, 362100u, 341700u, 325400u, 294500u, 270300u },
    { 224700u, 212400u, 201500u, 190000u, 180700u, 170600u, 162400u, 147100u, 135000u },
    { 112300u, 106200u, 100500u,  95000u,  90300u,  85300u,  81200u,  73500u,  67500u },
    {  56100u,  53000u,  50200u,  47400u,  45100u,  42600u,  40600u,  36700u,  33700u },
    {  28000u,  26500u,  25100u,  23700u,  22600u,  21300u,  20300u,  18400u,  16900u },
    {  14000u,  13300u,  12600u,  11900u,  11300u,  10600u,  10100u,   9200u,   8400u },
    {   7000u,   6600u,   6300u,   5900u,   5600u,   5300u,   5100u,   4600u,   4200u },
    {   3500u,   3300u,   3100u,   3000u,   2800u,   2700u,   2500u,   2300u,   2100u },
    {   1800u,   1700u,   1600u,   1500u,   1400u,   1300u,   1300u,   1200u,   1100u },
};

static const s2_lp_hal_pa_ramp_cfg_t pa_ramp_lut[S2LP_RADIO_HAL_MAX_PA_RAMP_TICKS_MAX + 1u] = {
    {.step_cnt = 0u, .step_len = 0u,}, /* 0 ticks */
    {.step_cnt = 1u, .step_len = 1u,}, /* 1 tick */
    {.step_cnt = 2u, .step_len = 1u,}, /* 2 ticks */
    {.step_cnt = 3u, .step_len = 1u,}, /* 3 ticks */
    {.step_cnt = 4u, .step_len = 1u,}, /* 4 ticks */
    {.step_cnt = 5u, .step_len = 1u,}, /* 5 ticks */
    {.step_cnt = 6u, .step_len = 1u,}, /* 6 ticks */
    {.step_cnt = 7u, .step_len = 1u,}, /* 7 ticks */
    {.step_cnt = 8u, .step_len = 1u,}, /* 8 ticks */
    {.step_cnt = 3u, .step_len = 3u,}, /* 9 ticks */
    {.step_cnt = 5u, .step_len = 2u,}, /* 10 ticks */
    {.step_cnt = 5u, .step_len = 2u,}, /* 11 ticks */
    {.step_cnt = 6u, .step_len = 2u,}, /* 12 ticks */
    {.step_cnt = 6u, .step_len = 2u,}, /* 13 ticks */
    {.step_cnt = 7u, .step_len = 2u,}, /* 14 ticks */
    {.step_cnt = 5u, .step_len = 3u,}, /* 15 ticks */
    {.step_cnt = 8u, .step_len = 2u,}, /* 16 ticks */
    {.step_cnt = 8u, .step_len = 2u,}, /* 17 ticks */
    {.step_cnt = 6u, .step_len = 3u,}, /* 18 ticks */
    {.step_cnt = 6u, .step_len = 3u,}, /* 19 ticks */
    {.step_cnt = 5u, .step_len = 4u,}, /* 20 ticks */
    {.step_cnt = 7u, .step_len = 3u,}, /* 21 ticks */
    {.step_cnt = 7u, .step_len = 3u,}, /* 22 ticks */
    {.step_cnt = 8u, .step_len = 3u,}, /* 23 ticks */
    {.step_cnt = 8u, .step_len = 3u,}, /* 24 ticks */
    {.step_cnt = 8u, .step_len = 3u,}, /* 25 ticks */
    {.step_cnt = 8u, .step_len = 3u,}, /* 26 ticks */
    {.step_cnt = 7u, .step_len = 4u,}, /* 27 ticks */
    {.step_cnt = 7u, .step_len = 4u,}, /* 28 ticks */
    {.step_cnt = 7u, .step_len = 4u,}, /* 29 ticks */
    {.step_cnt = 7u, .step_len = 4u,}, /* 30 ticks */
    {.step_cnt = 8u, .step_len = 4u,}, /* 31 ticks */
    {.step_cnt = 8u, .step_len = 4u,}, /* 32 ticks */
};

static const struct sid_timespec rco_calib_status_check_period = {
    .tv_sec  = 0u,
    .tv_nsec = S2LP_RADIO_HAL_RCO_CALIBRATION_PROBE_PERIOD_MS * 1000000u,
};

static const struct sid_timespec rco_calib_timeout = {
    .tv_sec  = 0u,
    .tv_nsec = S2LP_RADIO_HAL_RCO_CALIBRATION_TIMEOUT_MS * 1000000u,
};

/* Private function prototypes -----------------------------------------------*/

static inline s2_lp_hal_status_t    _assert_radio_reset(const halo_drv_s2_lp_ctx_t * const drv_ctx);
static inline s2_lp_hal_status_t    _release_radio_reset(const halo_drv_s2_lp_ctx_t * const drv_ctx);
static inline s2_lp_hal_status_t    _spi_write_registers(const halo_drv_s2_lp_ctx_t * const drv_ctx, const uint8_t reg_address, const uint8_t * const data, const uint32_t data_len);
static inline s2_lp_hal_status_t    _spi_read_registers(const halo_drv_s2_lp_ctx_t * const drv_ctx, const uint8_t reg_address, uint8_t * const buffer, const uint32_t read_len);
static inline s2_lp_hal_status_t    _spi_send_command(const halo_drv_s2_lp_ctx_t * const drv_ctx, const uint8_t cmd);
static inline s2_lp_hal_status_t    _wait_for_state_transition(const halo_drv_s2_lp_ctx_t * const drv_ctx, const s2_lp_ic_states_t desired_state, const uint32_t timeout_us);
static inline s2_lp_hal_status_t    _set_intermediate_frequency(const halo_drv_s2_lp_ctx_t * const drv_ctx, const uint32_t im_freq_hz);
static inline void                  _compute_charge_pump_settings(const halo_drv_s2_lp_ctx_t * const drv_ctx, const uint32_t freq_hz, sl2_lp_ic_reg_synt3_t * const synt3_reg, sl2_lp_ic_reg_synth_config2_t * const synth_cfg2_reg);
static inline void                  _compute_synth_pll_steps(const halo_drv_s2_lp_ctx_t * const drv_ctx, const uint32_t freq_hz, s2_lp_ic_reg_synt_t * const synt_regs);
static inline uint32_t              _compute_datarate(const halo_drv_s2_lp_ctx_t * const drv_ctx, const uint32_t exponent, const uint32_t mantissa);
static inline void                  _find_datarate_params(const halo_drv_s2_lp_ctx_t * const drv_ctx, const uint32_t target_datarate, sl2_lp_ic_reg_mod_params_t * const mod_params_regs);
static inline uint32_t              _compute_freq_deviation(const halo_drv_s2_lp_ctx_t * const drv_ctx, const uint32_t exponent, const uint32_t mantissa);
static inline void                  _find_fdev_params(const halo_drv_s2_lp_ctx_t * const drv_ctx, const uint32_t target_fdev, sl2_lp_ic_reg_mod_params_t * const mod_params_regs);
static inline sl2_lp_ic_reg_chflt_t _find_rx_ch_bandwidth_params(const halo_drv_s2_lp_ctx_t * const drv_ctx, const uint32_t desired_bandwidth);
static        void                  _on_radio_irq_detected(uint32_t pin, void * callback_arg);
static inline s2_lp_hal_status_t    _set_smps_krm(const halo_drv_s2_lp_ctx_t * const drv_ctx, const uint32_t krm);
static inline uint32_t              _cdbm_to_pa_level(const int32_t cdBm);
static inline void                  _convert_rssi_reg_to_dbi(const halo_drv_s2_lp_ctx_t * const drv_ctx, const uint8_t rssi_level_reg, s2_lp_hal_rssi_t * const out_rssi);
static inline uint32_t              _get_rco_frequency(const halo_drv_s2_lp_ctx_t * const drv_ctx);
static inline s2_lp_hal_status_t    _initiate_rco_calibration(halo_drv_s2_lp_ctx_t * const drv_ctx, const uint32_t wait_start);
static inline s2_lp_hal_status_t    _store_rco_calibration(halo_drv_s2_lp_ctx_t * const drv_ctx);

/* Private function definitions ----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline s2_lp_hal_status_t _assert_radio_reset(const halo_drv_s2_lp_ctx_t * const drv_ctx)
{
    s2_lp_hal_status_t err;
    sid_error_t        sid_err;

    sid_err = sid_pal_gpio_write(drv_ctx->config->gpio.radio_shutdown, 1u);
    if (sid_err != SID_ERROR_NONE)
    {
        S2LP_RADIO_HAL_LOG_ERROR("sid_pal_gpio_write() failed with error code %d, pin: %u", (int32_t)sid_err, drv_ctx->config->gpio.radio_shutdown);
        err = S2_LP_RADIO_HAL_STATUS_ERROR_HW;
    }
    else
    {
        err = S2_LP_RADIO_HAL_STATUS_OK;
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline s2_lp_hal_status_t _release_radio_reset(const halo_drv_s2_lp_ctx_t * const drv_ctx)
{
    s2_lp_hal_status_t err;
    sid_error_t        sid_err;

    sid_err = sid_pal_gpio_write(drv_ctx->config->gpio.radio_shutdown, 0u);
    if (sid_err != SID_ERROR_NONE)
    {
        S2LP_RADIO_HAL_LOG_ERROR("sid_pal_gpio_write() failed with error code %d, pin: %u", (int32_t)sid_err, drv_ctx->config->gpio.radio_shutdown);
        err = S2_LP_RADIO_HAL_STATUS_ERROR_HW;
    }
    else
    {
        err = S2_LP_RADIO_HAL_STATUS_OK;
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline s2_lp_hal_status_t _spi_write_registers(const halo_drv_s2_lp_ctx_t * const drv_ctx, const uint8_t reg_address, const uint8_t * const data, const uint32_t data_len)
{
    s2_lp_hal_status_t err      = S2_LP_RADIO_HAL_STATUS_ERROR_GENERIC;
    sid_error_t        sid_err;

    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->config != NULL);
    SID_PAL_ASSERT(drv_ctx->config->spi_client_cfg != NULL);
    SID_PAL_ASSERT(data != NULL);
    SID_PAL_ASSERT(sizeof(spi_tx_working_buf) >= (data_len + S2LP_RADIO_HAL_SPI_HEADER_LENGTH));

    const uint32_t sleepy_spi_handling_required = (drv_ctx->regs_cache.mc_state.reg_mc_state0.MC_STATE >= S2_LP_MC_STATE_SLEEP_NOFIFO)
                                                && (drv_ctx->regs_cache.mc_state.reg_mc_state0.MC_STATE <= S2_LP_MC_STATE_SLEEP);

    /* If S2-LP is in one of the low power modes we need an additional delay between driving the NSS pin and starting the SPI transaction */
    if (sleepy_spi_handling_required != FALSE)
    {
        sid_err = sid_pal_serial_bus_ext_ifc_activate_nss(drv_ctx->bus_iface, drv_ctx->config->spi_client_cfg);
        if (sid_err != SID_ERROR_NONE)
        {
            S2LP_RADIO_HAL_LOG_ERROR("S2-LP SPI Write Registers failed. NSS driving error %d", (int32_t)sid_err);
            err = S2_LP_RADIO_HAL_STATUS_ERROR_GPIO;
            return err;
        }

        sid_pal_delay_us(S2LP_RADIO_HAL_SPI_NSS_DELAY_IN_LPM_US);
    }

    do
    {
        /* Build transaction header */
        spi_tx_working_buf[0] = S2LP_RADIO_HAL_SPI_WRITE_HEADER;
        spi_tx_working_buf[1] = (uint8_t)reg_address;

        /* append user data to the Tx buffer */
        SID_STM32_UTIL_fast_memcpy(&spi_tx_working_buf[S2LP_RADIO_HAL_SPI_HEADER_LENGTH], data, data_len);


        sid_err = drv_ctx->bus_iface->xfer(drv_ctx->bus_iface, drv_ctx->config->spi_client_cfg, spi_tx_working_buf, spi_rx_working_buf, (data_len + S2LP_RADIO_HAL_SPI_HEADER_LENGTH));
        if (sid_err != SID_ERROR_NONE)
        {
            S2LP_RADIO_HAL_LOG_ERROR("S2-LP SPI Write Registers failed. sid_pal_serial_bus_spi error %d", (int32_t)sid_err);
            err = S2_LP_RADIO_HAL_STATUS_ERROR_SPI_XFER;
            break;
        }

        /* Extract device status info */
        s2_lp_ic_status_t * const mc_state_ptr = (s2_lp_ic_status_t *)(&drv_ctx->regs_cache.mc_state);
        mc_state_ptr->raw[0] = spi_rx_working_buf[0];
        mc_state_ptr->raw[1] = spi_rx_working_buf[1];

        err = S2_LP_RADIO_HAL_STATUS_OK;
    } while (0);

    if (sleepy_spi_handling_required != FALSE)
    {
        sid_err = sid_pal_serial_bus_ext_ifc_deactivate_nss(drv_ctx->bus_iface, drv_ctx->config->spi_client_cfg);
        if (sid_err != SID_ERROR_NONE)
        {
            S2LP_RADIO_HAL_LOG_ERROR("S2-LP SPI Write Registers failed. NSS release error %d", (int32_t)sid_err);
            err = S2_LP_RADIO_HAL_STATUS_ERROR_GPIO;
        }
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline s2_lp_hal_status_t _spi_read_registers(const halo_drv_s2_lp_ctx_t * const drv_ctx, const uint8_t reg_address, uint8_t * const buffer, const uint32_t read_len)
{
    s2_lp_hal_status_t err      = S2_LP_RADIO_HAL_STATUS_ERROR_GENERIC;
    sid_error_t        sid_err;

    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->config != NULL);
    SID_PAL_ASSERT(drv_ctx->config->spi_client_cfg != NULL);
    SID_PAL_ASSERT(buffer != NULL);
    SID_PAL_ASSERT(sizeof(spi_rx_working_buf) >= (read_len + S2LP_RADIO_HAL_SPI_HEADER_LENGTH));

    const uint32_t sleepy_spi_handling_required = (drv_ctx->regs_cache.mc_state.reg_mc_state0.MC_STATE >= S2_LP_MC_STATE_SLEEP_NOFIFO)
                                                && (drv_ctx->regs_cache.mc_state.reg_mc_state0.MC_STATE <= S2_LP_MC_STATE_SLEEP);

    /* If S2-LP is in one of the low power modes we need an additional delay between driving the NSS pin and starting the SPI transaction */
    if (sleepy_spi_handling_required != FALSE)
    {
        sid_err = sid_pal_serial_bus_ext_ifc_activate_nss(drv_ctx->bus_iface, drv_ctx->config->spi_client_cfg);
        if (sid_err != SID_ERROR_NONE)
        {
            S2LP_RADIO_HAL_LOG_ERROR("S2-LP SPI Read Registers failed. NSS driving error %d", (int32_t)sid_err);
            err = S2_LP_RADIO_HAL_STATUS_ERROR_GPIO;
            return err;
        }

        sid_pal_delay_us(S2LP_RADIO_HAL_SPI_NSS_DELAY_IN_LPM_US);
    }

    do
    {
        /* Build transaction header */
        spi_tx_working_buf[0] = S2LP_RADIO_HAL_SPI_READ_HEADER;
        spi_tx_working_buf[1] = (uint8_t)reg_address;

        sid_err = drv_ctx->bus_iface->xfer(drv_ctx->bus_iface, drv_ctx->config->spi_client_cfg, spi_tx_working_buf, spi_rx_working_buf, (read_len + S2LP_RADIO_HAL_SPI_HEADER_LENGTH));
        if (sid_err != SID_ERROR_NONE)
        {
            S2LP_RADIO_HAL_LOG_ERROR("S2-LP SPI Read Registers failed. sid_pal_serial_bus_spi error %d", (int32_t)sid_err);
            err = S2_LP_RADIO_HAL_STATUS_ERROR_SPI_XFER;
            break;
        }

        /* Extract device status info */
        s2_lp_ic_status_t * const mc_state_ptr = (s2_lp_ic_status_t *)(&drv_ctx->regs_cache.mc_state);
        mc_state_ptr->raw[0] = spi_rx_working_buf[0];
        mc_state_ptr->raw[1] = spi_rx_working_buf[1];

        /* Copy received data except the status bytes */
        SID_STM32_UTIL_fast_memcpy(buffer, &spi_rx_working_buf[S2LP_RADIO_HAL_SPI_HEADER_LENGTH], read_len);

        err = S2_LP_RADIO_HAL_STATUS_OK;
    } while (0);

    if (sleepy_spi_handling_required != FALSE)
    {
        sid_err = sid_pal_serial_bus_ext_ifc_deactivate_nss(drv_ctx->bus_iface, drv_ctx->config->spi_client_cfg);
        if (sid_err != SID_ERROR_NONE)
        {
            S2LP_RADIO_HAL_LOG_ERROR("S2-LP SPI Write Registers failed. NSS release error %d", (int32_t)sid_err);
            err = S2_LP_RADIO_HAL_STATUS_ERROR_GPIO;
        }
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline s2_lp_hal_status_t _spi_send_command(const halo_drv_s2_lp_ctx_t * const drv_ctx, const uint8_t cmd)
{
    s2_lp_hal_status_t err      = S2_LP_RADIO_HAL_STATUS_ERROR_GENERIC;
    sid_error_t        sid_err;

    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->config != NULL);
    SID_PAL_ASSERT(drv_ctx->config->spi_client_cfg != NULL);
    SID_PAL_ASSERT(sizeof(spi_tx_working_buf) >= (S2LP_RADIO_HAL_SPI_HEADER_LENGTH));

    const uint32_t sleepy_spi_handling_required = (drv_ctx->regs_cache.mc_state.reg_mc_state0.MC_STATE >= S2_LP_MC_STATE_SLEEP_NOFIFO)
                                                && (drv_ctx->regs_cache.mc_state.reg_mc_state0.MC_STATE <= S2_LP_MC_STATE_SLEEP);

    /* If S2-LP is in one of the low power modes we need an additional delay between driving the NSS pin and starting the SPI transaction */
    if (sleepy_spi_handling_required != FALSE)
    {
        sid_err = sid_pal_serial_bus_ext_ifc_activate_nss(drv_ctx->bus_iface, drv_ctx->config->spi_client_cfg);
        if (sid_err != SID_ERROR_NONE)
        {
            S2LP_RADIO_HAL_LOG_ERROR("S2-LP SPI Send Command failed. NSS driving error %d", (int32_t)sid_err);
            err = S2_LP_RADIO_HAL_STATUS_ERROR_GPIO;
            return err;
        }

        sid_pal_delay_us(S2LP_RADIO_HAL_SPI_NSS_DELAY_IN_LPM_US);
    }

    do
    {
        /* Build transaction header */
        spi_tx_working_buf[0] = S2LP_RADIO_HAL_SPI_COMMAND_HEADER;
        spi_tx_working_buf[1] = cmd;

        sid_err = drv_ctx->bus_iface->xfer(drv_ctx->bus_iface, drv_ctx->config->spi_client_cfg, spi_tx_working_buf, spi_rx_working_buf, S2LP_RADIO_HAL_SPI_HEADER_LENGTH);
        if (sid_err != SID_ERROR_NONE)
        {
            S2LP_RADIO_HAL_LOG_ERROR("S2-LP SPI Send Command failed. sid_pal_serial_bus_spi error %d", (int32_t)sid_err);
            err = S2_LP_RADIO_HAL_STATUS_ERROR_SPI_XFER;
            break;
        }

        /* Extract device status info */
        s2_lp_ic_status_t * const mc_state_ptr = (s2_lp_ic_status_t *)(&drv_ctx->regs_cache.mc_state);
        mc_state_ptr->raw[0] = spi_rx_working_buf[0];
        mc_state_ptr->raw[1] = spi_rx_working_buf[1];

        err = S2_LP_RADIO_HAL_STATUS_OK;
    } while (0);

    if (sleepy_spi_handling_required != FALSE)
    {
        sid_err = sid_pal_serial_bus_ext_ifc_deactivate_nss(drv_ctx->bus_iface, drv_ctx->config->spi_client_cfg);
        if (sid_err != SID_ERROR_NONE)
        {
            S2LP_RADIO_HAL_LOG_ERROR("S2-LP SPI Write Registers failed. NSS release error %d", (int32_t)sid_err);
            err = S2_LP_RADIO_HAL_STATUS_ERROR_GPIO;
        }
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline s2_lp_hal_status_t _wait_for_state_transition(const halo_drv_s2_lp_ctx_t * const drv_ctx, const s2_lp_ic_states_t desired_state, const uint32_t timeout_us)
{
    s2_lp_hal_status_t err                   = S2_LP_RADIO_HAL_STATUS_ERROR_GENERIC;
    s2_lp_ic_status_t  status_from_regs;
    uint32_t           accumulated_wait_time = 0u;

    do
    {
        /* Read out the status from MC_STATE registers and simultaneously store the status reported while sending header bytes */
        err = _spi_read_registers(drv_ctx, S2_LP_IC_REG_ADDR_MC_STATE1, status_from_regs.raw, sizeof(status_from_regs));
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_HAL_LOG_ERROR("_wait_for_state_transition - unable to retrieve S2-LP status");
            break;
        }

        if ((desired_state == status_from_regs.reg_mc_state0.MC_STATE) && (desired_state == drv_ctx->regs_cache.mc_state.reg_mc_state0.MC_STATE))
        {
            /* For MC_STATE_READY also wait for XO_ON bit to be set */
            if ((desired_state != S2_LP_MC_STATE_READY) || (status_from_regs.reg_mc_state0.XO_ON != FALSE))
            {
                /* Desired state reached */
                err = S2_LP_RADIO_HAL_STATUS_OK;
                break;
            }
        }

        sid_pal_delay_us(S2LP_RADIO_HAL_STATE_TRANSITION_PROBE_PERIOD_US);
        accumulated_wait_time += S2LP_RADIO_HAL_STATE_TRANSITION_PROBE_PERIOD_US;
    } while (accumulated_wait_time < timeout_us);

    if (accumulated_wait_time >= timeout_us)
    {
        err = S2_LP_RADIO_HAL_STATUS_ERROR_TIMEOUT;
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline s2_lp_hal_status_t _set_intermediate_frequency(const halo_drv_s2_lp_ctx_t * const drv_ctx, const uint32_t im_freq_hz)
{
    s2_lp_hal_status_t err;

    /* Calculate pre-divided values to fit into uint32_t. Both numerator and denominator in IF formula can be divided by 2^6 since XOSC frequency is exactly 24, 25, 26, 48, 50, or 52 MHz */
    const uint32_t pow2_prediv = 6u;
    const uint32_t pow2_multiplier = (13u - pow2_prediv);

    uint32_t dig_domain_freq_predivided = drv_ctx->config->xin_freq < S2_LP_IC_DIG_DOMAIN_XTAL_THRESH ? (drv_ctx->config->xin_freq >> pow2_prediv) : (drv_ctx->config->xin_freq >> (pow2_prediv + 1u));
    uint32_t ana_domain_freq_predivided = drv_ctx->config->xin_freq >> pow2_prediv;

    /* Calculate IF values as per the datasheet with arithmetical rounding */
    uint8_t write_buf[2] = {
        (uint8_t)((((im_freq_hz << pow2_multiplier) * 3u + (ana_domain_freq_predivided >> 1))/ ana_domain_freq_predivided) - 100u),
        (uint8_t)((((im_freq_hz << pow2_multiplier) * 3u + (dig_domain_freq_predivided >> 1))/ dig_domain_freq_predivided) - 100u),
    };

    /* Write both IF_OFFSET_ANA and IF_OFFSET_DIG registers at once */
    err = _spi_write_registers(drv_ctx, S2_LP_IC_REG_ADDR_IF_OFFSET_ANA, write_buf, sizeof(write_buf));

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline void _compute_charge_pump_settings(const halo_drv_s2_lp_ctx_t * const drv_ctx, const uint32_t freq_hz, sl2_lp_ic_reg_synt3_t * const synt3_reg, sl2_lp_ic_reg_synth_config2_t * const synth_cfg2_reg)
{
    /* Get the reference frequency clock */
    const uint32_t ref_freq = drv_ctx->ref_freq_hz;

    /* Determine the operating band */
    const uint32_t b_factor = FALSE == S2_LP_IC_IS_FREQ_BAND_HIGH(freq_hz) ? S2_LP_IC_MIDDLE_BAND_FACTOR : S2_LP_IC_HIGH_BAND_FACTOR;

    /* Calculate the VCO frequency (VCOFreq = freq_hz * b_factor) */
    const uint32_t vcofreq = freq_hz * b_factor;

    /* Set the correct charge pump word - see Table 37 in the datasheet*/
    if (vcofreq >= S2_LP_IC_VCO_CENTER_FREQ)
    {
        if (ref_freq >= S2_LP_IC_REF_FREQ_THRESH)
        {
            /* ICP = 120uA */
            synt3_reg->PLL_CP_ISEL           = 0x02u;
            synth_cfg2_reg->PLL_PFD_SPLIT_EN = 0u;
        }
        else
        {
            /* ICP = 200uA */
            synt3_reg->PLL_CP_ISEL           = 0x01u;
            synth_cfg2_reg->PLL_PFD_SPLIT_EN = 1u;
        }
    }
    else
    {
        if (ref_freq >= S2_LP_IC_REF_FREQ_THRESH)
        {
            /* ICP = 140uA */
            synt3_reg->PLL_CP_ISEL           = 0x03u;
            synth_cfg2_reg->PLL_PFD_SPLIT_EN = 0u;
        }
        else
        {
            /* ICP = 240uA */
           synt3_reg->PLL_CP_ISEL           = 0x02u;
           synth_cfg2_reg->PLL_PFD_SPLIT_EN = 1u;
        }
    }
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline void _compute_synth_pll_steps(const halo_drv_s2_lp_ctx_t * const drv_ctx, const uint32_t freq_hz, s2_lp_ic_reg_synt_t * const synt_regs)
{
    /* Get the reference frequency clock */
    const uint32_t ref_freq = drv_ctx->ref_freq_hz;

    /* Determine the operating band */
    uint32_t base_divider_pow2 = 20u; /* 2^20 as per formula 7, section 5.3.1 */
    if (FALSE == S2_LP_IC_IS_FREQ_BAND_HIGH(freq_hz))
    {
        /* Translate B/2 into pow2 exponent */
        base_divider_pow2 += (SID_STM32_UTIL_ROUNDUP_NEXT_POW_2_EXP(S2_LP_IC_MIDDLE_BAND_FACTOR) - 1u);
        synt_regs->synt3.BS = 1u;
    }
    else
    {
        /* Translate B/2 into pow2 exponent */
        base_divider_pow2 += (SID_STM32_UTIL_ROUNDUP_NEXT_POW_2_EXP(S2_LP_IC_HIGH_BAND_FACTOR) - 1u);
        synt_regs->synt3.BS = 0u;
    }

    const uint32_t pow2_adjustment = 6u; /*!< It is safe to divide all parts of the equation by this value since reference frequency is exactly 24Mhz, 25MHz, or 26MHz */
    base_divider_pow2 -= pow2_adjustment;
    const uint32_t adjusted_ref_freq = ref_freq >> pow2_adjustment;

    /* Now calculated the desired value in PLL steps */
    const uint32_t pll_steps = (uint32_t)((((uint64_t)freq_hz * ((uint64_t)1u << base_divider_pow2)) + (uint64_t)((adjusted_ref_freq + 1u) >> 1)/* arithmetical rounding */) / (uint64_t)adjusted_ref_freq);

    /* Store the value */
    synt_regs->synt3.SYNT_27_24 = (uint8_t)(pll_steps >> 24);
    synt_regs->synt2.SYNT_23_16 = (uint8_t)(pll_steps >> 16);
    synt_regs->synt1.SYNT_15_8  = (uint8_t)(pll_steps >> 8);
    synt_regs->synt0.SYNT_7_0   = (uint8_t)(pll_steps);
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline uint32_t _compute_datarate(const halo_drv_s2_lp_ctx_t * const drv_ctx, const uint32_t exponent, const uint32_t mantissa)
{
    uint32_t datarate_bps;

    SID_PAL_ASSERT(exponent <= S2_LP_IC_DATARATE_EXPONENT_MAX);
    SID_PAL_ASSERT(mantissa <= S2_LP_IC_DATARATE_MANTISSA_MAX);

    /* Calculate according to the Formula 14 in the datasheet */
    if (0u == exponent)
    {
        /* Case when exponent is zero */
        datarate_bps = (uint32_t)(((uint64_t)((uint64_t)drv_ctx->dig_freq_hz * (uint64_t)mantissa) + (uint64_t)(1u << 31)) >> 32);
    }
    else if (S2_LP_IC_DATARATE_EXPONENT_MAX == exponent)
    {
        /* Special case for exponent == 15 */
        datarate_bps = ((drv_ctx->dig_freq_hz >> 3) + (mantissa >> 1)) / mantissa;
    }
    else
    {
        /* For non-zero exponent */
        const uint32_t fdig_multiplier = 65536u + mantissa;
        const uint32_t fdig_pow2_div   = 33u - exponent;

        datarate_bps = (uint32_t)((uint64_t)(((uint64_t)drv_ctx->dig_freq_hz * (uint64_t)fdig_multiplier) + (uint64_t)(fdig_pow2_div - 1u)) >> fdig_pow2_div);
    }

    return datarate_bps;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline void _find_datarate_params(const halo_drv_s2_lp_ctx_t * const drv_ctx, const uint32_t target_datarate, sl2_lp_ic_reg_mod_params_t * const mod_params_regs)
{
    uint32_t datarate_tmp;
    uint32_t exponent;
    uint32_t mantissa;
    uint32_t lower_val;
    uint32_t upper_val;

    /* Use binary search to determine the exponent - search for the closest value */
    lower_val = 0u;
    upper_val = S2_LP_IC_DATARATE_EXPONENT_MAX;
    exponent = (upper_val + lower_val) >> 1;

    /* Compute the datarate with the current exponent value and the maximum mantissa value */
    datarate_tmp = _compute_datarate(drv_ctx, exponent, S2_LP_IC_DATARATE_MANTISSA_MAX);

    while (lower_val != upper_val)
    {
        if (datarate_tmp < target_datarate)
        {
            lower_val = exponent + 1u;
        }
        else
        {
            upper_val = exponent - 1u;
        }

        /* Update the values */
        exponent = (upper_val + lower_val) >> 1;
        datarate_tmp = _compute_datarate(drv_ctx, exponent, S2_LP_IC_DATARATE_MANTISSA_MAX);
    }

    /* We need the exponent value that gives datarate higher than the desired (since we have used maximized mantissa for the search) */
    if ((datarate_tmp < target_datarate) && (exponent < S2_LP_IC_DATARATE_EXPONENT_MAX))
    {
        exponent++;
    }

    /* Now the exponent is determined and it is guaranteed that we can match the target datarate with the mantissa in 0..65535 range */

    /* Calculate the mantissa that gives the closest datarate to the desired one - according to the Formula 14 in the datasheet */
    if (0u == exponent)
    {
        /* Case when exponent is zero */
        mantissa = (uint32_t)((uint64_t)(((uint64_t)target_datarate << 32) + (uint64_t)(drv_ctx->dig_freq_hz >> 1)) / (uint64_t)drv_ctx->dig_freq_hz);
    }
    else if (S2_LP_IC_DATARATE_EXPONENT_MAX == exponent)
    {
        /* Special case for exponent == 15 */
        mantissa = ((drv_ctx->dig_freq_hz >> 3) + (target_datarate >> 1)) / target_datarate;
    }
    else
    {
        /* For non-zero exponent */
        mantissa = (uint32_t)((uint32_t)((uint64_t)(((uint64_t)target_datarate << (33u - exponent)) + (uint64_t)(drv_ctx->dig_freq_hz >> 1)) / (uint64_t)drv_ctx->dig_freq_hz) - (uint32_t)65536u);
    }

    mod_params_regs->mod2.DATARATE_E      = (uint8_t) exponent;
    mod_params_regs->mod4.DATARATE_M_15_8 = (uint8_t)(mantissa >> 8);
    mod_params_regs->mod3.DATARATE_M_7_0  = (uint8_t) mantissa;

    S2LP_RADIO_HAL_LOG_DEBUG("Target datarate %ubps: E=%u, M=%u, actual datarate=%u", target_datarate, exponent, mantissa, _compute_datarate(drv_ctx, exponent, mantissa));
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline uint32_t _compute_freq_deviation(const halo_drv_s2_lp_ctx_t * const drv_ctx, const uint32_t exponent, const uint32_t mantissa)
{
    uint32_t fdev_hz;

    SID_PAL_ASSERT(exponent <= S2_LP_IC_FDEV_EXPONENT_MAX);
    SID_PAL_ASSERT(mantissa <= S2_LP_IC_FDEV_MANTISSA_MAX);

    /* Calculate according to the Formula 10 in the datasheet */
    if (0u == exponent)
    {
        /* Case when exponent is zero */
        fdev_hz = (uint32_t)((uint64_t)((uint64_t)((uint64_t)drv_ctx->config->xin_freq * (uint64_t)mantissa) + (uint64_t)(1u << 21) ) >> 22);
    }
    else
    {
        /* For non-zero exponent */
        const uint32_t fxo_multiplier = 256u + mantissa;
        const uint32_t fxo_pow2_div   = 23u - exponent;

        fdev_hz = (uint32_t)((uint64_t)((uint64_t)((uint64_t)drv_ctx->config->xin_freq * (uint64_t)fxo_multiplier) + (uint64_t)(1u << (fxo_pow2_div - 1u))) >> (23u - exponent));
    }

    return fdev_hz;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline void _find_fdev_params(const halo_drv_s2_lp_ctx_t * const drv_ctx, const uint32_t target_fdev, sl2_lp_ic_reg_mod_params_t * const mod_params_regs)
{
    uint32_t fdev_tmp;
    uint32_t exponent;
    uint32_t mantissa;
    uint32_t lower_val;
    uint32_t upper_val;

    /* Use binary search to determine the exponent - search for the closest value */
    lower_val = 0u;
    upper_val = S2_LP_IC_FDEV_EXPONENT_MAX;
    exponent = (upper_val + lower_val) >> 1;

    /* Compute the frequency deviation with the current exponent value and the maximum mantissa value */
    fdev_tmp = _compute_freq_deviation(drv_ctx, exponent, S2_LP_IC_FDEV_MANTISSA_MAX);

    while (lower_val != upper_val)
    {
        if (fdev_tmp < target_fdev)
        {
            lower_val = exponent + 1u;
        }
        else
        {
            upper_val = exponent - 1u;
        }

        /* Update the values */
        exponent = (upper_val + lower_val) >> 1;
        fdev_tmp = _compute_freq_deviation(drv_ctx, exponent, S2_LP_IC_FDEV_MANTISSA_MAX);
    }

    /* We need the exponent value that gives Fdev higher than the desired (since we have used maximized mantissa for the search) */
    if ((fdev_tmp < target_fdev) && (exponent < S2_LP_IC_FDEV_EXPONENT_MAX))
    {
        exponent++;
    }

    /* Now the exponent is determined and it is guaranteed that we can match the target Fdev with the mantissa in 0..255 range */

    /* Calculate the mantissa that gives the closest Fdev to the desired one - according to the Formula 10 in the datasheet */
    if (0u == exponent)
    {
        /* Case when exponent is zero */
        mantissa = (uint32_t)((uint64_t)((uint64_t)((uint64_t)target_fdev << 22) + (uint64_t)(drv_ctx->config->xin_freq >> 1)) / (uint64_t)drv_ctx->config->xin_freq);
    }
    else
    {
        /* For non-zero exponent */
        mantissa = (uint32_t)((uint32_t)((uint64_t)((uint64_t)((uint64_t)target_fdev << (23u - exponent)) + (uint64_t)(drv_ctx->config->xin_freq >> 1)) / (uint64_t)drv_ctx->config->xin_freq) - (uint32_t)256u);
    }

    mod_params_regs->mod1.FDEV_E = (uint8_t)exponent;
    mod_params_regs->mod0.FDEV_M = (uint8_t)mantissa;

    S2LP_RADIO_HAL_LOG_DEBUG("Target Fdev %uHz: E=%u, M=%u, actual Fdev=%u", target_fdev, exponent, mantissa, _compute_freq_deviation(drv_ctx, exponent, mantissa));
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline sl2_lp_ic_reg_chflt_t _find_rx_ch_bandwidth_params(const halo_drv_s2_lp_ctx_t * const drv_ctx, const uint32_t desired_bandwidth)
{
    sl2_lp_ic_reg_chflt_t out_reg;
    uint32_t target_bandwidth;
    uint32_t exponent_idx;
    uint32_t mantissa_idx;
    uint32_t lower_idx;
    uint32_t upper_idx;

    if (drv_ctx->dig_freq_mhz != S2LP_RADIO_HAL_RX_CH_BW_LUT_REF_FREQ_MHZ)
    {
        /* LUT was defined for 26MHz digital domain clock frequency but current implementation uses another XTAL.
         * As per the datasheet, LUT values shall be adjusted by (Fdig/26MHz) factor, but instead of adjusting the
         * entire LUT we can adjust the desired bandwidth by the (26MHz/Fdig) factor
         */
        target_bandwidth = ((S2LP_RADIO_HAL_RX_CH_BW_LUT_REF_FREQ_MHZ * desired_bandwidth) + (drv_ctx->dig_freq_mhz - 1u)) / drv_ctx->dig_freq_mhz; /* Round up since selected filter bandwidth shall be not less than the desired one */
    }
    else
    {
        target_bandwidth = desired_bandwidth;
    }

    /* Perform binary search for the exponent part */
    lower_idx = 0u;
    upper_idx = SID_STM32_UTIL_ARRAY_SIZE(rx_ch_bandwidth_lut_26mhz) - 1u;
    exponent_idx = (upper_idx + lower_idx) >> 1;

    while (lower_idx < upper_idx)
    {
        if (target_bandwidth < rx_ch_bandwidth_lut_26mhz[exponent_idx][0])
        {
            lower_idx = exponent_idx + 1u;
        }
        else /* Skip checking for equality since perfect matches are virtually impossible, assume target_bandwidth can be only > or < than LUT value */
        {
            upper_idx = exponent_idx - 1u;
        }

        exponent_idx = (upper_idx + lower_idx) >> 1;
    }

    /* Binary search may end up at the index that is lower band than the desired bandwidth since there's no perfect match between the LUT values and the desired frequency bandwidth */
    if ((target_bandwidth > rx_ch_bandwidth_lut_26mhz[exponent_idx][0]) && (exponent_idx > 0u))
    {
        exponent_idx--;
    }

    /* Perform the binary search for the mantissa part */
    lower_idx = 0u;
    upper_idx = SID_STM32_UTIL_ARRAY_SIZE(rx_ch_bandwidth_lut_26mhz[exponent_idx]) - 1u;
    mantissa_idx = (upper_idx + lower_idx) >> 1;

    while (lower_idx < upper_idx)
    {
        if (target_bandwidth < rx_ch_bandwidth_lut_26mhz[exponent_idx][mantissa_idx])
        {
            lower_idx = mantissa_idx + 1u;
        }
        else /* Skip checking for equality since perfect matches are virtually impossible, assume target_bandwidth can be only > or < than LUT value */
        {
            upper_idx = mantissa_idx - 1u;
        }

        mantissa_idx = (upper_idx + lower_idx) >> 1;
    }

    /* Binary search may end up at the index that is lower band than the desired bandwidth since there's no perfect match between the LUT values and the desired frequency bandwidth */
    if ((target_bandwidth > rx_ch_bandwidth_lut_26mhz[exponent_idx][mantissa_idx]) && (mantissa_idx > 0u))
    {
        mantissa_idx--;
    }

    S2LP_RADIO_HAL_LOG_DEBUG("Target BW %uHz: E=%u, M=%u, actual BW=%uHz", desired_bandwidth, exponent_idx, mantissa_idx,
                             (((drv_ctx->dig_freq_mhz * rx_ch_bandwidth_lut_26mhz[exponent_idx][mantissa_idx]) + (S2LP_RADIO_HAL_RX_CH_BW_LUT_REF_FREQ_MHZ >> 1)) / S2LP_RADIO_HAL_RX_CH_BW_LUT_REF_FREQ_MHZ));
    out_reg.CHFLT_E = (uint8_t)exponent_idx;
    out_reg.CHFLT_M = (uint8_t)mantissa_idx;

    return out_reg;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static void _on_radio_irq_detected(uint32_t pin, void * callback_arg)
{
    struct sid_timespec event_ts;

    /* Store the timestamp as soon as possible */
    (void)sid_pal_uptime_now(&event_ts);
    __COMPILER_BARRIER();

    (void)pin;

    halo_drv_s2_lp_ctx_t * const drv_ctx = (halo_drv_s2_lp_ctx_t *)callback_arg;
    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->radio_rx_packet != NULL);

    S2LP_RADIO_HAL_LOG_DEBUG("_on_radio_irq_detected...");

    /**
     * The time-sensitive part is completed, now we can lower Radio IRQ priority to allow other time-critical IRQs to be processed and to enable RTOS API calls below this point
     *
     * WARNING: Dynamic IRQ priority updates within the IRQ handler is supported starting from ARMv7-M architecture (Cortex-M3). For ARMv6-M (Cortex-M0/M0+/M1) priority shall
     *          be changed only when the corresponding IRQ is not active. Due to that radio_irq.prio in the radio driver configuration shall be set to a level that allows RTOS
     *          API calls (e.g. lower (meaning higher number) or equal to configMAX_SYSCALL_INTERRUPT_PRIORITY for FreeRTOS)
     */
#if (__ARM_ARCH_6M__ == 0)
    if (drv_ctx->radio_is_running_high_prio != FALSE)
    {
        /* Store the capture time stamp if this the ISR is in the hard realtime mode */
        drv_ctx->radio_rx_packet->rcv_tm = event_ts;

        /* Lower the IRQ priority */
        (void)sid_pal_gpio_ext_ifc_set_irq_priority(drv_ctx->config->gpio.radio_irq.mcu_pin, drv_ctx->config->gpio.radio_irq.prio_low, 0u);
        drv_ctx->radio_is_running_high_prio = FALSE;

        /**
         * Jump out of the ISR to allow NVIC to re-evaluate the priorities
         *
         * WARNING: This is a mandatory step after lower the IRQ priority to enable RTOS API calls. Just lowering the priority is not enough because if this ISR was entered
         *          while the respective priority was high, it will continue to run even after lowering the priority because NVIC does not re-evaluate the priority for an
         *          ISR that is already running. This means proceeding with the execution after lowering the priority still may interfere with the RTOS kernel and ruin the
         *          scheduling, resulting in undefined behavior. To overcome this limitation, the ISR must terminate from here. Since the respective GPIO IRQ is configured to
         *          be triggered by the level on the pin, not the edge, the NVIC will take the IRQ back immediately, but it will now re-evaluate the IRQ priority and call the
         *          ISR only when it is safe to do so from RTOS kernel perspective.
         */
        return;
    }
#else
    /* Just store the capture time stamp and proceed as IRQ priority change is not an option here */
    drv_ctx->radio_rx_packet->rcv_tm = event_ts;
#endif /* __ARM_ARCH_6M__ */

    /* Disable radio IRQ line  to allow the MCU's IRQ handler to return - Sidewalk may process radio IRQ in a RTOS task context */
    s2_lp_hal_status_t err = s2_lp_radio_hal_disarm_irq(drv_ctx);
    if (err != S2_LP_RADIO_HAL_STATUS_OK)
    {
        SID_PAL_LOG_ERROR("Failed to disarm S2-LP IRQ line. error %u", (uint32_t)err);
    }

    /* Route IRQ to Sidewalk stack */
    drv_ctx->radio_irq_handler();
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline s2_lp_hal_status_t _set_smps_krm(const halo_drv_s2_lp_ctx_t * const drv_ctx, const uint32_t krm)
{
    s2_lp_hal_status_t err;
    s2_lp_hal_smps_krm_regs_t krm_regs = {
        .pm_conf3 = {
            .KRM_EN   = 1u,
            .KRM_14_8 = (uint8_t)(krm >> 8),
        },
        .pm_conf2 = {
            .KRM_7_0  = (uint8_t)krm,
        },
    };

    err = _spi_write_registers(drv_ctx, S2_LP_IC_REG_ADDR_PM_CONF3, krm_regs.raw, sizeof(krm_regs.raw));
    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline uint32_t _cdbm_to_pa_level(const int32_t cdBm)
{
    uint32_t pa_level;

    if (cdBm > (S2_LP_IC_MAX_PA_POWER_DBM * 100))
    {
        /* Saturate to maximum output power */
        pa_level = 1u;
    }
    else if (cdBm < (S2_LP_IC_MIN_PA_POWER_DBM * 100))
    {
        /* Keep PA tristated if the desired output power is below the minimum */
        pa_level = 0u;
    }
    else
    {
        /* Requested power level is within the limits, recalculated to the PA Level register value */
        pa_level = (((uint32_t)((int32_t)(S2_LP_IC_MAX_PA_POWER_DBM * 100) - cdBm) + 25u) / 50u) + 1u; /* PA_LEVEL resolution is 0.5dBm, 1u corresponds to +14dBm, 3u corresponds to +13dBm, etc. */
    }

    return pa_level;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline void _convert_rssi_reg_to_dbi(const halo_drv_s2_lp_ctx_t * const drv_ctx, const uint8_t rssi_level_reg, s2_lp_hal_rssi_t * const out_rssi)
{
    /* Convert raw register value to the Sidewalk's internal representation (physical value multiplied by 100) */
    int32_t tmp_rssi = S2LP_RADIO_HAL_LVL_TO_DBM(rssi_level_reg) * 100;

    /* Store immediate RSSI reading */
    out_rssi->immediate_rssi = tmp_rssi;

    /* Compute adjusted RSSI level */
    tmp_rssi -= drv_ctx->regional_radio_params->ant_dbi; /* Adjust RSSI by antenna and LNA gain (if present) */
#if S2LP_RADIO_CFG_USE_EXTERNAL_PA
    tmp_rssi -= drv_ctx->config->pa_config.rx_gain_dbi; /* Exclude LNA gain */
#else
    tmp_rssi += drv_ctx->config->pa_config.rf_sw_insertion_loss; /* Discard losses introduced by the RF switch */
#endif /* S2LP_RADIO_CFG_USE_EXTERNAL_PA */
    out_rssi->adjusted_rssi = tmp_rssi >= 0 ? (tmp_rssi + 50) / 100 : (tmp_rssi - 50) / 100;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline uint32_t _get_rco_frequency(const halo_drv_s2_lp_ctx_t * const drv_ctx)
{
    uint32_t rco_freq;

    switch(drv_ctx->config->xin_freq)
    {
        case 24000000u:
        case 48000000u:
            rco_freq = 32000u;
            break;

        case 25000000u:
        case 50000000u:
            rco_freq = 33300u;
            break;

        case 26000000u:
        case 52000000u:
            rco_freq = 34700u;
            break;

        default:
            rco_freq = 0u;
            SID_PAL_LOG_ERROR("Invalid S2-LP XTAL frequency: %u", drv_ctx->config->xin_freq);
            SID_PAL_ASSERT(0);
            break;
    }

    return rco_freq;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline s2_lp_hal_status_t _initiate_rco_calibration(halo_drv_s2_lp_ctx_t * const drv_ctx, const uint32_t wait_start)
{
    s2_lp_hal_status_t err = S2_LP_RADIO_HAL_STATUS_ERROR_GENERIC;

    SID_PAL_ASSERT(drv_ctx != NULL);

    do
    {
        sl2_lp_ic_reg_xo_rco_conf0_t xo_rco_conf0 = drv_ctx->regs_cache.xo_rco_conf0_reg;
        sl2_lp_ic_reg_mc_state1_t    mc_state1;

        /* 1. Initiate the calibration */
        xo_rco_conf0.RCO_CALIBRATION = 1u;
        err = _spi_write_registers(drv_ctx, S2_LP_IC_REG_ADDR_XO_RCO_CONF0, &xo_rco_conf0.raw, sizeof(xo_rco_conf0.raw));
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_HAL_LOG_ERROR("S2-LP failed to update XO_RCO_CONF0 register. Error %u", (uint32_t)err);
            break;
        }
        /* Update cache after successful write */
        drv_ctx->regs_cache.xo_rco_conf0_reg = xo_rco_conf0;

        /* 2. Wait for calibration start */
        if (wait_start != FALSE)
        {
            err = S2_LP_RADIO_HAL_STATUS_ERROR_TIMEOUT;
            for (uint32_t i = 0u; i < S2LP_RADIO_HAL_RCO_CALIBRATION_START_WAIT_LIMIT; i++)
            {
                /* Read out the status from MC_STATE registers and simultaneously store the status reported while sending header bytes */
                s2_lp_hal_status_t tmp_err = _spi_read_registers(drv_ctx, S2_LP_IC_REG_ADDR_MC_STATE1, &mc_state1.raw, sizeof(mc_state1));
                if (tmp_err != S2_LP_RADIO_HAL_STATUS_OK)
                {
                    S2LP_RADIO_HAL_LOG_ERROR("RCO calibration - unable to retrieve S2-LP MC_STATE1 register. HAL error %u", (uint32_t)tmp_err);
                    err = tmp_err;
                    break;
                }

                if (FALSE == mc_state1.RCCAL_OK)
                {
                    err = S2_LP_RADIO_HAL_STATUS_OK;
                    break;
                }
            }

            /* Check if the wait loop was terminated due to an error or timeout */
            if (err != S2_LP_RADIO_HAL_STATUS_OK)
            {
                /* Increment error counter */
                drv_ctx->rco_cal_init_err_cnt++;

                if (drv_ctx->rco_cal_init_err_cnt >= S2LP_RADIO_HAL_RCO_CAL_INIT_ERROR_LIMIT)
                {
                    if (S2_LP_RADIO_HAL_STATUS_ERROR_TIMEOUT == err)
                    {
                        SID_PAL_LOG_ERROR("Timeout while waiting S2-LP to start RCO calibration");
                    }

                    /* Forward any errors that took place during the calibration start wait loop */
                    break;
                }
                else
                {
                    /* Do nothing at this point and proceed with the rest of the process - it may happen that calibration start was just missed due to polling (e.g. RCO completed calibration between two MC_STATE1 readouts) */
                }
            }
            else
            {
                /* Reset error counter on each successful RCO calibration start detection */
                drv_ctx->rco_cal_init_err_cnt = 0u;
            }
        }

        err = S2_LP_RADIO_HAL_STATUS_OK;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline s2_lp_hal_status_t _store_rco_calibration(halo_drv_s2_lp_ctx_t * const drv_ctx)
{
    s2_lp_hal_status_t err = S2_LP_RADIO_HAL_STATUS_ERROR_GENERIC;

    SID_PAL_ASSERT(drv_ctx != NULL);

    do
    {
        /* 1. Store offline calibration settings */
        sl2_lp_ic_reg_rco_calibr_out_t  rco_calibr_out;
        sl2_lp_ic_reg_rco_calibr_conf_t rco_calibr_conf;
        sl2_lp_ic_reg_xo_rco_conf0_t    xo_rco_conf0 = drv_ctx->regs_cache.xo_rco_conf0_reg;

        /* Read out calibrator outputs */
        err = _spi_read_registers(drv_ctx, S2_LP_IC_REG_ADDR_RCO_CALIBR_OUT4, rco_calibr_out.raw, sizeof(rco_calibr_out));
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_HAL_LOG_ERROR("S2-LP failed to read RCO_CALIBR_OUT registers. Error %u", (uint32_t)err);
            break;
        }

        /* Read out RCO calibration settings */
        err = _spi_read_registers(drv_ctx, S2_LP_IC_REG_ADDR_RCO_CALIBR_CONF3, rco_calibr_conf.raw, sizeof(rco_calibr_conf));
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_HAL_LOG_ERROR("S2-LP failed to read RCO_CALIBR_CONF registers. Error %u", (uint32_t)err);
            break;
        }

        /* Copy RCO calibration values */
        rco_calibr_conf.rco_calibr_conf3.raw      = rco_calibr_out.rco_calibr_out4.raw;       /* This register has no reserved bits and can be copied as is */
        rco_calibr_conf.rco_calibr_conf2.RFB_IN_0 = rco_calibr_out.rco_calibr_out3.RFB_OUT_0; /* This register contains reserved bits that shall not be altered */

        err = _spi_write_registers(drv_ctx, S2_LP_IC_REG_ADDR_RCO_CALIBR_CONF3, rco_calibr_conf.raw, sizeof(rco_calibr_conf));
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_HAL_LOG_ERROR("S2-LP failed to set RCO_CALIBR_CONF registers. Error %u", (uint32_t)err);
            break;
        }

        /* 2. Disable automatic re-calibration to make the wake-up time deterministic */
        xo_rco_conf0.RCO_CALIBRATION = 0u;
        err = _spi_write_registers(drv_ctx, S2_LP_IC_REG_ADDR_XO_RCO_CONF0, &xo_rco_conf0.raw, sizeof(xo_rco_conf0.raw));
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_HAL_LOG_ERROR("S2-LP failed to update XO_RCO_CONF0 register. Error %u", (uint32_t)err);
            break;
        }
        /* Update cache after successful write */
        drv_ctx->regs_cache.xo_rco_conf0_reg = xo_rco_conf0;

        err = S2_LP_RADIO_HAL_STATUS_OK;
    } while (0);

    return err;
}

/* Global function definitions -----------------------------------------------*/

s2_lp_hal_status_t s2_lp_radio_hal_init_gpio(const halo_drv_s2_lp_ctx_t * const drv_ctx)
{
    s2_lp_hal_status_t err = S2_LP_RADIO_HAL_STATUS_ERROR_GENERIC;
    sid_error_t        sid_err;

    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->config != NULL);

    do
    {
        /* Configure radio IRQ pin from MCU side as input ----------------------------*/
        sid_err = sid_pal_gpio_pull_mode(drv_ctx->config->gpio.radio_irq.mcu_pin, SID_PAL_GPIO_PULL_UP);
        if (sid_err != SID_ERROR_NONE)
        {
            S2LP_RADIO_HAL_LOG_ERROR("sid_pal_gpio_pull_mode() failed with error code %d, pin: %u", (int32_t)sid_err, drv_ctx->config->gpio.radio_irq.mcu_pin);
            err = S2_LP_RADIO_HAL_STATUS_ERROR_GPIO;
            break;
        }

        sid_err = sid_pal_gpio_input_mode(drv_ctx->config->gpio.radio_irq.mcu_pin, SID_PAL_GPIO_INPUT_CONNECT);
        if (sid_err != SID_ERROR_NONE)
        {
            S2LP_RADIO_HAL_LOG_ERROR("sid_pal_gpio_input_mode() failed with error code %d, pin: %u", (int32_t)sid_err, drv_ctx->config->gpio.radio_irq.mcu_pin);
            err = S2_LP_RADIO_HAL_STATUS_ERROR_GPIO;
            break;
        }
        /*----------------------------------------------------------------------------*/

        /* Configure radio reset pin -------------------------------------------------*/
        err = _assert_radio_reset(drv_ctx);
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_HAL_LOG_ERROR("Failed to assert S2-LP shutdown line. error %u", (uint32_t)err);
            break;
        }

        sid_err = sid_pal_gpio_output_mode(drv_ctx->config->gpio.radio_shutdown, SID_PAL_GPIO_OUTPUT_PUSH_PULL);
        if (sid_err != SID_ERROR_NONE)
        {
            S2LP_RADIO_HAL_LOG_ERROR("sid_pal_gpio_output_mode() failed with error code %d, pin: %u", (int32_t)sid_err, drv_ctx->config->gpio.radio_shutdown);
            err = S2_LP_RADIO_HAL_STATUS_ERROR_GPIO;
            break;
        }

        sid_err = sid_pal_gpio_set_direction(drv_ctx->config->gpio.radio_shutdown, SID_PAL_GPIO_DIRECTION_OUTPUT);
        if (sid_err != SID_ERROR_NONE)
        {
            S2LP_RADIO_HAL_LOG_ERROR("sid_pal_gpio_set_direction() failed with error code %d, pin: %u", (int32_t)sid_err, drv_ctx->config->gpio.radio_shutdown);
            err = S2_LP_RADIO_HAL_STATUS_ERROR_GPIO;
            break;
        }
        /*----------------------------------------------------------------------------*/

#if S2LP_RADIO_CFG_USE_EXTERNAL_PA
        /* Configure FEM control pins ------------------------------------------------*/
        (void)s2_lp_radio_hal_set_fem_mode(drv_ctx, S2_LP_RADIO_HAL_FEM_MODE_SHUTDOWN);

        sid_err = sid_pal_gpio_output_mode(drv_ctx->config->gpio.rf_fem_csd, SID_PAL_GPIO_OUTPUT_PUSH_PULL);
        if (sid_err != SID_ERROR_NONE)
        {
            S2LP_RADIO_HAL_LOG_ERROR("sid_pal_gpio_output_mode() failed with error code %d, pin: %u", (int32_t)sid_err, drv_ctx->config->gpio.rf_fem_csd);
            err = S2_LP_RADIO_HAL_STATUS_ERROR_GPIO;
            break;
        }
        sid_err = sid_pal_gpio_set_direction(drv_ctx->config->gpio.rf_fem_csd, SID_PAL_GPIO_DIRECTION_OUTPUT);
        if (sid_err != SID_ERROR_NONE)
        {
            S2LP_RADIO_HAL_LOG_ERROR("sid_pal_gpio_set_direction() failed with error code %d, pin: %u", (int32_t)sid_err, drv_ctx->config->gpio.rf_fem_csd);
            err = S2_LP_RADIO_HAL_STATUS_ERROR_GPIO;
            break;
        }

        sid_err = sid_pal_gpio_output_mode(drv_ctx->config->gpio.rf_fem_ctx, SID_PAL_GPIO_OUTPUT_PUSH_PULL);
        if (sid_err != SID_ERROR_NONE)
        {
            S2LP_RADIO_HAL_LOG_ERROR("sid_pal_gpio_output_mode() failed with error code %d, pin: %u", (int32_t)sid_err, drv_ctx->config->gpio.rf_fem_ctx);
            err = S2_LP_RADIO_HAL_STATUS_ERROR_GPIO;
            break;
        }
        sid_err = sid_pal_gpio_set_direction(drv_ctx->config->gpio.rf_fem_ctx, SID_PAL_GPIO_DIRECTION_OUTPUT);
        if (sid_err != SID_ERROR_NONE)
        {
            S2LP_RADIO_HAL_LOG_ERROR("sid_pal_gpio_set_direction() failed with error code %d, pin: %u", (int32_t)sid_err, drv_ctx->config->gpio.rf_fem_ctx);
            err = S2_LP_RADIO_HAL_STATUS_ERROR_GPIO;
            break;
        }

        sid_err = sid_pal_gpio_output_mode(drv_ctx->config->gpio.rf_fem_cps, SID_PAL_GPIO_OUTPUT_PUSH_PULL);
        if (sid_err != SID_ERROR_NONE)
        {
            S2LP_RADIO_HAL_LOG_ERROR("sid_pal_gpio_output_mode() failed with error code %d, pin: %u", (int32_t)sid_err, drv_ctx->config->gpio.rf_fem_cps);
            err = S2_LP_RADIO_HAL_STATUS_ERROR_GPIO;
            break;
        }
        sid_err = sid_pal_gpio_set_direction(drv_ctx->config->gpio.rf_fem_cps, SID_PAL_GPIO_DIRECTION_OUTPUT);
        if (sid_err != SID_ERROR_NONE)
        {
            S2LP_RADIO_HAL_LOG_ERROR("sid_pal_gpio_set_direction() failed with error code %d, pin: %u", (int32_t)sid_err, drv_ctx->config->gpio.rf_fem_cps);
            err = S2_LP_RADIO_HAL_STATUS_ERROR_GPIO;
            break;
        }
        /*----------------------------------------------------------------------------*/
#endif /* S2LP_RADIO_CFG_USE_EXTERNAL_PA */

#if S2LP_RADIO_CFG_USE_STATUS_LED
        /* Configure status LED ------------------------------------------------------*/
        /* Tx LED */
        if (HALO_GPIO_NOT_CONNECTED != drv_ctx->config->gpio.tx_led)
        {
            /* Make sure LED won't glitch - put GPIO output to Off state before the pin is configured as output */
            s2_lp_radio_hal_tx_led_off(drv_ctx);

            sid_err = sid_pal_gpio_output_mode(drv_ctx->config->gpio.tx_led, SID_PAL_GPIO_OUTPUT_PUSH_PULL);
            if (sid_err != SID_ERROR_NONE)
            {
                S2LP_RADIO_HAL_LOG_ERROR("sid_pal_gpio_output_mode() failed with error code %d, pin: %u", (int32_t)sid_err, drv_ctx->config->gpio.tx_led);
                err = S2_LP_RADIO_HAL_STATUS_ERROR_GPIO;
                break;
            }
            sid_err = sid_pal_gpio_set_direction(drv_ctx->config->gpio.tx_led, SID_PAL_GPIO_DIRECTION_OUTPUT);
            if (sid_err != SID_ERROR_NONE)
            {
                S2LP_RADIO_HAL_LOG_ERROR("sid_pal_gpio_set_direction() failed with error code %d, pin: %u", (int32_t)sid_err, drv_ctx->config->gpio.tx_led);
                err = S2_LP_RADIO_HAL_STATUS_ERROR_GPIO;
                break;
            }
        }

        /* Rx LED */
        if ((HALO_GPIO_NOT_CONNECTED != drv_ctx->config->gpio.rx_led) && (drv_ctx->config->gpio.rx_led != drv_ctx->config->gpio.tx_led))
        {
            s2_lp_radio_hal_rx_led_off(drv_ctx);

            sid_err = sid_pal_gpio_output_mode(drv_ctx->config->gpio.rx_led, SID_PAL_GPIO_OUTPUT_PUSH_PULL);
            if (sid_err != SID_ERROR_NONE)
            {
                S2LP_RADIO_HAL_LOG_ERROR("sid_pal_gpio_output_mode() failed with error code %d, pin: %u", (int32_t)sid_err, drv_ctx->config->gpio.rx_led);
                err = S2_LP_RADIO_HAL_STATUS_ERROR_GPIO;
                break;
            }
            sid_err = sid_pal_gpio_set_direction(drv_ctx->config->gpio.rx_led, SID_PAL_GPIO_DIRECTION_OUTPUT);
            if (sid_err != SID_ERROR_NONE)
            {
                S2LP_RADIO_HAL_LOG_ERROR("sid_pal_gpio_set_direction() failed with error code %d, pin: %u", (int32_t)sid_err, drv_ctx->config->gpio.rx_led);
                err = S2_LP_RADIO_HAL_STATUS_ERROR_GPIO;
                break;
            }
        }
        /*----------------------------------------------------------------------------*/
#endif /* S2LP_RADIO_CFG_USE_STATUS_LED */

        err = S2_LP_RADIO_HAL_STATUS_OK;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

s2_lp_hal_status_t s2_lp_radio_hal_deinit_gpio(const halo_drv_s2_lp_ctx_t * const drv_ctx)
{
    s2_lp_hal_status_t err = S2_LP_RADIO_HAL_STATUS_ERROR_GENERIC;
    sid_error_t sid_err;

    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->config != NULL);

    do
    {
        /* Set radio IRQ pin from MCU side to Hi-Z -----------------------------------*/
        err = s2_lp_radio_hal_disarm_irq(drv_ctx);
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_HAL_LOG_ERROR("Failed to disarm S2-LP IRQ line. error %u", (uint32_t)err);
            break;
        }

        sid_err = sid_pal_gpio_pull_mode(drv_ctx->config->gpio.radio_irq.mcu_pin, SID_PAL_GPIO_PULL_NONE);
        if (sid_err != SID_ERROR_NONE)
        {
            S2LP_RADIO_HAL_LOG_ERROR("sid_pal_gpio_pull_mode() failed with error code %d, pin: %u", (int32_t)sid_err, drv_ctx->config->gpio.radio_irq.mcu_pin);
            err = S2_LP_RADIO_HAL_STATUS_ERROR_GPIO;
            break;
        }

        sid_err = sid_pal_gpio_input_mode(drv_ctx->config->gpio.radio_irq.mcu_pin, SID_PAL_GPIO_INPUT_DISCONNECT);
        if (sid_err != SID_ERROR_NONE)
        {
            S2LP_RADIO_HAL_LOG_ERROR("sid_pal_gpio_input_mode() failed with error code %d, pin: %u", (int32_t)sid_err, drv_ctx->config->gpio.radio_irq.mcu_pin);
            err = S2_LP_RADIO_HAL_STATUS_ERROR_GPIO;
            break;
        }
        /*----------------------------------------------------------------------------*/

        /* Set radio reset pin to Hi-Z with pull-up ----------------------------------*/
        sid_err = sid_pal_gpio_input_mode(drv_ctx->config->gpio.radio_shutdown, SID_PAL_GPIO_INPUT_DISCONNECT);
        if (sid_err != SID_ERROR_NONE)
        {
            S2LP_RADIO_HAL_LOG_ERROR("sid_pal_gpio_input_mode() failed with error code %d, pin: %u", (int32_t)sid_err, drv_ctx->config->gpio.radio_shutdown);
            err = S2_LP_RADIO_HAL_STATUS_ERROR_GPIO;
            break;
        }

        sid_err = sid_pal_gpio_pull_mode(drv_ctx->config->gpio.radio_shutdown, SID_PAL_GPIO_PULL_UP); /* Ensure the pin isn't floating and keeps the radio in shutdown state */
        if (sid_err != SID_ERROR_NONE)
        {
            S2LP_RADIO_HAL_LOG_ERROR("sid_pal_gpio_pull_mode() failed with error code %d, pin: %u", (int32_t)sid_err, drv_ctx->config->gpio.radio_shutdown);
            err = S2_LP_RADIO_HAL_STATUS_ERROR_GPIO;
            break;
        }

        sid_err = sid_pal_gpio_set_direction(drv_ctx->config->gpio.radio_shutdown, SID_PAL_GPIO_DIRECTION_INPUT);
        if (sid_err != SID_ERROR_NONE)
        {
            S2LP_RADIO_HAL_LOG_ERROR("sid_pal_gpio_set_direction() failed with error code %d, pin: %u", (int32_t)sid_err, drv_ctx->config->gpio.radio_shutdown);
            err = S2_LP_RADIO_HAL_STATUS_ERROR_GPIO;
            break;
        }
        /*----------------------------------------------------------------------------*/

#if S2LP_RADIO_CFG_USE_EXTERNAL_PA
        /* Set FEM control pins to Hi-Z ----------------------------------------------*/
        sid_err = sid_pal_gpio_input_mode(drv_ctx->config->gpio.rf_fem_csd, SID_PAL_GPIO_INPUT_DISCONNECT);
        if (sid_err != SID_ERROR_NONE)
        {
            S2LP_RADIO_HAL_LOG_ERROR("sid_pal_gpio_input_mode() failed with error code %d, pin: %u", (int32_t)sid_err, drv_ctx->config->gpio.rf_fem_csd);
            err = S2_LP_RADIO_HAL_STATUS_ERROR_GPIO;
            break;
        }
        sid_err = sid_pal_gpio_set_direction(drv_ctx->config->gpio.rf_fem_csd, SID_PAL_GPIO_DIRECTION_INPUT);
        if (sid_err != SID_ERROR_NONE)
        {
            S2LP_RADIO_HAL_LOG_ERROR("sid_pal_gpio_set_direction() failed with error code %d, pin: %u", (int32_t)sid_err, drv_ctx->config->gpio.rf_fem_csd);
            err = S2_LP_RADIO_HAL_STATUS_ERROR_GPIO;
            break;
        }

        sid_err = sid_pal_gpio_input_mode(drv_ctx->config->gpio.rf_fem_ctx, SID_PAL_GPIO_INPUT_DISCONNECT);
        if (sid_err != SID_ERROR_NONE)
        {
            S2LP_RADIO_HAL_LOG_ERROR("sid_pal_gpio_input_mode() failed with error code %d, pin: %u", (int32_t)sid_err, drv_ctx->config->gpio.rf_fem_ctx);
            err = S2_LP_RADIO_HAL_STATUS_ERROR_GPIO;
            break;
        }
        sid_err = sid_pal_gpio_set_direction(drv_ctx->config->gpio.rf_fem_ctx, SID_PAL_GPIO_DIRECTION_INPUT);
        if (sid_err != SID_ERROR_NONE)
        {
            S2LP_RADIO_HAL_LOG_ERROR("sid_pal_gpio_set_direction() failed with error code %d, pin: %u", (int32_t)sid_err, drv_ctx->config->gpio.rf_fem_ctx);
            err = S2_LP_RADIO_HAL_STATUS_ERROR_GPIO;
            break;
        }

        sid_err = sid_pal_gpio_input_mode(drv_ctx->config->gpio.rf_fem_cps, SID_PAL_GPIO_INPUT_DISCONNECT);
        if (sid_err != SID_ERROR_NONE)
        {
            S2LP_RADIO_HAL_LOG_ERROR("sid_pal_gpio_input_mode() failed with error code %d, pin: %u", (int32_t)sid_err, drv_ctx->config->gpio.rf_fem_cps);
            err = S2_LP_RADIO_HAL_STATUS_ERROR_GPIO;
            break;
        }
        sid_err = sid_pal_gpio_set_direction(drv_ctx->config->gpio.rf_fem_cps, SID_PAL_GPIO_DIRECTION_INPUT);
        if (sid_err != SID_ERROR_NONE)
        {
            S2LP_RADIO_HAL_LOG_ERROR("sid_pal_gpio_set_direction() failed with error code %d, pin: %u", (int32_t)sid_err, drv_ctx->config->gpio.rf_fem_cps);
            err = S2_LP_RADIO_HAL_STATUS_ERROR_GPIO;
            break;
        }
        /*----------------------------------------------------------------------------*/
#endif /* S2LP_RADIO_CFG_USE_EXTERNAL_PA */

#if S2LP_RADIO_CFG_USE_STATUS_LED
        /* Set status LED pin to Hi-Z ------------------------------------------------*/
        /* Tx LED */
        if (HALO_GPIO_NOT_CONNECTED != drv_ctx->config->gpio.tx_led)
        {
            sid_err = sid_pal_gpio_input_mode(drv_ctx->config->gpio.tx_led, SID_PAL_GPIO_INPUT_DISCONNECT);
            if (sid_err != SID_ERROR_NONE)
            {
                S2LP_RADIO_HAL_LOG_ERROR("sid_pal_gpio_input_mode() failed with error code %d, pin: %u", (int32_t)sid_err, drv_ctx->config->gpio.tx_led);
                err = S2_LP_RADIO_HAL_STATUS_ERROR_GPIO;
                break;
            }

            sid_err = sid_pal_gpio_set_direction(drv_ctx->config->gpio.tx_led, SID_PAL_GPIO_DIRECTION_INPUT);
            if (sid_err != SID_ERROR_NONE)
            {
                S2LP_RADIO_HAL_LOG_ERROR("sid_pal_gpio_set_direction() failed with error code %d, pin: %u", (int32_t)sid_err, drv_ctx->config->gpio.tx_led);
                err = S2_LP_RADIO_HAL_STATUS_ERROR_GPIO;
                break;
            }
        }

        /* Rx LED */
        if ((HALO_GPIO_NOT_CONNECTED != drv_ctx->config->gpio.rx_led) && (drv_ctx->config->gpio.rx_led != drv_ctx->config->gpio.tx_led))
        {
            sid_err = sid_pal_gpio_input_mode(drv_ctx->config->gpio.rx_led, SID_PAL_GPIO_INPUT_DISCONNECT);
            if (sid_err != SID_ERROR_NONE)
            {
                S2LP_RADIO_HAL_LOG_ERROR("sid_pal_gpio_input_mode() failed with error code %d, pin: %u", (int32_t)sid_err, drv_ctx->config->gpio.rx_led);
                err = S2_LP_RADIO_HAL_STATUS_ERROR_GPIO;
                break;
            }

            sid_err = sid_pal_gpio_set_direction(drv_ctx->config->gpio.rx_led, SID_PAL_GPIO_DIRECTION_INPUT);
            if (sid_err != SID_ERROR_NONE)
            {
                S2LP_RADIO_HAL_LOG_ERROR("sid_pal_gpio_set_direction() failed with error code %d, pin: %u", (int32_t)sid_err, drv_ctx->config->gpio.rx_led);
                err = S2_LP_RADIO_HAL_STATUS_ERROR_GPIO;
                break;
            }
        }
        /*----------------------------------------------------------------------------*/
#endif /* S2LP_RADIO_CFG_USE_STATUS_LED */

        err = S2_LP_RADIO_HAL_STATUS_OK;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

#if S2LP_RADIO_CFG_USE_STATUS_LED
SID_STM32_SPEED_OPTIMIZED s2_lp_hal_status_t s2_lp_radio_hal_tx_led_on(const halo_drv_s2_lp_ctx_t * const drv_ctx)
{
    s2_lp_hal_status_t err = S2_LP_RADIO_HAL_STATUS_OK;

    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->config != NULL);

    if (HALO_GPIO_NOT_CONNECTED != drv_ctx->config->gpio.tx_led)
    {
        sid_error_t sid_err;
        uint8_t write_val = drv_ctx->config->gpio.tx_led_on_gpio_state != 0u ? 1u : 0u;

        sid_err = sid_pal_gpio_write(drv_ctx->config->gpio.tx_led, write_val);
        if (sid_err != SID_ERROR_NONE)
        {
            S2LP_RADIO_HAL_LOG_ERROR("sid_pal_gpio_write() failed with error code %d, pin: %u", (int32_t)sid_err, drv_ctx->config->gpio.tx_led);
            err = S2_LP_RADIO_HAL_STATUS_ERROR_HW;
        }
    }

    return err;
}
#endif /* S2LP_RADIO_CFG_USE_STATUS_LED */

/*----------------------------------------------------------------------------*/

#if S2LP_RADIO_CFG_USE_STATUS_LED
SID_STM32_SPEED_OPTIMIZED s2_lp_hal_status_t s2_lp_radio_hal_tx_led_off(const halo_drv_s2_lp_ctx_t * const drv_ctx)
{
    s2_lp_hal_status_t err = S2_LP_RADIO_HAL_STATUS_OK;

    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->config != NULL);

    if (HALO_GPIO_NOT_CONNECTED != drv_ctx->config->gpio.tx_led)
    {
        sid_error_t sid_err;
        uint8_t write_val = drv_ctx->config->gpio.tx_led_on_gpio_state != 0u ? 0u : 1u;

        sid_err = sid_pal_gpio_write(drv_ctx->config->gpio.tx_led, write_val);
        if (sid_err != SID_ERROR_NONE)
        {
            S2LP_RADIO_HAL_LOG_ERROR("sid_pal_gpio_write() failed with error code %d, pin: %u", (int32_t)sid_err, drv_ctx->config->gpio.tx_led);
            err = S2_LP_RADIO_HAL_STATUS_ERROR_HW;
        }
    }

    return err;
}
#endif /* S2LP_RADIO_CFG_USE_STATUS_LED */

/*----------------------------------------------------------------------------*/

#if S2LP_RADIO_CFG_USE_STATUS_LED
SID_STM32_SPEED_OPTIMIZED s2_lp_hal_status_t s2_lp_radio_hal_rx_led_on(const halo_drv_s2_lp_ctx_t * const drv_ctx)
{
    s2_lp_hal_status_t err = S2_LP_RADIO_HAL_STATUS_OK;

    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->config != NULL);

    if (HALO_GPIO_NOT_CONNECTED != drv_ctx->config->gpio.rx_led)
    {
        sid_error_t sid_err;
        uint8_t write_val = drv_ctx->config->gpio.rx_led_on_gpio_state != 0u ? 1u : 0u;

        sid_err = sid_pal_gpio_write(drv_ctx->config->gpio.rx_led, write_val);
        if (sid_err != SID_ERROR_NONE)
        {
            S2LP_RADIO_HAL_LOG_ERROR("sid_pal_gpio_write() failed with error code %d, pin: %u", (int32_t)sid_err, drv_ctx->config->gpio.rx_led);
            err = S2_LP_RADIO_HAL_STATUS_ERROR_HW;
        }
    }

    return err;
}
#endif /* S2LP_RADIO_CFG_USE_STATUS_LED */

/*----------------------------------------------------------------------------*/

#if S2LP_RADIO_CFG_USE_STATUS_LED
SID_STM32_SPEED_OPTIMIZED s2_lp_hal_status_t s2_lp_radio_hal_rx_led_off(const halo_drv_s2_lp_ctx_t * const drv_ctx)
{
    s2_lp_hal_status_t err = S2_LP_RADIO_HAL_STATUS_OK;

    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->config != NULL);

    if (HALO_GPIO_NOT_CONNECTED != drv_ctx->config->gpio.rx_led)
    {
        sid_error_t sid_err;
        uint8_t write_val = drv_ctx->config->gpio.rx_led_on_gpio_state != 0u ? 0u : 1u;

        sid_err = sid_pal_gpio_write(drv_ctx->config->gpio.rx_led, write_val);
        if (sid_err != SID_ERROR_NONE)
        {
            S2LP_RADIO_HAL_LOG_ERROR("sid_pal_gpio_write() failed with error code %d, pin: %u", (int32_t)sid_err, drv_ctx->config->gpio.rx_led);
            err = S2_LP_RADIO_HAL_STATUS_ERROR_HW;
        }
    }

    return err;
}
#endif /* S2LP_RADIO_CFG_USE_STATUS_LED */

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED s2_lp_hal_status_t s2_lp_radio_hal_reset_radio(const halo_drv_s2_lp_ctx_t * const drv_ctx)
{
    s2_lp_hal_status_t err = S2_LP_RADIO_HAL_STATUS_ERROR_GENERIC;

    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->config != NULL);

    do
    {
        /* Shutdown the radio IC */
        err = _assert_radio_reset(drv_ctx);
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_HAL_LOG_ERROR("_assert_radio_reset() failed with error code %u", (uint32_t)err);
            err = S2_LP_RADIO_HAL_STATUS_ERROR_HW;
            break;
        }

        /* Keep the shutdown hold time */
        sid_pal_delay_us(S2LP_RADIO_HAL_SHUTDOWN_HOLD_TIME_US);

        /* Turn on the IC */
        err = _release_radio_reset(drv_ctx);
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_HAL_LOG_ERROR("_release_radio_reset() failed with error code %u", (uint32_t)err);
            err = S2_LP_RADIO_HAL_STATUS_ERROR_HW;
            break;
        }

        /* Now wait for internal Reset pulse as per the datasheet */
#ifdef SID_PAL_ENABLE_SCHEDULER_DELAY
        sid_pal_scheduler_delay_ms(S2LP_RADIO_HAL_INTERNAL_RESET_PULSE_WIDTH_MS);
#else
        sid_pal_delay_us(S2LP_RADIO_HAL_INTERNAL_RESET_PULSE_WIDTH_MS * 1000u);
#endif /* SID_PAL_ENABLE_SCHEDULER_DELAY */

        SID_PAL_LOG_DEBUG("S2-LP reset done");
        err = S2_LP_RADIO_HAL_STATUS_OK;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

#if S2LP_RADIO_CFG_USE_EXTERNAL_PA
SID_STM32_SPEED_OPTIMIZED s2_lp_hal_status_t s2_lp_radio_hal_set_fem_mode(const halo_drv_s2_lp_ctx_t * const drv_ctx, const s2_lp_hal_fem_mode_t mode)
{
    s2_lp_hal_status_t err = S2_LP_RADIO_HAL_STATUS_ERROR_GENERIC;
    sid_error_t sid_err;

    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->config != NULL);

    switch (mode)
    {
        case S2_LP_RADIO_HAL_FEM_MODE_SHUTDOWN:
            sid_err = sid_pal_gpio_write(drv_ctx->config->gpio.rf_fem_csd, 0u);
            if (sid_err != SID_ERROR_NONE)
            {
                S2LP_RADIO_HAL_LOG_ERROR("sid_pal_gpio_write() failed with error code %d, pin: %u", (int32_t)sid_err, drv_ctx->config->gpio.rf_fem_csd);
                err = S2_LP_RADIO_HAL_STATUS_ERROR_HW;
                break;
            }
            sid_err = sid_pal_gpio_write(drv_ctx->config->gpio.rf_fem_ctx, 0u);
            if (sid_err != SID_ERROR_NONE)
            {
                S2LP_RADIO_HAL_LOG_ERROR("sid_pal_gpio_write() failed with error code %d, pin: %u", (int32_t)sid_err, drv_ctx->config->gpio.rf_fem_ctx);
                err = S2_LP_RADIO_HAL_STATUS_ERROR_HW;
                break;
            }
            sid_err = sid_pal_gpio_write(drv_ctx->config->gpio.rf_fem_cps, 0u);
            if (sid_err != SID_ERROR_NONE)
            {
                S2LP_RADIO_HAL_LOG_ERROR("sid_pal_gpio_write() failed with error code %d, pin: %u", (int32_t)sid_err, drv_ctx->config->gpio.rf_fem_cps);
                err = S2_LP_RADIO_HAL_STATUS_ERROR_HW;
                break;
            }
            err = S2_LP_RADIO_HAL_STATUS_OK;
            break;

        case S2_LP_RADIO_HAL_FEM_MODE_RX:
            sid_err = sid_pal_gpio_write(drv_ctx->config->gpio.rf_fem_csd, 1u);
            if (sid_err != SID_ERROR_NONE)
            {
                S2LP_RADIO_HAL_LOG_ERROR("sid_pal_gpio_write() failed with error code %d, pin: %u", (int32_t)sid_err, drv_ctx->config->gpio.rf_fem_csd);
                err = S2_LP_RADIO_HAL_STATUS_ERROR_HW;
                break;
            }
            sid_err = sid_pal_gpio_write(drv_ctx->config->gpio.rf_fem_ctx, 0u);
            if (sid_err != SID_ERROR_NONE)
            {
                S2LP_RADIO_HAL_LOG_ERROR("sid_pal_gpio_write() failed with error code %d, pin: %u", (int32_t)sid_err, drv_ctx->config->gpio.rf_fem_ctx);
                err = S2_LP_RADIO_HAL_STATUS_ERROR_HW;
                break;
            }
            sid_err = sid_pal_gpio_write(drv_ctx->config->gpio.rf_fem_cps, 0u);
            if (sid_err != SID_ERROR_NONE)
            {
                S2LP_RADIO_HAL_LOG_ERROR("sid_pal_gpio_write() failed with error code %d, pin: %u", (int32_t)sid_err, drv_ctx->config->gpio.rf_fem_cps);
                err = S2_LP_RADIO_HAL_STATUS_ERROR_HW;
                break;
            }
            err = S2_LP_RADIO_HAL_STATUS_OK;
            break;

        case S2_LP_RADIO_HAL_FEM_MODE_TX_LP:
            sid_err = sid_pal_gpio_write(drv_ctx->config->gpio.rf_fem_csd, 1u);
            if (sid_err != SID_ERROR_NONE)
            {
                S2LP_RADIO_HAL_LOG_ERROR("sid_pal_gpio_write() failed with error code %d, pin: %u", (int32_t)sid_err, drv_ctx->config->gpio.rf_fem_csd);
                err = S2_LP_RADIO_HAL_STATUS_ERROR_HW;
                break;
            }
            sid_err = sid_pal_gpio_write(drv_ctx->config->gpio.rf_fem_ctx, 1u);
            if (sid_err != SID_ERROR_NONE)
            {
                S2LP_RADIO_HAL_LOG_ERROR("sid_pal_gpio_write() failed with error code %d, pin: %u", (int32_t)sid_err, drv_ctx->config->gpio.rf_fem_ctx);
                err = S2_LP_RADIO_HAL_STATUS_ERROR_HW;
                break;
            }
            sid_err = sid_pal_gpio_write(drv_ctx->config->gpio.rf_fem_cps, 0u);
            if (sid_err != SID_ERROR_NONE)
            {
                S2LP_RADIO_HAL_LOG_ERROR("sid_pal_gpio_write() failed with error code %d, pin: %u", (int32_t)sid_err, drv_ctx->config->gpio.rf_fem_cps);
                err = S2_LP_RADIO_HAL_STATUS_ERROR_HW;
                break;
            }
            err = S2_LP_RADIO_HAL_STATUS_OK;
            break;

        case S2_LP_RADIO_HAL_FEM_MODE_TX_HP:
            sid_err = sid_pal_gpio_write(drv_ctx->config->gpio.rf_fem_csd, 1u);
            if (sid_err != SID_ERROR_NONE)
            {
                S2LP_RADIO_HAL_LOG_ERROR("sid_pal_gpio_write() failed with error code %d, pin: %u", (int32_t)sid_err, drv_ctx->config->gpio.rf_fem_csd);
                err = S2_LP_RADIO_HAL_STATUS_ERROR_HW;
                break;
            }
            sid_err = sid_pal_gpio_write(drv_ctx->config->gpio.rf_fem_ctx, 1u);
            if (sid_err != SID_ERROR_NONE)
            {
                S2LP_RADIO_HAL_LOG_ERROR("sid_pal_gpio_write() failed with error code %d, pin: %u", (int32_t)sid_err, drv_ctx->config->gpio.rf_fem_ctx);
                err = S2_LP_RADIO_HAL_STATUS_ERROR_HW;
                break;
            }
            sid_err = sid_pal_gpio_write(drv_ctx->config->gpio.rf_fem_cps, 1u);
            if (sid_err != SID_ERROR_NONE)
            {
                S2LP_RADIO_HAL_LOG_ERROR("sid_pal_gpio_write() failed with error code %d, pin: %u", (int32_t)sid_err, drv_ctx->config->gpio.rf_fem_cps);
                err = S2_LP_RADIO_HAL_STATUS_ERROR_HW;
                break;
            }
            err = S2_LP_RADIO_HAL_STATUS_OK;
            break;

        default:
            S2LP_RADIO_HAL_LOG_ERROR("FEM mode 0x%X is not a valid value", (uint32_t)mode);
            err = S2_LP_RADIO_HAL_STATUS_INVALID_ARGS;
            break;
    }

    return err;
}
#endif /* S2LP_RADIO_CFG_USE_EXTERNAL_PA */

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED s2_lp_hal_status_t s2_lp_radio_hal_disarm_irq(const halo_drv_s2_lp_ctx_t * const drv_ctx)
{
    s2_lp_hal_status_t err = S2_LP_RADIO_HAL_STATUS_ERROR_GENERIC;
    sid_error_t sid_err;

    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->config != NULL);

    do
    {
        /* Disable IRQ line */
        sid_err = sid_pal_gpio_irq_disable(drv_ctx->config->gpio.radio_irq.mcu_pin);
        if (sid_err != SID_ERROR_NONE)
        {
            S2LP_RADIO_HAL_LOG_ERROR("Failed to disable radio IRQ. Error %d", (int32_t)sid_err);
            err = S2_LP_RADIO_HAL_STATUS_ERROR_HW;
            break;
        }

        /* Disconnect GPIO pin from IRQ line */
        sid_err = sid_pal_gpio_set_irq(drv_ctx->config->gpio.radio_irq.mcu_pin, SID_PAL_GPIO_IRQ_TRIGGER_NONE, NULL, NULL);
        if (sid_err != SID_ERROR_NONE)
        {
            S2LP_RADIO_HAL_LOG_ERROR("Failed to deinit radio IRQ GPIO. Error %d", (int32_t)sid_err);
            err = S2_LP_RADIO_HAL_STATUS_ERROR_HW;
            break;
        }

        /* Everything is fine */
        err = S2_LP_RADIO_HAL_STATUS_OK;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED s2_lp_hal_status_t s2_lp_radio_hal_arm_irq(halo_drv_s2_lp_ctx_t * const drv_ctx)
{
    s2_lp_hal_status_t err = S2_LP_RADIO_HAL_STATUS_ERROR_GENERIC;
    sid_error_t sid_err;

    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->config != NULL);

    do
    {
#if (__ARM_ARCH_6M__ == 0)
        /**
         * Indicate the radio IRQ is going to use elevated priority
         *
         * WARNING: It's important to do it before calling sid_pal_gpio_set_irq() because the respective ISR may be invoked immediately, before the IRQ priority is actual configured */
        drv_ctx->radio_is_running_high_prio = TRUE;
#endif /* (__ARM_ARCH_6M__ == 0) */

        /* Configure MCU pin to trigger IRQ */
        sid_err = sid_pal_gpio_set_irq(drv_ctx->config->gpio.radio_irq.mcu_pin, SID_PAL_GPIO_IRQ_TRIGGER_LOW, _on_radio_irq_detected, (void*)drv_ctx);
        if (sid_err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to enable S2-LP IRQ in NVIC, error %d", (int32_t)sid_err);
            err = S2_LP_RADIO_HAL_STATUS_ERROR_HW;
            break;
        }

#if (__ARM_ARCH_6M__ == 0)
        /* Set elevated priority to S2-LP IRQ line so we can capture radio IRQ timestamp with more precision */
        sid_err = sid_pal_gpio_ext_ifc_set_irq_priority(drv_ctx->config->gpio.radio_irq.mcu_pin, drv_ctx->config->gpio.radio_irq.prio_high, 0u);
#else
        /* Set unified priority to S2-LP IRQ line since dynamic IRQ priority changing is not supported */
        sid_err = sid_pal_gpio_ext_ifc_set_irq_priority(drv_ctx->config->gpio.radio_irq.mcu_pin, drv_ctx->config->gpio.radio_irq.prio, 0u);
#endif /* (__ARM_ARCH_6M__ == 0) */
        if (sid_err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to set S2-LP IRQ priority, error %d", (int32_t)sid_err);
            err = S2_LP_RADIO_HAL_STATUS_ERROR_HW;
            break;
        }

        err = S2_LP_RADIO_HAL_STATUS_OK;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED s2_lp_hal_status_t s2_lp_radio_hal_set_radio_irq_mask(const halo_drv_s2_lp_ctx_t * const drv_ctx, const sl2_lp_ic_irq_mask_t mask)
{
    s2_lp_hal_status_t err = S2_LP_RADIO_HAL_STATUS_ERROR_GENERIC;

    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->config != NULL);

    do
    {
        /* Send out IRQ mask to S2-LP */
        sl2_lp_ic_reg_irq_mask_t irq_mask_regs = {
            .irq_mask3.INT_MASK_31_24 = (uint8_t)(mask.raw >> 24),
            .irq_mask2.INT_MASK_23_16 = (uint8_t)(mask.raw >> 16),
            .irq_mask1.INT_MASK_15_8  = (uint8_t)(mask.raw >>  8),
            .irq_mask0.INT_MASK_7_0   = (uint8_t)(mask.raw      ),
        };

        err = _spi_write_registers(drv_ctx, S2_LP_IC_REG_ADDR_IRQ_MASK3, irq_mask_regs.raw, sizeof(irq_mask_regs));
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_HAL_LOG_ERROR("S2-LP failed to set IRQ_MASK registers. Error %u", (uint32_t)err);
            break;
        }

        /* Clear any residual IRQ indications in S2-LP */
        err = s2_lp_radio_hal_get_clear_irq_status(drv_ctx, NULL);
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to clear S2-LP IRQ flags. HAL error %u", (uint32_t)err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        err = S2_LP_RADIO_HAL_STATUS_OK;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED s2_lp_hal_status_t s2_lp_radio_hal_ll_init(halo_drv_s2_lp_ctx_t * const drv_ctx)
{
    s2_lp_hal_status_t err = S2_LP_RADIO_HAL_STATUS_ERROR_GENERIC;

    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->config != NULL);

    do
    {
        /* Configure S2-LP GPIO  -----------------------------------------------------*/
        sl2_lp_ic_reg_gpiox_conf_t gpio_cfg_regs[S2_LP_IC_NUM_GPIO_PINS] = {0};

        for (uint32_t i = 0u; i < SID_STM32_UTIL_ARRAY_SIZE(gpio_cfg_regs); i++)
        {
            if (((uint32_t)S2_LP_GPIO_0 + i) != (uint32_t)drv_ctx->config->gpio.radio_irq.s2lp_pin)
            {
                /* By default set pin as low power output and drive it low - this will help to reduce power consumption and to avoid unwanted oscillations */
                gpio_cfg_regs[i].GPIOx_SELECT = S2_LP_GPIO_SIG_OUT_GND;
                gpio_cfg_regs[i].GPIOx_MODE   = S2_LP_GPIO_MODE_DIGITAL_OUTPUT_LP;
            }
            else
            {
                /* This pin shall drive the IRQ line interfacing the MCU */
                gpio_cfg_regs[i].GPIOx_SELECT = S2_LP_GPIO_SIG_OUT_IRQ;
                gpio_cfg_regs[i].GPIOx_MODE   = S2_LP_GPIO_MODE_DIGITAL_OUTPUT_LP;
            }
        }

        err = _spi_write_registers(drv_ctx, S2_LP_IC_REG_ADDR_GPIO0_CONF, (uint8_t *)(void *)&gpio_cfg_regs[0], sizeof(gpio_cfg_regs));
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_HAL_LOG_ERROR("S2-LP failed to set GPIOx_CONF registers. Error %u", (uint32_t)err);
            break;
        }
        /*----------------------------------------------------------------------------*/

        /* Put the S2-LP into STANDBY state before any further actions ---------------*/
        err = _spi_send_command(drv_ctx, S2_LP_IC_CMD_STANDBY);
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_HAL_LOG_ERROR("Failed to put S2-LP in STANDBY state. Error %u", (uint32_t)err);
            break;
        }

        /* Wait until the actual transition to the STANDBY state is finished */
        err = _wait_for_state_transition(drv_ctx, S2_LP_MC_STATE_STANDBY, S2LP_RADIO_HAL_STATE_TRANSITION_DEFAULT_TIMEOUT_US);
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_HAL_LOG_ERROR("S2-LP failed to reach STANDBY state. Error %u", (uint32_t)err);
            break;
        }
        /*----------------------------------------------------------------------------*/

        /* Configure XTAL frequency divider ------------------------------------------*/
        uint32_t valid_xtal_freqs_mhz[2][3] = {
            {24u, 25u, 26u}, /* XTAL range A */
            {48u, 50u, 52u}, /* XTAL range B */
        };
        uint32_t range_selector;

        if (drv_ctx->config->xin_freq < S2_LP_IC_DIG_DOMAIN_XTAL_THRESH)
        {
            range_selector = 0u; /* Use range A */
        }
        else
        {
            range_selector = 1u; /* Use range B */
        }

        /* Validity check */
        uint32_t xtal_freq_valid = FALSE;
        for (uint32_t i = 0u; i < SID_STM32_UTIL_ARRAY_SIZE(valid_xtal_freqs_mhz[range_selector]); i++)
        {
            const uint32_t valid_freq_hz = valid_xtal_freqs_mhz[range_selector][i] * 1000000u;
            if (valid_freq_hz == drv_ctx->config->xin_freq)
            {
                xtal_freq_valid = TRUE;
                break;
            }
        }

        if (FALSE == xtal_freq_valid)
        {
            SID_PAL_LOG_ERROR("Invalid S2-LP radio config. XTAL frequency of %uHz cannot be used. Valid XTAL options are 24MHz, 25MHz, 26MHz, 48MHz, 50MHz, 52MHz", drv_ctx->config->xin_freq);
            err = S2_LP_RADIO_HAL_STATUS_INVALID_ARGS;
            break;
        }

        /* Modify XO_RCO_CONF1 based on the used XTAL frequency */
        sl2_lp_ic_reg_xo_rco_conf1_t rco_cfg1_reg;

        err = _spi_read_registers(drv_ctx, S2_LP_IC_REG_ADDR_XO_RCO_CONF1, &rco_cfg1_reg.raw, sizeof(rco_cfg1_reg.raw));
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_HAL_LOG_ERROR("Failed to read XO_RCO_CONF1 register. Error %u", (uint32_t)err);
            break;
        }

        if (drv_ctx->config->xin_freq < S2_LP_IC_DIG_DOMAIN_XTAL_THRESH)
        {
            rco_cfg1_reg.PD_CLKDIV = 1u; /* 1: Disable clock dividers */
            drv_ctx->dig_freq_hz   = drv_ctx->config->xin_freq;
        }
        else
        {
            rco_cfg1_reg.PD_CLKDIV = 0u; /* 0: Enable clock dividers */
            drv_ctx->dig_freq_hz   = drv_ctx->config->xin_freq >> 1;
        }
        drv_ctx->dig_freq_mhz = drv_ctx->dig_freq_hz / 1000000u; /* Calculate this once at initialization time for future use in calculations */

        /* Write back the modified value */
        err = _spi_write_registers(drv_ctx, S2_LP_IC_REG_ADDR_XO_RCO_CONF1, &rco_cfg1_reg.raw, sizeof(rco_cfg1_reg));
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_HAL_LOG_ERROR("Failed to modify XO_RCO_CONF1 register. Error %u", (uint32_t)err);
            break;
        }
        /*----------------------------------------------------------------------------*/

        /* Configure external clock usage --------------------------------------------*/
        sl2_lp_ic_reg_xo_rco_conf0_t * const xo_rco_conf0 = &drv_ctx->regs_cache.xo_rco_conf0_reg;

        err = _spi_read_registers(drv_ctx, S2_LP_IC_REG_ADDR_XO_RCO_CONF0, &xo_rco_conf0->raw, sizeof(xo_rco_conf0->raw));
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_HAL_LOG_ERROR("Failed to read XO_RCO_CONF1 register. Error %u", (uint32_t)err);
            break;
        }

        xo_rco_conf0->EXT_REF         = FALSE == drv_ctx->config->use_external_clock ? 0u : 1u,          /* Usage of the external clock is determined by the driver config */
        xo_rco_conf0->GM_CONF         = S2_LP_XO_GM_43_0,                                                /* Selected gain margin (GM) shall be at least 5x critical Gm for the selected crystal to guarantee XO start in all operating conditions. See AN2867 for more details */
        xo_rco_conf0->REFDIV          = drv_ctx->config->xin_freq < S2_LP_IC_REF_FREQ_THRESH ? 0u : 1u , /* Keep the reference frequency around 25MHz since it provides better resolution for RF frequency setting */
        xo_rco_conf0->EXT_RCO_OSC     = FALSE,                                                           /* Use built-in low-speed RC oscillator */
        xo_rco_conf0->RCO_CALIBRATION = FALSE,                                                           /* Don't do automatic RCO calibration */

        err = _spi_write_registers(drv_ctx, S2_LP_IC_REG_ADDR_XO_RCO_CONF0, &xo_rco_conf0->raw, sizeof(xo_rco_conf0->raw));
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_HAL_LOG_ERROR("S2-LP failed to modify XO_RCO_CONF0 register. Error %u", (uint32_t)err);
            break;
        }

        /* Cache the settings to speed up S2-LP frequency configuration */
        if (xo_rco_conf0->REFDIV != 0u)
        {
            /* Reference clock divider is active, reference clock is XTAL/2 */
            drv_ctx->ref_clk_div_enabled = TRUE;
            drv_ctx->ref_freq_hz         = drv_ctx->config->xin_freq >> 1;
        }
        else
        {
            /* Reference clock divider is inactive, reference clock equals to XTAL */
            drv_ctx->ref_clk_div_enabled = FALSE;
            drv_ctx->ref_freq_hz         = drv_ctx->config->xin_freq;
        }
        /*----------------------------------------------------------------------------*/

        /* Adjust SMPS settings ------------------------------------------------------*/
        s2_lp_hal_smps_cfg_regs_t smps_cfg;

        /* Read out PM_CONF registers since we need to keep the values in the reserved bits */
        err = _spi_read_registers(drv_ctx, S2_LP_IC_REG_ADDR_PM_CONF1, smps_cfg.raw, sizeof(smps_cfg.raw));
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_HAL_LOG_ERROR("Failed to read PM_CONF1-0 registers. Error %u", (uint32_t)err);
            break;
        }

        /* Apply settings */
        smps_cfg.pm_conf1.BATTERY_LVL_EN = 0u;                            /* Don'tuse built-in brown-out detector to save power */
        smps_cfg.pm_conf1.SMPS_LVL_MODE  = S2_LP_SMPS_LVL_MODE_TX_ONLY;   /* Keep Rx fixed at 1.4V */
        smps_cfg.pm_conf1.BYPASS_LDO     = 0u;                            /* Use built-in LDO when applicable */

        smps_cfg.pm_conf0.SET_SMPS_LVL   = S2_LP_SMPS_LVL_1_6V;           /* Use 1.6V for Tx operations */
        smps_cfg.pm_conf0.SLEEP_MODE_SEL = S2_LP_SLEEP_MODE_NO_RETENTION; /* This driver does not need FIFO retention */

        /* Write back SMPS config */
        err = _spi_write_registers(drv_ctx, S2_LP_IC_REG_ADDR_PM_CONF1, smps_cfg.raw, sizeof(smps_cfg.raw));
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_HAL_LOG_ERROR("Cannot write PM_CONF1-0 registers. Error %u", (uint32_t)err);
            break;
        }

        /* Cache settings for the future SMPS voltage adjustments */
        drv_ctx->regs_cache.pm_conf0_reg = smps_cfg.pm_conf0;

        /* Calculate KRM multipliers for adjusting SMPS frequency for Tx and Rx modes */
        drv_ctx->smps_krm_tx = S2LP_RADIO_HAL_SMPS_FREQ_TO_KRM(S2LP_RADIO_HAL_SMPS_FREQ_FOR_TX, drv_ctx->dig_freq_hz);
        drv_ctx->smps_krm_rx = S2LP_RADIO_HAL_SMPS_FREQ_TO_KRM(S2LP_RADIO_HAL_SMPS_FREQ_FOR_RX, drv_ctx->dig_freq_hz);
        /*----------------------------------------------------------------------------*/

        /* Adjust PM start timeout to compensate for crystal deviations --------------*/
        sl2_lp_ic_reg_timer_conf3_t timer_conf3_reg = {
            .PM_START_COUNTER = S2LP_RADIO_HAL_PM_START_COUNTER > S2_LP_IC_PM_START_COUNTER_UPPER_LIMIT ? S2_LP_IC_PM_START_COUNTER_UPPER_LIMIT : S2LP_RADIO_HAL_PM_START_COUNTER,
        };

        err = _spi_write_registers(drv_ctx, S2_LP_IC_REG_ADDR_TIMER_CONF3, &timer_conf3_reg.raw, sizeof(timer_conf3_reg.raw));
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_HAL_LOG_ERROR("S2-LP failed to set TIMER_CONF3 register. Error %u", (uint32_t)err);
            break;
        }
        /*----------------------------------------------------------------------------*/

        /* Go back to the READY state after XTAL and SMPS config is done -------------*/
        /* Send CMD_READY */
        err = _spi_send_command(drv_ctx, S2_LP_IC_CMD_READY);
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            break;
        }

        /* Wait until the actual transition to the READY state is finished */
        err = _wait_for_state_transition(drv_ctx, S2_LP_MC_STATE_READY, S2LP_RADIO_HAL_READY_STATE_CLK_CFG_TIMEOUT_US);
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_HAL_LOG_ERROR("S2-LP failed to reach READY state. Error %u", (uint32_t)err);
            break;
        }
        /*----------------------------------------------------------------------------*/

        /* Run RCO calibration -------------------------------------------------------*/
        err = s2_lp_radio_hal_calibrate_rco_sync(drv_ctx);
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_HAL_LOG_ERROR("Failed to calibrate RCO. Error %u", (uint32_t)err);
            break;
        }
        /*----------------------------------------------------------------------------*/

        /* Configure intermediate frequency ------------------------------------------*/
        err = _set_intermediate_frequency(drv_ctx, S2LP_RADIO_HAL_INTERMEDIATE_FREQUENCY_HZ);
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_HAL_LOG_ERROR("S2-LP failed to set intermediate frequency. Error %u", (uint32_t)err);
            break;
        }
        /*----------------------------------------------------------------------------*/

        err = S2_LP_RADIO_HAL_STATUS_OK;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED s2_lp_hal_status_t s2_lp_radio_hal_static_mod_params_init(halo_drv_s2_lp_ctx_t * const drv_ctx)
{
    s2_lp_hal_status_t err = S2_LP_RADIO_HAL_STATUS_ERROR_GENERIC;

    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->regional_radio_params != NULL);

    do
    {
        /* Cache the current settings for future use ---------------------------------*/
        sl2_lp_ic_reg_pckt_params_t * const      pckt_cfg_regs        = &drv_ctx->regs_cache.pckt_cfg_regs;
        sl2_lp_ic_reg_pa_cfg_t * const           pa_mode_cfg_regs     = &drv_ctx->regs_cache.pa_mode_cfg_regs;
        sl2_lp_ic_reg_protocol_t * const         protocol_regs        = &drv_ctx->regs_cache.protocol_regs;
        sl2_lp_ic_reg_pckt_flt_options_t * const pckt_flt_options_reg = &drv_ctx->regs_cache.pckt_flt_options_reg;

        err = _spi_read_registers(drv_ctx, S2_LP_IC_REG_ADDR_PCKTCTRL6, pckt_cfg_regs->raw, sizeof(pckt_cfg_regs->raw));
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_HAL_LOG_ERROR("Cannot read packet parameter registers. Error %u", (uint32_t)err);
            break;
        }

        err = _spi_read_registers(drv_ctx, S2_LP_IC_REG_ADDR_PA_POWER0, pa_mode_cfg_regs->raw, sizeof(pa_mode_cfg_regs->raw));
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_HAL_LOG_ERROR("Cannot read PA config registers. Error %u", (uint32_t)err);
            break;
        }

        err = _spi_read_registers(drv_ctx, S2_LP_IC_REG_ADDR_PROTOCOL2, protocol_regs->raw, sizeof(protocol_regs->raw));
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_HAL_LOG_ERROR("Cannot read PROTOCOLx registers. Error %u", (uint32_t)err);
            break;
        }

        err = _spi_read_registers(drv_ctx, S2_LP_IC_REG_ADDR_PCKT_FLT_OPTIONS, &pckt_flt_options_reg->raw, sizeof(pckt_flt_options_reg->raw));
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_HAL_LOG_ERROR("Cannot read PCKT_FLT_OPTIONS register. Error %u", (uint32_t)err);
            break;
        }

        /*----------------------------------------------------------------------------*/

        /* Clock recovery settings ---------------------------------------------------*/
        sl2_lp_ic_reg_clockrec_t clk_recovery_regs = {
            .clockrec2 = {
                .CLK_REC_P_GAIN_SLOW = 6u,                           /* Used after the Sync Word detection */
                .CLK_REC_ALGO_SEL    = S2_LP_CLKREC_PLL_MODE,
                .CLK_REC_I_GAIN_SLOW = 0u,                           /* Used after the Sync Word detection */
            },
            .clockrec1 = {
                .CLK_REC_P_GAIN_FAST = 2u,                           /* Used before the Sync Word detection */
                .PSTFLT_LEN          = S2_LP_CLKREC_POSTFLT_16_SMBL,
                .CLK_REC_I_GAIN_FAST = 8u,                           /* Used before the Sync Word detection */
            },
        };

        err = _spi_write_registers(drv_ctx, S2_LP_IC_REG_ADDR_CLOCKREC2, clk_recovery_regs.raw, sizeof(clk_recovery_regs));
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_HAL_LOG_ERROR("S2-LP failed to update CLOCKREC registers. Error %u", (uint32_t)err);
            break;
        }
        /*----------------------------------------------------------------------------*/

        /* PA settings ---------------------------------------------------------------*/
        pa_mode_cfg_regs->pa_power0.DIG_SMOOTH_EN = 0u;
        pa_mode_cfg_regs->pa_config1.FIR_EN       = 0u;

        err = _spi_write_registers(drv_ctx, S2_LP_IC_REG_ADDR_PA_POWER0, pa_mode_cfg_regs->raw, sizeof(pa_mode_cfg_regs->raw));
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_HAL_LOG_ERROR("Cannot write PA config registers. Error %u", (uint32_t)err);
            break;
        }
        /*----------------------------------------------------------------------------*/

        /* Auto Frequency Compensation (AFC) settings --------------------------------*/
        sl2_lp_ic_reg_afc_t afc_regs;

        /* Read out AFC2 register since we need to keep the values in the reserved bits */
        err = _spi_read_registers(drv_ctx, S2_LP_IC_REG_ADDR_AFC2, &afc_regs.afc2.raw, sizeof(afc_regs.afc2));
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_HAL_LOG_ERROR("Cannot read AFC2 register. Error %u", (uint32_t)err);
            break;
        }

        /* Apply AFC settings */
        afc_regs.afc2.AFC_ENABLED        = 1u; /* Enable the AFC correction */
        afc_regs.afc2.AFC_FREEZE_ON_SYNC = 1u; /* Enable the freeze AFC correction upon sync word detection */
        afc_regs.afc2.AFC_MODE           = S2_LP_AFC_MODE_CLOSE_ON_SLICER;

        afc_regs.afc1.AFC_FAST_PERIOD    = S2LP_RADIO_HAL_AFC_ANTICIPATED_MIN_PREAMBLE_LEN << 1; /* Since the algorithm operates typically on 2 samples per symbol, the programmed value shall be twice the number of preamble symbols */

        afc_regs.afc0.AFC_FAST_GAIN      = 2u; /* The AFC loop gain in fast mode (2's log) */
        afc_regs.afc0.AFC_SLOW_GAIN      = 5u; /* The AFC loop gain in slow mode (2's log) */

        /* Write back AFC config */
        err = _spi_write_registers(drv_ctx, S2_LP_IC_REG_ADDR_AFC2, afc_regs.raw, sizeof(afc_regs.raw));
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_HAL_LOG_ERROR("Cannot write AFCx registers. Error %u", (uint32_t)err);
            break;
        }

        /* Set RSSI threshold for AFC operation */
        /* Antenna and LNA gains shall not contribute to the AFC RSSI threshold - we need to adjust the default threshold to eliminate their gains */
        int32_t ant_and_pa_gain = drv_ctx->regional_radio_params->ant_dbi;
#if S2LP_RADIO_CFG_USE_EXTERNAL_PA
        ant_and_pa_gain += drv_ctx->config->pa_config.rx_gain_dbi;
#endif /* S2LP_RADIO_CFG_USE_EXTERNAL_PA */
        ant_and_pa_gain = ant_and_pa_gain >= 0 ? (ant_and_pa_gain + 50) / 100 : (ant_and_pa_gain - 50) / 100;

        const int32_t adjusted_rssi_th = S2LP_RADIO_HAL_RSSI_THRESHOLD_DBM + ant_and_pa_gain;
        sl2_lp_ic_reg_rssi_th_t rssi_th_reg = {
            .RSSI_TH = S2LP_RADIO_HAL_DBM_TO_LVL(adjusted_rssi_th),
        };
        err = _spi_write_registers(drv_ctx, S2_LP_IC_REG_ADDR_RSSI_TH, &rssi_th_reg.raw, sizeof(rssi_th_reg.raw));
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_HAL_LOG_ERROR("Cannot write RSSI_TH register. Error %u", (uint32_t)err);
            break;
        }
        /*----------------------------------------------------------------------------*/

        /* Configure AGC -------------------------------------------------------------*/
        sl2_lp_ic_reg_agcctrl_t agcctrl_regs;

         /* Read out AGCCTRL registers since we need to keep the values in the reserved bits */
        err = _spi_read_registers(drv_ctx, S2_LP_IC_REG_ADDR_AGCCTRL4, agcctrl_regs.raw, sizeof(agcctrl_regs.raw));
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_HAL_LOG_ERROR("Cannot read AGCCTRLx registers. Error %u", (uint32_t)err);
            break;
        }

        /* Apply AGC settings */
        agcctrl_regs.agcctrl4.LOW_THRESHOLD_0   = 5u;    /* Keep recommended setting from the datasheet */
        agcctrl_regs.agcctrl4.LOW_THRESHOLD_1   = 4u;    /* Keep recommended setting from the datasheet */

        agcctrl_regs.agcctrl3.LOW_THRESHOLD_SEL = 0x10u; /* Keep recommended setting from the datasheet */

        agcctrl_regs.agcctrl2.FREEZE_ON_SYNC    = 1u;    /* Freeze AGC on Sync Word detection */
        agcctrl_regs.agcctrl2.MEAS_TIME         = 3u;    /* Approx. 4us */

        agcctrl_regs.agcctrl1.HIGH_THRESHOLD    = 5u;    /* Keep recommended setting from the datasheet */

        agcctrl_regs.agcctrl0.AGC_ENABLE        = 1u;
        agcctrl_regs.agcctrl0.HOLD_TIME         = 12u;   /* Keep recommended setting from the datasheet */

        /* Write back AGC config */
        err = _spi_write_registers(drv_ctx, S2_LP_IC_REG_ADDR_AGCCTRL4, agcctrl_regs.raw, sizeof(agcctrl_regs.raw));
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_HAL_LOG_ERROR("Cannot write AGCCTRLx registers. Error %u", (uint32_t)err);
            break;
        }
        /*----------------------------------------------------------------------------*/

        /* FIFO thresholds -----------------------------------------------------------*/
        sl2_lp_ic_reg_fifo_config_t fifo_cfg_regs = {0};

        fifo_cfg_regs.fifo_config3.RX_AFTHR = S2_LP_IC_FIFO_LEVEL_TO_AFTHR(S2LP_RADIO_HAL_RX_FIFO_ALMOST_FULL_LEVEL);
        fifo_cfg_regs.fifo_config2.RX_AETHR = 0u; /* Not interested in this event */
        fifo_cfg_regs.fifo_config1.TX_AFTHR = 0u; /* Not interested in this event */
        fifo_cfg_regs.fifo_config0.TX_AETHR = S2LP_RADIO_HAL_TX_FIFO_ALMOST_EMPTY_LEVEL;

        err = _spi_write_registers(drv_ctx, S2_LP_IC_REG_ADDR_FIFO_CONFIG3, fifo_cfg_regs.raw, sizeof(fifo_cfg_regs.raw));
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_HAL_LOG_ERROR("Cannot write FIFO_CONFIGx registers. Error %u", (uint32_t)err);
            break;
        }
        /*----------------------------------------------------------------------------*/

        err = S2_LP_RADIO_HAL_STATUS_OK;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED s2_lp_hal_status_t s2_lp_radio_hal_calibrate_rco_sync(halo_drv_s2_lp_ctx_t * const drv_ctx)
{
    s2_lp_hal_status_t err = S2_LP_RADIO_HAL_STATUS_ERROR_GENERIC;

    SID_PAL_ASSERT(drv_ctx != NULL);

    do
    {
        /* 1. Initiate RCO calibration and wait for actual start */
        err = _initiate_rco_calibration(drv_ctx, TRUE);
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            /* Logs are provided by _initiate_rco_calibration() */
            break;
        }

        /* 2. Wait for the calibration to finish */
        sl2_lp_ic_reg_mc_state1_t mc_state1;
        uint32_t                  accumulated_wait_time = 0u;
        do
        {
#ifdef SID_PAL_ENABLE_SCHEDULER_DELAY
            if (FALSE == SID_STM32_UTIL_IS_IRQ())
            {
                /* We can use non-blocking OS delay */
                sid_pal_scheduler_delay_ms(S2LP_RADIO_HAL_RCO_CALIBRATION_PROBE_PERIOD_MS);
            }
            else
#endif /* SID_PAL_ENABLE_SCHEDULER_DELAY */
            {
                /* Block for the predefined amount of time */
                sid_pal_delay_us(S2LP_RADIO_HAL_RCO_CALIBRATION_PROBE_PERIOD_MS * 1000u);
            }

            /* Read out the status from MC_STATE registers and simultaneously store the status reported while sending header bytes */
            err = _spi_read_registers(drv_ctx, S2_LP_IC_REG_ADDR_MC_STATE1, &mc_state1.raw, sizeof(mc_state1));
            if (err != S2_LP_RADIO_HAL_STATUS_OK)
            {
                S2LP_RADIO_HAL_LOG_ERROR("RCO calibration - unable to retrieve S2-LP MC_STATE1 register");
                break;
            }

            /* calibration success condition as per the datasheet */
            if ((mc_state1.RCCAL_OK != FALSE) && (FALSE == mc_state1.ERROR_LOCK))
            {
                /* Calibration done */
                err = S2_LP_RADIO_HAL_STATUS_OK;
                break;
            }

            accumulated_wait_time += S2LP_RADIO_HAL_RCO_CALIBRATION_PROBE_PERIOD_MS;
        } while (accumulated_wait_time < S2LP_RADIO_HAL_RCO_CALIBRATION_TIMEOUT_MS);

        /* Check if the loop was terminated due to the timeout */
        if (accumulated_wait_time >= S2LP_RADIO_HAL_RCO_CALIBRATION_TIMEOUT_MS)
        {
            err = S2_LP_RADIO_HAL_STATUS_ERROR_TIMEOUT;
        }

        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            /* Forward any errors that took place during the calibration loop */
            break;
        }

        /* 3. Store offline calibration settings */
        err = _store_rco_calibration(drv_ctx);
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            /* Logs are provided by _store_rco_calibration() */
            break;
        }

        SID_PAL_LOG_DEBUG("S2-LP RCO calibration done");
        err = S2_LP_RADIO_HAL_STATUS_OK;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED s2_lp_hal_status_t s2_lp_radio_hal_calibrate_rco_async(halo_drv_s2_lp_ctx_t * const drv_ctx)
{
    s2_lp_hal_status_t err = S2_LP_RADIO_HAL_STATUS_ERROR_GENERIC;
    sid_error_t        sid_err;

    SID_PAL_ASSERT(drv_ctx != NULL);

    do
    {
        struct sid_timespec status_check_start_time;

        /* 1. Store the timestamp as soon as possible */
        sid_err = sid_pal_uptime_now(&status_check_start_time);
        if (sid_err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("RCO calibration failed. Unable to get current time. Error %d", (int32_t)sid_err);
            err = S2_LP_RADIO_HAL_STATUS_ERROR_HW;
            break;
        }

        /* 2. Initiate RCO calibration and wait for actual start */
        err = _initiate_rco_calibration(drv_ctx, FALSE);
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            /* Logs are provided by _initiate_rco_calibration() */
            break;
        }

        /* 3. Schedule timer for periodic RCO calibration status checks. Unfortunately, S2-LP has no suitable IRQ to monitor RCO calibration process, so we have to use timer for polling */
        /* Store start timestamp */
        drv_ctx->rco_calibration.start_ts   = status_check_start_time;

        /* Calculate timeout timestamp */
        drv_ctx->rco_calibration.timeout_ts = status_check_start_time;
        sid_time_add(&drv_ctx->rco_calibration.timeout_ts, &rco_calib_timeout);

        /* Schedule status check */
        sid_time_add(&status_check_start_time, &rco_calib_status_check_period);
        sid_err = sid_pal_timer_arm(&drv_ctx->rco_calibration.timer, SID_PAL_TIMER_PRIO_CLASS_PRECISE, &status_check_start_time, &rco_calib_status_check_period);
        if (sid_err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("RCO calibration failed. Unable to arm timer. Error %d", (int32_t)sid_err);
            err = S2_LP_RADIO_HAL_STATUS_ERROR_HW;
            break;
        }

        SID_PAL_LOG_DEBUG("S2-LP RCO async calibration initiated");
        err = S2_LP_RADIO_HAL_STATUS_OK;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED void s2_lp_radio_hal_rco_calib_timer_cb(void * arg, sid_pal_timer_t * originator)
{
    s2_lp_hal_status_t           err = S2_LP_RADIO_HAL_STATUS_ERROR_GENERIC;
    halo_drv_s2_lp_ctx_t * const drv_ctx = (halo_drv_s2_lp_ctx_t *)arg;

    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(originator != NULL);

    /* Use critical region for processing to avoid concurrent access to the SPI bus by the Sidewalk stack */
    sid_pal_enter_critical_region();
    do
    {
        sid_error_t sid_err;
        sl2_lp_ic_reg_mc_state1_t mc_state1;

        /* Read out the status from MC_STATE registers and simultaneously store the status reported while sending header bytes */
        err = _spi_read_registers(drv_ctx, S2_LP_IC_REG_ADDR_MC_STATE1, &mc_state1.raw, sizeof(mc_state1));
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_HAL_LOG_ERROR("RCO calibration - unable to retrieve S2-LP MC_STATE1 register");
            break;
        }

        /* calibration success condition as per the datasheet */
        if ((mc_state1.RCCAL_OK != FALSE) && (FALSE == mc_state1.ERROR_LOCK))
        {
            /* Calibration done */

            /* 1. Stop the timer */
            sid_err = sid_pal_timer_cancel(originator);
            SID_PAL_ASSERT(SID_ERROR_NONE == sid_err);

            /* 2. Store offline calibration settings */
            err = _store_rco_calibration(drv_ctx);
            if (err != S2_LP_RADIO_HAL_STATUS_OK)
            {
                /* Logs are provided by _store_rco_calibration() */
                break;
            }

            /* 3. Put radio into sleep after RCO is calibrated */
            /* Do a dummy IRQ status read to clear any residual IRQ flags */
            err = s2_lp_radio_hal_get_clear_irq_status(drv_ctx, NULL);
            if (err != S2_LP_RADIO_HAL_STATUS_OK)
            {
                SID_PAL_LOG_ERROR("Failed to clear S2-LP IRQ flags. HAL error %u", (uint32_t)err);
                break;
            }

            /* Transition S2-LP into STANDBY state (don't confuse with Sidewalks Standby - it corresponds to S2-LP's READY state) */
            err = s2_lp_radio_hal_set_standby(drv_ctx);
            if (err != S2_LP_RADIO_HAL_STATUS_OK)
            {
                SID_PAL_LOG_ERROR("Failed to put S2-LP into Sleep state. HAL error %u", (uint32_t)err);
                break;
            }

            err = S2_LP_RADIO_HAL_STATUS_OK;
        }
        else
        {
            struct sid_timespec now;

            /* Calibration is not done yet  - check for timeout */
            sid_err = sid_pal_uptime_now(&now);
            SID_PAL_ASSERT(SID_ERROR_NONE == sid_err);
            if (sid_time_gt(&now, &drv_ctx->rco_calibration.timeout_ts) != FALSE)
            {
                err = S2_LP_RADIO_HAL_STATUS_ERROR_TIMEOUT;
                break;
            }
            else
            {
                /* Proceed with waiting */
                err = S2_LP_RADIO_HAL_STATUS_OK;
            }
        }
    } while (0);
    sid_pal_exit_critical_region();

    if (err != S2_LP_RADIO_HAL_STATUS_OK)
    {
        SID_PAL_LOG_ERROR("S2-LP async RCO calibration failed. HAL error %u", (uint32_t)err);
    }
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED s2_lp_hal_status_t s2_lp_radio_hal_get_ic_version(const halo_drv_s2_lp_ctx_t * const drv_ctx, s2_lp_ic_version_info_t * const out_version_info)
{
    s2_lp_hal_status_t err;

    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(out_version_info != NULL);

    /* Read both DEVICE_INFO1 and DEVICE_INFO0 registers at once since they are adjacent */
    err = _spi_read_registers(drv_ctx, S2_LP_IC_REG_ADDR_DEVICE_INFO1, out_version_info->raw, sizeof(*out_version_info));

    if (err != S2_LP_RADIO_HAL_STATUS_OK)
    {
        S2LP_RADIO_HAL_LOG_ERROR("s2_lp_radio_hal_get_ic_version() failed with error code %u", (uint32_t)err);
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED s2_lp_hal_status_t s2_lp_radio_hal_set_ready(const halo_drv_s2_lp_ctx_t * const drv_ctx)
{
    s2_lp_hal_status_t err = S2_LP_RADIO_HAL_STATUS_ERROR_GENERIC;

    SID_PAL_ASSERT(drv_ctx != NULL);

    do
    {
        s2_lp_ic_status_t  ic_status;

#if SID_SDK_CONFIG_RF_TIMINGS_MEASUREMENT
        /* Indicate S2-LP wakeup start point - keep this code exactly here since S2-LP state readout takes longer when radio is in sleep */
        __COMPILER_BARRIER();
        if (SID_PAL_RADIO_SLEEP == drv_ctx->radio_state)
        {
            SIDEWALK_RF_TIMINGS_RXTX_START_GPIO_Port->BRR = SIDEWALK_RF_TIMINGS_RXTX_START_Pin;
        }
        __COMPILER_BARRIER();
#endif /* SID_SDK_CONFIG_RF_TIMINGS_MEASUREMENT */

        /* Read out actual state of the S2-LP */
        err = _spi_read_registers(drv_ctx, S2_LP_IC_REG_ADDR_MC_STATE1, ic_status.raw, sizeof(ic_status));
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            break;
        }

        if (S2_LP_MC_STATE_READY == ic_status.reg_mc_state0.MC_STATE)
        {
            /* S2-LP is already in the READY state, no need to issue any additional commands */
            err = S2_LP_RADIO_HAL_STATUS_OK;
            break;
        }

        /* Ensure S2-LP is in one of the states that allow transition to READY state */
        if ((ic_status.reg_mc_state0.MC_STATE != S2_LP_MC_STATE_STANDBY)
          && (ic_status.reg_mc_state0.MC_STATE != S2_LP_MC_STATE_SLEEP)
          && (ic_status.reg_mc_state0.MC_STATE != S2_LP_MC_STATE_LOCKON))
        {
            /* Send CMD_SABORT first to abort any ongoing activities, otherwise transition to the READY state won't be accepted by S2-LP */
            err = _spi_send_command(drv_ctx, S2_LP_IC_CMD_SABORT);
            if (err != S2_LP_RADIO_HAL_STATUS_OK)
            {
                break;
            }
        }

        /* Send CMD_READY */
        err = _spi_send_command(drv_ctx, S2_LP_IC_CMD_READY);
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            break;
        }

        /* Wait until the actual transition to the READY state is finished */
        err = _wait_for_state_transition(drv_ctx, S2_LP_MC_STATE_READY, S2LP_RADIO_HAL_STATE_TRANSITION_DEFAULT_TIMEOUT_US);
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_HAL_LOG_ERROR("S2-LP failed to reach READY state. Error %u", (uint32_t)err);
            break;
        }

#if SID_SDK_CONFIG_RF_TIMINGS_MEASUREMENT
        /* Indicate S2-LP wakeup done point - keep this code exactly here since all the below lines are always executed, event if there's no need to wake up S2-LP */
        __COMPILER_BARRIER();
        SIDEWALK_RF_TIMINGS_RXTX_START_GPIO_Port->BSRR = SIDEWALK_RF_TIMINGS_RXTX_START_Pin;
        __COMPILER_BARRIER();
#endif /* SID_SDK_CONFIG_RF_TIMINGS_MEASUREMENT */

        err = S2_LP_RADIO_HAL_STATUS_OK;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED s2_lp_hal_status_t s2_lp_radio_hal_set_standby(const halo_drv_s2_lp_ctx_t * const drv_ctx)
{
    s2_lp_hal_status_t err = S2_LP_RADIO_HAL_STATUS_ERROR_GENERIC;

    SID_PAL_ASSERT(drv_ctx != NULL);

    do
    {
        s2_lp_ic_status_t  ic_status;

        /* Read out actual state of the S2-LP */
        err = _spi_read_registers(drv_ctx, S2_LP_IC_REG_ADDR_MC_STATE1, ic_status.raw, sizeof(ic_status));
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            break;
        }

        if (S2_LP_MC_STATE_STANDBY == ic_status.reg_mc_state0.MC_STATE)
        {
            /* S2-LP is already in the STANDBY state, no need to issue any additional commands */
            err = S2_LP_RADIO_HAL_STATUS_OK;
            break;
        }

        /* Ensure S2-LP is in one of the states that allow transition to STANDBY state */
        if ((ic_status.reg_mc_state0.MC_STATE != S2_LP_MC_STATE_READY)
          && (ic_status.reg_mc_state0.MC_STATE != S2_LP_MC_STATE_SYNTH_SETUP))
        {
            /* Move S2-LP to READY state first */
            err = s2_lp_radio_hal_set_ready(drv_ctx);
            if (err != S2_LP_RADIO_HAL_STATUS_OK)
            {
                break;
            }
        }

        /* Send CMD_STANDBY */
        err = _spi_send_command(drv_ctx, S2_LP_IC_CMD_STANDBY);
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            break;
        }

        /* Wait until the actual transition to the STANDBY state is finished */
        err = _wait_for_state_transition(drv_ctx, S2_LP_MC_STATE_STANDBY, S2LP_RADIO_HAL_STATE_TRANSITION_DEFAULT_TIMEOUT_US);
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_HAL_LOG_ERROR("S2-LP failed to reach STANDBY state. Error %u", (uint32_t)err);
            break;
        }

        err = S2_LP_RADIO_HAL_STATUS_OK;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED s2_lp_hal_status_t s2_lp_radio_hal_set_base_frequency(halo_drv_s2_lp_ctx_t * const drv_ctx, const uint32_t freq_hz, const uint32_t ch_spacing)
{
    s2_lp_hal_status_t err = S2_LP_RADIO_HAL_STATUS_ERROR_GENERIC;

    SID_PAL_ASSERT(drv_ctx != NULL);

    do
    {
        s2_lp_ic_reg_synt_t           synt_regs       = {0};
        sl2_lp_ic_reg_synth_config2_t synth_cfg2_reg;

        /* Validate the input */
        if (FALSE == S2_LP_IC_IS_VALID_FREQ_BAND(freq_hz))
        {
            S2LP_RADIO_HAL_LOG_ERROR("Cannot set base frequency to %uHz. Requested value is out of range", freq_hz);
            err = S2_LP_RADIO_HAL_STATUS_INVALID_ARGS;
            break;
        }

        /* Read out SYNTH_CONFIG2 register to modify it */
        err = _spi_read_registers(drv_ctx, S2_LP_IC_REG_ADDR_SYNTH_CONFIG2, &synth_cfg2_reg.raw, sizeof(synth_cfg2_reg));
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_HAL_LOG_ERROR("Cannot read SYNTH_CONFIG2 register. Error %u", (uint32_t)err);
            break;
        }

        /* Determine Charge Pump settings based on the desired RF frequency */
        _compute_charge_pump_settings(drv_ctx, freq_hz, &synt_regs.synt3, &synth_cfg2_reg);

        /* Calculate the number of PLL steps for the desired frequency */
        _compute_synth_pll_steps(drv_ctx, freq_hz, &synt_regs);

        /* Apply the settings */
        err = _spi_write_registers(drv_ctx, S2_LP_IC_REG_ADDR_SYNTH_CONFIG2, &synth_cfg2_reg.raw, sizeof(synth_cfg2_reg));
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_HAL_LOG_ERROR("Cannot modify SYNTH_CONFIG2 register. Error %u", (uint32_t)err);
            break;
        }

        err = _spi_write_registers(drv_ctx, S2_LP_IC_REG_ADDR_SYNT3, synt_regs.raw, sizeof(synt_regs));
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_HAL_LOG_ERROR("Cannot modify SYNT3..0 registers. Error %u", (uint32_t)err);
            break;
        }

        /* Update channel spacing settings */
        sl2_lp_ic_reg_chspace_t chspace_reg = {
            .CH_SPACE = (uint8_t)((uint64_t)(((uint64_t)ch_spacing << 15) + (drv_ctx->config->xin_freq >> 1)) / (uint64_t)drv_ctx->config->xin_freq),
        };

        err = _spi_write_registers(drv_ctx, S2_LP_IC_REG_ADDR_CHSPACE, &chspace_reg.raw, sizeof(chspace_reg));
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_HAL_LOG_ERROR("Cannot modify CHSPACE register. Error %u", (uint32_t)err);
            break;
        }

        /* Cache settings to ease modulation param updates */
        drv_ctx->radio_freq_hz          = freq_hz;
        drv_ctx->radio_freq_band_factor = FALSE == S2_LP_IC_IS_FREQ_BAND_HIGH(freq_hz) ? S2_LP_IC_MIDDLE_BAND_FACTOR : S2_LP_IC_HIGH_BAND_FACTOR;

        /* Done */
        err = S2_LP_RADIO_HAL_STATUS_OK;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED s2_lp_hal_status_t s2_lp_radio_hal_set_rf_channel(halo_drv_s2_lp_ctx_t * const drv_ctx, const uint32_t ch_num)
{
    s2_lp_hal_status_t err = S2_LP_RADIO_HAL_STATUS_ERROR_GENERIC;

    SID_PAL_ASSERT(drv_ctx != NULL);

    do
    {
        sl2_lp_ic_reg_chnum_t chnum_reg = {
            .CH_NUM = (uint8_t)ch_num,
        };

        err = _spi_write_registers(drv_ctx, S2_LP_IC_REG_ADDR_CHNUM, &chnum_reg.raw, sizeof(chnum_reg));
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_HAL_LOG_ERROR("Cannot modify CHNUM register. Error %u", (uint32_t)err);
            break;
        }

        /* Done */
        err = S2_LP_RADIO_HAL_STATUS_OK;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED s2_lp_hal_status_t s2_lp_radio_hal_set_syncword(halo_drv_s2_lp_ctx_t * const drv_ctx, const uint8_t * const sync_word, const uint32_t sync_word_length, const s2_lp_hal_sync_word_selector_t sync_word_sel)
{
    s2_lp_hal_status_t err = S2_LP_RADIO_HAL_STATUS_ERROR_GENERIC;

    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(sync_word != NULL);
    SID_PAL_ASSERT(sync_word_length > 0u);

    do
    {
        sl2_lp_ic_reg_pcktctrl6_t * const pcktctrl6_reg = &drv_ctx->regs_cache.pckt_cfg_regs.pcktctrl6;
        sl2_lp_ic_reg_sync_t      sync_word_regs        = {0};
        const uint32_t            sync_word_len_bits    = sync_word_length << 3; /* sync_word_len_bits = sync_word_length * 8u */

        if (sync_word_len_bits > S2_LP_IC_SYNC_WORD_LENGTH_LIMIT_BITS)
        {
            SID_PAL_LOG_ERROR("Sync word of %u bits exceeds capabilities of S2-LP (%u bits)", sync_word_len_bits, S2_LP_IC_SYNC_WORD_LENGTH_LIMIT_BITS);
            err = S2_LP_RADIO_HAL_STATUS_ERROR_DATA_TOO_LONG;
            break;
        }

        /* Prepare sync word buffer */
        for (uint32_t i = 0u; i < sync_word_length; i++)
        {
            /* IMPORTANT: S2-LP transmits/checks the Sync Word from SYNC0 to SYNC3 register - we need to write the sync word bytes in the reverse order */
            sync_word_regs.raw[(SID_STM32_UTIL_ARRAY_SIZE(sync_word_regs.raw) - 1u) - i] = sync_word[i];
        }

        /* Set the new Sync Word length in bits */
        pcktctrl6_reg->SYNC_LEN = (uint8_t)sync_word_len_bits;

        /* Write back PCKTCTRL6 register */
        err = _spi_write_registers(drv_ctx, S2_LP_IC_REG_ADDR_PCKTCTRL6, &pcktctrl6_reg->raw, sizeof(*pcktctrl6_reg));
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_HAL_LOG_ERROR("Cannot modify PCKTCTRL6 register. Error %u", (uint32_t)err);
            break;
        }

        if (S2_LP_RADIO_HAL_SYNC_WORD_PRIMARY == sync_word_sel)
        {
            /* Write Sync Word itself to SYNCx registers */
            err = _spi_write_registers(drv_ctx, S2_LP_IC_REG_ADDR_SYNC3, sync_word_regs.raw, sizeof(sync_word_regs));
            if (err != S2_LP_RADIO_HAL_STATUS_OK)
            {
                S2LP_RADIO_HAL_LOG_ERROR("Cannot write to SYNC3..0 registers. Error %u", (uint32_t)err);
                break;
            }
        }
        else
        {
            /* Write Sync Word to SEC_SYNCx registers - these will be used for Tx/Rx operations in 802.15.4g mode when FEC is enabled */
            err = _spi_write_registers(drv_ctx, S2_LP_IC_REG_ADDR_SEC_SYNC3, sync_word_regs.raw, sizeof(sync_word_regs));
            if (err != S2_LP_RADIO_HAL_STATUS_OK)
            {
                S2LP_RADIO_HAL_LOG_ERROR("Cannot write to SEC_SYNC3..0 registers. Error %u", (uint32_t)err);
                break;
            }
        }

        /* Done */
        err = S2_LP_RADIO_HAL_STATUS_OK;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED s2_lp_hal_status_t s2_lp_radio_hal_set_mod_params(halo_drv_s2_lp_ctx_t * const drv_ctx, const s2_lp_hal_mod_params_t * const mod_params)
{
    s2_lp_hal_status_t err = S2_LP_RADIO_HAL_STATUS_ERROR_GENERIC;

    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(mod_params != NULL);

    do
    {
        sl2_lp_ic_reg_mod_params_t mod_cfg_regs = {0};

        /* Validate the inputs */
        if (FALSE == S2_LP_IC_IS_VALID_DATARATE(mod_params->bit_rate_bps, drv_ctx->dig_freq_hz))
        {
            SID_PAL_LOG_ERROR("Cannot set datarate to %ubps - out of configurable range", mod_params->bit_rate_bps);
            err = S2_LP_RADIO_HAL_STATUS_INVALID_ARGS;
            break;
        }
        if (FALSE == S2_LP_IC_IS_VALID_CH_BW(mod_params->rx_filter_bandwidth_hz, drv_ctx->config->xin_freq))
        {
            SID_PAL_LOG_ERROR("Cannot set Rx filter bandwidth to %uHz - out of configurable range", mod_params->rx_filter_bandwidth_hz);
            err = S2_LP_RADIO_HAL_STATUS_INVALID_ARGS;
            break;
        }
        if (FALSE == S2_LP_IC_IS_VALID_FDEV(mod_params->fdev_in_hz, drv_ctx->config->xin_freq))
        {
            SID_PAL_LOG_ERROR("Cannot set Fdev to %uHz - out of configurable range", mod_params->fdev_in_hz);
            err = S2_LP_RADIO_HAL_STATUS_INVALID_ARGS;
            break;
        }

        /* Compute the register values for the desired bitrate */
        _find_datarate_params(drv_ctx, mod_params->bit_rate_bps, &mod_cfg_regs);

        /* Set modulation type */
        mod_cfg_regs.mod2.MOD_TYPE = mod_params->mod_type;

        /* Static settings */
        mod_cfg_regs.mod1.PA_INTERP_EN  = 1u; /* Always keep power amplifier interpolation on as this will result in smoother ramping (ramp curve wiil use linear interpolation between the refrence points instead of instant switching between the different levels) */
        mod_cfg_regs.mod1.MOD_INTERP_EN = 1u; /* Always keep it enabled, the radio will automatically ignore this setting if modulation interpolation is not applicable */
        mod_cfg_regs.mod1.CONST_MAP     = 0u; /* Keep the default setting, Sidewalk does not require constellation swapping */

        /* Compute frequency deviation settings */
        _find_fdev_params(drv_ctx, mod_params->fdev_in_hz, &mod_cfg_regs);

        /* Use LUT to determine Rx channel filter configuration */
        mod_cfg_regs.chflt = _find_rx_ch_bandwidth_params(drv_ctx, mod_params->rx_filter_bandwidth_hz);

        /* Write down all the modulation parameter settings into the radio */
        err = _spi_write_registers(drv_ctx, S2_LP_IC_REG_ADDR_MOD4, mod_cfg_regs.raw, sizeof(mod_cfg_regs));
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_HAL_LOG_ERROR("Cannot write modulation parameter registers. Error %u", (uint32_t)err);
            break;
        }

        /* Configure PA bessel filter */
        sl2_lp_ic_reg_pa_config0_t * const pa_config0_reg = &drv_ctx->regs_cache.pa_mode_cfg_regs.pa_config0;
        if (mod_params->bit_rate_bps < 16000u)
        {
            pa_config0_reg->PA_FC = S2_LP_PA_FILT_BW_12_5_KHZ;
        }
        else if (mod_params->bit_rate_bps < 32000u)
        {
            pa_config0_reg->PA_FC = S2_LP_PA_FILT_BW_25_KHZ;
        }
        else if (mod_params->bit_rate_bps < 62500u)
        {
            pa_config0_reg->PA_FC = S2_LP_PA_FILT_BW_50_KHZ;
        }
        else
        {
            pa_config0_reg->PA_FC = S2_LP_PA_FILT_BW_100_KHZ;
        }

        err = _spi_write_registers(drv_ctx, S2_LP_IC_REG_ADDR_PA_CONFIG0, &pa_config0_reg->raw, sizeof(pa_config0_reg->raw));
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_HAL_LOG_ERROR("Cannot write PA_CONFIG0 register. Error %u", (uint32_t)err);
            break;
        }

        /* Done */
        drv_ctx->current_bit_rate = mod_params->bit_rate_bps;
        err = S2_LP_RADIO_HAL_STATUS_OK;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED s2_lp_hal_status_t s2_lp_radio_hal_set_pckt_params(halo_drv_s2_lp_ctx_t * const drv_ctx, const s2_lp_hal_pckt_params_t * const pckt_params)
{
    s2_lp_hal_status_t err = S2_LP_RADIO_HAL_STATUS_ERROR_GENERIC;

    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(pckt_params != NULL);

    do
    {
        sl2_lp_ic_reg_pckt_params_t * const pckt_cfg_regs = &drv_ctx->regs_cache.pckt_cfg_regs;

        /* Preamble length */
        uint32_t preamble_len_pairs               = pckt_params->preamble_len_bits >> 1;
        pckt_cfg_regs->pcktctrl6.PREAMBLE_LEN_9_8 = (uint8_t)(preamble_len_pairs >> 8);
        pckt_cfg_regs->pcktctrl5.PREAMBLE_LEN_7_0 = (uint8_t) preamble_len_pairs;

        /* Packet preamble and header-related settings */
        pckt_cfg_regs->pcktctrl4.LEN_WID          = S2_LP_LEN_WID_2_BYTES;   /* This setting is static for 802.15.4g packet format */
        pckt_cfg_regs->pcktctrl4.ADDRESS_LEN      = 0u;                      /* Addressing is not used for Sidewalk */
        pckt_cfg_regs->pcktctrl3.FSK4_SYM_SWAP    = 0u;                      /* Irrelevant since we are using 2-GFSK, not 4-(G)FSK */
        pckt_cfg_regs->pcktctrl3.BYTE_SWAP        = 0u;                      /* Always send MSB first */
        pckt_cfg_regs->pcktctrl3.PREAMBLE_SEL     = S2_LP_PREAMBLE_SEL_0x55; /* Sidewalk uses 0x55 as preamble */

        /* Packet format settings */
        pckt_cfg_regs->pcktctrl3.PCKT_FRMT        = S2_LP_PCKT_FRMT_802_15_4G; /* Sidewalk FSK frame format follows 802.15.4g */
        pckt_cfg_regs->pcktctrl2.FCS_TYPE_4G      = (S2_LP_CRC_MODE_CCITT_16 == pckt_params->crc_mode) ? S2_LP_FCS_TYPE_1 : S2_LP_FCS_TYPE_0;
        pckt_cfg_regs->pcktctrl2.FEC_TYPE_4G      = S2_LP_FEC_TYPE_NRNSC;      /* Use NRNSC encoder */
        pckt_cfg_regs->pcktctrl2.INT_EN_4G        = 0u; /* Don't use interleaving for Sidewalk */
        pckt_cfg_regs->pcktctrl2.FIX_VAR_LEN      = (pckt_params->variable_packet_len_en != 0u) ? 1u : 0u;
        pckt_cfg_regs->pcktctrl1.CRC_MODE         = pckt_params->crc_mode;
        pckt_cfg_regs->pcktctrl1.WHIT_EN          = (pckt_params->data_whitening_en != 0u) ? 1u : 0u;

        /* Rx/Rx buffer handling */
        pckt_cfg_regs->pcktctrl3.RX_MODE          = S2_LP_RX_MODE_NORMAL;
        pckt_cfg_regs->pcktctrl1.TXSOURCE         = S2_LP_TX_MODE_NORMAL;

        /* Configure packet length */
        pckt_cfg_regs->pcktlen1.PCKTLEN_15_8      = (uint8_t)(pckt_params->packet_length >> 8);
        pckt_cfg_regs->pcktlen0.PCKTLEN_7_0       = (uint8_t) pckt_params->packet_length;

        /* Write down all the packet parameter settings into the radio */
        err = _spi_write_registers(drv_ctx, S2_LP_IC_REG_ADDR_PCKTCTRL6, pckt_cfg_regs->raw, sizeof(pckt_cfg_regs->raw));
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_HAL_LOG_ERROR("Cannot write packet parameter registers. Error %u", (uint32_t)err);
            break;
        }

        /* Adjust the minimum amount of bits for preamble validity check */
        sl2_lp_ic_reg_qi_t qi_reg = {
            .PQI_TH = (uint8_t)pckt_params->pqi_threshold,
            .SQI_EN = 1u, /* It is recommended to keep the SQI check always enabled */
            .SQI_TH = 0u, /* Expect a perfect match for the sync word */
        };
        err = _spi_write_registers(drv_ctx, S2_LP_IC_REG_ADDR_QI, &qi_reg.raw, sizeof(qi_reg));
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_HAL_LOG_ERROR("Cannot write QI register. Error %u", (uint32_t)err);
            break;
        }

        /* Done */
        err = S2_LP_RADIO_HAL_STATUS_OK;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED s2_lp_hal_status_t s2_lp_radio_hal_flush_fifo(const halo_drv_s2_lp_ctx_t * const drv_ctx, const s2_lp_hal_fifo_selector_t fifo_selector)
{
    s2_lp_hal_status_t err = S2_LP_RADIO_HAL_STATUS_ERROR_GENERIC;

    SID_PAL_ASSERT(drv_ctx != NULL);

    do
    {
        switch (fifo_selector)
        {
            case S2_LP_RADIO_HAL_FIFO_RX:
            /* Flush Rx FIFO */
            err = _spi_send_command(drv_ctx, S2_LP_IC_CMD_FLUSHRXFIFO);
            break;
 
            case S2_LP_RADIO_HAL_FIFO_TX:
                /* Flush Tx FIFO */
                err = _spi_send_command(drv_ctx, S2_LP_IC_CMD_FLUSHTXFIFO);
                break;

            case S2_LP_RADIO_HAL_FIFO_BOTH:
                err = _spi_send_command(drv_ctx, S2_LP_IC_CMD_FLUSHRXFIFO);
                if (err != S2_LP_RADIO_HAL_STATUS_OK)
                {
                    break;
                }
                err = _spi_send_command(drv_ctx, S2_LP_IC_CMD_FLUSHTXFIFO);
                break;

            default:
                err = S2_LP_RADIO_HAL_STATUS_INVALID_ARGS;
                break;
        }

        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
             S2LP_RADIO_HAL_LOG_ERROR("Failed to flush FIFO(s). Error %u", (uint32_t)err);
            break;
        }

        err = S2_LP_RADIO_HAL_STATUS_OK;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED s2_lp_hal_status_t s2_lp_radio_hal_configure_packet_engine_for_rx(halo_drv_s2_lp_ctx_t * const drv_ctx, const s2_lp_ic_rx_timeout_stop_condition_t timeout_stop_condition)
{
    s2_lp_hal_status_t err = S2_LP_RADIO_HAL_STATUS_ERROR_GENERIC;

    SID_PAL_ASSERT(drv_ctx != NULL);

    do
    {
        sl2_lp_ic_reg_protocol2_t * const        protocol2_reg     = &drv_ctx->regs_cache.protocol_regs.protocol2;
        sl2_lp_ic_reg_pckt_flt_options_t * const pckt_flt_opts_reg = &drv_ctx->regs_cache.pckt_flt_options_reg;

        /* Modify stop conditions in PROTOCOL2 reg */
        protocol2_reg->raw &= ~(S2_LP_IC_REG_PROTOCOL2_MASK_CS_TIMEOUT | S2_LP_IC_REG_PROTOCOL2_MASK_SQI_TIMEOUT | S2_LP_IC_REG_PROTOCOL2_MASK_PQI_TIMEOUT);
        protocol2_reg->raw |= (((uint8_t)(timeout_stop_condition & 0x07u)) << 5);

        /* Configure IRQ MUX to trigger IRQs for the Rx FIFO */
        protocol2_reg->FIFO_GPIO_OUT_MUX_SEL = S2_LP_FIFO_GPIO_OUT_MUX_SEL_RX;

        /* Apply new settings */
        err = _spi_write_registers(drv_ctx, S2_LP_IC_REG_ADDR_PROTOCOL2, &protocol2_reg->raw, sizeof(protocol2_reg->raw));
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            break;
        }

        /* Modify AND / OR boolean relationship in PCKT_FLT_OPTIONS reg */
        pckt_flt_opts_reg->RX_TIMEOUT_AND_OR_SEL = ((uint8_t)timeout_stop_condition & 0x08u) != 0u ? 1u : 0u;

        err = _spi_write_registers(drv_ctx, S2_LP_IC_REG_ADDR_PCKT_FLT_OPTIONS, &pckt_flt_opts_reg->raw, sizeof(pckt_flt_opts_reg->raw));
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            break;
        }

        err = S2_LP_RADIO_HAL_STATUS_OK;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED s2_lp_hal_status_t s2_lp_radio_hal_configure_packet_engine_for_tx(halo_drv_s2_lp_ctx_t * const drv_ctx)
{
    s2_lp_hal_status_t err = S2_LP_RADIO_HAL_STATUS_ERROR_GENERIC;

    SID_PAL_ASSERT(drv_ctx != NULL);

    do
    {
        sl2_lp_ic_reg_protocol2_t * const protocol2_reg = &drv_ctx->regs_cache.protocol_regs.protocol2;

        /* Configure IRQ MUX to trigger IRQs for the Tx FIFO */
        protocol2_reg->FIFO_GPIO_OUT_MUX_SEL = S2_LP_FIFO_GPIO_OUT_MUX_SEL_TX;

        /* Apply new settings */
        err = _spi_write_registers(drv_ctx, S2_LP_IC_REG_ADDR_PROTOCOL2, &protocol2_reg->raw, sizeof(protocol2_reg->raw));
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            break;
        }

        err = S2_LP_RADIO_HAL_STATUS_OK;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED s2_lp_hal_status_t s2_lp_radio_hal_configure_packet_engine_for_cw(halo_drv_s2_lp_ctx_t * const drv_ctx)
{
    s2_lp_hal_status_t err = S2_LP_RADIO_HAL_STATUS_ERROR_GENERIC;

    SID_PAL_ASSERT(drv_ctx != NULL);

    do
    {
        sl2_lp_ic_reg_protocol2_t * const protocol2_reg = &drv_ctx->regs_cache.protocol_regs.protocol2;

        /* Configure IRQ MUX to trigger IRQs for the Tx FIFO */
        protocol2_reg->FIFO_GPIO_OUT_MUX_SEL = S2_LP_FIFO_GPIO_OUT_MUX_SEL_TX;

        /* Apply new settings */
        err = _spi_write_registers(drv_ctx, S2_LP_IC_REG_ADDR_PROTOCOL2, &protocol2_reg->raw, sizeof(protocol2_reg->raw));
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            break;
        }

        /* Modify MOD2 reg to disable modulation. IMPORTANT: keep the datarate exponent for Tx ramp up to work correctly */
        sl2_lp_ic_reg_mod2_t mod2_reg;

        err = _spi_read_registers(drv_ctx, S2_LP_IC_REG_ADDR_MOD2, &mod2_reg.raw, sizeof(mod2_reg.raw));
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            break;
        }

        mod2_reg.MOD_TYPE = S2_LP_MOD_TYPE_UMODULATED;

        err = _spi_write_registers(drv_ctx, S2_LP_IC_REG_ADDR_MOD2, &mod2_reg.raw, sizeof(mod2_reg.raw));
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            break;
        }

        err = S2_LP_RADIO_HAL_STATUS_OK;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED s2_lp_hal_status_t s2_lp_radio_hal_set_rx_timeout(const halo_drv_s2_lp_ctx_t * const drv_ctx, const uint32_t timeout_us)
{
    s2_lp_hal_status_t err = S2_LP_RADIO_HAL_STATUS_ERROR_GENERIC;

    SID_PAL_ASSERT(drv_ctx != NULL);

    do
    {
        sl2_lp_ic_reg_rx_timer_cfg_t timer_regs;

        /* N cycles in the time base of the timer:
         * - clock of the timer is f_dig/1210
         * - use Fdig in MHz since the time is in microseconds
         * - ensure the calculated ticks number gives a timeout that is not less than requested (round up)
         */
        const uint32_t timeout_ticks = ((timeout_us * drv_ctx->dig_freq_mhz) + (S2_LP_IC_RX_TIMER_PREDIV - 1u)) / S2_LP_IC_RX_TIMER_PREDIV; 

        /* Check if it is possible to reach that target with prescaler and counter of S2-LP */
        if (((timeout_ticks + (UINT8_MAX - 1u)) / UINT8_MAX) >= 256u)
        {
            /* Set the maximum possible delay */
            timer_regs.timers4.RX_TIMER_PRESC = UINT8_MAX;
            timer_regs.timers5.RX_TIMER_CNTR  = UINT8_MAX;
        }
        else
        {
            /* Compute prescaler and counter values to match the desired timeout */
            const uint32_t tmp_prescaler = ((timeout_ticks + (UINT8_MAX - 1u)) / UINT8_MAX) + 2u;
            timer_regs.timers5.RX_TIMER_CNTR = (uint8_t)((timeout_ticks + (tmp_prescaler >> 1)) / tmp_prescaler);

            /* Decrement prescaler and counter according to the logic of this timer in S2-LP */
            timer_regs.timers4.RX_TIMER_PRESC = (uint8_t)(tmp_prescaler - 1u);

            /* Counter can't be zero */
            if (0u == timer_regs.timers5.RX_TIMER_CNTR)
            {
                timer_regs.timers5.RX_TIMER_CNTR = 1u;
            }
        }

        /* Write down Rx timer config */
        err = _spi_write_registers(drv_ctx, S2_LP_IC_REG_ADDR_TIMERS5, timer_regs.raw, sizeof(timer_regs));
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            break;
        }

        err = S2_LP_RADIO_HAL_STATUS_OK;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED s2_lp_hal_status_t s2_lp_radio_hal_set_ldc_timeout(halo_drv_s2_lp_ctx_t * const drv_ctx, const uint32_t timeout_us)
{
    s2_lp_hal_status_t err = S2_LP_RADIO_HAL_STATUS_ERROR_GENERIC;

    SID_PAL_ASSERT(drv_ctx != NULL);

    do
    {
        sl2_lp_ic_reg_ldc_wakeup_timer_cfg_t timer_regs;
        uint32_t ldc_clk_div = UINT32_MAX;

        /* Select LDC timer clock divider */
        const uint32_t rco_freq = _get_rco_frequency(drv_ctx);
        const uint32_t clk_threshold = (uint32_t)(((uint64_t)1000000u*(uint64_t)65536u)/(uint64_t)rco_freq);
        for(uint32_t i = 0; i < 4u; i++)
        {
            if (timeout_us < (clk_threshold << i))
            {
                ldc_clk_div = i;
                break;
            }
        }

        if (UINT32_MAX == ldc_clk_div)
        {
            SID_PAL_LOG_ERROR("Cannot set LDC wakeup time to %uus - requested time is out of range", timeout_us);
            err = S2_LP_RADIO_HAL_STATUS_ERROR_OUT_OF_RANGE;
            break;
        }

        /* N cycles in the time base of the timer:
         * - clock of the timer is RCO frequency
         * - divide times 1000000 more because we have an input in us
         */
        const uint32_t wakeup_time_ticks = (uint32_t)(((uint64_t)timeout_us * (uint64_t)(rco_freq >> ldc_clk_div)) / (uint64_t)1000000u);

        /* Check if it is possible to reach that target with prescaler and counter of S2-LP */
        if (((wakeup_time_ticks + (UINT8_MAX - 1u)) / UINT8_MAX) >= 256u)
        {
            /* Set the maximum possible delay */
            timer_regs.timers3.LDC_TIMER_PRESC = UINT8_MAX;
            timer_regs.timers2.LDC_TIMER_CNTR  = UINT8_MAX;
        }
        else
        {
            /* Compute prescaler and counter values to match the desired timeout */
            const uint32_t tmp_prescaler = ((wakeup_time_ticks + (UINT8_MAX - 1u)) / UINT8_MAX) + 2u;
            timer_regs.timers2.LDC_TIMER_CNTR = (uint8_t)((wakeup_time_ticks + (tmp_prescaler >> 1)) / tmp_prescaler);

            /* Decrement prescaler and counter according to the logic of this timer in S2-LP */
            timer_regs.timers3.LDC_TIMER_PRESC = (uint8_t)(tmp_prescaler - 1u);

            /* Counter can't be zero */
            if (0u == timer_regs.timers2.LDC_TIMER_CNTR)
            {
                timer_regs.timers2.LDC_TIMER_CNTR = 1u;
            }
        }

        /* Set the LDC timer divider */
        sl2_lp_ic_reg_protocol2_t protocol2_reg = drv_ctx->regs_cache.protocol_regs.protocol2;
        protocol2_reg.LDC_TIMER_MULT = (uint8_t)ldc_clk_div;

        /* Write configuration */
        err = _spi_write_registers(drv_ctx, S2_LP_IC_REG_ADDR_PROTOCOL2, &protocol2_reg.raw, sizeof(protocol2_reg.raw));
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_HAL_LOG_ERROR("Cannot write PROTOCOL2 register. Error %u", (uint32_t)err);
            break;
        }

        /* Update register cache */
        drv_ctx->regs_cache.protocol_regs.protocol2 = protocol2_reg;

         /* Write down LDC timer config */
         err = _spi_write_registers(drv_ctx, S2_LP_IC_REG_ADDR_TIMERS3, timer_regs.raw, sizeof(timer_regs));
         if (err != S2_LP_RADIO_HAL_STATUS_OK)
         {
             break;
         }

         err = S2_LP_RADIO_HAL_STATUS_OK;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED s2_lp_hal_status_t s2_lp_radio_hal_get_clear_irq_status(const halo_drv_s2_lp_ctx_t * const drv_ctx, sl2_lp_ic_irq_status_t * const out_irq_flags)
{
    s2_lp_hal_status_t err = S2_LP_RADIO_HAL_STATUS_ERROR_GENERIC;

    SID_PAL_ASSERT(drv_ctx != NULL);

    do
    {
        /* Read out all IRQ status registers - this will also clear them */
        sl2_lp_ic_reg_irq_status_t irq_status_regs = {0};

        err = _spi_read_registers(drv_ctx, S2_LP_IC_REG_ADDR_IRQ_STATUS3, irq_status_regs.raw, sizeof(irq_status_regs));
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            break;
        }

        if (out_irq_flags != NULL)
        {
            /* Compose single IRQ status word - this will align S2-LPs Big Endian encoding with the host MCU's encoding (regardless of its type) */
            out_irq_flags->raw = ((uint32_t)irq_status_regs.irq_status3.INT_LEVEL_31_24 << 24)
                               | ((uint32_t)irq_status_regs.irq_status2.INT_LEVEL_23_16 << 16)
                               | ((uint32_t)irq_status_regs.irq_status1.INT_LEVEL_15_8  <<  8)
                               | ((uint32_t)irq_status_regs.irq_status0.INT_LEVEL_7_0        );
        }

        err = S2_LP_RADIO_HAL_STATUS_OK;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED s2_lp_hal_status_t s2_lp_radio_hal_start_rx(const halo_drv_s2_lp_ctx_t * const drv_ctx)
{
    s2_lp_hal_status_t err = S2_LP_RADIO_HAL_STATUS_ERROR_GENERIC;

    SID_PAL_ASSERT(drv_ctx != NULL);

    do
    {
        /* Adjust SMPS frequency for better EMC */
        err = _set_smps_krm(drv_ctx, drv_ctx->smps_krm_rx);
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            break;
        }

        /* Send CMD_RX */
        err = _spi_send_command(drv_ctx, S2_LP_IC_CMD_RX);
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            break;
        }

        /* We don't need to wait for S2-LP to actually reach the RX state here - Sidewalk timeouts will take care of it if the transition fails */
        err = S2_LP_RADIO_HAL_STATUS_OK;

    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED s2_lp_hal_status_t s2_lp_radio_hal_start_tx(const halo_drv_s2_lp_ctx_t * const drv_ctx)
{
    s2_lp_hal_status_t err = S2_LP_RADIO_HAL_STATUS_ERROR_GENERIC;

    SID_PAL_ASSERT(drv_ctx != NULL);

    do
    {
        /* Adjust SMPS frequency for better EMC */
        err = _set_smps_krm(drv_ctx, drv_ctx->smps_krm_tx);
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            break;
        }

        /* Send CMD_TX */
        err = _spi_send_command(drv_ctx, S2_LP_IC_CMD_TX);
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            break;
        }

        /* We don't need to wait for S2-LP to actually reach the TX state here - Sidewalk timeouts will take care of it if the transition fails */
        err = S2_LP_RADIO_HAL_STATUS_OK;

    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED s2_lp_hal_status_t s2_lp_radio_hal_readout_from_rx_fifo(const halo_drv_s2_lp_ctx_t * const drv_ctx, uint8_t * const buf, const uint32_t bytes_to_read, uint32_t * const bytes_read)
{
    s2_lp_hal_status_t err = S2_LP_RADIO_HAL_STATUS_ERROR_GENERIC;

    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(buf != NULL);
    SID_PAL_ASSERT(bytes_read != NULL);

    do
    {
        uint32_t read_len;
        sl2_lp_ic_reg_rx_fifo_status_t fifo_status_reg;

        *bytes_read = 0u;

        /* Check how many bytes we have in the FIFO */
        err = _spi_read_registers(drv_ctx, S2_LP_IC_REG_ADDR_RX_FIFO_STATUS, &fifo_status_reg.raw, sizeof(fifo_status_reg.raw));
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            break;
        }

        read_len = fifo_status_reg.NELEM_RXFIFO > bytes_to_read ? bytes_to_read : fifo_status_reg.NELEM_RXFIFO;

        /* Readout from Rx FIFO */
        if (read_len > 0u)
        {
            err = _spi_read_registers(drv_ctx, S2_LP_IC_REG_ADDR_FIFO_ACCESS, buf, read_len);
            if (err != S2_LP_RADIO_HAL_STATUS_OK)
            {
                break;
            }
        }

        *bytes_read = read_len;

        err = S2_LP_RADIO_HAL_STATUS_OK;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED s2_lp_hal_status_t s2_lp_radio_hal_get_tx_fifo_free_space(const halo_drv_s2_lp_ctx_t * const drv_ctx, uint32_t * const out_free_space)
{
    s2_lp_hal_status_t err = S2_LP_RADIO_HAL_STATUS_ERROR_GENERIC;

    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(out_free_space != NULL);

    do
    {
        sl2_lp_ic_reg_tx_fifo_status_t fifo_status_reg;

        /* Check how many bytes we have in the FIFO */
        err = _spi_read_registers(drv_ctx, S2_LP_IC_REG_ADDR_TX_FIFO_STATUS, &fifo_status_reg.raw, sizeof(fifo_status_reg.raw));
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            break;
        }

        *out_free_space = S2_LP_IC_TX_FIFO_SIZE - fifo_status_reg.NELEM_TXFIFO;

        err = S2_LP_RADIO_HAL_STATUS_OK;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED s2_lp_hal_status_t s2_lp_radio_hal_write_to_tx_fifo(const halo_drv_s2_lp_ctx_t * const drv_ctx, const uint8_t * const buf, const uint32_t bytes_to_write)
{
    s2_lp_hal_status_t err = S2_LP_RADIO_HAL_STATUS_ERROR_GENERIC;

    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(buf != NULL);
    SID_PAL_ASSERT(bytes_to_write > 0u);

    do
    {
        err = _spi_write_registers(drv_ctx, S2_LP_IC_REG_ADDR_FIFO_ACCESS, buf, bytes_to_write);
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            break;
        }

        err = S2_LP_RADIO_HAL_STATUS_OK;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED s2_lp_hal_status_t s2_lp_radio_hal_get_rx_packet_info(const halo_drv_s2_lp_ctx_t * const drv_ctx, s2_lp_hal_rx_packet_info_t * const out_pckt_info)
{
    s2_lp_hal_status_t err = S2_LP_RADIO_HAL_STATUS_ERROR_GENERIC;

    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(out_pckt_info != NULL);

    do
    {
        s2_lp_hal_rx_packet_info_regs_t pckt_info_regs;

        err = _spi_read_registers(drv_ctx, S2_LP_IC_REG_ADDR_AFC_CORR, pckt_info_regs.raw, sizeof(pckt_info_regs.raw));
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_HAL_LOG_ERROR("Cannot read Rx packet status registers. Error %u", (uint32_t)err);
            break;
        }

        /* Fill in packet stats from raw register values */
        out_pckt_info->packet_length = ((uint32_t)pckt_info_regs.rx_pckt_len1 << 8) | ((uint32_t)pckt_info_regs.rx_pckt_len0);
        _convert_rssi_reg_to_dbi(drv_ctx, pckt_info_regs.rssi_level, &out_pckt_info->rssi);

        /* Reconstruct FCS type based on the calculated packet CRC value */
        if ((0x00u == pckt_info_regs.crc_field3) && (0x00u == pckt_info_regs.crc_field2))
        {
            /* Only 16 bits are used - this is FCS 1 */
            out_pckt_info->fcs_type = S2_LP_FCS_TYPE_1;
        }
        else
        {
            /* All 32 bits are used - this is FCS 0 */
            out_pckt_info->fcs_type = S2_LP_FCS_TYPE_0;
        }

        S2LP_RADIO_HAL_LOG_DEBUG("RXLEN %u RSSI SYNC %d, FCS: %u, SQI: %u, CORR: %d",
                                 out_pckt_info->packet_length,
                                 out_pckt_info->rssi.adjusted_rssi,
                                 out_pckt_info->fcs_type,
                                 pckt_info_regs.link_qualif1.SQI,
                                 pckt_info_regs.afc_corr.AFC_CORR);

        err = S2_LP_RADIO_HAL_STATUS_OK;

    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED s2_lp_hal_status_t s2_lp_radio_hal_get_rssi(const halo_drv_s2_lp_ctx_t * const drv_ctx, s2_lp_hal_rssi_t * const out_rssi)
{
    s2_lp_hal_status_t err = S2_LP_RADIO_HAL_STATUS_ERROR_GENERIC;

    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(out_rssi != NULL);

    do
    {
        uint8_t rssi_level_reg;

        /* Read out current RSSI */
        err = _spi_read_registers(drv_ctx, S2_LP_IC_REG_ADDR_RSSI_LEVEL_RUN, &rssi_level_reg, sizeof(rssi_level_reg));
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_HAL_LOG_ERROR("Cannot read RSSI  of the running Rx. Error %u", (uint32_t)err);
            break;
        }

        /* Compute RSSI from the raw register value */
        _convert_rssi_reg_to_dbi(drv_ctx, rssi_level_reg, out_rssi);

        err = S2_LP_RADIO_HAL_STATUS_OK;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED s2_lp_hal_status_t s2_lp_radio_hal_configure_tx_power(halo_drv_s2_lp_ctx_t * const drv_ctx)
{
    s2_lp_hal_status_t err = S2_LP_RADIO_HAL_STATUS_ERROR_GENERIC;

    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->regional_radio_params != NULL);

    do
    {
        /* Calculate target PA power level */
        int32_t target_pwr = (int32_t)drv_ctx->pa_params.pa_dyn_cfg.tx_power * 100;

        /* Adjust by antenna gain */
        switch (drv_ctx->regional_radio_params->param_region)
        {
            case RADIO_REGION_NA:
                if (drv_ctx->regional_radio_params->ant_dbi > S2LP_RADIO_HAL_ANT_GAIN_CAP_REGION_NA)
                {
                    /* As per the FCC requirements PA gain shall be reduced by the difference between antenna gain and the antenna gain cap value */
                    target_pwr -= ((int32_t)drv_ctx->regional_radio_params->ant_dbi - S2LP_RADIO_HAL_ANT_GAIN_CAP_REGION_NA);
                }
                break;

            case RADIO_REGION_EU:
                /* EU requirements put the limit on ERP - PA gain shall be reduced by antenna gain */
                target_pwr -= (int32_t)drv_ctx->regional_radio_params->ant_dbi;
                break;

            case RADIO_REGION_NONE:
            default:
                /* Apply PA gain as is */
                break;
            }

#if S2LP_RADIO_CFG_USE_EXTERNAL_PA
        /* If the board has external PA, calculate if we need to use it */
        if ((target_pwr + (int32_t)drv_ctx->config->pa_config.tx_bypass_loss) > (S2_LP_IC_MAX_PA_POWER_DBM * 100))
        {
            /* We won't be able to reach the desired Tx power using built-in PA only - use external PA */
            target_pwr -= (int32_t)drv_ctx->config->pa_config.tx_gain_dbi;
            drv_ctx->pa_params.ext_pa_en = TRUE;
        }
        else
        {
            /* Desired Tx power can be achieved with the build-in PA - bypass the external PA */
            target_pwr += (int32_t)drv_ctx->config->pa_config.tx_bypass_loss;
            drv_ctx->pa_params.ext_pa_en = FALSE;
        }
#else
        /* Compensate for RF switch losses */
        target_pwr += (int32_t)drv_ctx->config->pa_config.rf_sw_insertion_loss;
#endif /* S2LP_RADIO_CFG_USE_EXTERNAL_PA */

        /* Calculate ramp time in terms of the S2-LP radio (number of ticks, 1/8th of symbol time per tick) */
        uint32_t ramp_duration_8th_bits = (uint32_t)((((uint64_t)drv_ctx->current_bit_rate << 3) * (uint64_t)drv_ctx->pa_params.pa_dyn_cfg.ramp_time_us + (uint64_t)500000u) / (uint64_t)1000000u);
        if (ramp_duration_8th_bits > S2LP_RADIO_HAL_MAX_PA_RAMP_TICKS_MAX)
        {
            ramp_duration_8th_bits = S2LP_RADIO_HAL_MAX_PA_RAMP_TICKS_MAX;
        }

        /* Extract PA steps count and step duration from LUT */
        uint32_t steps_num     = pa_ramp_lut[ramp_duration_8th_bits].step_cnt;
        uint32_t step_duration = pa_ramp_lut[ramp_duration_8th_bits].step_len;

        /* Calculate min and max PA levels to use */
        const uint32_t target_pa_level = _cdbm_to_pa_level(target_pwr);
        const uint32_t min_pa_level    = _cdbm_to_pa_level(S2_LP_IC_MIN_PA_POWER_DBM * 100);


        /* Prepare configuration data */
        s2_lp_hal_pa_power_regs_t pa_power_regs = {0};
        pa_power_regs.pa_power0 = drv_ctx->regs_cache.pa_mode_cfg_regs.pa_power0;

        if (steps_num > 0u)
        {
            /* Use calculated values for the ramp start and end slots */
            pa_power_regs.pa_levels[SID_STM32_UTIL_ARRAY_SIZE(pa_power_regs.pa_levels) - 1u       ].PA_LEVEL = (uint8_t)min_pa_level;
            pa_power_regs.pa_levels[SID_STM32_UTIL_ARRAY_SIZE(pa_power_regs.pa_levels) - steps_num].PA_LEVEL = (uint8_t)target_pa_level;

            /* Interpolate for the remaining steps in between */
            const uint32_t pa_ramp_step = (min_pa_level - target_pa_level + (steps_num >> 1)) / steps_num;
            for (uint32_t i = 1u; i < (steps_num - 1u); i++)
            {
                pa_power_regs.pa_levels[(SID_STM32_UTIL_ARRAY_SIZE(pa_power_regs.pa_levels) - 1u) - i].PA_LEVEL = target_pa_level + ((steps_num - 1u - i) * pa_ramp_step);
            }

            pa_power_regs.pa_power0.PA_RAMP_EN       = 1u;
            pa_power_regs.pa_power0.PA_LEVEL_MAX_IDX = (uint8_t)(steps_num - 1u);
            pa_power_regs.pa_power0.PA_RAMP_STEP_LEN = (uint8_t)(step_duration - 1u);
        }
        else
        {
            /* No ramping - start directly with the desired power level */
            pa_power_regs.pa_levels[SID_STM32_UTIL_ARRAY_SIZE(pa_power_regs.pa_levels) - 1u].PA_LEVEL = (uint8_t)target_pa_level;

            pa_power_regs.pa_power0.PA_RAMP_EN       = 0u;
            pa_power_regs.pa_power0.PA_LEVEL_MAX_IDX = 0u;
            pa_power_regs.pa_power0.PA_RAMP_STEP_LEN = 0u;
        }

        /* Don't use the maximum output option since we need ramping */
        pa_power_regs.pa_power0.PA_MAXDBM            = 0u;

        /* Write configuration */
        err = _spi_write_registers(drv_ctx, S2_LP_IC_REG_ADDR_PA_POWER8, pa_power_regs.raw, sizeof(pa_power_regs.raw));
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_HAL_LOG_ERROR("Cannot write PA_POWERx registers. Error %u", (uint32_t)err);
            break;
        }

        /* Update register cache */
        drv_ctx->regs_cache.pa_mode_cfg_regs.pa_power0 = pa_power_regs.pa_power0;
        drv_ctx->pa_params.pa_ramp_bit_rate = drv_ctx->current_bit_rate;

        err = S2_LP_RADIO_HAL_STATUS_OK;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED s2_lp_hal_status_t s2_lp_radio_hal_set_ldc_mode(halo_drv_s2_lp_ctx_t * const drv_ctx, const uint32_t enable)
{
    s2_lp_hal_status_t err = S2_LP_RADIO_HAL_STATUS_ERROR_GENERIC;

    SID_PAL_ASSERT(drv_ctx != NULL);

    do
    {
        /* Build new register value */
        sl2_lp_ic_reg_protocol1_t protocol1_reg = drv_ctx->regs_cache.protocol_regs.protocol1;
        protocol1_reg.LDC_MODE = (FALSE == enable) ? 0u: 1u;

        /* Write configuration */
        err = _spi_write_registers(drv_ctx, S2_LP_IC_REG_ADDR_PROTOCOL1, &protocol1_reg.raw, sizeof(protocol1_reg.raw));
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_HAL_LOG_ERROR("Cannot write PROTOCOL1 register. Error %u", (uint32_t)err);
            break;
        }

        /* Update register cache */
        drv_ctx->regs_cache.protocol_regs.protocol1 = protocol1_reg;
    } while (0);

    return err;
}
