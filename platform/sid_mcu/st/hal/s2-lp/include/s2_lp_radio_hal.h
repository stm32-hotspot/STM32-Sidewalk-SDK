/**
  ******************************************************************************
  * @file    s2_lp_radio_hal.h
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

#ifndef __S2_LP_RADIO_HAL_H_
#define __S2_LP_RADIO_HAL_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "s2_lp_ic_definitions.h"
#include "s2_lp_radio.h"
#include "s2_lp_radio_config.h"

/* ST platform headers */
#include <sid_stm32_common_utils.h>

/* Exported constants --------------------------------------------------------*/

#define S2LP_RADIO_HAL_SIDEWALK_FSK_HEADER_SIZE           (2u)                                /*!< Size of the FSK packet header (in bytes) used in Sidewalk */

#define S2LP_RADIO_HAL_SIGNAL_LVL_OFFSET                  (146u)                              /*!< Internal representation of the dBm values in S2-LP is shifted by this value */

#define S2LP_RADIO_HAL_RX_FIFO_ALMOST_FULL_LEVEL          ((S2_LP_IC_RX_FIFO_SIZE * 3u) / 4u) /* Set Rx Almost Full event at 3/4 of the FIFO size */
#define S2LP_RADIO_HAL_TX_FIFO_ALMOST_EMPTY_LEVEL         (S2_LP_IC_TX_FIFO_SIZE / 4u)        /* Set Tx Almost Empty event at 1/4 of the FIFO size */

#define S2LP_RADIO_HAL_RCO_CALIBRATION_TIMEOUT_MS         (10u)                               /*!< Timeout (in milliseconds) for the S2-LP to calibrate its RCO */

/* Exported macro ------------------------------------------------------------*/

/**
 * @brief Convert S2-LP's internal signal level representation to the physical dBm value
 */
#define S2LP_RADIO_HAL_LVL_TO_DBM(__LVL__) ((int32_t)((int32_t)((uint32_t)(__LVL__)) - (int32_t)(S2LP_RADIO_HAL_SIGNAL_LVL_OFFSET)))

/**
 * @brief Convert physical dBm value to S2-LP's internal signal level representation
 */
#define S2LP_RADIO_HAL_DBM_TO_LVL(__DBM__) ((uint8_t)((uint32_t)((__DBM__) + (uint32_t)(S2LP_RADIO_HAL_SIGNAL_LVL_OFFSET))))

/* Exported types ------------------------------------------------------------*/

/**
 * @brief S2-LP Sidewalk Radio Driver HAL retun codes
 */
typedef enum {
    S2_LP_RADIO_HAL_STATUS_OK                    =  0, /*!< No errors */
    S2_LP_RADIO_HAL_STATUS_ERROR_GENERIC         =  1, /*!< Generic error of unknown/unspecified nature */
    S2_LP_RADIO_HAL_STATUS_INVALID_ARGS          =  2, /*!< Invalid arguments supplied by the caller to a function */
    S2_LP_RADIO_HAL_STATUS_ERROR_GPIO            =  3, /*!< GPIO pin state error */
    S2_LP_RADIO_HAL_STATUS_ERROR_HW              =  4, /*!< Hardware error (e.g. broken physical connection, shorted pin, etc.) */
    S2_LP_RADIO_HAL_STATUS_ERROR_SPI_XFER        =  5, /*!< SPI bus transfer failed */
    S2_LP_RADIO_HAL_STATUS_ERROR_DATA_TOO_LONG   =  6, /*!< Data cannot be processed because it's too long */
    S2_LP_RADIO_HAL_STATUS_ERROR_NOT_SUPPORTED   =  7, /*!< The requested mode or operation is not supported */
    S2_LP_RADIO_HAL_STATUS_ERROR_UNEXPECTED_DATA =  8, /*!< Unexpected data received */
    S2_LP_RADIO_HAL_STATUS_ERROR_BUFFER_OVERFLOW =  9, /*!< Buffer overflow */
    S2_LP_RADIO_HAL_STATUS_ERROR_INCOMPLETE_XFER = 10, /*!< Tx/Rx transaction ended before the desired amount of data was sent/received */
    S2_LP_RADIO_HAL_STATUS_ERROR_TIMEOUT         = 11, /*!< Requested operation aborted due to timeout */
    S2_LP_RADIO_HAL_STATUS_ERROR_BUSY            = 12, /*!< Radio driver is busy with some other request, you need to retry later */
    S2_LP_RADIO_HAL_STATUS_ERROR_OUT_OF_RANGE    = 13, /*!< Parameter or configuration value is out of the valid range */
    S2_LP_RADIO_HAL_STATUS_ERROR_INVALID_STATE   = 14, /*!< Invalid or prohibited logical state */
} s2_lp_hal_status_t;

/**
 * @brief Specifies the desired mode of the radio front-end module (FEM)
 */
typedef enum {
    S2_LP_RADIO_HAL_FEM_MODE_SHUTDOWN = 0, /*!< FEM is shut down */
    S2_LP_RADIO_HAL_FEM_MODE_RX       = 1, /*!< Receive mode - connects antenna to the receiver LNA */
    S2_LP_RADIO_HAL_FEM_MODE_TX_LP    = 2, /*!< Lower power (bypass) transmit mode - FEM is bypassed and antenna is connected directly to the S2-LP transmitter */
    S2_LP_RADIO_HAL_FEM_MODE_TX_HP    = 3, /*!< High power transmit mode - FEM's power amplifier is used to increase the Tx power */
} s2_lp_hal_fem_mode_t;

typedef struct {
    uint32_t             bit_rate_bps;           /*!< Desired bit rate in bits per second */
    uint32_t             fdev_in_hz;             /*!< FSK Frequency Deviation (Fdev) in Hz */
    uint32_t             rx_filter_bandwidth_hz; /*!< Desired RX channel filter bandwidth in Hz */
    s2_lp_ic_mod_types_t mod_type;               /*!< S2-LP modulation type selector */
} s2_lp_hal_mod_params_t;

typedef struct {
    uint32_t            preamble_len_bits;
    uint32_t            pqi_threshold;
    s2_lp_ic_crc_mode_t crc_mode;
    uint8_t             data_whitening_en;
    uint8_t             variable_packet_len_en;
    uint32_t            packet_length;
} s2_lp_hal_pckt_params_t;

typedef enum {
    S2_LP_RADIO_HAL_SYNC_WORD_PRIMARY   = 0,
    S2_LP_RADIO_HAL_SYNC_WORD_SECONDARY = 1,
} s2_lp_hal_sync_word_selector_t;

typedef struct {
    int32_t immediate_rssi; /*!< Raw RSSI value as reported by the S2-LP IC */
    int32_t adjusted_rssi;  /*!< RSSI level with antenna gain and LNA contribution excluded */
} s2_lp_hal_rssi_t;

typedef struct {
    uint32_t            packet_length;
    s2_lp_hal_rssi_t    rssi;
    s2_lp_ic_fcs_type_t fcs_type;
} s2_lp_hal_rx_packet_info_t;

typedef enum {
    S2_LP_RADIO_HAL_FIFO_RX   = 0, /*!< Rx FIFO only */
    S2_LP_RADIO_HAL_FIFO_TX   = 1, /*!< Tx FIFO only */
    S2_LP_RADIO_HAL_FIFO_BOTH = 2, /*!< Both Rx and Tx FIFOs */
} s2_lp_hal_fifo_selector_t;

/* Exported functions --------------------------------------------------------*/

/**
 * @brief Initializes GPIO pins required for S2-LP radio operation.
 *
 * @param[in] drv_ctx Pointer to the S2-LP driver context, which contains GPIO configuration.
 *
 * @return Status of the operation.
 * @retval S2_LP_RADIO_HAL_STATUS_OK on success.
 * @retval S2_LP_RADIO_HAL_STATUS_ERROR_GPIO on GPIO configuration failure.
 */
s2_lp_hal_status_t s2_lp_radio_hal_init_gpio(const halo_drv_s2_lp_ctx_t * const drv_ctx);

/**
 * @brief De-initializes GPIO pins used by the S2-LP radio, setting them to a low-power state.
 *
 * @param[in] drv_ctx Pointer to the S2-LP driver context.
 *
 * @return Status of the operation.
 * @retval S2_LP_RADIO_HAL_STATUS_OK on success.
 * @retval S2_LP_RADIO_HAL_STATUS_ERROR_GPIO on GPIO configuration failure.
 */
s2_lp_hal_status_t s2_lp_radio_hal_deinit_gpio(const halo_drv_s2_lp_ctx_t * const drv_ctx);

#if S2LP_RADIO_CFG_USE_STATUS_LED
/**
 * @brief Turns the transmit status LED on.
 *
 * @param[in] drv_ctx Pointer to the S2-LP driver context.
 *
 * @return Status of the operation.
 * @retval S2_LP_RADIO_HAL_STATUS_OK on success.
 * @retval S2_LP_RADIO_HAL_STATUS_ERROR_HW on GPIO write failure.
 */
s2_lp_hal_status_t s2_lp_radio_hal_tx_led_on(const halo_drv_s2_lp_ctx_t * const drv_ctx);

/**
 * @brief Turns the transmit status LED off.
 *
 * @param[in] drv_ctx Pointer to the S2-LP driver context.
 *
 * @return Status of the operation.
 * @retval S2_LP_RADIO_HAL_STATUS_OK on success.
 * @retval S2_LP_RADIO_HAL_STATUS_ERROR_HW on GPIO write failure.
 */
s2_lp_hal_status_t s2_lp_radio_hal_tx_led_off(const halo_drv_s2_lp_ctx_t * const drv_ctx);

/**
 * @brief Turns the receive status LED on.
 *
 * @param[in] drv_ctx Pointer to the S2-LP driver context.
 *
 * @return Status of the operation.
 * @retval S2_LP_RADIO_HAL_STATUS_OK on success.
 * @retval S2_LP_RADIO_HAL_STATUS_ERROR_HW on GPIO write failure.
 */
s2_lp_hal_status_t s2_lp_radio_hal_rx_led_on(const halo_drv_s2_lp_ctx_t * const drv_ctx);

/**
 * @brief Turns the receive status LED off.
 *
 * @param[in] drv_ctx Pointer to the S2-LP driver context.
 *
 * @return Status of the operation.
 * @retval S2_LP_RADIO_HAL_STATUS_OK on success.
 * @retval S2_LP_RADIO_HAL_STATUS_ERROR_HW on GPIO write failure.
 */
s2_lp_hal_status_t s2_lp_radio_hal_rx_led_off(const halo_drv_s2_lp_ctx_t * const drv_ctx);
#endif /* S2LP_RADIO_CFG_USE_STATUS_LED */

/**
 * @brief Resets the S2-LP radio IC by toggling its shutdown pin.
 *
 * @param[in] drv_ctx Pointer to the S2-LP driver context.
 *
 * @return Status of the operation.
 * @retval S2_LP_RADIO_HAL_STATUS_OK on success.
 * @retval S2_LP_RADIO_HAL_STATUS_ERROR_HW on GPIO control failure.
 */
s2_lp_hal_status_t s2_lp_radio_hal_reset_radio(const halo_drv_s2_lp_ctx_t * const drv_ctx);

#if S2LP_RADIO_CFG_USE_EXTERNAL_PA
/**
 * @brief Sets the operating mode of the external Front-End Module (FEM).
 *
 * @param[in] drv_ctx Pointer to the S2-LP driver context.
 * @param[in] mode The desired mode for the FEM (e.g., RX, TX, Shutdown).
 *
 * @return Status of the operation.
 * @retval S2_LP_RADIO_HAL_STATUS_OK on success.
 * @retval S2_LP_RADIO_HAL_STATUS_ERROR_HW on GPIO control failure.
 * @retval S2_LP_RADIO_HAL_STATUS_INVALID_ARGS if an invalid mode is provided.
 */
s2_lp_hal_status_t s2_lp_radio_hal_set_fem_mode(const halo_drv_s2_lp_ctx_t * const drv_ctx, const s2_lp_hal_fem_mode_t mode);
#endif /* S2LP_RADIO_CFG_USE_EXTERNAL_PA */

/**
 * @brief Disarms the radio interrupt line on the host MCU.
 *
 * @param[in] drv_ctx Pointer to the S2-LP driver context.
 *
 * @return Status of the operation.
 * @retval S2_LP_RADIO_HAL_STATUS_OK on success.
 * @retval S2_LP_RADIO_HAL_STATUS_ERROR_HW on failure to disable the IRQ.
 */
s2_lp_hal_status_t s2_lp_radio_hal_disarm_irq(const halo_drv_s2_lp_ctx_t * const drv_ctx);

/**
 * @brief Arms the radio interrupt line on the host MCU.
 *
 * @param[in,out] drv_ctx Pointer to the S2-LP driver context.
 *
 * @return Status of the operation.
 * @retval S2_LP_RADIO_HAL_STATUS_OK on success.
 * @retval S2_LP_RADIO_HAL_STATUS_ERROR_HW on failure to configure and enable the IRQ.
 */
s2_lp_hal_status_t s2_lp_radio_hal_arm_irq(halo_drv_s2_lp_ctx_t * const drv_ctx);

/**
 * @brief Sets the internal interrupt mask of the S2-LP radio.
 *
 * @param[in] drv_ctx Pointer to the S2-LP driver context.
 * @param[in] mask The 32-bit interrupt mask to apply.
 *
 * @return Status of the operation.
 * @retval S2_LP_RADIO_HAL_STATUS_OK on success.
 * @retval S2_LP_RADIO_HAL_STATUS_ERROR_SPI_XFER on SPI communication failure.
 */
s2_lp_hal_status_t s2_lp_radio_hal_set_radio_irq_mask(const halo_drv_s2_lp_ctx_t * const drv_ctx, const sl2_lp_ic_irq_mask_t mask);

/**
 * @brief Initializes static (non-channel-dependent) modulation parameters.
 *
 * @details This configures settings like clock recovery, AGC, AFC, and FIFO thresholds.
 *
 * @param[in,out] drv_ctx Pointer to the S2-LP driver context to update register cache.
 *
 * @return Status of the operation.
 * @retval S2_LP_RADIO_HAL_STATUS_OK on success.
 * @retval S2_LP_RADIO_HAL_STATUS_ERROR_SPI_XFER on SPI communication failure.
 */
s2_lp_hal_status_t s2_lp_radio_hal_static_mod_params_init(halo_drv_s2_lp_ctx_t * const drv_ctx);

/**
 * @brief Performs a synchronous calibration of the S2-LP's internal RCO.
 *
 * @details This function blocks until the calibration is complete or times out.
 *
 * @param[in,out] drv_ctx Pointer to the S2-LP driver context.
 *
 * @return Status of the operation.
 * @retval S2_LP_RADIO_HAL_STATUS_OK on success.
 * @retval S2_LP_RADIO_HAL_STATUS_ERROR_TIMEOUT if calibration does not complete in time.
 * @retval S2_LP_RADIO_HAL_STATUS_ERROR_SPI_XFER on SPI communication failure.
 */
s2_lp_hal_status_t s2_lp_radio_hal_calibrate_rco_sync(halo_drv_s2_lp_ctx_t * const drv_ctx);

/**
 * @brief Starts an asynchronous calibration of the S2-LP's internal RCO.
 *
 * @details This function initiates the calibration and schedules a timer to check for completion.
 *
 * @param[in,out] drv_ctx Pointer to the S2-LP driver context.
 *
 * @return Status of the operation.
 * @retval S2_LP_RADIO_HAL_STATUS_OK on success.
 * @retval S2_LP_RADIO_HAL_STATUS_ERROR_HW on timer arming failure.
 */
s2_lp_hal_status_t s2_lp_radio_hal_calibrate_rco_async(halo_drv_s2_lp_ctx_t * const drv_ctx);

/**
 * @brief Timer callback for asynchronous RCO calibration.
 *
 * @details Checks for calibration completion, stores the results, and puts the radio to sleep.
 *
 * @param[in] arg Pointer to user arguments (should be the driver context).
 * @param[in] originator Pointer to the timer instance that fired.
 */
void               s2_lp_radio_hal_rco_calib_timer_cb(void * arg, sid_pal_timer_t * originator);

/**
 * @brief Performs low-level initialization of the S2-LP radio IC.
 *
 * @details Configures S2-LP GPIOs, SMPS, XTAL settings, calibrates the RCO, and sets the intermediate frequency.
 *
 * @param[in,out] drv_ctx Pointer to the S2-LP driver context.
 *
 * @return Status of the operation.
 * @retval S2_LP_RADIO_HAL_STATUS_OK on success.
 * @retval Various error codes on failure.
 */
s2_lp_hal_status_t s2_lp_radio_hal_ll_init(halo_drv_s2_lp_ctx_t * const drv_ctx);

/**
 * @brief Reads the version information from the S2-LP IC.
 *
 * @param[in] drv_ctx Pointer to the S2-LP driver context.
 * @param[out] out_version_info Pointer to a structure where the version info will be stored.
 *
 * @return Status of the operation.
 * @retval S2_LP_RADIO_HAL_STATUS_OK on success.
 * @retval S2_LP_RADIO_HAL_STATUS_ERROR_SPI_XFER on SPI communication failure.
 */
s2_lp_hal_status_t s2_lp_radio_hal_get_ic_version(const halo_drv_s2_lp_ctx_t * const drv_ctx, s2_lp_ic_version_info_t * const out_version_info);

/**
 * @brief Puts the S2-LP radio into the READY state.
 *
 * @details In the READY state, the crystal oscillator is running, and the radio is prepared for RX/TX.
 *
 * @param[in] drv_ctx Pointer to the S2-LP driver context.
 *
 * @return Status of the operation.
 * @retval S2_LP_RADIO_HAL_STATUS_OK on success.
 * @retval S2_LP_RADIO_HAL_STATUS_ERROR_TIMEOUT if the state transition fails.
 */
s2_lp_hal_status_t s2_lp_radio_hal_set_ready(const halo_drv_s2_lp_ctx_t * const drv_ctx);

/**
 * @brief Puts the S2-LP radio into the STANDBY state.
 *
 * @details In STANDBY state, the crystal oscillator is off for power saving.
 *
 * @param[in] drv_ctx Pointer to the S2-LP driver context.
 *
 * @return Status of the operation.
 * @retval S2_LP_RADIO_HAL_STATUS_OK on success.
 * @retval S2_LP_RADIO_HAL_STATUS_ERROR_TIMEOUT if the state transition fails.
 */
s2_lp_hal_status_t s2_lp_radio_hal_set_standby(const halo_drv_s2_lp_ctx_t * const drv_ctx);

/**
 * @brief Sets the synthesizer base frequency and channel spacing.
 *
 * @param[in,out] drv_ctx Pointer to the S2-LP driver context.
 * @param[in] freq_hz The desired base frequency in Hz.
 * @param[in] ch_spacing The desired channel spacing in Hz.
 *
 * @return Status of the operation.
 * @retval S2_LP_RADIO_HAL_STATUS_OK on success.
 * @retval S2_LP_RADIO_HAL_STATUS_INVALID_ARGS if the frequency is out of range.
 */
s2_lp_hal_status_t s2_lp_radio_hal_set_base_frequency(halo_drv_s2_lp_ctx_t * const drv_ctx, const uint32_t freq_hz, const uint32_t ch_spacing);

/**
 * @brief Sets the radio frequency channel number.
 *
 * @param[in] drv_ctx Pointer to the S2-LP driver context.
 * @param[in] ch_num The channel number to set.
 *
 * @return Status of the operation.
 * @retval S2_LP_RADIO_HAL_STATUS_OK on success.
 * @retval S2_LP_RADIO_HAL_STATUS_ERROR_SPI_XFER on SPI communication failure.
 */
s2_lp_hal_status_t s2_lp_radio_hal_set_rf_channel(halo_drv_s2_lp_ctx_t * const drv_ctx, const uint32_t ch_num);

/**
 * @brief Sets the synchronization word and its length.
 *
 * @param[in,out] drv_ctx Pointer to the S2-LP driver context.
 * @param[in] sync_word Pointer to the buffer containing the sync word.
 * @param[in] sync_word_length The length of the sync word in bytes.
 * @param[in] sync_word_sel Selects the primary or secondary sync word registers.
 *
 * @return Status of the operation.
 * @retval S2_LP_RADIO_HAL_STATUS_OK on success.
 * @retval S2_LP_RADIO_HAL_STATUS_ERROR_DATA_TOO_LONG if the sync word is too long.
 */
s2_lp_hal_status_t s2_lp_radio_hal_set_syncword(halo_drv_s2_lp_ctx_t * const drv_ctx, const uint8_t * const sync_word, const uint32_t sync_word_length, const s2_lp_hal_sync_word_selector_t sync_word_sel);

/**
 * @brief Sets the modulation parameters.
 *
 * @param[in,out] drv_ctx Pointer to the S2-LP driver context.
 * @param[in] mod_params Pointer to a structure containing the desired modulation parameters (bitrate, fdev, bandwidth).
 *
 * @return Status of the operation.
 * @retval S2_LP_RADIO_HAL_STATUS_OK on success.
 * @retval S2_LP_RADIO_HAL_STATUS_INVALID_ARGS if any parameter is out of the configurable range.
 */
s2_lp_hal_status_t s2_lp_radio_hal_set_mod_params(halo_drv_s2_lp_ctx_t * const drv_ctx, const s2_lp_hal_mod_params_t * const mod_params);

/**
 * @brief Sets the packet handler parameters.
 *
 * @param[in,out] drv_ctx Pointer to the S2-LP driver context.
 * @param[in] pckt_params Pointer to a structure containing the packet parameters (preamble, CRC, whitening, etc.).
 *
 * @return Status of the operation.
 * @retval S2_LP_RADIO_HAL_STATUS_OK on success.
 * @retval S2_LP_RADIO_HAL_STATUS_ERROR_SPI_XFER on SPI communication failure.
 */
s2_lp_hal_status_t s2_lp_radio_hal_set_pckt_params(halo_drv_s2_lp_ctx_t * const drv_ctx, const s2_lp_hal_pckt_params_t * const pckt_params);

/**
 * @brief Flushes the RX, TX, or both FIFOs.
 *
 * @param[in] drv_ctx Pointer to the S2-LP driver context.
 * @param[in] fifo_selector Specifies which FIFO(s) to flush.
 *
 * @return Status of the operation.
 * @retval S2_LP_RADIO_HAL_STATUS_OK on success.
 * @retval S2_LP_RADIO_HAL_STATUS_ERROR_SPI_XFER on SPI communication failure.
 */
s2_lp_hal_status_t s2_lp_radio_hal_flush_fifo(const halo_drv_s2_lp_ctx_t * const drv_ctx, const s2_lp_hal_fifo_selector_t fifo_selector);

/**
 * @brief Configures the packet engine for a receive operation.
 *
 * @param[in,out] drv_ctx Pointer to the S2-LP driver context.
 * @param[in] timeout_stop_condition The condition(s) that will stop the RX timer.
 *
 * @return Status of the operation.
 * @retval S2_LP_RADIO_HAL_STATUS_OK on success.
 * @retval S2_LP_RADIO_HAL_STATUS_ERROR_SPI_XFER on SPI communication failure.
 */
s2_lp_hal_status_t s2_lp_radio_hal_configure_packet_engine_for_rx(halo_drv_s2_lp_ctx_t * const drv_ctx, const s2_lp_ic_rx_timeout_stop_condition_t timeout_stop_condition);

/**
 * @brief Configures the packet engine for a transmit operation.
 *
 * @param[in,out] drv_ctx Pointer to the S2-LP driver context.
 *
 * @return Status of the operation.
 * @retval S2_LP_RADIO_HAL_STATUS_OK on success.
 * @retval S2_LP_RADIO_HAL_STATUS_ERROR_SPI_XFER on SPI communication failure.
 */
s2_lp_hal_status_t s2_lp_radio_hal_configure_packet_engine_for_tx(halo_drv_s2_lp_ctx_t * const drv_ctx);

/**
 * @brief Configures the packet engine for continuous wave (CW) transmission.
 *
 * @param[in,out] drv_ctx Pointer to the S2-LP driver context.
 *
 * @return Status of the operation.
 * @retval S2_LP_RADIO_HAL_STATUS_OK on success.
 * @retval S2_LP_RADIO_HAL_STATUS_ERROR_SPI_XFER on SPI communication failure.
 */
s2_lp_hal_status_t s2_lp_radio_hal_configure_packet_engine_for_cw(halo_drv_s2_lp_ctx_t * const drv_ctx);

/**
 * @brief Sets the timeout for a receive operation.
 *
 * @param[in] drv_ctx Pointer to the S2-LP driver context.
 * @param[in] timeout_us The desired timeout in microseconds.
 *
 * @return Status of the operation.
 * @retval S2_LP_RADIO_HAL_STATUS_OK on success.
 * @retval S2_LP_RADIO_HAL_STATUS_ERROR_SPI_XFER on SPI communication failure.
 */
s2_lp_hal_status_t s2_lp_radio_hal_set_rx_timeout(const halo_drv_s2_lp_ctx_t * const drv_ctx, const uint32_t timeout_us);

/**
 * @brief Sets the wakeup timeout for Low Duty Cycle (LDC) mode.
 *
 * @param[in,out] drv_ctx Pointer to the S2-LP driver context.
 * @param[in] timeout_us The desired wakeup timeout in microseconds.
 *
 * @return Status of the operation.
 * @retval S2_LP_RADIO_HAL_STATUS_OK on success.
 * @retval S2_LP_RADIO_HAL_STATUS_ERROR_OUT_OF_RANGE if the requested time is too long.
 */
s2_lp_hal_status_t s2_lp_radio_hal_set_ldc_timeout(halo_drv_s2_lp_ctx_t * const drv_ctx, const uint32_t timeout_us);

/**
 * @brief Reads and clears the S2-LP interrupt status registers.
 *
 * @param[in] drv_ctx Pointer to the S2-LP driver context.
 * @param[out] out_irq_flags Optional pointer to store the 32-bit word of interrupt flags. Can be NULL.
 *
 * @return Status of the operation.
 * @retval S2_LP_RADIO_HAL_STATUS_OK on success.
 * @retval S2_LP_RADIO_HAL_STATUS_ERROR_SPI_XFER on SPI communication failure.
 */
s2_lp_hal_status_t s2_lp_radio_hal_get_clear_irq_status(const halo_drv_s2_lp_ctx_t * const drv_ctx, sl2_lp_ic_irq_status_t * const out_irq_flags);

/**
 * @brief Starts a receive operation.
 *
 * @param[in] drv_ctx Pointer to the S2-LP driver context.
 *
 * @return Status of the operation.
 * @retval S2_LP_RADIO_HAL_STATUS_OK on success.
 * @retval S2_LP_RADIO_HAL_STATUS_ERROR_SPI_XFER on SPI communication failure.
 */
s2_lp_hal_status_t s2_lp_radio_hal_start_rx(const halo_drv_s2_lp_ctx_t * const drv_ctx);

/**
 * @brief Starts a transmit operation.
 *
 * @param[in] drv_ctx Pointer to the S2-LP driver context.
 *
 * @return Status of the operation.
 * @retval S2_LP_RADIO_HAL_STATUS_OK on success.
 * @retval S2_LP_RADIO_HAL_STATUS_ERROR_SPI_XFER on SPI communication failure.
 */
s2_lp_hal_status_t s2_lp_radio_hal_start_tx(const halo_drv_s2_lp_ctx_t * const drv_ctx);

/**
 * @brief Reads data from the RX FIFO.
 *
 * @param[in] drv_ctx Pointer to the S2-LP driver context.
 * @param[out] buf Pointer to the destination buffer.
 * @param[in] bytes_to_read The maximum number of bytes to read.
 * @param[out] bytes_read Pointer to store the actual number of bytes read from the FIFO.
 *
 * @return Status of the operation.
 * @retval S2_LP_RADIO_HAL_STATUS_OK on success.
 * @retval S2_LP_RADIO_HAL_STATUS_ERROR_SPI_XFER on SPI communication failure.
 */
s2_lp_hal_status_t s2_lp_radio_hal_readout_from_rx_fifo(const halo_drv_s2_lp_ctx_t * const drv_ctx, uint8_t * const buf, const uint32_t bytes_to_read, uint32_t * const bytes_read);

/**
 * @brief Gets the amount of free space in the TX FIFO.
 *
 * @param[in] drv_ctx Pointer to the S2-LP driver context.
 * @param[out] out_free_space Pointer to store the number of free bytes.
 *
 * @return Status of the operation.
 * @retval S2_LP_RADIO_HAL_STATUS_OK on success.
 * @retval S2_LP_RADIO_HAL_STATUS_ERROR_SPI_XFER on SPI communication failure.
 */
s2_lp_hal_status_t s2_lp_radio_hal_get_tx_fifo_free_space(const halo_drv_s2_lp_ctx_t * const drv_ctx, uint32_t * const out_free_space);

/**
 * @brief Writes data to the TX FIFO.
 *
 * @param[in] drv_ctx Pointer to the S2-LP driver context.
 * @param[in] buf Pointer to the source data buffer.
 * @param[in] bytes_to_write The number of bytes to write to the FIFO.
 *
 * @return Status of the operation.
 * @retval S2_LP_RADIO_HAL_STATUS_OK on success.
 * @retval S2_LP_RADIO_HAL_STATUS_ERROR_SPI_XFER on SPI communication failure.
 */
s2_lp_hal_status_t s2_lp_radio_hal_write_to_tx_fifo(const halo_drv_s2_lp_ctx_t * const drv_ctx, const uint8_t * const buf, const uint32_t bytes_to_write);

/**
 * @brief Retrieves information about the last successfully received packet.
 *
 * @param[in] drv_ctx Pointer to the S2-LP driver context.
 * @param[out] out_pckt_info Pointer to a structure to store the packet info (length, RSSI, FCS type).
 *
 * @return Status of the operation.
 * @retval S2_LP_RADIO_HAL_STATUS_OK on success.
 * @retval S2_LP_RADIO_HAL_STATUS_ERROR_SPI_XFER on SPI communication failure.
 */
s2_lp_hal_status_t s2_lp_radio_hal_get_rx_packet_info(const halo_drv_s2_lp_ctx_t * const drv_ctx, s2_lp_hal_rx_packet_info_t * const out_pckt_info);

/**
 * @brief Gets the current RSSI value while in RX mode.
 *
 * @param[in] drv_ctx Pointer to the S2-LP driver context.
 * @param[out] out_rssi Pointer to a structure to store the RSSI value.
 *
 * @return Status of the operation.
 * @retval S2_LP_RADIO_HAL_STATUS_OK on success.
 * @retval S2_LP_RADIO_HAL_STATUS_ERROR_SPI_XFER on SPI communication failure.
 */
s2_lp_hal_status_t s2_lp_radio_hal_get_rssi(const halo_drv_s2_lp_ctx_t * const drv_ctx, s2_lp_hal_rssi_t * const out_rssi);

/**
 * @brief Configures the transmit power and power ramping.
 * 
 * @details This function calculates and sets the PA levels and ramping parameters based on the desired output power
 *          and regional regulations.
 * @param[in,out] drv_ctx Pointer to the S2-LP driver context.
 *
 * @return Status of the operation.
 * @retval S2_LP_RADIO_HAL_STATUS_OK on success.
 * @retval S2_LP_RADIO_HAL_STATUS_ERROR_SPI_XFER on SPI communication failure.
 */
s2_lp_hal_status_t s2_lp_radio_hal_configure_tx_power(halo_drv_s2_lp_ctx_t * const drv_ctx);

/**
 * @brief Enables or disables the Low Duty Cycle (LDC) mode.
 *
 * @param[in,out] drv_ctx Pointer to the S2-LP driver context.
 * @param[in] enable A non-zero value to enable LDC mode, zero to disable.
 *
 * @return Status of the operation.
 * @retval S2_LP_RADIO_HAL_STATUS_OK on success.
 * @retval S2_LP_RADIO_HAL_STATUS_ERROR_SPI_XFER on SPI communication failure.
 */
s2_lp_hal_status_t s2_lp_radio_hal_set_ldc_mode(halo_drv_s2_lp_ctx_t * const drv_ctx, const uint32_t enable);

#ifdef __cplusplus
}
#endif

#endif /* __S2_LP_RADIO_HAL_H_ */
