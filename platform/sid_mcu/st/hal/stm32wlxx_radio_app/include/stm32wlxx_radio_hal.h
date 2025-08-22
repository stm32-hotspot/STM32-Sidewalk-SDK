/**
  ******************************************************************************
  * @file    stm32wlxx_radio_hal.h
  * @brief   Hardware abstraction layer for the STM32WLxx Sidewalk Radio App
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

#ifndef __STM32WLXX_RADIO_APP_HAL_H_
#define __STM32WLXX_RADIO_APP_HAL_H_

/* Includes ------------------------------------------------------------------*/

#include "stm32wlxx_common_defs.h"
#include <comm_def.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Exported types ------------------------------------------------------------*/

/**
 * @brief STM32WLxx Sidewalk Radio App return codes
 */
typedef enum {
    STM32WLxx_HAL_STATUS_OK                    =  0, /*!< No errors */
    STM32WLxx_HAL_STATUS_ERROR_GENERIC         =  1, /*!< Generic error of unknown/unspecified nature */
    STM32WLxx_HAL_STATUS_INVALID_ARGS          =  2, /*!< Invalid arguments supplied by the caller to a function */
    STM32WLxx_HAL_STATUS_ERROR_GPIO            =  3, /*!< GPIO pin state error */
    STM32WLxx_HAL_STATUS_ERROR_HW              =  4, /*!< Hardware error (e.g. broken physical connection, shorted pin, etc.) */
    STM32WLxx_HAL_STATUS_ERROR_SPI_XFER        =  5, /*!< SPI bus transfer failed */
    STM32WLxx_HAL_STATUS_ERROR_DATA_TOO_LONG   =  6, /*!< Data cannot be processed because it's too long */
    STM32WLxx_HAL_STATUS_ERROR_NOT_SUPPORTED   =  7, /*!< The requested mode or operation is not supported */
    STM32WLxx_HAL_STATUS_ERROR_UNEXPECTED_DATA =  8, /*!< Unexpected data received */
    STM32WLxx_HAL_STATUS_ERROR_BUFFER_OVERFLOW =  9, /*!< Buffer overflow */
    STM32WLxx_HAL_STATUS_ERROR_INCOMPLETE_XFER = 10, /*!< Tx/Rx transaction ended before the desired amount of data was sent/received */
    STM32WLxx_HAL_STATUS_ERROR_TIMEOUT         = 11, /*!< Requested operation aborted due to timeout */
    STM32WLxx_HAL_STATUS_ERROR_BUSY            = 12, /*!< Radio driver is busy with some other request, you need to retry later */
    STM32WLxx_HAL_STATUS_ERROR_REJECTED        = 13, /*!< STM32WLxx rejected request of the host MCU or the request has failed on the STM32WLxx side */
    STM32WLxx_HAL_STATUS_ERROR_INVALID_STATE   = 14, /*!< Invalid or prohibited logical state */
} stm32wlxx_hal_status_t;

/* Exported constants --------------------------------------------------------*/

#define STM32WLxx_HAL_RADIO_IRQ_PIN_STATE_ACTIVE           (0u) /*!< IRQ GPIO pin state when STM32WLxx indicates an active IRQ */
#define STM32WLxx_HAL_RADIO_IRQ_PIN_STATE_INACTIVE         (1u) /*!< IRQ GPIO pin state when STM32WLxx indicates no IRQ */
#define STM32WLxx_HAL_RADIO_IRQ_PIN_STATE_PROBE_PERIOD_US  (3u)
#define STM32WLxx_HAL_RADIO_IRQ_PIN_STATE_PROBE_DEBOUNCE   (3u) /*!< Pin must remain in the desired state for this number of consecutive reads */
#define STM32WLxx_HAL_RADIO_IRQ_RELEASE_WAIT_TIME_US       (400u) /*!< Maximum allowed wait time for the STM32WLxx IRQ line to be released by the STM32WLxx side after successful IRQ acknowledgment */

#define STM32WLxx_HAL_RADIO_IRQ_PIN_FILTER_PROBE_COUNT     (3u) /*!< IRQ line will be inspected this amount of times before Handshake IRQ will be reported */
#define STM32WLxx_HAL_RADIO_IRQ_PIN_FILTER_PROBE_PERIOD_US (1u) /*!< Delay between consecutive IRQ line state samples */
#define STM32WLxx_HAL_RADIO_IRQ_PIN_FILTER_PROBE_DEBOUNCE  (2u) /*!< Pin must remain in the desired state for this number of consecutive reads */
#if STM32WLxx_HAL_RADIO_IRQ_PIN_FILTER_PROBE_DEBOUNCE > STM32WLxx_HAL_RADIO_IRQ_PIN_FILTER_PROBE_COUNT
#  error "STM32WLxx_HAL_RADIO_IRQ_PIN_FILTER_PROBE_DEBOUNCE cannot exceed STM32WLxx_HAL_RADIO_IRQ_PIN_FILTER_PROBE_COUNT since debounce counter physically cannot exceed the number of probe iterations"
#endif

#define STM32WLxx_ALL_APP_IRQ_MASK                         (STM32WLxx_APP_IRQ_SUBGHZ | STM32WLxx_APP_IRQ_SPI_REQ_DONE | STM32WLxx_APP_IRQ_SPI_HANDSHAKE | STM32WLxx_APP_IRQ_USER_DATA) /*!< All possible STM32WLxx Sidewalk Radio App IRQs (don't confuse with SuGHz IRQs, those are different) */

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief Generic IRQ handler for the STM32WLxx app. Fetches the active IRQ from STM32WLxx and handles it or calls Sidewalk-specific IRQ handler
 *
 * @param [in] drv_ctx Radio driver context that specifies SPI bus, GPIO pin assignments, etc.
 * @param [out] out_sidewalk_error The error specific to the Sidewalk-related event processing. Relevant only if a Sidewalk-related radio event is reported. Can be NULL
 * @return     @ref stm32wlxx_hal_status_t status code (STM32WLxx_HAL_STATUS_OK if no error occurred)
 */
stm32wlxx_hal_status_t stm32wlxx_hal_generic_irq_process(halo_drv_stm32wlxx_ctx_t * const drv_ctx, int32_t * const out_sidewalk_error);

/**
 * @brief Disables all STM32WLxx IRQs (app, radio, etc.) and removes IRQ trigger from GPIO pin
 *
 * @param [in] drv_ctx Radio driver context that specifies SPI bus, GPIO pin assignments, etc.
 * @return     @ref stm32wlxx_hal_status_t status code (STM32WLxx_HAL_STATUS_OK if no error occurred)
 */
stm32wlxx_hal_status_t stm32wlxx_hal_disarm_irq(halo_drv_stm32wlxx_ctx_t * const drv_ctx);

/**
 * @brief Configures IRQ trigger for GPIO pin and enables appl-level set of STM32WLxx IRQs
 *
 * @note SubGHz IRQ mask remains unchanged
 *
 * @param [in] drv_ctx Radio driver context that specifies SPI bus, GPIO pin assignments, etc.
 * @return     @ref stm32wlxx_hal_status_t status code (STM32WLxx_HAL_STATUS_OK if no error occurred)
 */
stm32wlxx_hal_status_t stm32wlxx_hal_arm_irq(halo_drv_stm32wlxx_ctx_t * const drv_ctx);

/**
 * @brief Performs GPIO Handshake procedure to bring STM32WLxx to the known init state
 *
 * @param [in] drv_ctx Radio driver context that specifies SPI bus, GPIO pin assignments, etc.
 * @return     @ref stm32wlxx_hal_status_t status code (STM32WLxx_HAL_STATUS_OK if no error occurred)
 */
stm32wlxx_hal_status_t stm32wlxx_hal_gpio_handshake_request(const halo_drv_stm32wlxx_ctx_t * const drv_ctx);

/**
 * @brief      Resets the SubGHz radio peripheral on STM32WLxx
 *
 * @param [in] drv_ctx Radio driver context that specifies SPI bus, GPIO pin assignments, etc.
 * @return     @ref stm32wlxx_hal_status_t status code (STM32WLxx_HAL_STATUS_OK if no error occurred)
 */
stm32wlxx_hal_status_t stm32wlxx_hal_subghz_reset(halo_drv_stm32wlxx_ctx_t * const drv_ctx);

/**
 * @brief Uploads the specified data into the STM32WLxx SubGHz's Tx buffer
 *
 * @param [in] drv_ctx  Host MCU's radio driver context
 * @param [in] data     Data to be uploaded into the STM32WLxx SubGHz Tx buffer
 * @param [in] data_len Length of the data to be uploaded
 * @returns    Operation status
 */
stm32wlxx_hal_status_t stm32wlxx_hal_set_subghz_tx_buf(halo_drv_stm32wlxx_ctx_t * const drv_ctx, const uint8_t * const data, const uint32_t data_len);

/**
 * @brief Send all the radio configuration data to be cached on STM32Wlxx side
 *
 * @note This call does not automatically apply the supplied config to the SubGHz peripheral, it just sends out the values
 *
 * @param [in] drv_ctx      Host MCU's radio driver context
 * @param [in] radio_config Radio Setup
 * @returns    Operation status
 */
stm32wlxx_hal_status_t stm32wlxx_hal_send_subghz_config(halo_drv_stm32wlxx_ctx_t * const drv_ctx, const stm32wlxx_radio_phy_cfg_t * const radio_config, const uint32_t need_ack);

/**
 * @brief Instructs STM32WLxx to apply the most essential HW settings, e.g. SubGHz clock source, regulator mode, etc.
 *
 * @note The settings to be applied should be uploaded to STM32WLxx in advance using @ref stm32wlxx_hal_send_subghz_config
 *
 * @param [in] drv_ctx Host MCU's radio driver context
 * @returns Operation status
 */
stm32wlxx_hal_status_t stm32wlxx_hal_apply_base_hw_config(halo_drv_stm32wlxx_ctx_t * const drv_ctx, const uint32_t need_ack);

/**
 * @brief Call setup of PA configuration in STM32WLxx
 *
 * @param [in] drv_ctx    Host MCU's radio driver context
 * @param [in] sid_pa_cfg New PA config for STM32WLxx
 * @returns    Operation status
 */
stm32wlxx_hal_status_t stm32wlxx_hal_set_tx_power(const halo_drv_stm32wlxx_ctx_t * const drv_ctx, const stm32wlxx_radio_pa_cfg_t * const sid_pa_cfg);

/**
 * @brief Request STM32WLxx SubGHz radio to be put into Standby state
 *
 * @param [in] drv_ctx Host MCU's radio driver context
 * @returns    Operation status
 */
stm32wlxx_hal_status_t stm32wlxx_hal_radio_standby(halo_drv_stm32wlxx_ctx_t * const drv_ctx);

/**
 * @brief Request STM32WLxx SubGHz radio to be put into Sleep mode
 *
 * @param [in] drv_ctx             Host MCU's radio driver context
 * @param [in] sleep_us            Anticipated sleep duration in microseconds
 * @param [in] deep_sleep_request Allow STM32WLxx to enter Standby or Off LPM modes - SPI communication won't be available after this request is completed
 * @returns    Operation status
 */
stm32wlxx_hal_status_t stm32wlxx_hal_radio_sleep(halo_drv_stm32wlxx_ctx_t * const drv_ctx, const uint32_t sleep_us, const uint32_t deep_sleep_request);

/**
 * @brief Call setup of Modem mode in STM32WLxx
 *
 * @param [in] drv_ctx Host MCU's radio driver context
 * @param [in] mode    Modem mode to set in STM32WLxx
 * @returns Operation status
 */
stm32wlxx_hal_status_t stm32wlxx_hal_radio_set_modem_mode(const halo_drv_stm32wlxx_ctx_t * const drv_ctx, stm32wlxx_pal_radio_modem_mode_t mode);

/**
 * @brief Call setup of RF frequency in STM32WLxx
 *
 * @param [in] drv_ctx Host MCU's radio driver context
 * @param [in]  freq to set in STM32WLxx SubGHz modem
 * @returns Operation status
 */
stm32wlxx_hal_status_t stm32wlxx_hal_radio_set_frequency(const halo_drv_stm32wlxx_ctx_t * const drv_ctx, const uint32_t freq);

/**
 * @brief Call setup of PAL layer Syncword setup procedure
 *
 * @param [in] drv_ctx       Host MCU's radio driver context
 * @param [in] target_modem  Modem to which the sync word shall be applied
 * @param [in] sync_word     Pointer to the sync word buffer
 * @param [in] sync_word_len Length of the sync word to be copied from the buffer
 * @returns Operation status
 */
stm32wlxx_hal_status_t stm32wlxx_hal_radio_set_syncword(const halo_drv_stm32wlxx_ctx_t * const drv_ctx, const stm32wlxx_pal_radio_modem_mode_t target_modem, const uint8_t * const sync_word, const uint32_t sync_word_len);

/** @brief Call setup of PAL layer LoRa symbol timeout setup procedure
 *
 * @param [in] drv_ctx      Host MCU's radio driver context
 * @param [in] symb_timeout symbols timeout
 * @returns Operation status
 */
stm32wlxx_hal_status_t stm32wlxx_hal_set_lora_symbol_timeout(const halo_drv_stm32wlxx_ctx_t * const drv_ctx, const uint8_t symb_timeout);

/**
 * @brief Call setup of PAL layer LoRa modulation params procedure
 *
 * @param [in] drv_ctx    Host MCU's radio driver context
 * @param [in] mod_params LoRa modulation params to set in STM32WLxx
 * @returns    Operation status
 */
stm32wlxx_hal_status_t stm32wlxx_hal_set_lora_modulation_params(const halo_drv_stm32wlxx_ctx_t * const drv_ctx, const stm32wlxx_radio_lora_phy_mod_params_t * const mod_params);

/**
 * @brief Call setup of PAL layer LoRa packet params procedure
 *
 * @param [in] drv_ctx    Host MCU's radio driver context
 * @param [in] pkt_params LoRa packet params to set in STM32WLxx
 *
 * @returns Operation status
 */
stm32wlxx_hal_status_t stm32wlxx_hal_radio_set_lora_pkt_params(const halo_drv_stm32wlxx_ctx_t * const drv_ctx, const stm32wlxx_radio_lora_phy_pkt_params_t pkt_params);

/**
 * @brief Call setup of PAL layer LoRa CAD params procedure
 *
 * @param [in] drv_ctx    Host MCU's radio driver context
 * @returns Operation status
 */
stm32wlxx_hal_status_t stm32wlxx_hal_radio_set_lora_cad_params(const halo_drv_stm32wlxx_ctx_t * const drv_ctx);

/**
 * @brief Call setup of PAL layer FSK mod params procedure
 *
 * @param [in] drv_ctx    Host MCU's radio driver context
 * @param [in] mod_params LoRa modulation params to set in STM32WLxx
 * @returns    Operation status
 */
stm32wlxx_hal_status_t stm32wlxx_hal_set_fsk_modulation_params(const halo_drv_stm32wlxx_ctx_t * const drv_ctx, const stm32wlxx_radio_fsk_phy_mod_params_t * const mod_params);

/**
 * @brief Call setup of PAL layer start receive procedure
 *
 * @param [in] drv_ctx    Host MCU's radio driver context
 * @param [in] timeout_us Rx timeout in us
 *
 * @returns Operation status
 */
stm32wlxx_hal_status_t stm32wlxx_hal_radio_start_rx(const halo_drv_stm32wlxx_ctx_t * const drv_ctx, const uint32_t timeout_us);

/**
 * @brief Reads out STM32WLxx Sidewalk Radio App IRQ status and any optional follow-up data (if applicable to the reported IRQ(s))
 *
 * @param [in]  drv_ctx            Host MCU's radio driver context
 * @param [out] irq_status         IRQ status response from the STM32WLxx App's side
 * @param [out] followup_data_buf  Buffer to receive any follow-up data (if the presence of such data is indicated in the IRQ status response)
 * @param [in]  followup_buf_limit Limit of how many bytes can be actually put into the @ref followup_data_buf. If received data exceeds the buffer an error is reported
 * @returns     Operation status
 */
stm32wlxx_hal_status_t stm32wlxx_hal_retrieve_radio_irq(const halo_drv_stm32wlxx_ctx_t * const drv_ctx, stm32wlxx_rcp_irq_status_t * const irq_status,
                                                        uint8_t * const followup_data_buf, const uint32_t followup_buf_limit);

/**
 * @brief Call setup of PAL layer start transmission procedure
 *
 * @param [in] drv_ctx Host MCU's radio driver context
 * @returns    Operation status
 */
stm32wlxx_hal_status_t stm32wlxx_hal_radio_start_tx(const halo_drv_stm32wlxx_ctx_t * const drv_ctx);

/**
 * @brief Sends out IRQ acknowledgment command to the STM32WLxx Sidewalk Radio App - this clears IRQ indication.
 *
 * @param [in] drv_ctx Host MCU's radio driver context
 * @returns    Operation status
 */
stm32wlxx_hal_status_t stm32wlxx_hal_acknowledge_radio_irq(const halo_drv_stm32wlxx_ctx_t * const drv_ctx);

/**
 * @brief Waits until the STM32WLxx releases the IRQ line. This method is blocking
 *
 * @param [in] drv_ctx    Host MCU's radio driver context
 * @param [in] timeout_us Timeout for releasing the IRQ line. If STM32WLxx still keeps the line an error is reported
 * @returns    Operation status
 */
stm32wlxx_hal_status_t stm32wlxx_hal_wait_radio_irq_released(const halo_drv_stm32wlxx_ctx_t * const drv_ctx, const uint32_t timeout_us);

/**
 * @brief Call setup of PAL layer Packet Params
 *
 * @param [in] drv_ctx    Host MCU's radio driver context
 * @param [in] pkt_params FSK packet params to set in STM32WLxx
 *
 * @returns Operation status
 */
stm32wlxx_hal_status_t stm32wlxx_hal_radio_set_fsk_pkt_params(const halo_drv_stm32wlxx_ctx_t * const drv_ctx, stm32wlxx_pal_radio_fsk_pkt_params_t pkt_params);

/**
 * @brief Call PAL layer start carrier sense
 *
 * @param [in] drv_ctx Host MCU's radio driver context
 * @returns    Operation status
 */
stm32wlxx_hal_status_t stm32wlxx_hal_radio_start_carrier_sense(const halo_drv_stm32wlxx_ctx_t * const drv_ctx);

#if HALO_ENABLE_DIAGNOSTICS
/**
 * @brief Call PAL layer start continuous wave tx
 *
 * @param [in] drv_ctx Host MCU's radio driver context
 * @param [in] freq    Radio frequency to use for CW output
 * @returns    Operation status
 */
stm32wlxx_hal_status_t stm32wlxx_hal_start_continuous_wave_tx(const halo_drv_stm32wlxx_ctx_t * const drv_ctx, const uint32_t freq);

/**
 * @brief Call PAL layer start continuous rx
 *
 * @param [in] drv_ctx Host MCU's radio driver context
 * @returns    Operation status
 */
stm32wlxx_hal_status_t stm32wlxx_hal_start_continuous_rx(const halo_drv_stm32wlxx_ctx_t * const drv_ctx);
#endif /* HALO_ENABLE_DIAGNOSTICS */

/**
 * @brief Sends out raw user data to the STM32WLxx
 *
 * @param [in] drv_ctx  Host MCU's radio driver context
 * @param [in] data     Data to be sent to the STM32WLxx MCU
 * @param [in] data_len Length of the data to be uploaded
 * @returns    Operation status
 */
stm32wlxx_hal_status_t stm32wlxx_hal_send_user_data(halo_drv_stm32wlxx_ctx_t * const drv_ctx, const uint8_t * const data, const uint32_t data_len);

stm32wlxx_hal_status_t stm32wlxx_radio_hal_init_gpio(halo_drv_stm32wlxx_ctx_t * const drv_ctx);
stm32wlxx_hal_status_t stm32wlxx_radio_hal_deinit_gpio(halo_drv_stm32wlxx_ctx_t * const drv_ctx);

#if STM32WLxx_RADIO_CFG_USE_STATUS_LED
/**
 * @brief Turn Tx LED on (if LED is present)
 *
 * @param [in] drv_ctx Radio implementation parameters
 *
 * @returns None
 */
stm32wlxx_hal_status_t stm32wlxx_radio_hal_tx_led_on(const halo_drv_stm32wlxx_ctx_t * const drv_ctx);

/**
 * @brief Turn Tx LED off (if LED is present)
 *
 * @param [in] drv_ctx Radio implementation parameters
 *
 * @returns None
 */
stm32wlxx_hal_status_t stm32wlxx_radio_hal_tx_led_off(const halo_drv_stm32wlxx_ctx_t * const drv_ctx);

/**
 * @brief Turn Rx LED on (if LED is present)
 *
 * @param [in] drv_ctx Radio implementation parameters
 *
 * @returns None
 */
stm32wlxx_hal_status_t stm32wlxx_radio_hal_rx_led_on(const halo_drv_stm32wlxx_ctx_t * const drv_ctx);

/**
 * @brief Turn Rx LED off (if LED is present)
 *
 * @param [in] drv_ctx Radio implementation parameters
 *
 * @returns None
 */
stm32wlxx_hal_status_t stm32wlxx_radio_hal_rx_led_off(const halo_drv_stm32wlxx_ctx_t * const drv_ctx);
#endif /* STM32WLxx_RADIO_CFG_USE_STATUS_LED */

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* __STM32WLXX_RADIO_APP_HAL_H_ */
