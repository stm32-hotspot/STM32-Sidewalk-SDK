/**
  ******************************************************************************
  * @file    stm32wlxx_radio.h
  * @brief   Sidewalk driver for the STM32WLxx Sidewalk Radio App
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

#ifndef __STM32WLXX_RADIO_H_
#define __STM32WLXX_RADIO_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>

#include "stm32wlxx_common_defs.h"
#include "stm32wlxx_app_radio_config.h"

/* Exported functions --------------------------------------------------------*/

/**
 * @brief Sends the full Sub-GHz radio configuration to the STM32WLxx coprocessor.
 * @details This function gathers the current PHY settings for both FSK and LoRa,
 * packs them into a configuration structure, and transmits them to the
 * STM32WLxx device.
 *
 * @param[in] need_ack A boolean flag (TRUE/FALSE) indicating if the driver should
 * wait for an acknowledgment from the coprocessor after sending
 * the configuration.
 * @return int32_t Returns RADIO_ERROR_NONE on successful transmission, otherwise
 * returns an error code.
 */
int32_t stm32wlxx_radio_send_subghz_config(const uint32_t need_ack);

/**
 * @brief Retrieves a pointer to the internal driver context structure.
 * @details This function provides read-only access to the driver's main
 * context, which contains state information, configuration, and interface
 * pointers.
 *
 * @return const halo_drv_stm32wlxx_ctx_t* A constant pointer to the driver's
 * context structure.
 */
const halo_drv_stm32wlxx_ctx_t * stm32wlxx_radio_get_drv_ctx_ctx(void);

/**
 * @brief Gets the current power amplifier (PA) configuration.
 * @details This function retrieves the PA settings currently held in the driver
 * context, such as duty cycle, power level, and ramp time.
 *
 * @param[out] cfg Pointer to a stm32wlxx_radio_pa_cfg_t structure where the
 * current PA configuration will be stored.
 * @return int32_t Returns RADIO_ERROR_NONE on success, or
 * RADIO_ERROR_INVALID_PARAMS if the cfg pointer is NULL.
 */
int32_t stm32wlxx_radio_get_pa_config(stm32wlxx_radio_pa_cfg_t * const cfg);

/**
 * @brief Processes a sub-GHz radio event from the STM32WLxx coprocessor.
 * @details This function is intended to be called from the main IRQ handler
 * when a sub-GHz radio-specific interrupt is detected. It interprets the
 * IRQ details, updates the radio state, and notifies the Sidewalk stack
 * of the event (e.g., RX_DONE, TX_DONE).
 * @note This function should not be called directly by the application layer.
 *
 * @return int32_t Returns RADIO_ERROR_NONE on successful event processing,
 * otherwise returns an error code.
 */
int32_t stm32wlxx_radio_sidewalk_event_process(void);

#ifdef __cplusplus
}
#endif

#endif /* __STM32WLXX_RADIO_H_ */
