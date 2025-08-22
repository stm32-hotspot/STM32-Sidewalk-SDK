/**
  ******************************************************************************
  * @file    sx126x_radio_ext_ifc.h
  * @brief   Extended API for the SX126x Sidewalk radio driver
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#ifndef __SID_SX126X_RADIO_EXT_IFC_H_
#define __SID_SX126X_RADIO_EXT_IFC_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include <sid_pal_radio_ifc.h>
#include "sx126x_radio.h"

/* Exported functions prototypes ---------------------------------------------*/

#if SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION
/**
 * @brief Turn on the bridge mode for LoRa Basics Modem (LBM) and route all radio events to LBM
 *
 * @note The radio must be in Sleep state, otherwise RADIO_ERROR_BUSY will be returned
 *
 * @attention It is higly recommended to stop sub-GHz Sidewalk links via sid_stop() before enabling LBM operations
 *
 * @param[in] desired_mode Specifies the desired behavior of LBM bridge - exclusive access or transparent blanking of Sidewalk
 * @returns Non-zero value in case of error
 */
int32_t sid_pal_radio_sx126x_start_lbm_bridge_mode(const radio_sx126x_lbm_bridge_state_t desired_mode);

/**
 * @brief Turn off the bridge mode for LoRa Basics Modem (LBM) and route all radio events to Sidewalk stack
 *
 * @note This method automatically restores last known Sidewalk packet and modulation parameters
 *
 * @attention It is higly recommended to stop sub-GHz Sidewalk links via sid_stop() before enabling LBM operations
 *
 * @returns Non-zero value in case of error
 */
int32_t sid_pal_radio_sx126x_stop_lbm_bridge_mode(void);

/**
 * @brief Report current operating mode of LoRa Basics Modem (LBM) bridge in Sidewalk radio driver
 *
 * @returns Current LBM bridge operating mode. See @ref radio_sx126x_lbm_bridge_state_t
 */
radio_sx126x_lbm_bridge_state_t sid_pal_radio_sx126x_get_lbm_bridge_mode(void);
#endif /* SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION */

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* __SID_SX126X_RADIO_EXT_IFC_H_ */
