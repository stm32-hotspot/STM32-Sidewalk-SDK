/**
  ******************************************************************************
  * @file    smtc_modem_ext_api.h
  * @brief   API extensions to the LoRa Basics Modem for Sidewalk compatibility
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

#ifndef __SID_SMTC_MODEM_EXT_API_IFC_H_
#define __SID_SMTC_MODEM_EXT_API_IFC_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* LoRa Basics Modem (LBM) interfaces */
#include <smtc_modem_api.h>

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief Puts the radio into sleep mode with data retention
 *
 * @param[in] sleep_duration_us Desired sleep duration in us. 0 or UINT32_MAX means infinite sleep
 */
smtc_modem_return_code_t smtc_modem_ext_sleep(const uint32_t sleep_duration_us);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* __SID_SMTC_MODEM_EXT_API_IFC_H_ */
