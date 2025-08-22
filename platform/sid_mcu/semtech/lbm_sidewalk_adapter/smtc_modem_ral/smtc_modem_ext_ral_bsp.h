/**
  ******************************************************************************
  * @file    smtc_modem_ext_ral_bsp.h
  * @brief   Board Support Package extended definitions for LoRa Basics Modem
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

#ifndef __SID_SMTC_MODEM_EXT_RAL_BSP_IFC_H_
#define __SID_SMTC_MODEM_EXT_RAL_BSP_IFC_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Sidewalk interfaces */
#include <sid_error.h>

/* LoRa Basics Modem (LBM) interfaces */
#include <smtc_ralf_drv.h>

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief Performs board- or app-specific initialization routines and constructs RALF instance
 *
 * @returns Pointer to the RALF interface of the board
 */
const ralf_t * smtc_modem_ext_ral_bsp_init(void);

/**
 * @brief User-defined initialization code called by @ref smtc_modem_ext_ral_bsp_init during board initialization
 *
 * @note Use this function to perform application-specific actions on RAL initialization
 *
 * @note ralf_instance object is constructed by smtc_modem_ext_ral_bsp_init() before invoking this method
 *
 * @param[inout] ralf_instance Pointer to the RALF instance being constructed
 * @returns SID_ERROR_NONE on success, error-specific code otherwise
 */
sid_error_t smtc_modem_ext_ral_bsp_app_specific_init(const ralf_t * const ralf_instance);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* __SID_SMTC_MODEM_EXT_RAL_BSP_IFC_H_ */
