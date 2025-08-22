/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    host_ble.h
  * @author  MCD Application Team
  * @brief   Header for host_ble.c
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022-2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HOST_BLE_H
#define HOST_BLE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include <cmsis_compiler.h> /* Required for ble_types.h */
#include <ble_types.h>
#include <sid_pal_ble_adapter_stm32wba_private_defs.h>
#include <sid_pal_ble_adapter_stm32wba_ext_ifc.h>

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* External variables --------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

#define SCAN_WIN_MS(x) ((uint16_t)((x)/0.625f))
#define SCAN_INT_MS(x) ((uint16_t)((x)/0.625f))
#define CONN_INT_MS(x) ((uint16_t)((x)/1.25f))
#define CONN_SUP_TIMEOUT_MS(x) ((uint16_t)((x)/10.0f))
#define CONN_CE_LENGTH_MS(x) ((uint16_t)((x)/0.625f))

#define SID_ADV_INT_TO_MS(x) ((uint16_t)((x)*0.625f))
#define MS_ADV_INT_TO_SID(x) ((uint16_t)((x)/0.625f))
#define MS_ADV_TOUT_TO_SID(x) ((uint16_t)((x)/10u))

#define BLE_HOST_NUM_GATT_SYSTEM_SERVICES (2)

/* Exported functions ------------------------------------------------------- */

/**
 * @brief  BLE Host initialization
 *
 * @param [in]  cfg BLE configuration supplied by the user app
 * @param [in]  init_type Stack initialization target, e.g. Sidewalk link, user-defined profile, etc.
 * @param [in]  nvm_cache_buffer RAM buffer for BLE host NVM data. @attention The size of the buffer shall be not less than CFG_BLE_NVM_SIZE_MAX
 * @param [out] out_mtu_size ATT MTU size selected during the initialization. None of the connections can exceed this limit
 * @retval      BLE_STATUS_SUCCESS on success, or error code otherwise
 */
tBleStatus HOST_BLE_Init(const sid_ble_adapter_ext_cfg_t * const cfg, const sid_pal_ble_prv_operating_mode_t init_type, uint64_t * const nvm_cache_buffer, uint16_t * const out_mtu_size);

#ifdef __cplusplus
}
#endif

#endif /*HOST_BLE_H */
