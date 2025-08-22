/**
  ******************************************************************************
  * @file    app_ble_config.h
  * @brief   BLE radio configuration for Sidewalk application
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#ifndef __SID_STM32_APP_BLE_CONFIG_H_
#define __SID_STM32_APP_BLE_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include <sid_ble_link_config_ifc.h>
#include <sid_pal_ble_adapter_stm32wba_ext_ifc.h>

/* Exported constants --------------------------------------------------------*/

#define APP_BLE_CONFIG_BEACON_VIRT_DEV_ID           (0x00u)
#define APP_BLE_CONFIG_PERIPH_LARGE_MTU_VIRT_DEV_ID (0x01u)
#define APP_BLE_CONFIG_PERIPH_SMALL_MTU_VIRT_DEV_ID (0x02u)

#ifdef __cplusplus
}
#endif

#endif /* __SID_STM32_APP_BLE_CONFIG_H_ */
