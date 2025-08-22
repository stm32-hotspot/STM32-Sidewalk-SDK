/**
  ******************************************************************************
  * @file    app_ble_config.c
  * @brief   BLE radio configuration for Sidewalk application
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

#include <sid_pal_ble_adapter_stm32wba_ext_ifc.h>
#include <sid_stm32_common_utils.h>

#include <app_conf.h>
#include <ble_defs.h>

#include "app_ble_config.h"

/* Private defines -----------------------------------------------------------*/

#if (SID_STM32_BLE_COEXISTENCE_MODE != SID_STM32_BLE_COEXISTENCE_MODE_NONE)
#  error "This configuration is not designed to be used in BLE coexistence applications"
#endif /* SID_STM32_BLE_COEXISTENCE_MODE */

/* Private constants ---------------------------------------------------------*/

static const sid_ble_adapter_ext_cfg_t ble_ext_config = {
    .sidewalk_profile = {
#if defined(NUCLEO_WBA52_BOARD)
        .device_name = "sid_sbdt_demo_nucleo-wba52",
#elif defined(NUCLEO_WBA55_BOARD)
        .device_name = "sid_sbdt_demo_nucleo-wba55",
#elif defined(NUCLEO_WBA65_BOARD)
        .device_name = "sid_sbdt_demo_nucleo-wba65",
#elif defined(STM32WBA5x)
        .device_name = "sid_sbdt_demo_stm32wba5x",
#elif defined(STM32WBA6x)
        .device_name = "sid_sbdt_demo_stm32wba6x",
#else
#  error "Unknown MCU platform"
#endif
        .max_att_mtu = SID_STM32_BLE_SIDEWALK_ATT_MTU_MAX,
    },
};

/* Global function definitions -----------------------------------------------*/

const sid_ble_adapter_ext_cfg_t * sid_pal_ble_adapter_ext_get_link_config(void)
{
    return &ble_ext_config;
}
