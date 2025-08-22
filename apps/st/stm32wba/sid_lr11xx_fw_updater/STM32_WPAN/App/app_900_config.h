/**
  ******************************************************************************
  * @file    app_900_config.h
  * @brief   Sub-GHz radio configuration for Sidewalk application
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023-2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#ifndef __APP_900_CONFIG_H_
#define __APP_900_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Exported macro ------------------------------------------------------------*/

#define APP_900_CONFIG_STR_HELPER(__s__)       #__s__
#define APP_900_CONFIG_STR(__s__)              APP_900_CONFIG_STR_HELPER(__s__)

#define APP_900_CONFIG_CAT_HELPER(__a__,__b__) __a__##__b__
#define APP_900_CONFIG_CAT(__a__,__b__)        APP_900_CONFIG_CAT_HELPER(__a__,__b__)

#define APP_900_CONFIG_RADIO_CFG_HEADER_PROTO  APP_900_CONFIG_CAT(SID_RADIO_PLATFORM, _radio_config.h)
#define APP_900_CONFIG_RADIO_CFG_HEADER        APP_900_CONFIG_STR(APP_900_CONFIG_RADIO_CFG_HEADER_PROTO)

#define APP_900_CONFIG_RADIO_CFG_DATATYPE      APP_900_CONFIG_CAT(SID_RADIO_PLATFORM, _radio_device_config_t)

/* Includes ------------------------------------------------------------------*/

#include APP_900_CONFIG_RADIO_CFG_HEADER
#include <sid_900_cfg.h>

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief Returns a pointer to the SubGHz radio hardware configuration
 */
const APP_900_CONFIG_RADIO_CFG_DATATYPE * get_radio_cfg(void);

/**
 * @brief Returns the pointer to the hardware-independent SubGHz Sidewalk link configuration
 */
const struct sid_sub_ghz_links_config * app_get_sub_ghz_config(void);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* __APP_900_CONFIG_H_ */

