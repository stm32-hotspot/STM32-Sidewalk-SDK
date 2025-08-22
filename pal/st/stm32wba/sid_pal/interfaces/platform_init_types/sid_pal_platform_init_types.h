/**
  ******************************************************************************
  * @file           : sid_pal_platform_init_types.h
  * @brief          : STM32WBA-specific SID platform initialization data types
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023-2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#ifndef __SID_PAL_PLATFORM_INIT_TYPES_H_
#define __SID_PAL_PLATFORM_INIT_TYPES_H_

/* Global macro --------------------------------------------------------------*/
/* Ensure radio link selection is configured properly */
#if !defined(SID_SDK_CONFIG_ENABLE_LINK_TYPE_1)
#  error "SID_SDK_CONFIG_ENABLE_LINK_TYPE_1 parameter is not defined. Please set it explicitly to 0 or 1"
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_1 */

#if !defined(SID_SDK_CONFIG_ENABLE_LINK_TYPE_2)
#  error "SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 parameter is not defined. Please set it explicitly to 0 or 1"
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 */

#if !defined(SID_SDK_CONFIG_ENABLE_LINK_TYPE_3)
#  error "SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 parameter is not defined. Please set it explicitly to 0 or 1"
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 */

#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 || SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
   /* Check that SubGHz radio platform is specified for FSK and LoRa links */
#  if !defined(SID_RADIO_PLATFORM)
#    error "SID_RADIO_PLATFORM parameter is not defined. Please set it explicitly to one of the supported radio platforms"
#  endif /* SID_RADIO_PLATFORM */

#  define SID_PAL_PIT_STR_HELPER(__s__)       #__s__
#  define SID_PAL_PIT_STR(__s__)              SID_PAL_PIT_STR_HELPER(__s__)

#  define SID_PAL_PIT_CAT_HELPER(__a__,__b__) __a__##__b__
#  define SID_PAL_PIT_CAT(__a__,__b__)        SID_PAL_PIT_CAT_HELPER(__a__,__b__)

#  define SID_PAL_PIT_RADIO_CFG_HEADER_PROTO  SID_PAL_PIT_CAT(SID_RADIO_PLATFORM, _radio_config.h)
#  define SID_PAL_PIT_RADIO_CFG_HEADER        SID_PAL_PIT_STR(SID_PAL_PIT_RADIO_CFG_HEADER_PROTO)

#  define SID_PAL_PIT_RADIO_CFG_DATATYPE      SID_PAL_PIT_CAT(SID_RADIO_PLATFORM, _radio_device_config_t)

#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 || SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 */

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 || SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
#  include SID_PAL_PIT_RADIO_CFG_HEADER
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 || SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 */

/* Exported types ------------------------------------------------------------*/
/* Placeholder for the platform-specific init parameters */
typedef struct {
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 || SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
    const SID_PAL_PIT_RADIO_CFG_DATATYPE * radio_cfg;
#else
    uint32_t _unused; /* Dummy parameter to avoid empty struct declaration */
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 || SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 */
} platform_specific_init_parameters_t;

#endif /* __SID_PAL_PLATFORM_INIT_TYPES_H_ */
