/**
  ******************************************************************************
  * @file           : common.c
  * @brief          : Implementation of the Sidewalk's sid_pal_delay module
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

#include <sid_error.h>
#include <sid_pal_common_ifc.h>
#include <sid_pal_log_ifc.h>
#include <sid_pal_platform_init_types.h>

#include "sid_stm32wba_sys_timer.h"

/* Private defines -----------------------------------------------------------*/

#define SID_PAL_COMMON_SET_RADIO_CFG_FUNC SID_PAL_PIT_CAT(SID_RADIO_PLATFORM, _radio_set_device_config)

/* Imported function prototypes ----------------------------------------------*/

#ifdef BLE
sid_error_t sid_stm32wba_ble_adapter_prv_platform_init(void);
#endif /* BLE */

/* Global function definitions -----------------------------------------------*/

sid_error_t sid_pal_common_init(const platform_specific_init_parameters_t *platform_init_parameters)
{
    sid_error_t ret = SID_ERROR_GENERIC;

    SID_PAL_LOG_DEBUG("sid_pal_common_init()...");
    do
    {
        /* Validate the inputs */
        if (NULL == platform_init_parameters)
        {
            SID_PAL_LOG_ERROR("platform_init_parameters cannot be NULL");
            ret = SID_ERROR_INCOMPATIBLE_PARAMS;
            break;
        }

#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 || SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
        if (NULL == platform_init_parameters->radio_cfg)
        {
            SID_PAL_LOG_ERROR("platform_init_parameters->radio_cfg cannot be NULL");
            ret = SID_ERROR_INCOMPATIBLE_PARAMS;
            break;
        }
#endif

        /* Perform the initialization actions */
        ret = sid_stm32wba_sys_timer_init();
        if (ret != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Sidewalk system timer init failed err: %d", ret);
            break;
        }

#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 || SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
        SID_PAL_COMMON_SET_RADIO_CFG_FUNC(platform_init_parameters->radio_cfg);
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 || SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 */

#ifdef BLE
        ret = sid_stm32wba_ble_adapter_prv_platform_init();
        if (ret != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Sidewalk BLE subsystem init failed err: %d", ret);
            break;
        }
#endif /* BLE */

        /* Everything is fine if we've got to this point */
        ret = SID_ERROR_NONE;
    } while (0);

    return ret;
}

/*----------------------------------------------------------------------------*/

sid_error_t sid_pal_common_deinit(void)
{
    SID_PAL_LOG_DEBUG("sid_pal_common_deinit()...");
    sid_error_t ret = SID_ERROR_GENERIC;

    do
    {
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 || SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
        // FIXME: sid_pal_radio_deinit wasn't called by sid_deinit function
        // It's a sidewalk sdk issue.
        // workaround is to call sid_pal_radio_deinit explicitly
        ret = sid_pal_radio_deinit();
        if (ret != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Sidewalk system timer deinit failed err: %d", ret);
            break;
        }
#endif

        /* Perform the initialization actions */
        ret = sid_stm32wba_sys_timer_deinit();
        if (ret != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Sidewalk system timer deinit failed err: %d", ret);
            break;
        }
    } while (0);

    return ret;
}
