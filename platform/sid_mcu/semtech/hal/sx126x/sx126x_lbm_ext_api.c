/**
  ******************************************************************************
  * @file    sx126x_lbm_ext_api.c
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

/* Includes ------------------------------------------------------------------*/

/* Sidewalk interfaces */
#include <sid_pal_assert_ifc.h>
#include <sid_pal_log_ifc.h>

/* LoRa Basics Modem (LBM) interfaces */
#include <smtc_modem_ext_api.h>

/* Sidewalk radio driver */
#include "sx126x_hal.h"
#include "sx126x_radio.h"

/* Platform-specific includes */
#include <sid_stm32_common_utils.h>

/* Private defines -----------------------------------------------------------*/

#ifndef SMTC_MODEM_EXT_API_EXTRA_LOGGING
/* Set SMTC_MODEM_EXT_API_EXTRA_LOGGING to 1 to enable extended logs */
#  define SMTC_MODEM_EXT_API_EXTRA_LOGGING    (0)
#endif

#if SMTC_MODEM_EXT_API_EXTRA_LOGGING
#  define SMTC_MODEM_EXT_API_LOG_ERROR(...)   SID_PAL_LOG_ERROR(__VA_ARGS__)
#  define SMTC_MODEM_EXT_API_LOG_WARNING(...) SID_PAL_LOG_WARNING(__VA_ARGS__)
#  define SMTC_MODEM_EXT_API_LOG_INFO(...)    SID_PAL_LOG_INFO(__VA_ARGS__)
#  define SMTC_MODEM_EXT_API_LOG_DEBUG(...)   SID_PAL_LOG_DEBUG(__VA_ARGS__)
#  define SMTC_MODEM_EXT_API_LOG_TRACE(...)   SID_PAL_LOG_TRACE(__VA_ARGS__)
#else
#  define SMTC_MODEM_EXT_API_LOG_ERROR(...)   ((void)0u)
#  define SMTC_MODEM_EXT_API_LOG_WARNING(...) ((void)0u)
#  define SMTC_MODEM_EXT_API_LOG_INFO(...)    ((void)0u)
#  define SMTC_MODEM_EXT_API_LOG_DEBUG(...)   ((void)0u)
#  define SMTC_MODEM_EXT_API_LOG_TRACE(...)   ((void)0u)
#endif /* SMTC_MODEM_EXT_API_EXTRA_LOGGING */

/* Global function definitions -----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED smtc_modem_return_code_t smtc_modem_ext_sleep(const uint32_t sleep_duration_us)
{
    const halo_drv_semtech_ctx_t * const drv_ctx = sx126x_get_drv_ctx();
    const uint32_t sleep_duration_ticks = ((0u == sleep_duration_us) || (UINT32_MAX == sleep_duration_us)) ? SX126X_RADIO_INFINITE_TIME : SX126X_RX_TX_TIMEOUT_US_TO_TUS(sleep_duration_us);
    sx126x_status_t sys_err;
    sx126x_hal_status_t hal_err;
    smtc_modem_return_code_t ret;

    /* Validate inputs */
    SID_PAL_ASSERT(drv_ctx != NULL);

    do
    {
        if (sleep_duration_ticks != SX126X_RADIO_INFINITE_TIME)
        {
            SMTC_MODEM_EXT_API_LOG_ERROR("SX126x driver doesn't support timed sleep, only sleep with no timeout can be used");
            ret = SMTC_MODEM_RC_INVALID;
            break;
        }

        if (FALSE == drv_ctx->init_done)
        {
            SMTC_MODEM_EXT_API_LOG_ERROR("Can't put radio into sleep. SX126x Sidewalk driver is not initialized");
            ret = SMTC_MODEM_RC_NOT_INIT;
            break;
        }

        sys_err = sx126x_set_sleep(drv_ctx, SX126X_SLEEP_CFG_WARM_START);
        if (sys_err != SX126X_STATUS_OK)
        {
            SMTC_MODEM_EXT_API_LOG_ERROR("Failed to put the radio to sleep. Error %d", (int32_t)sys_err);
            ret = SMTC_MODEM_RC_FAIL;
            break;
        }

        /* Ensure the radio rises Busy pin as indication of entering Sleep state */
        hal_err = sx126x_hal_wait_busy_indicated(drv_ctx);
        if (hal_err != SX126X_HAL_STATUS_OK)
        {
            /* Logs provided by sx126x_hal_wait_busy_indicated() */
            ret = SMTC_MODEM_RC_FAIL;
            break;
        }

        ret = SMTC_MODEM_RC_OK;
    } while (0);

    if (ret != SMTC_MODEM_RC_OK)
    {
        /* Re-enable IRQ line */
        hal_err = sx126x_hal_arm_irq(drv_ctx);
        if (hal_err != SX126X_HAL_STATUS_OK)
        {
            SMTC_MODEM_EXT_API_LOG_ERROR("Failed to re-arm radio IRQ on sleep entry failure. Error %d", (int32_t)hal_err);
            ret = SMTC_MODEM_RC_FAIL;
        }
    }

    return ret;
}
