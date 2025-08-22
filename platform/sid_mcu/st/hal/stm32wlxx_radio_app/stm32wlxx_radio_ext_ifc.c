/**
  ******************************************************************************
  * @file    stm32wlxx_radio_ext_ifc.c
  * @brief   Extended API for the STM32WLxx Sidewalk Radio App
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

/* Includes ------------------------------------------------------------------*/

#include <sid_pal_critical_region_ifc.h>
#include <sid_pal_log_ifc.h>

#include <cmsis_os2.h>

#include "stm32wlxx_radio.h"
#include "stm32wlxx_radio_hal.h"
#include "stm32wlxx_radio_ext_ifc.h"

#include <sid_stm32_common_utils.h>

/* Global function definitions -----------------------------------------------*/

sid_error_t sid_pal_radio_stm32wlxx_set_user_data_received_cb(stm32wlxx_ext_ifc_on_incoming_user_data callback)
{
#if STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER
    sid_error_t err = SID_ERROR_GENERIC;

    do
    {
        halo_drv_stm32wlxx_ctx_t * const drv_ctx = (halo_drv_stm32wlxx_ctx_t *)stm32wlxx_radio_get_drv_ctx_ctx();

        if (NULL == drv_ctx)
        {
            err = SID_ERROR_NULL_POINTER;
            break;
        }

        sid_pal_enter_critical_region();
        drv_ctx->on_user_data_rx = callback;
        sid_pal_exit_critical_region();

        if (callback != NULL)
        {
            SID_PAL_LOG_DEBUG("STM32WLxx On User Data Received callback set");
        }
        else
        {
            SID_PAL_LOG_DEBUG("STM32WLxx On User Data Received callback cleared");
        }

        err = SID_ERROR_NONE;
    } while (0);

    return err;
#else
    (void)callback;
    return SID_ERROR_NOSUPPORT;
#endif /* STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER */
}

/*----------------------------------------------------------------------------*/

sid_error_t sid_pal_radio_stm32wlxx_send_user_data(const uint8_t * const data, const uint32_t data_len, const uint32_t auto_free)
{
#if STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER
    sid_error_t err = SID_ERROR_GENERIC;
    osStatus_t os_err;

    do
    {
        /* Validate the inputs */
        if ((NULL == data) || (0u == data_len))
        {
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        /* Check driver status */
        halo_drv_stm32wlxx_ctx_t * const drv_ctx = (halo_drv_stm32wlxx_ctx_t *)stm32wlxx_radio_get_drv_ctx_ctx();

        if (NULL == drv_ctx)
        {
            err = SID_ERROR_NULL_POINTER;
            break;
        }

        if ((FALSE == drv_ctx->init_done) || (NULL == drv_ctx->udt_ctx.outbound_msg_queue))
        {
            /* Driver is not ready to accept user messages */
            err = SID_ERROR_INVALID_STATE;
            break;
        }

        /* Put the data in the queue */
        stm32wlxx_ext_ifc_out_msg_desc_t msg = {
            .data_ptr =  (uint8_t *)data,
            .data_len  = data_len,
            .auto_free = auto_free,
        };

        os_err = osMessageQueuePut(drv_ctx->udt_ctx.outbound_msg_queue, &msg, 0u, 0u);
        if (os_err != osOK)
        {
            /* Failed to put the message in the queue, probably it is full */
            err = SID_ERROR_OUT_OF_RESOURCES;
            break;
        }

        err = SID_ERROR_NONE;
    } while (0);

    return err;
#else
    (void)data;
    (void)data_len;
    (void)auto_free;
    return SID_ERROR_NOSUPPORT;
#endif /* STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER */
}

/*----------------------------------------------------------------------------*/

sid_error_t sid_pal_radio_stm32wlxx_deep_sleep(void)
{
    sid_error_t err = SID_ERROR_GENERIC;
    stm32wlxx_hal_status_t hal_err;

    /* Check driver status */
    halo_drv_stm32wlxx_ctx_t * const drv_ctx = (halo_drv_stm32wlxx_ctx_t *)stm32wlxx_radio_get_drv_ctx_ctx();
    if ((NULL == drv_ctx) || (FALSE == drv_ctx->init_done))
    {
        SID_PAL_LOG_ERROR("Can't put STM32WLxx into Deep Sleep - radio driver is not initialized");
        return SID_ERROR_UNINITIALIZED;
    }

    sid_pal_enter_critical_region();

    /* Store a copy of the app IRQ mask on entry */
    const uint16_t app_irq_mask = drv_ctx->app_irq_mask;

    do
    {
        /* Check radio state */
        if (drv_ctx->radio_state != SID_PAL_RADIO_SLEEP)
        {
            SID_PAL_LOG_ERROR("SubGHz is not in sleep state, Deep Sleep request skipped"); 
            err = SID_ERROR_INVALID_STATE;
            break;
        }

        /* Mask SubGHz events until the radio reports back it has entered sleep state */
        drv_ctx->app_irq_mask &= ~STM32WLxx_APP_IRQ_SUBGHZ;

        hal_err = stm32wlxx_hal_radio_sleep(drv_ctx, 0u, TRUE);
        if (hal_err != STM32WLxx_HAL_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to put STM32WLxx into Deep Sleep mode. HAL error %u", (uint32_t)hal_err);
            err = SID_ERROR_INVALID_RESPONSE;
            break;
        }

#if STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER
        /* UDT cannot be used in Deep Sleep since SPI communication won't be functioning */
        drv_ctx->udt_ctx.udt_enabled = FALSE;
#endif

        /* Disable radio IRQ line since its state may not be well defined when STM32WLxx is in Standby/Off or when it is waking up */
        hal_err = stm32wlxx_hal_disarm_irq(drv_ctx);
        if (hal_err != STM32WLxx_HAL_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to put STM32WLxx into Deep Sleep mode. HAL error %u", (uint32_t)hal_err);
            err = SID_ERROR_IO_ERROR;
            break;
        }

        err = SID_ERROR_NONE;
    } while (0);

    /* Restore the app IRQ mask */
    drv_ctx->app_irq_mask = app_irq_mask;

    sid_pal_exit_critical_region();

    return err;
}
