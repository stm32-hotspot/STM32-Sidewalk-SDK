/**
  ******************************************************************************
  * @file    stm32wlxx_radio_hal.c
  * @brief   hardware-specific operations to drive the Sidewalk STM32WLxx Radio
  *
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

/* Includes ------------------------------------------------------------------*/

/* STD headers */
#include <stdint.h>

/* Sidewalk SDK headers */
#include <sid_pal_assert_ifc.h>
#include <sid_pal_critical_region_ifc.h>
#include <sid_pal_delay_ifc.h>
#include <sid_pal_gpio_ifc.h>
#include <sid_pal_gpio_ext_ifc.h>
#include <sid_pal_log_ifc.h>
#include <sid_pal_serial_bus_ext_ifc.h>
#include <sid_pal_uptime_ifc.h>
#include <sid_time_ops.h>
#include <sid_time_types.h>

/* Radio driver headers */
#include <comm_def.h>
#include <comm_opcodes.h>
#include "stm32wlxx_app_radio_config.h"
#include "stm32wlxx_radio.h"
#include "stm32wlxx_radio_hal.h"
#include "stm32wlxx_phy.h"

/* ST HW platform headers */
#include <sid_stm32_common_utils.h>
#include "stm32wbaxx_hal.h"

/* OS */
#include "cmsis_os2.h"

/* Private defines -----------------------------------------------------------*/

#define STM32WLxx_HAL_IRQ_LINE_GPIO_SETTLE_TIME_US        (75u)  /* Delay after configuring the GPIO as input with a PU/PD to allow it to settle */
#define STM32WLxx_HAL_GPIO_HANDSHAKE_WAIT_TIMEOUT_MS      (200u) /* The time given to the STM32WLxx to react on GPIO handshake request */
#define STM32WLxx_HAL_GPIO_HANDSHAKE_PROBE_PERIOD_MS      (1u)   /* The pause between the continuous GPIO handshake state probes */

#define STM32WLxx_HAL_RADIO_REQUEST_ACK_PROBE_PERIOD_US   (100u) /* If SubGHz request is running in IRQ context the handler will use polling with this period to check if the request was completed */

#define STM32WLxx_HAL_RADIO_SUBGHZ_RESET_TIMEOUT_MS       (50u)  /* Time limit for STM32WLxx completing the SubGHz Reset request and reporting back to the host MCU */
#define STM32WLxx_HAL_RADIO_SUBGHZ_STANDBY_TIMEOUT_MS     (20u)  /* Time limit for STM32WLxx completing the SubGHz Standby request and reporting back to the host MCU */
#define STM32WLxx_HAL_RADIO_SUBGHZ_SLEEP_TIMEOUT_MS       (5u)   /* Time limit for STM32WLxx completing the SubGHz Sleep mode request and reporting back to the host MCU */
#define STM32WLxx_HAL_RADIO_SUBGHZ_SEND_CFG_TIMEOUT_MS    (2u)   /* Time limit for STM32WLxx for acknowledging the reception of the supplied SubGHz configuration */
#define STM32WLxx_HAL_RADIO_SUBGHZ_APPLY_CFG_TIMEOUT_MS   (30u)  /* Time limit for STM32WLxx completing the application of the supplied SubGHz configuration */

#define STM32WLxx_HAL_RADIO_SUBGHZ_SPI_FLUSH_ATTEMPTS     (100u) /* Maximum amount of dummy reads to flush SPI communication - STM32WLxx shall sent a dummy frame before this limit is reached */

#define STM32WLxx_HAL_STM32WLxx_WAKEUP_TIME_US            (450u) /* STM32WLxx wakeup delay - HAL will wait at least for this time to allow STM32WLxx to wakeup and enter Run mode. IMPORTANT: this is not a full wakeup, it's just the time required to rstart the clocks and enter Run mode */

#ifndef STM32WLxx_RADIO_APP_LPM_SUPPORT
#  error "STM32WLxx_RADIO_APP_LPM_SUPPORT is not defined. Please set it explicitly to 0 or 1"
#endif /* STM32WLxx_RADIO_APP_LPM_SUPPORT */

/* Private macro -------------------------------------------------------------*/

#ifndef STM32WLxx_HAL_EXTRA_LOGGING
/* Set STM32WLxx_HAL_EXTRA_LOGGING to 1 to enable extended logs */
#  define STM32WLxx_HAL_EXTRA_LOGGING (0)
#endif

#if defined(STM32WLxx_HAL_EXTRA_LOGGING) && STM32WLxx_HAL_EXTRA_LOGGING
#  define STM32WLxx_HAL_LOG_ERROR(...)   SID_PAL_LOG_ERROR(__VA_ARGS__)
#  define STM32WLxx_HAL_LOG_WARNING(...) SID_PAL_LOG_WARNING(__VA_ARGS__)
#  define STM32WLxx_HAL_LOG_INFO(...)    SID_PAL_LOG_INFO(__VA_ARGS__)
#  define STM32WLxx_HAL_LOG_DEBUG(...)   SID_PAL_LOG_DEBUG(__VA_ARGS__)
#  define STM32WLxx_HAL_LOG_TRACE(...)   SID_PAL_LOG_TRACE(__VA_ARGS__)
#else
#  define STM32WLxx_HAL_LOG_ERROR(...)   ((void)0u)
#  define STM32WLxx_HAL_LOG_WARNING(...) ((void)0u)
#  define STM32WLxx_HAL_LOG_INFO(...)    ((void)0u)
#  define STM32WLxx_HAL_LOG_DEBUG(...)   ((void)0u)
#  define STM32WLxx_HAL_LOG_TRACE(...)   ((void)0u)
#endif

/* Private function prototypes -----------------------------------------------*/

static inline stm32wlxx_hal_status_t _spi_transmit_receive_full_frame(const halo_drv_stm32wlxx_ctx_t * const drv_ctx, const uint8_t * const tx_buf,
                                                                      uint8_t * const rx_buf, const size_t data_length);
static inline stm32wlxx_hal_status_t _spi_transmit_receive_partial_frame(const halo_drv_stm32wlxx_ctx_t * const drv_ctx, const uint8_t * const tx_buf,
                                                                         uint8_t * const rx_buf, const size_t data_length);
static        stm32wlxx_hal_status_t _spi_transmit_data(const halo_drv_stm32wlxx_ctx_t * const drv_ctx, const uint8_t * const data, const size_t data_length);

static        stm32wlxx_hal_status_t _spi_receive_data(const halo_drv_stm32wlxx_ctx_t * const drv_ctx, uint8_t * const rx_buf,
                                                       const size_t buf_size, size_t * const bytes_received);
static        stm32wlxx_hal_status_t _spi_transmit_receive_data(const halo_drv_stm32wlxx_ctx_t * const drv_ctx, const uint8_t * const tx_data,
                                                                uint8_t * const rx_data, const size_t data_length);
static         stm32wlxx_hal_status_t _flush_spi(const halo_drv_stm32wlxx_ctx_t * const drv_ctx);
static         stm32wlxx_hal_status_t _send_request_with_ack(halo_drv_stm32wlxx_ctx_t * const drv_ctx, const uint8_t * const tx_data,
                                                             const size_t data_length, const uint32_t timeout_ms);
static inline  uint32_t               _radio_irq_gpio_filtered_read(const halo_drv_stm32wlxx_ctx_t * const drv_ctx);
static         void                   _on_radio_irq_detected(uint32_t pin, void * callback_arg);

/* Private function definitions ----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline stm32wlxx_hal_status_t _spi_transmit_receive_full_frame(const halo_drv_stm32wlxx_ctx_t * const drv_ctx, const uint8_t * const tx_buf, uint8_t * const rx_buf, const size_t data_length)
{
    stm32wlxx_hal_status_t err = STM32WLxx_HAL_STATUS_ERROR_GENERIC;
    sid_error_t sid_err;

    SID_PAL_ASSERT(data_length == STM32WLxx_RADIO_COMM_SPI_DATA_FRAME_SIZE);

    do
    {
#if STM32WLxx_RADIO_APP_LPM_SUPPORT
        /* If STM32WLxx can be in an LPM we need to activate NSS and hold it to wakeup the MCU */
        uint32_t wakeup_delay_required = ((SID_PAL_RADIO_SLEEP == drv_ctx->radio_state) || (SID_PAL_RADIO_UNKNOWN == drv_ctx->radio_state)) && (FALSE == drv_ctx->ldt_ongoing) ? TRUE : FALSE;
        if ((wakeup_delay_required != FALSE) && (SID_PAL_RADIO_SLEEP == drv_ctx->radio_state))
        {
            /* There's a special use case for radio in SID_PAL_RADIO_SLEEP but indicating active IRQ - we don't need to wait for wakeup here */
            const uint32_t radio_irq_pin_state = _radio_irq_gpio_filtered_read(drv_ctx);

            if (STM32WLxx_HAL_RADIO_IRQ_PIN_STATE_ACTIVE == radio_irq_pin_state)
            {
                wakeup_delay_required = FALSE;
            }
        }

        /* Activate NSS line and hold it if wakeup is needed */
        if (wakeup_delay_required != FALSE)
        {
            const sid_pal_serial_bus_stm32wbaxx_transaction_config_t * const spi_transaction_cfg = (sid_pal_serial_bus_stm32wbaxx_transaction_config_t *)drv_ctx->config->spi_client_cfg->client_selector_extension;
            const uint32_t nss_to_nss_delay = ((spi_transaction_cfg->nss_to_nss_cycles * 1000000u) + drv_ctx->config->spi_client_cfg->speed_hz) / drv_ctx->config->spi_client_cfg->speed_hz; /* Round-up */

            /* Ensure NSS-to-NSS delay is respected */
            sid_pal_delay_us(nss_to_nss_delay);

            sid_err = sid_pal_serial_bus_ext_ifc_activate_nss(drv_ctx->bus_iface, drv_ctx->config->spi_client_cfg);
            if (sid_err != SID_ERROR_NONE)
            {
                STM32WLxx_HAL_LOG_ERROR("Failed to activate STM32WLxx NSS line for wakeup. sid_pal_serial_bus_spi error %d", (int32_t)sid_err);
                err = STM32WLxx_HAL_STATUS_ERROR_SPI_XFER;
                break;
            }

            /* Wait for STM32WLxx MCU to wakeup */
            if (STM32WLxx_HAL_STM32WLxx_WAKEUP_TIME_US > nss_to_nss_delay)
            {
                sid_pal_delay_us(STM32WLxx_HAL_STM32WLxx_WAKEUP_TIME_US - nss_to_nss_delay);
            }
        }
#endif /* STM32WLxx_RADIO_APP_LPM_SUPPORT */

        /* Do the transfer */
        sid_err = drv_ctx->bus_iface->xfer(drv_ctx->bus_iface, drv_ctx->config->spi_client_cfg, (uint8_t *)tx_buf, (uint8_t *)rx_buf, STM32WLxx_RADIO_COMM_SPI_DATA_FRAME_SIZE);
        if (sid_err != SID_ERROR_NONE)
        {
            STM32WLxx_HAL_LOG_ERROR("Failed to xfer single SPI frame to STM32WLxx. sid_pal_serial_bus_spi error %d", (int32_t)sid_err);
            err = STM32WLxx_HAL_STATUS_ERROR_SPI_XFER;
            break;
        }

#if STM32WLxx_RADIO_APP_LPM_SUPPORT
        /* Release the manual NSS control if it was requested */
        if (wakeup_delay_required != FALSE)
        {
            sid_err = sid_pal_serial_bus_ext_ifc_deactivate_nss(drv_ctx->bus_iface, drv_ctx->config->spi_client_cfg);
            if (sid_err != SID_ERROR_NONE)
            {
                STM32WLxx_HAL_LOG_ERROR("Failed to deactivate STM32WLxx NSS line after wakeup. sid_pal_serial_bus_spi error %d", (int32_t)sid_err);
                err = STM32WLxx_HAL_STATUS_ERROR_SPI_XFER;
                break;
            }
        }
#endif /* STM32WLxx_RADIO_APP_LPM_SUPPORT */

        /* Done */
        err = STM32WLxx_HAL_STATUS_OK;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline stm32wlxx_hal_status_t _spi_transmit_receive_partial_frame(const halo_drv_stm32wlxx_ctx_t * const drv_ctx, const uint8_t * const tx_buf, uint8_t * const rx_buf, const size_t data_length)
{
    stm32wlxx_hal_status_t err = STM32WLxx_HAL_STATUS_ERROR_GENERIC;
    sid_error_t sid_err;
    uint8_t full_size_tx_buf[STM32WLxx_RADIO_COMM_SPI_DATA_FRAME_SIZE];
    uint8_t full_size_rx_buf[STM32WLxx_RADIO_COMM_SPI_DATA_FRAME_SIZE];
    uint8_t * tx_ptr;
    uint8_t * rx_ptr;

    SID_PAL_ASSERT(data_length < STM32WLxx_RADIO_COMM_SPI_DATA_FRAME_SIZE);

    STM32WLxx_HAL_LOG_DEBUG("_spi_transmit_data() with padding. %u bytes padded", STM32WLxx_RADIO_COMM_SPI_DATA_FRAME_SIZE - data_length);

    do
    {
        if (tx_buf != NULL)
        {
            /* Build Tx buffer with padding */
            SID_STM32_UTIL_fast_memset(full_size_tx_buf, 0u, sizeof(tx_buf));
            SID_STM32_UTIL_fast_memcpy(full_size_tx_buf, tx_buf, data_length);
            tx_ptr = full_size_tx_buf;
        }
        else
        {
            /* No Tx */
            tx_ptr = NULL;
        }

        /* Prepare Rx buffer */
        if (rx_buf != NULL)
        {
            tx_ptr = full_size_rx_buf;
        }
        else
        {
            /* No Rx */
            rx_ptr = NULL;
        }

#if STM32WLxx_RADIO_APP_LPM_SUPPORT
        /* If STM32WLxx can be in an LPM we need to activate NSS and hold it to wakeup the MCU */
        uint32_t wakeup_delay_required = ((SID_PAL_RADIO_SLEEP == drv_ctx->radio_state) || (SID_PAL_RADIO_UNKNOWN == drv_ctx->radio_state)) && (FALSE == drv_ctx->ldt_ongoing) ? TRUE : FALSE;
        if ((wakeup_delay_required != FALSE) && (SID_PAL_RADIO_SLEEP == drv_ctx->radio_state))
        {
            /* There's a special use case for radio in SID_PAL_RADIO_SLEEP but indicating active IRQ - we don't need to wait for wakeup here */
            const uint32_t radio_irq_pin_state = _radio_irq_gpio_filtered_read(drv_ctx);

            if (STM32WLxx_HAL_RADIO_IRQ_PIN_STATE_ACTIVE == radio_irq_pin_state)
            {
                wakeup_delay_required = FALSE;
            }
        }

        /* Activate NSS line and hold it if wakeup is needed */
        if (wakeup_delay_required != FALSE)
        {
            const sid_pal_serial_bus_stm32wbaxx_transaction_config_t * const spi_transaction_cfg = (sid_pal_serial_bus_stm32wbaxx_transaction_config_t *)drv_ctx->config->spi_client_cfg->client_selector_extension;
            const uint32_t nss_to_nss_delay = ((spi_transaction_cfg->nss_to_nss_cycles * 1000000u) + drv_ctx->config->spi_client_cfg->speed_hz) / drv_ctx->config->spi_client_cfg->speed_hz; /* Round-up */

            /* Ensure NSS-to-NSS delay is respected */
            sid_pal_delay_us(nss_to_nss_delay);

            sid_err = sid_pal_serial_bus_ext_ifc_activate_nss(drv_ctx->bus_iface, drv_ctx->config->spi_client_cfg);
            if (sid_err != SID_ERROR_NONE)
            {
                STM32WLxx_HAL_LOG_ERROR("Failed to activate STM32WLxx NSS line for wakeup. sid_pal_serial_bus_spi error %d", (int32_t)sid_err);
                err = STM32WLxx_HAL_STATUS_ERROR_SPI_XFER;
                break;
            }

            /* Wait for STM32WLxx MCU to wakeup */
            if (STM32WLxx_HAL_STM32WLxx_WAKEUP_TIME_US > nss_to_nss_delay)
            {
                sid_pal_delay_us(STM32WLxx_HAL_STM32WLxx_WAKEUP_TIME_US - nss_to_nss_delay);
            }
        }
#endif /* STM32WLxx_RADIO_APP_LPM_SUPPORT */

        /* Transfer data */
        sid_err = drv_ctx->bus_iface->xfer(drv_ctx->bus_iface, drv_ctx->config->spi_client_cfg, tx_ptr, rx_ptr, STM32WLxx_RADIO_COMM_SPI_DATA_FRAME_SIZE);
        if (sid_err != SID_ERROR_NONE)
        {
            STM32WLxx_HAL_LOG_ERROR("Failed to xfer single SPI frame to STM32WLxx. sid_pal_serial_bus_spi error %d", (int32_t)sid_err);
            err = STM32WLxx_HAL_STATUS_ERROR_SPI_XFER;
            break;
        }

#if STM32WLxx_RADIO_APP_LPM_SUPPORT
        /* Release the manual NSS control if it was requested */
        if (wakeup_delay_required != FALSE)
        {
            sid_err = sid_pal_serial_bus_ext_ifc_deactivate_nss(drv_ctx->bus_iface, drv_ctx->config->spi_client_cfg);
            if (sid_err != SID_ERROR_NONE)
            {
                STM32WLxx_HAL_LOG_ERROR("Failed to deactivate STM32WLxx NSS line after wakeup. sid_pal_serial_bus_spi error %d", (int32_t)sid_err);
                err = STM32WLxx_HAL_STATUS_ERROR_SPI_XFER;
                break;
            }
        }
#endif /* STM32WLxx_RADIO_APP_LPM_SUPPORT */

        /* Copy partial data */
        if (rx_buf != NULL)
        {
            SID_STM32_UTIL_fast_memcpy(rx_buf, full_size_rx_buf, data_length);
        }

        /* Done */
        err = STM32WLxx_HAL_STATUS_OK;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static stm32wlxx_hal_status_t _spi_transmit_data(const halo_drv_stm32wlxx_ctx_t * const drv_ctx, const uint8_t * const data, const size_t data_length)
{
    stm32wlxx_hal_status_t err = STM32WLxx_HAL_STATUS_ERROR_GENERIC;

    /* Protect from systematic SW failures */
    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->bus_iface != NULL);
    SID_PAL_ASSERT(drv_ctx->bus_iface->xfer != NULL);

    halo_drv_stm32wlxx_ctx_t * const modifiable_drv_ctx = (halo_drv_stm32wlxx_ctx_t *)drv_ctx;

    /* Capture STM32WLxx IRQ line status before the transaction */
    const uint32_t irq_was_enabled = drv_ctx->irq_enabled;

    do
    {
        /* Validate inputs */
        if ((NULL == data) || (0u == data_length))
        {
            err = STM32WLxx_HAL_STATUS_INVALID_ARGS;
            STM32WLxx_HAL_LOG_ERROR("STM32WLxx SPI Tx failed - invalid args");
            break;
        }

        /* Disable all STM32WLxx IRQs - if an IRQ is indicated during the ongoing transfer, the IRQ handler will try to read out IRQ status, causing clashes on the SPI line */
        err = stm32wlxx_hal_disarm_irq(modifiable_drv_ctx);
        if (err != STM32WLxx_HAL_STATUS_OK)
        {
            STM32WLxx_HAL_LOG_ERROR("STM32WLxx SPI Tx failed - unble to disarm IRQ line. error %u", (uint32_t)err);
            break;
        }

        if (STM32WLxx_RADIO_COMM_SPI_DATA_FRAME_SIZE == data_length)
        {
            /* Data perfectly matches the SPI data frame size - transfer as is */
            err = _spi_transmit_receive_full_frame(drv_ctx, data, NULL, data_length);
            if (err != STM32WLxx_HAL_STATUS_OK)
            {
                break;
            }
        }
        else if (data_length > STM32WLxx_RADIO_COMM_SPI_DATA_FRAME_SIZE)
        {
            /* Data size exceeds single SPI frame - transfer as long data */
            stm32wlxx_radio_comm_spi_frame_t tx_frame;
            const uint8_t * ingest_ptr = data;
            size_t remaining_bytes = data_length;

            if (data_length > UINT16_MAX)
            {
                STM32WLxx_HAL_LOG_ERROR("STM32WLxx SPI Tx failed - data is too long");
                err = STM32WLxx_HAL_STATUS_ERROR_DATA_TOO_LONG;
                break;
            }

            /* Prepare the long data start frame */
            stm32wlxx_rcp_ldts_t * const start_payload = &tx_frame.payload.ldts;
            tx_frame.opcode = STM32WLxx_RADIO_COMM_OPCODE_LONG_DATA_START;
            start_payload->full_data_size = (uint16_t)data_length;
            SID_STM32_UTIL_fast_memcpy(start_payload->partial_data, ingest_ptr, sizeof(start_payload->partial_data));

            /* Advance the pointers */
            ingest_ptr += sizeof(start_payload->partial_data);
            remaining_bytes -= sizeof(start_payload->partial_data);

            /* Transmit the starting frame */
            err = _spi_transmit_receive_full_frame(drv_ctx, (uint8_t*)(void *)&tx_frame, NULL, sizeof(tx_frame));
            if (err != STM32WLxx_HAL_STATUS_OK)
            {
                break;
            }

#if STM32WLxx_RADIO_APP_LPM_SUPPORT
            /* Indicate active LDT so SPI comm will assume the STM32WLxx side does not need wakeup till the end of the current LDT frame */
            modifiable_drv_ctx->ldt_ongoing = TRUE;
#endif /* STM32WLxx_RADIO_APP_LPM_SUPPORT */

            /* Proceed with the continuation packets */
            tx_frame.opcode = STM32WLxx_RADIO_COMM_OPCODE_LONG_DATA_CONT;
            while (remaining_bytes > 0u)
            {
                /* Compute how many bytes we can squeeze into the continuation frame */
                stm32wlxx_rcp_ldtc_t * const continuation_payload = &tx_frame.payload.ldtc;
                const size_t process_length = remaining_bytes > sizeof(continuation_payload->partial_data) ? sizeof(continuation_payload->partial_data) : remaining_bytes;

                /* Populate the upcoming continuation frame */
                SID_STM32_UTIL_fast_memcpy(continuation_payload->partial_data, ingest_ptr, process_length);

                /* Send out the frame */
                if (process_length <  sizeof(continuation_payload->partial_data))
                {
                    err = _spi_transmit_receive_partial_frame(drv_ctx, (uint8_t*)(void *)&tx_frame, NULL, sizeof(tx_frame) - (sizeof(continuation_payload->partial_data) - remaining_bytes));
                }
                else
                {
                    err = _spi_transmit_receive_full_frame(drv_ctx, (uint8_t*)(void *)&tx_frame, NULL, sizeof(tx_frame));
                }

                /* Send out error check */
                if (err != STM32WLxx_HAL_STATUS_OK)
                {
                    break;
                }

                /* Advance the pointers */
                ingest_ptr += process_length;
                remaining_bytes -= process_length;
            }

            /* Jump out if the loop above has terminated abnormally */
            if (err != STM32WLxx_HAL_STATUS_OK)
            {
                break;
            }
        }
        else
        {
            /* Data size is smaller than a single SPI frame - use padding */
            err = _spi_transmit_receive_partial_frame(drv_ctx, data, NULL, data_length);
            if (err != STM32WLxx_HAL_STATUS_OK)
            {
                break;
            }
        }
    } while (0);

#if STM32WLxx_RADIO_APP_LPM_SUPPORT
    /* Ensure LDT status is cleaned regardless of the outcome */
    modifiable_drv_ctx->ldt_ongoing = FALSE;
#endif /* STM32WLxx_RADIO_APP_LPM_SUPPORT */

    if (irq_was_enabled != FALSE)
    {
        /* Ensure the IRQs are re-enabled after the transaction */
        stm32wlxx_hal_status_t rearm_err = stm32wlxx_hal_arm_irq(modifiable_drv_ctx);
        if (rearm_err != STM32WLxx_HAL_STATUS_OK)
        {
            STM32WLxx_HAL_LOG_ERROR("STM32WLxx SPI TxRx failed - unble to re-arm IRQ line. error %u", (uint32_t)rearm_err);
        }

        if (STM32WLxx_HAL_STATUS_OK == err)
        {
            err = rearm_err;
        }
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static stm32wlxx_hal_status_t _spi_receive_data(const halo_drv_stm32wlxx_ctx_t * const drv_ctx, uint8_t * const rx_buf,
                                                                          const size_t buf_size, size_t * const bytes_received)
{
    stm32wlxx_hal_status_t err = STM32WLxx_HAL_STATUS_ERROR_GENERIC;

    /* Protect from systematic SW failures */
    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->bus_iface != NULL);
    SID_PAL_ASSERT(drv_ctx->bus_iface->xfer != NULL);

    halo_drv_stm32wlxx_ctx_t * const modifiable_drv_ctx = (halo_drv_stm32wlxx_ctx_t *)drv_ctx;

    /* Capture STM32WLxx IRQ line status before the transaction */
    const uint32_t irq_was_enabled = drv_ctx->irq_enabled;

    do
    {
        /* Use dummy frame to guarantee that the dummy OpCode will be transferred */
        const stm32wlxx_radio_comm_spi_frame_t dummy_tx_frame = {
            .opcode = STM32WLxx_RADIO_COMM_OPCODE_DUMMY_DATA,
            .payload = {
                .raw = {0u},
            }
        };
        stm32wlxx_radio_comm_spi_frame_t rx_frame;
        uint8_t * ingest_ptr = rx_buf;
        size_t expected_length;
        size_t received_length;

        /* Validate inputs */
        if (NULL == rx_buf)
        {
            if (0u == buf_size)
            {
                STM32WLxx_HAL_LOG_DEBUG("Performing dummy STM32WLxx SPI Rx transaction - all the received data will be discarded");
            }
            else
            {
                err = STM32WLxx_HAL_STATUS_INVALID_ARGS;
                STM32WLxx_HAL_LOG_ERROR("STM32WLxx SPI Rx failed - invalid args (null rx buffer with non-zero length)");
                break;
            }
        }
        else
        {
            if (0u == buf_size)
            {
                err = STM32WLxx_HAL_STATUS_INVALID_ARGS;
                STM32WLxx_HAL_LOG_ERROR("STM32WLxx SPI Rx failed - invalid args (non-null rx buffer with zero length)");
                break;
            }
        }

        if (bytes_received != NULL)
        {
            /* Set default value */
            *bytes_received = 0u;
        }

        /* Disable all STM32WLxx IRQs - if an IRQ is indicated during the ongoing transfer, the IRQ handler will try to read out IRQ status, causing clashes on the SPI line */
        err = stm32wlxx_hal_disarm_irq(modifiable_drv_ctx);
        if (err != STM32WLxx_HAL_STATUS_OK)
        {
            STM32WLxx_HAL_LOG_ERROR("STM32WLxx SPI Tx failed - unble to disarm IRQ line. error %u", (uint32_t)err);
            break;
        }

        /* Unconditionally receive the first frame */
        err = _spi_transmit_receive_full_frame(drv_ctx, (uint8_t *)(void *)&dummy_tx_frame, (uint8_t *)(void *)&rx_frame, sizeof(rx_frame));
        if (err != STM32WLxx_HAL_STATUS_OK)
        {
            STM32WLxx_HAL_LOG_ERROR("STM32WLxx SPI Rx failed - SPI error %u", (uint32_t)err);
            break;
        }

        /* Inspect the frame - it may indicate long data */
        if (STM32WLxx_RADIO_COMM_OPCODE_LONG_DATA_START == rx_frame.opcode)
        {
            /* This is the start of the long data - more frames are pending */
            const stm32wlxx_rcp_ldts_t * const start_payload = &rx_frame.payload.ldts;
            expected_length = (size_t)start_payload->full_data_size;

            /* Perform the validity check */
            if (expected_length <= sizeof(start_payload->partial_data))
            {
                SID_PAL_LOG_ERROR("Invalid long data start frame - full data length (%u) is too short", expected_length); /* This a serious error, always log it */
                err = STM32WLxx_HAL_STATUS_ERROR_UNEXPECTED_DATA;
                break;
            }

            STM32WLxx_HAL_LOG_DEBUG("Received long data start payload from STM32WLxx. Full data length: %u", expected_length);

            if (rx_buf != NULL)
            {
                if (expected_length > buf_size)
                {
                    STM32WLxx_HAL_LOG_ERROR("Unable to receive long data - supplied Rx buffer is too short");
                    err = STM32WLxx_HAL_STATUS_ERROR_BUFFER_OVERFLOW;
                    break;
                }

                /* Copy partial data */
                SID_STM32_UTIL_fast_memcpy(ingest_ptr, start_payload->partial_data, sizeof(start_payload->partial_data));
            }
            else
            {
                /* Perform dummy read */
            }

            /* Advance the pointers & counters regardless if it is normal or dummy read */
            received_length = sizeof(start_payload->partial_data);
            ingest_ptr += received_length;

#if STM32WLxx_RADIO_APP_LPM_SUPPORT
            /* Indicate active LDT so SPI comm will assume the STM32WLxx side does not need wakeup till the end of the current LDT frame */
            modifiable_drv_ctx->ldt_ongoing = TRUE;
#endif /* STM32WLxx_RADIO_APP_LPM_SUPPORT */
        }
        else if (STM32WLxx_RADIO_COMM_OPCODE_LONG_DATA_CONT == rx_frame.opcode)
        {
            /* Invalid state - we've got long data continuation without getting long data start */
            STM32WLxx_HAL_LOG_ERROR("Unexpectedly received long data continuation frame from STM32WLxx");
            if (bytes_received != NULL)
            {
                *bytes_received = 0u;
            }
            err = STM32WLxx_HAL_STATUS_ERROR_UNEXPECTED_DATA;
            break;
        }
        else
        {
            /* This is a generic single-frame message */
            if (rx_buf != NULL)
            {
                /* Store the received data */
                const size_t copy_length = buf_size > sizeof(rx_frame) ? sizeof(rx_frame) : buf_size; /* Allow storing partial single frames as the valid payload may be shorter than an SPI frame */
                SID_STM32_UTIL_fast_memcpy(rx_buf, (uint8_t *)(void *)&rx_frame, copy_length);

                /* Provide how many bytes we've received if possible */
                if (bytes_received != NULL)
                {
                    *bytes_received = copy_length;
                }

                STM32WLxx_HAL_LOG_DEBUG("Received single-frame payload from STM32WLxx, stored %u bytes", copy_length);
            }
            else
            {
                STM32WLxx_HAL_LOG_DEBUG("Completed single-frame dummy read from STM32WLxx");
            }

            err = STM32WLxx_HAL_STATUS_OK;
            break;
        }

        /* Proceed with receiving the reminder of the long data */
        while (received_length < expected_length)
        {
            const stm32wlxx_rcp_ldtc_t * const continuation_payload = &rx_frame.payload.ldtc;

            err = _spi_transmit_receive_full_frame(drv_ctx, (uint8_t *)(void*)&dummy_tx_frame, (uint8_t *)(void*)&rx_frame, sizeof(rx_frame));
            if (err != STM32WLxx_HAL_STATUS_OK)
            {
                /* Something went wrong */
                break;
            }

            /* Ensure we've got expected data */
            if (rx_frame.opcode != STM32WLxx_RADIO_COMM_OPCODE_LONG_DATA_CONT)
            {
                /* Invalid state - we've not got long data continuation  */
                STM32WLxx_HAL_LOG_ERROR("Unexpectedly received opcode 0x%x instead of long data continuation frame from STM32WLxx", rx_frame.opcode);
                err = STM32WLxx_HAL_STATUS_ERROR_UNEXPECTED_DATA;
                break;
            }

            /* This is a valid long data continuation, add partial data to the Rx buffer */
            const size_t copy_length = (expected_length - received_length) > sizeof(continuation_payload->partial_data) ? sizeof(continuation_payload->partial_data) : (expected_length - received_length);

            if (rx_buf != NULL)
            {
                SID_STM32_UTIL_fast_memcpy(ingest_ptr, continuation_payload->partial_data, copy_length);
                STM32WLxx_HAL_LOG_DEBUG("Received %u bytes of long data continuation from STM32WLxx. Total bytes received: %u", copy_length, received_length);
            }
            else
            {
                /* Perform dummy read */
            }

            /* Advance the pointers & counters regardless if it is normal or dummy read */
            received_length += copy_length;
            ingest_ptr += copy_length;
        }

        if (bytes_received != NULL)
        {
            *bytes_received = received_length;
        }
    } while (0);

#if STM32WLxx_RADIO_APP_LPM_SUPPORT
    /* Ensure LDT status is cleaned regardless of the outcome */
    modifiable_drv_ctx->ldt_ongoing = FALSE;
#endif /* STM32WLxx_RADIO_APP_LPM_SUPPORT */

    if (irq_was_enabled != FALSE)
    {
        /* Ensure the IRQs are re-enabled after the transaction */
        stm32wlxx_hal_status_t rearm_err = stm32wlxx_hal_arm_irq(modifiable_drv_ctx);
        if (rearm_err != STM32WLxx_HAL_STATUS_OK)
        {
            STM32WLxx_HAL_LOG_ERROR("STM32WLxx SPI TxRx failed - unble to re-arm IRQ line. error %u", (uint32_t)rearm_err);
        }

        if (STM32WLxx_HAL_STATUS_OK == err)
        {
            err = rearm_err;
        }
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static stm32wlxx_hal_status_t _spi_transmit_receive_data(const halo_drv_stm32wlxx_ctx_t * const drv_ctx, const uint8_t * const tx_data, uint8_t * const rx_data, const size_t data_length)
{
    stm32wlxx_hal_status_t err = STM32WLxx_HAL_STATUS_ERROR_GENERIC;

    /* Protect from systematic SW failures */
    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->bus_iface != NULL);
    SID_PAL_ASSERT(drv_ctx->bus_iface->xfer != NULL);

    halo_drv_stm32wlxx_ctx_t * const modifiable_drv_ctx = (halo_drv_stm32wlxx_ctx_t *)drv_ctx;

    /* Capture STM32WLxx IRQ line status before the transaction */
    const uint32_t irq_was_enabled = drv_ctx->irq_enabled;

    do
    {
        /* Validate inputs */
        if ((NULL == tx_data) || (NULL == rx_data) || (0u == data_length))
        {
            err = STM32WLxx_HAL_STATUS_INVALID_ARGS;
            STM32WLxx_HAL_LOG_ERROR("STM32WLxx SPI TxRx failed - invalid args");
            break;
        }

        /* Disable all STM32WLxx IRQs - if an IRQ is indicated during the ongoing transfer, the IRQ handler will try to read out IRQ status, causing clashes on the SPI line */
        err = stm32wlxx_hal_disarm_irq(modifiable_drv_ctx);
        if (err != STM32WLxx_HAL_STATUS_OK)
        {
            STM32WLxx_HAL_LOG_ERROR("STM32WLxx SPI Tx failed - unble to disarm IRQ line. error %u", (uint32_t)err);
            break;
        }

        if (STM32WLxx_RADIO_COMM_SPI_DATA_FRAME_SIZE == data_length)
        {
            /* Data perfectly matches the SPI data frame size - transfer as is */
            err = _spi_transmit_receive_full_frame(drv_ctx, tx_data, rx_data, data_length);
            if (err != STM32WLxx_HAL_STATUS_OK)
            {
                break;
            }
        }
        else if (data_length > STM32WLxx_RADIO_COMM_SPI_DATA_FRAME_SIZE)
        {
            /* Data size exceeds single SPI frame - transfer as long data */
            STM32WLxx_HAL_LOG_ERROR("STM32WLxx SPI TxRx failed - simultaneous Tx and Rx of long data is not supported");
            err = STM32WLxx_HAL_STATUS_ERROR_NOT_SUPPORTED;
            break;
        }
        else
        {
            /* Data size is smaller than a single SPI frame - use padding */
            err = _spi_transmit_receive_partial_frame(drv_ctx, tx_data, rx_data, data_length);
            if (err != STM32WLxx_HAL_STATUS_OK)
            {
                break;
            }
        }
    } while (0);

    if (irq_was_enabled != FALSE)
    {
        /* Ensure the IRQs are re-enabled after the transaction */
        stm32wlxx_hal_status_t rearm_err = stm32wlxx_hal_arm_irq(modifiable_drv_ctx);
        if (rearm_err != STM32WLxx_HAL_STATUS_OK)
        {
            STM32WLxx_HAL_LOG_ERROR("STM32WLxx SPI TxRx failed - unble to re-arm IRQ line. error %u", (uint32_t)rearm_err);
        }

        if (STM32WLxx_HAL_STATUS_OK == err)
        {
            err = rearm_err;
        }
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static stm32wlxx_hal_status_t _flush_spi(const halo_drv_stm32wlxx_ctx_t * const drv_ctx)
{
    stm32wlxx_hal_status_t err = STM32WLxx_HAL_STATUS_ERROR_GENERIC;
    uint32_t remaining_attempts = STM32WLxx_HAL_RADIO_SUBGHZ_SPI_FLUSH_ATTEMPTS;

    do
    {
        stm32wlxx_radio_comm_spi_frame_t rx_frame;
        size_t bytes_received;

        /* Perform a dummy read */
        err = _spi_receive_data(drv_ctx, (uint8_t *)(void *)&rx_frame, sizeof(rx_frame), &bytes_received);
        if ((err != STM32WLxx_HAL_STATUS_OK) || (bytes_received != sizeof(rx_frame)))
        {
            STM32WLxx_HAL_LOG_ERROR("Failed perform dummy SPI read. Error %u, bytes rcvd: %u", (int32_t)err, bytes_received);
            break;
        }

        if (STM32WLxx_RADIO_COMM_OPCODE_DUMMY_DATA == rx_frame.opcode)
        {
            /* Flushing done, STM32WLxx sent dummy frame */
            err = STM32WLxx_HAL_STATUS_OK;
            break;
        }

        /* Check if the  */
        --remaining_attempts;
        if (0u == remaining_attempts)
        {
            err = STM32WLxx_HAL_STATUS_ERROR_TIMEOUT;
            STM32WLxx_HAL_LOG_ERROR("Failed to flush SPI - attempts limit reached, STM32WLxx still provides non-dummy data");
            break;
        }
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static stm32wlxx_hal_status_t _send_request_with_ack(halo_drv_stm32wlxx_ctx_t * const drv_ctx, const uint8_t * const tx_data,
                                                                               const size_t data_length, const uint32_t timeout_ms)
{
    stm32wlxx_hal_status_t err = STM32WLxx_HAL_STATUS_ERROR_GENERIC;
    sid_error_t sid_err;

    if ((NULL == tx_data) || (0u == data_length))
    {
        return STM32WLxx_HAL_STATUS_INVALID_ARGS;
    }

    /* Ensure there's no other request with ACK running */
    osStatus_t os_err = osSemaphoreAcquire(drv_ctx->req_completed_wait_lock, 0u);
    if (os_err != osOK)
    {
        /* Radio is busy with some other request */
        SID_PAL_LOG_ERROR("Failed to send request with ACK to STM32WLxx - another request is already running");
        err = STM32WLxx_HAL_STATUS_ERROR_BUSY;

        /* Terminate from here to avoid any collisions with the ongoing transaction */
        return err;
    }

    /* Capture STM32WLxx IRQ line status before the transaction */
    const uint32_t irq_was_enabled = drv_ctx->irq_enabled;

    /* Determine if we are in a task context and can use IRQ and semaphore-based wait mechanism */
    const uint32_t use_polling = (SID_STM32_UTIL_IS_IRQ()) || (irq_was_enabled == FALSE) ? TRUE: FALSE;

    do
    {
        if (FALSE == use_polling)
        {
            /* IRQ-based wait mechanism can be used. Prepare the semaphore to lock the caller task until radio IRQ is reported */
            while (osSemaphoreGetCount(drv_ctx->irq_detected_wait_lock) != 0u)
            {
                os_err = osSemaphoreAcquire(drv_ctx->irq_detected_wait_lock, 0u);
                if (os_err != osOK)
                {
                    STM32WLxx_HAL_LOG_ERROR("Failed to send request with ACK to STM32WLxx - cannot arm IRQ wait mechanism. Error 0x%08X", (uint32_t)os_err);
                    break;
                }
            }

            /* Propagate error to terminate the main while() loop */
            if (os_err != osOK)
            {
                break;
            }
        }
        else
        {
            /* Disable IRQ line - we are going to use polling to monitor IRQ line status because IRQ-based mechanism cannot be used */
            err = stm32wlxx_hal_disarm_irq(drv_ctx);
            if (err != STM32WLxx_HAL_STATUS_OK)
            {
                STM32WLxx_HAL_LOG_ERROR("Failed to send request with ACK to STM32WLxx - unable to disarm IRQ line. error %u", (uint32_t)err);
                break;
            }
        }

        /* Clear the previous IRQ status storage */
        SID_STM32_UTIL_fast_memset((void *)&drv_ctx->last_irq_status, 0u, sizeof(drv_ctx->last_irq_status));

        /* Indicate the request-with-ack transaction is now ongoing */
        drv_ctx->req_with_ack_ongoing = TRUE;

        /* Send the request to STM32WLxx */
        err = _spi_transmit_data(drv_ctx, tx_data, data_length);
        if (err != STM32WLxx_HAL_STATUS_OK)
        {
            /* SPI failed */
            STM32WLxx_HAL_LOG_ERROR("Unable to send request with ACK to STM32WLxx. SPI transfer error %u", (uint32_t)err);
            break;
        }

        /* Wait for the IRQ to be indicated by the STM32WLxx side */
        if (FALSE == use_polling)
        {
            /* We are in a task context - just lock on the semaphore until the reset is completed or the timeout happens */
            const uint32_t wait_timeout_ticks = (timeout_ms * osKernelGetTickFreq()) / 1000u;
            os_err = osSemaphoreAcquire(drv_ctx->irq_detected_wait_lock, wait_timeout_ticks);
            if (os_err != osOK)
            {
                /* Check for systematic failures */
                SID_PAL_ASSERT(os_err != osErrorParameter);

                /* Report timeout error */
                err = STM32WLxx_HAL_STATUS_ERROR_TIMEOUT;
                break;
            }
        }
        else
        {
            /* We are in interrupt or interrupt-like context - semaphore cannot be acquired using normal wait states */
            uint32_t accumulated_wait_time = 0u;
            err = STM32WLxx_HAL_STATUS_ERROR_TIMEOUT;

            do
            {
                uint8_t pin_state;

                sid_err = sid_pal_gpio_read(drv_ctx->config->gpio.radio_irq, &pin_state);
                if (SID_ERROR_NONE != sid_err)
                {
                    STM32WLxx_HAL_LOG_ERROR("Unable to read IRQ line state. Error %d", (int32_t)sid_err);
                    err = STM32WLxx_HAL_STATUS_ERROR_HW;
                    break;
                }

                /* Check pin status */
                if (pin_state != STM32WLxx_HAL_RADIO_IRQ_PIN_STATE_ACTIVE)
                {
                    /* No IRQ indicated - wait */
                    sid_pal_delay_us(STM32WLxx_HAL_RADIO_REQUEST_ACK_PROBE_PERIOD_US);
                    accumulated_wait_time += STM32WLxx_HAL_RADIO_REQUEST_ACK_PROBE_PERIOD_US;
                }
                else
                {
                    /* IRQ indicated - we can process it */
                    err = STM32WLxx_HAL_STATUS_OK;
                    break;
                }
            } while (accumulated_wait_time < (timeout_ms * 1000u));

            /* React on ACK wait error */
            if (err != STM32WLxx_HAL_STATUS_OK)
            {
                break;
            }
        }

        /* Regardless of the wait method used, the IRQ is now indicated and can be fetched and processed */
        err = stm32wlxx_hal_generic_irq_process(drv_ctx, NULL);
        if (err != STM32WLxx_HAL_STATUS_OK)
        {
            /* Logs provided by stm32wlxx_hal_generic_irq_process() */
            break;
        }

        /* Check if the request was acknowledged */
        if (drv_ctx->req_with_ack_ongoing != FALSE)
        {
            /* Ongoing transcation flag was not cleared by the IRQ handler - report ACK timeout */
            err = STM32WLxx_HAL_STATUS_ERROR_TIMEOUT;
            break;
        }

        /* Everything is fine, request acknowledged via IRQ */
        err = STM32WLxx_HAL_STATUS_OK;
    } while (0);

    /* Ensure we restore IRQ line configuration regardless of success or failure of ACK wait */
    if (irq_was_enabled != FALSE)
    {
        stm32wlxx_hal_status_t rearm_err = stm32wlxx_hal_arm_irq(drv_ctx);
        if (rearm_err != STM32WLxx_HAL_STATUS_OK)
        {
            STM32WLxx_HAL_LOG_ERROR("Sending request with ACK to STM32WLxx failed - unable to re-arm IRQ line. error %u", (uint32_t)rearm_err);
        }

        if (STM32WLxx_HAL_STATUS_OK == err)
        {
            err = rearm_err;
        }
    }

    /* Release the semaphore and clear the flag when we are done regardless of the success */
    drv_ctx->req_with_ack_ongoing = FALSE;
    __COMPILER_BARRIER(); /* Ensure the flag is always cleared before the semaphore is released */
    (void)osSemaphoreRelease(drv_ctx->req_completed_wait_lock);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline uint32_t _radio_irq_gpio_filtered_read(const halo_drv_stm32wlxx_ctx_t * const drv_ctx)
{
    /* Perform filtering since Open-Drain line is not immune to EMI and noise ----*/
    register uint32_t    probe_cnt                = 0u;
    register uint32_t    debounce_cnt             = 0u;
    register sid_error_t sid_err;
             uint8_t     pin_state;

    SID_PAL_ASSERT(drv_ctx != NULL);

    do
    {
        /* Wait a bit for pin state to settle */
        sid_pal_delay_us(STM32WLxx_HAL_RADIO_IRQ_PIN_FILTER_PROBE_PERIOD_US);

        sid_err = sid_pal_gpio_read(drv_ctx->config->gpio.radio_irq, &pin_state);
        if (SID_ERROR_NONE != sid_err)
        {
            STM32WLxx_HAL_LOG_ERROR("Unable to read IRQ line state. Error %d", (int32_t)sid_err);

            /* Can't determine IRQ line state - assume IRQ is active, it is safer to react on noise-induced pseudo-event
             * than to skip a real IRQ indication due to GPIO API errors
             */
            debounce_cnt++;
        }
        else
        {
            /* Check pin status */
            if (STM32WLxx_HAL_RADIO_IRQ_PIN_STATE_ACTIVE == pin_state)
            {
                /* IRQ line is driven low */
                debounce_cnt++;
            }
            else
            {
                /* IRQ line is now high, IRQ may have been triggered by a surge */
                debounce_cnt = 0u;
            }
        }

        /* Increment counter and proceed to the next iteration */
        probe_cnt++;
    } while (probe_cnt < STM32WLxx_HAL_RADIO_IRQ_PIN_FILTER_PROBE_COUNT);

    if (debounce_cnt < STM32WLxx_HAL_RADIO_IRQ_PIN_FILTER_PROBE_DEBOUNCE)
    {
        /* Seems the IRQ was accidentally triggered by a glitch - ignore it */
        STM32WLxx_HAL_LOG_DEBUG("IRQ line filtered");
        pin_state = STM32WLxx_HAL_RADIO_IRQ_PIN_STATE_INACTIVE;
    }

    return pin_state;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static void _on_radio_irq_detected(uint32_t pin, void * callback_arg)
{
    struct sid_timespec event_ts;

    /* Store the timestamp as soon as possible */
    (void)sid_pal_uptime_now(&event_ts);
    __COMPILER_BARRIER();

    (void)pin;

    STM32WLxx_HAL_LOG_DEBUG("radio_stm32wlxx_on_radio_irq_detected...");

    halo_drv_stm32wlxx_ctx_t * const drv_ctx = (halo_drv_stm32wlxx_ctx_t *)callback_arg;
    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->radio_rx_packet != NULL);

    /**
     * The time-sensitive part is completed, now we can lower Radio IRQ priority to allow other time-critical IRQs to be processed and to enable RTOS API calls below this point
     *
     * WARNING: Dynamic IRQ priority updates within the IRQ handler is supported starting from ARMv7-M architecture (Cortex-M3). For ARMv6-M (Cortex-M0/M0+/M1) priority shall
     *          be changed only when the corresponding IRQ is not active. Due to that radio_irq.prio in the radio driver configuration shall be set to a level that allows RTOS
     *          API calls (e.g. lower (meaning higher number) or equal to configMAX_SYSCALL_INTERRUPT_PRIORITY for FreeRTOS)
     */
#if (__ARM_ARCH_6M__ == 0)
    if (drv_ctx->radio_is_running_high_prio != FALSE)
    {
        /* Store the capture time stamp if this the ISR is in the hard realtime mode */
        drv_ctx->radio_rx_packet->rcv_tm = event_ts;

        /* Lower the IRQ priority */
        (void)sid_pal_gpio_ext_ifc_set_irq_priority(drv_ctx->config->gpio.radio_irq, drv_ctx->config->gpio.radio_irq_prio_low, 0u);
        drv_ctx->radio_is_running_high_prio = FALSE;

        /**
         * Jump out of the ISR to allow NVIC to re-evaluate the priorities
         *
         * WARNING: This is a mandatory step after lower the IRQ priority to enable RTOS API calls. Just lowering the priority is not enough because if this ISR was entered
         *          while the respective priority was high, it will continue to run even after lowering the priority because NVIC does not re-evaluate the priority for an
         *          ISR that is already running. This means proceeding with the execution after lowering the priority still may interfere with the RTOS kernel and ruin the
         *          scheduling, resulting in undefined behavior. To overcome this limitation, the ISR must terminate from here. Since the respective GPIO IRQ is configured to
         *          be triggered by the level on the pin, not the edge, the NVIC will take the IRQ back immediately, but it will now re-evaluate the IRQ priority and call the
         *          ISR only when it is safe to do so from RTOS kernel perspective.
         */
        return;
    }
#else
    /* Just store the capture time stamp and proceed as IRQ priority change is not an option here */
    drv_ctx->radio_rx_packet->rcv_tm = event_ts;
#endif /* __ARM_ARCH_6M__ */

    /* Disable radio IRQ line  to allow the MCU's IRQ handler to return - Sidewalk may process radio IRQ in a RTOS task context */
    stm32wlxx_hal_status_t err = stm32wlxx_hal_disarm_irq(drv_ctx);
    if (err != STM32WLxx_HAL_STATUS_OK)
    {
        SID_PAL_LOG_ERROR("Failed to disarm STM32WLxx IRQ line. error %u", (uint32_t)err);
    }

    /* Terminate from here if special IRQ processing is required */
    if (drv_ctx->req_with_ack_ongoing != FALSE)
    {
        (void)osSemaphoreRelease(drv_ctx->irq_detected_wait_lock);
        return;
    }

    /* Schedule generic IRQ processing in the SWI task context */
    drv_ctx->irq_handler();
}

/* Global function definitions -----------------------------------------------*/

stm32wlxx_hal_status_t stm32wlxx_hal_generic_irq_process(halo_drv_stm32wlxx_ctx_t * const drv_ctx, int32_t * const out_sidewalk_error)
{
    stm32wlxx_hal_status_t hal_err;
    uint32_t handshake_request_active = FALSE;

    STM32WLxx_HAL_LOG_DEBUG("radio_stm32wlxx_on_irq_detected...");

    SID_PAL_ASSERT(drv_ctx != NULL);

    if (out_sidewalk_error != NULL)
    {
        *out_sidewalk_error = RADIO_ERROR_NONE;
    }

    /* Perform filtering since Open-Drain line is not immune to EMI and noise ----*/
    const uint32_t radio_irq_pin_state = _radio_irq_gpio_filtered_read(drv_ctx);

    if (radio_irq_pin_state != STM32WLxx_HAL_RADIO_IRQ_PIN_STATE_ACTIVE)
    {
        /* Seems the IRQ was accidentally triggered by a glitch - ignore it */
        return STM32WLxx_HAL_STATUS_OK;
    }
    /*----------------------------------------------------------------------------*/

    /* Proceed with regular IRQ handling -----------------------------------------*/
    register uint32_t trigger_sid_radio_event_handler = FALSE;
    register uint32_t indicate_radio_request_completed = FALSE;

    do
    {
        /* Read out IRQ status and any potential follow-up data */
        SID_PAL_ASSERT(drv_ctx->config                       != NULL);
        SID_PAL_ASSERT(drv_ctx->config->internal_buffer.p    != NULL);
        SID_PAL_ASSERT(drv_ctx->config->internal_buffer.size >  0u);
        hal_err = stm32wlxx_hal_retrieve_radio_irq(drv_ctx, &drv_ctx->last_irq_status, drv_ctx->config->internal_buffer.p, drv_ctx->config->internal_buffer.size);
        if (hal_err != STM32WLxx_HAL_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Unable to retrieve STM32WLxx IRQ status. HAL error %u", (uint32_t)hal_err);
            goto recovery;
        }

        /* Store a copy of the received IRQ status */
        const stm32wlxx_app_irq_mask_t active_irq = (stm32wlxx_app_irq_mask_t)drv_ctx->last_irq_status.irq_flags;

        STM32WLxx_HAL_LOG_DEBUG("Retrieved STM32WLxx IRQ status. IRQ flags: 0x%x, follow-up data size: %u", drv_ctx->last_irq_status.irq_flags, drv_ctx->last_irq_status.followup_payload_len);

        switch (active_irq)
        {
            case STM32WLxx_APP_IRQ_NONE:
                SID_PAL_LOG_WARNING("STM32WLxx Radio App IRQ reported with no flags set - ignoring event");
                hal_err = STM32WLxx_HAL_STATUS_ERROR_INVALID_STATE;
                break;

            case STM32WLxx_APP_IRQ_SUBGHZ:
                /* This is a SubGHz radio event, pass it to the Sidewalk stack */
                if ((drv_ctx->app_irq_mask & active_irq) != 0u)
                {
                    SID_PAL_ASSERT(drv_ctx->radio_rx_packet != NULL);

                    /* Apply the radio event timestamp compensation as reported by the STM32WLxx to drv_ctx->radio_rx_packet->rcv_tm. The initial timestamp value is captured at the moment when GPIO IRQ is detected */
                    const stm32wlxx_subghz_irq_details_t * const subghz_irq_details = &drv_ctx->last_irq_status.subghz_irq_details;
                    struct sid_timespec irq_timestamp_compensation = {
                        .tv_sec = 0u,
                        .tv_nsec = subghz_irq_details->compensatory_ns,
                    };
                    if (sid_time_lt(&drv_ctx->radio_rx_packet->rcv_tm, &irq_timestamp_compensation) == FALSE)
                    {
                        sid_time_sub(&drv_ctx->radio_rx_packet->rcv_tm, &irq_timestamp_compensation);
                    }
                    else
                    {
                        /* Time cannot be negative so our best option is to report zero */
                        drv_ctx->radio_rx_packet->rcv_tm.tv_sec  = 0u;
                        drv_ctx->radio_rx_packet->rcv_tm.tv_nsec = 0u;
                    }

                    /* Provide any received additional payload */
                    if (drv_ctx->last_irq_status.followup_payload_len > 0u)
                    {
                        const stm32wlxx_rcp_set_subghz_rx_buf_t * const subghz_additional_data = (stm32wlxx_rcp_set_subghz_rx_buf_t *)(void *)drv_ctx->config->internal_buffer.p;
                        if (subghz_additional_data->opcode != STM32WLxx_RADIO_COMM_OPCODE_RAW_DATA)
                        {
                            SID_PAL_LOG_ERROR("Invalid SubGHz IRQ follow-up data. Expected OpCode 0x%02X but got 0x%02X", STM32WLxx_RADIO_COMM_OPCODE_RAW_DATA, subghz_additional_data->opcode);
                            hal_err = STM32WLxx_HAL_STATUS_ERROR_UNEXPECTED_DATA;
                            break;
                        }

                        drv_ctx->radio_rx_packet->payload_len = drv_ctx->last_irq_status.followup_payload_len - (sizeof(*subghz_additional_data) - sizeof(subghz_additional_data->subghz_rx_buf));
                        SID_PAL_ASSERT(sizeof(drv_ctx->radio_rx_packet->rcv_payload) >= drv_ctx->radio_rx_packet->payload_len);
                        SID_STM32_UTIL_fast_memcpy(drv_ctx->radio_rx_packet->rcv_payload, subghz_additional_data->subghz_rx_buf, drv_ctx->radio_rx_packet->payload_len);
                    }
                    else
                    {
                        drv_ctx->radio_rx_packet->payload_len = 0u;
                    }

                    /* Indicate that Sidewalk's radio event handler shall be invoked */
                    trigger_sid_radio_event_handler = TRUE;
                }
                else
                {
                    STM32WLxx_HAL_LOG_DEBUG("STM32WLxx_APP_IRQ_SUBGHZ is masked - skipped");
                }
                hal_err = STM32WLxx_HAL_STATUS_OK;
                break;

            case STM32WLxx_APP_IRQ_SPI_REQ_DONE:
                if ((drv_ctx->app_irq_mask & active_irq) != 0u)
                {
                    indicate_radio_request_completed = TRUE;
                }
                else
                {
                    STM32WLxx_HAL_LOG_DEBUG("STM32WLxx_APP_IRQ_SPI_REQ_DONE is masked - skipped");
                }
                hal_err = STM32WLxx_HAL_STATUS_OK;
                break;

            case STM32WLxx_APP_IRQ_SPI_HANDSHAKE:
                if ((drv_ctx->app_irq_mask & active_irq) != 0u)
                {
                    SID_PAL_LOG_WARNING("STM32WLxx APP indicated a handshake request");
                    handshake_request_active = TRUE;
                }
                else
                {
                    STM32WLxx_HAL_LOG_DEBUG("STM32WLxx_APP_IRQ_SPI_HANDSHAKE is masked - skipped");
                }
                hal_err = STM32WLxx_HAL_STATUS_OK;
                break;

            case STM32WLxx_APP_IRQ_USER_DATA:
                if ((drv_ctx->app_irq_mask & active_irq) != 0u)
                {
                    STM32WLxx_HAL_LOG_DEBUG("STM32WLxx APP - received incoming user data");
                    if (drv_ctx->on_user_data_rx != NULL)
                    {
                        const stm32wlxx_rcp_user_data_irq_followup_t * const user_data_frame = (stm32wlxx_rcp_user_data_irq_followup_t *)(void *)drv_ctx->config->internal_buffer.p;
                        if (user_data_frame->opcode != STM32WLxx_RADIO_COMM_OPCODE_USER_DATA)
                        {
                            SID_PAL_LOG_ERROR("Invalid User Data IRQ follow-up frame. Expected OpCode 0x%02X but got 0x%02X", STM32WLxx_RADIO_COMM_OPCODE_USER_DATA, user_data_frame->opcode);
                            hal_err = STM32WLxx_HAL_STATUS_ERROR_UNEXPECTED_DATA;
                            break;
                        }
                        else
                        {
                            drv_ctx->on_user_data_rx(user_data_frame->data, drv_ctx->last_irq_status.followup_payload_len);
                        }
                    }
                }
                else
                {
                    STM32WLxx_HAL_LOG_DEBUG("STM32WLxx_APP_IRQ_USER_DATA is masked - skipped");
                }
                hal_err = STM32WLxx_HAL_STATUS_OK;
                break;

            default:
                SID_PAL_LOG_ERROR("Multiple STM32WLxx Radio App IRQ flags reported simultaneously: 0x%x", active_irq);
                hal_err = STM32WLxx_HAL_STATUS_ERROR_INVALID_STATE;
                break;
        }

        /* If this is not a Handshake request we need to acknowledge the IRQ regardless of the errors in the processing */
recovery:
        if (FALSE == handshake_request_active)
        {
            stm32wlxx_hal_status_t tmp_err;

            /* If we are in  error state flush the SPI first so that the GPIO Handshake can proceed */
            if (hal_err != STM32WLxx_HAL_STATUS_OK)
            {
                tmp_err = _flush_spi(drv_ctx);
                if (tmp_err != STM32WLxx_HAL_STATUS_OK)
                {
                    STM32WLxx_HAL_LOG_ERROR("Unable to flush STM32WLxx SPI bus. HAL error %u", (uint32_t)tmp_err);
                    break;

                    //TODO: this seems to be a fatal error, handshake won't be possible. Probably we need to disable radio IRQ and report a permanent fault
                }
            }

            /* Send out IRQ acknowledgment to clear the IRQ indication by the STM32WLxx side regardless of any possible local error */
            tmp_err = stm32wlxx_hal_acknowledge_radio_irq(drv_ctx);
            if (tmp_err != STM32WLxx_HAL_STATUS_OK)
            {
                STM32WLxx_HAL_LOG_ERROR("Unable to acknowledge STM32WLxx IRQ status. HAL error %u", (uint32_t)tmp_err);
                break;
            }

            /* Wait for IRQ line to be cleared. Keep in mind this function is invoked from MCU's IRQ context - we cannot wait for long here */
            tmp_err = stm32wlxx_hal_wait_radio_irq_released(drv_ctx, STM32WLxx_HAL_RADIO_IRQ_RELEASE_WAIT_TIME_US);
            if (tmp_err != STM32WLxx_HAL_STATUS_OK)
            {
                STM32WLxx_HAL_LOG_ERROR("STM32WLxx failed to release the IRQ line after successful ACK. HAL error %u", (uint32_t)tmp_err);
                break;
            }

            /* Invoke callbacks after the IRQ is fully processed and acknowledged */
            if (STM32WLxx_HAL_STATUS_OK == hal_err)
            {
                /* SubGHz radio event */
                if (trigger_sid_radio_event_handler != FALSE)
                {
                    /* Call the Sidewalk-specific radio event handler */
                    const int32_t sidewalk_event_proc_error = stm32wlxx_radio_sidewalk_event_process();

                    /* Provide the Sidewalk-specific error outside if storage is supplied */
                    if (out_sidewalk_error != NULL)
                    {
                        *out_sidewalk_error = sidewalk_event_proc_error;
                    }

                    /* Do not set hal_err even if the stm32wlxx_radio_sidewalk_event_process() call reported an error */
                }

                /* Acknowledgment of a request (e.g. standby, sleep, etc.) */
                if (indicate_radio_request_completed != FALSE)
                {
                    /* Clear the ongoing transaction flag to indicate the completion of th request with ACK */
                    drv_ctx->req_with_ack_ongoing = FALSE;
                }
            }

            /* Report any failure, but do not override hal_err with STM32WLxx_HAL_STATUS_OK */
            if (tmp_err != STM32WLxx_HAL_STATUS_OK)
            {
                hal_err = tmp_err;
                break;
            }
        }
    } while (0);
    /*----------------------------------------------------------------------------*/

    /* DANGER ZONE: re-synchronization procedure below must not be interrupted by any other
     * STM32WLxx operation. If you are using SWI handler that runs in IRQ context please
     * make sure your SWI has lower priority than GPIO IRQ for STM32WLxx IRQ line. If SWI
     * handler runs in a task context than no special actions are required since the code
     * below runs in GPIO IRQ context and cannot be interrupted by a task.
     */

    /* If an error occurred we need to do a Handshake request before proceeding --*/
    if ((hal_err != STM32WLxx_HAL_STATUS_OK))
    {
        do
        {
            stm32wlxx_rcp_irq_status_t irq_status;

            SID_PAL_LOG_WARNING("Performing Handshake to reset STM32WLxx state");

            /* Perform GPIO handshake request */
            hal_err = stm32wlxx_hal_gpio_handshake_request(drv_ctx);

            if (hal_err != STM32WLxx_HAL_STATUS_OK)
            {
                STM32WLxx_HAL_LOG_ERROR("STM32WLxx GPIO handshaking failed. HAL error %d", (int32_t)hal_err);
                break;
            }

            STM32WLxx_HAL_LOG_INFO("Successful GPIO handshake with STM32WLxx, proceeding with SPI handshake...");

            /* If we got here the IRQ pin is actively driven low by an external source - we can send a handshake message via SPI now */
            hal_err = stm32wlxx_hal_retrieve_radio_irq(drv_ctx,  &irq_status, NULL, 0u);

            if (hal_err != STM32WLxx_HAL_STATUS_OK)
            {
                STM32WLxx_HAL_LOG_ERROR("Unable to retrieve STM32WLxx IRQ status. HAL error %u", (uint32_t)hal_err);
                break;
            }

            STM32WLxx_HAL_LOG_DEBUG("Retrieved STM32WLxx IRQ status. IRQ flags: 0x%x, followup data size: %u", irq_status.irq_flags, irq_status.followup_payload_len);

            /* Check that we have received a valid Handshake IRQ */
            if ((irq_status.irq_flags != STM32WLxx_APP_IRQ_SPI_HANDSHAKE) || (irq_status.followup_payload_len != 0u))
            {
                STM32WLxx_HAL_LOG_ERROR("STM32WLxx SPI Handshaking failed. STM32WLxx IRQ does not indicate the desired Handshake status. IRQ flags: 0x%x", irq_status.irq_flags);
                hal_err = STM32WLxx_HAL_STATUS_ERROR_INVALID_STATE;
                break;
            }

            STM32WLxx_HAL_LOG_DEBUG("STM32WLxx IRQ indicates the desired Handshake status");
            handshake_request_active = TRUE;
            hal_err = STM32WLxx_HAL_STATUS_OK;
        } while (0);
    }
    /*----------------------------------------------------------------------------*/

    /* If a valid handshake request is active we need to resend the config -------*/
    if (handshake_request_active != FALSE)
    {
        do
        {
            int32_t err;
            const uint8_t prev_radio_state = drv_ctx->radio_state;

            SID_PAL_LOG_DEBUG("Re-sending radio cfg due to Handshake request");

            /* Send out IRQ acknowledgment to clear the IRQ indication by the STM32WLxx side - this is a mandatory step regardless of the valid or invalid IRQ indication */
            hal_err = stm32wlxx_hal_acknowledge_radio_irq(drv_ctx);
            if (hal_err != STM32WLxx_HAL_STATUS_OK)
            {
                STM32WLxx_HAL_LOG_ERROR("Unable to acknowledge STM32WLxx IRQ status. HAL error %u", (uint32_t)hal_err);
                break;
            }

            hal_err = stm32wlxx_hal_wait_radio_irq_released(drv_ctx, STM32WLxx_HAL_RADIO_IRQ_RELEASE_WAIT_TIME_US);
            if (hal_err != STM32WLxx_HAL_STATUS_OK)
            {
                STM32WLxx_HAL_LOG_ERROR("STM32WLxx failed to release the IRQ line after successful ACK. HAL error %u", (uint32_t)hal_err);
                break;
            }

            /* STM32WLxx radio will be in Standby state at this point and ready to accept config data */
            drv_ctx->radio_state = SID_PAL_RADIO_STANDBY;

            /* Send out SubGHz radio configuration via SPI to STM32WLxx ------------------*/
            err = stm32wlxx_radio_send_subghz_config(FALSE);
            if (err != RADIO_ERROR_NONE)
            {
                /* Logs are provided by stm32wlxx_radio_send_subghz_config() */
                SID_PAL_LOG_ERROR("Failed to re-send SubGHz HW config. Error %d", err);
                hal_err = STM32WLxx_HAL_STATUS_ERROR_HW;
                break;
            }

            /* Perform the final STM32WLxx Sidewalk Radio App initialization steps */
            hal_err = stm32wlxx_hal_apply_base_hw_config(drv_ctx, FALSE);
            if (hal_err != STM32WLxx_HAL_STATUS_OK)
            {
                SID_PAL_LOG_ERROR("Failed to apply essential SubGHz HW config. Error code %u", (uint32_t)hal_err);
                err = RADIO_ERROR_IO_ERROR;
                break;
            }

            /* Explicitly set modem mode on the remote side */
            hal_err = stm32wlxx_hal_radio_set_modem_mode(drv_ctx, drv_ctx->modem);
            if (hal_err != STM32WLxx_HAL_STATUS_OK)
            {
                SID_PAL_LOG_ERROR("Failed to set SubGHz modem mode. Error code %u", (uint32_t)hal_err);
                err = RADIO_ERROR_IO_ERROR;
                break;
            }

            /* Restore the selected radio frequency */
            hal_err = stm32wlxx_hal_radio_set_frequency(drv_ctx, drv_ctx->radio_freq_hz);
            if (hal_err != STM32WLxx_HAL_STATUS_OK)
            {
                SID_PAL_LOG_ERROR("Failed to set SubGHz frequency. Error code %u", (uint32_t)hal_err);
                err = RADIO_ERROR_IO_ERROR;
                break;
            }

            /* If the radio was in Sleep before the handshake request send it back into Sleep for power saving */
            if (SID_PAL_RADIO_SLEEP == prev_radio_state)
            {
                hal_err = stm32wlxx_hal_radio_sleep(drv_ctx, 0u, FALSE);
                if (hal_err != STM32WLxx_HAL_STATUS_OK)
                {
                    /* This is not a fatal error. The radio is in Standby already, software may proceed even if the radio failed to enter Sleep back for some reason. There's only power consumption penalty here */
                    SID_PAL_LOG_ERROR("Failed to put STM32WLxx back to sleep. Error code %u", (uint32_t)hal_err);
                    err = RADIO_ERROR_IO_ERROR;
                    drv_ctx->radio_state = SID_PAL_RADIO_UNKNOWN;
                    break;
                }

                /* The radio is back in Sleep now */
                drv_ctx->radio_state = SID_PAL_RADIO_SLEEP;
            }
            else
            {
                /* The radio was in Standby or in some active state (Tx, Rx, etc.) - now it's in Standby as a result of the handshake procedure */
            }

            drv_ctx->error_monitor.drv_err_cntr = 0u;
            SID_PAL_LOG_INFO("Re-synchronized radio config with STM32WLxx");
        } while (0);
    }
    /* End of DANGER ZONE --------------------------------------------------------*/

    if (hal_err != STM32WLxx_HAL_STATUS_OK)
    {
        SID_PAL_LOG_ERROR("STM32WLxx IRQ handling finished with error %u", hal_err);
    }
    STM32WLxx_HAL_LOG_DEBUG("radio_stm32wlxx_on_irq_detected - done");

    return hal_err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED stm32wlxx_hal_status_t stm32wlxx_hal_disarm_irq(halo_drv_stm32wlxx_ctx_t * const drv_ctx)
{
    stm32wlxx_hal_status_t err = STM32WLxx_HAL_STATUS_ERROR_GENERIC;
    sid_error_t sid_err;

    /* Run in a critical region to ensure consistency of  hardware and logical states */
    sid_pal_enter_critical_region();

    do
    {
        /* Disable IRQ line */
        sid_err = sid_pal_gpio_irq_disable(drv_ctx->config->gpio.radio_irq);
        if (sid_err != SID_ERROR_NONE)
        {
            STM32WLxx_HAL_LOG_ERROR("Failed to disable radio IRQ. Error %d", (int32_t)sid_err);
            err = STM32WLxx_HAL_STATUS_ERROR_HW;
            break;
        }

        /* Disconnect GPIO pin from IRQ line */
        sid_err = sid_pal_gpio_set_irq(drv_ctx->config->gpio.radio_irq, SID_PAL_GPIO_IRQ_TRIGGER_NONE, NULL, NULL);
        if (sid_err != SID_ERROR_NONE)
        {
            STM32WLxx_HAL_LOG_ERROR("Failed to deinit radio IRQ GPIO. Error %d", (int32_t)sid_err);
            err = STM32WLxx_HAL_STATUS_ERROR_HW;
            break;
        }

        /* Update logical state of IRQ line */
        drv_ctx->irq_enabled = FALSE;

        /* Everything is fine */
        err = STM32WLxx_HAL_STATUS_OK;
    } while (0);

    sid_pal_exit_critical_region();

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED stm32wlxx_hal_status_t stm32wlxx_hal_arm_irq(halo_drv_stm32wlxx_ctx_t * const drv_ctx)
{
    stm32wlxx_hal_status_t err = STM32WLxx_HAL_STATUS_ERROR_GENERIC;
    sid_error_t sid_err;

    /* Run in a critical region to ensure that hardware and logical states are fully configured before the IRQ can be processed */
    sid_pal_enter_critical_region();

    do
    {
#if (__ARM_ARCH_6M__ == 0)
        /**
         * Indicate the radio IRQ is going to use elevated priority
         *
         * WARNING: It's important to do it before calling sid_pal_gpio_set_irq() because the respective ISR may be invoked immediately, before the IRQ priority is actual configured */
        drv_ctx->radio_is_running_high_prio = TRUE;
#endif /* (__ARM_ARCH_6M__ == 0) */

        /* Configure MCU pin to trigger IRQ */
        sid_err = sid_pal_gpio_set_irq(drv_ctx->config->gpio.radio_irq, SID_PAL_GPIO_IRQ_TRIGGER_LOW, _on_radio_irq_detected, (void*)drv_ctx);
        if (sid_err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to enable STM32WLxx IRQ in NVIC, error %d", (int32_t)sid_err);
            err = STM32WLxx_HAL_STATUS_ERROR_HW;
            break;
        }

        /* Update logical state of IRQ line */
        drv_ctx->irq_enabled = TRUE;

#if (__ARM_ARCH_6M__ == 0)
        /* Set elevated priority to SX126x IRQ line so we can capture radio IRQ timestamp with more precision */
        sid_err = sid_pal_gpio_ext_ifc_set_irq_priority(drv_ctx->config->gpio.radio_irq, drv_ctx->config->gpio.radio_irq_prio_high, 0u);
#else
        /* Set unified priority to SX126x IRQ line since dynamic IRQ priority changing is not supported */
        sid_err = sid_pal_gpio_ext_ifc_set_irq_priority(drv_ctx->config->gpio.radio_irq, drv_ctx->config->gpio.radio_irq_prio, 0u);
#endif /* (__ARM_ARCH_6M__ == 0) */
        if (sid_err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to set STM32WLxx IRQ priority, error %d", (int32_t)sid_err);
            err = STM32WLxx_HAL_STATUS_ERROR_HW;
            break;
        }

        /* Done */
        err = STM32WLxx_HAL_STATUS_OK;
    } while (0);

    sid_pal_exit_critical_region();

    return err;
}

/*----------------------------------------------------------------------------*/

stm32wlxx_hal_status_t stm32wlxx_hal_gpio_handshake_request(const halo_drv_stm32wlxx_ctx_t * const drv_ctx)
{
    stm32wlxx_hal_status_t err = STM32WLxx_HAL_STATUS_ERROR_GENERIC;
    sid_error_t sid_err;
    uint8_t pin_state;

    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->config != NULL);
    SID_PAL_ASSERT(drv_ctx != NULL);

    do
    {
        /* Step 1 - configure IRQ line as input and enable a pull-down */
        sid_err = sid_pal_gpio_pull_mode(drv_ctx->config->gpio.radio_irq, SID_PAL_GPIO_PULL_DOWN);
        if (sid_err != SID_ERROR_NONE)
        {
            STM32WLxx_HAL_LOG_ERROR("gpio_handshake #1 - failed to enable PD on IRQ. Error: %d", (int32_t)sid_err);
            err = STM32WLxx_HAL_STATUS_ERROR_GPIO;
            break;
        }
        sid_err = sid_pal_gpio_input_mode(drv_ctx->config->gpio.radio_irq, SID_PAL_GPIO_INPUT_CONNECT);
        if (sid_err != SID_ERROR_NONE)
        {
            STM32WLxx_HAL_LOG_ERROR("gpio_handshake #1 - failed to set IRQ pin input type. Error: %d", (int32_t)sid_err);
            err = STM32WLxx_HAL_STATUS_ERROR_GPIO;
            break;
        }
        sid_err = sid_pal_gpio_set_direction(drv_ctx->config->gpio.radio_irq, SID_PAL_GPIO_DIRECTION_INPUT);
        if (sid_err != SID_ERROR_NONE)
        {
            STM32WLxx_HAL_LOG_ERROR("gpio_handshake #1 - failed to set IRQ pin as input. Error: %d", (int32_t)sid_err);
            err = STM32WLxx_HAL_STATUS_ERROR_GPIO;
            break;
        }
        sid_pal_delay_us(STM32WLxx_HAL_IRQ_LINE_GPIO_SETTLE_TIME_US); /* Wait for pin to settle if it was floating */

        /* Capture pin state */
        sid_err = sid_pal_gpio_read(drv_ctx->config->gpio.radio_irq, &pin_state);
        if (sid_err != SID_ERROR_NONE)
        {
            STM32WLxx_HAL_LOG_ERROR("gpio_handshake #1 - failed to read IRQ pin state. Error: %d", (int32_t)sid_err);
            err = STM32WLxx_HAL_STATUS_ERROR_GPIO;
            break;
        }

        if (pin_state != 0u)
        {
            /* IRQ pin is actively driven high by an external source - that's not right since STM32WLxx shall configure
             * IRQ line as an open-drain output without a pull-up on STM32WLxx side
             */
            SID_PAL_LOG_ERROR("Wrong HW state - STM32WLxx radio IRQ line is actively driven high");
            err = STM32WLxx_HAL_STATUS_ERROR_HW;
            break;
        }
        else
        {
            STM32WLxx_HAL_LOG_DEBUG("gpio_handshake #1 - IRQ line is low with PD enabled");
        }

        /* Step 2 - now enable pull-up and check the state */
        sid_err = sid_pal_gpio_pull_mode(drv_ctx->config->gpio.radio_irq, SID_PAL_GPIO_PULL_UP);
        if (sid_err != SID_ERROR_NONE)
        {
            STM32WLxx_HAL_LOG_ERROR("gpio_handshake #2 - failed to enable PU on IRQ pin. Error: %d", (int32_t)sid_err);
            err = STM32WLxx_HAL_STATUS_ERROR_GPIO;
            break;
        }
        sid_pal_delay_us(STM32WLxx_HAL_IRQ_LINE_GPIO_SETTLE_TIME_US); /* Wait for pin to settle if it was floating */

        /* Capture pin state */
        sid_err = sid_pal_gpio_read(drv_ctx->config->gpio.radio_irq, &pin_state);
        if (sid_err != SID_ERROR_NONE)
        {
            STM32WLxx_HAL_LOG_ERROR("gpio_handshake #2 - failed to read IRQ pin state. Error: %d", (int32_t)sid_err);
            err = STM32WLxx_HAL_STATUS_ERROR_GPIO;
            break;
        }

        /* Do handshake requests and periodically test if the Handshake state was reached */
        uint32_t accumulated_wait_time = 0u;
        while ((pin_state != 0u) && (accumulated_wait_time < STM32WLxx_HAL_GPIO_HANDSHAKE_WAIT_TIMEOUT_MS))
        {
            /* Either nothing is connected to the radio IRQ line or STM32WLxx is not in the handshake state - do the GPIO handshake request */
            STM32WLxx_HAL_LOG_DEBUG("gpio_handshake #2 - IRQ line is high with PU enabled. Performing open circuit check");

            /* Set output to predefined state before direction configuration to avoid glitches */
            sid_err = sid_pal_gpio_write(drv_ctx->config->gpio.radio_irq, 0u);
            if (sid_err != SID_ERROR_NONE)
            {
                STM32WLxx_HAL_LOG_ERROR("gpio_handshake #2 - failed to drive IRQ pin. Error: %d", (int32_t)sid_err);
                err = STM32WLxx_HAL_STATUS_ERROR_GPIO;
                break;
            }

            /* Configure IRQ line as output, this will actively drive the line low and STM32WLxx shall recognize this as the GPIO handshake request */
            sid_err = sid_pal_gpio_output_mode(drv_ctx->config->gpio.radio_irq, SID_PAL_GPIO_OUTPUT_OPEN_DRAIN);
            if (sid_err != SID_ERROR_NONE)
            {
                STM32WLxx_HAL_LOG_ERROR("gpio_handshake #2 - failed to set IRQ pin output type. Error: %d", (int32_t)sid_err);
                err = STM32WLxx_HAL_STATUS_ERROR_GPIO;
                break;
            }
            sid_err = sid_pal_gpio_set_direction(drv_ctx->config->gpio.radio_irq, SID_PAL_GPIO_DIRECTION_OUTPUT);
            if (sid_err != SID_ERROR_NONE)
            {
                STM32WLxx_HAL_LOG_ERROR("gpio_handshake #2 - failed configure IRQ pin as output. Error: %d", (int32_t)sid_err);
                err = STM32WLxx_HAL_STATUS_ERROR_GPIO;
                break;
            }

            /* Give the STM32WLxx some time to react on the GPIO handshake request */
#ifdef SID_PAL_ENABLE_SCHEDULER_DELAY
            if (SID_STM32_UTIL_IS_IRQ() == FALSE)
            {
                sid_pal_scheduler_delay_ms(STM32WLxx_HAL_GPIO_HANDSHAKE_PROBE_PERIOD_MS);
            }
            else
#endif
            {
                /* Fall back to the blocking delay if scheduler delays are disabled */
                sid_pal_delay_us(STM32WLxx_HAL_GPIO_HANDSHAKE_PROBE_PERIOD_MS * 1000u);
            }

            accumulated_wait_time += STM32WLxx_HAL_GPIO_HANDSHAKE_PROBE_PERIOD_MS;

            /* Step 3 - reconfigure IRQ line back as input with PU enabled and check if the counterpart keeps it low */
            sid_err = sid_pal_gpio_input_mode(drv_ctx->config->gpio.radio_irq, SID_PAL_GPIO_INPUT_CONNECT);
            if (sid_err != SID_ERROR_NONE)
            {
                STM32WLxx_HAL_LOG_ERROR("gpio_handshake #3 - failed to set IRQ pin input type. Error: %d", (int32_t)sid_err);
                err = STM32WLxx_HAL_STATUS_ERROR_GPIO;
                break;
            }
            sid_err = sid_pal_gpio_set_direction(drv_ctx->config->gpio.radio_irq, SID_PAL_GPIO_DIRECTION_INPUT);
            if (sid_err != SID_ERROR_NONE)
            {
                STM32WLxx_HAL_LOG_ERROR("gpio_handshake #3 - failed to set IRQ pin as input. Error: %d", (int32_t)sid_err);
                err = STM32WLxx_HAL_STATUS_ERROR_GPIO;
                break;
            }

            /* Wait for pin to settle if it was floating */
            sid_pal_delay_us(STM32WLxx_HAL_IRQ_LINE_GPIO_SETTLE_TIME_US);

            sid_err = sid_pal_gpio_read(drv_ctx->config->gpio.radio_irq, &pin_state);
            if (sid_err != SID_ERROR_NONE)
            {
                STM32WLxx_HAL_LOG_ERROR("gpio_handshake #3 - failed to read IRQ pin state. Error: %d", (int32_t)sid_err);
                err = STM32WLxx_HAL_STATUS_ERROR_GPIO;
                break;
            }
        }

        if (err != STM32WLxx_HAL_STATUS_ERROR_GENERIC)
        {
            /* The loop above terminated with some error - just fast-forward it */
            break;
        }

        /* Final check */
        if (pin_state != 0u)
        {
            /* IRQ pin is not driven low - GPIO handshake failed */
            SID_PAL_LOG_ERROR("Wrong HW state - STM32WLxx radio IRQ line is open or STM32WLxx MCU is inoperable");
            err = STM32WLxx_HAL_STATUS_ERROR_HW;
            break;
        }
        else
        {
            if (0u == accumulated_wait_time)
            {
                STM32WLxx_HAL_LOG_DEBUG("gpio_handshake #2 - IRQ line is low with PU enabled. Open circuit check skipped");
            }
            else
            {
                STM32WLxx_HAL_LOG_DEBUG("gpio_handshake #3 - IRQ line is driven low after HS request. Total wait time: %u", accumulated_wait_time);
            }
        }

        /* Final IRQ pin config at this point - input with PU enabled, actively driven low by the STM32WLxx device */
        err = STM32WLxx_HAL_STATUS_OK;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED stm32wlxx_hal_status_t stm32wlxx_hal_subghz_reset(halo_drv_stm32wlxx_ctx_t * const drv_ctx)
{
    stm32wlxx_hal_status_t err = STM32WLxx_HAL_STATUS_ERROR_GENERIC;
    stm32wlxx_radio_comm_spi_frame_t subghz_reset_frame = {
        .opcode = STM32WLxx_RADIO_COMM_OPCODE_SUBGHZ_RESET,
        .payload = {
            .subghz_reset_req = {
                .reset_key_1 = STM32WLxx_RADIO_COMM_SUBGHZ_RESET_KEY_1,
                .reset_key_2 = STM32WLxx_RADIO_COMM_SUBGHZ_RESET_KEY_2,
            },
        }
    };

    do
    {
        /* Send out and wait the acknowledgment via IRQ */
        err = _send_request_with_ack(drv_ctx, (void *)&subghz_reset_frame, sizeof(subghz_reset_frame), STM32WLxx_HAL_RADIO_SUBGHZ_RESET_TIMEOUT_MS);

        if (STM32WLxx_HAL_STATUS_OK == err)
        {
            /* Everything is fine */
        }
        else if (STM32WLxx_HAL_STATUS_ERROR_TIMEOUT == err)
        {
            STM32WLxx_HAL_LOG_ERROR("Failed to send STM32WLxx SubGHz reset request - acknowledgment timeout");
            break;
        }
        else
        {
            STM32WLxx_HAL_LOG_ERROR("Failed to send STM32WLxx SubGHz reset request. HAL error %u", (uint32_t)err);
            break;
        }

        /* Inspect the received IRQ status */
        const stm32wlxx_req_completed_irq_details_t * const irq_details = &drv_ctx->last_irq_status.req_cmpltd_details;
        if ((drv_ctx->last_irq_status.irq_flags != STM32WLxx_APP_IRQ_SPI_REQ_DONE) || (irq_details->request_id != STM32WLxx_RADIO_COMM_OPCODE_SUBGHZ_RESET))
        {
            SID_PAL_LOG_ERROR("SubGHz reset failed. IRQ clash detected. Expected 0x%x flag, but got 0x%x, req_id 0x%x",
                              STM32WLxx_RADIO_COMM_OPCODE_SUBGHZ_RESET, drv_ctx->last_irq_status.irq_flags, irq_details->request_id);
            err = STM32WLxx_HAL_STATUS_ERROR_UNEXPECTED_DATA;
            break;
        }

        if (irq_details->status != 0u)
        {
            SID_PAL_LOG_ERROR("SubGHz reset failed. STM32WLxx returned error code %u", irq_details->status);
            err = STM32WLxx_HAL_STATUS_ERROR_REJECTED;
            break;
        }

        STM32WLxx_HAL_LOG_DEBUG("SubGHz Reset request succeeded");
        err = STM32WLxx_HAL_STATUS_OK;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED stm32wlxx_hal_status_t stm32wlxx_hal_set_subghz_tx_buf(halo_drv_stm32wlxx_ctx_t * const drv_ctx, const uint8_t * const data, const uint32_t data_len)
{
    stm32wlxx_hal_status_t err = STM32WLxx_HAL_STATUS_ERROR_GENERIC;

    SID_PAL_ASSERT(drv_ctx->config->internal_buffer.p != NULL);
    SID_PAL_ASSERT(data_len <= STM32WLxx_RADIO_COMM_SUBGHZ_TX_MAX_SIZE);

    stm32wlxx_rcp_set_subghz_tx_buf_t * const set_tx_buf_frame = (stm32wlxx_rcp_set_subghz_tx_buf_t *)(void *)drv_ctx->config->internal_buffer.p;

    set_tx_buf_frame->opcode       = STM32WLxx_RADIO_COMM_OPCODE_SUBGHZ_SET_TX_BUF;
    set_tx_buf_frame->write_length = (uint16_t)data_len;
    SID_STM32_UTIL_fast_memcpy(set_tx_buf_frame->tx_buf_content, data, data_len);

    uint32_t actual_frame_length = sizeof(*set_tx_buf_frame) - sizeof(set_tx_buf_frame->tx_buf_content) + data_len;

    err = _spi_transmit_data(drv_ctx, (uint8_t *)(void *)set_tx_buf_frame, actual_frame_length);
    if (err != STM32WLxx_HAL_STATUS_OK)
    {
        /* SPI failed */
        STM32WLxx_HAL_LOG_ERROR("Unable to send STM32WLxx SubGHz Set Tx Buffer request. SPI transfer error %u", (uint32_t)err);
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED stm32wlxx_hal_status_t stm32wlxx_hal_send_subghz_config(halo_drv_stm32wlxx_ctx_t * const drv_ctx, const stm32wlxx_radio_phy_cfg_t * const radio_config, const uint32_t need_ack)
{
    stm32wlxx_hal_status_t err = STM32WLxx_HAL_STATUS_ERROR_GENERIC;

    do
    {
        stm32wlxx_rcp_apply_cfg_t * const send_cfg_frame = (stm32wlxx_rcp_apply_cfg_t *)(void *)drv_ctx->config->internal_buffer.p;

        if (NULL == send_cfg_frame)
        {
            err = STM32WLxx_HAL_STATUS_INVALID_ARGS;
            break;
        }

        if (drv_ctx->config->internal_buffer.size < sizeof(*send_cfg_frame))
        {
            err = STM32WLxx_HAL_STATUS_ERROR_BUFFER_OVERFLOW;
            break;
        }

        /* Construct data */
        send_cfg_frame->opcode   = STM32WLxx_RADIO_COMM_OPCODE_SEND_CONFIG;
        send_cfg_frame->need_ack = need_ack;
        SID_STM32_UTIL_fast_memcpy(&send_cfg_frame->radio_config, radio_config, sizeof(send_cfg_frame->radio_config));

        /* Send out and wait the acknowledgment via IRQ */
        if (need_ack != FALSE)
        {
            err = _send_request_with_ack(drv_ctx, (void *)send_cfg_frame, sizeof(*send_cfg_frame), STM32WLxx_HAL_RADIO_SUBGHZ_SEND_CFG_TIMEOUT_MS);
        }
        else
        {
            err = _spi_transmit_data(drv_ctx, (void *)send_cfg_frame, sizeof(*send_cfg_frame));
        }

        if (STM32WLxx_HAL_STATUS_OK == err)
        {
            /* Everything is fine */
            if (FALSE == need_ack)
            {
                STM32WLxx_HAL_LOG_DEBUG("SubGHz config sent to STM32WLxx");
                break;
            }
        }
        else if (STM32WLxx_HAL_STATUS_ERROR_TIMEOUT == err)
        {
            STM32WLxx_HAL_LOG_ERROR("Failed to send SubGHz config - acknowledgment timeout");
            break;
        }
        else
        {
            STM32WLxx_HAL_LOG_ERROR("Failed to send SubGHz config. HAL error %u", (uint32_t)err);
            break;
        }

        /* If ACK was received - inspect the received IRQ status */
        const stm32wlxx_req_completed_irq_details_t * const irq_details = &drv_ctx->last_irq_status.req_cmpltd_details;
        if ((drv_ctx->last_irq_status.irq_flags != STM32WLxx_APP_IRQ_SPI_REQ_DONE) || (irq_details->request_id != STM32WLxx_RADIO_COMM_OPCODE_SEND_CONFIG))
        {
            SID_PAL_LOG_ERROR("Failed to send SubGHz config. IRQ clash detected. Expected 0x%x flag, but got 0x%x, req_id 0x%x",
                              STM32WLxx_RADIO_COMM_OPCODE_SEND_CONFIG, drv_ctx->last_irq_status.irq_flags, irq_details->request_id);
            err = STM32WLxx_HAL_STATUS_ERROR_UNEXPECTED_DATA;
            break;
        }

        if (irq_details->status != 0u)
        {
            SID_PAL_LOG_ERROR("Failed to send SubGHz config. STM32WLxx returned error code %u", irq_details->status);
            err = STM32WLxx_HAL_STATUS_ERROR_REJECTED;
            break;
        }

        STM32WLxx_HAL_LOG_DEBUG("SubGHz config sent to STM32WLxx");
        err = STM32WLxx_HAL_STATUS_OK;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED stm32wlxx_hal_status_t stm32wlxx_hal_apply_base_hw_config(halo_drv_stm32wlxx_ctx_t * const drv_ctx, const uint32_t need_ack)
{
    stm32wlxx_hal_status_t err = STM32WLxx_HAL_STATUS_ERROR_GENERIC;
    stm32wlxx_radio_comm_spi_frame_t apply_cfg_frame = {
        .opcode = STM32WLxx_RADIO_COMM_OPCODE_APPLY_BASE_HW_CONFIG,
        .payload = {
            .apply_hw_cfg = {
                .need_ack = (uint8_t)need_ack,
            },
        },
    };

    do
    {
        /* Send out and wait the acknowledgment via IRQ */
        if (need_ack != FALSE)
        {
            err = _send_request_with_ack(drv_ctx, (void *)&apply_cfg_frame, sizeof(apply_cfg_frame), STM32WLxx_HAL_RADIO_SUBGHZ_APPLY_CFG_TIMEOUT_MS);
        }
        else
        {
            err = _spi_transmit_data(drv_ctx, (void *)&apply_cfg_frame, sizeof(apply_cfg_frame));
        }

        if (STM32WLxx_HAL_STATUS_OK == err)
        {
            /* Everything is fine */
            if (FALSE == need_ack)
            {
                STM32WLxx_HAL_LOG_DEBUG("SubGHz base HW config applied");
                break;
            }
        }
        else if (STM32WLxx_HAL_STATUS_ERROR_TIMEOUT == err)
        {
            STM32WLxx_HAL_LOG_ERROR("Failed to apply base SubGHz config - acknowledgment timeout");
            break;
        }
        else
        {
            STM32WLxx_HAL_LOG_ERROR("Failed to apply base SubGHz config. HAL error %u", (uint32_t)err);
            break;
        }

        /* Inspect the received IRQ status */
        const stm32wlxx_req_completed_irq_details_t * const irq_details = &drv_ctx->last_irq_status.req_cmpltd_details;
        if ((drv_ctx->last_irq_status.irq_flags != STM32WLxx_APP_IRQ_SPI_REQ_DONE) || (irq_details->request_id != STM32WLxx_RADIO_COMM_OPCODE_APPLY_BASE_HW_CONFIG))
        {
            SID_PAL_LOG_ERROR("Failed to apply base SubGHz config. IRQ clash detected. Expected 0x%x flag, but got 0x%x, req_id 0x%x",
                              STM32WLxx_RADIO_COMM_OPCODE_APPLY_BASE_HW_CONFIG, drv_ctx->last_irq_status.irq_flags, irq_details->request_id);
            err = STM32WLxx_HAL_STATUS_ERROR_UNEXPECTED_DATA;
            break;
        }

        if (irq_details->status != 0u)
        {
            SID_PAL_LOG_ERROR("Failed to apply base SubGHz config. STM32WLxx returned error code %u", irq_details->status);
            err = STM32WLxx_HAL_STATUS_ERROR_REJECTED;
            break;
        }

        STM32WLxx_HAL_LOG_DEBUG("SubGHz base HW config applied");
        err = STM32WLxx_HAL_STATUS_OK;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED stm32wlxx_hal_status_t stm32wlxx_hal_set_tx_power(const halo_drv_stm32wlxx_ctx_t * const drv_ctx, const stm32wlxx_radio_pa_cfg_t * const sid_pa_cfg)
{
    stm32wlxx_hal_status_t err = STM32WLxx_HAL_STATUS_ERROR_GENERIC;
    stm32wlxx_radio_comm_spi_frame_t subghz_set_tx_power_frame = {
        .opcode = STM32WLxx_RADIO_COMM_OPCODE_SUBGHZ_SET_TX_POWER,
        .payload = {
            .set_tx_power = {
                .pa_duty_cycle   = sid_pa_cfg->pa_duty_cycle,
                .hp_max          = sid_pa_cfg->hp_max,
                .device_sel      = sid_pa_cfg->device_sel,
                .pa_lut          = sid_pa_cfg->pa_lut,
                .tx_power_reg    = sid_pa_cfg->tx_power_reg,
                .ramp_time       = sid_pa_cfg->ramp_time,
                .target_tx_power = sid_pa_cfg->target_tx_power,
            },
        },
    };

    err = _spi_transmit_data(drv_ctx, (uint8_t *)(void *)&subghz_set_tx_power_frame, sizeof(subghz_set_tx_power_frame));
    if (err != STM32WLxx_HAL_STATUS_OK)
    {
        /* SPI failed */
        STM32WLxx_HAL_LOG_ERROR("Unable to send STM32WLxx SubGHz Set Tx Power request. SPI transfer error %u", (uint32_t)err);
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED stm32wlxx_hal_status_t stm32wlxx_hal_radio_standby(halo_drv_stm32wlxx_ctx_t * const drv_ctx)
{
    stm32wlxx_hal_status_t err = STM32WLxx_HAL_STATUS_ERROR_GENERIC;
    stm32wlxx_radio_comm_spi_frame_t subghz_standby_frame = {
        .opcode = STM32WLxx_RADIO_COMM_OPCODE_SUBGHZ_STANDBY,
        .payload = {
            .raw = {0},
        },
    };

    do
    {
        /* Send out and wait the acknowledgment via IRQ */
        err = _send_request_with_ack(drv_ctx, (void *)&subghz_standby_frame, sizeof(subghz_standby_frame), STM32WLxx_HAL_RADIO_SUBGHZ_STANDBY_TIMEOUT_MS);

        if (STM32WLxx_HAL_STATUS_OK == err)
        {
            /* Everything is fine */
        }
        else if (STM32WLxx_HAL_STATUS_ERROR_TIMEOUT == err)
        {
            STM32WLxx_HAL_LOG_ERROR("Failed to send STM32WLxx SubGHz Standby request - acknowledgment timeout");
            break;
        }
        else
        {
            STM32WLxx_HAL_LOG_ERROR("Failed to send STM32WLxx SubGHz Standby request. HAL error %u", (uint32_t)err);
            break;
        }

        /* Inspect the received IRQ status */
        const stm32wlxx_req_completed_irq_details_t * const irq_details = &drv_ctx->last_irq_status.req_cmpltd_details;
        if ((drv_ctx->last_irq_status.irq_flags != STM32WLxx_APP_IRQ_SPI_REQ_DONE) || (irq_details->request_id != STM32WLxx_RADIO_COMM_OPCODE_SUBGHZ_STANDBY))
        {
            SID_PAL_LOG_ERROR("SubGHz Standby failed. IRQ clash detected. Expected 0x%x flag, but got 0x%x, req_id 0x%x",
                              STM32WLxx_RADIO_COMM_OPCODE_SUBGHZ_STANDBY, drv_ctx->last_irq_status.irq_flags, irq_details->request_id);
            err = STM32WLxx_HAL_STATUS_ERROR_UNEXPECTED_DATA;
            break;
        }

        if (irq_details->status != 0u)
        {
            SID_PAL_LOG_ERROR("SubGHz Standby failed. STM32WLxx returned error code %u", irq_details->status);
            err = STM32WLxx_HAL_STATUS_ERROR_REJECTED;
            break;
        }

        STM32WLxx_HAL_LOG_DEBUG("SubGHz Standby request succeeded");
        err = STM32WLxx_HAL_STATUS_OK;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED stm32wlxx_hal_status_t stm32wlxx_hal_radio_sleep(halo_drv_stm32wlxx_ctx_t * const drv_ctx, const uint32_t sleep_us, const uint32_t deep_sleep_request)
{
    stm32wlxx_hal_status_t err = STM32WLxx_HAL_STATUS_ERROR_GENERIC;
    stm32wlxx_radio_comm_spi_frame_t subghz_sleep_frame = {
        .opcode = STM32WLxx_RADIO_COMM_OPCODE_SUBGHZ_SLEEP,
        .payload = {
            .radio_sleep = {
                .sleep_duration_us = sleep_us,
                .deep_sleep_key_1  = 0u,
                .deep_sleep_key_2  = 0u,
                .deep_sleep_en     = FALSE,
            },
        },
    };

    do
    {
        if (deep_sleep_request != FALSE)
        {
            if (sleep_us != 0u)
            {
                /* Deep sleep is only allowed if Sidewalk is not going to operate the radio */
                err = STM32WLxx_HAL_STATUS_INVALID_ARGS;
                break;
            }

            /* Indicate that this is a deep-sleep request */
            subghz_sleep_frame.payload.radio_sleep.deep_sleep_en    = 1u;
            subghz_sleep_frame.payload.radio_sleep.deep_sleep_key_1 = STM32WLxx_RADIO_COMM_DEEP_SLEEP_KEY_1;
            subghz_sleep_frame.payload.radio_sleep.deep_sleep_key_2 = STM32WLxx_RADIO_COMM_DEEP_SLEEP_KEY_2;
        }

        /* Send out and wait the acknowledgment via IRQ */
        err = _send_request_with_ack(drv_ctx, (void *)&subghz_sleep_frame, sizeof(subghz_sleep_frame), STM32WLxx_HAL_RADIO_SUBGHZ_SLEEP_TIMEOUT_MS);

        if (STM32WLxx_HAL_STATUS_OK == err)
        {
            /* Everything is fine */
        }
        else if (STM32WLxx_HAL_STATUS_ERROR_TIMEOUT == err)
        {
            STM32WLxx_HAL_LOG_ERROR("Failed to send STM32WLxx SubGHz sleep mode request - acknowledgment timeout");
            break;
        }
        else
        {
            STM32WLxx_HAL_LOG_ERROR("Failed to send STM32WLxx SubGHz sleep mode request. HAL error %u", (uint32_t)err);
            break;
        }

        /* Inspect the received IRQ status */
        const stm32wlxx_req_completed_irq_details_t * const irq_details = &drv_ctx->last_irq_status.req_cmpltd_details;
        if ((drv_ctx->last_irq_status.irq_flags != STM32WLxx_APP_IRQ_SPI_REQ_DONE) || (irq_details->request_id != STM32WLxx_RADIO_COMM_OPCODE_SUBGHZ_SLEEP))
        {
            SID_PAL_LOG_ERROR("SubGHz Sleep failed. IRQ clash detected. Expected 0x%x flag, but got 0x%x, req_id 0x%x",
                              STM32WLxx_RADIO_COMM_OPCODE_SUBGHZ_SLEEP, drv_ctx->last_irq_status.irq_flags, irq_details->request_id);
            err = STM32WLxx_HAL_STATUS_ERROR_UNEXPECTED_DATA;
            break;
        }

        if (irq_details->status != 0u)
        {
            SID_PAL_LOG_ERROR("SubGHz Sleep failed. STM32WLxx returned error code %u", irq_details->status);
            err = STM32WLxx_HAL_STATUS_ERROR_REJECTED;
            break;
        }

        STM32WLxx_HAL_LOG_DEBUG("SubGHz Sleep request succeeded");
        err = STM32WLxx_HAL_STATUS_OK;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED stm32wlxx_hal_status_t stm32wlxx_hal_radio_set_modem_mode(const halo_drv_stm32wlxx_ctx_t * const drv_ctx, stm32wlxx_pal_radio_modem_mode_t mode)
{
    SID_PAL_ASSERT(mode <= UINT8_MAX); /* Ensure the enum value fits into one byte */

    stm32wlxx_hal_status_t err = STM32WLxx_HAL_STATUS_ERROR_GENERIC;
    stm32wlxx_radio_comm_spi_frame_t set_modem_mode_frame = {
        .opcode = STM32WLxx_RADIO_COMM_OPCODE_SUBGHZ_SET_MODEM_MODE,
        .payload = {
            .set_modem_mode = {
                .mode = (uint8_t)mode,
            },
        },
    };

    err = _spi_transmit_data(drv_ctx, (uint8_t *)(void *)&set_modem_mode_frame, sizeof(set_modem_mode_frame));
    if (err != STM32WLxx_HAL_STATUS_OK)
    {
        /* SPI failed */
        STM32WLxx_HAL_LOG_ERROR("Unable to send STM32WLxx SubGHz Set Modem Mode request. SPI transfer error %u", (uint32_t)err);
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED stm32wlxx_hal_status_t stm32wlxx_hal_radio_set_frequency(const halo_drv_stm32wlxx_ctx_t * const drv_ctx, const uint32_t freq)
{
    stm32wlxx_hal_status_t err = STM32WLxx_HAL_STATUS_ERROR_GENERIC;
    stm32wlxx_radio_comm_spi_frame_t set_frequency_frame = {
        .opcode = STM32WLxx_RADIO_COMM_OPCODE_SUBGHZ_SET_FREQUENCY,
        .payload = {
            .set_frequency = {
                .frequency = freq,
            },
        },
    };

    err = _spi_transmit_data(drv_ctx, (uint8_t *)(void *)&set_frequency_frame, sizeof(set_frequency_frame));
    if (err != STM32WLxx_HAL_STATUS_OK)
    {
        /* SPI failed */
        STM32WLxx_HAL_LOG_ERROR("Unable to send STM32WLxx SubGHz Set Frequency request. SPI transfer error %u", (uint32_t)err);
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED stm32wlxx_hal_status_t stm32wlxx_hal_radio_set_syncword(const halo_drv_stm32wlxx_ctx_t * const drv_ctx, const stm32wlxx_pal_radio_modem_mode_t target_modem, const uint8_t * const sync_word, const uint32_t sync_word_len)
{
    stm32wlxx_hal_status_t err;
    stm32wlxx_radio_comm_spi_frame_t set_syncword_req = {
        .opcode = STM32WLxx_RADIO_COMM_OPCODE_SUBGHZ_SET_SYNCWORD,
        .payload = {
            .set_sync_word = {
                .modem = target_modem,
            },
        },
    };

    do
    {
        if (STM32WLxx_SUBGHZ_RADIO_MODEM_MODE_FSK == target_modem)
        {
            SID_PAL_ASSERT(sync_word_len <= STM32WLxx_RADIO_COMM_FSK_SYNC_WORD_LENGTH);
            set_syncword_req.payload.set_sync_word.fsk_sync_word.data_len = (uint8_t)sync_word_len;
            SID_STM32_UTIL_fast_memcpy(set_syncword_req.payload.set_sync_word.fsk_sync_word.data, sync_word, sync_word_len);
        }
        else if (STM32WLxx_SUBGHZ_RADIO_MODEM_MODE_LORA == target_modem)
        {
        	SID_PAL_ASSERT(sync_word_len == sizeof(set_syncword_req.payload.set_sync_word.lora_sync_word));
            const uint16_t * const src_ptr = (uint16_t *)(void *)sync_word;
            set_syncword_req.payload.set_sync_word.lora_sync_word = *src_ptr;
        }
        else
        {
            STM32WLxx_HAL_LOG_ERROR("Can't set sync word - invalid modem specified: %u", (uint32_t)target_modem);
            err = STM32WLxx_HAL_STATUS_INVALID_ARGS;
            break;
        }

        err = _spi_transmit_data(drv_ctx, (void *)&set_syncword_req, sizeof(set_syncword_req));
        if (err != STM32WLxx_HAL_STATUS_OK)
        {
            STM32WLxx_HAL_LOG_ERROR("Failed to send Set Sync Word request. SPI transfer error %u", (uint32_t)err);
        }
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED stm32wlxx_hal_status_t stm32wlxx_hal_set_lora_symbol_timeout(const halo_drv_stm32wlxx_ctx_t * const drv_ctx, const uint8_t symb_timeout)
{
    stm32wlxx_hal_status_t err;
    stm32wlxx_radio_comm_spi_frame_t set_lora_symb_to_req = {
        .opcode = STM32WLxx_RADIO_COMM_OPCODE_SET_LORA_SYMB_TIMEOUT,
        .payload = {
            .set_lora_symb_timeout = {
                .symbol_timeout = symb_timeout,
            },
        },
    };

    err = _spi_transmit_data(drv_ctx, (void *)&set_lora_symb_to_req, sizeof(set_lora_symb_to_req));
    if (err != STM32WLxx_HAL_STATUS_OK)
    {
        STM32WLxx_HAL_LOG_ERROR("Failed to send Set LoRa Symbol Timeout request. SPI transfer error %u", (uint32_t)err);
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED stm32wlxx_hal_status_t stm32wlxx_hal_set_lora_modulation_params(const halo_drv_stm32wlxx_ctx_t * const drv_ctx, const stm32wlxx_radio_lora_phy_mod_params_t * const mod_params)
{
    stm32wlxx_hal_status_t err;
    stm32wlxx_radio_comm_spi_frame_t set_lora_mod_params_req = {
        .opcode = STM32WLxx_RADIO_COMM_OPCODE_SET_LORA_MOD_PARAMS,
        .payload = {
            .set_lora_mod_params = {
                .mod_params = *mod_params,
            },
        },
    };

    err = _spi_transmit_data(drv_ctx, (void *)&set_lora_mod_params_req, sizeof(set_lora_mod_params_req));
    if (err != STM32WLxx_HAL_STATUS_OK)
    {
        STM32WLxx_HAL_LOG_ERROR("Failed to send Set LoRa Modulation Params request. SPI transfer error %u", (uint32_t)err);
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED stm32wlxx_hal_status_t stm32wlxx_hal_radio_set_lora_pkt_params(const halo_drv_stm32wlxx_ctx_t * const drv_ctx, const stm32wlxx_radio_lora_phy_pkt_params_t pkt_params)
{
    stm32wlxx_hal_status_t err;
    stm32wlxx_radio_comm_spi_frame_t set_lora_pkt_params_req = {
        .opcode = STM32WLxx_RADIO_COMM_OPCODE_SET_LORA_PKT_PARAMS,
        .payload = {
            .set_lora_pkt_params = {
                .packet_params = pkt_params,
            },
        },
    };

    err = _spi_transmit_data(drv_ctx, (void *)&set_lora_pkt_params_req, sizeof(set_lora_pkt_params_req));
    if (err != STM32WLxx_HAL_STATUS_OK)
    {
        STM32WLxx_HAL_LOG_ERROR("Failed to send Set LoRa Packet Params request. SPI transfer error %u", (uint32_t)err);
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED stm32wlxx_hal_status_t stm32wlxx_hal_radio_set_lora_cad_params(const halo_drv_stm32wlxx_ctx_t * const drv_ctx)
{
    stm32wlxx_hal_status_t err;
    stm32wlxx_radio_comm_spi_frame_t set_lora_cad_params_req = {
        .opcode = STM32WLxx_RADIO_COMM_OPCODE_SET_LORA_CAD_PARAMS,
        .payload = {
            .raw = {0},
        },
    };

    err = _spi_transmit_data(drv_ctx, (void *)&set_lora_cad_params_req, sizeof(set_lora_cad_params_req));
    if (err != STM32WLxx_HAL_STATUS_OK)
    {
        STM32WLxx_HAL_LOG_ERROR("Failed to send Set LoRa CAD Params request. SPI transfer error %u", (uint32_t)err);
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED stm32wlxx_hal_status_t stm32wlxx_hal_set_fsk_modulation_params(const halo_drv_stm32wlxx_ctx_t * const drv_ctx, const stm32wlxx_radio_fsk_phy_mod_params_t * const mod_params)
{
    stm32wlxx_hal_status_t err;
    stm32wlxx_radio_comm_spi_frame_t set_fsk_mod_params_req = {
        .opcode = STM32WLxx_RADIO_COMM_OPCODE_SET_FSK_MOD_PARAMS,
        .payload = {
            .set_fsk_mod_params = {
                .mod_params = *mod_params,
            },
        },
    };

    err = _spi_transmit_data(drv_ctx, (void *)&set_fsk_mod_params_req, sizeof(set_fsk_mod_params_req));
    if (err != STM32WLxx_HAL_STATUS_OK)
    {
        STM32WLxx_HAL_LOG_ERROR("Failed to send Set FSK Modulation Params request. SPI transfer error %u", (uint32_t)err);
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED stm32wlxx_hal_status_t stm32wlxx_hal_radio_start_tx(const halo_drv_stm32wlxx_ctx_t * const drv_ctx)
{
    stm32wlxx_hal_status_t err;
    stm32wlxx_radio_comm_spi_frame_t start_tx_req = {
        .opcode = STM32WLxx_RADIO_COMM_OPCODE_SUBGHZ_START_TX,
        .payload = {
            .raw = {0},
        },
    };

    err = _spi_transmit_data(drv_ctx, (void *)&start_tx_req, sizeof(start_tx_req));
    if (err != STM32WLxx_HAL_STATUS_OK)
    {
        STM32WLxx_HAL_LOG_ERROR("Failed to send Start Tx request. SPI transfer error %u", (uint32_t)err);
    }

    return err;
}

/*----------------------------------------------------------------------------*/

stm32wlxx_hal_status_t stm32wlxx_hal_radio_start_rx(const halo_drv_stm32wlxx_ctx_t * const drv_ctx, const uint32_t timeout_us)
{
    stm32wlxx_hal_status_t err;
    stm32wlxx_radio_comm_spi_frame_t start_rx_req = {
        .opcode = STM32WLxx_RADIO_COMM_OPCODE_SUBGHZ_START_RX,
        .payload = {
            .start_rx_params = {
                .timeout_us = timeout_us,
            },
        },
    };

    err = _spi_transmit_data(drv_ctx, (void *)&start_rx_req, sizeof(start_rx_req));
    if (err != STM32WLxx_HAL_STATUS_OK)
    {
        STM32WLxx_HAL_LOG_ERROR("Failed to send Start Rx request. SPI transfer error %u", (uint32_t)err);
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED stm32wlxx_hal_status_t stm32wlxx_hal_retrieve_radio_irq(const halo_drv_stm32wlxx_ctx_t * const drv_ctx, stm32wlxx_rcp_irq_status_t * const irq_status,
                                                                                  uint8_t * const followup_data_buf, const uint32_t followup_buf_limit)
{
    stm32wlxx_hal_status_t err = STM32WLxx_HAL_STATUS_ERROR_GENERIC;

    /* Validate the inputs. followup_data_buf and followup_buf_limit can be null if the caller does not expect or need the IRQ follow-up data */
    if ((NULL == drv_ctx) || (NULL == irq_status))
    {
        return STM32WLxx_HAL_STATUS_INVALID_ARGS;
    }

    do
    {
        /* Construct the IRQ status read frame
         * IMPORTANT: IRQ status and follow-up data (if any) are preloaded by the STM32WLxx before indicating the IRQ. This means the following:
         *            1. Actual payload of the irq_status_read_req does not matter, STM32WLxx will send out IRQ status anyway. irq_status_read_req is sent for visual reference only
         *            2. Since all the data is preloaded by the STM32WLxx side, this function shall read out everything, even if the caller did not supply the buffer for the follow-up data or if such buffer is too short
         */
        stm32wlxx_radio_comm_spi_frame_t irq_status_read_req = {
            .opcode = STM32WLxx_RADIO_COMM_OPCODE_IRQ_STATUS,
            .payload = {
                .raw = {0},
            }
        };
        stm32wlxx_radio_comm_spi_frame_t irq_status_read_reply;

        /* Read out the IRQ status first */
        err = _spi_transmit_receive_data(drv_ctx, (void *)&irq_status_read_req, (void *)&irq_status_read_reply, sizeof(irq_status_read_reply));
        if (err != STM32WLxx_HAL_STATUS_OK)
        {
            STM32WLxx_HAL_LOG_ERROR("Unable to read out IRQ status. SPI transfer error %u", (uint32_t)err);
            break;
        }

        /* Check if we've actually received IRQ Status reply and not something else */
        if (irq_status_read_reply.opcode != STM32WLxx_RADIO_COMM_OPCODE_IRQ_STATUS)
        {
            STM32WLxx_HAL_LOG_ERROR("Unable to read out IRQ status. Received opcode 0x%2x instead", irq_status_read_reply.opcode);
            err = STM32WLxx_HAL_STATUS_ERROR_UNEXPECTED_DATA;
            break;
        }

        /* Store the received IRQ status */
        SID_STM32_UTIL_fast_memcpy(irq_status, &irq_status_read_reply.payload.irq_status, sizeof(*irq_status));

        /* Check if there's follow-up data pending */
        if (irq_status->followup_payload_len > 0u)
        {
            /* There's some data to read out */
            if ((followup_data_buf != NULL) && (followup_buf_limit >= irq_status->followup_payload_len))
            {
                /* Case 1 - the caller supplied the Rx buffer and there's enough space in there - receive directly into the buffer */
                size_t actually_received;
                err = _spi_receive_data(drv_ctx, followup_data_buf, irq_status->followup_payload_len, &actually_received);

                if (err != STM32WLxx_HAL_STATUS_OK)
                {
                    /* SPI failed */
                    STM32WLxx_HAL_LOG_ERROR("Unable to read out IRQ follow-up data. SPI transfer error %u", (uint32_t)err);
                    break;
                }
                else
                {
                    /* SPI transaction completed successfully, but we have a few more thing to assert */
                    if (actually_received != irq_status->followup_payload_len)
                    {
                        /* Mismatch between the expected and actually received amount of the follow-up data */
                        SID_PAL_LOG_ERROR("Failed to read out IRQ follow-up data. Expected %u bytes, but got only %u", irq_status->followup_payload_len, actually_received); /* This is serious as it may mean mismatch between irq_status->followup_payload_len and what is actually enqueued on STM32WLxx side - always log this error */
                        err = STM32WLxx_HAL_STATUS_ERROR_INCOMPLETE_XFER;
                        break;
                    }
                    else
                    {
                        /* Everything is ok */
                        STM32WLxx_HAL_LOG_DEBUG("Successfully received %u bytes of IRQ follow-up data.", actually_received);
                    }
                }
            }
            else
            {
                /* Case 2 - the provided buffer is too short + Case 3 - caller is not interested in the follow-up data */
                size_t actually_received;

                /* Perform dummy read to clean up STM32WLxx Tx buffer from the IRQ data completely */
                err = _spi_receive_data(drv_ctx, NULL, 0u, &actually_received);

                if (err != STM32WLxx_HAL_STATUS_OK)
                {
                    /* SPI failed */
                    STM32WLxx_HAL_LOG_ERROR("Unable to do dummy read of IRQ follow-up data. SPI transfer error %u", (uint32_t)err);
                    break;
                }
                else
                {
                    /* SPI transaction completed successfully, but we have a few more thing to assert */
                    if (actually_received != irq_status->followup_payload_len)
                    {
                        /* Mismatch between the expected and actually received amount of the follow-up data */
                        SID_PAL_LOG_ERROR("Failed to read out IRQ follow-up data. Expected %u bytes, but got only %u", irq_status->followup_payload_len, actually_received); /* This is serious as it may mean mismatch between irq_status->followup_payload_len and what is actually enqueued on STM32WLxx side - always log this error */
                        err = STM32WLxx_HAL_STATUS_ERROR_INCOMPLETE_XFER;
                        break;
                    }
                    else
                    {
                        /* Everything is ok */
                        STM32WLxx_HAL_LOG_DEBUG("Successfully done dummy read of IRQ follow-up data. Received %u bytes ", actually_received);
                    }
                }

                /* Report and error  */
                if ((followup_data_buf != NULL) && (followup_buf_limit < irq_status->followup_payload_len))
                {
                    STM32WLxx_HAL_LOG_ERROR("Failed to read out IRQ follow-up data. Expected %u bytes, but reception buffer can store only %u", irq_status->followup_payload_len, followup_buf_limit);
                    err = STM32WLxx_HAL_STATUS_ERROR_BUFFER_OVERFLOW;
                    break;
                }
            }
        }

        /* Everything is ok if we've got here */
        err = STM32WLxx_HAL_STATUS_OK;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED stm32wlxx_hal_status_t stm32wlxx_hal_acknowledge_radio_irq(const halo_drv_stm32wlxx_ctx_t * const drv_ctx)
{
    stm32wlxx_hal_status_t err = STM32WLxx_HAL_STATUS_ERROR_GENERIC;
    stm32wlxx_radio_comm_spi_frame_t irq_ack_frame = {
        .opcode = STM32WLxx_RADIO_COMM_OPCODE_IRQ_ACK,
        .payload = {
            .raw = {0},
        }
    };

    err = _spi_transmit_data(drv_ctx, (uint8_t *)(void *)&irq_ack_frame, sizeof(irq_ack_frame));
    if (err != STM32WLxx_HAL_STATUS_OK)
    {
        /* SPI failed */
        STM32WLxx_HAL_LOG_ERROR("Unable to send out STM32WLxx IRQ acknowledgment. SPI transfer error %u", (uint32_t)err);
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED stm32wlxx_hal_status_t stm32wlxx_hal_wait_radio_irq_released(const halo_drv_stm32wlxx_ctx_t * const drv_ctx, const uint32_t timeout_us)
{
    stm32wlxx_hal_status_t err = STM32WLxx_HAL_STATUS_ERROR_TIMEOUT;
    sid_error_t sid_err;
    uint8_t pin_state = STM32WLxx_HAL_RADIO_IRQ_PIN_STATE_ACTIVE;
    uint32_t accumulated_wait_time = 0u;
    uint32_t debounce_cnt = 0u;

    while ((accumulated_wait_time < timeout_us) && (debounce_cnt < STM32WLxx_HAL_RADIO_IRQ_PIN_STATE_PROBE_DEBOUNCE))
    {
        sid_err = sid_pal_gpio_read(drv_ctx->config->gpio.radio_irq, &pin_state);
        if (sid_err != SID_ERROR_NONE)
        {
            STM32WLxx_HAL_LOG_ERROR("Wait for IRQ released - failed to read IRQ pin state. Error: %d", (int32_t)sid_err);
            err = STM32WLxx_HAL_STATUS_ERROR_GPIO;
            break;
        }

        /* Return immediatly if the IRQ indication is cleared */
        if (STM32WLxx_HAL_RADIO_IRQ_PIN_STATE_INACTIVE == pin_state)
        {
            debounce_cnt++;
        }
        else
        {
            debounce_cnt = 0u;
        }

        /* Wait a bit to avoid consuming a lot of power and blocking the APB bus */
        sid_pal_delay_us(STM32WLxx_HAL_RADIO_IRQ_PIN_STATE_PROBE_PERIOD_US);
        accumulated_wait_time += STM32WLxx_HAL_RADIO_IRQ_PIN_STATE_PROBE_PERIOD_US;
    }

    if ((debounce_cnt >= STM32WLxx_HAL_RADIO_IRQ_PIN_STATE_PROBE_DEBOUNCE) && (SID_ERROR_NONE == sid_err))
    {
        err = STM32WLxx_HAL_STATUS_OK;
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED stm32wlxx_hal_status_t stm32wlxx_hal_radio_set_fsk_pkt_params(const halo_drv_stm32wlxx_ctx_t * const drv_ctx, stm32wlxx_pal_radio_fsk_pkt_params_t pkt_params)
{
    stm32wlxx_hal_status_t err;
    stm32wlxx_radio_comm_spi_frame_t set_fsk_pkt_params_req = {
        .opcode = STM32WLxx_RADIO_COMM_OPCODE_SET_FSK_PKT_PARAMS,
        .payload = {
            .set_fsk_pkt_params = {
                .packet_params = pkt_params,
            },
        },
    };

    err = _spi_transmit_data(drv_ctx, (void *)&set_fsk_pkt_params_req, sizeof(set_fsk_pkt_params_req));
    if (err != STM32WLxx_HAL_STATUS_OK)
    {
        STM32WLxx_HAL_LOG_ERROR("Failed to send Set FSK Packet Params request. SPI transfer error %u", (uint32_t)err);
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED stm32wlxx_hal_status_t stm32wlxx_hal_radio_start_carrier_sense(const halo_drv_stm32wlxx_ctx_t * const drv_ctx)
{
    stm32wlxx_hal_status_t err;
    stm32wlxx_radio_comm_spi_frame_t start_carrier_sense_req = {
        .opcode = STM32WLxx_RADIO_COMM_OPCODE_START_CARRIER_SENSE,
        .payload = {
            .raw = {0},
        }
    };

    err = _spi_transmit_data(drv_ctx, (void *)&start_carrier_sense_req, sizeof(start_carrier_sense_req));
    if (err != STM32WLxx_HAL_STATUS_OK)
    {
        STM32WLxx_HAL_LOG_ERROR("Failed to send Start Carrier Sense request to SubGHz. SPI transfer error %u", (uint32_t)err);
    }

    return err;
}

/*----------------------------------------------------------------------------*/

#if HALO_ENABLE_DIAGNOSTICS
SID_STM32_SPEED_OPTIMIZED stm32wlxx_hal_status_t stm32wlxx_hal_start_continuous_wave_tx(const halo_drv_stm32wlxx_ctx_t * const drv_ctx, const uint32_t freq)
{
    stm32wlxx_hal_status_t err;
    stm32wlxx_radio_comm_spi_frame_t start_cw_tx_req = {
        .opcode = STM32WLxx_RADIO_COMM_OPCODE_START_CW_TX,
        .payload = {
            .start_cw_tx_params = {
                .frequecy = freq,
            },
        }
    };

    err = _spi_transmit_data(drv_ctx, (void *)&start_cw_tx_req, sizeof(start_cw_tx_req));
    if (err != STM32WLxx_HAL_STATUS_OK)
    {
        STM32WLxx_HAL_LOG_ERROR("Failed to send Start SubGHz CW Tx request. SPI transfer error %u", (uint32_t)err);
    }

    return err;
}
#endif /* HALO_ENABLE_DIAGNOSTICS */

/*----------------------------------------------------------------------------*/

#if HALO_ENABLE_DIAGNOSTICS
SID_STM32_SPEED_OPTIMIZED stm32wlxx_hal_status_t stm32wlxx_hal_start_continuous_rx(const halo_drv_stm32wlxx_ctx_t * const drv_ctx)
{
    stm32wlxx_hal_status_t err;
    stm32wlxx_radio_comm_spi_frame_t start_continuous_rx_req = {
        .opcode = STM32WLxx_RADIO_COMM_OPCODE_START_CONTINUOUS_RX,
        .payload = {
            .raw = {0},
        }
    };

    err = _spi_transmit_data(drv_ctx, (void *)&start_continuous_rx_req, sizeof(start_continuous_rx_req));
    if (err != STM32WLxx_HAL_STATUS_OK)
    {
        STM32WLxx_HAL_LOG_ERROR("Failed to send Start SubGHz Continuous Rx request. SPI transfer error %u", (uint32_t)err);
    }

    return err;
}
#endif /* HALO_ENABLE_DIAGNOSTICS */

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED stm32wlxx_hal_status_t stm32wlxx_hal_send_user_data(halo_drv_stm32wlxx_ctx_t * const drv_ctx, const uint8_t * const data, const uint32_t data_len)
{
    stm32wlxx_hal_status_t err = STM32WLxx_HAL_STATUS_ERROR_GENERIC;

    SID_PAL_ASSERT(drv_ctx->config->internal_buffer.p != NULL);
    SID_PAL_ASSERT(data_len <= STM32WLxx_RADIO_COMM_SUBGHZ_TX_MAX_SIZE);

    stm32wlxx_rcp_user_data_t * const user_data_frame = (stm32wlxx_rcp_user_data_t *)(void *)drv_ctx->config->internal_buffer.p;

    user_data_frame->opcode      = STM32WLxx_RADIO_COMM_OPCODE_USER_DATA;
    user_data_frame->data_length = (uint16_t)data_len;
    SID_STM32_UTIL_fast_memcpy(user_data_frame->data, data, data_len);

    uint32_t actual_frame_length = sizeof(*user_data_frame) - sizeof(user_data_frame->data) + data_len;

    err = _spi_transmit_data(drv_ctx, (uint8_t *)(void *)user_data_frame, actual_frame_length);
    if (err != STM32WLxx_HAL_STATUS_OK)
    {
        /* SPI failed */
        STM32WLxx_HAL_LOG_ERROR("Unable to send user data to the STM32WLxx side. SPI transfer error %u", (uint32_t)err);
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED stm32wlxx_hal_status_t stm32wlxx_radio_hal_init_gpio(halo_drv_stm32wlxx_ctx_t * const drv_ctx)
{
    stm32wlxx_hal_status_t err = STM32WLxx_HAL_STATUS_ERROR_GENERIC;
    sid_error_t sid_err;

    do
    {
#if STM32WLxx_RADIO_CFG_USE_STATUS_LED
        /* Configure status LED ------------------------------------------------------*/
        /* Tx LED */
        if (HALO_GPIO_NOT_CONNECTED != drv_ctx->config->gpio.tx_led)
        {
            /* Make sure LED won't glitch - put GPIO output to Off state before the pin is configured as output */
            stm32wlxx_radio_hal_tx_led_off(drv_ctx);

            sid_err = sid_pal_gpio_output_mode(drv_ctx->config->gpio.tx_led, SID_PAL_GPIO_OUTPUT_PUSH_PULL);
            if (sid_err != SID_ERROR_NONE)
            {
                STM32WLxx_HAL_LOG_ERROR("sid_pal_gpio_output_mode() failed with error code %d, pin: %u", (int32_t)sid_err, drv_ctx->config->gpio.tx_led);
                err = STM32WLxx_HAL_STATUS_ERROR_GPIO;
                break;
            }
            sid_err = sid_pal_gpio_set_direction(drv_ctx->config->gpio.tx_led, SID_PAL_GPIO_DIRECTION_OUTPUT);
            if (sid_err != SID_ERROR_NONE)
            {
                STM32WLxx_HAL_LOG_ERROR("sid_pal_gpio_set_direction() failed with error code %d, pin: %u", (int32_t)sid_err, drv_ctx->config->gpio.tx_led);
                err = STM32WLxx_HAL_STATUS_ERROR_GPIO;
                break;
            }
        }

        /* Rx LED */
        if ((HALO_GPIO_NOT_CONNECTED != drv_ctx->config->gpio.rx_led) && (drv_ctx->config->gpio.rx_led != drv_ctx->config->gpio.tx_led))
        {
            stm32wlxx_radio_hal_rx_led_off(drv_ctx);

            sid_err = sid_pal_gpio_output_mode(drv_ctx->config->gpio.rx_led, SID_PAL_GPIO_OUTPUT_PUSH_PULL);
            if (sid_err != SID_ERROR_NONE)
            {
                STM32WLxx_HAL_LOG_ERROR("sid_pal_gpio_output_mode() failed with error code %d, pin: %u", (int32_t)sid_err, drv_ctx->config->gpio.rx_led);
                err = STM32WLxx_HAL_STATUS_ERROR_GPIO;
                break;
            }
            sid_err = sid_pal_gpio_set_direction(drv_ctx->config->gpio.rx_led, SID_PAL_GPIO_DIRECTION_OUTPUT);
            if (sid_err != SID_ERROR_NONE)
            {
                STM32WLxx_HAL_LOG_ERROR("sid_pal_gpio_set_direction() failed with error code %d, pin: %u", (int32_t)sid_err, drv_ctx->config->gpio.rx_led);
                err = STM32WLxx_HAL_STATUS_ERROR_GPIO;
                break;
            }
        }
        /*----------------------------------------------------------------------------*/
#endif /* STM32WLxx_RADIO_CFG_USE_STATUS_LED */

        err = STM32WLxx_HAL_STATUS_OK;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED stm32wlxx_hal_status_t stm32wlxx_radio_hal_deinit_gpio(halo_drv_stm32wlxx_ctx_t * const drv_ctx)
{
    stm32wlxx_hal_status_t err = STM32WLxx_HAL_STATUS_ERROR_GENERIC;
    sid_error_t sid_err;

    do
    {
#if STM32WLxx_RADIO_CFG_USE_STATUS_LED
        /* Set status LED pin to Hi-Z ------------------------------------------------*/
        /* Tx LED */
        if (HALO_GPIO_NOT_CONNECTED != drv_ctx->config->gpio.tx_led)
        {
            sid_err = sid_pal_gpio_input_mode(drv_ctx->config->gpio.tx_led, SID_PAL_GPIO_INPUT_DISCONNECT);
            if (sid_err != SID_ERROR_NONE)
            {
                STM32WLxx_HAL_LOG_ERROR("sid_pal_gpio_input_mode() failed with error code %d, pin: %u", (int32_t)sid_err, drv_ctx->config->gpio.tx_led);
                err = STM32WLxx_HAL_STATUS_ERROR_GPIO;
                break;
            }

            sid_err = sid_pal_gpio_set_direction(drv_ctx->config->gpio.tx_led, SID_PAL_GPIO_DIRECTION_INPUT);
            if (sid_err != SID_ERROR_NONE)
            {
                STM32WLxx_HAL_LOG_ERROR("sid_pal_gpio_set_direction() failed with error code %d, pin: %u", (int32_t)sid_err, drv_ctx->config->gpio.tx_led);
                err = STM32WLxx_HAL_STATUS_ERROR_GPIO;
                break;
            }
        }

        /* Rx LED */
        if ((HALO_GPIO_NOT_CONNECTED != drv_ctx->config->gpio.rx_led) && (drv_ctx->config->gpio.rx_led != drv_ctx->config->gpio.tx_led))
        {
            sid_err = sid_pal_gpio_input_mode(drv_ctx->config->gpio.rx_led, SID_PAL_GPIO_INPUT_DISCONNECT);
            if (sid_err != SID_ERROR_NONE)
            {
                STM32WLxx_HAL_LOG_ERROR("sid_pal_gpio_input_mode() failed with error code %d, pin: %u", (int32_t)sid_err, drv_ctx->config->gpio.rx_led);
                err = STM32WLxx_HAL_STATUS_ERROR_GPIO;
                break;
            }

            sid_err = sid_pal_gpio_set_direction(drv_ctx->config->gpio.rx_led, SID_PAL_GPIO_DIRECTION_INPUT);
            if (sid_err != SID_ERROR_NONE)
            {
                STM32WLxx_HAL_LOG_ERROR("sid_pal_gpio_set_direction() failed with error code %d, pin: %u", (int32_t)sid_err, drv_ctx->config->gpio.rx_led);
                err = STM32WLxx_HAL_STATUS_ERROR_GPIO;
                break;
            }
        }
        /*----------------------------------------------------------------------------*/
#endif /* STM32WLxx_RADIO_CFG_USE_STATUS_LED */

        err = STM32WLxx_HAL_STATUS_OK;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

#if STM32WLxx_RADIO_CFG_USE_STATUS_LED
SID_STM32_SPEED_OPTIMIZED stm32wlxx_hal_status_t stm32wlxx_radio_hal_tx_led_on(const halo_drv_stm32wlxx_ctx_t * const drv_ctx)
{
    stm32wlxx_hal_status_t err = STM32WLxx_HAL_STATUS_OK;

    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->config != NULL);

    if (HALO_GPIO_NOT_CONNECTED != drv_ctx->config->gpio.tx_led)
    {
        sid_error_t sid_err;
        uint8_t write_val = drv_ctx->config->gpio.tx_led_on_gpio_state != 0u ? 1u : 0u;

        sid_err = sid_pal_gpio_write(drv_ctx->config->gpio.tx_led, write_val);
        if (sid_err != SID_ERROR_NONE)
        {
            STM32WLxx_HAL_LOG_ERROR("sid_pal_gpio_write() failed with error code %d, pin: %u", (int32_t)sid_err, drv_ctx->config->gpio.tx_led);
            err = STM32WLxx_HAL_STATUS_ERROR_HW;
        }
    }

    return err;
}
#endif /* STM32WLxx_RADIO_CFG_USE_STATUS_LED */

/*----------------------------------------------------------------------------*/

#if STM32WLxx_RADIO_CFG_USE_STATUS_LED
SID_STM32_SPEED_OPTIMIZED stm32wlxx_hal_status_t stm32wlxx_radio_hal_tx_led_off(const halo_drv_stm32wlxx_ctx_t * const drv_ctx)
{
    stm32wlxx_hal_status_t err = STM32WLxx_HAL_STATUS_OK;

    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->config != NULL);

    if (HALO_GPIO_NOT_CONNECTED != drv_ctx->config->gpio.tx_led)
    {
        sid_error_t sid_err;
        uint8_t write_val = drv_ctx->config->gpio.tx_led_on_gpio_state != 0u ? 0u : 1u;

        sid_err = sid_pal_gpio_write(drv_ctx->config->gpio.tx_led, write_val);
        if (sid_err != SID_ERROR_NONE)
        {
            STM32WLxx_HAL_LOG_ERROR("sid_pal_gpio_write() failed with error code %d, pin: %u", (int32_t)sid_err, drv_ctx->config->gpio.tx_led);
            err = STM32WLxx_HAL_STATUS_ERROR_HW;
        }
    }

    return err;
}
#endif /* STM32WLxx_RADIO_CFG_USE_STATUS_LED */

/*----------------------------------------------------------------------------*/

#if STM32WLxx_RADIO_CFG_USE_STATUS_LED
SID_STM32_SPEED_OPTIMIZED stm32wlxx_hal_status_t stm32wlxx_radio_hal_rx_led_on(const halo_drv_stm32wlxx_ctx_t * const drv_ctx)
{
    stm32wlxx_hal_status_t err = STM32WLxx_HAL_STATUS_OK;

    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->config != NULL);

    if (HALO_GPIO_NOT_CONNECTED != drv_ctx->config->gpio.rx_led)
    {
        sid_error_t sid_err;
        uint8_t write_val = drv_ctx->config->gpio.rx_led_on_gpio_state != 0u ? 1u : 0u;

        sid_err = sid_pal_gpio_write(drv_ctx->config->gpio.rx_led, write_val);
        if (sid_err != SID_ERROR_NONE)
        {
            STM32WLxx_HAL_LOG_ERROR("sid_pal_gpio_write() failed with error code %d, pin: %u", (int32_t)sid_err, drv_ctx->config->gpio.rx_led);
            err = STM32WLxx_HAL_STATUS_ERROR_HW;
        }
    }

    return err;
}
#endif /* STM32WLxx_RADIO_CFG_USE_STATUS_LED */

/*----------------------------------------------------------------------------*/

#if STM32WLxx_RADIO_CFG_USE_STATUS_LED
SID_STM32_SPEED_OPTIMIZED stm32wlxx_hal_status_t stm32wlxx_radio_hal_rx_led_off(const halo_drv_stm32wlxx_ctx_t * const drv_ctx)
{
    stm32wlxx_hal_status_t err = STM32WLxx_HAL_STATUS_OK;

    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->config != NULL);

    if (HALO_GPIO_NOT_CONNECTED != drv_ctx->config->gpio.rx_led)
    {
        sid_error_t sid_err;
        uint8_t write_val = drv_ctx->config->gpio.rx_led_on_gpio_state != 0u ? 0u : 1u;

        sid_err = sid_pal_gpio_write(drv_ctx->config->gpio.rx_led, write_val);
        if (sid_err != SID_ERROR_NONE)
        {
            STM32WLxx_HAL_LOG_ERROR("sid_pal_gpio_write() failed with error code %d, pin: %u", (int32_t)sid_err, drv_ctx->config->gpio.rx_led);
            err = STM32WLxx_HAL_STATUS_ERROR_HW;
        }
    }

    return err;
}
#endif /* STM32WLxx_RADIO_CFG_USE_STATUS_LED */
