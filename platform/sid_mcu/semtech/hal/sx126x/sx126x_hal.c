/*
 * Copyright (c) 2019-2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * This file defines interface used by Semtech driver to perform platform specific
 * operations
 */
/**
  ******************************************************************************
  * @file    sx126x_hal.c
  * @brief   Semtech SX126x radio driver for Sidewalk running on STM32 platform
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

#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include <sx126x.h>
#include <sx126x_radio.h>
#include <sx126x_hal.h>

/* Sidewalk interfaces */
#include <sid_time_ops.h>
#include <sid_pal_assert_ifc.h>
#include <sid_pal_delay_ifc.h>
#include <sid_pal_gpio_ifc.h>
#include <sid_pal_gpio_ext_ifc.h>
#include <sid_pal_log_ifc.h>
#include <sid_pal_radio_ifc.h>
#include <sid_pal_serial_bus_ext_ifc.h>
#include <sid_pal_uptime_ifc.h>

/* Utilities and helpers */
#include <sid_stm32_common_utils.h>
#include <cmsis_compiler.h>

/* Private defines -----------------------------------------------------------*/

#ifndef SX126X_RADIO_HAL_EXTRA_LOGGING
/* Set SX126X_RADIO_HAL_EXTRA_LOGGING to 1 to enable extended logs */
#  define SX126X_RADIO_HAL_EXTRA_LOGGING            (0)
#endif

#if SX126X_RADIO_HAL_EXTRA_LOGGING
#  define SX126X_RADIO_HAL_LOG_ERROR(...)           SID_PAL_LOG_ERROR(__VA_ARGS__)
#  define SX126X_RADIO_HAL_LOG_WARNING(...)         SID_PAL_LOG_WARNING(__VA_ARGS__)
#  define SX126X_RADIO_HAL_LOG_INFO(...)            SID_PAL_LOG_INFO(__VA_ARGS__)
#  define SX126X_RADIO_HAL_LOG_DEBUG(...)           SID_PAL_LOG_DEBUG(__VA_ARGS__)
#  define SX126X_RADIO_HAL_LOG_TRACE(...)           SID_PAL_LOG_TRACE(__VA_ARGS__)
#else
#  define SX126X_RADIO_HAL_LOG_ERROR(...)           ((void)0u)
#  define SX126X_RADIO_HAL_LOG_WARNING(...)         ((void)0u)
#  define SX126X_RADIO_HAL_LOG_INFO(...)            ((void)0u)
#  define SX126X_RADIO_HAL_LOG_DEBUG(...)           ((void)0u)
#  define SX126X_RADIO_HAL_LOG_TRACE(...)           ((void)0u)
#endif

#define SX126X_RADIO_HAL_RESET_HOLD_TIME_US         (100u)     /*!< Time is microseconds to hold Reset pin low - should be at least as specified in the datasheet (100us) */
#define SX126X_RADIO_HAL_BOOTUP_TIME_MS             (4u)       /*!< SX126x startup time (in milliseconds) after reset - SX126x is inoperable for this period of time */

#define SX126X_WAIT_ON_BUSY_TIMEOUT_US              (20000u)   /*!< Wait timeout for Busy pin to go low */
#define SX126X_WAIT_ON_BUSY_PROBE_PERIOD_US         (5u)       /*!< Probe period to check if Busy pin state has changed */

#define SEMTECH_MAX_WAIT_ON_BUSY_NS                 (20000000UL)

#define SX126X_RADIO_SPI_BUFFER_TX_PARTITION_OFFSET (sizeof(uint32_t))      /*!< The absolute minimum amount of bytes the Tx partition shall be ahead of Rx partition in the SPI buffer */
static_assert((SX126X_RADIO_SPI_BUFFER_TX_PARTITION_OFFSET % sizeof(uint32_t)) == 0u); /* Tx partition should be aligned to 4-byte boundary to avoid issues with DMA access to it */

#define SX126X_CMD_CLR_IRQ_STATUS                   (0x02u)
#define SX126X_CMD_SET_DIO_IRQ_PARAMS               (0x08u)
#define SX126X_CMD_SET_SLEEP                        (0x84u)
#define SX126X_CMD_WRITE_REGISTER                   (0x0Du)
#define SX126X_CMD_READ_REGISTER                    (0x1Du)
#define SX126X_CMD_WRITE_BUFFER                     (0x0Eu)
#define SX126X_CMD_READ_BUFFER                      (0x1Eu)
#define SX126X_CLR_DEVICE_ERRORS                    (0x07u)

/* Private constants ---------------------------------------------------------*/

/**
 * @brief The list of commands for which the wait on Busy signal release shall be skipped after command execution
 */
static const uint8_t no_wait_on_busy_cmds[] = {
    SX126X_CMD_CLR_IRQ_STATUS,
    SX126X_CMD_SET_DIO_IRQ_PARAMS,
    SX126X_CMD_SET_SLEEP,
    SX126X_CMD_WRITE_REGISTER,
    SX126X_CMD_READ_REGISTER,
    SX126X_CMD_WRITE_BUFFER,
    SX126X_CMD_READ_BUFFER,
    SX126X_CLR_DEVICE_ERRORS,
};

/* Imported function prototypes ----------------------------------------------*/

#if SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION
void smtc_modem_hal_radio_irq(void);
#endif /* SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION */

/* Private function prototypes -----------------------------------------------*/

static        sx126x_hal_status_t sx126x_wait_on_busy(const halo_drv_semtech_ctx_t * const drv_ctx);
static        sx126x_hal_status_t sx126x_wait_for_device_ready(const halo_drv_semtech_ctx_t * const drv_ctx);
static        void                _on_radio_irq_detected(uint32_t pin, void * callback_arg);
static inline sx126x_hal_status_t radio_bus_xfer(const halo_drv_semtech_ctx_t * const drv_ctx, const uint8_t * const cmd_buffer, const uint16_t cmd_buffer_size,
                                                 uint8_t * const buffer, const uint16_t size, const uint8_t read_offset);

/* Private function definitions ----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sx126x_hal_status_t sx126x_wait_on_busy(const halo_drv_semtech_ctx_t *const drv_ctx)
{
    sx126x_hal_status_t err;
    sid_error_t sid_err;
    uint32_t is_edge_detected = 0;
    uint32_t accumulated_wait_time = 0u;

    SID_PAL_ASSERT(drv_ctx != NULL);

    /* Poll Busy pin latch status */
    do
    {
        /* Retrieve latch status */
        sid_err = sid_pal_gpio_ext_ifc_check_latch_state(drv_ctx->config->gpio.radio_busy, SID_PAL_GPIO_IRQ_TRIGGER_FALLING, &is_edge_detected);

        /* If no errors during the Busy pin check and a falling edge on Busy pin was detected */
        if ((SID_ERROR_NONE == sid_err) && (is_edge_detected != FALSE))
        {
            err = SX126X_HAL_STATUS_OK;
            break;
        }

        sid_pal_delay_us(SX126X_WAIT_ON_BUSY_PROBE_PERIOD_US);
        accumulated_wait_time += SX126X_WAIT_ON_BUSY_PROBE_PERIOD_US;
    } while (accumulated_wait_time < SX126X_WAIT_ON_BUSY_TIMEOUT_US);

    /* Handle wait timeout */
    if (accumulated_wait_time >= SX126X_WAIT_ON_BUSY_TIMEOUT_US)
    {
        SID_PAL_LOG_ERROR("Timeout on waiting for the radio to clear Busy pin");
        err = SX126X_HAL_STATUS_ERROR;
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sx126x_hal_status_t sx126x_wait_for_device_ready(const halo_drv_semtech_ctx_t * const drv_ctx)
{
    sx126x_hal_status_t err;
    sid_error_t sid_err;
    uint8_t is_radio_busy = 0u;
    uint32_t accumulated_wait_time = 0u;

    SID_PAL_ASSERT(drv_ctx != NULL);

    /* Poll Busy pin status */
    do
    {
        sid_err = sid_pal_gpio_read(drv_ctx->config->gpio.radio_busy, &is_radio_busy);

        /* If no errors during the Busy pin check and Busy state is low the radio is ready */
        if ((SID_ERROR_NONE == sid_err) && (0u == is_radio_busy))
        {
            err = SX126X_HAL_STATUS_OK;
            break;
        }

        sid_pal_delay_us(SX126X_WAIT_ON_BUSY_PROBE_PERIOD_US);
        accumulated_wait_time += SX126X_WAIT_ON_BUSY_PROBE_PERIOD_US;
    } while (accumulated_wait_time < SX126X_WAIT_ON_BUSY_TIMEOUT_US);

    /* Handle wait timeout */
    if (accumulated_wait_time >= SX126X_WAIT_ON_BUSY_TIMEOUT_US)
    {
        SID_PAL_LOG_ERROR("Timeout on waiting for the radio to clear Busy pin");
        err = SX126X_HAL_STATUS_ERROR;
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static void _on_radio_irq_detected(uint32_t pin, void * callback_arg)
{
    struct sid_timespec event_ts;

    /* Store the timestamp as soon as possible */
    (void)sid_pal_uptime_now(&event_ts);
    __COMPILER_BARRIER();

    (void)pin;

    halo_drv_semtech_ctx_t * const drv_ctx = (halo_drv_semtech_ctx_t *)callback_arg;
    SID_PAL_ASSERT(drv_ctx != NULL);

    /* Immediately stop software timer for Tx/Rx/CS/CAD timeout - we don't care about the exact IRQ here*/
    (void)sid_pal_timer_cancel(&drv_ctx->radio_timeout_mon);
    __COMPILER_BARRIER();

    SID_PAL_ASSERT(drv_ctx->radio_rx_packet != NULL);

    SX126X_RADIO_HAL_LOG_DEBUG("_on_radio_irq_detected...");

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

#if SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION
    if (drv_ctx->lbm.bridge_state > RADIO_SX126X_LBM_BRIDGE_STATE_ACTIVATION_PENDING)
    {
        /* Route radio IRQ to the LoRa Basics Modem */
        smtc_modem_hal_radio_irq();
    }
    else
#endif /* SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION */
    {
        /* Disable radio IRQ line  to allow the MCU's IRQ handler to return - Sidewalk may process radio IRQ in a RTOS task context */
        sx126x_hal_status_t err = sx126x_hal_disarm_irq(drv_ctx);
        if (err != SX126X_HAL_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to disarm SX126x IRQ line. error %u", (uint32_t)err);
        }

        /* Route IRQ to Sidewalk stack */
        drv_ctx->irq_handler();
    }
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline sx126x_hal_status_t radio_bus_xfer(const halo_drv_semtech_ctx_t * const drv_ctx, const uint8_t * const cmd_buffer, const uint16_t cmd_buffer_size,
                                                                           uint8_t * const buffer, const uint16_t size, const uint8_t read_offset)
{
    sx126x_hal_status_t err;
    sid_error_t sid_err;

    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->config != NULL);
    SID_PAL_ASSERT(drv_ctx->config->internal_buffer.p != NULL);
    SID_PAL_ASSERT(drv_ctx->bus_iface != NULL);

    do
    {
        const uint8_t * tx_buf;
        uint8_t *       rx_buf;
        uint32_t        transfer_size;
        uint32_t        required_buffer_size;

        /* Validate inputs */
        if ((NULL == drv_ctx->config->internal_buffer.p) || (NULL == cmd_buffer) || (0u == cmd_buffer_size))
        {
            err = SX126X_HAL_STATUS_ERROR;
            break;
        }

        /**
         * General concept of splitting the SPI buffer (drv_ctx->config->internal_buffer.p) into Tx and Rx partitions:
         * - Rx buffer always starts at drv_ctx->config->internal_buffer.p[0]
         * - Tx Buffer starts at `command_length` offset from the start of the Rx partition (rounded up to 4 bytes boundary so that DMA won't struggle with unaligned access)
         * - During the SPI transaction the Rx partition overwrites the Tx partition with the received data. Since Tx partition is placed slightly ahead of the Rx partition
         *   and SPI is synchronous, this does not cause data corruption
         * |Rx Partition______________________________________________________________|
         * |------Offset------|Tx Partition___________________________________________|
         * |____________________drv_ctx->config->internal_buffer.p____________________|
         * ^                  ^
         * Rx buffer start    Tx buffer start
         */

        if (drv_ctx->config->internal_buffer.size < (size + cmd_buffer_size)) {
            return RADIO_ERROR_NOMEM;
        }

        if (NULL == buffer)
        {
            /* This is a command-only transfer - use cmd_buffer directly */
            transfer_size = cmd_buffer_size;
        }
        else
        {
            /* This is command + data transfer - build Tx buffer */
            transfer_size = cmd_buffer_size + size;
        }

        /* Ensure there's no buffer overflow for cmd send out */
        required_buffer_size = SX126X_RADIO_SPI_BUFFER_TX_PARTITION_OFFSET + transfer_size /* space for Tx partition */;
        if (drv_ctx->config->internal_buffer.size < required_buffer_size)
        {
            SID_PAL_LOG_ERROR("Insufficient radio SPI processing buffer size, required bytes: %u, available: %u", required_buffer_size, drv_ctx->config->internal_buffer.size);
            err = SX126X_HAL_STATUS_ERROR;
            break;
        }

        /* Build command Tx buffer */
        tx_buf = &drv_ctx->config->internal_buffer.p[SX126X_RADIO_SPI_BUFFER_TX_PARTITION_OFFSET];

        if ((buffer != NULL) && (size > 0u))
        {
            /* This is command with data - compile command and data into single buffer */
            SID_STM32_UTIL_fast_memcpy((uint8_t *)tx_buf                  , cmd_buffer, cmd_buffer_size);
            SID_STM32_UTIL_fast_memcpy((uint8_t *)&tx_buf[cmd_buffer_size], buffer    , size);
        }
        else
        {
            /* This is a command-only transfer, but we still have to copy command into the Tx buffer to ensure data alignment */
            SID_STM32_UTIL_fast_memcpy((uint8_t *)tx_buf, cmd_buffer, cmd_buffer_size);
        }

        /* Setup read buffer for command transfer */
        rx_buf = drv_ctx->config->internal_buffer.p;

        /* Ensure Busy pin latch is ready for edge detection */
        sid_err = sid_pal_gpio_ext_ifc_clear_latch_state(drv_ctx->config->gpio.radio_busy, SID_PAL_GPIO_IRQ_TRIGGER_FALLING);
        if (sid_err != SID_ERROR_NONE)
        {
            err = SX126X_HAL_STATUS_ERROR;
            break;
        }

        /* Send the command */
        sid_err = drv_ctx->bus_iface->xfer(drv_ctx->bus_iface, &drv_ctx->config->bus_selector, (void *)tx_buf, rx_buf, transfer_size);
        if (sid_err != SID_ERROR_NONE)
        {
            err = SX126X_HAL_STATUS_ERROR;
            break;
        }

        if ((read_offset != 0u) && (buffer != NULL))
        {
            SID_STM32_UTIL_fast_memcpy(buffer, &rx_buf[read_offset], (transfer_size - read_offset));
        }

        err = SX126X_HAL_STATUS_OK;
    } while(0);

    return err;
}

/* Global function definitions -----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sx126x_hal_status_t sx126x_hal_init_gpio(const void * const context)
{
    sx126x_hal_status_t err = RADIO_ERROR_GENERIC;
    sid_error_t sid_err;
    const halo_drv_semtech_ctx_t * const drv_ctx = (const halo_drv_semtech_ctx_t *)context;

    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->config != NULL);

    do
    {
        /* Validate inputs */
        if (HALO_GPIO_NOT_CONNECTED == drv_ctx->config->gpio.radio_busy)
        {
            SID_PAL_LOG_ERROR("SX126x Busy pin must be connected for Sidewalk driver to operate correctly");
            err = RADIO_ERROR_INVALID_PARAMS;
            break;
        }
        if (HALO_GPIO_NOT_CONNECTED == drv_ctx->config->gpio.radio_reset)
        {
            SID_PAL_LOG_ERROR("SX126x Reset pin must be connected for Sidewalk driver to operate correctly");
            err = RADIO_ERROR_INVALID_PARAMS;
            break;
        }
        if (HALO_GPIO_NOT_CONNECTED == drv_ctx->config->gpio.radio_irq)
        {
            SID_PAL_LOG_ERROR("SX126x IRQ pin must be connected for Sidewalk driver to operate correctly");
            err = RADIO_ERROR_INVALID_PARAMS;
            break;
        }

        /* Configure essential pins --------------------------------------------------*/

        /* Reset line */
        sid_err = sid_pal_gpio_write(drv_ctx->config->gpio.radio_reset, 0u); /* Ensure SX126x Reset is asserted before the GPIO direction is set to avoid glitches */
        if (sid_err != SID_ERROR_NONE)
        {
            SX126X_RADIO_HAL_LOG_ERROR("Failed to drive SX126x reset low");
            err = SX126X_HAL_STATUS_ERROR;
            break;
        }
        sid_err = sid_pal_gpio_output_mode(drv_ctx->config->gpio.radio_reset, SID_PAL_GPIO_OUTPUT_PUSH_PULL);
        if (sid_err != SID_ERROR_NONE)
        {
            SX126X_RADIO_HAL_LOG_ERROR("Failed to set SX126x reset GPIO as PP");
            err = SX126X_HAL_STATUS_ERROR;
            break;
        }
        sid_err = sid_pal_gpio_set_direction(drv_ctx->config->gpio.radio_reset, SID_PAL_GPIO_DIRECTION_OUTPUT);
        if (sid_err != SID_ERROR_NONE)
        {
            SX126X_RADIO_HAL_LOG_ERROR("Failed to configure SX126x reset GPIO");
            err = SX126X_HAL_STATUS_ERROR;
            break;
        }

        /* Busy signal */
        sid_err = sid_pal_gpio_pull_mode(drv_ctx->config->gpio.radio_busy, SID_PAL_GPIO_PULL_NONE);
        if (sid_err != SID_ERROR_NONE)
        {
            SX126X_RADIO_HAL_LOG_ERROR("Failed to configure SX126x IRQ Busy pull resistors");
            err = SX126X_HAL_STATUS_ERROR;
            break;
        }
        sid_err = sid_pal_gpio_input_mode(drv_ctx->config->gpio.radio_busy, SID_PAL_GPIO_INPUT_CONNECT);
        if (sid_err != SID_ERROR_NONE)
        {
            SX126X_RADIO_HAL_LOG_ERROR("Failed to configure SX126x Busy GPIO as input");
            err = SX126X_HAL_STATUS_ERROR;
            break;
        }
        sid_err = sid_pal_gpio_set_direction(drv_ctx->config->gpio.radio_busy, SID_PAL_GPIO_DIRECTION_INPUT);
        if (sid_err != SID_ERROR_NONE)
        {
            SX126X_RADIO_HAL_LOG_ERROR("Failed to configure SX126x Busy GPIO");
            err = SX126X_HAL_STATUS_ERROR;
            break;
        }
        /**
         * Configure MCU to detect falling edge on the Busy pin without generating an IRQ. This is required because SX126x
         * rises Busy signal after the SPI transfer ends. Simple polling may not work because pin state is ambiguous (e.g.
         * Busy is low AFTER operation has completed vs Busy is low BEFORE SPI command processing has started). To avoid
         * guesswork the MCU shall detect edge transitions instead of checking the static pin state */
        sid_err = sid_pal_gpio_ext_ifc_set_latch(drv_ctx->config->gpio.radio_busy, SID_PAL_GPIO_IRQ_TRIGGER_FALLING);
        if (sid_err != SID_ERROR_NONE)
        {
            err = SX126X_HAL_STATUS_ERROR;
            break;
        }
        sid_err = sid_pal_gpio_ext_ifc_clear_latch_state(drv_ctx->config->gpio.radio_busy, SID_PAL_GPIO_IRQ_TRIGGER_FALLING);
        if (sid_err != SID_ERROR_NONE)
        {
            err = SX126X_HAL_STATUS_ERROR;
            break;
        }

        /* IRQ line */
        sid_err = sid_pal_gpio_pull_mode(drv_ctx->config->gpio.radio_irq, SID_PAL_GPIO_PULL_DOWN);
        if (sid_err != SID_ERROR_NONE)
        {
            SX126X_RADIO_HAL_LOG_ERROR("Failed to configure SX126x IRQ GPIO pull-down");
            err = SX126X_HAL_STATUS_ERROR;
            break;
        }
        sid_err = sid_pal_gpio_input_mode(drv_ctx->config->gpio.radio_irq, SID_PAL_GPIO_INPUT_CONNECT);
        if (sid_err != SID_ERROR_NONE)
        {
            SX126X_RADIO_HAL_LOG_ERROR("Failed to configure SX126x IRQ GPIO as input");
            err = SX126X_HAL_STATUS_ERROR;
            break;
        }
        sid_err = sid_pal_gpio_set_direction(drv_ctx->config->gpio.radio_irq, SID_PAL_GPIO_DIRECTION_INPUT);
        if (sid_err != SID_ERROR_NONE)
        {
            SX126X_RADIO_HAL_LOG_ERROR("Failed to configure SX126x IRQ GPIO");
            err = SX126X_HAL_STATUS_ERROR;
            break;
        }
        /*----------------------------------------------------------------------------*/

        /* RF Switch/FEM control pins ------------------------------------------------*/
#if SX126X_RADIO_CFG_USE_FEM_PWR_CTRL
        /* RF Switch (FEM) Power control */
        if (HALO_GPIO_NOT_CONNECTED != drv_ctx->config->gpio.fem_power)
        {
            /* Ensure pin state before the GPIO direction is set to avoid glitches */
            uint8_t write_val = drv_ctx->config->gpio.fem_power_en_gpio_state != 0u ? 1u : 0u;
            sid_err = sid_pal_gpio_write(drv_ctx->config->gpio.fem_power, write_val); 
            if (sid_err != SID_ERROR_NONE)
            {
                SX126X_RADIO_HAL_LOG_ERROR("Failed to set SX126x FEM/RF Switch Power to Off");
                err = SX126X_HAL_STATUS_ERROR;
                break;
            }
            sid_err = sid_pal_gpio_output_mode(drv_ctx->config->gpio.fem_power, SID_PAL_GPIO_OUTPUT_PUSH_PULL);
            if (sid_err != SID_ERROR_NONE)
            {
                SX126X_RADIO_HAL_LOG_ERROR("Failed to set SX126x FEM/RF Switch Power GPIO as PP");
                err = SX126X_HAL_STATUS_ERROR;
                break;
            }
            sid_err = sid_pal_gpio_set_direction(drv_ctx->config->gpio.fem_power, SID_PAL_GPIO_DIRECTION_OUTPUT);
            if (sid_err != SID_ERROR_NONE)
            {
                SX126X_RADIO_HAL_LOG_ERROR("Failed to configure SX126x FEM/RF Switch Power GPIO");
                err = SX126X_HAL_STATUS_ERROR;
                break;
            }
        }
#endif /* SX126X_RADIO_CFG_USE_FEM_PWR_CTRL */

#if SX126X_RADIO_CFG_USE_TX_RX_CTRL
        /* RF Switch (FEM) direction selection (Tx/Rx) */
        if (HALO_GPIO_NOT_CONNECTED != drv_ctx->config->gpio.fem_tx_rx_mode)
        {
            /* Ensure pin state before the GPIO direction is set to avoid glitches */
            uint8_t write_val = drv_ctx->config->gpio.fem_tx_mode_sel_gpio_state != 0u ? 0u : 1u;
            sid_err = sid_pal_gpio_write(drv_ctx->config->gpio.fem_tx_rx_mode, write_val); 
            if (sid_err != SID_ERROR_NONE)
            {
                SX126X_RADIO_HAL_LOG_ERROR("Failed to set SX126x FEM/RF Switch mode to Rx");
                err = SX126X_HAL_STATUS_ERROR;
                break;
            }
            sid_err = sid_pal_gpio_output_mode(drv_ctx->config->gpio.fem_tx_rx_mode, SID_PAL_GPIO_OUTPUT_PUSH_PULL);
            if (sid_err != SID_ERROR_NONE)
            {
                SX126X_RADIO_HAL_LOG_ERROR("Failed to set SX126x FEM/RF Switch Mode GPIO as PP");
                err = SX126X_HAL_STATUS_ERROR;
                break;
            }
            sid_err = sid_pal_gpio_set_direction(drv_ctx->config->gpio.fem_tx_rx_mode, SID_PAL_GPIO_DIRECTION_OUTPUT);
            if (sid_err != SID_ERROR_NONE)
            {
                SX126X_RADIO_HAL_LOG_ERROR("Failed to configure SX126x FEM/RF Switch Mode GPIO");
                err = SX126X_HAL_STATUS_ERROR;
                break;
            }
        }
#endif /* SX126X_RADIO_CFG_USE_TX_RX_CTRL */

#if SX126X_RADIO_CFG_USE_EXTERNAL_PA && SX126X_RADIO_CFG_USE_FEM_BYPASS_CTRL
        /* FEM Tx bypass */
        if (HALO_GPIO_NOT_CONNECTED != drv_ctx->config->gpio.fem_bypass)
        {
            /* Ensure pin state before the GPIO direction is set to avoid glitches */
            uint8_t write_val = drv_ctx->config->gpio.fem_bypass_en_gpio_state != 0u ? 1u : 0u;
            sid_err = sid_pal_gpio_write(drv_ctx->config->gpio.fem_bypass, write_val);
            if (sid_err != SID_ERROR_NONE)
            {
                SX126X_RADIO_HAL_LOG_ERROR("Failed to drive SX126x FEM Tx Bypass to enabled state");
                err = SX126X_HAL_STATUS_ERROR;
                break;
            }
            sid_err = sid_pal_gpio_output_mode(drv_ctx->config->gpio.fem_bypass, SID_PAL_GPIO_OUTPUT_PUSH_PULL);
            if (sid_err != SID_ERROR_NONE)
            {
                SX126X_RADIO_HAL_LOG_ERROR("Failed to set SX126x FEM Tx Bypass GPIO as PP");
                err = SX126X_HAL_STATUS_ERROR;
                break;
            }
            sid_err = sid_pal_gpio_set_direction(drv_ctx->config->gpio.fem_bypass, SID_PAL_GPIO_DIRECTION_OUTPUT);
            if (sid_err != SID_ERROR_NONE)
            {
                SX126X_RADIO_HAL_LOG_ERROR("Failed to configure SX126x FEM Tx Bypass GPIO");
                err = SX126X_HAL_STATUS_ERROR;
                break;
            }
        }
#endif /* SX126X_RADIO_CFG_USE_EXTERNAL_PA && SX126X_RADIO_CFG_USE_FEM_BYPASS_CTRL */
        /*----------------------------------------------------------------------------*/

        /* Status LEDs ---------------------------------------------------------------*/
#if SX126X_RADIO_CFG_USE_STATUS_LED
        /* Tx LED */
        if (HALO_GPIO_NOT_CONNECTED != drv_ctx->config->gpio.tx_led)
        {
            /* Ensure pin state before the GPIO direction is set to avoid glitches */
            sx126x_radio_hal_tx_led_off(drv_ctx);

            sid_err = sid_pal_gpio_output_mode(drv_ctx->config->gpio.tx_led, SID_PAL_GPIO_OUTPUT_PUSH_PULL);
            if (sid_err != SID_ERROR_NONE)
            {
                SX126X_RADIO_HAL_LOG_ERROR("Failed to set SX126x Tx LED GPIO as PP");
                err = SX126X_HAL_STATUS_ERROR;
                break;
            }
            sid_err = sid_pal_gpio_set_direction(drv_ctx->config->gpio.tx_led, SID_PAL_GPIO_DIRECTION_OUTPUT);
            if (sid_err != SID_ERROR_NONE)
            {
                SX126X_RADIO_HAL_LOG_ERROR("Failed to configure SX126x Tx LED GPIO");
                err = SX126X_HAL_STATUS_ERROR;
                break;
            }
        }

        /* Rx LED */
        if ((HALO_GPIO_NOT_CONNECTED != drv_ctx->config->gpio.rx_led) && (drv_ctx->config->gpio.rx_led != drv_ctx->config->gpio.tx_led))
        {
            sx126x_radio_hal_rx_led_off(drv_ctx);

            sid_err = sid_pal_gpio_output_mode(drv_ctx->config->gpio.rx_led, SID_PAL_GPIO_OUTPUT_PUSH_PULL);
            if (sid_err != SID_ERROR_NONE)
            {
                SX126X_RADIO_HAL_LOG_ERROR("Failed to set SX126x Rx LED GPIO as PP");
                err = SX126X_HAL_STATUS_ERROR;
                break;
            }
            sid_err = sid_pal_gpio_set_direction(drv_ctx->config->gpio.rx_led, SID_PAL_GPIO_DIRECTION_OUTPUT);
            if (sid_err != SID_ERROR_NONE)
            {
                SX126X_RADIO_HAL_LOG_ERROR("Failed to configure SX126x Tx LED GPIO");
                err = SX126X_HAL_STATUS_ERROR;
                break;
            }
        }
#endif /* SX126X_RADIO_CFG_USE_STATUS_LED */
        /*----------------------------------------------------------------------------*/

        /* Done */
        err = SX126X_HAL_STATUS_OK;
    }
    while(0);

    return err;
}

/*----------------------------------------------------------------------------*/

sx126x_hal_status_t sx126x_hal_deinit_gpio(const void *ctx)
{
    sx126x_hal_status_t err = RADIO_ERROR_GENERIC;
    sid_error_t         sid_err;
    const halo_drv_semtech_ctx_t *drv_ctx = (const halo_drv_semtech_ctx_t *)ctx;

    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->config != NULL);

    do
    {
        /* Set all pins to Hi-Z */

        /* Essential pins ------------------------------------------------------------*/

        /* IRQ line */
        sid_err = sid_pal_gpio_pull_mode(drv_ctx->config->gpio.radio_irq, SID_PAL_GPIO_PULL_NONE);
        if (sid_err != SID_ERROR_NONE)
        {
            SX126X_RADIO_HAL_LOG_ERROR("Failed to disable SX126x IRQ GPIO pull-down");
            err = SX126X_HAL_STATUS_ERROR;
            break;
        }
        sid_err = sid_pal_gpio_input_mode(drv_ctx->config->gpio.radio_irq, SID_PAL_GPIO_INPUT_DISCONNECT);
        if (sid_err != SID_ERROR_NONE)
        {
            SX126X_RADIO_HAL_LOG_ERROR("Failed to configure SX126x IRQ GPIO as Hi-Z input");
            err = SX126X_HAL_STATUS_ERROR;
            break;
        }
        sid_err = sid_pal_gpio_set_direction(drv_ctx->config->gpio.radio_irq, SID_PAL_GPIO_DIRECTION_INPUT);
        if (sid_err != SID_ERROR_NONE)
        {
            SX126X_RADIO_HAL_LOG_ERROR("Failed to configure SX126x IRQ GPIO");
            err = SX126X_HAL_STATUS_ERROR;
            break;
        }

        /* Busy signal */
        sid_err = sid_pal_gpio_input_mode(drv_ctx->config->gpio.radio_busy, SID_PAL_GPIO_INPUT_DISCONNECT);
        if (sid_err != SID_ERROR_NONE)
        {
            SX126X_RADIO_HAL_LOG_ERROR("Failed to configure SX126x Busy GPIO as Hi-Z input");
            err = SX126X_HAL_STATUS_ERROR;
            break;
        }
        sid_err = sid_pal_gpio_set_direction(drv_ctx->config->gpio.radio_busy, SID_PAL_GPIO_DIRECTION_INPUT);
        if (sid_err != SID_ERROR_NONE)
        {
            SX126X_RADIO_HAL_LOG_ERROR("Failed to configure SX126x Busy GPIO");
            err = SX126X_HAL_STATUS_ERROR;
            break;
        }
        /* Cancel hardware edge transition detection */
        sid_err = sid_pal_gpio_ext_ifc_set_latch(drv_ctx->config->gpio.radio_busy, SID_PAL_GPIO_IRQ_TRIGGER_NONE);
        if (sid_err != SID_ERROR_NONE)
        {
            SX126X_RADIO_HAL_LOG_ERROR("Failed to release SX126x Busy GPIO latch");
            err = SX126X_HAL_STATUS_ERROR;
            break;
        }

        /* Reset line */
        sid_err = sid_pal_gpio_input_mode(drv_ctx->config->gpio.radio_reset, SID_PAL_GPIO_INPUT_DISCONNECT);
        if (sid_err != SID_ERROR_NONE)
        {
            SX126X_RADIO_HAL_LOG_ERROR("Failed to configure SX126x reset GPIO to Hi-Z");
            err = SX126X_HAL_STATUS_ERROR;
            break;
        }
        sid_err = sid_pal_gpio_set_direction(drv_ctx->config->gpio.radio_reset, SID_PAL_GPIO_DIRECTION_INPUT);
        if (sid_err != SID_ERROR_NONE)
        {
            SX126X_RADIO_HAL_LOG_ERROR("Failed to set SX126x reset GPIO as Hi-Z input");
            err = SX126X_HAL_STATUS_ERROR;
            break;
        }
        /*----------------------------------------------------------------------------*/

        /* RF Switch/FEM control pins ------------------------------------------------*/
#if SX126X_RADIO_CFG_USE_FEM_PWR_CTRL
        /* RF Switch (FEM) Power control */
        if (HALO_GPIO_NOT_CONNECTED != drv_ctx->config->gpio.fem_power)
        {
            sid_err = sid_pal_gpio_input_mode(drv_ctx->config->gpio.fem_power, SID_PAL_GPIO_INPUT_DISCONNECT);
            if (sid_err != SID_ERROR_NONE)
            {
                SX126X_RADIO_HAL_LOG_ERROR("Failed to set SX126x FEM/RF Switch Power GPIO as Hi-Z input");
                err = SX126X_HAL_STATUS_ERROR;
                break;
            }
            sid_err = sid_pal_gpio_set_direction(drv_ctx->config->gpio.fem_power, SID_PAL_GPIO_DIRECTION_INPUT);
            if (sid_err != SID_ERROR_NONE)
            {
                SX126X_RADIO_HAL_LOG_ERROR("Failed to configure SX126x FEM/RF Switch Power GPIO");
                err = SX126X_HAL_STATUS_ERROR;
                break;
            }
        }
#endif /* SX126X_RADIO_CFG_USE_FEM_PWR_CTRL */

#if SX126X_RADIO_CFG_USE_TX_RX_CTRL
        /* RF Switch (FEM) direction selection (Tx/Rx) */
        if (HALO_GPIO_NOT_CONNECTED != drv_ctx->config->gpio.fem_tx_rx_mode)
        {
            sid_err = sid_pal_gpio_input_mode(drv_ctx->config->gpio.fem_tx_rx_mode, SID_PAL_GPIO_INPUT_DISCONNECT);
            if (sid_err != SID_ERROR_NONE)
            {
                SX126X_RADIO_HAL_LOG_ERROR("Failed to set SX126x FEM/RF Switch Mode GPIO as Hi-Z input");
                err = SX126X_HAL_STATUS_ERROR;
                break;
            }
            sid_err = sid_pal_gpio_set_direction(drv_ctx->config->gpio.fem_tx_rx_mode, SID_PAL_GPIO_DIRECTION_INPUT);
            if (sid_err != SID_ERROR_NONE)
            {
                SX126X_RADIO_HAL_LOG_ERROR("Failed to configure SX126x FEM/RF Switch Mode GPIO");
                err = SX126X_HAL_STATUS_ERROR;
                break;
            }
        }
#endif /* SX126X_RADIO_CFG_USE_TX_RX_CTRL */

#if SX126X_RADIO_CFG_USE_EXTERNAL_PA && SX126X_RADIO_CFG_USE_FEM_BYPASS_CTRL
        /* FEM Tx bypass */
        if (HALO_GPIO_NOT_CONNECTED != drv_ctx->config->gpio.fem_bypass)
        {
            sid_err = sid_pal_gpio_input_mode(drv_ctx->config->gpio.fem_bypass, SID_PAL_GPIO_INPUT_DISCONNECT);
            if (sid_err != SID_ERROR_NONE)
            {
                SX126X_RADIO_HAL_LOG_ERROR("Failed to set SX126x FEM Tx Bypass GPIO as Hi-Z input");
                err = SX126X_HAL_STATUS_ERROR;
                break;
            }
            sid_err = sid_pal_gpio_set_direction(drv_ctx->config->gpio.fem_bypass, SID_PAL_GPIO_DIRECTION_INPUT);
            if (sid_err != SID_ERROR_NONE)
            {
                SX126X_RADIO_HAL_LOG_ERROR("Failed to configure SX126x FEM Tx Bypass GPIO");
                err = SX126X_HAL_STATUS_ERROR;
                break;
            }
        }
#endif /* SX126X_RADIO_CFG_USE_EXTERNAL_PA && SX126X_RADIO_CFG_USE_FEM_BYPASS_CTRL */
        /*----------------------------------------------------------------------------*/

        /* Status LEDs ---------------------------------------------------------------*/
#if SX126X_RADIO_CFG_USE_STATUS_LED
        /* Tx LED */
        if (HALO_GPIO_NOT_CONNECTED != drv_ctx->config->gpio.tx_led)
        {
            sid_err = sid_pal_gpio_input_mode(drv_ctx->config->gpio.tx_led, SID_PAL_GPIO_INPUT_DISCONNECT);
            if (sid_err != SID_ERROR_NONE)
            {
                SX126X_RADIO_HAL_LOG_ERROR("Failed to configure SX126x Tx LED GPIO as Hi-Z input");
                err = SX126X_HAL_STATUS_ERROR;
                break;
            }
            sid_err = sid_pal_gpio_set_direction(drv_ctx->config->gpio.tx_led, SID_PAL_GPIO_DIRECTION_INPUT);
            if (sid_err != SID_ERROR_NONE)
            {
                SX126X_RADIO_HAL_LOG_ERROR("Failed to configure SX126x Tx LED GPIO");
                err = SX126X_HAL_STATUS_ERROR;
                break;
            }
        }

        /* Rx LED */
        if ((HALO_GPIO_NOT_CONNECTED != drv_ctx->config->gpio.rx_led) && (drv_ctx->config->gpio.rx_led != drv_ctx->config->gpio.tx_led))
        {
            sid_err = sid_pal_gpio_input_mode(drv_ctx->config->gpio.rx_led, SID_PAL_GPIO_INPUT_DISCONNECT);
            if (sid_err != SID_ERROR_NONE)
            {
                SX126X_RADIO_HAL_LOG_ERROR("Failed to configure SX126x Rx LED GPIO as Hi-Z input");
                err = SX126X_HAL_STATUS_ERROR;
                break;
            }
            sid_err = sid_pal_gpio_set_direction(drv_ctx->config->gpio.rx_led, SID_PAL_GPIO_DIRECTION_INPUT);
            if (sid_err != SID_ERROR_NONE)
            {
                SX126X_RADIO_HAL_LOG_ERROR("Failed to configure SX126x Tx LED GPIO");
                err = SX126X_HAL_STATUS_ERROR;
                break;
            }
        }
#endif /* SX126X_RADIO_CFG_USE_STATUS_LED */
        /*----------------------------------------------------------------------------*/

        /* Done */
        err = RADIO_ERROR_NONE;
    }
    while(0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sx126x_hal_status_t sx126x_hal_reset(const void * const context)
{
    int32_t err;
    sid_error_t sid_err;
    const halo_drv_semtech_ctx_t * const drv_ctx = (const halo_drv_semtech_ctx_t *)context;

    do
    {
        /* Validate inputs */
        if (NULL == drv_ctx)
        {
            err = RADIO_ERROR_INVALID_PARAMS;
            break;
        }

        /* Assert radio reset */
        sid_err = sid_pal_gpio_write(drv_ctx->config->gpio.radio_reset, 0u);
        if (sid_err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to assert radio reset. Error %d", (int32_t)sid_err);
            err = SX126X_HAL_STATUS_ERROR;
            break;
        }

        /* Keep the reset hold time, other wise reset may not be detected by the transceiver */
        sid_pal_delay_us(SX126X_RADIO_HAL_RESET_HOLD_TIME_US);

        /* Prepare Busy pin latch */
        sid_err = sid_pal_gpio_ext_ifc_clear_latch_state(drv_ctx->config->gpio.radio_busy, SID_PAL_GPIO_IRQ_TRIGGER_FALLING);
        if(SID_ERROR_NONE != sid_err)
        {
            err = SX126X_HAL_STATUS_ERROR;
            break;
        }

        /* Release reset line */
        sid_err = sid_pal_gpio_write(drv_ctx->config->gpio.radio_reset, 1u);
        if (sid_err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to release radio reset. Error %d", (int32_t)sid_err);
            err = SX126X_HAL_STATUS_ERROR;
            break;
        }

        /* Wait for reset completion */
#ifdef SID_PAL_ENABLE_SCHEDULER_DELAY
        if (SID_STM32_UTIL_IS_IRQ() == FALSE)
        {
            sid_pal_scheduler_delay_ms(SX126X_RADIO_HAL_BOOTUP_TIME_MS);
        }
        else
        {
            sid_pal_delay_us(SX126X_RADIO_HAL_BOOTUP_TIME_MS * 1000u);
        }
#else
        sid_pal_delay_us(SX126X_RADIO_HAL_BOOTUP_TIME_MS * 1000u);
#endif /* SID_PAL_ENABLE_SCHEDULER_DELAY */

        /* Wait for for SX126x to clear Busy signal */
        err = sx126x_wait_on_busy(drv_ctx);
        if (err != SX126X_HAL_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Timeout on waiting SX126x boot-up. Radio is not responding");
            break;
        }

        /* Done */
        err = SX126X_HAL_STATUS_OK;
    } while(0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sx126x_hal_status_t sx126x_hal_wakeup(const void * const context)
{
    const halo_drv_semtech_ctx_t * const drv_ctx = (halo_drv_semtech_ctx_t *)context;
    sid_error_t sid_err;
    sx126x_hal_status_t err = SX126X_HAL_STATUS_ERROR;

    do
    {
        uint8_t is_radio_busy;
        sid_err = sid_pal_gpio_read(drv_ctx->config->gpio.radio_busy, &is_radio_busy);
        if ( sid_err != SID_ERROR_NONE )
        {
            err = SX126X_HAL_STATUS_ERROR;
            break;
        }

        if (0u == is_radio_busy)
        {
            /* No need to wake up the radio, it's already active */
            err = SX126X_HAL_STATUS_OK;
            break;
        }

        /* Prepare GPIO pin latch */
        sid_err = sid_pal_gpio_ext_ifc_clear_latch_state(drv_ctx->config->gpio.radio_busy, SID_PAL_GPIO_IRQ_TRIGGER_FALLING);
        if(SID_ERROR_NONE != sid_err)
        {
            err = SX126X_HAL_STATUS_ERROR;
            break;
        }

        /* Drive NSS low to wake up the radio */
        sid_err = sid_pal_serial_bus_ext_ifc_activate_nss(drv_ctx->bus_iface, &drv_ctx->config->bus_selector);
        if(SID_ERROR_NONE != sid_err) {
            err = SX126X_HAL_STATUS_ERROR;
            break;
        }

        /* Wait for chip to be ready */
        err = sx126x_wait_on_busy(drv_ctx);
        if (err != SX126X_HAL_STATUS_OK)
        {
            break;
        }

        /* Reset GPIO latch state */
        sid_err = sid_pal_gpio_ext_ifc_clear_latch_state(drv_ctx->config->gpio.radio_busy, SID_PAL_GPIO_IRQ_TRIGGER_FALLING);
        if(SID_ERROR_NONE != sid_err) {
            err = SX126X_HAL_STATUS_ERROR;
            break;
        }

        /* Release NSS line */
        sid_err = sid_pal_serial_bus_ext_ifc_deactivate_nss(drv_ctx->bus_iface, &drv_ctx->config->bus_selector);
        if(SID_ERROR_NONE != sid_err)
        {
            err =  SX126X_HAL_STATUS_ERROR;
            break;
        }

        err = SX126X_HAL_STATUS_OK;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sx126x_hal_status_t sx126x_hal_wait_busy_indicated(const void * const context)
{
    sx126x_hal_status_t err;
    sid_error_t sid_err;
    const halo_drv_semtech_ctx_t * const drv_ctx = (halo_drv_semtech_ctx_t *)context;
    uint8_t is_radio_busy = 0u;
    uint32_t accumulated_wait_time = 0u;

    SID_PAL_ASSERT(drv_ctx != NULL);

    /* Poll Busy pin status */
    do
    {
        sid_err = sid_pal_gpio_read(drv_ctx->config->gpio.radio_busy, &is_radio_busy);

        /* If no errors during the Busy pin check and Busy state is low the radio is ready */
        if ((SID_ERROR_NONE == sid_err) && (is_radio_busy != 0u))
        {
            err = SX126X_HAL_STATUS_OK;
            break;
        }

        sid_pal_delay_us(SX126X_WAIT_ON_BUSY_PROBE_PERIOD_US);
        accumulated_wait_time += SX126X_WAIT_ON_BUSY_PROBE_PERIOD_US;
    } while (accumulated_wait_time < SX126X_WAIT_ON_BUSY_TIMEOUT_US);

    /* Handle wait timeout */
    if (accumulated_wait_time >= SX126X_WAIT_ON_BUSY_TIMEOUT_US)
    {
        SID_PAL_LOG_ERROR("Timeout on waiting for the radio to set Busy pin");
        err = SX126X_HAL_STATUS_ERROR;
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sx126x_hal_status_t sx126x_hal_read(const void * const context, const uint8_t * const command, const uint16_t command_length, uint8_t * const data, const uint16_t data_length)
{
    sx126x_hal_status_t err = SX126X_HAL_STATUS_ERROR;
    const halo_drv_semtech_ctx_t * const drv_ctx = (halo_drv_semtech_ctx_t *)context;

    SID_PAL_ASSERT(drv_ctx != NULL);

    do
    {
        /* Validate inputs */
        if ((NULL == command) || (0u == command_length) || (NULL == data) || (0u == data_length))
        {
            err = SX126X_HAL_STATUS_ERROR;
            break;
        }

        /* Ensure SX126x can accept commands */
        err = sx126x_wait_for_device_ready(drv_ctx);
        if (err != SX126X_HAL_STATUS_OK)
        {
            break;
        }

        /* Run SPI communication */
        err = radio_bus_xfer(drv_ctx, command, command_length, data, data_length, command_length);
        if (err != SX126X_HAL_STATUS_OK)
        {
            break;
        }

        err = SX126X_HAL_STATUS_OK;
    } while(0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sx126x_hal_status_t sx126x_hal_write(const void * const context, const uint8_t * const command, const uint16_t command_length, const uint8_t * const data, const uint16_t data_length)
{
    sx126x_hal_status_t err = SX126X_HAL_STATUS_ERROR;
    sx126x_status_t sys_err;
    halo_drv_semtech_ctx_t * const drv_ctx = (halo_drv_semtech_ctx_t *)context;

    SID_PAL_ASSERT(drv_ctx != NULL);

    do
    {
        /* Validate inputs - data can be null */
        if ((NULL == command) || (0u == command_length))
        {
            err = SX126X_HAL_STATUS_ERROR;
            break;
        }

        /* Check if radio wakeup is required */
        if ((SID_PAL_RADIO_SLEEP == drv_ctx->radio_state) || (SID_PAL_RADIO_UNKNOWN == drv_ctx->radio_state) || (SID_PAL_RADIO_RX_DC == drv_ctx->radio_state))
        {
            sys_err = sx126x_wakeup(drv_ctx);
            if (sys_err != SX126X_STATUS_OK)
            {
                drv_ctx->radio_state = SID_PAL_RADIO_UNKNOWN;
                err = SX126X_HAL_STATUS_ERROR;
                break;
            }
        }

        /* Ensure SX126x can accept commands */
        err = sx126x_wait_for_device_ready(drv_ctx);
        if (err != SX126X_HAL_STATUS_OK)
        {
            break;
        }

        /* Run SPI communication */
        err = radio_bus_xfer(drv_ctx, command, command_length, (uint8_t *)data, data_length, 0u);
        if (err != SX126X_HAL_STATUS_OK)
        {
            break;
        }

        if (memchr(no_wait_on_busy_cmds, command[0], sizeof(no_wait_on_busy_cmds)) == NULL)
        {
            err = sx126x_wait_on_busy(drv_ctx);
            if (err != SX126X_HAL_STATUS_OK)
            {
                SID_PAL_LOG_ERROR("Radio stuck with command execution. Command 0x%02X", command[0]);
                break;
            }
        }

        if (SX126X_CMD_SET_SLEEP == command[0])
        {
            /* Radio is in sleep, nothing more can be transferred till wakeup */
            drv_ctx->radio_state = SID_PAL_RADIO_SLEEP;
        }

        err = SX126X_HAL_STATUS_OK;
    } while(0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sx126x_hal_status_t sx126x_hal_disarm_irq(const void * const context)
{
    sx126x_hal_status_t err = SX126X_HAL_STATUS_ERROR;
    sid_error_t sid_err;
    const halo_drv_semtech_ctx_t * const drv_ctx = (const halo_drv_semtech_ctx_t *)context;

    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->config != NULL);

    do
    {
        /* Disable IRQ line */
        sid_err = sid_pal_gpio_irq_disable(drv_ctx->config->gpio.radio_irq);
        if (sid_err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to disable SX126x IRQ in NVIC. Error %d", (int32_t)sid_err);
            err = SX126X_HAL_STATUS_ERROR;
            break;
        }

        /* Disconnect GPIO pin from IRQ line */
        sid_err = sid_pal_gpio_set_irq(drv_ctx->config->gpio.radio_irq, SID_PAL_GPIO_IRQ_TRIGGER_NONE, NULL, NULL);
        if (sid_err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to deinit SX126x IRQ GPIO. Error %d", (int32_t)sid_err);
            err = SX126X_HAL_STATUS_ERROR;
            break;
        }

        /* Everything is fine */
        err = SX126X_HAL_STATUS_OK;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sx126x_hal_status_t sx126x_hal_arm_irq(void * const context)
{
    sx126x_hal_status_t err = SX126X_HAL_STATUS_ERROR;
    sid_error_t sid_err;
    halo_drv_semtech_ctx_t * const drv_ctx = (halo_drv_semtech_ctx_t *)context;

    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->config != NULL);

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
        sid_err = sid_pal_gpio_set_irq(drv_ctx->config->gpio.radio_irq, SID_PAL_GPIO_IRQ_TRIGGER_HIGH, _on_radio_irq_detected, (void*)drv_ctx);
        if (sid_err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to enable SX126x IRQ in NVIC, error %d", (int32_t)sid_err);
            err = SX126X_HAL_STATUS_ERROR;
            break;
        }

#if (__ARM_ARCH_6M__ == 0)
        /* Set elevated priority to SX126x IRQ line so we can capture radio IRQ timestamp with more precision */
        sid_err = sid_pal_gpio_ext_ifc_set_irq_priority(drv_ctx->config->gpio.radio_irq, drv_ctx->config->gpio.radio_irq_prio_high, 0u);
#else
        /* Set unified priority to SX126x IRQ line since dynamic IRQ priority changing is not supported */
        sid_err = sid_pal_gpio_ext_ifc_set_irq_priority(drv_ctx->config->gpio.radio_irq, drv_ctx->config->gpio.radio_irq_prio, 0u);
#endif /* (__ARM_ARCH_6M__ == 0) */
        if (sid_err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to set SX126x IRQ priority, error %d", (int32_t)sid_err);
            err = SX126X_HAL_STATUS_ERROR;
            break;
        }

        err = SX126X_HAL_STATUS_OK;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sx126x_hal_status_t sx126x_hal_wait_readiness(const void * const context)
{
    const halo_drv_semtech_ctx_t * const drv_ctx = (const halo_drv_semtech_ctx_t *)context;
    return sx126x_wait_on_busy(drv_ctx);
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int8_t sx126x_hal_get_adjusted_rssi(const void * const context, const int8_t raw_rssi)
{
    const halo_drv_semtech_ctx_t * const drv_ctx = (const halo_drv_semtech_ctx_t *)context;
    int32_t adjusted_rssi;
    int32_t correction = 0;

#if SX126X_RADIO_CFG_USE_EXTERNAL_PA
    /* External LNA is used - compensate for its gain */
    correction += drv_ctx->config->pa_config.rx_gain_dbi;
#else
    /* No external LNA is used, account for RF switch losses*/
    correction -= drv_ctx->config->pa_config.rf_sw_insertion_loss;
#endif /* SX126X_RADIO_CFG_USE_EXTERNAL_PA */
    /* Account for antenna gain */
    correction += drv_ctx->regional_radio_param->ant_dbi;

    adjusted_rssi = ((int32_t)raw_rssi * 100) - correction;

    /* Convert to dB with rounding */
    if (adjusted_rssi >= 0)
    {
        adjusted_rssi = (adjusted_rssi + 50) / 100;

        /* Saturate to the maximum return value */
        if (adjusted_rssi > INT8_MAX)
        {
            adjusted_rssi = INT8_MAX;
        }
    }
    else
    {
        adjusted_rssi = (adjusted_rssi - 50) / 100;

        /* Saturate to the minimum return value */
        if (adjusted_rssi < INT8_MIN)
        {
            adjusted_rssi = INT8_MIN;
        }
    }

    return (int8_t)adjusted_rssi;
}

/*----------------------------------------------------------------------------*/

#if SX126X_RADIO_CFG_USE_STATUS_LED
SID_STM32_SPEED_OPTIMIZED void sx126x_radio_hal_tx_led_on(const void * const context)
{
    const halo_drv_semtech_ctx_t * const drv_ctx = (const halo_drv_semtech_ctx_t *)context;

    if (HALO_GPIO_NOT_CONNECTED != drv_ctx->config->gpio.tx_led)
    {
        uint8_t led_on_pin_state = drv_ctx->config->gpio.tx_led_on_gpio_state != 0u ? 1u : 0u;
        (void)sid_pal_gpio_write(drv_ctx->config->gpio.tx_led, led_on_pin_state);
    }
}
#endif /* SX126X_RADIO_CFG_USE_STATUS_LED */

/*----------------------------------------------------------------------------*/

#if SX126X_RADIO_CFG_USE_STATUS_LED
SID_STM32_SPEED_OPTIMIZED void sx126x_radio_hal_tx_led_off(const void * const context)
{
    const halo_drv_semtech_ctx_t * const drv_ctx = (const halo_drv_semtech_ctx_t *)context;

    if (HALO_GPIO_NOT_CONNECTED != drv_ctx->config->gpio.tx_led)
    {
        uint8_t led_off_pin_state = drv_ctx->config->gpio.tx_led_on_gpio_state != 0u ? 0u : 1u;
        (void)sid_pal_gpio_write(drv_ctx->config->gpio.tx_led, led_off_pin_state);
    }
}
#endif /* SX126X_RADIO_CFG_USE_STATUS_LED */

/*----------------------------------------------------------------------------*/

#if SX126X_RADIO_CFG_USE_STATUS_LED
SID_STM32_SPEED_OPTIMIZED void sx126x_radio_hal_rx_led_on(const void * const context)
{
        const halo_drv_semtech_ctx_t * const drv_ctx = (const halo_drv_semtech_ctx_t *)context;

    if (HALO_GPIO_NOT_CONNECTED != drv_ctx->config->gpio.rx_led)
    {
        uint8_t led_on_pin_state = drv_ctx->config->gpio.rx_led_on_gpio_state != 0u ? 1u : 0u;
        (void)sid_pal_gpio_write(drv_ctx->config->gpio.rx_led, led_on_pin_state);
    }
}
#endif /* SX126X_RADIO_CFG_USE_STATUS_LED */

/*----------------------------------------------------------------------------*/

#if SX126X_RADIO_CFG_USE_STATUS_LED
SID_STM32_SPEED_OPTIMIZED void sx126x_radio_hal_rx_led_off(const void * const context)
{
    const halo_drv_semtech_ctx_t * const drv_ctx = (const halo_drv_semtech_ctx_t *)context;

    if (HALO_GPIO_NOT_CONNECTED != drv_ctx->config->gpio.rx_led)
    {
        uint8_t led_off_pin_state = drv_ctx->config->gpio.rx_led_on_gpio_state != 0u ? 0u : 1u;
        (void)sid_pal_gpio_write(drv_ctx->config->gpio.rx_led, led_off_pin_state);
    }
}
#endif /* SX126X_RADIO_CFG_USE_STATUS_LED */
