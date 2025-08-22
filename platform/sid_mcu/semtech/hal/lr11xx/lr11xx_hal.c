/*
 * Copyright (c) 2019-2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * This file defines interface used by Semtech driver to perform platform specific
 * operations
 */
/**
  ******************************************************************************
  * @file    lr11xx_hal.c
  * @brief   Semtech LR11xx radio driver for Sidewalk running on STM32 platform
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

#include "halo_lr11xx_radio.h"
#include "lr11xx_radio.h"
#include "lr11xx_hal.h"

/* Sidewalk interfaces */
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

#ifndef LR11XX_RADIO_HAL_EXTRA_LOGGING
/* Set LR11XX_RADIO_HAL_EXTRA_LOGGING to 1 to enable extended logs */
#  define LR11XX_RADIO_HAL_EXTRA_LOGGING                  (0)
#endif

#if LR11XX_RADIO_HAL_EXTRA_LOGGING
#  define LR11XX_RADIO_HAL_LOG_ERROR(...)                 SID_PAL_LOG_ERROR(__VA_ARGS__)
#  define LR11XX_RADIO_HAL_LOG_WARNING(...)               SID_PAL_LOG_WARNING(__VA_ARGS__)
#  define LR11XX_RADIO_HAL_LOG_INFO(...)                  SID_PAL_LOG_INFO(__VA_ARGS__)
#  define LR11XX_RADIO_HAL_LOG_DEBUG(...)                 SID_PAL_LOG_DEBUG(__VA_ARGS__)
#  define LR11XX_RADIO_HAL_LOG_TRACE(...)                 SID_PAL_LOG_TRACE(__VA_ARGS__)
#else
#  define LR11XX_RADIO_HAL_LOG_ERROR(...)                 ((void)0u)
#  define LR11XX_RADIO_HAL_LOG_WARNING(...)               ((void)0u)
#  define LR11XX_RADIO_HAL_LOG_INFO(...)                  ((void)0u)
#  define LR11XX_RADIO_HAL_LOG_DEBUG(...)                 ((void)0u)
#  define LR11XX_RADIO_HAL_LOG_TRACE(...)                 ((void)0u)
#endif

#define LR11XX_RADIO_HAL_RESET_HOLD_TIME_US               (100u)     /*!< Time is microseconds to hold Reset pin low - should be at least as specified in the datasheet (100us) */
#define LR11XX_RADIO_HAL_BOOTUP_TIMEOUT_MS                (350u)     /*!< Maximum acceptable LR11xx boot time (in milliseconds) after reset - LR11xx may be inoperable for this period of time */
#define LR11XX_RADIO_HAL_ERASEFLASH_TIME_MS               (2500u)    /*!< Additional time (in milliseconds) required to complete EraseFlash command. As per Semtech's AN1200-57 this is about 2.5 seconds */
#define LR11XX_RADIO_HAL_CHECK_FW_TIME_MS                 (200u)     /*!< Timeout (in milliseconds) for CryptoCheckEncryptedFirmwareImage command to complete */

#define STATUS_FIELD_OFFSET_BITS                          (1)
#define STATUS_OK_MASK                                    (LR11XX_SYSTEM_CMD_STATUS_OK << STATUS_FIELD_OFFSET_BITS)
#define LR11XX_WAKEUP_NSS_HOLD_TIME_US                    (100u)     /*!< Hold time of the NSS pin to wake up the radio from Sleep state - 100us minimum as per the datasheet */

#define LR11XX_WAIT_CFG_LF_CLK_TIMEOUT_MS                 (400u)     /*!< Time allowance for ConfigLfClock command to complete */
#define LR11XX_WAIT_CALIBRATE_TIMEOUT_MS                  (50u)      /*!< Time allowance for Calibrate command to complete */
#define LR11XX_WAIT_CALIBRATE_PROBE_MS                    (2u)       /*!< Scheduler mode probe period to check if Busy pin state has changed */

#define LR11XX_WAIT_ON_BUSY_DEFAULT_TIMEOUT_US            (8000u)    /*!< Wait timeout for Busy pin to go low */
#define LR11XX_WAIT_ON_BUSY_WAKEUP_TIMEOUT_US             (32000u)   /*!< Wait timeout for Busy pin when the wakeup sequence is performed*/
#define LR11XX_WAIT_ON_BUSY_GEOLOC_CMD_TIMEOUT_US         (80000u)   /*!< Wait time for Busy pin to go low for geolocation commands (GNSS and WiFi) */
#define LR11XX_WAIT_ON_BUSY_GNSS_ABORT_CMD_TIMEOUT_MS     (6000u)    /*!< Wait time for a blocking command to be aborted. The delay before the GNSS gets aborted is variable, since it depends on the current task executed by the internal MCU. The maximum delay is approximately 2.9s */
#define LR11XX_WAIT_ON_BUSY_WIFI_ABORT_CMD_TIMEOUT_MS     (150000u)  /*!< Wait time for a blocking command to be aborted. Since WiFi commands cannot be actually aborted, this is a wait time till WiFi scan completion */
#define LR11XX_WAIT_ON_BUSY_PROBE_PERIOD_US               (5u)       /*!< Probe period to check if Busy pin state has changed */
#define LR11XX_LONG_WAIT_ON_BUSY_PROBE_MS                 (10u)      /*!< Probe period for long waits (e.g. reboot) - uses scheduler delays if possible */

#define LR11XX_RADIO_COMMAND_NONE                         (0x0000u)  /*!< Empty command - used to readout command response data */
#define LR11XX_RADIO_COMMAND_CFG_LF_CLK                   (0x0116u)
#define LR11XX_RADIO_COMMAND_CALIBRATE                    (0x010Fu)
#define LR11XX_RADIO_COMMAND_SET_SLEEP                    (0x011Bu)
#define LR11XX_RADIO_COMMAND_CHECK_FW                     (0x050Fu)  /*!< CryptoCheckEncryptedFirmwareImage command */
#define LR11XX_RADIO_COMMAND_REBOOT                       (0x0118u)  /*!< Reboot opcode in app mode*/
#define LR11XX_RADIO_COMMAND_BL_REBOOT                    (0x8005u)  /*!< Reboot opcode in Bootloader mode */
#define LR11XX_RADIO_COMMAND_BL_ERASEFLASH                (0x8000u)  /*!< EraseFlash opcode in Bootloader mode */

#define LR11XX_RADIO_COMMAND_GROUP_GNSS_PREFIX            (0x04u)
#define LR11XX_RADIO_COMMAND_GNSS_SCAN_OC                 (0x040Bu) /*!< Launch the scan */
#define LR11XX_RADIO_COMMAND_GNSS_FETCH_TIME_OC           (0x0432u) /*!< Start the time acquisition/demodulation */
#define LR11XX_RADIO_COMMAND_GNSS_ALMANAC_UPDATE_FROM_SAT (0x0454u) /*!< Launches one scan to download from satellite almanac parameters broadcasted */

#define LR11XX_RADIO_COMMAND_GROUP_WIFI_PREFIX            (0x03u)
#define LR11XX_RADIO_COMMAND_WIFI_SCAN                    (0x0300u)
#define LR11XX_RADIO_COMMAND_WIFI_SCAN_TIME_LIMIT         (0x0301u)
#define LR11XX_RADIO_COMMAND_WIFI_SEARCH_COUNTRY_CODE     (0x0302u)
#define LR11XX_RADIO_COMMAND_WIFI_COUNTRY_CODE_TIME_LIMIT (0x0303u)

#define LR11XX_RADIO_SPI_BUFFER_TX_PARTITION_OFFSET       (sizeof(uint32_t))      /*!< The absolute minimum amount of bytes the Tx partition shall be ahead of Rx partition in the SPI buffer */
static_assert((LR11XX_RADIO_SPI_BUFFER_TX_PARTITION_OFFSET % sizeof(uint32_t)) == 0u); /* Tx partition should be aligned to 4-byte boundary to avoid issues with DMA access to it */

/* Imported function prototypes ----------------------------------------------*/

#if LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION
void smtc_modem_hal_radio_irq(void);
#endif /* LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION */

/* Private function prototypes -----------------------------------------------*/

static        void                _on_radio_irq_detected(uint32_t pin, void * callback_arg);
static inline lr11xx_hal_status_t lr11xx_is_busy_indicated(const halo_drv_semtech_ctx_t * const drv_ctx, uint32_t * const out_is_radio_busy);
static inline lr11xx_hal_status_t lr11xx_wait_on_busy(const halo_drv_semtech_ctx_t * const drv_ctx, const uint32_t timeout_us);
static        lr11xx_hal_status_t lr11xx_hal_rdwr(halo_drv_semtech_ctx_t * const drv_ctx,
                                                  const uint8_t * const command, const uint16_t command_length,
                                                  uint8_t * const data, const uint16_t data_length,
                                                  const bool read);

/* Private function definitions ----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static void _on_radio_irq_detected(uint32_t pin, void * callback_arg)
{
    struct sid_timespec event_ts;

    /* Store the timestamp as soon as possible */
    (void)sid_pal_uptime_now(&event_ts);
    __COMPILER_BARRIER();

    (void)pin;

    halo_drv_semtech_ctx_t * const drv_ctx = (halo_drv_semtech_ctx_t *)callback_arg;
    SID_PAL_ASSERT(drv_ctx != NULL);

    /* Immediately stop software timer for Tx/Rx/CS/CAD timeout - we don't care about the exact IRQ here */
    (void)sid_pal_timer_cancel(&drv_ctx->radio_timeout_mon);
    __COMPILER_BARRIER();

    SID_PAL_ASSERT(drv_ctx->radio_rx_packet != NULL);

    LR11XX_RADIO_HAL_LOG_DEBUG("_on_radio_irq_detected...");

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

#if LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION
    if (drv_ctx->lbm.bridge_state > RADIO_LR11XX_LBM_BRIDGE_STATE_ACTIVATION_PENDING)
    {
        /* Route radio IRQ to the LoRa Basics Modem */
        smtc_modem_hal_radio_irq();
    }
    else
#endif /* LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION */
    {
        /* Disable radio IRQ line  to allow the MCU's IRQ handler to return - Sidewalk may process radio IRQ in a RTOS task context */
        lr11xx_hal_status_t err = lr11xx_hal_disarm_irq(drv_ctx);
        if (err != LR11XX_HAL_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to disarm LR11xx IRQ line. error %u", (uint32_t)err);
        }

        /* Route IRQ to Sidewalk stack */
        drv_ctx->irq_handler();
    }
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline lr11xx_hal_status_t lr11xx_is_busy_indicated(const halo_drv_semtech_ctx_t * const drv_ctx, uint32_t * const out_is_radio_busy)
{
    lr11xx_hal_status_t err;
    sid_error_t sid_err;
    uint8_t is_radio_busy;

    /* Read the GPIO pin state */
    sid_err = sid_pal_gpio_read(drv_ctx->config->gpio.radio_busy, &is_radio_busy);

    if (SID_ERROR_NONE == sid_err)
    {
        *out_is_radio_busy = is_radio_busy != 0u? TRUE : FALSE;
        err = LR11XX_HAL_STATUS_OK;
    }
    else
    {
        /* Failed to read pin */
        LR11XX_RADIO_HAL_LOG_ERROR("Failed to read radio Busy pin state. Error %d", (int32_t)sid_err);
        err = LR11XX_HAL_STATUS_ERROR;
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline lr11xx_hal_status_t lr11xx_wait_on_busy(const halo_drv_semtech_ctx_t * const drv_ctx, const uint32_t timeout_us)
{
    lr11xx_hal_status_t err;
    uint32_t is_radio_busy = 0;
    uint32_t accumulated_wait_time = 0u;

    SID_PAL_ASSERT(drv_ctx != NULL);

    /* Generic wait procedure - poll Busy pin */
    do
    {
        err = lr11xx_is_busy_indicated(drv_ctx, &is_radio_busy);

        /* If no errors during the Busy pin check and radio does not indicate Busy state */
        if ((LR11XX_HAL_STATUS_OK == err) && (FALSE == is_radio_busy))
        {
            break;
        }

        sid_pal_delay_us(LR11XX_WAIT_ON_BUSY_PROBE_PERIOD_US);
        accumulated_wait_time += LR11XX_WAIT_ON_BUSY_PROBE_PERIOD_US;
    } while (accumulated_wait_time < (timeout_us / 2u)); /* Spend half of timeout */

    /* Handle half-timeout */
    if (accumulated_wait_time >= (timeout_us / 2u))
    {
        uint32_t long_probe_period_ms = LR11XX_LONG_WAIT_ON_BUSY_PROBE_MS;
        uint32_t long_wait_time;

        if (LR11XX_RADIO_COMMAND_CFG_LF_CLK == drv_ctx->last.command)
        {
            long_wait_time = LR11XX_WAIT_CFG_LF_CLK_TIMEOUT_MS;
        }
        else if (LR11XX_RADIO_COMMAND_CALIBRATE == drv_ctx->last.command)
        {
            long_probe_period_ms = LR11XX_WAIT_CALIBRATE_PROBE_MS;
            long_wait_time       = LR11XX_WAIT_CALIBRATE_TIMEOUT_MS;
        }
        else
        {
            /* Proceed normally for the other commands */
            long_wait_time = 0u;
        }

        while (long_wait_time > 0u)
        {
            if (long_wait_time >= long_probe_period_ms)
            {
                long_wait_time -= long_probe_period_ms;
            }
            else
            {
                long_probe_period_ms = long_wait_time;
                long_wait_time = 0u;
            }

#ifdef SID_PAL_ENABLE_SCHEDULER_DELAY
            if (SID_STM32_UTIL_IS_IRQ() == FALSE)
            {
                sid_pal_scheduler_delay_ms(long_probe_period_ms);
            }
            else
            {
                sid_pal_delay_us(long_probe_period_ms * 1000u);
            }
#else
            SID_PAL_LOG_WARNING("Consider enabling scheduler delays. ConfigLfClock command takes about 250ms, system is completely blocked during this time");
            sid_pal_delay_us(long_probe_period_ms * 1000u);
#endif /* SID_PAL_ENABLE_SCHEDULER_DELAY */

            err = lr11xx_is_busy_indicated(drv_ctx, &is_radio_busy);

            /* If no errors during the Busy pin check and radio does not indicate Busy state */
            if ((LR11XX_HAL_STATUS_OK == err) && (FALSE == is_radio_busy))
            {
                break;
            }
        }
    }

    /* Proceed with the generic wait procedure - poll Busy pin */
    do
    {
        err = lr11xx_is_busy_indicated(drv_ctx, &is_radio_busy);

        /* If no errors during the Busy pin check and radio does not indicate Busy state */
        if ((LR11XX_HAL_STATUS_OK == err) && (FALSE == is_radio_busy))
        {
            break;
        }

        sid_pal_delay_us(LR11XX_WAIT_ON_BUSY_PROBE_PERIOD_US);
        accumulated_wait_time += LR11XX_WAIT_ON_BUSY_PROBE_PERIOD_US;
    } while (accumulated_wait_time < timeout_us);

    /* Handle wait timeout */
    if (accumulated_wait_time >= timeout_us)
    {
        SID_PAL_LOG_ERROR("Timeout on waiting for the radio to clear Busy pin. Wait limit: %uus", timeout_us);
        err = LR11XX_HAL_STATUS_ERROR;
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static lr11xx_hal_status_t lr11xx_hal_rdwr(halo_drv_semtech_ctx_t * const drv_ctx,
                                                                     const uint8_t * const command, const uint16_t command_length,
                                                                     uint8_t * const data, const uint16_t data_length,
                                                                     const bool read)
{
    lr11xx_hal_status_t err;
    lr11xx_status_t sys_err;
    sid_error_t sid_err;

    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->config != NULL);
    SID_PAL_ASSERT(drv_ctx->config->internal_buffer.p != NULL);
    SID_PAL_ASSERT((false == read) || ((data != NULL) && (data_length > 0u))); /* Either readback is not required, or readback buffer pointer is valid and buffer size is non-zero */

    do
    {
        uint32_t        cmd_transfer_size;
        uint32_t        readback_transfer_size;
        uint32_t        required_buffer_size;
        const uint8_t * tx_buff;
        uint8_t *       rx_buff;
        uint32_t        post_command_busy_timeout_us;

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

        /* Compute the required SPI buffer size and make sure the data fits in the buffer  */
        if ((false == read) && (data_length > 0u))
        {
            /* This is command with data */
            cmd_transfer_size = command_length + data_length;
        }
        else
        {
            /* This is command only */
            cmd_transfer_size = command_length;
        }

        /* Ensure there's no buffer overflow for cmd send out */
        required_buffer_size = LR11XX_RADIO_SPI_BUFFER_TX_PARTITION_OFFSET + cmd_transfer_size /* space for Tx partition */;
        if (drv_ctx->config->internal_buffer.size < required_buffer_size)
        {
            SID_PAL_LOG_ERROR("Insufficient radio SPI processing buffer size for command 0x%02X%02X, required bytes: %u, available: %u", command[0], command[1], required_buffer_size, drv_ctx->config->internal_buffer.size);
            err = LR11XX_HAL_STATUS_ERROR;
            break;
        }

        if (read != false)
        {
            /* Calculate SPI buffer requirements for command response readout */
            readback_transfer_size = data_length + 1u; /* First byte is Stat1 */
            required_buffer_size = LR11XX_RADIO_SPI_BUFFER_TX_PARTITION_OFFSET + readback_transfer_size; /* We still need space for the Tx buffer since we are obligated to send out NOPs (0x00) to LR11xx even for readback */

            /* Ensure there's no buffer overflow */
            if (drv_ctx->config->internal_buffer.size <= required_buffer_size)
            {
                SID_PAL_LOG_ERROR("Insufficient radio SPI processing buffer size for command 0x%02X%02X, required bytes: %u, available: %u", command[0], command[1], required_buffer_size, drv_ctx->config->internal_buffer.size);
                err = LR11XX_HAL_STATUS_ERROR;
                break;
            }
        }
        else
        {
            readback_transfer_size = 0u;
        }

        /* Check if radio wakeup is required */
        if ((SID_PAL_RADIO_SLEEP == drv_ctx->radio_state) || (SID_PAL_RADIO_UNKNOWN == drv_ctx->radio_state) || (SID_PAL_RADIO_RX_DC == drv_ctx->radio_state))
        {
            sys_err = lr11xx_system_wakeup(drv_ctx);
            if (sys_err != LR11XX_STATUS_OK)
            {
                drv_ctx->radio_state = SID_PAL_RADIO_UNKNOWN;
                err = LR11XX_HAL_STATUS_ERROR;
                break;
            }
        }

        /* Wait for readiness */
        err = lr11xx_wait_on_busy(drv_ctx, LR11XX_WAIT_ON_BUSY_DEFAULT_TIMEOUT_US);
        if (err != LR11XX_HAL_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Radio stuck with previous command execution. Previous cmd: 0x%04X, current cmd: 0x%02X%02X", drv_ctx->last.command, command[0], command[1]);
            break;
        }

#if LR11XX_RADIO_HAL_EXTRA_LOGGING
        LR11XX_RADIO_HAL_LOG_INFO("-----------------------------");
        LR11XX_RADIO_HAL_LOG_INFO((read)? "Command (read):" : "Command (write):");
        SID_PAL_HEXDUMP(SID_PAL_LOG_SEVERITY_INFO, command, command_length);
        if ((false == read) && (data_length > 0u))
        {
            LR11XX_RADIO_HAL_LOG_INFO("Data:");
            SID_PAL_HEXDUMP(SID_PAL_LOG_SEVERITY_INFO, data, data_length);
        }
#endif /* LR11XX_RADIO_HAL_EXTRA_LOGGING */

        /* Build command Tx buffer */
        tx_buff = &drv_ctx->config->internal_buffer.p[LR11XX_RADIO_SPI_BUFFER_TX_PARTITION_OFFSET];

        if ((false == read) && (data_length > 0u))
        {
            /* This is command with data - compile command and data into single buffer */
            SID_STM32_UTIL_fast_memcpy((uint8_t *)tx_buff                 , command, command_length);
            SID_STM32_UTIL_fast_memcpy((uint8_t *)&tx_buff[command_length], data   , data_length);
        }
        else
        {
            /* This is a command-only transfer, but we still have to copy command into the Tx buffer to ensure data alignment */
            SID_STM32_UTIL_fast_memcpy((uint8_t *)tx_buff, command, command_length);
        }

        /* Setup read buffer for command transfer */
        rx_buff = drv_ctx->config->internal_buffer.p;

        /* Send the command */
        sid_err = drv_ctx->bus_iface->xfer(drv_ctx->bus_iface, &drv_ctx->config->bus_selector, (uint8_t *)tx_buff, rx_buff, cmd_transfer_size);
        if (sid_err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_WARNING("LR11xx: Command 0x%02X%02X failed; SPI error: %d", command[0], command[1], (int32_t)sid_err);
            err = LR11XX_HAL_STATUS_ERROR;
            break;
        }

        /* Update status bytes based on the received data - these status bytes belong to the previous command, not to the one currently sent */
        drv_ctx->last.stat1 = rx_buff[0];
        drv_ctx->last.stat2 = rx_buff[1];

        if (((drv_ctx->last.stat1 & STATUS_OK_MASK) == 0u) && (drv_ctx->last.command != LR11XX_RADIO_COMMAND_NONE))
        {
            /* This failure belongs to the previous command, so just report it and proceed with the current command */
            SID_PAL_LOG_WARNING("LR11xx: Command 0x%04X failed; Stat1 0x%02X", drv_ctx->last.command, drv_ctx->last.stat1);
        }

        /* Now update the last command - next status bytes will provide info about the currently executed command */
        drv_ctx->last.command = ((uint16_t)command[0] << 8) | (uint16_t)command[1];

#if LR11XX_RADIO_HAL_EXTRA_LOGGING
        LR11XX_RADIO_HAL_LOG_INFO("Cmd read back");
        SID_PAL_HEXDUMP(SID_PAL_LOG_SEVERITY_INFO, rx_buff, cmd_transfer_size);
#endif /* LR11XX_RADIO_HAL_EXTRA_LOGGING */

        /* Wait for command processing to complete */
        if (LR11XX_RADIO_COMMAND_SET_SLEEP == drv_ctx->last.command) /* SetSleep command is a special case - it will make LR11xx to indicate Busy state till wakeup */
        {
            /* Disable IRQ pin on the MCU side to avoid reaction on any spikes while the radio is in sleep and does not actively drive the IRQ line */
            err = lr11xx_hal_disarm_irq(drv_ctx);
            if (err != LR11XX_HAL_STATUS_OK)
            {
                LR11XX_RADIO_HAL_LOG_WARNING("Failed to disarm radio IRQ before sleep entry. Error %d", (int32_t)err);
                /* Continue execution since this is not a critical failure */
            }

            /* Radio is in sleep, nothing more can be transferred till wakeup */
            drv_ctx->radio_state = SID_PAL_RADIO_SLEEP;
            err = LR11XX_HAL_STATUS_OK;
            break;
        }

#if LR11XX_RADIO_CFG_GEOLOCATION_SUPPORT
        /* The below GNSS and WiFi commands keep Busy for the prolonged time. For these cases Busy is handled by LBM middleware, no more actions required here */
        if ( (drv_ctx->last.command == LR11XX_RADIO_COMMAND_GNSS_SCAN_OC)
          || (drv_ctx->last.command == LR11XX_RADIO_COMMAND_WIFI_SCAN_TIME_LIMIT)
          || (drv_ctx->last.command == LR11XX_RADIO_COMMAND_GNSS_ALMANAC_UPDATE_FROM_SAT)
          || (drv_ctx->last.command == LR11XX_RADIO_COMMAND_WIFI_SCAN)
          || (drv_ctx->last.command == LR11XX_RADIO_COMMAND_GNSS_FETCH_TIME_OC)
          || (drv_ctx->last.command == LR11XX_RADIO_COMMAND_WIFI_SEARCH_COUNTRY_CODE)
          || (drv_ctx->last.command == LR11XX_RADIO_COMMAND_WIFI_COUNTRY_CODE_TIME_LIMIT)
        )
        {
            err = LR11XX_HAL_STATUS_OK;
            break;
        }
#endif /* LR11XX_RADIO_CFG_GEOLOCATION_SUPPORT */

        /* Select wait time for Busy pin to be cleared based on the specific command */
        switch (drv_ctx->last.command)
        {
            case LR11XX_RADIO_COMMAND_REBOOT:
            case LR11XX_RADIO_COMMAND_BL_REBOOT:
                /* Reboot command is a special case - it requires longer wait time to allow LR11xx to boot */
                post_command_busy_timeout_us = LR11XX_RADIO_HAL_BOOTUP_TIMEOUT_MS * 1000u;
                break;

            case LR11XX_RADIO_COMMAND_BL_ERASEFLASH:
                /* EraseFlash command is a special case - it requires longer wait time too */
                post_command_busy_timeout_us = LR11XX_RADIO_HAL_ERASEFLASH_TIME_MS * 1000u;
                break;

            case LR11XX_RADIO_COMMAND_CHECK_FW:
                post_command_busy_timeout_us = LR11XX_RADIO_HAL_CHECK_FW_TIME_MS * 1000u;
                break;

            default:
#if LR11XX_RADIO_CFG_GEOLOCATION_SUPPORT
                if ((LR11XX_RADIO_COMMAND_GROUP_GNSS_PREFIX == command[0]) || (LR11XX_RADIO_COMMAND_GROUP_WIFI_PREFIX == command[0]))
                {
                    /* GNSS- and WiFi-related commands are a special case and require longer wait times */
                    post_command_busy_timeout_us = LR11XX_WAIT_ON_BUSY_GEOLOC_CMD_TIMEOUT_US;
                }
                else
#endif /* LR11XX_RADIO_CFG_GEOLOCATION_SUPPORT */
                {
                    /* Use the default (short) Busy wait timeout */
                    post_command_busy_timeout_us = LR11XX_WAIT_ON_BUSY_DEFAULT_TIMEOUT_US;
                }
                break;
        }

        /* Wait for command completion */
        err = lr11xx_wait_on_busy(drv_ctx, post_command_busy_timeout_us);
        if (err != LR11XX_HAL_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Radio stuck with command execution. Command 0x%04X", drv_ctx->last.command);
            break;
        }

        /* If no command response is expected we're done with the transfer */
        if (false == read)
        {
            err = LR11XX_HAL_STATUS_OK;
            break;
        }

        /* Read out command response */
        /* Configure SPI buffer pointers */
        rx_buff = drv_ctx->config->internal_buffer.p;
        tx_buff = &drv_ctx->config->internal_buffer.p[LR11XX_RADIO_SPI_BUFFER_TX_PARTITION_OFFSET];

        /* Fill Tx buffer with NOPs (0x00) */
        SID_STM32_UTIL_fast_memset((uint8_t *)tx_buff, 0x00u, readback_transfer_size);

        /* Do SPI transfer. Don't use Tx buffer here since we are only interested in LR11xx response */
        sid_err = drv_ctx->bus_iface->xfer(drv_ctx->bus_iface, &drv_ctx->config->bus_selector, (uint8_t *)tx_buff, rx_buff, readback_transfer_size);
        if (sid_err != SID_ERROR_NONE)
        {
            err = LR11XX_HAL_STATUS_ERROR;
            break;
        }

        /* Update status registers */
        drv_ctx->last.stat1 = rx_buff[0];
        if (((drv_ctx->last.stat1 & STATUS_OK_MASK) == 0u) && (drv_ctx->last.command != LR11XX_RADIO_COMMAND_NONE))
        {
            /* This failure belongs to the current command, terminate with error */
            SID_PAL_LOG_ERROR("LR11xx: Command rsp 0x%04X failed; Stat1 0x%02X. Received response data is not valid", drv_ctx->last.command, drv_ctx->last.stat1);
            err = LR11XX_HAL_STATUS_ERROR;
            break;
        }

#if LR11XX_RADIO_HAL_EXTRA_LOGGING
        LR11XX_RADIO_HAL_LOG_INFO("Data");
        SID_PAL_HEXDUMP(SID_PAL_LOG_SEVERITY_INFO, rx_buff, transfer_size);
#endif /* LR11XX_RADIO_HAL_EXTRA_LOGGING */

        /* Copy relevant part of the response into the caller-supplied buffer */
        SID_STM32_UTIL_fast_memcpy(data, &rx_buff[1], data_length);

        /* Done */
        err = LR11XX_HAL_STATUS_OK;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED lr11xx_hal_status_t lr11xx_hal_abort_blocking_cmd(const void * context)
{
    lr11xx_hal_status_t err, tmp_err;
    lr11xx_status_t sys_err;
    sid_error_t sid_err;
    halo_drv_semtech_ctx_t * const drv_ctx = (halo_drv_semtech_ctx_t *)context;

    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->config != NULL);

    do
    {
        uint32_t is_radio_busy = 0;

        err = lr11xx_is_busy_indicated(drv_ctx, &is_radio_busy);
        if ((LR11XX_HAL_STATUS_OK == err) && (FALSE == is_radio_busy))
        {
            /* No blocking command is being executed, return back immediately */
            break;
        }

        err = lr11xx_hal_disarm_irq(drv_ctx);
        if (err != LR11XX_HAL_STATUS_OK)
        {
            LR11XX_RADIO_HAL_LOG_ERROR("Failed to abort LR11xx blocking command - error %d on disarming radio IRQ pin", (int32_t)err);
            break;
        }

        if ((drv_ctx->last.command >> 8) == LR11XX_RADIO_COMMAND_GROUP_GNSS_PREFIX)
        {
            /* Issue SPI abort command */
            uint8_t dummy_command[1] = { 0 };

            /* Perform SPI transfer */
            sid_err = drv_ctx->bus_iface->xfer(drv_ctx->bus_iface, &drv_ctx->config->bus_selector, dummy_command, NULL, sizeof(dummy_command));
            if (sid_err != SID_ERROR_NONE)
            {
                LR11XX_RADIO_HAL_LOG_ERROR("Failed to abort LR11xx blocking command. SPI transfer error %d", (int32_t)sid_err);
                err = LR11XX_HAL_STATUS_ERROR;
                break;
            }

            /* Wait for abort completion */
            err = lr11xx_wait_on_busy(drv_ctx, (LR11XX_WAIT_ON_BUSY_GNSS_ABORT_CMD_TIMEOUT_MS * 1000u));
            if (err != LR11XX_HAL_STATUS_OK)
            {
                LR11XX_RADIO_HAL_LOG_ERROR("Failed to abort LR11xx blocking command - Busy still active after wait");
                break;
            }
        }
        else if ((drv_ctx->last.command >> 8) == LR11XX_RADIO_COMMAND_GROUP_WIFI_PREFIX)
        {
            /* WiFi commands do not support SPI abort command as of FW 04.01, the only option is to wait for WiFi scan completion */

            err = lr11xx_wait_on_busy(drv_ctx, (LR11XX_WAIT_ON_BUSY_WIFI_ABORT_CMD_TIMEOUT_MS * 1000u));
            if (err != LR11XX_HAL_STATUS_OK)
            {
                LR11XX_RADIO_HAL_LOG_ERROR("Failed to abort LR11xx blocking command - Busy still active after wait");
                break;
            }
        }
        else
        {
            LR11XX_RADIO_HAL_LOG_ERROR("Command 0x%04X cannot be aborted", drv_ctx->last.command);
            err = LR11XX_HAL_STATUS_ERROR;
            break;
        }

        /* Clear last command storage */
        drv_ctx->last.command = LR11XX_RADIO_COMMAND_NONE;

        /* Clear IRQ that will be generated after aborting a command */
        sys_err = lr11xx_system_clear_irq_status(drv_ctx, LR11XX_SYSTEM_IRQ_ALL_MASK);
        if (sys_err != LR11XX_STATUS_OK)
        {
            LR11XX_RADIO_HAL_LOG_ERROR("Failed to abort LR11xx blocking command - error %d on clearing radio IRQs", (int32_t)sys_err);
            err = LR11XX_HAL_STATUS_ERROR;
            break;
        }

        err = LR11XX_HAL_STATUS_OK;
    } while (0);

    /* Restore IRQ reaction */
    tmp_err = lr11xx_hal_arm_irq(drv_ctx);
    if (tmp_err != LR11XX_HAL_STATUS_OK)
    {
        LR11XX_RADIO_HAL_LOG_ERROR("Failed to abort LR11xx blocking command - error %d on re-arming radio IRQ pin", (int32_t)tmp_err);
    }

    /* Select errorto report */
    err = (LR11XX_HAL_STATUS_OK == err) ? tmp_err : err;

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED lr11xx_hal_status_t lr11xx_hal_direct_read(const void * context, uint8_t * const data, const uint16_t data_length)
{
    lr11xx_hal_status_t err;
    sid_error_t sid_err;
    halo_drv_semtech_ctx_t * const drv_ctx = (halo_drv_semtech_ctx_t *)context;

    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->config != NULL);
    SID_PAL_ASSERT(drv_ctx->config->internal_buffer.p != NULL);
    SID_PAL_ASSERT(data != NULL);
    SID_PAL_ASSERT(data_length != 0u);

    do
    {
        uint32_t required_buffer_size;

        /* Ensure there's no buffer overflow */
        required_buffer_size = data_length + LR11XX_RADIO_SPI_BUFFER_TX_PARTITION_OFFSET;
        if (drv_ctx->config->internal_buffer.size < required_buffer_size)
        {
            SID_PAL_LOG_ERROR("Insufficient radio SPI processing buffer size, required bytes: %u, available: %u", required_buffer_size, drv_ctx->config->internal_buffer.size);
            err = LR11XX_HAL_STATUS_ERROR;
            break;
        }

        /* Prepare Tx and Rx buffers */
        uint8_t * const rx_buff = drv_ctx->config->internal_buffer.p; /* Use this Rx buffer instead of data variable to avoid alignment issues */
        uint8_t * const tx_buff = &drv_ctx->config->internal_buffer.p[LR11XX_RADIO_SPI_BUFFER_TX_PARTITION_OFFSET];

        /* Fill Tx buffer with NOPs (0x00) */
        SID_STM32_UTIL_fast_memset(tx_buff, 0x00u, data_length);

        /* Wait for readiness */
        err = lr11xx_wait_on_busy(drv_ctx, LR11XX_WAIT_ON_BUSY_DEFAULT_TIMEOUT_US);
        if (err != LR11XX_HAL_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Can't communicate with the radio - busy status is indicated");
            break;
        }

        /* Perform SPI transfer */
        sid_err = drv_ctx->bus_iface->xfer(drv_ctx->bus_iface, &drv_ctx->config->bus_selector, tx_buff, rx_buff, data_length);

        if (((drv_ctx->last.stat1 & STATUS_OK_MASK) == 0u) && (drv_ctx->last.command != LR11XX_RADIO_COMMAND_NONE))
        {
            SID_PAL_LOG_WARNING("LR11xx: Command 0x%04X failed; Stat1 0x%02X", drv_ctx->last.command, drv_ctx->last.stat1);
        }
        drv_ctx->last.command = LR11XX_RADIO_COMMAND_NONE; /* No command  - only read. Chip will complain about it */

        /* Copy received data into the output buffer */
        SID_STM32_UTIL_fast_memcpy(data, rx_buff, data_length);

#if LR11XX_RADIO_HAL_EXTRA_LOGGING
        LR11XX_RADIO_HAL_LOG_INFO("-----------------------------");
        LR11XX_RADIO_HAL_LOG_INFO("Direct read");
        SID_PAL_HEXDUMP(SID_PAL_LOG_SEVERITY_INFO, data, data_length);
#endif /* LR11XX_RADIO_HAL_EXTRA_LOGGING */

        err = (SID_ERROR_NONE == sid_err) ? LR11XX_HAL_STATUS_OK : LR11XX_HAL_STATUS_ERROR;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED lr11xx_hal_status_t lr11xx_hal_reset(const void * context)
{
    lr11xx_hal_status_t err;
    sid_error_t sid_err;
    const halo_drv_semtech_ctx_t * const drv_ctx = (const halo_drv_semtech_ctx_t *)context;

    do
    {
        uint32_t reset_wait_time = LR11XX_RADIO_HAL_BOOTUP_TIMEOUT_MS;
        uint32_t probe_period_ms = LR11XX_LONG_WAIT_ON_BUSY_PROBE_MS;
        uint32_t is_radio_busy;

        /* Validate inputs */
        if (NULL == drv_ctx)
        {
            err = LR11XX_HAL_STATUS_ERROR;
            break;
        }

        /* Assert radio reset */
        sid_err = sid_pal_gpio_write(drv_ctx->config->gpio.radio_reset, 0u);
        if (sid_err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to assert radio reset. Error %d", (int32_t)sid_err);
            err = LR11XX_HAL_STATUS_ERROR;
            break;
        }

        /* Keep the reset hold time, other wise reset may not be detected by the transceiver */
        sid_pal_delay_us(LR11XX_RADIO_HAL_RESET_HOLD_TIME_US);

        /* Release reset line */
        sid_err = sid_pal_gpio_write(drv_ctx->config->gpio.radio_reset, 1u);
        if (sid_err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to release radio reset. Error %d", (int32_t)sid_err);
            err = LR11XX_HAL_STATUS_ERROR;
            break;
        }

        /* Wait for reset completion. LR11xx boot time is around 180ms */
        while (reset_wait_time > 0u)
        {
            if (reset_wait_time >= probe_period_ms)
            {
                reset_wait_time -= probe_period_ms;
            }
            else
            {
                probe_period_ms = reset_wait_time;
                reset_wait_time = 0u;
            }

#ifdef SID_PAL_ENABLE_SCHEDULER_DELAY
            if (SID_STM32_UTIL_IS_IRQ() == FALSE)
            {
                sid_pal_scheduler_delay_ms(probe_period_ms);
            }
            else
            {
                sid_pal_delay_us(probe_period_ms * 1000u);
            }
#else
            SID_PAL_LOG_WARNING("Consider enabling scheduler delays. LR11xx boot time is approx. 180ms, system is completely blocked during this time");
            sid_pal_delay_us(probe_period_ms * 1000u);
#endif /* SID_PAL_ENABLE_SCHEDULER_DELAY */

            /* Read Busy pin state */
            err = lr11xx_is_busy_indicated(drv_ctx, &is_radio_busy);

            /* If no errors during the Busy pin check and radio does not indicate Busy state */
            if ((LR11XX_HAL_STATUS_OK == err) && (FALSE == is_radio_busy))
            {
                break;
            }
        }

        /* Check the wait loop completed normally */
        if (err != LR11XX_HAL_STATUS_OK)
        {
            break;
        }

        /* Check if radio is ready */
        if (is_radio_busy != FALSE)
        {
            SID_PAL_LOG_ERROR("Timeout on waiting LR11xx boot-up. Radio is not responding");
            err = LR11XX_HAL_STATUS_ERROR;
        }

        /* Done */
        err = LR11XX_HAL_STATUS_OK;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED lr11xx_hal_status_t lr11xx_hal_wakeup(const void * context)
{
    lr11xx_hal_status_t err;
    sid_error_t sid_err;
    uint32_t is_radio_sleeping;
    halo_drv_semtech_ctx_t * const drv_ctx = (halo_drv_semtech_ctx_t *)context;

    do
    {
        /* Validate inputs */
        if (NULL == drv_ctx)
        {
            err = LR11XX_HAL_STATUS_ERROR;
            break;
        }

#if LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION
        /* Remove any pending sleep requests from LBM */
        drv_ctx->lbm.radio_sleep_pending = FALSE;
#endif /* LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION */

        /* Check if the radio is in sleep state at all */
        err = lr11xx_is_busy_indicated(drv_ctx, &is_radio_sleeping);
        if (err != LR11XX_HAL_STATUS_OK)
        {
            break;
        }

        if (FALSE == is_radio_sleeping)
        {
            /* Busy signal is not asserted -> radio is not in a sleep state */
            err = LR11XX_HAL_STATUS_OK;
            break;
        }

        /* Drive SPI NSS to wakeup the radio */
        sid_err = sid_pal_serial_bus_ext_ifc_activate_nss(drv_ctx->bus_iface, &drv_ctx->config->bus_selector);
        if (sid_err != SID_ERROR_NONE)
        {
            LR11XX_RADIO_HAL_LOG_ERROR("LR11xx wakeup failed. NSS driving error %d", (int32_t)sid_err);
            err = LR11XX_HAL_STATUS_ERROR;
            break;
        }

        /* Give the radio enough time to wakeup */
        sid_pal_delay_us(LR11XX_WAKEUP_NSS_HOLD_TIME_US);

        /* Release NSS as it is not needed any longer */
        sid_err = sid_pal_serial_bus_ext_ifc_deactivate_nss(drv_ctx->bus_iface, &drv_ctx->config->bus_selector);
        if (sid_err != SID_ERROR_NONE)
        {
            LR11XX_RADIO_HAL_LOG_ERROR("LR11xx wakeup failed. NSS release error %d", (int32_t)sid_err);
            err = LR11XX_HAL_STATUS_ERROR;
            break;
        }

        /* Wait for chip to be ready */
        err = lr11xx_wait_on_busy(drv_ctx, LR11XX_WAIT_ON_BUSY_WAKEUP_TIMEOUT_US);
        if (err != LR11XX_HAL_STATUS_OK)
        {
            break;
        }

        /* Reset last command in the context since it's not valid after wakeup */
        drv_ctx->last.command = LR11XX_RADIO_COMMAND_NONE;

        /* Re-enable IRQ line */
        err = lr11xx_hal_arm_irq(drv_ctx);
        if (err != LR11XX_HAL_STATUS_OK)
        {
            LR11XX_RADIO_HAL_LOG_ERROR("Failed to re-arm radio IRQ on wakeup. Error %d", (int32_t)err);
            break;
        }

        /* The radio is now in STANDBY_RC sate */
        drv_ctx->radio_state = SID_PAL_RADIO_STANDBY;

        /* Done */
        err = LR11XX_HAL_STATUS_OK;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED lr11xx_hal_status_t lr11xx_hal_read(const void * context, const uint8_t* command, const uint16_t command_length,
                                                             uint8_t* data, const uint16_t data_length)
{
    lr11xx_hal_status_t err;

    do
    {
        if ((NULL == context) || (NULL == command) || (NULL == data) || (0u == command_length) || (0u == data_length))
        {
            err = LR11XX_HAL_STATUS_ERROR;
            break;
        }

        err = lr11xx_hal_rdwr((halo_drv_semtech_ctx_t *)context, command, command_length, data, data_length, true);
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED lr11xx_hal_status_t lr11xx_hal_write(const void * context, const uint8_t* command, const uint16_t command_length,
                                                               const uint8_t* data, const uint16_t data_length)
{
    halo_drv_semtech_ctx_t * const drv_ctx = (halo_drv_semtech_ctx_t *)context;
    lr11xx_hal_status_t err;

    do
    {
        /* For write data can be null and data length 0 */
        if ((NULL == drv_ctx) || (NULL == command) || (0u == command_length))
        {
            err = LR11XX_HAL_STATUS_ERROR;
            break;
        }

#if LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION
        /* Special processing is required for sleep commands coming from LoRa Basics Modem stack */
        if (LR11XX_RADIO_COMMAND_SET_SLEEP == ((command[0] << 8) | (command[1])))
        {
            if (SID_PAL_RADIO_SLEEP == drv_ctx->radio_state)
            {
                /* Radio is in sleep already, skip the sleep command */
                err = LR11XX_HAL_STATUS_OK;
                break;
            }

            if ((drv_ctx->lbm.gnss_scan_active != FALSE) || (drv_ctx->lbm.wifi_scan_active != FALSE))
            {
                /* There's an active scan session, postpone radio Sleep since post-scan actions have to talk to the radio */
                LR11XX_RADIO_HAL_LOG_DEBUG("LR11xx deferred radio sleep");
                drv_ctx->lbm.radio_sleep_pending = TRUE;
                err = LR11XX_HAL_STATUS_OK;
                break;
            }
            else
            {
                /* We are about to actually put the radio into Sleep mode, clear Pending indication */
                LR11XX_RADIO_HAL_LOG_DEBUG("LR11xx actual radio sleep");
                drv_ctx->lbm.radio_sleep_pending = FALSE;
            }
        }
#endif /* LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION */

        err = lr11xx_hal_rdwr(drv_ctx, command, command_length, (void*)data, data_length, false);
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED lr11xx_hal_status_t lr11xx_hal_gpio_init(const halo_drv_semtech_ctx_t * const drv_ctx)
{
    lr11xx_hal_status_t err = LR11XX_HAL_STATUS_ERROR;
    sid_error_t sid_err;

    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->config != NULL);

    do
    {
        /* Validate inputs */
        if (HALO_GPIO_NOT_CONNECTED == drv_ctx->config->gpio.radio_busy)
        {
            SID_PAL_LOG_ERROR("LR11xx Busy pin must be connected for Sidewalk driver to operate correctly");
            err = RADIO_ERROR_INVALID_PARAMS;
            break;
        }
        if (HALO_GPIO_NOT_CONNECTED == drv_ctx->config->gpio.radio_reset)
        {
            SID_PAL_LOG_ERROR("LR11xx Reset pin must be connected for Sidewalk driver to operate correctly");
            err = RADIO_ERROR_INVALID_PARAMS;
            break;
        }
        if (HALO_GPIO_NOT_CONNECTED == drv_ctx->config->gpio.radio_irq)
        {
            SID_PAL_LOG_ERROR("LR11xx IRQ pin must be connected for Sidewalk driver to operate correctly");
            err = RADIO_ERROR_INVALID_PARAMS;
            break;
        }

        /* Configure essential pins */

        /* Reset line */
        sid_err = sid_pal_gpio_write(drv_ctx->config->gpio.radio_reset, 0u); /* Ensure LR11xx Reset is asserted before the GPIO direction is set to avoid glitches */
        if (sid_err != SID_ERROR_NONE)
        {
            LR11XX_RADIO_HAL_LOG_ERROR("Failed to drive LR11xx reset low");
            err = LR11XX_HAL_STATUS_ERROR;
            break;
        }
        sid_err = sid_pal_gpio_output_mode(drv_ctx->config->gpio.radio_reset, SID_PAL_GPIO_OUTPUT_PUSH_PULL);
        if (sid_err != SID_ERROR_NONE)
        {
            LR11XX_RADIO_HAL_LOG_ERROR("Failed to set LR11xx reset GPIO as PP");
            err = LR11XX_HAL_STATUS_ERROR;
            break;
        }
        sid_err = sid_pal_gpio_set_direction(drv_ctx->config->gpio.radio_reset, SID_PAL_GPIO_DIRECTION_OUTPUT);
        if (sid_err != SID_ERROR_NONE)
        {
            LR11XX_RADIO_HAL_LOG_ERROR("Failed to configure LR11xx reset GPIO");
            err = LR11XX_HAL_STATUS_ERROR;
            break;
        }

        /* Busy signal */
        sid_err = sid_pal_gpio_input_mode(drv_ctx->config->gpio.radio_busy, SID_PAL_GPIO_INPUT_CONNECT);
        if (sid_err != SID_ERROR_NONE)
        {
            LR11XX_RADIO_HAL_LOG_ERROR("Failed to configure LR11xx Busy GPIO as input");
            err = LR11XX_HAL_STATUS_ERROR;
            break;
        }
        sid_err = sid_pal_gpio_set_direction(drv_ctx->config->gpio.radio_busy, SID_PAL_GPIO_DIRECTION_INPUT);
        if (sid_err != SID_ERROR_NONE)
        {
            LR11XX_RADIO_HAL_LOG_ERROR("Failed to configure LR11xx Busy GPIO");
            err = LR11XX_HAL_STATUS_ERROR;
            break;
        }

        /* IRQ line */
        sid_err = sid_pal_gpio_pull_mode(drv_ctx->config->gpio.radio_irq, SID_PAL_GPIO_PULL_DOWN);
        if (sid_err != SID_ERROR_NONE)
        {
            LR11XX_RADIO_HAL_LOG_ERROR("Failed to configure LR11xx IRQ GPIO pull-down");
            err = LR11XX_HAL_STATUS_ERROR;
            break;
        }
        sid_err = sid_pal_gpio_input_mode(drv_ctx->config->gpio.radio_irq, SID_PAL_GPIO_INPUT_CONNECT);
        if (sid_err != SID_ERROR_NONE)
        {
            LR11XX_RADIO_HAL_LOG_ERROR("Failed to configure LR11xx IRQ GPIO as input");
            err = LR11XX_HAL_STATUS_ERROR;
            break;
        }
        sid_err = sid_pal_gpio_set_direction(drv_ctx->config->gpio.radio_irq, SID_PAL_GPIO_DIRECTION_INPUT);
        if (sid_err != SID_ERROR_NONE)
        {
            LR11XX_RADIO_HAL_LOG_ERROR("Failed to configure LR11xx IRQ GPIO");
            err = LR11XX_HAL_STATUS_ERROR;
            break;
        }

        /* Optional pins */

        /* RF Switch (FEM) Enable */
        if (HALO_GPIO_NOT_CONNECTED != drv_ctx->config->gpio.fem_power)
        {
            sid_err = sid_pal_gpio_write(drv_ctx->config->gpio.fem_power, !drv_ctx->config->gpio.fem_power_en_gpio_state); /* Ensure pin state before the GPIO direction is set to avoid glitches */
            if (sid_err != SID_ERROR_NONE)
            {
                LR11XX_RADIO_HAL_LOG_ERROR("Failed to drive LR11xx RF Switch EN low");
                err = LR11XX_HAL_STATUS_ERROR;
                break;
            }
            sid_err = sid_pal_gpio_output_mode(drv_ctx->config->gpio.fem_power, SID_PAL_GPIO_OUTPUT_PUSH_PULL);
            if (sid_err != SID_ERROR_NONE)
            {
                LR11XX_RADIO_HAL_LOG_ERROR("Failed to set LR11xx RF Switch EN GPIO as PP");
                err = LR11XX_HAL_STATUS_ERROR;
                break;
            }
            sid_err = sid_pal_gpio_set_direction(drv_ctx->config->gpio.fem_power, SID_PAL_GPIO_DIRECTION_OUTPUT);
            if (sid_err != SID_ERROR_NONE)
            {
                LR11XX_RADIO_HAL_LOG_ERROR("Failed to configure LR11xx RF Switch EN GPIO");
                err = LR11XX_HAL_STATUS_ERROR;
                break;
            }
        }

        /* FEM Tx bypass */
        if (HALO_GPIO_NOT_CONNECTED != drv_ctx->config->gpio.fem_bypass)
        {
            sid_err = sid_pal_gpio_write(drv_ctx->config->gpio.fem_bypass, !drv_ctx->config->gpio.fem_bypass_en_gpio_state); /* Ensure pin state before the GPIO direction is set to avoid glitches */
            if (sid_err != SID_ERROR_NONE)
            {
                LR11XX_RADIO_HAL_LOG_ERROR("Failed to drive LR11xx RF Switch Tx Bypass low");
                err = LR11XX_HAL_STATUS_ERROR;
                break;
            }
            sid_err = sid_pal_gpio_output_mode(drv_ctx->config->gpio.fem_bypass, SID_PAL_GPIO_OUTPUT_PUSH_PULL);
            if (sid_err != SID_ERROR_NONE)
            {
                LR11XX_RADIO_HAL_LOG_ERROR("Failed to set LR11xx RF Switch Tx Bypass GPIO as PP");
                err = LR11XX_HAL_STATUS_ERROR;
                break;
            }
            sid_err = sid_pal_gpio_set_direction(drv_ctx->config->gpio.fem_bypass, SID_PAL_GPIO_DIRECTION_OUTPUT);
            if (sid_err != SID_ERROR_NONE)
            {
                LR11XX_RADIO_HAL_LOG_ERROR("Failed to configure LR11xx RF Switch Tx Bypass GPIO");
                err = LR11XX_HAL_STATUS_ERROR;
                break;
            }
        }

        /* RF Switch/FEM Tx/Rx selection */
        if (HALO_GPIO_NOT_CONNECTED != drv_ctx->config->gpio.fem_tx_rx_mode)
        {
            sid_err = sid_pal_gpio_write(drv_ctx->config->gpio.fem_tx_rx_mode, !drv_ctx->config->gpio.fem_tx_mode_sel_gpio_state); /* Ensure pin state before the GPIO direction is set to avoid glitches */
            if (sid_err != SID_ERROR_NONE)
            {
                LR11XX_RADIO_HAL_LOG_ERROR("Failed to drive LR11xx RF Switch Tx/Rx Selector low");
                err = LR11XX_HAL_STATUS_ERROR;
                break;
            }
            sid_err = sid_pal_gpio_output_mode(drv_ctx->config->gpio.fem_tx_rx_mode, SID_PAL_GPIO_OUTPUT_PUSH_PULL);
            if (sid_err != SID_ERROR_NONE)
            {
                LR11XX_RADIO_HAL_LOG_ERROR("Failed to set LR11xx RF Switch Tx/Rx Selector as PP");
                err = LR11XX_HAL_STATUS_ERROR;
                break;
            }
            sid_err = sid_pal_gpio_set_direction(drv_ctx->config->gpio.fem_tx_rx_mode, SID_PAL_GPIO_DIRECTION_OUTPUT);
            if (sid_err != SID_ERROR_NONE)
            {
                LR11XX_RADIO_HAL_LOG_ERROR("Failed to configure LR11xx RF Switch Tx/Rx Selector GPIO");
                err = LR11XX_HAL_STATUS_ERROR;
                break;
            }
        }

        /* RF Switch (FEM) Enable */
        if (HALO_GPIO_NOT_CONNECTED != drv_ctx->config->gpio.hpa_lpa_sw)
        {
            sid_err = sid_pal_gpio_write(drv_ctx->config->gpio.hpa_lpa_sw, !drv_ctx->config->gpio.hpa_lpa_sw_on_gpio_state); /* Ensure pin state before the GPIO direction is set to avoid glitches */
            if (sid_err != SID_ERROR_NONE)
            {
                LR11XX_RADIO_HAL_LOG_ERROR("Failed to drive LR11xx LPA-HPA low");
                err = LR11XX_HAL_STATUS_ERROR;
                break;
            }
            sid_err = sid_pal_gpio_output_mode(drv_ctx->config->gpio.hpa_lpa_sw, SID_PAL_GPIO_OUTPUT_PUSH_PULL);
            if (sid_err != SID_ERROR_NONE)
            {
                LR11XX_RADIO_HAL_LOG_ERROR("Failed to set LR11xx LPA-HPA GPIO as PP");
                err = LR11XX_HAL_STATUS_ERROR;
                break;
            }
            sid_err = sid_pal_gpio_set_direction(drv_ctx->config->gpio.hpa_lpa_sw, SID_PAL_GPIO_DIRECTION_OUTPUT);
            if (sid_err != SID_ERROR_NONE)
            {
                LR11XX_RADIO_HAL_LOG_ERROR("Failed to configure LR11xx LPA-HPA GPIO");
                err = LR11XX_HAL_STATUS_ERROR;
                break;
            }
        }

#if LR11XX_RADIO_CFG_GEOLOCATION_SUPPORT
        /* GNSS LNA Enable */
        if (HALO_GPIO_NOT_CONNECTED != drv_ctx->config->gpio.gnss_lna_ctrl)
        {
            sid_err = sid_pal_gpio_write(drv_ctx->config->gpio.gnss_lna_ctrl, !drv_ctx->config->gpio.gnss_lna_ctrl_en_gpio_state); /* Ensure pin state before the GPIO direction is set to avoid glitches */
            if (sid_err != SID_ERROR_NONE)
            {
                LR11XX_RADIO_HAL_LOG_ERROR("Failed to drive LR11xx GNSS LNA Enable low");
                err = LR11XX_HAL_STATUS_ERROR;
                break;
            }
            sid_err = sid_pal_gpio_output_mode(drv_ctx->config->gpio.gnss_lna_ctrl, SID_PAL_GPIO_OUTPUT_PUSH_PULL);
            if (sid_err != SID_ERROR_NONE)
            {
                LR11XX_RADIO_HAL_LOG_ERROR("Failed to set LR11xx GNSS LNA Enable GPIO as PP");
                err = LR11XX_HAL_STATUS_ERROR;
                break;
            }
            sid_err = sid_pal_gpio_set_direction(drv_ctx->config->gpio.gnss_lna_ctrl, SID_PAL_GPIO_DIRECTION_OUTPUT);
            if (sid_err != SID_ERROR_NONE)
            {
                LR11XX_RADIO_HAL_LOG_ERROR("Failed to configure LR11xx GNSS LNA Enable GPIO");
                err = LR11XX_HAL_STATUS_ERROR;
                break;
            }
        }
#endif /* LR11XX_RADIO_CFG_GEOLOCATION_SUPPORT */

        /* Status LEDs */
#if LR11XX_RADIO_CFG_USE_STATUS_LED
        /* Tx LED */
        if (HALO_GPIO_NOT_CONNECTED != drv_ctx->config->gpio.tx_led)
        {
            /* Ensure pin state before the GPIO direction is set to avoid glitches */
            lr11xx_radio_hal_tx_led_off(drv_ctx);

            sid_err = sid_pal_gpio_output_mode(drv_ctx->config->gpio.tx_led, SID_PAL_GPIO_OUTPUT_PUSH_PULL);
            if (sid_err != SID_ERROR_NONE)
            {
                LR11XX_RADIO_HAL_LOG_ERROR("Failed to set LR11xx Tx LED GPIO as PP");
                err = LR11XX_HAL_STATUS_ERROR;
                break;
            }
            sid_err = sid_pal_gpio_set_direction(drv_ctx->config->gpio.tx_led, SID_PAL_GPIO_DIRECTION_OUTPUT);
            if (sid_err != SID_ERROR_NONE)
            {
                LR11XX_RADIO_HAL_LOG_ERROR("Failed to configure LR11xx Tx LED GPIO");
                err = LR11XX_HAL_STATUS_ERROR;
                break;
            }
        }

        /* Rx LED */
        if ((HALO_GPIO_NOT_CONNECTED != drv_ctx->config->gpio.rx_led) && (drv_ctx->config->gpio.rx_led != drv_ctx->config->gpio.tx_led))
        {
            lr11xx_radio_hal_rx_led_off(drv_ctx);

            sid_err = sid_pal_gpio_output_mode(drv_ctx->config->gpio.rx_led, SID_PAL_GPIO_OUTPUT_PUSH_PULL);
            if (sid_err != SID_ERROR_NONE)
            {
                LR11XX_RADIO_HAL_LOG_ERROR("Failed to set LR11xx Rx LED GPIO as PP");
                err = LR11XX_HAL_STATUS_ERROR;
                break;
            }
            sid_err = sid_pal_gpio_set_direction(drv_ctx->config->gpio.rx_led, SID_PAL_GPIO_DIRECTION_OUTPUT);
            if (sid_err != SID_ERROR_NONE)
            {
                LR11XX_RADIO_HAL_LOG_ERROR("Failed to configure LR11xx Rx LED GPIO");
                err = LR11XX_HAL_STATUS_ERROR;
                break;
            }
        }

#  if LR11XX_RADIO_CFG_GEOLOCATION_SUPPORT
        /* GNSS scan activity LED */
        if (HALO_GPIO_NOT_CONNECTED != drv_ctx->config->gpio.gnss_scan_led)
        {
            /* Ensure pin state before the GPIO direction is set to avoid glitches */
            lr11xx_radio_hal_gnss_scan_led_off(drv_ctx);

            sid_err = sid_pal_gpio_output_mode(drv_ctx->config->gpio.gnss_scan_led, SID_PAL_GPIO_OUTPUT_PUSH_PULL);
            if (sid_err != SID_ERROR_NONE)
            {
                LR11XX_RADIO_HAL_LOG_ERROR("Failed to set LR11xx GNSS Scan LED GPIO as PP");
                err = LR11XX_HAL_STATUS_ERROR;
                break;
            }
            sid_err = sid_pal_gpio_set_direction(drv_ctx->config->gpio.gnss_scan_led, SID_PAL_GPIO_DIRECTION_OUTPUT);
            if (sid_err != SID_ERROR_NONE)
            {
                LR11XX_RADIO_HAL_LOG_ERROR("Failed to configure LR11xx GNSS Scan LED GPIO");
                err = LR11XX_HAL_STATUS_ERROR;
                break;
            }
        }

        /* WiFi scan activity LED */
        if ((HALO_GPIO_NOT_CONNECTED != drv_ctx->config->gpio.wifi_scan_led) && (drv_ctx->config->gpio.wifi_scan_led != drv_ctx->config->gpio.gnss_scan_led))
        {
            lr11xx_radio_hal_wifi_scan_led_off(drv_ctx);

            sid_err = sid_pal_gpio_output_mode(drv_ctx->config->gpio.wifi_scan_led, SID_PAL_GPIO_OUTPUT_PUSH_PULL);
            if (sid_err != SID_ERROR_NONE)
            {
                LR11XX_RADIO_HAL_LOG_ERROR("Failed to set LR11xx WiFi Scan LED GPIO as PP");
                err = LR11XX_HAL_STATUS_ERROR;
                break;
            }
            sid_err = sid_pal_gpio_set_direction(drv_ctx->config->gpio.wifi_scan_led, SID_PAL_GPIO_DIRECTION_OUTPUT);
            if (sid_err != SID_ERROR_NONE)
            {
                LR11XX_RADIO_HAL_LOG_ERROR("Failed to configure LR11xx WiFi Scan LED GPIO");
                err = LR11XX_HAL_STATUS_ERROR;
                break;
            }
        }
#  endif /* LR11XX_RADIO_CFG_GEOLOCATION_SUPPORT */
#endif /* LR11XX_RADIO_CFG_USE_STATUS_LED */

        /* Done */
        err = LR11XX_HAL_STATUS_OK;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED lr11xx_hal_status_t lr11xx_hal_gpio_deinit(const halo_drv_semtech_ctx_t * const drv_ctx)
{
    lr11xx_hal_status_t err = LR11XX_HAL_STATUS_ERROR;
    sid_error_t sid_err;

    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->config != NULL);

    do
    {
        /* Set all pins to Hi-Z */

        /* IRQ line */
        sid_err = sid_pal_gpio_pull_mode(drv_ctx->config->gpio.radio_irq, SID_PAL_GPIO_PULL_NONE);
        if (sid_err != SID_ERROR_NONE)
        {
            LR11XX_RADIO_HAL_LOG_ERROR("Failed to disable LR11xx IRQ GPIO pull-down");
            err = LR11XX_HAL_STATUS_ERROR;
            break;
        }
        sid_err = sid_pal_gpio_input_mode(drv_ctx->config->gpio.radio_irq, SID_PAL_GPIO_INPUT_DISCONNECT);
        if (sid_err != SID_ERROR_NONE)
        {
            LR11XX_RADIO_HAL_LOG_ERROR("Failed to configure LR11xx IRQ GPIO as Hi-Z input");
            err = LR11XX_HAL_STATUS_ERROR;
            break;
        }
        sid_err = sid_pal_gpio_set_direction(drv_ctx->config->gpio.radio_irq, SID_PAL_GPIO_DIRECTION_INPUT);
        if (sid_err != SID_ERROR_NONE)
        {
            LR11XX_RADIO_HAL_LOG_ERROR("Failed to configure LR11xx IRQ GPIO");
            err = LR11XX_HAL_STATUS_ERROR;
            break;
        }

        /* Busy signal */
        sid_err = sid_pal_gpio_input_mode(drv_ctx->config->gpio.radio_busy, SID_PAL_GPIO_INPUT_DISCONNECT);
        if (sid_err != SID_ERROR_NONE)
        {
            LR11XX_RADIO_HAL_LOG_ERROR("Failed to configure LR11xx Busy GPIO as Hi-Z input");
            err = LR11XX_HAL_STATUS_ERROR;
            break;
        }
        sid_err = sid_pal_gpio_set_direction(drv_ctx->config->gpio.radio_busy, SID_PAL_GPIO_DIRECTION_INPUT);
        if (sid_err != SID_ERROR_NONE)
        {
            LR11XX_RADIO_HAL_LOG_ERROR("Failed to configure LR11xx Busy GPIO");
            err = LR11XX_HAL_STATUS_ERROR;
            break;
        }

        /* Reset line */
        sid_err = sid_pal_gpio_input_mode(drv_ctx->config->gpio.radio_reset, SID_PAL_GPIO_INPUT_DISCONNECT);
        if (sid_err != SID_ERROR_NONE)
        {
            LR11XX_RADIO_HAL_LOG_ERROR("Failed to configure LR11xx reset GPIO to Hi-Z");
            err = LR11XX_HAL_STATUS_ERROR;
            break;
        }
        sid_err = sid_pal_gpio_set_direction(drv_ctx->config->gpio.radio_reset, SID_PAL_GPIO_DIRECTION_INPUT);
        if (sid_err != SID_ERROR_NONE)
        {
            LR11XX_RADIO_HAL_LOG_ERROR("Failed to set LR11xx reset GPIO as Hi-Z input");
            err = LR11XX_HAL_STATUS_ERROR;
            break;
        }

        /* Optional pins */

        /* RF Switch (FEM) Enable */
        if (HALO_GPIO_NOT_CONNECTED != drv_ctx->config->gpio.fem_power)
        {
            sid_err = sid_pal_gpio_input_mode(drv_ctx->config->gpio.fem_power, SID_PAL_GPIO_INPUT_DISCONNECT);
            if (sid_err != SID_ERROR_NONE)
            {
                LR11XX_RADIO_HAL_LOG_ERROR("Failed to configure LR11xx RF Switch EN GPIO as Hi-Z input");
                err = LR11XX_HAL_STATUS_ERROR;
                break;
            }
            sid_err = sid_pal_gpio_set_direction(drv_ctx->config->gpio.fem_power, SID_PAL_GPIO_DIRECTION_INPUT);
            if (sid_err != SID_ERROR_NONE)
            {
                LR11XX_RADIO_HAL_LOG_ERROR("Failed to configure LR11xx RF Switch EN GPIO");
                err = LR11XX_HAL_STATUS_ERROR;
                break;
            }
        }

        /* FEM Tx bypass */
        if (HALO_GPIO_NOT_CONNECTED != drv_ctx->config->gpio.fem_bypass)
        {
            sid_err = sid_pal_gpio_input_mode(drv_ctx->config->gpio.fem_bypass, SID_PAL_GPIO_INPUT_DISCONNECT);
            if (sid_err != SID_ERROR_NONE)
            {
                LR11XX_RADIO_HAL_LOG_ERROR("Failed to configure LR11xx RF Switch Tx Bypass GPIO as Hi-Z input");
                err = LR11XX_HAL_STATUS_ERROR;
                break;
            }
            sid_err = sid_pal_gpio_set_direction(drv_ctx->config->gpio.fem_bypass, SID_PAL_GPIO_DIRECTION_INPUT);
            if (sid_err != SID_ERROR_NONE)
            {
                LR11XX_RADIO_HAL_LOG_ERROR("Failed to configure LR11xx RF Switch Tx Bypass GPIO");
                err = LR11XX_HAL_STATUS_ERROR;
                break;
            }
        }

        /* RF Switch/FEM Tx/Rx selection */
        if (HALO_GPIO_NOT_CONNECTED != drv_ctx->config->gpio.fem_bypass)
        {
            sid_err = sid_pal_gpio_input_mode(drv_ctx->config->gpio.fem_tx_rx_mode, SID_PAL_GPIO_INPUT_DISCONNECT);
            if (sid_err != SID_ERROR_NONE)
            {
                LR11XX_RADIO_HAL_LOG_ERROR("Failed to configure LR11xx RF Switch Tx/Rx Selector GPIO as Hi-Z input");
                err = LR11XX_HAL_STATUS_ERROR;
                break;
            }
            sid_err = sid_pal_gpio_set_direction(drv_ctx->config->gpio.fem_tx_rx_mode, SID_PAL_GPIO_DIRECTION_INPUT);
            if (sid_err != SID_ERROR_NONE)
            {
                LR11XX_RADIO_HAL_LOG_ERROR("Failed to configure LR11xx RF Switch Tx/Rx Selector GPIO");
                err = LR11XX_HAL_STATUS_ERROR;
                break;
            }
        }

        /* RF Switch/FEM Tx/Rx selection */
        if (HALO_GPIO_NOT_CONNECTED != drv_ctx->config->gpio.hpa_lpa_sw)
        {
            sid_err = sid_pal_gpio_input_mode(drv_ctx->config->gpio.hpa_lpa_sw, SID_PAL_GPIO_INPUT_DISCONNECT);
            if (sid_err != SID_ERROR_NONE)
            {
                LR11XX_RADIO_HAL_LOG_ERROR("Failed to configure LR11xx LPA-HPA Selector GPIO as Hi-Z input");
                err = LR11XX_HAL_STATUS_ERROR;
                break;
            }
            sid_err = sid_pal_gpio_set_direction(drv_ctx->config->gpio.hpa_lpa_sw, SID_PAL_GPIO_DIRECTION_INPUT);
            if (sid_err != SID_ERROR_NONE)
            {
                LR11XX_RADIO_HAL_LOG_ERROR("Failed to configure LR11xx LPA-HPA Selector GPIO");
                err = LR11XX_HAL_STATUS_ERROR;
                break;
            }
        }

        /* Status LEDs */
#if LR11XX_RADIO_CFG_USE_STATUS_LED
        /* Tx LED */
        if (HALO_GPIO_NOT_CONNECTED != drv_ctx->config->gpio.tx_led)
        {
            sid_err = sid_pal_gpio_input_mode(drv_ctx->config->gpio.tx_led, SID_PAL_GPIO_INPUT_DISCONNECT);
            if (sid_err != SID_ERROR_NONE)
            {
                LR11XX_RADIO_HAL_LOG_ERROR("Failed to configure LR11xx Tx LED GPIO as Hi-Z input");
                err = LR11XX_HAL_STATUS_ERROR;
                break;
            }
            sid_err = sid_pal_gpio_set_direction(drv_ctx->config->gpio.tx_led, SID_PAL_GPIO_DIRECTION_INPUT);
            if (sid_err != SID_ERROR_NONE)
            {
                LR11XX_RADIO_HAL_LOG_ERROR("Failed to configure LR11xx Tx LED GPIO");
                err = LR11XX_HAL_STATUS_ERROR;
                break;
            }
        }

        /* Rx LED */
        if ((HALO_GPIO_NOT_CONNECTED != drv_ctx->config->gpio.rx_led) && (drv_ctx->config->gpio.rx_led != drv_ctx->config->gpio.tx_led))
        {
            sid_err = sid_pal_gpio_input_mode(drv_ctx->config->gpio.rx_led, SID_PAL_GPIO_INPUT_DISCONNECT);
            if (sid_err != SID_ERROR_NONE)
            {
                LR11XX_RADIO_HAL_LOG_ERROR("Failed to configure LR11xx Rx LED GPIO as Hi-Z input");
                err = LR11XX_HAL_STATUS_ERROR;
                break;
            }
            sid_err = sid_pal_gpio_set_direction(drv_ctx->config->gpio.rx_led, SID_PAL_GPIO_DIRECTION_INPUT);
            if (sid_err != SID_ERROR_NONE)
            {
                LR11XX_RADIO_HAL_LOG_ERROR("Failed to configure LR11xx Rx LED GPIO");
                err = LR11XX_HAL_STATUS_ERROR;
                break;
            }
        }

#  if LR11XX_RADIO_CFG_GEOLOCATION_SUPPORT
        /* GNSS scan activity LED */
        if (HALO_GPIO_NOT_CONNECTED != drv_ctx->config->gpio.gnss_scan_led)
        {
            sid_err = sid_pal_gpio_input_mode(drv_ctx->config->gpio.gnss_scan_led, SID_PAL_GPIO_INPUT_DISCONNECT);
            if (sid_err != SID_ERROR_NONE)
            {
                LR11XX_RADIO_HAL_LOG_ERROR("Failed to configure LR11xx GNSS Scan LED GPIO as Hi-Z input");
                err = LR11XX_HAL_STATUS_ERROR;
                break;
            }
            sid_err = sid_pal_gpio_set_direction(drv_ctx->config->gpio.gnss_scan_led, SID_PAL_GPIO_DIRECTION_INPUT);
            if (sid_err != SID_ERROR_NONE)
            {
                LR11XX_RADIO_HAL_LOG_ERROR("Failed to configure LR11xx GNSS Scan LED GPIO");
                err = LR11XX_HAL_STATUS_ERROR;
                break;
            }
        }

        /* WiFi scan activity LED */
        if ((HALO_GPIO_NOT_CONNECTED != drv_ctx->config->gpio.wifi_scan_led) && (drv_ctx->config->gpio.wifi_scan_led != drv_ctx->config->gpio.gnss_scan_led))
        {
            sid_err = sid_pal_gpio_input_mode(drv_ctx->config->gpio.wifi_scan_led, SID_PAL_GPIO_INPUT_DISCONNECT);
            if (sid_err != SID_ERROR_NONE)
            {
                LR11XX_RADIO_HAL_LOG_ERROR("Failed to configure LR11xx WiF Scan LED GPIO as Hi-Z input");
                err = LR11XX_HAL_STATUS_ERROR;
                break;
            }
            sid_err = sid_pal_gpio_set_direction(drv_ctx->config->gpio.wifi_scan_led, SID_PAL_GPIO_DIRECTION_INPUT);
            if (sid_err != SID_ERROR_NONE)
            {
                LR11XX_RADIO_HAL_LOG_ERROR("Failed to configure LR11xx WiFi Scan LED GPIO");
                err = LR11XX_HAL_STATUS_ERROR;
                break;
            }
        }
#  endif /* LR11XX_RADIO_CFG_GEOLOCATION_SUPPORT */
#endif /* LR11XX_RADIO_CFG_USE_STATUS_LED */

        /* Done */
        err = LR11XX_HAL_STATUS_OK;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED lr11xx_hal_status_t lr11xx_hal_disarm_irq(const halo_drv_semtech_ctx_t * const drv_ctx)
{
    lr11xx_hal_status_t err = LR11XX_HAL_STATUS_ERROR;
    sid_error_t sid_err;

    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->config != NULL);

    do
    {
        /* Disable IRQ line */
        sid_err = sid_pal_gpio_irq_disable(drv_ctx->config->gpio.radio_irq);
        if (sid_err != SID_ERROR_NONE)
        {
            LR11XX_RADIO_HAL_LOG_ERROR("Failed to disable LR11xx IRQ in NVIC. Error %d", (int32_t)sid_err);
            err = LR11XX_HAL_STATUS_ERROR;
            break;
        }

        /* Disconnect GPIO pin from IRQ line */
        sid_err = sid_pal_gpio_set_irq(drv_ctx->config->gpio.radio_irq, SID_PAL_GPIO_IRQ_TRIGGER_NONE, NULL, NULL);
        if (sid_err != SID_ERROR_NONE)
        {
            LR11XX_RADIO_HAL_LOG_ERROR("Failed to deinit LR11xx IRQ GPIO. Error %d", (int32_t)sid_err);
            err = LR11XX_HAL_STATUS_ERROR;
            break;
        }

        /* Everything is fine */
        err = LR11XX_HAL_STATUS_OK;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED lr11xx_hal_status_t lr11xx_hal_arm_irq(halo_drv_semtech_ctx_t * const drv_ctx)
{
    lr11xx_hal_status_t err = LR11XX_HAL_STATUS_ERROR;
    sid_error_t sid_err;

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
            SID_PAL_LOG_ERROR("Failed to enable LR11xx IRQ in NVIC, error %d", (int32_t)sid_err);
            err = LR11XX_HAL_STATUS_ERROR;
            break;
        }

#if (__ARM_ARCH_6M__ == 0)
        /* Set elevated priority to LR11xx IRQ line so we can capture radio IRQ timestamp with more precision */
        sid_err = sid_pal_gpio_ext_ifc_set_irq_priority(drv_ctx->config->gpio.radio_irq, drv_ctx->config->gpio.radio_irq_prio_high, 0u);
#else
        /* Set unified priority to LR11xx IRQ line since dynamic IRQ priority changing is not supported */
        sid_err = sid_pal_gpio_ext_ifc_set_irq_priority(drv_ctx->config->gpio.radio_irq, drv_ctx->config->gpio.radio_irq_prio, 0u);
#endif /* (__ARM_ARCH_6M__ == 0) */
        if (sid_err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to set LR11xx IRQ priority, error %d", (int32_t)sid_err);
            err = LR11XX_HAL_STATUS_ERROR;
            break;
        }

        err = LR11XX_HAL_STATUS_OK;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED lr11xx_hal_status_t lr11xx_hal_wait_readiness(const halo_drv_semtech_ctx_t * const drv_ctx)
{
    return lr11xx_wait_on_busy(drv_ctx, LR11XX_WAIT_ON_BUSY_DEFAULT_TIMEOUT_US);
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int16_t lr11xx_radio_hal_get_adjusted_rssi(const halo_drv_semtech_ctx_t * const drv_ctx, const int8_t raw_rssi)
{
    int32_t adjusted_rssi;
    int32_t correction = 0;

#if LR11XX_RADIO_CFG_USE_EXTERNAL_PA
    /* External LNA is used - compensate for its gain */
    correction += drv_ctx->config->pa_config.rx_gain_dbi;
#else
    /* No external LNA is used, account for RF switch losses*/
    correction -= drv_ctx->config->pa_config.rf_sw_insertion_loss;
#endif /* LR11XX_RADIO_CFG_USE_EXTERNAL_PA */
    /* Account for antenna gain */
    correction += drv_ctx->regional_radio_param->ant_dbi;

    adjusted_rssi = ((int32_t)raw_rssi * 100) - correction;

    /* Convert to dB with rounding */
    if (adjusted_rssi >= 0)
    {
        adjusted_rssi = (adjusted_rssi + 50) / 100;

        /* Saturate to the maximum return value */
        if (adjusted_rssi > INT16_MAX)
        {
            adjusted_rssi = INT16_MAX;
        }
    }
    else
    {
        adjusted_rssi = (adjusted_rssi - 50) / 100;

        /* Saturate to the minimum return value */
        if (adjusted_rssi < INT16_MIN)
        {
            adjusted_rssi = INT16_MIN;
        }
    }

    return (int8_t)adjusted_rssi;
}

/*----------------------------------------------------------------------------*/

#if LR11XX_RADIO_CFG_USE_STATUS_LED
SID_STM32_SPEED_OPTIMIZED void lr11xx_radio_hal_tx_led_on(const halo_drv_semtech_ctx_t * const drv_ctx)
{
    if (HALO_GPIO_NOT_CONNECTED != drv_ctx->config->gpio.tx_led)
    {
        uint8_t led_on_pin_state = drv_ctx->config->gpio.tx_led_on_gpio_state != 0u ? 1u : 0u;
        (void)sid_pal_gpio_write(drv_ctx->config->gpio.tx_led, led_on_pin_state);
    }
}
#endif /* LR11XX_RADIO_CFG_USE_STATUS_LED */

/*----------------------------------------------------------------------------*/

#if LR11XX_RADIO_CFG_USE_STATUS_LED
SID_STM32_SPEED_OPTIMIZED void lr11xx_radio_hal_tx_led_off(const halo_drv_semtech_ctx_t * const drv_ctx)
{
    if (HALO_GPIO_NOT_CONNECTED != drv_ctx->config->gpio.tx_led)
    {
        uint8_t led_off_pin_state = drv_ctx->config->gpio.tx_led_on_gpio_state != 0u ? 0u : 1u;
        (void)sid_pal_gpio_write(drv_ctx->config->gpio.tx_led, led_off_pin_state);
    }
}
#endif /* LR11XX_RADIO_CFG_USE_STATUS_LED */

/*----------------------------------------------------------------------------*/

#if LR11XX_RADIO_CFG_USE_STATUS_LED
SID_STM32_SPEED_OPTIMIZED void lr11xx_radio_hal_rx_led_on(const halo_drv_semtech_ctx_t * const drv_ctx)
{
    if (HALO_GPIO_NOT_CONNECTED != drv_ctx->config->gpio.rx_led)
    {
        uint8_t led_on_pin_state = drv_ctx->config->gpio.rx_led_on_gpio_state != 0u ? 1u : 0u;
        (void)sid_pal_gpio_write(drv_ctx->config->gpio.rx_led, led_on_pin_state);
    }
}
#endif /* LR11XX_RADIO_CFG_USE_STATUS_LED */

/*----------------------------------------------------------------------------*/

#if LR11XX_RADIO_CFG_USE_STATUS_LED
SID_STM32_SPEED_OPTIMIZED void lr11xx_radio_hal_rx_led_off(const halo_drv_semtech_ctx_t * const drv_ctx)
{
    if (HALO_GPIO_NOT_CONNECTED != drv_ctx->config->gpio.rx_led)
    {
        uint8_t led_off_pin_state = drv_ctx->config->gpio.rx_led_on_gpio_state != 0u ? 0u : 1u;
        (void)sid_pal_gpio_write(drv_ctx->config->gpio.rx_led, led_off_pin_state);
    }
}
#endif /* LR11XX_RADIO_CFG_USE_STATUS_LED */

/*----------------------------------------------------------------------------*/

#if LR11XX_RADIO_CFG_USE_STATUS_LED && LR11XX_RADIO_CFG_GEOLOCATION_SUPPORT
SID_STM32_SPEED_OPTIMIZED void lr11xx_radio_hal_gnss_scan_led_on(const halo_drv_semtech_ctx_t * const drv_ctx)
{
    if (HALO_GPIO_NOT_CONNECTED != drv_ctx->config->gpio.gnss_scan_led)
    {
        uint8_t led_on_pin_state = drv_ctx->config->gpio.gnss_scan_led_on_gpio_state != 0u ? 1u : 0u;
        (void)sid_pal_gpio_write(drv_ctx->config->gpio.gnss_scan_led, led_on_pin_state);
    }
}
#endif /* LR11XX_RADIO_CFG_USE_STATUS_LED && LR11XX_RADIO_CFG_GEOLOCATION_SUPPORT */

/*----------------------------------------------------------------------------*/

#if LR11XX_RADIO_CFG_USE_STATUS_LED && LR11XX_RADIO_CFG_GEOLOCATION_SUPPORT
SID_STM32_SPEED_OPTIMIZED void lr11xx_radio_hal_gnss_scan_led_off(const halo_drv_semtech_ctx_t * const drv_ctx)
{
    if (HALO_GPIO_NOT_CONNECTED != drv_ctx->config->gpio.gnss_scan_led)
    {
        uint8_t led_off_pin_state = drv_ctx->config->gpio.gnss_scan_led_on_gpio_state != 0u ? 0u : 1u;
        (void)sid_pal_gpio_write(drv_ctx->config->gpio.gnss_scan_led, led_off_pin_state);
    }
}
#endif /* LR11XX_RADIO_CFG_USE_STATUS_LED && LR11XX_RADIO_CFG_GEOLOCATION_SUPPORT */

/*----------------------------------------------------------------------------*/

#if LR11XX_RADIO_CFG_USE_STATUS_LED && LR11XX_RADIO_CFG_GEOLOCATION_SUPPORT
SID_STM32_SPEED_OPTIMIZED void lr11xx_radio_hal_wifi_scan_led_on(const halo_drv_semtech_ctx_t * const drv_ctx)
{
    if (HALO_GPIO_NOT_CONNECTED != drv_ctx->config->gpio.wifi_scan_led)
    {
        uint8_t led_on_pin_state = drv_ctx->config->gpio.wifi_scan_led_on_gpio_state != 0u ? 1u : 0u;
        (void)sid_pal_gpio_write(drv_ctx->config->gpio.wifi_scan_led, led_on_pin_state);
    }
}
#endif /* LR11XX_RADIO_CFG_USE_STATUS_LED && LR11XX_RADIO_CFG_GEOLOCATION_SUPPORT */

/*----------------------------------------------------------------------------*/

#if LR11XX_RADIO_CFG_USE_STATUS_LED && LR11XX_RADIO_CFG_GEOLOCATION_SUPPORT
SID_STM32_SPEED_OPTIMIZED void lr11xx_radio_hal_wifi_scan_led_off(const halo_drv_semtech_ctx_t * const drv_ctx)
{
    if (HALO_GPIO_NOT_CONNECTED != drv_ctx->config->gpio.wifi_scan_led)
    {
        uint8_t led_off_pin_state = drv_ctx->config->gpio.wifi_scan_led_on_gpio_state != 0u ? 0u : 1u;
        (void)sid_pal_gpio_write(drv_ctx->config->gpio.wifi_scan_led, led_off_pin_state);
    }
}
#endif /* LR11XX_RADIO_CFG_USE_STATUS_LED && LR11XX_RADIO_CFG_GEOLOCATION_SUPPORT */
