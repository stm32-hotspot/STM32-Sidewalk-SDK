/**
  ******************************************************************************
  * @file    stm32wlxx_radio.c
  * @brief   Handling of the radio driver app running on STM32WLxx MCU
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

#include <stdbool.h>

#include <assert.h>
#include <cmsis_compiler.h>
#include "stm32_rtos.h"

#include "stm32wlxx_radio.h"
#include "stm32wlxx_radio_hal.h"

#include <sid_error.h>
#include <sid_time_ops.h>
#include <sid_time_types.h>

#include <sid_pal_assert_ifc.h>
#include <sid_pal_critical_region_ifc.h>
#include <sid_pal_delay_ifc.h>
#include <sid_pal_log_ifc.h>
#include <sid_pal_uptime_ifc.h>

#include <sid_lora_phy_cfg.h>
#include <sid_fsk_phy_cfg.h>

#include <comm_opcodes.h>

#include <sid_stm32_common_utils.h>

#if SID_SDK_CONFIG_RF_TIMINGS_MEASUREMENT
#  include "main.h"
#endif

/* LPM handling */
#include "app_conf.h"
#if (CFG_LPM_LEVEL != 0)
#  include <stm32_lpm.h>
#endif /* (CFG_LPM_LEVEL != 0) */

/* Private defines -----------------------------------------------------------*/

#define STM32WLxx_HANDSHAKE_ATTEMPTS_LIMIT           (5u)   /*!< Defines how many times in a row the driver will attempt to do a Handshake with the STM32WLxx side before the handshake error is reported */

#define STM32WLxx_MIN_CHANNEL_FREE_DELAY_US          (1u)
#define STM32WLxx_MIN_CHANNEL_NOISE_DELAY_US         (30u)
#define STM32WLxx_NOISE_SAMPLE_SIZE                  (32u)

#define STM32WLxx_UDT_TASK_STACK_SIZE                (768u)
#define STM32WLxx_UDT_TASK_PRIO                      ((osPriority_t) osPriorityRealtime7)
#define STM32WLxx_UDT_OUT_MSG_QUEUE_LEN              (10u)
#define STM32WLxx_UDT_SUSPEND_RESERVED_TIME_US       (5000u) /*!< Amount of time (in us) reserved before the planned radio wakeup event - this safety gap is used to suspend UDT before waking up the radio */

#define STM32WLxx_ANT_GAIN_CAP_REGION_NA             ((int32_t)(6 * 100)) /* Antenna gain (in dBi * 100) upper limit as per FCC requirements */

#define STM32WLxx_PA_CFG_PARAM_INVALID               (0xFFu) /*!< Indicates that PA config parameter is invalid */
#define STM32WLxx_PA_CFG_USE_LPA                     (0x01u) /*!< Use Low-Power Amplifier for Tx */
#define STM32WLxx_PA_CFG_USE_HPA                     (0x00u) /*!< Use High-Power Amplifier for Tx */
#define STM32WLxx_PA_CFG_LPA_OPTIMIZATION_14_DBM     (14)    /*!< LPA Tx power level for which an optimized PA config exists */
#define STM32WLxx_PA_CFG_LPA_OPTIMIZATION_10_DBM     (10)    /*!< LPA Tx power level for which an optimized PA config exists */
#define STM32WLxx_PA_CFG_HPA_OPTIMIZATION_20_DBM     (20)    /*!< HPA Tx power level for which an optimized PA config exists */
#define STM32WLxx_PA_CFG_HPA_OPTIMIZATION_17_DBM     (17)    /*!< HPA Tx power level for which an optimized PA config exists */
#define STM32WLxx_PA_CFG_HPA_OPTIMIZATION_14_DBM     (14)    /*!< HPA Tx power level for which an optimized PA config exists */


#define STM32WLxx_RADIO_ERROR_LIMIT                  (3u)    /*!< If the number of errors in a burst exceeds this limit the driver will reset the radio */
#define STM32WLxx_RADIO_ERROR_ASYM_INC_FACTOR        (2u)    /*!< Error counter will be incremented this times faster (and decremented always by 1). This asymmetric growth protects against the intermittent errors */

#define STM32WLxx_RADIO_LORA_UL_SEPARATION_US        (367000u) /*!< Uplink slot separation time for LoRa link */
#define STM32WLxx_RADIO_LORA_RX_TIMEOUT_US           (35000u) /*!< Timeout for LoRa Rx */

/* Private macros ------------------------------------------------------------*/

#ifndef STM32WLxx_RADIO_EXTRA_LOGGING
/* Set STM32WLxx_RADIO_EXTRA_LOGGING to 1 to enable extended logs */
#  define STM32WLxx_RADIO_EXTRA_LOGGING (0)
#endif

#if STM32WLxx_RADIO_EXTRA_LOGGING
#  define STM32WLxx_LOG_ERROR(...)   SID_PAL_LOG_ERROR(__VA_ARGS__)
#  define STM32WLxx_LOG_WARNING(...) SID_PAL_LOG_WARNING(__VA_ARGS__)
#  define STM32WLxx_LOG_INFO(...)    SID_PAL_LOG_INFO(__VA_ARGS__)
#  define STM32WLxx_LOG_DEBUG(...)   SID_PAL_LOG_DEBUG(__VA_ARGS__)
#  define STM32WLxx_LOG_TRACE(...)   SID_PAL_LOG_TRACE(__VA_ARGS__)
#else
#  define STM32WLxx_LOG_ERROR(...)   ((void)0u)
#  define STM32WLxx_LOG_WARNING(...) ((void)0u)
#  define STM32WLxx_LOG_INFO(...)    ((void)0u)
#  define STM32WLxx_LOG_DEBUG(...)   ((void)0u)
#  define STM32WLxx_LOG_TRACE(...)   ((void)0u)
#endif

/** This Macro implements compilation time comparison of types of two given variables.
 *  Reverse is done because standard implementation have problems comparing constant types in such a way
 */
#define STM32WLxx_COMPARE_CONST_TYPES(A,B) (_Generic( (A) , typeof(B): true, default: false) || _Generic( (B) , typeof(A): true, default: false))

/** This macro copies B to A comparing types during compilation in advance to avoid unexpected types change
 */
#define STM32WLxx_COPY_CFG_PARAM(A,B) static_assert(STM32WLxx_COMPARE_CONST_TYPES(A,B), "Types of copied fields should be same");\
                                      static_assert(sizeof(A)==sizeof(B), "Sizes of copied fields should be same");\
                                      (A)=(B);

/** This macro copies B to A comparing sizes during compilation in advance to avoid unexpected type size change
 *  Usage of this macro should be avoided. STM32WLxx_COPY_CFG_PARAM should be used instead
 */
#define STM32WLxx_COPY_CFG_PARAM_NO_TYPE_COMPARISON(A,B) static_assert(sizeof(A)==sizeof(B), "Sizes of copied fields should be same");\
                                                         SID_STM32_UTIL_fast_memcpy(A,B,sizeof(B));

/** This macro copies array B to A comparing types during compilation in advance to avoid unexpected types change
 */
#define STM32WLxx_COPY_CFG_ARRAY_PARAM(A,B) static_assert(STM32WLxx_COMPARE_CONST_TYPES(A[0],B[0]), "Types of copied fields should be same");\
                                            static_assert(sizeof(A)==sizeof(B), "Sizes of copied fields should be same");\
                                            SID_STM32_UTIL_fast_memcpy(A,B,sizeof(B));

/* Compare radio_fsk_fcs_t against stm32wlxx_radio_fsk_fcs_t */
static_assert((uint32_t)RADIO_FSK_FCS_TYPE_0 == (uint32_t)STM32WLxx_RADIO_FSK_FCS_TYPE_0);
static_assert((uint32_t)RADIO_FSK_FCS_TYPE_1 == (uint32_t)STM32WLxx_RADIO_FSK_FCS_TYPE_1);

/* Compare sid_pal_radio_events_t against stm32wlxx_pal_radio_events_t */
static_assert((uint32_t)SID_PAL_RADIO_EVENT_UNKNOWN      == (uint32_t)STM32WLxx_PAL_RADIO_EVENT_UNKNOWN);
static_assert((uint32_t)SID_PAL_RADIO_EVENT_TX_DONE      == (uint32_t)STM32WLxx_PAL_RADIO_EVENT_TX_DONE);
static_assert((uint32_t)SID_PAL_RADIO_EVENT_RX_DONE      == (uint32_t)STM32WLxx_PAL_RADIO_EVENT_RX_DONE);
static_assert((uint32_t)SID_PAL_RADIO_EVENT_CAD_DONE     == (uint32_t)STM32WLxx_PAL_RADIO_EVENT_CAD_DONE);
static_assert((uint32_t)SID_PAL_RADIO_EVENT_CAD_TIMEOUT  == (uint32_t)STM32WLxx_PAL_RADIO_EVENT_CAD_TIMEOUT);
static_assert((uint32_t)SID_PAL_RADIO_EVENT_RX_ERROR     == (uint32_t)STM32WLxx_PAL_RADIO_EVENT_RX_ERROR);
static_assert((uint32_t)SID_PAL_RADIO_EVENT_TX_TIMEOUT   == (uint32_t)STM32WLxx_PAL_RADIO_EVENT_TX_TIMEOUT);
static_assert((uint32_t)SID_PAL_RADIO_EVENT_RX_TIMEOUT   == (uint32_t)STM32WLxx_PAL_RADIO_EVENT_RX_TIMEOUT);
static_assert((uint32_t)SID_PAL_RADIO_EVENT_CS_DONE      == (uint32_t)STM32WLxx_PAL_RADIO_EVENT_CS_DONE);
static_assert((uint32_t)SID_PAL_RADIO_EVENT_CS_TIMEOUT   == (uint32_t)STM32WLxx_PAL_RADIO_EVENT_CS_TIMEOUT);
static_assert((uint32_t)SID_PAL_RADIO_EVENT_HEADER_ERROR == (uint32_t)STM32WLxx_PAL_RADIO_EVENT_HEADER_ERROR);
static_assert((uint32_t)SID_PAL_RADIO_EVENT_SYNC_DET     == (uint32_t)STM32WLxx_PAL_RADIO_EVENT_SYNC_DET);

/* Compare sid_pal_radio_cad_param_exit_mode_t against stm32wlxx_pal_radio_cad_param_exit_mode_t */
static_assert((uint32_t)SID_PAL_RADIO_CAD_EXIT_MODE_CS_ONLY == (uint32_t)STM32WLxx_RADIO_CAD_EXIT_MODE_CS_ONLY);
static_assert((uint32_t)SID_PAL_RADIO_CAD_EXIT_MODE_CS_RX   == (uint32_t)STM32WLxx_RADIO_CAD_EXIT_MODE_CS_RX);
static_assert((uint32_t)SID_PAL_RADIO_CAD_EXIT_MODE_CS_LBT  == (uint32_t)STM32WLxx_RADIO_CAD_EXIT_MODE_CS_LBT);
static_assert((uint32_t)SID_PAL_RADIO_CAD_EXIT_MODE_ED_ONLY == (uint32_t)STM32WLxx_RADIO_CAD_EXIT_MODE_ED_ONLY);
static_assert((uint32_t)SID_PAL_RADIO_CAD_EXIT_MODE_ED_RX   == (uint32_t)STM32WLxx_RADIO_CAD_EXIT_MODE_ED_RX);
static_assert((uint32_t)SID_PAL_RADIO_CAD_EXIT_MODE_ED_LBT  == (uint32_t)STM32WLxx_RADIO_CAD_EXIT_MODE_ED_LBT);
static_assert((uint32_t)SID_PAL_RADIO_CAD_EXIT_MODE_NONE    == (uint32_t)STM32WLxx_RADIO_CAD_EXIT_MODE_NONE);

/* Compare sid_pal_radio_lora_crc_present against stm32wlxx_pal_radio_lora_crc_present_t */
static_assert((uint32_t)SID_PAL_RADIO_CRC_PRESENT_INVALID == (uint32_t)STM32WLxx_PAL_RADIO_CRC_PRESENT_INVALID);
static_assert((uint32_t)SID_PAL_RADIO_CRC_PRESENT_OFF     == (uint32_t)STM32WLxx_PAL_RADIO_CRC_PRESENT_OFF);
static_assert((uint32_t)SID_PAL_RADIO_CRC_PRESENT_ON      == (uint32_t)STM32WLxx_PAL_RADIO_CRC_PRESENT_ON);
static_assert((uint32_t)SID_PAL_RADIO_CRC_PRESENT_MAX_NUM == (uint32_t)STM32WLxx_PAL_RADIO_CRC_PRESENT_MAX_NUM);

/* Private variables ---------------------------------------------------------*/

static halo_drv_stm32wlxx_ctx_t drv_ctx = {0};

/* Private constants ---------------------------------------------------------*/

static const osSemaphoreAttr_t wlxx_request_ack_lock_attributes = {
    .name       = "STM32WLxx Req Ack Semaphore",
    .attr_bits  = SEMAPHORE_DEFAULT_ATTR_BITS,
    .cb_mem     = SEMAPHORE_DEFAULT_CB_MEM,
    .cb_size    = SEMAPHORE_DEFAULT_CB_SIZE,
};

static const osSemaphoreAttr_t wlxx_irq_wait_lock_attributes = {
    .name       = "STM32WLxx IRQ Wait Semaphore",
    .attr_bits  = SEMAPHORE_DEFAULT_ATTR_BITS,
    .cb_mem     = SEMAPHORE_DEFAULT_CB_MEM,
    .cb_size    = SEMAPHORE_DEFAULT_CB_SIZE,
};

#if STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER
static const osThreadAttr_t wlxx_user_data_transfer_thread_attributes = {
    .name       = "STM32WLxx UDT Task",
    .priority   = STM32WLxx_UDT_TASK_PRIO,
    .stack_size = STM32WLxx_UDT_TASK_STACK_SIZE,
    .attr_bits  = TASK_DEFAULT_ATTR_BITS,
    .cb_mem     = TASK_DEFAULT_CB_MEM,
    .cb_size    = TASK_DEFAULT_CB_SIZE,
    .stack_mem  = TASK_DEFAULT_STACK_MEM,
};
#endif /* STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER */

/* Private function prototypes -----------------------------------------------*/

static inline int32_t radio_stm32wlxx_platform_init(void);
static inline int32_t radio_stm32wlxx_platform_deinit(void);
static        int32_t radio_stm32wlxx_do_handshake(void);

static inline void    radio_stm32wlxx_pack_settings(const sid_pal_radio_fsk_phy_settings_t * const fsk_phy_cfg,
                                                    const sid_pal_radio_lora_phy_settings_t * const lora_phy_cfg,
                                                    stm32wlxx_radio_phy_cfg_t * const out_cfg);

static inline int32_t radio_stm32wlxx_set_tx_power_config(const int8_t power);
static inline void    radio_stm32wlxx_error_manager(const int32_t reported_err, const uint32_t set_radio_sleep);

#if STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER
static void _user_data_transfer_thread(void * context);
#endif /* STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER */

/* Private function definitions ----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline int32_t radio_stm32wlxx_platform_init(void)
{
    int32_t err = RADIO_ERROR_GENERIC;
    sid_error_t sid_err;
    stm32wlxx_hal_status_t hal_err;

    STM32WLxx_LOG_DEBUG("radio_stm32wlxx_platform_init");

    do
    {
        /* Validate inputs */
        if (NULL == drv_ctx.config->pa_cfg_callback)
        {
            STM32WLxx_LOG_ERROR("radio_stm32wlxx_platform_init - null pa_cfg_callback!");
            err = RADIO_ERROR_INVALID_PARAMS;
            break;
        }

        /* Configure essential GPIO pins */
        hal_err = stm32wlxx_radio_hal_init_gpio(&drv_ctx);
        if (hal_err != STM32WLxx_HAL_STATUS_OK)
        {
            STM32WLxx_LOG_ERROR("radio_stm32wlxx_platform_init - failed to configure essential GPIO. HAL error %u", hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Bring up SPI bus */
        sid_err = drv_ctx.config->bus_factory->create(&drv_ctx.bus_iface, drv_ctx.config->bus_factory->config);
        if ((sid_err != SID_ERROR_NONE) || (NULL == drv_ctx.bus_iface))
        {
            STM32WLxx_LOG_ERROR("radio_stm32wlxx_platform_init - error in bus_factory->create(). Error %d, iface: 0x%x", (int32_t)sid_err, drv_ctx.bus_iface);
            err = RADIO_ERROR_IO_ERROR;
            break;
        }

        /* Initialization is competed */
        STM32WLxx_LOG_DEBUG("radio_stm32wlxx_platform_init - done");
        err = RADIO_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline int32_t radio_stm32wlxx_platform_deinit(void)
{
    int32_t err = RADIO_ERROR_GENERIC;
    stm32wlxx_hal_status_t hal_err;
    sid_error_t sid_err;

    STM32WLxx_LOG_DEBUG("radio_stm32wlxx_platform_deinit");

    do
    {
        /* Deactivate IRQ pin */
        hal_err = stm32wlxx_hal_disarm_irq(&drv_ctx);
        if (hal_err != STM32WLxx_HAL_STATUS_OK)
        {
            STM32WLxx_LOG_ERROR("radio_stm32wlxx_platform_deinit - error in stm32wlxx_hal_disarm_irq(). Error %u", (uint32_t)hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Bring down SPI bus */
        const struct sid_pal_serial_bus_iface * const spi_bus_iface = drv_ctx.bus_iface;
        if (spi_bus_iface != NULL)
        {
            if (NULL == spi_bus_iface->destroy)
            {
                STM32WLxx_LOG_WARNING("radio_stm32wlxx_platform_deinit - spi_bus_iface has no destroy() method");
                err = RADIO_ERROR_NOT_SUPPORTED;
                break;
            }

            sid_err = spi_bus_iface->destroy(spi_bus_iface);
            if (sid_err != SID_ERROR_NONE)
            {
                STM32WLxx_LOG_ERROR("radio_stm32wlxx_platform_deinit - error in spi_bus_iface->destroy(). Error %d", (int32_t)sid_err);
                err = RADIO_ERROR_IO_ERROR;
                break;
            }
        }
        else
        {
            SID_PAL_LOG_WARNING("radio_stm32wlxx_platform_deinit - SPI bus interface is null, bus deinitialization skipped");
        }

        /* Release GPIO pins */
        hal_err = stm32wlxx_radio_hal_deinit_gpio(&drv_ctx);
        if (hal_err != STM32WLxx_HAL_STATUS_OK)
        {
            STM32WLxx_LOG_ERROR("radio_stm32wlxx_platform_deinit - failed to release GPIO. HAL error %u", hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Deinitialization is competed */
        STM32WLxx_LOG_DEBUG("radio_stm32wlxx_platform_deinit - done");
        err = RADIO_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static int32_t radio_stm32wlxx_do_handshake(void)
{
    uint32_t err = RADIO_ERROR_GENERIC;
    stm32wlxx_hal_status_t hal_err;
    stm32wlxx_rcp_irq_status_t irq_status;

    do
    {
        /* Perform GPIO handshake request */
        hal_err = stm32wlxx_hal_gpio_handshake_request(&drv_ctx);

        if (hal_err != STM32WLxx_HAL_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("STM32WLxx GPIO handshaking failed. HAL error %d", (int32_t)hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        STM32WLxx_LOG_INFO("Successful GPIO handshake with STM32WLxx, proceeding with SPI handshake...");

        /* If we got here the IRQ pin is actively driven low by an external source - we can send a handshake message via SPI now */
        hal_err = stm32wlxx_hal_retrieve_radio_irq(&drv_ctx,  &irq_status, NULL, 0u);

        if (hal_err != STM32WLxx_HAL_STATUS_OK)
        {
            STM32WLxx_LOG_ERROR("Unable to retrieve STM32WLxx IRQ status. HAL error %u", (uint32_t)hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
        }
        else
        {
            err = RADIO_ERROR_NONE;
            STM32WLxx_LOG_DEBUG("Retrieved STM32WLxx IRQ status. IRQ flags: 0x%x, followup data size: %u", irq_status.irq_flags, irq_status.followup_payload_len);
        }

        /* Send out IRQ acknowledgment to clear the IRQ indication by the STM32WLxx side - this is a mandatory step regardless of the valid or invalid IRQ indication */
        hal_err = stm32wlxx_hal_acknowledge_radio_irq(&drv_ctx);
        if (hal_err != STM32WLxx_HAL_STATUS_OK)
        {
            STM32WLxx_LOG_ERROR("Unable to acknowledge STM32WLxx IRQ status. HAL error %u", (uint32_t)hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        hal_err = stm32wlxx_hal_wait_radio_irq_released(&drv_ctx, STM32WLxx_HAL_RADIO_IRQ_RELEASE_WAIT_TIME_US);
        if (hal_err != STM32WLxx_HAL_STATUS_OK)
        {
            STM32WLxx_LOG_ERROR("STM32WLxx failed to release the IRQ line after successful ACK. HAL error %u", (uint32_t)hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Check that we have received a valid Handshake IRQ */
        if ((irq_status.irq_flags != STM32WLxx_APP_IRQ_SPI_HANDSHAKE) || (irq_status.followup_payload_len != 0u))
        {
            STM32WLxx_LOG_ERROR("STM32WLxx SPI Handshaking failed. STM32WLxx IRQ does not indicate the desired Handshake status. IRQ flags: 0x%x", irq_status.irq_flags);
            err = RADIO_ERROR_INVALID_STATE;
            break;
        }
        STM32WLxx_LOG_INFO("Successful SPI handshake with STM32WLxx");

        /* Do the protocol compatibility checks */
        const stm32wlxx_version_info_t * const protocol_version = &irq_status.handshake_details.protocol_version;
        const stm32wlxx_version_info_t * const app_version      = &irq_status.handshake_details.app_version;
        SID_PAL_LOG_INFO("Detected STM32WLxx with application version %u.%u.%u", app_version->major, app_version->minor, app_version->patch);

        if ((protocol_version->major != STM32WLxx_RADIO_COMM_PROTOCOL_MAJOR_VERSION)
          || (protocol_version->minor != STM32WLxx_RADIO_COMM_PROTOCOL_MINOR_VERSION)
          || (protocol_version->patch != STM32WLxx_RADIO_COMM_PROTOCOL_PATCH_VERSION))
        {
            SID_PAL_LOG_ERROR("STM32WLxx implements incompatible SPI protocol version: %u.%u.%u vs %u.%u.%u",
                              protocol_version->major, protocol_version->minor, protocol_version->patch,
                              STM32WLxx_RADIO_COMM_PROTOCOL_MAJOR_VERSION, STM32WLxx_RADIO_COMM_PROTOCOL_MINOR_VERSION, STM32WLxx_RADIO_COMM_PROTOCOL_PATCH_VERSION);
            err = RADIO_ERROR_NOT_SUPPORTED;
            break;
        }

        SID_PAL_LOG_INFO("Reported STM32WLxx SPI protocol version: %u.%u.%u", protocol_version->major, protocol_version->minor, protocol_version->patch);

        /* Handshake succeeded */
        STM32WLxx_LOG_DEBUG("Handshake procedure succeeded");
        err = RADIO_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline void radio_stm32wlxx_pack_settings(const sid_pal_radio_fsk_phy_settings_t * const fsk_phy_cfg,
                                                                           const sid_pal_radio_lora_phy_settings_t * const lora_phy_cfg,
                                                                           stm32wlxx_radio_phy_cfg_t * const out_cfg)
{
    /*---------------------------------- FSK -------------------------------------*/
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_2
    STM32WLxx_COPY_CFG_PARAM(out_cfg->fsk_cfg.freq ,                             fsk_phy_cfg->freq);
    STM32WLxx_COPY_CFG_PARAM(out_cfg->fsk_cfg.power,                             fsk_phy_cfg->power);
    STM32WLxx_COPY_CFG_ARRAY_PARAM(out_cfg->fsk_cfg.sync_word,                   fsk_phy_cfg->sync_word);
    STM32WLxx_COPY_CFG_PARAM(out_cfg->fsk_cfg.sync_word_len,                     fsk_phy_cfg->sync_word_len);
    STM32WLxx_COPY_CFG_PARAM(out_cfg->fsk_cfg.whitening_seed,                    fsk_phy_cfg->whitening_seed);
    STM32WLxx_COPY_CFG_PARAM(out_cfg->fsk_cfg.tx_timeout,                        fsk_phy_cfg->tx_timeout);
    STM32WLxx_COPY_CFG_PARAM(out_cfg->fsk_cfg.symbol_timeout,                    fsk_phy_cfg->symbol_timeout);

    /* fsk_modulation_params */
    STM32WLxx_COPY_CFG_PARAM(out_cfg->fsk_cfg.mod.bit_rate,                      fsk_phy_cfg->fsk_modulation_params.bit_rate);
    STM32WLxx_COPY_CFG_PARAM(out_cfg->fsk_cfg.mod.freq_dev,                      fsk_phy_cfg->fsk_modulation_params.freq_dev);
    STM32WLxx_COPY_CFG_PARAM(out_cfg->fsk_cfg.mod.shaping,                       fsk_phy_cfg->fsk_modulation_params.mod_shaping);
    STM32WLxx_COPY_CFG_PARAM(out_cfg->fsk_cfg.mod.bandwidth,                     fsk_phy_cfg->fsk_modulation_params.bandwidth);

    /* fsk_packet_params */
    STM32WLxx_COPY_CFG_PARAM(out_cfg->fsk_cfg.pkt.preamble_length,               fsk_phy_cfg->fsk_packet_params.preamble_length);
    STM32WLxx_COPY_CFG_PARAM(out_cfg->fsk_cfg.pkt.preamble_min_detect,           fsk_phy_cfg->fsk_packet_params.preamble_min_detect);
    STM32WLxx_COPY_CFG_PARAM(out_cfg->fsk_cfg.pkt.sync_word_length,              fsk_phy_cfg->fsk_packet_params.sync_word_length);
    STM32WLxx_COPY_CFG_PARAM(out_cfg->fsk_cfg.pkt.addr_comp,                     fsk_phy_cfg->fsk_packet_params.addr_comp);
    STM32WLxx_COPY_CFG_PARAM(out_cfg->fsk_cfg.pkt.header_type,                   fsk_phy_cfg->fsk_packet_params.header_type);
    STM32WLxx_COPY_CFG_PARAM(out_cfg->fsk_cfg.pkt.crc_type,                      fsk_phy_cfg->fsk_packet_params.crc_type);
    STM32WLxx_COPY_CFG_PARAM(out_cfg->fsk_cfg.pkt.radio_whitening_mode,          fsk_phy_cfg->fsk_packet_params.radio_whitening_mode);

    /* fsk_cad_params */
    STM32WLxx_COPY_CFG_PARAM(out_cfg->fsk_cfg.fsk_ed_rssi_threshold,             fsk_phy_cfg->fsk_cad_params.fsk_ed_rssi_threshold);
    STM32WLxx_COPY_CFG_PARAM(out_cfg->fsk_cfg.fsk_ed_duration_mus,               fsk_phy_cfg->fsk_cad_params.fsk_ed_duration_mus);
    STM32WLxx_COPY_CFG_PARAM(out_cfg->fsk_cfg.fsk_cs_min_prm_det,                fsk_phy_cfg->fsk_cad_params.fsk_cs_min_prm_det);
    STM32WLxx_COPY_CFG_PARAM(out_cfg->fsk_cfg.fsk_cs_duration_us,                fsk_phy_cfg->fsk_cad_params.fsk_cs_duration_us);

    /* fsk_phy_hdr */
    STM32WLxx_COPY_CFG_PARAM(out_cfg->fsk_cfg.phy_hdr_fcs_type,                  (stm32wlxx_radio_fsk_fcs_t)fsk_phy_cfg->fsk_phy_hdr.fcs_type);
    STM32WLxx_COPY_CFG_PARAM(out_cfg->fsk_cfg.phy_hdr_is_data_whitening_enabled, fsk_phy_cfg->fsk_phy_hdr.is_data_whitening_enabled);
    STM32WLxx_COPY_CFG_PARAM(out_cfg->fsk_cfg.phy_hdr_is_fec_enabled,            fsk_phy_cfg->fsk_phy_hdr.is_fec_enabled);
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 */
    /*----------------------------------------------------------------------------*/

    /*---------------------------------- LoRa ------------------------------------*/
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
    STM32WLxx_COPY_CFG_PARAM(out_cfg->lora_cfg.freq,                             lora_phy_cfg->freq);
    STM32WLxx_COPY_CFG_PARAM(out_cfg->lora_cfg.power,                            lora_phy_cfg->power);
    STM32WLxx_COPY_CFG_PARAM(out_cfg->lora_cfg.sync_word,                        lora_phy_cfg->sync_word);
    STM32WLxx_COPY_CFG_PARAM(out_cfg->lora_cfg.symbol_timeout,                   lora_phy_cfg->symbol_timeout);
    STM32WLxx_COPY_CFG_PARAM(out_cfg->lora_cfg.tx_timeout,                       lora_phy_cfg->tx_timeout);
    STM32WLxx_COPY_CFG_PARAM(out_cfg->lora_cfg.lora_ldr_long_interleaved_enable, lora_phy_cfg->lora_ldr_long_interleaved_enable);

    /* sid_pal_radio_lora_modulation_params */
    STM32WLxx_COPY_CFG_PARAM(out_cfg->lora_cfg.mod.spreading_factor,             lora_phy_cfg->lora_modulation_params.spreading_factor);
    STM32WLxx_COPY_CFG_PARAM(out_cfg->lora_cfg.mod.bandwidth,                    lora_phy_cfg->lora_modulation_params.bandwidth);
    STM32WLxx_COPY_CFG_PARAM(out_cfg->lora_cfg.mod.coding_rate,                  lora_phy_cfg->lora_modulation_params.coding_rate);

    /* sid_pal_radio_lora_packet_params */
    STM32WLxx_COPY_CFG_PARAM(out_cfg->lora_cfg.pkt.preamble_length,              lora_phy_cfg->lora_packet_params.preamble_length);
    STM32WLxx_COPY_CFG_PARAM(out_cfg->lora_cfg.pkt.header_type,                  lora_phy_cfg->lora_packet_params.header_type);
    STM32WLxx_COPY_CFG_PARAM(out_cfg->lora_cfg.pkt.payload_length,               lora_phy_cfg->lora_packet_params.payload_length);
    STM32WLxx_COPY_CFG_PARAM(out_cfg->lora_cfg.pkt.crc_mode,                     lora_phy_cfg->lora_packet_params.crc_mode);
    STM32WLxx_COPY_CFG_PARAM(out_cfg->lora_cfg.pkt.invert_IQ,                    lora_phy_cfg->lora_packet_params.invert_IQ);

    /* radio_lora_cad_params */
    STM32WLxx_COPY_CFG_PARAM(out_cfg->lora_cfg.cad_symbol_num,                   lora_phy_cfg->lora_cad_params.cad_symbol_num);
    STM32WLxx_COPY_CFG_PARAM(out_cfg->lora_cfg.cad_detect_peak,                  lora_phy_cfg->lora_cad_params.cad_detect_peak);
    STM32WLxx_COPY_CFG_PARAM(out_cfg->lora_cfg.cad_detect_min,                   lora_phy_cfg->lora_cad_params.cad_detect_min);
    STM32WLxx_COPY_CFG_PARAM(out_cfg->lora_cfg.cad_exit_mode,                    lora_phy_cfg->lora_cad_params.cad_exit_mode);
    STM32WLxx_COPY_CFG_PARAM(out_cfg->lora_cfg.cad_timeout,                      lora_phy_cfg->lora_cad_params.cad_timeout);
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 */
    /*----------------------------------------------------------------------------*/

    /* Common driver setup -------------------------------------------------------*/
    STM32WLxx_COPY_CFG_PARAM(out_cfg->drv_cfg.irq_mask,                          drv_ctx.subghz_irq_mask);
    STM32WLxx_COPY_CFG_PARAM(out_cfg->drv_cfg.rx_boost,                          drv_ctx.config->rx_boost);
    STM32WLxx_COPY_CFG_PARAM(out_cfg->drv_cfg.cad_exit_mode,                     (stm32wlxx_pal_radio_cad_param_exit_mode_t)drv_ctx.cad_exit_mode);
    STM32WLxx_COPY_CFG_PARAM(out_cfg->drv_cfg.lna_gain,                          drv_ctx.config->lna_gain);
    /*----------------------------------------------------------------------------*/

    /* Power amplifier config ----------------------------------------------------*/
    STM32WLxx_COPY_CFG_PARAM(out_cfg->pa_cfg.pa_duty_cycle,                      drv_ctx.pa_cfg.pa_duty_cycle);
    STM32WLxx_COPY_CFG_PARAM(out_cfg->pa_cfg.hp_max,                             drv_ctx.pa_cfg.hp_max);
    STM32WLxx_COPY_CFG_PARAM(out_cfg->pa_cfg.device_sel,                         drv_ctx.pa_cfg.device_sel);
    STM32WLxx_COPY_CFG_PARAM(out_cfg->pa_cfg.pa_lut,                             drv_ctx.pa_cfg.pa_lut);
    STM32WLxx_COPY_CFG_PARAM(out_cfg->pa_cfg.tx_power_reg,                       drv_ctx.pa_cfg.tx_power_reg);
    STM32WLxx_COPY_CFG_PARAM(out_cfg->pa_cfg.ramp_time,                          drv_ctx.pa_cfg.ramp_time);
    STM32WLxx_COPY_CFG_PARAM(out_cfg->pa_cfg.target_tx_power,                    drv_ctx.pa_cfg.target_tx_power);
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline int32_t radio_stm32wlxx_set_tx_power_config(const int8_t power)
{
    SID_PAL_ASSERT(drv_ctx.regional_radio_param != NULL);

    if (drv_ctx.radio_state != SID_PAL_RADIO_STANDBY)
    {
        return RADIO_ERROR_INVALID_STATE;
    }

#if HALO_ENABLE_DIAGNOSTICS
    if (false == drv_ctx.pa_cfg_configured)
#endif
    {
        /* Compute the target Tx power taking into account the antenna gain */
        int32_t target_pwr = power;
        switch (drv_ctx.regional_radio_param->param_region)
        {
            case RADIO_REGION_NA:
            if (drv_ctx.regional_radio_param->ant_dbi > STM32WLxx_ANT_GAIN_CAP_REGION_NA)
            {
                /* As per the FCC requirements PA gain shall be reduced by the difference between antenna gain and the antenna gain cap value */
                target_pwr -= ((int32_t)drv_ctx.regional_radio_param->ant_dbi - STM32WLxx_ANT_GAIN_CAP_REGION_NA + 50) / 100;
            }
            break;

        case RADIO_REGION_EU:
            /* EU requirements put the limit on ERP - PA gain shall be reduced by antenna gain */
            target_pwr -= ((int32_t)drv_ctx.regional_radio_param->ant_dbi + 50) / 100;
            break;

        case RADIO_REGION_NONE:
        default:
            /* Apply PA gain as is */
            break;
        }

        /* Fill-in drv_ctx.pa_cfg with invalid values */
        SID_STM32_UTIL_fast_memset(&drv_ctx.pa_cfg, STM32WLxx_PA_CFG_PARAM_INVALID, sizeof(drv_ctx.pa_cfg));

        /* Call app callback to populate PA config - this allows the app to override the default values */
        if (drv_ctx.config->pa_cfg_callback(target_pwr, drv_ctx.regional_radio_param, &drv_ctx.pa_cfg) != RADIO_ERROR_NONE)
        {
            return RADIO_ERROR_INVALID_PARAMS;
        }

        /* Check if the callback applied custom PA config */
        if ((STM32WLxx_PA_CFG_PARAM_INVALID == drv_ctx.pa_cfg.pa_duty_cycle)
         || (STM32WLxx_PA_CFG_PARAM_INVALID == drv_ctx.pa_cfg.hp_max)
         || (STM32WLxx_PA_CFG_PARAM_INVALID == drv_ctx.pa_cfg.device_sel)
         || (STM32WLxx_PA_CFG_PARAM_INVALID == drv_ctx.pa_cfg.pa_lut)
         || (STM32WLxx_PA_CFG_PARAM_INVALID == (uint8_t)drv_ctx.pa_cfg.tx_power_reg)
         || (STM32WLxx_PA_CFG_PARAM_INVALID == (uint8_t)drv_ctx.pa_cfg.target_tx_power))
        {
            /* At least one PA config param was not set - apply default config */

            /* Check if the callback changed the target Tx power */
            if (STM32WLxx_PA_CFG_PARAM_INVALID == (uint8_t)drv_ctx.pa_cfg.target_tx_power)
            {
                drv_ctx.pa_cfg.target_tx_power = (int8_t)target_pwr;
            }

            /* Apply the optimized PA settings as per the application note to achieve the lower power consumption */
            if ((drv_ctx.config->enable_lpa != false)  && (drv_ctx.config->enable_hpa != false))
            {
                /* Both HPA and LPA are enabled - select the PA to use based on the targeted Tx power */
                if (drv_ctx.pa_cfg.target_tx_power > RADIO_STM32WLxx_LPA_UPPER_LIMIT_DBM)
                {
                    /* Requested Tx power exceeds the capabilities of LPA - use HPA for it */
                    drv_ctx.pa_cfg.device_sel = STM32WLxx_PA_CFG_USE_HPA;
                }
                else
                {
                    /* Requested Tx power can be achieved with LPA */
                    drv_ctx.pa_cfg.device_sel = STM32WLxx_PA_CFG_USE_LPA;
                }
            }
            else if (drv_ctx.config->enable_hpa != false)
            {
                /* Only HPA is enabled */
                drv_ctx.pa_cfg.device_sel = STM32WLxx_PA_CFG_USE_HPA;
            }
            else
            {
                /* Only LPA is enabled */
                drv_ctx.pa_cfg.device_sel = STM32WLxx_PA_CFG_USE_LPA;
            }

            /* Configure PA parameters based on the selected PA */
            if (STM32WLxx_PA_CFG_USE_HPA == drv_ctx.pa_cfg.device_sel)
            {
                /* Saturate by upper limit for HPA */
                if (drv_ctx.pa_cfg.target_tx_power > RADIO_STM32WLxx_HPA_UPPER_LIMIT_DBM)
                {
                    drv_ctx.pa_cfg.target_tx_power = RADIO_STM32WLxx_HPA_UPPER_LIMIT_DBM;
                }

                /* Search for the closest optimized config - configure as per AN5457 */
                if (drv_ctx.pa_cfg.target_tx_power <= STM32WLxx_PA_CFG_HPA_OPTIMIZATION_14_DBM)
                {
                    /* +14dBm optimization */
                    drv_ctx.pa_cfg.pa_duty_cycle = 0x02u;
                    drv_ctx.pa_cfg.hp_max        = 0x02u;
                    drv_ctx.pa_cfg.pa_lut        = 0x01u;
                    drv_ctx.pa_cfg.tx_power_reg  = drv_ctx.pa_cfg.target_tx_power;
                }
                else if (drv_ctx.pa_cfg.target_tx_power <= STM32WLxx_PA_CFG_HPA_OPTIMIZATION_17_DBM)
                {
                    /* +17dBm optimization */
                    drv_ctx.pa_cfg.pa_duty_cycle = 0x02u;
                    drv_ctx.pa_cfg.hp_max        = 0x03u;
                    drv_ctx.pa_cfg.pa_lut        = 0x01u;
                    drv_ctx.pa_cfg.tx_power_reg  = RADIO_STM32WLxx_HPA_UPPER_LIMIT_DBM - (STM32WLxx_PA_CFG_HPA_OPTIMIZATION_17_DBM - drv_ctx.pa_cfg.target_tx_power);
                }
                else if (drv_ctx.pa_cfg.target_tx_power <= STM32WLxx_PA_CFG_HPA_OPTIMIZATION_20_DBM)
                {
                    /* +20dBm optimization */
                    drv_ctx.pa_cfg.pa_duty_cycle = 0x03u;
                    drv_ctx.pa_cfg.hp_max        = 0x05u;
                    drv_ctx.pa_cfg.pa_lut        = 0x01u;
                    drv_ctx.pa_cfg.tx_power_reg  = RADIO_STM32WLxx_HPA_UPPER_LIMIT_DBM - (STM32WLxx_PA_CFG_HPA_OPTIMIZATION_20_DBM - drv_ctx.pa_cfg.target_tx_power);
                }
                else
                {
                    /* +22dBm optimization */
                    drv_ctx.pa_cfg.pa_duty_cycle = 0x04u;
                    drv_ctx.pa_cfg.hp_max        = 0x07u;
                    drv_ctx.pa_cfg.pa_lut        = 0x01u;
                    drv_ctx.pa_cfg.tx_power_reg  = drv_ctx.pa_cfg.target_tx_power;
                }

                /* Saturate by lower limit for HPA */
                if (drv_ctx.pa_cfg.tx_power_reg < RADIO_STM32WLxx_HPA_LOWER_LIMIT_DBM)
                {
                    drv_ctx.pa_cfg.tx_power_reg = RADIO_STM32WLxx_HPA_LOWER_LIMIT_DBM;
                }
            }
            else
            {
                /* Saturate by upper limit for LPA */
                if (drv_ctx.pa_cfg.target_tx_power > RADIO_STM32WLxx_LPA_UPPER_LIMIT_DBM)
                {
                    drv_ctx.pa_cfg.target_tx_power = RADIO_STM32WLxx_LPA_UPPER_LIMIT_DBM;
                }

                /* Search for the closest optimized config - configure as per AN5457 */
                if (drv_ctx.pa_cfg.target_tx_power <= STM32WLxx_PA_CFG_LPA_OPTIMIZATION_10_DBM)
                {
                    /* +10dBm optimization */
                    drv_ctx.pa_cfg.pa_duty_cycle = 0x01u;
                    drv_ctx.pa_cfg.hp_max        = 0x00u;
                    drv_ctx.pa_cfg.pa_lut        = 0x01u;
                    drv_ctx.pa_cfg.tx_power_reg  = 13 - (STM32WLxx_PA_CFG_LPA_OPTIMIZATION_10_DBM - drv_ctx.pa_cfg.target_tx_power);
                }
                else if (drv_ctx.pa_cfg.target_tx_power <= STM32WLxx_PA_CFG_LPA_OPTIMIZATION_14_DBM)
                {
                    /* +14dBm optimization */
                    drv_ctx.pa_cfg.pa_duty_cycle = 0x04u;
                    drv_ctx.pa_cfg.hp_max        = 0x00u;
                    drv_ctx.pa_cfg.pa_lut        = 0x01u;
                    drv_ctx.pa_cfg.tx_power_reg  = drv_ctx.pa_cfg.target_tx_power;
                }
                else
                {
                    /* +15dBm optimization */
                    drv_ctx.pa_cfg.pa_duty_cycle = 0x07u;
                    drv_ctx.pa_cfg.hp_max        = 0x00u;
                    drv_ctx.pa_cfg.pa_lut        = 0x01u;
                    drv_ctx.pa_cfg.tx_power_reg  = STM32WLxx_PA_CFG_LPA_OPTIMIZATION_14_DBM - (RADIO_STM32WLxx_LPA_UPPER_LIMIT_DBM - drv_ctx.pa_cfg.target_tx_power);
                }

                /* Saturate by lower limit for HPA */
                if (drv_ctx.pa_cfg.tx_power_reg < RADIO_STM32WLxx_LPA_LOWER_LIMIT_DBM)
                {
                    drv_ctx.pa_cfg.tx_power_reg = RADIO_STM32WLxx_LPA_LOWER_LIMIT_DBM;
                }
            }
        }

        /* Check if the callback specified the ramp up time */
        if (STM32WLxx_PA_CFG_PARAM_INVALID == drv_ctx.pa_cfg.ramp_time)
        {
            /* Use the default value */
            drv_ctx.pa_cfg.ramp_time = RADIO_STM32WLxx_RAMP_40_US;
        }
    }

    return RADIO_ERROR_NONE;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline void radio_stm32wlxx_error_manager(const int32_t reported_err, const uint32_t set_radio_sleep)
{
    if (reported_err != RADIO_ERROR_NONE)
    {
        /* A new error was reported - increment error counter and update the timestamp */
        drv_ctx.error_monitor.drv_err_cntr += STM32WLxx_RADIO_ERROR_ASYM_INC_FACTOR; /* Make the error counter increment faster than decrement to address interleaved errors (e.g. error - no error - error - no error, etc.) */
        (void)sid_pal_uptime_now(&drv_ctx.error_monitor.last_err_timestamp);
    }
    else
    {
        /* Operation completed successfully - decrement the error counter */
        if (drv_ctx.error_monitor.drv_err_cntr > 0u)
        {
            drv_ctx.error_monitor.drv_err_cntr--;
        }
    }

    /* Check if we are above the error threshold and react if needed */
    if (drv_ctx.error_monitor.drv_err_cntr > (STM32WLxx_RADIO_ERROR_LIMIT * STM32WLxx_RADIO_ERROR_ASYM_INC_FACTOR))
    {
        int32_t                err;
        stm32wlxx_hal_status_t hal_err;

        SID_PAL_LOG_WARNING("Too many radio errors in a row. Resetting the radio...");

        drv_ctx.radio_state = SID_PAL_RADIO_UNKNOWN;

        do
        {
            /* Ensure STM32WLxx won't trigger any interrupt if we are re-initializing ----*/
            hal_err = stm32wlxx_hal_disarm_irq(&drv_ctx);
            if (hal_err != STM32WLxx_HAL_STATUS_OK)
            {
                /* Logs are provided by the stm32wlxx_hal_disarm_irq() */
                err = RADIO_ERROR_HARDWARE_ERROR;
                break;
            }
            __COMPILER_BARRIER(); /* Ensure IRQ is disarmed right now before any further actions take place */
            /*----------------------------------------------------------------------------*/

            /* Bring down the underlying hardware */
            err = radio_stm32wlxx_platform_deinit();
            if (err != RADIO_ERROR_NONE)
            {
                STM32WLxx_LOG_ERROR("radio_stm32wlxx_platform_deinit() failed. Error %d", err);
                break;
            }

            /* Initialize the underlying hardware */
            err = radio_stm32wlxx_platform_init();
            if (err != RADIO_ERROR_NONE)
            {
                STM32WLxx_LOG_ERROR("radio_stm32wlxx_platform_init() failed with error %d", err);
                err = RADIO_ERROR_HARDWARE_ERROR;
                break;
            }

            /* Invalidate radio state since the WL side may be in LPM mode */
            drv_ctx.radio_state = SID_PAL_RADIO_UNKNOWN;

            /* Perform multiple attempts of handshaking with the STM32WLxx ---------------*/
            uint32_t handshake_attempts_counter = 0u;
            do
            {
                handshake_attempts_counter++;

                err = radio_stm32wlxx_do_handshake();
                if ((RADIO_ERROR_NONE == err) || (RADIO_ERROR_NOT_SUPPORTED == err))
                {
                    /* Exit either on success or unrecoverable error */
                    break;
                }
            } while (handshake_attempts_counter < STM32WLxx_HANDSHAKE_ATTEMPTS_LIMIT);

            /* Check if Handshake procedure succeeded */
            if (err != RADIO_ERROR_NONE)
            {
                STM32WLxx_LOG_ERROR("radio_stm32wlxx_do_handshake() failed with error %d", err);
                break;
            }
            /*----------------------------------------------------------------------------*/

            /* Now configure STM32WLxx IRQ line in as a regular IRQ line -----------------*/
            hal_err = stm32wlxx_hal_arm_irq(&drv_ctx);
            if (hal_err != STM32WLxx_HAL_STATUS_OK)
            {
                /* Logs are provided by the stm32wlxx_hal_arm_irq() */
                err = RADIO_ERROR_HARDWARE_ERROR;
                break;
            }
            /*----------------------------------------------------------------------------*/

            /* Ensure STM32WLxx's SubGHz radio is in Standby -----------------------------*/
            hal_err = stm32wlxx_hal_radio_standby(&drv_ctx); /* Don't use sid_pal_radio_standby() to avoid recursive calls */
            if (hal_err != STM32WLxx_HAL_STATUS_OK)
            {
                STM32WLxx_LOG_ERROR("Failed to put SubGHz to StandBy. HAL error %u", (uint32_t)hal_err);
                err = RADIO_ERROR_HARDWARE_ERROR;
                break;
            }

            drv_ctx.radio_state = SID_PAL_RADIO_STANDBY;
            /*----------------------------------------------------------------------------*/

            /* Send out SubGHz radio configuration via SPI to STM32WLxx ------------------*/
            err = stm32wlxx_radio_send_subghz_config(FALSE);
            if (err != RADIO_ERROR_NONE)
            {
                /* Logs are provided by stm32wlxx_radio_send_subghz_config() */
                SID_PAL_LOG_ERROR("Failed to re-send SubGHz HW config. Error %d", err);
                break;
            }

            /* Perform the final STM32WLxx Sidewalk Radio App initialization steps */
            hal_err = stm32wlxx_hal_apply_base_hw_config(&drv_ctx, FALSE);
            if (hal_err != STM32WLxx_HAL_STATUS_OK)
            {
                SID_PAL_LOG_ERROR("Failed to apply essential SubGHz HW config. Error code %u", (uint32_t)hal_err);
                err = RADIO_ERROR_IO_ERROR;
                break;
            }

            /* Explicitly set modem mode on the remote side */
            hal_err = stm32wlxx_hal_radio_set_modem_mode(&drv_ctx, drv_ctx.modem);
            if (hal_err != STM32WLxx_HAL_STATUS_OK)
            {
                SID_PAL_LOG_ERROR("Failed to set SubGHz modem mode. Error code %u", (uint32_t)hal_err);
                err = RADIO_ERROR_IO_ERROR;
                break;
            }

            /* Restore the selected radio frequency */
            hal_err = stm32wlxx_hal_radio_set_frequency(&drv_ctx, drv_ctx.radio_freq_hz);
            if (hal_err != STM32WLxx_HAL_STATUS_OK)
            {
                SID_PAL_LOG_ERROR("Failed to set SubGHz frequency. Error code %u", (uint32_t)hal_err);
                err = RADIO_ERROR_IO_ERROR;
                break;
            }

            /* Put the radio into Sleep mode if requested */
            if (set_radio_sleep != FALSE)
            {
                hal_err = stm32wlxx_hal_radio_sleep(&drv_ctx, 0u, FALSE); /* Don't use sid_pal_radio_sleep() to avoid recursive calls */
                if (hal_err != STM32WLxx_HAL_STATUS_OK)
                {
                    /* This is not a fatal error. The radio is in Standby already, software may proceed even if the radio failed to enter Sleep back for some reason. There's only power consumption penalty here */
                    SID_PAL_LOG_ERROR("Failed to put STM32WLxx back to sleep. Error code %u", (uint32_t)hal_err);
                    err = RADIO_ERROR_IO_ERROR;
                    drv_ctx.radio_state = SID_PAL_RADIO_UNKNOWN;
                    break;
                }

                /* The radio is back in Sleep now */
                drv_ctx.radio_state = SID_PAL_RADIO_SLEEP;
            }
            else
            {
                /* The radio is in Standby now as a result of the handshake procedure */
            }

            /* Radio re-initialized successfully */
            err = RADIO_ERROR_NONE;
            drv_ctx.error_monitor.drv_err_cntr = 0u;
            SID_PAL_LOG_INFO("Radio re-initialized");
        } while (0);

        if (err != RADIO_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to re-initialize STM32WLxx radio. This may indicate a permanent failure");
        }
    }
}

/*----------------------------------------------------------------------------*/

#if STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER
SID_STM32_SPEED_OPTIMIZED static void _user_data_transfer_thread(void * context)
{
    sid_error_t sid_err;
    osStatus_t os_err;

    (void)context;

    do
    {
        SID_PAL_ASSERT(drv_ctx.udt_ctx.outbound_msg_queue != NULL);

        /* Ensure the radio is fully initialized before any further actions */
        while (FALSE == drv_ctx.init_done)
        {
            /* Wait for driver initialization */
            sid_pal_scheduler_delay_ms(10u);
        }

        /* Wait for any user app requests to send out data to the STM32WLxx side */
        stm32wlxx_ext_ifc_out_msg_desc_t msg_desc;
        os_err = osMessageQueueGet(drv_ctx.udt_ctx.outbound_msg_queue, &msg_desc, NULL, osWaitForever);

        if (osOK == os_err)
        {
            if ((msg_desc.data_ptr != NULL) && (msg_desc.data_len > 0u))
            {
                stm32wlxx_hal_status_t hal_err;
                struct sid_timespec t_now;

                /* Calculate available time for UDT */
                sid_err = sid_pal_uptime_now(&t_now);
                if (sid_err != SID_ERROR_NONE)
                {
                    SID_PAL_LOG_ERROR("Unable to determine current time. UDT skipped")
                    break;
                }

                if ((sid_time_is_infinity(&drv_ctx.udt_ctx.cut_off_time) == FALSE)
                  && (sid_time_gt(&t_now, &drv_ctx.udt_ctx.cut_off_time) != FALSE))
                {
                    /* No time left for UDT */
                    drv_ctx.udt_ctx.udt_enabled = FALSE;
                }

                /* Wait until UDT is allowed */
                while (FALSE == drv_ctx.udt_ctx.udt_enabled)
                {
                    sid_pal_scheduler_delay_ms(1u);
                }

                /* Send out user data. Use critical section to guarantee atomic data transfer (e.g. avoid interruption mid-flight by sid_pal_radio_standby() ) */
                sid_pal_enter_critical_region();
                hal_err = stm32wlxx_hal_send_user_data(&drv_ctx, msg_desc.data_ptr, msg_desc.data_len);
                sid_pal_exit_critical_region();
                if (hal_err != STM32WLxx_HAL_STATUS_OK)
                {
                    SID_PAL_LOG_ERROR("UDT failed to send out %u bytes. HAL error %u", msg_desc.data_len, (uint32_t)hal_err);
                }
                else
                {
                    SID_PAL_LOG_DEBUG("UDT sent %u bytes", msg_desc.data_len);
                }

                /* Automatically free the buffer after processing is done (if requested) */
                if (msg_desc.auto_free != FALSE)
                {
                    free(msg_desc.data_ptr);
                }
            }
            else
            {
                SID_PAL_LOG_WARNING("UDT transfer skipped - empty data provided");
            }
        }
        else
        {
            /* Some unexpected CMSIS/OS error occurred */
            SID_PAL_LOG_ERROR("Unexpected error while accessing UDT queue. OS error 0x%08x", (uint32_t)os_err);
        }
    } while (TRUE);

    /* Terminate the thread properly if the code ever gets here */
    SID_PAL_LOG_WARNING("UDT thread terminated");
    osThreadExit();
}
#endif /* STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER */

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t stm32wlxx_radio_send_subghz_config(const uint32_t need_ack)
{
    uint32_t err = RADIO_ERROR_GENERIC;
    stm32wlxx_hal_status_t hal_err;

    do
    {
        /* Collect the settings */
        const sid_pal_radio_fsk_phy_settings_t * const fsk_phy_settings =
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_2
                                                                            rnet_get_fsk_phy_settings();
#else
                                                                            NULL;
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 */

        const sid_pal_radio_lora_phy_settings_t * const lora_phy_settings =
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
                                                                            rnet_get_lora_phy_settings();
#else
                                                                            NULL;
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 */

        stm32wlxx_radio_phy_cfg_t subghz_cfg;

        radio_stm32wlxx_pack_settings(fsk_phy_settings, lora_phy_settings, &subghz_cfg);

        /* Send out the settings and wait for them to be applied */
        hal_err = stm32wlxx_hal_send_subghz_config(&drv_ctx, &subghz_cfg, need_ack);
        if (hal_err != STM32WLxx_HAL_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Unable to send out config to STM32WLxx. HAL error %u", (uint32_t)hal_err);
            err = RADIO_ERROR_IO_ERROR;
            break;
        }

        /* Save the applied Phy settings as sent to the STM32WLxx */
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_2
        SID_STM32_UTIL_fast_memcpy(&drv_ctx.applied_fsk_phy_cfg,  fsk_phy_settings,  sizeof(drv_ctx.applied_fsk_phy_cfg));
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 */

#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
        SID_STM32_UTIL_fast_memcpy(&drv_ctx.applied_lora_phy_cfg, lora_phy_settings, sizeof(drv_ctx.applied_lora_phy_cfg));
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 */

        err = RADIO_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED const halo_drv_stm32wlxx_ctx_t* stm32wlxx_radio_get_drv_ctx_ctx(void)
{
    return &drv_ctx;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED void stm32wlxx_app_radio_set_device_config(const stm32wlxx_app_radio_device_config_t * const cfg)
{
    /* Validate the config */
    if ((false == cfg->enable_lpa) && (false == cfg->enable_hpa))
    {
        SID_PAL_LOG_ERROR("Invalid radio config - both LPA and HPA are disabled. At least one PA shall be enabled");
        SID_PAL_ASSERT(0);
    }

    /* Store the config */
    drv_ctx.config = cfg;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED uint8_t sid_pal_radio_get_status(void)
{
    return drv_ctx.radio_state;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_pal_radio_modem_mode_t sid_pal_radio_get_modem_mode(void)
{
    sid_pal_radio_modem_mode_t mode;

    switch (drv_ctx.modem)
    {
        case STM32WLxx_SUBGHZ_RADIO_MODEM_MODE_FSK:
            mode = SID_PAL_RADIO_MODEM_MODE_FSK;
            break;

        case STM32WLxx_SUBGHZ_RADIO_MODEM_MODE_LORA:
            mode = SID_PAL_RADIO_MODEM_MODE_LORA;
            break;

        case STM32WLxx_SUBGHZ_RADIO_MODEM_MODE_UNDEFINED:
        default:
            SID_PAL_LOG_ERROR("sid_pal_radio_get_modem_mode() called with modem in undefined state");
            mode = SID_PAL_RADIO_MODEM_MODE_LORA; /* Default to LoRa for compatibility with previous implementations */
            break;
    }

    return mode;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_set_modem_mode(sid_pal_radio_modem_mode_t mode)
{
    int32_t err = RADIO_ERROR_GENERIC;
    stm32wlxx_hal_status_t hal_err;

    STM32WLxx_LOG_DEBUG("sid_pal_radio_set_modem_mode...");

    /* Skip the switch if we are already in the requested modem mode */
    if ((sid_pal_radio_get_modem_mode() == mode) && (drv_ctx.modem != STM32WLxx_SUBGHZ_RADIO_MODEM_MODE_UNDEFINED))
    {
        STM32WLxx_LOG_DEBUG("Set Modem Mode skipped - already in requested mode");
        err = RADIO_ERROR_NONE;
        return err;
    }

    /* Send out modem mode switch request */
    switch (mode)
    {
        case SID_PAL_RADIO_MODEM_MODE_FSK:
            hal_err = stm32wlxx_hal_radio_set_modem_mode(&drv_ctx, STM32WLxx_SUBGHZ_RADIO_MODEM_MODE_FSK);
            if (STM32WLxx_HAL_STATUS_OK == hal_err)
            {
                drv_ctx.modem = STM32WLxx_SUBGHZ_RADIO_MODEM_MODE_FSK;
                drv_ctx.subghz_irq_mask = STM32WLxx_RADIO_COMM_DEFAULT_FSK_IRQ_MASK;
                err = RADIO_ERROR_NONE;
            }
            else
            {
                STM32WLxx_LOG_ERROR("sid_pal_radio_set_modem_mode failed to set FSK mode. HAL error %u", (uint32_t)hal_err);
                err = RADIO_ERROR_IO_ERROR;
            }
            break;

        case SID_PAL_RADIO_MODEM_MODE_LORA:
            hal_err = stm32wlxx_hal_radio_set_modem_mode(&drv_ctx, STM32WLxx_SUBGHZ_RADIO_MODEM_MODE_LORA);
            if (STM32WLxx_HAL_STATUS_OK == hal_err)
            {
                drv_ctx.modem = STM32WLxx_SUBGHZ_RADIO_MODEM_MODE_LORA;
                drv_ctx.subghz_irq_mask = STM32WLxx_RADIO_COMM_DEFAULT_LORA_IRQ_MASK;
                err = RADIO_ERROR_NONE;
            }
            else
            {
                STM32WLxx_LOG_ERROR("sid_pal_radio_set_modem_mode failed to set LoRa mode. HAL error %u", (uint32_t)hal_err);
                err = RADIO_ERROR_IO_ERROR;
            }
            break;

        default:
            STM32WLxx_LOG_ERROR("sid_pal_radio_set_modem_mode failed - invalid mode requested: %u", (uint32_t)mode);
            err = RADIO_ERROR_NOT_SUPPORTED;
            break;
    }

    if (err != RADIO_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("Failed to set SubGHz modem mode. Error %d", err);
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_irq_process(void)
{
    int32_t err;
    stm32wlxx_hal_status_t hal_err;

    do
    {
        /* Call the generic IRQ handler. It may subsequently call the Sidewalk-specific handler if a Sidewalk-related radio IRQ was reported */
        hal_err = stm32wlxx_hal_generic_irq_process(&drv_ctx, &err);

        /* Give priority to the error reported by the Sidewalk-specific handler */
        if (err != RADIO_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Sidewalk sub-GHz radio event processing failed. error %d", err);
            break;
        }

        /* React on the generic IRQ handler errors */
        if (hal_err != STM32WLxx_HAL_STATUS_OK)
        {
            /* Logs are provided by stm32wlxx_hal_generic_irq_process() */
            break;
        }

        /* Done */
        hal_err = STM32WLxx_HAL_STATUS_OK;
    } while (0);

    /* Unconditionally re-enable STM32WLxx radio IRQ line regardless of the IRQ processing outcomes */
    stm32wlxx_hal_status_t rearm_err = stm32wlxx_hal_arm_irq(&drv_ctx);
    if (rearm_err != STM32WLxx_HAL_STATUS_OK)
    {
        SID_PAL_LOG_ERROR("Failed to re-arm STM32WLxx IRQ line. error %u", (uint32_t)hal_err);
    }

    hal_err = hal_err != STM32WLxx_HAL_STATUS_OK ? hal_err : rearm_err;
    err = err != RADIO_ERROR_NONE ? err : RADIO_ERROR_IO_ERROR;

    return RADIO_ERROR_NONE;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t stm32wlxx_radio_sidewalk_event_process(void)
{
    int32_t err;
    sid_pal_radio_events_t radio_event = SID_PAL_RADIO_EVENT_UNKNOWN;

    do
    {
        /* Last IRQ status shall be stored already in the driver context when we get here */
        const stm32wlxx_app_irq_mask_t active_irq = (stm32wlxx_app_irq_mask_t)drv_ctx.last_irq_status.irq_flags;

        /* Ensure that STM32WLxx_APP_IRQ_SUBGHZ is indicated and this is the only interrupt being present */
        if (active_irq != STM32WLxx_APP_IRQ_SUBGHZ)
        {
            STM32WLxx_LOG_ERROR("SubGHz IRQ processing called with unexpected IRQ flags set: 0x%x", (uint32_t)active_irq);
            err = RADIO_ERROR_INVALID_STATE;
            break;
        }

        /* Now handle the STM32WLxx_APP_IRQ_SUBGHZ IRQ */
        const stm32wlxx_subghz_irq_details_t * const irq_details = &drv_ctx.last_irq_status.subghz_irq_details;

        /* fill in fields of internal structures */
        radio_event           = (sid_pal_radio_events_t)irq_details->radio_event;
        drv_ctx.radio_state   = irq_details->radio_state;
        drv_ctx.cad_exit_mode = (sid_pal_radio_cad_param_exit_mode_t)irq_details->cad_exit_mode;

        /* IMPORTANT: any received radio Rx buffer is already stored to drv_ctx.radio_rx_packet->rcv_payload and
         *            drv_ctx.radio_rx_packet->payload_len is set to the valid length is set by
         *            radio_stm32wlxx_on_irq_detected(), so we don't have to deal with it here
         */

        if (STM32WLxx_SUBGHZ_RADIO_MODEM_MODE_FSK == drv_ctx.modem)
        {
            /* copy FSK status */
            STM32WLxx_COPY_CFG_PARAM(drv_ctx.radio_rx_packet->fsk_rx_packet_status.rssi_avg,        irq_details->received_packet_info.fsk_rx_packet_status.rssi_avg);
            STM32WLxx_COPY_CFG_PARAM(drv_ctx.radio_rx_packet->fsk_rx_packet_status.rssi_sync,       irq_details->received_packet_info.fsk_rx_packet_status.rssi_sync);
            STM32WLxx_COPY_CFG_PARAM(drv_ctx.radio_rx_packet->fsk_rx_packet_status.snr,             irq_details->received_packet_info.fsk_rx_packet_status.snr);

            /* Compensate for antenna gain to get unified RSSI readings */
            const int32_t rssi_adjustment = (drv_ctx.regional_radio_param->ant_dbi + 50) / 100;
            drv_ctx.radio_rx_packet->fsk_rx_packet_status.rssi_avg  -= (int8_t)rssi_adjustment;
            drv_ctx.radio_rx_packet->fsk_rx_packet_status.rssi_sync -= (int8_t)rssi_adjustment;
        }
        else if (STM32WLxx_SUBGHZ_RADIO_MODEM_MODE_LORA == drv_ctx.modem)
        {
            /* copy LoRa status */
            STM32WLxx_COPY_CFG_PARAM(drv_ctx.radio_rx_packet->lora_rx_packet_status.rssi,           irq_details->received_packet_info.lora_rx_packet_status.rssi);
            STM32WLxx_COPY_CFG_PARAM(drv_ctx.radio_rx_packet->lora_rx_packet_status.signal_rssi,    irq_details->received_packet_info.lora_rx_packet_status.signal_rssi);
            STM32WLxx_COPY_CFG_PARAM(drv_ctx.radio_rx_packet->lora_rx_packet_status.snr,            irq_details->received_packet_info.lora_rx_packet_status.snr);
            STM32WLxx_COPY_CFG_PARAM(drv_ctx.radio_rx_packet->lora_rx_packet_status.is_crc_present, (sid_pal_radio_lora_crc_present_t)irq_details->received_packet_info.lora_rx_packet_status.is_crc_present);

            /* Compensate for antenna gain to get unified RSSI readings */
            const int32_t rssi_adjustment = (drv_ctx.regional_radio_param->ant_dbi + 50) / 100;
            drv_ctx.radio_rx_packet->lora_rx_packet_status.rssi        -= (int8_t)rssi_adjustment;
            drv_ctx.radio_rx_packet->lora_rx_packet_status.signal_rssi -= (int8_t)rssi_adjustment;
        }
        else
        {
            SID_PAL_LOG_ERROR("STM32WLxx SubGHz IRQ reported with modem in unknown state. Cannot process");
            err = RADIO_ERROR_INVALID_STATE;
            break;
        }

        /* Radio is automatically transitioned into Standby state by the STM32WLxx side for the events below */
        if ((SID_PAL_RADIO_EVENT_RX_TIMEOUT == radio_event) || (SID_PAL_RADIO_EVENT_RX_DONE == radio_event) || (SID_PAL_RADIO_EVENT_TX_DONE == radio_event) || (SID_PAL_RADIO_EVENT_TX_TIMEOUT == radio_event)
          || (SID_PAL_RADIO_EVENT_RX_ERROR == radio_event) || (SID_PAL_RADIO_EVENT_HEADER_ERROR == radio_event) || (SID_PAL_RADIO_EVENT_CS_DONE == radio_event) || (SID_PAL_RADIO_EVENT_CAD_DONE == radio_event)
          || (SID_PAL_RADIO_EVENT_CAD_TIMEOUT == radio_event))
        {
            drv_ctx.radio_state = SID_PAL_RADIO_STANDBY;

#if (CFG_LPM_LEVEL != 0)
            /* Allow Stop mode after Rx/Tx is finished */
            UTIL_LPM_SetStopMode((1u << CFG_LPM_SIDEWALK), UTIL_LPM_ENABLE);
#  if (CFG_LPM_STDBY_SUPPORTED != 0)
            UTIL_LPM_SetOffMode((1u << CFG_LPM_SIDEWALK), UTIL_LPM_ENABLE);
#  endif /* CFG_LPM_STDBY_SUPPORTED */
#endif /* (CFG_LPM_LEVEL != 0) */
        }

        /* Notify the stack about SubGHz radio event */
        if (SID_PAL_RADIO_EVENT_UNKNOWN != radio_event)
        {
#if SID_SDK_CONFIG_RF_TIMINGS_MEASUREMENT
            /* Clear Rx/Tx starting point indication */
            SIDEWALK_RF_TIMINGS_RXTX_START_GPIO_Port->BRR = SIDEWALK_RF_TIMINGS_RXTX_START_Pin;
#endif /* SID_SDK_CONFIG_RF_TIMINGS_MEASUREMENT */

#if STM32WLxx_RADIO_CFG_USE_STATUS_LED
            /* Turn off activity LEDs - any event means activities are finished */
            stm32wlxx_radio_hal_tx_led_off(&drv_ctx);
            stm32wlxx_radio_hal_rx_led_off(&drv_ctx);
#endif /* STM32WLxx_RADIO_CFG_USE_STATUS_LED */

            /* Notify the radio about the event - make sure this is the last action after all GPIO handling and LPM management since drv_ctx.report_radio_event() may modify GPIO and LPM states */
            drv_ctx.report_radio_event(radio_event);
        }

        err = RADIO_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_set_frequency(uint32_t freq)
{
    int32_t err = RADIO_ERROR_GENERIC;
    stm32wlxx_hal_status_t hal_err;

    STM32WLxx_LOG_DEBUG("sid_pal_radio_set_frequency - freq: %u", freq);

    do
    {
        if (drv_ctx.radio_state != SID_PAL_RADIO_STANDBY)
        {
            err = RADIO_ERROR_INVALID_STATE;
            break;
        }

        if (freq == drv_ctx.radio_freq_hz)
        {
            STM32WLxx_LOG_DEBUG("freq is already set to %u - skipping request", freq);
            err = RADIO_ERROR_NONE;
            break;
        }

        hal_err = stm32wlxx_hal_radio_set_frequency(&drv_ctx, freq);
        if (hal_err != STM32WLxx_HAL_STATUS_OK)
        {
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Store the newly applied frequency to the context */
        drv_ctx.radio_freq_hz = freq;

        err = RADIO_ERROR_NONE;
    } while (0);

    if (err != RADIO_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("Failed to set STM32WLxx SubGHz modem frequency to %u. Error %d, state: %u", freq, err, drv_ctx.radio_state);
    }

    radio_stm32wlxx_error_manager(err, FALSE);
    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_get_max_tx_power(sid_pal_radio_data_rate_t data_rate, int8_t *tx_power)
{
    int32_t err;

    SID_PAL_ASSERT(drv_ctx.regional_radio_param != NULL);

    if ((data_rate <= SID_PAL_RADIO_DATA_RATE_INVALID) || (data_rate > SID_PAL_RADIO_DATA_RATE_MAX_NUM))
    {
        err = RADIO_ERROR_INVALID_PARAMS;
    }
    else
    {
        *tx_power = drv_ctx.regional_radio_param->max_tx_power[data_rate - 1];
        err = RADIO_ERROR_NONE;
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_set_region(sid_pal_radio_region_code_t region)
{
    /* By default report the requested region is not supported */
    int32_t err = RADIO_ERROR_NOT_SUPPORTED;

    /* Validate inputs */
    if ((region <= SID_PAL_RADIO_RC_NONE) || (region >= SID_PAL_RADIO_RC_MAX))
    {
        SID_PAL_LOG_ERROR("%d is not a valid radio region ID", (int32_t)region);
        return RADIO_ERROR_INVALID_PARAMS;
    }

    /* Loop through the available regional params to find a matching config */
    for (uint32_t i = 0u; i < drv_ctx.config->regional_config.reg_param_table_size; i++)
    {
        if (drv_ctx.config->regional_config.reg_param_table[i].param_region == region)
        {
            /* Found the desired regional config */
            drv_ctx.regional_radio_param = &drv_ctx.config->regional_config.reg_param_table[i];
            err = RADIO_ERROR_NONE;
            break;
        }
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t stm32wlxx_radio_get_pa_config(stm32wlxx_radio_pa_cfg_t * const cfg)
{
    int32_t err;
    if (NULL == cfg)
    {
        err = RADIO_ERROR_INVALID_PARAMS;
    }
    else
    {
        *cfg = drv_ctx.pa_cfg;
        err = RADIO_ERROR_NONE;
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_set_tx_power(int8_t power)
{
    int32_t                err = RADIO_ERROR_GENERIC;
    stm32wlxx_hal_status_t hal_err;

    STM32WLxx_LOG_DEBUG("sid_pal_radio_set_tx_power - set pwr: %s%ddB", power > 0 ? "+" : "", power);

    do
    {
        /* Store the currently used value */
        const stm32wlxx_radio_pa_cfg_t current_pa_cfg = drv_ctx.pa_cfg;

        /* Process config update */
        err = radio_stm32wlxx_set_tx_power_config(power);
        if (err != RADIO_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to set STM32WLxx SubGHz Tx power to %ddB. Error %d, state: %u", power, err, drv_ctx.radio_state);
            drv_ctx.pa_cfg = current_pa_cfg; /* restore the state if we've failed to apply the new setting */
            break;
        }

        /* Apply new PA config if something has changed */
        if (SID_STM32_UTIL_fast_memcmp(&current_pa_cfg, &drv_ctx.pa_cfg, sizeof(current_pa_cfg)) != 0u)
        {
            hal_err = stm32wlxx_hal_set_tx_power(&drv_ctx, &drv_ctx.pa_cfg);
            STM32WLxx_LOG_DEBUG("Updating STM32WLxx Tx power to %s%ddB", drv_ctx.pa_cfg.target_tx_power > 0 ? "+" : "", drv_ctx.pa_cfg.target_tx_power); /* tiny_vsnprintf may not support %+d */

            if (hal_err != STM32WLxx_HAL_STATUS_OK)
            {
                SID_PAL_LOG_ERROR("Failed to send out new PA config to STM32WLxx. HAL error %u", (uint32_t)hal_err);
                err = RADIO_ERROR_HARDWARE_ERROR;
                break;
            }
        }
        else
        {
            STM32WLxx_LOG_DEBUG("Skipped updating the STM32WLxx Tx power - already set to %s%ddB",  drv_ctx.pa_cfg.target_tx_power > 0 ? "+" : "",  drv_ctx.pa_cfg.target_tx_power); /* tiny_vsnprintf may not support %+d */
        }

        err = RADIO_ERROR_NONE;
    } while (0);

    radio_stm32wlxx_error_manager(err, FALSE);
    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_sleep(uint32_t sleep_us)
{
    int32_t err;
    stm32wlxx_hal_status_t hal_err;

#if STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER
    sid_error_t sid_err;
    struct sid_timespec t_now;

    sid_err = sid_pal_uptime_now(&t_now);
    __COMPILER_BARRIER();
#endif /* STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER */

    STM32WLxx_LOG_DEBUG("sid_pal_radio_sleep... Duration: %uus", sleep_us);

    /* Store a copy of the app IRQ mask on entry */
    const uint16_t app_irq_mask = drv_ctx.app_irq_mask;

    do
    {
        /* Skip the request if the radio is in the sleep mode already and this is not a request for indefinite sleep */
        if ((SID_PAL_RADIO_SLEEP == drv_ctx.radio_state) && (sleep_us != 0u))
        {
            STM32WLxx_LOG_DEBUG("SubGHz is already in sleep state, request skipped");
            err = RADIO_ERROR_NONE;
            break;
        }

        /* Workaround for the Sidewalk stack issue for LoRa --------------------------*/
        if (UINT32_MAX == sleep_us)
        {
            /* The stack will first call sid_pal_radio_sleep() with 0xFFFFFFFFu just to stop any potentially running radio operation and after that
             * it will invoke sid_pal_radio_sleep() once again with the actual expected sleep time. To address this we invoke sid_pal_radio_standby()
             * instead for this special case to just stop the possibly running Tx/Rx operation but to still keep the STM32WLxx MCU running and ready
             * to accept the sleep request with the actual sleep duration expectation.
             */
            err = sid_pal_radio_standby();
            break; /* Terminate from here and keep the radio state at SID_PAL_RADIO_STANDBY */
        }
        /*----------------------------------------------------------------------------*/

#if STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER
        if ((sleep_us != 0u) && (drv_ctx.modem == STM32WLxx_SUBGHZ_RADIO_MODEM_MODE_LORA) && (sleep_us >= (STM32WLxx_RADIO_LORA_UL_SEPARATION_US - STM32WLxx_RADIO_LORA_RX_TIMEOUT_US)))
        {
            /* Allow UDT only during the first UL slot if Sidewalk link is active. This will prevent collisions between UDT and Tx that can be potentially
             * scheduled by Sidewalk within the current DL separation slot
             */
            sleep_us = STM32WLxx_RADIO_LORA_UL_SEPARATION_US - STM32WLxx_RADIO_LORA_RX_TIMEOUT_US;
        }
#endif /* STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER */

        /* Mask SubGHz events until the radio reports back it has entered sleep state */
        drv_ctx.app_irq_mask &= ~STM32WLxx_APP_IRQ_SUBGHZ;

        hal_err = stm32wlxx_hal_radio_sleep(&drv_ctx, sleep_us, FALSE);
        if (hal_err != STM32WLxx_HAL_STATUS_OK)
        {
            STM32WLxx_LOG_ERROR("Failed to put SubGHz to sleep mode. HAL error %u", (uint32_t)hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

#if (CFG_LPM_LEVEL != 0)
        /* Allow Stop mode after the radio was put into Sleep */
        UTIL_LPM_SetStopMode((1u << CFG_LPM_SIDEWALK), UTIL_LPM_ENABLE);
#  if (CFG_LPM_STDBY_SUPPORTED != 0)
        UTIL_LPM_SetOffMode((1u << CFG_LPM_SIDEWALK), UTIL_LPM_ENABLE);
#  endif /* CFG_LPM_STDBY_SUPPORTED */
#endif /* (CFG_LPM_LEVEL != 0) */

        drv_ctx.radio_state = SID_PAL_RADIO_SLEEP;
        err = RADIO_ERROR_NONE;

#if STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER
        /* Determine if UDT operations can be enabled during the planned sleep period */
        if (0u == sleep_us)
        {
            /* This is a special case - radio is put into Sleep for undefined amount of time (e.g. due to sid_stop() call) */
            drv_ctx.udt_ctx.cut_off_time = SID_TIME_INFINITY;
            __COMPILER_BARRIER();

            /* Allow UDT task to run */
            drv_ctx.udt_ctx.udt_enabled = TRUE;
        }
        else if (sleep_us > STM32WLxx_UDT_SUSPEND_RESERVED_TIME_US)
        {
            struct sid_timespec t_sleep;

            /* Check if the current time was obtained successfully on entry to this function */
            if (sid_err != SID_ERROR_NONE)
            {
                SID_PAL_LOG_ERROR("Unable to determine current time. UDT will be skipped");
                break;
            }

            /* Calculate the cut-off timestamp for UDT */
            drv_ctx.udt_ctx.cut_off_time = t_now;
            sid_us_to_timespec((sleep_us - STM32WLxx_UDT_SUSPEND_RESERVED_TIME_US), &t_sleep);
            sid_time_add(&drv_ctx.udt_ctx.cut_off_time, &t_sleep);
            __COMPILER_BARRIER();

            /* Allow UDT task to run */
            drv_ctx.udt_ctx.udt_enabled = TRUE;
        }
        else
        {
            /* Not enough time for UDT */
            drv_ctx.udt_ctx.udt_enabled = FALSE;
        }
#endif /* STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER */
    } while (0);

    /* Restore the app IRQ mask */
    drv_ctx.app_irq_mask = app_irq_mask;

    radio_stm32wlxx_error_manager(err, TRUE);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_standby(void)
{
    int32_t err;
    stm32wlxx_hal_status_t hal_err;

    /* Store a copy of the app IRQ mask on entry */
    const uint16_t app_irq_mask = drv_ctx.app_irq_mask;

    do
    {
#if STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER
        drv_ctx.udt_ctx.udt_enabled = FALSE;
        __COMPILER_BARRIER();
#endif /* STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER */

        if (SID_PAL_RADIO_STANDBY == drv_ctx.radio_state)
        {
            STM32WLxx_LOG_DEBUG("SubGHz is already in StandBy state, request skipped");
            err = RADIO_ERROR_NONE;
            break;
        }

#if SID_SDK_CONFIG_RF_TIMINGS_MEASUREMENT
        /* Indicate STM32WLxx wakeup start point - keep this code exactly here since all the above lines are always executed, event if there's no need to wake up STM32WLxx */
        __COMPILER_BARRIER();
        if (SID_PAL_RADIO_SLEEP == drv_ctx.radio_state)
        {
            SIDEWALK_RF_TIMINGS_RXTX_START_GPIO_Port->BRR = SIDEWALK_RF_TIMINGS_RXTX_START_Pin;
        }
        __COMPILER_BARRIER();
#endif /* SID_SDK_CONFIG_RF_TIMINGS_MEASUREMENT */

        /* Mask SubGHz events until the radio reports back it has entered Standby state */
        drv_ctx.app_irq_mask &= ~STM32WLxx_APP_IRQ_SUBGHZ;

        hal_err = stm32wlxx_hal_radio_standby(&drv_ctx);
        if (hal_err != STM32WLxx_HAL_STATUS_OK)
        {
            STM32WLxx_LOG_ERROR("Failed to put SubGHz to StandBy. HAL error %u", (uint32_t)hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

#if STM32WLxx_RADIO_CFG_USE_STATUS_LED
        /* Turn off status LEDs after the radio has reached Standby state */
        stm32wlxx_radio_hal_tx_led_off(&drv_ctx);
        stm32wlxx_radio_hal_rx_led_off(&drv_ctx);
#endif /* STM32WLxx_RADIO_CFG_USE_STATUS_LED */

        drv_ctx.radio_state = SID_PAL_RADIO_STANDBY;
        err = RADIO_ERROR_NONE;
    } while (0);

#if SID_SDK_CONFIG_RF_TIMINGS_MEASUREMENT
    /* Indicate STM32WLxx wakeup done point - keep this code exactly here since all the below lines are always executed, event if there's no need to wake up STM32WLxx */
    __COMPILER_BARRIER();
    SIDEWALK_RF_TIMINGS_RXTX_START_GPIO_Port->BSRR = SIDEWALK_RF_TIMINGS_RXTX_START_Pin;
    __COMPILER_BARRIER();
#endif /* SID_SDK_CONFIG_RF_TIMINGS_MEASUREMENT */

    /* Restore the app IRQ mask */
    drv_ctx.app_irq_mask = app_irq_mask;

    radio_stm32wlxx_error_manager(err, FALSE);
    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_set_tx_payload(const uint8_t *buffer, uint8_t size)
{
    int32_t err = RADIO_ERROR_GENERIC;
    stm32wlxx_hal_status_t hal_err;

    STM32WLxx_LOG_DEBUG("sid_pal_radio_set_tx_payload");
    do
    {
        if ((NULL == buffer) || (0u == size))
        {
            err = RADIO_ERROR_INVALID_PARAMS;
            break;
        }

        hal_err = stm32wlxx_hal_set_subghz_tx_buf(&drv_ctx, buffer, size);
        if (hal_err != STM32WLxx_HAL_STATUS_OK)
        {
            err = RADIO_ERROR_IO_ERROR;
            break;
        }

        STM32WLxx_LOG_DEBUG("set_tx_payload %u", size);
        err = RADIO_ERROR_NONE;
    } while (0);

    if (err != RADIO_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("Failed to set SubGHz radio Tx payload. Error %u", (uint32_t)err);
    }

    radio_stm32wlxx_error_manager(err, FALSE);
    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_start_tx(uint32_t timeout)
{
    int32_t err = RADIO_ERROR_GENERIC;
    stm32wlxx_hal_status_t hal_err;

    STM32WLxx_LOG_DEBUG("sid_pal_radio_start_tx... timeout: %d", timeout);

    do
    {
        hal_err = stm32wlxx_hal_radio_start_tx(&drv_ctx);
        if (hal_err != STM32WLxx_HAL_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to start Tx via STM32WLxx. HAL error %u", (uint32_t)hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        drv_ctx.radio_state = SID_PAL_RADIO_TX;

#if (CFG_LPM_LEVEL != 0)
        if (STM32WLxx_SUBGHZ_RADIO_MODEM_MODE_FSK == drv_ctx.modem)
        {
            /* Prohibit Stop mode until Tx is done as we need fast response on Tx Done IRQ */
            UTIL_LPM_SetStopMode((1u << CFG_LPM_SIDEWALK), UTIL_LPM_DISABLE);
#  if (CFG_LPM_STDBY_SUPPORTED != 0)
            UTIL_LPM_SetOffMode((1u << CFG_LPM_SIDEWALK), UTIL_LPM_DISABLE);
#  endif /* CFG_LPM_STDBY_SUPPORTED */
        }
        else
        {
            /* For LoRa link we don't need such rapid response. Additional 200-250us added by the wakeup from Stop LPM won't affect the link performance and we can save some power */
        }
#endif /* (CFG_LPM_LEVEL != 0) */

#if STM32WLxx_RADIO_CFG_USE_STATUS_LED
        /* Drive the status LED after the actual start of Tx to avoid impact on timings */
        stm32wlxx_radio_hal_tx_led_on(&drv_ctx);
#endif /* STM32WLxx_RADIO_CFG_USE_STATUS_LED */

        err = RADIO_ERROR_NONE;
    } while (0);

    radio_stm32wlxx_error_manager(err, FALSE);
    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_set_tx_continuous_wave(uint32_t freq, int8_t power)
{
#if HALO_ENABLE_DIAGNOSTICS
    int32_t err = RADIO_ERROR_GENERIC;
    stm32wlxx_hal_status_t hal_err;

    STM32WLxx_LOG_INFO("sid_pal_radio_set_tx_continuous_wave... ");

    do
    {
        /* sid_pal_radio_set_tx_power() since adjusting Tx power requires update of all the related PA settings */
        err = sid_pal_radio_set_tx_power(power);
        if (err != RADIO_ERROR_NONE)
        {
            /* Logs are provided by sid_pal_radio_set_tx_power() */
            break;
        }

        hal_err = stm32wlxx_hal_start_continuous_wave_tx(&drv_ctx, freq);
        if (hal_err != STM32WLxx_HAL_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to start STM32WLxx SubGHz CW Tx. HAL error %u", (uint32_t)hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        drv_ctx.radio_state = SID_PAL_RADIO_TX;

#if STM32WLxx_RADIO_CFG_USE_STATUS_LED
        /* Drive the status LED after the actual start of Tx to avoid impact on timings */
        stm32wlxx_radio_hal_tx_led_on(&drv_ctx);
#endif /* STM32WLxx_RADIO_CFG_USE_STATUS_LED */

        err = RADIO_ERROR_NONE;
    } while (0);

    return err;
#else
    return RADIO_ERROR_NOT_SUPPORTED;
#endif /* HALO_ENABLE_DIAGNOSTICS */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_start_rx(uint32_t timeout)
{
    int32_t err = RADIO_ERROR_GENERIC;
    stm32wlxx_hal_status_t hal_err;

    do
    {
        hal_err = stm32wlxx_hal_radio_start_rx(&drv_ctx, timeout);
        if (hal_err != STM32WLxx_HAL_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to start STM32WLxx SubGHz Rx. HAL error %u", (uint32_t)hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        drv_ctx.radio_state = SID_PAL_RADIO_RX;

#if (CFG_LPM_LEVEL != 0)
        if (STM32WLxx_SUBGHZ_RADIO_MODEM_MODE_FSK == drv_ctx.modem)
        {
            /* Prohibit Stop mode until Rx is done as we need fast response on Rx Done IRQ  */
            UTIL_LPM_SetStopMode((1u << CFG_LPM_SIDEWALK), UTIL_LPM_DISABLE);
#  if (CFG_LPM_STDBY_SUPPORTED != 0)
            UTIL_LPM_SetOffMode((1u << CFG_LPM_SIDEWALK), UTIL_LPM_DISABLE);
#  endif /* CFG_LPM_STDBY_SUPPORTED */
        }
        else
        {
            /* For LoRa link we don't need such rapid response. Additional 200-250us added by the wakeup from Stop LPM won't affect the link performance and we can save some power */
        }
#endif /* (CFG_LPM_LEVEL != 0) */

#if STM32WLxx_RADIO_CFG_USE_STATUS_LED
        /* Drive the status LED after the actual start of Rx to avoid impact on timings */
        stm32wlxx_radio_hal_rx_led_on(&drv_ctx);
#endif /* STM32WLxx_RADIO_CFG_USE_STATUS_LED */

        err = RADIO_ERROR_NONE;
    } while (0);

    radio_stm32wlxx_error_manager(err, FALSE);
    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_is_cad_exit_mode(sid_pal_radio_cad_param_exit_mode_t exit_mode)
{
    int32_t err;

    if ( (exit_mode != SID_PAL_RADIO_CAD_EXIT_MODE_CS_ONLY)
      && (exit_mode != SID_PAL_RADIO_CAD_EXIT_MODE_CS_RX)
      && (exit_mode != SID_PAL_RADIO_CAD_EXIT_MODE_CS_LBT)
      && (exit_mode != SID_PAL_RADIO_CAD_EXIT_MODE_ED_ONLY)
      && (exit_mode != SID_PAL_RADIO_CAD_EXIT_MODE_ED_RX)
      && (exit_mode != SID_PAL_RADIO_CAD_EXIT_MODE_ED_LBT))
    {
        err = RADIO_ERROR_INVALID_PARAMS;
    }
    else
    {
        err = RADIO_ERROR_NONE;
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_start_carrier_sense(const sid_pal_radio_fsk_cad_params_t * cad_params, sid_pal_radio_cad_param_exit_mode_t exit_mode)
{
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_2
    int32_t err = RADIO_ERROR_GENERIC;
    stm32wlxx_hal_status_t hal_err;

    STM32WLxx_LOG_DEBUG("sid_pal_radio_start_carrier_sense... ");

    //FIXME: That's not guaranteed: timeout and exit_mode are never changed and are part of global config that is sent to Wl55
    (void)cad_params;
    if (SID_STM32_UTIL_fast_memcmp(&drv_ctx.applied_fsk_phy_cfg.fsk_cad_params, cad_params, sizeof(*cad_params)) != 0u)
    {
        SID_PAL_LOG_WARNING("CS mismatch: %u vs %u", drv_ctx.applied_fsk_phy_cfg.fsk_cad_params.fsk_cs_duration_us, cad_params->fsk_cs_duration_us);
    }

    do
    {
        /* Validate params */
        err = sid_pal_radio_is_cad_exit_mode(exit_mode);
        if (err != RADIO_ERROR_NONE)
        {
            break;
        }

        /* Initiate CS */
        hal_err = stm32wlxx_hal_radio_start_carrier_sense(&drv_ctx);
        if (hal_err != STM32WLxx_HAL_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to start STM32WLxx SubGHz Carrier Sense. HAL error %u", (uint32_t)hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Update driver context */
        drv_ctx.radio_state = SID_PAL_RADIO_RX;
        drv_ctx.cad_exit_mode = exit_mode;

#  if (CFG_LPM_LEVEL != 0)
        /* Prohibit Stop mode until CS is done as we need fast response on the radio IRQs */
        UTIL_LPM_SetStopMode((1u << CFG_LPM_SIDEWALK), UTIL_LPM_DISABLE);
#    if (CFG_LPM_STDBY_SUPPORTED != 0)
        UTIL_LPM_SetOffMode((1u << CFG_LPM_SIDEWALK), UTIL_LPM_DISABLE);
#    endif /* CFG_LPM_STDBY_SUPPORTED */
#  endif /* (CFG_LPM_LEVEL != 0) */


#  if STM32WLxx_RADIO_CFG_USE_STATUS_LED
        /* Drive the status LEDs after the actual start of CS to avoid impact on timings */
#    if (HALO_ENABLE_DIAGNOSTICS == 0)
        if (SID_PAL_RADIO_CAD_EXIT_MODE_CS_LBT == exit_mode)
        {
            /* This is carrier sense for FSK Tx */
            stm32wlxx_radio_hal_tx_led_on(&drv_ctx);
        }
        else
#    endif /* HALO_ENABLE_DIAGNOSTICS */
        {
            stm32wlxx_radio_hal_rx_led_on(&drv_ctx);
        }
#  endif /* STM32WLxx_RADIO_CFG_USE_STATUS_LED */

        err = RADIO_ERROR_NONE;
    } while (0);

    radio_stm32wlxx_error_manager(err, FALSE);
    return err;
#else
    return RADIO_ERROR_NOT_SUPPORTED;
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_start_continuous_rx(void)
{
#if HALO_ENABLE_DIAGNOSTICS
    int32_t err = RADIO_ERROR_GENERIC;
    stm32wlxx_hal_status_t hal_err;

    STM32WLxx_LOG_DEBUG("sid_pal_radio_start_continuous_rx... ");

    do
    {
        hal_err = stm32wlxx_hal_start_continuous_rx(&drv_ctx);
        if (hal_err != STM32WLxx_HAL_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to start STM32WLxx SubGHz Continuous Rx. HAL error %u", (uint32_t)hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        drv_ctx.radio_state = SID_PAL_RADIO_RX;

#  if (CFG_LPM_LEVEL != 0)
        if (STM32WLxx_SUBGHZ_RADIO_MODEM_MODE_FSK == drv_ctx.modem)
        {
            /* Prohibit Stop mode until Rx is done as we need fast response on Rx Done IRQ */
            UTIL_LPM_SetStopMode((1u << CFG_LPM_SIDEWALK), UTIL_LPM_DISABLE);
#    if (CFG_LPM_STDBY_SUPPORTED != 0)
            UTIL_LPM_SetOffMode((1u << CFG_LPM_SIDEWALK), UTIL_LPM_DISABLE);
#    endif /* CFG_LPM_STDBY_SUPPORTED */
        }
        else
        {
            /* For LoRa link we don't need such rapid response. Additional 200-250us added by the wakeup from Stop LPM won't affect the link performance and we can save some power */
        }
#  endif /* (CFG_LPM_LEVEL != 0) */

#  if STM32WLxx_RADIO_CFG_USE_STATUS_LED
        /* Drive the status LEDs after the actual start of Rx to avoid impact on timings */
        stm32wlxx_radio_hal_rx_led_on(&drv_ctx);
#  endif /* STM32WLxx_RADIO_CFG_USE_STATUS_LED */

        err = RADIO_ERROR_NONE;
    } while (0);

    return err;
#else
    return RADIO_ERROR_NOT_SUPPORTED;
#endif /* HALO_ENABLE_DIAGNOSTICS */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_set_rx_duty_cycle(uint32_t rx_time, uint32_t sleep_time)
{
#if HALO_ENABLE_DIAGNOSTICS
    //TODO: implement
    return RADIO_ERROR_NOT_SUPPORTED;
#else
    return RADIO_ERROR_NOT_SUPPORTED;
#endif /* HALO_ENABLE_DIAGNOSTICS */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_lora_start_cad(void)
{
#if HALO_ENABLE_DIAGNOSTICS
    //TODO: implement
    return RADIO_ERROR_NOT_SUPPORTED;
#else
    return RADIO_ERROR_NOT_SUPPORTED;
#endif /* HALO_ENABLE_DIAGNOSTICS */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int16_t sid_pal_radio_rssi(void)
{
#if HALO_ENABLE_DIAGNOSTICS
    //TODO: implement
    return RADIO_ERROR_NOT_SUPPORTED;
#else
    return RADIO_ERROR_NOT_SUPPORTED;
#endif /* HALO_ENABLE_DIAGNOSTICS */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_is_channel_free(uint32_t freq, int16_t threshold, uint32_t delay_us, bool * is_channel_free)
{
#if HALO_ENABLE_DIAGNOSTICS
    //TODO: implement
    return RADIO_ERROR_NOT_SUPPORTED;
#else
    return RADIO_ERROR_NOT_SUPPORTED;
#endif /* HALO_ENABLE_DIAGNOSTICS */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_random(uint32_t *random)
{
    SID_PAL_LOG_ERROR("sid_pal_radio_random() is not implemented");
    return RADIO_ERROR_NOT_SUPPORTED;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int16_t sid_pal_radio_get_ant_dbi(void)
{
    SID_PAL_ASSERT(drv_ctx.regional_radio_param != NULL);
    return drv_ctx.regional_radio_param->ant_dbi;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_get_cca_level_adjust(sid_pal_radio_data_rate_t data_rate, int8_t *adj_level)
{
    int32_t err;

    SID_PAL_ASSERT(drv_ctx.regional_radio_param != NULL);

    if ((data_rate <= SID_PAL_RADIO_DATA_RATE_INVALID) || (data_rate > SID_PAL_RADIO_DATA_RATE_MAX_NUM))
    {
        err = RADIO_ERROR_INVALID_PARAMS;
    }
    else
    {
        *adj_level = drv_ctx.regional_radio_param->cca_level_adjust[data_rate - 1];
        err = RADIO_ERROR_NONE;
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_get_chan_noise(uint32_t freq, int16_t *noise)
{
#if HALO_ENABLE_DIAGNOSTICS
    //TODO: implement
    return RADIO_ERROR_NOT_SUPPORTED;
#else
    return RADIO_ERROR_NOT_SUPPORTED;
#endif /* HALO_ENABLE_DIAGNOSTICS */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_get_radio_state_transition_delays(sid_pal_radio_state_transition_timings_t * state_delay)
{
    static_assert(sizeof(*state_delay) == sizeof(drv_ctx.config->state_timings));
    SID_STM32_UTIL_fast_memcpy(state_delay, &drv_ctx.config->state_timings, sizeof(*state_delay));
    return RADIO_ERROR_NONE;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_init(sid_pal_radio_event_notify_t notify, sid_pal_radio_irq_handler_t dio_irq_handler, sid_pal_radio_rx_packet_t *rx_packet)
{
    int32_t err = RADIO_ERROR_GENERIC;
    stm32wlxx_hal_status_t hal_err;
    int8_t tx_power;

    SID_PAL_LOG_DEBUG("sid_pal_radio_init... ");
    do
    {
        /* Validate the inputs -------------------------------------------------------*/
        if ((NULL == notify) || (NULL == dio_irq_handler) || (NULL == rx_packet))
        {
            STM32WLxx_LOG_ERROR("sid_pal_radio_init() aborted due to invalid input parameters");
            err = RADIO_ERROR_INVALID_PARAMS;
            break;
        }
        /*----------------------------------------------------------------------------*/

        /* Check if the driver is initialized already */
        if (drv_ctx.init_done != FALSE)
        {
            SID_PAL_LOG_WARNING("STM32WLxx driver is initialized already. Requesting deinitialization");

            /* Deinitialize any hardware that may have been partially initialized to bring it to the known state */
            err = sid_pal_radio_deinit();
            if (err != RADIO_ERROR_NONE)
            {
                SID_PAL_LOG_ERROR("Unable to initialize STM32WLxx radio - driver is initialized already and deinitialization failed. Error %d", err);
                break;
            }
        }
        /*----------------------------------------------------------------------------*/

        /* Ensure STM32WLxx won't trigger any interrupt if we are re-initializing ----*/
        hal_err = stm32wlxx_hal_disarm_irq(&drv_ctx);
        if (hal_err != STM32WLxx_HAL_STATUS_OK)
        {
            /* Logs are provided by the stm32wlxx_hal_disarm_irq() */
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }
        __COMPILER_BARRIER(); /* Ensure IRQ is disarmed right now before any further actions take place */
        /*----------------------------------------------------------------------------*/

        /* Initialize variables, statuses, etc. --------------------------------------*/
        /* Create a binary semaphore to wait for Request Completed IRQ when required */
        SID_PAL_ASSERT(NULL == drv_ctx.req_completed_wait_lock);
        drv_ctx.req_completed_wait_lock = osSemaphoreNew(1u, 1u, &wlxx_request_ack_lock_attributes);

        /* Check iof the semaphore was actually created */
        if (NULL == drv_ctx.req_completed_wait_lock)
        {
            SID_PAL_LOG_ERROR("Failed to create semaphore for Request Processed IRQ handling");
            err = RADIO_ERROR_NOMEM;
            break;
        }
        /*----------------------------------------------------------------------------*/

        /* Create a binary semaphore to wait for a generic IRQ indication when required */
        SID_PAL_ASSERT(NULL == drv_ctx.irq_detected_wait_lock);
        drv_ctx.irq_detected_wait_lock = osSemaphoreNew(1u, 0u, &wlxx_irq_wait_lock_attributes);

        /* Check iof the semaphore was actually created */
        if (NULL == drv_ctx.irq_detected_wait_lock)
        {
            SID_PAL_LOG_ERROR("Failed to create semaphore for IRQ detection handling");
            err = RADIO_ERROR_NOMEM;
            break;
        }
        /*----------------------------------------------------------------------------*/

#if STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER
        /* Initialize User Data Transfer service -------------------------------------*/
        /* Create UDT message queue */
        if (NULL == drv_ctx.udt_ctx.outbound_msg_queue)
        {
            drv_ctx.udt_ctx.outbound_msg_queue = osMessageQueueNew(STM32WLxx_UDT_OUT_MSG_QUEUE_LEN, sizeof(stm32wlxx_ext_ifc_out_msg_desc_t), NULL);
            if (NULL == drv_ctx.udt_ctx.outbound_msg_queue)
            {
                SID_PAL_LOG_ERROR("Can't create UDT message queue. No memory");
                err = RADIO_ERROR_NOMEM;
                break;
            }
        }

        /* Create the task that will handle the UDT */
        if (NULL == drv_ctx.udt_ctx.task)
        {
            drv_ctx.udt_ctx.task = osThreadNew(_user_data_transfer_thread, NULL, &wlxx_user_data_transfer_thread_attributes);
            if (NULL == drv_ctx.udt_ctx.task)
            {
                SID_PAL_LOG_ERROR("Can't create UDT processing thread. No memory");
                err = RADIO_ERROR_NOMEM;
                break;
            }
        }
#endif /* STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER */

        /* Configure driver context with the supplied parameters */
        drv_ctx.radio_rx_packet      = rx_packet;
        drv_ctx.report_radio_event   = notify;
        drv_ctx.irq_handler          = dio_irq_handler;

#if ((SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 != 0) && (SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 == 0))
        /* FSK-only SubGHz link, default to FSK modem mode for initialization */
        drv_ctx.modem                = STM32WLxx_SUBGHZ_RADIO_MODEM_MODE_FSK;
        drv_ctx.subghz_irq_mask      = STM32WLxx_RADIO_COMM_DEFAULT_FSK_IRQ_MASK;
#else
        /* LoRa-only or FSK-LoRa SubGHz link - default to LoRa modem mode for initialization */
        drv_ctx.modem                = STM32WLxx_SUBGHZ_RADIO_MODEM_MODE_LORA;
        drv_ctx.subghz_irq_mask      = STM32WLxx_RADIO_COMM_DEFAULT_LORA_IRQ_MASK;
#endif

        /* Store the selected region the in driver context. It won't be sent to the STM32WLxx immediately */
        err = sid_pal_radio_set_region(drv_ctx.config->regional_config.radio_region);
        if (err != RADIO_ERROR_NONE)
        {
            STM32WLxx_LOG_ERROR("sid_pal_radio_set_region() failed with error %d", err);
            break;
        }

        /* At this point the SubGHz radio and the entire STM32WLxx MCU may be in any state (e.g. in LPM Sleep or Stop) */
        drv_ctx.radio_state = SID_PAL_RADIO_UNKNOWN;
        /*----------------------------------------------------------------------------*/

        /* Initialize the underlying hardware ----------------------------------------*/
        err = radio_stm32wlxx_platform_init();
        if (err != RADIO_ERROR_NONE)
        {
            STM32WLxx_LOG_ERROR("radio_stm32wlxx_platform_init() failed with error %d", err);
            break;
        }

        /* Perform multiple attempts of handshaking with the STM32WLxx ---------------*/
        uint32_t handshake_attempts_counter = 0u;
        do
        {
            handshake_attempts_counter++;

            err = radio_stm32wlxx_do_handshake();
            if ((RADIO_ERROR_NONE == err) || (RADIO_ERROR_NOT_SUPPORTED == err))
            {
                /* Exit either on success or unrecoverable error */
                break;
            }
        } while (handshake_attempts_counter < STM32WLxx_HANDSHAKE_ATTEMPTS_LIMIT);

        /* Check if Handshake procedure succeeded */
        if (err != RADIO_ERROR_NONE)
        {
            STM32WLxx_LOG_ERROR("radio_stm32wlxx_do_handshake() failed with error %d", err);
            break;
        }
        /*----------------------------------------------------------------------------*/

        /* Now configure STM32WLxx IRQ line in as a regular IRQ line -----------------*/
        hal_err = stm32wlxx_hal_arm_irq(&drv_ctx);
        if (hal_err != STM32WLxx_HAL_STATUS_OK)
        {
            /* Logs are provided by the stm32wlxx_hal_arm_irq() */
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Enable STM32WLxx Sidewalk Radio App-level IRQs */
        drv_ctx.app_irq_mask = STM32WLxx_ALL_APP_IRQ_MASK;
        /*----------------------------------------------------------------------------*/

        /* Ensure STM32WLxx's SubGHz radio is in Standby -----------------------------*/
        err = sid_pal_radio_standby();
        if (err != RADIO_ERROR_NONE)
        {
            STM32WLxx_LOG_ERROR("sid_pal_radio_standby() failed with error %d", err);
            break;
        }
        /*----------------------------------------------------------------------------*/

        /* Setup power amplifier config values - not sent immediately ----------------*/
        err = sid_pal_radio_get_max_tx_power(SID_PAL_RADIO_DATA_RATE_50KBPS, &tx_power);
        if (err != RADIO_ERROR_NONE)
        {
        	STM32WLxx_LOG_ERROR("sid_pal_radio_get_max_tx_power() failed with error code %d", err);
            break;
        }
        err = radio_stm32wlxx_set_tx_power_config(tx_power);
        if (err != RADIO_ERROR_NONE)
        {
        	STM32WLxx_LOG_ERROR("radio_stm32wlxx_set_tx_power_config() failed with error code %d", err);
            break;
        }
        /*----------------------------------------------------------------------------*/

        /* Send out SubGHz radio configuration via SPI to STM32WLxx ------------------*/
        err = stm32wlxx_radio_send_subghz_config(TRUE);
        if (err != RADIO_ERROR_NONE)
        {
            /* Logs are provided by stm32wlxx_radio_send_subghz_config() */
            break;
        }

        /* Perform the final STM32WLxx Sidewalk Radio App initalization steps */
        hal_err = stm32wlxx_hal_apply_base_hw_config(&drv_ctx, TRUE);
        if (hal_err != STM32WLxx_HAL_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to apply essential SubGHz HW config. Error code %u", (uint32_t)hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* STM32WLxx puts SubGHz radio back to Standby mode after HW init */

        /* Explicitly set modem mode on the remote side */
        hal_err = stm32wlxx_hal_radio_set_modem_mode(&drv_ctx, drv_ctx.modem);
        if (hal_err != STM32WLxx_HAL_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to set SubGHz modem mode. Error code %u", (uint32_t)hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }
        /*----------------------------------------------------------------------------*/

        /* Initialization sequence is finished with no issues */
        err = RADIO_ERROR_NONE;
    } while (0);

    if (err != RADIO_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("Unable to initialize STM32WLxx radio. Error %d", err);

        /* Ensure we deinitialize any hardware that may have been partially initialized */
        (void)sid_pal_radio_deinit();
    }
    else
    {
        drv_ctx.init_done = TRUE;
        SID_PAL_LOG_INFO("STM32WLxx radio initialized");
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_deinit(void)
{
    int32_t err = RADIO_ERROR_GENERIC;
    osStatus_t os_err;

    SID_PAL_LOG_DEBUG("sid_pal_radio_deinit... ");

    do
    {
#if STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER
        drv_ctx.udt_ctx.udt_enabled = FALSE;
        __COMPILER_BARRIER();
#endif /* STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER */

        /* Bring down the underlying hardware */
        err = radio_stm32wlxx_platform_deinit();
        if (err != RADIO_ERROR_NONE)
        {
            STM32WLxx_LOG_ERROR("radio_stm32wlxx_platform_deinit() failed. Error %d", err);
            break;
        }

        /* From here radio IRQs won't be processed */

        /* Delete the Request Completed lock */
        os_err = osSemaphoreDelete(drv_ctx.req_completed_wait_lock);
        SID_PAL_ASSERT(osOK == os_err); /* osSemaphoreDelete() can fail only on systematic issues, so assertion is better and more efficient here */
        drv_ctx.req_completed_wait_lock = NULL;

        /* Delete the IRQ Detection lock */
        os_err = osSemaphoreDelete(drv_ctx.irq_detected_wait_lock);
        SID_PAL_ASSERT(osOK == os_err); /* osSemaphoreDelete() can fail only on systematic issues, so assertion is better and more efficient here */
        drv_ctx.irq_detected_wait_lock = NULL;

        /* Invalidate the context */
        drv_ctx.init_done            = FALSE;
        drv_ctx.irq_enabled          = FALSE;
        drv_ctx.app_irq_mask         = STM32WLxx_APP_IRQ_NONE;
        drv_ctx.subghz_irq_mask      = STM32WLxx_SUBGHZ_IRQ_NONE;
        drv_ctx.req_with_ack_ongoing = FALSE;
        drv_ctx.modem                = STM32WLxx_SUBGHZ_RADIO_MODEM_MODE_UNDEFINED;
        drv_ctx.radio_rx_packet      = NULL;
        drv_ctx.radio_freq_hz        = 0u;
        drv_ctx.radio_state          = SID_PAL_RADIO_UNKNOWN;
        SID_STM32_UTIL_fast_memset(&drv_ctx.last_irq_status,      0u, sizeof(drv_ctx.last_irq_status));
        SID_STM32_UTIL_fast_memset(&drv_ctx.error_monitor,        0u, sizeof(drv_ctx.error_monitor));

#if STM32WLxx_RADIO_APP_LPM_SUPPORT
        drv_ctx.ldt_ongoing          = FALSE;
#endif /* STM32WLxx_RADIO_APP_LPM_SUPPORT */

#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_2
        SID_STM32_UTIL_fast_memset(&drv_ctx.applied_fsk_phy_cfg,  0u, sizeof(drv_ctx.applied_fsk_phy_cfg));
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 */

#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
        SID_STM32_UTIL_fast_memset(&drv_ctx.applied_lora_phy_cfg, 0u, sizeof(drv_ctx.applied_lora_phy_cfg));
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 */
    } while (0);

    if (err != RADIO_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("Unable to deinitialize STM32WLxx radio. Error %d", err);
    }
    else
    {
        SID_PAL_LOG_INFO("STM32WLxx radio deinitialized");
    }

#if (CFG_LPM_LEVEL != 0)
    /* Allow Stop mode after the radio driver is deinitialized */
    UTIL_LPM_SetStopMode((1u << CFG_LPM_SIDEWALK), UTIL_LPM_ENABLE);
#  if (CFG_LPM_STDBY_SUPPORTED != 0)
    UTIL_LPM_SetOffMode((1u << CFG_LPM_SIDEWALK), UTIL_LPM_ENABLE);
#  endif /* CFG_LPM_STDBY_SUPPORTED */
#endif /* (CFG_LPM_LEVEL != 0) */

    return err;
}

/*----------------------------------------------------------------------------*/

#if SID_SDK_CONFIG_RF_TIMINGS_MEASUREMENT
SID_STM32_SPEED_OPTIMIZED void sid_pal_radio_rxtx_start_cb(void)
{
    /* Keep this function as short as possible to minimize its contribution to the Rx/Tx processing delays */
    SIDEWALK_RF_TIMINGS_RXTX_START_GPIO_Port->BSRR = SIDEWALK_RF_TIMINGS_RXTX_START_Pin;
}
#endif /* SID_SDK_CONFIG_RF_TIMINGS_MEASUREMENT */
