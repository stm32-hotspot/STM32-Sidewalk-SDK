/**
  ******************************************************************************
  * @file    smtc_modem_hal.c
  * @brief   Hardware abstractions for LoRa Basics Modem leveraging Sidewalk API
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

/* Private macro -------------------------------------------------------------*/

#if defined(STM32WBA50xx) || defined(STM32WBA52xx) || defined(STM32WBA54xx) || defined(STM32WBA55xx) || defined(STM32WBA5Mxx) || \
    defined(STM32WBA62xx) || defined(STM32WBA63xx) || defined(STM32WBA64xx) || defined(STM32WBA65xx) || defined(STM32WBA6Mxx)
#  define STM32WBAxx_FAMILY
#  else
#    error "Unable to identify the target STM32 MCU. Please check if you have specified the MCU type in compile definitions and that your selected MCU is supported"
#endif

/* Includes ------------------------------------------------------------------*/

#include <string.h>
#include <cmsis_compiler.h>

/* Sidewalk interfaces */
#include <sid_hal_reset_ifc.h>
#include <sid_pal_assert_ifc.h>
#include <sid_pal_crypto_ifc.h>
#include <sid_pal_critical_region_ifc.h>
#include <sid_pal_log_ifc.h>
#include <sid_pal_storage_kv_ifc.h>
#include <sid_pal_temperature_ifc.h>
#include <sid_pal_timer_ifc.h>
#include <sid_pal_uptime_ifc.h>
#include <sid_time_ops.h>
#if defined(STM32WBAxx_FAMILY)
#  include <sid_pal_gpio_ext_ifc.h>
#endif /* STM32WBAxx_FAMILY */

/* LoRa Basics Modem (LBM) interfaces */
#include <lr1mac_defs.h>
#include <smtc_modem_hal.h>
#include <smtc_modem_hal_dbg_trace.h>

/* Sidewalk radio driver */
#if defined(LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION) && (LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION != 0)
#  include "lr11xx_hal.h"
#  include "halo_lr11xx_radio.h"
#elif defined(SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION) && (SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION != 0)
#  include "sx126x_hal.h"
#  include "sx126x_radio.h"
#endif /* SID_RADIO_PLATFORM */

/* App-specific headers */
#include "app_conf.h"
#include "utilities_conf.h"

/* Platform-specific includes */
#  include <adc_ctrl.h>
#if defined(BLE) && (USE_TEMPERATURE_BASED_RADIO_CALIBRATION != 0)
#  include <cmsis_os2.h>
#  include <ll_sys_if.h>
#endif /* BLE && USE_TEMPERATURE_BASED_RADIO_CALIBRATION */
#include <sid_stm32_common_utils.h>
#include "stm32_adv_trace.h"
#if defined(STM32WBAxx_FAMILY)
#  include <stm32wbaxx.h>
#  include <stm32wbaxx_ll_adc.h>
#endif /* STM32WBAxx_FAMILY */

/* Private defines -----------------------------------------------------------*/

#define LBM_CRASH_LOG_SIZE                                    (CRASH_LOG_SIZE)
#define LBM_CRASH_LOG_VALID_WATERMARK                         (0xDEADBEEFu)

#define LBM_LOG_TYPE_INFO_PREFIX                              MODEM_HAL_DBG_TRACE_COLOR_GREEN "INFO: "
#define LBM_LOG_TYPE_WARN_PREFIX                              MODEM_HAL_DBG_TRACE_COLOR_YELLOW "WARN: "
#define LBM_LOG_TYPE_ERROR_PREFIX                             MODEM_HAL_DBG_TRACE_COLOR_RED "ERROR: "
#define LBM_LOG_PREFIX_MAX_LEN                                (MAX(MAX(sizeof(LBM_LOG_TYPE_INFO_PREFIX), sizeof(LBM_LOG_TYPE_WARN_PREFIX)), sizeof(LBM_LOG_TYPE_ERROR_PREFIX)) -1u)

#define LBM_LOG_SID_PREFIX                                    "\e[1;36m" "[LBM] " "\e[0m"

#ifndef STORAGE_KV_LBM_CONTEXT_STORAGE_GROUP
#  define STORAGE_KV_LBM_CONTEXT_STORAGE_GROUP                (0x1BC7u)
#endif

#if (STORAGE_KV_LBM_CONTEXT_STORAGE_GROUP >= 0x6FFEu)
#  error "Do not use group Ids greater than 0x6FFE"
#endif
#if (STORAGE_KV_LBM_CONTEXT_STORAGE_GROUP == SID_PAL_STORAGE_KV_INTERNAL_PROTOCOL_GROUP_ID)
#  error "Invalid KV Storage Group IDs config: STORAGE_KV_LBM_CONTEXT_STORAGE_GROUP is equal to SID_PAL_STORAGE_KV_INTERNAL_PROTOCOL_GROUP_ID"
#endif
#if (STORAGE_KV_LBM_CONTEXT_STORAGE_GROUP == SID_PAL_STORAGE_KV_INTERNAL_CONFIG_GROUP_ID)
#  error "Invalid KV Storage Group IDs config: STORAGE_KV_LBM_CONTEXT_STORAGE_GROUP is equal to SID_PAL_STORAGE_KV_INTERNAL_CONFIG_GROUP_ID"
#endif
#if (STORAGE_KV_LBM_CONTEXT_STORAGE_GROUP == SID_PAL_STORAGE_KV_INTERNAL_BULK_DATA_TRANSFER_GROUP_ID)
#  error "Invalid KV Storage Group IDs config: STORAGE_KV_LBM_CONTEXT_STORAGE_GROUP is equal to SID_PAL_STORAGE_KV_INTERNAL_BULK_DATA_TRANSFER_GROUP_ID"
#endif

#define STORAGE_KV_LBM_CONTEXT_MODEM_KEY                      (0xC700u)
#define STORAGE_KV_LBM_CONTEXT_KEY_MODEM_KEY                  (0xC701u)
#define STORAGE_KV_LBM_CONTEXT_SECURE_ELEMENT_KEY             (0xC703u)
#define STORAGE_KV_LBM_CONTEXT_LORAWAN_STACK_KEY_BASE         (0xC800u) /*!< Range from 0xC800u to (0xC800u + NUMBER_OF_STACKS - 1u) is reserved for LBM stack context storage */

#if defined(NDEBUG) && !defined(LBM_REBOOT_ON_PANIC)
#  warning "You are assembling a release build without enabling MCU reset on LoRa Basics Modem panic. System will be halted on LBM errors"
#endif /* DEBUG && LBM_REBOOT_ON_PANIC */

#ifndef SMTC_LBM_SID_INTEGRATION_SUPPLY_VOLTAGE_ADC_CHANNEL
#  define SMTC_LBM_SID_INTEGRATION_SUPPLY_VOLTAGE_ADC_CHANNEL (LL_ADC_CHANNEL_VCORE) /*!< Specifies which ADC channel is used to measure MCU supply voltage. User application may override this definition */
#endif /* SMTC_LBM_SID_INTEGRATION_SUPPLY_VOLTAGE_ADC_CHANNEL */

/* Private variables ---------------------------------------------------------*/

/* Crash log storage */
UTIL_MEM_PLACE_IN_SECTION(".no_init")
static uint8_t lbm_crash_log_buf[LBM_CRASH_LOG_SIZE];
UTIL_MEM_PLACE_IN_SECTION(".no_init")
static uint16_t lbm_crash_log_valid_length;
UTIL_MEM_PLACE_IN_SECTION(".no_init")
static uint32_t lbm_crash_log_validity_marker;

/* Radio IRQ handling */
static void (*lbm_hal_radio_irq_cb)(void * context);
static void * lbm_hal_radio_irq_ctx;

/* Radio event scheduling */
static sid_pal_timer_t lbm_hal_timer;
static bool lbm_hal_timer_initialized = false;
static void (*lbm_hal_timer_callback)(void* context) = NULL;
static volatile bool lbm_hal_timer_irq_enabled = true;
static volatile bool lbm_hal_timer_irq_pending = false;

/* MCU supply voltage measurement */
static ADCCTRL_Handle_t supply_voltage_meas_handle = {
    .Uid         = 0x00, /* Uid is assigned by ADC_Ctrl as a part of ADCCTRL_RegisterHandle() */
    .State       = ADCCTRL_HANDLE_NOT_REG,
    .InitConf    = {
        .ConvParams = {
            .TriggerFrequencyMode = LL_ADC_TRIGGER_FREQ_LOW,
            .Resolution           = LL_ADC_RESOLUTION_12B,
            .DataAlign            = LL_ADC_DATA_ALIGN_RIGHT,
            .TriggerStart         = LL_ADC_REG_TRIG_SOFTWARE,
            .TriggerEdge          = LL_ADC_REG_TRIG_EXT_RISING,
            .ConversionMode       = LL_ADC_REG_CONV_SINGLE,
            .DmaTransfer          = LL_ADC_REG_DMA_TRANSFER_NONE,
            .Overrun              = LL_ADC_REG_OVR_DATA_OVERWRITTEN,
            .SamplingTimeCommon1  = LL_ADC_SAMPLINGTIME_814CYCLES_5,
            .SamplingTimeCommon2  = LL_ADC_SAMPLINGTIME_1CYCLE_5,
        },
        .SeqParams = {
            .Setup                = LL_ADC_REG_SEQ_CONFIGURABLE,
            .Length               = LL_ADC_REG_SEQ_SCAN_DISABLE,
            .DiscMode             = LL_ADC_REG_SEQ_DISCONT_DISABLE,
        },
        .LowPowerParams = {
            .AutoPowerOff         = DISABLE,
            .AutonomousDPD        = LL_ADC_LP_AUTONOMOUS_DPD_DISABLE,
        }
    },
    .ChannelConf = {
        .Channel                  = LL_ADC_CHANNEL_VCORE,
        .Rank                     = LL_ADC_REG_RANK_1,
        .SamplingTime             = LL_ADC_SAMPLINGTIME_COMMON_1,
    }
};
static int16_t last_measured_supply_voltage = 0;
#if defined(BLE) && (USE_TEMPERATURE_BASED_RADIO_CALIBRATION != 0)
static uint8_t ll_lock_valid;
#endif /* BLE && USE_TEMPERATURE_BASED_RADIO_CALIBRATION */

/* Imported function prototypes ----------------------------------------------*/

#ifndef LBM_REBOOT_ON_PANIC
void Error_Handler(void);
#endif /* LBM_REBOOT_ON_PANIC */

/* Private function prototypes -----------------------------------------------*/

static inline int      snprintf_like(char * buf, const size_t size, const char *fmt, ...);
static        void     lbm_hal_timer_callback_proxy(void * arg, sid_pal_timer_t * originator);
static inline uint32_t adc_hw_obtain_control(void);
static inline void     adc_hw_release_control(void);

/* Private function definitions ----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline int snprintf_like(char * buf, const size_t size, const char *fmt, ...)
{
    va_list args;
    int print_len;

    va_start(args, fmt);
    print_len = UTIL_ADV_TRACE_VSNPRINTF(buf, size, fmt, args);
    va_end(args);

    return print_len;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static void lbm_hal_timer_callback_proxy(void * arg, sid_pal_timer_t * originator)
{
    (void)originator;

    /* As per LBM documentation, LBM expects the timer callback to be executed with IRQs disabled */
    sid_pal_enter_critical_region();
    if (lbm_hal_timer_irq_enabled != false)
    {
        /* Call LBM callback for timer event */
        if (lbm_hal_timer_callback != NULL)
        {
            lbm_hal_timer_callback(arg);
        }
    }
    else
    {
        /* Timer events are masked, set Pending flag and do nothing more */
        lbm_hal_timer_irq_pending = true;
    }
    sid_pal_exit_critical_region();
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline uint32_t adc_hw_obtain_control(void)
{
#if defined(BLE) && (USE_TEMPERATURE_BASED_RADIO_CALIBRATION != 0)
    if (NULL == LinkLayerMutex)
    {
        ll_lock_valid = FALSE;
        return TRUE;
    }
    else
    {
        ll_lock_valid = TRUE;

        /* Wait until BLE Link Layer thread releases all the resources */
        osStatus_t os_err = osMutexAcquire(LinkLayerMutex, osWaitForever);
        return (osOK == os_err) ? TRUE : FALSE;
    }
#else
    return TRUE;
#endif /* BLE && USE_TEMPERATURE_BASED_RADIO_CALIBRATION */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline void adc_hw_release_control(void)
{
#if defined(BLE) && (USE_TEMPERATURE_BASED_RADIO_CALIBRATION != 0)
    if (ll_lock_valid != FALSE)
    {
        /* Signal BLE Link Layer that HW resources is now free */
        (void)osMutexRelease(LinkLayerMutex);
    }
#endif /* BLE && USE_TEMPERATURE_BASED_RADIO_CALIBRATION */
}

/* Global function definitions -----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED void smtc_modem_hal_reset_mcu(void)
{
    /* Ensure all the remaining logs are printed out */
    SID_PAL_LOG_FLUSH();

    /* Trigger a soft reset */
    sid_hal_reset(SID_HAL_RESET_NORMAL);
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED void smtc_modem_hal_reload_wdog(void)
{
    /* Nothing to do here */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED uint32_t smtc_modem_hal_get_time_in_s(void)
{
    struct sid_timespec now;
    uint32_t now_s;

    (void)sid_pal_uptime_now(&now);
    now_s = (uint32_t)(sid_timespec_to_ms_64(&now) / (uint64_t)1000u);

    return now_s;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED uint32_t smtc_modem_hal_get_time_in_ms(void)
{
    struct sid_timespec now;
    uint32_t now_ms;

    (void)sid_pal_uptime_now(&now);
    now_ms = sid_timespec_to_ms(&now);

    return now_ms;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED uint32_t smtc_modem_hal_get_time_in_100us(void)
{
    struct sid_timespec now;
    uint32_t now_100us;

    (void)sid_pal_uptime_now(&now);
    now_100us = (uint32_t)(sid_timespec_to_us_64(&now) / (uint64_t)100u);

    return now_100us;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED void smtc_modem_hal_start_timer(const uint32_t milliseconds, void ( *callback )( void* context ), void * context)
{
    struct sid_timespec alarm_time;

    sid_pal_enter_critical_region();
    if (false == lbm_hal_timer_initialized)
    {
        (void)sid_pal_timer_init(&lbm_hal_timer, lbm_hal_timer_callback_proxy, context);
        lbm_hal_timer_initialized = true;
    }
    else if (callback != lbm_hal_timer_callback)
    {
        (void)sid_pal_timer_deinit(&lbm_hal_timer);
        lbm_hal_timer_callback = callback;
        (void)sid_pal_timer_init(&lbm_hal_timer, lbm_hal_timer_callback_proxy, context);
    }
    else if (sid_pal_timer_is_armed(&lbm_hal_timer) != false)
    {
        (void)sid_pal_timer_cancel(&lbm_hal_timer);
    }
    else
    {
        /* Nothing to do here */
    }
    sid_pal_exit_critical_region();

    (void)sid_pal_uptime_now(&alarm_time);
    sid_add_ms_to_timespec(&alarm_time, milliseconds);

    (void)sid_pal_timer_arm(&lbm_hal_timer, SID_PAL_TIMER_PRIO_CLASS_PRECISE, &alarm_time, NULL);
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED void smtc_modem_hal_stop_timer(void)
{
    sid_pal_enter_critical_region();
    if ((lbm_hal_timer_initialized != false) && (sid_pal_timer_is_armed(&lbm_hal_timer) != false))
    {
        (void)sid_pal_timer_cancel(&lbm_hal_timer);
    }
    sid_pal_exit_critical_region();
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED void smtc_modem_hal_disable_modem_irq(void)
{
    sid_pal_enter_critical_region();

#if defined(SID_RADIO_PLATFORM_LR11XX)
    const halo_drv_semtech_ctx_t * const drv_ctx = lr11xx_get_drv_ctx();
#elif defined(SID_RADIO_PLATFORM_SX126X)
    const halo_drv_semtech_ctx_t * const drv_ctx = sx126x_get_drv_ctx();
#endif /* SID_RADIO_PLATFORM */

    /* Validate inputs */
    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->config != NULL);

    /* Disable radio IRQ */
#if defined(SID_RADIO_PLATFORM_LR11XX)
    (void)lr11xx_hal_disarm_irq(drv_ctx);
#elif defined(SID_RADIO_PLATFORM_SX126X)
    (void)sx126x_hal_disarm_irq(drv_ctx);
#endif /* SID_RADIO_PLATFORM */

    /* Suppress LBM HAL timer events */
    lbm_hal_timer_irq_enabled = false;

    sid_pal_exit_critical_region();
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED void smtc_modem_hal_enable_modem_irq(void)
{
    sid_pal_enter_critical_region();

#if defined(SID_RADIO_PLATFORM_LR11XX)
    halo_drv_semtech_ctx_t * const drv_ctx = lr11xx_get_drv_ctx();
#elif defined(SID_RADIO_PLATFORM_SX126X)
    halo_drv_semtech_ctx_t * const drv_ctx = sx126x_get_drv_ctx();
#endif /* SID_RADIO_PLATFORM */

    /* Validate inputs */
    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->config != NULL);

    /* Enable radio IRQ */
#if defined(SID_RADIO_PLATFORM_LR11XX)
    (void)lr11xx_hal_arm_irq(drv_ctx);
#elif defined(SID_RADIO_PLATFORM_SX126X)
    (void)sx126x_hal_arm_irq(drv_ctx);
#endif /* SID_RADIO_PLATFORM */

    /* Re-enable LBM HAL timer events */
    lbm_hal_timer_irq_enabled = true;

    /* Invoke the callback if any timer event is pending */
    if (lbm_hal_timer_irq_pending != false)
    {
        lbm_hal_timer_irq_pending = false;
        if (lbm_hal_timer_callback != NULL)
        {
            lbm_hal_timer_callback(lbm_hal_timer.callback_arg);
        }
    }
    sid_pal_exit_critical_region();
}

/*----------------------------------------------------------------------------*/

__WEAK SID_STM32_SPEED_OPTIMIZED sid_error_t smtc_modem_hal_fuota_context_restore(uint32_t offset, uint8_t * buffer, const uint32_t size)
{
    /**
     * Override this method in your application if you plan to use FUOTA mechanism provided by LoRa Basics Modem.
     * The parameters for this method are transparently passed from the smtc_modem_hal_context_store() call.
     * This method should return SID_ERROR_NONE on success or an error code otherwise
     */
    (void)offset;
    (void)buffer;
    (void)size;
    return SID_ERROR_NOSUPPORT;
}

/*----------------------------------------------------------------------------*/

__WEAK SID_STM32_SPEED_OPTIMIZED sid_error_t smtc_modem_hal_store_and_forward_context_restore(uint32_t offset, uint8_t * buffer, const uint32_t size)
{
    /**
     * Override this method in your application if you plan to use Store-and-Forward mechanism provided by LoRa Basics Modem.
     * The parameters for this method are transparently passed from the smtc_modem_hal_context_store() call.
     * This method should return SID_ERROR_NONE on success or an error code otherwise
     */
    (void)offset;
    (void)buffer;
    (void)size;
    return SID_ERROR_NOSUPPORT;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED void smtc_modem_hal_context_restore(const modem_context_type_t ctx_type, uint32_t offset, uint8_t * buffer, const uint32_t size)
{
    sid_error_t err;
    uint16_t kv_sotrage_key;

    do
    {
        uint32_t kv_available_len;

        /* Select corresponding key for Sidewalk Key-Value storage */
        switch (ctx_type)
        {
            case CONTEXT_MODEM:
                kv_sotrage_key = STORAGE_KV_LBM_CONTEXT_MODEM_KEY;
                err = (0u == offset) ? SID_ERROR_NONE : SID_ERROR_PARAM_OUT_OF_RANGE;
                break;

            case CONTEXT_KEY_MODEM:
                kv_sotrage_key = STORAGE_KV_LBM_CONTEXT_KEY_MODEM_KEY;
                err = (0u == offset) ? SID_ERROR_NONE : SID_ERROR_PARAM_OUT_OF_RANGE;
                break;

            case CONTEXT_LORAWAN_STACK:
                kv_sotrage_key = offset / sizeof(lr1_mac_nvm_context_t);
                if ((kv_sotrage_key >= NUMBER_OF_STACKS) || ((offset % sizeof(lr1_mac_nvm_context_t)) != 0u))
                {
                    err = SID_ERROR_PARAM_OUT_OF_RANGE;
                }
                else
                {
                    kv_sotrage_key += STORAGE_KV_LBM_CONTEXT_LORAWAN_STACK_KEY_BASE;
                    err = SID_ERROR_NONE;
                }
                break;

            case CONTEXT_FUOTA:
                err = smtc_modem_hal_fuota_context_restore(offset, buffer, size);
                break;

            case CONTEXT_SECURE_ELEMENT:
                kv_sotrage_key = STORAGE_KV_LBM_CONTEXT_SECURE_ELEMENT_KEY;
                err = (0u == offset) ? SID_ERROR_NONE : SID_ERROR_PARAM_OUT_OF_RANGE;
                break;

            case CONTEXT_STORE_AND_FORWARD:
                err = smtc_modem_hal_store_and_forward_context_restore(offset, buffer, size);
                break;

            default:
                SID_PAL_LOG_ERROR(LBM_LOG_SID_PREFIX "cannot store context, unknown context type: 0x%02X", ctx_type);
                err = SID_ERROR_INVALID_ARGS;
                break;
        }

        /* Terminate on error */
        if (err != SID_ERROR_NONE)
        {
            if (SID_ERROR_PARAM_OUT_OF_RANGE == err)
            {
                SID_PAL_LOG_ERROR(LBM_LOG_SID_PREFIX "can't store context 0x%02X. Invalid offset: 0x%08X", ctx_type, offset);
            }
            break;
        }

        if ((ctx_type != CONTEXT_FUOTA) && (ctx_type != CONTEXT_STORE_AND_FORWARD))
        {
            /* Check if context is available by requesting the length of available data */
            err = sid_pal_storage_kv_record_get_len(STORAGE_KV_LBM_CONTEXT_STORAGE_GROUP, kv_sotrage_key, &kv_available_len);
            if (SID_ERROR_NOT_FOUND == err)
            {
                /* Context was not stored previously */
                err = SID_ERROR_NONE;
                break;
            }
            if (err != SID_ERROR_NONE)
            {
                SID_PAL_LOG_ERROR(LBM_LOG_SID_PREFIX "failed to retrieve context 0x%02X from KV storage, error code: %d", ctx_type, err);
                break;
            }
            if (kv_available_len != size)
            {
                /* Skip loading since records do not match */
                SID_PAL_LOG_WARNING(LBM_LOG_SID_PREFIX "context size mismatch on restore. Context: 0x%02X, requested len: %u, stored len: %u", ctx_type, size, kv_available_len);
                err = SID_ERROR_NONE;
                break;
            }

            /* Load the context from KV storage */
            err = sid_pal_storage_kv_record_get(STORAGE_KV_LBM_CONTEXT_STORAGE_GROUP, kv_sotrage_key, buffer, kv_available_len);
            if (err != SID_ERROR_NONE)
            {
                SID_PAL_LOG_ERROR(LBM_LOG_SID_PREFIX "unable to restore context 0x%02X from KV storage, error code: %d", ctx_type, err);
                break;
            }
        }

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    if (err != SID_ERROR_NONE)
    {
#ifdef LBM_REBOOT_ON_PANIC
        /* Reset the MCU */
        smtc_modem_hal_reset_mcu();
#else
        /* Halt the MCU */
        Error_Handler();
#endif /* LBM_REBOOT_ON_PANIC */
    }
}

/*----------------------------------------------------------------------------*/

__WEAK SID_STM32_SPEED_OPTIMIZED sid_error_t smtc_modem_hal_fuota_context_store(uint32_t offset, const uint8_t * buffer, const uint32_t size)
{
    /**
     * Override this method in your application if you plan to use FUOTA mechanism provided by LoRa Basics Modem.
     * The parameters for this method are transparently passed from the smtc_modem_hal_context_store() call.
     * This method should return SID_ERROR_NONE on success or an error code otherwise
     */
    (void)offset;
    (void)buffer;
    (void)size;
    return SID_ERROR_NOSUPPORT;
}

/*----------------------------------------------------------------------------*/

__WEAK SID_STM32_SPEED_OPTIMIZED sid_error_t smtc_modem_hal_store_and_forward_context_store(uint32_t offset, const uint8_t * buffer, const uint32_t size)
{
    /**
     * Override this method in your application if you plan to use Store-and-Forward mechanism provided by LoRa Basics Modem.
     * The parameters for this method are transparently passed from the smtc_modem_hal_context_store() call.
     * This method should return SID_ERROR_NONE on success or an error code otherwise
     */
    (void)offset;
    (void)buffer;
    (void)size;
    return SID_ERROR_NOSUPPORT;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED void smtc_modem_hal_context_store(const modem_context_type_t ctx_type, uint32_t offset, const uint8_t * buffer, const uint32_t size)
{
    sid_error_t err;
    uint16_t kv_sotrage_key;

    do
    {
        /* Select corresponding key for Sidewalk Key-Value storage */
        switch (ctx_type)
        {
            case CONTEXT_MODEM:
                kv_sotrage_key = STORAGE_KV_LBM_CONTEXT_MODEM_KEY;
                err = (0u == offset) ? SID_ERROR_NONE : SID_ERROR_PARAM_OUT_OF_RANGE;
                break;

            case CONTEXT_KEY_MODEM:
                kv_sotrage_key = STORAGE_KV_LBM_CONTEXT_KEY_MODEM_KEY;
                err = (0u == offset) ? SID_ERROR_NONE : SID_ERROR_PARAM_OUT_OF_RANGE;
                break;

            case CONTEXT_LORAWAN_STACK:
                kv_sotrage_key = offset / sizeof(lr1_mac_nvm_context_t);
                if ((kv_sotrage_key >= NUMBER_OF_STACKS) || ((offset % sizeof(lr1_mac_nvm_context_t)) != 0u))
                {
                    err = SID_ERROR_PARAM_OUT_OF_RANGE;
                }
                else
                {
                    kv_sotrage_key += STORAGE_KV_LBM_CONTEXT_LORAWAN_STACK_KEY_BASE;
                    err = SID_ERROR_NONE;
                }
                break;

            case CONTEXT_FUOTA:
                err = smtc_modem_hal_fuota_context_store(offset, buffer, size);
                break;

            case CONTEXT_SECURE_ELEMENT:
                kv_sotrage_key = STORAGE_KV_LBM_CONTEXT_SECURE_ELEMENT_KEY;
                err = (0u == offset) ? SID_ERROR_NONE : SID_ERROR_PARAM_OUT_OF_RANGE;
                break;

            case CONTEXT_STORE_AND_FORWARD:
                err = smtc_modem_hal_store_and_forward_context_store(offset, buffer, size);
                break;

            default:
                SID_PAL_LOG_ERROR(LBM_LOG_SID_PREFIX "cannot store context, unknown context type: 0x%02X", ctx_type);
                err = SID_ERROR_INVALID_ARGS;
                break;
        }

        /* Terminate on error */
        if (err != SID_ERROR_NONE)
        {
            if (SID_ERROR_PARAM_OUT_OF_RANGE == err)
            {
                SID_PAL_LOG_ERROR(LBM_LOG_SID_PREFIX "can't store context 0x%02X. Invalid ofset: 0x%08X", ctx_type, offset);
            }
            break;
        }

        if ((ctx_type != CONTEXT_FUOTA) && (ctx_type != CONTEXT_STORE_AND_FORWARD))
        {
            /* Write to Sidewalk KV storage */
            err = sid_pal_storage_kv_record_set(STORAGE_KV_LBM_CONTEXT_STORAGE_GROUP, kv_sotrage_key, buffer, size);
            if (err != SID_ERROR_NONE)
            {
                SID_PAL_LOG_ERROR(LBM_LOG_SID_PREFIX "unable to store context 0x%02X to KV storage, error code: %d", ctx_type, err);
                break;
            }
        }

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    if (err != SID_ERROR_NONE)
    {
#ifdef LBM_REBOOT_ON_PANIC
        /* Reset the MCU */
        smtc_modem_hal_reset_mcu();
#else
        /* Halt the MCU */
        Error_Handler();
#endif /* LBM_REBOOT_ON_PANIC */
    }
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED void smtc_modem_hal_on_panic(uint8_t * func, uint32_t line, const char * fmt, ...)
{
    char print_buf[LBM_CRASH_LOG_SIZE > SID_PAL_LOG_MSG_LENGTH_MAX ? (LBM_CRASH_LOG_SIZE + 1u) : (SID_PAL_LOG_MSG_LENGTH_MAX + 1u)]; /* Adding 1 additional char space for '\0' */
    uint32_t crash_log_str_len = 0u;

    do
    {
        int print_len;

        /* Store function name and code line */
        print_len = snprintf_like(print_buf, sizeof(print_buf), "%s:%u ", func, line);
        if (print_len < 0)
        {
            /* Buffer is full */
            crash_log_str_len = sizeof(print_buf) - 1u;
            break;
        }
        else
        {
            crash_log_str_len += (uint32_t)print_len;
        }

        /* Append crash log message */
        va_list args;
        va_start(args, fmt);
        print_len = UTIL_ADV_TRACE_VSNPRINTF(&print_buf[crash_log_str_len], (sizeof(print_buf) - crash_log_str_len), fmt, args);
        va_end(args);
        if (print_len < 0)
        {
            /* Buffer is full */
            crash_log_str_len = sizeof(print_buf) - 1u;
            break;
        }
        else
        {
            crash_log_str_len += (uint32_t)print_len;
        }
    } while (0);

    /* Store the crash log so it can be used after the reset (e.g. for sending out the log to the cloud) */
    smtc_modem_hal_crashlog_store((uint8_t *)print_buf, crash_log_str_len);

    /* Provide console output for the crash log */
    SID_PAL_LOG_ERROR(LBM_LOG_SID_PREFIX "\e[1;35m" "Panic: " "\e[0m" "%s", print_buf);

#ifdef LBM_REBOOT_ON_PANIC
    /* Reset the MCU */
    smtc_modem_hal_reset_mcu();
#else
    /* Halt the MCU */
    Error_Handler();
#endif /* LBM_REBOOT_ON_PANIC */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED uint32_t smtc_modem_hal_get_random_nb_in_range(const uint32_t val_1, const uint32_t val_2)
{
    sid_error_t err;
    uint32_t random_val;

    /* Use Sidewalk platform RNG */
    err = sid_pal_crypto_rand((void *)&random_val, sizeof(random_val));
    if (err != SID_ERROR_NONE)
    {
        struct sid_timespec now;

        /* Fall back to a software RNG */
        (void)sid_pal_uptime_now(&now);
        srand(sid_timespec_to_ms(&now));
        random_val = rand();
    }

    if (val_1 <= val_2)
    {
        const uint32_t limit = val_2 - val_1 + 1u;
        random_val = (random_val % limit) + val_1;
    }
    else
    {
        const uint32_t limit = val_1 - val_2 + 1;
        random_val = (random_val % limit) + val_2;
    }
    return random_val;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED void smtc_modem_hal_irq_config_radio_irq(void ( *callback )( void* context ), void * context)
{
    sid_pal_enter_critical_region();
    lbm_hal_radio_irq_cb  = callback;
    lbm_hal_radio_irq_ctx = context;
    sid_pal_exit_critical_region();
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED void smtc_modem_hal_radio_irq(void)
{
    if (lbm_hal_radio_irq_cb != NULL)
    {
        /* Disable modem IRQ line until IRQ is processed. This is required to process the IRQ in a task context, otherwise the code will stuck in IRQ handler */
        smtc_modem_hal_disable_modem_irq();
        lbm_hal_radio_irq_cb(lbm_hal_radio_irq_ctx);
    }
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED void smtc_modem_hal_post_radio_irq_process(void)
{
    smtc_modem_hal_enable_modem_irq();
}

/*----------------------------------------------------------------------------*/

__WEAK SID_STM32_SPEED_OPTIMIZED void smtc_modem_hal_start_radio_tcxo(void)
{
#if defined(SID_RADIO_PLATFORM_LR11XX)
    const halo_drv_semtech_ctx_t * const drv_ctx = lr11xx_get_drv_ctx();
#elif defined(SID_RADIO_PLATFORM_SX126X)
    const halo_drv_semtech_ctx_t * const drv_ctx = sx126x_get_drv_ctx();
#endif /* SID_RADIO_PLATFORM */

    /* Validate inputs */
    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->config != NULL);

    /* Map Sidewalk radio configuration settings for TCXO control to LBM HAL */
#if defined(SID_RADIO_PLATFORM_LR11XX) || defined(SID_RADIO_PLATFORM_SX126X)
#  if defined(SID_RADIO_PLATFORM_LR11XX)
    if (LR11XX_TCXO_CTRL_VDD == drv_ctx->config->tcxo_config.ctrl)
#  elif defined(SID_RADIO_PLATFORM_SX126X)
    if (SX126X_TCXO_CTRL_VDD == drv_ctx->config->tcxo_config.ctrl)
#  endif /* SID_RADIO_PLATFORM */
    {
        // TODO: if your board supplies TCXO via an MCU pin you should power the TCXO up here
    }
    else
#endif /* SID_RADIO_PLATFORM_LR11XX || SID_RADIO_PLATFORM_SX126X */
    {
        /* Either TCXO is not used or it is controlled by the radio directly - no actions required */
    }
}

/*----------------------------------------------------------------------------*/

__WEAK SID_STM32_SPEED_OPTIMIZED void smtc_modem_hal_stop_radio_tcxo(void)
{
#if defined(SID_RADIO_PLATFORM_LR11XX)
    const halo_drv_semtech_ctx_t * const drv_ctx = lr11xx_get_drv_ctx();
#elif defined(SID_RADIO_PLATFORM_SX126X)
    const halo_drv_semtech_ctx_t * const drv_ctx = sx126x_get_drv_ctx();
#endif /* SID_RADIO_PLATFORM */

    /* Validate inputs */
    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->config != NULL);

    /* Map Sidewalk radio configuration settings for TCXO control to LBM HAL */
#if defined(SID_RADIO_PLATFORM_LR11XX) || defined(SID_RADIO_PLATFORM_SX126X)
#  if defined(SID_RADIO_PLATFORM_LR11XX)
    if (LR11XX_TCXO_CTRL_VDD == drv_ctx->config->tcxo_config.ctrl)
#  elif defined(SID_RADIO_PLATFORM_SX126X)
    if (SX126X_TCXO_CTRL_VDD == drv_ctx->config->tcxo_config.ctrl)
#  endif /* SID_RADIO_PLATFORM */
    {
        // TODO: if your board supplies TCXO via an MCU pin you should power the TCXO down here
    }
    else
#endif /* SID_RADIO_PLATFORM_LR11XX || SID_RADIO_PLATFORM_SX126X */
    {
        /* Either TCXO is not used or it is controlled by the radio directly - no actions required */
    }
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED uint32_t smtc_modem_hal_get_radio_tcxo_startup_delay_ms(void)
{
    uint32_t tcxo_delay_ms = 0u;
#if defined(SID_RADIO_PLATFORM_LR11XX)
    const halo_drv_semtech_ctx_t * const drv_ctx = lr11xx_get_drv_ctx();
#elif defined(SID_RADIO_PLATFORM_SX126X)
    const halo_drv_semtech_ctx_t * const drv_ctx = sx126x_get_drv_ctx();
#endif /* SID_RADIO_PLATFORM */

    /* Validate inputs */
    SID_PAL_ASSERT(drv_ctx != NULL);
    SID_PAL_ASSERT(drv_ctx->config != NULL);

    /* Map Sidewalk radio configuration settings for TCXO control to LBM HAL */
#if defined(SID_RADIO_PLATFORM_LR11XX) || defined(SID_RADIO_PLATFORM_SX126X)
    tcxo_delay_ms = (drv_ctx->config->tcxo_config.timeout_us + 500u) / 1000u; /* Convert us to ms with rounding */
#endif /* SID_RADIO_PLATFORM */

    return tcxo_delay_ms;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED void smtc_modem_hal_set_ant_switch(bool is_tx_on)
{
    int32_t radio_err = RADIO_ERROR_GENERIC;

#if defined(SID_RADIO_PLATFORM_LR11XX)
    const lr11xx_subghz_fe_mode_t fe_mode = (false == is_tx_on) ? LR11XX_RADIO_FRONTEND_MODE_RX : LR11XX_RADIO_FRONTEND_MODE_TX;
    radio_err = radio_lr11xx_set_subghz_fem_mode(fe_mode);
#elif defined(SID_RADIO_PLATFORM_SX126X)
    const sx126x_subghz_fe_mode_t fe_mode = (false == is_tx_on) ? SX126X_RADIO_FRONTEND_MODE_RX : SX126X_RADIO_FRONTEND_MODE_TX;
    radio_err = radio_sx126x_set_fem_mode(fe_mode);
#endif /* SID_RADIO_PLATFORM */

    SMTC_MODEM_HAL_PANIC_ON_FAILURE(RADIO_ERROR_NONE == radio_err);
}

/*----------------------------------------------------------------------------*/

__WEAK SID_STM32_SPEED_OPTIMIZED uint8_t smtc_modem_hal_get_battery_level(void)
{
    /**
     * According to LoRaWan 1.0.4 spec:
     * - 0: The end-device is connected to an external power source.
     * - 1..254: Battery level, where 1 is the minimum and 254 is the maximum.
     * - 255: The end-device was not able to measure the battery level.
     */
    return 0u;
}

/*----------------------------------------------------------------------------*/

__WEAK SID_STM32_SPEED_OPTIMIZED int8_t smtc_modem_hal_get_board_delay_ms(void)
{
    uint32_t delay;

    // TODO: Rx/Tx processing delay in LBM mode may significantly differ from Sidewalk mode. Please measure it accurately for your board and adjust the code below accordingly
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
    delay = (sid_pal_radio_get_lora_rx_process_delay() + sid_pal_radio_get_lora_tx_process_delay()) / 2u; /* Compute average of Rx and Tx delays */
#else
    delay = 1000u; /* Use microseconds here to be aligned with sid_pal_radio_get_lora_xx_process_delay() functions */
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 */

    /* Convert us to ms with rounding */
    delay = (delay + 500u) / 1000u;

    return (int8_t)delay;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED void smtc_modem_hal_print_trace(const char * fmt, ...)
{
#if SID_PAL_LOG_ENABLED && (MODEM_HAL_DBG_TRACE == MODEM_HAL_FEATURE_ON)
    static char print_buf[SID_PAL_LOG_MSG_LENGTH_MAX + LBM_LOG_PREFIX_MAX_LEN + 2u]; /* Adding 1 additional char space for '\n' and 1 for '\0' */
    static uint32_t log_str_len = 0u;
    static bool continuation = false;
    int print_len;

    va_list args;
    va_start(args, fmt);

    if (false == continuation)
    {
        print_len = UTIL_ADV_TRACE_VSNPRINTF(print_buf, sizeof(print_buf), fmt, args);
        if (print_len < 0)
        {
            /* Buffer is full */
            log_str_len = sizeof(print_buf) - 1u;
        }
        else if (0u == print_len)
        {
            /* Empty string, do nothing */
            return;
        }
        else
        {
            log_str_len = (uint32_t)print_len;

            if ((log_str_len >= 2u)
             && ('\n' == print_buf[log_str_len - 2u]) /* One symbol before last is LF */
             && (('\r' == print_buf[log_str_len - 1u]) || (' ' == print_buf[log_str_len - 1u]))) /* Last symbol is CR or space (this is a special case since some log strings in LBM terminate string with "\n " sequence) */
            {
                print_buf[log_str_len - 1u] = '\0';
                log_str_len--;
            }

            if ((print_buf[log_str_len - 1u] != '\n') && (print_buf[0] != '\e')) /* Line is not terminated and it did not start with ESC symbol (color code) */
            {
                /* Incomplete string, new parts will follow */
                continuation = true;
            }
        }
    }
    else
    {
        print_len = UTIL_ADV_TRACE_VSNPRINTF(&print_buf[log_str_len], (sizeof(print_buf) - log_str_len), fmt, args);
        if (print_len < 0)
        {
            /* Buffer is full */
            log_str_len = sizeof(print_buf) - 1u;
            continuation = false;
        }
        else
        {
            log_str_len += (uint32_t)print_len;

            /* Check if we assembled a complete string */
            if ('\n' == print_buf[log_str_len - 1u])
            {
                continuation = false;
            }
            else if ((log_str_len >= 2u)
                  && ('\n' == print_buf[log_str_len - 2u]) /* One symbol before last is LF */
                  && (('\r' == print_buf[log_str_len - 1u]) || (' ' == print_buf[log_str_len - 1u]))) /* Last symbol is CR or space (this is a special case since some log strings in LBM terminate string with "\n " sequence) */
            {
                print_buf[log_str_len - 1u] = '\0';
                log_str_len--;
                continuation = false;
            }
        }
    }

    va_end(args);

    /* Print out the log message */
    if ((false == continuation) && (log_str_len > 0u))
    {
        /* Exclude duplicated line feeds or CR-LF sequence */
        if ((log_str_len >= 2u) && (('\n' == print_buf[log_str_len - 2u]) || ('\r' == print_buf[log_str_len - 2u])))
        {
            print_buf[log_str_len - 2u] = '\n';
            print_buf[log_str_len - 1u] = '\0';
            log_str_len--;
        }

        /* Do not print empty lines */
        if ((1u == log_str_len) && ('\n' == print_buf[0]))
        {
            return;
        }

        if ('\n' == print_buf[log_str_len - 1u])
        {
            /* Line feed is added automatically by SID_PAL_LOG, we don't need it */
            print_buf[log_str_len - 1u] = '\0';
        }

        if (strcmp(print_buf, "\e[0m") == 0)
        {
            /*Color reset sequence - print out directly */
            UTIL_ADV_TRACE_FSend("%s", print_buf);
        }
        else
        {
            /* Normal string - print via SID_PAL_LOG mechanism */
            if (strncmp(print_buf, LBM_LOG_TYPE_INFO_PREFIX, (sizeof(LBM_LOG_TYPE_INFO_PREFIX) - 1u)) == 0)
            {
                SID_PAL_LOG_INFO(LBM_LOG_SID_PREFIX "%s", &print_buf[sizeof(LBM_LOG_TYPE_INFO_PREFIX) - 1u]);
            }
            else if (strncmp(print_buf, LBM_LOG_TYPE_WARN_PREFIX, (sizeof(LBM_LOG_TYPE_WARN_PREFIX) - 1u)) == 0)
            {
                SID_PAL_LOG_WARNING(LBM_LOG_SID_PREFIX "%s", &print_buf[sizeof(LBM_LOG_TYPE_WARN_PREFIX) - 1u]);
            }
            else if (strncmp(print_buf, LBM_LOG_TYPE_ERROR_PREFIX, (sizeof(LBM_LOG_TYPE_ERROR_PREFIX) - 1u)) == 0)
            {
                SID_PAL_LOG_ERROR(LBM_LOG_SID_PREFIX "%s", &print_buf[sizeof(LBM_LOG_TYPE_ERROR_PREFIX) - 1u]);
            }
            else
            {
                /* Skip any leading LFs */
                uint32_t print_start_idx = 0u;
                while ('\n' == print_buf[print_start_idx])
                {
                    print_start_idx++;
                }

                /* Replace LFs that follow leading spaces */
                uint32_t lf_search_idx = print_start_idx;
                while (' ' == print_buf[lf_search_idx])
                {
                    /* Skip leading spaces */
                    lf_search_idx++;
                }

                /* If we have not reached end of line replace LFs with spaces */
                while ((lf_search_idx < (log_str_len - 1u)) && ('\n' == print_buf[lf_search_idx]))
                {
                    print_buf[lf_search_idx] = ' ';
                    lf_search_idx++;
                }

                /* Finally print out the formatted line */
                SID_PAL_LOG_DEBUG(LBM_LOG_SID_PREFIX "%s", &print_buf[print_start_idx]);
            }
        }
    }
#else
    (void)fmt;
#endif /* SID_PAL_LOG_ENABLED && MODEM_HAL_DBG_TRACE */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int8_t smtc_modem_hal_get_temperature(void)
{
    int16_t mcu_temperature = sid_pal_temperature_get();
    return (int8_t)mcu_temperature;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED uint16_t smtc_modem_hal_get_voltage_mv( void )
{
    int16_t supply_voltage_value;
    uint32_t lock_acquired;
    ADCCTRL_Cmd_Status_t adctrl_status;

    /* Obtain shared HW hardware resource */
    lock_acquired = adc_hw_obtain_control();

    /* Enter limited critical section : disable all the interrupts with priority higher than RCC one
     * Concerns link layer interrupts (high and SW low) or any other high priority user system interrupt
     */
#ifdef STM32WBA
    UTILS_ENTER_LIMITED_CRITICAL_SECTION(RCC_INTR_PRIO << 4);
#else
#  error "This implementation supports STM32WBA series only. Make sure the above code lines are still valid if you use other MCU family"
#endif /* STM32WBA */

    do
    {
        if (ADCCTRL_HANDLE_NOT_REG == supply_voltage_meas_handle.State)
        {
            adctrl_status = ADCCTRL_RegisterHandle(&supply_voltage_meas_handle);
            if ((adctrl_status != ADCCTRL_OK) && (adctrl_status != ADCCTRL_HANDLE_ALREADY_REGISTERED))
            {
                SID_PAL_LOG_ERROR(LBM_LOG_SID_PREFIX "Failed to initialize ADC for MCU supply voltage measurement. ADCCTRL error %u", (uint32_t)adctrl_status);
                break;
            }
        }

        if (FALSE == lock_acquired)
        {
            /* Failed to get exclusive access to ADC, skip the measurement and return last known value */
            supply_voltage_value = last_measured_supply_voltage;
            break;
        }

        /* Request ADC IP activation */
        adctrl_status = ADCCTRL_RequestIpState(&supply_voltage_meas_handle, ADC_ON);
        if (adctrl_status != ADCCTRL_OK)
        {
            /* Failed to enable ADC */
            SID_PAL_LOG_ERROR(LBM_LOG_SID_PREFIX "Failed to activate ADC for MCU supply voltage measurement. ADCCTRL error %u", (uint32_t)adctrl_status);
            supply_voltage_value = last_measured_supply_voltage;
            break;
        }

        /* Get supply voltage from ADC dedicated channel */
        adctrl_status = ADCCTRL_RequestCoreVoltage(&supply_voltage_meas_handle, (uint16_t*)&supply_voltage_value);
        if (adctrl_status != ADCCTRL_OK)
        {
            /* Failed to measure the supply voltage, use previous value */
            if (adctrl_status != ADCCTRL_BUSY)
            {
                SID_PAL_LOG_ERROR(LBM_LOG_SID_PREFIX "Failed to measure MCU supply voltage. ADCCTRL error %u", (uint32_t)adctrl_status);
            }
            supply_voltage_value = last_measured_supply_voltage;
            /* Don't break here - we still need to deactivate the ADC */
        }
        else
        {
            /* The measurement is good, update last known supply voltage reading */
            last_measured_supply_voltage = supply_voltage_value;
        }

        /* Request ADC IP deactivation */
        adctrl_status = ADCCTRL_RequestIpState(&supply_voltage_meas_handle, ADC_OFF);
        if (adctrl_status != ADCCTRL_OK)
        {
            SID_PAL_LOG_ERROR(LBM_LOG_SID_PREFIX "Failed to deactivate ADC after MCU supply voltage measurement. ADCCTRL error %u", (uint32_t)adctrl_status);
        }
    } while (0);

    /* Exit limited critical section */
    UTILS_EXIT_LIMITED_CRITICAL_SECTION();

    /* Release the shared ADC hardware resource */
    if (lock_acquired != FALSE)
    {
        adc_hw_release_control();
    }

    return (uint16_t)supply_voltage_value;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED void smtc_modem_hal_crashlog_store(const uint8_t * crash_string, uint8_t crash_string_length)
{
    sid_pal_enter_critical_region();
    lbm_crash_log_valid_length = MIN(crash_string_length, sizeof(lbm_crash_log_buf));
    SID_STM32_UTIL_fast_memcpy(lbm_crash_log_buf, crash_string, lbm_crash_log_valid_length);
    lbm_crash_log_validity_marker = LBM_CRASH_LOG_VALID_WATERMARK;
    sid_pal_exit_critical_region();
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED void smtc_modem_hal_crashlog_restore(uint8_t * crash_string, uint8_t * crash_string_length)
{
    if (LBM_CRASH_LOG_VALID_WATERMARK == lbm_crash_log_validity_marker)
    {
        *crash_string_length = ( lbm_crash_log_valid_length > CRASH_LOG_SIZE) ? CRASH_LOG_SIZE : lbm_crash_log_valid_length; /* Should not exceed the crash log size limit in LBM stack */
        SID_STM32_UTIL_fast_memcpy(crash_string, lbm_crash_log_buf, *crash_string_length);
    }
    else
    {
        *crash_string = '\0';
        crash_string_length = 0u;
    }
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED void smtc_modem_hal_crashlog_set_status(bool available)
{
    sid_pal_enter_critical_region();
    if (false == available)
    {
        lbm_crash_log_validity_marker = 0u;
    }
    else
    {
        lbm_crash_log_validity_marker = LBM_CRASH_LOG_VALID_WATERMARK;
    }
    sid_pal_exit_critical_region();
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED bool smtc_modem_hal_crashlog_get_status(void)
{
    bool available;

    sid_pal_enter_critical_region();
    if (LBM_CRASH_LOG_VALID_WATERMARK == lbm_crash_log_validity_marker)
    {
        available = true;
    }
    else
    {
        available = false;
    }
    sid_pal_exit_critical_region();

    return available;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED uint16_t smtc_modem_hal_flash_get_page_size( void )
{
    return FLASH_PAGE_SIZE;
}

/*----------------------------------------------------------------------------*/

__WEAK SID_STM32_SPEED_OPTIMIZED void smtc_modem_hal_user_lbm_irq(void)
{
    /**
     * This method should trigger smtc_modem_engine_run() execution. Depending on your application design, you may or may not need to take specific actions here.
     * Feel free to override this default implementation in your application
     */
    SID_PAL_LOG_WARNING("Default smtc_modem_hal_user_lbm_irq() implementation does not do anything");
}
