/**
  ******************************************************************************
  * @file    swi.c
  * @brief   Software Interrupt (SWI) implementation
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

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

/* Sidewalk interfaces */
#include <sid_pal_critical_region_ifc.h>
#include <sid_pal_log_ifc.h>
#include <sid_pal_swi_ifc.h>

/* RTOS interfaces */
#include "stm32_rtos.h"

/* Utils */
#include <sid_stm32_common_utils.h>

/* Private typedef -----------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

#define SID_PAL_SWI_TASK_FLAG_TRIGGER   ((uint32_t)(1u << 0))
#define SID_PAL_SWI_TASK_FLAG_TERMINATE ((uint32_t)(1u << 1))
#define SID_PAL_SWI_TASK_FLAG_ALL       ((SID_PAL_SWI_TASK_FLAG_TRIGGER) | (SID_PAL_SWI_TASK_FLAG_TERMINATE))

//#define SID_PAL_SWI_RUNTIME_DIAGS /* Uncomment this line to enable runtime RAM consumption diagnostics */

#ifdef SID_PAL_SWI_RUNTIME_DIAGS
#  define SWI_RUNTIME_DIAGS_PERIOD_TICKS (10000u) /* Runtime diagnostics period in OS ticks */
#endif

/* Private variables ---------------------------------------------------------*/

static sid_pal_swi_cb_t swi_callback = NULL;
static bool is_initialized = false;

static const osThreadAttr_t SID_SWI_Thread_Attr = {
    .name         = "Sidewalk SWI Task",
    .priority     = osPriorityISR,
    .stack_size   = 1568u, /* Use SID_PAL_SWI_RUNTIME_DIAGS to tweak this */
    .attr_bits    = TASK_DEFAULT_ATTR_BITS,
    .cb_mem       = TASK_DEFAULT_CB_MEM,
    .cb_size      = TASK_DEFAULT_CB_SIZE,
    .stack_mem    = TASK_DEFAULT_STACK_MEM
};
static osThreadId_t swi_task = NULL;

#ifdef SID_PAL_SWI_RUNTIME_DIAGS
static uint32_t last_diag_timestamp = 0u;
#endif

/* Global variables ----------------------------------------------------------*/

#ifdef SID_PAL_SWI_RUNTIME_DIAGS
volatile uint32_t swi_free_stack_min = UINT32_MAX;
#endif

/* Imported function prototypes ----------------------------------------------*/

#if SID_SDK_CONFIG_RF_TIMINGS_MEASUREMENT
extern void sid_pal_radio_rxtx_start_cb(void);
#endif /* SID_SDK_CONFIG_RF_TIMINGS_MEASUREMENT */

/* Private function prototypes -----------------------------------------------*/

static void SID_SWI_Thread_Entry(void *context);

/* Private function definitions ----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static void SID_SWI_Thread_Entry(void *context)
{
    bool terminate = false;
    (void)context;

    SID_PAL_LOG_DEBUG("SWI thread started");

    while (false == terminate)
    {
        const uint32_t events_to_wait = SID_PAL_SWI_TASK_FLAG_ALL;
        const uint32_t active_events = osThreadFlagsWait(events_to_wait, osFlagsWaitAny, osWaitForever);

        /* Process SWI trigger event - this should have the highest priority over any other event flags */
        if ((active_events & SID_PAL_SWI_TASK_FLAG_TRIGGER) != 0u)
        {
            if (swi_callback != NULL)
            {
                swi_callback();
            }
        }

        /* Process deinitialization request */
        if ((active_events & SID_PAL_SWI_TASK_FLAG_TERMINATE) != 0u)
        {
            SID_PAL_LOG_DEBUG("SWI thread requested to terminate");
            terminate = true;
        }

#ifdef SID_PAL_SWI_RUNTIME_DIAGS
        const uint32_t ticks_now = osKernelGetTickCount();
        const uint32_t elapsed = ticks_now - last_diag_timestamp;

        /* Perform diagnostic once in a while, not on each SWI trigger */
        if ((elapsed >= SWI_RUNTIME_DIAGS_PERIOD_TICKS) || (terminate != false))
        {
            /* Update the remaining task stack space */
            uint32_t free_stack = osThreadGetStackSpace(swi_task);
            if (free_stack < swi_free_stack_min)
            {
                swi_free_stack_min = free_stack;
            }

            /* Update the timestamp of the latest run */
            last_diag_timestamp = ticks_now;
        }
#endif
    }

    SID_PAL_LOG_DEBUG("SWI thread terminated");
    osThreadExit();
}

/* Global function definitions -----------------------------------------------*/

sid_error_t sid_pal_swi_init(void)
{
    sid_error_t ret = SID_ERROR_GENERIC;

    do
    {
        /* Skip if initialization was done already */
        if (is_initialized != false)
        {
            ret = SID_ERROR_NONE;
            break;
        }

        /* Checks for systematic SW failures */
        if (SID_SWI_Thread_Attr.priority != osPriorityISR)
        {
            SID_PAL_LOG_ERROR("SWI handler thread should have the highest priority in the system");
            ret = SID_ERROR_INVALID_ARGS;
            break;
        }

        /* Create the task to run SWI callbacks */
        osThreadId_t tmp = osThreadNew(SID_SWI_Thread_Entry, NULL, &SID_SWI_Thread_Attr);
        if (NULL == tmp)
        {
            /* Failed to create task */
            SID_PAL_LOG_ERROR("Can't create Sidewalk SWI thread. Check available RAM space and ensure caller context is not IRQ");
            ret = SID_ERROR_INVALID_STATE;
            break;
        }
        else
        {
            /* Done */
            sid_pal_enter_critical_region();
            swi_task = tmp;
            is_initialized = true;
            sid_pal_exit_critical_region();
            ret = SID_ERROR_NONE;
        }
    } while (0);

    return ret;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_pal_swi_start(sid_pal_swi_cb_t event_callback)
{
    sid_error_t ret = SID_ERROR_GENERIC;

    do
    {
        /* Validate input params */
        if (NULL == event_callback)
        {
            ret = SID_ERROR_NULL_POINTER;
            break;
        }

        /* Ensure SWI is initialized */
        if (false == is_initialized)
        {
            ret = sid_pal_swi_init();
            if (ret != SID_ERROR_NONE)
            {
                break;
            }
        }

        /* Set SWI callback to enable it */
        swi_callback = event_callback;
        ret = SID_ERROR_NONE;
    } while (0);

    return ret;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_pal_swi_stop(void)
{
    /* Clear callback function pointer to disable handling.
     * Even if the SID_PAL_SWI_TASK_FLAG_TRIGGER is set already, it will trigger no action after this line
     */
    swi_callback = NULL;
    return SID_ERROR_NONE;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_pal_swi_trigger(void)
{
    sid_error_t ret = SID_ERROR_GENERIC;

    do
    {
        if ((false == is_initialized) || (NULL == swi_callback))
        {
            ret = SID_ERROR_INVALID_STATE;
            break;
        }

        const uint32_t flags_set = osThreadFlagsSet(swi_task, SID_PAL_SWI_TASK_FLAG_TRIGGER);

        if ((flags_set & osFlagsError) != 0u)
        {
            SID_PAL_LOG_ERROR("Failed to trigger SWI. Error code: %d", (int32_t)flags_set);
            if ((uint32_t)osErrorParameter == flags_set)
            {
                ret = SID_ERROR_INVALID_ARGS;
            }
            else
            {
                ret = SID_ERROR_GENERIC;
            }
            break;
        }

        ret = SID_ERROR_NONE;
    } while (0);

    return ret;
}

/*----------------------------------------------------------------------------*/

sid_error_t sid_pal_swi_deinit(void)
{
    sid_error_t ret = SID_ERROR_GENERIC;

    do
    {
        if (false == is_initialized)
        {
            ret = SID_ERROR_NONE;
            break;
        }

        /* Stop SWI processing first */
        (void)sid_pal_swi_stop();

        /* Request SWI thread to exit */
        const uint32_t flags_set = osThreadFlagsSet(swi_task, SID_PAL_SWI_TASK_FLAG_TERMINATE);
        if ((flags_set & osFlagsError) != 0u)
        {
            /* Not something critical, but ghost task will consume some heap space for its stack */
            SID_PAL_LOG_WARNING("Failed to properly terminate SWI thread. This may result in memory leak.");
        }

        sid_pal_enter_critical_region();
        is_initialized = false;
        swi_task = NULL;
        sid_pal_exit_critical_region();
        ret = SID_ERROR_NONE;
    } while (0);

    return ret;
}

/*----------------------------------------------------------------------------*/

#if SID_SDK_CONFIG_RF_TIMINGS_MEASUREMENT
/**
 * If the sub-GHz radio timings measurement mode is enabled and GCC toolchain is used, the sid_raa_tx_initiate()
 * and sid_raa_rx_initiate() functions should be wrapped by linker using -Wl,--wrap=sid_raa_tx_initiate and
 * -Wl,--wrap=sid_raa_rx_initiate flags.
 */
SID_STM32_FUNC_WRAPPER_DECLARE_WRAPPED_FUNCTION(SID_STM32_SPEED_OPTIMIZED sid_error_t, sid_raa_tx_initiate(void* param1, void* param2, uint8_t param3))
{
    sid_pal_radio_rxtx_start_cb();
    return SID_STM32_FUNC_WRAPPER_CALL_ORIGINAL_FUNCTION(sid_raa_tx_initiate(param1, param2, param3));
}

/*----------------------------------------------------------------------------*/

/**
 * If the sub-GHz radio timings measurement mode is enabled and GCC toolchain is used, the sid_raa_tx_initiate()
 * and sid_raa_rx_initiate() functions should be wrapped by linker using -Wl,--wrap=sid_raa_tx_initiate and
 * -Wl,--wrap=sid_raa_rx_initiate flags.
 */
SID_STM32_FUNC_WRAPPER_DECLARE_WRAPPED_FUNCTION(SID_STM32_SPEED_OPTIMIZED sid_error_t, sid_raa_rx_initiate(void* param1, void* param2))
{
    sid_pal_radio_rxtx_start_cb();
    return SID_STM32_FUNC_WRAPPER_CALL_ORIGINAL_FUNCTION(sid_raa_rx_initiate(param1, param2));
}
#endif /* SID_SDK_CONFIG_RF_TIMINGS_MEASUREMENT */
