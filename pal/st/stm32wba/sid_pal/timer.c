/**
  ******************************************************************************
  * @file    timer.c
  * @brief   Sidewalk sid_pal_timer implementation for STM32WBA platform
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

/* Sidewalk interfaces */
#include <sid_pal_assert_ifc.h>
#include <sid_pal_critical_region_ifc.h>
#include <sid_pal_log_ifc.h>
#include <sid_pal_timer_ifc.h>
#include <sid_time_ops.h>

/* Platform interfaces */
#include <sid_stm32wba_sys_timer.h>
#include <sid_stm32_common_utils.h>

/* Private defines -----------------------------------------------------------*/

#define STM_LIST_INITIAL_VALUE(list) {.next = &(list), .prev = &(list)}

/* Private macro -------------------------------------------------------------*/

#ifndef containerof
#define containerof(ptr, type, member) \
    ((type *)((uintptr_t)(ptr) - offsetof(type, member)))
#endif

#ifndef SID_PAL_TIMER_DEBUG
#  define SID_PAL_TIMER_DEBUG       0 /* Set to non-zero value to enable debug messages */
#endif /* SID_PAL_TIMER_DEBUG */

#if SID_PAL_TIMER_DEBUG
#  define SID_PAL_TIMER_LOG_ERROR(...)   SID_PAL_LOG_ERROR(__VA_ARGS__)
#  define SID_PAL_TIMER_LOG_WARNING(...) SID_PAL_LOG_WARNING(__VA_ARGS__)
#  define SID_PAL_TIMER_LOG_INFO(...)    SID_PAL_LOG_INFO(__VA_ARGS__)
#  define SID_PAL_TIMER_LOG_DEBUG(...)   SID_PAL_LOG_DEBUG(__VA_ARGS__)
#  define SID_PAL_TIMER_LOG_TRACE(...)   SID_PAL_LOG_TRACE(__VA_ARGS__)
#else
#  define SID_PAL_TIMER_LOG_ERROR(...)   ((void)0u)
#  define SID_PAL_TIMER_LOG_WARNING(...) ((void)0u)
#  define SID_PAL_TIMER_LOG_INFO(...)    ((void)0u)
#  define SID_PAL_TIMER_LOG_DEBUG(...)   ((void)0u)
#  define SID_PAL_TIMER_LOG_TRACE(...)   ((void)0u)
#endif /* SID_PAL_TIMER_DEBUG */

/* Private typedef -----------------------------------------------------------*/

typedef struct {
    uint32_t  initialized; /*!< Indicates if the driver was initialized */
    tListNode timer_list;  /*!< List of timers that are managed by this driver */
} sid_pal_timer_ctx_t;

/* Private variables ---------------------------------------------------------*/

static sid_pal_timer_ctx_t sid_pal_timer_ctx = { 
    .initialized = FALSE,
    .timer_list  = STM_LIST_INITIAL_VALUE(sid_pal_timer_ctx.timer_list),
};

/* Private function prototypes -----------------------------------------------*/

static        void     _first_fire_callback(void *arg);
static        void     _generic_fire_callback(void *arg);
static inline void     _timer_list_insert(sid_pal_timer_t * const timer);
static inline uint32_t _timer_in_list(const sid_pal_timer_t * const timer);
static inline void     _timer_list_delete(sid_pal_timer_t * const timer);

/* Private function definitions ----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static void _first_fire_callback(void *arg)
{
    UTIL_TIMER_Status_t     timer_err;
    sid_pal_timer_cb_t      user_cb;
    void *                  user_arg;
    sid_pal_timer_t * const timer = (sid_pal_timer_t *)arg;

    if (NULL == timer)
    {
        /* Arguments are invalid, leave */
        return;
    }

    /* Run in the critical section to avoid race conditions */
    sid_pal_enter_critical_region();

    /* Store local copies of the data to avoid race conditions upon invocation */
    user_cb  = timer->callback;
    user_arg = timer->callback_arg;

    /* Reconfigure the timer to run with the specified period */
    timer->stm32_timer.Callback = _generic_fire_callback;
    timer_err = UTIL_TIMER_SetReloadMode(&timer->stm32_timer, UTIL_TIMER_PERIODIC);
    SID_PAL_ASSERT(UTIL_TIMER_OK == timer_err);
    __COMPILER_BARRIER();

    /* Restart the timer now to minimize time drift */
    timer_err = UTIL_TIMER_StartWithPeriodUs(&timer->stm32_timer, timer->period_us);
    SID_PAL_ASSERT(UTIL_TIMER_OK == timer_err);
    __COMPILER_BARRIER();

    /* Done with the critical stuff */
    sid_pal_exit_critical_region();

    /* Invoke generic timer fire callback */
    if (user_cb != NULL)
    {
        user_cb(user_arg, timer);
    }
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static void _generic_fire_callback(void *arg)
{
    sid_pal_timer_cb_t      user_cb;
    void *                  user_arg;
    sid_pal_timer_t * const timer = (sid_pal_timer_t *)arg;

    if (NULL == timer)
    {
        /* Arguments are invalid, leave */
        return;
    }

    /* Run in the critical section to avoid race conditions */
    sid_pal_enter_critical_region();

    /* Store local copies of the data to avoid race conditions */
    user_cb  = timer->callback;
    user_arg = timer->callback_arg;

    /* Done with the critical stuff */
    sid_pal_exit_critical_region();

    /* Now it is safe to invoke user callback without residing in a critical section */
    if (user_cb != NULL)
    {
        user_cb(user_arg, timer);
    }
}

/*----------------------------------------------------------------------------*/

/**
 * @brief Adds the timer into the list of timers for this driver
 *
 * @param [in] timer Timer node to be added to the list
 */
SID_STM32_SPEED_OPTIMIZED static inline void _timer_list_insert(sid_pal_timer_t * const timer)
{
    SID_PAL_ASSERT(timer != NULL);

    if (_timer_in_list(timer) == FALSE)
    {
        LST_insert_tail(&sid_pal_timer_ctx.timer_list, &timer->node);
    }
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline uint32_t _timer_in_list(const sid_pal_timer_t * const timer)
{
    uint32_t in_list;

    SID_PAL_ASSERT(timer != NULL);

    do
    {
        /* Check if the timer node is in any list at all */
        if ((NULL == timer->node.prev) || (NULL == timer->node.next))
        {
            /* Not in a list at all or the node links are invalid */
            in_list = FALSE;
            break;
        }

        /* Check if this timer node belongs to the list of the timers maintained by this driver */
        tListNode * current_node;

        in_list = FALSE; /* Assume the node won't be found */
        LST_get_next_node(&sid_pal_timer_ctx.timer_list, &current_node); /* If the list is empty this will set current_node back to the sid_pal_timer_ctx.timer_list */
        while ((current_node != &sid_pal_timer_ctx.timer_list) && (current_node != NULL))
        {
            if (&timer->node == current_node)
            {
                /* Found current timer in the list */
                in_list = TRUE;
                break;
            }

            /* Move to the next list item */
            LST_get_next_node(current_node, &current_node);
        }
    } while (0);

    return in_list;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline void _timer_list_delete(sid_pal_timer_t * const timer)
{
    SID_PAL_ASSERT(timer != NULL);

    if (_timer_in_list(timer) != FALSE)
    {
        LST_remove_node(&timer->node);
        timer->node.prev = NULL;
        timer->node.next = NULL;
    }
}

/* Global function definitions -----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_pal_timer_init(sid_pal_timer_t * timer, sid_pal_timer_cb_t event_callback, void * event_callback_arg)
{
    sid_error_t         err = SID_ERROR_GENERIC;
    UTIL_TIMER_Status_t timer_err;

    /* Run in a critical section to ensure atomicity of the timer configuration */
    sid_pal_enter_critical_region();

    do
    {
        /* Validate inputs */
        if ((NULL == timer) || (NULL == event_callback))
        {
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        /* Ensure the timer is not initialized already */
        if (_timer_in_list(timer) != FALSE)
        {
            SID_PAL_LOG_WARNING("Initializing timer that is initialized already. Timer 0x%08X, cb 0x%08X", (uint32_t)(void *)timer, (uint32_t)(void *)event_callback);
            err = sid_pal_timer_deinit(timer);
            if (err != SID_ERROR_NONE)
            {
                /* Logs are provided by sid_pal_timer_deinit() */
                break;
            }
        }

        /* Store user callback and argument */
        timer->callback     = event_callback;
        timer->callback_arg = event_callback_arg;

        /* Create UTIL_TIMER node */
        timer_err = UTIL_TIMER_CreateUs(&timer->stm32_timer, UTIL_TIMER_US_INFINITY, UTIL_TIMER_ONESHOT, _generic_fire_callback, timer);
        if (timer_err != UTIL_TIMER_OK)
        {
            SID_PAL_LOG_ERROR("Unable to initialize SID timer instance. Error code: %d", timer_err);
            err = SID_ERROR_UNINITIALIZED;
            break;
        }

        /* Add this timer to the list of the managed timers. This is required to properly deallocate RAM when this driver deinitialized or reinitialized */
        _timer_list_insert(timer);

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    /* Done with the critical stuff */
    sid_pal_exit_critical_region();

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_pal_timer_deinit(sid_pal_timer_t * timer)
{
    sid_error_t err = SID_ERROR_GENERIC;

    /* Run in a critical section to ensure atomicity of the timer configuration */
    sid_pal_enter_critical_region();

    do
    {
        /* Validate inputs */
        if (NULL == timer)
        {
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        if (_timer_in_list(timer) == FALSE)
        {
            /* Check if the timer is in any list at all */
            if ((timer->node.prev != NULL) && (timer->node.next != NULL) && (timer->node.prev != timer->node.next))
            {
                /* Timer is linked to some other list */
                SID_PAL_LOG_WARNING("Timer was not created by Sidewalk stack. Timer 0x%08X, cb 0x%08X", (uint32_t)(void *)timer, (uint32_t)(void *)timer->callback);
                err = SID_ERROR_INVALID_ARGS;
                break;
            }
        }

        /* Ensure the timer is stopped */
        err = sid_pal_timer_cancel(timer);
        if (err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to stop timer 0x%08X", (uint32_t)(void *)timer);
            break;
        }

        /* Invalidate timer callback so the timer can no longer be used */
        timer->stm32_timer.Callback = NULL;
        timer->stm32_timer.argument = NULL;

        /* Drop the timer from the driver's registry */
        _timer_list_delete(timer);

        err = SID_ERROR_NONE;
    } while (0);

    /* Done with the critical stuff */
    sid_pal_exit_critical_region();

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_pal_timer_arm(sid_pal_timer_t * timer, sid_pal_timer_prio_class_t type,
                                                        const struct sid_timespec * when, const struct sid_timespec * period)
{
    sid_error_t           err = SID_ERROR_GENERIC;
    UTIL_TIMER_Status_t   timer_err;

    /* Current implementation uses the same RTC for all power modes */
    (void)type;

    /* Run in a critical section to ensure atomicity of the timer configuration and keep configuration time as low as possible */
    sid_pal_enter_critical_region();

    do
    {
        uint64_t            first_fire_delay_us;
        struct sid_timespec time_now;

        /* Validate inputs */
        if ((NULL == timer) || (NULL == when))
        {
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        /* Ensure the timer is not running already */
        if (UTIL_TIMER_IsRunning(&timer->stm32_timer) != FALSE)
        {
            err = SID_ERROR_INVALID_STATE;
            break;
        }

        /* Get the current time */
        if( sid_stm32wba_sys_timer_get(&time_now) != SID_ERROR_NONE)
        {
            err = SID_ERROR_IO_ERROR;
            break;
        }

        if (sid_time_lt(when, &time_now) != false)
        {
            /* It may happen that there was a delay between calling this function and actually getting to this point (e.g. due to an IRQ, NMI, etc.).
             * To cover this we need to schedule the timer to fire immediately. It's not ok to call timer callback from here directly because the
             * callback shall be executed from the context of the timer IRQ.
             */
            SID_PAL_TIMER_LOG_WARNING("Software timer was requested to be triggered in the past. Arming to now");
            first_fire_delay_us = 0u;
        }
        else
        {
            /* Compute the delay for the initial timer callback firing */
            struct sid_timespec first_fire_time_delta;
            sid_time_delta(&first_fire_time_delta, when, &time_now);
            first_fire_delay_us = sid_timespec_to_us_64(&first_fire_time_delta);
        }

        if (period != NULL)
        {
            /* Periodic mode */
            timer->period_us = sid_timespec_to_us_64(period);

            if (timer->period_us == first_fire_delay_us)
            {
                /* First start delay and period are equal, start directly with the period */
                timer_err = UTIL_TIMER_SetReloadMode(&timer->stm32_timer, UTIL_TIMER_PERIODIC);
                if (timer_err != UTIL_TIMER_OK)
                {
                    err = SID_ERROR_IO_ERROR;
                    break;
                }

                /* Set the callback and its argument */
                timer->stm32_timer.Callback = _generic_fire_callback;
                timer->stm32_timer.argument = timer;

                /* Start the timer */ 
                timer_err = UTIL_TIMER_StartWithPeriodUs(&timer->stm32_timer, timer->period_us);
                if (timer_err != UTIL_TIMER_OK)
                {
                    err = SID_ERROR_IO_ERROR;
                    break;
                }
            }
            else
            {
                /* Initially configure the timer for a single shot to avoid race conditions when first_fire_delay_us is very short */
                timer_err = UTIL_TIMER_SetReloadMode(&timer->stm32_timer, UTIL_TIMER_ONESHOT);
                if (timer_err != UTIL_TIMER_OK)
                {
                    err = SID_ERROR_IO_ERROR;
                    break;
                }

                /* Set the callback and its argument */
                timer->stm32_timer.Callback = _first_fire_callback;
                timer->stm32_timer.argument = timer;

                /* Start the timer */
                timer_err = UTIL_TIMER_StartWithPeriodUs(&timer->stm32_timer, first_fire_delay_us);
                if (timer_err != UTIL_TIMER_OK)
                {
                    err = SID_ERROR_IO_ERROR;
                    break;
                }
            }
        }
        else
        {
            /* One shot mode */
            timer->period_us = 0u;

            timer_err = UTIL_TIMER_SetReloadMode(&timer->stm32_timer, UTIL_TIMER_ONESHOT);
            if (timer_err != UTIL_TIMER_OK)
            {
                err = SID_ERROR_IO_ERROR;
                break;
            }

            /* Set the callback and its argument */
            timer->stm32_timer.Callback = _generic_fire_callback;
            timer->stm32_timer.argument = timer;

            /* Start the timer */
            timer_err = UTIL_TIMER_StartWithPeriodUs(&timer->stm32_timer, first_fire_delay_us);
            if (timer_err != UTIL_TIMER_OK)
            {
                err = SID_ERROR_IO_ERROR;
                break;
            }
        }

        /* Timer was started successfully */
        err = SID_ERROR_NONE;
    } while (0);

    /* Done with the critical stuff */
    sid_pal_exit_critical_region();

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_pal_timer_cancel(sid_pal_timer_t * timer)
{
    sid_error_t err = SID_ERROR_GENERIC;

    /* Run in a critical section to ensure atomicity of the timer configuration */
    sid_pal_enter_critical_region();

    do
    {
        /* Validate inputs */
        if (NULL == timer)
        {
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        /* Ensure the timer is stopped */
        if (UTIL_TIMER_IsRunning(&timer->stm32_timer) != FALSE)
        {
            UTIL_TIMER_Status_t ret = UTIL_TIMER_Stop(&timer->stm32_timer);
            if (ret != UTIL_TIMER_OK)
            {
                SID_PAL_LOG_ERROR("Failed to cancel software timer. Reason: %d", ret);
                err = SID_ERROR_IO_ERROR;
                break;
            }
        }

        /* Everything is ok */
        err = SID_ERROR_NONE;
    } while (0);

    /* Done with the critical stuff */
    sid_pal_exit_critical_region();

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED bool sid_pal_timer_is_armed(const sid_pal_timer_t * timer)
{
    bool is_armed;

    /* Run in a critical section to ensure atomicity of the timer state check */
    sid_pal_enter_critical_region();

    do
    {
        /* Validate inputs */
        if (NULL == timer)
        {
            is_armed = false;
            break;
        }

        /* Evaluate the state of the timer */
        is_armed = UTIL_TIMER_IsRunning(&timer->stm32_timer) != FALSE;
    } while (0);

    /* Done with the critical stuff */
    sid_pal_exit_critical_region();

    return is_armed;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_pal_timer_facility_init(void * arg)
{
    sid_error_t err = SID_ERROR_GENERIC;
    (void)arg;

    /* Run in a critical section to ensure atomicity of the timer configuration */
    sid_pal_enter_critical_region();

    do
    {
        /* De-initialize anything active */
        if (sid_pal_timer_ctx.initialized != FALSE)
        {
            err = sid_pal_timer_facility_deinit();
            if (err != SID_ERROR_NONE)
            {
                /* Logs are provided by sid_pal_timer_facility_deinit() */
                break;
            }
        }

        /* Initialize timers list */
        LST_init_head(&sid_pal_timer_ctx.timer_list);

        /* Initialize the time server utility */
        UTIL_TIMER_Status_t timer_err = UTIL_TIMER_Init();
        if (timer_err != UTIL_TIMER_OK)
        {
            SID_PAL_LOG_ERROR("Failed to initialize UTIL_TIMER. Error %u", (uint32_t)timer_err);
            err = SID_ERROR_IO_ERROR;
            break;
        }
        
        /* Done */
        sid_pal_timer_ctx.initialized = TRUE;
        err = SID_ERROR_NONE;
    } while (0);

    /* Done with the critical stuff */
    sid_pal_exit_critical_region();

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_pal_timer_facility_deinit(void)
{
    sid_error_t         err = SID_ERROR_GENERIC;
    UTIL_TIMER_Status_t timer_err;

    /* Run in a critical section to ensure atomicity of the timer configuration */
    sid_pal_enter_critical_region();

    do
    {
        if (FALSE == sid_pal_timer_ctx.initialized)
        {
            err = SID_ERROR_UNINITIALIZED;
            break;
        }

        /* De-initialize anything active */
        tListNode * current_node;
        LST_get_next_node(&sid_pal_timer_ctx.timer_list, &current_node); /* If the list is empty this will set current_node back to the sid_pal_timer_ctx.timer_list */
        while (current_node != &sid_pal_timer_ctx.timer_list)
        {
            /* Get tiemr object by node */
            sid_pal_timer_t * const timer = containerof(current_node, sid_pal_timer_t, node);

            /* Advance to the next node before the current timer is destroyed */
            LST_get_next_node(current_node, &current_node);

            /* Delete the timer and release all associated resources */
            err = sid_pal_timer_deinit(timer);
            if (err != SID_ERROR_NONE)
            {
                SID_PAL_LOG_WARNING("Failed to deinit timer 0x%08X. Memory leak is possible", (uint32_t)(void *)timer);
            }
        }

        /* Deinitialize timer facility */
        timer_err = UTIL_TIMER_DeInit();
        if (timer_err != UTIL_TIMER_OK)
        {
            SID_PAL_LOG_ERROR("Failed to deinitialize UTIL_TIMER. Error %u", (uint32_t)timer_err);
            err = SID_ERROR_IO_ERROR;
            break;
        }

        /* Done */
        sid_pal_timer_ctx.initialized = FALSE;
        err = SID_ERROR_NONE;
    } while (0);

    /* Done with the critical stuff */
    sid_pal_exit_critical_region();

    return err;
}
