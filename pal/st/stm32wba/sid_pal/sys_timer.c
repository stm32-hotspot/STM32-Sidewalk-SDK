/**
  ******************************************************************************
  * @file    sys_timer.c
  * @brief   STM32WBA platform routines to support sid_pal_timer and
  *          sid_pal_uptime Sidewalk modules
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

/* Sidewalk interfaces */
#include <sid_pal_assert_ifc.h>
#include <sid_pal_critical_region_ifc.h>
#include <sid_pal_log_ifc.h>
#include <sid_pal_timer_ifc.h>

/* Platform interfaces */
#include <sid_stm32_common_utils.h>
#include <sid_stm32wba_sys_timer.h>
#include <stm32_timer.h>
#include <timer_if.h>

#include <stddef.h>
#include <stdlib.h>

/*----------------------------------------------------------------------------*/

struct sid_stm32wba_sys_timer_drift {
    struct sid_timespec pos;     /*!< Positive drift since board start */
    struct sid_timespec neg;     /*!< Negative drift since board start */
    struct sid_timespec lst_cal; /*!< Last time that drift was calculated */
};

/*----------------------------------------------------------------------------*/

static struct sid_stm32wba_sys_timer_drift uptime_drift = {
        .pos = SID_TIME_ZERO,
        .neg = SID_TIME_ZERO,
        .lst_cal = SID_TIME_ZERO
};
static int16_t sid_stm32wba_sys_timer_xtal_ppm = 0;
static uint32_t is_initialized = FALSE;

/*----------------------------------------------------------------------------*/

/**
 * @attention For code optimization only, do not use without normalized input
 *            otherwise ouput normalization is required
 */
SID_STM32_SPEED_OPTIMIZED static inline void time_add(struct sid_timespec * const tm1, const struct sid_timespec * const tm2)
{
    tm1->tv_sec  += tm2->tv_sec;
    tm1->tv_nsec += tm2->tv_nsec;
}

/*----------------------------------------------------------------------------*/

/**
 * @attention For code optimization only, do not use without normalized input
 *            otherwise ouput normalization is required
 */
SID_STM32_SPEED_OPTIMIZED static inline void time_sub(struct sid_timespec * const tm1, const struct sid_timespec * const tm2)
{
    if ((tm1->tv_sec != 0u) && (tm1->tv_nsec < tm2->tv_nsec))
    {
        tm1->tv_sec  = tm1->tv_sec - tm2->tv_sec - 1u;
        tm1->tv_nsec = tm1->tv_nsec - tm2->tv_nsec + SID_TIME_NSEC_PER_SEC;
    }
    else
    {
        tm1->tv_sec  -= tm2->tv_sec;
        tm1->tv_nsec -= tm2->tv_nsec;
    }
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t sid_stm32wba_sys_timer_rtc_get(struct sid_timespec * const result, struct sid_stm32wba_sys_timer_drift * const drift_info)
{
    sid_error_t err = SID_ERROR_GENERIC;

    do
    {
        /* validate inputs */
        if (NULL == result)
        {
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        /* Ensure this module is initialized */
        if (FALSE == is_initialized)
        {
            err = SID_ERROR_UNINITIALIZED;
            break;
        }

        /* Get RTC time */
        result->tv_sec = TIMER_IF_GetTimeUs(&result->tv_nsec);

        /* Condition the obtained values - convert microseconds to nanoseconds */
        result->tv_nsec *= SID_TIME_NSEC_PER_USEC;

        /* Populate time drift info if necessary */
        if (drift_info != NULL)
        {
            /* Use a critical section here to ensure uptime_drift is not modified while copying */
            sid_pal_enter_critical_region();
            SID_STM32_UTIL_fast_memcpy(drift_info, &uptime_drift, sizeof(uptime_drift));
            sid_pal_exit_critical_region();
        }

        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static int16_t sid_stm32wba_sys_timer_get_time_ppm(void)
{
    return -sid_stm32wba_sys_timer_get_xtal_ppm();
}

/* Global function definitions -----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_stm32wba_sys_timer_init(void)
{
    sid_error_t err = SID_ERROR_GENERIC;

    do
    {
        /* Ensure this part is not initialized already */
        if (is_initialized != FALSE)
        {
            err = SID_ERROR_ALREADY_INITIALIZED;
            break;
        }

        /* Init the platform time server */
        err = sid_pal_timer_facility_init(NULL);
        if (err != SID_ERROR_NONE)
        {
            break;
        }

        /* Done */
        is_initialized = TRUE;
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_stm32wba_sys_timer_deinit(void)
{
    sid_error_t err = SID_ERROR_GENERIC;

    do
    {
        /* Check if this part is initialized */
        if (FALSE == is_initialized)
        {
            /* Not initialized, no actions required */
            err = SID_ERROR_NONE;
            break;
        }

        /* Bring down underlying layers */
        err = sid_pal_timer_facility_deinit();
        if (err != SID_ERROR_NONE)
        {
            break;
        }

        /* Done */
        is_initialized = FALSE;
        err = SID_ERROR_NONE;
    } while (0);

    return  err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline int8_t sid_stm32wba_sys_timer_calculate_drift(struct sid_timespec * const uptime, const struct sid_timespec * const old_uptime, const int16_t ppm, struct sid_timespec * const drift)
{
    struct sid_timespec duration = *uptime;
    int32_t sec_drift  = 0;
    int32_t usec_drift = 0;
    int32_t nsec_drift = 0;
    int32_t abs_ppm = abs((int32_t) ppm);

    SID_PAL_ASSERT(uptime);
    SID_PAL_ASSERT(old_uptime);
    SID_PAL_ASSERT(drift);
    SID_PAL_ASSERT(!sid_time_gt(old_uptime, uptime));

    /* Calculate the duration */
    time_sub(&duration, old_uptime);

    /* Non-overflow calculation without 64-bit operations */
    /* First, calculate the drift in step of sec */
    sec_drift = (duration.tv_sec / TIMER_RTC_PPM_DENOMINATOR) * abs_ppm;

    /* Second, the remainder duration in sec */
    usec_drift = (duration.tv_sec % SID_TIME_USEC_PER_SEC) * abs_ppm;

    /* Third, we calculate the drift of duration in nanosec in step of nanosec */
    nsec_drift = ((duration.tv_nsec / 1000) * abs_ppm) / 1000;

    /* Last, normalize usec_drift into sec_drift and nsec_drift */
    drift->tv_sec  = sec_drift + (usec_drift / SID_TIME_USEC_PER_SEC);
    drift->tv_nsec = nsec_drift + ((usec_drift % SID_TIME_USEC_PER_SEC) * 1000);

    /* Normalize the final drift before return */
    sid_time_normalize(drift);

    /* Negative drift return -1, else return 0 */
    return (ppm < 0 ? -1 : 0);
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_stm32wba_sys_timer_get(struct sid_timespec * const result)
{
    struct sid_stm32wba_sys_timer_drift drift_info;
    struct sid_timespec                   drift = SID_TIME_ZERO;
    struct sid_timespec                   delta = SID_TIME_ZERO;

    if (NULL == result)
    {
        return SID_ERROR_INVALID_ARGS;
    }

    /* Get the actual RTC time and drift_info when we read the time */
    sid_stm32wba_sys_timer_rtc_get(result, &drift_info);

    /* Calculate addition drift and compensate additional drift */
    if (sid_stm32wba_sys_timer_calculate_drift(result, &drift_info.lst_cal, sid_stm32wba_sys_timer_get_time_ppm(), &drift) < 0)
    {
        time_sub(result, &drift);
    }
    else
    {
        time_add(result, &drift);
    }

    /* Normalize the result after compensation */
    sid_time_normalize(result);

    /* Compensate global drift */
    if (sid_time_gt(&drift_info.neg, &drift_info.pos))
    {
        delta = drift_info.neg;
        time_sub(&delta, &drift_info.pos);
        time_sub(result, &delta);
    }
    else
    {
        delta = drift_info.pos;
        time_sub(&delta, &drift_info.neg);
        time_add(result, &delta);
    }

    /* Normalize the result after compensation */
    sid_time_normalize(result);

    return SID_ERROR_NONE;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static void sid_stm32wba_sys_timer_update_global_drift(const int16_t ppm)
{
    sid_error_t           err;
    struct sid_timespec   now          = SID_TIME_ZERO;
    struct sid_timespec   drift        = SID_TIME_ZERO;
    struct sid_timespec * global_drift = NULL;

    /* use a critical section here to ensure the update completes ASAP with no interrupts and delays, and to ensure uptime_drift is accessed atomically */
    sid_pal_enter_critical_region();

    do
    {
        /* Get the actual RTC time for drift calculation */
        err = sid_stm32wba_sys_timer_rtc_get(&now, NULL);
        if (err != SID_ERROR_NONE)
        {
            break;
        }

        /* Calculate and update global drift */
        if (sid_stm32wba_sys_timer_calculate_drift(&now, &uptime_drift.lst_cal, ppm, &drift) < 0)
        {
            global_drift = &uptime_drift.neg;
        }
        else
        {
            global_drift = &uptime_drift.pos;
        }

        if (global_drift != NULL)
        {
            sid_time_add(global_drift, &drift);
        }
    
        /* Update last_drift_calc */
        uptime_drift.lst_cal = now;
    } while (0);

    sid_pal_exit_critical_region();
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED void sid_stm32wba_sys_timer_set_xtal_ppm(int16_t ppm)
{
    sid_stm32wba_sys_timer_update_global_drift(sid_stm32wba_sys_timer_get_time_ppm());

    /* Boundary checks */
    if (ppm > TIMER_RTC_MAX_PPM_TO_COMPENSATE)
    {
        /* Trim to upper boundary */
        ppm = TIMER_RTC_MAX_PPM_TO_COMPENSATE;
    }
    else if (ppm < (-TIMER_RTC_MAX_PPM_TO_COMPENSATE))
    {
        /* Trim to lower boundary */
        ppm = (-TIMER_RTC_MAX_PPM_TO_COMPENSATE);
    }
    else
    {
        /* Keep ppm value as is */
    }

    sid_stm32wba_sys_timer_xtal_ppm = ppm;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int16_t sid_stm32wba_sys_timer_get_xtal_ppm(void)
{
    return sid_stm32wba_sys_timer_xtal_ppm;
}
