/**
  ******************************************************************************
  * @file    uptime.c
  * @brief   Sidewalk sid_pal_uptime implementation for STM32WBA platform
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
#include <sid_pal_uptime_ifc.h>
#include <sid_time_ops.h>

/* Platform interfaces */
#include <sid_stm32wba_sys_timer.h>
#include <sid_stm32_common_utils.h>

/* Global function definitions -----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_pal_uptime_now(struct sid_timespec *result)
{
    return sid_stm32wba_sys_timer_get(result);
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED void sid_pal_uptime_set_xtal_ppm(int16_t ppm)
{
    // FIXME: Temporarily disabled due to unresolved issues with the current compensation algorithm. Refer to Jira Ticket STMC-222 for more details.
    //sid_stm32wba_sys_timer_set_xtal_ppm(ppm);
    (void)ppm;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int16_t sid_pal_uptime_get_xtal_ppm(void)
{
    return sid_stm32wba_sys_timer_get_xtal_ppm();
}
