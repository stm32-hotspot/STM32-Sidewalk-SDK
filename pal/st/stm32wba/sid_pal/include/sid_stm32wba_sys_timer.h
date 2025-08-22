/**
  ******************************************************************************
  * @file    sid_stm32wba_sys_timer.h
  * @brief   System timer HAL for Sidewalk on STM32WBAxx MCUs
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

#ifndef SID_STM32WBA_SYS_TIMER_H_
#define SID_STM32WBA_SYS_TIMER_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include <sid_error.h>
#include <sid_time_ops.h>

#include <stm32_systime.h>

#include <stdint.h>

/* Exported constants --------------------------------------------------------*/

#define SID_STM32WBA_SYS_TIMER_RTC_MAX_PPM ( (int16_t)150 )
#define TIMER_RTC_MAX_PPM_TO_COMPENSATE      ( SID_STM32WBA_SYS_TIMER_RTC_MAX_PPM )
#define TIMER_RTC_PPM_DENOMINATOR            ( 1000000u )

/* Exported functions --------------------------------------------------------*/

sid_error_t sid_stm32wba_sys_timer_init(void);
sid_error_t sid_stm32wba_sys_timer_deinit(void);
sid_error_t sid_stm32wba_sys_timer_get(struct sid_timespec * const result);
void sid_stm32wba_sys_timer_set_xtal_ppm(int16_t ppm);
int16_t sid_stm32wba_sys_timer_get_xtal_ppm(void);
const struct sid_timespec sid_stm32wba_sys_timer_systime_to_timespec(const SysTime_t * const sys_time);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* SID_STM32WBA_SYS_TIMER_H_ */
