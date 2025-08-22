/**
  ******************************************************************************
  * @file           : sid_pal_timer_types.h
  * @brief          : Platform-specific definitions for the sid_pal_timer unit
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023-2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#ifndef _SID_PAL_TIMER_TYPES_H__
#define _SID_PAL_TIMER_TYPES_H__

/* Includes ------------------------------------------------------------------*/

#include <sid_error.h>
#include <stm_list.h>
#include <stm32_timer.h>
#include <stm32_timer_us.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Alias for the struct sid_pal_timer_impl_t
 *
 */
typedef struct sid_pal_timer_impl_t sid_pal_timer_t;

/**
 * @brief Timer callback type
 *
 * @note The callback is allowed to execute absolute minimum amount of work and return as soon as possible
 * @note Implementer of the callback should consider the callback is executed from the ISR context
 */
typedef void(*sid_pal_timer_cb_t)(void * arg, sid_pal_timer_t * originator);

/**
 * @brief Timer storage type
 *
 * @note This is the implementor defined storage type for timers.
 */
struct sid_pal_timer_impl_t {
    UTIL_TIMER_Object_t stm32_timer;  /*!< Timer object to be used with ST's UTIL_TIMER utlity */
    tListNode           node;         /*!< Node to maintain the linked list of the timers */
    uint64_t            period_us;    /*!< Timer period in us */
    sid_pal_timer_cb_t  callback;     /*!< User callback to be invoked each time the timer expires */
    void *              callback_arg; /*!< User-defined parameter to be passed to the user callback */
};

/* Exported functions prototypes ---------------------------------------------*/

/**
 * De-init the timer facility
 *
 * This function is typically used to release HW or SW resources needed for the timer.
 * If none are needed by the timer implementation then this function is unnecessary.
 *
 * @retval SID_ERROR_NONE in case of success
 */
sid_error_t sid_pal_timer_facility_deinit(void);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* _SID_PAL_TIMER_TYPES_H__ */
