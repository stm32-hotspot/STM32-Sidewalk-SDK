/**
  ******************************************************************************
  * @file           : led_indication.h
  * @brief          : STM32WBA5x-specific Sidewalk states indication using LEDs
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#ifndef __LED_INDICATION_H_
#define __LED_INDICATION_H_

/* Includes ------------------------------------------------------------------*/

#include <sid_error.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Exported types ------------------------------------------------------------*/

/**
 * @brief LED indication states
 */
typedef enum
{
    LED_INDICATE_IDLE          = 0,
    LED_INDICATE_BONDING       = 1,
    LED_INDICATE_CONNECTED     = 2,
    LED_INDICATE_SEND_ENQUEUED = 3,
    LED_INDICATE_SENT_OK       = 4,
    LED_INDICATE_SEND_ERROR    = 5,
    LED_INDICATE_RCV_OK        = 6,
    LED_INDICATE_ERROR         = 7,
    LED_INDICATE_OFF           = 8,
} led_indication_t;

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief  Performs intialization of the LED status indication module
 * @return SID_ERROR_NONE on success
 */
sid_error_t led_indication_init(void);

/**
 * @brief     Activates the specified LED indication option
 * @param[in] indicate Sidewalk status to indicate with the LEDs
 * @return    SID_ERROR_NONE on success
 */
sid_error_t led_indication_set(led_indication_t indicate);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* __LED_INDICATION_H_ */
