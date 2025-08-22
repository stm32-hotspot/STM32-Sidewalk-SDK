/**
  ******************************************************************************
  * @file    reset.c
  * @brief   MCU reset interface for Sidewalk SDK
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

/* Includes ------------------------------------------------------------------*/

#include <sid_hal_reset_ifc.h>
#include <stm32wbaxx_hal.h>

/* Global function definitions -----------------------------------------------*/

sid_error_t sid_hal_reset(sid_hal_reset_type_t type)
{
    sid_error_t err = SID_ERROR_GENERIC;

    switch (type)
    {
        case SID_HAL_RESET_NORMAL:
            HAL_NVIC_SystemReset();
            /* No return from here */
            break;

        case SID_HAL_RESET_DFU:
            err = SID_ERROR_NOSUPPORT;
            break;

        default:
            err = SID_ERROR_INVALID_ARGS;
            break;
    }

    return err;
}

