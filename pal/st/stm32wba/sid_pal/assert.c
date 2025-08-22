/**
  ******************************************************************************
  * @file    assert.c
  * @brief   sid_pal_assert module implementation for STM32WBAxx MCUs
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

#include <sid_pal_assert_ifc.h>
#include <sid_pal_critical_region_ifc.h>
#include <sid_pal_log_ifc.h>

#include "stm32wbaxx.h"
#include "stm32wbaxx_hal_conf.h"

#include <stdint.h>

/* Global function definitions -----------------------------------------------*/

void sid_pal_assert(int line, const char * file)
{

#ifdef USE_FULL_ASSERT

    assert_failed((uint8_t *)file, (uint32_t)line);
    sid_pal_log_flush();

#endif /* USE_FULL_ASSERT */

    __disable_irq();
    while (1);
}
