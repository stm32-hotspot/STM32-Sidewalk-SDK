/**
  ******************************************************************************
  * @file    malloc_wrappers.c
  * @brief   libc memory management functions wrappers
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

#include <assert.h>
#include <sid_stm32_common_utils.h>

#if defined (FREERTOS)
#  include <FreeRTOS.h>
#else
#  error "This implementation supports FreeRTOS only"
#endif

/* Global function definitions -----------------------------------------------*/

SID_STM32_FUNC_WRAPPER_DECLARE_WRAPPED_FUNCTION(SID_STM32_SPEED_OPTIMIZED void*, malloc(size_t n))
{
    return pvPortMalloc(n);
}

/*----------------------------------------------------------------------------*/

SID_STM32_FUNC_WRAPPER_DECLARE_WRAPPED_FUNCTION(SID_STM32_SPEED_OPTIMIZED void*, calloc(size_t n, size_t n1))
{
    void *p = NULL;
    const size_t size = n * n1;

    p = pvPortMalloc(size);

    if (p != NULL)
    {
        SID_STM32_UTIL_fast_memset(p, 0u, size);
    }

    return p;
}

/*----------------------------------------------------------------------------*/

SID_STM32_FUNC_WRAPPER_DECLARE_WRAPPED_FUNCTION(SID_STM32_SPEED_OPTIMIZED void, free(void *p))
{
    vPortFree(p);
}

/*----------------------------------------------------------------------------*/

SID_STM32_FUNC_WRAPPER_DECLARE_WRAPPED_FUNCTION(SID_STM32_SPEED_OPTIMIZED void*, realloc(void *p, size_t n))
{
    /* Not supported */
    assert(0);
    return NULL;
}
