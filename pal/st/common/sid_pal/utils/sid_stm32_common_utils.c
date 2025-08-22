/**
  ******************************************************************************
  * @file    sid_stm32_common_utils.c
  * @brief   Utility functions applicable to all ST-based Sidewalk apps
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

/* Includes ------------------------------------------------------------------*/

#include <sid_stm32_common_defs.h>
#include <sid_stm32_common_utils.h>

/* Private macro -------------------------------------------------------------*/

#define MOD_4(_x_) ((uint32_t)(_x_) & (uint32_t)0x03u) /*!< Fast equivalent to (x % 4) operation */

/* Global function definitions -----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED void SID_STM32_UTIL_fast_memcpy(void * _dst, const void * _src, const uint32_t size)
{
    register uint32_t unaligned_prefix_sz = size < sizeof(uint32_t) ? size : MOD_4(_dst) != 0u ? (sizeof(uint32_t) - (MOD_4(_dst))) : 0u; /* Favor _dst alignment since unaligned writes are more time consuming than unaligned reads */
    register uint32_t unaligned_postfix_sz = MOD_4(size - unaligned_prefix_sz);
    register uint32_t aligned_data_sz = (size - (unaligned_prefix_sz + unaligned_postfix_sz));

    /* Calculated the aligned data pointers first */
    register uint32_t * _dst32 = (uint32_t *)(void *)((uint32_t)_dst + unaligned_prefix_sz);
    register const uint32_t * _src32 = (uint32_t *)(void *)((uint32_t)_src + unaligned_prefix_sz);

    /* Copy any prepending unaligned data */
    if (aligned_data_sz > 0u)
    {
        /* Since unaligned prefix is guaranteed to be 3 bytes or less and there's more space in the buffer after that, we can go with a single 4 byte unaligned copy operation */
        register uint32_t * _unaligned_dst32 = (uint32_t*)_dst;
        register const uint32_t * _unaligned_src32 = (uint32_t*)_src;
        *_unaligned_dst32 = *_unaligned_src32;
    }
    else
    {
        /* Unfortunately there's no room for a word-sized copy. Go with byte-by-byte solution */
        register uint8_t * _dst8 = (uint8_t*)_dst;
        register const uint8_t * _src8 = (uint8_t*)_src;
        while (unaligned_prefix_sz > 0u)
        {
            *_dst8 = *_src8;
            ++_dst8;
            ++_src8;
            unaligned_prefix_sz -= sizeof(uint8_t);
        }
    }

    /* For aligned data copy 4 bytes at a time */
    while (aligned_data_sz > 0)
    {
        *_dst32 = *_src32;
        ++_dst32;
        ++_src32;
        aligned_data_sz -= sizeof(uint32_t);
    }

    /* Copy any remaining unaligned data. Since we don't know what's behind these last bytes we can only go with byte-by-byte copy */
    register uint8_t * _dst8 = (uint8_t*)(void *)_dst32;
    register const uint8_t * _src8 = (uint8_t*)(void *)_src32;
    while (unaligned_postfix_sz > 0u)
    {
        *_dst8 = *_src8;
        ++_dst8;
        ++_src8;
        unaligned_postfix_sz -= sizeof(uint8_t);
    }
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED void SID_STM32_UTIL_fast_memset(void * _dst, const uint8_t c, const uint32_t size)
{
	register uint32_t unaligned_prefix_sz = size < sizeof(uint32_t) ? size : MOD_4(_dst) != 0u ? (sizeof(uint32_t) - (MOD_4(_dst))) : 0u; /* Favor _dst alignment since unaligned writes are more time consuming than unaligned reads */
    register uint32_t unaligned_postfix_sz = MOD_4(size - unaligned_prefix_sz);
    register uint32_t aligned_data_sz = (size - (unaligned_prefix_sz + unaligned_postfix_sz));

    /* Prepare data */
    register const uint8_t c_byte_reg = c;
    register const uint32_t c_word_reg = ((uint32_t)c << 24) | ((uint32_t)c << 16) | ((uint32_t)c << 8) | ((uint32_t)c);

    /* Calculated the aligned data pointers first */
    register uint32_t * _dst32 = (uint32_t *)(void *)((uint32_t)_dst + unaligned_prefix_sz);

    /* Copy any prepending unaligned data */
    if (aligned_data_sz > 0u)
    {
        /* Since unaligned prefix is guaranteed to be 3 bytes or less and there's more space in the buffer after that, we can go with a single 4 byte unaligned copy operation */
        register uint32_t * _unaligned_dst32 = (uint32_t*)_dst;
        *_unaligned_dst32 = c_word_reg;
    }
    else
    {
        /* Unfortunately there's no room for a word-sized copy. Go with byte-by-byte solution */
        register uint8_t * _dst8 = (uint8_t*)_dst;
        while (unaligned_prefix_sz > 0u)
        {
            *_dst8 = c_byte_reg;
            ++_dst8;
            unaligned_prefix_sz -= sizeof(uint8_t);
        }
    }

    /* For aligned data copy 4 bytes at a time */
    while (aligned_data_sz > 0)
    {
        *_dst32 = c_word_reg;
        ++_dst32;
        aligned_data_sz -= sizeof(uint32_t);
    }

    /* Copy any remaining unaligned data. Since we don't know what's behind these last bytes we can only go with byte-by-byte copy */
    register uint8_t * _dst8 = (uint8_t*)(void *)_dst32;
    while (unaligned_postfix_sz > 0u)
    {
        *_dst8 = c_byte_reg;
        ++_dst8;
        unaligned_postfix_sz -= sizeof(uint8_t);
    }
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED uint32_t SID_STM32_UTIL_fast_memcmp(const void * const _a, const void * const _b, const uint32_t size)
{
    register uint32_t unaligned_prefix_sz = size < sizeof(uint32_t) ? size : MOD_4(_a) != 0u ? (sizeof(uint32_t) - (MOD_4(_a))) : 0u; /* Favor _a alignment */
    register uint32_t unaligned_postfix_sz = MOD_4(size - unaligned_prefix_sz);
    register uint32_t aligned_data_sz = (size - (unaligned_prefix_sz + unaligned_postfix_sz));

    /* Calculated the aligned data pointers first */
    register const uint32_t * _a32 = (uint32_t *)(void *)((uint32_t)_a + unaligned_prefix_sz);
    register const uint32_t * _b32 = (uint32_t *)(void *)((uint32_t)_b + unaligned_prefix_sz);

    /* Compare any prepending unaligned data */
    if (aligned_data_sz > 0u)
    {
        /* Since unaligned prefix is guaranteed to be 3 bytes or less and there's more space in the buffer after that, we can go with a single 4 byte unaligned copy operation */
        register const uint32_t * _unaligned_a32 = (uint32_t*)_a;
        register const uint32_t * _unaligned_b32 = (uint32_t*)_b;
        if (*_unaligned_a32 != *_unaligned_b32)
        {
            return 1u;
        }
    }
    else
    {
        /* Unfortunately there's no room for a word-sized compare. Go with byte-by-byte solution */
        register const uint8_t * _a8 = (uint8_t*)_a;
        register const uint8_t * _b8 = (uint8_t*)_b;
        while (unaligned_prefix_sz > 0u)
        {
            if (*_a8 != *_b8)
            {
                return 1u;
            }
            ++_a8;
            ++_b8;
            unaligned_prefix_sz -= sizeof(uint8_t);
        }
    }

    /* For aligned data copy 4 bytes at a time */
    while (aligned_data_sz > 0)
    {
        if (*_a32 != *_b32)
        {
            return 1;
        }
        ++_a32;
        ++_b32;
        aligned_data_sz -= sizeof(uint32_t);
    }

    /* Copy any remaining unaligned data. Since we don't know what's behind these last bytes we can only go with byte-by-byte copy */
    register const uint8_t * _a8 = (uint8_t*)(void *)_a32;
    register const uint8_t * _b8 = (uint8_t*)(void *)_b32;
    while (unaligned_postfix_sz > 0u)
    {
        if (*_a8 != *_b8)
        {
            return 1u;
        }
        ++_a8;
        ++_b8;
        unaligned_postfix_sz -= sizeof(uint8_t);
    }

    return 0u;
}
