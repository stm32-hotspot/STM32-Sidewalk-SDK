/**
  ******************************************************************************
  * @file    sid_stm32_common_utils.h
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

 /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SID_STM32_COMMON_UTILS_H_
#define __SID_STM32_COMMON_UTILS_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>
#include <sid_stm32_common_defs.h>

#ifdef SID_STM32_UTIL_USE_CMSIS
#  include <cmsis_os.h>
#endif

/* Exported macros -----------------------------------------------------------*/
/** @defgroup SID_STM32_COMMON_UTILS_Exported_Macros SID STM32 Exported Macros
  * @{
  */

/**
 * @brief Computes the length (number of elements) for an array
 */
#define SID_STM32_UTIL_ARRAY_SIZE(__ARRAY__)         (sizeof((__ARRAY__))/(sizeof((__ARRAY__)[0])))

/**
 * @brief Gets the size of a struct member by using the struct's data type definition
 */
#define SID_STM32_UTIL_STRUCT_MEMBER_SIZE(__STRUCT_TYPE__, __MEMBER__) \
                                                     (sizeof( ((__STRUCT_TYPE__ *)(void *)0)->__MEMBER__))

 /**
  * @brief Computes bit number of the first set lsb (e.g. for 0b00110000 it will return 4)
  */
#define SID_STM32_UTIL_POSITION_VAL(__val__)         (__CLZ(__RBIT(__val__)))

/**
 * @brief Computes exponent of the nearest power of 2 that is greater or equal to the supplied value
 */
#define SID_STM32_UTIL_ROUNDUP_NEXT_POW_2_EXP(__x__) ((uint32_t)(32u - __CLZ((__x__) - 1u)))

/**
 * @brief Rounds the supplied value up to the nearest power of 2
 */
#define SID_STM32_UTIL_ROUNDUP_NEXT_POW_2(__x__)     ((uint32_t)(1u << (ROUNDUP_NEXT_POW_2_EXP(__x__))))

/**
 * @brief Swap endianess of a 16-bit value
 */
#define SID_STM32_UTIL_SWAP_BYTES_16(__x__)          ((uint16_t)((((uint16_t)(__x__) & 0x00FFu) << 8) | (((uint16_t)(__x__) & 0xFF00u) >> 8)))

/**
 * @brief Swap endianess of a 32-bit value
 */
#define SID_STM32_UTIL_SWAP_BYTES_32(__x__)          ((uint32_t)((((uint32_t)(__x__) & 0x000000FFu) << 24) | (((uint32_t)(__x__) & 0xFF000000u) >> 24) | \
                                                                 (((uint32_t)(__x__) & 0x0000FF00u) <<  8) | (((uint32_t)(__x__) & 0x00FF0000u) >> 8)))

/** @defgroup SID_STM32_COMMON_Execution_Context_Getters Helper methods to identify if the calling code is running in an IRQ or IRQ-like context
  * @{
  */
#if   ((__ARM_ARCH_7M__      == 1U) || \
       (__ARM_ARCH_7EM__     == 1U) || \
       (__ARM_ARCH_8M_MAIN__ == 1U))
#  define SID_STM32_UTIL_IS_IRQ_MASKED()             ((__get_PRIMASK() != 0U) || (__get_BASEPRI() != 0U))
#elif  (__ARM_ARCH_6M__      == 1U)
#  define SID_STM32_UTIL_IS_IRQ_MASKED()             (__get_PRIMASK() != 0U)
#elif (__ARM_ARCH_7A__       == 1U)
#  define SID_STM32_UTIL_IS_IRQ_MASKED()             ((__get_CPSR() & SID_STM32_UTIL_CPSR_MASKBIT_I) != 0U)
#else
#  define SID_STM32_UTIL_IS_IRQ_MASKED()             (__get_PRIMASK() != 0U)
#endif

#if    (__ARM_ARCH_7A__      == 1U)
#  define SID_STM32_UTIL_IS_IRQ_MODE()               ((__get_mode() != SID_STM32_CPSR_MODE_USER) && (__get_mode() != SID_STM32_CPSR_MODE_SYSTEM))
#else
#  define SID_STM32_UTIL_IS_IRQ_MODE()               (__get_IPSR() != 0U)
#endif

#ifdef SID_STM32_UTIL_USE_CMSIS
#  define SID_STM32_UTIL_IS_IRQ()                    (SID_STM32_UTIL_IS_IRQ_MODE() || (SID_STM32_UTIL_IS_IRQ_MASKED() && ((osKernelGetState() != osKernelInactive) && (osKernelGetState() != osKernelReady))))
#else
#  define SID_STM32_UTIL_IS_IRQ()                    (SID_STM32_UTIL_IS_IRQ_MODE() || SID_STM32_UTIL_IS_IRQ_MASKED())
#endif

/**
  * @}
  */

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup SID_STM32_COMMON_Exported_Functions SID STM32 Exported Functions
  * @{
  */

/**
  * @brief  Speed-optimized implementation of the memcopy() function for 32-bit MCUs
  * @note   This implementation allows unaligned _dst and _src locations, however performance is degraded in such cases
  * @param  _dst Pointer to the destination memory location
  * @param  _src Pointer to the source memory location
  * @param  size Amount of data to copy (in bytes)
  * @retval None.
  */
void SID_STM32_UTIL_fast_memcpy(void * _dst, const void * _src, const uint32_t size);

/**
  * @brief  Speed-optimized implementation of the memset() function for 32-bit MCUs
  * @note   This implementation allows unaligned _dst and _src locations, however performance is degraded in such cases
  * @param  _dst Pointer to the destination memory location
  * @param  c    Value to set the memory to
  * @param  size Amount of data to set (in bytes)
  * @retval None.
  */
void SID_STM32_UTIL_fast_memset(void * _dst, const uint8_t c, const uint32_t size);

/**
  * @brief  Speed-optimized implementation of the memcmp()-like function for 32-bit MCUs
  * @note   This implementation allows unaligned _a and _a locations, however performance is degraded in such cases
  * @param  _a Pointer to one memory location to be compared
  * @param  _b Pointer to another memory location to be compared
  * @param  size Amount of data to compare (in bytes)
  * @retval Zero if two memory blocks are identical and non-zero value otherwise. Unlike the standard memcmp(),
  *         this implementation does not detect if a > b or a < b, just a non-zero value is returned
  */
uint32_t SID_STM32_UTIL_fast_memcmp(const void * const _a, const void * const _b, const uint32_t size);

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __SID_STM32_COMMON_UTILS_H_ */
