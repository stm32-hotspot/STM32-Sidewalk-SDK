/**
  ******************************************************************************
  * @file    sid_st_common_defs.h
  * @brief   Header file with shared definitions for all ST-based Sidewalk apps.
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
#ifndef __SID_STM32_COMMON_DEFS_H_
#define __SID_STM32_COMMON_DEFS_H_

#ifdef __cplusplus
extern "C" {
#endif

#if !defined(__ICCARM__) && !(defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)) && !defined(__GNUC__)
#  error "Your compiler is not recognized. This file supports only IAR, GCC, and RealView (CC-ARM)"
#endif

/* Exported constants --------------------------------------------------------*/

/** @defgroup SID_STM32_COMMON_Exported_Constants SID STM32 Exported Constants
  * @{
  */

/** @defgroup SID_STM32_COMMON_Helpers Misc helper constants that are broadly used in code
  * @{
  */

#ifndef FALSE
#  define FALSE (0u)
#endif

#ifndef TRUE
#  define TRUE  (1u)
#endif

/**
  * @}
  */

/** @defgroup SID_STM32_COMMON_Optimization_Options Compiler-independent enforced optimization options
  * @{
  */

#if defined(__ICCARM__) /* IAR Compiler */
#  define SID_STM32_SPEED_OPTIMIZED _Pragma("optimize=speed")
#elif defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)  /* ARM Compiler V6 */
#  define SID_STM32_SPEED_OPTIMIZED _Pragma("O3") _Pragma("Otime")
#elif defined(__GNUC__) /* GNU Compiler */
#  define SID_STM32_SPEED_OPTIMIZED __attribute__((optimize("Ofast")))
#endif /* __ICCARM__ */

#if defined(__ICCARM__) /* IAR Compiler */
#  define SID_STM32_SIZE_OPTIMIZED _Pragma("optimize=size")
#elif defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)  /* ARM Compiler V6 */
#  define SID_STM32_SIZE_OPTIMIZED _Pragma("Ospace")
#elif defined(__GNUC__) /* GNU Compiler */
#  define SID_STM32_SIZE_OPTIMIZED __attribute__((optimize("Os")))
#endif /* __ICCARM__ */

/**
  * @}
  */

/** @defgroup SID_STM32_COMMON_Justification_Options Compiler-independent helpers to suppress certain warnings
  * @{
  */
#if defined(__ICCARM__) /* IAR Compiler */
#  define SID_STM32_JUSTIFY_FALLTHROUGH()     ((void)0)
#elif defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)  /* ARM Compiler V6 */
#  define SID_STM32_JUSTIFY_FALLTHROUGH()     ((void)0)
#elif defined(__GNUC__) /* GNU Compiler */
#  if GCC_VERSION >= 7000 && defined(__has_attribute)
#    if __has_attribute(fallthrough)
#      define SID_STM32_JUSTIFY_FALLTHROUGH() __attribute__((fallthrough))
#    else
#      define SID_STM32_JUSTIFY_FALLTHROUGH() ((void)0)
#    endif
#  else
#    define SID_STM32_JUSTIFY_FALLTHROUGH()   ((void)0)
#  endif /* GCC_VERSION */
#endif /* __ICCARM__ */

/**
  * @}
  */

/** @defgroup SID_STM32_COMMON_Arch_Specific_Constants CPU architecture-specific definitions
  * @{
  */

#ifndef __ARM_ARCH_6M__
  #define __ARM_ARCH_6M__               0
#endif
#ifndef __ARM_ARCH_7M__
  #define __ARM_ARCH_7M__               0
#endif
#ifndef __ARM_ARCH_7EM__
  #define __ARM_ARCH_7EM__              0
#endif
#ifndef __ARM_ARCH_8M_MAIN__
  #define __ARM_ARCH_8M_MAIN__          0
#endif
#ifndef __ARM_ARCH_7A__
  #define __ARM_ARCH_7A__               0
#endif

#if (__ARM_ARCH_7A__ != 0)
/* CPSR mask bits */
#  define SID_STM32_UTIL_CPSR_MASKBIT_I (0x80U)

/* CPSR mode bitmasks */
#  define SID_STM32_CPSR_MODE_USER      (0x10U)
#  define SID_STM32_CPSR_MODE_SYSTEM    (0x1FU)
#endif

/**
  * @}
  */

/**
  * @}
  */

/* Exported macros -----------------------------------------------------------*/
/** @defgroup SID_STM32_COMMON_Alignment_Options Compiler-independent enforced data alignment options
  * @{
  */

/* Macro to get variable aligned on 4-bytes */
#if defined(__ICCARM__) /* IAR Compiler */
#  define SID_STM32_ALIGN_4BYTES(buf) _Pragma("data_alignment=4") buf
#elif defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)  /* ARM Compiler V6 */
#  define SID_STM32_ALIGN_4BYTES(buf) __ALIGNED(4) buf
#elif defined(__GNUC__) /* GNU Compiler */
#  define SID_STM32_ALIGN_4BYTES(buf) buf __attribute__ ((aligned (4)))
#endif /* __ICCARM__ */

/* Macro to get variable aligned on 8-bytes */
#if defined(__ICCARM__) /* IAR Compiler */
#  define SID_STM32_ALIGN_8BYTES(buf) _Pragma("data_alignment=8") buf
#elif defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)  /* ARM Compiler V6 */
#  define SID_STM32_ALIGN_8BYTES(buf) __ALIGNED(8) buf
#elif defined(__GNUC__) /* GNU Compiler */
#  define SID_STM32_ALIGN_8BYTES(buf) buf __attribute__ ((aligned (8)))
#endif /* __ICCARM__ */

/* Macro to get variable aligned on 16-bytes */
#if defined(__ICCARM__) /* IAR Compiler */
#  define SID_STM32_ALIGN_16BYTES(buf) _Pragma("data_alignment=16") buf
#elif defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)  /* ARM Compiler V6 */
#  define SID_STM32_ALIGN_16BYTES(buf) __ALIGNED(16) buf
#elif defined(__GNUC__) /* GNU Compiler */
#  define SID_STM32_ALIGN_16BYTES(buf) buf __attribute__ ((aligned (16)))
#endif /* __ICCARM__ */

/* Macro to get variable aligned on 32-bytes */
#if defined(__ICCARM__) /* IAR Compiler */
#  define SID_STM32_ALIGN_32BYTES(buf) _Pragma("data_alignment=32") buf
#elif defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)  /* ARM Compiler V6 */
#  define SID_STM32_ALIGN_32BYTES(buf) __ALIGNED(32) buf
#elif defined(__GNUC__) /* GNU Compiler */
#  define SID_STM32_ALIGN_32BYTES(buf) buf __attribute__ ((aligned (32)))
#endif /* __ICCARM__ */

/**
  * @}
  */

/** @defgroup SID_STM32_COMMON_Endianess Macro to determine the endianess of the current platform
  * @{
  */

#if   (( defined( __BYTE_ORDER__ ) && defined( __ORDER_LITTLE_ENDIAN__ ) && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__ ) || ( __little_endian__ == 1 ) || ( __BYTE_ORDER == __LITTLE_ENDIAN ) || WIN32)
#  define SID_STM32_UTIL_IS_PLATFORM_LITLE_ENDIAN() (1)
#  define SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN()   (0)
#elif (( defined( __BYTE_ORDER__ ) && defined( __ORDER_BIG_ENDIAN__ )    && __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__ )    || ( __big_endian__ == 1 )    || ( __BYTE_ORDER == __BIG_ENDIAN ))
#  define SID_STM32_UTIL_IS_PLATFORM_LITLE_ENDIAN() (0)
#  define SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN()   (1)
#else
#  error "Unable to determine the endianess of the current platform"
#endif

/**
  * @}
  */

 /** @defgroup SID_STM32_COMMON_Symbol_Wrap Macro to wrap function calls with linker
  * @{
  */

#if defined(__ICCARM__) /* IAR Compiler */
#  define SID_STM32_FUNC_WRAPPER_DECLARE_WRAPPED_FUNCTION(ret_type, orig_func_proto) \
                                                                   extern ret_type $Super$$ ## orig_func_proto; \
                                                                   ret_type $Sub$$ ## orig_func_proto
#  define SID_STM32_FUNC_WRAPPER_CALL_ORIGINAL_FUNCTION(func_call) $Super$$ ## func_call
#elif defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)  /* ARM Compiler V6 */
                                                                   extern ret_type $Super$$ ## orig_func_proto; \
                                                                   ret_type $Sub$$ ## orig_func_proto
#  define SID_STM32_FUNC_WRAPPER_CALL_ORIGINAL_FUNCTION(func_call) $Super$$ ## func_call
#elif defined(__GNUC__) /* GNU Compiler */
#  define SID_STM32_FUNC_WRAPPER_DECLARE_WRAPPED_FUNCTION(ret_type, orig_func_proto) \
                                                                   extern ret_type __real_ ## orig_func_proto; \
                                                                   ret_type __wrap_ ## orig_func_proto
#  define SID_STM32_FUNC_WRAPPER_CALL_ORIGINAL_FUNCTION(func_call) __real_ ## func_call
#endif /* __ICCARM__ */

 /**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __SID_STM32_COMMON_DEFS_H_ */
