/**
  ******************************************************************************
  * @file    common_memory_symbols.h
  * @brief   Glue code to use linker-defined symbols in C code
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

#ifndef __TARGET_COMMON_MEMORY_SYMBOLS_H_
#define __TARGET_COMMON_MEMORY_SYMBOLS_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>

/* Exported constants --------------------------------------------------------*/

extern uint32_t __FLASH_region_start__;
extern uint32_t __FLASH_region_size__;
extern uint32_t __FLASH_region_end__;
#define APP_FLASH_START                ( (uint32_t)&__FLASH_region_start__)
#define APP_FLASH_SIZE                 ( (uint32_t)&__FLASH_region_size__)
#define APP_FLASH_END                  ( (uint32_t)&__FLASH_region_end__)


extern uint32_t __MFG_DATA_region_start__;
extern uint32_t __MFG_DATA_region_size__;
extern uint32_t __MFG_DATA_region_end__;
#define MANUFACTURE_FLASH_START        ( (uint32_t)&__MFG_DATA_region_start__)
#define MANUFACTURE_FLASH_SIZE         ( (uint32_t)&__MFG_DATA_region_size__)
#define MANUFACTURE_FLASH_END          ( (uint32_t)&__MFG_DATA_region_end__)


extern uint32_t __LITTLE_FS_region_start__;
extern uint32_t __LITTLE_FS_region_size__;
extern uint32_t __LITTLE_FS_region_end__;
#define APP_CONFIG_FLASH_START         ( (uint32_t)&__LITTLE_FS_region_start__)
#define APP_CONFIG_FLASH_SIZE          ( (uint32_t)&__LITTLE_FS_region_size__)
#define APP_CONFIG_FLASH_END           ( (uint32_t)&__LITTLE_FS_region_end__)


extern uint32_t __SNVMA_ARBTR_region_start__;
extern uint32_t __SNVMA_ARBTR_region_size__;
extern uint32_t __SNVMA_ARBTR_region_end__;
#define APP_CONFIG_SNVMA_FLASH_START   ( (uint32_t)&__SNVMA_ARBTR_region_start__)
#define APP_CONFIG_SNVMA_FLASH_SIZE    ( (uint32_t)&__SNVMA_ARBTR_region_size__)
#define APP_CONFIG_SNVMA_FLASH_END     ( (uint32_t)&__SNVMA_ARBTR_region_end__)


extern uint32_t __start_STACK;
extern uint32_t __end_STACK;
extern uint32_t __start_STACK_GUARD;
extern uint32_t __end_STACK_GUARD;
#define APP_CONFIG_USER_STACK_START       ( (uint32_t)&__start_STACK)
#define APP_CONFIG_USER_STACK_END         ( (uint32_t)&__end_STACK)
#define APP_CONFIG_USER_STACK_GUARD_START ( (uint32_t)&__start_STACK_GUARD)
#define APP_CONFIG_USER_STACK_GUARD_END   ( (uint32_t)&__end_STACK_GUARD)


extern uint32_t __start_HEAP;
extern uint32_t __end_HEAP;
#define APP_CONFIG_USER_HEAP_START       ( (uint32_t)&__start_HEAP)
#define APP_CONFIG_USER_HEAP_END         ( (uint32_t)&__end_HEAP)

#ifdef __cplusplus
}
#endif

#endif /* __TARGET_COMMON_MEMORY_SYMBOLS_H_ */
