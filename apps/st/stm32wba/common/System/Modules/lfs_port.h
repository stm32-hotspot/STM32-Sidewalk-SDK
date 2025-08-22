/**
  ******************************************************************************
  * @file  lfs_port.h
  * @brief Functions and macro declarations for lfs_port.c
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

#ifndef __SID_STM32_LFS_PORT_H_
#define __SID_STM32_LFS_PORT_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include <stm32wbaxx_hal.h>

#include "lfs.h"
#include "lfs_util.h"
#include "target/memory.h"

/* Exported constants --------------------------------------------------------*/

#define LFS_CONFIG_BLOCK_COUNT          ( APP_CONFIG_FLASH_SIZE / FLASH_PAGE_SIZE )
#define CONFIG_LFS_FLASH_BASE           ( APP_CONFIG_FLASH_START )
#define CONFIG_LFS_FLASH_BASE_PAGE      ( (CONFIG_LFS_FLASH_BASE - FLASH_BASE) / FLASH_PAGE_SIZE )
#define LFS_CONFIG_LOOKAHEAD_SIZE       (256u)
#define LFS_CONFIG_CACHE_SIZE           (256u)

#define LFS_NO_MALLOC

/* Exported functions --------------------------------------------------------*/

#ifdef LFS_NO_MALLOC
const struct lfs_config * initialize_internal_flash_fs_static( void );
#else
const struct lfs_config * initialize_internal_flash_fs( void);
#endif

/* Provided outside of the lfs port */
lfs_t * pxGetDefaultFsCtx( void );

#ifdef __cplusplus
}
#endif

#endif /* __SID_STM32_LFS_PORT_H_ */