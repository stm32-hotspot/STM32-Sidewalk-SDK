/**
  ******************************************************************************
  * @file    storage_kv.c
  * @brief   sid_pal_storage_kv module implementation for STM32WBAxx MCUs
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

#include <sid_pal_storage_kv_ifc.h>
#include <sid_pal_critical_region_ifc.h>
#include <sid_pal_log_ifc.h>
#include <sid_pal_assert_ifc.h>

#include "lfs_adapter.h"
#include "lfs.h"

#include <sid_stm32_common_utils.h>

#include <stdalign.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app_conf.h"

/* Private defines -----------------------------------------------------------*/

#define SID_KVS_GROUP_NAME_PREFIX                           "/G"
#define SID_KVS_KEY_NAME_PREFIX                             "K"

#define SID_KVS_GROUP_ID_BINARY_SIZE                        (sizeof(uint16_t))
#define SID_KVS_KEY_ID_BINARY_SIZE                          (sizeof(uint16_t))

#define SID_KVS_HEX_CHARS_PER_BYTE                          (2u)

#define SID_KVS_GROUP_DIR_NAME_MAX_LEN                      (sizeof(SID_KVS_GROUP_NAME_PREFIX) + ((SID_KVS_HEX_CHARS_PER_BYTE) * (SID_KVS_GROUP_ID_BINARY_SIZE)))
#define SID_KVS_KEY_FILE_MAX_LEN                            (sizeof(SID_KVS_KEY_NAME_PREFIX)   + ((SID_KVS_HEX_CHARS_PER_BYTE) * (SID_KVS_KEY_ID_BINARY_SIZE)))

#define SID_KVS_DECLARE_GROUP_DIR_NAME(__dir__, __name__)   char __dir__[SID_KVS_GROUP_DIR_NAME_MAX_LEN];\
                                                            snprintf_like(__dir__, sizeof(__dir__), SID_KVS_GROUP_NAME_PREFIX"%X", __name__);
#define SID_KVS_DECLARE_KEY_FILE_NAME(__fname__, __name__)  char __fname__[SID_KVS_KEY_FILE_MAX_LEN];\
                                                            snprintf_like(__fname__, sizeof(__fname__), SID_KVS_KEY_NAME_PREFIX"%X", __name__);

/* Private variables ---------------------------------------------------------*/

static bool storage_kv_init_done = false;

/* Private function prototypes -----------------------------------------------*/

static inline sid_error_t lfs_to_sid_error(int32_t err);
static inline int snprintf_like(char * buf, const size_t size, const char *fmt, ...);

/* Private function definitions ----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline sid_error_t lfs_to_sid_error(int32_t err)
{
    sid_error_t sid_err;

    switch (err)
    {
        case LFS_ERR_OK:
            sid_err = SID_ERROR_NONE;
            break;

        case LFS_ERR_NOENT:
            sid_err = SID_ERROR_NOT_FOUND;
            break;

        case LFS_ERR_NOMEM:
            sid_err = SID_ERROR_OOM;
            break;

        case LFS_ERR_IO:
            sid_err = SID_ERROR_IO_ERROR;
            break;
        
        case LFS_ERR_EXIST:
            sid_err = SID_ERROR_ALREADY_EXISTS;
            break;

        case LFS_ERR_NOSPC:
            sid_err = SID_ERROR_STORAGE_FULL;
            break;

        case LFS_ERR_NOTDIR:
        case LFS_ERR_ISDIR:
        case LFS_ERR_BADF:
            sid_err = SID_ERROR_INVALID_ARGS;
            break;

        case LFS_ERR_INVAL:
            sid_err = SID_ERROR_INCOMPATIBLE_PARAMS;
            break;

        case LFS_ERR_FBIG:
        case LFS_ERR_NAMETOOLONG:
            sid_err = SID_ERROR_PARAM_OUT_OF_RANGE;
            break;

        default:
            sid_err = SID_ERROR_GENERIC;
            break;
    }

    return sid_err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline int snprintf_like(char * buf, const size_t size, const char *fmt, ...)
{
    va_list args;
    int print_len;

    va_start(args, fmt);
    print_len = UTIL_ADV_TRACE_VSNPRINTF(buf, size, fmt, args);
    va_end(args);

    return print_len;
}

/* Global function definitions -----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_pal_storage_kv_init(void)
{
    sid_error_t ret = SID_ERROR_NONE;

    if (storage_kv_init_done) {
        return SID_ERROR_NONE;
    }

    int32_t err = sid_pal_internal_fs_init();
    if(err == LFS_ERR_OK)
    {
        storage_kv_init_done = true;
        ret = SID_ERROR_NONE;
    }
    else
    {
        ret = SID_ERROR_GENERIC;
    }

    return ret;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_pal_storage_kv_deinit()
{
    sid_error_t ret = SID_ERROR_NONE;

    if (storage_kv_init_done) 
    {
        int32_t err = sid_pal_internal_fs_deinit();
        if(err == LFS_ERR_OK)
        {
            storage_kv_init_done = false;
            ret = SID_ERROR_NONE;
        }
        else
        {
            ret = SID_ERROR_GENERIC;
        }
    }

    return ret;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_pal_storage_kv_record_get(uint16_t group, uint16_t key, void * p_data, uint32_t len)
{
    SID_KVS_DECLARE_GROUP_DIR_NAME(dir, group);
    SID_KVS_DECLARE_KEY_FILE_NAME(fname, key);
    int32_t err = sid_pal_internal_fs_read_value( dir,
                                                  fname,
                                                  p_data, 
                                                  &len );

    return lfs_to_sid_error(err);
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_pal_storage_kv_record_get_len(uint16_t group, uint16_t key, uint32_t * p_len)
{
    SID_KVS_DECLARE_GROUP_DIR_NAME(dir, group);
    SID_KVS_DECLARE_KEY_FILE_NAME(fname, key);
    int32_t err = sid_pal_internal_fs_get_value_len( dir,
                                                fname,
                                                p_len );

    return lfs_to_sid_error(err);
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_pal_storage_kv_record_set(uint16_t group, uint16_t key, void const * p_data, uint32_t len)
{
    SID_KVS_DECLARE_GROUP_DIR_NAME(dir, group);
    SID_KVS_DECLARE_KEY_FILE_NAME(fname, key);
    int32_t err = sid_pal_internal_fs_write_value(  dir,
                                                    fname,
                                                    p_data, 
                                                    len );

    return lfs_to_sid_error(err);
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_pal_storage_kv_record_delete(uint16_t group, uint16_t key)
{
    SID_KVS_DECLARE_GROUP_DIR_NAME(dir, group);
    SID_KVS_DECLARE_KEY_FILE_NAME(fname, key);
    int32_t err = sid_pal_internal_delete_value( dir,
                                                 fname );


    return lfs_to_sid_error(err);
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_pal_storage_kv_group_delete(uint16_t group)
{
    SID_KVS_DECLARE_GROUP_DIR_NAME(dir, group);
    int32_t err = sid_pal_internal_delete_dir( dir );

    return lfs_to_sid_error(err);
}
