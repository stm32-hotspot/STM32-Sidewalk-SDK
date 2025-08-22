/**
  ******************************************************************************
  * @file    lfs_adapter.c
  * @brief   Bridge layer for LittleFS
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

#include "lfs_adapter.h"

#include "app_conf.h"

#include <sid_pal_log_ifc.h>

#include <stdatomic.h>
#include <stdbool.h>

#include <cmsis_os2.h>

#include "lfs.h"
#include "lfs_port.h"

#include <sid_stm32_common_utils.h>

/* Private defines -----------------------------------------------------------*/

#define FSTORE_MAX_FNANME       (LFS_NAME_MAX+1)
#define FSTORE_MAX_FSIZE        (4096U - sizeof(FStoreHeader_t))

/* Private typedef -----------------------------------------------------------*/

typedef struct
{
    uint16_t len;
} FStoreHeader_t;

/* Private variables ---------------------------------------------------------*/

// configuration of the filesystem is provided by this struct
static lfs_t lfs;
const struct lfs_config * p_cfg = NULL;
static bool sid_pal_internal_fstorage_initialized = false;

/* Private function prototypes -----------------------------------------------*/

static inline void lfs_size_to_err(int32_t * const p_return_value, const size_t expected_length);

/* Private function definitions ----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline void lfs_size_to_err(int32_t * const p_return_value, const size_t expected_length)
{
    if (*p_return_value == expected_length)
    {
        *p_return_value = LFS_ERR_OK;
    }
    else if (*p_return_value >= 0)
    {
        *p_return_value = LFS_ERR_CORRUPT;
    }
    else
    {
        /* Pass through the error code otherwise */
    }
}

/* Global function definitions -----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_internal_fs_init(void)
{
    int32_t err = LFS_ERR_OK;

    if(sid_pal_internal_fstorage_initialized)
    {
        return err;
    }

    /* internal flash initialize */
    p_cfg = initialize_internal_flash_fs_static();
    if(p_cfg == NULL)
    {
        err = LFS_ERR_IO;
    }

    /* mount the filesystem */
    if (err == LFS_ERR_OK)
    {
        err = lfs_mount(&lfs, p_cfg);
    }

    /* format if we can't mount the filesystem
     * this should only happen on the first boot
     */
    if (err != LFS_ERR_OK)
    {
        SID_PAL_LOG_DEBUG("lfs: Failed to mount LFS partition. Formatting...");

        err = lfs_format(&lfs, p_cfg);

        if (err == 0)
        {
            err = lfs_mount(&lfs, p_cfg);
        }

        if (err != LFS_ERR_OK)
        {
            SID_PAL_LOG_ERROR("lfs: Failed to format LFS partition");
        }
    }

    if (err == LFS_ERR_OK)
    {
        sid_pal_internal_fstorage_initialized = true;
    }
    else
    {
        sid_pal_internal_fstorage_initialized = false;
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_internal_fs_deinit(void)
{
    int32_t err = LFS_ERR_OK;
    if (!sid_pal_internal_fstorage_initialized)
    {
        return err;
    }

    /* unmount the filesystem */
    if (err == LFS_ERR_OK)
    {
        err = lfs_unmount(&lfs);
    }

    if (err == LFS_ERR_OK)
    {
        sid_pal_internal_fstorage_initialized = false;
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_internal_fs_write_value(const char * p_dir, const char * p_name, const void * p_buf, uint32_t buf_len)
{
    struct lfs_info dir_info = { 0 };
    char file_name[ FSTORE_MAX_FNANME + 1] = { 0 };
    bool file_open_flag = false;
    lfs_file_t fil = { 0 };
    int32_t err = LFS_ERR_INVAL;

    if ((p_buf == NULL) || (buf_len > FSTORE_MAX_FSIZE))
    {
        err = LFS_ERR_INVAL;
        return err;
    }

    err = lfs_stat(&lfs, p_dir, &dir_info);
    if (err == LFS_ERR_NOENT)
    {
        err = lfs_mkdir(&lfs, p_dir);
    }


    if (err != LFS_ERR_OK)
    {
        SID_PAL_LOG_ERROR("lfs: Failed to create directory.");
    }
    else
    {
        /* Construct file name */
        (void)strncpy(file_name, p_dir, FSTORE_MAX_FNANME);
        (void)strncat(file_name, "/", FSTORE_MAX_FNANME);
        (void)strncat(file_name, p_name, FSTORE_MAX_FNANME);

        /* Open the file */
        err = lfs_file_open(&lfs, &fil, file_name, LFS_O_WRONLY | LFS_O_TRUNC | LFS_O_CREAT);
    }

    if (err != LFS_ERR_OK)
    {
        SID_PAL_LOG_ERROR("lfs: Error while opening file: %s.", file_name);
    }
    else /* Write the header if opened successfully */
    {
        file_open_flag = true;
        FStoreHeader_t file_header =
        {
            .len = buf_len
        };

        err = lfs_file_write(&lfs, &fil, &file_header, sizeof(FStoreHeader_t));
        lfs_size_to_err(&err, sizeof(FStoreHeader_t));
    }

    if (err != LFS_ERR_OK)
    {
        SID_PAL_LOG_ERROR("lfs: Error while writing FStoreHeader_t of length %ld bytes to file: %s.", sizeof(FStoreHeader_t), file_name);
    }
    else /* Write the data */
    {
        err = lfs_file_write(&lfs, &fil, p_buf, buf_len);
        lfs_size_to_err(&err, buf_len);
    }

    if (err != LFS_ERR_OK)
    {
        SID_PAL_LOG_ERROR("lfs: Error while writing data of length %ld bytes to file: %s.", buf_len, file_name);
    }

    if (file_open_flag)
    {
        (void)lfs_file_close(&lfs, &fil);

        /* Delete partially written file if writing was not successful */
        if (err != LFS_ERR_OK)
        {
            (void)lfs_remove(&lfs, file_name);
        }
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_internal_fs_read_value(const char * p_dir, const char * p_name, void * p_buf, uint32_t * buf_len)
{
    char file_name[ FSTORE_MAX_FNANME + 1 ] = { 0 };
    bool file_open_flag = false;
    lfs_file_t fil = { 0 };
    int32_t err = LFS_ERR_INVAL;

    if(p_buf == NULL)
    {
        err = LFS_ERR_INVAL;
        return err;
    }

    /* Construct file name */
    (void)strncpy(file_name, p_dir, FSTORE_MAX_FNANME);
    (void)strncat(file_name, "/", FSTORE_MAX_FNANME);
    (void)strncat(file_name, p_name, FSTORE_MAX_FNANME);

    FStoreHeader_t file_header = { 0 };

    /* Open the file */
    err = lfs_file_open(&lfs, &fil, file_name, LFS_O_RDONLY);

    /* Read the header */
    if (err == LFS_ERR_OK)
    {
        file_open_flag = true;
        err = lfs_file_read(&lfs, &fil, &file_header, sizeof(FStoreHeader_t));
        lfs_size_to_err(&err, sizeof(FStoreHeader_t));
    }

    /* copy data to provided buffer */
    if (err >= LFS_ERR_OK)
    {
        err = lfs_file_read(&lfs, &fil, p_buf, file_header.len);
        lfs_size_to_err(&err, file_header.len);
    }

    if (err == LFS_ERR_OK)
    {
        if (buf_len != NULL)
        {
            if (err == LFS_ERR_OK)
            {
                *buf_len = file_header.len;
            }
            else
            {
                *buf_len = 0;
            }
        }
    }

    if (file_open_flag)
    {
        (void)lfs_file_close(&lfs, &fil);
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_internal_fs_get_value_len(const char * const p_dir, const char * const p_name, uint32_t * const p_len)
{
    struct lfs_info file_info = { 0 };
    char file_name[ FSTORE_MAX_FNANME + 1 ] = { 0 };
    int32_t err = LFS_ERR_INVAL;

    if(p_len == NULL)
    {
        err = LFS_ERR_INVAL;
        return err;
    }

    /* Construct file name */
    (void)strncpy(file_name, p_dir, FSTORE_MAX_FNANME);
    (void)strncat(file_name, "/", FSTORE_MAX_FNANME);
    (void)strncat(file_name, p_name, FSTORE_MAX_FNANME);

    err = lfs_stat(&lfs, file_name, &file_info);
    if (LFS_ERR_OK == err)
    {
        *p_len = (file_info.size - sizeof(FStoreHeader_t));
    }
    else
    {
        *p_len = 0;
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_internal_delete_value(const char * p_dir, const char * p_name)
{
    char file_name[ FSTORE_MAX_FNANME + 1 ] = { 0 };
    int32_t err = LFS_ERR_INVAL;

    /* Construct file name */
    (void)strncpy(file_name, p_dir, FSTORE_MAX_FNANME);
    (void)strncat(file_name, "/", FSTORE_MAX_FNANME);
    (void)strncat(file_name, p_name, FSTORE_MAX_FNANME);

    err = lfs_remove(&lfs, file_name);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_internal_delete_dir(const char * p_dir)
{
    int32_t err = LFS_ERR_INVAL;
    lfs_dir_t dir;
    struct lfs_info dir_info;
    struct lfs_info file_info;
    char file_name[ FSTORE_MAX_FNANME + 1 ] = { 0 };

    /* check the directory */
    err = lfs_stat(&lfs, p_dir, &dir_info);
    if (err == LFS_ERR_NOENT)
    {
        /* directory doesn't exist */
        err = LFS_ERR_OK;
    }
    else
    {
        /* open the directory */
        err = lfs_dir_open(&lfs, &dir, p_dir);
        if(err == LFS_ERR_OK)
        {
            /* remove all files in the directory */
            while (lfs_dir_read(&lfs, &dir, &file_info) > 0)
            {
                if (file_info.type == LFS_TYPE_REG)
                {
                    /* Construct file name */
                    memset(file_name, 0, sizeof(file_name));
                    (void)strncpy(file_name, p_dir, FSTORE_MAX_FNANME);
                    (void)strncat(file_name, "/", FSTORE_MAX_FNANME);
                    (void)strncat(file_name, file_info.name, FSTORE_MAX_FNANME);
                    err = lfs_remove(&lfs, file_name);
                    if(err != LFS_ERR_OK)
                    {
                        break;
                    }
                }
            }
            err = lfs_dir_close(&lfs, &dir);
        }

        /* remove the directory */
        if(err == LFS_ERR_OK)
        {
            err = lfs_remove(&lfs, p_dir);
        }
    }

    return err;
}
