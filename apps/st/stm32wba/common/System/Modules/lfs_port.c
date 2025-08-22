/**
  ******************************************************************************
  * @file    lfs_port.c
  * @brief Sidewalk LittleFS port for STM32WBAxx implementation
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

/* Includes ------------------------------------------------------------------*/

#include "lfs_util.h"
#include "lfs.h"
#include "lfs_port.h"

#include "app_conf.h"
#include "stm32_rtos.h"
#ifdef BLE
#  include <ll_sys_if.h>
#endif /* BLE */

#include <cmsis_os.h>
#include <flash_driver.h>
#include <flash_manager.h>
#include <stm32wbaxx_hal.h>
#include <stm32wbaxx_hal_flash.h>

/* Private defines -----------------------------------------------------------*/

/**
 * @brief Size of a flash memory word in bytes.
 *
 * This macro defines the size (in bytes) of a flash memory word.
 */
#define FLASH_WORD_SIZE sizeof(uint32_t)

/**
 * @brief Number of flash memory words in a quad.
 *
 * This macro represents the number of flash memory words in a quad, typically used in memory operations.
 */
#define FLASH_WORDS_IN_QUAD (4U)

/**
 * @brief Number of flash memory words in a burst.
 *
 * This macro specifies the number of flash memory words in a burst, which is a group of consecutive words
 * often used in memory write to speed up the process.
 */
#define FLASH_WORDS_IN_BURST (32)

/**
 * @brief Size of a burst in bytes for flash memory operations.
 *
 * This macro calculates the size (in bytes) of a burst in flash memory based on the flash word size
 * and the number of flash words in a burst.
 */
#define FLASH_WRITE_BURST_SIZE (FLASH_WORD_SIZE * FLASH_WORDS_IN_BURST)

/* Private variables ---------------------------------------------------------*/

#ifdef LFS_NO_MALLOC
static uint8_t __ALIGN_BEGIN ucReadBuffer[ LFS_CONFIG_CACHE_SIZE ] __ALIGN_END = { 0 };
static uint8_t __ALIGN_BEGIN ucProgBuffer[ LFS_CONFIG_CACHE_SIZE ] __ALIGN_END = { 0 };
static uint8_t __ALIGN_BEGIN ucLookAheadBuffer[ LFS_CONFIG_LOOKAHEAD_SIZE ] __ALIGN_END = { 0 };
static struct lfs_config p_lfs_cfg = { 0 };
#endif

uint32_t littlefs_ecc_error_address;

static osSemaphoreId_t lfs_fm_semaphore = NULL;

static const osSemaphoreAttr_t lfs_fm_semaphore_attributes = {
    .name      = "LFS FM Semaphore",
    .attr_bits = SEMAPHORE_DEFAULT_ATTR_BITS,
    .cb_mem    = SEMAPHORE_DEFAULT_CB_MEM,
    .cb_size   = SEMAPHORE_DEFAULT_CB_SIZE,
};

static FM_FlashOp_Status_t last_fm_status;

static FM_CallbackNode_t lfs_fm_callback_node;

static FM_FlashOp_Status_t last_fm_status;

/* Private function prototypes -----------------------------------------------*/

static void lfs_fm_callback(FM_FlashOp_Status_t Status);
static inline bool address_within_borders(const uint32_t address);
static inline bool pages_within_borders(const uint32_t first_page, const uint32_t quantity);
static int lfs_port_read(const struct lfs_config * c, lfs_block_t block, lfs_off_t off, void * buffer, lfs_size_t size);
static int lfs_port_prog(const struct lfs_config * c, lfs_block_t block, lfs_off_t off, const void * buffer, lfs_size_t size);
static int lfs_port_erase(const struct lfs_config * c, lfs_block_t block);
static int lfs_port_sync(const struct lfs_config * c);
static void populate_config(struct lfs_config * p_cfg);

/* Private function definitions ----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static void lfs_fm_callback(FM_FlashOp_Status_t Status)
{
    // FIXME: check for errors
    last_fm_status = Status;
    osSemaphoreRelease(lfs_fm_semaphore);
}

/*----------------------------------------------------------------------------*/

/**
 * @brief Checks if the given address is within the specified flash memory borders.
 *
 * This function compares the provided address with the start and end boundaries
 * of the littleFS flash memory range.
 *
 * @param address The address to be checked for within the flash memory borders.
 * @return true if the address is within the specified flash memory range, false otherwise.
 */
SID_STM32_SPEED_OPTIMIZED static inline bool address_within_borders(const uint32_t address)
{
    return (address >= APP_CONFIG_FLASH_START && address < APP_CONFIG_FLASH_END);
}

/*----------------------------------------------------------------------------*/

/**
 * @brief Checks if the specified range of flash memory pages is within the valid flash memory borders.
 *
 * This function validates a range of flash memory pages defined by the first page and quantity parameters.
 * It checks if the range falls within the littleFS flash memory region.
 *
 * @param first_page The starting page number of the flash memory range.
 * @param quantity   The quantity of pages to check.
 * @return true if the specified flash memory page range is within the valid flash memory borders, false otherwise.
 */
SID_STM32_SPEED_OPTIMIZED static inline bool pages_within_borders(const uint32_t first_page, const uint32_t quantity)
{
    bool retCode = true;
    
    if(quantity > 0)
    {
        /* Calculate the last page in the specified range */
        uint32_t last_page = first_page + quantity - 1;

        /* Check if the range is within the valid flash memory borders */
        if(first_page < CONFIG_LFS_FLASH_BASE_PAGE || 
           last_page > CONFIG_LFS_FLASH_BASE_PAGE + LFS_CONFIG_BLOCK_COUNT)
        {
            retCode = false;
        }
    }
    else
    {
        /* If quantity is non-positive, the range is invalid */
        retCode = false;
    }

    return retCode;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static int lfs_port_read(const struct lfs_config * c, lfs_block_t block, lfs_off_t off, void * buffer, lfs_size_t size)
{
    int retCode;

    do
    {
        const uint32_t src_address = CONFIG_LFS_FLASH_BASE + block * c->block_size + off;

        if ((address_within_borders(src_address) == false)
            || (address_within_borders(src_address + size) == false))
        {
            LFS_ERROR("Read address range 0x%08X-0x%08X is beyond the LFS region", src_address, (src_address + size - 1u));
            retCode = LFS_ERR_INVAL;
            break;
        }

        /* Invalidate ECC detection address */
        littlefs_ecc_error_address = 0u;

        /* Read data */
        (void)memcpy(buffer, (void *)src_address, size);

        /* Check for ECC detection */
        if (address_within_borders(littlefs_ecc_error_address) != false)
        {
            LFS_ERROR("Uncorrectable ECC error at address 0x%08X", littlefs_ecc_error_address);
            retCode = LFS_ERR_CORRUPT;
        }

        /* Done */
        retCode = LFS_ERR_OK;
    } while (0);

    return retCode;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static int lfs_port_prog(const struct lfs_config * c, lfs_block_t block, lfs_off_t off, const void * buffer, lfs_size_t size)
{
    int                 retCode = LFS_ERR_OK;
    FM_Cmd_Status_t     fm_err; /* Flash Manager error code */
    FD_FlashOp_Status_t fd_status; /* Flash Driver error code */
    osStatus_t          os_status;
    const uint32_t      write_start_addr = CONFIG_LFS_FLASH_BASE + (block * c->block_size) + off;

    /* Data has to be aligned to prog size */
    if ((off % c->prog_size) != 0u)
    {
        LFS_ERROR("Write offset 0x%08X is not aligned to flash word size", off);
        return LFS_ERR_INVAL;
    }

    /* Data has to be aligned to prog size */
    if ((size % c->prog_size) != 0u)
    {
        LFS_ERROR("Write data size %u is not aligned to flash word size", size);
        return LFS_ERR_INVAL;
    }

    /* Checks if addresses are within borders */
    if ((address_within_borders(write_start_addr) == false)
        || (address_within_borders(write_start_addr + size) == false))
    {
        LFS_ERROR("Write address range 0x%08X-0x%08X is beyond the LFS region", write_start_addr, (write_start_addr + size - 1u));
        return LFS_ERR_INVAL;
    }

    if (osKernelGetState() == osKernelRunning)
    {
        /* RTOS is running, we can use Flash Manager module for async operations and to avoid conflicts with 2.4GHz radio operations */
        LFS_DEBUG("Writing %u bytes to flash staring at 0x%08X with FM", size, write_start_addr);
        do
        {
            fm_err = FM_Write((uint32_t *)buffer, (uint32_t *)write_start_addr, size / sizeof(uint32_t), &lfs_fm_callback_node);
            if (fm_err != FM_OK)
            {
                retCode = LFS_ERR_IO;
                LFS_ERROR("Failed to initiate flash write");
                break;
            }

            os_status = osSemaphoreAcquire(lfs_fm_semaphore, osWaitForever);
            if (os_status != osOK)
            {
                retCode = LFS_ERR_IO;
                LFS_ERROR("Failed to wait for flash write completion. Error %d", (int32_t)os_status);
                break;
            }
        } while (last_fm_status != FM_OPERATION_COMPLETE);
    }
    else
    {
        /* RTOS not running, fall back to direct calls to Flash Driver */
        LFS_DEBUG("Writing %u bytes to flash staring at 0x%08X with FD", size, write_start_addr);

        uint32_t src_addr = (uint32_t)buffer;
        uint32_t dst_addr = write_start_addr;
        uint32_t bytes_left = size;

        (void)HAL_FLASH_Unlock();

        /* Data writing loop */
        while(bytes_left > 0u)
        {
            fd_status = FD_WriteData(dst_addr, src_addr);
            if (fd_status != FD_FLASHOP_SUCCESS)
            {
                retCode = LFS_ERR_IO;
                LFS_ERROR("Failed to program flash at 0x%08X", dst_addr);
                break;
            }

            dst_addr   += c->prog_size;
            src_addr   += c->prog_size;
            bytes_left -= c->prog_size;
        }

        (void)HAL_FLASH_Lock();
    }

    if (LFS_ERR_OK == retCode)
    {
        LFS_DEBUG("Stored %u bytes to flash starting at 0x%08X", size, write_start_addr);
    }
    else
    {
        LFS_ERROR("Failed to store %u bytes to flash starting at 0x%08X", size, write_start_addr);
    }

    return retCode;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static int lfs_port_erase(const struct lfs_config * c, lfs_block_t block)
{
    int                 retCode = LFS_ERR_OK;
    FM_Cmd_Status_t     fm_err; /* Flash Manager error code */
    FD_FlashOp_Status_t fd_status; /* Flash Driver error code */
    osStatus_t          os_status;
    const uint32_t      page_to_erase = CONFIG_LFS_FLASH_BASE_PAGE + block;
    const uint32_t      num_pages = 1u;

    LFS_DEBUG("Requested to erase flash page %u", page_to_erase);

    /* Checks if block is within the borders */
    if (false == pages_within_borders(page_to_erase, num_pages))
    {
        LFS_ERROR("Page %u does not belong to LFS region", page_to_erase);
        return LFS_ERR_INVAL;
    }

    if (osKernelGetState() == osKernelRunning)
    {
        /* RTOS is running, we can use Flash Manager module for async operations and to avoid conflicts with 2.4GHz radio operations */
        LFS_DEBUG("Erasing flash page %u with FM", page_to_erase);
        do
        {
            fm_err = FM_Erase(page_to_erase, 1u, &lfs_fm_callback_node);
            if (FM_ERROR == fm_err)
            {
                retCode = LFS_ERR_IO;
                LFS_ERROR("Failed to initiate flash erase");
                break;
            }

            os_status = osSemaphoreAcquire(lfs_fm_semaphore, osWaitForever);
            if (os_status != osOK)
            {
                retCode = LFS_ERR_IO;
                LFS_ERROR("Failed to wait for flash erase completion. Error %d", (int32_t)os_status);
                break;
            }
        } while (last_fm_status != FM_OPERATION_COMPLETE);
    }
    else
    {
        /* RTOS not running, fall back to direct calls to Flash Driver */
        LFS_DEBUG("Erasing flash page %u with FD", page_to_erase);

        (void)HAL_FLASH_Unlock();

        fd_status = FD_EraseSectors(page_to_erase);
        if (fd_status != FD_FLASHOP_SUCCESS)
        {
            retCode = LFS_ERR_IO;
        }

        (void)HAL_FLASH_Lock();
    }

    if (LFS_ERR_OK == retCode)
    {
        LFS_DEBUG("Flash page %u erased", page_to_erase);
    }
    else
    {
        LFS_ERROR("Flash erase failed on page %u", page_to_erase);
    }

    return retCode;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static int lfs_port_sync(const struct lfs_config * c)
{
    int retCode = LFS_ERR_OK;
#ifdef HAL_ICACHE_MODULE_ENABLED
    HAL_StatusTypeDef hal_err;

    hal_err = HAL_ICACHE_Invalidate();
    if(hal_err != HAL_OK)
    {
        retCode = LFS_ERR_IO;
        LFS_ERROR("Cache invalidation error 0x%02X", hal_err);
    }
#endif /* HAL_ICACHE_MODULE_ENABLED */

    return retCode;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static void populate_config(struct lfs_config * p_cfg)
{
    p_cfg->read = lfs_port_read;
    p_cfg->prog = lfs_port_prog;
    p_cfg->erase = lfs_port_erase;
    p_cfg->sync = lfs_port_sync;

#ifdef LFS_THREADSAFE
    p_cfg->lock = &lfs_port_lock;
    p_cfg->lock = &lfs_port_unlock;
#endif

    p_cfg->read_size = 1;
    p_cfg->prog_size = 16;
    p_cfg->block_size = FLASH_PAGE_SIZE;

    p_cfg->block_count = LFS_CONFIG_BLOCK_COUNT;
    p_cfg->block_cycles = 1000;

    p_cfg->cache_size = LFS_CONFIG_CACHE_SIZE;
    p_cfg->lookahead_size = LFS_CONFIG_LOOKAHEAD_SIZE;

#ifdef LFS_NO_MALLOC
    p_cfg->read_buffer = ucReadBuffer;
    p_cfg->prog_buffer = ucProgBuffer;
    p_cfg->lookahead_buffer = ucLookAheadBuffer;
#else
    p_cfg->read_buffer = NULL;
    p_cfg->prog_buffer = NULL;
    p_cfg->lookahead_buffer = NULL;
#endif

    /* Accept default maximums for now */
    p_cfg->name_max = 0;
    p_cfg->file_max = 0;
    p_cfg->attr_max = 0;
    p_cfg->metadata_max = 0;
}

/* Global function definitions -----------------------------------------------*/

#ifdef LFS_NO_MALLOC
SID_STM32_SPEED_OPTIMIZED const struct lfs_config * initialize_internal_flash_fs_static(void)
{
    // FIXME: check for errors
    lfs_fm_callback_node.Callback = lfs_fm_callback;
    lfs_fm_semaphore = osSemaphoreNew(1u, 0u, &lfs_fm_semaphore_attributes);
    populate_config( &p_lfs_cfg );
    return &p_lfs_cfg;
}
#else /* ifdef LFS_NO_MALLOC */
SID_STM32_SPEED_OPTIMIZED const struct lfs_config * initialize_internal_flash_fs( void )
{
    return NULL;
}
#endif /* LFS_NO_MALLOC */

/*----------------------------------------------------------------------------*/

#ifdef LFS_CONFIG /* Overrides for lfs_util.c when using custom LFS config */
#  ifndef LFS_CRC /* If no alternative lfs_crc implementation is defined use this one */
//FIXME: use WBAxx's CRC peripheral for this
SID_STM32_SPEED_OPTIMIZED uint32_t lfs_crc(uint32_t crc, const void *buffer, size_t size)
{
    static const uint32_t rtable[16] = {
        0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
        0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
        0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
        0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c,
    };

    const uint8_t *data = buffer;

    for (size_t i = 0; i < size; i++)
    {
        crc = (crc >> 4) ^ rtable[(crc ^ (data[i] >> 0)) & 0xf];
        crc = (crc >> 4) ^ rtable[(crc ^ (data[i] >> 4)) & 0xf];
    }

    return crc;
}
#  endif /* LFS_CRC */
#endif /* LFS_CONFIG */
