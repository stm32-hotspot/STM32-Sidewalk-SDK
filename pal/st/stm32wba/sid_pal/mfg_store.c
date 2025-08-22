/**
  ******************************************************************************
  * @file    mfg_store.c
  * @brief   sid_pal_mfg_store library implementation for STM32WBAxx MCUs
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
#include <sid_pal_mfg_store_ifc.h>

#include "stm32wbaxx_hal.h"
#include "stm32wbaxx_ll_system.h"

#include "mfg_store_offsets.h"
#include "sid_stm32_common_utils.h"

/* Use private Sidewalk SDK config for builds from SDK source code */
#if defined(USE_SID_PRECOMPILED_LIBS) && (USE_SID_PRECOMPILED_LIBS == 0)
#  include <sid_sdk_internal_config.h>
#endif

#include <sid_pal_assert_ifc.h>
#include <sid_pal_log_ifc.h>
#include <sid_endian.h>

#include <assert.h>
#include <stdalign.h>
#include <stdint.h>
#include <string.h>

/* Private defines -----------------------------------------------------------*/
/* Manufacturing store write capability is not enabled by default. It
 * is currently required for internal diagnostic apps and for SWAT
 */
#if (defined (HALO_ENABLE_DIAGNOSTICS) && HALO_ENABLE_DIAGNOSTICS) || defined(SWAT_DEVICE_TYPE)
#  define ENABLE_MFG_STORE_WRITE
#endif

#ifdef ENABLE_MFG_STORE_WRITE
#  warning "You are building Sidewalk MFG Storage code with flash write capabilities. Normally this functionality shall be excluded from builds for security reasons. Ignore this warning if write capability is enabled intentionally"
#endif /* ENABLE_MFG_STORE_WRITE */

#define MFG_VERSION_1_VAL                           (0x01000000u)
#define MFG_VERSION_2_VAL                           (0x02u)

#define ENCODED_DEV_ID_SIZE_5_BYTES_MASK            (0xA0u)
#define DEV_ID_MSB_MASK                             (0x1Fu)

#define MFG_STORE_TLV_HEADER_SIZE                   (4u)
#define MFG_STORE_TLV_TAG_EMPTY                     (0xFFFFu)

#define MFG_STORE_DEVID_V1_SIZE                     (8u)

#define MFG_STORE_MAGIC_WORD_SIZE                   (4u)

#define SID_MFG_STORE_SUPPORT_FIXED_OFFSETS_DEFAULT (1)

#if defined(USE_SID_PRECOMPILED_LIBS) && (USE_SID_PRECOMPILED_LIBS == 0) && defined(SID_MFG_STORE_SUPPORT_FIXED_OFFSETS) && (SID_MFG_STORE_SUPPORT_FIXED_OFFSETS != SID_MFG_STORE_SUPPORT_FIXED_OFFSETS_DEFAULT)
#  warning "The specified SID_MFG_STORE_SUPPORT_FIXED_OFFSETS value differs from the default one for the builds from static libs. Ignore this warning if this is intentional, otherwise make sure SID_MFG_STORE_SUPPORT_FIXED_OFFSETS_DEFAULT and SID_MFG_STORE_SUPPORT_FIXED_OFFSETS in sid_sdk_internal_config.h are aligned"
#endif

#ifndef SID_MFG_STORE_SUPPORT_FIXED_OFFSETS
#  define SID_MFG_STORE_SUPPORT_FIXED_OFFSETS       (SID_MFG_STORE_SUPPORT_FIXED_OFFSETS_DEFAULT)
#endif /* SID_MFG_STORE_SUPPORT_FIXED_OFFSETS */

/* Private typedef -----------------------------------------------------------*/
struct sid_pal_mfg_store_tlv_info {
    uint16_t tag;
    uint16_t length;
    /** TLV offset from mfg_store_region.addr_start in bytes */
    size_t offset;
};

struct sid_pal_mfg_store_value_to_address_offset {
    sid_pal_mfg_store_value_t value;
    uint16_t size;
    uint32_t offset;
};

/* Private function prototypes -----------------------------------------------*/
static uint32_t default_app_value_to_offset(int value);
static void *checked_addr_return(const int32_t offset, const uintptr_t start_address, const uintptr_t end_address);
static bool is_valid_value_offset(const uint32_t offset);
static void *value_to_address(const sid_pal_mfg_store_value_t value, const uintptr_t start_address, const uintptr_t end_address);
static uint16_t value_to_size(const sid_pal_mfg_store_value_t value);
static bool sid_pal_mfg_store_search_for_tag(const uint16_t tag, struct sid_pal_mfg_store_tlv_info * const tlv_info);
#ifdef ENABLE_MFG_STORE_WRITE
static bool write_to_flash(const void * const dest_address, void * const src_address, const size_t length);
#endif

/* Private variables ---------------------------------------------------------*/
static bool mfg_store_init_done = false;

static sid_pal_mfg_store_region_t stm_mfg_store_region = {
    .app_value_to_offset = default_app_value_to_offset,
};

/* Private constants ---------------------------------------------------------*/
static const struct sid_pal_mfg_store_value_to_address_offset sid_pal_mfg_store_app_value_to_offset_table[] = {
    {SID_PAL_MFG_STORE_VERSION,                      SID_PAL_MFG_STORE_VERSION_SIZE,                      SID_PAL_MFG_STORE_OFFSET_VERSION},
    {SID_PAL_MFG_STORE_DEVID,                        SID_PAL_MFG_STORE_DEVID_SIZE,                        SID_PAL_MFG_STORE_OFFSET_DEVID},
    {SID_PAL_MFG_STORE_SERIAL_NUM,                   SID_PAL_MFG_STORE_SERIAL_NUM_SIZE,                   SID_PAL_MFG_STORE_OFFSET_SERIAL_NUM},
    {SID_PAL_MFG_STORE_SMSN,                         SID_PAL_MFG_STORE_SMSN_SIZE,                         SID_PAL_MFG_STORE_OFFSET_SMSN},
    {SID_PAL_MFG_STORE_APID,                         SID_PAL_MFG_STORE_APID_SIZE,                         SID_PAL_MFG_STORE_OFFSET_APID},
    {SID_PAL_MFG_STORE_APP_PUB_ED25519,              SID_PAL_MFG_STORE_APP_PUB_ED25519_SIZE,              SID_PAL_MFG_STORE_OFFSET_APP_PUB_ED25519},
    {SID_PAL_MFG_STORE_DEVICE_PRIV_ED25519,          SID_PAL_MFG_STORE_DEVICE_PRIV_ED25519_SIZE,          SID_PAL_MFG_STORE_OFFSET_DEVICE_PRIV_ED25519},
    {SID_PAL_MFG_STORE_DEVICE_PUB_ED25519,           SID_PAL_MFG_STORE_DEVICE_PUB_ED25519_SIZE,           SID_PAL_MFG_STORE_OFFSET_DEVICE_PUB_ED25519},
    {SID_PAL_MFG_STORE_DEVICE_PUB_ED25519_SIGNATURE, SID_PAL_MFG_STORE_DEVICE_PUB_ED25519_SIGNATURE_SIZE, SID_PAL_MFG_STORE_OFFSET_DEVICE_PUB_ED25519_SIGNATURE},
    {SID_PAL_MFG_STORE_DEVICE_PRIV_P256R1,           SID_PAL_MFG_STORE_DEVICE_PRIV_P256R1_SIZE,           SID_PAL_MFG_STORE_OFFSET_DEVICE_PRIV_P256R1},
    {SID_PAL_MFG_STORE_DEVICE_PUB_P256R1,            SID_PAL_MFG_STORE_DEVICE_PUB_P256R1_SIZE,            SID_PAL_MFG_STORE_OFFSET_DEVICE_PUB_P256R1},
    {SID_PAL_MFG_STORE_DEVICE_PUB_P256R1_SIGNATURE,  SID_PAL_MFG_STORE_DEVICE_PUB_P256R1_SIGNATURE_SIZE,  SID_PAL_MFG_STORE_OFFSET_DEVICE_PUB_P256R1_SIGNATURE},
    {SID_PAL_MFG_STORE_DAK_PUB_ED25519,              SID_PAL_MFG_STORE_DAK_PUB_ED25519_SIZE,              SID_PAL_MFG_STORE_OFFSET_DAK_PUB_ED25519},
    {SID_PAL_MFG_STORE_DAK_PUB_ED25519_SIGNATURE,    SID_PAL_MFG_STORE_DAK_PUB_ED25519_SIGNATURE_SIZE,    SID_PAL_MFG_STORE_OFFSET_DAK_PUB_ED25519_SIGNATURE},
    {SID_PAL_MFG_STORE_DAK_ED25519_SERIAL,           SID_PAL_MFG_STORE_DAK_ED25519_SERIAL_SIZE,           SID_PAL_MFG_STORE_OFFSET_DAK_ED25519_SERIAL},
    {SID_PAL_MFG_STORE_DAK_PUB_P256R1,               SID_PAL_MFG_STORE_DAK_PUB_P256R1_SIZE,               SID_PAL_MFG_STORE_OFFSET_DAK_PUB_P256R1},
    {SID_PAL_MFG_STORE_DAK_PUB_P256R1_SIGNATURE,     SID_PAL_MFG_STORE_DAK_PUB_P256R1_SIGNATURE_SIZE,     SID_PAL_MFG_STORE_OFFSET_DAK_PUB_P256R1_SIGNATURE},
    {SID_PAL_MFG_STORE_DAK_P256R1_SERIAL,            SID_PAL_MFG_STORE_DAK_P256R1_SERIAL_SIZE,            SID_PAL_MFG_STORE_OFFSET_DAK_P256R1_SERIAL},
    {SID_PAL_MFG_STORE_PRODUCT_PUB_ED25519,          SID_PAL_MFG_STORE_PRODUCT_PUB_ED25519_SIZE,          SID_PAL_MFG_STORE_OFFSET_PRODUCT_PUB_ED25519},
    {SID_PAL_MFG_STORE_PRODUCT_PUB_ED25519_SIGNATURE,SID_PAL_MFG_STORE_PRODUCT_PUB_ED25519_SIGNATURE_SIZE,SID_PAL_MFG_STORE_OFFSET_PRODUCT_PUB_ED25519_SIGNATURE},
    {SID_PAL_MFG_STORE_PRODUCT_ED25519_SERIAL,       SID_PAL_MFG_STORE_PRODUCT_ED25519_SERIAL_SIZE,       SID_PAL_MFG_STORE_OFFSET_PRODUCT_ED25519_SERIAL},
    {SID_PAL_MFG_STORE_PRODUCT_PUB_P256R1,           SID_PAL_MFG_STORE_PRODUCT_PUB_P256R1_SIZE,           SID_PAL_MFG_STORE_OFFSET_PRODUCT_PUB_P256R1},
    {SID_PAL_MFG_STORE_PRODUCT_PUB_P256R1_SIGNATURE, SID_PAL_MFG_STORE_PRODUCT_PUB_P256R1_SIGNATURE_SIZE, SID_PAL_MFG_STORE_OFFSET_PRODUCT_PUB_P256R1_SIGNATURE},
    {SID_PAL_MFG_STORE_PRODUCT_P256R1_SERIAL,        SID_PAL_MFG_STORE_PRODUCT_P256R1_SERIAL_SIZE,        SID_PAL_MFG_STORE_OFFSET_PRODUCT_P256R1_SERIAL},
    {SID_PAL_MFG_STORE_MAN_PUB_ED25519,              SID_PAL_MFG_STORE_MAN_PUB_ED25519_SIZE,              SID_PAL_MFG_STORE_OFFSET_MAN_PUB_ED25519},
    {SID_PAL_MFG_STORE_MAN_PUB_ED25519_SIGNATURE,    SID_PAL_MFG_STORE_MAN_PUB_ED25519_SIGNATURE_SIZE,    SID_PAL_MFG_STORE_OFFSET_MAN_PUB_ED25519_SIGNATURE},
    {SID_PAL_MFG_STORE_MAN_ED25519_SERIAL,           SID_PAL_MFG_STORE_MAN_ED25519_SERIAL_SIZE,           SID_PAL_MFG_STORE_OFFSET_MAN_ED25519_SERIAL},
    {SID_PAL_MFG_STORE_MAN_PUB_P256R1,               SID_PAL_MFG_STORE_MAN_PUB_P256R1_SIZE,               SID_PAL_MFG_STORE_OFFSET_MAN_PUB_P256R1},
    {SID_PAL_MFG_STORE_MAN_PUB_P256R1_SIGNATURE,     SID_PAL_MFG_STORE_MAN_PUB_P256R1_SIGNATURE_SIZE,     SID_PAL_MFG_STORE_OFFSET_MAN_PUB_P256R1_SIGNATURE},
    {SID_PAL_MFG_STORE_MAN_P256R1_SERIAL,            SID_PAL_MFG_STORE_MAN_P256R1_SERIAL_SIZE,            SID_PAL_MFG_STORE_OFFSET_MAN_P256R1_SERIAL},
    {SID_PAL_MFG_STORE_SW_PUB_ED25519,               SID_PAL_MFG_STORE_SW_PUB_ED25519_SIZE,               SID_PAL_MFG_STORE_OFFSET_SW_PUB_ED25519},
    {SID_PAL_MFG_STORE_SW_PUB_ED25519_SIGNATURE,     SID_PAL_MFG_STORE_SW_PUB_ED25519_SIGNATURE_SIZE,     SID_PAL_MFG_STORE_OFFSET_SW_PUB_ED25519_SIGNATURE},
    {SID_PAL_MFG_STORE_SW_ED25519_SERIAL,            SID_PAL_MFG_STORE_SW_ED25519_SERIAL_SIZE,            SID_PAL_MFG_STORE_OFFSET_SW_ED25519_SERIAL},
    {SID_PAL_MFG_STORE_SW_PUB_P256R1,                SID_PAL_MFG_STORE_SW_PUB_P256R1_SIZE,                SID_PAL_MFG_STORE_OFFSET_SW_PUB_P256R1},
    {SID_PAL_MFG_STORE_SW_PUB_P256R1_SIGNATURE,      SID_PAL_MFG_STORE_SW_PUB_P256R1_SIGNATURE_SIZE,      SID_PAL_MFG_STORE_OFFSET_SW_PUB_P256R1_SIGNATURE},
    {SID_PAL_MFG_STORE_SW_P256R1_SERIAL,             SID_PAL_MFG_STORE_SW_P256R1_SERIAL_SIZE,             SID_PAL_MFG_STORE_OFFSET_SW_P256R1_SERIAL},
    {SID_PAL_MFG_STORE_AMZN_PUB_ED25519,             SID_PAL_MFG_STORE_AMZN_PUB_ED25519_SIZE,             SID_PAL_MFG_STORE_OFFSET_AMZN_PUB_ED25519},
    {SID_PAL_MFG_STORE_AMZN_PUB_P256R1,              SID_PAL_MFG_STORE_AMZN_PUB_P256R1_SIZE,              SID_PAL_MFG_STORE_OFFSET_AMZN_PUB_P256R1},
};

#if SID_PAL_SHORT_FORM_CERT_CHAIN_ENABLED
static const uint8_t product_apid[] = {0x76u, 0x43u, 0x74u, 0x32u};
static const uint8_t app_server_public_key[] = {0xB2u, 0x40u, 0xBFu, 0x98u, 0xC6u, 0x5Cu, 0xDFu, 0x84u,
                                                0xBFu, 0x2Au, 0xA1u, 0xACu, 0x29u, 0x11u, 0x14u, 0x1Fu,
                                                0xB4u, 0x80u, 0x7Cu, 0xBCu, 0xB6u, 0x6Eu, 0xCFu, 0x09u,
                                                0x1Cu, 0x20u, 0x04u, 0xB3u, 0x37u, 0xB4u, 0x06u, 0x47u
                                               };
#endif

/* Private function definitions ----------------------------------------------*/

static uint32_t default_app_value_to_offset(int value)
{
    SID_PAL_LOG_WARNING("No support for app_value_to_offset");
    return SID_PAL_MFG_STORE_INVALID_OFFSET;
}

/*----------------------------------------------------------------------------*/

static void *checked_addr_return(const int32_t offset, const uintptr_t start_address, const uintptr_t end_address)
{
    if ((start_address + offset) >= end_address)
    {
        SID_PAL_LOG_ERROR("Offset past manufacturing store end: %d", offset);
        return NULL;
    }
    return (void*)(start_address + offset);
}

/*----------------------------------------------------------------------------*/

static bool is_valid_value_offset(const uint32_t offset)
{
    return offset != SID_PAL_MFG_STORE_INVALID_OFFSET;
}

/*----------------------------------------------------------------------------*/

static void *value_to_address(const sid_pal_mfg_store_value_t value, const uintptr_t start_address, const uintptr_t end_address)
{
    const size_t table_count = SID_STM32_UTIL_ARRAY_SIZE(sid_pal_mfg_store_app_value_to_offset_table);

    for (size_t i = 0u; i < table_count; i++)
    {
        if (value == sid_pal_mfg_store_app_value_to_offset_table[i].value)
        {
            const uint32_t offset = sid_pal_mfg_store_app_value_to_offset_table[i].offset;
            return is_valid_value_offset(offset) ? (void *)(start_address + (offset * MFG_WORD_SIZE)) : NULL;
        }
    }

    if (value >= SID_PAL_MFG_STORE_CORE_VALUE_MAX)
    {
        /* This is not a core value. Search for this value among those provided by the application. */
        uint32_t custom_offset = stm_mfg_store_region.app_value_to_offset(value);
        if (is_valid_value_offset(custom_offset))
        {
            return checked_addr_return(custom_offset, start_address, end_address);
        }
    }

    SID_PAL_LOG_ERROR("No manufacturing store offset for: %d", value);
    return NULL;
}

/*----------------------------------------------------------------------------*/

static uint16_t value_to_size(const sid_pal_mfg_store_value_t value)
{
    const size_t table_count = SID_STM32_UTIL_ARRAY_SIZE(sid_pal_mfg_store_app_value_to_offset_table);

    for (size_t i = 0u; i < table_count; i++)
    {
        if (value == sid_pal_mfg_store_app_value_to_offset_table[i].value)
        {
            return is_valid_value_offset(sid_pal_mfg_store_app_value_to_offset_table[i].offset) ?
                sid_pal_mfg_store_app_value_to_offset_table[i].size : 0u;
        }
    }

    /* NOTE: Getting size for App values >= SID_PAL_MFG_STORE_CORE_VALUE_MAX is not supported */
    return 0u;
}

/*----------------------------------------------------------------------------*/

static bool sid_pal_mfg_store_search_for_tag(const uint16_t tag, struct sid_pal_mfg_store_tlv_info * const tlv_info)
{
    const uint8_t *address = (uint8_t *)MFG_ALIGN_TO_WORD_BOUNDARY(stm_mfg_store_region.addr_start +
        (SID_PAL_MFG_STORE_OFFSET_VERSION * MFG_WORD_SIZE) +
        SID_PAL_MFG_STORE_VERSION_SIZE);

    while ((uintptr_t)(address + (MFG_STORE_TLV_HEADER_SIZE + MFG_WORD_SIZE)) <= stm_mfg_store_region.addr_end) /* while there's a space for at least one record */
    {
        /* Parse TLV Tag ID and TLV Data Length */
        const uint16_t current_tag = address[0] << 8 | address[1]; /* Stored in Big Endian on flash */
        const uint16_t length = address[2] << 8 | address[3]; /* Stored in Big Endian on flash */

        if (current_tag == tag)
        {
            /* Desired tag found */
            tlv_info->tag = tag;
            tlv_info->length = length;
            tlv_info->offset = (size_t)(address - stm_mfg_store_region.addr_start);
            return true;
        }
        else
        {
            if (MFG_STORE_TLV_TAG_EMPTY == current_tag)
            {
                /* Last TLV record was processed */
                break;
            }
            /*
             * Go to the next TLV.
             * Since data is written to flash with data aligned to 4, we must take this
             * into account if the data length is not a multiple of 4.
             */
            address += MFG_ALIGN_TO_WORD_BOUNDARY(MFG_STORE_TLV_HEADER_SIZE + length);
        }
    }
    return false;
}

/*----------------------------------------------------------------------------*/

#ifdef ENABLE_MFG_STORE_WRITE
static bool write_to_flash(const void * const dest_address, void * const src_address, const size_t length)
{
    HAL_StatusTypeDef ret = HAL_OK;

    if ((length % MFG_WORD_SIZE) != 0u)
    {
        SID_PAL_LOG_ERROR("Invalid data length for flash write. It can only be written in multiples of %u bytes. Requested length: %u bytes", MFG_WORD_SIZE, length);
        return false;
    }

    ret = HAL_FLASH_Unlock();
    if (ret != HAL_OK)
    {
        SID_PAL_LOG_ERROR("MFG storage write failed - cannot unlock the flash. Error 0x%x", ret);
        return false;
    }

    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

    uint32_t write_address = (uint32_t)dest_address;
    uint32_t read_address = (uint32_t)src_address;
    size_t bytes_processed = 0u;

    /* Write data in quad-word chunks */
    while ((bytes_processed < length) && (HAL_OK == ret))
    {
        ret = HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, write_address, read_address);

        /* Move to the next quad-word */
        write_address += MFG_WORD_SIZE;
        read_address += MFG_WORD_SIZE;
        bytes_processed += MFG_WORD_SIZE;
    }

    ret = HAL_FLASH_Lock();
    if (ret != HAL_OK)
    {
        SID_PAL_LOG_ERROR("MFG storage write failed - cannot lock the flash. Error 0x%x", ret);
        return false;
    }

    /* Invalidate the cache to ensure writing is actually performed */
    ret = HAL_ICACHE_Invalidate();
    if(ret != HAL_OK)
    {
        SID_PAL_LOG_ERROR("MFG storage write failed - cache invalidation error 0x%x", ret);
        return false;
    }

    return true;
}
#endif

/* Global function definitions -----------------------------------------------*/

void sid_pal_mfg_store_init(sid_pal_mfg_store_region_t mfg_store_region)
{
    if (mfg_store_init_done != false)
    {
        return;
    }

    stm_mfg_store_region = mfg_store_region;

    if (NULL == stm_mfg_store_region.app_value_to_offset)
    {
        stm_mfg_store_region.app_value_to_offset = default_app_value_to_offset;
    }
}

/*----------------------------------------------------------------------------*/

void sid_pal_mfg_store_deinit(void)
{
    if (false == mfg_store_init_done)
    {
        return;
    }

    memset(&stm_mfg_store_region, 0u, sizeof(sid_pal_mfg_store_region_t));
    mfg_store_init_done = false;
}

/*----------------------------------------------------------------------------*/

int32_t sid_pal_mfg_store_write(uint16_t value, const uint8_t *buffer, uint16_t length)
{
#ifdef ENABLE_MFG_STORE_WRITE
    alignas(MFG_WORD_SIZE) uint8_t wr_array[SID_PAL_MFG_STORE_MAX_FLASH_WRITE_LEN];
    bool ret;

    if ((0u == length)
       || (MFG_STORE_TLV_TAG_EMPTY == value)
       || (NULL == buffer))
    {
        return -1;
    }

    /* The SID_PAL_MFG_STORE_VERSION should be written using fixed offset */
    if (SID_PAL_MFG_STORE_VERSION == value)
    {
        if (!sid_pal_mfg_store_is_empty())
        {
            SID_PAL_LOG_ERROR("MFG must be erased before writing version");
            return -1;
        }

#if !defined(SID_PAL_SHORT_FORM_CERT_CHAIN_ENABLED) || (SID_PAL_SHORT_FORM_CERT_CHAIN_ENABLED == 0)
        const void * const magic_word_address = (void *)(stm_mfg_store_region.addr_start +
            SID_PAL_MFG_STORE_OFFSET_MAGIC * MFG_WORD_SIZE);

        const size_t magic_word_write_length = MFG_ALIGN_TO_WORD_BOUNDARY(MFG_STORE_MAGIC_WORD_SIZE);

        memset(wr_array, 0xFFu, magic_word_write_length);
        wr_array[0] = 'S';
        wr_array[1] = 'I';
        wr_array[2] = 'D';
        wr_array[3] = '0';

        ret = write_to_flash(magic_word_address, wr_array, magic_word_write_length);
        if (false == ret)
        {
            return -1;
        }
#endif /* SID_PAL_SHORT_FORM_CERT_CHAIN_ENABLED */

        const void * const version_address = (void *)(stm_mfg_store_region.addr_start +
            SID_PAL_MFG_STORE_OFFSET_VERSION * MFG_WORD_SIZE);

        const size_t write_length = MFG_ALIGN_TO_WORD_BOUNDARY(length);
        if (write_length != length)
        {
            /* Prefill the buffer with padding if data alignment will take place */
            memset(wr_array, 0xFFu, write_length);
        }
        memcpy(wr_array, buffer, length);
        ret = write_to_flash(version_address, wr_array, write_length);

        return ret ? 0 : -1;
    }

    if (sid_pal_mfg_store_get_version() == SID_PAL_MFG_STORE_TLV_VERSION)
    {
        struct sid_pal_mfg_store_tlv_info tlv_info = {};

        if (sid_pal_mfg_store_search_for_tag(value, &tlv_info))
        {
            SID_PAL_LOG_ERROR("The tag value %u already exists. We can't write duplicate", value);
            return -1;
        }

        /* Search for the end of data */
        if (!sid_pal_mfg_store_search_for_tag(MFG_STORE_TLV_TAG_EMPTY, &tlv_info))
        {
            SID_PAL_LOG_ERROR("Can't write tag %u. MFG storage is full", value);
            return -1;
        }

        /* The length should be a multiple of the program unit */
        const size_t full_length = MFG_ALIGN_TO_WORD_BOUNDARY(length + MFG_STORE_TLV_HEADER_SIZE);
        uintptr_t address = stm_mfg_store_region.addr_start + tlv_info.offset;

        /* Check the remaining storage size */
        if ((address + full_length) > stm_mfg_store_region.addr_end)
        {
            SID_PAL_LOG_ERROR("Not enough space to store: %u", value);
            return -1;
        }

        /* Clean up flash write buffer */
        memset(wr_array, 0xFFu, sizeof(wr_array));

        /* Construct TLV header */
        wr_array[0] = (uint8_t)(value >> 8); /* Tag ID in Big Endian */
        wr_array[1] = (uint8_t)value;
        wr_array[2] = (uint8_t)(length >> 8); /* Value length in Big Endian */
        wr_array[3] = (uint8_t)(length);
        
        /* Add as many data as fits into the remaining part of the write buffer */
        const size_t firstr_copy_length = full_length > sizeof(wr_array) ? (sizeof(wr_array) - MFG_STORE_TLV_HEADER_SIZE) : (full_length - MFG_STORE_TLV_HEADER_SIZE);
        memcpy(&wr_array[MFG_STORE_TLV_HEADER_SIZE], buffer, firstr_copy_length);

        /* Write the first chunk containing TLV header */
        size_t write_length = firstr_copy_length + MFG_STORE_TLV_HEADER_SIZE;
        uintptr_t write_address = address;
        size_t bytes_processed = 0u;
        bool ret = write_to_flash((void*)write_address, wr_array, write_length);

        /* Move to the next chunk */
        bytes_processed += write_length;
        write_address += write_length;

        /* Write any remaining chunks until entire tag is processed */
        while ((bytes_processed < full_length) && (ret != false))
        {
            write_length = (full_length - bytes_processed) > sizeof(wr_array) ? sizeof(wr_array) : (full_length - bytes_processed);
            memset(wr_array, 0xFFu, sizeof(wr_array));
            memcpy(wr_array, &buffer[bytes_processed - MFG_STORE_TLV_HEADER_SIZE], write_length);
            ret = write_to_flash((void*)write_address, wr_array, write_length);

            /* Move to the next chunk */
            bytes_processed += write_length;
            write_address += write_length;
        }

        return ret ? 0 : -1;

    }
    else
    {
#if SID_MFG_STORE_SUPPORT_FIXED_OFFSETS
        if (length > sizeof(wr_array))
        {
            return -1;
        }

        const void * const value_address = value_to_address(
            value, stm_mfg_store_region.addr_start, stm_mfg_store_region.addr_end);

        if (value_address == NULL)
        {
            SID_PAL_LOG_ERROR("Unable to determine MFG value offset for writing. Value ID: %u", value);
            return -1;
        }

        const size_t write_length = MFG_ALIGN_TO_WORD_BOUNDARY(length);
        if (write_length != length)
        {
            /* Prefill the buffer with padding if data alignment will take place */
            memset(wr_array, 0xFFu, write_length);
        }
        memcpy(wr_array, buffer, length);
        ret = write_to_flash(value_address, wr_array, write_length);

        return ret ? 0 : -1;
#else
        SID_PAL_ASSERT(false);
        return -1;
#endif
    }
#else
    return -1;
#endif
}

/*----------------------------------------------------------------------------*/

void sid_pal_mfg_store_read(uint16_t value, uint8_t *buffer, uint16_t length)
{
    if (sid_pal_mfg_store_get_version() == SID_PAL_MFG_STORE_TLV_VERSION)
    {
        /* The SID_PAL_MFG_STORE_VERSION we should read as fixed offset */
        if (SID_PAL_MFG_STORE_VERSION == value)
        {
            const void * const version_address = (void *)(stm_mfg_store_region.addr_start +
                SID_PAL_MFG_STORE_OFFSET_VERSION * MFG_WORD_SIZE);

            memcpy(buffer, version_address, length);
            return;
        }

        struct sid_pal_mfg_store_tlv_info tlv_info;
        if (sid_pal_mfg_store_search_for_tag(value, &tlv_info))
        {
            const size_t copy_length = tlv_info.length >= length ? length : tlv_info.length; /* Limit readout length to a single record */
            if (copy_length < length)
            {
                SID_PAL_LOG_WARNING("Invalid read length requested while reading MFG value %u. Record length: %u, requested length: %u", value, copy_length, length);
            }
            memcpy(buffer,
                (void *)(stm_mfg_store_region.addr_start + tlv_info.offset + MFG_STORE_TLV_HEADER_SIZE),
                copy_length);
        }
        else
        {
            /*
             * For backwards compatibility with MFG version with fixed offsets,
             * we must fill the buffer with empty data.
             */
            memset(buffer, 0xFFu, length);
        }
    }
    else
    {
#if SID_MFG_STORE_SUPPORT_FIXED_OFFSETS
        const void * const value_address = value_to_address(value, stm_mfg_store_region.addr_start, stm_mfg_store_region.addr_end);

        if (value_address != NULL)
        {
            memcpy(buffer, value_address, length);
        }
#else
        SID_PAL_ASSERT(false);
#endif
    }
}

/*----------------------------------------------------------------------------*/

uint16_t sid_pal_mfg_store_get_length_for_value(uint16_t value)
{
    uint16_t length = 0u;
    if (sid_pal_mfg_store_get_version() == SID_PAL_MFG_STORE_TLV_VERSION)
    {
        struct sid_pal_mfg_store_tlv_info tlv_info;
        if (sid_pal_mfg_store_search_for_tag(value, &tlv_info))
        {
            length = tlv_info.length;
        }
    } else {
#if SID_MFG_STORE_SUPPORT_FIXED_OFFSETS
    length = value_to_size(value);
#else
    SID_PAL_ASSERT(false);
#endif
    }
    return length;
}

/*----------------------------------------------------------------------------*/

int32_t sid_pal_mfg_store_erase(void)
{
#ifdef ENABLE_MFG_STORE_WRITE
    uint32_t ulPageError = 0;
    FLASH_EraseInitTypeDef erase_Config = { 0 };
    const uint32_t mfg_size = stm_mfg_store_region.addr_end - stm_mfg_store_region.addr_start;

    if ((mfg_size % FLASH_PAGE_SIZE != 0u) || ((stm_mfg_store_region.addr_start & (FLASH_PAGE_SIZE - 1u)) != 0u))
    {
        /**
         * NOTE: Erase is only supported on a per-page basis. If the user configures the manufacturing
         * store so that it partially uses a page, that ENTIRE page will be erased here.
         */
        SID_PAL_LOG_WARNING("Erasing entire contents of all pages which contain manufacturing store data");
    }

    erase_Config.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_Config.Page = ((stm_mfg_store_region.addr_start - FLASH_BASE_NS) / FLASH_PAGE_SIZE );
    erase_Config.NbPages = (mfg_size + (FLASH_PAGE_SIZE - 1u)) / FLASH_PAGE_SIZE; /* Round up to the nearest whole number of pages */

    SID_PAL_LOG_DEBUG("Manufacturing data: erasing flash. Starting page: %u, pages to erase: %u\r\n", erase_Config.Page, erase_Config.NbPages);
    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG( FLASH_FLAG_ALL_ERRORS );
    HAL_StatusTypeDef hal_status = HAL_FLASHEx_Erase( &erase_Config, &ulPageError );

    HAL_FLASH_Lock();

    return hal_status == HAL_OK ? 0 : -1;
#else
    return -1;
#endif
}

/*----------------------------------------------------------------------------*/

#ifdef ENABLE_MFG_STORE_WRITE
bool sid_pal_mfg_store_is_empty(void)
{
    for (uint32_t addr = stm_mfg_store_region.addr_start; addr < stm_mfg_store_region.addr_end; ++addr)
    {
        static const unsigned char EMPTY_BYTE = 0xFFu;
        const unsigned char * const p = (const unsigned char * const)addr;
        if (*p != EMPTY_BYTE)
        {
            return false;
        }
    }

    return true;
}
#endif

/*----------------------------------------------------------------------------*/

bool sid_pal_mfg_store_is_tlv_support(void)
{
    return true;
}

/*----------------------------------------------------------------------------*/

uint32_t sid_pal_mfg_store_get_version(void)
{
    uint32_t version = 0;
    const void * const version_address = (void *)(stm_mfg_store_region.addr_start +
        SID_PAL_MFG_STORE_OFFSET_VERSION * MFG_WORD_SIZE);

    memcpy((uint8_t *)&version, version_address, SID_PAL_MFG_STORE_VERSION_SIZE);
    /* Assuming that we keep this behavior for both 1P & 3P */
    return sid_ntohl(version);
}

/*----------------------------------------------------------------------------*/

bool sid_pal_mfg_store_dev_id_get(uint8_t dev_id[SID_PAL_MFG_STORE_DEVID_SIZE])
{
    bool error_code = false;
    uint8_t buffer[] = {0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu};

    static_assert(sizeof(buffer) == SID_PAL_MFG_STORE_DEVID_SIZE, "dev ID buffer wrong size");

    sid_pal_mfg_store_read(SID_PAL_MFG_STORE_DEVID,
                            buffer, SID_PAL_MFG_STORE_DEVID_SIZE);

    static const uint8_t UNSET_DEV_ID[] = {0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu};
    static_assert(sizeof(UNSET_DEV_ID) == SID_PAL_MFG_STORE_DEVID_SIZE, "Unset dev ID wrong size");
    if (memcmp(buffer, UNSET_DEV_ID, SID_PAL_MFG_STORE_DEVID_SIZE) == 0)
    {
        const uint32_t udn = LL_FLASH_GetUDN(); /* The 32-bit unique device number is a sequential number, different for each individual device. */

        buffer[0] = 0xBFu;
        buffer[1] = 0xFFu;
        buffer[2] = (uint8_t)((udn >> 16) & 0xFFu);
        buffer[3] = (uint8_t)((udn >> 8) & 0xFFu);
        buffer[4] = (uint8_t)(udn & 0xFFu);
    }
    else
    {
        const uint32_t version = sid_pal_mfg_store_get_version();
        if ((MFG_VERSION_1_VAL == version) || (0x1u == version))
        {
            /**
             * Correct dev_id for mfg version 1
             * For devices with mfg version 1, the device Id is stored as two words
             * in network endian format.
             * To read the device Id two words at SID_PAL_MFG_STORE_DEVID has to be
             * read and each word needs to be changed to host endian format.
             */
            uint8_t dev_id_buffer[MFG_STORE_DEVID_V1_SIZE];
            uint32_t val = 0u;
            sid_pal_mfg_store_read(SID_PAL_MFG_STORE_DEVID, dev_id_buffer, MFG_STORE_DEVID_V1_SIZE);
            memcpy(&val, &dev_id_buffer[0], sizeof(val));
            val = sid_ntohl(val);
            memcpy(&dev_id_buffer[0], &val, sizeof(val));
            memcpy(&val, &dev_id_buffer[sizeof(val)], sizeof(val));
            val = sid_ntohl(val);
            memcpy(&dev_id_buffer[sizeof(val)], &val, sizeof(val));
            /* Encode the size in the first 3 bits in MSB of the devId */
            dev_id_buffer[0] = (dev_id_buffer[0] & DEV_ID_MSB_MASK) | ENCODED_DEV_ID_SIZE_5_BYTES_MASK;
            memcpy(buffer, dev_id_buffer, SID_PAL_MFG_STORE_DEVID_SIZE);
        }
        error_code = true;
    }

    memcpy(dev_id, buffer, SID_PAL_MFG_STORE_DEVID_SIZE);
    return error_code;
}

/*----------------------------------------------------------------------------*/

bool sid_pal_mfg_store_serial_num_get(uint8_t serial_num[SID_PAL_MFG_STORE_SERIAL_NUM_SIZE])
{
    uint32_t buffer[((SID_PAL_MFG_STORE_SERIAL_NUM_SIZE + (MFG_WORD_SIZE - 1u)) / MFG_WORD_SIZE) * ((MFG_WORD_SIZE + (sizeof(uint32_t) - 1u)) / sizeof(uint32_t))];

    sid_pal_mfg_store_read(SID_PAL_MFG_STORE_SERIAL_NUM,
                            (uint8_t*)buffer, SID_PAL_MFG_STORE_SERIAL_NUM_SIZE);

    static const uint8_t UNSET_SERIAL_NUM[] = {0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu,
                                               0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu};
    static_assert(sizeof(UNSET_SERIAL_NUM) == SID_PAL_MFG_STORE_SERIAL_NUM_SIZE, "Unset serial num wrong size");
    if(memcmp(buffer, UNSET_SERIAL_NUM, SID_PAL_MFG_STORE_SERIAL_NUM_SIZE) == 0)
    {
        return false;
    }

    const uint32_t version = sid_pal_mfg_store_get_version();

    if ((MFG_VERSION_1_VAL == version) || (0x1u == version))
    {
        for (size_t i = 0u; i < SID_STM32_UTIL_ARRAY_SIZE(buffer); ++i)
        {
            buffer[i] = sid_ntohl(buffer[i]);
        }
    }

    memcpy(serial_num, buffer, SID_PAL_MFG_STORE_SERIAL_NUM_SIZE);
    return true;
}

/*----------------------------------------------------------------------------*/

#if SID_PAL_SHORT_FORM_CERT_CHAIN_ENABLED
void sid_pal_mfg_store_apid_get(uint8_t apid[SID_PAL_MFG_STORE_APID_SIZE])
{
    memcpy(apid, product_apid, sizeof(product_apid));

}
/*----------------------------------------------------------------------------*/

void sid_pal_mfg_store_app_pub_key_get(uint8_t app_pub[SID_PAL_MFG_STORE_APP_PUB_ED25519_SIZE])
{
    memcpy(app_pub, app_server_public_key, sizeof(app_server_public_key));
}
#endif /* SID_PAL_SHORT_FORM_CERT_CHAIN_ENABLED */

