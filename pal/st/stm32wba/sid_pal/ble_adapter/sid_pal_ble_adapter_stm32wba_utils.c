/**
  ******************************************************************************
  * @file    sid_pal_ble_adapter_stm32wba_utils.c
  * @brief   Private utility functions and helpers of the Sidewalk BLE driver
  * 
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

#include <stdio.h>

#include "sid_pal_ble_adapter_stm32wba_utils.h"
#include "bluetooth_hci_defs.h"

/* Sidewalk interfaces */
#include <sid_pal_assert_ifc.h>
#include <sid_pal_critical_region_ifc.h>
#include <sid_pal_crypto_ifc.h>
#include <sid_pal_log_ifc.h>
#include <sid_pal_storage_kv_ifc.h>

/* Platform-specific headers */
#include <hw.h>
#include <hw_if.h>
#include <otp.h>
#include <stm32wbaxx_hal.h>

/* Utilities and helpers */
#include <sid_stm32_common_utils.h>

/* Private defines -----------------------------------------------------------*/

#define SID_BLE_ADAPTER_UTILS_PADDED_PRAND_LEN         (16u)

/* Private macro -------------------------------------------------------------*/

#ifndef SID_BLE_ADAPTER_UTILS_EXTRA_LOGGING
/* Set SID_BLE_ADAPTER_UTILS_EXTRA_LOGGING to 1 to enable extended logs */
#  define SID_BLE_ADAPTER_UTILS_EXTRA_LOGGING          (0)
#endif

#if SID_BLE_ADAPTER_UTILS_EXTRA_LOGGING
#  define SID_BLE_ADAPTER_UTILS_LOG_ERROR(...)         SID_PAL_LOG_ERROR(__VA_ARGS__)
#  define SID_BLE_ADAPTER_UTILS_LOG_WARNING(...)       SID_PAL_LOG_WARNING(__VA_ARGS__)
#  define SID_BLE_ADAPTER_UTILS_LOG_INFO(...)          SID_PAL_LOG_INFO(__VA_ARGS__)
#  define SID_BLE_ADAPTER_UTILS_LOG_DEBUG(...)         SID_PAL_LOG_DEBUG(__VA_ARGS__)
#  define SID_BLE_ADAPTER_UTILS_LOG_TRACE(...)         SID_PAL_LOG_TRACE(__VA_ARGS__)
#else
#  define SID_BLE_ADAPTER_UTILS_LOG_ERROR(...)         ((void)0u)
#  define SID_BLE_ADAPTER_UTILS_LOG_WARNING(...)       ((void)0u)
#  define SID_BLE_ADAPTER_UTILS_LOG_INFO(...)          ((void)0u)
#  define SID_BLE_ADAPTER_UTILS_LOG_DEBUG(...)         ((void)0u)
#  define SID_BLE_ADAPTER_UTILS_LOG_TRACE(...)         ((void)0u)
#endif

/* External variables --------------------------------------------------------*/

extern sid_pal_ble_adapter_ctx_t sid_ble_drv_ctx;

/* Imported function prototypes ----------------------------------------------*/

extern void SMA_Generate_IRK(uint8_t irk[16]);

/* Private function prototypes -----------------------------------------------*/

static inline sid_error_t _verify_random_mac_address_random_part(sid_pal_ble_prv_bt_addr_buffer_t * const buffer);
static inline sid_error_t _generate_generic_random_mac_address(sid_pal_ble_prv_bt_addr_buffer_t * const buffer);
static inline sid_error_t _generate_random_key(uint32_t * const buffer, const uint32_t words_len);

/* Private function definitions ----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline sid_error_t _verify_random_mac_address_random_part(sid_pal_ble_prv_bt_addr_buffer_t * const buffer)
{
    sid_error_t err;
    uint32_t    rnd_part;

    do
    {
        /* At least one bit in random part shall be zero */
        if ((buffer->words[0] ^ 0xFFFFu) == 0u)
        {
            rnd_part = buffer->words[1] | 0xC0FFu; /* Filter out two MSB as they do not belong to the random part of address and 16 LSB as they are not used */
            if ((rnd_part ^ 0xFFFFu) == 0u)
            {
                /* All random bits are 1, address is invalid */
                err = SID_ERROR_PARAM_OUT_OF_RANGE;
                break;
            }
        }

        /* At least one bit in random part shall be 1 */
        if ((buffer->words[0] ^ 0x0000u) == 0u)
        {
            rnd_part = buffer->words[1] & 0x4F00u; /* Filter out two MSB as they do not belong to the random part of address and 16 LSB as they are not used */
            if ((rnd_part ^ 0x0000u) == 0u)
            {
                /* All random bits are 0, address is invalid */
                err = SID_ERROR_PARAM_OUT_OF_RANGE;
                break;
            }
        }

        err = SID_ERROR_NONE;
    } while(0);

#if defined(CFG_STATIC_RANDOM_ADDRESS)
    /* Prevent infinite loop if invalid static address override is provided in configuration */
    if (err != SID_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("CFG_STATIC_RANDOM_ADDRESS is invalid. Enforced static address does not comply with BLE specification section 1.3.2 and cannot be used");
        SID_PAL_ASSERT(0);
    }
#endif

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline sid_error_t _generate_generic_random_mac_address(sid_pal_ble_prv_bt_addr_buffer_t * const buffer)
{
    sid_error_t status = SID_ERROR_GENERIC;

#ifdef CFG_STATIC_RANDOM_ADDRESS
    buffer->words[0] = CFG_STATIC_RANDOM_ADDRESS & 0xFFFFFFFFu;
    buffer->words[1] = (uint32_t)((uint64_t)CFG_STATIC_RANDOM_ADDRESS >> 32);
    status = SID_ERROR_NONE;
#else
    status = sid_pal_crypto_rand(buffer->bytes, sizeof(buffer));
#endif /* CFG_STATIC_RANDOM_ADDRESS */

    return status;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline sid_error_t _generate_random_key(uint32_t * const buffer, const uint32_t words_len)
{
    sid_error_t status = SID_ERROR_GENERIC;
    uint32_t zero_check;
    uint32_t one_check;

    do
    {
        if ((NULL == buffer) || (0u == words_len))
        {
            status = SID_ERROR_INVALID_ARGS;
            break;
        }

        /* Generate random data */
        status = sid_pal_crypto_rand((void *)buffer, words_len * sizeof(uint32_t));
        if (status != SID_ERROR_NONE)
        {
            SID_BLE_ADAPTER_UTILS_LOG_ERROR("  Fail   : Unable to generate random key data. sid_pal_crypto_rand failed, error: %d", (int32_t)status);
            break;
        }

        /* Check the generated random data is not all zeros or all ones */
        zero_check = 0u;
        one_check = 0u;
        for (uint32_t i = 0u; i < words_len; i++)
        {
            zero_check |= buffer[i];
            one_check |= (buffer[i] ^ UINT32_MAX);
        }

        status = SID_ERROR_NONE;
    } while ((0u == zero_check) || (0u == one_check));

    return status;
}

/* Global function definitions -----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_stm32wba_ble_adapter_util_sprintf_uuid(char * const buf, const sid_ble_cfg_uuid_info_t * const uuid)
{
    int32_t pos = 0;

    if (UUID_TYPE_16 == uuid->type)
    {
        pos = sprintf(buf, "%02x%02x", uuid->uu[0], uuid->uu[1]);
    }
    else if (UUID_TYPE_32 == uuid->type)
    {
        pos = sprintf(buf, "%02x%02x%02x%02x", uuid->uu[0], uuid->uu[1], uuid->uu[2], uuid->uu[3]);
    }
    else if (UUID_TYPE_128 == uuid->type)
    {
        pos = sprintf(buf, "%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x",
                      uuid->uu[ 0], uuid->uu[ 1], uuid->uu[ 2], uuid->uu[ 3],
                      uuid->uu[ 4], uuid->uu[ 5], uuid->uu[ 6], uuid->uu[ 7],
                      uuid->uu[ 8], uuid->uu[ 9], uuid->uu[10], uuid->uu[11],
                      uuid->uu[12], uuid->uu[13], uuid->uu[14], uuid->uu[15]
        );
    }
    else
    {
        /* do nothing */
    }

    return pos;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_stm32wba_ble_adapter_util_convert_sid_uuid_to_stm32_svc_uuid(uint8_t * const out_uuid_type, Service_UUID_t * const out_uuid, const sid_ble_cfg_uuid_info_t * const in_uuid)
{
    if ((NULL == out_uuid_type) || (NULL == out_uuid) || (NULL == in_uuid))
    {
        return SID_ERROR_INVALID_ARGS;
    }

    switch (in_uuid->type)
    {
        case UUID_TYPE_16:
            {
                out_uuid->Service_UUID_16 = in_uuid->uu[1] | (in_uuid->uu[0] << 8);
                *out_uuid_type = STM32_WPAN_BLE_UUID_TYPE_16;
            }
            break;

        case UUID_TYPE_128:
            {
                const uint32_t uuid_len = sizeof(out_uuid->Service_UUID_128);
                for (uint32_t i = 0u; i < uuid_len; i++)
                {
                    out_uuid->Service_UUID_128[i] = in_uuid->uu[uuid_len - i - 1u];
                }
                *out_uuid_type = STM32_WPAN_BLE_UUID_TYPE_128;
            }
            break;

        default:
            return SID_ERROR_INVALID_ARGS;
    }

    return SID_ERROR_NONE;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_stm32wba_ble_adapter_util_convert_sid_uuid_to_stm32_char_uuid(uint8_t * const out_uuid_type, Char_UUID_t * const out_uuid, const sid_ble_cfg_uuid_info_t * const in_uuid)
{
    if ((NULL == out_uuid_type) || (NULL == out_uuid) || (NULL == in_uuid))
    {
        return SID_ERROR_INVALID_ARGS;
    }

    switch (in_uuid->type)
    {
        case UUID_TYPE_16:
            {
                out_uuid->Char_UUID_16 = in_uuid->uu[1] | (in_uuid->uu[0] << 8);
                *out_uuid_type = STM32_WPAN_BLE_UUID_TYPE_16;
            }
            break;

        case UUID_TYPE_128:
            {
                const uint32_t uuid_len = sizeof(out_uuid->Char_UUID_128);
                for (uint32_t i = 0u; i < uuid_len; i++)
                {
                    out_uuid->Char_UUID_128[i] = in_uuid->uu[uuid_len - i - 1u];
                }
                *out_uuid_type = STM32_WPAN_BLE_UUID_TYPE_128;
            }
            break;

        default:
            return SID_ERROR_INVALID_ARGS;
    }

    return SID_ERROR_NONE;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_stm32wba_ble_adapter_util_convert_sid_uuid_to_stm32_char_desc_uuid(uint8_t * const out_uuid_type, Char_Desc_Uuid_t * const out_uuid, const sid_ble_cfg_uuid_info_t * const in_uuid)
{
    if ((NULL == out_uuid_type) || (NULL == out_uuid) || (NULL == in_uuid))
    {
        return SID_ERROR_INVALID_ARGS;
    }

    switch (in_uuid->type)
    {
        case UUID_TYPE_16:
            {
                out_uuid-> Char_UUID_16 = in_uuid->uu[1] | (in_uuid->uu[0] << 8);
                *out_uuid_type = STM32_WPAN_BLE_UUID_TYPE_16;
            }
            break;

        case UUID_TYPE_128:
            {
                const uint32_t uuid_len = sizeof(out_uuid->Char_UUID_128);
                for (uint32_t i = 0u; i < uuid_len; i++)
                {
                    out_uuid->Char_UUID_128[i] = in_uuid->uu[uuid_len - i - 1u];
                }
                *out_uuid_type = STM32_WPAN_BLE_UUID_TYPE_128;
            }
            break;

        default:
            return SID_ERROR_INVALID_ARGS;
    }

    return SID_ERROR_NONE;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED uint8_t sid_stm32wba_ble_adapter_util_convert_sid_ble_addr_type_to_stm32_ble_addr_type(const sid_ble_cfg_mac_address_type_t mac_addr_type)
{
    uint8_t hal_addr_type;

    switch (mac_addr_type)
    {
        case SID_BLE_CFG_MAC_ADDRESS_TYPE_RANDOM_PRIVATE_NON_RESOLVABLE:
            hal_addr_type = GAP_NON_RESOLVABLE_PRIVATE_ADDR;
            break;

        case SID_BLE_CFG_MAC_ADDRESS_TYPE_RANDOM_PRIVATE_RESOLVABLE:
            hal_addr_type = GAP_RESOLVABLE_PRIVATE_ADDR;
            break;

        case SID_BLE_CFG_MAC_ADDRESS_TYPE_STATIC_RANDOM:
            hal_addr_type = GAP_STATIC_RANDOM_ADDR;

        case SID_BLE_CFG_MAC_ADDRESS_TYPE_PUBLIC:
        default:
            hal_addr_type = GAP_PUBLIC_ADDR;
            break;
    }

    return hal_addr_type;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED uint8_t sid_stm32wba_ble_adapter_util_convert_sid_ble_addr_type_to_hci_own_adv_addr_type(const sid_ble_cfg_mac_address_type_t mac_addr_type)
{
    uint8_t own_address_type;

    /* Set advertisement parameters */
    switch (mac_addr_type)
    {
        case SID_BLE_CFG_MAC_ADDRESS_TYPE_PUBLIC:
            own_address_type = SID_BLE_HCI_ADV_OWN_ADDR_TYPE_PUBLIC;
            break;

        case SID_BLE_CFG_MAC_ADDRESS_TYPE_RANDOM_PRIVATE_NON_RESOLVABLE:
        case SID_BLE_CFG_MAC_ADDRESS_TYPE_STATIC_RANDOM:
            own_address_type = SID_BLE_HCI_ADV_OWN_ADDR_TYPE_RANDOM;
            break;

        case SID_BLE_CFG_MAC_ADDRESS_TYPE_RANDOM_PRIVATE_RESOLVABLE:
            own_address_type = SID_BLE_HCI_ADV_OWN_ADDR_TYPE_RESOLVABLE_OR_RANDOM;
            break;

        default:
            own_address_type = SID_BLE_HCI_ADV_OWN_ADDR_TYPE_RANDOM;
            break;
    }

    return own_address_type;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_stm32wba_ble_adapter_util_generate_private_nonresolvable_random_mac_address(sid_pal_ble_prv_bt_addr_buffer_t * const buffer)
{
    sid_error_t err = SID_ERROR_GENERIC;

    do
    {
        do
        {
            err = _generate_generic_random_mac_address(buffer);
        } while ((SID_ERROR_NONE == err)
              && (_verify_random_mac_address_random_part(buffer) != SID_ERROR_NONE)); /* Ensure random data complies with BLE spec */

        if (err != SID_ERROR_NONE)
        {
            break;
        }

        buffer->bytes[5] &= 0x3Fu; /* The two upper bits shall be set to 00b */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_stm32wba_ble_adapter_util_generate_private_resolvable_random_mac_address(sid_pal_ble_prv_bt_addr_buffer_t * const buffer)
{
    sid_error_t err = SID_ERROR_GENERIC;
    tBleStatus ble_status;

    do
    {
        sid_pal_ble_prv_irk_buffer_t irk;
        uint8_t padded_prand[SID_BLE_ADAPTER_UTILS_PADDED_PRAND_LEN];
        uint8_t aes_enc_data[SID_BLE_ADAPTER_UTILS_PADDED_PRAND_LEN];

        /* Generate random data */
        err = _generate_generic_random_mac_address(buffer);
        if (err != SID_ERROR_NONE)
        {
            break;
        }

        /* The two upper bits shall be set to 01b */
        buffer->bytes[5] &= 0x3Fu;
        buffer->bytes[5] |= 0x40u;

        /* Load local IRK */
        SMA_Generate_IRK(irk.bytes);

        /* Prepare padded prand data */
        SID_STM32_UTIL_fast_memset(padded_prand, 0u, sizeof(padded_prand));
        padded_prand[0] = buffer->bytes[3];
        padded_prand[1] = buffer->bytes[4];
        padded_prand[2] = buffer->bytes[5];

        /* Compute random address hash using ah security function */
        ble_status = hci_le_encrypt(irk.bytes, padded_prand, aes_enc_data);
        if (ble_status != BLE_STATUS_SUCCESS)
        {
            err = SID_ERROR_ENCRYPTION_FAIL;
            break;
        }

        /* Put computed hash into BLE address buffer */
        buffer->bytes[0] = aes_enc_data[0];
        buffer->bytes[1] = aes_enc_data[1];
        buffer->bytes[2] = aes_enc_data[2];

        err = SID_ERROR_NONE;
    } while ((SID_ERROR_NONE == err)
          && (_verify_random_mac_address_random_part(buffer) != SID_ERROR_NONE)); /* Ensure random data complies with BLE spec */

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_stm32wba_ble_adapter_util_generate_static_random_mac_address(sid_pal_ble_prv_bt_addr_buffer_t * const buffer)
{
    sid_error_t err = SID_ERROR_GENERIC;

    do
    {
        do
        {
            err = _generate_generic_random_mac_address(buffer);
        } while ((SID_ERROR_NONE == err)
              && (_verify_random_mac_address_random_part(buffer) != SID_ERROR_NONE)); /* Ensure random data complies with BLE spec */

        if (err != SID_ERROR_NONE)
        {
            break;
        }

        buffer->bytes[5] |= 0xC0u; /* The two upper bits shall be set to 11b */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_stm32wba_ble_adapter_util_get_host_public_addr(sid_pal_ble_prv_bt_addr_buffer_t * const buffer)
{
    sid_error_t err = SID_ERROR_GENERIC;

    do
    {
        if (NULL == buffer)
        {
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

#if (CFG_BD_ADDRESS != 0)
        #warning "DON'T USE HARD-CODED BLE ADDRESS IN PRODUCTION"
        SID_PAL_LOG_WARNING("USING HARD-CODED BLE ADDRESS");
        buffer->dwords[0] = (uint64_t)(CFG_ADV_BD_ADDRESS);
#else
        /* Load Device Unique Number (UDN) */
        const uint32_t  udn = LL_FLASH_GetUDN();

        /* Build public address based on UDN if possible */
        if (udn != 0xFFFFFFFFu)
        {
            const uint32_t company_id = LL_FLASH_GetSTCompanyID();
            const uint32_t device_id = LL_FLASH_GetDeviceID();

            /**
             * Public Address with the ST company ID
             * bit[47:24] : 24bits (OUI) equal to the company ID
             * bit[23:16] : Device ID.
             * bit[15:0] : The last 16bits from the UDN
             * Note: In order to use the Public Address in a final product, a dedicated
             * 24bits company ID (OUI) shall be bought.
             */
            buffer->bytes[0] = (uint8_t)( udn       & 0xFFu);
            buffer->bytes[1] = (uint8_t)((udn >> 8) & 0xFFu);
            buffer->bytes[2] = (uint8_t)( device_id & 0xFFu);
            buffer->bytes[3] = (uint8_t)( company_id        & 0xFFu);
            buffer->bytes[4] = (uint8_t)((company_id >>  8) & 0xFFu);
            buffer->bytes[5] = (uint8_t)((company_id >> 16) & 0xFFu);
        }
        else
        {
            /* Try to read public address from OTP */
            OTP_Data_s *      p_otp_addr = NULL;
            HAL_StatusTypeDef hal_ret;

            hal_ret = OTP_Read(0, &p_otp_addr);
            if ((HAL_OK == hal_ret) && (p_otp_addr != NULL))
            {
                SID_STM32_UTIL_fast_memcpy(buffer->bytes, p_otp_addr->bd_address, sizeof(buffer->bytes));
            }
            else
            {
                SID_PAL_LOG_ERROR("Failed to read BLE address from OTP");
                err = SID_ERROR_STORAGE_READ_FAIL;
                break;
            }
        }
#endif /* CFG_BD_ADDRESS */

        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_stm32wba_ble_adapter_util_get_host_static_random_addr(sid_pal_ble_prv_bt_addr_buffer_t * const buffer)
{
    sid_error_t err = SID_ERROR_GENERIC;

    do
    {
        if (NULL == buffer)
        {
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        /* Get static random address if it has already been generated */
        err = sid_pal_storage_kv_record_get(STORAGE_KV_SID_BLE_ADAPTER_CONFIG_GROUP, STORAGE_KV_SID_BLE_ADAPTER_STATIC_RANDOM_ADDRESS_KEY, &buffer->words[0], sizeof(buffer->words));
        if (SID_ERROR_NONE == err)
        {
            SID_BLE_ADAPTER_UTILS_LOG_DEBUG("Loaded BLE static random address from KV storage");
            break;
        }
        else if (SID_ERROR_NOT_FOUND == err)
        {
            /* Generate new static random address */
            err = sid_stm32wba_ble_adapter_util_generate_static_random_mac_address(buffer);
            if (err != SID_ERROR_NONE)
            {
                break;
            }

            err = sid_pal_storage_kv_record_set(STORAGE_KV_SID_BLE_ADAPTER_CONFIG_GROUP, STORAGE_KV_SID_BLE_ADAPTER_STATIC_RANDOM_ADDRESS_KEY, &buffer->words[0], sizeof(buffer->words));
            if (err != SID_ERROR_NONE)
            {
                SID_BLE_ADAPTER_UTILS_LOG_ERROR("  Fail   : Unable to store BLE static random address to KV storage, error code: %d", err);
                break;
            }
            SID_BLE_ADAPTER_UTILS_LOG_DEBUG("Stored new BLE static random address");
        }
        else
        {
            /* Failed to read from KV storage */\
            break;
        }
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_stm32wba_ble_adapter_util_get_local_irk_seed(sid_pal_ble_prv_irk_buffer_t * const buffer)
{
    sid_error_t err = SID_ERROR_GENERIC;

    do
    {
        if (NULL == buffer)
        {
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        /* Get static random address if it has already been generated */
        err = sid_pal_storage_kv_record_get(STORAGE_KV_SID_BLE_ADAPTER_CONFIG_GROUP, STORAGE_KV_SID_BLE_ADAPTER_LOCAL_IRK_SEED_KEY, &buffer->words[0], sizeof(buffer->words));
        if (SID_ERROR_NONE == err)
        {
            SID_BLE_ADAPTER_UTILS_LOG_DEBUG("Loaded local BLE IRK seed from KV storage");
            break;
        }
        else if (SID_ERROR_NOT_FOUND == err)
        {
            /* Generate new IRK seed */
#if SID_STM32_BLE_USE_HARDCODED_SECURITY_KEYS
            #warning "DON'T USE HARD-CODED IRK IN PRODUCTION"
            uint8_t tmp_irk[] = CFG_BLE_IRK;
            SID_STM32_UTIL_fast_memcpy(buffer->bytes, tmp_irk, sizeof(buffer));
#else
            err = _generate_random_key(buffer->words, SID_STM32_UTIL_ARRAY_SIZE(buffer->words));
            if (err != SID_ERROR_NONE)
            {
                SID_BLE_ADAPTER_UTILS_LOG_ERROR("Failed to generate new BLE IRK seed. Error %d", (int32_t)err);
                break;
            }
#endif /* SID_STM32_BLE_USE_HARDCODED_SECURITY_KEYS */

            err = sid_pal_storage_kv_record_set(STORAGE_KV_SID_BLE_ADAPTER_CONFIG_GROUP, STORAGE_KV_SID_BLE_ADAPTER_LOCAL_IRK_SEED_KEY, &buffer->words[0], sizeof(buffer->words));
            if (err != SID_ERROR_NONE)
            {
                SID_BLE_ADAPTER_UTILS_LOG_ERROR("  Fail   : Unable to store local BLE IRK seed to KV storage, error code: %d", err);
                break;
            }
            SID_BLE_ADAPTER_UTILS_LOG_DEBUG("Stored new local BLE IRK seed");
        }
        else
        {
            /* Failed to read from KV storage */
            break;
        }
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_stm32wba_ble_adapter_util_get_erk(sid_pal_ble_prv_erk_buffer_t * const buffer)
{
    sid_error_t err = SID_ERROR_GENERIC;

    do
    {
        if (NULL == buffer)
        {
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        /* Get static random address if it has already been generated */
        err = sid_pal_storage_kv_record_get(STORAGE_KV_SID_BLE_ADAPTER_CONFIG_GROUP, STORAGE_KV_SID_BLE_ADAPTER_ERK_KEY, &buffer->words[0], sizeof(buffer->words));
        if (SID_ERROR_NONE == err)
        {
            SID_BLE_ADAPTER_UTILS_LOG_DEBUG("Loaded BLE ERK from KV storage");
            break;
        }
        else if (SID_ERROR_NOT_FOUND == err)
        {
            /* Generate new ERK */
#if SID_STM32_BLE_USE_HARDCODED_SECURITY_KEYS
            #warning "DON'T USE HARD-CODED ERK IN PRODUCTION"
            uint8_t tmp_erk[] = CFG_BLE_ERK;
            SID_STM32_UTIL_fast_memcpy(buffer->bytes, tmp_erk, sizeof(buffer));
#else
            err = _generate_random_key(buffer->words, SID_STM32_UTIL_ARRAY_SIZE(buffer->words));
            if (err != SID_ERROR_NONE)
            {
                SID_BLE_ADAPTER_UTILS_LOG_ERROR("Failed to generate new BLE ERK. Error %d", (int32_t)err);
                break;
            }
#endif /* SID_STM32_BLE_USE_HARDCODED_SECURITY_KEYS */

            err = sid_pal_storage_kv_record_set(STORAGE_KV_SID_BLE_ADAPTER_CONFIG_GROUP, STORAGE_KV_SID_BLE_ADAPTER_ERK_KEY, &buffer->words[0], sizeof(buffer->words));
            if (err != SID_ERROR_NONE)
            {
                SID_BLE_ADAPTER_UTILS_LOG_ERROR("  Fail   : Unable to store BLE ERK to KV storage, error code: %d", err);
                break;
            }
            SID_BLE_ADAPTER_UTILS_LOG_DEBUG("Stored new BLE ERK");
        }
        else
        {
            /* Failed to read from KV storage */
            break;
        }
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_stm32wba_ble_adapter_util_set_gap_ppcp_char(const sid_ble_cfg_conn_param_t * const conn_param)
{
    sid_error_t err = SID_ERROR_GENERIC;
    tBleStatus ble_status;

    do
    {
        uint8_t ppcp_value_buffer[SID_STM32_BLE_PPCP_CHAR_VALUE_LENGTH];

        if (NULL == conn_param)
        {
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        ppcp_value_buffer[SID_STM32_BLE_PPCP_CHAR_CONN_INTERVAL_MIN_OFFSET]     = (uint8_t)( conn_param->min_conn_interval       & 0xFFu);
        ppcp_value_buffer[SID_STM32_BLE_PPCP_CHAR_CONN_INTERVAL_MIN_OFFSET + 1] = (uint8_t)((conn_param->min_conn_interval >> 8) & 0xFFu);

        ppcp_value_buffer[SID_STM32_BLE_PPCP_CHAR_CONN_INTERVAL_MAX_OFFSET]     = (uint8_t)( conn_param->max_conn_interval       & 0xFFu);
        ppcp_value_buffer[SID_STM32_BLE_PPCP_CHAR_CONN_INTERVAL_MAX_OFFSET + 1] = (uint8_t)((conn_param->max_conn_interval >> 8) & 0xFFu);

        ppcp_value_buffer[SID_STM32_BLE_PPCP_CHAR_CONN_LATENCY_OFFSET]          = (uint8_t)( conn_param->slave_latency           & 0xFFu);
        ppcp_value_buffer[SID_STM32_BLE_PPCP_CHAR_CONN_LATENCY_OFFSET + 1]      = (uint8_t)((conn_param->slave_latency     >> 8) & 0xFFu);

        ppcp_value_buffer[SID_STM32_BLE_PPCP_CHAR_CONN_TIMEOUT_OFFSET]          = (uint8_t)( conn_param->conn_sup_timeout        & 0xFFu);
        ppcp_value_buffer[SID_STM32_BLE_PPCP_CHAR_CONN_TIMEOUT_OFFSET + 1]      = (uint8_t)((conn_param->conn_sup_timeout  >> 8) & 0xFFu);

        ble_status = aci_gatt_update_char_value(sid_ble_drv_ctx.gap_service_handle,
                                                sid_ble_drv_ctx.gap_ppcp_char_handle,
                                                0u,
                                                sizeof(ppcp_value_buffer),
                                                (uint8_t *)ppcp_value_buffer);
        if (ble_status != BLE_STATUS_SUCCESS)
        {
            SID_BLE_ADAPTER_UTILS_LOG_ERROR("  Fail   : aci_gatt_update_char_value - Peripheral Preferred Connection Parameters, result: 0x%02X", ble_status);
            err = SID_ERROR_IO_ERROR;
            break;
        }
        SID_BLE_ADAPTER_UTILS_LOG_DEBUG("  Success: aci_gatt_update_char_value - Peripheral Preferred Connection Parameters");

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED uint32_t sid_stm32wba_ble_adapter_util_calc_min_conn_timeout(const uint32_t conn_interval, const uint32_t conn_latency)
{
    uint32_t conn_timeout = SID_STM32_BLE_MS_TO_CONN_SUPERVISION_UNITS(SID_STM32_BLE_CONN_INTERVAL_UNITS_TO_MS((conn_latency + 1u) * conn_interval * 2u));
    return conn_timeout;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED uint32_t sid_stm32wba_ble_adapter_util_calc_max_conn_latency(const uint32_t conn_interval, const uint32_t conn_timeout)
{
    uint32_t conn_latency = (uint32_t)((float)(SID_STM32_BLE_CONN_SUPERVISION_UNITS_TO_MS(conn_timeout)) / (2.f * SID_STM32_BLE_CONN_INTERVAL_UNITS_TO_MS(conn_interval))) - 1u; /* Intentional underflow, UINT32_MAX indicates there's no conn_latency to meet the timing requirements */
    return conn_latency;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED uint32_t sid_stm32wba_ble_adapter_util_calc_max_conn_interval(const uint32_t conn_timeout, const uint32_t conn_latency)
{
    uint32_t conn_interval = SID_STM32_BLE_MS_TO_CONN_INTERVAL_UNITS(SID_STM32_BLE_CONN_SUPERVISION_UNITS_TO_MS(conn_timeout) / (2u * (conn_latency + 1u)));

    /* Ensure conn_interval does not lead to conn_timeout violation due to rounding */
    if (sid_stm32wba_ble_adapter_util_calc_min_conn_timeout(conn_interval, conn_latency) > conn_timeout)
    {
        conn_interval--;
    }

    return conn_interval;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_stm32wba_ble_adapter_util_validate_proposed_conn_params(const sid_ble_ext_proposed_conn_params_t * const proposed_params, const sid_ble_ext_conn_params_limits_t * const limits,
                                                                                                  sid_ble_ext_accepted_conn_params_t * const out_accepted_params, tBleStatus * const out_le_resp_code)
{
    SID_PAL_ASSERT(proposed_params != NULL);
    SID_PAL_ASSERT(limits != NULL);
    SID_PAL_ASSERT(out_accepted_params != NULL);
    SID_PAL_ASSERT(out_le_resp_code != NULL);

    do
    {
        uint32_t min_required_conn_timeout;

        /* Check minimum connection interval */
        if (proposed_params->interval_min < limits->interval_min)
        {
            SID_BLE_ADAPTER_UTILS_LOG_DEBUG("Unacceptable BLE connection interval - too low");
            if (proposed_params->interval_max >= limits->interval_min)
            {
                /* There's a room to increase the minimum connection interval value to meet the lower boundary limit */
                out_accepted_params->interval_min = limits->interval_min;
            }
            else
            {
                /* Cannot increase minimum connection interval, reject proposed params */
                *out_le_resp_code = SID_BLE_HCI_STATUS_UNACCEPTABLE_CONN_PARAMS;
                break;
            }
        }
        else
        {
            /* Accept the proposed value */
            out_accepted_params->interval_min = proposed_params->interval_min;
        }

        /* Check maximum connection interval */
        if (proposed_params->interval_max > limits->interval_max)
        {
            SID_BLE_ADAPTER_UTILS_LOG_DEBUG("Unacceptable BLE connection interval - too high");
            if (proposed_params->interval_min <= limits->interval_max)
            {
                /* There's a room to decrease the maximum connection interval value to meet the lower boundary limit */
                out_accepted_params->interval_max = limits->interval_max;
            }
            else
            {
                /* Cannot decrease maximum connection interval, reject proposed params */
                *out_le_resp_code = SID_BLE_HCI_STATUS_UNACCEPTABLE_CONN_PARAMS;
                break;
            }
        }
        else
        {
            /* Accept the proposed value */
            out_accepted_params->interval_max = proposed_params->interval_max;
        }

        /* Ensure the requested connection timeout is within the valid range */
        if ((proposed_params->timeout < SID_BLE_HCI_SUPERVISION_TIMEOUT_LOWER_LIMIT) || (proposed_params->timeout > SID_BLE_HCI_SUPERVISION_TIMEOUT_UPPER_LIMIT))
        {
            SID_BLE_ADAPTER_UTILS_LOG_DEBUG("Unacceptable BLE connection parameters - proposed timeout is out of BLE spec range");
            *out_le_resp_code = SID_BLE_HCI_STATUS_UNACCEPTABLE_CONN_PARAMS;
            break;
        }

        /* Accept latency as is for now */
        out_accepted_params->latency_max = MIN(proposed_params->latency_max, limits->latency_max);

        /* Calculate the bare minimum connection supervision timeout for the selected connection interval and latency */
        min_required_conn_timeout = sid_stm32wba_ble_adapter_util_calc_min_conn_timeout(out_accepted_params->interval_max, out_accepted_params->latency_max);

        /* Ensure we are within BLE specification */
        if (min_required_conn_timeout < SID_BLE_HCI_SUPERVISION_TIMEOUT_LOWER_LIMIT)
        {
            min_required_conn_timeout = SID_BLE_HCI_SUPERVISION_TIMEOUT_LOWER_LIMIT;
        }
        else if (min_required_conn_timeout > proposed_params->timeout)
        {
            /* The minimum required connection interval exceeds the upper boundary with the given connection interval and latency */
            uint32_t exceeds_limit = TRUE;

            /* Try to reduce connection timeout requirements by reducing latency first */
            uint32_t max_allowed_conn_latency = sid_stm32wba_ble_adapter_util_calc_max_conn_latency(out_accepted_params->interval_max, proposed_params->timeout);
            if (max_allowed_conn_latency != UINT32_MAX)
            {
                /* Timing constraints can be met by reducing connection latency only */
                out_accepted_params->latency_max = max_allowed_conn_latency;
                exceeds_limit = FALSE;
            }
            else
            {
                /* Timing constraints cannot be met for the given interval and supervision timeout by decreasing latency only. Set the latency to zero (bare minimum) and continue the optimization with tweaking connection interval */
                out_accepted_params->latency_max = 0u;
            }

            /* If we cannot optimize with latency and minimum timeout requirements are still too high try optimizing with connection interval */
            if (exceeds_limit != FALSE)
            {
                uint32_t max_allowed_conn_interval = sid_stm32wba_ble_adapter_util_calc_max_conn_interval(proposed_params->timeout, out_accepted_params->latency_max);

                if ((max_allowed_conn_interval >= out_accepted_params->interval_min) && (max_allowed_conn_interval <= out_accepted_params->interval_max))
                {
                    out_accepted_params->interval_max = max_allowed_conn_interval;
                    exceeds_limit = FALSE;
                }
            }

            if (exceeds_limit != FALSE)
            {
                /* Still unacceptable, the remote peer should reduce the minimum connection interval or extend connection timeout */
                SID_BLE_ADAPTER_UTILS_LOG_DEBUG("Unacceptable BLE connection parameters - minimum required conn timeout for given interval and latency exceeds the limit");
                *out_le_resp_code = SID_BLE_HCI_STATUS_UNACCEPTABLE_CONN_PARAMS;
                break;
            }
        }
        else
        {
            /* Within the limits, keep the value */
        }

        if (proposed_params->timeout > limits->reasonable_timeout) /* proposed_params->timeout is guaranteed to be >= min_required_conn_timeout at this point */
        {
            if (min_required_conn_timeout > limits->reasonable_timeout)
            {
                /* Nothing we can do about that, long timeout is required here */
                out_accepted_params->timeout = min_required_conn_timeout;
            }
            else
            {
                /* The new connection timeout is unnecessarily long, cut it down */
                SID_BLE_ADAPTER_UTILS_LOG_DEBUG("Unacceptable Sidewalk BLE connection supervision timeout - unreasonably long");
                out_accepted_params->timeout = limits->reasonable_timeout;
            }
        }
        else
        {
            out_accepted_params->timeout = proposed_params->timeout;
        }

        /* Set connection duration expectations */
        out_accepted_params->ce_length_min = limits->ce_length_min;
        out_accepted_params->ce_length_max = limits->ce_length_max;

        /* New connection parameters are acceptable if we got to this point, we can keep them */
        *out_le_resp_code = BLE_STATUS_SUCCESS;
    } while (0);

    return SID_ERROR_NONE;
}
