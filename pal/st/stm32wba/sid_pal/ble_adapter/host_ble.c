/**
  ******************************************************************************
  * @file    host_ble.c
  * @author  MCD Application Team
  * @brief   host_ble definition.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022-2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#include "host_ble.h"
#include "sid_pal_ble_adapter_stm32wba_config.h"

/* BLE stack */
#include "blestack.h"
#include "ble_types.h"
#include "stm32_wpan_common.h"

/* App-specific headers */
#include "app_conf.h"

/* Platform-specific headers */
#include "stm32wbaxx.h"

/* Sidewalk interfaces */
#include <sid_pal_log_ifc.h>

/* Utilities and helpers */
#include <sid_stm32_common_utils.h>

/* Private defines -----------------------------------------------------------*/

#ifndef SID_STM32_BLE_ENHANCED_MTU_EXCHANGE_HANDLING
#  error "SID_STM32_BLE_ENHANCED_MTU_EXCHANGE_HANDLING is not defined. Please set it explicitly"
#endif /* SID_STM32_BLE_ENHANCED_MTU_EXCHANGE_HANDLING */

/*****************************************************************************/

/* GATT buffer size (in bytes) */
#define BLE_GATT_BUF_SIZE                          (BLE_TOTAL_BUFFER_SIZE_GATT(CFG_BLE_NUM_GATT_ATTRIBUTES, \
                                                                               (SID_STM32_BLE_HOST_MAX_NUM_GATT_SERVICES + BLE_HOST_NUM_GATT_SYSTEM_SERVICES), \
                                                                               CFG_BLE_ATT_VALUE_ARRAY_SIZE))

/*****************************************************************************/

#define MBLOCK_COUNT                               (BLE_MBLOCKS_CALC(PREP_WRITE_LIST_SIZE, \
                                                                     SID_STM32_BLE_HOST_MAX_ATT_MTU, \
                                                                     SID_STM32_BLE_HOST_MAX_NUM_LINK) \
                                                    + CFG_BLE_MBLOCK_COUNT_MARGIN)

#define BLE_DYN_ALLOC_SIZE                         (BLE_TOTAL_BUFFER_SIZE(SID_STM32_BLE_HOST_MAX_NUM_LINK, MBLOCK_COUNT, (CFG_BLE_EATT_BEARER_PER_LINK * CFG_BLE_NUM_LINK)))

/* Private variables ---------------------------------------------------------*/

SID_STM32_ALIGN_4BYTES(static uint8_t ble_dyn_alloc_buffer[BLE_DYN_ALLOC_SIZE]);
SID_STM32_ALIGN_4BYTES(static uint8_t ble_gatt_buffer[BLE_GATT_BUF_SIZE]);

/* Private function prototypes -----------------------------------------------*/

static inline uint16_t _get_validated_sidewalk_mtu_size(const uint16_t desired_size);

#if (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_INTERLEAVING) || (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_CONCURRENCY)
static inline uint32_t _get_custom_profiles_max_mtu_size(const sid_ble_adapter_ext_cfg_t * const cfg);
#endif /* SID_STM32_BLE_COEXISTENCE_MODE */

/* Private function definitions ----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline uint16_t _get_validated_sidewalk_mtu_size(const uint16_t desired_size)
{
    uint16_t validated_mtu;

    validated_mtu = desired_size;
    if (validated_mtu > SID_STM32_BLE_SIDEWALK_ATT_MTU_MAX)
    {
        validated_mtu = SID_STM32_BLE_SIDEWALK_ATT_MTU_MAX;
    }
    else if (validated_mtu < SID_STM32_BLE_SIDEWALK_ATT_MTU_MIN)
    {
        validated_mtu = SID_STM32_BLE_SIDEWALK_ATT_MTU_MIN;
    }
    else
    {
        /* MTU size is valid */
    }

    return validated_mtu;
}

/*----------------------------------------------------------------------------*/

#if (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_INTERLEAVING) || (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_CONCURRENCY)
SID_STM32_SPEED_OPTIMIZED static inline uint32_t _get_custom_profiles_max_mtu_size(const sid_ble_adapter_ext_cfg_t * const cfg)
{
    uint32_t discovered_max_mtu = 0u;

    for (uint32_t i = 0u; i < cfg->custom_profiles.num_virtual_devices; i++)
    {
        const sid_ble_ext_virtual_device_t * const virt_dev = &cfg->custom_profiles.virtual_devices[i];
        if (SBEVDR_BLE_PERIPHERAL == virt_dev->device_type)
        {
            if (virt_dev->peripheral_cfg.max_att_mtu > discovered_max_mtu)
            {
                discovered_max_mtu = virt_dev->peripheral_cfg.max_att_mtu;
            }
        }
        else if (SBEVDR_BLE_CENTRAL == virt_dev->device_type)
        {
            if (virt_dev->central_cfg.max_att_mtu > discovered_max_mtu)
            {
                discovered_max_mtu = virt_dev->central_cfg.max_att_mtu;
            }
        }
        else
        {
            /* Broadcaster and Observer roles do not specify MTU size because these roles are non-connectable by definition */
        }
    }

    return discovered_max_mtu;
}
#endif /* SID_STM32_BLE_COEXISTENCE_MODE */

/* Global function definitions -----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED  tBleStatus HOST_BLE_Init(const sid_ble_adapter_ext_cfg_t * const cfg, const sid_pal_ble_prv_operating_mode_t init_type, uint64_t * const nvm_cache_buffer, uint16_t * const out_mtu_size)
{
    tBleStatus      status = BLE_STATUS_FAILED;
    BleStack_init_t stack_init_params;
    uint16_t        ble_stack_options;
    uint16_t        desired_mtu_size;
    uint16_t        selected_mtu_size;

    do
    {
        if (cfg == NULL)
        {
            status = BLE_STATUS_INVALID_PARAMS;
            break;
        }

#if (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_INTERLEAVING) || (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_CONCURRENCY)
        const uint16_t user_max_att_mtu = _get_custom_profiles_max_mtu_size(cfg);
#endif /* SID_STM32_BLE_COEXISTENCE_MODE */

        /* Determine ATT MTU size to use */
        switch (init_type)
        {
            case SPBP_OPERATING_MODE_SIDEWALK:
                desired_mtu_size  = cfg->sidewalk_profile.max_att_mtu;
                selected_mtu_size = _get_validated_sidewalk_mtu_size(desired_mtu_size);
                status = BLE_STATUS_SUCCESS;
                break;

            case SPBP_OPERATING_MODE_USER:
#if (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_INTERLEAVING)
                /* User may select any MTU size */
                desired_mtu_size  = user_max_att_mtu;
                selected_mtu_size = desired_mtu_size;
                status = BLE_STATUS_SUCCESS;
#else
                SID_PAL_LOG_ERROR("BLE cannot be initialized in interleaved mode. Not supported by the current config");
                status = BLE_STATUS_INVALID_PARAMS;
#endif /* SID_STM32_BLE_COEXISTENCE_MODE_INTERLEAVING */
                break;

            case SPBP_OPERATING_MODE_CONCURRENT:
#if (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_CONCURRENCY)
#  if SID_STM32_BLE_ENHANCED_MTU_EXCHANGE_HANDLING
                /* Validate Sidewalk settings */
                if (_get_validated_sidewalk_mtu_size(cfg->sidewalk_profile.max_att_mtu) != cfg->sidewalk_profile.max_att_mtu)
                {
                    /* For concurrency mode to operate properly the config shall be valid because it will be used in runtime to adjust MTU dynamically */
                    SID_PAL_LOG_ERROR("Sidewalk BLE ATT MTU size shall be in %u..%u range", SID_STM32_BLE_SIDEWALK_ATT_MTU_MIN, SID_STM32_BLE_SIDEWALK_ATT_MTU_MAX);
                    status = BLE_STATUS_INVALID_PARAMS;
                    break;
                }
                /* Choose the largest MTU value */
                desired_mtu_size = cfg->sidewalk_profile.max_att_mtu > user_max_att_mtu ? cfg->sidewalk_profile.max_att_mtu : user_max_att_mtu;
                selected_mtu_size = desired_mtu_size;
#  else
                /* No enhanced MTU exchange handling - select the largest MTU, but respect Sidewalk limits */
                desired_mtu_size = cfg->sidewalk_profile.max_att_mtu > user_max_att_mtu ? cfg->sidewalk_profile.max_att_mtu : user_max_att_mtu;
                selected_mtu_size = _get_validated_sidewalk_mtu_size(desired_mtu_size);
#  endif /* SID_STM32_BLE_ENHANCED_MTU_EXCHANGE_HANDLING */
                status = BLE_STATUS_SUCCESS;
#else
                SID_PAL_LOG_ERROR("BLE cannot be initialized in concurrency mode. Not supported by the current config");
                status = BLE_STATUS_INVALID_PARAMS;
#endif /* SID_STM32_BLE_COEXISTENCE_MODE_CONCURRENCY */
                break;

            default:
                SID_PAL_LOG_ERROR("BLE requested to be initialized in unknown mode: %u", (uint32_t)init_type);
                status = BLE_STATUS_INVALID_PARAMS;
                break;
        }

        /* Check if ATT MTU selection has failed */
        if (status != BLE_STATUS_SUCCESS)
        {
            break;
        }

        /* Store the selected ATT MTU size */
        *out_mtu_size = selected_mtu_size;

        /* Notify user about any restrictions applied */
        if (selected_mtu_size != desired_mtu_size)
        {
            SID_PAL_LOG_WARNING("Global BLE max ATT MTU forced from %u to %u bytes due to cfg limit", desired_mtu_size, selected_mtu_size);
        }
        else
        {
            SID_PAL_LOG_INFO("Global BLE max ATT MTU: %u bytes", selected_mtu_size);
        }

        /* Select BLE stack options based on the used stack variant and user configuration */
        ble_stack_options = (
            BLE_OPTIONS_DEV_NAME_READ_ONLY
          | BLE_OPTIONS_REDUCED_DB_IN_NVM
          | BLE_OPTIONS_CS_ALGO_2
#if SID_STM32_BLE_EXTENDED_ADV_SUPPORTED
          | BLE_OPTIONS_EXTENDED_ADV
#endif /* SID_STM32_BLE_EXTENDED_ADV_SUPPORTED */
        );

        /* Populate BLE stack initialization parameters */
        stack_init_params.numAttrRecord           = CFG_BLE_NUM_GATT_ATTRIBUTES;
        stack_init_params.numAttrServ             = (BLE_HOST_NUM_GATT_SYSTEM_SERVICES + SID_STM32_BLE_HOST_MAX_NUM_GATT_SERVICES);
        stack_init_params.attrValueArrSize        = CFG_BLE_ATT_VALUE_ARRAY_SIZE;
        stack_init_params.prWriteListSize         = CFG_BLE_ATTR_PREPARE_WRITE_VALUE_SIZE;
        stack_init_params.attMtu                  = selected_mtu_size;
        stack_init_params.max_coc_nbr             = CFG_BLE_COC_NBR_MAX;
        stack_init_params.max_coc_mps             = CFG_BLE_COC_MPS_MAX;
        stack_init_params.max_coc_initiator_nbr   = CFG_BLE_COC_INITIATOR_NBR_MAX;
        stack_init_params.max_add_eatt_bearers    = CFG_BLE_EATT_BEARER_PER_LINK * CFG_BLE_NUM_LINK;
        stack_init_params.numOfLinks              = SID_STM32_BLE_HOST_MAX_NUM_LINK;
        stack_init_params.mblockCount             = MBLOCK_COUNT;
        stack_init_params.bleStartRamAddress      = (uint8_t*)ble_dyn_alloc_buffer;
        stack_init_params.total_buffer_size       = sizeof(ble_dyn_alloc_buffer);
        stack_init_params.bleStartRamAddress_GATT = (uint8_t*)ble_gatt_buffer;
        stack_init_params.total_buffer_size_GATT  = sizeof(ble_gatt_buffer);
        stack_init_params.nvm_cache_buffer        = nvm_cache_buffer;
        stack_init_params.nvm_cache_max_size      = CFG_BLE_NVM_SIZE_MAX;
        stack_init_params.nvm_cache_size          = CFG_BLE_NVM_SIZE_MAX - 1u;
        stack_init_params.debug                   = BLE_DEBUG_RAND_ADDR_INIT; /* Required to enable the use of random addresses */
        stack_init_params.options                 = ble_stack_options;

        status = BleStack_Init(&stack_init_params);
    } while (0);

    return status;
}
