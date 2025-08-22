/**
  ******************************************************************************
  * @file    sid_ble_profile_config.c
  * @brief   Sidewalk-specific BLE GATT profile configuration data
  * 
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024-2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#include <sid_ble_link_config_ifc.h>
#include <sid_stm32_common_utils.h>

#include "sid_pal_ble_adapter_stm32wba_ext_ifc.h"

/* Forward declarations ------------------------------------------------------*/
const sid_ble_config_t sid_ble_default_cfg;

/* Private constants ---------------------------------------------------------*/

/**
 * @brief GATT service declaration for Amazon's Alexa Mobile Accessory (AMA) service
 */
static const sid_ble_cfg_service_t sid_ble_ama_service = {
    /* AMA service - fe03 */
    .type = AMA_SERVICE,
    .id = {
        .type = UUID_TYPE_16,
        .uu = { 0xFEu, 0x03u },
    },
};

/**
 * @brief GATT characteristics of the AMA service
 */
static const sid_ble_cfg_characteristics_t sid_ble_ama_svc_chars[] = {
    {
        /* AMA Inbox - used to receive downlinks - 74f996c9-7d6c-4d58-9232-0427ab61c53c */
        .id = {
            .type = UUID_TYPE_128,
            .uu = { 0x74u, 0xF9u, 0x96u, 0xC9u, 0x7Du, 0x6Cu, 0x4Du, 0x58u, 0x92u, 0x32u, 0x04u, 0x27u, 0xABu, 0x61u, 0xC5u, 0x3Cu },
        },
        .properties = {
            .is_write_no_resp = true,
        },
        .perm = {
            .is_none = true,
        },
    },
    {
        /* AMA Outbox - used to send uplinks - b32e83c0-fece-47c1-9015-53b7e7f0d2fe */
        .id = {
            .type = UUID_TYPE_128,
            .uu = { 0xB3u, 0x2Eu, 0x83u, 0xC0u, 0xFEu, 0xCEu, 0x47u, 0xC1u, 0x90u, 0x15u, 0x53u, 0xB7u, 0xE7u, 0xF0u, 0xD2u, 0xFEu },
        },
        .properties = {
            .is_notify = true,
        },
        .perm = {
            .is_none = true,
        },
    },
};

/*----------------------------------------------------------------------------*/

/**
 * @brief Collection of the GATT services related to the Sidewalk link
 */
static const sid_ble_cfg_gatt_profile_t sid_ble_profiles[] = {
    {
        /* AMA GATT Service */
        .service        = sid_ble_ama_service,
        .char_count     = SID_STM32_UTIL_ARRAY_SIZE(sid_ble_ama_svc_chars),
        .characteristic = sid_ble_ama_svc_chars,
        .desc_count     = 0u,
        .desc           = NULL, /* Standard descriptors (e.g. CCCD) are added automatically by the BLE stack based on the characteristic properties. Use this field to instantiate only custom descriptors */
    },
};

/**
 * @brief A default Sidewalk BLE link config - to be used whenever user application does not supply app-specific configuration
 */
static const sid_ble_link_config_t default_ble_link_cfg = {
    .create_ble_adapter = sid_pal_ble_adapter_create,
    .config = &sid_ble_default_cfg,
};

/* Exported constants --------------------------------------------------------*/

/**
 * @brief BLE configuration for the Sidewalk link
 */
const sid_ble_config_t sid_ble_default_cfg = {
    .name                = NULL,                                                                /* Ignored by the driver, name is supplied via the user app config */
    .mtu                 = 0u,                                                                  /* Ignored by the driver, MTU size for the Sidewalk link is supplied via the user app config */
    .is_adv_available    = true,                                                                /* Ignored by the driver, Sidewalk link requires advertisements to work */
    .mac_addr_type       = SID_BLE_CFG_MAC_ADDRESS_TYPE_RANDOM_PRIVATE_NON_RESOLVABLE,          /* Typically a non-resolvable private address is used for Sidewalk advertisement for security reasons, but allow the user to override this */
    .adv_param           = {
                               .type          = AMA_SERVICE,                                    /* Ignored by the driver, Sidewalk link is forced to include AMA service UUID into advertisement data regardless of this setting */
                               .fast_enabled  = true,
                               .slow_enabled  = true,
                               .fast_interval = SID_STM32_BLE_MS_TO_ADV_INTERVAL_UNITS(160u),   /*!< BLE advertisement interval during the fast advertising phase - by default Sidewalk specification defines this as 160ms */
                               .fast_timeout  = SID_STM32_BLE_MS_TO_ADV_DURATION_UNITS(30000u), /*!< Duration of the BLE fast advertising phase - by default Sidewalk specification defines this as 30s */
                               .slow_interval = SID_STM32_BLE_MS_TO_ADV_INTERVAL_UNITS(1000u),  /*!< BLE advertisement interval during the slow advertising phase - by default Sidewalk specification defines this as 1s */
                               .slow_timeout  = SID_STM32_BLE_MS_TO_ADV_DURATION_UNITS(0u),     /*!< Duration of the BLE slow advertising phase - by default Sidewalk specification defines this as infinite */
                           },
    .is_conn_available   = true,                                                                /* Ignored by the driver, Sidewalk cannot work if BLE device is non-connectable */
    .conn_param          = {
                               .min_conn_interval = SID_STM32_BLE_MS_TO_CONN_INTERVAL_UNITS(30u),
                               .max_conn_interval = SID_STM32_BLE_MS_TO_CONN_INTERVAL_UNITS(75u),
                               .slave_latency     = 0u,
                               .conn_sup_timeout  = SID_STM32_BLE_MS_TO_CONN_SUPERVISION_UNITS(4000u),
                           },
    .num_profile         = SID_STM32_UTIL_ARRAY_SIZE(sid_ble_profiles),
    .profile             = sid_ble_profiles,
    .max_tx_power_in_dbm = SID_STM32_BLE_RADIO_MAX_TX_POWER_NA,
    .enable_link_metrics = true,
    .metrics_msg_retries = 3u,
};

/* Global function definitions -----------------------------------------------*/

__WEAK const sid_ble_link_config_t* app_get_sidewalk_ble_config(void)
{
    return &default_ble_link_cfg;
}
