/**
  ******************************************************************************
  * @file    app_ble_config.c
  * @brief   BLE radio configuration for Sidewalk application
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

/* Includes ------------------------------------------------------------------*/

#include <sid_pal_ble_adapter_stm32wba_ext_ifc.h>
#include <sid_stm32_common_utils.h>

#include <app_conf.h>
#include <ble_defs.h>

#include "app_ble_config.h"
#include "sid_ble_coexistence_cli.h"

/* Private defines -----------------------------------------------------------*/

#define SID_BLE_USER_DEMO_SMALL_MTU_CHAR_MAX_LEN  (20u)
#define SID_BLE_USER_DEMO_LARGE_MTU_CHAR_MAX_LEN (248u)

/* Private constants ---------------------------------------------------------*/

/**
 * @brief Advertising parameters used by BLE Beacon virtual device
 */
static const sid_ble_ext_adv_param_t beacon_adv_param = {
    .adv_tx_power      = SID_STM32_BLE_ADV_TX_POWER_NO_PREFERENCE,
    .scan_resp_en      = TRUE,
    .fast_enabled      = TRUE,
    .slow_enabled      = FALSE,
    .fast_interval_min = SID_STM32_BLE_MS_TO_ADV_INTERVAL_UNITS(250u),
    .fast_interval_max = SID_STM32_BLE_MS_TO_ADV_INTERVAL_UNITS(260u),
    .fast_timeout      = 0u,
};

/*----------------------------------------------------------------------------*/

/**
 * @brief Advertising parameters used by BLE Peripheral A virtual device
 */
static const sid_ble_ext_adv_param_t peripheral_a_adv_param = {
    .adv_tx_power      = SID_STM32_BLE_ADV_TX_POWER_NO_PREFERENCE,
    .auto_restart      = TRUE,
    .scan_resp_en      = TRUE,
    .fast_enabled      = TRUE,
    .slow_enabled      = TRUE,
    .fast_interval_min = SID_STM32_BLE_MS_TO_ADV_INTERVAL_UNITS(250u),
    .fast_interval_max = SID_STM32_BLE_MS_TO_ADV_INTERVAL_UNITS(260u),
    .fast_timeout      = SID_STM32_BLE_MS_TO_ADV_DURATION_UNITS(60000u),
    .slow_interval_min = SID_STM32_BLE_MS_TO_ADV_INTERVAL_UNITS(1200u),
    .slow_interval_max = SID_STM32_BLE_MS_TO_ADV_INTERVAL_UNITS(1300u),
    .slow_timeout      = 0u,
};

/**
 * @brief Connection parameter preferences for BLE Peripheral A virtual device
 */
static const sid_ble_cfg_conn_param_t peripheral_a_conn_params = {
    .min_conn_interval = SID_STM32_BLE_MS_TO_CONN_INTERVAL_UNITS(20u),
    .max_conn_interval = SID_STM32_BLE_MS_TO_CONN_INTERVAL_UNITS(75u),
    .slave_latency     = 0u,
    .conn_sup_timeout  = SID_STM32_BLE_MS_TO_CONN_SUPERVISION_UNITS(4000u),
};

/*----------------------------------------------------------------------------*/

/**
 * @brief Advertising parameters used by BLE Peripheral B virtual device
 */
static const sid_ble_ext_adv_param_t peripheral_b_adv_param = {
    .adv_tx_power      = SID_STM32_BLE_ADV_TX_POWER_NO_PREFERENCE,
    .auto_restart      = TRUE,
    .scan_resp_en      = TRUE,
    .fast_enabled      = TRUE,
    .slow_enabled      = TRUE,
    .fast_interval_min = SID_STM32_BLE_MS_TO_ADV_INTERVAL_UNITS(300u),
    .fast_interval_max = SID_STM32_BLE_MS_TO_ADV_INTERVAL_UNITS(350u),
    .fast_timeout      = SID_STM32_BLE_MS_TO_ADV_DURATION_UNITS(30000u),
    .slow_interval_min = SID_STM32_BLE_MS_TO_ADV_INTERVAL_UNITS(1200u),
    .slow_interval_max = SID_STM32_BLE_MS_TO_ADV_INTERVAL_UNITS(1300u),
    .slow_timeout      = 0u,
};

/**
 * @brief Connection parameter preferences for BLE Peripheral B virtual device
 */
static const sid_ble_cfg_conn_param_t peripheral_b_conn_params = {
    .min_conn_interval = SID_STM32_BLE_MS_TO_CONN_INTERVAL_UNITS(75u),
    .max_conn_interval = SID_STM32_BLE_MS_TO_CONN_INTERVAL_UNITS(150u),
    .slave_latency     = 0u,
    .conn_sup_timeout  = SID_STM32_BLE_MS_TO_CONN_SUPERVISION_UNITS(5000u),
};

/*----------------------------------------------------------------------------*/


static const sid_ble_ext_char_def_t demo_secure_chars[] = {
    /**
    * @brief Demo of the secure Read/Notify characteristic - access requires pairing and a secure LE connection
    *
    * @note UUID: 6d5ccc35-bfc3-4562-996f-f0e75ef020fc
    */
    {
        .char_name       = "Secure Read/Notify Char",
        .uuid = {
            .type        = UUID_TYPE_128,
            .uu          = {0x6Du, 0x5Cu, 0xCCu, 0x35, 0xBFu, 0xC3, 0x45u, 0x62u, 0x99u, 0x6Fu, 0xF0u, 0xE7u, 0x5Eu, 0xF0u, 0x20u, 0xFCu},
        },
        .max_length      = SID_BLE_USER_DEMO_LARGE_MTU_CHAR_MAX_LEN,
        .properties = {
            .read        = TRUE,
            .notify      = TRUE,
        },
        .permissions = {
            .authen_read = TRUE,
            .encry_read  = TRUE,
        },
        .desc_count      = 0u,
        .descriptors     = NULL,
    },

    /**
    * @brief Demo of the secure Write characteristic - access requires pairing and a secure LE connection
    *
    * @note UUID: 1a97fc1d-1fbe-4162-91fa-505969086990
    */
    {
        .char_name        = "Secure Write Char",
        .uuid = {
            .type         = UUID_TYPE_128,
            .uu           = {0x1Au, 0x97u, 0xFCu, 0x1Du, 0x1Fu, 0xBEu, 0x41u, 0x62u, 0x91u, 0xFAu, 0x50u, 0x59u, 0x69u, 0x08u, 0x69u, 0x90u},
        },
        .max_length       = SID_BLE_USER_DEMO_LARGE_MTU_CHAR_MAX_LEN,
        .properties = {
            .read         = TRUE,
            .write        = TRUE,
        },
        .permissions = {
            .authen_read  = TRUE,
            .encry_read   = TRUE,
            .authen_write = TRUE,
            .encry_write  = TRUE,
        },
        .desc_count       = 0u,
        .descriptors      = NULL,
    },
};

const sid_ble_cfg_uuid_info_t * const secure_char_with_notify = &demo_secure_chars[0].uuid; /* Pointer to characteristic UUID for easier access in the app */

static const sid_ble_ext_char_def_t demo_non_secure_chars[] = {
    /**
    * @brief Demo of the non-secure Read/Notify characteristic - no pairing required for access
    *
    * @note UUID: 839d12f9-7b78-4aff-aca9-7ee96197dfdc
    */
    {
        .char_name   = "Read/Notify Char",
        .uuid = {
            .type    = UUID_TYPE_128,
            .uu      = {0x83u, 0x9Du, 0x12u, 0xF9u, 0x7Bu, 0x78u, 0x4Au, 0xFFu, 0xACu, 0xA9u, 0x7Eu, 0xE9u, 0x61u, 0x97u, 0xDFu, 0xDCu},
        },
        .max_length  = SID_BLE_USER_DEMO_SMALL_MTU_CHAR_MAX_LEN,
        .properties = {
            .read    = TRUE,
            .notify  = TRUE,
        },
        .desc_count  = 0u,
        .descriptors = NULL,
    },

    /**
    * @brief Demo of the non-secure Write characteristic - no pairing required for access
    *
    * @note UUID: 957aa639-ec8d-469f-9357-3c0cc07abc5e
    */
    {
        .char_name   = "Write Char",
        .uuid = {
            .type    = UUID_TYPE_128,
            .uu      = {0x95u, 0x7Au, 0xA6u, 0x39u, 0xECu, 0x8Du, 0x46u, 0x9Fu, 0x93u, 0x57u, 0x3Cu, 0x0Cu, 0xC0u, 0x7Au, 0xBCu, 0x5Eu},
        },
        .max_length  = SID_BLE_USER_DEMO_SMALL_MTU_CHAR_MAX_LEN,
        .properties = {
            .read    = TRUE,
            .write   = TRUE,
        },
        .desc_count  = 0u,
        .descriptors = NULL,
    },
};

const sid_ble_cfg_uuid_info_t * const non_secure_char_with_notify = &demo_non_secure_chars[0].uuid; /* Pointer to characteristic UUID for easier access in the app */

/*----------------------------------------------------------------------------*/

/**
 * @brief Demo of the secure capabilities of the driver - this service contains secure characteristics and require pairing
 *
 * @note UUID: aaab9b8c-1594-4974-9704-8c344b88b8cb
 */
static const sid_ble_ext_gatt_svc_def_t demo_secure_svcs[] = {
    {
        .service_name    = "Secure Service Demo",
        .uuid = {
            .type        = UUID_TYPE_128,
            .uu          = {0xAAu, 0xABu, 0x9Bu, 0x8Cu, 0x15u, 0x94u, 0x49u, 0x74u, 0x97u, 0x04u, 0x8Cu, 0x34u, 0x4Bu, 0x88u, 0xB8u, 0xCBu},
        },
        .char_count      = SID_STM32_UTIL_ARRAY_SIZE(demo_secure_chars),
        .characteristics = demo_secure_chars,
    },
};

/**
 * @brief Demo of the plain capabilities of the driver - this service contains non-restricted characteristics and requires no pairing
 *
 * @note UUID: fe94614c-e10e-4cb5-b3c7-eee403117f44
 */
static const sid_ble_ext_gatt_svc_def_t demo_non_secure_svcs[] = {
    {
        .service_name    = "Plain Service Demo",
        .uuid = {
            .type        = UUID_TYPE_128,
            .uu          = {0xFEu, 0x94u, 0x61u, 0x4Cu, 0xE1u, 0x0Eu, 0x4Cu, 0xB5u, 0xB3u, 0xC7u, 0xEEu, 0xE4u, 0x03u, 0x11u, 0x7Fu, 0x44u},
        },
        .char_count      = SID_STM32_UTIL_ARRAY_SIZE(demo_non_secure_chars),
        .characteristics = demo_non_secure_chars,
    },
};

/*----------------------------------------------------------------------------*/

/**
 * @brief Collection of the virtual BLE devices that can be instantiated
 */
static const sid_ble_ext_virtual_device_t custom_ble_profiles[] = {
    /* BLE beacon (Broadcaster) demo - non-connectable advertiser */
    {
        .device_id       = APP_BLE_CONFIG_BEACON_VIRT_DEV_ID,
        .device_name     = "sid_beacon",
        .device_type     = SBEVDR_BLE_BROADCASTER,
        .mac_addr_type   = SID_BLE_CFG_MAC_ADDRESS_TYPE_RANDOM_PRIVATE_NON_RESOLVABLE,
        .broadcaster_cfg = {
            .adv_param   = &beacon_adv_param,
        },
    },

    /* BLE Peripheral with large MTU demo - connectable GATT server */
    {
        .device_id         = APP_BLE_CONFIG_PERIPH_LARGE_MTU_VIRT_DEV_ID,
        .device_name       = "sid_peripheral_max_mtu",
        .device_type       = SBEVDR_BLE_PERIPHERAL,
        .mac_addr_type     = SID_BLE_CFG_MAC_ADDRESS_TYPE_RANDOM_PRIVATE_RESOLVABLE,
        .peripheral_cfg = {
            .adv_param     = &peripheral_a_adv_param,
            .conn_param    = &peripheral_a_conn_params,
            .appearance    = GAP_APPEARANCE_GENERIC_KEYRING,
            .max_att_mtu   = CFG_BLE_ATT_MTU_MAX,
            .max_conn      = 1u,
            .gatt_profile = {
                .svc_count = SID_STM32_UTIL_ARRAY_SIZE(demo_secure_svcs),
                .services  = demo_secure_svcs,
            },
            .callbacks = {
                .on_pairing_pass_key_request         = sid_ble_coexistence_cli_on_pairing_pass_key_request,
                .on_pairing_numeric_comparison_value = sid_ble_coexistence_cli_on_pairing_numeric_comparison,
                .on_data_received                    = sid_ble_coexistence_cli_on_periph_data_received,
                .on_cccd_changed                     = sid_ble_coexistence_cli_on_periph_cccd_modified,
            }
        },
    },

    /* BLE Peripheral with small MTU demo - connectable GATT server */
    {
        .device_id         = APP_BLE_CONFIG_PERIPH_SMALL_MTU_VIRT_DEV_ID,
        .device_name       = "sid_peripheral_small_mtu",
        .device_type       = SBEVDR_BLE_PERIPHERAL,
        .mac_addr_type     = SID_BLE_CFG_MAC_ADDRESS_TYPE_RANDOM_PRIVATE_NON_RESOLVABLE,
        .peripheral_cfg = {
            .adv_param     = &peripheral_b_adv_param,
            .conn_param    = &peripheral_b_conn_params,
            .appearance    = GAP_APPEARANCE_GENERIC_REMOTE_CONTROL,
            .max_att_mtu   = 50u,
            .max_conn      = 2u,
            .gatt_profile = {
                .svc_count = SID_STM32_UTIL_ARRAY_SIZE(demo_non_secure_svcs),
                .services  = demo_non_secure_svcs,
            },
            .callbacks = {
                .on_data_received = sid_ble_coexistence_cli_on_periph_data_received,
                .on_cccd_changed  = sid_ble_coexistence_cli_on_periph_cccd_modified,
            },
        },
    },
};

/*----------------------------------------------------------------------------*/

/**
 * @brief Global configuration of the BLE driver
 */
static const sid_ble_adapter_ext_cfg_t ble_ext_config = {
    .sidewalk_profile = {
#if defined(NUCLEO_WBA52_BOARD)
        .device_name          = "sid_dut_ble_nucleo-wba52",
#elif defined(NUCLEO_WBA55_BOARD)
        .device_name          = "sid_dut_ble_nucleo-wba55",
#elif defined(NUCLEO_WBA65_BOARD)
        .device_name          = "sid_dut_ble_nucleo-wba65",
#elif defined(STM32WBA5x)
        .device_name          = "sid_dut_ble_stm32wba5x",
#elif defined(STM32WBA6x)
        .device_name          = "sid_dut_ble_stm32wba6x",
#else
#  error "Unknown MCU platform"
#endif
        .max_att_mtu          = SID_STM32_BLE_SIDEWALK_ATT_MTU_MAX,
        .appearance           = GAP_APPEARANCE_GENERIC_TAG,
    },
#if (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_INTERLEAVING) || (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_CONCURRENCY)
    .custom_profiles = {
        .virtual_devices      = custom_ble_profiles,
        .num_virtual_devices  = SID_STM32_UTIL_ARRAY_SIZE(custom_ble_profiles),
    },
#endif /* SID_STM32_BLE_COEXISTENCE_MODE */
};

/* Global function definitions -----------------------------------------------*/

const sid_ble_adapter_ext_cfg_t * sid_pal_ble_adapter_ext_get_link_config(void)
{
    return &ble_ext_config;
}
