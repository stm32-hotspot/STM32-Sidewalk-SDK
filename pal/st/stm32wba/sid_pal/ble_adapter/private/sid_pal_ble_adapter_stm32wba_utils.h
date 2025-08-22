/**
  ******************************************************************************
  * @file    sid_pal_ble_adapter_stm32wba_utils.h
  * @brief   Private utility functions and helpers of the Sidewalk BLE driver
  * 
  * This interface expands sid_pal_ble_adapter_ifc with additional functionality
  * that is not part of the default Sidewalk interface. This module is hardware-
  * specific and targets STM32WBA platform only
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

#ifndef __SID_PAL_BLE_ADAPTER_STM32WBA_PRIVATE_UTILS_H_
#define __SID_PAL_BLE_ADAPTER_STM32WBA_PRIVATE_UTILS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "sid_pal_ble_adapter_stm32wba_private_defs.h"

/* Sidewalk interfaces */
#include <sid_ble_config_ifc.h>
#include <sid_error.h>
#include <sid_pal_storage_kv_internal_group_ids.h>

/* BLE stack */
#include <ble_core.h>
#include <ble_types.h>

/* Exported constants --------------------------------------------------------*/

#ifndef STORAGE_KV_SID_BLE_ADAPTER_CONFIG_GROUP
#  define STORAGE_KV_SID_BLE_ADAPTER_CONFIG_GROUP                   (0x1F40u)
#endif

#if (STORAGE_KV_SID_BLE_ADAPTER_CONFIG_GROUP >= 0x6FFEu)
#  error "Do not use group Ids greater than 0x6FFE"
#endif
#if (STORAGE_KV_SID_BLE_ADAPTER_CONFIG_GROUP == SID_PAL_STORAGE_KV_INTERNAL_PROTOCOL_GROUP_ID)
#  error "Invalid KV Storage Group IDs config: STORAGE_KV_SID_BLE_ADAPTER_CONFIG_GROUP is equal to SID_PAL_STORAGE_KV_INTERNAL_PROTOCOL_GROUP_ID"
#endif
#if (STORAGE_KV_SID_BLE_ADAPTER_CONFIG_GROUP == SID_PAL_STORAGE_KV_INTERNAL_CONFIG_GROUP_ID)
#  error "Invalid KV Storage Group IDs config: STORAGE_KV_SID_BLE_ADAPTER_CONFIG_GROUP is equal to SID_PAL_STORAGE_KV_INTERNAL_CONFIG_GROUP_ID"
#endif
#if (STORAGE_KV_SID_BLE_ADAPTER_CONFIG_GROUP == SID_PAL_STORAGE_KV_INTERNAL_BULK_DATA_TRANSFER_GROUP_ID)
#  error "Invalid KV Storage Group IDs config: STORAGE_KV_SID_BLE_ADAPTER_CONFIG_GROUP is equal to SID_PAL_STORAGE_KV_INTERNAL_BULK_DATA_TRANSFER_GROUP_ID"
#endif

#define STORAGE_KV_SID_BLE_ADAPTER_STATIC_RANDOM_ADDRESS_KEY        (0xF001u)
#define STORAGE_KV_SID_BLE_ADAPTER_LOCAL_IRK_SEED_KEY               (0xF002u)
#define STORAGE_KV_SID_BLE_ADAPTER_ERK_KEY                          (0xF003u)
#define STORAGE_KV_SID_BLE_ADAPTER_ERK_KEY                          (0xF003u)


#ifndef STORAGE_KV_SID_BLE_ADAPTER_BOND_PAIR_GROUP
#  define STORAGE_KV_SID_BLE_ADAPTER_BOND_PAIR_GROUP                (0x1F41u)
#endif

#if (STORAGE_KV_SID_BLE_ADAPTER_BOND_PAIR_GROUP >= 0x6FFEu)
#  error "Do not use group Ids greater than 0x6FFE"
#endif
#if (STORAGE_KV_SID_BLE_ADAPTER_BOND_PAIR_GROUP == SID_PAL_STORAGE_KV_INTERNAL_PROTOCOL_GROUP_ID)
#  error "Invalid KV Storage Group IDs config: STORAGE_KV_SID_BLE_ADAPTER_BOND_PAIR_GROUP is equal to SID_PAL_STORAGE_KV_INTERNAL_PROTOCOL_GROUP_ID"
#endif
#if (STORAGE_KV_SID_BLE_ADAPTER_BOND_PAIR_GROUP == SID_PAL_STORAGE_KV_INTERNAL_CONFIG_GROUP_ID)
#  error "Invalid KV Storage Group IDs config: STORAGE_KV_SID_BLE_ADAPTER_BOND_PAIR_GROUP is equal to SID_PAL_STORAGE_KV_INTERNAL_CONFIG_GROUP_ID"
#endif
#if (STORAGE_KV_SID_BLE_ADAPTER_BOND_PAIR_GROUP == SID_PAL_STORAGE_KV_INTERNAL_BULK_DATA_TRANSFER_GROUP_ID)
#  error "Invalid KV Storage Group IDs config: STORAGE_KV_SID_BLE_ADAPTER_BOND_PAIR_GROUP is equal to SID_PAL_STORAGE_KV_INTERNAL_BULK_DATA_TRANSFER_GROUP_ID"
#endif
#if (STORAGE_KV_SID_BLE_ADAPTER_BOND_PAIR_GROUP == STORAGE_KV_SID_BLE_ADAPTER_CONFIG_GROUP)
#  error "Invalid KV Storage Group IDs config: STORAGE_KV_SID_BLE_ADAPTER_BOND_PAIR_GROUP is equal to STORAGE_KV_SID_BLE_ADAPTER_CONFIG_GROUP"
#endif

#define STORAGE_KV_SID_BLE_ADAPTER_BOND_PAIR_KEY_MIN                (0x0000u) /*!< Starting value for Virtual BLE Device ID - remote peer bonding pairs */
#define STORAGE_KV_SID_BLE_ADAPTER_BOND_PAIR_KEY_VIRT_DEV_ID_OFFSET (8u)      /*!< Offset of the Virtual BLE Device ID in the key value */
#define STORAGE_KV_SID_BLE_ADAPTER_BOND_PAIR_KEY_MAX                (((SID_STM32_BLE_CUSTOM_VIRT_DEV_ID_MAX) << STORAGE_KV_SID_BLE_ADAPTER_BOND_PAIR_KEY_VIRT_DEV_ID_OFFSET) \
                                                                     | (((1u << STORAGE_KV_SID_BLE_ADAPTER_BOND_PAIR_KEY_VIRT_DEV_ID_OFFSET) - 1u)) \
                                                                    )         /*!< Maximum value for Virtual BLE Device ID - remote peer bonding pairs */

#if ((STORAGE_KV_SID_BLE_ADAPTER_BOND_PAIR_KEY_MIN & ((1u << STORAGE_KV_SID_BLE_ADAPTER_BOND_PAIR_KEY_VIRT_DEV_ID_OFFSET) - 1u)) != 0u)
#  error "STORAGE_KV_SID_BLE_ADAPTER_BOND_PAIR_KEY_MIN must be aligned with STORAGE_KV_SID_BLE_ADAPTER_BOND_PAIR_KEY_VIRT_DEV_ID_OFFSET, meaning lower bits shall be all zeros"
#endif

/* Exported functions ------------------------------------------------------- */

/**
 * @brief sprintf-like function to print out BLE UUIDs
 * 
 * @param [in] buf  Pointer to the string buffer
 * @param [in] uuid Sidewalk representation of a Bluetooth UUID
 * @return See @ref sprintf
 */
int32_t sid_stm32wba_ble_adapter_util_sprintf_uuid(char * const buf, const sid_ble_cfg_uuid_info_t * const uuid);

/**
 * @brief Converts byte order of the Sidewalk GATT service UUID representation to the byte order of the STM32WBA BLE stack
 * 
 * @param [out] out_uuid_type UUID type identifier for STM32 WPAN stack (0x01 - 16-bit, 0x02 - 128-bit, other values - invalid)
 * @param [out] out_uuid      UUID representation for STM32 WPAN stack
 * @param [in]  in_uuid       Sidewalk representation of the  GATT UUID
 * @return SID_ERROR_NONE if everything is ok, SID_ERROR_INVALID_ARGS if something is wrong (e.g. null pointers in args)
 */
sid_error_t sid_stm32wba_ble_adapter_util_convert_sid_uuid_to_stm32_svc_uuid(uint8_t * const out_uuid_type, Service_UUID_t * const out_uuid, const sid_ble_cfg_uuid_info_t * const in_uuid);

/**
 * @brief Converts byte order of the Sidewalk GATT characteristic UUID representation to the byte order of the STM32WBA BLE stack
 * 
 * @param [out] out_uuid_type UUID type identifier for STM32 WPAN stack (0x01 - 16-bit, 0x02 - 128-bit, other values - invalid)
 * @param [out] out_uuid      UUID representation for STM32 WPAN stack
 * @param [in]  in_uuid       Sidewalk representation of the  GATT UUID
 * @return SID_ERROR_NONE if everything is ok, SID_ERROR_INVALID_ARGS if something is wrong (e.g. null pointers in args)
 */
sid_error_t sid_stm32wba_ble_adapter_util_convert_sid_uuid_to_stm32_char_uuid(uint8_t * const out_uuid_type, Char_UUID_t * const out_uuid, const sid_ble_cfg_uuid_info_t * const in_uuid);

/**
 * @brief Converts byte order of the Sidewalk GATT characteristic UUID representation to the byte order of the STM32WBA BLE stack
 * 
 * @param [out] out_uuid_type UUID type identifier for STM32 WPAN stack (0x01 - 16-bit, 0x02 - 128-bit, other values - invalid)
 * @param [out] out_uuid      UUID representation for STM32 WPAN stack
 * @param [in]  in_uuid       Sidewalk representation of the  GATT UUID
 * @return SID_ERROR_NONE if everything is ok, SID_ERROR_INVALID_ARGS if something is wrong (e.g. null pointers in args)
 */
sid_error_t sid_stm32wba_ble_adapter_util_convert_sid_uuid_to_stm32_char_desc_uuid(uint8_t * const out_uuid_type, Char_Desc_Uuid_t * const out_uuid, const sid_ble_cfg_uuid_info_t * const in_uuid);

uint8_t sid_stm32wba_ble_adapter_util_convert_sid_ble_addr_type_to_stm32_ble_addr_type(const sid_ble_cfg_mac_address_type_t mac_addr_type);
uint8_t sid_stm32wba_ble_adapter_util_convert_sid_ble_addr_type_to_hci_own_adv_addr_type(const sid_ble_cfg_mac_address_type_t mac_addr_type);

sid_error_t sid_stm32wba_ble_adapter_util_generate_private_nonresolvable_random_mac_address(sid_pal_ble_prv_bt_addr_buffer_t * const buffer);
sid_error_t sid_stm32wba_ble_adapter_util_generate_private_resolvable_random_mac_address(sid_pal_ble_prv_bt_addr_buffer_t * const buffer);
sid_error_t sid_stm32wba_ble_adapter_util_generate_static_random_mac_address(sid_pal_ble_prv_bt_addr_buffer_t * const buffer);

/**
 * @brief Get BLE public address. This address can be derived from Unique Device Number (UDN) or loaded from OTP, depending on the configuration
 *
 * @param [out] buffer Storage for the address
 * @return SID_ERROR_NONE if everything is ok, SID_ERROR_INVALID_ARGS if something is wrong (e.g. null pointers in args)
 */
sid_error_t sid_stm32wba_ble_adapter_util_get_host_public_addr(sid_pal_ble_prv_bt_addr_buffer_t * const buffer);

/**
 * @brief Load static random BLE address form KV storage or generate a new one if KV storage is empty
 *
 * @param [out] buffer Storage for the address
 * @return SID_ERROR_NONE if everything is ok, SID_ERROR_INVALID_ARGS if something is wrong (e.g. null pointers in args)
 */
sid_error_t sid_stm32wba_ble_adapter_util_get_host_static_random_addr(sid_pal_ble_prv_bt_addr_buffer_t * const buffer);

/**
 * @brief Load local BLE Identity Resolving Key (IRK) seed form KV storage or generate a new one if KV storage is empty
 *
 * @note This is not the actual IRK used by the BLE stack. The IRK is generated by the stack based on this seed
 *
 * @param [out] buffer Storage for the IRK
 * @return SID_ERROR_NONE if everything is ok, SID_ERROR_INVALID_ARGS if something is wrong (e.g. null pointers in args)
 */
sid_error_t sid_stm32wba_ble_adapter_util_get_local_irk_seed(sid_pal_ble_prv_irk_buffer_t * const buffer);

/**
 * @brief Load BLE Encryption Root Key (ERK) form KV storage or generate a new one if KV storage is empty
 *
 * @param [out] buffer Storage for the ERK
 * @return SID_ERROR_NONE if everything is ok, SID_ERROR_INVALID_ARGS if something is wrong (e.g. null pointers in args)
 */
sid_error_t sid_stm32wba_ble_adapter_util_get_erk(sid_pal_ble_prv_erk_buffer_t * const buffer);

/**
 * @brief Set GAP Peripheral Preferred Connection Parameters (PPCP) characteristic value
 *
 * @param [in] conn_param Connection parmeters specification
 * @return SID_ERROR_NONE if everything is ok, SID_ERROR_INVALID_ARGS if something is wrong (e.g. null pointers in args)
 */
sid_error_t sid_stm32wba_ble_adapter_util_set_gap_ppcp_char(const sid_ble_cfg_conn_param_t * const conn_param);

/**
 * @brief Calculates the absolute minimum required connection supervision timeout for the given connection interval and latency
 *
 * @param [in] conn_interval Connection interval in BLE stack units (1.25ms per unit)
 * @param [in] conn_latency  Connection latency expressed in the number of LL events
 * @return Minimum required connection timeout expressed in BLE stack units (10ms per unit)
 */
uint32_t sid_stm32wba_ble_adapter_util_calc_min_conn_timeout(const uint32_t conn_interval, const uint32_t conn_latency);

/**
 * @brief Calculates the absolute maximum allowed connection latency for the given connection interval and supervision timeout
 *
 * @param [in] conn_interval Connection interval in BLE stack units (1.25ms per unit)
 * @param [in] conn_timeout Connection timeout expressed in BLE stack units (10ms per unit)
 * @return Maximum allowed connection latency expressed in the number of LL events, UINT32_MAX if BLE timing requirements cannot be met even with zero latency
 */
uint32_t sid_stm32wba_ble_adapter_util_calc_max_conn_latency(const uint32_t conn_interval, const uint32_t conn_timeout);

/**
 * @brief Calculates the absolute maximum allowed connection interval for the given connection supervision timeout and latency
 *
 * @param [in] conn_timeout Connection timeout expressed in BLE stack units (10ms per unit)
 * @param [in] conn_latency  Connection latency expressed in the number of LL events
 * @return Maximum allowed connection interval in BLE stack units (1.25ms per unit)
 */
uint32_t sid_stm32wba_ble_adapter_util_calc_max_conn_interval(const uint32_t conn_timeout, const uint32_t conn_latency);

/**
 * @brief Validates connection parameters propose by the remote peer, adjusts parameters to respect both remote peer's and local limits
 * @param [in]  proposed_params     Connection parameters to be validated and adjusted if needed
 * @param [in]  limits              Local limits against which validation and adjustments are performed
 * @param [out] out_accepted_params Validated and adjusted connection parameters that can be accepted
 * @param [out] out_le_resp_code    BLE_STATUS_SUCCESS if the method managed to validate and adjust parameters, otherwise a BLE status coded to be reported to the remote peer as a reason for rejecting suggested parameters
 * @return SID_ERROR_NONE if everything is ok, SID_ERROR_INVALID_ARGS if something is wrong (e.g. null pointers in args)
 */
sid_error_t sid_stm32wba_ble_adapter_util_validate_proposed_conn_params(const sid_ble_ext_proposed_conn_params_t * const proposed_params, const sid_ble_ext_conn_params_limits_t * const limits,
                                                                        sid_ble_ext_accepted_conn_params_t * const out_accepted_params, tBleStatus * const out_le_resp_code);

#ifdef __cplusplus
}
#endif

#endif /* __SID_PAL_BLE_ADAPTER_STM32WBA_PRIVATE_UTILS_H_ */
