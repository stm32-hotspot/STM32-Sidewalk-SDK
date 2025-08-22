/**
  ******************************************************************************
  * @file    sid_pal_ble_adapter_stm32wba_user_gap_gatt.h
  * @brief   User GAP and GATT (not related to Sidewalk operation) handling
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

#ifndef __SID_PAL_BLE_ADAPTER_STM32WBA_USER_GAP_GATT_H_
#define __SID_PAL_BLE_ADAPTER_STM32WBA_USER_GAP_GATT_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "sid_pal_ble_adapter_stm32wba_ext_ifc.h"
#include "sid_pal_ble_adapter_stm32wba_private_defs.h"

/* Sidewalk interfaces */
#include <sid_error.h>

/* Exported types ------------------------------------------------------------*/

typedef enum {
    SPBUM_STATE_UNINITIALIZED  = 0,
    SPBUM_STATE_INITIALIZING   = 1,
    SPBUM_STATE_INITIALIZED    = 2,
    SPBUM_STATE_DEINITIALIZING = 3,
} sid_pal_ble_user_mode_state_t;

typedef struct {
    sid_pal_ble_user_mode_state_t state;
} sid_pal_ble_user_mode_ctx_t;

/* Exported functions ------------------------------------------------------- */

/**
 * @brief Ground-base initialization actions to be performed on MCU start before BLE drive initialization can be started
 *
 * @return SID_ERROR_NONE on success
 */
sid_error_t sid_stm32wba_ble_adapter_user_platform_init(void);

/**
 * @brief Callback for Sidewalk-related BLE advertisement timeout
 *
 * @note This is an internal function of this BLE driver and normally it shall not be called by the end users directly
 *
 * @param [in] virt_dev_id Unique virtual BLE device identifier
 * @return SID_ERROR_NONE on success
 */
sid_error_t sid_stm32wba_ble_adapter_user_on_adv_timeout(const sid_ble_ext_virtual_device_id_t virt_dev_id);

/**
 * @brief Driver's internal callback for Connection Established event for local Peripheral
 *
 * @note This is an internal function of this BLE driver and normally it shall not be called by the end users directly
 *
 * @note This callback is triggered whenever a remote BLE Central connects to one of the Virtual BLE Peripherals, but this is not
 *       a direct equivalent for the HCI_LE_CONNECTION_COMPLETE_EVENT
 *
 * @param [in] virt_dev_id Local ID of the Virtual BLE Peripheral that got connected to a BLE Central
 * @param [in] conn_ctx Connection context - it keeps info about the link (e.g. remote peer address, negotiated MTU size, etc.)
 * @return SID_ERROR_NONE on success
 */
sid_error_t sid_stm32wba_ble_adapter_user_on_ble_connected_to_central(const sid_ble_ext_virtual_device_id_t virt_dev_id, sid_pal_ble_prv_connection_ctx_t * const conn_ctx);

/**
 * @brief Driver's internal callback for Connection Established event for local Central
 *
 * @note This is an internal function of this BLE driver and normally it shall not be called by the end users directly
 *
 * @note This callback is triggered whenever a remote BLE Peripheral connects to one of the Virtual BLE Centrals, but this is not
 *       a direct equivalent for the HCI_LE_CONNECTION_COMPLETE_EVENT
 *
 * @param [in] conn_ctx Connection context - it keeps info about the link (e.g. remote peer address, negotiated MTU size, etc.)
 * @return SID_ERROR_NONE on success
 */
sid_error_t sid_stm32wba_ble_adapter_user_on_ble_connected_to_peripheral(sid_pal_ble_prv_connection_ctx_t * const conn_ctx);

/**
 * @brief Driver's internal callback for HCI_DISCONNECTION_COMPLETE_EVENT
 *
 * @note This is an internal function of this BLE driver and normally it shall not be called by the end users directly
 *
 * @param [in] conn_id Connection handle of the link for which HCI_DISCONNECTION_COMPLETE_EVENT is indicated
 * @param [in] reason HCI disconnect reason code. See Core Specification Volume 1, Part F
 * @return SID_ERROR_NONE on success
 */
sid_error_t sid_stm32wba_ble_adapter_user_on_ble_disconnected(const uint16_t conn_id, const uint8_t reason);

/**
 * @brief Driver's internal callback for ACI_ATT_EXCHANGE_MTU_RESP_EVENT
 *
 * @note This is an internal function of this BLE driver and normally it shall not be called by the end users directly
 *
 * @param [in] conn_id Connection handle of the link for which ACI_ATT_EXCHANGE_MTU_RESP_EVENT is indicated
 * @return SID_ERROR_NONE on success
 */
sid_error_t sid_stm32wba_ble_adapter_user_on_ble_mtu_changed(const uint16_t conn_id);

/**
 * @brief Driver's internal callback for HCI_LE_CONNECTION_UPDATE_COMPLETE_EVENT
 *
 * @note This is an internal function of this BLE driver and normally it shall not be called by the end users directly
 *
 * @param [in] conn_id Connection handle of the link for which HCI_LE_CONNECTION_UPDATE_COMPLETE_EVENT is indicated
 * @return SID_ERROR_NONE on success
 */
sid_error_t sid_stm32wba_ble_adapter_user_on_conn_param_changed(const uint16_t conn_id);

/**
 * @brief Driver's internal callback for HCI_LE_REMOTE_CONNECTION_PARAMETER_REQUEST_EVENT
 *
 * @note This is an internal function of this BLE driver and normally it shall not be called by the end users directly
 *
 * @param [in] conn_id Connection handle of the link for which HCI_LE_REMOTE_CONNECTION_PARAMETER_REQUEST_EVENT is indicated
 * @param [in]  proposed_params Connection parameters to be validated and adjusted if needed
 * @param [out] out_accepted_params Validated and adjusted connection parameters that can be accepted
 * @param [out] out_le_resp_code BLE_STATUS_SUCCESS if the method managed to validate and adjust parameters, otherwise a BLE status coded to be reported to the remote peer as a reason for rejecting suggested parameters
 * @return SID_ERROR_NONE on success
 */
sid_error_t sid_stm32wba_ble_adapter_user_on_conn_params_update_req(const uint16_t conn_id, const sid_ble_ext_proposed_conn_params_t * const proposed_params,
                                                                    sid_ble_ext_accepted_conn_params_t * const out_accepted_params, tBleStatus * const out_le_resp_code);

/**
 * @brief Driver's internal callback for HCI_ENCRYPTION_CHANGE_EVENT
 *
 * @note This is an internal function of this BLE driver and normally it shall not be called by the end users directly
 *
 * @param [in] conn_id Connection handle of the link for which HCI_ENCRYPTION_CHANGE_EVENT is indicated
 * @return SID_ERROR_NONE on success
 */
sid_error_t sid_stm32wba_ble_adapter_user_on_encryption_changed(const uint16_t conn_id);

/**
 * @brief Driver's internal callback for ACI_GAP_KEYPRESS_NOTIFICATION_EVENT
 *
 * @note This is an internal function of this BLE driver and normally it shall not be called by the end users directly
 *
 * @param [in] conn_id Connection handle of the link for which ACI_GAP_KEYPRESS_NOTIFICATION_EVENT is indicated
 * @return SID_ERROR_NONE on success
 */
sid_error_t sid_stm32wba_ble_adapter_user_on_pairing_keypress_notification(const uint16_t conn_id, const uint8_t notification_type);

/**
 * @brief Driver's internal callback for ACI_GAP_PASS_KEY_REQ_EVENT
 *
 * @note This is an internal function of this BLE driver and normally it shall not be called by the end users directly
 *
 * @param [in] conn_id Connection handle of the link for which ACI_GAP_PASS_KEY_REQ_EVENT is indicated
 * @return SID_ERROR_NONE on success
 */
sid_error_t sid_stm32wba_ble_adapter_user_on_pairing_pass_key_request(const uint16_t conn_id, uint32_t * const out_passkey);

/**
 * @brief Driver's internal callback for ACI_GAP_NUMERIC_COMPARISON_VALUE_EVENT
 *
 * @note This is an internal function of this BLE driver and normally it shall not be called by the end users directly
 *
 * @param [in] conn_id Connection handle of the link for which ACI_GAP_NUMERIC_COMPARISON_VALUE_EVENT is indicated
 * @return SID_ERROR_NONE on success
 */
sid_error_t sid_stm32wba_ble_adapter_user_on_pairing_numeric_comparison_value(const uint16_t conn_id, const uint32_t numeric_value, uint32_t * const out_numeric_value_matches);

/**
 * @brief Driver's internal callback for ACI_GAP_PAIRING_COMPLETE_EVENT
 *
 * @note This is an internal function of this BLE driver and normally it shall not be called by the end users directly
 *
 * @param [in] conn_id Connection handle of the link for which ACI_GAP_PAIRING_COMPLETE_EVENT is indicated
 * @return SID_ERROR_NONE on success
 */
sid_error_t sid_stm32wba_ble_adapter_user_on_pairing_complete(const uint16_t conn_id, tBleStatus status, uint8_t reason);

/**
 * @brief Driver's internal callback for ACI_GAP_AUTHORIZATION_REQ_EVENT
 *
 * @note This is an internal function of this BLE driver and normally it shall not be called by the end users directly
 *
 * @param [in] conn_id Connection handle of the link for which ACI_GAP_AUTHORIZATION_REQ_EVENT is indicated
 * @return SID_ERROR_NONE on success
 */
sid_error_t sid_stm32wba_ble_adapter_user_on_authorization_req(const uint16_t conn_id);

#ifdef __cplusplus
}
#endif

#endif /* __SID_PAL_BLE_ADAPTER_STM32WBA_USER_GAP_GATT_H_ */
