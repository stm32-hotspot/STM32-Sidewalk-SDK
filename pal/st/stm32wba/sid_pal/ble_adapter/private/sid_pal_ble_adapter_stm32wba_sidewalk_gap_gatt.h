/**
  ******************************************************************************
  * @file    sid_pal_ble_adapter_stm32wba_sidewalk_gap_gatt.h
  * @brief   Sidewalk-specific GAP and GATT handling
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

#ifndef __SID_PAL_BLE_ADAPTER_STM32WBA_SIDEWALK_GAP_GATT_H_
#define __SID_PAL_BLE_ADAPTER_STM32WBA_SIDEWALK_GAP_GATT_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "sid_pal_ble_adapter_stm32wba_private_defs.h"

/* Sidewalk interfaces */
#include <sid_error.h>
#include <sid_ble_config_ifc.h>
#include <sid_pal_ble_adapter_ifc.h>

/* Exported types ------------------------------------------------------------*/

typedef struct {
    sid_ble_cfg_gatt_profile_t *        service_def;
    sid_pal_ble_prv_svc_evt_handler_t   svc_event_handler;
} sid_ble_adapter_sid_gatt_svc_init_t;

typedef struct {
    sid_ble_cfg_characteristics_t characteristic_info;
    uint16_t                      characteristic_handle;
} sid_ble_adapter_sid_gatt_char_ctx_t;

typedef struct {
    sid_ble_cfg_service_t                 service_info;
    uint16_t                              service_handle;
    uint16_t                              num_characteristics;
    sid_ble_adapter_sid_gatt_char_ctx_t * characteristic_context;
    sid_pal_ble_prv_svc_evt_handler_t     event_handler;
} sid_ble_adapter_sid_gatt_svc_ctx_t;

typedef struct {
    sid_ble_adapter_sid_gatt_svc_ctx_t service_context;
    uint16_t                           outbox_char_handle;
    uint16_t                           outbox_char_index;
    uint16_t                           inbox_char_handle;
    uint16_t                           inbox_char_index;
    bool                               is_notification_enabled; /*!< Indicates if the peer is subscribed to the AMA Outbox notifications */

    size_t                             inbox_valid_length;      /*!< Length of the valid data in the AMA Inbox buffer */
    uint8_t *                          inbox_buf;               /*!< Pointer to the data buffer for the AMA Inbox characteristic */
} sid_ble_ama_service_context_t;

typedef struct {
    sid_pal_ble_prv_connection_ctx_t *      conn_ctx;
    const sid_ble_config_t *                ble_cfg;
    Adv_Set_t                               adv_set;
    const sid_pal_ble_adapter_callbacks_t * event_callbacks;
    sid_pal_ble_prv_adv_state_t             advertising_state;
    uint8_t                                 init_done;
    sid_ble_ama_service_context_t           ama_ctx;
} sid_pal_ble_sidewalk_link_ctx_t;

/* Exported functions ------------------------------------------------------- */

/**
 * @brief Callback for Sidewalk-related BLE advertisement timeout
 *
 * @return SID_ERROR_NONE on success
 */
sid_error_t sid_stm32wba_ble_adapter_sidewalk_on_adv_timeout(void);

/**
 * @brief Callback for successful start of Sidewalk link BLE connection
 *
 * @param [in] conn_ctx Pointer to the connection context data (e.g. connectio handle, connection parameters, etc.)
 * @return SID_ERROR_NONE on success
 */
sid_error_t sid_stm32wba_ble_adapter_sidewalk_on_ble_connected(sid_pal_ble_prv_connection_ctx_t * const conn_ctx);

/**
 * @brief Callback for termination of the Sidewalk link BLE connection
 *
 * @param [in] reason BLE disconnection reason code as per Core Specification, Volume 1, Part F
 */
void sid_stm32wba_ble_adapter_sidewalk_on_ble_disconnected(const uint8_t reason);

/**
 * @brief Callback for MTU size change for the Sidewalk link BLE connection
 *
 * @note The updated MTU value can be accessed in the associated connection context, it's not needed to pass it as a parameter
 */
void sid_stm32wba_ble_adapter_sidewalk_on_ble_mtu_changed(void);

/**
 * @brief Callback for connection parameter update request for the Sidewalk link BLE connection
 *
 * @param [in]  proposed_params Connection parameters to be validated and adjusted if needed
 * @param [out] out_accepted_params Validated and adjusted connection parameters that can be accepted
 * @param [out] out_le_resp_code BLE_STATUS_SUCCESS if the method managed to validate and adjust parameters, otherwise a BLE status coded to be reported to the remote peer as a reason for rejecting suggested parameters
 * @return SID_ERROR_NONE on success
 */
sid_error_t sid_stm32wba_ble_adapter_sidewalk_on_ble_conn_params_update_req(const sid_ble_ext_proposed_conn_params_t * const proposed_params,
                                                                           sid_ble_ext_accepted_conn_params_t * const out_accepted_params, tBleStatus * const out_le_resp_code);

/**
 * @brief Callback for connection parameter update completed event for the Sidewalk link BLE connection
 *
 * @note The updated connection parameters can be accessed in the associated connection context, it's not needed to pass it as a parameter
 */
sid_error_t sid_stm32wba_ble_adapter_sidewalk_on_ble_conn_params_changed(void);

#ifdef __cplusplus
}
#endif

#endif /* __SID_PAL_BLE_ADAPTER_STM32WBA_SIDEWALK_GAP_GATT_H_ */
