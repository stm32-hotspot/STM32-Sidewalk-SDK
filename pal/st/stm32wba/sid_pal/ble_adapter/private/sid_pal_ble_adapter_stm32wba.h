/**
  ******************************************************************************
  * @file    sid_pal_ble_adapter_stm32wba.h
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

#ifndef __SID_PAL_BLE_ADAPTER_STM32WBA_H_
#define __SID_PAL_BLE_ADAPTER_STM32WBA_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "sid_pal_ble_adapter_stm32wba_private_defs.h"

/* BLE stack */
#include <ble_types.h>

/* Sidewalk interfaces */
#include <sid_ble_config_ifc.h>
#include <sid_error.h>

/* Exported types ------------------------------------------------------------*/

/**
  * HCI Event Packet Types
  */
typedef __PACKED_STRUCT {
    uint32_t *next;
    uint32_t *prev;
} BleEvtPacketHeader_t;

typedef __PACKED_STRUCT {
    uint8_t   evtcode;
    uint8_t   plen;
    uint8_t   payload[1];
} BleEvt_t;

typedef __PACKED_STRUCT {
    uint8_t   type;
    BleEvt_t  evt;
} BleEvtSerial_t;

typedef __PACKED_STRUCT __ALIGNED(4) {
    BleEvtPacketHeader_t  header;
    BleEvtSerial_t        evtserial;
} BleEvtPacket_t;

typedef enum
{
  PROC_GAP_GEN_PHY_TOGGLE,
  PROC_GAP_GEN_CONN_TERMINATE,
  PROC_GATT_EXCHANGE_CONFIG,
  /* USER CODE BEGIN ProcGapGeneralId_t*/

  /* USER CODE END ProcGapGeneralId_t */
}ProcGapGeneralId_t;

/* Exported functions ---------------------------------------------*/

/**
 * @brief Generic BLE initialization routine
 *
 * This methods brings up common BLE hardware and software components regardless of the targeted BLE operational mode
 *
 * @warning This is a private API and it shall not be used by the user app directly under any circumstances. Failing to comply will result in undefined behavior
 *
 * @param [in] init_type   Targeted BLE driver operational mode (e.g. Sidewalk-only, concurrent, etc.)
 * @param [in] sid_ble_cfg Sidewalk BLE link config supplied by the Sidewalk stack
 * @return SID_ERROR_NONE on success, error code otherwise
 */
sid_error_t sid_stm32wba_ble_adapter_prv_generic_init(const sid_pal_ble_prv_operating_mode_t init_type, const sid_ble_config_t * const sid_ble_cfg);

/**
 * @brief Generic BLE deinitialization routine
 *
 * This methods brings down common BLE hardware and software components
 *
 * @warning This is a private API and it shall not be used by the user app directly under any circumstances. Failing to comply will result in undefined behavior
 *
 * @return SID_ERROR_NONE on success, error code otherwise
 */
sid_error_t sid_stm32wba_ble_adapter_prv_generic_deinit(void);

/**
 * @brief Generic BLE advertisement start routine
 *
 * This methods performs the necessary actions to start BLE advertisement regardless if extended advertising is used or not
 *
 * @warning This is a private API and it shall not be used by the user app directly under any circumstances. Failing to comply will result in undefined behavior
 *
 * @param [in] adv_set      Advertisement set to be started. This parameter shall point to a valid advertisement set descriptor even ff extended advertising is not used
 * @return SID_ERROR_NONE on success, error code otherwise
 */
sid_error_t sid_stm32wba_ble_adapter_prv_generic_start_advertisement(Adv_Set_t * const adv_set);

/**
 * @brief Generic BLE advertisement stop routine
 *
 * This methods performs the necessary actions to stop BLE advertisement regardless if extended advertising is used or not
 *
 * @warning This is a private API and it shall not be used by the user app directly under any circumstances. Failing to comply will result in undefined behavior
 *
 * @param [in] adv_set_handle Advertisement set handle of the advertisement set to be stopped. If extended advertising is not used this value is irrelevant
 * @return SID_ERROR_NONE on success, error code otherwise
 */
sid_error_t sid_stm32wba_ble_adapter_prv_generic_stop_advertisement(const uint32_t adv_set_handle);

/**
 * @brief Registers SVCCTL event handler with the local callback manager
 *
 * @note Don't use the WPAN's SVCCTL_RegisterSvcHandler() API because it does not allow to remove a handler without full BLE stack reset
 *
 * @param [in] gatt_svc_event_handler Handler to be added to the manager
 * @return SID_ERROR_NONE on success, error code otherwise
 */
sid_error_t sid_stm32wba_ble_adapter_prv_generic_register_svc_handler(const SVC_CTL_p_EvtHandler_t gatt_svc_event_handler);

/**
 * @brief Removes SVCCTL event handler from the local callback manager
 *
 * @param [in] gatt_svc_event_handler Handler to be removed from the manager
 * @return SID_ERROR_NONE on success, error code otherwise
 */
sid_error_t sid_stm32wba_ble_adapter_prv_generic_deregister_svc_handler(const SVC_CTL_p_EvtHandler_t gatt_svc_event_handler);

/**
 * @brief Prepares GAP command response wait
 *
 * @note Use with care, normally user app does not need this API
 *
 * @param conn_ctx BLE connection context
 */
void sid_stm32wba_ble_adapter_prv_gap_cmd_arm_resp_wait(const sid_pal_ble_prv_connection_ctx_t * const conn_ctx);

/**
 * @brief Blocks the execution until GAP finishes the processing of the GAP command
 *
 * @note Not all GAP event handlers provide responses. Calling this function for non-supported GAP command will result in a deadlock.
 *
 * @note Use with care, normally user app does not need this API
 *
 * @param conn_ctx   BLE connection context
 * @return SID_ERROR_NONE on success
 */
sid_error_t sid_stm32wba_ble_adapter_prv_gap_cmd_resp_wait(const sid_pal_ble_prv_connection_ctx_t * const conn_ctx);

/**
 * @brief   Generates new random BLE address of the selected type (resolvable, non-resolvable, etc.) and sets it in the controller
 *
 * @warning This is private API and shall not be used outside the Sidewalk BLE driver
 *
 * @param [out] out_addr  Optional buffer to receive the new address that was applied to the controller. Can be NULL
 * @param [in]  addr_type Type of the BLE address to generate and set in the controller
 * @return  Pointer to the configuration data
 */
sid_error_t sid_stm32wba_ble_adapter_prv_rotate_random_mac_address(sid_pal_ble_prv_bt_addr_buffer_t * const out_addr, const sid_ble_cfg_mac_address_type_t addr_type);

#ifdef __cplusplus
}
#endif

#endif /* __SID_PAL_BLE_ADAPTER_STM32WBA_H_ */
