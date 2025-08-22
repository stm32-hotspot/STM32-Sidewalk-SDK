/**
  ******************************************************************************
  * @file    sid_pal_ble_adapter_stm32wba_private_defs.h
  * @brief   Private definitions for STM32WBA-specific Sidewalk BLE driver
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

#ifndef __SID_PAL_BLE_ADAPTER_STM32WBA_PRIVATE_DEFS_H_
#define __SID_PAL_BLE_ADAPTER_STM32WBA_PRIVATE_DEFS_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "bluetooth_hci_defs.h"
#include "sid_pal_ble_adapter_stm32wba_ext_ifc.h"

/* RTOS */
#include <cmsis_os2.h>

/* STM32 BLE stack */
#include <ble_types.h>
#include <ble_std.h>
#include <svc_ctl.h>

/* Sidewalk interfaces */
#include <sid_ble_config_ifc.h>
#include <sid_error.h>

/* Utilities */
#include <stm32_timer.h>

/* Exported constants --------------------------------------------------------*/

#define SID_STM32_BLE_HANDLE_INVALID_VALUE                                  (0xFFFFu) /*!< Used to identify invalid BLE handles: connections, services, characteristics, etc. This value is not part of the BLE Core Specification, but it is out of range of valid handles */

#define SID_STM32_BLE_GATT_SERVICE_HANDLE                                   (0x0001u) /*!< Generic Attribute (GATT) Service. See Volume 3, Part G, Section 7. On STM32 platform this service (if present) has a fixed handle */
#define SID_STM32_BLE_GATT_SVC_SERVICE_CHANGED_CHAR_HANDLE                  (0x0002u) /*!< Service Changed characteristic of the GATT service. See Volume 3, Part G, Section 7.1. On STM32 platform this characteristic (if present) has a fixed handle */

#define SID_STM32_BLE_CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET                 (1u)
#define SID_STM32_BLE_CHARACTERISTIC_CCCD_ATTRIBUTE_OFFSET                  (2u)
#define SID_STM32_BLE_CHARACTERISTIC_CCCD_ATTRIBUTE_SIZE                    (2u)
#define SID_STM32_BLE_CHARACTERISTIC_CCCD_NOTIFY_BIT_MASK                   (0x0001u)
#define SID_STM32_BLE_CHARACTERISTIC_CCCD_INDICATE_BIT_MASK                 (0x0002u)

#define SID_STM32_BLE_PPCP_CHAR_HANDLE_OFFSET                               (2u) /* as per https://www.st.com/resource/en/application_note/an5270-stm32wb-bluetooth-low-energy-wireless-interface-stmicroelectronics.pdf, section 2.4.10 */
#define SID_STM32_BLE_PPCP_CHAR_VALUE_LENGTH                                (8u)
#define SID_STM32_BLE_PPCP_CHAR_CONN_INTERVAL_MIN_OFFSET                    (0u)
#define SID_STM32_BLE_PPCP_CHAR_CONN_INTERVAL_MAX_OFFSET                    (2u)
#define SID_STM32_BLE_PPCP_CHAR_CONN_LATENCY_OFFSET                         (4u)
#define SID_STM32_BLE_PPCP_CHAR_CONN_TIMEOUT_OFFSET                         (6u)

#define SID_STM32_BLE_MAX_ADVERTISING_BUFFER_SIZE                           (31u)
#define SID_STM32_BLE_CHAR_ENC_KEY_MINIMUM_LENGTH                           (0x10u)

#define SID_STM32_BLE_ACI_CHARACTERISTIC_VALUE_UPDATE_SIZE_LIMIT            (251u) /* ACI calls use 1 byte to specify data size and 4 bytes are used for internal data, so maximum we can pass (255 - 4 = 251) bytes */
#define SID_STM32_BLE_ACI_CHARACTERISTIC_VALUE_UPDATE_TYPE_DO_NOT_NOTIFY    (0x00u)
#define SID_STM32_BLE_ACI_CHARACTERISTIC_VALUE_UPDATE_TYPE_NOTIFY           (0x01u)

#define SID_STM32_BLE_ACI_GATT_ATTRIBUTE_MODIFIED_MORE_DATA_PENDING_FLAG    (1u << 15)

/* ACI GAP event mask bits */
#define SID_STM32_BLE_ACI_GAP_LIMITED_DISCOVERABLE_EVENT                    ((uint16_t)0x0001u)
#define SID_STM32_BLE_ACI_GAP_PAIRING_COMPLETE_EVENT                        ((uint16_t)0x0002u)
#define SID_STM32_BLE_ACI_GAP_PASS_KEY_REQ_EVENT                            ((uint16_t)0x0004u)
#define SID_STM32_BLE_ACI_GAP_AUTHORIZATION_REQ_EVENT                       ((uint16_t)0x0008u)
#define SID_STM32_BLE_ACI_GAP_BOND_LOST_EVENT                               ((uint16_t)0x0020u)
#define SID_STM32_BLE_ACI_GAP_PROC_COMPLETE_EVENT                           ((uint16_t)0x0080u)
#define SID_STM32_BLE_ACI_L2CAP_CONNECTION_UPDATE_REQ_EVENT                 ((uint16_t)0x0100u)
#define SID_STM32_BLE_ACI_L2CAP_CONNECTION_UPDATE_RESP_EVENT                ((uint16_t)0x0200u)
#define SID_STM32_BLE_ACI_L2CAP_PROC_TIMEOUT_EVENT                          ((uint16_t)0x0400u)
#define SID_STM32_BLE_ACI_GAP_ADDR_NOT_RESOLVED_EVENT                       ((uint16_t)0x0800u)

/* ACI GATT event mask bits */
#define SID_STM32_BLE_ACI_GATT_ATTRIBUTE_MODIFIED_EVENT_MASK                ((uint32_t)0x00000001u)
#define SID_STM32_BLE_ACI_GATT_PROC_TIMEOUT_EVENT_MASK                      ((uint32_t)0x00000002u)
#define SID_STM32_BLE_ACI_ATT_EXCHANGE_MTU_RESP_EVENT_MASK                  ((uint32_t)0x00000004u)
#define SID_STM32_BLE_ACI_ATT_FIND_INFO_RESP_EVENT_MASK                     ((uint32_t)0x00000008u)
#define SID_STM32_BLE_ACI_ATT_FIND_BY_TYPE_VALUE_RESP_EVENT_MASK            ((uint32_t)0x00000010u)
#define SID_STM32_BLE_ACI_ATT_READ_BY_TYPE_RESP_EVENT_MASK                  ((uint32_t)0x00000020u)
#define SID_STM32_BLE_ACI_ATT_READ_RESP_EVENT_MASK                          ((uint32_t)0x00000040u)
#define SID_STM32_BLE_ACI_ATT_READ_BLOB_RESP_EVENT_MASK                     ((uint32_t)0x00000080u)
#define SID_STM32_BLE_ACI_ATT_READ_MULTIPLE_RESP_EVENT_MASK                 ((uint32_t)0x00000100u)
#define SID_STM32_BLE_ACI_ATT_READ_BY_GROUP_TYPE_RESP_EVENT_MASK            ((uint32_t)0x00000200u)
#define SID_STM32_BLE_ACI_ATT_PREPARE_WRITE_RESP_EVENT_MASK                 ((uint32_t)0x00000800u)
#define SID_STM32_BLE_ACI_ATT_EXEC_WRITE_RESP_EVENT_MASK                    ((uint32_t)0x00001000u)
#define SID_STM32_BLE_ACI_GATT_INDICATION_EVENT_MASK                        ((uint32_t)0x00002000u)
#define SID_STM32_BLE_ACI_GATT_NOTIFICATION_EVENT_MASK                      ((uint32_t)0x00004000u)
#define SID_STM32_BLE_ACI_GATT_ERROR_RESP_EVENT_MASK                        ((uint32_t)0x00008000u)
#define SID_STM32_BLE_ACI_GATT_PROC_COMPLETE_EVENT_MASK                     ((uint32_t)0x00010000u)
#define SID_STM32_BLE_ACI_GATT_DISC_READ_CHAR_BY_UUID_RESP_EVENT_MASK       ((uint32_t)0x00020000u)
#define SID_STM32_BLE_ACI_GATT_TX_POOL_AVAILABLE_EVENT_MASK                 ((uint32_t)0x00040000u)
#define SID_STM32_BLE_ACI_GATT_READ_EXT_EVENT_MASK                          ((uint32_t)0x00100000u)
#define SID_STM32_BLE_ACI_GATT_INDICATION_EXT_EVENT_MASK                    ((uint32_t)0x00200000u)
#define SID_STM32_BLE_ACI_GATT_NOTIFICATION_EXT_EVENT_MASK                  ((uint32_t)0x00400000u)

#define SID_STM32_BLE_LL_MUTEX_WAIT_TIMEOUT_ON_INIT_TICKS                   (1000u)   /*!< Wait time limit (in OS ticks) to acquire BLE LL mutex on stack initialization */
#define SID_STM32_BLE_LL_MUTEX_WAIT_TIMEOUT_ON_DEINIT_TICKS                 (1000u)   /*!< Wait time limit (in OS ticks) to acquire BLE LL mutex on stack deinitialization. Normally the mutex will be free on entry, this is just a protection from a deadlock */

#define SID_STM32_BLE_ADD_TO_LIST_MODE_APPEND_RESOLVING_ONLY                (0x00u)
#define SID_STM32_BLE_ADD_TO_LIST_MODE_CLEAR_SET_RESOLVING_ONLY             (0x01u)
#define SID_STM32_BLE_ADD_TO_LIST_MODE_APPEND_FILT_ACCEPT_ONLY              (0x02u)
#define SID_STM32_BLE_ADD_TO_LIST_MODE_CLEAR_SET_FILT_ACCEPT_ONLY           (0x03u)
#define SID_STM32_BLE_ADD_TO_LIST_MODE_APPEND_BOTH                          (0x04u)
#define SID_STM32_BLE_ADD_TO_LIST_MODE_CLEAR_SET_BOTH                       (0x05u)

/* Exported macro ------------------------------------------------------------*/

#define SID_STM32_BLE_LEGACY_ADV_TYPE_TO_EXT_ADV_EVT_PROPS(__ADV_TYPE__)    (\
                                                                                (__ADV_TYPE__) == ADV_IND                      ? (HCI_ADV_EVENT_PROP_CONNECTABLE | HCI_ADV_EVENT_PROP_SCANNABLE | HCI_ADV_EVENT_PROP_LEGACY) :\
                                                                                (__ADV_TYPE__) == LOW_DUTY_CYCLE_DIRECTED_ADV  ? (HCI_ADV_EVENT_PROP_CONNECTABLE | HCI_ADV_EVENT_PROP_DIRECTED | HCI_ADV_EVENT_PROP_LEGACY) :\
                                                                                (__ADV_TYPE__) == HIGH_DUTY_CYCLE_DIRECTED_ADV ? (HCI_ADV_EVENT_PROP_CONNECTABLE | HCI_ADV_EVENT_PROP_DIRECTED | HCI_ADV_EVENT_PROP_HDC_DIRECTED | HCI_ADV_EVENT_PROP_LEGACY) :\
                                                                                (__ADV_TYPE__) == ADV_SCAN_IND                 ? (HCI_ADV_EVENT_PROP_SCANNABLE | HCI_ADV_EVENT_PROP_LEGACY) :\
                                                                                (__ADV_TYPE__) == ADV_NONCONN_IND              ? (HCI_ADV_EVENT_PROP_LEGACY) :\
                                                                                0u \
                                                                            )

#define SID_STM32_BLE_ATT_LEN_LIMIT(__MTU__)                                ((__MTU__) - 3u) /*!< Maximum attribute value size when the user performs characteristic operations (notification/write max. size is ATT_MTU-3) */

/* Exported types ------------------------------------------------------------*/

typedef SVC_CTL_p_EvtHandler_t sid_pal_ble_prv_svc_evt_handler_t;

typedef enum {
    BADVS_ADVERTISEMENT_OFF  = 0,
    BADVS_ADVERTISEMENT_FAST = 1,
    BADVS_ADVERTISEMENT_SLOW = 2,
} sid_pal_ble_prv_adv_state_t;

typedef enum {
    SPBP_OPERATING_MODE_OFF        = 0, /*!< The driver is inactive, BLE hardware is free */
    SPBP_OPERATING_MODE_SIDEWALK   = 1, /*!< BLE stack is initialized to run Sidewalk link */
    SPBP_OPERATING_MODE_USER       = 2, /*!< BLE stack is initialized to run user BLE profiles */
    SPBP_OPERATING_MODE_CONCURRENT = 3, /*!< BLE stack is initialized in concurrency mode, allowing both Sidewalk and user profiles to run in parallel */
} sid_pal_ble_prv_operating_mode_t;

/**
 * @brief Mapping of the Core Specification connection role type definitions for HCI_LE_Enhanced_Connection_Complete event
 */
typedef enum {
    SPBP_CONN_CENTRAL = SID_BLE_HCI_CONN_COMPLETE_ROLE_CENTRAL,
    SPBP_CONN_PERIPH  = SID_BLE_HCI_CONN_COMPLETE_ROLE_PERIPHERAL,
} sid_pal_ble_prv_conn_role_t;

typedef union {
    uint64_t dwords[((SID_STM32_BLE_ADDRESS_LENGTH) + (sizeof(uint64_t) - 1u)) / sizeof(uint64_t)];
    uint32_t words [((SID_STM32_BLE_ADDRESS_LENGTH) + (sizeof(uint32_t) - 1u)) / sizeof(uint32_t)];
    uint8_t  bytes [SID_STM32_BLE_ADDRESS_LENGTH];
} sid_pal_ble_prv_bt_addr_buffer_t;

typedef union {
    uint32_t words[((CONFIG_DATA_IR_LEN) + (sizeof(uint32_t) - 1u)) / sizeof(uint32_t)];
    uint8_t  bytes[CONFIG_DATA_IR_LEN];
} sid_pal_ble_prv_irk_buffer_t;

typedef union {
    uint32_t words[((CONFIG_DATA_ER_LEN) + (sizeof(uint32_t) - 1u)) / sizeof(uint32_t)];
    uint8_t  bytes[CONFIG_DATA_ER_LEN];
} sid_pal_ble_prv_erk_buffer_t;

typedef struct {
    sid_ble_ext_connection_ctx_t     public_ctx;                                  /*!< Publicly exposed part of the connection context */
    uint16_t                         adv_set_id;                                  /*!< Associated advertisement set - valid only for peripheral connections and only if the host supports extended advertisements */
    sid_pal_ble_prv_conn_role_t      role;                                        /*!< Connection role (peripheral or central) */
    osSemaphoreId_t                  gap_cmd_lock;                                /*!< Lock object to wait for GAP commands completion */
} sid_pal_ble_prv_connection_ctx_t;

/**
 * Security parameters structure
 */
typedef struct {
    uint8_t  io_capability;    /*!< IO capability of the device */
    uint8_t  keypress_support; /*!< Specifies if the device is capable of a key press */
    uint8_t  mitm_mode;        /*!< Man In the Middle protection mode */
    uint8_t  sc_mode;          /*!< Secure Connection mode */
    uint8_t  enable_bonding;   /*!< Bonding support */
    uint8_t  use_fixed_pin;    /*!< 0x00: use the fixed pin specified in this structure during pairing, 0x01: generate ACI_GAP_PASS_KEY_REQ event for application to generate a unique pin */
    uint32_t fixed_pin;        /*!< Fixed pin to be used in the pairing process if disable_fixed_pin is set to 0x00 */

  /**
   * minimum encryption key size requirement
   */
  uint8_t encryptionKeySizeMin;

  /**
   * maximum encryption key size requirement
   */
  uint8_t encryptionKeySizeMax;


  /**
   * this flag indicates whether the host has to initiate
   * the security, wait for pairing or does not have any security
   * requirements.
   * 0x00 : no security required
   * 0x01 : host should initiate security by sending the slave security
   *        request command
   * 0x02 : host need not send the slave security request but it
   * has to wait for pairing to complete before doing any other
   * processing
   */
  uint8_t initiateSecurity;

  uint8_t identity_address_type; /*!< Identity address type used for authentication */
} sid_pal_ble_security_params_t;

typedef struct {
    sid_pal_ble_prv_svc_evt_handler_t handler;
    uint32_t                          ref_count;
} sid_pal_ble_prv_evt_hndlr_desc_t;

typedef struct {
    /* Generic context data */
    uint32_t                          init_counter;               /*!< Driver initializations counter. Actual init/deinit takes place only when this counter is at zero */
    sid_pal_ble_prv_operating_mode_t  operating_mode;             /*!< Operation state of the driver (e.g. Turned off, Running Sidewalk only, etc. )*/
    uint16_t                          global_att_mtu_size;        /*!< THe maximum allowed ATT MTU size, applies to all connections */
    const sid_ble_adapter_ext_cfg_t * cfg;                        /*!< User-supplied configuration of the BLE driver */

#if (SID_STM32_BLE_EXTENDED_ADV_SUPPORTED == 0)
    UTIL_TIMER_Object_t               adv_timer;                  /*!< Timer to control advertisement duration in legacy adv mode */
    osThreadId_t                      advertising_cmd_task;
    osMessageQueueId_t                advertising_cmd_queue;
#endif /* SID_STM32_BLE_EXTENDED_ADV_SUPPORTED */

    /* GAP context */
    sid_pal_ble_security_params_t     security_cfg;               /*!< Security requirements of the host */
    uint16_t                          gap_service_handle;         /*!< GAP service handle */
    uint16_t                          gap_dev_name_char_handle;   /*!< GAP Device Name characteristic handle */
    uint16_t                          gap_appearance_char_handle; /*!< GAP Appearance characteristic handle */
    uint16_t                          gap_ppcp_char_handle;       /*!< GAP Peripheral Preferred Connection Parameters characteristic handle */

    /* Generic connection contexts for centralized connection management */
    sid_pal_ble_prv_connection_ctx_t  conn_ctxs[SID_STM32_BLE_HOST_MAX_NUM_LINK]; /*!< Generic connection info for connection management */

    /* Private registry of BLE SVC Event Handlers */
    sid_pal_ble_prv_evt_hndlr_desc_t  svc_event_handlers[SID_STM32_BLE_PRV_SVC_HANDLERS_MAX_NUM];
} sid_pal_ble_adapter_ctx_t;

#ifdef __cplusplus
}
#endif

#endif /* __SID_PAL_BLE_ADAPTER_STM32WBA_PRIVATE_DEFS_H_ */
