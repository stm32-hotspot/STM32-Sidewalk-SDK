/**
  ******************************************************************************
  * @file    sid_pal_ble_adapter_stm32wba_ext_ifc.h
  * @brief   STM32WBA-specific extended Sidewalk BLE interface
  * 
  * This interface expands sid_pal_ble_adapter_ifc with additional functionality
  * that is not part of the default Sidewalk interface. This module is hardware-
  * specific and targets STM32WBA platform only
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

#ifndef __SID_PAL_BLE_ADAPTER_STM32WBA_EXT_IFC_H_
#define __SID_PAL_BLE_ADAPTER_STM32WBA_EXT_IFC_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Sidewalk interfaces */
#include <sid_ble_link_config_ifc.h>
#include <sid_error.h>
#include "sid_pal_ble_adapter_stm32wba_config.h"

/* BLE stack */
#include "ble_types.h"

/* Utilities */
#include <cmsis_compiler.h>
#include <stm_list.h>

/* Exported constants --------------------------------------------------------*/

#define SID_STM32_BLE_SIDEWALK_VIRT_DEV_ID                 (0xEFu)                                         /*!< This ID is reserved to identify the advertisement set associated with the Sidewalk link and to identify virtual BLE device in the driver. User can utilize any ID in 0x00..0xEE range for their BLEprofiles/virtual devices */
#define SID_STM32_BLE_CUSTOM_VIRT_DEV_ID_MIN               (0x00u)                                         /*!< Minimum value for the user-defined virtual BLE device. This value must be aligned with BLE adverting handle as per Core Specification Volume 4, Part E, Section 7.8.53 */
#define SID_STM32_BLE_CUSTOM_VIRT_DEV_ID_MAX               (0xEEu)                                         /*!< Maximum value for the user-defined virtual BLE device. This value must be aligned with BLE adverting handle as per Core Specification Volume 4, Part E, Section 7.8.53 */

#define SID_STM32_BLE_ADV_TX_POWER_NO_PREFERENCE           (127)


#define SID_STM32_BLE_ATT_STATUS_SUCCESS                   (0x00u)
#define SID_STM32_BLE_ATT_STATUS_INVALID_HANDLE            (0x01u)                                         /*!< The attribute handle given was not valid on this server */
#define SID_STM32_BLE_ATT_STATUS_READ_NOT_PERMITTED        (0x02u)                                         /*!< The attribute cannot be read */
#define SID_STM32_BLE_ATT_STATUS_WRITE_NOT_PERMITTED       (0x03u)                                         /*!< The attribute cannot be written */
#define SID_STM32_BLE_ATT_STATUS_INVALID_PDU               (0x04u)                                         /*!< The attribute PDU was invalid */
#define SID_STM32_BLE_ATT_STATUS_INSUFFICIENT_AUTHEN       (0x05u)                                         /*!< The attribute requires authentication before it can be read or written */
#define SID_STM32_BLE_ATT_STATUS_REQ_NOT_SUPPORTED         (0x06u)                                         /*!< ATT Server does not support the request received from the client */
#define SID_STM32_BLE_ATT_STATUS_INVALID_OFFSET            (0x07u)                                         /*!< Offset specified was past the end of the attribute */
#define SID_STM32_BLE_ATT_STATUS_INSUFFICIENT_AUTHOR       (0x08u)                                         /*!< The attribute requires authorization before it can be read or written */
#define SID_STM32_BLE_ATT_STATUS_PREPARE_QUEUE_FULL        (0x09u)                                         /*!< Too many prepare writes have been queued */
#define SID_STM32_BLE_ATT_STATUS_ATT_NOT_FOUND             (0x0Au)                                         /*!< No attribute found within the given attribute handle range */
#define SID_STM32_BLE_ATT_STATUS_ATT_NOT_LONG              (0x0Bu)                                         /*!< The attribute cannot be read using the ATT_READ_BLOB_REQ PDU */
#define SID_STM32_BLE_ATT_STATUS_ENC_KEY_TOO_SHORT         (0x0Cu)                                         /*!< The Encryption Key Size used for encrypting this link is too short */
#define SID_STM32_BLE_ATT_STATUS_INVALUID_ATT_VALUE_LENGTH (0x0Du)                                         /*!< The attribute value length is invalid for the operation */
#define SID_STM32_BLE_ATT_STATUS_UNLIKELY                  (0x0Eu)                                         /*!< The attribute request that was requested has encountered an error that was unlikely, and therefore could not be completed as requested */
#define SID_STM32_BLE_ATT_STATUS_INSUFFICIENT_ENC          (0x0Fu)                                         /*!< The attribute requires encryption before it can be read or written */
#define SID_STM32_BLE_ATT_STATUS_UNSUPPORTED_GROUP_TYPE    (0x10u)                                         /*!< The attribute type is not a supported grouping attribute as defined by a higher layer specification */
#define SID_STM32_BLE_ATT_STATUS_INSUFFICIENT_RESOURCES    (0x11u)                                         /*!< Insufficient Resources to complete the request */
#define SID_STM32_BLE_ATT_STATUS_DATABASE_OUT_OF_SYNC      (0x12u)                                         /*!< The server requests the client to rediscover the database */
#define SID_STM32_BLE_ATT_STATUS_VALUE_NOT_ALLOWED         (0x13u)                                         /*!< The attribute parameter value was not allowed */
#define SID_STM32_BLE_ATT_STATUS_APP_ERROR_MIN             (0x80u)                                         /*!< Application error code defined by a higher layer specification */
#define SID_STM32_BLE_ATT_STATUS_APP_ERROR_GENERIC         (0x80u)                                         /*!< Application error: generic error without any specific reason */
#define SID_STM32_BLE_ATT_STATUS_APP_ERROR_INVALID_STATE   (0x81u)                                         /*!< Application error: invalid logical state of the application */
#define SID_STM32_BLE_ATT_STATUS_APP_ERROR_WRONG_VIRT_DEV  (0x82u)                                         /*!< Application error: attribute is accessed via a wrong virtual device (e.g. the attribute belongs to virtual device A, but virtual device B tries to read or write it) */
#define SID_STM32_BLE_ATT_STATUS_APP_ERROR_EXCEEDS_LIMIT   (0x83u)                                         /*!< Application error: attempted attribute access tried to read/write more data that the limit for this attribute */
#define SID_STM32_BLE_ATT_STATUS_APP_ERROR_MAX             (0x9Fu)                                         /*!< Application error code defined by a higher layer specification */

#define SID_STM32_BLE_VIRTUAL_DEVICE_ID_INVALID            (0xFFu)                                         /*!< Explicitly invalid value for the virtual BLE device ID */

#define SID_STM32_BLE_RADIO_MAX_TX_POWER_NA                (20)                                            /*!< Maximum allowed BLE Tx power to meet FCC regulations */
#define SID_STM32_BLE_RADIO_MAX_TX_POWER_EU                (10)                                            /*!< Maximum allowed BLE Tx power to meet ETSI regulations */

/* Exported macro ------------------------------------------------------------*/

#define SID_STM32_BLE_MS_TO_CONN_INTERVAL_UNITS(__MS__)    ((uint32_t)(((float)(__MS__) / 1.25f) + 0.5f))  /*!< Converts milliseconds into BLE connection interval units (1.25ms per unit) */
#define SID_STM32_BLE_CONN_INTERVAL_UNITS_TO_MS(__IU__)    (((float)(__IU__) * 1.25f))                     /*!< Converts BLE connection interval units (1.25ms per unit) into milliseconds */

#define SID_STM32_BLE_MS_TO_CONN_SUPERVISION_UNITS(__MS__) ((uint32_t)(((float)(__MS__) / 10.f) + 0.5f))   /*!< Converts milliseconds into BLE connection supervision units (10ms per unit) */
#define SID_STM32_BLE_CONN_SUPERVISION_UNITS_TO_MS(__SU__) ((uint32_t)((uint32_t)(__SU__) * 10u))          /*!< Converts milliseconds into BLE connection supervision units (10ms per unit) */

#define SID_STM32_BLE_MS_TO_CONN_CE_LENGTH_UNITS(__MS__)   ((uint32_t)(((float)(__MS__) / 0.625f) + 0.5f))  /*!< Converts milliseconds into BLE connection CE length units (0.625ms per unit) */
#define SID_STM32_BLE_CONN_CE_LENGTH_UNITS_TO_MS(__IU__)   (((float)(__IU__) * 0.625f))                     /*!< Converts BLE connection CE length units (0.625ms per unit) into milliseconds */

#define SID_STM32_BLE_MS_TO_ADV_INTERVAL_UNITS(__MS__)     ((uint32_t)(((float)(__MS__) / 0.625f) + 0.5f)) /*!< Converts milliseconds into BLE advertisement interval units (0.625ms per unit) */
#define SID_STM32_BLE_ADV_INTERVAL_UNITS_TO_MS(__IU__)     (((float)(__IU__) * 0.625f))                    /*!< Converts BLE advertisement interval units (0.625ms per unit) into milliseconds */

#define SID_STM32_BLE_MS_TO_ADV_DURATION_UNITS(__MS__)     ((uint32_t)(((float)(__MS__) / 10.f) + 0.5f))   /*!< Converts milliseconds into BLE advertisement duration units (10ms per unit) */
#define SID_STM32_BLE_ADV_DURATION_UNITS_TO_MS(__DU__)     ((uint32_t)((uint32_t)(__DU__) * 10u))          /*!< Converts BLE advertisement duration units into milliseconds (10ms per unit) */

/* Exported constants --------------------------------------------------------*/

extern const sid_ble_config_t sid_ble_default_cfg;

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Mapping of the Core Specification peer address type definitions for HCI_LE_Enhanced_Connection_Complete event
 *
 * @note enum values follow Bluetooth Core Specification
 */
typedef enum {
    SPBP_PEER_ADDR_TYPE_PUBLIC      = 0x00u,                                      /*!< Public Device Address */
    SPBP_PEER_ADDR_TYPE_RANDOM      = 0x01u,                                      /*!< Random Device Address */
    SPBP_PEER_ADDR_TYPE_RPA_PUB_ID  = 0x02u,                                      /*!< Public Identity Address (Corresponds to Resolved Private Address) */
    SPBP_PEER_ADDR_TYPE_RPA_RAND_ID = 0x03u,                                      /*!< Random (Static) Identity Address (Corresponds to Resolved Private Address) */
} sid_pal_ble_ext_peer_addr_type_t;

/**
 * @brief Peer identity address information
 * 
 * @note If the peer is using Public or (Static) Random address, the Peer Identity address will be equal to the peer address.
 *       If the peer is using RPA address and this peer is known to host (paired/bonded), the Peer Identity address is the resolved address from the RPA.
 * @note This datatype is used to store peer bonding data in NVM, meaning it should be declared as __PACKED_STRUCT and the use of the use of the inner
 *       datatypes with loosely defined size (e.g., enums) is not allowed. These requirements are in place to ensure compatibility of the NVM data with
 *       firmware updates (OTA, using a different toolchain or compilation parameters, etc.).
 */
typedef __PACKED_STRUCT {
  uint8_t identity_address_type; /*!< Type of the identity address. 0x00: Public Identity Address, 0x01: Random (static) Identity Address */
  uint8_t identity_address[6];   /*!< Identity address of the peer. If peer uses RPA, this address will be the resolved one from the RAP address */
} sid_pal_ble_ext_peer_identity_addr_t;

/**
 * @brief Definitions of the LE PHY types for HCI LE
 * 
 */
typedef enum {
    SPBP_LE_PHY_UNKNOWN = 0x00u,                                                  /*!< PHY in use is unknown */
    SPBP_LE_PHY_1M      = 0x01u,                                                  /*!< 1Mbit physical layer */
    SPBP_LE_PHY_2M      = 0x02u,                                                  /*!< 2Mbit physical layer */
    SPBP_LE_PHY_CODED   = 0x03u,                                                  /*!< LE-coded PHY */
} sid_pal_ble_ext_le_phy_t;

/**
 * @brief Public BLE connection context
 *
 * @note This context is made available to the end user
 */
typedef struct {
    uint16_t                         conn_id;                                     /*!< BLE connection handle */
    uint16_t                         mtu_size;                                    /*!< Currently applied MTU size for this connection */
    uint16_t                         max_mtu_size;                                /*!< Maximum allowed ATT MTU size for this connection */
    uint8_t                          is_secure;                                   /*!< Indicates if this connection uses BLE Secure Connection */
    sid_pal_ble_ext_peer_addr_type_t peer_addr_type;                              /*!< Type of the BLE address used by the remote peer (as per Core Spec)*/
    uint8_t                          peer_identity[SID_STM32_BLE_ADDRESS_LENGTH]; /*!< Public Device Address, or Random Device Address, Public Identity Address or Random (static) Identity Address of the connected device */
    uint8_t                          local_rpa[SID_STM32_BLE_ADDRESS_LENGTH];     /*!< Resolvable Private Address being used by the local device for this connection. This is only valid when the RPA was generated and used for advertisement */
    uint8_t                          peer_rpa[SID_STM32_BLE_ADDRESS_LENGTH];      /*!< Resolvable Private Address being used by the peer device for this connection. This is only valid for peer_addr_type SID_BLE_HCI_ADV_PEER_ADDR_TYPE_RPA_PUBLIC_IDENTITY and SID_BLE_HCI_ADV_PEER_ADDR_TYPE_RPA_RANDOM_IDENTITY */
    uint16_t                         conn_interval;                               /*!< Currently applied connection interval in BLE stack units. Actual interval = (conn_interval * 1.25) ms. */
    uint16_t                         conn_latency;                                /*!< Currently applied connection latency in number of connection events */
    uint16_t                         supervision_timeout;                         /*!< Currently applied connection supervision timeout in BLE stack units. Actual timeout = (supervision_timeout * 10) ms */
    sid_pal_ble_ext_le_phy_t         tx_phy;                                      /*!< Currently used PHY for transmissions */
    sid_pal_ble_ext_le_phy_t         rx_phy;                                      /*!< Currently used PHY for receptions */
} sid_ble_ext_connection_ctx_t;

typedef struct {
    uint16_t interval_min;       /*!< Hard limit for the minimum connection interval suitable for the host in BLE stack units. Actual interval = (conn_interval * 1.25) ms */
    uint16_t interval_max;       /*!< Hard limit for the maximum connection interval suitable for the host in BLE stack units. Actual interval = (conn_interval * 1.25) ms */
    uint16_t latency_max;        /*!< Hard limit for the maximum connection latency in number of connection events suitable for the host */
    uint16_t reasonable_timeout; /*!< Soft limit for the connection supervision timeout (whenever latency and interval require longer minimum timeout this value is ignored). Expressed in BLE stack units suggested by the local host. Actual timeout = (reasonable_timeout * 10) ms */
    uint16_t ce_length_min;      /*!< Hard limit for the minimum length of connection needed for this LE connection in BLE stack units. Actual length = (ce_length_min * 0.625) ms */
    uint16_t ce_length_max;      /*!< Hard limit for the maximum length of connection needed for this LE connection in BLE stack units. Actual length = (ce_length_max * 0.625) ms */
} sid_ble_ext_conn_params_limits_t;

/**
 * @brief BLE connection parameters proposed by peer (e.g. during the Connection Parameter Request procedure)
 */
typedef struct {
    uint16_t interval_min; /*!< Minimum connection interval suitable for the remote peer in BLE stack units. Actual interval = (interval_min * 1.25) ms */
    uint16_t interval_max; /*!< Maximum connection interval suitable for the remote peer in BLE stack units. Actual interval = (interval_max * 1.25) ms */
    uint16_t latency_max;  /*!< Maximum connection latency in number of connection events suitable for the remote peer */
    uint16_t timeout;      /*!< Connection supervision timeout in BLE stack units suggested by the remote peer. Actual timeout = (timeout * 10) ms */
} sid_ble_ext_proposed_conn_params_t;

/**
 * @brief BLE connection parameters proposed by peer (e.g. during the Connection Parameter Request procedure)
 * 
 */
typedef struct {
    uint16_t interval_min;  /*!< Minimum connection interval suitable for the host in BLE stack units. Actual interval = (interval_min * 1.25) ms */
    uint16_t interval_max;  /*!< Maximum connection interval suitable for the host in BLE stack units. Actual interval = (interval_max * 1.25) ms */
    uint16_t latency_max;   /*!< Maximum connection latency in number of connection events suitable for the host */
    uint16_t timeout;       /*!< Connection supervision timeout in BLE stack units suggested by the local host. Actual timeout = (timeout * 10) ms */
    uint16_t ce_length_min; /*!< Minimum length of connection needed for this LE connection in BLE stack units. Actual length = (ce_length_min * 0.625) ms */
    uint16_t ce_length_max; /*!< Maximum length of connection needed for this LE connection in BLE stack units. Actual length = (ce_length_max * 0.625) ms */
} sid_ble_ext_accepted_conn_params_t;

#if (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_INTERLEAVING) || (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_CONCURRENCY)

/**
 * @brief Unique virtual device ID. This ID should follow the guidelines for BLE extended advertising set ID
 *
 * @note Valid range: 0x00 - 0xEE
 *
 * @warning ID of 0xEF is reserved for Sidewalk and cannot be used
 */
typedef uint8_t sid_ble_ext_virtual_device_id_t;

/**
 * @brief Type of the virtual BLE device
 *
 * @note This corresponds to the BLE device roles
 */
typedef enum {
    SBEVDR_BLE_PERIPHERAL  = 1u << 0,
    SBEVDR_BLE_BROADCASTER = 1u << 1,
    SBEVDR_BLE_CENTRAL     = 1u << 2,
    SBEVDR_BLE_OBSERVER    = 1u << 3,
} sid_ble_ext_virtual_device_role_t;

/**
 * @brief List node or the list of BLE connections
 * 
 */
typedef struct {
    tListNode                                  node; /*!< Linked list node to be used by stm_list utility */
    const sid_ble_ext_connection_ctx_t * const ctx;  /*!< BLE connection context */
} sid_ble_ext_conn_list_node_t;

/**
 * @brief Logical states of a Virtual BLE Device
 *
 * @warning SBEVD_STATE_ADV_FAST, SBEVD_STATE_ADV_SLOW, SBEVD_STATE_SCANNING, and SBEVD_STATE_CONNECTED states are bit fields. Be careful not to
 *          interfere with this bits when defining the other states - the bits corresponding to the mentioned states should never be set by any
 *          other state.
 */
typedef enum {
    SBEVD_STATE_UNINITIALIZED          = 0,             /*!< Virtual device is not initialized at all */
    SBEVD_STATE_BOOTSTRAPPING          = 1,             /*!< Virtual device is in the process of initialization. This is a transient state, the user must wait for initialization to complete */
    SBEVD_STATE_SUSPENDED              = 2,             /*!< Virtual device is suspended - no advertising, scanning, connections are allowed, but resources are not offloaded from BLE controller */
    SBEVD_STATE_IDLE                   = 3,             /*!< Virtual device is bootstrapped and ready to be used, but it's currently doing nothing */
    SBEVD_STATE_ADV_FAST               = (1 << 2),      /*!< Virtual device is actively advertising at the moment using fast advertising settings. Valid for Peripheral and Broadcaster roles only */
    SBEVD_STATE_ADV_SLOW               = (1 << 3),      /*!< Virtual device is actively advertising at the moment using slow advertising settings. Valid for Peripheral and Broadcaster roles only */
    SBEVD_STATE_SCANNING               = (3 << 2),      /*!< Virtual device is actively scanning at the moment. Valid for Central and Observer roles only */
    SBEVD_STATE_CONNECTED              = (1 << 4),
    SBEVD_STATE_CONNECTED_AND_ADV_FAST =
        (SBEVD_STATE_CONNECTED | SBEVD_STATE_ADV_FAST), /*!< Virtual device device is connected to peer(s) and is advertising in fast mode. This state is valid for Peripherals which allow multiple connections */
    SBEVD_STATE_CONNECTED_AND_ADV_SLOW =
        (SBEVD_STATE_CONNECTED | SBEVD_STATE_ADV_SLOW), /*!< Virtual device device is connected to peer(s) and is advertising in slow mode. This state is valid for Peripherals which allow multiple connections */
    SBEVD_STATE_CONNECTED_AND_SCANNING =
        (SBEVD_STATE_CONNECTED | SBEVD_STATE_SCANNING), /*!< Virtual device device is connected to peer(s) and is still scanning. This state is valid for Centrals which allow multiple connections */
    SBEVD_STATE_FATAL_ERROR            = 32,            /*!< Virtual device faced an unrecoverable aerror and cannot be used any longer. The user may terminate it and try to restart */
    SBEVD_STATE_TERMINATING            = 33,            /*!< Virtual device is in the process of termination. This is a transient state, the user must wait for termination to complete */
} sid_ble_ext_virtual_device_state_t;

/**
 * @brief BLE advertising parameters
 *
 * @note This configuration is used by both Peripheral ( @ref SBEVDR_BLE_PERIPHERAL ) and Broadcaster ( @ref SBEVDR_BLE_BROADCASTER ) roles
 */
typedef struct {
    int8_t   adv_tx_power;      /*!< Preferred Tx power for advertising. Valid range: -127..20; +127 means no preference. It's not guaranteed that the requested power level will be used */
    uint8_t  auto_restart;      /*!< Automatically restart the advertisement when connection is lost. This parameter is only valid for peripheral config */
    uint8_t  scan_resp_en;      /*!< Enable scan response.  */
    uint8_t  fast_enabled;      /*!< Enable fast advertising phase */
    uint8_t  slow_enabled;      /*!< Enable slow advertising phase */
    uint32_t fast_interval_min; /*!< Minimum advertising interval for the fast phase. In units of 0.625 ms */
    uint32_t fast_interval_max; /*!< Maximum advertising interval for the fast phase. In units of 0.625 ms */
    uint32_t fast_timeout;      /*!< Advertising timeout for the fast phase. In units of 10 ms. Setting to zero means infinite advertisement in fast mode */
    uint32_t slow_interval_min; /*!< Minimum advertising interval for the slow phase. In units of 0.625 ms */
    uint32_t slow_interval_max; /*!< Maximum advertising interval for the slow phase. In units of 0.625 ms */
    uint32_t slow_timeout;      /*!< Advertising timeout for the slow phase. In units of 10 ms. Setting to zero means infinite advertisement in slow mode */
} sid_ble_ext_adv_param_t;

typedef enum {
    SBEASC_ADV_STARTED_FAST,                      /*!< Advertisement started in Fast mode (using Fast settings) */
    SBEASC_ADV_STARTED_SLOW,                      /*!< Advertisement started in Slow mode (using Slow settings) */
    SBEASC_ADV_SWITCHED_FAST_TO_SLOW,             /*!< Advertisement switched from Fast mode to SLow mode */
    SBEASC_ADV_SWITCHED_TERMINATED_BY_TIMEOUT,    /*!< Advertisement stopped due to advertising timeout */
    SBEASC_ADV_SWITCHED_TERMINATED_BY_CONNECTION, /*!< Advertisement stopped because it resulted in a connection - valid for Peripheral devices only */
    SBEASC_ADV_SWITCHED_TERMINATED_ON_REQUEST,    /*!< Advertisement stopped on explicit request from the user */
} sid_ble_ext_adv_state_change_t;

/**
 * @brief Properties for a characteristic
 */
typedef struct {
    uint8_t read          : 1;
    uint8_t write_wo_resp : 1;
    uint8_t write         : 1;
    uint8_t notify        : 1;
} sid_ble_ext_char_prop_t;

/**
 * @brief Access permissions for an attribute
 */
typedef struct {
    uint8_t read          : 1;
    uint8_t write_wo_resp : 1;
    uint8_t write         : 1;
} sid_ble_ext_attr_access_t;

/**
 * @brief Security permissions for an attribute
 */
typedef struct {
    uint8_t authen_read  : 1; /*!< Need authentication to read */
    uint8_t author_read  : 1; /*!< Need authorization to read */
    uint8_t encry_read   : 1; /*!< Need encryption to read */
    uint8_t authen_write : 1; /*!< Need authentication to write */
    uint8_t author_write : 1; /*!< Need authorization to write */
    uint8_t encry_write  : 1; /*!< Need encryption to write */
} sid_ble_ext_attr_perm_t;

/**
 * @brief Definition of the CCCD value
 */
typedef struct {
    uint8_t notify_en   : 1;
    uint8_t indicate_en : 1;
    uint8_t : 6;
    uint8_t : 8;
} sid_ble_ext_cccd_val_t;

/**
 * @brief Definition of a ATT Descriptor in the BLE configuration
 *
 * @warning This is for the custom descriptors only. The default descriptors like CCCD are added automatically by the driver and shall never be defined in the user config
 */
typedef struct {
    char *                    desc_name;  /*!< Display name of the characteristic descriptor. Used for logging purposes only, can be NULL */
    sid_ble_cfg_uuid_info_t   uuid;       /*!< UUID of this descriptor. This field uses Big Endian byte order, making it easier to comprehend UUIDs */
    uint8_t                   max_length; /*!< Maximum data length for this characteristic descriptor - this should not exceed maximum MTU size */
    sid_ble_ext_attr_access_t access;     /*!< Access permissions for this descriptor */
    sid_ble_ext_attr_perm_t   security;   /*!< Security permissions for this descriptor */
} sid_ble_ext_char_desc_t;

/**
 * @brief GATT characteristic definition in the BLE configuration
 */
typedef struct {
    char *                          char_name;   /*!< Display name of the characteristic. Used for logging purposes only, can be NULL */
    sid_ble_cfg_uuid_info_t         uuid;        /*!< UUID of this GATT characteristic. This field uses Big Endian byte order, making it easier to comprehend UUIDs */
    uint16_t                        max_length;  /*!< Maximum data length for this characteristic - this should not exceed maximum MTU size */
    sid_ble_ext_char_prop_t         properties;  /*!< Properties of this characteristic */
    sid_ble_ext_attr_perm_t         permissions; /*!< Security permissions for this characteristic */
    uint8_t                         desc_count;  /*!< Number of additional attributes in this characteristic. IMPORTANT: this is for the custom descriptors only. Standard descriptors like CCCD are added automatically and shall not be defined by the user */
    const sid_ble_ext_char_desc_t * descriptors;
} sid_ble_ext_char_def_t;

/**
 * @brief GATT service definition in the BLE configuration
 */
typedef struct {
    char *                         service_name;    /*!< Display name of the service. Used for logging purposes only, can be NULL */
    sid_ble_cfg_uuid_info_t        uuid;            /*!< UUID of this GATT service. This field uses Big Endian byte order, making it easier to comprehend UUIDs */
    uint8_t                        char_count;      /*!< Number if the characteristics that belong to this service */
    const sid_ble_ext_char_def_t * characteristics; /*!< The list of characteristic descriptors that belong to this service */
} sid_ble_ext_gatt_svc_def_t;

/**
 * @brief Complete GATT profile definition in the BLE configuration
 */
typedef struct {
    uint8_t                            svc_count; /*!< The number of services in the profile */
    const sid_ble_ext_gatt_svc_def_t * services;  /*!< Definition of the services that belong to this profile */
} sid_ble_ext_gatt_profile_def_t;

/**
 * @brief Type of attempted access to a GATT attribute on a GATT server (BLE peripheral) when read/write permission request is issued
 *
 * @note This datatype is valid for a BLE peripheral only
 */
typedef enum {
    SBEAAT_READ_ACCESS,
    SBEAAT_WRITE_ACCESS,
} sid_ble_ext_att_access_type_t;

struct sid_ble_ext_virtual_device_ctx_s;        /*!< Forward declaration of the sid_ble_ext_virtual_device_ctx_t data type */
struct sid_ble_ext_gatt_server_svc_ctx_s;       /*!< Forward declaration of the sid_ble_ext_gatt_server_svc_ctx_t data type */
struct sid_ble_ext_gatt_server_char_ctx_s;      /*!< Forward declaration of the sid_ble_ext_gatt_server_char_ctx_t data type */
struct sid_ble_ext_gatt_server_char_desc_ctx_s; /*!< Forward declaration of the sid_ble_ext_gatt_server_char_desc_ctx_t data type */

/**
 * @brief User-defined callback for BLE Advertising State Changed event
 *        This callback is invoked by the driver for Peripheral and Broadcaster roles whenever advertisement state changes
 *
 * @param [in] device_ctx Context of the virtual device for which the advertisement state has changed
 * @param [in] adv_event Details on advertising state change
 */
typedef void (*sid_ble_ext_on_adv_state_changed_cb_t)(const struct sid_ble_ext_virtual_device_ctx_s * const device_ctx, sid_ble_ext_adv_state_change_t adv_event);

/**
 * @brief User-defined callback for BLE Connection Established event
 *        This callback is invoked by the driver for Peripheral and Central roles whenever a BLE connection is established
 *
 * @param [in] device_ctx Context of the virtual device for which the connection was established
 * @param [in] conn_ctx Connection context
 */
typedef void (*sid_ble_ext_on_connected_cb_t)(const struct sid_ble_ext_virtual_device_ctx_s * const device_ctx, const sid_ble_ext_connection_ctx_t * const conn_ctx);

/**
 * @brief User-defined callback for BLE Connection Established event
 *        This callback is invoked by the driver for Peripheral and Central roles whenever a BLE connection is established
 *
 * @param [in] device_ctx Context of the virtual device for which the connection was terminated
 * @param [in] conn_ctx Connection context of the terminated connection
 */
typedef void (*sid_ble_ext_on_disconnected_cb_t)(const struct sid_ble_ext_virtual_device_ctx_s * const device_ctx, const sid_ble_ext_connection_ctx_t * const conn_ctx);

/**
 * @brief User-defined callback for BLE MTU Changed event
 *        This callback is invoked by the driver for Peripheral and Central roles whenever a BLE attribute MTU size is updated
 *
 * @note Updated mtu size can be accessed via conn_ctx->mtu_size, there's no need to pass as additional parameter of this call
 *
 * @param [in] device_ctx Context of the virtual device for which the MTU size has changed
 * @param [in] conn_ctx Connection context
 */
typedef void (*sid_ble_ext_on_mtu_changed_cb_t)(const struct sid_ble_ext_virtual_device_ctx_s * const device_ctx, const sid_ble_ext_connection_ctx_t * const conn_ctx);

/**
 * @brief User-defined callback for BLE Connection Parameters Changed event
 *        This callback is invoked by the driver for Peripheral and Central roles whenever a BLE connection parameters are updated
 *
 * @note Updated mtu size can be accessed via conn_ctx->mtu_size, there's no need to pass as additional parameter of this call
 *
 * @param [in] device_ctx Context of the virtual device for which the connection parameters have changed
 * @param [in] conn_ctx Connection context
 */
typedef void (*sid_ble_ext_on_conn_param_changed_cb_t)(const struct sid_ble_ext_virtual_device_ctx_s * const device_ctx, const sid_ble_ext_connection_ctx_t * const conn_ctx);

/**
 * @brief User-defined callback for BLE Connection Parameters Changed event
 *        This callback is invoked by the driver for Peripheral and Central roles whenever a BLE connection parameters are updated
 *
 * @note Updated mtu size can be accessed via conn_ctx->mtu_size, there's no need to pass as additional parameter of this call
 *
 * @param [in] device_ctx Context of the virtual device for which the connection parameters have changed
 * @param [in] conn_ctx Connection context
 * @param [in] proposed_params
 * @param [out] out_accepted_params
 * @param [out] out_le_resp_code
 */
typedef void (*sid_ble_ext_on_conn_param_update_req_cb_t)(const struct sid_ble_ext_virtual_device_ctx_s * const device_ctx, const sid_ble_ext_connection_ctx_t * const conn_ctx,
                                                          const sid_ble_ext_proposed_conn_params_t * const proposed_params, sid_ble_ext_accepted_conn_params_t * const out_accepted_params, tBleStatus * const out_le_resp_code);

/**
 * @brief User-defined callback for BLE Connection Encryption Changed event
 *        This callback is invoked by the driver for Peripheral and Central roles whenever a BLE connection encryption state is modified
 *
 * @note Updated encryption state can be accessed via conn_ctx->is_secure, there's no need to pass as additional parameter of this call
 *
 * @param [in] device_ctx Context of the virtual device for which the encryption changed
 * @param [in] conn_ctx Connection context
 */
typedef void (*sid_ble_ext_on_encryption_changed_cb_t)(const struct sid_ble_ext_virtual_device_ctx_s * const device_ctx, const sid_ble_ext_connection_ctx_t * const conn_ctx);

/**
 * @brief User-defined callback for BLE Keypress notification event
 *        This callback is invoked by the driver for Peripheral and Central roles whenever a key press notification arrives during the pairing process
 *
 * @note This is just a notification. No actions are required
 *
 * @param [in] device_ctx Context of the virtual device for which the key press notification took place
 * @param [in] conn_ctx Connection context
 * @param [in] notification_type Type of Keypress input notified/signaled by peer device (having Keyboard only I/O capabilities)
 */
typedef void (*sid_ble_ext_on_pairing_keypress_notification_cb_t)(const struct sid_ble_ext_virtual_device_ctx_s * const device_ctx, const sid_ble_ext_connection_ctx_t * const conn_ctx, const uint8_t notification_type);

/**
 * @brief User-defined callback for BLE Passkey Request event
 *        This callback is invoked by the driver for Peripheral and Central roles whenever the Numeric Comparison pairing method is used for pairing
 *
 * @note This is the request for passkey during the process. User must generate a valid passkey and display it
 *
 * @note This callback may not be invoked depending on the IO capabilities of this device and the remote peer's ones
 *
 * @param [in]  device_ctx Context of the virtual device for which the encryption changed
 * @param [in]  conn_ctx Connection context
 * @param [out] out_passkey 6-digit passkey value to be used during the pairing process
 */
typedef void (*sid_ble_ext_on_pairing_pass_key_request_cb_t)(const struct sid_ble_ext_virtual_device_ctx_s * const device_ctx, const sid_ble_ext_connection_ctx_t * const conn_ctx, uint32_t * const out_passkey);

/**
 * @brief User-defined callback for BLE Numeric Comparison event
 *        This callback is invoked by the driver for Peripheral and Central roles whenever the Numeric Comparison pairing method is used for pairing
 *
 * @note This callback may not be invoked depending on the IO capabilities of this device and the remote peer's ones
 *
 * @param [in]  device_ctx Context of the virtual device for which the Numeric Comparison event takes place
 * @param [in]  conn_ctx Connection context
 * @param [in]  numeric_value Te value supplied by the remote peer for Numeric Comparison
 * @param [out] out_numeric_value_matches The user must set this to a non-zero value if the passkey displayed on the remote peer's screen matches the numeric_value parameter. Setting this value to zero will reject the pairing attempt
 */
typedef void (*sid_ble_ext_on_pairing_numeric_comparison_value_cb_t)(const struct sid_ble_ext_virtual_device_ctx_s * const device_ctx, const sid_ble_ext_connection_ctx_t * const conn_ctx, const uint32_t numeric_value, uint32_t * const out_numeric_value_matches);

/**
 * @brief User-defined callback for BLE Pairing Complete event
 *        This callback is invoked by the driver for Peripheral and Central roles whenever the pairing process is finished (either successfully or not)
 *
 * @param [in] device_ctx Context of the virtual device for which pairing process is completed
 * @param [in] conn_ctx Connection context
 * @param [in] hci_status HCI status of the pairing process (see Core Specification Volume 1, Part F). 0x00 means succeq, other values determine error code
 * @param [in] hci_reason HCI reason code for pairing failure. Valid only if hci_status indicates a failure of the pairing procedure
 */
typedef void (*sid_ble_ext_on_pairing_complete_cb_t)(const struct sid_ble_ext_virtual_device_ctx_s * const device_ctx, const sid_ble_ext_connection_ctx_t * const conn_ctx, const uint8_t hci_status, uint8_t hci_reason);

/**
 * @brief User-defined callback to handle BLE authorization request
 *        This callback is invoked by the driver for Peripherals whenever a Central is trying to access an attribute which requires authorization
 *
 * @param [in] device_ctx Context of the virtual device for which authorization request is issued
 * @param [in] conn_ctx COntext of the connection which initiated the authorization request
 * @return SID_ERROR_NONE if the request is authorized, SID_ERROR_NO_PERMISSION if not
 */
typedef sid_error_t (*sid_ble_ext_on_peripheral_authorization_req_cb_t)(const struct sid_ble_ext_virtual_device_ctx_s * const device_ctx, const sid_ble_ext_connection_ctx_t * const conn_ctx);

/**
 * @brief User-defined callback for BLE write or read permission request
 *        This callback is invoked whenever a GATT client tries to read an attribute or write to it. For long attributes it may be invoked multiple times for the same read/write operation because the controller provides/receives data in chunks
 *
 * @note This callback can be used to update the corresponding characteristic or characteristic descriptor value right before a GATT client will read it.
 *
 * @param [in] device_ctx Context of the virtual device for which pairing process is completed
 * @param [in] conn_ctx Connection context
 * @param [in] svc_ctx Context of the GATT service to which the attribute in question belongs. Cannot be NULL
 * @param [in] char_ctx Context of the GATT characteristic to which the attribute in question belongs. Cannot be NULL
 * @param [in] desc_ctx Context of the GATT characteristic descriptor to which the attribute in question belongs. Can be null if attribute in question corresponds to the characteristic value attribute
 * @param [in] access_mode Type of the access (read or write0 for which the permission is requested
 *
 * @return ATT status code. SID_STM32_BLE_ATT_STATUS_SUCCESS means access is granted, non-zero values will be transferred to the GATT client.
 *         For read access the only valid values are SID_STM32_BLE_ATT_STATUS_INSUFFICIENT_AUTHOR, and anything between SID_STM32_BLE_ATT_STATUS_APP_ERROR_MIN and SID_STM32_BLE_ATT_STATUS_APP_ERROR_MAX
 */
typedef uint8_t (*sid_ble_ext_on_peripheral_att_access_req_cb_t)(const struct sid_ble_ext_virtual_device_ctx_s * const device_ctx, const sid_ble_ext_connection_ctx_t * const conn_ctx, const struct sid_ble_ext_gatt_server_svc_ctx_s * const svc_ctx,
                                                                 const struct sid_ble_ext_gatt_server_char_ctx_s * const char_ctx, const struct sid_ble_ext_gatt_server_char_desc_ctx_s * const desc_ctx, const sid_ble_ext_att_access_type_t access_mode);

/**
 * @brief User-defined callback for BLE GATT Server Attribute Written event
 *        This callback is invoked whenever a GATT client successfully writes to an attribute (either characteristic value or characteristic descriptor)
 *
 * @note This callback is valid for BLE Peripheral devices only
 *
 * @param [in] device_ctx Context of the virtual device for which new downlink was received
 * @param [in] conn_ctx Connection context
 * @param [in] svc_ctx Context of the GATT service to which the attribute in question belongs. Cannot be NULL
 * @param [in] char_ctx Context of the GATT characteristic to which the attribute in question belongs. Cannot be NULL
 * @param [in] desc_ctx Context of the GATT characteristic descriptor to which the attribute in question belongs. Can be null if attribute in question corresponds to the characteristic value attribute
 * @param [in] data Pointer to the data buffer. This is a convenience-only pointer to the corresponding data buffer in char_ctx or desc_ctx
 * @param [in] data_length Length of the received data. This is a convenience-only value, it corresponds to the valid length of the data buffer specified in char_ctx or desc_ctx
 *
 * @return ATT status code. SID_STM32_BLE_ATT_STATUS_SUCCESS means access is granted, non-zero values will be transferred to the GATT client.
 *         For read access the only valid values are SID_STM32_BLE_ATT_STATUS_INSUFFICIENT_AUTHOR, and anything between SID_STM32_BLE_ATT_STATUS_APP_ERROR_MIN and SID_STM32_BLE_ATT_STATUS_APP_ERROR_MAX
 */
typedef void (*sid_ble_ext_on_peripheral_data_received_cb_t)(const struct sid_ble_ext_virtual_device_ctx_s * const device_ctx, const sid_ble_ext_connection_ctx_t * const conn_ctx, const struct sid_ble_ext_gatt_server_svc_ctx_s * const svc_ctx,
                                                                 const struct sid_ble_ext_gatt_server_char_ctx_s * const char_ctx, const struct sid_ble_ext_gatt_server_char_desc_ctx_s * const desc_ctx, const uint8_t * const data, const uint32_t data_length);

/**
 * @brief User-defined callback for GATT characteristic CCCD update
 *        This callback is invoked whenever a GATT client successfully writes to characteristic's CCCD
 *
 * @note This callback is valid for BLE Peripheral devices only
 *
 * @param [in] device_ctx Context of the virtual device for which new downlink was received
 * @param [in] conn_ctx Connection context
 * @param [in] svc_ctx Context of the GATT service to which the characteristic in question belongs. Cannot be NULL
 * @param [in] char_ctx Context of the GATT characteristic to which the CCCD in question belongs. Cannot be NULL
 * @param [in] cccd_val New value of the CCCD - it show if notifications and/or indications for this characteristic are enabled
 *
 * @return ATT status code. SID_STM32_BLE_ATT_STATUS_SUCCESS means access is granted, non-zero values will be transferred to the GATT client.
 *         For read access the only valid values are SID_STM32_BLE_ATT_STATUS_INSUFFICIENT_AUTHOR, and anything between SID_STM32_BLE_ATT_STATUS_APP_ERROR_MIN and SID_STM32_BLE_ATT_STATUS_APP_ERROR_MAX
 */
typedef void (*sid_ble_ext_on_peripheral_cccd_changed_cb_t)(const struct sid_ble_ext_virtual_device_ctx_s * const device_ctx, const sid_ble_ext_connection_ctx_t * const conn_ctx, const struct sid_ble_ext_gatt_server_svc_ctx_s * const svc_ctx,
                                                            const struct sid_ble_ext_gatt_server_char_ctx_s * const char_ctx, const sid_ble_ext_cccd_val_t cccd_val);

/**
 * @brief Virtual BLE device parameters that are specific to the Peripheral role
 */
typedef struct {
    const sid_ble_ext_adv_param_t *  adv_param;    /*!< Advertising parameters for this virtual device */
    const sid_ble_cfg_conn_param_t * conn_param;   /*!< Connection parameters preferences for the specific virtual device */
    uint16_t                         appearance;   /*!< Value for the GAP Appearance characteristic */
    uint16_t                         max_att_mtu;  /*!< Maximum attribute MTU size for this virtual Peripheral. This setting does not affect other virtual devices and Sidewalk profile */
    uint16_t                         max_conn;     /*!< Maximum number of simultaneous connections for the device */
    sid_ble_ext_gatt_profile_def_t   gatt_profile; /*!< GATT profile of this peripheral device */
    struct {
        sid_ble_ext_on_adv_state_changed_cb_t                on_adv_state_changed;
        sid_ble_ext_on_connected_cb_t                        on_connected;
        sid_ble_ext_on_disconnected_cb_t                     on_disconnected;
        sid_ble_ext_on_mtu_changed_cb_t                      on_mtu_changed;
        sid_ble_ext_on_conn_param_changed_cb_t               on_conn_param_changed;
        sid_ble_ext_on_conn_param_update_req_cb_t            on_conn_param_update_req;
        sid_ble_ext_on_encryption_changed_cb_t               on_encryption_changed;
        sid_ble_ext_on_pairing_keypress_notification_cb_t    on_pairing_keypress_notification;
        sid_ble_ext_on_pairing_pass_key_request_cb_t         on_pairing_pass_key_request;
        sid_ble_ext_on_pairing_numeric_comparison_value_cb_t on_pairing_numeric_comparison_value;
        sid_ble_ext_on_pairing_complete_cb_t                 on_pairing_complete;
        sid_ble_ext_on_peripheral_authorization_req_cb_t     on_authorization_req;
        sid_ble_ext_on_peripheral_att_access_req_cb_t        on_att_access_request;
        sid_ble_ext_on_peripheral_data_received_cb_t         on_data_received;
        sid_ble_ext_on_peripheral_cccd_changed_cb_t          on_cccd_changed;
    } callbacks;
} sid_ble_ext_virtual_peripheral_cfg_t;

/**
 * @brief Virtual BLE device parameters that are specific to the Broadcaster role
 */
typedef struct {
    const sid_ble_ext_adv_param_t * adv_param;  /*!< Advertising parameters for this virtual device */
    uint16_t                        appearance; /*!< BLE Device Appearance value. Can be used in advertisement data */
    struct {
        sid_ble_ext_on_adv_state_changed_cb_t on_adv_state_changed;
    } callbacks;
} sid_ble_ext_virtual_broadcaster_cfg_t;

/**
 * @brief Virtual BLE device parameters that are specific to the Central role
 */
typedef struct {
    uint16_t                        max_att_mtu; /*!< Maximum attribute MTU size for this virtual Central. This setting does not affect other virtual devices and Sidewalk profile */
    uint16_t                        max_conn;    /*!< Maximum number of simultaneous connections for the device */
    struct {
        sid_ble_ext_on_adv_state_changed_cb_t                on_adv_state_changed;
        sid_ble_ext_on_connected_cb_t                        on_connected;
        sid_ble_ext_on_disconnected_cb_t                     on_disconnected;
        sid_ble_ext_on_mtu_changed_cb_t                      on_mtu_changed;
        sid_ble_ext_on_conn_param_changed_cb_t               on_conn_param_changed;
        sid_ble_ext_on_conn_param_update_req_cb_t            on_conn_param_update_req;
        sid_ble_ext_on_encryption_changed_cb_t               on_encryption_changed;
        sid_ble_ext_on_pairing_keypress_notification_cb_t    on_pairing_keypress_notification;
        sid_ble_ext_on_pairing_pass_key_request_cb_t         on_pairing_pass_key_request;
        sid_ble_ext_on_pairing_numeric_comparison_value_cb_t on_pairing_numeric_comparison_value;
        sid_ble_ext_on_pairing_complete_cb_t                 on_pairing_complete;
    } callbacks;
} sid_ble_ext_virtual_central_cfg_t;

/**
 * @brief Virtual BLE device parameters that are specific to the Observer role
 */
typedef struct {
    const sid_ble_ext_adv_param_t * adv_param; /*!< Advertising parameters for this virtual device */
} sid_ble_ext_virtual_observer_cfg_t;

/**
 * @brief Profile definition for a virtual BLE device
 * 
 */
typedef struct {
    sid_ble_ext_virtual_device_id_t           device_id;       /*!< Unique virtual device ID. This ID should follow the guidelines for BLE extended advertising set ID. Valid range: 0x00 - 0xEE. ID of 0xEF is reserved for Sidewalk and cannot be used */
    const char *                              device_name;     /*!< This name can be used when this virtual device is advertising */
    sid_ble_ext_virtual_device_role_t         device_type;     /*!< Role of the BLE device */
    sid_ble_cfg_mac_address_type_t            mac_addr_type;   /*!< Type of BLE address used by this device */
    union {
        sid_ble_ext_virtual_peripheral_cfg_t  peripheral_cfg;
        sid_ble_ext_virtual_broadcaster_cfg_t broadcaster_cfg;
        sid_ble_ext_virtual_central_cfg_t     central_cfg;
        sid_ble_ext_virtual_observer_cfg_t    observer_cfg;
    };
} sid_ble_ext_virtual_device_t;

/**
 * @brief Context of an characteristic descriptor for GATT server (BLE peripheral)
 */
typedef struct sid_ble_ext_gatt_server_char_desc_ctx_s {
    const sid_ble_ext_char_desc_t * desc_def;      /*!< Descriptor definition from the user config. Pointer for quick access */
    uint16_t                        handle;        /*!< Descriptor handle assigned by the controller */
    uint8_t *                       data;          /*!< Buffer for the full characteristic descriptor value (up to 509 bytes) */
    uint16_t                        data_length;   /*!< Length of the valid data in the buffer */
    uint16_t                        upd_accum_len; /*!< Accumulated data length during chunked modifications (e.g. receiving long data) */
} sid_ble_ext_gatt_server_char_desc_ctx_t;

/**
 * @brief Context of a characteristic for GATT server (BLE peripheral)
 */
typedef struct sid_ble_ext_gatt_server_char_ctx_s {
    const sid_ble_ext_char_def_t *            char_def;        /*!< Characteristic definition from the user config. Pointer for quick access */
    uint16_t                                  handle;          /*!< Characteristic handle assigned by the controller */
    uint8_t                                   desc_ctxs_count; /*!< Number of contexts for custom characteristic descriptors in this characteristic */
    sid_ble_ext_gatt_server_char_desc_ctx_t * desc_ctxs;       /*!< Array of descriptor contexts */
    uint8_t *                                 data;            /*!< Buffer for the full characteristic value (up to 509 bytes) */
    uint16_t                                  data_length;     /*!< Length of the valid data in the buffer */
    uint16_t                                  upd_accum_len;   /*!< Accumulated data length during chunked modifications (e.g. receiving long data) */
} sid_ble_ext_gatt_server_char_ctx_t;

/**
 * @brief Context of a service for GATT server (BLE peripheral)
 */
typedef struct sid_ble_ext_gatt_server_svc_ctx_s {
    const sid_ble_ext_gatt_svc_def_t *   svc_def;         /*!< Service definition from the user config. Pointer for quick access */
    uint16_t                             handle;          /*!< Service handle assigned by the controller */
    uint8_t                              char_ctxs_count; /*!< Number of contexts for characteristics in this service */
    sid_ble_ext_gatt_server_char_ctx_t * char_ctxs;       /*!< Array of characteristic contexts */
} sid_ble_ext_gatt_server_svc_ctx_t;

/**
 * @brief Context of the virtual BLE devices. It reflects both static configuration and dynamic states
 */
typedef struct sid_ble_ext_virtual_device_ctx_s {
    sid_ble_ext_virtual_device_state_t   state;       /*!< Current logical state of the virtual BLE device */
    const sid_ble_ext_virtual_device_t * device_cfg;  /*!< Pointer to the device-specific configuration */
    sid_ble_ext_conn_list_node_t         conn_list;   /*!< List of connection contexts related to this virtual BLE device. Valid for Peripheral and Central roles only */
    uint8_t                              conn_cnt;    /*!< Number of active connections */
    struct {
        sid_ble_ext_gatt_server_svc_ctx_t * svc_ctxs;
        uint8_t                             svc_ctxs_count;
    } periph;
} sid_ble_ext_virtual_device_ctx_t;

#endif /* SID_STM32_BLE_COEXISTENCE_MODE */

/**
 * @brief Global BLE driver configuration definition
 *        This configuration is user-supplied and it describes both Sidewalk and custom BLE profiles
 */
typedef struct {
    struct {
        const char *                         device_name;          /*!< This name will be used when BLE adapter is advertising Sidewalk profile. Additionally, GAP device name attribute is also set with this name */
        uint16_t                             max_att_mtu;          /*!< Maximum attribute MTU size for Sidewalk. This does not affect virtual devices */
        uint16_t                             appearance;           /*!< Value for GAP Appearance characteristic */
    } sidewalk_profile;                                            /*!< Sidewalk-specific profile definition */
#if (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_INTERLEAVING) || (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_CONCURRENCY)
    struct {
        const sid_ble_ext_virtual_device_t * virtual_devices;      /*!< Profiles of the virtual BLE devices. This parameter is valid only if Sidewalk BLE driver is used to share BLE between Sidewalk stack and User App */
        uint16_t                             num_virtual_devices;  /*!< Number of the virtual BLE devices. This parameter is valid only if Sidewalk BLE driver is used to share BLE between Sidewalk stack and User App */
    } custom_profiles;                                             /*!< Collection of the custom profiles (aka "virtual devices") defined by the end-user */
#endif /* SID_STM32_BLE_COEXISTENCE_MODE */
} sid_ble_adapter_ext_cfg_t;

#if (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_INTERLEAVING) || (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_CONCURRENCY)
typedef struct {
    /**
     * Initialize the BLE stack to run user-defined GATT profiles
     *
     * @note In SID_STM32_BLE_COEXISTENCE_MODE_CONCURRENCY this method can be used independently of Sidewalk state,
     *       the driver will manage low-level BLE initialization automatically to avoid interference with Sidewalk
     *
     * @return SID_ERROR_NONE on success, error code otherwise
     */
    sid_error_t (*init) (void);

    /**
     * De-initialize the user-defined portion of BLE stack
     *
     * @note In SID_STM32_BLE_COEXISTENCE_MODE_CONCURRENCY this method can be used independently of Sidewalk state,
     *       the driver will manage low-level BLE deinitialization automatically to avoid interference with Sidewalk
     *
     * @return SID_ERROR_NONE on success, error code otherwise
     */
    sid_error_t (*deinit) (void);

    /**
     * Do a full factory reset of the BLE
     *
     * @note This affects both Sidewalk and user-mode BLE
     *
     * @note BLE must be fully de-initialized before the factory reset, otherwise this method will fail
     *
     * @return SID_ERROR_NONE on success, error code otherwise
     */
    sid_error_t (*factory_reset) (void);

    /**
     * @brief Initialize a specific virtual BLE device
     *
     * @note This method loads device configuration into the host controller (e.g. adds GATT services, registers
     *       advertising set, etc.), but does not start any activities (e.g. advertisement)
     *
     * @param [in]  virt_dev_id ID of the device to be activated
     * @param [out] out_device_ctx Pointer to the context of the created device. If the device is initialized
     *                             already, SID_ERROR_ALREADY_INITIALIZED will be returned and out_device_ctx
     *                             will be set to point to the context of the already-created device
     * @return      SID_ERROR_NONE on success, error code otherwise
     */
    sid_error_t (*virtual_dev_activate) (const sid_ble_ext_virtual_device_id_t virt_dev_id, sid_ble_ext_virtual_device_ctx_t * * const out_device_ctx);

    /**
     * @brief De-initialize a specific virtual BLE device
     *
     * @note This method releases all the resources assigned to the specified virtual device. If there's an active
     *       connection to the device it will be terminated as part of de-initialization process
     *
     * @param [in] virt_dev_id ID of the device to be terminated
     * @return      SID_ERROR_NONE on success, error code otherwise
     */
    sid_error_t (*virtual_dev_terminate) (const sid_ble_ext_virtual_device_id_t virt_dev_id);

    /**
     * @brief Set advertising data for the specified virtual device
     *
     * @note This method is valid for Peripheral and Broadcaster roles only and it will return error for the other roles
     *
     * @param [in] virt_dev_id ID of the device
     * @param [in] adv_data Raw advertisement data as per Core Specification, except the advertising flags field. Length is limited to 28 bytes.
     *                      Necessary advertising flags are added automatically by the driver based on the config of the virtual device
     * @param [in] adv_data_length Length of the supplied raw data
     * @return     SID_ERROR_NONE on success, error code otherwise
     */
    sid_error_t (*virtual_dev_set_adv_data) (const sid_ble_ext_virtual_device_id_t virt_dev_id, const uint8_t * const adv_data, const uint32_t adv_data_length);

    /**
     * @brief Set scan response data for the specified virtual device
     *
     * @note This method is valid for Peripheral and Broadcaster roles only and it will return error for the other roles
     *
     * @param [in] virt_dev_id ID of the device
     * @param [in] scan_resp_data Raw scan response data as per Core Specification, except the advertising flags field. Length is limited to 31 bytes
     * @param [in] scan_resp_data_length Length of the supplied raw data
     * @return      SID_ERROR_NONE on success, error code otherwise
     */
    sid_error_t (*virtual_dev_set_scan_resp_data) (const sid_ble_ext_virtual_device_id_t virt_dev_id, const uint8_t * const scan_resp_data, const uint32_t scan_resp_data_length);

    /**
     * @brief Start advertising for the specified virtual device
     *
     * @note This method is valid for Peripheral and Broadcaster roles only and it will return error for the other roles
     *
     * @param [in] virt_dev_id ID of the device
     * @return     SID_ERROR_NONE on success, error code otherwise
     */
    sid_error_t (*virtual_dev_adv_start) (const sid_ble_ext_virtual_device_id_t virt_dev_id);

    /**
     * @brief Stop advertising for the specified virtual device
     *
     * @note This method is valid for Peripheral and Broadcaster roles only and it will return error for the other roles
     *
     * @param [in] virt_dev_id ID of the device
     * @return     SID_ERROR_NONE on success, error code otherwise
     */
    sid_error_t (*virtual_dev_adv_stop) (const sid_ble_ext_virtual_device_id_t virt_dev_id);

    /**
     * @brief Get the list of the remote peers that are bonded to the specified Virtual BLE device
     *
     * @note This method is valid for Peripheral and Central roles only and it will return error for the other roles
     *
     * @note This method can be called event if the BLE driver is de-initialized completely
     *
     * @param [in]  virt_dev_id ID of the device
     * @param [out] out_bonded_peers_list Receiving buffer to store the identities of the bonded peers
     * @param [out] bonded_peers_count The number of the entries that are discovered. If this amount does not feet into the list than only the bonded_peers_list_size_limit records will be copied, but this parameter will still indicate the full count
     * @param [in]  bonded_peers_list_size_limit Length limit for the out_bonded_peers_list buffer.
     * @return     SID_ERROR_NONE on success, error code otherwise. If all the available records cannot be put into the out_bonded_peers_list SID_ERROR_STORAGE_FULL will be reported
     */
    sid_error_t (*virtual_dev_get_bonded_peers) (const sid_ble_ext_virtual_device_id_t virt_dev_id, sid_pal_ble_ext_peer_identity_addr_t * const out_bonded_peers_list, uint32_t * const out_bonded_peers_count, const uint32_t bonded_peers_list_size_limit);

    /**
     * @brief Removes the remote peer bonding info from the specified Virtual BLE device non-volatile bonding context
     *
     * @note This method is valid for Peripheral and Central roles only and it will return error for the other roles
     *
     * @note It's the responsibility of the caller to ensure that BLE controller is in an appropriate state for bonding removal. This method will report an error if bonding cannot be removed immediately
     *
     * @param [in]  virt_dev_id ID of the device
     * @param [out] out_bonded_peers_list Receiving buffer to store the identities of the bonded peers
     * @param [out] bonded_peers_count The number of the entries that are discovered. If this amount does not feet into the list than only the bonded_peers_list_size_limit records will be copied, but this parameter will still indicate the full count
     * @param [in]  peer_identity Identity record of the remote peer to be deleted. See @ref virtual_dev_get_bonded_peers to get the identity record.
     * @return     SID_ERROR_NONE on success, error code otherwise. If all the available records cannot be put into the out_bonded_peers_list SID_ERROR_STORAGE_FULL will be reported
     */
    sid_error_t (*virtual_dev_remove_bond) (const sid_ble_ext_virtual_device_id_t virt_dev_id, const sid_pal_ble_ext_peer_identity_addr_t * const peer_identity);

    /**
     * @brief Disconnect the specified virtual device
     *
     * @note This method is valid for Peripheral and Central roles only and it will return error for the other roles
     *
     * @param [in] virt_dev_id ID of the device
     * @param [in] conn_id Connection handle of the connection to be terminated. THe connection must belong to the virtual device
     * @return     SID_ERROR_NONE on success, error code otherwise
     */
    sid_error_t (*virtual_dev_disconnect) (const sid_ble_ext_virtual_device_id_t virt_dev_id, const uint16_t conn_id);

    /**
     * @brief Terminate all the connections for the specified virtual device
     *
     * @note This method is valid for Peripheral and Central roles only and it will return error for the other roles
     *
     * @param [in] virt_dev_id ID of the device
     * @return     SID_ERROR_NONE on success, error code otherwise
     */
    sid_error_t (*virtual_dev_disconnect_all) (const sid_ble_ext_virtual_device_id_t virt_dev_id);

    /**
     * @brief Suspend the virtual device - all connections, advertisements, etc. will terminated, but the device won't be unloaded from Controller
     *
     * @param [in] virt_dev_id ID of the device to be suspended
     * @return     SID_ERROR_NONE on success, error code otherwise
     */
    sid_error_t (*virtual_dev_suspend) (const sid_ble_ext_virtual_device_id_t virt_dev_id);

    /**
     * @brief Resume operations for the specified virtual device. This moves the device to Idle state
     *
     * @param [in] virt_dev_id ID of the device to be resumed
     * @return     SID_ERROR_NONE on success, error code otherwise
     */
    sid_error_t (*virtual_dev_resume) (const sid_ble_ext_virtual_device_id_t virt_dev_id);

    /**
     * @brief Update characteristic value with new data and trigger GATT notification if needed
     *
     * @note This method is valid for Peripheral role only and it will return error for the other roles
     *
     * @param [in] virt_dev_id ID of the device to be resumed
     * @param [in] char_uuid UUID of the characteristic for which the value should be updated
     * @param [in] data New data to set
     * @param [in] data_length Length of the new data
     * @param [in] notify Enable notification. ANy non-zero value will send out GATT notifications for all active connections of the related virtual BLE device
     */
    sid_error_t (*virtual_dev_update_char) (const sid_ble_ext_virtual_device_id_t virt_dev_id, const sid_ble_cfg_uuid_info_t * const char_uuid, const uint8_t * const data, const uint32_t data_length, uint8_t notify);

} sid_pal_ble_adapter_extended_interface_t;
#endif /* SID_STM32_BLE_COEXISTENCE_MODE */

/* Exported functions --------------------------------------------------------*/

/**
 * @brief User-supplied function that returns BLE adapter configuration
 *
 * @note This function shall be implemented by the end user in the user application since BLE adapter config is app-specific
 * 
 * @return Pointer to the user-supplied BLE adapter configuration
 */
const sid_ble_adapter_ext_cfg_t * sid_pal_ble_adapter_ext_get_link_config(void);

#if (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_INTERLEAVING) || (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_CONCURRENCY)
sid_error_t sid_pal_ble_adapter_ext_create_extended_ifc(const sid_pal_ble_adapter_extended_interface_t * * const ext_ifc_handle);
#endif /* SID_STM32_BLE_COEXISTENCE_MODE */

/**
 * @brief Get Sidewalk BLE link configuration data
 *
 * @note User application may implement this method to override the default parameters. If the user application does not provide a definition for this function
 *       a default built-in variant is used that supplies the default settings as per Sidewalk specification for BLE link
 *
 * @return  Pointer to the Sidewalk BLE link configuration
 */
const sid_ble_link_config_t* app_get_sidewalk_ble_config(void);

#ifdef __cplusplus
}
#endif

#endif /* __SID_PAL_BLE_ADAPTER_STM32WBA_EXT_IFC_H_ */
