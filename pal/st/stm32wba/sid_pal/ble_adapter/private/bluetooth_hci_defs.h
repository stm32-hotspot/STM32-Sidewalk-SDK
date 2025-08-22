/**
  ******************************************************************************
  * @file    bluetooth_hci_defs.h
  * @brief   Definitions and constants from Bluetooth Core specification
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

#ifndef __SID_PAL_STM32_BLUETOOTH_HCI_DEFS_H_
#define __SID_PAL_STM32_BLUETOOTH_HCI_DEFS_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Bluetooth status codes as per Core Specification Volume 1, Part F */
#define SID_BLE_HCI_STATUS_SUCCESS                        (0x00u) /*!< Success */
#define SID_BLE_HCI_STATUS_UNKNOWN_HCI_CMD                (0x01u) /*!< Unknown HCI Command */
#define SID_BLE_HCI_STATUS_UNKNOWN_CONN_ID                (0x02u) /*!< Unknown Connection Identifier */
#define SID_BLE_HCI_STATUS_HARDWARE_FAILURE               (0x03u) /*!< Hardware Failure */
#define SID_BLE_HCI_STATUS_PAGE_TIMEOUT                   (0x04u) /*!< Page Timeout */
#define SID_BLE_HCI_STATUS_AUTH_FAILURE                   (0x05u) /*!< Authentication Failure */
#define SID_BLE_HCI_STATUS_PIN_KEY_MISSING                (0x06u) /*!< PIN or Key Missing */
#define SID_BLE_HCI_STATUS_OUT_OF_MEM                     (0x07u) /*!< Memory Capacity Exceeded */
#define SID_BLE_HCI_STATUS_CONN_TIMEOUT                   (0x08u) /*!< Connection Timeout */
#define SID_BLE_HCI_STATUS_CONN_LIMIT_EXCEEDED            (0x09u) /*!< Connection Limit Exceeded */
#define SID_BLE_HCI_STATUS_SYNC_CONN_LIMIT_EXCEEDED       (0x0Au) /*!< Synchronous Connection Limit To A Device Exceeded */
#define SID_BLE_HCI_STATUS_CONN_EXISTS                    (0x0Bu) /*!< Connection Already Exists */
#define SID_BLE_HCI_STATUS_CMD_DISALLOWED                 (0x0Cu) /*!< Command Disallowed */
#define SID_BLE_HCI_STATUS_CONN_OUT_OF_RESOURCES          (0x0Du) /*!< Connection Rejected due to Limited Resources */
#define SID_BLE_HCI_STATUS_CONN_SECURITY                  (0x0Eu) /*!< Connection Rejected Due To Security Reasons */
#define SID_BLE_HCI_STATUS_CONN_INVALID_BD_ADDR           (0x0Fu) /*!< Connection Rejected due to Unacceptable BD_ADDR */
#define SID_BLE_HCI_STATUS_CONN_ACCPET_TIMEOUT            (0x10u) /*!< Connection Accept Timeout Exceeded */
#define SID_BLE_HCI_STATUS_NOT_SUPPORTED                  (0x11u) /*!< Unsupported Feature or Parameter Value */
#define SID_BLE_HCI_STATUS_INVALID_HCI_PARAMS             (0x12u) /*!< Invalid HCI Command Parameters */
#define SID_BLE_HCI_STATUS_REMOTE_SIDE_TERMINATED         (0x13u) /*!< Remote User Terminated Connection */
#define SID_BLE_HCI_STATUS_REMOTE_SIDE_OUT_OF_RESOURCES   (0x14u) /*!< Remote Device Terminated Connection due to Low Resources */
#define SID_BLE_HCI_STATUS_REMOTE_SIDE_POWER_OFF          (0x15u) /*!< Remote Device Terminated Connection due to Power Off */
#define SID_BLE_HCI_STATUS_HOST_SIDE_TERMINATED           (0x16u) /*!< Connection Terminated By Local Host */
#define SID_BLE_HCI_STATUS_REPEATED_ATTEMPTS              (0x17u) /*!< Repeated Attempts */
#define SID_BLE_HCI_STATUS_PAIRING_NOT_ALLOWED            (0x18u) /*!< Pairing Not Allowed */
#define SID_BLE_HCI_STATUS_UNKNOWN_LPM_PDU                (0x19u) /*!< Unknown LMP PDU */
#define SID_BLE_HCI_STATUS_UNSUPPORTED_REMOTE_FEATURE     (0x1Au) /*!< Unsupported Remote Feature */
#define SID_BLE_HCI_STATUS_SCO_OFFSET_REJECTED            (0x1Bu) /*!< SCO Offset Rejected */
#define SID_BLE_HCI_STATUS_SCO_INTERVAL_REJECTED          (0x1Cu) /*!< SCO Interval Rejected */
#define SID_BLE_HCI_STATUS_SCO_AIR_MODE_REJECTED          (0x1Du) /*!< SCO Air Mode Rejected */
#define SID_BLE_HCI_STATUS_INVALID_LMP_LL_PARAMS          (0x1Eu) /*!< Invalid LMP Parameters / Invalid LL Parameters */
#define SID_BLE_HCI_STATUS_UNSPECIFIED_ERROR              (0x1Fu) /*!< Unspecified Error */
#define SID_BLE_HCI_STATUS_UNSUPPORTED_LMP_LL_PARAMS      (0x20u) /*!< Unsupported LMP Parameter Value / Unsupported LL Parameter Value */
#define SID_BLE_HCI_STATUS_ROLE_CHANGE_NOT_ALLOWED        (0x21u) /*!< Role Change Not Allowed */
#define SID_BLE_HCI_STATUS_LMP_LL_RESP_TIMEOUT            (0x22u) /*!< LMP Response Timeout / LL Response Timeout */
#define SID_BLE_HCI_STATUS_LMP_LL_PROC_COLLISION          (0x23u) /*!< LMP Error Transaction Collision / LL Procedure Collision */
#define SID_BLE_HCI_STATUS_LMP_PDU_NOT_ALLOWED            (0x24u) /*!< LMP PDU Not Allowed */
#define SID_BLE_HCI_STATUS_ENCRYPT_MODE_NOT_ACCEPTABLE    (0x25u) /*!< Encryption Mode Not Acceptable */
#define SID_BLE_HCI_STATUS_LINL_KEY_CHANGE_DISALLOWED     (0x26u) /*!< Link Key cannot be Changed */
#define SID_BLE_HCI_STATUS_REQUEST_QOS_NOT_SUPPORTED      (0x27u) /*!< Requested QoS Not Supported */
#define SID_BLE_HCI_STATUS_INSTANT_PASSED                 (0x28u) /*!< Instant Passed */
#define SID_BLE_HCI_STATUS_UNIT_KEY_PAIRING_NOT_SUPPORTED (0x29u) /*!< Pairing With Unit Key Not Supported */
#define SID_BLE_HCI_STATUS_TRANSACTIONS_COLLISION         (0x2Au) /*!< Different Transaction Collision */
#define SID_BLE_HCI_STATUS_QOS_UNACCEPTABLE_PARAMS        (0x2Cu) /*!< QoS Unacceptable Parameter */
#define SID_BLE_HCI_STATUS_QOS_REJECTED                   (0x2Du) /*!< QoS Rejected */
#define SID_BLE_HCI_STATUS_CHAN_CLASS_NOT_SUPPORTED       (0x2Eu) /*!< Channel Classification Not Supported */
#define SID_BLE_HCI_STATUS_INSUFFICIENT_SECURITY          (0x2Fu) /*!< Insufficient Security */
#define SID_BLE_HCI_STATUS_PARAM_OUT_OF_RANGE             (0x30u) /*!< Parameter Out Of Mandatory Range */
#define SID_BLE_HCI_STATUS_ROLE_SWITCH_PENDING            (0x32u) /*!< Role Switch Pending */
#define SID_BLE_HCI_STATUS_SLOT_VIOLATION                 (0x34u) /*!< Reserved Slot Violation */
#define SID_BLE_HCI_STATUS_ROLE_SWITCH_FAILED             (0x35u) /*!< Role Switch Failed */
#define SID_BLE_HCI_STATUS_EXT_INQ_RESP_TOO_LARGE         (0x36u) /*!< Extended Inquiry Response Too Large */
#define SID_BLE_HCI_STATUS_SEC_SIMP_PAIR_NOT_SUPPORTED    (0x37u) /*!< Secure Simple Pairing Not Supported By Host */
#define SID_BLE_HCI_STATUS_HOST_BUSY_PAIRING              (0x38u) /*!< Host Busy - Pairing */
#define SID_BLE_HCI_STATUS_CONN_NO_SUITABLE_CHAN          (0x39u) /*!< Connection Rejected due to No Suitable Channel Found */
#define SID_BLE_HCI_STATUS_CONTROLLER_BUSY                (0x3Au) /*!< Controller Busy */
#define SID_BLE_HCI_STATUS_UNACCEPTABLE_CONN_PARAMS       (0x3Bu) /*!< Unacceptable Connection Parameters */
#define SID_BLE_HCI_STATUS_ADV_TIMEOUT                    (0x3Cu) /*!< Advertising Timeout */
#define SID_BLE_HCI_STATUS_MCI_FAILURE                    (0x3Du) /*!< Connection Terminated due to MIC Failure */
#define SID_BLE_HCI_STATUS_CONN_SYNC_TIMEOUT              (0x3Eu) /*!< Connection Failed to be Established / Synchronization Timeout */
#define SID_BLE_HCI_STATUS_PREVIOUSLY_USED                (0x3Fu) /*!< Previously used */
#define SID_BLE_HCI_STATUS_COARSE_CLK_ADJ_REJECTED        (0x40u) /*!< Coarse Clock Adjustment Rejected but Will Try to Adjust Using Clock Dragging */
#define SID_BLE_HCI_STATUS_TYPE0_SUBMAP_NOT_DEFINED       (0x41u) /*!< Type0 Submap Not Defined */
#define SID_BLE_HCI_STATUS_UNKNOWN_ADV_IDENTIFIER         (0x42u) /*!< Unknown Advertising Identifier */
#define SID_BLE_HCI_STATUS_LIMIT_REACHED                  (0x43u) /*!< Limit Reached */
#define SID_BLE_HCI_STATUS_CANCELLED_BY_HOST              (0x44u) /*!< Operation Cancelled by Host */
#define SID_BLE_HCI_STATUS_PACKET_TOO_LONG                (0x45u) /*!< Packet Too Long */
#define SID_BLE_HCI_STATUS_TOO_LATE                       (0x46u) /*!< Too Late */
#define SID_BLE_HCI_STATUS_TOO_EARLY                      (0x47u) /*!< Too Early */

/*----------------------------------------------------------------------------*/

/* HCI LE Event Mask as per Core Specification Volume 4, Part 4, Section 7.8.1 */
 #define SID_BLE_HCI_LE_EVENT_MASK_CONN_COMPLETE                          ((uint64_t)1U <<  0) /*!< LE Connection Complete event */
 #define SID_BLE_HCI_LE_EVENT_MASK_ADV_REPORT                             ((uint64_t)1U <<  1) /*!< LE Advertising Report event */
 #define SID_BLE_HCI_LE_EVENT_MASK_CONN_UPDATE_COMPLETE                   ((uint64_t)1U <<  2) /*!< LE Connection Update Complete event */
 #define SID_BLE_HCI_LE_EVENT_MASK_READ_REMOTE_FEATURES_COMPLETE          ((uint64_t)1U <<  3) /*!< LE Read Remote Features Complete event */
 #define SID_BLE_HCI_LE_EVENT_MASK_LTK_REQUEST                            ((uint64_t)1U <<  4) /*!< LE Long Term Key Request event */
 #define SID_BLE_HCI_LE_EVENT_MASK_REMOTE_CONN_PARAMETER_REQ              ((uint64_t)1U <<  5) /*!< LE Remote Connection Parameter Request event */
 #define SID_BLE_HCI_LE_EVENT_MASK_DATA_LENGTH_CHANGE                     ((uint64_t)1U <<  6) /*!< LE Data Length Change event */
 #define SID_BLE_HCI_LE_EVENT_MASK_READ_LOCAL_P256_KEY_COMPLETE           ((uint64_t)1U <<  7) /*!< LE Read Local P-256 Public Key Complete event */
 #define SID_BLE_HCI_LE_EVENT_MASK_GENERATE_DHKEY_COMPLETE                ((uint64_t)1U <<  8) /*!< LE Generate DHKey Complete event */
 #define SID_BLE_HCI_LE_EVENT_MASK_ENHANCED_CONN_COMPLETE                 ((uint64_t)1U <<  9) /*!< LE Enhanced Connection Complete event */
 #define SID_BLE_HCI_LE_EVENT_MASK_DIRECTED_ADV_REPORT                    ((uint64_t)1U << 10) /*!< LE Directed Advertising Report event */
 #define SID_BLE_HCI_LE_EVENT_MASK_PHY_UPDATE_COMPLETE                    ((uint64_t)1U << 11) /*!< LE PHY Update Complete event */
 #define SID_BLE_HCI_LE_EVENT_MASK_EXTENDED_ADV_REPORT                    ((uint64_t)1U << 12) /*!< LE Extended Advertising Report event */
 #define SID_BLE_HCI_LE_EVENT_MASK_PERIODIC_ADV_SYNC_ESTABLISHED          ((uint64_t)1U << 13) /*!< LE Periodic Advertising Sync Established event */
 #define SID_BLE_HCI_LE_EVENT_MASK_PERIODIC_ADV_REPORT                    ((uint64_t)1U << 14) /*!< LE Periodic Advertising Report event */
 #define SID_BLE_HCI_LE_EVENT_MASK_PERIODIC_ADV_SYNC_LOST                 ((uint64_t)1U << 15) /*!< LE Periodic Advertising Sync Lost event */
 #define SID_BLE_HCI_LE_EVENT_MASK_SCAN_TIMEOUT                           ((uint64_t)1U << 16) /*!< LE Scan Timeout event */
 #define SID_BLE_HCI_LE_EVENT_MASK_ADV_SET_TERMINATED                     ((uint64_t)1U << 17) /*!< LE Advertising Set Terminated event */
 #define SID_BLE_HCI_LE_EVENT_MASK_SCAN_REQUEST_RECEIVED                  ((uint64_t)1U << 18) /*!< LE Scan Request Received event */
 #define SID_BLE_HCI_LE_EVENT_MASK_CHANNEL_SELECTION_ALGO                 ((uint64_t)1U << 19) /*!< LE Channel Selection Algorithm event */
 #define SID_BLE_HCI_LE_EVENT_MASK_CONNECTIONLESS_IQ_REPORT               ((uint64_t)1U << 20) /*!< LE Connectionless IQ Report event */
 #define SID_BLE_HCI_LE_EVENT_MASK_CONN_IQ_REPORT                         ((uint64_t)1U << 21) /*!< LE Connection IQ Report event */
 #define SID_BLE_HCI_LE_EVENT_MASK_CTE_REQUEST_FAILED                     ((uint64_t)1U << 22) /*!< LE CTE Request Failed event */
 #define SID_BLE_HCI_LE_EVENT_MASK_PERIODIC_ADV_SYNC_TRANSFER_RECEIVED    ((uint64_t)1U << 23) /*!< LE Periodic Advertising Sync Transfer Received event */
 #define SID_BLE_HCI_LE_EVENT_MASK_CIS_ESTABLISHED                        ((uint64_t)1U << 24) /*!< LE CIS Established event */
 #define SID_BLE_HCI_LE_EVENT_MASK_CIS_REQUEST                            ((uint64_t)1U << 25) /*!< LE CIS Request event */
 #define SID_BLE_HCI_LE_EVENT_MASK_CREATE_BIG_COMPLETE                    ((uint64_t)1U << 26) /*!< LE Create BIG Complete event */
 #define SID_BLE_HCI_LE_EVENT_MASK_TERMINATE_BIG_COMPLETE                 ((uint64_t)1U << 27) /*!< LE Terminate BIG Complete event */
 #define SID_BLE_HCI_LE_EVENT_MASK_BIG_SYNC_ESTABLISHED                   ((uint64_t)1U << 28) /*!< LE BIG Sync Established event */
 #define SID_BLE_HCI_LE_EVENT_MASK_BIG_SYNC_LOST                          ((uint64_t)1U << 29) /*!< LE BIG Sync Lost event */
 #define SID_BLE_HCI_LE_EVENT_MASK_REQUEST_PEER_SCA_COMPLETE              ((uint64_t)1U << 30) /*!< LE Request Peer SCA Complete event */
 #define SID_BLE_HCI_LE_EVENT_MASK_PATH_LOSS_THRESHOLD                    ((uint64_t)1U << 31) /*!< LE Path Loss Threshold event */
 #define SID_BLE_HCI_LE_EVENT_MASK_TRANSMIT_POWER_REPORTING               ((uint64_t)1U << 32) /*!< LE Transmit Power Reporting event */
 #define SID_BLE_HCI_LE_EVENT_MASK_BIGINFO_ADV_REPORT                     ((uint64_t)1U << 33) /*!< LE BIGInfo Advertising Report event */
 #define SID_BLE_HCI_LE_EVENT_MASK_SUBRATE_CHANGE                         ((uint64_t)1U << 34) /*!< LE Subrate Change event */
 #define SID_BLE_HCI_LE_EVENT_MASK_PERIODIC_ADV_SYNC_ESTABLISHED_V2       ((uint64_t)1U << 35) /*!< LE Periodic Advertising Sync Established event [v2] */
 #define SID_BLE_HCI_LE_EVENT_MASK_PERIODIC_ADV_REPORT_V2                 ((uint64_t)1U << 36) /*!< LE Periodic Advertising Report event [v2] */
 #define SID_BLE_HCI_LE_EVENT_MASK_PERIODIC_ADV_SYNC_TRANSFER_RECEIVED_V2 ((uint64_t)1U << 37) /*!< LE Periodic Advertising Sync Transfer Received event [v2] */
 #define SID_BLE_HCI_LE_EVENT_MASK_PERIODIC_ADV_SUBEVT_DATA_REQUEST       ((uint64_t)1U << 38) /*!< LE Periodic Advertising Subevent Data Request event */
 #define SID_BLE_HCI_LE_EVENT_MASK_PERIODIC_ADV_RESPONSE_REPORT           ((uint64_t)1U << 39) /*!< LE Periodic Advertising Response Report event */
 #define SID_BLE_HCI_LE_EVENT_MASK_ENHANCED_CONN_COMPLETE_V2              ((uint64_t)1U << 40) /*!< LE Enhanced Connection Complete event [v2] */

/*----------------------------------------------------------------------------*/

/* HCI_LE_Set_Extended_Advertising_Data:Fragment_Preference */
#define SID_BLE_HCI_SET_ADV_DATA_FRAGMENT_PREF_MAY_FRAGMENT (0x00u) /*!< The Controller may fragment all scan response data */
#define SID_BLE_HCI_SET_ADV_DATA_FRAGMENT_PREF_NO_FRAGMENT  (0x01u) /*!< The Controller should not fragment or should minimize fragmentation of scan response data */

/* HCI_LE_Set_Extended_Advertising_Parameters:Peer_Address_Type */
#define SID_BLE_HCI_ADV_PEER_ADDR_TYPE_PUBLIC               (0x00u) /*!< Public Device Address */
#define SID_BLE_HCI_ADV_PEER_ADDR_TYPE_RANDOM               (0x01u) /*!< Random Device Address */
#define SID_BLE_HCI_ADV_PEER_ADDR_TYPE_RPA_PUBLIC_IDENTITY  (0x02u) /*!< Public Identity Address (Corresponds to Resolved Private Address) */
#define SID_BLE_HCI_ADV_PEER_ADDR_TYPE_RPA_RANDOM_IDENTITY  (0x03u) /*!< Random (Static) Identity Address (Corresponds to Resolved Private Address) */

/* HCI_LE_Set_Extended_Advertising_Parameters:Own_Address_Type */
#define SID_BLE_HCI_ADV_OWN_ADDR_TYPE_PUBLIC                (0x00u)
#define SID_BLE_HCI_ADV_OWN_ADDR_TYPE_RANDOM                (0x01u)
#define SID_BLE_HCI_ADV_OWN_ADDR_TYPE_RESOLVABLE_OR_PUBLIC  (0x02u)
#define SID_BLE_HCI_ADV_OWN_ADDR_TYPE_RESOLVABLE_OR_RANDOM  (0x03u)

/* HCI_LE_Set_Extended_Advertising_Parameters:Advertising_Handle */
#define SID_BLE_HCI_ADV_HANDLE_MAX                          (0xEFu)

/* HCI_LE_Set_Extended_Advertising_Parameters:Adv_TX_Power */
#define SID_BLE_HCI_ADV_TX_POWER_MIN                        (-127)
#define SID_BLE_HCI_ADV_TX_POWER_MAX                        (20)
#define SID_BLE_HCI_ADV_TX_POWER_NO_PREFERENCE              (127)

/* HCI_LE_Set_Extended_Advertising_Enable:Enable */
#define SID_BLE_HCI_SET_ADV_DISABLE                         (0x00u)
#define SID_BLE_HCI_SET_ADV_ENABLE                          (0x01u)

/* HCI_LE_Enhanced_Connection_Complete:Role */
#define SID_BLE_HCI_CONN_COMPLETE_ROLE_CENTRAL              (0x00u)
#define SID_BLE_HCI_CONN_COMPLETE_ROLE_PERIPHERAL           (0x01u)

/* Advertisement interval limits */
#define SID_BLE_HCI_ADV_INTERVAL_MIN                        (0x0020u)   /* As per BLE 5.4 spec */
#define SID_BLE_HCI_ADV_INTERVAL_MAX                        (0x4000u)   /* As per BLE 5.4 spec */
#define SID_BLE_HCI_EXTENDED_ADV_INTERVAL_MIN               (0x000020u) /* As per BLE 5.4 spec */
#define SID_BLE_HCI_EXTENDED_ADV_INTERVAL_MAX               (0xFFFFFFu) /* As per BLE 5.4 spec */

/* Advertisement data tags and sizes */
#define SID_BLE_HCI_ADV_RECORD_LENGTH_SIZE                  (0x01u) /*!< Length of the Record Length field */
#define SID_BLE_HCI_ADV_RECORD_SIZE_MIN                     (0x01u) /*!< Minimum length of the record - 1 byte */
#define SID_BLE_HCI_ADV_TYPE_RECORD_SIZE                    (0x01u) /*!< Length of the Record Type field */
#define SID_BLE_HCI_ADV_FLAGS_RECORD_SIZE                   (0x01u) /*!< Length of the Advertising Flags record */
#define SID_BLE_HCI_ADV_MFG_DATA_MANUFACTURER_ID_LENGTH     (0x02u) /*!< Length of the Manufacturer ID record */

/*----------------------------------------------------------------------------*/

#define SID_BLE_HCI_DEFAULT_ATT_MTU                         (23u) /*!< Default MTU size for new connections */

/*----------------------------------------------------------------------------*/

#define SID_BLE_HCI_INVALID_RSSI_VALUE                      (127u) /*!< 127 is returned by HCI when RSSI cannot be read. See Bluetooth spec. v.6.0 [Vol 4, Part E, 7.5.4] */

/*----------------------------------------------------------------------------*/

#define SID_BLE_HCI_CONN_INTERVAL_LOWER_LIMIT               (0x0006u) /*!< Minimum value for Connection Interval parameter, expressed in 1.25ms units (7.5ms) */
#define SID_BLE_HCI_CONN_INTERVAL_UPPER_LIMIT               (0x0C80u) /*!< Maximum value for Connection Interval parameter, expressed in 1.25ms units (4000ms) */

#define SID_BLE_HCI_CONN_LATENCY_LOWER_LIMIT                (0x0000u) /*!< Minimum value for Connection Latency parameter, expressed in the number of connection events */
#define SID_BLE_HCI_CONN_LATENCY_UPPER_LIMIT                (0x01F3u) /*!< Maximum value for Connection Latency parameter, expressed in the number of connection events */

#define SID_BLE_HCI_SUPERVISION_TIMEOUT_LOWER_LIMIT         (0x000Au) /*!< Minimum value for Connection Supervision Timeout parameter, expressed in 10ms units (100ms) */
#define SID_BLE_HCI_SUPERVISION_TIMEOUT_UPPER_LIMIT         (0x0C80u) /*!< Maximum value for Connection Supervision Timeout parameter, expressed in 10ms units (32000ms) */

#define SID_BLE_HCI_CONN_EVENT_LENGTH_LOWER_LIMIT           (0x0000u) /*!< Minimum value for Connection Event Length parameter, expressed in 0.625ms units (0ms) */
#define SID_BLE_HCI_CONN_EVENT_LENGTH_UPPER_LIMIT           (0xFFFFu) /*!< Maximum value for Connection Event Length parameter, expressed in 0.625ms units (40959.375ms) */

#ifdef __cplusplus
}
#endif

#endif /* __SID_PAL_STM32_BLUETOOTH_HCI_DEFS_H_ */
