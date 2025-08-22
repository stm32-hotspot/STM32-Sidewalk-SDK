/**
  ******************************************************************************
  * @file    sid_pal_ble_adapter_stm32wba_config.h
  * @brief   STM32WBA-specific BLE driver configuration
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

#ifndef __SID_PAL_BLE_ADAPTER_STM32WBA_CONFIG_H_
#define __SID_PAL_BLE_ADAPTER_STM32WBA_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "app_conf.h"

#include <ble_const.h>
#include <ll_fw_config.h>

#include <sid_ble_config_ifc.h>

/* Exported constants --------------------------------------------------------*/

/* BLE security keys (IRK, ERK) */
#ifndef SID_STM32_BLE_USE_HARDCODED_SECURITY_KEYS
#  define SID_STM32_BLE_USE_HARDCODED_SECURITY_KEYS    (0) /*!< Enable usage of the hard-coded IRK and ERK keys. Exercise extreme caution when enabled and never use this mode in production */
#endif /* SID_STM32_BLE_USE_HARDCODED_SECURITY_KEYS */

#if SID_STM32_BLE_USE_HARDCODED_SECURITY_KEYS
#  if DEBUG
#    warning "The code is assembled with hard-coded BLE security keys. Don't even think of releasing this to production"
#  else
#    error "The code is assembled with hard-coded BLE security keys. Release build are not al;lowed in this configuration"
#  endif /* DEBUG */
#endif /* SID_STM32_BLE_USE_HARDCODED_SECURITY_KEYS */

#ifndef SID_STM32_BLE_RPA_ROTATION_TIMEOUT
#  define SID_STM32_BLE_RPA_ROTATION_TIMEOUT           (900u) /*!< Period (in seconds) the controller uses a Resolvable Private Address before a new resolvable private address is generated and starts being used */
#endif /* SID_STM32_BLE_RPA_ROTATION_TIMEOUT */

#ifndef SID_STM32_BLE_AUTO_MANAGE_OLD_BONDS
#  define SID_STM32_BLE_AUTO_MANAGE_OLD_BONDS          (1u) /*!< Automatically remove the oldest BLE bonds if the bonding table is full */
#endif /* SID_STM32_BLE_AUTO_MANAGE_OLD_BONDS */

#ifndef SID_STM32_BLE_BONDING_LIST_MAX_ENTRIES
#  define SID_STM32_BLE_BONDING_LIST_MAX_ENTRIES       (15u) /*!< Maximum amount of the bonded devices */
#endif /* SID_STM32_BLE_BONDING_LIST_MAX_ENTRIES */

#ifndef SID_STM32_BLE_BONDING_LIST_OVERHEAD_ENTRIES
#  define SID_STM32_BLE_BONDING_LIST_OVERHEAD_ENTRIES  (5u) /*!< Safety gap for the list of bonded devices in RAM. This amount of slots is reserved on top of the SID_STM32_BLE_BONDING_LIST_MAX_ENTRIES to avoid buffer overflow when dealing with bonding list */
#endif /* SID_STM32_BLE_BONDING_LIST_OVERHEAD_ENTRIES */

#define SID_STM32_BLE_ADDRESS_LENGTH                   (CONFIG_DATA_RANDOM_ADDRESS_LEN > CONFIG_DATA_PUBLIC_ADDRESS_LEN ? CONFIG_DATA_RANDOM_ADDRESS_LEN : CONFIG_DATA_PUBLIC_ADDRESS_LEN) /*!< Size of the BLE address, normally this should be 6 bytes */

#define SID_STM32_BLE_GAP_DEVICE_NAME_MAX_LEN          (30u) /*!< Maximum length of the device name that can be put into the corresponding GAP attribute. This does not include \0 terminator */

#ifndef SID_STM32_BLE_CLEAR_CFG_ON_FACTORY_RESET
#  define SID_STM32_BLE_CLEAR_CFG_ON_FACTORY_RESET     (1u) /*!< Clear static BLE configuration data (static random addres, IRK, ERK, etc.) on BLE factory reset. If deactivated, only the bonding information is erased */
#endif /* SID_STM32_BLE_CLEAR_CFG_ON_FACTORY_RESET */

#ifndef SID_STM32_BLE_AUTO_NEGOTIATE_MTU_SIZE
#  define SID_STM32_BLE_AUTO_NEGOTIATE_MTU_SIZE        (1u) /*!< When enabled (and if supported by the selected BLE stack option) the BLE peripheral will automatically send MTU exchange request upon connection */
#endif /* SID_STM32_BLE_AUTO_NEGOTIATE_MTU_SIZE */

#ifndef SID_STM32_BLE_AUTO_NEGOTIATE_CONN_PARAMS
#  define SID_STM32_BLE_AUTO_NEGOTIATE_CONN_PARAMS     (1u) /*!< When enabled the BLE peripheral will automatically send Connection Update upon connection to negotiate connection parameters with the Central */
#endif /* SID_STM32_BLE_AUTO_NEGOTIATE_CONN_PARAMS */

#ifndef SID_STM32_BLE_REASONABLE_CONN_TIMEOUT
#  define SID_STM32_BLE_REASONABLE_CONN_TIMEOUT        (SID_STM32_BLE_MS_TO_CONN_SUPERVISION_UNITS(5000u)) /*!< Some reasonable time for BLE connection timeout. This value is used only when it exceeds the bare minimum for the given connection interval and latency, otherwise the bare minimum timeout is enforced */
#endif /* SID_STM32_BLE_REASONABLE_CONN_TIMEOUT */

/* Data Length Extension Configuration ---------------------------------------*/

#ifndef SID_STM32_BLE_DLE_ENABLE
#  define SID_STM32_BLE_DLE_ENABLE                     (1u) /*!< Enable BLE Data Length Extension (DLE). If enabled, the LL data PDU can be extended up to 251 bytes (supported from BLE 4.2 onwards). When disabled the LL data PDU is limited to 27 bytes. This parameter affects the user mode only. For Sidewalk link DLE is permanently enabled */
#endif /* SID_STM32_BLE_DLE_ENABLE */

#ifndef SID_STM32_BLE_AUTO_NEGOTIATE_DLE_PARAMS
#  define SID_STM32_BLE_AUTO_NEGOTIATE_DLE_PARAMS      (1u) /*!< When enabled the driver will proactively send the HCI_LE_SET_DATA_LENGTH request to the remote peer to negotiate DLE parameters. When disabled the DLE will remain active, by the the remote peer must initiate the negotiation, otherwise the connection will go with the default parameters */
#endif /* SID_STM32_BLE_AUTO_NEGOTIATE_DLE_PARAMS */

#ifndef SID_STM32_BLE_DLE_MAX_TX_OCTETS
#  define SID_STM32_BLE_DLE_MAX_TX_OCTETS              (HCI_ACLDATA_MAX_DATA_LEN) /*!< Suggested Max Tx Octets value when BLE Data Length Extension is enabled. Defaults to 251 bytes */
#endif /* SID_STM32_BLE_DLE_MAX_TX_OCTETS */

#ifndef SID_STM32_BLE_DLE_MAX_TX_TIME_US
#  define SID_STM32_BLE_DLE_MAX_TX_TIME_US             (2120u) /*!< Suggested maximum number of microseconds that the local device will take to transmit a packet to the remote device. This value is used to configure BLE Data Length Extension (DLE) */
#endif /* SID_STM32_BLE_DLE_MAX_TX_TIME_US */

/*----------------------------------------------------------------------------*/

/* Specify available BLE and LL stack options */
#define SID_STM32_BLE_STACK_BASIC                      (BASIC_FEATURES)
#define SID_STM32_BLE_STACK_BASIC_PLUS                 (BASIC_PLUS)
#define SID_STM32_BLE_STACK_PERIPHERAL_ONLY            (PERIPHERAL_ONLY)
#define SID_STM32_BLE_STACK_LL_ONLY                    (LL_ONLY)
#define SID_STM32_BLE_STACK_LL_ONLY_BASIC              (LL_ONLY_BASIC)

#if (BASIC_FEATURES == 0) && (BASIC_PLUS == 0) && (PERIPHERAL_ONLY == 0) && (LL_ONLY == 0) && (LL_ONLY_BASIC == 0)
#  define SID_STM32_BLE_STACK_FULL                     (1u)
#else
#  define SID_STM32_BLE_STACK_FULL                     (0u)
#endif

/* Check for invalid config selections */
#if SID_STM32_BLE_STACK_LL_ONLY || SID_STM32_BLE_STACK_LL_ONLY_BASIC
#  error "LL-only BLE stack variants are not supported by this driver"
#endif

#if ((SID_STM32_BLE_STACK_BASIC           && (SID_STM32_BLE_STACK_BASIC_PLUS || SID_STM32_BLE_STACK_PERIPHERAL_ONLY || SID_STM32_BLE_STACK_FULL)) \
  || (SID_STM32_BLE_STACK_BASIC_PLUS      && (SID_STM32_BLE_STACK_BASIC      || SID_STM32_BLE_STACK_PERIPHERAL_ONLY || SID_STM32_BLE_STACK_FULL)) \
  || (SID_STM32_BLE_STACK_PERIPHERAL_ONLY && (SID_STM32_BLE_STACK_BASIC      || SID_STM32_BLE_STACK_BASIC_PLUS      || SID_STM32_BLE_STACK_FULL)) \
  || (SID_STM32_BLE_STACK_FULL            && (SID_STM32_BLE_STACK_BASIC      || SID_STM32_BLE_STACK_PERIPHERAL_ONLY || SID_STM32_BLE_STACK_BASIC_PLUS)))
#  error "Multiple BLE stack options selected. Please choose exactly one variant to use"
#endif

/* BLE-Sidewalk co-existence variant selection */
#define SID_STM32_BLE_COEXISTENCE_MODE_NONE            (1u) /*!< BLE can only be used by Sidewalk stack. User application is expected not to touch BLE at all - this option is recommended when the User app does not need BLE, it provides the smallest memory footprint */
#define SID_STM32_BLE_COEXISTENCE_MODE_ROUTE_NON_SID   (2u) /*!< User application may use BLE stack when Sidewalk does not use it. All stack events will be routed to the User application, which is expected to implement own event handlers - this option forwards raw BLE and LL stack events to the User app, it provides maximum flexibility */
#define SID_STM32_BLE_COEXISTENCE_MODE_INTERLEAVING    (3u) /*!< User application may use BLE stack when Sidewalk does not use it. User application is expected to use this driver to access BLE - this option allows to simplify BLE handling in the User app, but puts some constraints on what can be done */
#define SID_STM32_BLE_COEXISTENCE_MODE_CONCURRENCY     (4u) /*!< User application and Sidewalk stack may use BLE concurrently - this option enables concurrency, but requires at least Basic Plus stack option, resulting in higher memory footprint */
#ifndef SID_STM32_BLE_COEXISTENCE_MODE
#  define SID_STM32_BLE_COEXISTENCE_MODE               SID_STM32_BLE_COEXISTENCE_MODE_NONE
#endif

/* Determine if the selected BLE and LL libs support extended advertisements */
#define SID_STM32_BLE_EXTENDED_ADV_SUPPORTED           (SUPPORT_LE_EXTENDED_ADVERTISING)

/* Determine if extended advertisement are mandatory for the selected config */
#if (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_CONCURRENCY)
#  define SID_STM32_BLE_EXTENDED_ADV_REQUIRED          (1u)
#else
#  define SID_STM32_BLE_EXTENDED_ADV_REQUIRED          (0u)
#endif
/* Validity check */
#if SID_STM32_BLE_EXTENDED_ADV_REQUIRED && (SID_STM32_BLE_EXTENDED_ADV_SUPPORTED == 0)
#  error "The selected BLE driver configuration requires extended advertisement support, but the selected stack option does not have it. Either change the driver configuration or select the matching BLE stack variant"
#endif

/* GAP roles supported by the selected BLE stack variant */
#define SID_STM32_BLE_GAP_ROLE_OBSERVER_SUPPORTED      (SUPPORT_EXPLCT_OBSERVER_ROLE)
#define SID_STM32_BLE_GAP_ROLE_BROADCASTER_SUPPORTED   (SUPPORT_EXPLCT_BROADCASTER_ROLE)
#define SID_STM32_BLE_GAP_ROLE_CENTRAL_SUPPORTED       (SUPPORT_MASTER_CONNECTION)
#define SID_STM32_BLE_GAP_ROLE_PERIPHERAL_SUPPORTED    (SUPPORT_SLAVE_CONNECTION)

#define SID_STM32_BLE_GAP_MAX_ADV_SETS_NUM             (8u)                              /*<! Maximum number of advertisement sets the driver can handle if extended advertisements are supported by the BLE stack */

/* GATT features */
#define SID_STM32_BLE_GATT_EXCHANGE_CONFIG_SUPPORTED   (PERIPHERAL_ONLY == 0)

#ifndef SID_STM32_BLE_USE_RANDOM_IDENTITY_ADDRESS
#  define SID_STM32_BLE_USE_RANDOM_IDENTITY_ADDRESS    (1u)    /*!< When RPA is enabled, use Static Random address as Identity Address. If disabled, Public Address will be used as Identity Address - this mode is not recommended because it exposes Public Address to external devices */
#endif /* SID_STM32_BLE_USE_RANDOM_IDENTITY_ADDRESS */

/* Sidewalk-related features and limitations */
#define SID_STM32_BLE_SIDEWALK_ATT_MTU_MAX             (250u)
#define SID_STM32_BLE_SIDEWALK_ATT_MTU_MIN             (185u)

#define SID_STM32_BLE_SIDEWALK_NUM_GATT_SERVICES       (2u)    /*!< Maximum number of GATT services the Sidewalk link may instantiate */

#ifndef SID_STM32_BLE_SIDEWALK_ADV_WINDOW_MS
#  define SID_STM32_BLE_SIDEWALK_ADV_WINDOW_MS         (1)     /*!< BLE advertising interval will be set at +/- half of this window for each advertisement transmission, and BLE stack will add random delay of 0-10ms to that */
#endif /* SID_STM32_BLE_SIDEWALK_ADV_WINDOW_MS */

/* Concurrent connections and MTU limits - used to determine RAM buffer sizes */
#if (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_NONE) || (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_ROUTE_NON_SID)
#  define SID_STM32_BLE_HOST_MAX_NUM_LINK              (1u)                                            /* Sidewalk requires exactly one connection, non-Sidewalk connections are handled entirely by the User App */
#  define SID_STM32_BLE_HOST_MAX_ATT_MTU               (SID_STM32_BLE_SIDEWALK_ATT_MTU_MAX)            /* Reserve MTU size up to the maximum allowed for Sidewalk connection */
#  define SID_STM32_BLE_HOST_MAX_NUM_GATT_SERVICES     (SID_STM32_BLE_SIDEWALK_NUM_GATT_SERVICES)
#elif (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_INTERLEAVING)
#  define SID_STM32_BLE_HOST_MAX_NUM_LINK              (CFG_BLE_NUM_LINK > 0u ? CFG_BLE_NUM_LINK : 1u) /* Use the application settings */
#  define SID_STM32_BLE_HOST_MAX_ATT_MTU               (SID_STM32_BLE_SIDEWALK_ATT_MTU_MAX > CFG_BLE_ATT_MTU_MAX ? SID_STM32_BLE_SIDEWALK_ATT_MTU_MAX : CFG_BLE_ATT_MTU_MAX)
#  define SID_STM32_BLE_HOST_MAX_NUM_GATT_SERVICES     (CFG_BLE_NUM_GATT_SERVICES > SID_STM32_BLE_SIDEWALK_NUM_GATT_SERVICES ? CFG_BLE_NUM_GATT_SERVICES : SID_STM32_BLE_SIDEWALK_NUM_GATT_SERVICES)
#else /* SID_STM32_BLE_COEXISTENCE_MODE_CONCURRENCY */
#  define SID_STM32_BLE_HOST_MAX_NUM_LINK              (CFG_BLE_NUM_LINK + 1u)                         /* Reserve one link for Sidewalk on top of the application settings */
#  define SID_STM32_BLE_HOST_MAX_ATT_MTU               (SID_STM32_BLE_SIDEWALK_ATT_MTU_MAX > CFG_BLE_ATT_MTU_MAX ? SID_STM32_BLE_SIDEWALK_ATT_MTU_MAX : CFG_BLE_ATT_MTU_MAX)
#  define SID_STM32_BLE_HOST_MAX_NUM_GATT_SERVICES     (SID_STM32_BLE_SIDEWALK_NUM_GATT_SERVICES + CFG_BLE_NUM_GATT_SERVICES)
#endif /* SID_STM32_BLE_COEXISTENCE_MODE */

#define SID_STM32_BLE_PRV_SVC_HANDLERS_MAX_NUM         (2u) /*!< Maximum amount of private SVC handlers maintained by the driver */

/* Enhanced MTU Exchange handling - allows to negotiate MTU size on per-connection basis */
#ifndef SID_STM32_BLE_ENHANCED_MTU_EXCHANGE_HANDLING
/* Enable enhanced MTU exchange handling for concurrency and interleaving modes */
#  define SID_STM32_BLE_ENHANCED_MTU_EXCHANGE_HANDLING ((SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_CONCURRENCY) || (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_INTERLEAVING))
#endif /* SID_STM32_BLE_ENHANCED_MTU_EXCHANGE_HANDLING */

#define SID_STM32_BLE_DISCONNECT_TIMEOUT_ALLOWANCE_MS  (10u) /*!< The actual disconnect timeout is set to connection supervision timeout (specific to the connection being terminated) plus this value */

/* Exported types ------------------------------------------------------------*/

typedef enum sid_ble_cfg_mac_address_type sid_ble_cfg_mac_address_type_t;

#ifdef __cplusplus
}
#endif

#endif /* __SID_PAL_BLE_ADAPTER_STM32WBA_CONFIG_H_ */
