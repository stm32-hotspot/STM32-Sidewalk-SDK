/**
  ******************************************************************************
  * @file    mfg_store_offsets.h
  * @brief   Fixed offsets for manufacturing data storage on STM32WBAxx MCUs
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023-2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#ifndef __SID_PAL_MFG_STORE_STM32WBAXX_OFFSETS_H_
#define __SID_PAL_MFG_STORE_STM32WBAXX_OFFSETS_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include <sid_pal_mfg_store_ifc.h>

/* Exported macro ------------------------------------------------------------*/

/* This enumeration stores 16-byte word offset constants for the contents of the
 * manufacturing store. Each value will consume the smallest number of 16-byte
 * words capable of storing the value based on on the value size.
 *
 * Take care when adding new values to maintain backward compatibility.
 */

#define MFG_WORD_SIZE                       (16u) /* in bytes */
#define MFG_ALIGN_TO_WORD_BOUNDARY(_VALUE_) ((((size_t)(_VALUE_) + (MFG_WORD_SIZE - 1u)) / MFG_WORD_SIZE) * MFG_WORD_SIZE)

/* Exported types ------------------------------------------------------------*/

#if SID_PAL_SHORT_FORM_CERT_CHAIN_ENABLED

enum {
    SID_PAL_MFG_STORE_OFFSET_VERSION                          = 0,
    SID_PAL_MFG_STORE_OFFSET_DEVID                            = 1,
    SID_PAL_MFG_STORE_OFFSET_SERIAL_NUM                       = 8,
    SID_PAL_MFG_STORE_OFFSET_DEVICE_PRIV_ED25519              = 25,
    SID_PAL_MFG_STORE_OFFSET_DEVICE_PUB_ED25519               = 27,
    SID_PAL_MFG_STORE_OFFSET_DEVICE_PUB_ED25519_SIGNATURE     = 29,
    SID_PAL_MFG_STORE_OFFSET_DEVICE_PRIV_P256R1               = 33,
    SID_PAL_MFG_STORE_OFFSET_DEVICE_PUB_P256R1                = 35,
    SID_PAL_MFG_STORE_OFFSET_DEVICE_PUB_P256R1_SIGNATURE      = 39,
    SID_PAL_MFG_STORE_OFFSET_PRODUCT_PUB_ED25519              = 43,
    SID_PAL_MFG_STORE_OFFSET_PRODUCT_PUB_ED25519_SIGNATURE    = 45,
    SID_PAL_MFG_STORE_OFFSET_PRODUCT_ED25519_SERIAL           = 49,
    SID_PAL_MFG_STORE_OFFSET_PRODUCT_PUB_P256R1               = 50,
    SID_PAL_MFG_STORE_OFFSET_PRODUCT_PUB_P256R1_SIGNATURE     = 54,
    SID_PAL_MFG_STORE_OFFSET_PRODUCT_P256R1_SERIAL            = 58,
    SID_PAL_MFG_STORE_OFFSET_MAN_PUB_ED25519                  = 59,
    SID_PAL_MFG_STORE_OFFSET_MAN_PUB_ED25519_SIGNATURE        = 61,
    SID_PAL_MFG_STORE_OFFSET_MAN_ED25519_SERIAL               = 65,
    SID_PAL_MFG_STORE_OFFSET_MAN_PUB_P256R1                   = 66,
    SID_PAL_MFG_STORE_OFFSET_MAN_PUB_P256R1_SIGNATURE         = 70,
    SID_PAL_MFG_STORE_OFFSET_MAN_P256R1_SERIAL                = 74,
    SID_PAL_MFG_STORE_OFFSET_AMZN_PUB_ED25519                 = 91,
    SID_PAL_MFG_STORE_OFFSET_AMZN_PUB_P256R1                  = 93,

    SID_PAL_MFG_STORE_OFFSET_SMSN                             = SID_PAL_MFG_STORE_INVALID_OFFSET,
    SID_PAL_MFG_STORE_OFFSET_DAK_PUB_ED25519                  = SID_PAL_MFG_STORE_INVALID_OFFSET,
    SID_PAL_MFG_STORE_OFFSET_DAK_PUB_ED25519_SIGNATURE        = SID_PAL_MFG_STORE_INVALID_OFFSET,
    SID_PAL_MFG_STORE_OFFSET_DAK_ED25519_SERIAL               = SID_PAL_MFG_STORE_INVALID_OFFSET,
    SID_PAL_MFG_STORE_OFFSET_DAK_PUB_P256R1                   = SID_PAL_MFG_STORE_INVALID_OFFSET,
    SID_PAL_MFG_STORE_OFFSET_DAK_PUB_P256R1_SIGNATURE         = SID_PAL_MFG_STORE_INVALID_OFFSET,
    SID_PAL_MFG_STORE_OFFSET_DAK_P256R1_SERIAL                = SID_PAL_MFG_STORE_INVALID_OFFSET,
    SID_PAL_MFG_STORE_OFFSET_SW_PUB_ED25519                   = SID_PAL_MFG_STORE_INVALID_OFFSET,
    SID_PAL_MFG_STORE_OFFSET_SW_PUB_ED25519_SIGNATURE         = SID_PAL_MFG_STORE_INVALID_OFFSET,
    SID_PAL_MFG_STORE_OFFSET_SW_ED25519_SERIAL                = SID_PAL_MFG_STORE_INVALID_OFFSET,
    SID_PAL_MFG_STORE_OFFSET_SW_PUB_P256R1                    = SID_PAL_MFG_STORE_INVALID_OFFSET,
    SID_PAL_MFG_STORE_OFFSET_SW_PUB_P256R1_SIGNATURE          = SID_PAL_MFG_STORE_INVALID_OFFSET,
    SID_PAL_MFG_STORE_OFFSET_SW_P256R1_SERIAL                 = SID_PAL_MFG_STORE_INVALID_OFFSET,
    SID_PAL_MFG_STORE_OFFSET_APP_PUB_ED25519                  = SID_PAL_MFG_STORE_INVALID_OFFSET,
    SID_PAL_MFG_STORE_OFFSET_APID                             = SID_PAL_MFG_STORE_INVALID_OFFSET,
};

#else
/*
 * Common values for Sidewalk for Sidewalk/BLE support
 * These are 8-byte word offsets.
 */
/*
 * Magic number format (little-endian)
 * bytes 0-2 = 'S' 'I' 'D', byte 3 = version
 */
enum {
    SID_PAL_MFG_STORE_OFFSET_MAGIC                          = 0,
    SID_PAL_MFG_STORE_OFFSET_VERSION                        = 1,
    SID_PAL_MFG_STORE_OFFSET_SERIAL_NUM                     = 2,
    SID_PAL_MFG_STORE_OFFSET_SMSN                           = 4,
    SID_PAL_MFG_STORE_OFFSET_APID                           = 6,
    SID_PAL_MFG_STORE_OFFSET_APP_PUB_ED25519                = 7,
    SID_PAL_MFG_STORE_OFFSET_DEVICE_PRIV_ED25519            = 9,
    SID_PAL_MFG_STORE_OFFSET_DEVICE_PUB_ED25519             = 11,
    SID_PAL_MFG_STORE_OFFSET_DEVICE_PUB_ED25519_SIGNATURE   = 13,
    SID_PAL_MFG_STORE_OFFSET_DEVICE_PRIV_P256R1             = 17,
    SID_PAL_MFG_STORE_OFFSET_DEVICE_PUB_P256R1              = 19,
    SID_PAL_MFG_STORE_OFFSET_DEVICE_PUB_P256R1_SIGNATURE    = 23,
    SID_PAL_MFG_STORE_OFFSET_DAK_PUB_ED25519                = 27,
    SID_PAL_MFG_STORE_OFFSET_DAK_PUB_ED25519_SIGNATURE      = 29,
    SID_PAL_MFG_STORE_OFFSET_DAK_ED25519_SERIAL             = 33,
    SID_PAL_MFG_STORE_OFFSET_DAK_PUB_P256R1                 = 34,
    SID_PAL_MFG_STORE_OFFSET_DAK_PUB_P256R1_SIGNATURE       = 38,
    SID_PAL_MFG_STORE_OFFSET_DAK_P256R1_SERIAL              = 42,
    SID_PAL_MFG_STORE_OFFSET_PRODUCT_PUB_ED25519            = 43,
    SID_PAL_MFG_STORE_OFFSET_PRODUCT_PUB_ED25519_SIGNATURE  = 45,
    SID_PAL_MFG_STORE_OFFSET_PRODUCT_ED25519_SERIAL         = 49,
    SID_PAL_MFG_STORE_OFFSET_PRODUCT_PUB_P256R1             = 50,
    SID_PAL_MFG_STORE_OFFSET_PRODUCT_PUB_P256R1_SIGNATURE   = 54,
    SID_PAL_MFG_STORE_OFFSET_PRODUCT_P256R1_SERIAL          = 58,
    SID_PAL_MFG_STORE_OFFSET_MAN_PUB_ED25519                = 59,
    SID_PAL_MFG_STORE_OFFSET_MAN_PUB_ED25519_SIGNATURE      = 61,
    SID_PAL_MFG_STORE_OFFSET_MAN_ED25519_SERIAL             = 65,
    SID_PAL_MFG_STORE_OFFSET_MAN_PUB_P256R1                 = 66,
    SID_PAL_MFG_STORE_OFFSET_MAN_PUB_P256R1_SIGNATURE       = 70,
    SID_PAL_MFG_STORE_OFFSET_MAN_P256R1_SERIAL              = 74,
    SID_PAL_MFG_STORE_OFFSET_SW_PUB_ED25519                 = 75,
    SID_PAL_MFG_STORE_OFFSET_SW_PUB_ED25519_SIGNATURE       = 77,
    SID_PAL_MFG_STORE_OFFSET_SW_ED25519_SERIAL              = 81,
    SID_PAL_MFG_STORE_OFFSET_SW_PUB_P256R1                  = 82,
    SID_PAL_MFG_STORE_OFFSET_SW_PUB_P256R1_SIGNATURE        = 86,
    SID_PAL_MFG_STORE_OFFSET_SW_P256R1_SERIAL               = 90,
    SID_PAL_MFG_STORE_OFFSET_AMZN_PUB_ED25519               = 91,
    SID_PAL_MFG_STORE_OFFSET_AMZN_PUB_P256R1                = 93,
    SID_PAL_MFG_STORE_SID_V0_MAX_OFFSET                     = 97,

    SID_PAL_MFG_STORE_OFFSET_DEVID                          = SID_PAL_MFG_STORE_INVALID_OFFSET,
};

#endif /* SID_PAL_SHORT_FORM_CERT_CHAIN_ENABLED */

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* __SID_PAL_MFG_STORE_STM32WBAXX_OFFSETS_H_ */
