/**
  ******************************************************************************
  * @file    lr11xx_firmware_collection.h
  * @brief   Definitions of LR11xx firmware library storage
  *
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#ifndef __SID_STM32_LR11XX_FIRMWARE_COLLECTION_H_
#define __SID_STM32_LR11XX_FIRMWARE_COLLECTION_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include <cmsis_compiler.h>
#include <lr11xx_system_types.h>

/* Exported types ------------------------------------------------------------*/

typedef __PACKED_UNION {
    __PACKED_STRUCT {
#if SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN
        uint8_t major;
        uint8_t minor;
#else
        uint8_t minor;
        uint8_t major;
#endif /* SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN */
    };
    uint16_t raw;
} lr11xx_fw_version_t;

typedef enum {
    LR11XX_FW_TYPE_TRX        = 0, /*!< Regular transceiver firmware */
    LR11XX_FW_TYPE_MODEM_E_V1 = 1, /*!< Modem-E advanced firmware, type V1 */
    LR11XX_FW_TYPE_MODEM_E_V2 = 2, /*!< Modem-E advanced firmware, type V2 */
} lr11xx_fw_type_t;

typedef struct {
    lr11xx_fw_version_t          version;        /*!< Firmware image version in hex format (e.g. 0x0401 corresponds to firmware version 04.01) */
    lr11xx_system_version_type_t target;         /*!< Target transceiver variant (e.g. LR1110/LR1120/LR1121) */
    lr11xx_fw_type_t             type;           /*!< Specifies whether the image is of Transceiver (TRX) or Modem-E type */
    uint32_t                     word_size;      /*!< Firmware image size in 4 byte words */
    const uint32_t * const       fw_binary_data; /*!< Raw firmware binary data in Big Endian format */
} lr11xx_fw_descriptor_t;

/* Exported functions --------------------------------------------------------*/

const lr11xx_fw_descriptor_t * lr11xx_fw_collection_get_collection(uint32_t * const out_desc_count);
const lr11xx_fw_descriptor_t * lr11xx_fw_collection_get_factory_fw(void);
const lr11xx_fw_descriptor_t * lr11xx_fw_collection_get_latest_fw(void);

#ifdef __cplusplus
}
#endif

#endif /* __SID_STM32_LR11XX_FIRMWARE_COLLECTION_H_ */
