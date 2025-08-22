/**
  ******************************************************************************
  * @file    stm32_mcu_info.h
  * @brief   Provides MCU information (printable MCU name, revision ID, etc.)
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

#ifndef __STM32_MCU_INFO_H_
#define __STM32_MCU_INFO_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>

#define STM32_MCU_INFO_DEVICE_ID_STM32WBA5x                            (0x0492u)
#define STM32_MCU_INFO_STM32WBA5x_REV_ID_A                             (0x1000u)
#define STM32_MCU_INFO_STM32WBA5x_REV_ID_B                             (0x2000u)

#define STM32_MCU_INFO_DEVICE_ID_STM32WBA6x                            (0x04B0u)
#define STM32_MCU_INFO_STM32WBA5x_REV_ID_Z                             (0x1001u)

#define STM32_MCU_INFO_DEVICE_ID_STM32WLxx                             (0x0497u)
#define STM32_MCU_INFO_STM32WLxx_REV_ID_Z                              (0x1001u)
#define STM32_MCU_INFO_STM32WLxx_REV_ID_Y                              (0x1003u)


typedef struct {
    uint32_t     device_id;
    const char * device_name;
    uint32_t     rev_id;
    const char * rev_name;
} stm32_mcu_info_t;

/* Exported functions --------------------------------------------------------*/

const char * stm32_mcu_info_get_revision_name(const uint32_t device_id, const uint32_t rev_id);

const char * stm32_mcu_info_get_device_name(const uint32_t device_id);

stm32_mcu_info_t stm32_mcu_info_describe_host(void);

#ifdef __cplusplus
}
#endif

#endif /* __STM32_MCU_INFO_H_ */
