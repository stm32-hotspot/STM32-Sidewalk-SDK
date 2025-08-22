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

#ifndef __SID_PAL_SPI_CLIENT_STM32WBAXX_H_
#define __SID_PAL_SPI_CLIENT_STM32WBAXX_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include <sid_error.h>
#include <sid_pal_serial_client_ifc.h>

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Stub for the sid_pal_spi_config datatype
 */
typedef struct sid_pal_spi_config {
    uint32_t dummy;
} sid_pal_spi_config_t;

/* Exported functions prototypes ---------------------------------------------*/

sid_error_t sid_pal_spi_client_create(sid_pal_serial_ifc_t const **_this, const void * config, const sid_pal_serial_params_t * params);

#ifdef __cplusplus
}
#endif


#endif /* __SID_PAL_SPI_CLIENT_STM32WBAXX_H_ */
