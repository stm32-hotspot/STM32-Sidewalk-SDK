/**
  ******************************************************************************
  * @file    lr11xx_cli.h
  * @brief   CLI for driving LR11xx firmware update process
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

#ifndef __SID_STM32_LR11XX_CLI_H_
#define __SID_STM32_LR11XX_CLI_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include <sid_error.h>
#include <sid_pal_ble_adapter_stm32wba_ext_ifc.h>

/* Exported functions --------------------------------------------------------*/

sid_error_t lr11xx_cli_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __SID_STM32_LR11XX_CLI_H_ */
