/**
  ******************************************************************************
  * @file    sid_pal_spi_client.c
  * @brief   Stub for sid_pal_spi_client module
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

/* Includes ------------------------------------------------------------------*/

#include <sid_pal_serial_client_ifc.h>
#include <sid_pal_spi_client.h>

/* Global function definitions -----------------------------------------------*/

sid_error_t sid_pal_spi_client_create(sid_pal_serial_ifc_t const **_this, const void * config, sid_pal_serial_params_t const * params)
{
    sid_error_t err = SID_ERROR_GENERIC;

    do
    {
        /* Validate inputs */
        if ((NULL == _this) || (NULL == config) || (NULL == params))
        {
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        const sid_pal_spi_config_t * const spi_cfg = config;

        (void)_this;
        (void)spi_cfg;
        (void)params;

        err = SID_ERROR_NOSUPPORT;
    } while (0);

    return err;
}
