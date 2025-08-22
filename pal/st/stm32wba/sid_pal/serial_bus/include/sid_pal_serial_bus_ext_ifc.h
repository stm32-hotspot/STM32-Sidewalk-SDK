/**
  ******************************************************************************
  * @file    sid_pal_serial_bus_ext_ifc.h
  * @brief   Custom Sidewalk SPI interface extensions
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#ifndef SID_PAL_SERIAL_BUS_EXT_IFC_H
#define SID_PAL_SERIAL_BUS_EXT_IFC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include <sid_pal_serial_bus_ifc.h>

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief SPI interface extension method that activates NSS line (selects the client) without initiating any SPI transaction
 * 
 * @note SPI mode will be configured according to the client settings to ensure SCK, MOSI, and MISO pins will have valid state
 *
 * @param[in] iface pointer to serial bus interface.
 * @param[in] client pointer to serial bus client.
 */
sid_error_t sid_pal_serial_bus_ext_ifc_activate_nss(const struct sid_pal_serial_bus_iface * const iface, const struct sid_pal_serial_bus_client * const client);

/**
 * @brief SPI interface extension method that deactivates NSS line (de-selects the client)
 *
 * @param[in] iface pointer to serial bus interface.
 * @param[in] client pointer to serial bus client.
 */
sid_error_t sid_pal_serial_bus_ext_ifc_deactivate_nss(const struct sid_pal_serial_bus_iface * const iface, const struct sid_pal_serial_bus_client * const client);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* SID_PAL_SERIAL_BUS_EXT_IFC_H */
