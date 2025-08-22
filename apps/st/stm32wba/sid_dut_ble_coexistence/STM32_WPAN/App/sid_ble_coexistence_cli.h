/**
  ******************************************************************************
  * @file    sid_ble_coexistence_cli.h
  * @brief   CLI for operating user-defined BLE in Sidewalk coexistence mode
  *
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

#ifndef __SID_STM32_BLE_COEXISTENCE_CLI_H_
#define __SID_STM32_BLE_COEXISTENCE_CLI_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include <sid_error.h>
#include <sid_pal_ble_adapter_stm32wba_ext_ifc.h>

/* Exported functions --------------------------------------------------------*/

void sid_ble_coexistence_cli_on_pairing_pass_key_request(const struct sid_ble_ext_virtual_device_ctx_s * const device_ctx, const sid_ble_ext_connection_ctx_t * const conn_ctx, uint32_t * const out_passkey);
void sid_ble_coexistence_cli_on_pairing_numeric_comparison(const struct sid_ble_ext_virtual_device_ctx_s * const device_ctx, const sid_ble_ext_connection_ctx_t * const conn_ctx, const uint32_t numeric_value, uint32_t * const out_numeric_value_matches);
void sid_ble_coexistence_cli_on_periph_data_received(const struct sid_ble_ext_virtual_device_ctx_s * const device_ctx, const sid_ble_ext_connection_ctx_t * const conn_ctx, const struct sid_ble_ext_gatt_server_svc_ctx_s * const svc_ctx,
                                                     const struct sid_ble_ext_gatt_server_char_ctx_s * const char_ctx, const struct sid_ble_ext_gatt_server_char_desc_ctx_s * const desc_ctx, const uint8_t * const data, const uint32_t data_length);
void sid_ble_coexistence_cli_on_periph_cccd_modified(const struct sid_ble_ext_virtual_device_ctx_s * const device_ctx, const sid_ble_ext_connection_ctx_t * const conn_ctx, const struct sid_ble_ext_gatt_server_svc_ctx_s * const svc_ctx,
                                                     const struct sid_ble_ext_gatt_server_char_ctx_s * const char_ctx, const sid_ble_ext_cccd_val_t cccd_val);
sid_error_t sid_ble_coexistence_cli_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __SID_STM32_BLE_COEXISTENCE_CLI_H_ */
