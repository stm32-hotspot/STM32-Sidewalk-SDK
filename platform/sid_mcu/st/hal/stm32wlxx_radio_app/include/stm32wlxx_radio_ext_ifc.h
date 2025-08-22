/**
  ******************************************************************************
  * @file    stm32wlxx_radio_ext_ifc.h
  * @brief   Extended API for the STM32WLxx Sidewalk Radio App
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

#ifndef __STM32WLXX_RADIO_EXT_IFC_H_
#define __STM32WLXX_RADIO_EXT_IFC_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include <sid_error.h>

/* Exported constants --------------------------------------------------------*/

#ifndef STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER
#  define STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER 0 /*!< Setting this macro to non-zero value enables User Data Transfer (UDT) extension in the radio driver */
#endif /* STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER */

/* Exported types ------------------------------------------------------------*/

/**
 * @brief User-defined callback for incoming UDT from STM32WLxx side
 * 
 */
typedef void (*stm32wlxx_ext_ifc_on_incoming_user_data)(const uint8_t * const data, const uint32_t data_len);

typedef struct {
    uint8_t * data_ptr;  /*!< Pointer to the data buffer to be sent. The pointer shall remain valid until the outbound message is processed */
    uint32_t  data_len;  /*!< Length of the data to be sent to the STM32WLxx side */
    uint32_t  auto_free; /*!< Set to zero if data_ptr points to a static buffer. Set to non-zero value to automatically call free() after the message is sent */
} stm32wlxx_ext_ifc_out_msg_desc_t;

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief Set the user callback to invoke when UDT data arrives from the STM32WLxx side
 *
 * @param [in] callback Pointer to the user-defined callback function. Can be set to NULL.
 * @returns             Non-zero value in case of error
 */
sid_error_t sid_pal_radio_stm32wlxx_set_user_data_received_cb(stm32wlxx_ext_ifc_on_incoming_user_data callback);

/**
 * @brief Enqueues user data for sending to the STM32WLxx side
 *
 * @note Supplied data must remain valid until sent by the radio. The driver does not make any copy of the supplied data buffer
 * 
 * @param [in] data      Pointer to the data buffer
 * @param [in] data_len  Length of the data buffer
 * @param [in] auto_free If set to non-zero value a free() call will be made to automatically free the data buffer after send out. This option is useful for dynamically allocated buffers
 * @returns              Non-zero value in case of error
 */
sid_error_t sid_pal_radio_stm32wlxx_send_user_data(const uint8_t * const data, const uint32_t data_len, const uint32_t auto_free);

/**
 * @brief Allows STM32WLxx side to enter deep sleep LPM modes (Standby, Off)
 * 
 * @note This command shall only be used after sid_stop() is called, but before sid_deinit() call. Once completed, SPI communication with the STM32WLxx side may be lost
 * 
 * @returns Non-zero value in case of error
 */
sid_error_t sid_pal_radio_stm32wlxx_deep_sleep(void);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* __STM32WLXX_RADIO_EXT_IFC_H_ */
