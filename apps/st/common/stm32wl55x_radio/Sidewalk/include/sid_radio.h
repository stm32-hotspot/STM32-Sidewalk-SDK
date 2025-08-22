/**
  ******************************************************************************
  * @file    sid_radio.c
  * @brief   SubGHz peripheral handling: app commands and IRQ processing
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023-2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#ifndef __SID_RADIO_H_
#define __SID_RADIO_H_

/* Exported types ------------------------------------------------------------*/

typedef enum {
    SID_RADIO_ERROR_NONE            =  0,
    SID_RADIO_ERROR_GENERIC         =  1,
    SID_RADIO_ERROR_INVALID_ARGS    =  2,
    SID_RADIO_ERROR_INVALID_OPCODE  =  3, /*!< Provided Sidewalk Radio App protocol OpCode (command) is invalid/unknown */
    SID_RADIO_ERROR_HARDWARE        =  4,
    SID_RADIO_ERROR_UNEXPECTED_DATA =  5,
    SID_RADIO_ERROR_WRONG_SIZE      =  6,
    SID_RADIO_ERROR_FORBIDDEN       =  7,
    SID_RADIO_ERROR_TIMEOUT         =  8,
    SID_RADIO_ERROR_EXCEEDS_MTU     =  9,
    SID_RADIO_ERROR_NOT_SUPPORTED   = 10,
    SID_RADIO_ERROR_INVALID_STATE   = 11,
    SID_RADIO_ERROR_RESOURCE_BUSY   = 12, /*!< Unable to access the resource (e.g. SubGHz peripheral) because it is locked by other user */
    SID_RADIO_ERROR_CORRUPTED_DATA  = 13, /*!< SubGHz radio received invalid/corrupted data (e.g. CRC error, invalid values in the header, etc.) */
    SID_RADIO_ERROR_NOMEM           = 14, /*!< Dynamic memory allocation error */
} sid_radio_error_t;

/* Exported functions prototypes ---------------------------------------------*/

sid_radio_error_t sid_radio_process_app_frame(const uint8_t * const frame_data, const uint32_t available_frame_length, const uint32_t full_frame_length);

sid_radio_error_t sid_radio_set_handshake_response(void);

/**
 * @brief Resets the SUBDRV radio module.
 *
 * This function initiates the reset process for the radio module and returns
 * a status code indicating the success or failure of the operation.
 *
 * @return  Status code.
 */
sid_radio_error_t sid_radio_reset_hardware(void);

sid_radio_error_t sid_radio_subghz_init(void);

/**
 * @brief Runs SubGHz IRQ handling routines, including notifications to the host MCU
 * 
 * @return Status code.
 */
sid_radio_error_t sid_radio_handle_subghz_irq(const uint32_t irq_timestamp_s, const uint32_t irq_timestamp_us);

/**
 * @brief Basic initialization of the application part (tasks, timers, queues, etc.)
 *
 * @return Status code.
 */
sid_radio_error_t sid_radio_app_init(void);

#endif /* __SID_RADIO_H_ */
