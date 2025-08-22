/**
  ******************************************************************************
  * @file  comm_opcodes.h
  * @brief Inter-MCU protocol commands for the STM32WLxx Radio App
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

#ifndef COMM_OPCODES_H
#define COMM_OPCODES_H

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>
#include "comm_def.h"

/* Exported constants --------------------------------------------------------*/

#define STM32WLxx_RADIO_COMM_OPCODE_DUMMY_DATA                (0x00u) /*!< Indicates a dummy transaction, any attached payload will be ignored */
#define STM32WLxx_RADIO_COMM_OPCODE_LONG_DATA_START           (0x01u) /*!< Long Data Transfer Start (LDTS) frame */
#define STM32WLxx_RADIO_COMM_OPCODE_LONG_DATA_CONT            (0x02u) /*!< Long Data Transfer Continuation (LDTC) frame */
#define STM32WLxx_RADIO_COMM_OPCODE_RAW_DATA                  (0x03u) /*!< Just raw binary data, no commands/processing expected */
#define STM32WLxx_RADIO_COMM_OPCODE_IRQ_STATUS                (0x04u) /*!< Indicates a transaction that sends/receives STM32WLxx's Radio App IRQ status. IMPORTANT: not actually a command, IRQ status payload is unconditionally preloaded into the Tx buffer right before IRQ pin is driven. This OpCode is just for visual indication and debugging - to be able to see that host MCU really intents to read the IRQ status upon indication */
#define STM32WLxx_RADIO_COMM_OPCODE_IRQ_ACK                   (0x05u) /*!< IRQ readout acknowledge command that is sent by the host MCU to the STM32WLxx */
#define STM32WLxx_RADIO_COMM_OPCODE_SEND_CONFIG               (0x06u) /*!< Sends out a configuration to be applied by STM32WLxx to the SubGHz radio */
#define STM32WLxx_RADIO_COMM_OPCODE_APPLY_BASE_HW_CONFIG      (0x07u) /*!< Apply the most essential HW settings, e.g. SubGHz clock source, regulator mode, etc. */
#define STM32WLxx_RADIO_COMM_OPCODE_SUBGHZ_RESET              (0x08u) /*!< Request SubGHz peripheral reset */
#define STM32WLxx_RADIO_COMM_OPCODE_SUBGHZ_SLEEP              (0x09u) /*!< Requests SubGHz peripheral to enter Sleep state */
#define STM32WLxx_RADIO_COMM_OPCODE_SUBGHZ_STANDBY            (0x0Au) /*!< Requests SubGHz peripheral to enter Standby state */
#define STM32WLxx_RADIO_COMM_OPCODE_SUBGHZ_SET_MODEM_MODE     (0x0Bu) /*!< Selects the modem mode for SubGHz radio (e.g. FSK, LoRa, etc.) */
#define STM32WLxx_RADIO_COMM_OPCODE_SUBGHZ_SET_FREQUENCY      (0x0Cu) /*!< Sets the frequency band to be used by SubGHz radio modem */
#define STM32WLxx_RADIO_COMM_OPCODE_SUBGHZ_SET_TX_POWER       (0x0Du) /*!< Sets the Tx power of the SubGHz radio */
#define STM32WLxx_RADIO_COMM_OPCODE_SUBGHZ_SET_SYNCWORD       (0x0Eu) /*!< Sets the sync word in STM32WLxx SubGHz radio */
#define STM32WLxx_RADIO_COMM_OPCODE_SET_LORA_SYMB_TIMEOUT     (0x0Fu) /*!< Set LoRa symbol timeout in STM32WLxx SubGHz radio */
#define STM32WLxx_RADIO_COMM_OPCODE_SET_LORA_MOD_PARAMS       (0x10u) /*!< Sends out a request to apply LoRa modulation parameters from the pre-uploaded config */
#define STM32WLxx_RADIO_COMM_OPCODE_SET_LORA_PKT_PARAMS       (0x11u) /*!< Sets LoRa packet params */
#define STM32WLxx_RADIO_COMM_OPCODE_SET_LORA_CAD_PARAMS       (0x12u) /*!< Sets LoRa CAD params */
#define STM32WLxx_RADIO_COMM_OPCODE_SET_FSK_MOD_PARAMS        (0x13u) /*!< Sends out a request to apply FSK modulation parameters from the pre-uploaded config */
#define STM32WLxx_RADIO_COMM_OPCODE_SET_FSK_PKT_PARAMS        (0x14u) /*!< Sets FSK packet params */
#define STM32WLxx_RADIO_COMM_OPCODE_SUBGHZ_SET_TX_BUF         (0x15u) /*!< Upload contents to the SubGHz's internal Tx buffer */
#define STM32WLxx_RADIO_COMM_OPCODE_SUBGHZ_START_TX           (0x16u) /*!< Starts SubGHz radio Tx */
#define STM32WLxx_RADIO_COMM_OPCODE_SUBGHZ_START_RX           (0x17u) /*!< Starts SubGHz radio Rx */
#define STM32WLxx_RADIO_COMM_OPCODE_START_CARRIER_SENSE       (0x18u) /*!< Start SubGHz Carrier Sense operation */
#define STM32WLxx_RADIO_COMM_OPCODE_USER_DATA                 (0x19u) /*!< Indicates that host MCU sends the user's custom data to the STM32WLxx side. This data will forwarded to the user app with no processing */

/* The commands below are used for diagnostic purposes only, they are never issued during the normal operation */
#define STM32WLxx_RADIO_COMM_OPCODE_START_CW_TX               (0x1Au) /*!< Start Continuous Wave Tx - emits non-modulated radio signal at specified frequency until the radio is pushed into Standby or Sleep state */
#define STM32WLxx_RADIO_COMM_OPCODE_START_CONTINUOUS_RX       (0x1Bu) /*!< Start continuous Rx (with no timeout) */
#define STM32WLxx_RADIO_COMM_OPCODE_SET_RX_DUTY_CYCLE         (0x1Cu) /*!< Configures the radio to periodically open Rx windows */
#define STM32WLxx_RADIO_COMM_OPCODE_START_LORA_CAD            (0x1Du) /*!< Perform LoRa Channel Activity Detection (CAD) without the subsequent Tx */
#define STM32WLxx_RADIO_COMM_OPCODE_GET_LIVE_RSSI             (0x1Eu) /*!< Reads out current RSSI value. Radio must be in the on of the Rx modes for this command to be valid */
#define STM32WLxx_RADIO_COMM_OPCODE_CHECK_CHANNEL_FREE        (0x1Fu) /*!< Performs radio channel occupancy check based on the continuous RSSI monitoring for the specified period of time */
#define STM32WLxx_RADIO_COMM_OPCODE_GET_CHANNEL_NOISE         (0x20u) /*!< Measures channel noise level based on the continuous RSSI reading and averaging */

#endif /* COMM_OPCODES_H */
