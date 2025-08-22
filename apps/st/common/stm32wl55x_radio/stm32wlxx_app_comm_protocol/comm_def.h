/**
  ******************************************************************************
  * @file  comm_def.h
  * @brief Inter-MCU protocol definitions for the STM32WLxx Radio App
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

#ifndef COMM_DEF_H
#define COMM_DEF_H

/* Includes ------------------------------------------------------------------*/

#include <stdbool.h>
#include <assert.h>
#include <cmsis_compiler.h>

/* Exported constants --------------------------------------------------------*/

/* STM32WLxx Sidewalk Radio App protocol version - this version should match between the STM32WLxx and the host MCU */
#define STM32WLxx_RADIO_COMM_PROTOCOL_MAJOR_VERSION      (1u)
#define STM32WLxx_RADIO_COMM_PROTOCOL_MINOR_VERSION      (0u)
#define STM32WLxx_RADIO_COMM_PROTOCOL_PATCH_VERSION      (4u)

#define STM32WLxx_RADIO_COMM_SPI_DATA_FRAME_SIZE         (16u) /*!< SPI frame size without the CRC part. Tis value shall be a multiple of 2 */
#define STM32WLxx_RADIO_COMM_OPCODE_SIZE                 (sizeof(uint8_t)) /*!< Fixed size of the OpCode (command) that is located in the very beginning of the SPI data frame */
#define STM32WLxx_RADIO_COMM_FRAME_PAYLOAD_SIZE          ((STM32WLxx_RADIO_COMM_SPI_DATA_FRAME_SIZE) - (STM32WLxx_RADIO_COMM_OPCODE_SIZE)) /*!< The length of the payload that follows the OpCode in the SPI data frame */
#define STM32WLxx_RADIO_COMM_MTU_SIZE                    (288u) /*!< Maximum amount of bytes that are transferrable as a single communication transaction. Keep in mind that single transaction may mean multiple SPI frames using Long Data Transfer sequence */

#define STM32WLxx_RADIO_COMM_SUBGHZ_RESET_KEY_1          (0x27D6D5ABu)
#define STM32WLxx_RADIO_COMM_SUBGHZ_RESET_KEY_2          (0x4E4F5DB0u)

#define STM32WLxx_RADIO_COMM_DEEP_SLEEP_KEY_1            (0xE49A253Au)
#define STM32WLxx_RADIO_COMM_DEEP_SLEEP_KEY_2            (0x932B214Du)

#define STM32WLxx_RADIO_COMM_SUBGHZ_TX_MAX_SIZE          (255u) /*!< Maximum size of the SubGHz Tx operation (limit comes from the size of the SubGHz's internal RAM buffer) */
#define STM32WLxx_RADIO_COMM_SUBGHZ_RX_MAX_SIZE          (255u) /*!< Maximum size of the SubGHz Rx operation (limit comes from the size of the SubGHz's internal RAM buffer) */
#define STM32WLxx_RADIO_COMM_USER_DATA_MAX_SIZE          (256u) /*!< Maximum size of the single User Data payload. A user may send up to this amount of bytes in a single transaction */

#define STM32WLxx_RADIO_COMM_FSK_SYNC_WORD_LENGTH        (8u)

#define STM32WLxx_RADIO_COMM_CAD_DEFAULT_TX_TIMEOUT      (0u) /* disable Tx timeout for CAD */
#define STM32WLxx_RADIO_COMM_LORA_CAD_DEFAULT_TX_TIMEOUT (5000000u) /*!< copy of SID_PAL_RADIO_LORA_CAD_DEFAULT_TX_TIMEOUT */

#define STM32WLxx_RADIO_COMM_TCXO_TIMEOUT_DURATION_TUS   (1u) /*!< TCXO turn on timeout in ticks (1tick = 15.625us) */

#define STM32WLxx_RADIO_COMM_FSK_PHY_HEADER_LENGTH       (2u)

#define STM32WLxx_RADIO_COMM_DEFAULT_LORA_IRQ_MASK       ( STM32WLxx_SUBGHZ_IRQ_TX_DONE      \
                                                         | STM32WLxx_SUBGHZ_IRQ_RX_DONE      \
                                                         | STM32WLxx_SUBGHZ_IRQ_PBL_DET      \
                                                         | STM32WLxx_SUBGHZ_IRQ_HEADER_ERROR \
                                                         | STM32WLxx_SUBGHZ_IRQ_CRC_ERROR    \
                                                         | STM32WLxx_SUBGHZ_IRQ_CAD_DONE     \
                                                         | STM32WLxx_SUBGHZ_IRQ_CAD_DET      \
                                                         )

#define STM32WLxx_RADIO_COMM_DEFAULT_FSK_IRQ_MASK        ( STM32WLxx_SUBGHZ_IRQ_TX_DONE         \
                                                         | STM32WLxx_SUBGHZ_IRQ_RX_DONE         \
                                                         | STM32WLxx_SUBGHZ_IRQ_PBL_DET         \
                                                         | STM32WLxx_SUBGHZ_IRQ_SYNC_WORD_VALID \
                                                         )

#define STM32WLxx_RADIO_COMM_FSK_CARRIER_SENSE_IRQ_MASK  ( STM32WLxx_SUBGHZ_IRQ_PBL_DET )

#define STM32WLxx_RADIO_COMM_FSK_CS_RSSI_THRESHOLD       (-80) /*!< Carrier Sense RSSI threshold in dBm - any signal above that level means the radio channel is busy */

#define STM32WLxx_SUBGHZ_TUS_IN_SEC                      (64000u)
#define STM32WLxx_SUBGHZ_TUS_IN_MSEC                     (64u)
#define STM32WLxx_SUBGHZ_US_IN_SEC                       (1000000u)
#define STM32WLxx_SUBGHZ_US_IN_MSEC                      (1000u)

/* Exported macro ------------------------------------------------------------*/

#define STM32WLxx_SUBGHZ_US_TO_SUBGHZ_TICKS(X)           ((uint32_t)((((uint64_t)(X) * (uint64_t)(STM32WLxx_SUBGHZ_TUS_IN_MSEC)) + ((uint64_t)(STM32WLxx_SUBGHZ_US_IN_MSEC) - 1u)) / (uint64_t)(STM32WLxx_SUBGHZ_US_IN_MSEC)))
#define STM32WLxx_SUBGHZ_TICKS_TO_US(X)                  ((uint32_t)((((uint64_t)(X) * (uint64_t)(STM32WLxx_SUBGHZ_US_IN_SEC)) + ((uint64_t)(STM32WLxx_SUBGHZ_TUS_IN_SEC) - 1u)) / (STM32WLxx_SUBGHZ_TUS_IN_SEC)))
#define STM32WLxx_SUBGHZ_US_TO_SYMBOLS(us, bps)          ((uint32_t)(((uint64_t)us * (uint64_t)bps) / (uint64_t)STM32WLxx_SUBGHZ_US_IN_SEC))

/* Exported types ------------------------------------------------------------*/

typedef uint8_t stm32wlxx_radio_comm_opcode_t;

/**
 * @brief copy of Radio FSK FCS enumeration definition
 */
typedef enum {
    STM32WLxx_RADIO_FSK_FCS_TYPE_0 = 0, /*!< 4-octet FCS */
    STM32WLxx_RADIO_FSK_FCS_TYPE_1 = 1, /*!< 2-octet FCS */
} stm32wlxx_radio_fsk_fcs_t;

/**
 * @brief Transparently maps to the Sidewalk Phy Radio Event
 */
typedef enum {
    STM32WLxx_PAL_RADIO_EVENT_UNKNOWN       = 0,
    STM32WLxx_PAL_RADIO_EVENT_TX_DONE       = 1,
    STM32WLxx_PAL_RADIO_EVENT_RX_DONE       = 2,
    STM32WLxx_PAL_RADIO_EVENT_CAD_DONE      = 3,
    STM32WLxx_PAL_RADIO_EVENT_CAD_TIMEOUT   = 4,
    STM32WLxx_PAL_RADIO_EVENT_RX_ERROR      = 5,
    STM32WLxx_PAL_RADIO_EVENT_TX_TIMEOUT    = 6,
    STM32WLxx_PAL_RADIO_EVENT_RX_TIMEOUT    = 7,
    STM32WLxx_PAL_RADIO_EVENT_CS_DONE       = 8,
    STM32WLxx_PAL_RADIO_EVENT_CS_TIMEOUT    = 9,
    STM32WLxx_PAL_RADIO_EVENT_HEADER_ERROR  = 10,
    STM32WLxx_PAL_RADIO_EVENT_SYNC_DET      = 11,
} stm32wlxx_pal_radio_events_t;

/**
 * @brief SX126X LoRa packet length enumeration definition
 */
typedef enum {
    STM32WLxx_LORA_PKT_EXPLICIT = 0x00, /*!< Header included in the packet */
    STM32WLxx_LORA_PKT_IMPLICIT = 0x01, /*!< Header not included in the packet */
} stm32wlxx_lora_pkt_len_modes_t;

/**
 * @brief STM32WLxx SubGHz packet types enumeration definition
 */
typedef enum {
    STM32WLxx_PKT_TYPE_GFSK = 0x00,
    STM32WLxx_PKT_TYPE_LORA = 0x01,
} stm32wlxx_pkt_type_t;

/**
 * @brief STM32WLxx SubGHz LoRa Modem bandwidth enumeration definition
 */
typedef enum {
    STM32WLxx_LORA_BW_500 = 6,
    STM32WLxx_LORA_BW_250 = 5,
    STM32WLxx_LORA_BW_125 = 4,
    STM32WLxx_LORA_BW_062 = 3,
    STM32WLxx_LORA_BW_041 = 10,
    STM32WLxx_LORA_BW_031 = 2,
    STM32WLxx_LORA_BW_020 = 9,
    STM32WLxx_LORA_BW_015 = 1,
    STM32WLxx_LORA_BW_010 = 8,
    STM32WLxx_LORA_BW_007 = 0,
} stm32wlxx_lora_bw_t;

/**
 * @brief STM32WLxx SubGHz LoRa Modem spreading factor enumeration definition
 */
typedef enum {
    STM32WLxx_LORA_SF5  = 0x05,
    STM32WLxx_LORA_SF6  = 0x06,
    STM32WLxx_LORA_SF7  = 0x07,
    STM32WLxx_LORA_SF8  = 0x08,
    STM32WLxx_LORA_SF9  = 0x09,
    STM32WLxx_LORA_SF10 = 0x0A,
    STM32WLxx_LORA_SF11 = 0x0B,
    STM32WLxx_LORA_SF12 = 0x0C,
} stm32wlxx_lora_sf_t;

/**
 * @brief STM32WLxx SubGHz LoRa Modem coding rate enumeration definition
 */
typedef enum {
    STM32WLxx_LORA_CR_4_5    = 0x01,
    STM32WLxx_LORA_CR_4_6    = 0x02,
    STM32WLxx_LORA_CR_4_7    = 0x03,
    STM32WLxx_LORA_CR_4_8    = 0x04,
    STM32WLxx_LORA_CR_4_5_LI = 0x05,
    STM32WLxx_LORA_CR_4_6_LI = 0x06,
    STM32WLxx_LORA_CR_4_8_LI = 0x07,
} stm32wlxx_lora_cr_t;

/**
 * @brief STM32WLxx SubGHz interrupt masks enumeration definition
 */
typedef enum {
    STM32WLxx_SUBGHZ_IRQ_NONE            = (0u <<  0),
    STM32WLxx_SUBGHZ_IRQ_TX_DONE         = (1u <<  0),
    STM32WLxx_SUBGHZ_IRQ_RX_DONE         = (1u <<  1),
    STM32WLxx_SUBGHZ_IRQ_PBL_DET         = (1u <<  2),
    STM32WLxx_SUBGHZ_IRQ_SYNC_WORD_VALID = (1u <<  3),
    STM32WLxx_SUBGHZ_IRQ_HEADER_VALID    = (1u <<  4),
    STM32WLxx_SUBGHZ_IRQ_HEADER_ERROR    = (1u <<  5),
    STM32WLxx_SUBGHZ_IRQ_CRC_ERROR       = (1u <<  6),
    STM32WLxx_SUBGHZ_IRQ_CAD_DONE        = (1u <<  7),
    STM32WLxx_SUBGHZ_IRQ_CAD_DET         = (1u <<  8),
    STM32WLxx_SUBGHZ_IRQ_TIMEOUT         = (1u <<  9),
    STM32WLxx_SUBGHZ_IRQ_LR_FHSS_HOP     = (1u << 10),
    STM32WLxx_SUBGHZ_IRQ_ALL             = (STM32WLxx_SUBGHZ_IRQ_TX_DONE | STM32WLxx_SUBGHZ_IRQ_RX_DONE | STM32WLxx_SUBGHZ_IRQ_PBL_DET | STM32WLxx_SUBGHZ_IRQ_SYNC_WORD_VALID |
                                            STM32WLxx_SUBGHZ_IRQ_HEADER_VALID | STM32WLxx_SUBGHZ_IRQ_HEADER_ERROR | STM32WLxx_SUBGHZ_IRQ_CRC_ERROR | STM32WLxx_SUBGHZ_IRQ_CAD_DONE |
                                            STM32WLxx_SUBGHZ_IRQ_CAD_DET | STM32WLxx_SUBGHZ_IRQ_TIMEOUT | STM32WLxx_SUBGHZ_IRQ_LR_FHSS_HOP),
} stm32wlxx_subghz_irq_flag_t;

/* enum for calibration bands in semtech radio */
typedef enum {
    STM32WLxx_SUBGHZ_BAND_900M    = 0,
    STM32WLxx_SUBGHZ_BAND_850M    = 1,
    STM32WLxx_SUBGHZ_BAND_770M    = 2,
    STM32WLxx_SUBGHZ_BAND_460M    = 3,
    STM32WLxx_SUBGHZ_BAND_430M    = 4,
    STM32WLxx_SUBGHZ_BAND_INVALID = 5,
} stm32wlxx_freq_cal_band_t;

/** Sidewalk Phy Radio Modem Mode*/
typedef enum {
    STM32WLxx_SUBGHZ_RADIO_MODEM_MODE_UNDEFINED = 0,
    STM32WLxx_SUBGHZ_RADIO_MODEM_MODE_FSK       = 1,
    STM32WLxx_SUBGHZ_RADIO_MODEM_MODE_LORA      = 2,
} stm32wlxx_pal_radio_modem_mode_t;

/**
 * Commands Interface buffer sizes
 */
typedef enum {
    /* Operational Modes Functions */
    STM32WLxx_SIZE_SET_SLEEP                 = 1,
    STM32WLxx_SIZE_SET_STANDBY               = 1,
    STM32WLxx_SIZE_SET_FS                    = 0,
    STM32WLxx_SIZE_SET_TX                    = 3,
    STM32WLxx_SIZE_SET_RX                    = 3,
    STM32WLxx_SIZE_SET_STOPTIMERONPREAMBLE   = 1,
    STM32WLxx_SIZE_SET_RXDUTYCYCLE           = 6,
    STM32WLxx_SIZE_SET_CAD                   = 0,
    STM32WLxx_SIZE_SET_TXCONTINUOUSWAVE      = 0,
    STM32WLxx_SIZE_SET_TXCONTINUOUSPREAMBLE  = 0,
    STM32WLxx_SIZE_SET_REGULATORMODE         = 1,
    STM32WLxx_SIZE_CALIBRATE                 = 1,
    STM32WLxx_SIZE_CALIBRATEIMAGE            = 2,
    STM32WLxx_SIZE_SET_PACONFIG              = 4,
    STM32WLxx_SIZE_SET_RXTXFALLBACKMODE      = 1,
    /* Registers and buffer Access */
    STM32WLxx_SIZE_WRITE_REGISTER            = 3, /* Full size: this value plus buffer size */
    STM32WLxx_SIZE_READ_REGISTER             = 4, /* Full size: this value plus buffer size */
    STM32WLxx_SIZE_WRITE_BUFFER              = 2, /* Full size: this value plus buffer size */
    STM32WLxx_SIZE_READ_BUFFER               = 3, /* Full size: this value plus buffer size */
    /* DIO and IRQ Control Functions */
    STM32WLxx_SIZE_SET_DIOIRQPARAMS          = 8,
    STM32WLxx_SIZE_GET_IRQSTATUS             = 2,
    STM32WLxx_SIZE_CLR_IRQSTATUS             = 2,
    STM32WLxx_SIZE_SET_DIO2ASRFSWITCHCTRL    = 1,
    STM32WLxx_SIZE_SET_TCXOMODE              = 4,
    /* RF Modulation and Packet-Related Functions */
    STM32WLxx_SIZE_SET_RFFREQUENCY           = 4,
    STM32WLxx_SIZE_SET_PACKETTYPE            = 1,
    STM32WLxx_SIZE_GET_PACKETTYPE            = 3,
    STM32WLxx_SIZE_SET_TXPARAMS              = 2,
    STM32WLxx_SIZE_SET_MODULATIONPARAMS_GFSK = 8,
    STM32WLxx_SIZE_SET_MODULATIONPARAMS_LORA = 4,
    STM32WLxx_SIZE_SET_PACKETPARAMS_GFSK     = 9,
    STM32WLxx_SIZE_SET_PACKETPARAMS_LORA     = 6,
    STM32WLxx_SIZE_SET_CADPARAMS             = 7,
    STM32WLxx_SIZE_SET_BUFFERBASEADDRESS     = 2,
    STM32WLxx_SIZE_SET_LORASYMBNUMTIMEOUT    = 1,
    /* Communication Status Information */
    STM32WLxx_SIZE_GET_STATUS                = 1,
    STM32WLxx_SIZE_GET_RXBUFFERSTATUS        = 2,
    STM32WLxx_SIZE_GET_PACKETSTATUS          = 2,
    STM32WLxx_SIZE_GET_RSSIINST              = 2,
    STM32WLxx_SIZE_GET_STATS                 = 2,
    STM32WLxx_SIZE_RESET_STATS               = 6,
    /* Miscellaneous */
    STM32WLxx_SIZE_GET_DEVICEERRORS          = 2,
    STM32WLxx_SIZE_CLR_DEVICEERRORS          = 2,
    STM32WLxx_SIZE_MAX_BUFFER                = 255,
    STM32WLxx_SIZE_DUMMY_BYTE                = 1,
} stm32wlxx_subghz_commands_size_t;

/**
 * Commands Interface
 */
typedef enum {
    /* Operational Modes Functions */
    STM32WLxx_SET_SLEEP                = 0x84,
    STM32WLxx_SET_STANDBY              = 0x80,
    STM32WLxx_SET_FS                   = 0xC1,
    STM32WLxx_SET_TX                   = 0x83,
    STM32WLxx_SET_RX                   = 0x82,
    STM32WLxx_SET_STOPTIMERONPREAMBLE  = 0x9F,
    STM32WLxx_SET_RXDUTYCYCLE          = 0x94,
    STM32WLxx_SET_CAD                  = 0xC5,
    STM32WLxx_SET_TXCONTINUOUSWAVE     = 0xD1,
    STM32WLxx_SET_TXCONTINUOUSPREAMBLE = 0xD2,
    STM32WLxx_SET_REGULATORMODE        = 0x96,
    STM32WLxx_CALIBRATE                = 0x89,
    STM32WLxx_CALIBRATEIMAGE           = 0x98,
    STM32WLxx_SET_PACONFIG             = 0x95,
    STM32WLxx_SET_RXTXFALLBACKMODE     = 0x93,
    /* Registers and buffer Access */
    STM32WLxx_WRITE_REGISTER           = 0x0D,
    STM32WLxx_READ_REGISTER            = 0x1D,
    STM32WLxx_WRITE_BUFFER             = 0x0E,
    STM32WLxx_READ_BUFFER              = 0x1E,
    /* DIO and IRQ Control Functions */
    STM32WLxx_SET_DIOIRQPARAMS         = 0x08,
    STM32WLxx_GET_IRQSTATUS            = 0x12,
    STM32WLxx_CLR_IRQSTATUS            = 0x02,
    STM32WLxx_SET_DIO2ASRFSWITCHCTRL   = 0x9D,
    STM32WLxx_SET_TCXOMODE             = 0x97,
    /* RF Modulation and Packet-Related Functions */
    STM32WLxx_SET_RFFREQUENCY          = 0x86,
    STM32WLxx_SET_PACKETTYPE           = 0x8A,
    STM32WLxx_GET_PACKETTYPE           = 0x11,
    STM32WLxx_SET_TXPARAMS             = 0x8E,
    STM32WLxx_SET_MODULATIONPARAMS     = 0x8B,
    STM32WLxx_SET_PACKETPARAMS         = 0x8C,
    STM32WLxx_SET_CADPARAMS            = 0x88,
    STM32WLxx_SET_BUFFERBASEADDRESS    = 0x8F,
    STM32WLxx_SET_LORASYMBNUMTIMEOUT   = 0xA0,
    /* Communication Status Information */
    STM32WLxx_GET_STATUS               = 0xC0,
    STM32WLxx_GET_RXBUFFERSTATUS       = 0x13,
    STM32WLxx_GET_PACKETSTATUS         = 0x14,
    STM32WLxx_GET_RSSIINST             = 0x15,
    STM32WLxx_GET_STATS                = 0x10,
    STM32WLxx_RESET_STATS              = 0x00,
    /* Miscellaneous */
    STM32WLxx_GET_DEVICEERRORS         = 0x17,
    STM32WLxx_CLR_DEVICEERRORS         = 0x07,
} stm32wlxx_subghz_commands_t;

/**
 * @brief STM32WLxx LoRa CAD number of symbols enumeration definition
 *
 * @note Represents the number of symbols to be used for a CAD operation
 */
typedef enum {
    STM32WLxx_LORA_CAD_01_SYMB = 0x00,
    STM32WLxx_LORA_CAD_02_SYMB = 0x01,
    STM32WLxx_LORA_CAD_04_SYMB = 0x02,
    STM32WLxx_LORA_CAD_08_SYMB = 0x03,
    STM32WLxx_LORA_CAD_16_SYMB = 0x04,
} stm32wlxx_lora_cad_symbs_t;

/**
 * @brief STM32WLxx LoRa CAD exit modes enumeration definition
 *
 * @note Represents the action to be performed after a CAD is done
 */
typedef enum {
    STM32WLxx_LORA_CAD_ONLY = 0x00,
    STM32WLxx_LORA_CAD_RX   = 0x01,
    STM32WLxx_LORA_CAD_LBT  = 0x10,
} stm32wlxx_lora_cad_exit_modes_t;

/** Sidewalk phy lora crc present*/
typedef enum {
        STM32WLxx_PAL_RADIO_CRC_PRESENT_INVALID = 0,
        STM32WLxx_PAL_RADIO_CRC_PRESENT_OFF     = 1,
        STM32WLxx_PAL_RADIO_CRC_PRESENT_ON      = 2,
        STM32WLxx_PAL_RADIO_CRC_PRESENT_MAX_NUM = STM32WLxx_PAL_RADIO_CRC_PRESENT_ON,
}  stm32wlxx_pal_radio_lora_crc_present_t;

/**
 * @brief STM32WLxx GFSK Rx bandwidth enumeration definition
 */
typedef enum {
    STM32WLxx_GFSK_BW_4800   = 0x1F,
    STM32WLxx_GFSK_BW_5800   = 0x17,
    STM32WLxx_GFSK_BW_7300   = 0x0F,
    STM32WLxx_GFSK_BW_9700   = 0x1E,
    STM32WLxx_GFSK_BW_11700  = 0x16,
    STM32WLxx_GFSK_BW_14600  = 0x0E,
    STM32WLxx_GFSK_BW_19500  = 0x1D,
    STM32WLxx_GFSK_BW_23400  = 0x15,
    STM32WLxx_GFSK_BW_29300  = 0x0D,
    STM32WLxx_GFSK_BW_39000  = 0x1C,
    STM32WLxx_GFSK_BW_46900  = 0x14,
    STM32WLxx_GFSK_BW_58600  = 0x0C,
    STM32WLxx_GFSK_BW_78200  = 0x1B,
    STM32WLxx_GFSK_BW_93800  = 0x13,
    STM32WLxx_GFSK_BW_117300 = 0x0B,
    STM32WLxx_GFSK_BW_156200 = 0x1A,
    STM32WLxx_GFSK_BW_187200 = 0x12,
    STM32WLxx_GFSK_BW_234300 = 0x0A,
    STM32WLxx_GFSK_BW_312000 = 0x19,
    STM32WLxx_GFSK_BW_373600 = 0x11,
    STM32WLxx_GFSK_BW_467000 = 0x09,
} stm32wlxx_gfsk_bw_t;

/**
 * @brief STM32WLxx GFSK modulation shaping enumeration definition
 */
typedef enum {
    STM32WLxx_GFSK_MOD_SHAPE_OFF   = 0x00,
    STM32WLxx_GFSK_MOD_SHAPE_BT_03 = 0x08,
    STM32WLxx_GFSK_MOD_SHAPE_BT_05 = 0x09,
    STM32WLxx_GFSK_MOD_SHAPE_BT_07 = 0x0A,
    STM32WLxx_GFSK_MOD_SHAPE_BT_1  = 0x0B,
} stm32wlxx_gfsk_mod_shapes_t;

/** Copy of sid_pal_radio_cad_param_exit_mode_t */
typedef enum {
    STM32WLxx_RADIO_CAD_EXIT_MODE_CS_ONLY = 0x00000, /*!< Carrier sense only */
    STM32WLxx_RADIO_CAD_EXIT_MODE_CS_RX   = 0x00001, /*!< Carrier sense followed by Rx */
    STM32WLxx_RADIO_CAD_EXIT_MODE_CS_LBT  = 0x00010, /*!< Carrier sense followed by Tx */
    STM32WLxx_RADIO_CAD_EXIT_MODE_ED_ONLY = 0x00100, /*!< Energy detect only */
    STM32WLxx_RADIO_CAD_EXIT_MODE_ED_RX   = 0x00101, /*!< Energy detect followed by Rx */
    STM32WLxx_RADIO_CAD_EXIT_MODE_ED_LBT  = 0x00110, /*!< Energy detect followed by Tx */
    STM32WLxx_RADIO_CAD_EXIT_MODE_NONE    = 0x10000, /*!< No CAD mode set */
} stm32wlxx_pal_radio_cad_param_exit_mode_t;

/**
 * @brief STM32WLxx GFSK packet status structure definition
 */
typedef struct {
    uint8_t rx_status;
    int8_t  rssi_sync;  /*!< The RSSI measured on last packet */
    int8_t  rssi_avg;   /*!< The averaged RSSI */
} stm32wlxx_pkt_status_gfsk_t;

/**
 * @brief Transparently maps to the Sidewalk Phy received LORA packet status
 */
typedef __PACKED_STRUCT {
    int16_t rssi;
    int8_t  snr;
    int8_t  signal_rssi;
    uint8_t is_crc_present; /** Maps to stm32wlxx_pal_radio_lora_crc_present_t */
} stm32wlxx_rcp_lora_rx_packet_info_t;

/**
 * @brief Transparently maps to the Sidewalk Phy received FSK packet status
 */
typedef __PACKED_STRUCT {
    int8_t rssi_avg;
    int8_t rssi_sync;
    int8_t snr;
} stm32wlxx_rcp_fsk_rx_packet_info_t;

/**
 * @brief Copy of Radio FSK PHY HDR structure definition
 */
typedef struct {
	stm32wlxx_radio_fsk_fcs_t   fcs_type;
    bool                        is_data_whitening_enabled;
    bool                        is_fec_enabled;
} stm32wlxx_pal_radio_fsk_phy_hdr_t;

/**
 * @brief Transparently maps to the Sidewalk Phy received LoRa packet status
 */
typedef __PACKED_STRUCT {
    union
    {
        stm32wlxx_rcp_lora_rx_packet_info_t lora_rx_packet_status;
        stm32wlxx_rcp_fsk_rx_packet_info_t  fsk_rx_packet_status;
    };
} stm32wlxx_rcp_radio_rx_packet_info_t;

/**
 * @brief STM32WLxx LoRa CAD parameters structure definition
 */
typedef struct {
    stm32wlxx_lora_cad_symbs_t      cad_symb_nb;    /*!< CAD number of symbols */
    uint8_t                         cad_det_peak;   /*!< CAD peak detection */
    uint8_t                         cad_det_min;    /*!< CAD minimum detection */
    stm32wlxx_lora_cad_exit_modes_t cad_exit_mode;  /*!< CAD exit mode */
    uint32_t                        cad_timeout;    /*!< CAD timeout value */
} stm32wlxx_lora_cad_params_t;

/**
 * @brief STM32WLxx SubGHz LoRa packet parameters structure definition
 */
typedef struct {
    uint16_t                       pbl_len_in_symb;  /*!< Preamble length in symbols */
    stm32wlxx_lora_pkt_len_modes_t hdr_type;         /*!< Header type */
    uint8_t                        pld_len_in_bytes; /*!< Payload length in bytes */
    bool                           crc_is_on;        /*!< CRC activation */
    bool                           invert_iq_is_on;  /*!< IQ polarity setup */
} stm32wlxx_pkt_params_lora_t;

/**
 * @brief STM32WLxx GFSK preamble length Rx detection size enumeration definition
 */
typedef enum {
    STM32WLxx_GFSK_PBL_DET_OFF     = 0x00,
    STM32WLxx_GFSK_PBL_DET_08_BITS = 0x04,
    STM32WLxx_GFSK_PBL_DET_16_BITS = 0x05,
    STM32WLxx_GFSK_PBL_DET_24_BITS = 0x06,
    STM32WLxx_GFSK_PBL_DET_32_BITS = 0x07,
} stm32wlxx_gfsk_pbl_det_t;

/**
 * @brief STM32WLxx GFSK address filtering configuration enumeration definition
 */
typedef enum {
    STM32WLxx_GFSK_ADDR_CMP_FILT_OFF        = 0x00,
    STM32WLxx_GFSK_ADDR_CMP_FILT_NODE       = 0x01,
    STM32WLxx_GFSK_ADDR_CMP_FILT_NODE_BROAD = 0x02,
} stm32wlxx_gfsk_addr_cmp_t;

/**
 * @brief STM32WLxx GFSK packet length enumeration definition
 */
typedef enum {
    STM32WLxx_GFSK_PKT_FIX_LEN = 0x00,  /*!< The packet length is known on both sides, no header included */
    STM32WLxx_GFSK_PKT_VAR_LEN = 0x01,  /*!< The packet length is variable, header included */
} stm32wlxx_gfsk_pkt_len_modes_t;

/**
 * @brief STM32WLxx GFSK CRC type enumeration definition
 */
typedef enum {
    STM32WLxx_GFSK_CRC_OFF         = 0x01,
    STM32WLxx_GFSK_CRC_1_BYTE      = 0x00,
    STM32WLxx_GFSK_CRC_2_BYTES     = 0x02,
    STM32WLxx_GFSK_CRC_1_BYTE_INV  = 0x04,
    STM32WLxx_GFSK_CRC_2_BYTES_INV = 0x06,
} stm32wlxx_gfsk_crc_types_t;

/**
 * @brief STM32WLxx GFSK whitening control enumeration definition
 */
typedef enum {
    STMWLxx_GFSK_DC_FREE_OFF       = 0x00,
    STMWLxx_GFSK_DC_FREE_WHITENING = 0x01,
} stm32wlxx_gfsk_dc_free_t;

/**
 * @brief STM32WLxx GFSK packet parameters structure definition
 */
typedef struct {
    uint16_t                       pbl_len_in_bits;        /*!< Preamble length in bits */
    stm32wlxx_gfsk_pbl_det_t       pbl_min_det;            /*!< Preamble detection length */
    uint8_t                        sync_word_len_in_bits;  /*!< Sync word length in bits */
    stm32wlxx_gfsk_addr_cmp_t      addr_cmp;               /*!< Address filtering configuration */
    stm32wlxx_gfsk_pkt_len_modes_t hdr_type;               /*!< Header type */
    uint8_t                        pld_len_in_bytes;       /*!< Payload length in bytes */
    stm32wlxx_gfsk_crc_types_t     crc_type;               /*!< CRC type configuration */
    stm32wlxx_gfsk_dc_free_t       dc_free;                /*!< Whitening configuration */
} stm32wlxx_pkt_params_gfsk_t;

/**
 * @brief STM32WLxx LoRa modulation parameters structure definition
 */
typedef struct {
    stm32wlxx_lora_sf_t sf;    /*!< LoRa Spreading Factor */
    stm32wlxx_lora_bw_t bw;    /*!< LoRa Bandwidth */
    stm32wlxx_lora_cr_t cr;    /*!< LoRa Coding Rate */
    uint8_t             ldro;  /*!< Low DataRate Optimization configuration */
} stm32wlxx_mod_params_lora_t;

/** sid_pal_radio_lora_modulation_params_t */
typedef __PACKED_STRUCT {
    uint32_t bit_rate;
    uint32_t freq_dev;
    uint8_t  shaping;
    uint8_t  bandwidth;
} stm32wlxx_radio_fsk_phy_mod_params_t;

/** Sidewalk Phy Radio LoRa Packet Params */
typedef __PACKED_STRUCT {
    uint16_t preamble_length;
    uint8_t  preamble_min_detect;
    uint8_t  sync_word_length;
    uint8_t  addr_comp;
    uint8_t  header_type;
    uint8_t  payload_length;
    uint8_t  crc_type;
    uint8_t  radio_whitening_mode;
} stm32wlxx_pal_radio_fsk_pkt_params_t;

/** Sidewalk phy FSK configuration to be sent by host MCU to STM32WLxx */
typedef __PACKED_STRUCT {
    uint32_t freq;
    int8_t   power;
    uint8_t  sync_word[8];
    uint8_t  sync_word_len;
    uint16_t whitening_seed;
    uint32_t tx_timeout;
    uint32_t symbol_timeout;

    /* sid_pal_radio_fsk_modulation_params_t */
    stm32wlxx_radio_fsk_phy_mod_params_t mod;

    /* sid_pal_radio_fsk_packet_params_t */
    stm32wlxx_pal_radio_fsk_pkt_params_t pkt;

    /* sid_pal_radio_fsk_cad_params_t */
    int16_t  fsk_ed_rssi_threshold;
    uint16_t fsk_ed_duration_mus;
    uint8_t  fsk_cs_min_prm_det;
    uint32_t fsk_cs_duration_us;

    /* sid_pal_radio_fsk_phy_hdr_t */
    stm32wlxx_radio_fsk_fcs_t phy_hdr_fcs_type;
    bool     phy_hdr_is_data_whitening_enabled;
    bool     phy_hdr_is_fec_enabled;
} stm32wlxx_radio_fsk_phy_settings_t;

/** sid_pal_radio_lora_modulation_params_t */
typedef __PACKED_STRUCT {
    uint8_t  spreading_factor;
    uint8_t  bandwidth;
    uint8_t  coding_rate;
} stm32wlxx_radio_lora_phy_mod_params_t;

/** Sidewalk Phy Radio LoRa Packet Params */
typedef __PACKED_STRUCT {
    uint16_t preamble_length;
    uint8_t  header_type;
    uint8_t  payload_length;
    uint8_t  crc_mode;
    uint8_t  invert_IQ;
} stm32wlxx_radio_lora_phy_pkt_params_t;

/** Sidewalk phy LoRa configuration */
typedef __PACKED_STRUCT {
    uint32_t freq;
    int8_t   power;
    uint16_t sync_word;
    uint8_t  symbol_timeout;
    uint32_t tx_timeout;
    uint8_t  lora_ldr_long_interleaved_enable;

    /* sid_pal_radio_lora_modulation_params_t */
    stm32wlxx_radio_lora_phy_mod_params_t mod;

    /* sid_pal_radio_lora_packet_params_t */
    stm32wlxx_radio_lora_phy_pkt_params_t pkt;

    /* sid_pal_radio_lora_cad_params_t */
    uint8_t  cad_symbol_num;
    uint8_t  cad_detect_peak;
    uint8_t  cad_detect_min;
    uint8_t  cad_exit_mode;
    uint32_t cad_timeout;
} stm32wlxx_radio_lora_phy_settings_t;

/** Sidewalk phy driver configuration */
typedef __PACKED_STRUCT {
    uint16_t              irq_mask;
    bool                  rx_boost;
    uint16_t              trim;

    stm32wlxx_pal_radio_cad_param_exit_mode_t cad_exit_mode;

    int8_t                lna_gain;
} stm32wlxx_radio_drv_settings_t;

/** Power amplifier configuration - copy of stm32wlxx_radio_pa_cfg_t */
typedef __PACKED_STRUCT {
    uint8_t pa_duty_cycle;
    uint8_t hp_max;
    uint8_t device_sel;
    uint8_t pa_lut;
    int8_t  tx_power_reg;    /*!< The value to be written into the SubGHz register - this may differ from the target Tx power if optimized PA config is applied */
    uint8_t ramp_time;
    int8_t  target_tx_power; /*!< The actual Tx power targeted by the PA configuration */
} stm32wlxx_radio_pa_setup_t;

/** Radio phy configuration to be sent by WBA to WL */
typedef __PACKED_STRUCT {
    stm32wlxx_radio_fsk_phy_settings_t  fsk_cfg;
    stm32wlxx_radio_lora_phy_settings_t lora_cfg;
    stm32wlxx_radio_drv_settings_t      drv_cfg;
    stm32wlxx_radio_pa_setup_t          pa_cfg;
} stm32wlxx_radio_phy_cfg_t;

/** helper define - copy from pal */
typedef uint16_t stm32wlxx_subghz_irq_mask_t;

/** Sidewalk Phy Radio Start Rx Params */
typedef __PACKED_STRUCT {
    uint32_t                             compensatory_ns;      /*!< The amount of time that was spent on processing the SubGHz IRQ - can be used by host MCU to estimate the actual SubGHz IRQ timestamp */
    stm32wlxx_rcp_radio_rx_packet_info_t received_packet_info; /*!< LoRa- or FSK-specific data */
    uint8_t                              radio_event;          /*!< Maps tp stm32wlxx_pal_radio_events_t */
    uint8_t                              cad_exit_mode;        /*!< Maps to stm32wlxx_pal_radio_cad_param_exit_mode_t */
    uint8_t                              radio_state;          /*!< Maps to Sidewalk Phy Radio State macros */
} stm32wlxx_subghz_irq_details_t;


/**
 * @brief STM32WLxx Radio app interrupt masks enumeration definition
 */
typedef enum {
    STM32WLxx_APP_IRQ_NONE          = (0u << 0),
    STM32WLxx_APP_IRQ_SUBGHZ        = (1u << 0),
    STM32WLxx_APP_IRQ_SPI_REQ_DONE  = (1u << 1),
    STM32WLxx_APP_IRQ_SPI_HANDSHAKE = (1u << 2),
    STM32WLxx_APP_IRQ_USER_DATA     = (1u << 3)
} stm32wlxx_app_irq_mask_t;

typedef __PACKED_STRUCT {
    uint8_t  major;
    uint8_t  minor;
    uint16_t patch;
} stm32wlxx_version_info_t;

typedef __PACKED_STRUCT {
    stm32wlxx_version_info_t protocol_version;
    stm32wlxx_version_info_t app_version;
} stm32wlxx_handshake_irq_details_t;

typedef __PACKED_STRUCT {
    uint16_t request_id;
    uint16_t status;
} stm32wlxx_req_completed_irq_details_t;

/**
 * @brief Long Data Transfer Start (LDTS) SPI frame payload
 */
typedef __PACKED_STRUCT {
    uint16_t full_data_size;
    uint8_t  partial_data[STM32WLxx_RADIO_COMM_FRAME_PAYLOAD_SIZE - sizeof(uint16_t)];
} stm32wlxx_rcp_ldts_t;
static_assert(sizeof(stm32wlxx_rcp_ldts_t) <= STM32WLxx_RADIO_COMM_FRAME_PAYLOAD_SIZE, "Size of stm32wlxx_rcp_ldts_t shall match STM32WLxx_RADIO_COMM_FRAME_PAYLOAD_SIZE");

/**
 * @brief Long Data Transfer Continuation (LDTC) SPI frame payload
 */
typedef __PACKED_STRUCT {
    uint8_t partial_data[STM32WLxx_RADIO_COMM_FRAME_PAYLOAD_SIZE];
} stm32wlxx_rcp_ldtc_t;
static_assert(sizeof(stm32wlxx_rcp_ldtc_t) <= STM32WLxx_RADIO_COMM_FRAME_PAYLOAD_SIZE, "Size of stm32wlxx_rcp_ldtc_t shall match STM32WLxx_RADIO_COMM_FRAME_PAYLOAD_SIZE");

/**
 * @brief Unified STM32WLxx Sidewalk Radio App interrupt status frame definition. Contains common and IRQ-specific definitions
 */
typedef __PACKED_STRUCT {
    uint8_t  irq_flags;            /*!< Active IRQ flags, maps to stm32wlxx_app_irq_mask_t */
    uint16_t followup_payload_len; /*!< Length of the potential additional data followin up the IRQ status frame that contain more details or provides some value (e.g. Rx DOne IRQ will be followed by the received radio frame) */
    union {
        uint8_t                               raw_irq_payload[STM32WLxx_RADIO_COMM_FRAME_PAYLOAD_SIZE - sizeof(uint8_t) - sizeof(uint16_t)]; /*!< Raw additional payload, actual contents depend on the flags set in irq_status */
        stm32wlxx_subghz_irq_details_t        subghz_irq_details;
        stm32wlxx_handshake_irq_details_t     handshake_details;
        stm32wlxx_req_completed_irq_details_t req_cmpltd_details;
    };
} stm32wlxx_rcp_irq_status_t;
static_assert(sizeof(stm32wlxx_rcp_irq_status_t) <= STM32WLxx_RADIO_COMM_FRAME_PAYLOAD_SIZE, "Size of stm32wlxx_rcp_irq_status_t shall match STM32WLxx_RADIO_COMM_FRAME_PAYLOAD_SIZE");

/**
 * @brief Payload for SubGHz peripheral reset request
 */
typedef __PACKED_STRUCT {
    uint32_t reset_key_1;
    uint32_t reset_key_2;
} stm32wlxx_rcp_reset_subghz_req_t;
static_assert(sizeof(stm32wlxx_rcp_reset_subghz_req_t) <= STM32WLxx_RADIO_COMM_FRAME_PAYLOAD_SIZE, "Size of stm32wlxx_rcp_reset_subghz_req_t shall match STM32WLxx_RADIO_COMM_FRAME_PAYLOAD_SIZE");

/**
 * @brief Payload for SubGHz apply base HW config
 */
typedef __PACKED_STRUCT {
    uint8_t need_ack;
} stm32wlxx_rcp_apply_base_hw_cfg_req_t;
static_assert(sizeof(stm32wlxx_rcp_apply_base_hw_cfg_req_t) <= STM32WLxx_RADIO_COMM_FRAME_PAYLOAD_SIZE, "Size of stm32wlxx_rcp_apply_base_hw_cfg_req_t shall match STM32WLxx_RADIO_COMM_FRAME_PAYLOAD_SIZE");


/**
 * @brief Payload for the Set SubGHz Modem Mode command
 */
typedef __PACKED_STRUCT {
    uint8_t mode; /*!< Desired modem mode. Maps to stm32wlxx_pal_radio_modem_mode_t */
} stm32wlxx_rcp_set_modem_mode_req_t;
static_assert(sizeof(stm32wlxx_rcp_set_modem_mode_req_t) <= STM32WLxx_RADIO_COMM_FRAME_PAYLOAD_SIZE, "Size of stm32wlxx_rcp_set_modem_mode_req_t shall match STM32WLxx_RADIO_COMM_FRAME_PAYLOAD_SIZE");

/**
 * @brief Payload for the SubGHz Set Frequency command
 */
typedef __PACKED_STRUCT {
    uint32_t frequency;
} stm32wlxx_rcp_set_frequency_req_t;
static_assert(sizeof(stm32wlxx_rcp_set_frequency_req_t) <= STM32WLxx_RADIO_COMM_FRAME_PAYLOAD_SIZE, "Size of stm32wlxx_rcp_set_frequency_req_t shall match STM32WLxx_RADIO_COMM_FRAME_PAYLOAD_SIZE");

/**
 * @brief Payload for the SubGHz Set Tx Power command
 */
typedef stm32wlxx_radio_pa_setup_t stm32wlxx_rcp_set_tx_power_req_t;
static_assert(sizeof(stm32wlxx_rcp_set_tx_power_req_t) <= STM32WLxx_RADIO_COMM_FRAME_PAYLOAD_SIZE, "Size of stm32wlxx_rcp_set_tx_power_req_t shall match STM32WLxx_RADIO_COMM_FRAME_PAYLOAD_SIZE");

/**
 * @brief Payload for the Enter Sleep Mode command
 */
typedef __PACKED_STRUCT {
    uint32_t sleep_duration_us;
    uint32_t deep_sleep_key_1;
    uint32_t deep_sleep_key_2;
    uint8_t  deep_sleep_en;
} stm32wlxx_rcp_radio_sleep_req_t;
static_assert(sizeof(stm32wlxx_rcp_radio_sleep_req_t) <= STM32WLxx_RADIO_COMM_FRAME_PAYLOAD_SIZE, "Size of stm32wlxx_rcp_radio_sleep_req_t shall match STM32WLxx_RADIO_COMM_FRAME_PAYLOAD_SIZE");

/**
 * @brief Payload for the Set Sync Word command
 */
typedef __PACKED_STRUCT {
    uint8_t modem; /*!< Modem mode to which the sync word settings apply. Maps to stm32wlxx_pal_radio_modem_mode_t */
    union {
        uint16_t    lora_sync_word;
        struct {
            uint8_t data[STM32WLxx_RADIO_COMM_FSK_SYNC_WORD_LENGTH];
            uint8_t data_len;
        } fsk_sync_word;
    };
} stm32wlxx_rcp_set_sync_word_req_t;
static_assert(sizeof(stm32wlxx_rcp_set_sync_word_req_t) <= STM32WLxx_RADIO_COMM_FRAME_PAYLOAD_SIZE, "Size of stm32wlxx_rcp_set_sync_word_req_t shall match STM32WLxx_RADIO_COMM_FRAME_PAYLOAD_SIZE");

/**
 * @brief Payload for the Set LoRa Symbol Timeout command
 */
typedef __PACKED_STRUCT {
    uint8_t symbol_timeout;
} stm32wlxx_rcp_set_lora_symb_timeout_req_t;
static_assert(sizeof(stm32wlxx_rcp_set_lora_symb_timeout_req_t) <= STM32WLxx_RADIO_COMM_FRAME_PAYLOAD_SIZE, "Size of stm32wlxx_rcp_set_lora_symb_timeout_req_t shall match STM32WLxx_RADIO_COMM_FRAME_PAYLOAD_SIZE");

/**
 * @brief Payload for the Set LoRa Modulation Params command
 */
typedef __PACKED_STRUCT {
	stm32wlxx_radio_lora_phy_mod_params_t mod_params;
} stm32wlxx_rcp_set_lora_mod_params_req_t;
static_assert(sizeof(stm32wlxx_rcp_set_lora_mod_params_req_t) <= STM32WLxx_RADIO_COMM_FRAME_PAYLOAD_SIZE, "Size of stm32wlxx_rcp_set_lora_mod_params_req_t shall match STM32WLxx_RADIO_COMM_FRAME_PAYLOAD_SIZE");

/**
 * @brief Payload for the Set LoRa Packet Params command
 */
typedef __PACKED_STRUCT {
    stm32wlxx_radio_lora_phy_pkt_params_t packet_params;
} stm32wlxx_rcp_set_lora_pkt_params_req_t;
static_assert(sizeof(stm32wlxx_rcp_set_lora_pkt_params_req_t) <= STM32WLxx_RADIO_COMM_FRAME_PAYLOAD_SIZE, "Size of stm32wlxx_rcp_set_lora_pkt_params_req_t shall match STM32WLxx_RADIO_COMM_FRAME_PAYLOAD_SIZE");

/**
 * @brief Payload for the Set FSK Modulation Params command
 */
typedef __PACKED_STRUCT {
	stm32wlxx_radio_fsk_phy_mod_params_t mod_params;
} stm32wlxx_rcp_set_fsk_mod_params_req_t;
static_assert(sizeof(stm32wlxx_rcp_set_fsk_mod_params_req_t) <= STM32WLxx_RADIO_COMM_FRAME_PAYLOAD_SIZE, "Size of stm32wlxx_rcp_set_fsk_mod_params_req_t shall match STM32WLxx_RADIO_COMM_FRAME_PAYLOAD_SIZE");

/**
 * @brief Payload for the Set FSK packet Params command
 */
typedef __PACKED_STRUCT {
    stm32wlxx_pal_radio_fsk_pkt_params_t packet_params;
} stm32wlxx_rcp_set_fsk_pkt_params_req_t;
static_assert(sizeof(stm32wlxx_rcp_set_fsk_pkt_params_req_t) <= STM32WLxx_RADIO_COMM_FRAME_PAYLOAD_SIZE, "Size of stm32wlxx_rcp_set_fsk_pkt_params_req_t shall match STM32WLxx_RADIO_COMM_FRAME_PAYLOAD_SIZE");

/** Sidewalk Phy Radio Start Rx Params */
typedef __PACKED_STRUCT {
    uint32_t timeout_us;
} stm32wlxx_rcp_start_rx_params_req_t;
static_assert(sizeof(stm32wlxx_rcp_start_rx_params_req_t) <= STM32WLxx_RADIO_COMM_FRAME_PAYLOAD_SIZE, "Size of stm32wlxx_rcp_start_rx_params_req_t shall match STM32WLxx_RADIO_COMM_FRAME_PAYLOAD_SIZE");

/** Sidewalk Phy Radio Start Continuous Wave Tx Params */
typedef __PACKED_STRUCT {
    uint32_t frequecy;
} stm32wlxx_rcp_start_cw_tx_params_req_t;
static_assert(sizeof(stm32wlxx_rcp_start_cw_tx_params_req_t) <= STM32WLxx_RADIO_COMM_FRAME_PAYLOAD_SIZE, "Size of stm32wlxx_rcp_start_cw_tx_params_req_t shall match STM32WLxx_RADIO_COMM_FRAME_PAYLOAD_SIZE");

/**
 * @brief Root level definition of a single SPI frame
 */
typedef __PACKED_STRUCT {
    stm32wlxx_radio_comm_opcode_t opcode;
    union {
        uint8_t raw[STM32WLxx_RADIO_COMM_FRAME_PAYLOAD_SIZE];
        stm32wlxx_rcp_ldts_t                      ldts;
        stm32wlxx_rcp_ldtc_t                      ldtc;
        stm32wlxx_rcp_irq_status_t                irq_status;
        stm32wlxx_rcp_reset_subghz_req_t          subghz_reset_req;
        stm32wlxx_rcp_apply_base_hw_cfg_req_t     apply_hw_cfg;
        stm32wlxx_rcp_set_modem_mode_req_t        set_modem_mode;
        stm32wlxx_rcp_set_frequency_req_t         set_frequency;
        stm32wlxx_rcp_set_tx_power_req_t          set_tx_power;
        stm32wlxx_rcp_radio_sleep_req_t           radio_sleep;
        stm32wlxx_rcp_set_sync_word_req_t         set_sync_word;
        stm32wlxx_rcp_set_lora_symb_timeout_req_t set_lora_symb_timeout;
        stm32wlxx_rcp_set_lora_mod_params_req_t   set_lora_mod_params;
        stm32wlxx_rcp_set_lora_pkt_params_req_t   set_lora_pkt_params;
        stm32wlxx_rcp_set_fsk_mod_params_req_t    set_fsk_mod_params;
        stm32wlxx_rcp_set_fsk_pkt_params_req_t    set_fsk_pkt_params;
        stm32wlxx_rcp_start_rx_params_req_t       start_rx_params;
        stm32wlxx_rcp_start_cw_tx_params_req_t    start_cw_tx_params;
    }                             payload;
} stm32wlxx_radio_comm_spi_frame_t;
static_assert(sizeof(stm32wlxx_radio_comm_spi_frame_t) == STM32WLxx_RADIO_COMM_SPI_DATA_FRAME_SIZE, "Size of stm32wlxx_radio_comm_spi_frame_t shall match STM32WLxx_RADIO_COMM_SPI_DATA_FRAME_SIZE");


/** App-level frames that exceed SPI frame */

/**
 * @brief Payload for the Apply Radio Config command
 */
typedef __PACKED_STRUCT {
    stm32wlxx_radio_comm_opcode_t opcode;
    uint8_t                       need_ack;
    stm32wlxx_radio_phy_cfg_t     radio_config;
} stm32wlxx_rcp_apply_cfg_t;
static_assert(sizeof(stm32wlxx_rcp_apply_cfg_t) <= STM32WLxx_RADIO_COMM_MTU_SIZE, "Size of stm32wlxx_rcp_apply_cfg_t shall not exceed STM32WLxx_RADIO_COMM_MTU_SIZE");

/**
 * @brief Payload for the Apply Radio Config command
 */
typedef __PACKED_STRUCT {
    stm32wlxx_radio_comm_opcode_t opcode;
    uint16_t                      write_length;
    uint8_t                       tx_buf_content[STM32WLxx_RADIO_COMM_SUBGHZ_TX_MAX_SIZE]; /*!< It's not mandatory to always use the full buffer, so the actual length of this frame is variable */
} stm32wlxx_rcp_set_subghz_tx_buf_t;
static_assert(sizeof(stm32wlxx_rcp_set_subghz_tx_buf_t) <= STM32WLxx_RADIO_COMM_MTU_SIZE, "Size of stm32wlxx_rcp_set_subghz_tx_buf_t shall not exceed STM32WLxx_RADIO_COMM_MTU_SIZE");

/**
 * @brief Payload for the Apply Radio Config command
 */
typedef __PACKED_STRUCT {
    stm32wlxx_radio_comm_opcode_t opcode;
    uint8_t                       subghz_rx_buf[STM32WLxx_RADIO_COMM_SUBGHZ_RX_MAX_SIZE]; /*!< It's not mandatory to always use the full buffer, so the actual length of this frame is variable */
} stm32wlxx_rcp_set_subghz_rx_buf_t;
static_assert(sizeof(stm32wlxx_rcp_set_subghz_rx_buf_t) <= STM32WLxx_RADIO_COMM_MTU_SIZE, "Size of stm32wlxx_rcp_set_subghz_rx_buf_t shall not exceed STM32WLxx_RADIO_COMM_MTU_SIZE");

/**
 * @brief Payload for the User Data command
 */
typedef __PACKED_STRUCT {
    stm32wlxx_radio_comm_opcode_t opcode;
    uint16_t                      data_length;
    uint8_t                       data[STM32WLxx_RADIO_COMM_USER_DATA_MAX_SIZE]; /*!< It's not mandatory to always use the full buffer, so the actual length of this frame is variable */
} stm32wlxx_rcp_user_data_t;
static_assert(sizeof(stm32wlxx_rcp_user_data_t) <= STM32WLxx_RADIO_COMM_MTU_SIZE, "Size of stm32wlxx_rcp_user_data_t shall not exceed STM32WLxx_RADIO_COMM_MTU_SIZE");

/**
 * @brief Payload for the Apply Radio Config command
 */
typedef __PACKED_STRUCT {
    stm32wlxx_radio_comm_opcode_t opcode;
    uint8_t                       data[STM32WLxx_RADIO_COMM_USER_DATA_MAX_SIZE]; /*!< It's not mandatory to always use the full buffer, so the actual length of this frame is variable */
} stm32wlxx_rcp_user_data_irq_followup_t;
static_assert(sizeof(stm32wlxx_rcp_user_data_irq_followup_t) <= STM32WLxx_RADIO_COMM_MTU_SIZE, "Size of stm32wlxx_rcp_user_data_irq_followup_t shall not exceed STM32WLxx_RADIO_COMM_MTU_SIZE");


#endif /* COMM_DEF_H */
