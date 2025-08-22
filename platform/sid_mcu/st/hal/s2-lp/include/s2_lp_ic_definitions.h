/**
  ******************************************************************************
  * @file    s2_lp_ic_definitions.h
  * @brief   S2-LP hardware-related definitions (e.g. registers, commands, etc.)
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024-2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#ifndef __S2_LP_IC_DEFINITIONS_H_
#define __S2_LP_IC_DEFINITIONS_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include <assert.h>
#include <stdint.h>
#include <cmsis_compiler.h>

/* Exported constants --------------------------------------------------------*/

/** @defgroup S2_LP_Register_Addresses Addresses of the internal S2-LP registers
 * @{
 */
#define S2_LP_IC_REG_ADDR_GPIO0_CONF                ((uint8_t)0x00u)
#define S2_LP_IC_REG_ADDR_GPIO1_CONF                ((uint8_t)0x01u)
#define S2_LP_IC_REG_ADDR_GPIO2_CONF                ((uint8_t)0x02u)
#define S2_LP_IC_REG_ADDR_GPIO3_CONF                ((uint8_t)0x03u)
#define S2_LP_IC_REG_ADDR_SYNT3                     ((uint8_t)0x05u)
#define S2_LP_IC_REG_ADDR_SYNT2                     ((uint8_t)0x06u)
#define S2_LP_IC_REG_ADDR_SYNT1                     ((uint8_t)0x07u)
#define S2_LP_IC_REG_ADDR_SYNT0                     ((uint8_t)0x08u)
#define S2_LP_IC_REG_ADDR_IF_OFFSET_ANA             ((uint8_t)0x09u)
#define S2_LP_IC_REG_ADDR_IF_OFFSET_DIG             ((uint8_t)0x0Au)
#define S2_LP_IC_REG_ADDR_CHSPACE                   ((uint8_t)0x0Cu)
#define S2_LP_IC_REG_ADDR_CHNUM                     ((uint8_t)0x0Du)
#define S2_LP_IC_REG_ADDR_MOD4                      ((uint8_t)0x0Eu)
#define S2_LP_IC_REG_ADDR_MOD3                      ((uint8_t)0x0Fu)
#define S2_LP_IC_REG_ADDR_MOD2                      ((uint8_t)0x10u)
#define S2_LP_IC_REG_ADDR_MOD1                      ((uint8_t)0x11u)
#define S2_LP_IC_REG_ADDR_MOD0                      ((uint8_t)0x12u)
#define S2_LP_IC_REG_ADDR_CHFLT                     ((uint8_t)0x13u)
#define S2_LP_IC_REG_ADDR_AFC2                      ((uint8_t)0x14u)
#define S2_LP_IC_REG_ADDR_AFC1                      ((uint8_t)0x15u)
#define S2_LP_IC_REG_ADDR_AFC0                      ((uint8_t)0x16u)
#define S2_LP_IC_REG_ADDR_RSSI_FLT                  ((uint8_t)0x17u)
#define S2_LP_IC_REG_ADDR_RSSI_TH                   ((uint8_t)0x18u)

#define S2_LP_IC_REG_ADDR_AGCCTRL4                  ((uint8_t)0x1Au)
#define S2_LP_IC_REG_ADDR_AGCCTRL3                  ((uint8_t)0x1Bu)
#define S2_LP_IC_REG_ADDR_AGCCTRL2                  ((uint8_t)0x1Cu)
#define S2_LP_IC_REG_ADDR_AGCCTRL1                  ((uint8_t)0x1Du)
#define S2_LP_IC_REG_ADDR_AGCCTRL0                  ((uint8_t)0x1Eu)
#define S2_LP_IC_REG_ADDR_ANT_SELECT_CONF           ((uint8_t)0x1Fu)
#define S2_LP_IC_REG_ADDR_CLOCKREC2                 ((uint8_t)0x20u)
#define S2_LP_IC_REG_ADDR_CLOCKREC1                 ((uint8_t)0x21u)

#define S2_LP_IC_REG_ADDR_PCKTCTRL6                 ((uint8_t)0x2Bu)
#define S2_LP_IC_REG_ADDR_PCKTCTRL5                 ((uint8_t)0x2Cu)
#define S2_LP_IC_REG_ADDR_PCKTCTRL4                 ((uint8_t)0x2Du)
#define S2_LP_IC_REG_ADDR_PCKTCTRL3                 ((uint8_t)0x2Eu)
#define S2_LP_IC_REG_ADDR_PCKTCTRL2                 ((uint8_t)0x2Fu)
#define S2_LP_IC_REG_ADDR_PCKTCTRL1                 ((uint8_t)0x30u)
#define S2_LP_IC_REG_ADDR_PCKTLEN1                  ((uint8_t)0x31u)
#define S2_LP_IC_REG_ADDR_PCKTLEN0                  ((uint8_t)0x32u)
#define S2_LP_IC_REG_ADDR_SYNC3                     ((uint8_t)0x33u)
#define S2_LP_IC_REG_ADDR_SYNC2                     ((uint8_t)0x34u)
#define S2_LP_IC_REG_ADDR_SYNC1                     ((uint8_t)0x35u)
#define S2_LP_IC_REG_ADDR_SYNC0                     ((uint8_t)0x36u)
#define S2_LP_IC_REG_ADDR_QI                        ((uint8_t)0x37u)
#define S2_LP_IC_REG_ADDR_PCKT_PSTMBL               ((uint8_t)0x38u)
#define S2_LP_IC_REG_ADDR_PROTOCOL2                 ((uint8_t)0x39u)
#define S2_LP_IC_REG_ADDR_PROTOCOL1                 ((uint8_t)0x3Au)
#define S2_LP_IC_REG_ADDR_PROTOCOL0                 ((uint8_t)0x3Bu)
#define S2_LP_IC_REG_ADDR_FIFO_CONFIG3              ((uint8_t)0x3Cu)
#define S2_LP_IC_REG_ADDR_FIFO_CONFIG2              ((uint8_t)0x3Du)
#define S2_LP_IC_REG_ADDR_FIFO_CONFIG1              ((uint8_t)0x3Eu)
#define S2_LP_IC_REG_ADDR_FIFO_CONFIG0              ((uint8_t)0x3Fu)
#define S2_LP_IC_REG_ADDR_PCKT_FLT_OPTIONS          ((uint8_t)0x40u)
#define S2_LP_IC_REG_ADDR_PCKT_FLT_GOALS4           ((uint8_t)0x41u)
#define S2_LP_IC_REG_ADDR_PCKT_FLT_GOALS3           ((uint8_t)0x42u)
#define S2_LP_IC_REG_ADDR_SEC_SYNC3                 ((uint8_t)0x42u)
#define S2_LP_IC_REG_ADDR_PCKT_FLT_GOALS2           ((uint8_t)0x43u)
#define S2_LP_IC_REG_ADDR_SEC_SYNC2                 ((uint8_t)0x43u)
#define S2_LP_IC_REG_ADDR_PCKT_FLT_GOALS1           ((uint8_t)0x44u)
#define S2_LP_IC_REG_ADDR_SEC_SYNC1                 ((uint8_t)0x44u)
#define S2_LP_IC_REG_ADDR_PCKT_FLT_GOALS0           ((uint8_t)0x45u)
#define S2_LP_IC_REG_ADDR_SEC_SYNC0                 ((uint8_t)0x45u)
#define S2_LP_IC_REG_ADDR_TIMERS5                   ((uint8_t)0x46u)
#define S2_LP_IC_REG_ADDR_TIMERS4                   ((uint8_t)0x47u)
#define S2_LP_IC_REG_ADDR_TIMERS3                   ((uint8_t)0x48u)
#define S2_LP_IC_REG_ADDR_TIMERS2                   ((uint8_t)0x49u)
#define S2_LP_IC_REG_ADDR_TIMERS1                   ((uint8_t)0x4Au)
#define S2_LP_IC_REG_ADDR_TIMERS0                   ((uint8_t)0x4Bu)

#define S2_LP_IC_REG_ADDR_IRQ_MASK3                 ((uint8_t)0x50u)
#define S2_LP_IC_REG_ADDR_IRQ_MASK2                 ((uint8_t)0x51u)
#define S2_LP_IC_REG_ADDR_IRQ_MASK1                 ((uint8_t)0x52u)
#define S2_LP_IC_REG_ADDR_IRQ_MASK0                 ((uint8_t)0x53u)

#define S2_LP_IC_REG_ADDR_TIMER_CONF3               ((uint8_t)0x55u)

#define S2_LP_IC_REG_ADDR_PA_POWER8                 ((uint8_t)0x5Au)
#define S2_LP_IC_REG_ADDR_PA_POWER7                 ((uint8_t)0x5Bu)
#define S2_LP_IC_REG_ADDR_PA_POWER6                 ((uint8_t)0x5Cu)
#define S2_LP_IC_REG_ADDR_PA_POWER5                 ((uint8_t)0x5Du)
#define S2_LP_IC_REG_ADDR_PA_POWER4                 ((uint8_t)0x5Eu)
#define S2_LP_IC_REG_ADDR_PA_POWER3                 ((uint8_t)0x5Fu)
#define S2_LP_IC_REG_ADDR_PA_POWER2                 ((uint8_t)0x60u)
#define S2_LP_IC_REG_ADDR_PA_POWER1                 ((uint8_t)0x61u)
#define S2_LP_IC_REG_ADDR_PA_POWER0                 ((uint8_t)0x62u)
#define S2_LP_IC_REG_ADDR_PA_CONFIG1                ((uint8_t)0x63u)
#define S2_LP_IC_REG_ADDR_PA_CONFIG0                ((uint8_t)0x64u)
#define S2_LP_IC_REG_ADDR_SYNTH_CONFIG2             ((uint8_t)0x65u)

#define S2_LP_IC_REG_ADDR_XO_RCO_CONF1              ((uint8_t)0x6Cu)
#define S2_LP_IC_REG_ADDR_XO_RCO_CONF0              ((uint8_t)0x6Du)
#define S2_LP_IC_REG_ADDR_RCO_CALIBR_CONF3          ((uint8_t)0x6Eu)
#define S2_LP_IC_REG_ADDR_RCO_CALIBR_CONF2          ((uint8_t)0x6Fu)
#define S2_LP_IC_REG_ADDR_PM_CONF4                  ((uint8_t)0x75u)
#define S2_LP_IC_REG_ADDR_PM_CONF3                  ((uint8_t)0x76u)
#define S2_LP_IC_REG_ADDR_PM_CONF2                  ((uint8_t)0x77u)
#define S2_LP_IC_REG_ADDR_PM_CONF1                  ((uint8_t)0x78u)
#define S2_LP_IC_REG_ADDR_PM_CONF0                  ((uint8_t)0x79u)
#define S2_LP_IC_REG_ADDR_MC_STATE1                 ((uint8_t)0x8Du)
#define S2_LP_IC_REG_ADDR_MC_STATE0                 ((uint8_t)0x8Eu)
#define S2_LP_IC_REG_ADDR_TX_FIFO_STATUS            ((uint8_t)0x8Fu)
#define S2_LP_IC_REG_ADDR_RX_FIFO_STATUS            ((uint8_t)0x90u)
#define S2_LP_IC_REG_ADDR_RCO_CALIBR_OUT4           ((uint8_t)0x94u)
#define S2_LP_IC_REG_ADDR_RCO_CALIBR_OUT3           ((uint8_t)0x95u)

#define S2_LP_IC_REG_ADDR_AFC_CORR                  ((uint8_t)0x9Eu)
#define S2_LP_IC_REG_ADDR_LINK_QUALIF2              ((uint8_t)0x9Fu)
#define S2_LP_IC_REG_ADDR_LINK_QUALIF1              ((uint8_t)0xA0u)
#define S2_LP_IC_REG_ADDR_RSSI_LEVEL                ((uint8_t)0xA2u)
#define S2_LP_IC_REG_ADDR_RX_PCKT_LEN1              ((uint8_t)0xA4u)
#define S2_LP_IC_REG_ADDR_RX_PCKT_LEN0              ((uint8_t)0xA5u)
#define S2_LP_IC_REG_ADDR_CRC_FIELD3                ((uint8_t)0xA6u)
#define S2_LP_IC_REG_ADDR_CRC_FIELD2                ((uint8_t)0xA7u)
#define S2_LP_IC_REG_ADDR_CRC_FIELD1                ((uint8_t)0xA8u)
#define S2_LP_IC_REG_ADDR_CRC_FIELD0                ((uint8_t)0xA9u)

#define S2_LP_IC_REG_ADDR_RSSI_LEVEL_RUN            ((uint8_t)0xEFu)
#define S2_LP_IC_REG_ADDR_DEVICE_INFO1              ((uint8_t)0xF0u)
#define S2_LP_IC_REG_ADDR_DEVICE_INFO0              ((uint8_t)0xF1u)
#define S2_LP_IC_REG_ADDR_IRQ_STATUS3               ((uint8_t)0xFAu)
#define S2_LP_IC_REG_ADDR_IRQ_STATUS2               ((uint8_t)0xFBu)
#define S2_LP_IC_REG_ADDR_IRQ_STATUS1               ((uint8_t)0xFCu)
#define S2_LP_IC_REG_ADDR_IRQ_STATUS0               ((uint8_t)0xFDu)

#define S2_LP_IC_REG_ADDR_FIFO_ACCESS               ((uint8_t)0xFFu)

/**
 * 
 */

/** @defgroup S2_LP_Register_Masks_Offsets Bit masks and bit offsets of S2-LP's registers
 * @{
 */
#define S2_LP_IC_REG_XO_RCO_CONF1_MASK_PD_CLKDIV    ((uint8_t)0x10u)
#define S2_LP_IC_REG_XO_RCO_CONF1_VAL_PD_CLKDIV_ON  ((uint8_t)0x00u)          /*!< 0: enable both dividers of digital clock (and reference clock for the SMPS) and IF-ADC clock */
#define S2_LP_IC_REG_XO_RCO_CONF1_VAL_PD_CLKDIV_OFF ((uint8_t)0x10u)          /*!< 1: disable both dividers of digital clock (and reference clock for the SMPS) and IF-ADC clock */

#define S2_LP_IC_REG_PROTOCOL2_MASK_CS_TIMEOUT      ((uint8_t)0x80u)
#define S2_LP_IC_REG_PROTOCOL2_MASK_SQI_TIMEOUT     ((uint8_t)0x40u)
#define S2_LP_IC_REG_PROTOCOL2_MASK_PQI_TIMEOUT     ((uint8_t)0x20u)
/**
 * 
 */

/** @defgroup S2_LP_Commands Available S2-LP command codes
 * @{
 */
#define S2_LP_IC_CMD_TX                             ((uint8_t)(0x60u))        /*!< Start to transmit; valid only from READY */
#define S2_LP_IC_CMD_RX                             ((uint8_t)(0x61u))        /*!< Start to receive; valid only from READY */
#define S2_LP_IC_CMD_READY                          ((uint8_t)(0x62u))        /*!< Go to READY; valid only from STANDBY or SLEEP or LOCK */
#define S2_LP_IC_CMD_STANDBY                        ((uint8_t)(0x63u))        /*!< Go to STANDBY; valid only from READY */
#define S2_LP_IC_CMD_SLEEP                          ((uint8_t)(0x64u))        /*!< Go to SLEEP; valid only from READY */
#define S2_LP_IC_CMD_LOCKRX                         ((uint8_t)(0x65u))        /*!< Go to LOCK state by using the RX configuration of the synth; valid only from READY */
#define S2_LP_IC_CMD_LOCKTX                         ((uint8_t)(0x66u))        /*!< Go to LOCK state by using the TX configuration of the synth; valid only from READY */
#define S2_LP_IC_CMD_SABORT                         ((uint8_t)(0x67u))        /*!< Force exit form TX or RX states and go to READY state; valid only from TX or RX */
#define S2_LP_IC_CMD_LDC_RELOAD                     ((uint8_t)(0x68u))        /*!< LDC Mode: Reload the LDC timer with the value stored in the  LDC_PRESCALER / COUNTER  registers; valid from all states  */
#define S2_LP_IC_CMD_RCO_CALIB                      ((uint8_t)(0x69u))        /*!< Start (or re-start) the RCO calibration */
#define S2_LP_IC_CMD_SRES                           ((uint8_t)(0x70u))        /*!< Reset of all digital part, except SPI registers */
#define S2_LP_IC_CMD_FLUSHRXFIFO                    ((uint8_t)(0x71u))        /*!< Clean the RX FIFO; valid from all states */
#define S2_LP_IC_CMD_FLUSHTXFIFO                    ((uint8_t)(0x72u))        /*!< Clean the TX FIFO; valid from all states */
#define S2_LP_IC_CMD_SEQUENCE_UPDATE                ((uint8_t)(0x73u))        /*!< Autoretransmission: Reload the Packet sequence counter with the value stored in the PROTOCOL[2] register valid from all states */
/**
 * 
 */

/** @defgroup S2_LP_IRQ_Flags S2-LP IRQ flags
 * @{
 */
#define S2_LP_IC_IRQ_MASK_NONE                      ((uint32_t)(0u))
#define S2_LP_IC_IRQ_MASK_RX_DATA_READY             ((uint32_t)(1u <<  0))    /*!< IRQ: RX data ready */
#define S2_LP_IC_IRQ_MASK_RX_DATA_DISC              ((uint32_t)(1u <<  1))    /*!< IRQ: RX data discarded (upon filtering) */
#define S2_LP_IC_IRQ_MASK_TX_DATA_SENT              ((uint32_t)(1u <<  2))    /*!< IRQ: TX data sent */
#define S2_LP_IC_IRQ_MASK_MAX_RE_TX_REACH           ((uint32_t)(1u <<  3))    /*!< IRQ: Max re-TX reached */
#define S2_LP_IC_IRQ_MASK_CRC_ERROR                 ((uint32_t)(1u <<  4))    /*!< IRQ: CRC error */
#define S2_LP_IC_IRQ_MASK_TX_FIFO_ERROR             ((uint32_t)(1u <<  5))    /*!< IRQ: TX FIFO underflow/overflow error */
#define S2_LP_IC_IRQ_MASK_RX_FIFO_ERROR             ((uint32_t)(1u <<  6))    /*!< IRQ: RX FIFO underflow/overflow error */
#define S2_LP_IC_IRQ_MASK_TX_FIFO_ALMOST_FULL       ((uint32_t)(1u <<  7))    /*!< IRQ: TX FIFO almost full */
#define S2_LP_IC_IRQ_MASK_TX_FIFO_ALMOST_EMPTY      ((uint32_t)(1u <<  8))    /*!< IRQ: TX FIFO almost empty */
#define S2_LP_IC_IRQ_MASK_RX_FIFO_ALMOST_FULL       ((uint32_t)(1u <<  9))    /*!< IRQ: RX FIFO almost full */
#define S2_LP_IC_IRQ_MASK_RX_FIFO_ALMOST_EMPTY      ((uint32_t)(1u << 10))    /*!< IRQ: RX FIFO almost empty  */
#define S2_LP_IC_IRQ_MASK_MAX_BO_CCA_REACH          ((uint32_t)(1u << 11))    /*!< IRQ: Max number of back-off during CCA */
#define S2_LP_IC_IRQ_MASK_VALID_PREAMBLE            ((uint32_t)(1u << 12))    /*!< IRQ: Valid preamble detected */
#define S2_LP_IC_IRQ_MASK_VALID_SYNC                ((uint32_t)(1u << 13))    /*!< IRQ: Sync word detected */
#define S2_LP_IC_IRQ_MASK_RSSI_ABOVE_TH             ((uint32_t)(1u << 14))    /*!< IRQ: RSSI above threshold */
#define S2_LP_IC_IRQ_MASK_WKUP_TOUT_LDC             ((uint32_t)(1u << 15))    /*!< IRQ: Wake-up timeout in LDC mode */
#define S2_LP_IC_IRQ_MASK_READY                     ((uint32_t)(1u << 16))    /*!< IRQ: READY state */
#define S2_LP_IC_IRQ_MASK_STANDBY_DELAYED           ((uint32_t)(1u << 17))    /*!< IRQ: STANDBY state after MCU_CK_CONF_CLOCK_TAIL_X clock cycles */
#define S2_LP_IC_IRQ_MASK_LOW_BATT_LVL              ((uint32_t)(1u << 18))    /*!< IRQ: Battery level below threshold*/
#define S2_LP_IC_IRQ_MASK_POR                       ((uint32_t)(1u << 19))    /*!< IRQ: Power On Reset */
#define S2_LP_IC_IRQ_MASK_BOR                       ((uint32_t)(1u << 20))    /*!< IRQ: Brown out event (both accurate and inaccurate)*/
#define S2_LP_IC_IRQ_MASK_LOCK                      ((uint32_t)(1u << 21))    /*!< IRQ: LOCK state */
#define S2_LP_IC_IRQ_MASK_VCO_CALIBRATION_END       ((uint32_t)(1u << 22))    /*!< IRQ: only for debug; Power Management startup timer expiration (see reg PM_START_COUNTER, 0xB5) */
#define S2_LP_IC_IRQ_MASK_PA_CALIBRATION_END        ((uint32_t)(1u << 23))    /*!< IRQ: only for debug; Crystal oscillator settling time counter expired */
#define S2_LP_IC_IRQ_MASK_PM_COUNT_EXPIRED          ((uint32_t)(1u << 24))    /*!< IRQ: only for debug; Power Management startup timer expiration (see reg PM_START_COUNTER, 0xB5) */
#define S2_LP_IC_IRQ_MASK_XO_COUNT_EXPIRED          ((uint32_t)(1u << 25))    /*!< IRQ: only for debug; Crystal oscillator settling time counter expired */
#define S2_LP_IC_IRQ_MASK_TX_START_TIME             ((uint32_t)(1u << 26))    /*!< IRQ: only for debug; TX circuitry startup time; see TX_START_COUNTER */
#define S2_LP_IC_IRQ_MASK_RX_START_TIME             ((uint32_t)(1u << 27))    /*!< IRQ: only for debug; RX circuitry startup time; see TX_START_COUNTER */
#define S2_LP_IC_IRQ_MASK_RX_TIMEOUT                ((uint32_t)(1u << 28))    /*!< IRQ: RX operation timeout */
#define S2_LP_IC_IRQ_MASK_RX_SNIFF_TIMEOUT          ((uint32_t)(1u << 29))    /*!< IRQ: RX sniff operation timeout */
#define S2_LP_IC_IRQ_MASK_ALL_IRQ                   ((uint32_t)(0x3FFFFFFFu)) /*!< All the above mentioned IRQs */
/**
 * 
 */

/** @defgroup S2_LP_Limits_And_Specs Defintions for various limiting factors and fixed values coming from S2-LP IC design
 * @{
 */
#define S2_LP_IC_DIG_DOMAIN_XTAL_THRESH             ((uint32_t)  30000000u)   /*!< Digital domain logic threshold for XTAL in Hz */
#define S2_LP_IC_REF_FREQ_THRESH                    ((uint32_t)  30000000u)   /*!< Synthesizer reference frequency threshold in Hz */

#define S2_LP_IC_HIGH_BAND_LOWER_LIMIT              ((uint32_t) 825900000u)   /*!< Lower limit of the high band: 860 MHz   (S2-LPQTR)*/
#define S2_LP_IC_HIGH_BAND_UPPER_LIMIT              ((uint32_t)1056000000u)   /*!< Upper limit of the high band: 940 MHz  (S2-LPCBQTR)*/
#define S2_LP_IC_MIDDLE_BAND_LOWER_LIMIT            ((uint32_t) 412900000u)   /*!< Lower limit of the middle band: 430 MHz (S2-LPQTR)*/
#define S2_LP_IC_MIDDLE_BAND_UPPER_LIMIT            ((uint32_t) 527100000u)   /*!< Upper limit of the middle band: 470 MHz (S2-LPCBQTR)*/

#define S2_LP_IC_MINIMUM_DATARATE                   ((uint32_t)   100u)       /*!< Minimum datarate supported by S2-LP 100 bps */
#define S2_LP_IC_MAXIMUM_DATARATE                   ((uint32_t)250000u)       /*!< Maximum datarate supported by S2-LP 250 kbps */

#define S2_LP_IC_VCO_CENTER_FREQ                    ((uint32_t)3600000000u)   /*!< VCO center frequency in Hz */

#define S2_LP_IC_HIGH_BAND_FACTOR                   (4u)                      /*!< Band select factor for high band. Factor B in the equation 2 */
#define S2_LP_IC_MIDDLE_BAND_FACTOR                 (8u)                      /*!< Band select factor for middle band. Factor B in the equation 2 */

#define S2_LP_IC_SYNC_WORD_LENGTH_LIMIT_BITS        (32u)                     /*!< Maximum size of the sync word that can be configured in the S2-LP (in bits) */

#define S2_LP_IC_DATARATE_EXPONENT_MAX              (15u)                     /*!< Maximum possible value for the exponent in datarate calculation */
#define S2_LP_IC_DATARATE_MANTISSA_MAX              (UINT16_MAX)              /*!< Maximum possible value for the mantissa in datarate calculation */

#define S2_LP_IC_FDEV_EXPONENT_MAX                  (15u)                     /*!< Maximum possible value for the exponent in frequency deviation (Fdev) calculation */
#define S2_LP_IC_FDEV_MANTISSA_MAX                  (UINT8_MAX)               /*!< Maximum possible value for the mantissa in frequency deviation (Fdev) calculation */

#define S2_LP_IC_RX_TIMER_PREDIV                    (1210u)                   /*!< Rx timer clock is Fdig divided by this factor */

#define S2_LP_IC_NUM_GPIO_PINS                      (4u)                      /*!< Total numbers of GPIO pins on S2-LP device */

#define S2_LP_IC_PQI_TH_FACTOR                      (4u)                      /*!< The preamble quality indicator threshold is 4 x PQI_TH */

#define S2_LP_IC_RX_FIFO_SIZE                       (128u)                    /*!< Rx FIFO size in bytes */
#define S2_LP_IC_TX_FIFO_SIZE                       (128u)                    /*!< Tx FIFO size in bytes */

#define S2_LP_IC_MAX_PA_POWER_DBM                   (14)                      /*!< Maximum output power of the built-in PA (expressed in dBm) */
#define S2_LP_IC_MIN_PA_POWER_DBM                   (-30)                     /*!< Minimum output power of the built-in PA (expressed in dBm) */

#define S2_LP_IC_PM_START_COUNTER_UPPER_LIMIT       (0xFFu)                   /*!< Maximum allowed value for the PM_START_COUNTER register value */

/**
 * 
 */

/* Exported macro ------------------------------------------------------------*/

#define S2_LP_IC_IS_FREQ_BAND_HIGH(FREQUENCY)       (((FREQUENCY) >= (S2_LP_IC_HIGH_BAND_LOWER_LIMIT)) && ((FREQUENCY) <= (S2_LP_IC_HIGH_BAND_UPPER_LIMIT)))
#define S2_LP_IC_IS_FREQ_BAND_MIDDLE(FREQUENCY)     (((FREQUENCY) >= (S2_LP_IC_MIDDLE_BAND_LOWER_LIMIT)) && ((FREQUENCY) <= (S2_LP_IC_MIDDLE_BAND_UPPER_LIMIT)))
#define S2_LP_IC_IS_VALID_FREQ_BAND(FREQUENCY)      ((S2_LP_IC_IS_FREQ_BAND_HIGH(FREQUENCY)) || ( S2_LP_IC_IS_FREQ_BAND_MIDDLE(FREQUENCY)))

#define S2_LP_IC_IS_VALID_DATARATE(DR, __F_dig__)   (((DR) >= S2_LP_IC_MINIMUM_DATARATE) && ((DR) <= ((S2_LP_IC_MAXIMUM_DATARATE * ((__F_dig__) / 1000000u))/ 26u)))

#define S2_LP_IC_FDEV_LOWER_LIMIT(__F_xo__)         ((__F_xo__) >> 22)                          /*!< Minimum value of the frequency deviation */
#define S2_LP_IC_FDEV_UPPER_LIMIT(__F_xo__)         ((787109u * ((__F_xo__) / 1000000u)) / 26u) /*!< Maximum value of the frequency deviation */
#define S2_LP_IC_IS_VALID_FDEV(FDEV, __F_xo__)      (((FDEV) >= S2_LP_IC_FDEV_LOWER_LIMIT(__F_xo__)) && ((FDEV) <= S2_LP_IC_FDEV_UPPER_LIMIT(__F_xo__)))

#define S2_LP_IC_CH_BW_LOWER_LIMIT(__F_xo__)        ((1100u * ((__F_xo__) / 1000000u)) / 26u)   /*!< Minimum value of the channel filter bandwidth */
#define S2_LP_IC_CH_BW_UPPER_LIMIT(__F_xo__)        ((800100u * ((__F_xo__) / 1000000u)) / 26u) /*!< Maximum value of the channel filter bandwidth */
#define S2_LP_IC_IS_VALID_CH_BW(BW,__F_xo__)        (((BW) >= S2_LP_IC_CH_BW_LOWER_LIMIT(__F_xo__)) && ((BW) <= S2_LP_IC_CH_BW_UPPER_LIMIT(__F_xo__)))

#define S2_LP_IC_FIFO_LEVEL_TO_AFTHR(__LVL__)       (S2_LP_IC_RX_FIFO_SIZE + 2u - (__LVL__)) /*!< Convert FIFO level in bytes into the Almost Full Threshold in terms of S2-LP */

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Known revisions of the S2-LP IC
 */
typedef enum
{
    S2_LP_CUT_2_0 = 0x81u, /*!< Cut 2.0 */
    S2_LP_CUT_2_1 = 0x91u, /*!< Cut 2.1 */
    S2_LP_CUT_3_0 = 0xC1u, /*!< Cut 3.0 */
    S2_LP_CUT_3_1 = 0xD2u, /*!< Cut 3.1 */
} s2_lp_ic_cut_t;

/**
 * @brief Available S2-LP GPIO pins
 */
typedef enum {
    S2_LP_GPIO_0 = S2_LP_IC_REG_ADDR_GPIO0_CONF, /*!< GPIO_0 selected */
    S2_LP_GPIO_1 = S2_LP_IC_REG_ADDR_GPIO1_CONF, /*!< GPIO_1 selected */
    S2_LP_GPIO_2 = S2_LP_IC_REG_ADDR_GPIO2_CONF, /*!< GPIO_2 selected */
    S2_LP_GPIO_3 = S2_LP_IC_REG_ADDR_GPIO3_CONF, /*!< GPIO_3 selected */
} s2_lp_ic_gpio_pin_t;

/**
 * @brief Available S2-LP GPIO modes
 */
typedef enum {
    S2_LP_GPIO_MODE_HI_Z              = 0, /*!< Pin is tri-stated */
    S2_LP_GPIO_MODE_DIGITAL_INPUT     = 1, /*!< Pin is a digital input */
    S2_LP_GPIO_MODE_DIGITAL_OUTPUT_LP = 2, /*!< Pin is a push-pull output (low current) */
    S2_LP_GPIO_MODE_DIGITAL_OUTPUT_HP = 3, /*!< Pin is a push-pull output (high current) */
} s2_lp_ic_gpio_mode_t;

/**
 * @brief  S2-LP I/O signals that can be routed to the GPIO pins
 */
typedef enum {
    /* Available outputs */
    S2_LP_GPIO_SIG_OUT_IRQ                       =  0, /*!< nIRQ (Interrupt Request, active low) */
    S2_LP_GPIO_SIG_OUT_POR_INV                   =  1, /*!< POR inverted (active low) */
    S2_LP_GPIO_SIG_OUT_WUT_EXP                   =  2, /*!< Wake-Up Timer expiration: "1" when WUT has expired */
    S2_LP_GPIO_SIG_OUT_LOW_BAT                   =  3, /*!< Low battery detection: "1" when battery is below threshold setting */
    S2_LP_GPIO_SIG_OUT_TX_DATA                   =  4, /*!< TX data internal clock output (TX data are sampled on the rising edge of it) */
    S2_LP_GPIO_SIG_OUT_TX_STATE                  =  5, /*!< TX state indication: "1" when S2LP1 is passing in the TX state */
    S2_LP_GPIO_SIG_OUT_TXRX_FIFO_ALMOST_EMPTY    =  6, /*!< TX/RX FIFO Almost Empty Flag */
    S2_LP_GPIO_SIG_OUT_TXRX_FIFO_ALMOST_FULL     =  7, /*!< TX/RX FIFO Almost Full Flag */
    S2_LP_GPIO_SIG_OUT_RX_DATA                   =  8, /*!< RX data output */
    S2_LP_GPIO_SIG_OUT_RX_CLOCK                  =  9, /*!< RX clock output (recovered from received data) */
    S2_LP_GPIO_SIG_OUT_RX_STATE                  = 10, /*!< RX state indication: "1" when demodulator is ON */
    S2_LP_GPIO_SIG_OUT_NOT_STANDBY_SLEEP         = 11, /*!< VDD when the device is not in SLEEP or STANDBY */
    S2_LP_GPIO_SIG_OUT_STANDBY                   = 12, /*!< VDD when device is in STANDBY */
    S2_LP_GPIO_SIG_OUT_ANTENNA_SWITCH            = 13, /*!< Antenna switch used for antenna diversity  */
    S2_LP_GPIO_SIG_OUT_VALID_PREAMBLE            = 14, /*!< Valid Preamble Detected Flag */
    S2_LP_GPIO_SIG_OUT_SYNC_DETECTED             = 15, /*!< Sync WordSync Word Detected Flag */
    S2_LP_GPIO_SIG_OUT_RSSI_THRESHOLD            = 16, /*!< RSSI above threshold */
    S2_LP_GPIO_SIG_OUT_MCU_CLOCK                 = 17, /*!< MCU Clock */
    S2_LP_GPIO_SIG_OUT_TX_RX_MODE                = 18, /*!< TX or RX mode indicator (to enable an external range extender) */
    S2_LP_GPIO_SIG_OUT_VDD                       = 19, /*!< VDD (to emulate an additional GPIO of the MCU, programmable by SPI) */
    S2_LP_GPIO_SIG_OUT_GND                       = 20, /*!< GND (to emulate an additional GPIO of the MCU, programmable by SPI) */
    S2_LP_GPIO_SIG_OUT_SMPS_EXT                  = 21, /*!< External SMPS enable signal (active high) */
    S2_LP_GPIO_SIG_OUT_SLEEP                     = 22, /*!< Device in SLEEP (active high) */
    S2_LP_GPIO_SIG_OUT_READY                     = 23, /*!< Device in READY (active high) */
    S2_LP_GPIO_SIG_OUT_LOCK                      = 24, /*!< Device in LOCK (active high) */
    S2_LP_GPIO_SIG_OUT_WAIT_FOR_LOCK_SIG         = 25, /*!< Device waiting for LOCK (active high) */
    S2_LP_GPIO_SIG_OUT_TX_DATA_OOK_SIGNAL        = 26, /*!< TX_DATA_OOK signal (internal control signal generated in the OOK analog smooth mode) */
    S2_LP_GPIO_SIG_OUT_WAIT_FOR_READY2_SIG       = 27, /*!< Device waiting for a high level of the READY2 signal from XO */
    S2_LP_GPIO_SIG_OUT_WAIT_FOR_TIMER_FOR_PM_SET = 28, /*!< Device waiting for timer expiration to allow PM block settling */
    S2_LP_GPIO_SIG_OUT_WAIT_VCO_CALIBRATION      = 29, /*!< Device waiting for end of VCO calibration */
    S2_LP_GPIO_SIG_OUT_ENABLE_SYNTH_FULL_CIRCUIT = 30, /*!< Device enables the full circuitry of the SYNTH block */

    /* Available inputs */
    S2_LP_GPIO_SIG_IN_TX_COMMAND                 =  0, /*!< 1 >> TX command */
    S2_LP_GPIO_SIG_IN_RX_COMMAND                 =  1, /*!< 1 >> RX command */
    S2_LP_GPIO_SIG_IN_TX_DATA_INPUT_FOR_DIRECTRF =  2, /*!< TX data input for direct modulation */
    S2_LP_GPIO_SIG_IN_DATA_WAKEUP                =  3, /*!< Wake-up from external input (sensor output) */
    S2_LP_GPIO_SIG_IN_EXT_CLOCK_AT_34_7KHZ       =  4, /*!< External clock @ 34.7 kHz (used for LDC modes timing) */
} s2_lp_ic_gpio_signal_t;

/**
 * @brief Helper type for IRQ status indication
 */
typedef enum {
    S2_LP_EVENT_NOT_PRESENT = 0,
    S2_LP_EVENT_ACTIVE      = 1,
} s2_lp_ic_event_status_t;

/**
 * @brief IRQ bitfield structure for S2-LP to be used to retrieve all the IRQ events from the IRQ registers IRQ_STATUS[3:0]
 * @note  The fields order in the structure depends on used endianness (little or big endian). S2-LP provides IRQ status in
 *        Big endian foramt. It is the responsibility of the user to swap the IRQ_STATUS[3:0] bytes on Little Endian MCUs
 */
typedef __PACKED_UNION {
    __PACKED_STRUCT {
#if SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN
        uint8_t                                          : 2; /*!< Unused/reserved bits */
        s2_lp_ic_event_status_t  IRQ_RX_SNIFF_TIMEOUT    : 1; /*!< IRQ: RX sniff operation timeout */
        s2_lp_ic_event_status_t  IRQ_RX_TIMEOUT          : 1; /*!< IRQ: RX operation timeout */
        s2_lp_ic_event_status_t  IRQ_RX_START_TIME       : 1; /*!< IRQ: only for debug; RX circuitry startup time; see TX_START_COUNTER */
        s2_lp_ic_event_status_t  IRQ_TX_START_TIME       : 1; /*!< IRQ: only for debug; TX circuitry startup time; see TX_START_COUNTER */
        s2_lp_ic_event_status_t  IRQ_XO_COUNT_EXPIRED    : 1; /*!< IRQ: only for debug; Crystal oscillator settling time counter expired */
        s2_lp_ic_event_status_t  IRQ_PM_COUNT_EXPIRED    : 1; /*!< IRQ: only for debug; Power Management startup timer expiration (see reg PM_START_COUNTER, 0xB5) */

        s2_lp_ic_event_status_t  IRQ_PA_CALIBRATION_END  : 1; /*!< IRQ: End of PA calibration procedure */
        s2_lp_ic_event_status_t  IRQ_VCO_CALIBRATION_END : 1; /*!< IRQ: End of VCO calibration procedure */
        s2_lp_ic_event_status_t  IRQ_LOCK                : 1; /*!< IRQ: LOCK state */
        s2_lp_ic_event_status_t  IRQ_BOR                 : 1; /*!< IRQ: Brown out event (both accurate and inaccurate)*/
        s2_lp_ic_event_status_t  IRQ_POR                 : 1; /*!< IRQ: Power On Reset */
        s2_lp_ic_event_status_t  IRQ_LOW_BATT_LVL        : 1; /*!< IRQ: Battery level below threshold*/
        s2_lp_ic_event_status_t  IRQ_STANDBY_DELAYED     : 1; /*!< IRQ: STANDBY state after MCU_CK_CONF_CLOCK_TAIL_X clock cycles */
        s2_lp_ic_event_status_t  IRQ_READY               : 1; /*!< IRQ: READY state */

        s2_lp_ic_event_status_t  IRQ_WKUP_TOUT_LDC       : 1; /*!< IRQ: Wake-up timeout in LDC mode */
        s2_lp_ic_event_status_t  IRQ_RSSI_ABOVE_TH       : 1; /*!< IRQ: RSSI above threshold */
        s2_lp_ic_event_status_t  IRQ_VALID_SYNC          : 1; /*!< IRQ: Sync word detected */
        s2_lp_ic_event_status_t  IRQ_VALID_PREAMBLE      : 1; /*!< IRQ: Valid preamble detected */
        s2_lp_ic_event_status_t  IRQ_MAX_BO_CCA_REACH    : 1; /*!< IRQ: Max number of back-off during CCA */
        s2_lp_ic_event_status_t  IRQ_RX_FIFO_ALMOST_EMPTY: 1; /*!< IRQ: RX FIFO almost empty  */
        s2_lp_ic_event_status_t  IRQ_RX_FIFO_ALMOST_FULL : 1; /*!< IRQ: RX FIFO almost full */
        s2_lp_ic_event_status_t  IRQ_TX_FIFO_ALMOST_EMPTY: 1; /*!< IRQ: TX FIFO almost empty */

        s2_lp_ic_event_status_t  IRQ_TX_FIFO_ALMOST_FULL : 1; /*!< IRQ: TX FIFO almost full */
        s2_lp_ic_event_status_t  IRQ_RX_FIFO_ERROR       : 1; /*!< IRQ: RX FIFO underflow/overflow error */
        s2_lp_ic_event_status_t  IRQ_TX_FIFO_ERROR       : 1; /*!< IRQ: TX FIFO underflow/overflow error */
        s2_lp_ic_event_status_t  IRQ_CRC_ERROR           : 1; /*!< IRQ: CRC error */
        s2_lp_ic_event_status_t  IRQ_MAX_RE_TX_REACH     : 1; /*!< IRQ: Max re-TX reached */
        s2_lp_ic_event_status_t  IRQ_TX_DATA_SENT        : 1; /*!< IRQ: TX data sent */
        s2_lp_ic_event_status_t  IRQ_RX_DATA_DISC        : 1; /*!< IRQ: RX data discarded (upon filtering) */
        s2_lp_ic_event_status_t  IRQ_RX_DATA_READY       : 1; /*!< IRQ: RX data ready */
#else
        s2_lp_ic_event_status_t  IRQ_RX_DATA_READY       : 1; /*!< IRQ: RX data ready */
        s2_lp_ic_event_status_t  IRQ_RX_DATA_DISC        : 1; /*!< IRQ: RX data discarded (upon filtering) */
        s2_lp_ic_event_status_t  IRQ_TX_DATA_SENT        : 1; /*!< IRQ: TX data sent */
        s2_lp_ic_event_status_t  IRQ_MAX_RE_TX_REACH     : 1; /*!< IRQ: Max re-TX reached */
        s2_lp_ic_event_status_t  IRQ_CRC_ERROR           : 1; /*!< IRQ: CRC error */
        s2_lp_ic_event_status_t  IRQ_TX_FIFO_ERROR       : 1; /*!< IRQ: TX FIFO underflow/overflow error */
        s2_lp_ic_event_status_t  IRQ_RX_FIFO_ERROR       : 1; /*!< IRQ: RX FIFO underflow/overflow error */
        s2_lp_ic_event_status_t  IRQ_TX_FIFO_ALMOST_FULL : 1; /*!< IRQ: TX FIFO almost full */

        s2_lp_ic_event_status_t  IRQ_TX_FIFO_ALMOST_EMPTY: 1; /*!< IRQ: TX FIFO almost empty */
        s2_lp_ic_event_status_t  IRQ_RX_FIFO_ALMOST_FULL : 1; /*!< IRQ: RX FIFO almost full */
        s2_lp_ic_event_status_t  IRQ_RX_FIFO_ALMOST_EMPTY: 1; /*!< IRQ: RX FIFO almost empty  */
        s2_lp_ic_event_status_t  IRQ_MAX_BO_CCA_REACH    : 1; /*!< IRQ: Max number of back-off during CCA */
        s2_lp_ic_event_status_t  IRQ_VALID_PREAMBLE      : 1; /*!< IRQ: Valid preamble detected */
        s2_lp_ic_event_status_t  IRQ_VALID_SYNC          : 1; /*!< IRQ: Sync word detected */
        s2_lp_ic_event_status_t  IRQ_RSSI_ABOVE_TH       : 1; /*!< IRQ: RSSI above threshold */
        s2_lp_ic_event_status_t  IRQ_WKUP_TOUT_LDC       : 1; /*!< IRQ: Wake-up timeout in LDC mode */

        s2_lp_ic_event_status_t  IRQ_READY               : 1; /*!< IRQ: READY state */
        s2_lp_ic_event_status_t  IRQ_STANDBY_DELAYED     : 1; /*!< IRQ: STANDBY state after MCU_CK_CONF_CLOCK_TAIL_X clock cycles */
        s2_lp_ic_event_status_t  IRQ_LOW_BATT_LVL        : 1; /*!< IRQ: Battery level below threshold*/
        s2_lp_ic_event_status_t  IRQ_POR                 : 1; /*!< IRQ: Power On Reset */
        s2_lp_ic_event_status_t  IRQ_BOR                 : 1; /*!< IRQ: Brown out event (both accurate and inaccurate)*/
        s2_lp_ic_event_status_t  IRQ_LOCK                : 1; /*!< IRQ: LOCK state */
        s2_lp_ic_event_status_t  IRQ_VCO_CALIBRATION_END : 1; /*!< IRQ: End of VCO calibration procedure */
        s2_lp_ic_event_status_t  IRQ_PA_CALIBRATION_END  : 1; /*!< IRQ: End of PA calibration procedure */

        s2_lp_ic_event_status_t  IRQ_PM_COUNT_EXPIRED    : 1; /*!< IRQ: only for debug; Power Management startup timer expiration (see reg PM_START_COUNTER, 0xB5) */
        s2_lp_ic_event_status_t  IRQ_XO_COUNT_EXPIRED    : 1; /*!< IRQ: only for debug; Crystal oscillator settling time counter expired */
        s2_lp_ic_event_status_t  IRQ_TX_START_TIME       : 1; /*!< IRQ: only for debug; TX circuitry startup time; see TX_START_COUNTER */
        s2_lp_ic_event_status_t  IRQ_RX_START_TIME       : 1; /*!< IRQ: only for debug; RX circuitry startup time; see TX_START_COUNTER */
        s2_lp_ic_event_status_t  IRQ_RX_TIMEOUT          : 1; /*!< IRQ: RX operation timeout */
        s2_lp_ic_event_status_t  IRQ_RX_SNIFF_TIMEOUT    : 1; /*!< IRQ: RX sniff operation timeout */
        uint8_t                                          : 2; /*!< Unused/reserved bits */
#endif /* SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN */
    };
    uint32_t raw;
} sl2_lp_ic_irq_status_t;

/**
 * @brief Helper type for IRQ mask configuration
 */
typedef sl2_lp_ic_irq_status_t sl2_lp_ic_irq_mask_t;

/**
 * @brief  S2-LP States enumeration.
 */
typedef enum {
    S2_LP_MC_STATE_READY        = 0x00, /*!< READY */
    S2_LP_MC_STATE_SLEEP_NOFIFO = 0x01, /*!< SLEEP NO FIFO RETENTION */
    S2_LP_MC_STATE_STANDBY      = 0x02, /*!< STANDBY */
    S2_LP_MC_STATE_SLEEP        = 0x03, /*!< SLEEP */
    S2_LP_MC_STATE_LOCKON       = 0x0C, /*!< LOCKON */
    S2_LP_MC_STATE_RX           = 0x30, /*!< RX */
    S2_LP_MC_STATE_LOCK_ST      = 0x14, /*!< LOCK_ST */
    S2_LP_MC_STATE_TX           = 0x5C, /*!< TX */
    S2_LP_MC_STATE_SYNTH_SETUP  = 0x50, /*!< SYNTH_SETUP */
} s2_lp_ic_states_t;

/**
 * @brief Possible values for the MOD_TYPE field in the MOD2 register 
 */
typedef enum {
    S2_LP_MOD_TYPE_2FSK        =  0,
    S2_LP_MOD_TYPE_4FSK        =  1,
    S2_LP_MOD_TYPE_2GFSK_BT_1  =  2,
    S2_LP_MOD_TYPE_4GFSK_BT_1  =  3,
    S2_LP_MOD_TYPE_ASK_OOK     =  5,
    S2_LP_MOD_TYPE_UMODULATED  =  7,
    S2_LP_MOD_TYPE_2GFSK_BT_05 = 10,
    S2_LP_MOD_TYPE_4GFSK_BT_05 = 11,
} s2_lp_ic_mod_types_t;

/**
 * @brief The number of bytes used for the length field in PCKTCTRL4 register:
 *        • 0: 1 byte
 *        • 1: 2 bytes
 */
typedef enum {
    S2_LP_LEN_WID_1_BYTE  = 0,
    S2_LP_LEN_WID_2_BYTES = 1,
} s2_lp_ic_len_wid_t;

/**
 * @brief SP2-LP packet format for the packet handler engine
 * @note Set by the PCK_FRMT field of the PCKTCTRL3 register
 */
typedef enum {
    S2_LP_PCKT_FRMT_BASIC     = 0, /*!< Basic packet format */
    S2_LP_PCKT_FRMT_802_15_4G = 1, /*!< 802.15.4g-compliant format */
    S2_LP_PCKT_FRMT_UART_OTA  = 2, /*!< UART Over-the-Air */
    S2_LP_PCKT_FRMT_STACK     = 3, /*!< STack format */
} s2_lp_ic_pckt_frmt_t;

/**
 * @brief RX mode selection specified in PCKTCTRL3 register
 */
typedef enum {
    S2_LP_RX_MODE_NORMAL              = 0,
    S2_LP_RX_MODE_DIRECT_THROUGH_FIFO = 1, /*!< The packet bytes are continuously received and written in the RX FIFO without any processing */
    S2_LP_RX_MODE_DIRECT_THROUGH_GPIO = 2, /*!< The packet bits are continuously written to one of the GPIO pins without any processing */
} s2_lp_ic_rx_mode_t;

/**
 * @brief TX mode selection specified in PCKTCTRL1 register
 */
typedef enum {
    S2_LP_TX_MODE_NORMAL              = 0,
    S2_LP_TX_MODE_DIRECT_THROUGH_FIFO = 1, /*!< The packet is written in TX FIFO. The user builds the packet according to their needs including preamble, payload and soon on. The data are transmitted without any processing */
    S2_LP_TX_MODE_DIRECT_THROUGH_GPIO = 2, /*!< The packet bits are continuously read from one of the GPIO pins, properly configured, and transmitted without any processing */
    S2_LP_TX_MODE_PN9                 = 3, /*!< A pseudo-random binary sequence is generated internally. This mode is provided for test purposes only */
} s2_lp_ic_tx_mode_t;

/**
 * @brief This is the FCS type in header field of 802.15.4g packet
 */
typedef enum {
    S2_LP_FCS_TYPE_0 = 0, /*!< CRC mode 5 is selected (32 bit ANSI X3.66-1979) */
    S2_LP_FCS_TYPE_1 = 1, /*!< CRC mode 3 is selected (16 bit CCIT) */
} s2_lp_ic_fcs_type_t;

/**
 * @brief For the 802.15.4g packet format, two different coding schemes can be selected depending on the setting of the FEC_TYPE_4G register
 */
typedef enum {
    S2_LP_FEC_TYPE_NRNSC = 0, /*!< NRNSC encoder is selected */
    S2_LP_FEC_TYPE_RSC   = 1, /*!< RSC encoder is selected*/
} s2_lp_ic_fec_type_t;

/**
 * @brief The following standard CRC polynomials can be selected
 */
typedef enum {
    S2_LP_CRC_MODE_OFF      = 0, /*!< CRC is not used */
    S2_LP_CRC_MODE_ATM_8    = 1, /*!< CRC using poly 0x07 */
    S2_LP_CRC_MODE_IBM_16   = 2, /*!< CRC using poly 0x8005 */
    S2_LP_CRC_MODE_CCITT_16 = 3, /*!< CRC using poly 0x1021 */
    S2_LP_CRC_MODE_RTCM_24  = 4, /*!< CRC using poly 0x864CFB */
    S2_LP_CRC_MODE_ADCCP_32 = 5, /*!< CRC using poly 0x4C11DB7 */
} s2_lp_ic_crc_mode_t;

/**
 * @brief Selectors for the preable binary sequences transmitted in the various modulation modes
 * @note  Set in PREAMBLE_SEL field of the PCKTCTRL3 register
 */
typedef enum {
    S2_LP_PREAMBLE_SEL_0x55      = 0,
    S2_LP_PREAMBLE_SEL_4FSK_0x77 = 0,
    S2_LP_PREAMBLE_SEL_0xAA      = 1,
    S2_LP_PREAMBLE_SEL_4FSK_0x22 = 1,
    S2_LP_PREAMBLE_SEL_0xCC      = 2,
    S2_LP_PREAMBLE_SEL_4FSK_0xDD = 2,
    S2_LP_PREAMBLE_SEL_0x33      = 3,
    S2_LP_PREAMBLE_SEL_4FSK_0x88 = 3,
} s2_lp_ic_preamble_sel_t;

/**
 * @brief  All the possible RX timeout stop conditions enumeration.
 */
typedef enum {
    S2_LP_RX_TIMEOUT_STOP_NO_TIMEOUT                   = 0x00, /*!< The RX timeout never expires and the reception ends at the reception of the packet */
    S2_LP_RX_TIMEOUT_STOP_PQI_ABOVE_THRESHOLD          = 0x01, /*!< Timeout stopped on PQI above threshold */
    S2_LP_RX_TIMEOUT_STOP_SQI_ABOVE_THRESHOLD          = 0x02, /*!< Timeout stopped on SQI above threshold */
    S2_LP_RX_TIMEOUT_STOP_SQI_AND_PQI_ABOVE_THRESHOLD  = 0x03, /*!< Timeout stopped on both SQI and PQI above threshold */
    S2_LP_RX_TIMEOUT_STOP_RSSI_ABOVE_THRESHOLD         = 0x04, /*!< Timeout stopped on RSSI above threshold */
    S2_LP_RX_TIMEOUT_STOP_RSSI_AND_PQI_ABOVE_THRESHOLD = 0x05, /*!< Timeout stopped on both RSSI and PQI above threshold */
    S2_LP_RX_TIMEOUT_STOP_RSSI_AND_SQI_ABOVE_THRESHOLD = 0x06, /*!< Timeout stopped on both RSSI and SQI above threshold */
    S2_LP_RX_TIMEOUT_STOP_ALL_ABOVE_THRESHOLD          = 0x07, /*!< Timeout stopped only if RSSI, SQI and PQI are above threshold */
    S2_LP_RX_TIMEOUT_STOP_TIMEOUT_ALWAYS_STOPPED       = 0x08, /*!< Timeout always stopped (default) */
    S2_LP_RX_TIMEOUT_STOP_SQI_OR_PQI_ABOVE_THRESHOLD   = 0x0B, /*!< Timeout stopped if one between SQI or PQI are above threshold */
    S2_LP_RX_TIMEOUT_STOP_RSSI_OR_PQI_ABOVE_THRESHOLD  = 0x0D, /*!< Timeout stopped if one between RSSI or PQI are above threshold */
    S2_LP_RX_TIMEOUT_STOP_RSSI_OR_SQI_ABOVE_THRESHOLD  = 0x0E, /*!< Timeout stopped if one between RSSI or SQI are above threshold */
    S2_LP_RX_TIMEOUT_STOP_ANY_ABOVE_THRESHOLD          = 0x0F, /*!< Timeout stopped if one among RSSI, SQI or SQI are above threshold */
} s2_lp_ic_rx_timeout_stop_condition_t;

/**
 * @brief  S2-LP Clock Recovery mode enumeration
 */
typedef enum {
    S2_LP_CLKREC_DLL_MODE = 0, /*!< DLL mode */
    S2_LP_CLKREC_PLL_MODE = 1, /*!< PLL mode */
} s2_lp_ic_clk_rec_mode_t;

/**
 * @brief  S2-LP Clock Recovery post-filter length
 */
typedef enum {
    S2_LP_CLKREC_POSTFLT_8_SMBL  = 0, /*!< 8 symbols */
    S2_LP_CLKREC_POSTFLT_16_SMBL = 1, /*!< 16 symbols */
} s2_lp_ic_clk_rec_postflt_t;

/**
 * @brief S2-LP FIR configuration
 */
typedef enum {
    S2_LP_FIR_CFG_FILTERING = 0,
    S2_LP_FIR_CFG_RAMPING   = 1,
    S2_LP_FIR_CFG_SWITCHING = 2,
} s2_lp_ic_fir_cfg_t;

/**
 * @brief S2-LP PA 'degeneration' mode code threshold
 */
typedef enum {
    S2_LP_PA_PA_DEGEN_TRIM_CODE_TH_418 = 0,
    S2_LP_PA_PA_DEGEN_TRIM_CODE_TH_439 = 1,
    S2_LP_PA_PA_DEGEN_TRIM_CODE_TH_465 = 2,
    S2_LP_PA_PA_DEGEN_TRIM_CODE_TH_485 = 3,
} s2_lp_ic_pa_degen_trim_code_th_t;

/**
 * @brief S2-LP XO circuit gain margin (transconductance) configuration
 */
typedef enum {
    S2_LP_XO_GM_13_2 = 0, /*!< Transconductance of 13.2 mS at XO startup */
    S2_LP_XO_GM_18_2 = 1, /*!< Transconductance of 18.2 mS at XO startup */
    S2_LP_XO_GM_21_5 = 2, /*!< Transconductance of 21.5 mS at XO startup */
    S2_LP_XO_GM_25_6 = 3, /*!< Transconductance of 25.6 mS at XO startup */
    S2_LP_XO_GM_28_8 = 4, /*!< Transconductance of 28.8 mS at XO startup */
    S2_LP_XO_GM_33_9 = 5, /*!< Transconductance of 33.9 mS at XO startup */
    S2_LP_XO_GM_38_5 = 6, /*!< Transconductance of 38.5 mS at XO startup */
    S2_LP_XO_GM_43_0 = 7, /*!< Transconductance of 43.0 mS at XO startup */
} s2_lp_ic_xo_gm_t;

/**
 * @brief S2-LP Battery Level Detector threshold
 */
typedef enum {
    S2_LP_BLD_TH_2_7V = 0,
    S2_LP_BLD_TH_2_5V = 1,
    S2_LP_BLD_TH_2_3V = 2,
    S2_LP_BLD_TH_2_1V = 3,
} s2_lp_ic_bld_thershold_t;

/**
 * @brief S2-LP SMPS output level control options
 */
typedef enum {
    S2_LP_SMPS_LVL_MODE_TXRX    = 0, /*!< SMPS output level depends upon the value written in the PM_CONFIG0 register (SET_SMPS_LEVEL field) both in RX and TX state */
    S2_LP_SMPS_LVL_MODE_TX_ONLY = 1, /*!< SMPS output level depends upon the value in PM_CONFIG register just in TX state, while in RX state it is fixed to 1.4 V */
} s2_lp_ic_smps_lvl_mode_t;

/**
 * @brief S2-LP SMPS output voltage
 */
typedef enum {
    S2_LP_SMPS_LVL_RESERVED = 0, /*!< Not used */
    S2_LP_SMPS_LVL_1_2V     = 1, /*!< 1.2 V */
    S2_LP_SMPS_LVL_1_3V     = 2, /*!< 1.3 V */
    S2_LP_SMPS_LVL_1_4V     = 3, /*!< 1.4 V */
    S2_LP_SMPS_LVL_1_5V     = 4, /*!< 1.5 V */
    S2_LP_SMPS_LVL_1_6V     = 5, /*!< 1.6 V */
    S2_LP_SMPS_LVL_1_7V     = 6, /*!< 1.7 V */
    S2_LP_SMPS_LVL_1_8V     = 7, /*!< 1.8 V */
} s2_lp_ic_smps_lvl_t;

/**
 * @brief S2-LP sleep mode options
 */
typedef enum {
    S2_LP_SLEEP_MODE_NO_RETENTION   = 0, /*!< SLEEP without FIFO retention (SLEEP A) */
    S2_LP_SLEEP_MODE_WITH_RETENTION = 1, /*!< SLEEP with FIFO retention (SLEEP B) */
} s2_lp_ic_sleep_mode_t;

/**
 * @brief S2-LP PA 'degeneration' mode clamp voltage
 */
typedef enum {
    S2_LP_PA_PA_DEGEN_TRIM_CLAMP_V_400 = 0, /*!< 0.40 V */
    S2_LP_PA_PA_DEGEN_TRIM_CLAMP_V_450 = 1, /*!< 0.44 V */
    S2_LP_PA_PA_DEGEN_TRIM_CLAMP_V_500 = 2, /*!< 0.50 V */
    S2_LP_PA_PA_DEGEN_TRIM_CLAMP_V_550 = 3, /*!< 0.55 V */
} s2_lp_ic_pa_degen_trim_clamp_v_t;

/**
 * @brief S2-LP PA bessel filter bandwidth
 */
typedef enum {
    S2_LP_PA_FILT_BW_12_5_KHZ = 0, /*!< 12.5 kHz (data rate 16.2 kbps) */
    S2_LP_PA_FILT_BW_25_KHZ   = 1, /*!< 25 kHz (data rate 32 kbps) */
    S2_LP_PA_FILT_BW_50_KHZ   = 2, /*!< 50 kHz (data rate 62.5 kbps) */
    S2_LP_PA_FILT_BW_100_KHZ  = 3, /*!< 100 kHz (data rate 125 kbps) */
} s2_lp_ic_pa_fc_t;

/**
 * @brief S2-LP AFC operating mode
 */
typedef enum {
    S2_LP_AFC_MODE_CLOSE_ON_SLICER = 0, /*!< AFC loop closed on slicer */
    S2_LP_AFC_MODE_CLOSE_ON_2_IF   = 1, /*!< AFC loop closed on second conversion stage */
} s2_lp_ic_afc_mode_t;

/**
 * @brief S2-LP Carrier Sense mode
 */
typedef enum {
    S2_LP_CS_MODE_STATIC       = 0u, /*!< Static CS */
    S2_LP_CS_MODE_DYNAMIC_6DB  = 1u, /*!< Dynamic CS with 6dB dynamic threshold */
    S2_LP_CS_MODE_DYNAMIC_12DB = 2u, /*!< Dynamic CS with 12dB dynamic threshold */
    S2_LP_CS_MODE_DYNAMIC_18DB = 3u, /*!< Dynamic CS with 18dB dynamic threshold */
} s2_lp_ic_cs_mode_t;

/**
 * @brief S2-LP FIFO almost full/empty IRQ MUX settings
 */
typedef enum {
    S2_LP_FIFO_GPIO_OUT_MUX_SEL_TX = 0, /*!< Select the almost empty/full control for TX FIFO */
    S2_LP_FIFO_GPIO_OUT_MUX_SEL_RX = 1, /*!< Select the almost empty/full control for RX FIFO */
} s2_lp_ic_fifo_irq_mux_sel_t;

/**
 * @brief GPIOx_CONF register definition
 * @note  Applicable to GPIO0_CONF, GPIO1_CONF, GPIO2_CONF, and GPIO3_CONF registers
 */
typedef __PACKED_UNION {
    __PACKED_STRUCT {
#if SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN
        s2_lp_ic_gpio_signal_t GPIOx_SELECT: 5; /*!< Specify the GPIOx I/O signal */
        uint8_t                            : 1;
        s2_lp_ic_gpio_mode_t   GPIOx_MODE  : 2; /*!< GPIOx Mode (e.g. Hi-Z, input, output, etc.) */
#else
        s2_lp_ic_gpio_mode_t   GPIOx_MODE  : 2; /*!< GPIOx Mode (e.g. Hi-Z, input, output, etc.) */
        uint8_t                            : 1;
        s2_lp_ic_gpio_signal_t GPIOx_SELECT: 5; /*!< Specify the GPIOx I/O signal */
#endif /* SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN */
    };
    uint8_t raw;
} sl2_lp_ic_reg_gpiox_conf_t;
static_assert(sizeof(sl2_lp_ic_reg_gpiox_conf_t) == sizeof(uint8_t));

/**
 * @brief SYNT3 register definition
 */
typedef __PACKED_UNION {
    __PACKED_STRUCT {
#if SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN
        uint8_t PLL_CP_ISEL: 3; /*!< Set the charge pump current according to the XTAL frequency */
        uint8_t BS         : 1; /*!< Synthesizer band select. This parameter selects the out-of loop divide factor of the synthesizer: 0: 4, band select factor for high band; 1: 8, band select factor for middle band */
        uint8_t SYNT_27_24 : 4; /*!< MSB bits of the PLL programmable divider */
#else
        uint8_t SYNT_27_24 : 4; /*!< MSB bits of the PLL programmable divider */
        uint8_t BS         : 1; /*!< Synthesizer band select. This parameter selects the out-of loop divide factor of the synthesizer: 0: 4, band select factor for high band; 1: 8, band select factor for middle band */
        uint8_t PLL_CP_ISEL: 3; /*!< Set the charge pump current according to the XTAL frequency */
#endif /* SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN */
    };
    uint8_t raw;
} sl2_lp_ic_reg_synt3_t;
static_assert(sizeof(sl2_lp_ic_reg_synt3_t) == sizeof(uint8_t));

/**
 * @brief SYNT2 register definition
 */
typedef __PACKED_UNION {
    uint8_t SYNT_23_16; /*!< Intermediate bits of the PLL programmable divider */
    uint8_t raw;
} sl2_lp_ic_reg_synt2_t;
static_assert(sizeof(sl2_lp_ic_reg_synt2_t) == sizeof(uint8_t));

/**
 * @brief SYNT1 register definition
 */
typedef __PACKED_UNION {
    uint8_t SYNT_15_8; /*!< Intermediate bits of the PLL programmable divider */
    uint8_t raw;
} sl2_lp_ic_reg_synt1_t;
static_assert(sizeof(sl2_lp_ic_reg_synt1_t) == sizeof(uint8_t));

/**
 * @brief SYNT0 register definition
 */
typedef __PACKED_UNION {
    uint8_t SYNT_7_0; /*!< LSB bits of the PLL programmable divider */
    uint8_t raw;
} sl2_lp_ic_reg_synt0_t;
static_assert(sizeof(sl2_lp_ic_reg_synt0_t) == sizeof(uint8_t));

/**
 * @brief Aggregator for all SYNT registers
 */
typedef __PACKED_UNION {
    __PACKED_STRUCT {
        sl2_lp_ic_reg_synt3_t synt3;
        sl2_lp_ic_reg_synt2_t synt2;
        sl2_lp_ic_reg_synt1_t synt1;
        sl2_lp_ic_reg_synt0_t synt0;
    };
    uint8_t raw[4];
} s2_lp_ic_reg_synt_t;
static_assert(sizeof(s2_lp_ic_reg_synt_t) == 4u * sizeof(uint8_t));

/**
 * @brief CHSPACE register definition
 */
typedef __PACKED_UNION {
    uint8_t CH_SPACE; /*!< Channel spacing setting */
    uint8_t raw;
} sl2_lp_ic_reg_chspace_t;
static_assert(sizeof(sl2_lp_ic_reg_chspace_t) == sizeof(uint8_t));

/**
 * @brief CHNUM register definition
 */
typedef __PACKED_UNION {
    uint8_t CH_NUM; /*!< Channel number. This value is multiplied by the channel spacing and added to the synthesizer base frequency to generate the actual RF carrier frequency */
    uint8_t raw;
} sl2_lp_ic_reg_chnum_t;
static_assert(sizeof(sl2_lp_ic_reg_chnum_t) == sizeof(uint8_t));

/**
 * @brief MOD4 register definition
 */
typedef __PACKED_UNION {
    uint8_t DATARATE_M_15_8; /*!< The MSB of the mantissa value of the data rate equation */
    uint8_t raw;
} sl2_lp_ic_reg_mod4_t;
static_assert(sizeof(sl2_lp_ic_reg_mod4_t) == sizeof(uint8_t));

/**
 * @brief MOD3 register definition
 */
typedef __PACKED_UNION {
    uint8_t DATARATE_M_7_0; /*!< The LSB of the mantissa value of the data rate equation */
    uint8_t raw;
} sl2_lp_ic_reg_mod3_t;
static_assert(sizeof(sl2_lp_ic_reg_mod3_t) == sizeof(uint8_t));

/**
 * @brief MOD2 register definition
 */
typedef __PACKED_UNION {
    __PACKED_STRUCT {
#if SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN
        s2_lp_ic_mod_types_t MOD_TYPE  : 4; /*!< The MSB of the mantissa value of the data rate equation */
        uint8_t              DATARATE_E: 4; /*!< The exponent value of the data rate equation */
#else
        uint8_t              DATARATE_E: 4; /*!< The exponent value of the data rate equation */
        s2_lp_ic_mod_types_t MOD_TYPE  : 4; /*!< The MSB of the mantissa value of the data rate equation */
#endif /* SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN */
    };
    uint8_t raw;
} sl2_lp_ic_reg_mod2_t;
static_assert(sizeof(sl2_lp_ic_reg_mod2_t) == sizeof(uint8_t));

/**
 * @brief MOD1 register definition
 */
typedef __PACKED_UNION {
    __PACKED_STRUCT {
#if SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN
        uint8_t PA_INTERP_EN : 1; /*!< 1: enable the PA power interpolator */
        uint8_t MOD_INTERP_EN: 1; /*!< 1: enable frequency interpolator for the GFSK shaping */
        uint8_t CONST_MAP    : 2; /*!< Select the constellation map for 4-(G)FSK or 2-(G)FSK modulations */
        uint8_t FDEV_E       : 4; /*!< The exponent value of the frequency deviation equation */
#else
        uint8_t FDEV_E       : 4; /*!< The exponent value of the frequency deviation equation */
        uint8_t CONST_MAP    : 2; /*!< Select the constellation map for 4-(G)FSK or 2-(G)FSK modulations */
        uint8_t MOD_INTERP_EN: 1; /*!< 1: enable frequency interpolator for the GFSK shaping */
        uint8_t PA_INTERP_EN : 1; /*!< 1: enable the PA power interpolator */
#endif /* SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN */
    };
    uint8_t raw;
} sl2_lp_ic_reg_mod1_t;
static_assert(sizeof(sl2_lp_ic_reg_mod1_t) == sizeof(uint8_t));

/**
 * @brief MOD0 register definition
 */
typedef __PACKED_UNION {
    uint8_t FDEV_M; /*!< The mantissa value of the frequency deviation equation */
    uint8_t raw;
} sl2_lp_ic_reg_mod0_t;
static_assert(sizeof(sl2_lp_ic_reg_mod0_t) == sizeof(uint8_t));

/**
 * @brief CHFLT register definition
 */
typedef __PACKED_UNION {
    __PACKED_STRUCT {
#if SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN
        uint8_t CHFLT_M: 4; /*!< The mantissa value of the receiver channel filter */
        uint8_t CHFLT_E: 4; /*!< The exponent value of the receiver channel filter */
#else
        uint8_t CHFLT_E: 4; /*!< The exponent value of the receiver channel filter */
        uint8_t CHFLT_M: 4; /*!< The mantissa value of the receiver channel filter */
#endif /* SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN */
    };
    uint8_t raw;
} sl2_lp_ic_reg_chflt_t;
static_assert(sizeof(sl2_lp_ic_reg_chflt_t) == sizeof(uint8_t));

/**
 * @brief Aggregator for modulation params registers
 */
typedef __PACKED_UNION {
    __PACKED_STRUCT {
        sl2_lp_ic_reg_mod4_t  mod4;
        sl2_lp_ic_reg_mod3_t  mod3;
        sl2_lp_ic_reg_mod2_t  mod2;
        sl2_lp_ic_reg_mod1_t  mod1;
        sl2_lp_ic_reg_mod0_t  mod0;
        sl2_lp_ic_reg_chflt_t chflt;
    };
    uint8_t raw[6];
} sl2_lp_ic_reg_mod_params_t;
static_assert(sizeof(sl2_lp_ic_reg_mod_params_t) == 6u * sizeof(uint8_t));

/**
 * @brief AFC2 register definition
 */
typedef __PACKED_UNION {
    __PACKED_STRUCT {
#if SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN
        uint8_t             AFC_FREEZE_ON_SYNC: 1; /*!< 1: enable the freeze AFC correction upon sync word detection */
        uint8_t             AFC_ENABLED       : 1; /*!< 1: enable the AFC correction */
        s2_lp_ic_afc_mode_t AFC_MODE          : 1; /*!< 0: AFC loop closed on slicer; 1: AFC loop closed on second conversion stage */
        uint8_t                               : 5;
#else
        uint8_t                               : 5;
        s2_lp_ic_afc_mode_t AFC_MODE          : 1; /*!< 0: AFC loop closed on slicer; 1: AFC loop closed on second conversion stage */
        uint8_t             AFC_ENABLED       : 1; /*!< 1: enable the AFC correction */
        uint8_t             AFC_FREEZE_ON_SYNC: 1; /*!< 1: enable the freeze AFC correction upon sync word detection */
#endif /* SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN */
    };
    uint8_t raw;
} sl2_lp_ic_reg_afc2_t;
static_assert(sizeof(sl2_lp_ic_reg_afc2_t) == sizeof(uint8_t));

/**
 * @brief AFC1 register definition
 */
typedef __PACKED_UNION {
    uint8_t AFC_FAST_PERIOD; /*!< The length of the AFC fast period */
    uint8_t raw;
} sl2_lp_ic_reg_afc1_t;
static_assert(sizeof(sl2_lp_ic_reg_afc1_t) == sizeof(uint8_t));

/**
 * @brief AFC0 register definition
 */
typedef __PACKED_UNION {
    __PACKED_STRUCT {
#if SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN
        uint8_t AFC_FAST_GAIN: 4; /*!< The AFC loop gain in fast mode (2's log) */
        uint8_t AFC_SLOW_GAIN: 4; /*!< The AFC loop gain in slow mode (2's log) */
#else
        uint8_t AFC_SLOW_GAIN: 4; /*!< The AFC loop gain in slow mode (2's log) */
        uint8_t AFC_FAST_GAIN: 4; /*!< The AFC loop gain in fast mode (2's log) */
#endif /* SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN */
    };
    uint8_t raw;
} sl2_lp_ic_reg_afc0_t;
static_assert(sizeof(sl2_lp_ic_reg_afc0_t) == sizeof(uint8_t));

/**
 * @brief Aggregator for AFC registers
 */
typedef __PACKED_UNION {
    __PACKED_STRUCT {
        sl2_lp_ic_reg_afc2_t afc2;
        sl2_lp_ic_reg_afc1_t afc1;
        sl2_lp_ic_reg_afc0_t afc0;
    };
    uint8_t raw[3];
} sl2_lp_ic_reg_afc_t;
static_assert(sizeof(sl2_lp_ic_reg_afc_t) == 3u * sizeof(uint8_t));

/**
 * @brief RSSI_FLT register definition
 */
typedef __PACKED_UNION {
    __PACKED_STRUCT {
#if SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN
        uint8_t            RSSI_FLT: 4; /*!< Gain of the RSSI filter */
        s2_lp_ic_cs_mode_t CS_MODE : 2; /*!< Carrier sense mode */
        uint8_t                    : 2;
#else
        uint8_t                    : 2;
        s2_lp_ic_cs_mode_t CS_MODE : 2; /*!< Carrier sense mode */
        uint8_t            RSSI_FLT: 4; /*!< Gain of the RSSI filter */
#endif /* SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN */
    };
    uint8_t raw;
} sl2_lp_ic_reg_rssi_flt_t;
static_assert(sizeof(sl2_lp_ic_reg_rssi_flt_t) == sizeof(uint8_t));

/**
 * @brief RSSI_TH register definition
 */
typedef __PACKED_UNION {
    uint8_t RSSI_TH;
    uint8_t raw;
} sl2_lp_ic_reg_rssi_th_t;
static_assert(sizeof(sl2_lp_ic_reg_rssi_th_t) == sizeof(uint8_t));

/**
 * @brief AGCCTRL4 register definition
 */
typedef __PACKED_UNION {
    __PACKED_STRUCT {
#if SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN
        uint8_t LOW_THRESHOLD_0: 4; /*!< Low threshold 0 for the AGC */
        uint8_t LOW_THRESHOLD_1: 4; /*!< Low threshold 1 for the AGC */
#else
        uint8_t LOW_THRESHOLD_1: 4; /*!< Low threshold 1 for the AGC */
        uint8_t LOW_THRESHOLD_0: 4; /*!< Low threshold 0 for the AGC */
#endif /* SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN */
    };
    uint8_t raw;
} sl2_lp_ic_reg_agcctrl4_t;
static_assert(sizeof(sl2_lp_ic_reg_agcctrl4_t) == sizeof(uint8_t));

/**
 * @brief AGCCTRL3 register definition
 */
typedef __PACKED_UNION {
    uint8_t LOW_THRESHOLD_SEL; /*!< Low threshold selection (defined in the AGCCTRL4). Bitmask for each attenuation step */
    uint8_t raw;
} sl2_lp_ic_reg_agcctrl3_t;
static_assert(sizeof(sl2_lp_ic_reg_agcctrl3_t) == sizeof(uint8_t));

/**
 * @brief AGCCTRL2 register definition
 */
typedef __PACKED_UNION {
    __PACKED_STRUCT {
#if SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN
        uint8_t               : 2;
        uint8_t FREEZE_ON_SYNC: 1; /*!< Enable the AGC algorithm to be frozen on SYNC */
        uint8_t               : 1;
        uint8_t MEAS_TIME     : 4; /*!< AGC measurement time */
#else
        uint8_t MEAS_TIME     : 4; /*!< AGC measurement time */
        uint8_t               : 1;
        uint8_t FREEZE_ON_SYNC: 1; /*!< Enable the AGC algorithm to be frozen on SYNC */
        uint8_t               : 2;
#endif /* SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN */
    };
    uint8_t raw;
} sl2_lp_ic_reg_agcctrl2_t;
static_assert(sizeof(sl2_lp_ic_reg_agcctrl2_t) == sizeof(uint8_t));

/**
 * @brief AGCCTRL1 register definition
 */
typedef __PACKED_UNION {
    __PACKED_STRUCT {
#if SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN
        uint8_t HIGH_THRESHOLD: 4; /*!< High threshold for the AGC */
        uint8_t               : 4;
#else
        uint8_t               : 4;
        uint8_t HIGH_THRESHOLD: 4; /*!< High threshold for the AGC */
#endif /* SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN */
    };
    uint8_t raw;
} sl2_lp_ic_reg_agcctrl1_t;
static_assert(sizeof(sl2_lp_ic_reg_agcctrl1_t) == sizeof(uint8_t));

/**
 * @brief AGCCTRL0 register definition
 */
typedef __PACKED_UNION {
    __PACKED_STRUCT {
#if SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN
        uint8_t AGC_ENABLE: 1; /*!< 0: disabled; 1: enabled */
        uint8_t           : 1;
        uint8_t HOLD_TIME : 6; /*!< Hold time for after gain adjustment for the AGC */
#else
        uint8_t HOLD_TIME : 6; /*!< Hold time for after gain adjustment for the AGC */
        uint8_t           : 1;
        uint8_t AGC_ENABLE: 1; /*!< 0: disabled; 1: enabled */
#endif /* SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN */
    };
    uint8_t raw;
} sl2_lp_ic_reg_agcctrl0_t;
static_assert(sizeof(sl2_lp_ic_reg_agcctrl0_t) == sizeof(uint8_t));

/**
 * @brief Aggregator for AGCCTRL registers
 */
typedef __PACKED_UNION {
    __PACKED_STRUCT {
        sl2_lp_ic_reg_agcctrl4_t agcctrl4;
        sl2_lp_ic_reg_agcctrl3_t agcctrl3;
        sl2_lp_ic_reg_agcctrl2_t agcctrl2;
        sl2_lp_ic_reg_agcctrl1_t agcctrl1;
        sl2_lp_ic_reg_agcctrl0_t agcctrl0;
    };
    uint8_t raw[5];
} sl2_lp_ic_reg_agcctrl_t;
static_assert(sizeof(sl2_lp_ic_reg_agcctrl_t) == 5u * sizeof(uint8_t));

/**
 * @brief CLOCKREC2 register definition
 */
typedef __PACKED_UNION {
    __PACKED_STRUCT {
#if SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN
        uint8_t                 CLK_REC_P_GAIN_SLOW: 3; /*!< Clock recovery slow loop gain (log2) */
        s2_lp_ic_clk_rec_mode_t CLK_REC_ALGO_SEL   : 1; /*!< Select the symbol timing recovery algorithm:  0 - DLL, 1 - PLL */
        uint8_t                 CLK_REC_I_GAIN_SLOW: 4; /*!< Set the integral slow gain for symbol timing recovery (PLL mode only) */
#else
        uint8_t                 CLK_REC_I_GAIN_SLOW: 4; /*!< Set the integral slow gain for symbol timing recovery (PLL mode only) */
        s2_lp_ic_clk_rec_mode_t CLK_REC_ALGO_SEL   : 1; /*!< Select the symbol timing recovery algorithm:  0 - DLL, 1 - PLL */
        uint8_t                 CLK_REC_P_GAIN_SLOW: 3; /*!< Clock recovery slow loop gain (log2) */
#endif /* SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN */
    };
    uint8_t raw;
} sl2_lp_ic_reg_clockrec2_t;
static_assert(sizeof(sl2_lp_ic_reg_clockrec2_t) == sizeof(uint8_t));

/**
 * @brief CLOCKREC1 register definition
 */
typedef __PACKED_UNION {
    __PACKED_STRUCT {
#if SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN
        uint8_t                    CLK_REC_P_GAIN_FAST: 3; /*!< Clock recovery fast loop gain (log2) */
        s2_lp_ic_clk_rec_postflt_t PSTFLT_LEN         : 1; /*!< Select the post filter length: 0 - 8 symbols, 1 - 16 symbols */
        uint8_t                    CLK_REC_I_GAIN_FAST: 4; /*!< Set the integral fast gain for symbol timing recovery (PLL mode only) */
#else
        uint8_t                    CLK_REC_I_GAIN_FAST: 4; /*!< Set the integral fast gain for symbol timing recovery (PLL mode only) */
        s2_lp_ic_clk_rec_postflt_t PSTFLT_LEN         : 1; /*!< Select the post filter length: 0 - 8 symbols, 1 - 16 symbols */
        uint8_t                    CLK_REC_P_GAIN_FAST: 3; /*!< Clock recovery fast loop gain (log2) */
#endif /* SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN */
    };
    uint8_t raw;
} sl2_lp_ic_reg_clockrec1_t;
static_assert(sizeof(sl2_lp_ic_reg_clockrec1_t) == sizeof(uint8_t));

/**
 * @brief Aggregator for Clock Recovery registers
 */
typedef __PACKED_UNION {
    __PACKED_STRUCT {
        sl2_lp_ic_reg_clockrec2_t clockrec2;
        sl2_lp_ic_reg_clockrec1_t clockrec1;
    };
    uint8_t raw[2];
} sl2_lp_ic_reg_clockrec_t;
static_assert(sizeof(sl2_lp_ic_reg_clockrec_t) == 2u * sizeof(uint8_t));

/**
 * @brief PCKTCTRL6 register definition
 */
typedef __PACKED_UNION {
    __PACKED_STRUCT {
#if SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN
        uint8_t SYNC_LEN        : 6; /*!< The number of bits used for the SYNC field in the packet */
        uint8_t PREAMBLE_LEN_9_8: 2; /*!< The MSB of the number of '01 or '10' of the preamble of the packet */
#else
        uint8_t PREAMBLE_LEN_9_8: 2; /*!< The MSB of the number of '01 or '10' of the preamble of the packet */
        uint8_t SYNC_LEN        : 6; /*!< The number of bits used for the SYNC field in the packet */
#endif /* SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN */
    };
    uint8_t raw;
} sl2_lp_ic_reg_pcktctrl6_t;
static_assert(sizeof(sl2_lp_ic_reg_pcktctrl6_t) == sizeof(uint8_t));

/**
 * @brief PCKTCTRL5 register definition
 */
typedef __PACKED_UNION {
    uint8_t PREAMBLE_LEN_7_0; /*!< The MSB of the number of '01 or '10' of the preamble of the packet */
    uint8_t raw;
} sl2_lp_ic_reg_pcktctrl5_t;
static_assert(sizeof(sl2_lp_ic_reg_pcktctrl5_t) == sizeof(uint8_t));

/**
 * @brief PCKTCTRL4 register definition
 */
typedef __PACKED_UNION {
    __PACKED_STRUCT {
#if SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN
        s2_lp_ic_len_wid_t LEN_WID    : 1; /*!< The number of bytes used for the length field: 0: 1 byte; 1: 2 bytes */
        uint8_t                       : 3;
        uint8_t            ADDRESS_LEN: 1; /*!< 1: include the ADDRESS field in the packet */
        uint8_t                       : 3;
#else
        uint8_t                       : 3;
        uint8_t            ADDRESS_LEN: 1; /*!< 1: include the ADDRESS field in the packet */
        uint8_t                       : 3;
        s2_lp_ic_len_wid_t LEN_WID    : 1; /*!< The number of bytes used for the length field: 0: 1 byte; 1: 2 bytes */
#endif /* SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN */
    };
    uint8_t raw;
} sl2_lp_ic_reg_pcktctrl4_t;
static_assert(sizeof(sl2_lp_ic_reg_pcktctrl4_t) == sizeof(uint8_t));

/**
 * @brief PCKTCTRL3 register definition
 */
typedef __PACKED_UNION {
    __PACKED_STRUCT {
#if SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN
        s2_lp_ic_pckt_frmt_t    PCKT_FRMT    : 2;
        s2_lp_ic_rx_mode_t      RX_MODE      : 2;
        uint8_t                 FSK4_SYM_SWAP: 1; /*!< When 4FSK_SYM_SWAP = 0: <b7b6><b5b4><b3b2><b1b0>; when 4FSK_SYM_SWAP = 1: <b6b7><b4b5><b2b3><b0b1> */
        uint8_t                 BYTE_SWAP    : 1; /*!< Select the transmission order: 0: MSB first; 1: LSB first */
        s2_lp_ic_preamble_sel_t PREAMBLE_SEL : 2;
#else
        s2_lp_ic_preamble_sel_t PREAMBLE_SEL : 2;
        uint8_t                 BYTE_SWAP    : 1; /*!< Select the transmission order: 0: MSB first; 1: LSB first */
        uint8_t                 FSK4_SYM_SWAP: 1; /*!< When 4FSK_SYM_SWAP = 0: <b7b6><b5b4><b3b2><b1b0>; when 4FSK_SYM_SWAP = 1: <b6b7><b4b5><b2b3><b0b1> */
        s2_lp_ic_rx_mode_t      RX_MODE      : 2;
        s2_lp_ic_pckt_frmt_t    PCKT_FRMT    : 2;
#endif /* SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN */
    };
    uint8_t raw;
} sl2_lp_ic_reg_pcktctrl3_t;
static_assert(sizeof(sl2_lp_ic_reg_pcktctrl3_t) == sizeof(uint8_t));

/**
 * @brief PCKTCTRL2 register definition
 */
typedef __PACKED_UNION {
#if SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN
    __PACKED_STRUCT {
        uint8_t                          : 2;
        s2_lp_ic_fcs_type_t FCS_TYPE_4G  : 1;
        s2_lp_ic_fec_type_t FEC_TYPE_4G  : 1;
        uint8_t             INT_EN_4G    : 1; /*!< 1: enable the interleaving of 802.15.4g packet */
        uint8_t             MBUS_3OF6_EN : 1;
        uint8_t             MANCHESTER_EN: 1;
        uint8_t             FIX_VAR_LEN  : 1; /*!< 0: fixed; 1: variable */
    };
    __PACKED_STRUCT {
        uint8_t                          : 3;
        uint8_t             STOP_BIT     : 1; /*!< The value of the stop bit for the UART packet format */
        uint8_t             START_BIT    : 1; /*!< The value of the start bit for the UART packet format */
        uint8_t                          : 3;
    };
#else
    __PACKED_STRUCT {
        uint8_t             FIX_VAR_LEN  : 1; /*!< 0: fixed; 1: variable */
        uint8_t             MANCHESTER_EN: 1;
        uint8_t             MBUS_3OF6_EN : 1;
        uint8_t             INT_EN_4G    : 1; /*!< 1: enable the interleaving of 802.15.4g packet */
        s2_lp_ic_fec_type_t FEC_TYPE_4G  : 1;
        s2_lp_ic_fcs_type_t FCS_TYPE_4G  : 1;
        uint8_t                          : 2;
    };
    __PACKED_STRUCT {
        uint8_t                          : 3;
        uint8_t             START_BIT    : 1; /*!< The value of the start bit for the UART packet format */
        uint8_t             STOP_BIT     : 1; /*!< The value of the stop bit for the UART packet format */
        uint8_t                          : 3;
    };
#endif /* SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN */
    uint8_t raw;
} sl2_lp_ic_reg_pcktctrl2_t;
static_assert(sizeof(sl2_lp_ic_reg_pcktctrl2_t) == sizeof(uint8_t));

/**
 * @brief PCKTCTRL1 register definition
 */
typedef __PACKED_UNION {
    __PACKED_STRUCT {
#if SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN
        s2_lp_ic_crc_mode_t CRC_MODE       : 3;
        uint8_t             WHIT_EN        : 1; /*!< 1: enable the data whitening mode */
        s2_lp_ic_tx_mode_t  TXSOURCE       : 2;
        uint8_t             SECOND_SYNC_SEL: 1; /*!< In TX mode: 0 select the primary SYNC word, 1 select the secondary SYNC word. In RX mode, if 1 enable the dual SYNC word detection mode */
        uint8_t             FEC_EN         : 1; /*!< 1: enable the FEC encoding in TX or the Viterbi decoding in RX */
#else
        uint8_t             FEC_EN         : 1; /*!< 1: enable the FEC encoding in TX or the Viterbi decoding in RX */
        uint8_t             SECOND_SYNC_SEL: 1; /*!< In TX mode: 0 select the primary SYNC word, 1 select the secondary SYNC word. In RX mode, if 1 enable the dual SYNC word detection mode */
        s2_lp_ic_tx_mode_t  TXSOURCE       : 2;
        uint8_t             WHIT_EN        : 1; /*!< 1: enable the data whitening mode */
        s2_lp_ic_crc_mode_t CRC_MODE       : 3;
#endif /* SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN */
    };
    uint8_t raw;
} sl2_lp_ic_reg_pcktctrl1_t;
static_assert(sizeof(sl2_lp_ic_reg_pcktctrl1_t) == sizeof(uint8_t));

/**
 * @brief PCKTLEN1 register definition
 */
typedef __PACKED_UNION {
    uint8_t PCKTLEN_15_8; /*!< MSB of length of packet in bytes */
    uint8_t raw;
} sl2_lp_ic_reg_pcktlen1_t;
static_assert(sizeof(sl2_lp_ic_reg_pcktlen1_t) == sizeof(uint8_t));

/**
 * @brief PCKTLEN0 register definition
 */
typedef __PACKED_UNION {
    uint8_t PCKTLEN_7_0; /*!< LSB of length of packet in bytes */
    uint8_t raw;
} sl2_lp_ic_reg_pcktlen0_t;
static_assert(sizeof(sl2_lp_ic_reg_pcktlen0_t) == sizeof(uint8_t));

/**
 * @brief Aggregator for packet params registers
 */
typedef __PACKED_UNION {
    __PACKED_STRUCT {
        sl2_lp_ic_reg_pcktctrl6_t pcktctrl6;
        sl2_lp_ic_reg_pcktctrl5_t pcktctrl5;
        sl2_lp_ic_reg_pcktctrl4_t pcktctrl4;
        sl2_lp_ic_reg_pcktctrl3_t pcktctrl3;
        sl2_lp_ic_reg_pcktctrl2_t pcktctrl2;
        sl2_lp_ic_reg_pcktctrl1_t pcktctrl1;
        sl2_lp_ic_reg_pcktlen1_t pcktlen1;
        sl2_lp_ic_reg_pcktlen0_t pcktlen0;
    };
    uint8_t raw[8];
} sl2_lp_ic_reg_pckt_params_t;
static_assert(sizeof(sl2_lp_ic_reg_pckt_params_t) == 8u * sizeof(uint8_t));

/**
 * @brief SYNC3 register definition
 */
typedef __PACKED_UNION {
    uint8_t SYNC3; /*!< SYNC word byte 3 */
    uint8_t raw;
} sl2_lp_ic_reg_sync3_t;
static_assert(sizeof(sl2_lp_ic_reg_sync3_t) == sizeof(uint8_t));

/**
 * @brief SYNC2 register definition
 */
typedef __PACKED_UNION {
    uint8_t SYNC2; /*!< SYNC word byte 2 */
    uint8_t raw;
} sl2_lp_ic_reg_sync2_t;
static_assert(sizeof(sl2_lp_ic_reg_sync2_t) == sizeof(uint8_t));

/**
 * @brief SYNC1 register definition
 */
typedef __PACKED_UNION {
    uint8_t SYNC1; /*!< SYNC word byte 1 */
    uint8_t raw;
} sl2_lp_ic_reg_sync1_t;
static_assert(sizeof(sl2_lp_ic_reg_sync1_t) == sizeof(uint8_t));

/**
 * @brief SYNC0 register definition
 */
typedef __PACKED_UNION {
    uint8_t SYNC0; /*!< SYNC word byte 0 */
    uint8_t raw;
} sl2_lp_ic_reg_sync0_t;
static_assert(sizeof(sl2_lp_ic_reg_sync0_t) == sizeof(uint8_t));

/**
 * @brief Aggregator for SYNCx registers
 */
typedef __PACKED_UNION {
    __PACKED_STRUCT {
        sl2_lp_ic_reg_sync3_t sync3;
        sl2_lp_ic_reg_sync2_t sync2;
        sl2_lp_ic_reg_sync1_t sync1;
        sl2_lp_ic_reg_sync0_t sync0;
    };
    uint8_t raw[4];
} sl2_lp_ic_reg_sync_t;
static_assert(sizeof(sl2_lp_ic_reg_sync_t) == 4u * sizeof(uint8_t));

/**
 * @brief QI register definition
 */
typedef __PACKED_UNION {
    __PACKED_STRUCT {
#if SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN
        uint8_t SQI_TH: 3; /*!< SQI threshold */
        uint8_t PQI_TH: 4; /*!< PQI threshold */
        uint8_t SQI_EN: 1; /*!< 1: enable the SQI check */
#else
        uint8_t SQI_EN: 1; /*!< 1: enable the SQI check */
        uint8_t PQI_TH: 4; /*!< PQI threshold */
        uint8_t SQI_TH: 3; /*!< SQI threshold */
#endif /* SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN */
    };
    uint8_t raw;
} sl2_lp_ic_reg_qi_t;
static_assert(sizeof(sl2_lp_ic_reg_qi_t) == sizeof(uint8_t));

/**
 * @brief PROTOCOL2 register definition
 */
typedef __PACKED_UNION {
    __PACKED_STRUCT {
#if SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN
        uint8_t                     CS_TIMEOUT_MASK      : 1; /*!< 1: enable the CS value contributes to timeout disabling */
        uint8_t                     SQI_TIMEOUT_MASK     : 1; /*!< 1: enable the SQI value contributes to timeout disabling */
        uint8_t                     PQI_TIMEOUT_MASK     : 1; /*!< 1: enable the PQI value contributes to timeout disabling */
        uint8_t                     TX_SEQ_NUM_RELOAD    : 2; /*!< TX sequence number to be used when counting reset is required using the related command */
        s2_lp_ic_fifo_irq_mux_sel_t FIFO_GPIO_OUT_MUX_SEL: 1; /*!< 0: select the almost empty/full control for TX FIFO; 1: select the almost empty/full control for RX FIFO */
        uint8_t                     LDC_TIMER_MULT       : 2; /*!< Set the LDC timer multiplier factor (exponent of pow2) */
#else
        uint8_t                     LDC_TIMER_MULT       : 2; /*!< Set the LDC timer multiplier factor (exponent of pow2) */
        s2_lp_ic_fifo_irq_mux_sel_t FIFO_GPIO_OUT_MUX_SEL: 1; /*!< 0: select the almost empty/full control for TX FIFO; 1: select the almost empty/full control for RX FIFO */
        uint8_t                     TX_SEQ_NUM_RELOAD    : 2; /*!< TX sequence number to be used when counting reset is required using the related command */
        uint8_t                     PQI_TIMEOUT_MASK     : 1; /*!< 1: enable the PQI value contributes to timeout disabling */
        uint8_t                     SQI_TIMEOUT_MASK     : 1; /*!< 1: enable the SQI value contributes to timeout disabling */
        uint8_t                     CS_TIMEOUT_MASK      : 1; /*!< 1: enable the CS value contributes to timeout disabling */
#endif /* SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN */
    };
    uint8_t raw;
} sl2_lp_ic_reg_protocol2_t;
static_assert(sizeof(sl2_lp_ic_reg_protocol2_t) == sizeof(uint8_t));

/**
 * @brief PROTOCOL1 register definition
 */
typedef __PACKED_UNION {
    __PACKED_STRUCT {
#if SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN
        uint8_t LDC_MODE          : 1; /*!< 1: enable the Low Duty Cycle mode */
        uint8_t LDC_RELOAD_ON_SYNC: 1; /*!< 1: enable the LDC timer reload mode */
        uint8_t PIGGYBACKING      : 1; /*!< 1: enable the piggybacking */
        uint8_t FAST_CS_TERM_EN   : 1; /*!< 1: enable the RX sniff timer */
        uint8_t SEED_RELOAD       : 1; /*!< 1: enable the reload of the back-off random generator seed using the value written in the BU_COUNTER_SEED */
        uint8_t CSMA_ON           : 1; /*!< 1: enable the CSMA channel access mode */
        uint8_t CSMA_PERS_ON      : 1; /*!< 1: enable the CSMA persistent mode (no back-off cycles) */
        uint8_t AUTO_PCKT_FLT     : 1; /*!< 1: enable the automatic packet filtering control */
#else
        uint8_t AUTO_PCKT_FLT     : 1; /*!< 1: enable the automatic packet filtering control */
        uint8_t CSMA_PERS_ON      : 1; /*!< 1: enable the CSMA persistent mode (no back-off cycles) */
        uint8_t CSMA_ON           : 1; /*!< 1: enable the CSMA channel access mode */
        uint8_t SEED_RELOAD       : 1; /*!< 1: enable the reload of the back-off random generator seed using the value written in the BU_COUNTER_SEED */
        uint8_t FAST_CS_TERM_EN   : 1; /*!< 1: enable the RX sniff timer */
        uint8_t PIGGYBACKING      : 1; /*!< 1: enable the piggybacking */
        uint8_t LDC_RELOAD_ON_SYNC: 1; /*!< 1: enable the LDC timer reload mode */
        uint8_t LDC_MODE          : 1; /*!< 1: enable the Low Duty Cycle mode */
#endif /* SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN */
    };
    uint8_t raw;
} sl2_lp_ic_reg_protocol1_t;
static_assert(sizeof(sl2_lp_ic_reg_protocol1_t) == sizeof(uint8_t));

/**
 * @brief PROTOCOL0 register definition
 */
typedef __PACKED_UNION {
    __PACKED_STRUCT {
#if SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN
        uint8_t NMAX_RETX: 4; /*!< Max. number of re-TX (from 0 to 15)(0: re-transmission is not performed) */
        uint8_t NACK_TX  : 1; /*!< 1: field NO_ACK=1 on transmitted packet */
        uint8_t AUTO_ACK : 1; /*!< 1: enable the automatic acknowledgment if packet received request */
        uint8_t PERS_RX  : 1; /*!< 1: enable the persistent RX mode */
        uint8_t          : 1;
#else
        uint8_t          : 1;
        uint8_t PERS_RX  : 1; /*!< 1: enable the persistent RX mode */
        uint8_t AUTO_ACK : 1; /*!< 1: enable the automatic acknowledgment if packet received request */
        uint8_t NACK_TX  : 1; /*!< 1: field NO_ACK=1 on transmitted packet */
        uint8_t NMAX_RETX: 4; /*!< Max. number of re-TX (from 0 to 15)(0: re-transmission is not performed) */
#endif /* SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN */
    };
    uint8_t raw;
} sl2_lp_ic_reg_protocol0_t;
static_assert(sizeof(sl2_lp_ic_reg_protocol0_t) == sizeof(uint8_t));

/**
 * @brief Aggregator for PROTOCOLx registers
 */
typedef __PACKED_UNION {
    __PACKED_STRUCT {
        sl2_lp_ic_reg_protocol2_t protocol2;
        sl2_lp_ic_reg_protocol1_t protocol1;
        sl2_lp_ic_reg_protocol0_t protocol0;
    };
    uint8_t raw[3];
} sl2_lp_ic_reg_protocol_t;
static_assert(sizeof(sl2_lp_ic_reg_protocol_t) == 3u * sizeof(uint8_t));

/**
 * @brief FIFO_CONFIG3 register definition
 *
 * @note  The almost full threshold is encountered from the top of the FIFO. For example, if it is set to 7 the almost
 *        full FIFO irq will be raised when the number of elements is equals to 128-7 = 121
 */
typedef __PACKED_UNION {
    __PACKED_STRUCT {
#if SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN
        uint8_t         : 1;
        uint8_t RX_AFTHR: 7; /*!< Set the RX FIFO almost full threshold */
#else
        uint8_t RX_AFTHR: 7; /*!< Set the RX FIFO almost full threshold */
        uint8_t         : 1;
#endif /* SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN */
    };
    uint8_t raw;
} sl2_lp_ic_reg_fifo_config3_t;
static_assert(sizeof(sl2_lp_ic_reg_fifo_config3_t) == sizeof(uint8_t));

/**
 * @brief FIFO_CONFIG2 register definition
 */
typedef __PACKED_UNION {
    __PACKED_STRUCT {
#if SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN
        uint8_t         : 1;
        uint8_t RX_AETHR: 7; /*!< Set the RX FIFO almost empty threshold */
#else
        uint8_t RX_AETHR: 7; /*!< Set the RX FIFO almost empty threshold */
        uint8_t         : 1;
#endif /* SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN */
    };
    uint8_t raw;
} sl2_lp_ic_reg_fifo_config2_t;
static_assert(sizeof(sl2_lp_ic_reg_fifo_config2_t) == sizeof(uint8_t));

/**
 * @brief FIFO_CONFIG1 register definition
 *
 * @note  The almost full threshold is encountered from the top of the FIFO. For example, if it is set to 7 the almost
 *        full FIFO irq will be raised when the number of elements is equals to 128-7 = 121
 */
typedef __PACKED_UNION {
    __PACKED_STRUCT {
#if SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN
        uint8_t         : 1;
        uint8_t TX_AFTHR: 7; /*!< Set the TX FIFO almost full threshold */
#else
        uint8_t TX_AFTHR: 7; /*!< Set the TX FIFO almost full threshold */
        uint8_t         : 1;
#endif /* SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN */
    };
    uint8_t raw;
} sl2_lp_ic_reg_fifo_config1_t;
static_assert(sizeof(sl2_lp_ic_reg_fifo_config1_t) == sizeof(uint8_t));

/**
 * @brief FIFO_CONFIG0 register definition
 */
typedef __PACKED_UNION {
    __PACKED_STRUCT {
#if SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN
        uint8_t         : 1;
        uint8_t TX_AETHR: 7; /*!< Set the TX FIFO almost empty threshold */
#else
        uint8_t TX_AETHR: 7; /*!< Set the TX FIFO almost empty threshold */
        uint8_t         : 1;
#endif /* SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN */
    };
    uint8_t raw;
} sl2_lp_ic_reg_fifo_config0_t;
static_assert(sizeof(sl2_lp_ic_reg_fifo_config0_t) == sizeof(uint8_t));

/**
 * @brief Aggregator for FIFO_CONFIGx registers
 */
typedef __PACKED_UNION {
    __PACKED_STRUCT {
        sl2_lp_ic_reg_fifo_config3_t fifo_config3;
        sl2_lp_ic_reg_fifo_config2_t fifo_config2;
        sl2_lp_ic_reg_fifo_config1_t fifo_config1;
        sl2_lp_ic_reg_fifo_config0_t fifo_config0;
    };
    uint8_t raw[4];
} sl2_lp_ic_reg_fifo_config_t;
static_assert(sizeof(sl2_lp_ic_reg_fifo_config_t) == 4u * sizeof(uint8_t));

/**
 * @brief PCKT_FLT_OPTIONS register definition
 */
typedef __PACKED_UNION {
    __PACKED_STRUCT {
#if SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN
        uint8_t                       : 1;
        uint8_t RX_TIMEOUT_AND_OR_SEL : 1; /*!< Logical Boolean function applied to CS/SQI/PQI values: 1: OR, 0: AND */
        uint8_t                       : 1;
        uint8_t SOURCE_ADDR_FLT       : 1; /*!< 1: RX packet accepted if its source field matches with RX_SOURCE_ADDR register */
        uint8_t DEST_VS_BROADCAST_ADDR: 1; /*!< 1: RX packet accepted if its source field matches with BROADCAST_ADDR register */
        uint8_t DEST_VS_MULTICAST_ADDR: 1; /*!< 1: RX packet accepted if its destination address matches with MULTICAST_ADDR register */
        uint8_t DEST_VS_SOURCE_ADDR   : 1; /*!< 1: RX packet accepted if its destination address matches with RX_SOURCE_ADDR register */
        uint8_t CRC_FLT               : 1; /*!< 1: packet discarded if CRC is not valid */
#else
        uint8_t CRC_FLT               : 1; /*!< 1: packet discarded if CRC is not valid */
        uint8_t DEST_VS_SOURCE_ADDR   : 1; /*!< 1: RX packet accepted if its destination address matches with RX_SOURCE_ADDR register */
        uint8_t DEST_VS_MULTICAST_ADDR: 1; /*!< 1: RX packet accepted if its destination address matches with MULTICAST_ADDR register */
        uint8_t DEST_VS_BROADCAST_ADDR: 1; /*!< 1: RX packet accepted if its source field matches with BROADCAST_ADDR register */
        uint8_t SOURCE_ADDR_FLT       : 1; /*!< 1: RX packet accepted if its source field matches with RX_SOURCE_ADDR register */
        uint8_t                       : 1;
        uint8_t RX_TIMEOUT_AND_OR_SEL : 1; /*!< Logical Boolean function applied to CS/SQI/PQI values: 1: OR, 0: AND */
        uint8_t                       : 1;
#endif /* SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN */
    };
    uint8_t raw;
} sl2_lp_ic_reg_pckt_flt_options_t;
static_assert(sizeof(sl2_lp_ic_reg_pckt_flt_options_t) == sizeof(uint8_t));

/**
 * @brief TIMERS5 register definition
 */
typedef __PACKED_UNION {
    uint8_t RX_TIMER_CNTR; /*!< Counter for RX timer */
    uint8_t raw;
} sl2_lp_ic_reg_timers5_t;
static_assert(sizeof(sl2_lp_ic_reg_timers5_t) == sizeof(uint8_t));

/**
 * @brief TIMERS4 register definition
 */
typedef __PACKED_UNION {
    uint8_t RX_TIMER_PRESC; /*!< Prescaler for RX timer */
    uint8_t raw;
} sl2_lp_ic_reg_timers4_t;
static_assert(sizeof(sl2_lp_ic_reg_timers4_t) == sizeof(uint8_t));

/**
 * @brief TIMERS3 register definition
 */
typedef __PACKED_UNION {
    uint8_t LDC_TIMER_PRESC; /*!< Prescaler for wake up timer */
    uint8_t raw;
} sl2_lp_ic_reg_timers3_t;
static_assert(sizeof(sl2_lp_ic_reg_timers3_t) == sizeof(uint8_t));

/**
 * @brief TIMERS2 register definition
 */
typedef __PACKED_UNION {
    uint8_t LDC_TIMER_CNTR; /*!< Counter for wake up timer */
    uint8_t raw;
} sl2_lp_ic_reg_timers2_t;
static_assert(sizeof(sl2_lp_ic_reg_timers2_t) == sizeof(uint8_t));

/**
 * @brief TIMERS1 register definition
 */
typedef __PACKED_UNION {
    uint8_t LDC_RELOAD_PRSC; /*!< Prescaler value for reload operation of wake up timer */
    uint8_t raw;
} sl2_lp_ic_reg_timers1_t;
static_assert(sizeof(sl2_lp_ic_reg_timers1_t) == sizeof(uint8_t));

/**
 * @brief TIMERS0 register definition
 */
typedef __PACKED_UNION {
    uint8_t LDC_RELOAD_CNTR; /*!< Counter value for reload operation of wake up timer */
    uint8_t raw;
} sl2_lp_ic_reg_timers0_t;
static_assert(sizeof(sl2_lp_ic_reg_timers0_t) == sizeof(uint8_t));

/**
 * @brief Aggregator for Rx timer params registers
 */
typedef __PACKED_UNION {
    __PACKED_STRUCT {
        sl2_lp_ic_reg_timers5_t timers5;
        sl2_lp_ic_reg_timers4_t timers4;
    };
    uint8_t raw[2];
} sl2_lp_ic_reg_rx_timer_cfg_t;
static_assert(sizeof(sl2_lp_ic_reg_rx_timer_cfg_t) == 2u * sizeof(uint8_t));

/**
 * @brief Aggregator for LDC wakeup timer params registers
 */
typedef __PACKED_UNION {
    __PACKED_STRUCT {
        sl2_lp_ic_reg_timers3_t timers3;
        sl2_lp_ic_reg_timers2_t timers2;
    };
    uint8_t raw[2];
} sl2_lp_ic_reg_ldc_wakeup_timer_cfg_t;
static_assert(sizeof(sl2_lp_ic_reg_ldc_wakeup_timer_cfg_t) == 2u * sizeof(uint8_t));

/**
 * @brief IRQ_MASK3 register definition
 */
typedef __PACKED_UNION {
    uint8_t INT_MASK_31_24;
    uint8_t raw;
} sl2_lp_ic_reg_irq_mask3_t;
static_assert(sizeof(sl2_lp_ic_reg_irq_mask3_t) == sizeof(uint8_t));

/**
 * @brief IRQ_MASK2 register definition
 */
typedef __PACKED_UNION {
    uint8_t INT_MASK_23_16;
    uint8_t raw;
} sl2_lp_ic_reg_irq_mask2_t;
static_assert(sizeof(sl2_lp_ic_reg_irq_mask2_t) == sizeof(uint8_t));

/**
 * @brief IRQ_MASK1 register definition
 */
typedef __PACKED_UNION {
    uint8_t INT_MASK_15_8;
    uint8_t raw;
} sl2_lp_ic_reg_irq_mask1_t;
static_assert(sizeof(sl2_lp_ic_reg_irq_mask1_t) == sizeof(uint8_t));

/**
 * @brief IRQ_MASK0 register definition
 */
typedef __PACKED_UNION {
    uint8_t INT_MASK_7_0;
    uint8_t raw;
} sl2_lp_ic_reg_irq_mask0_t;
static_assert(sizeof(sl2_lp_ic_reg_irq_mask0_t) == sizeof(uint8_t));

/**
 * @brief Aggregator for IRQ mask registers
 */
typedef __PACKED_UNION {
    __PACKED_STRUCT {
        sl2_lp_ic_reg_irq_mask3_t irq_mask3;
        sl2_lp_ic_reg_irq_mask2_t irq_mask2;
        sl2_lp_ic_reg_irq_mask1_t irq_mask1;
        sl2_lp_ic_reg_irq_mask0_t irq_mask0;
    };
    uint8_t raw[4];
} sl2_lp_ic_reg_irq_mask_t;
static_assert(sizeof(sl2_lp_ic_reg_irq_mask_t) == 4u * sizeof(uint8_t));

/**
 * @brief TIMER_CONF3 register definition
 */
typedef __PACKED_UNION {
    uint8_t PM_START_COUNTER; /*!< Power management start timeout expressed in unit of about 2us. Default value is 0x11 */
    uint8_t raw;
} sl2_lp_ic_reg_timer_conf3_t;
static_assert(sizeof(sl2_lp_ic_reg_timer_conf3_t) == sizeof(uint8_t));

/**
 * @brief PA_POWER8 - PA_POWER1 registers definition
 */
typedef __PACKED_UNION {
    __PACKED_STRUCT {
#if SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN
        uint8_t         : 1;
        uint8_t PA_LEVEL: 7; /*!< Output power level for Nth slot */
#else
        uint8_t PA_LEVEL: 7; /*!< Output power level for Nth slot */
        uint8_t         : 1;
#endif /* SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN */
    };
    uint8_t raw;
} sl2_lp_ic_reg_pa_powerx_t;
static_assert(sizeof(sl2_lp_ic_reg_pa_powerx_t) == sizeof(uint8_t));

/**
 * @brief PA_POWER0 register definition
 */
typedef __PACKED_UNION {
    __PACKED_STRUCT {
#if SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN
        uint8_t DIG_SMOOTH_EN   : 1; /*!< 1: enable the generation of the internal signal TX_DATA which is the input of the FIR. Needed when FIR_EN=1 */
        uint8_t PA_MAXDBM       : 1; /*!< 1: configure the PA to send maximum output power. Power ramping is disabled with this bit set to 1 */
        uint8_t PA_RAMP_EN      : 1; /*!< 1: enable the power ramping */
        uint8_t PA_RAMP_STEP_LEN: 2; /*!< Set the step width (unit: 1/8 of bit period) */
        uint8_t PA_LEVEL_MAX_IDX: 3; /*!< Final level for power ramping or selected output power index */
#else
        uint8_t PA_LEVEL_MAX_IDX: 3; /*!< Final level for power ramping or selected output power index */
        uint8_t PA_RAMP_STEP_LEN: 2; /*!< Set the step width (unit: 1/8 of bit period) */
        uint8_t PA_RAMP_EN      : 1; /*!< 1: enable the power ramping */
        uint8_t PA_MAXDBM       : 1; /*!< 1: configure the PA to send maximum output power. Power ramping is disabled with this bit set to 1 */
        uint8_t DIG_SMOOTH_EN   : 1; /*!< 1: enable the generation of the internal signal TX_DATA which is the input of the FIR. Needed when FIR_EN=1 */
#endif /* SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN */
    };
    uint8_t raw;
} sl2_lp_ic_reg_pa_power0_t;
static_assert(sizeof(sl2_lp_ic_reg_pa_power0_t) == sizeof(uint8_t));

/**
 * @brief PA_CONFIG1 register definition
 */
typedef __PACKED_UNION {
    __PACKED_STRUCT {
#if SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN
        uint8_t                   : 4;
        s2_lp_ic_fir_cfg_t FIR_CFG: 2; /*!< FIR configuration */
        uint8_t FIR_EN            : 1; /*!< 1: enable FIR */
        uint8_t                   : 1;
#else
        uint8_t                   : 1;
        uint8_t FIR_EN            : 1; /*!< 1: enable FIR */
        s2_lp_ic_fir_cfg_t FIR_CFG: 2; /*!< FIR configuration */
        uint8_t                   : 4;
#endif /* SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN */
    };
    uint8_t raw;
} sl2_lp_ic_reg_pa_config1_t;
static_assert(sizeof(sl2_lp_ic_reg_pa_config1_t) == sizeof(uint8_t));

/**
 * @brief PA_CONFIG0 register definition
 */
typedef __PACKED_UNION {
    __PACKED_STRUCT {
#if SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN
        s2_lp_ic_pa_degen_trim_code_th_t PA_DEGEN_TRIM_CODE_TH: 2;
        s2_lp_ic_pa_degen_trim_clamp_v_t PA_DEGEN_TRIM_CLAMP_V: 2;
        uint8_t                          PA_DEGEN_ON          : 1; /*!< Enables the 'degeneration' mode that introduces a pre-distortion to linearize the power control curve. */
        uint8_t                          SAFE_ASK_CAL         : 1; /*!< During a TX operation, enables and starts the digital ASK calibrator */
        s2_lp_ic_pa_fc_t                 PA_FC                : 2; /*!< PA bessel filter bandwidth */
#else
        s2_lp_ic_pa_fc_t                 PA_FC                : 2; /*!< PA bessel filter bandwidth */
        uint8_t                          SAFE_ASK_CAL         : 1; /*!< During a TX operation, enables and starts the digital ASK calibrator */
        uint8_t                          PA_DEGEN_ON          : 1; /*!< Enables the 'degeneration' mode that introduces a pre-distortion to linearize the power control curve. */
        s2_lp_ic_pa_degen_trim_clamp_v_t PA_DEGEN_TRIM_CLAMP_V: 2;
        s2_lp_ic_pa_degen_trim_code_th_t PA_DEGEN_TRIM_CODE_TH: 2;
#endif /* SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN */
    };
    uint8_t raw;
} sl2_lp_ic_reg_pa_config0_t;
static_assert(sizeof(sl2_lp_ic_reg_pa_config0_t) == sizeof(uint8_t));

/**
 * @brief Aggregator for PA configuration registers
 */
typedef __PACKED_UNION {
    __PACKED_STRUCT {
        sl2_lp_ic_reg_pa_power0_t  pa_power0;
        sl2_lp_ic_reg_pa_config1_t pa_config1;
        sl2_lp_ic_reg_pa_config0_t pa_config0;
    };
    uint8_t raw[3];
} sl2_lp_ic_reg_pa_cfg_t;
static_assert(sizeof(sl2_lp_ic_reg_pa_cfg_t) == 3u * sizeof(uint8_t));

/**
 * @brief SYNTH_CONFIG2 register definition
 */
typedef __PACKED_UNION {
    __PACKED_STRUCT {
#if SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN
        uint8_t                 : 5; /*!< Reserved */
        uint8_t PLL_PFD_SPLIT_EN: 1; /*!< Enables increased DN current pulses to improve linearization of CP/PFD */
        uint8_t                 : 2; /*!< Reserved */
#else
        uint8_t                 : 2; /*!< Reserved */
        uint8_t PLL_PFD_SPLIT_EN: 1; /*!< Enables increased DN current pulses to improve linearization of CP/PFD */
        uint8_t                 : 5; /*!< Reserved */
#endif /* SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN */
    };
    uint8_t raw;
} sl2_lp_ic_reg_synth_config2_t;
static_assert(sizeof(sl2_lp_ic_reg_synth_config2_t) == sizeof(uint8_t));

/**
 * @brief XO_RCO_CONF1 register definition
 */
typedef __PACKED_UNION {
    __PACKED_STRUCT {
#if SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN
        uint8_t          : 3; /*!< Reserved */
        uint8_t PD_CLKDIV: 1; /*!< 1: disable both dividers of digital clock (and reference clock for the SMPS) and IF-ADC clock */
        uint8_t          : 4; /*!< Reserved */
#else
        uint8_t          : 4; /*!< Reserved */
        uint8_t PD_CLKDIV: 1; /*!< 1: disable both dividers of digital clock (and reference clock for the SMPS) and IF-ADC clock */
        uint8_t          : 3; /*!< Reserved */
#endif /* SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN */
    };
    uint8_t raw;
} sl2_lp_ic_reg_xo_rco_conf1_t;
static_assert(sizeof(sl2_lp_ic_reg_xo_rco_conf1_t) == sizeof(uint8_t));

/**
 * @brief XO_RCO_CONF0 register definition
 */
typedef __PACKED_UNION {
    __PACKED_STRUCT {
#if SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN
        uint8_t EXT_REF         : 1; /*!< 0: reference signal from XO circuit; 1: reference signal from XIN pin */
        s2_lp_ic_xo_gm_t GM_CONF: 3; /*!< Set the driver gain margin (transconductance) of the XO at start up */
        uint8_t REFDIV          : 1; /*!< 1: enable the the reference clock divider */
        uint8_t                 : 1; /*!< Reserved */
        uint8_t EXT_RCO_OSC     : 1; /*!< 1: the 34.7 kHz signal must be supplied from any GPIO */
        uint8_t RCO_CALIBRATION : 1; /*!< 1: enable the automatic RCO calibration */
#else
        uint8_t RCO_CALIBRATION : 1; /*!< 1: enable the automatic RCO calibration */
        uint8_t EXT_RCO_OSC     : 1; /*!< 1: the 34.7 kHz signal must be supplied from any GPIO */
        uint8_t                 : 1; /*!< Reserved */
        uint8_t REFDIV          : 1; /*!< 1: enable the the reference clock divider */
        s2_lp_ic_xo_gm_t GM_CONF: 3; /*!< Set the driver gm of the XO at start up */
        uint8_t EXT_REF         : 1; /*!< 0: reference signal from XO circuit; 1: reference signal from XIN pin */
#endif /* SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN */
    };
    uint8_t raw;
} sl2_lp_ic_reg_xo_rco_conf0_t;
static_assert(sizeof(sl2_lp_ic_reg_xo_rco_conf0_t) == sizeof(uint8_t));

/**
 * @brief RCO_CALIBR_CONF3 register definition
 */
typedef __PACKED_UNION {
    __PACKED_STRUCT {
#if SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN
        uint8_t RWT_IN    : 4; /*!< RWT word value for the RCO */
        uint8_t RFB_IN_4_1: 4; /*!< MSB part of RFB word value for RCO */
#else
        uint8_t RFB_IN_4_1: 4; /*!< MSB part of RFB word value for RCO */
        uint8_t RWT_IN    : 4; /*!< RWT word value for the RCO */
#endif /* SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN */
    };
    uint8_t raw;
} sl2_lp_ic_reg_rco_calibr_conf3_t;
static_assert(sizeof(sl2_lp_ic_reg_rco_calibr_conf3_t) == sizeof(uint8_t));

/**
 * @brief RCO_CALIBR_CONF2 register definition
 */
typedef __PACKED_UNION {
    __PACKED_STRUCT {
#if SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN
        uint8_t RFB_IN_0: 1; /*!< LSB part of RFB word value for RCO */
        uint8_t         : 7;
#else
        uint8_t         : 7;
        uint8_t RFB_IN_0: 1; /*!< LSB part of RFB word value for RCO */
#endif /* SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN */
    };
    uint8_t raw;
} sl2_lp_ic_reg_rco_calibr_conf2_t;
static_assert(sizeof(sl2_lp_ic_reg_rco_calibr_conf2_t) == sizeof(uint8_t));

/**
 * @brief Aggregated S2-LP RCO calibration values
 */
typedef __PACKED_UNION {
    __PACKED_STRUCT {
        sl2_lp_ic_reg_rco_calibr_conf3_t rco_calibr_conf3;
        sl2_lp_ic_reg_rco_calibr_conf2_t rco_calibr_conf2;
    };
    uint8_t raw[2];
} sl2_lp_ic_reg_rco_calibr_conf_t;
static_assert(sizeof(sl2_lp_ic_reg_rco_calibr_conf_t) == 2u * sizeof(uint8_t));

/**
 * @brief PM_CONF4 register definition
 */
typedef __PACKED_UNION {
    __PACKED_STRUCT {
#if SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN
        uint8_t         : 2;
        uint8_t EXT_SMPS: 1; /*!< 1: disable the internal SMPS */
        uint8_t         : 5;
#else
        uint8_t         : 5;
        uint8_t EXT_SMPS: 1; /*!< 1: disable the internal SMPS */
        uint8_t         : 2;
#endif /* SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN */
    };
    uint8_t raw;
} sl2_lp_ic_reg_pm_conf4_t;
static_assert(sizeof(sl2_lp_ic_reg_pm_conf4_t) == sizeof(uint8_t));

/**
 * @brief PM_CONF3 register definition
 */
typedef __PACKED_UNION {
    __PACKED_STRUCT {
#if SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN
        uint8_t KRM_EN  : 1; /*!< 0: divider by 4 enabled (SMPS' switching frequency is FSW=Fdig/4); 1: rate multiplier enabled (SMPS' switching frequency is FSW=KRM*Fdig/(2^15) */
        uint8_t KRM_14_8: 7; /*!< KRM multiplier MSB */
#else
        uint8_t KRM_14_8: 7; /*!< KRM multiplier MSB */
        uint8_t KRM_EN  : 1; /*!< 0: divider by 4 enabled (SMPS' switching frequency is FSW=Fdig/4); 1: rate multiplier enabled (SMPS' switching frequency is FSW=KRM*Fdig/(2^15) */
#endif /* SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN */
    };
    uint8_t raw;
} sl2_lp_ic_reg_pm_conf3_t;
static_assert(sizeof(sl2_lp_ic_reg_pm_conf3_t) == sizeof(uint8_t));

/**
 * @brief PM_CONF2 register definition
 */
typedef __PACKED_UNION {
    uint8_t KRM_7_0; /*!< KRM multiplier LSB */
    uint8_t raw;
} sl2_lp_ic_reg_pm_conf2_t;
static_assert(sizeof(sl2_lp_ic_reg_pm_conf2_t) == sizeof(uint8_t));

/**
 * @brief PM_CONF1 register definition
 */
typedef __PACKED_UNION {
    __PACKED_STRUCT {
#if SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN
        uint8_t                                : 1;
        uint8_t                  BATTERY_LVL_EN: 1; /*!< 1: enable battery level detector circuit */
        s2_lp_ic_bld_thershold_t SET_BLD_TH    : 2; /*!< Set the Battery Level Detector threshold */
        s2_lp_ic_smps_lvl_mode_t SMPS_LVL_MODE : 1;
        uint8_t                  BYPASS_LDO    : 1; /*!< 0: use LDO; 1: bypass LDO and rely on SMPS only */
        uint8_t                                : 2;
#else
        uint8_t                                : 2;
        uint8_t                  BYPASS_LDO    : 1; /*!< 0: use LDO; 1: bypass LDO and rely on SMPS only */
        s2_lp_ic_smps_lvl_mode_t SMPS_LVL_MODE : 1;
        s2_lp_ic_bld_thershold_t SET_BLD_TH    : 2; /*!< Set the Battery Level Detector threshold */
        uint8_t                  BATTERY_LVL_EN: 1; /*!< 1: enable battery level detector circuit */
        uint8_t                                : 1;
#endif /* SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN */
    };
    uint8_t raw;
} sl2_lp_ic_reg_pm_conf1_t;
static_assert(sizeof(sl2_lp_ic_reg_pm_conf1_t) == sizeof(uint8_t));

/**
 * @brief PM_CONF0 register definition
 */
typedef __PACKED_UNION {
    __PACKED_STRUCT {
#if SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN
        uint8_t                             : 1;
        s2_lp_ic_smps_lvl_t   SET_SMPS_LVL  : 3; /*!< SMPS output voltage */
        uint8_t                             : 3;
        s2_lp_ic_sleep_mode_t SLEEP_MODE_SEL: 1;
#else
        s2_lp_ic_sleep_mode_t SLEEP_MODE_SEL: 1;
        uint8_t                             : 3;
        s2_lp_ic_smps_lvl_t   SET_SMPS_LVL  : 3; /*!< SMPS output voltage */
        uint8_t                             : 1;
#endif /* SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN */
    };
    uint8_t raw;
} sl2_lp_ic_reg_pm_conf0_t;
static_assert(sizeof(sl2_lp_ic_reg_pm_conf0_t) == sizeof(uint8_t));

/**
 * @brief MC_STATE1 register definition
 */
typedef __PACKED_UNION {
    __PACKED_STRUCT {
#if SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN
        uint8_t                   : 3; /*!< This 3 bits field are reserved and equal to 2 */
        uint8_t RCCAL_OK          : 1; /*!< RCO successfully calibrated */
        uint8_t ANT_SELECT        : 1; /*!< Currently selected antenna */
        uint8_t TX_FIFO_FULL      : 1; /*!< TX FIFO is full */
        uint8_t RX_FIFO_EMPTY     : 1; /*!< RX FIFO is empty */
        uint8_t ERROR_LOCK        : 1; /*!< RCO calibration error */
#else
        uint8_t ERROR_LOCK        : 1; /*!< RCO calibration error */
        uint8_t RX_FIFO_EMPTY     : 1; /*!< RX FIFO is empty */
        uint8_t TX_FIFO_FULL      : 1; /*!< TX FIFO is full */
        uint8_t ANT_SELECT        : 1; /*!< Currently selected antenna */
        uint8_t RCCAL_OK          : 1; /*!< RCO successfully calibrated */
        uint8_t                   : 3; /*!< This 3 bits field are reserved and equal to 2 */
#endif /* SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN */
    };
    uint8_t raw;
} sl2_lp_ic_reg_mc_state1_t;
static_assert(sizeof(sl2_lp_ic_reg_mc_state1_t) == sizeof(uint8_t));

/**
 * @brief MC_STATE0 register definition
 */
typedef __PACKED_UNION {
    __PACKED_STRUCT {
#if SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN
        s2_lp_ic_states_t MC_STATE: 7; /*!< The state of the Main Controller of S2-LP */
        uint8_t XO_ON             : 1; /*!< XO is operating state */
#else
        uint8_t XO_ON             : 1; /*!< XO is operating state */
        s2_lp_ic_states_t MC_STATE: 7; /*!< The state of the Main Controller of S2-LP */
#endif /* SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN */
    };
    uint8_t raw;
} sl2_lp_ic_reg_mc_state0_t;
static_assert(sizeof(sl2_lp_ic_reg_mc_state0_t) == sizeof(uint8_t));

/**
 * @brief Aggregated S2-LP status info
 */
typedef __PACKED_UNION {
    __PACKED_STRUCT {
        sl2_lp_ic_reg_mc_state1_t reg_mc_state1;
        sl2_lp_ic_reg_mc_state0_t reg_mc_state0;
    };
    uint8_t raw[2];
} s2_lp_ic_status_t;
static_assert(sizeof(s2_lp_ic_status_t) == 2u * sizeof(uint8_t));

/**
 * @brief TX_FIFO_STATUS register definition
 */
typedef __PACKED_UNION {
    uint8_t NELEM_TXFIFO; /*!< Number of elements in TX FIFO */
    uint8_t raw;
} sl2_lp_ic_reg_tx_fifo_status_t;
static_assert(sizeof(sl2_lp_ic_reg_tx_fifo_status_t) == sizeof(uint8_t));

/**
 * @brief RX_FIFO_STATUS register definition
 */
typedef __PACKED_UNION {
    uint8_t NELEM_RXFIFO; /*!< Number of elements in RX FIFO */
    uint8_t raw;
} sl2_lp_ic_reg_rx_fifo_status_t;
static_assert(sizeof(sl2_lp_ic_reg_rx_fifo_status_t) == sizeof(uint8_t));

/**
 * @brief RCO_CALIBR_OUT4 register definition
 */
typedef __PACKED_UNION {
    __PACKED_STRUCT {
#if SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN
        uint8_t RWT_OUT    : 4; /*!< RWT word from internal RCO calibrator */
        uint8_t RFB_OUT_4_1: 4; /*!< RFB word (MSB) from internal RCO calibrator */
#else
        uint8_t RFB_OUT_4_1: 4; /*!< RFB word (MSB) from internal RCO calibrator */
        uint8_t RWT_OUT    : 4; /*!< RWT word from internal RCO calibrator */
#endif /* SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN */
    };
    uint8_t raw;
} sl2_lp_ic_reg_rco_calibr_out4_t;
static_assert(sizeof(sl2_lp_ic_reg_rco_calibr_out4_t) == sizeof(uint8_t));

/**
 * @brief RCO_CALIBR_OUT3 register definition
 */
typedef __PACKED_UNION {
    __PACKED_STRUCT {
#if SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN
        uint8_t RFB_OUT_0: 1; /*!< RF word (LSB) from internal RCO calibrator */
        uint8_t          : 7;
#else
        uint8_t          : 7;
        uint8_t RFB_OUT_0: 1; /*!< RF word (LSB) from internal RCO calibrator */
#endif /* SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN */
    };
    uint8_t raw;
} sl2_lp_ic_reg_rco_calibr_out3_t;
static_assert(sizeof(sl2_lp_ic_reg_rco_calibr_out3_t) == sizeof(uint8_t));

/**
 * @brief Aggregated S2-LP RCO calibrator output values
 */
typedef __PACKED_UNION {
    __PACKED_STRUCT {
        sl2_lp_ic_reg_rco_calibr_out4_t rco_calibr_out4;
        sl2_lp_ic_reg_rco_calibr_out3_t rco_calibr_out3;
    };
    uint8_t raw[2];
} sl2_lp_ic_reg_rco_calibr_out_t;
static_assert(sizeof(sl2_lp_ic_reg_rco_calibr_out_t) == 2u * sizeof(uint8_t));

/**
 * @brief AFC_CORR register definition
 */
typedef __PACKED_UNION 
{
    int8_t  AFC_CORR; /*!< AFC correction value */
    uint8_t raw;
} sl2_lp_ic_reg_afc_corr_t;
static_assert(sizeof(sl2_lp_ic_reg_afc_corr_t) == sizeof(uint8_t));

/**
 * @brief LINK_QUALIF2 register definition
 */
typedef __PACKED_UNION 
{
    uint8_t PQI; /*!< PQI value of the received packet */
    uint8_t raw;
} sl2_lp_ic_reg_link_qualif2_t;
static_assert(sizeof(sl2_lp_ic_reg_link_qualif2_t) == sizeof(uint8_t));

/**
 * @brief LINK_QUALIF1 register definition
 */
typedef __PACKED_UNION 
{
    __PACKED_STRUCT {
#if SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN
        uint8_t CS        : 1; /*!< Carrier sense indication */
        uint8_t SQI_SW_IDX: 1; /*!< 0: SQI peak value refers to the primary SYNC word; 1: SQI peak value refers to the secondary SYNC word */
        uint8_t SQI       : 6; /*!< SQI value of the received packet */
#else
        uint8_t SQI       : 6; /*!< SQI value of the received packet */
        uint8_t SQI_SW_IDX: 1; /*!< 0: SQI peak value refers to the primary SYNC word; 1: SQI peak value refers to the secondary SYNC word */
        uint8_t CS        : 1; /*!< Carrier sense indication */
#endif /* SID_STM32_UTIL_IS_PLATFORM_BIG_ENDIAN */
    };
    uint8_t raw;
} sl2_lp_ic_reg_link_qualif1_t;
static_assert(sizeof(sl2_lp_ic_reg_link_qualif1_t) == sizeof(uint8_t));

/**
 * @brief DEVICE_INFO1 register definition
 */
typedef __PACKED_UNION {
    uint8_t PARTNUM; /*!< S2-LP part number */
    uint8_t raw;
} sl2_lp_ic_reg_device_info1_t;
static_assert(sizeof(sl2_lp_ic_reg_device_info1_t) == sizeof(uint8_t));

/**
 * @brief DEVICE_INFO0 register definition
 */
typedef __PACKED_UNION {
    uint8_t VERSION; /*!< S2-LP version number */
    uint8_t raw;
} sl2_lp_ic_reg_device_info0_t;
static_assert(sizeof(sl2_lp_ic_reg_device_info0_t) == sizeof(uint8_t));

/**
 * @brief Aggregated S2-LP device version info
 */
typedef __PACKED_UNION {
    __PACKED_STRUCT {
        sl2_lp_ic_reg_device_info1_t part_number;
        sl2_lp_ic_reg_device_info0_t version;
    };
    uint8_t raw[2];
} s2_lp_ic_version_info_t;
static_assert(sizeof(s2_lp_ic_version_info_t) == 2u * sizeof(uint8_t));

/**
 * @brief IRQ_STATUS3 register definition
 */
typedef __PACKED_UNION {
    uint8_t INT_LEVEL_31_24;
    uint8_t raw;
} sl2_lp_ic_reg_irq_status3_t;
static_assert(sizeof(sl2_lp_ic_reg_irq_status3_t) == sizeof(uint8_t));

/**
 * @brief IRQ_STATUS2 register definition
 */
typedef __PACKED_UNION {
    uint8_t INT_LEVEL_23_16;
    uint8_t raw;
} sl2_lp_ic_reg_irq_status2_t;
static_assert(sizeof(sl2_lp_ic_reg_irq_status2_t) == sizeof(uint8_t));

/**
 * @brief IRQ_STATUS1 register definition
 */
typedef __PACKED_UNION {
    uint8_t INT_LEVEL_15_8;
    uint8_t raw;
} sl2_lp_ic_reg_irq_status1_t;
static_assert(sizeof(sl2_lp_ic_reg_irq_status1_t) == sizeof(uint8_t));

/**
 * @brief IRQ_STATUS0 register definition
 */
typedef __PACKED_UNION {
    uint8_t INT_LEVEL_7_0;
    uint8_t raw;
} sl2_lp_ic_reg_irq_status0_t;
static_assert(sizeof(sl2_lp_ic_reg_irq_status0_t) == sizeof(uint8_t));

/**
 * @brief Aggregator for IRQ status registers
 */
typedef __PACKED_UNION {
    __PACKED_STRUCT {
        sl2_lp_ic_reg_irq_status3_t irq_status3;
        sl2_lp_ic_reg_irq_status2_t irq_status2;
        sl2_lp_ic_reg_irq_status1_t irq_status1;
        sl2_lp_ic_reg_irq_status0_t irq_status0;
    };
    uint8_t raw[4];
} sl2_lp_ic_reg_irq_status_t;
static_assert(sizeof(sl2_lp_ic_reg_irq_status_t) == 4u * sizeof(uint8_t));

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* __S2_LP_IC_DEFINITIONS_H_ */
