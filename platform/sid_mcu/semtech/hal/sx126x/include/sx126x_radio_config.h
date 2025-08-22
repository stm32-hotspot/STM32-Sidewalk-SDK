/*
 * Copyright (c) 2019-2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * This file is exposed to each product configuration to customize the driver behavior
 */
/**
  ******************************************************************************
  * @file    sx126x_radio_config.h
  * @brief   Semtech SX126x Sidewalk driver config
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

#ifndef __SID_STM32_SX126X_CONFIG_H_
#define __SID_STM32_SX126X_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include <stdbool.h>
#include <stdint.h>

/* Semtech radio driver interfaces */
#include "sx126x.h"
#include <semtech_radio_ifc.h>

/* Sidewalk interfaces */
#include <sid_fsk_phy_cfg.h>
#include <sid_pal_radio_ifc.h>
#include <sid_pal_serial_bus_ifc.h>

/* Radio driver interfaces */
#include "sx126x_radio_default_timings.h"

/* Exported constants --------------------------------------------------------*/

#define SID_RADIO_PLATFORM_SX126X

#define SEMTECH_ID_SX1261                           (0x01u)
#define SEMTECH_ID_SX1262                           (0x02u)

#define HALO_GPIO_NOT_CONNECTED                     (0u)

#ifndef SX126X_RADIO_CFG_USE_STATUS_LED             /*!< Allows driver status indication with an LED */
#  define SX126X_RADIO_CFG_USE_STATUS_LED           (1u)
#endif /* SX126X_RADIO_CFG_USE_STATUS_LED */

#ifndef SX126X_RADIO_CFG_USE_FEM_PWR_CTRL           /*!< Set this to 1 if your setup has a GPIO pin to turn radio FEM on/off */
#  define SX126X_RADIO_CFG_USE_FEM_PWR_CTRL         (1u)
#endif /* SX126X_RADIO_CFG_USE_FEM_PWR_CTRL */

#ifndef SX126X_RADIO_CFG_USE_TX_RX_CTRL             /*!< Set this to 1 if your setup has an MCU GPIO pin to select Tx or Rx mode. The reference implementation uses SX126x's DIO2 pin for that and does not need this option */
#  define SX126X_RADIO_CFG_USE_TX_RX_CTRL           (0u)
#endif /* SX126X_RADIO_CFG_USE_TX_RX_CTRL */

#ifndef SX126X_RADIO_CFG_USE_EXTERNAL_PA            /*!< Set this to 1 if your setup has an external power amplifier for SX126x. The reference implementation drives PE4259 RF switch and does not use this mode */
#  define SX126X_RADIO_CFG_USE_EXTERNAL_PA          (0u)
#endif /* SX126X_RADIO_CFG_USE_EXTERNAL_PA */

#ifndef SX126X_RADIO_CFG_USE_FEM_BYPASS_CTRL        /*!< Set this to 1 if your setup uses external PA and it provides a Tx bypass GPIO control, allowing a direct connection of SX126x's built-in PA to antenna */
#  define SX126X_RADIO_CFG_USE_FEM_BYPASS_CTRL      (SX126X_RADIO_CFG_USE_EXTERNAL_PA)
#endif /* SX126X_RADIO_CFG_USE_FEM_BYPASS_CTRL */

#ifndef SX126X_RADIO_CFG_IRQ_PROCESS_GUARD_TIME_MS
# define SX126X_RADIO_CFG_IRQ_PROCESS_GUARD_TIME_MS (2u) /*!< Acceptable delay (in ms) between radio IRQ detection and actual processing. Increase this allowance if radio IRQ processing gets delayed in your application (e.g. by higher priority BLE radio IRQ) */
#endif /* SX126X_RADIO_CFG_IRQ_PROCESS_GUARD_TIME_MS */

#ifndef SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION
#  define SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION     (0u) /*!< Include additional code to integrate LoRa Basic Modem support features */
#endif /* SX126X_RADIO_CFG_SMTC_LBM_INTEGRATION */

#define RADIO_REGION_NA                             (SID_PAL_RADIO_RC_NA)
#define RADIO_REGION_EU                             (SID_PAL_RADIO_RC_EU)
#define RADIO_REGION_NONE                           (SID_PAL_RADIO_RC_NONE)

#define SX1261_MIN_TX_POWER                         (-17)
#define SX1261_MAX_TX_POWER                         (15)

#define SX1262_MIN_TX_POWER                         (-9)
#define SX1262_MAX_TX_POWER                         (22)


#define SX126X_TUS_IN_SEC                           (64000UL)
#define SX126X_US_IN_SEC                            (1000000UL)

#define SX126X_SIDEWALK_FSK_CS_DURATION_US          (RNET_FSK_DEFAULT_CS_DURATION_MUS)    /*!< Carrier Sense timeout for FSK link as defined by Sidewalk specification (1200us) */
#define SX126X_SIDEWALK_FSK_CS_RSSI_THRESHOLD       (RNET_FSK_DEFAULT_CAD_RSSI_THRESHOLD) /*!< Carrier Sense RSSI threshold in dBm - any signal above that level means the radio channel is busy */

#define SX126X_PA_CFG_DB_MULT                       (100)     /*!< Multiplier to conduct integer calculations with more precision */

#define SX1261_DEFAULT_OCP_VAL                      (0x18u)
#define SX1262_DEFAULT_OCP_VAL                      (0x38u)

/* Exported macro ------------------------------------------------------------*/

#define SX126X_TUS_TO_US(_TU_)                      ((((_TU_) * SX126X_US_IN_SEC) + ((SX126X_TUS_IN_SEC + 1u) / 2u)) / SX126X_TUS_IN_SEC)
#define SX126X_US_TO_TUS(_US_)                      ((((_US_) * SX126X_TUS_IN_SEC) + (SX126X_US_IN_SEC / 2u)) / SX126X_US_IN_SEC)

#define CONFIG_RADIO_GAIN_INT_TO_DB(__X__)          ((__X__) / (SX126X_PA_CFG_DB_MULT))
#define CONFIG_RADIO_GAIN_DB_TO_INT(__X__)          ((__X__) * (SX126X_PA_CFG_DB_MULT))

/* Exported types ------------------------------------------------------------*/

typedef enum {
    SX126X_TCXO_CTRL_NONE = 0, /*!< TCXO is not used, the HF clock source is XTAL */
    SX126X_TCXO_CTRL_VDD  = 1, /*!< TCXO is powered directly from VDD and is permanently powered up. */
    SX126X_TCXO_CTRL_DIO3 = 2, /*!< TCXO is controlled by the DIO3 pin of SX126x */
} sx126x_tcxo_ctrl_t;

typedef struct radio_sx126x_pa_cfg {
    uint8_t            pa_duty_cycle;
    uint8_t            hp_max;
    uint8_t            pa_lut;
    int8_t             tx_power_reg;    /*!< The value to be written into the SubGHz register - this may differ from the target Tx power if optimized PA config is applied */
    int16_t            target_tx_power; /*!< The actual Tx power targeted by the PA configuration */
    sx126x_ramp_time_t ramp_time;
#if SX126X_RADIO_CFG_USE_EXTERNAL_PA
    uint8_t            use_ext_pa;
#endif /* SX126X_RADIO_CFG_USE_EXTERNAL_PA */
} radio_sx126x_pa_dynamic_cfg_t;

typedef struct radio_sx126x_regional_param {
    uint8_t param_region;
    int8_t  max_tx_power[SID_PAL_RADIO_DATA_RATE_MAX_NUM];
    int8_t  cca_level_adjust[SID_PAL_RADIO_DATA_RATE_MAX_NUM];
    int16_t ant_dbi;
} radio_sx126x_regional_param_t;

typedef struct radio_sx126x_regional_config {
   uint8_t                               radio_region;
   uint8_t                               reg_param_table_size;
   const radio_sx126x_regional_param_t * reg_param_table;
} radio_sx126x_regional_config_t;

/**
 * @brief Application-specific processing times to initiate Tx or Rx
 *
 * @attention Tx/Rx processing time is delay between the software starts to configure the transceiver for Tx/Rx and the actual time the preamble starts (for Tx)
 *            or the radio starts to listen (for Rx). This processing time covers all the necessary actions (e.g., waking up the radio, configuring frequency,
 *            modulation, and packet parameters, interrupts, etc. and initiating a Tx or Rx). Sidewalk radio scheduler uses these values to compensate
 *            the processing delays and to ensure the actual radio operations are performed as close to the targeted time slots as possible.  Since processing
 *            time heavily depends on the selected MCU, its clock frequency, and other board- and application-specific factors, it is required to measure these
 *            processing timings for the final products before going in production.
 *            The driver provides some reference values for the processing delays for the evaluation boards (e.g., NUCLEO-WBAxx), but these values are intended for
 *            the evaluation purposes only. It is the responsibility of the developer re-measure these timings and apply the values that fit their product.
 */
typedef struct {
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
    struct {
        uint32_t tx_process_delay_us; /*!< Time required to initiate a LoRa (CSS) Tx, microseconds */
        uint32_t rx_process_delay_us; /*!< Time required to initiate a LoRa (CSS) Rx, microseconds */
    } lora;                           /*!< LoRa (CSS) link-specific processing delays */
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 */
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_2
    struct {
        uint32_t tx_process_delay_us; /*!< Time required to initiate an FSK Tx, microseconds */
        uint32_t rx_process_delay_us; /*!< Time required to initiate an FSK Rx, microseconds */
    } fsk;                            /*!< FSK link-specific processing delays */
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 */
} radio_sx126x_tx_rx_processing_timings_t;

/**
 * @brief A user-defined callback invoked by the radio driver whenever Tx power is configured. It allows the user app to override any default power amplifier (PA) settings based on application needs
 *
 * @param [in] desired_tx_power Desired Tx power (in dBm) that the radio driver targets to achieve for the transmitted signal (this is not equal to the output of SX126x, e.g. if an external PA is used the output of SX126x alone will be set to a lower value)
 * @param [in] regional_params  Regional configuration data that is currently used by the radio driver
 * @param [out] pa_cfg          Power amplifier configuration to apply. For any non-modified fields the default settings will be applied
 */
typedef int32_t (*radio_sx126x_get_pa_cfg_t)(const int32_t desired_tx_power, const radio_sx126x_regional_param_t * const regional_params, radio_sx126x_pa_dynamic_cfg_t * const pa_cfg);

typedef struct {
    uint8_t                         rx_boost_en;          /*!< Enables Rx boost in SX126x. This results in approx. +2dB Rx gain increase */
    uint8_t                         dio2_ctrl_en;         /*!< Use DIO2 pin of SX126x to control Rx/Tx mode selection on RF switch/FEM. DIO2 is high for Tx and low for Rx */
    const radio_sx126x_get_pa_cfg_t pa_cfg_callback;      /*!< User-defined callback that is triggered by the driver when PA or Tx power is being configured */
#if SX126X_RADIO_CFG_USE_EXTERNAL_PA                      /*!< This part is relevant only when an external RF Power Amplifier (PA) is mounted on PCB */
    int16_t                         tx_gain_dbi;          /*!< Tx gain of the external PA, expressed in cdBi (dBi * 100) */
    int16_t                         tx_bypass_loss;       /*!< Insertion loss when Tx PA is bypassed, expressed in cdBi (dBi * 100) */
    int16_t                         rx_gain_dbi;          /*!< Rx gain of the external LNA, expressed in cdBi (dBi * 100) */
#else                                                     /*!< This part is valid when just an RF switch with no active amplifiers is used */
    int16_t                         rf_sw_insertion_loss; /*!< RF switch insertion loss - applies when no external PA is used, just an RF switch, expressed in cdBi (dBi * 100) */
#endif /* SX126X_RADIO_CFG_USE_EXTERNAL_PA */
} radio_sx126x_pa_static_cfg_t;

typedef struct radio_sx126x_tcxo {
    sx126x_tcxo_ctrl_t          ctrl;         /*!< Supply voltage source for TCXO */
    sx126x_tcxo_ctrl_voltages_t ctrl_voltage; /*!< Supply voltage level. Valid only when TCXO is powered via SX126x's DIO3 pin */
    uint32_t                    timeout_us;   /*!< TCXO start time in microseconds */
} radio_sx126x_tcxo_t;

/**
 * @brief User-defined callback triggered by the driver to get XTAL trim capacitor value
 *        This callback can be used to provide individual calibrated values (e.g. from
 *         manufacturing data, EOL calibration, etc.).
 * @note  This callback is invoked only if XTAL is used. It's never called if TCXO is configured
 */
typedef int32_t (*radio_sx126x_get_mfg_trim_val_t)(uint16_t *trim);

/**
 * @brief User-defined callback called by the driver during initialization to setup SX126x's DIO3 pin.
 *        This callback can be used to define user-specific behavior for DIO3 pin if it is not used
 *        by the driver (e.g. react on specific radio IRQs, drive additional lines of the RF switch, etc.)
 * @note  This callback is invoked only if DIO3 is not used to drive TCXO
 */
typedef int32_t (*radio_sx126x_get_dio3_cfg_t)(void);

typedef struct {
    uint8_t                                  id;                         /*!< Specify which exact SX126x transceiver is used (e.g. SX1261, SX1262, etc.). SX126x family does not include a chip ID register, so this config must be provided by the user. IMPORTANT: wrong value may damage SX126x and there's no way to verify this setting in runtime */
    sx126x_reg_mod_t                         regulator_mode;             /*<! SX126x power regulator mode (SMPS or LDO). IMPORTANT: this setting should correspond to your HW design, otherwise SX126x can be damaged */

    struct {
        uint32_t                             radio_reset;                /*!< GPIO pin to control SX126x Reset line */
        uint32_t                             radio_irq;                  /*!< GPIO pin to monitor SX126x IRQ */
#if (__ARM_ARCH_6M__ == 0)
        uint8_t                              radio_irq_prio_high;        /*!< Priority for the radio IRQ line in NVIC that is applied for time-sensitive operations (e.g. capturing IRQ timestamp) */
        uint8_t                              radio_irq_prio_low;         /*!< Priority for the radio IRQ line in NVIC that is applied when no time-critical operations are expected. This priority should be configured in a way to allow RTOS API calls */
#else
        uint8_t                              radio_irq_prio;             /*!< Priority for the radio IRQ line in NVIC. For ARMv6-M (Cortex-M0/M0+/M1) the priority can be changed only when the corresponding IRQ is not active. Due to that the priority shall be set to a level that allows RTOS API calls */
#endif /* (__ARM_ARCH_6M__ == 0) */
        uint32_t                             radio_busy;                 /*!< GPIO pin to monitor SX126x Busy signal */

#if SX126X_RADIO_CFG_USE_FEM_PWR_CTRL
        uint32_t                             fem_power;                  /*!< Use this pin to power on and shutdown FEM/RF switch */
#endif /* SX126X_RADIO_CFG_USE_FEM_PWR_CTRL */
#if SX126X_RADIO_CFG_USE_TX_RX_CTRL
        uint32_t                             fem_tx_rx_mode;             /*!< Select signal direction in FEM/RF switch (Tx or Rx) */
#endif /* SX126X_RADIO_CFG_USE_TX_RX_CTRL */
#if SX126X_RADIO_CFG_USE_EXTERNAL_PA && SX126X_RADIO_CFG_USE_FEM_BYPASS_CTRL
        uint32_t                             fem_bypass;                 /*!< Enables Tx bypass on FEM/PA, connecting SX126x's built-in PA to the antenna directly */
#endif /* SX126X_RADIO_CFG_USE_EXTERNAL_PA && SX126X_RADIO_CFG_USE_FEM_BYPASS_CTRL */
#if SX126X_RADIO_CFG_USE_FEM_PWR_CTRL
        uint8_t                              fem_power_en_gpio_state;    /*!< State of the MCU GPIO pin to turn FEM/RF switch on */
#endif /* SX126X_RADIO_CFG_USE_FEM_PWR_CTRL */
#if SX126X_RADIO_CFG_USE_TX_RX_CTRL
        uint8_t                              fem_tx_mode_sel_gpio_state; /*!< State of the MCU GPIO pin to select Tx mode in FEM/RF switch */
#endif /* SX126X_RADIO_CFG_USE_TX_RX_CTRL */
#if SX126X_RADIO_CFG_USE_EXTERNAL_PA && SX126X_RADIO_CFG_USE_FEM_BYPASS_CTRL
        uint8_t                              fem_bypass_en_gpio_state;   /*!< State of the MCU GPIO pin to activate bypass mode */
#endif /* SX126X_RADIO_CFG_USE_EXTERNAL_PA && SX126X_RADIO_CFG_USE_FEM_BYPASS_CTRL */

#if SX126X_RADIO_CFG_USE_STATUS_LED
        uint32_t                             tx_led;
        uint32_t                             rx_led;
        uint8_t                              tx_led_on_gpio_state;       /*!< State of the MCU GPIO pin to turn LED on */
        uint8_t                              rx_led_on_gpio_state;       /*!< State of the MCU GPIO pin to turn LED on */
#endif /* SX126X_RADIO_CFG_USE_STATUS_LED */
    }                                        gpio;                       /*!< MCU GPIO pin configuration */
    const radio_sx126x_get_dio3_cfg_t        dio3_cfg_callback;

    radio_sx126x_pa_static_cfg_t             pa_config;

    radio_sx126x_tcxo_t                      tcxo_config;
    const radio_sx126x_get_mfg_trim_val_t    trim_cap_val_callback;

    const struct sid_pal_serial_bus_factory *bus_factory;
    struct sid_pal_serial_bus_client         bus_selector;

    struct {
        uint8_t *p;
        size_t   size;
    } internal_buffer;

    sid_pal_radio_state_transition_timings_t state_timings;
    radio_sx126x_tx_rx_processing_timings_t  processing_timings;
    radio_sx126x_regional_config_t           regional_config;
} sx126x_radio_device_config_t;

/** @brief Set Semtech radio config parameters
 *
 *  @param pointer to sx126x radio device config
 */
void sx126x_radio_set_device_config(const sx126x_radio_device_config_t * const cfg);

#ifdef __cplusplus
}
#endif

#endif /* __SID_STM32_SX126X_CONFIG_H_ */
