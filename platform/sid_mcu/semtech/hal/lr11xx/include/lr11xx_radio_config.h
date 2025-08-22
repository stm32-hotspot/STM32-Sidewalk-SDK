/*
 * Copyright (c) 2019-2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * This file is exposed to each product configuration to customize the driver behavior
 */
/**
  ******************************************************************************
  * @file    lr11xx_radio_config.h
  * @brief   Semtech LR11xx Sidewalk driver config
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

#ifndef __SID_STM32_LR11XX_CONFIG_H_
#define __SID_STM32_LR11XX_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include <stdbool.h>
#include <stdint.h>
#include <time.h>

/* Semtech radio driver interfaces */
#include "lr11xx_gnss.h"
#include "lr11xx_radio_types.h"
#include "lr11xx_system_types.h"
#include "lr11xx_wifi.h"
#include <semtech_radio_ifc.h>

/* Sidewalk interfaces */
#include <sid_time_types.h>
#include <sid_fsk_phy_cfg.h>
#include <sid_pal_radio_ifc.h>
#include <sid_pal_serial_bus_ifc.h>

/* Radio driver interfaces */
#include "lr11xx_radio_default_timings.h"

/* Exported constants --------------------------------------------------------*/

#ifndef LR11XX_RADIO_CFG_USE_FEM_POW_ENA
#  define LR11XX_RADIO_CFG_USE_FEM_POW_ENA          (0u) /*!< Set this to 1 if your setup has an MCU pin to control the power of sub-GHz frontend */
#endif  /* LR11XX_RADIO_CFG_USE_FEM_POW_ENA */

#ifndef LR11XX_RADIO_CFG_USE_TX_RX_SW
#  define LR11XX_RADIO_CFG_USE_TX_RX_SW             (0u) /*!< Set this to 1 if your setup has a tx/rx direction switch pin for LR11xx. */
#endif  /* LR11XX_RADIO_CFG_USE_TX_RX_SW */

#ifndef LR11XX_RADIO_CFG_USE_HPA_LPA_SW
#  define LR11XX_RADIO_CFG_USE_HPA_LPA_SW           (0u) /*!< Set this to 1 if your setup has a low/high power amplifier switch pin for LR11xx. */
#endif  /* LR11XX_RADIO_CFG_USE_HPA_LPA_SW */

#ifndef LR11XX_RADIO_CFG_USE_FEM_BYPASS
#  define LR11XX_RADIO_CFG_USE_FEM_BYPASS           (0u) /*!< Set this to 1 if your setup has a tx bypass enable pin for LR11xx. */
#endif  /* LR11XX_RADIO_CFG_USE_FEM_BYPASS */

#ifndef LR11XX_RADIO_CFG_USE_EXTERNAL_PA
#  define LR11XX_RADIO_CFG_USE_EXTERNAL_PA          (0u) /*!< Set this to 1 if your setup has an external power amplifier for LR11xx. */
#endif /* LR11XX_RADIO_CFG_USE_EXTERNAL_PA */

#ifndef LR11XX_RADIO_CFG_USE_STATUS_LED
#  define LR11XX_RADIO_CFG_USE_STATUS_LED           (1u) /*!< Allows driver status indication with an LED */
#endif /* LR11XX_RADIO_CFG_USE_STATUS_LED */

#ifndef LR11XX_RADIO_CFG_IRQ_PROCESS_GUARD_TIME_MS
# define LR11XX_RADIO_CFG_IRQ_PROCESS_GUARD_TIME_MS (1u) /*!< Acceptable delay (in ms) between radio IRQ detection and actual processing. Increase this allowance if radio IRQ processing gets delayed in your application (e.g. by higher priority BLE radio IRQ) */
#endif /* LR11XX_RADIO_CFG_IRQ_PROCESS_GUARD_TIME_MS */

#ifndef LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION
#  define LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION     (0u) /*!< Include additional code to integrate LoRa Basic Modem support features */
#endif /* LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION */

#ifndef LR11XX_RADIO_CFG_GEOLOCATION_SUPPORT
#  define LR11XX_RADIO_CFG_GEOLOCATION_SUPPORT      (LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION) /*!< Include additional code to support LR11xx geolocation features */
#endif /* LR11XX_RADIO_CFG_GEOLOCATION_SUPPORT */

#ifndef LR11XX_RADIO_CFG_WIFI_SCAN_USE_BLANKING
#  define LR11XX_RADIO_CFG_WIFI_SCAN_USE_BLANKING   (0u) /*!< Enable blanking of Sidewalk operations (e.g. simulating Rx timeouts) whenever LoRa Basics Modem runs WiFi scanning. WARNING: this is not the recommended way of integration, use with caution */
#endif /* LR11XX_RADIO_CFG_WIFI_SCAN_USE_BLANKING */

#ifndef LR11XX_RADIO_CFG_GNSS_SCAN_USE_BLANKING
#  define LR11XX_RADIO_CFG_GNSS_SCAN_USE_BLANKING   (0u) /*!< Enable blanking of Sidewalk operations (e.g. simulating Rx timeouts) whenever LoRa Basics Modem runs GNSS scanning. WARNING: this is not the recommended way of integration, use with caution */
#endif /* LR11XX_RADIO_CFG_GNSS_SCAN_USE_BLANKING */

#define LR11XX_RADIO_CFG_LBM_CONCURRENCY_SUPPORT    (LR11XX_RADIO_CFG_WIFI_SCAN_USE_BLANKING || LR11XX_RADIO_CFG_GNSS_SCAN_USE_BLANKING) /*!< Enable support for Sidewalk and LoRa Basics Modem concurrency. WARNING: this is not the recommended way of integration, use with caution */

#if LR11XX_RADIO_CFG_GEOLOCATION_SUPPORT && !LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION
#  error "LR11xx geolocation features require LoRa Basic Modem (LBM) as prerequisite. Set LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION to 1 to enable LBM support"
#endif /* LR11XX_RADIO_CFG_GEOLOCATION_SUPPORT && !LR11XX_RADIO_CFG_SMTC_LBM_INTEGRATION */

#if LR11XX_RADIO_CFG_WIFI_SCAN_USE_BLANKING && !LR11XX_RADIO_CFG_LBM_CONCURRENCY_SUPPORT
#  error "Inconsistent LoRa Basics Modem (LBM) bridge configuration - WiFi scan is configured to use Sidewalk blanking mode, but general concurrency of SIdewalk and LBM stacks is disabled"
#endif /* LR11XX_RADIO_CFG_WIFI_SCAN_USE_BLANKING && !LR11XX_RADIO_CFG_LBM_CONCURRENCY_SUPPORT */

#if LR11XX_RADIO_CFG_GNSS_SCAN_USE_BLANKING && !LR11XX_RADIO_CFG_LBM_CONCURRENCY_SUPPORT
#  error "Inconsistent LoRa Basics Modem (LBM) bridge configuration - GNSS scan is configured to use Sidewalk blanking mode, but general concurrency of SIdewalk and LBM stacks is disabled"
#endif /* LR11XX_RADIO_CFG_GNSS_SCAN_USE_BLANKING && !LR11XX_RADIO_CFG_LBM_CONCURRENCY_SUPPORT */

#define SID_RADIO_PLATFORM_LR11XX

#define HALO_GPIO_NOT_CONNECTED                     (0u)

#define RADIO_REGION_NA                             (SID_PAL_RADIO_RC_NA)
#define RADIO_REGION_EU                             (SID_PAL_RADIO_RC_EU)
#define RADIO_REGION_NONE                           (SID_PAL_RADIO_RC_NONE)

#define LR11XX_TUS_IN_SEC                           (32768UL)
#define LR11XX_US_IN_SEC                            (1000000UL)

#define LR11XX_SIDEWALK_FSK_CS_DURATION_US          (RNET_FSK_DEFAULT_CS_DURATION_MUS)    /*!< Carrier Sense timeout for FSK link as defined by Sidewalk specification (1200us) */
#define LR11XX_SIDEWALK_FSK_CS_RSSI_THRESHOLD       (RNET_FSK_DEFAULT_CAD_RSSI_THRESHOLD) /*!< Carrier Sense RSSI threshold in dBm - any signal above that level means the radio channel is busy */

#define LR11XX_PA_CFG_DB_MULT                       (100)     /*!< Multiplier to conduct integer calculations with more precision */

/* Exported macro ------------------------------------------------------------*/

#define LR11XX_TUS_TO_US(_TU_)                      ((((_TU_) * LR11XX_US_IN_SEC) + ((LR11XX_TUS_IN_SEC + 1u) / 2u)) / LR11XX_TUS_IN_SEC)
#define LR11XX_US_TO_TUS(_US_)                      ((((_US_) * LR11XX_TUS_IN_SEC) + (LR11XX_US_IN_SEC / 2u)) / LR11XX_US_IN_SEC)

#define CONFIG_RADIO_GAIN_INT_TO_DB(__X__)          ((__X__) / (LR11XX_PA_CFG_DB_MULT))
#define CONFIG_RADIO_GAIN_DB_TO_INT(__X__)          ((__X__) * (LR11XX_PA_CFG_DB_MULT))

/* Exported types ------------------------------------------------------------*/

typedef struct radio_lr11xx_pa_cfg {
    lr11xx_radio_pa_cfg_t    pa_cfg;
    lr11xx_radio_ramp_time_t ramp_time;
    int16_t                  target_tx_power;
    int8_t                   tx_power_reg;    /*!< The value to be written into the SubGHz register - this may differ from the target Tx power if optimized PA config is applied */
#if LR11XX_RADIO_CFG_USE_EXTERNAL_PA && LR11XX_RADIO_CFG_USE_FEM_BYPASS
    bool                     enable_ext_pa;
#endif
} radio_lr11xx_pa_cfg_t;

typedef struct radio_lr11xx_regional_param {
    uint8_t param_region;
    int8_t max_tx_power[SID_PAL_RADIO_DATA_RATE_MAX_NUM];
    int8_t cca_level_adjust[SID_PAL_RADIO_DATA_RATE_MAX_NUM];
    int16_t ant_dbi;
} radio_lr11xx_regional_param_t;

typedef struct radio_lr11xx_regional_config {
   uint8_t radio_region;
   uint8_t reg_param_table_size;
   const radio_lr11xx_regional_param_t *reg_param_table;
} radio_lr11xx_regional_config_t;

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
} radio_lr11xx_tx_rx_processing_timings_t;

typedef struct {
#if LR11XX_RADIO_CFG_USE_EXTERNAL_PA                      /*!< This part is relevant only when an external RF Power Amplifier (PA) is mounted on PCB */
    int16_t                         tx_gain_dbi;          /*!< Tx gain of the external PA, expressed in cdBi (dBi * 100) */
    int16_t                         tx_bypass_loss;       /*!< Insertion loss when Tx PA is bypassed, expressed in cdBi (dBi * 100) */
    int16_t                         rx_gain_dbi;          /*!< Rx gain of the external LNA, expressed in cdBi (dBi * 100) */
#else                                                     /*!< This part is valid when just an RF switch with no active amplifiers is used */
    int16_t                         rf_sw_insertion_loss; /*!< RF switch insertion loss - applies when no external PA is used, just an RF switch, expressed in cdBi (dBi * 100) */
#endif /* LR11XX_RADIO_CFG_USE_EXTERNAL_PA */
} radio_lr11xx_pa_static_cfg_t;

typedef enum {
    LR11XX_TCXO_CTRL_NONE  = 0, /*!< TCXO is not used, the HF clock source is XTAL */
    LR11XX_TCXO_CTRL_VDD   = 1, /*!< TCXO is powered directly from VDD and is permanently powered up. */
    LR11XX_TCXO_CTRL_VTCXO = 2, /*!< TCXO is controlled by the VTCXO pin of LR11xx */
} lr11xx_tcxo_ctrl_t;

/**
 * @brief A user-defined callback invoked by the radio driver whenever Tx power is configured. It allows the user app to override any default power amplifier (PA) settings based on application needs
 *
 * @param [in] desired_tx_power Desired Tx power (in dBm) that the radio driver targets to achieve for the transmitted signal (this is not equal to the output of LR11xx, e.g. if an external PA is used the output of LR11xx alone will be set to a lower value)
 * @param [in] regional_params  Regional configuration data that is currently used by the radio driver
 * @param [out] pa_cfg          Power amplifier configuration to apply. For any non-modified fields the default settings will be applied
 */
typedef int32_t (*radio_lr11xx_get_pa_cfg_t)(const int32_t desired_tx_power, const radio_lr11xx_regional_param_t * const regional_params, radio_lr11xx_pa_cfg_t * const pa_cfg);

typedef struct {
    lr11xx_system_reg_mode_t                regulator_mode;
    bool                                    rx_boost;
    bool    enable_lpa; /*!< Specifies if the driver can use Low-Power Amplifier (LPA) for Tx */
    bool    enable_hpa; /*!< Specifies if the driver can use High-Power Amplifier (HPA) for Tx */
    const radio_lr11xx_get_pa_cfg_t         pa_cfg_callback;
    radio_lr11xx_pa_static_cfg_t            pa_config;

    struct {
        uint32_t                            radio_reset;
        uint32_t                            radio_irq;
#if (__ARM_ARCH_6M__ == 0)
        uint8_t                             radio_irq_prio_high; /*!< Priority for the radio IRQ line in NVIC that is applied for time-sensitive operations (e.g. capturing IRQ timestamp) */
        uint8_t                             radio_irq_prio_low;  /*!< Priority for the radio IRQ line in NVIC that is applied when no time-critical operations are expected. This priority should be configured in a way to allow RTOS API calls */
#else
        uint8_t                             radio_irq_prio;      /*!< Priority for the radio IRQ line in NVIC. For ARMv6-M (Cortex-M0/M0+/M1) the priority can be changed only when the corresponding IRQ is not active. Due to that the priority shall be set to a level that allows RTOS API calls */
#endif /* (__ARM_ARCH_6M__ == 0) */
        uint32_t                            radio_busy;
        uint32_t                            fem_power;
        uint32_t                            fem_power_en_gpio_state;
        uint32_t                            fem_bypass;
        uint32_t                            fem_bypass_en_gpio_state;
        uint32_t                            fem_tx_rx_mode;
        uint32_t                            fem_tx_mode_sel_gpio_state;
        uint32_t                            hpa_lpa_sw;
        uint32_t                            hpa_lpa_sw_on_gpio_state;   /*!< State of the MCU GPIO pin to turn switch on */
#if LR11XX_RADIO_CFG_GEOLOCATION_SUPPORT
        uint32_t                            gnss_lna_ctrl;
        uint32_t                            gnss_lna_ctrl_en_gpio_state;   /*!< State of the MCU GPIO pin to turn GNSS LNA on */
# endif /* LR11XX_RADIO_CFG_GEOLOCATION_SUPPORT */

#if LR11XX_RADIO_CFG_USE_STATUS_LED
        uint32_t                            tx_led;
        uint32_t                            tx_led_on_gpio_state;    /*!< State of the MCU GPIO pin to turn LED on */
        uint32_t                            rx_led;
        uint32_t                            rx_led_on_gpio_state;    /*!< State of the MCU GPIO pin to turn LED on */
#  if LR11XX_RADIO_CFG_GEOLOCATION_SUPPORT
        uint32_t                            gnss_scan_led;
        uint32_t                            gnss_scan_led_on_gpio_state;    /*!< State of the MCU GPIO pin to turn LED on */
        uint32_t                            wifi_scan_led;
        uint32_t                            wifi_scan_led_on_gpio_state;    /*!< State of the MCU GPIO pin to turn LED on */
#  endif /* LR11XX_RADIO_CFG_GEOLOCATION_SUPPORT */
#endif /* LR11XX_RADIO_CFG_USE_STATUS_LED */
    }                                       gpio;

    lr11xx_system_lfclk_cfg_t               lfclock_cfg;
    struct {
        lr11xx_tcxo_ctrl_t                  ctrl;                    /*!< Supply voltage source for TCXO */
        lr11xx_system_tcxo_supply_voltage_t ctrl_voltage;            /*!< Supply voltage level. Valid only when TCXO is powered via LR11xx's VTCXO pin */
        uint32_t                            timeout_us;              /*!< TCXO start time in microseconds */
#if LR11XX_RADIO_CFG_GEOLOCATION_SUPPORT
        uint32_t                            gnss_timeout_us;         /*!< TCXO start time in microseconds for GNSS use cases (typically it will be longer than normal TCXO startup since GNSS puts stricter requirements for frequency stability)*/
# endif /* LR11XX_RADIO_CFG_GEOLOCATION_SUPPORT */
    }                                       tcxo_config;

    lr11xx_system_rfswitch_cfg_t            rfswitch;

    struct {
        void                                (*pre_hook)(void*arg);
        void                                (*post_hook)(void*arg);
        void                                *arg;
    }                                       wifi_scan;

    struct {
        void                                (*pre_hook)(void*arg);
        void                                (*post_hook)(void*arg);
        void                                *arg;
    }                                       gnss_scan;

    const struct sid_pal_serial_bus_factory *bus_factory;
    struct sid_pal_serial_bus_client        bus_selector;

    struct {
        uint8_t *p;
        size_t   size;
    } internal_buffer;

    sid_pal_radio_state_transition_timings_t state_timings;
    radio_lr11xx_tx_rx_processing_timings_t  processing_timings;
    radio_lr11xx_regional_config_t           regional_config;
} lr11xx_radio_device_config_t;

/**
 * Callback to notify when radio is about to switch to sleep state
 * The callback is called in software irq context. The user has to
 * switch context from software irq in this callback.
 */
typedef void (*sid_pal_radio_sleep_start_notify_handler_t)(struct sid_timespec * const wakeup_time);

/* Exported functions --------------------------------------------------------*/

/** @brief Set Semtech radio config parameters
 *
 *  @param pointer to lr11xx radio device config
 */
void lr11xx_radio_set_device_config(const lr11xx_radio_device_config_t * const cfg);

/** @brief Set callback to trigger external action when radio is going to sleep.
 *
 *  @param[in]   sid_pal_radio_action_in_sleep_handler_t callback to handle
 *  entering radio sleep mode. Accepts NULL if no action is required.
 *
 *  @retval  On success RADIO_ERROR_NONE.
 */
int32_t sid_hal_set_sleep_start_notify_cb(sid_pal_radio_sleep_start_notify_handler_t callback);

/** @brief Get tx power range the semtech chip supports
 *  This API is used by diagnostics firmware only.
 *  This is used to determine the max tx power the chip supports so that the diag
 *  firmware will not exceed the max tx power setting the chip supports
 *
 *  @param  max tx power to be populated.
 *  @param  min tx power to be populated.
 *  @return On success RADIO_ERROR_NONE, on error a negative number is returned
 */
int32_t get_lr11xx_tx_power_range(int8_t *max_tx_power, int8_t *min_tx_power);

#ifdef __cplusplus
}
#endif

#endif /* __SID_STM32_LR11XX_CONFIG_H_ */
