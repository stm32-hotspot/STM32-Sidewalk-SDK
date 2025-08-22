/**
  ******************************************************************************
  * @file    stm32wlxx_app_radio_config.h
  * @brief   Sub-GHz radio configuration for Sidewalk application
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

#ifndef __HALO_STM32WLXX_APP_RADIO_CONFIG_H_
#define __HALO_STM32WLXX_APP_RADIO_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include <stdbool.h>
#include <stdint.h>

/* Sidewalk interfaces */
#include <sid_pal_radio_ifc.h>
#include <sid_pal_serial_bus_ifc.h>
#include <sid_pal_serial_bus_spi_config.h>

/* Radio driver interfaces */
#include "stm32wlxx_radio_default_timings.h"
#include <comm_def.h>

/* Exported constants --------------------------------------------------------*/

#define SID_RADIO_PLATFORM_STM32WLXX_APP

#define HALO_GPIO_NOT_CONNECTED                    (0u)

#ifndef STM32WLxx_RADIO_CFG_USE_STATUS_LED       /*!< Allows driver status indication with an LED */
#  define STM32WLxx_RADIO_CFG_USE_STATUS_LED       (1u)
#endif /* STM32WLxx_RADIO_CFG_USE_STATUS_LED */

#define RADIO_STM32WLxx_RAMP_10_US                 (0x00u)
#define RADIO_STM32WLxx_RAMP_20_US                 (0x01u)
#define RADIO_STM32WLxx_RAMP_40_US                 (0x02u)
#define RADIO_STM32WLxx_RAMP_80_US                 (0x03u)
#define RADIO_STM32WLxx_RAMP_200_US                (0x04u)
#define RADIO_STM32WLxx_RAMP_800_US                (0x05u)
#define RADIO_STM32WLxx_RAMP_1700_US               (0x06u)
#define RADIO_STM32WLxx_RAMP_3400_US               (0x07u)

#define RADIO_STM32WLxx_LPA_UPPER_LIMIT_DBM        (15)     /*!< The upper limit of the built-in LPA in dBm */
#define RADIO_STM32WLxx_LPA_LOWER_LIMIT_DBM        (-17)    /*!< The lower limit of the built-in LPA in dBm */
#define RADIO_STM32WLxx_HPA_UPPER_LIMIT_DBM        (22)     /*!< The upper limit of the built-in HPA in dBm */
#define RADIO_STM32WLxx_HPA_LOWER_LIMIT_DBM        (-9)     /*!< The lower limit of the built-in HPA in dBm */

#define RADIO_REGION_NA                            (SID_PAL_RADIO_RC_NA)
#define RADIO_REGION_EU                            (SID_PAL_RADIO_RC_EU)
#define RADIO_REGION_NONE                          (SID_PAL_RADIO_RC_NONE)

#ifndef STM32WLxx_RADIO_APP_LPM_SUPPORT
#  define STM32WLxx_RADIO_APP_LPM_SUPPORT          (1u)     /*!< 0 - LPM on STM32WLxx side is not supported, 1 - STM32WLxx may enter LPM and the driver needs to perform wakeup procedures whenever necessary */
#endif /* STM32WLxx_RADIO_APP_LPM_SUPPORT */

#define STM32WLxx_PA_CFG_DB_MULT                   (100)    /*!< Multiplier to conduct integer calculations with more precision */

/* Exported macro ------------------------------------------------------------*/

#define CONFIG_RADIO_GAIN_INT_TO_DB(__X__)          ((__X__) / (STM32WLxx_PA_CFG_DB_MULT))
#define CONFIG_RADIO_GAIN_DB_TO_INT(__X__)          ((__X__) * (STM32WLxx_PA_CFG_DB_MULT))

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Power amplifier configuration
 */
typedef struct {
    uint8_t pa_duty_cycle;
    uint8_t hp_max;
    uint8_t device_sel;
    uint8_t pa_lut;
    int8_t  tx_power_reg;    /*!< The value to be written into the SubGHz register - this may differ from the target Tx power if optimized PA config is applied */
    uint8_t ramp_time;
    int8_t  target_tx_power; /*!< The actual Tx power targeted by the PA configuration */
} stm32wlxx_radio_pa_cfg_t;

typedef struct {
    uint8_t param_region;
    int8_t max_tx_power[SID_PAL_RADIO_DATA_RATE_MAX_NUM];
    int8_t cca_level_adjust[SID_PAL_RADIO_DATA_RATE_MAX_NUM];
    int16_t ant_dbi;
} stm32wlxx_radio_regional_param_t;

typedef struct {
    uint32_t radio_region;
    uint32_t reg_param_table_size;
    const stm32wlxx_radio_regional_param_t * reg_param_table;
} stm32wlxx_radio_regional_config_t;

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
} stm32wlxx_radio_tx_rx_processing_timings_t;

typedef int32_t (*stm32wlxx_radio_get_pa_cfg)(const int32_t desired_tx_power, const stm32wlxx_radio_regional_param_t * const regional_params, stm32wlxx_radio_pa_cfg_t * const out_pa_cfg);

/**
 * @brief User-defined function to get the trim capacitor selection
 *
 * @param [out] trim Store for the trim value
 * @return RADIO_ERROR_NONE on success.
 */
typedef int32_t (*radio_stm32wlxx_get_mfg_trim_val)(uint16_t *trim);

typedef struct {
    bool                                       rx_boost;
    int8_t                                     lna_gain;
    bool                                       enable_lpa;               /*!< Specifies if the driver can use Low-Power Amplifier (LPA) for Tx */
    bool                                       enable_hpa;               /*!< Specifies if the driver can use High-Power Amplifier (HPA) for Tx */
    const stm32wlxx_radio_get_pa_cfg           pa_cfg_callback;

    const sid_pal_serial_bus_factory_t * const bus_factory;
    const sid_pal_serial_bus_client_t * const  spi_client_cfg;

    struct {
        uint32_t                                   radio_irq;
#if (__ARM_ARCH_6M__ == 0)
        uint8_t                                    radio_irq_prio_high;  /*!< Priority for the radio IRQ line in NVIC that is applied for time-sensitive operations (e.g. capturing IRQ timestamp) */
        uint8_t                                    radio_irq_prio_low;   /*!< Priority for the radio IRQ line in NVIC that is applied when no time-critical operations are expected. This priority should be configured in a way to allow RTOS API calls */
#else
        uint8_t                                    radio_irq_prio;       /*!< Priority for the radio IRQ line in NVIC. For ARMv6-M (Cortex-M0/M0+/M1) the priority can be changed only when the corresponding IRQ is not active. Due to that the priority shall be set to a level that allows RTOS API calls */
#endif /* (__ARM_ARCH_6M__ == 0) */

#if STM32WLxx_RADIO_CFG_USE_STATUS_LED  /*!< This part is relevant only if there's an LED available on  PCB to be controlled by the driver */
        uint32_t                                   tx_led;
        uint32_t                                   tx_led_on_gpio_state; /*!< State of the MCU GPIO pin to turn LED on */
        uint32_t                                   rx_led;
        uint32_t                                   rx_led_on_gpio_state;  /*!< State of the MCU GPIO pin to turn LED on */
#endif /* STM32WLxx_RADIO_CFG_USE_STATUS_LED */
    }                                          gpio;

    sid_pal_radio_state_transition_timings_t   state_timings;
    stm32wlxx_radio_tx_rx_processing_timings_t processing_timings;

    struct {
        uint8_t *                                  p;
        size_t                                     size;
    }                                          internal_buffer;

    stm32wlxx_radio_regional_config_t          regional_config;
} stm32wlxx_app_radio_device_config_t;

/* Exported functions --------------------------------------------------------*/

/**
 * @brief Set STM32WLxx radio config parameters
 * @param [in] cfg Pointer to the SubGHz radio config
 */
void stm32wlxx_app_radio_set_device_config(const stm32wlxx_app_radio_device_config_t * const cfg);

#ifdef __cplusplus
}
#endif

#endif /* __HALO_STM32WLXX_APP_RADIO_CONFIG_H_ */
