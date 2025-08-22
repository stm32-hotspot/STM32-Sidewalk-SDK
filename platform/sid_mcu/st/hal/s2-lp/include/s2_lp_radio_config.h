/**
  ******************************************************************************
  * @file    s2_lp_radio_config.h
  * @brief   Sub-GHz radio configuration for Sidewalk application
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

#ifndef __HALO_S2_LP_RADIO_CONFIG_H_
#define __HALO_S2_LP_RADIO_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Radio interfaces */
#include "s2_lp_ic_definitions.h"
#include "s2_lp_radio_default_timings.h"

/* Sidewalk interfaces */
#include <sid_pal_radio_ifc.h>
#include <sid_pal_serial_bus_ifc.h>
#include <sid_pal_serial_bus_spi_config.h>

/* Exported constants --------------------------------------------------------*/

#define SID_RADIO_PLATFORM_S2LP

#define HALO_GPIO_NOT_CONNECTED                   (0u)

#ifndef S2LP_RADIO_CFG_USE_STATUS_LED
#  define S2LP_RADIO_CFG_USE_STATUS_LED           (1u) /*!< Allows driver status indication with an LED */
#endif /* S2LP_RADIO_CFG_USE_STATUS_LED */

#ifndef S2LP_RADIO_CFG_USE_EXTERNAL_PA
#  define S2LP_RADIO_CFG_USE_EXTERNAL_PA          (1u) /*!< Set this to 1 if your setup has an external power amplifier for S2-LP. The reference implementation drives SKY66420 FEM */
#endif /* S2LP_RADIO_CFG_USE_EXTERNAL_PA */

#ifndef S2LP_RADIO_CFG_IRQ_PROCESS_GUARD_TIME_MS
# define S2LP_RADIO_CFG_IRQ_PROCESS_GUARD_TIME_MS (2u) /*!< Acceptable delay (in ms) between radio IRQ detection and actual processing. Increase this allowance if radio IRQ processing gets delayed in your application (e.g. by higher priority BLE radio IRQ) */
#endif /* S2LP_RADIO_CFG_IRQ_PROCESS_GUARD_TIME_MS */

#define RADIO_REGION_NA                           (SID_PAL_RADIO_RC_NA)
#define RADIO_REGION_EU                           (SID_PAL_RADIO_RC_EU)
#define RADIO_REGION_NONE                         (SID_PAL_RADIO_RC_NONE)

#define S2LP_PA_CFG_DB_MULT                       (100) /*!< Multiplier to conduct integer calculations with more precision */

/* Exported macro ------------------------------------------------------------*/

#define CONFIG_RADIO_GAIN_INT_TO_DB(__X__)        ((__X__) / (S2LP_PA_CFG_DB_MULT))
#define CONFIG_RADIO_GAIN_DB_TO_INT(__X__)        ((__X__) * (S2LP_PA_CFG_DB_MULT))

/* Exported types ------------------------------------------------------------*/

typedef struct {
#if S2LP_RADIO_CFG_USE_EXTERNAL_PA /*!< This part is relevant only when an external RF Power Amplifier (PA) is mounted on PCB */
    int16_t  tx_gain_dbi;          /*!< Tx gain of the external PA, expressed in cdBi (dBi * 100) */
    int16_t  tx_bypass_loss;       /*!< Insertion loss when Tx PA is bypassed, expressed in cdBi (dBi * 100) */
    int16_t  rx_gain_dbi;          /*!< Rx gain of the external LNA, expressed in cdBi (dBi * 100) */
#else
    int16_t  rf_sw_insertion_loss; /*!< RF switch insertion loss - applies when no external PA is used, just an RF switch, expressed in cdBi (dBi * 100) */
#endif /* S2LP_RADIO_CFG_USE_EXTERNAL_PA */
} s2_lp_radio_pa_static_cfg_t;

typedef struct {
    int8_t   tx_power;       /*!< Desired Tx power in dBi */
    uint32_t ramp_time_us;   /*!< Desired Tx PA ramp up time in us */
} s2_lp_radio_pa_dynamic_cfg_t;

typedef struct {
    uint8_t  param_region;
    int8_t   max_tx_power[SID_PAL_RADIO_DATA_RATE_MAX_NUM];
    int8_t   cca_level_adjust[SID_PAL_RADIO_DATA_RATE_MAX_NUM];
    int16_t  ant_dbi;
    uint32_t base_frequency_hz;
    uint32_t channel_spacing_hz;
} s2_lp_radio_regional_params_t;

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
    uint32_t tx_process_delay_us; /*!< Time required to initiate an FSK Tx, microseconds */
    uint32_t rx_process_delay_us; /*!< Time required to initiate an FSK Rx, microseconds */
} s2_lp_radio_tx_rx_processing_timings_t;

/**
 * @brief A user-defined callback invoked by the radio driver whenever Tx power is configured. It allows the user app to override any default power amplifier (PA) settings based on application needs
 *
 * @param [in] desired_tx_power Desired Tx power (in dBm) that the radio driver targets to achieve for the transmitted signal (this is not equal to the output of S2-LP, e.g. if an external PA is used the output of S2-LP alone will be set to a lower value)
 * @param [in] regional_params  Regional configuration data that is currently used by the radio driver
 * @param [out] pa_cfg          Power amplifier configuration to apply. For any non-modified fields the default settings will be applied
 */
typedef int32_t (*s2_lp_radio_get_pa_cfg)(const int32_t desired_tx_power, const s2_lp_radio_regional_params_t * const regional_params, s2_lp_radio_pa_dynamic_cfg_t * const pa_cfg);

typedef struct {
    uint32_t                              radio_region;
    uint32_t                              reg_param_table_size;
    const s2_lp_radio_regional_params_t * reg_param_table;
} s2_lp_radio_regional_config_t;

typedef struct {
    const s2_lp_radio_get_pa_cfg pa_cfg_callback;

    const sid_pal_serial_bus_factory_t * const bus_factory;
    const sid_pal_serial_bus_client_t * const  spi_client_cfg;

    const struct {
        uint32_t radio_shutdown;                                        /*!< MCU pin that is connected to the SDN line of the S2-LP */
        struct {
            uint32_t            mcu_pin;                                /*!< IRQ input pin from the host MCU side */
            s2_lp_ic_gpio_pin_t s2lp_pin;                               /*!< IRQ output pin from the S2-LP side */
#if (__ARM_ARCH_6M__ == 0)
            uint8_t             prio_high;                              /*!< Priority for the radio IRQ line in NVIC that is applied for time-sensitive operations (e.g. capturing IRQ timestamp) */
            uint8_t             prio_low;                               /*!< Priority for the radio IRQ line in NVIC that is applied when no time-critical operations are expected. This priority shall be configured in a way to allow RTOS API calls */
#else
            uint8_t             prio;                                   /*!< Priority for the radio IRQ line in NVIC. For ARMv6-M (Cortex-M0/M0+/M1) the priority can be changed only when the corresponding IRQ is not active. Due to that the priority shall be set to a level that allows RTOS API calls */
#endif /* (__ARM_ARCH_6M__ == 0) */
        }        radio_irq;                                             /*!< Configuration for the radio IRQ line */
#if S2LP_RADIO_CFG_USE_EXTERNAL_PA /*!< This part is relevant only when an external RF Power Amplifier (PA) is mounted on PCB. Reference implementation is based on SKY66420 FEM */
        uint32_t rf_fem_csd;                                            /*!< MCU pin that is connected to the CSD (Shutdown Control) pin of the SKY66420 FEM */
        uint32_t rf_fem_ctx;                                            /*!< MCU pin that is connected to the CTX (Transmit Mode Select) pin of the SKY66420 FEM */
        uint32_t rf_fem_cps;                                            /*!< MCU pin that is connected to the CPS (Bypass Mode Select) pin of the SKY66420 FEM */
#endif /* S2LP_RADIO_CFG_USE_EXTERNAL_PA */
#if S2LP_RADIO_CFG_USE_STATUS_LED  /*!< This part is relevant only if there's an LED available on  PCB to be controlled by the driver */
        uint32_t                               tx_led;
        uint32_t                               tx_led_on_gpio_state;    /*!< State of the MCU GPIO pin to turn LED on */
        uint32_t                               rx_led;
        uint32_t                               rx_led_on_gpio_state;    /*!< State of the MCU GPIO pin to turn LED on */
#endif /* S2LP_RADIO_CFG_USE_STATUS_LED */
    } gpio;                                                             /*!< GPIO lines required to drive the S2-LP radio */

    const uint32_t                                 use_external_clock;  /*!< FALSE - use XTAL connected to the XIN and XOUT pins; TRUE: use external clock source connected to the XIN (e.g. TCXO) */
    const uint32_t                                 xin_freq;            /*!< Frequency (in Hz) of the XTAL/TCXO/External Clock that feeds S2-LP - this value is board-dependent. Valid options are 24MHz, 25MH, 26Mhz and 48MHz, 50Mhz, 52MHz. Other frequencies are not supported */

    const s2_lp_radio_pa_static_cfg_t              pa_config;

    const s2_lp_radio_regional_config_t            regional_config;     /*!< Regional parameters and limitations for the physical layer */

    const sid_pal_radio_state_transition_timings_t state_timings;       /*!< Transition timings for the radio (e.g. from sleep to Tx/Rx). These are dependent on numerous factors: MCU clock, SPI speed, S2-LP response time, etc. If any changes are made to the reference implementation these timings shall be adjusted accordingly */
    const s2_lp_radio_tx_rx_processing_timings_t   processing_timings;  /*!< Application-specific Tx/Rx initiation delays */

    struct {
        uint8_t *p;
        size_t   size;
    } internal_buffer;
} s2_lp_radio_device_config_t;

/* Exported functions --------------------------------------------------------*/

/** @brief Set S2-LP radio config parameters
 *  @param [in] cfg Pointer to the SubGHz radio config
 */
void s2_lp_radio_set_device_config(const s2_lp_radio_device_config_t * const cfg);

#ifdef __cplusplus
}
#endif

#endif /* __HALO_S2_LP_RADIO_CONFIG_H_ */
