/**
  ******************************************************************************
  * @file    temperature.c
  * @brief   STM32WBA platform routines to support sid_pal_temperature
  *          Sidewalk module
  * 
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

/* Includes ------------------------------------------------------------------*/

/* Sidewalk interfaces */
#include <sid_pal_temperature_ifc.h>
#include <sid_pal_log_ifc.h>

/* Platform interfaces */
#include "adc_ctrl.h"
#include "adc_ctrl_conf.h"
#include <sid_stm32_common_utils.h>
#include <stm32wbaxx_ll_adc.h>

/* App configuration */
#include "app_conf.h"
#include "utilities_conf.h"

/* Private defines -----------------------------------------------------------*/

/* Default temperature */
#define DEFAULT_TEMPERATURE         ((int16_t)25)

/* Private variables ---------------------------------------------------------*/

/**
 * @brief ADC Handle configuration for MCU temperature measurement
 * @note Keep this independent of Link Layer temperature measurement handle to avoid race conditions for ADC access and power control
 */
static ADCCTRL_Handle_t mcu_temp_meas_handle = {
    .Uid         = 0x00, /* Uid is assigned by ADC_Ctrl as a part of ADCCTRL_RegisterHandle() */
    .State       = ADCCTRL_HANDLE_NOT_REG,
    .InitConf    = {
        .ConvParams = {
            .TriggerFrequencyMode = LL_ADC_TRIGGER_FREQ_LOW,
            .Resolution           = LL_ADC_RESOLUTION_12B,
            .DataAlign            = LL_ADC_DATA_ALIGN_RIGHT,
            .TriggerStart         = LL_ADC_REG_TRIG_SOFTWARE,
            .TriggerEdge          = LL_ADC_REG_TRIG_EXT_RISING,
            .ConversionMode       = LL_ADC_REG_CONV_SINGLE,
            .DmaTransfer          = LL_ADC_REG_DMA_TRANSFER_NONE,
            .Overrun              = LL_ADC_REG_OVR_DATA_OVERWRITTEN,
            .SamplingTimeCommon1  = LL_ADC_SAMPLINGTIME_814CYCLES_5,
            .SamplingTimeCommon2  = LL_ADC_SAMPLINGTIME_1CYCLE_5,
        },
        .SeqParams = {
            .Setup                = LL_ADC_REG_SEQ_CONFIGURABLE,
            .Length               = LL_ADC_REG_SEQ_SCAN_DISABLE,
            .DiscMode             = LL_ADC_REG_SEQ_DISCONT_DISABLE,
        },
        .LowPowerParams = {
            .AutoPowerOff         = DISABLE,
            .AutonomousDPD        = LL_ADC_LP_AUTONOMOUS_DPD_DISABLE,
        }
    },
    .ChannelConf = {
        .Channel                  = LL_ADC_CHANNEL_TEMPSENSOR,
        .Rank                     = LL_ADC_REG_RANK_1,
        .SamplingTime             = LL_ADC_SAMPLINGTIME_COMMON_1,
    }
};

static int16_t last_measured_temp = DEFAULT_TEMPERATURE;

/* Global function definitions -----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_pal_temperature_init(void)
{
    sid_error_t err;
    ADCCTRL_Cmd_Status_t adctrl_status;

    /* It's assumed that ADCCTRL_Init() is called at application bootstrap and ADC_Ctrl module is fully configured when this function is reached */

    adctrl_status = ADCCTRL_RegisterHandle(&mcu_temp_meas_handle);
    if ((adctrl_status != ADCCTRL_OK) && (adctrl_status != ADCCTRL_HANDLE_ALREADY_REGISTERED))
    {
        SID_PAL_LOG_ERROR("Failed to initialize ADC for MCU temperature measurement. ADCCTRL error %u", (uint32_t)adctrl_status);
        err = SID_ERROR_IO_ERROR;
    }
    else
    {
        /* Done */
        err = SID_ERROR_NONE;
    };

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int16_t sid_pal_temperature_get(void)
{
    int16_t temperature_value;
    ADCCTRL_Cmd_Status_t adctrl_status;

    do
    {
        /* Request ADC IP activation */
        adctrl_status = ADCCTRL_RequestIpState(&mcu_temp_meas_handle, ADC_ON);
        if (adctrl_status != ADCCTRL_OK)
        {
            /* Failed to enable ADC */
            SID_PAL_LOG_ERROR("Failed to activate ADC for MCU temperature measurement. ADCCTRL error %u", (uint32_t)adctrl_status);
            temperature_value = last_measured_temp;
            break;
        }

        /* Get temperature from ADC dedicated channel */
        adctrl_status = ADCCTRL_RequestTemperature(&mcu_temp_meas_handle, &temperature_value);
        if (adctrl_status != ADCCTRL_OK)
        {
            /* Failed to measure the temperature, use previous value */
            if (adctrl_status != ADCCTRL_BUSY)
            {
                SID_PAL_LOG_ERROR("Failed to measure MCU temperature. ADCCTRL error %u", (uint32_t)adctrl_status);
            }
            temperature_value = last_measured_temp;
            /* Don't break here - we still need to deactivate the ADC */
        }
        else
        {
            if (LL_ADC_TEMPERATURE_CALC_ERROR == temperature_value)
            {
                /* Failed to measure the temperature, use previous value */
                temperature_value = last_measured_temp;
                SID_PAL_LOG_ERROR("Failed to measure MCU temperature - invalid calibration data");
                /* Don't break here - we still need to deactivate the ADC */
            }
            else
            {
                /* The measurement is good, update last known temperature reading */
                last_measured_temp = temperature_value;
            }
        }

        /* Request ADC IP deactivation */
        adctrl_status = ADCCTRL_RequestIpState(&mcu_temp_meas_handle, ADC_OFF);
        if (adctrl_status != ADCCTRL_OK)
        {
            SID_PAL_LOG_ERROR("Failed to deactivate ADC after MCU temperature measurement. ADCCTRL error %u", (uint32_t)adctrl_status);
        }
    } while (0);

    return temperature_value;
}
