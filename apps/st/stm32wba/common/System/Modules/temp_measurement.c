/**
  ******************************************************************************
  * @file    temp_measurement.c
  * @author  MCD Application Team
  * @brief   Temp measurement module
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

/* Includes ------------------------------------------------------------------*/

/* Common utilties */
#include "utilities_common.h"

#if (USE_TEMPERATURE_BASED_RADIO_CALIBRATION == 1)
/* Own header file */
#include "temp_measurement.h"
/* ADC Controller module */
#include "adc_ctrl.h"
#include "adc_ctrl_conf.h"

/* Link layer interfaces */
#include "ll_intf.h"
#include "ll_intf_cmn.h"

/* Private defines -----------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Global variables ----------------------------------------------------------*/
/* Error Handler */
extern void Error_Handler(void);

/* Private function prototypes -----------------------------------------------*/
/* Functions Definition ------------------------------------------------------*/
TEMPMEAS_Cmd_Status_t TEMPMEAS_Init (void)
{
  TEMPMEAS_Cmd_Status_t error = TEMPMEAS_UNKNOWN;

  ADCCTRL_Cmd_Status_t eReturn = ADCCTRL_UNKNOWN;

  eReturn = ADCCTRL_RegisterHandle (&LLTempRequest_Handle);

  if ((ADCCTRL_HANDLE_ALREADY_REGISTERED == eReturn) ||
      (ADCCTRL_OK == eReturn))
  {
    error = TEMPMEAS_OK;
  }
  else
  {
    error = TEMPMEAS_ADC_INIT;
  }

  return error;
}

void TEMPMEAS_RequestTemperatureMeasurement (void)
{
  /*vvv PATCH STMC-485 vvvvvvvvvv*/
  ADCCTRL_Cmd_Status_t error;
  /*^^^ END OF PATCH STMC-485 ^^^*/
  int16_t temperature_value = 0;

  /*vvv PATCH STMC-485 vvvvvvvvvv*/
  /* There's no added value in critical section since ADC operates autonomously and ADC access conflicts are managed by ADC_Ctrl module */
  // /* Enter limited critical section : disable all the interrupts with priority higher than RCC one
  //  * Concerns link layer interrupts (high and SW low) or any other high priority user system interrupt
  //  */
  // UTILS_ENTER_LIMITED_CRITICAL_SECTION(RCC_INTR_PRIO<<4);
  /*^^^ END OF PATCH STMC-485 ^^^*/

  /* Request ADC IP activation */
  /*vvv PATCH STMC-485 vvvvvvvvvv*/
  // ADCCTRL_RequestIpState(&LLTempRequest_Handle, ADC_ON);
  error = ADCCTRL_RequestIpState(&LLTempRequest_Handle, ADC_ON);
  if (error != ADCCTRL_OK)
  {
    /* Can't enable ADC, skip LL temperature update */
    return;
  }
  /*^^^ END OF PATCH STMC-485 ^^^*/

  /* Get temperature from ADC dedicated channel */
  /*vvv PATCH STMC-485 vvvvvvvvvv*/
  // ADCCTRL_RequestTemperature (&LLTempRequest_Handle,
  //                             &temperature_value);
  error = ADCCTRL_RequestTemperature (&LLTempRequest_Handle,
                                      &temperature_value);
  if (error != ADCCTRL_OK)
  {
    /* Can't read MCU temperature from ADC, skip LL temperature update */
    return;
  }
  /*^^^ END OF PATCH STMC-485 ^^^*/

  /* Request ADC IP deactivation */
  /*vvv PATCH STMC-485 vvvvvvvvvv*/
  // ADCCTRL_RequestIpState(&LLTempRequest_Handle, ADC_OFF);
  (void)ADCCTRL_RequestIpState(&LLTempRequest_Handle, ADC_OFF);
  /*^^^ END OF PATCH STMC-485 ^^^*/

  /* Give shifted value of the temperature to the link layer */
  /*vvv PATCH STMC-485 vvvvvvvvvv*/
  // ll_intf_cmn_set_temperature_value((uint32_t)(temperature_value + TEMPMEAS_MIN_TEMP_LIMIT));
  ll_intf_cmn_set_temperature_value((uint32_t)((int32_t)temperature_value + (int32_t)(TEMPMEAS_MIN_TEMP_LIMIT)));
  /*^^^ END OF PATCH STMC-485 ^^^*/

  /*vvv PATCH STMC-485 vvvvvvvvvv*/
  // /* Exit limited critical section */
  // UTILS_EXIT_LIMITED_CRITICAL_SECTION();
  /*^^^ END OF PATCH STMC-485 ^^^*/
}
#endif /* USE_TEMPERATURE_BASED_RADIO_CALIBRATION */
/* Private Functions Definition ------------------------------------------------------*/
