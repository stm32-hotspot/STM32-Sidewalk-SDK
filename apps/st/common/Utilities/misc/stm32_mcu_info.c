/**
  ******************************************************************************
  * @file    stm32_mcu_info.c
  * @brief   Provides MCU information (printable MCU name, revision ID, etc.)
  *
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#include "stm32_mcu_info.h"

#if defined(STM32WBA50xx) || defined(STM32WBA52xx) || defined(STM32WBA54xx) || defined(STM32WBA55xx) || defined(STM32WBA5Mxx) || \
    defined(STM32WBA62xx) || defined(STM32WBA63xx) || defined(STM32WBA64xx) || defined(STM32WBA65xx) || defined(STM32WBA6Mxx)
#  define STM32WBAxx_FAMILY
#elif defined(STM32WL55xx) || defined(STM32WL54xx) || defined(STM32WLE5xx) || defined(STM32WLE4xx) || defined(STM32WL5Mxx)
#  define STM32WLxx_FAMILY
#  else
#    error "Unable to identify the target STM32 MCU. Please check if you have specified the MCU type in compile definitions and that your selected MCU is supported"
#endif

#ifdef STM32WBAxx_FAMILY
#include <stm32wbaxx_ll_system.h>
#endif /* STM32WBAxx_FAMILY */

#ifdef STM32WLxx_FAMILY
#include <stm32wlxx_ll_system.h>
#endif /* STM32WLxx_FAMILY */

/* Global function definitions -----------------------------------------------*/

const char * stm32_mcu_info_get_revision_name(const uint32_t device_id, const uint32_t rev_id)
{
    const char * rev_name;

    switch (device_id)
    {
#ifdef STM32WBAxx_FAMILY
        case STM32_MCU_INFO_DEVICE_ID_STM32WBA5x:
            switch (rev_id)
            {
                case STM32_MCU_INFO_STM32WBA5x_REV_ID_A:
                    rev_name = "Rev. A";
                    break;

                case STM32_MCU_INFO_STM32WBA5x_REV_ID_B:
                    rev_name = "Rev. B";
                    break;

                default:
                    /* Unknown revision ID */
                    rev_name = "Unknown";
                    break;
            }
            break;

        case STM32_MCU_INFO_DEVICE_ID_STM32WBA6x:
            switch (rev_id)
            {
                case STM32_MCU_INFO_STM32WBA5x_REV_ID_Z:
                    rev_name = "Rev. Z";
                    break;

                default:
                    /* Unknown revision ID */
                    rev_name = "Unknown";
                    break;
            }
            break;
#endif /* STM32WBAxx_FAMILY */

#ifdef STM32WLxx_FAMILY
        case STM32_MCU_INFO_DEVICE_ID_STM32WLxx:
            switch (rev_id)
            {
                case STM32_MCU_INFO_STM32WLxx_REV_ID_Z:
                    rev_name = "Rev. Z";
                    break;

                case STM32_MCU_INFO_STM32WLxx_REV_ID_Y:
                    rev_name = "Rev. Y";
                    break;

                default:
                    /* Unknown revision ID */
                    rev_name = "Unknown";
                    break;
            }
            break;
#endif /* STM32WLxx_FAMILY */

        default:
            /* Unknown Device ID */
            rev_name = "Unknown";
            break;
    }

    return rev_name;
}

/*----------------------------------------------------------------------------*/

const char * stm32_mcu_info_get_device_name(const uint32_t device_id)
{
    const char * dev_name;

    switch (device_id)
    {
#ifdef STM32WBAxx_FAMILY
        case STM32_MCU_INFO_DEVICE_ID_STM32WBA5x:
            dev_name = "STM32WBA5x";
            break;

        case STM32_MCU_INFO_DEVICE_ID_STM32WBA6x:
            dev_name = "STM32WBA6x";
            break;
#endif /* STM32WBAxx_FAMILY */

#ifdef STM32WLxx_FAMILY
        case STM32_MCU_INFO_DEVICE_ID_STM32WLxx:
            dev_name = "STM32WLxx";
            break;
#endif /* STM32WLxx_FAMILY */

        default:
            /* Unknown Device ID */
            dev_name = "Unknown";
            break;
    }

    return dev_name;
}

/*----------------------------------------------------------------------------*/

stm32_mcu_info_t stm32_mcu_info_describe_host(void)
{
    stm32_mcu_info_t host_info;

    host_info.device_id   = LL_DBGMCU_GetDeviceID();
    host_info.device_name = stm32_mcu_info_get_device_name(host_info.device_id);
    host_info.rev_id      = LL_DBGMCU_GetRevisionID();
    host_info.rev_name    = stm32_mcu_info_get_revision_name(host_info.device_id, host_info.rev_id);

    return host_info;
}
