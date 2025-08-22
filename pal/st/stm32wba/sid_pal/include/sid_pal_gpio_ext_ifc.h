/**
  ******************************************************************************
  * @file  sid_pal_gpio_ext_ifc.h
  * @brief Sidewalk GPIO PAL extensions for STM32WBAxx
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

#ifndef __SID_PAL_GPIO_EXT_IFC_H_
#define __SID_PAL_GPIO_EXT_IFC_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include <sid_pal_gpio_ifc.h>

#include "stm32wbaxx.h"
#include "stm32wbaxx_hal_gpio.h"

/* Exported types ------------------------------------------------------------*/

typedef enum
{
    GPIO_PORT_NC = 0,
    GPIO_PORTA   = 1,
    GPIO_PORTB   = 2,
    GPIO_PORTC   = 3,
    GPIO_PORTD   = 4,
    GPIO_PORTE   = 5,
    /* GPIO_PORTF   = 6, - not currently present on STM32WBA devices */
    GPIO_PORTG   = 7,
    GPIO_PORTH   = 8,
    GPIO_PORT_LAST = GPIO_PORTH,
} sid_pal_gpio_stm32wbaxx_ports;

/* Exported constants --------------------------------------------------------*/

#define NUM_GPIO_PORT_OFFSET        (16UL)
#define NUM_GPIO_PIN_MASK           (0xFFFFu)
#define GPIO_EXTI_DEFAULT_PRIORITY  (14UL)
#define GPIO_PORT_PIN_NOT_CONNECTED (0U)

/* Exported macro ------------------------------------------------------------*/

/* Obtains pin and port from a GPIO number */
#define NUM_TO_GPIO_PIN(num)              ((num) & NUM_GPIO_PIN_MASK)
#define NUM_TO_GPIO_PORT(num)             ((num)>> NUM_GPIO_PORT_OFFSET)
#define GET_GPIO_PIN_POSITION(gpio_pin)   (POSITION_VAL((gpio_pin)))

#if defined(STM32WBA)
#  if defined(STM32WBA50xx) || defined(STM32WBA52xx) || defined(STM32WBA54xx) || defined(STM32WBA55xx) || defined(STM32WBA5Mxx) ||  defined(STM32WBA63xx)
#    define ST_GPIO_PORT_TO_NUM(port)  (\
                                       (port == GPIOA) ? GPIO_PORTA : \
                                       (port == GPIOB) ? GPIO_PORTB : \
                                       (port == GPIOC) ? GPIO_PORTC : \
                                       (port == GPIOH) ? GPIO_PORTH : \
                                       GPIO_PORT_NC)

#    define NUM_TO_GPIO_ST_PORT(num)   ( \
                                       (NUM_TO_GPIO_PORT((num)) == GPIO_PORTA) ? GPIOA : \
                                       (NUM_TO_GPIO_PORT((num)) == GPIO_PORTB) ? GPIOB : \
                                       (NUM_TO_GPIO_PORT((num)) == GPIO_PORTC) ? GPIOC : \
                                       (NUM_TO_GPIO_PORT((num)) == GPIO_PORTH) ? GPIOH : NULL)
#  elif defined(STM32WBA64xx)
#    define ST_GPIO_PORT_TO_NUM(port)  (\
                                       (port == GPIOA) ? GPIO_PORTA : \
                                       (port == GPIOB) ? GPIO_PORTB : \
                                       (port == GPIOC) ? GPIO_PORTC : \
                                       (port == GPIOD) ? GPIO_PORTD : \
                                       (port == GPIOH) ? GPIO_PORTH : \
                                       GPIO_PORT_NC)

#    define NUM_TO_GPIO_ST_PORT(num)   ( \
                                       (NUM_TO_GPIO_PORT((num)) == GPIO_PORTA) ? GPIOA : \
                                       (NUM_TO_GPIO_PORT((num)) == GPIO_PORTB) ? GPIOB : \
                                       (NUM_TO_GPIO_PORT((num)) == GPIO_PORTC) ? GPIOC : \
                                       (NUM_TO_GPIO_PORT((num)) == GPIO_PORTD) ? GPIOD : \
                                       (NUM_TO_GPIO_PORT((num)) == GPIO_PORTH) ? GPIOH : NULL)
#  elif defined(STM32WBA62xx) || defined(STM32WBA65xx) || defined(STM32WBA6Mxx)
#    define ST_GPIO_PORT_TO_NUM(port)  (\
                                       (port == GPIOA) ? GPIO_PORTA : \
                                       (port == GPIOB) ? GPIO_PORTB : \
                                       (port == GPIOC) ? GPIO_PORTC : \
                                       (port == GPIOD) ? GPIO_PORTD : \
                                       (port == GPIOE) ? GPIO_PORTE : \
                                       (port == GPIOG) ? GPIO_PORTG : \
                                       (port == GPIOH) ? GPIO_PORTH : \
                                       GPIO_PORT_NC)

#    define NUM_TO_GPIO_ST_PORT(num)   ( \
                                       (NUM_TO_GPIO_PORT((num)) == GPIO_PORTA) ? GPIOA : \
                                       (NUM_TO_GPIO_PORT((num)) == GPIO_PORTB) ? GPIOB : \
                                       (NUM_TO_GPIO_PORT((num)) == GPIO_PORTC) ? GPIOC : \
                                       (NUM_TO_GPIO_PORT((num)) == GPIO_PORTD) ? GPIOD : \
                                       (NUM_TO_GPIO_PORT((num)) == GPIO_PORTE) ? GPIOE : \
                                       (NUM_TO_GPIO_PORT((num)) == GPIO_PORTG) ? GPIOG : \
                                       (NUM_TO_GPIO_PORT((num)) == GPIO_PORTH) ? GPIOH : NULL)
#  else
#    error "Unknown STM32WBA device"
#  endif /* STM32WBAxx */
#else
#  error "This implementation applies to the STM32WBAxx MCU family only"
#endif /* STM32WBA */

/* SID PAL interface functions require to pass one number for pin and port. 
  This macro converts GPIO pin and port to a single number */
#define GPIO_PORT_PIN_TO_NUM(port, pin) (ST_GPIO_PORT_TO_NUM(port) << NUM_GPIO_PORT_OFFSET | pin )

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief  Enables EXTI to latch the desired edge transition without triggering the interrupt in NVIC
 *         This method is useful when hardware reaction on a specific pin edge transition rather than
 *         a specific pin state is required but IRQ cannot be used to react on the edge (e.g. waiting
 *         on the edge transition in a critical section)
 *
 * @note This function does not affect NVIC and IRQ settings
 *
 * @param[in] gpio_number The logical GPIO number.
 * @param[in] edge        Desired edge transition to capture with EXTI
 *
 * @retval SID_ERROR_NONE in case of success.
 */
sid_error_t sid_pal_gpio_ext_ifc_set_latch(const uint32_t gpio_number, const sid_pal_gpio_irq_trigger_t edge);

/**
 * @brief   sid_pal_gpio_set_irq is used to check an interrupt status
 *
 * @param[in] gpio_number      The logical GPIO number.
 * @param[in] edge             Selected edge transition events flags to clear in EXTI
 * @param[in] is_edge_detected True if the edges specified by the edge parameter were detected by EXTI
 *
 * @retval SID_ERROR_NONE in case of success.
 */
sid_error_t sid_pal_gpio_ext_ifc_check_latch_state(const uint32_t gpio_number, const sid_pal_gpio_irq_trigger_t edge, uint32_t * const is_edge_detected);

/**
 * @brief Clears detected edges in EXTI peripheral
 *
 * @note This function does not affect NVIC and IRQ settings
 *
 * @param[in] gpio_number The logical GPIO number.
 * @param[in] edge        Selected edge transition event flags to clear in EXTI
 *
 * @retval SID_ERROR_NONE in case of success.
 */
sid_error_t sid_pal_gpio_ext_ifc_clear_latch_state(const uint32_t gpio_number, const sid_pal_gpio_irq_trigger_t edge);

/**
 * @brief Configures the IRQ priority in NVIC for the EXTI line associated with the specified pin
 *
 * @note This function does not affect EXTI settings (e.g. which transition triggers IRQ), nor it enables/disables the IRQ in NVIC, it only modifies the priority of the associated IRQ
 *
 * @param[in] gpio_number      The logical GPIO number.
 * @param[in] irq_preempt_prio Desired EXTI IRQ priority in NVIC for the EXTI line associated with the GPIO pin
 * @param[in] irq_sub_prio     Desired EXTI IRQ sub-priority in NVIC for the EXTI line associated with the GPIO pin
 *
 * @retval SID_ERROR_NONE in case of success.
 */
sid_error_t sid_pal_gpio_ext_ifc_set_irq_priority(const uint32_t gpio_number, const uint32_t irq_preempt_prio, const uint32_t irq_sub_prio);

/**
 * @brief Configures a GPIO pin as a wakeup source for the Standby LPM
 *
 * @param[in] gpio_number The logical GPIO number to operate on.
 * @param[in] edge        Selected edge transition to wakeup the MCU
 *
 * @retval SID_ERROR_NONE in case of success.
 */
sid_error_t sid_pal_gpio_ext_ifc_set_as_wakeup(const uint32_t gpio_number, const sid_pal_gpio_irq_trigger_t edge);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* __SID_PAL_GPIO_EXT_IFC_H_ */
