/**
  ******************************************************************************
  * @file    gpio.c
  * @brief Sidewalk GPIO PAL implementation for STM32WBAxx
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
#include <sid_pal_gpio_ifc.h>
#include <sid_pal_assert_ifc.h>
#include <sid_pal_log_ifc.h>
#include <sid_stm32_common_utils.h>

#include "stm32wbaxx_ll_gpio.h"
#include "stm32wbaxx_hal_exti.h"
#include "stm32wbaxx_ll_exti.h"
#include "sid_pal_gpio_ext_ifc.h"

/* Private typedef -----------------------------------------------------------*/

/**
 * @brief Configuration of the user-defined callbacks for GPIO-triggered interrupts
 */
typedef struct {
    sid_pal_gpio_irq_handler_t callback;     /*!< User-defined callback to be invoked whenever the associated GPIo IRQ occurs */
    void *                     callback_arg; /*!< User-defined argument to be passed to the callback */
    uint32_t                   gpio_number;  /*!< Associated GPIO pin (in Sidewalk format). This defines both the associated GPIO port and pin number on that port */
    sid_pal_gpio_irq_trigger_t trigger;      /*!< Trigger mode for the GPIO IRQ. Used for software emulation of voltage level-based triggers (SID_PAL_GPIO_IRQ_TRIGGER_LOW and SID_PAL_GPIO_IRQ_TRIGGER_HIGH) as there's no native support for it on STM32WBA */
} gpio_irq_callback_cfg_t;

/* Private defines -----------------------------------------------------------*/

#if defined(STM32WBA)
#  define GPIO_PINS_PER_PORT                         (16u)
#else
#  error "This implementation applies to the STM32WBAxx MCU family only"
#endif

#define SID_PAL_GPIO_STM32WBAxx_WAKEUP_POLARITY_LOW  (0u)
#define SID_PAL_GPIO_STM32WBAxx_WAKEUP_POLARITY_HIGH (1u)

/* Private macro -------------------------------------------------------------*/

#ifndef SID_PAL_GPIO_STM32WBAxx_EXTRA_LOGGING
/* Set SID_PAL_GPIO_STM32WBAxx_EXTRA_LOGGING to 1 to enable extended logs */
#  define SID_PAL_GPIO_STM32WBAxx_EXTRA_LOGGING      (0)
#endif

#if defined(SID_PAL_GPIO_STM32WBAxx_EXTRA_LOGGING) && SID_PAL_GPIO_STM32WBAxx_EXTRA_LOGGING
#  define SID_PAL_GPIO_STM32WBAxx_LOG_ERROR(...)     SID_PAL_LOG_ERROR(__VA_ARGS__)
#  define SID_PAL_GPIO_STM32WBAxx_LOG_WARNING(...)   SID_PAL_LOG_WARNING(__VA_ARGS__)
#  define SID_PAL_GPIO_STM32WBAxx_LOG_INFO(...)      SID_PAL_LOG_INFO(__VA_ARGS__)
#  define SID_PAL_GPIO_STM32WBAxx_LOG_DEBUG(...)     SID_PAL_LOG_DEBUG(__VA_ARGS__)
#  define SID_PAL_GPIO_STM32WBAxx_LOG_TRACE(...)     SID_PAL_LOG_TRACE(__VA_ARGS__)
#else
#  define SID_PAL_GPIO_STM32WBAxx_LOG_ERROR(...)     ((void)0u)
#  define SID_PAL_GPIO_STM32WBAxx_LOG_WARNING(...)   ((void)0u)
#  define SID_PAL_GPIO_STM32WBAxx_LOG_INFO(...)      ((void)0u)
#  define SID_PAL_GPIO_STM32WBAxx_LOG_DEBUG(...)     ((void)0u)
#  define SID_PAL_GPIO_STM32WBAxx_LOG_TRACE(...)     ((void)0u)
#endif

#if defined(STM32WBA)
#  define GET_GPIO_PIN_IRQN(gpio_pin)                ((((gpio_pin) >= GPIO_PIN_0) && ((gpio_pin) <= GPIO_PIN_15)) \
                                                     ? (EXTI0_IRQn + GET_GPIO_PIN_POSITION((gpio_pin))) \
                                                     : 0u)

#  define GET_LL_EXTI_LINE(gpio_pin)                 ((((gpio_pin) >= GPIO_PIN_0) && ((gpio_pin) <= GPIO_PIN_15)) \
                                                     ? (1UL << GET_GPIO_PIN_POSITION((gpio_pin))) \
                                                     : 0u)

#  define GET_LL_EXTI_CONFIG_PIN(pin_num) \
                                                     ( ( ( (GET_GPIO_PIN_POSITION(pin_num) & 0x3u) << 3) << LL_EXTI_REGISTER_PINPOS_SHFT) | (GET_GPIO_PIN_POSITION(pin_num) >> 2) )

#  if defined(STM32WBA50xx) || defined(STM32WBA52xx) || defined(STM32WBA54xx) || defined(STM32WBA55xx) || defined(STM32WBA5Mxx) ||  defined(STM32WBA63xx)
#    define GET_LL_EXTI_CONFIG_PORT(port_num)        (\
                                                     ((port_num) == GPIOA) ? LL_EXTI_CONFIG_PORTA : \
                                                     ((port_num) == GPIOB) ? LL_EXTI_CONFIG_PORTB : \
                                                     ((port_num) == GPIOC) ? LL_EXTI_CONFIG_PORTC : \
                                                     ((port_num) == GPIOH) ? LL_EXTI_CONFIG_PORTH : 0u)

#    define IS_GPIO_CLK_ENABLED(port_num)            ( \
                                                     ((port_num) == GPIOA) ? __HAL_RCC_GPIOA_IS_CLK_ENABLED() : \
                                                     ((port_num) == GPIOB) ? __HAL_RCC_GPIOB_IS_CLK_ENABLED() : \
                                                     ((port_num) == GPIOC) ? __HAL_RCC_GPIOC_IS_CLK_ENABLED() : \
                                                     ((port_num) == GPIOH) ? __HAL_RCC_GPIOH_IS_CLK_ENABLED() : 0u)

#    define NUM_TO_GPIO_PORT_STR(num)                ( \
                                                     (NUM_TO_GPIO_PORT((num)) == GPIO_PORTA) ? "A" : \
                                                     (NUM_TO_GPIO_PORT((num)) == GPIO_PORTB) ? "B" : \
                                                     (NUM_TO_GPIO_PORT((num)) == GPIO_PORTC) ? "C" : \
                                                     (NUM_TO_GPIO_PORT((num)) == GPIO_PORTH) ? "H" : "unknown")

#    define NUM_TO_PWR_GPIO_PORT(num)                ( \
                                                     (NUM_TO_GPIO_PORT(num) == GPIO_PORTA) ? PWR_GPIO_A : \
                                                     (NUM_TO_GPIO_PORT(num) == GPIO_PORTB) ? PWR_GPIO_B : \
                                                     (NUM_TO_GPIO_PORT(num) == GPIO_PORTC) ? PWR_GPIO_C : \
                                                     (NUM_TO_GPIO_PORT(num) == GPIO_PORTH) ? PWR_GPIO_H : 0xFFu)
#  elif defined(STM32WBA64xx)
#    define GET_LL_EXTI_CONFIG_PORT(port_num)        (\
                                                     ((port_num) == GPIOA) ? LL_EXTI_CONFIG_PORTA : \
                                                     ((port_num) == GPIOB) ? LL_EXTI_CONFIG_PORTB : \
                                                     ((port_num) == GPIOC) ? LL_EXTI_CONFIG_PORTC : \
                                                     ((port_num) == GPIOD) ? LL_EXTI_CONFIG_PORTD : \
                                                     ((port_num) == GPIOH) ? LL_EXTI_CONFIG_PORTH : 0u)

#    define IS_GPIO_CLK_ENABLED(port_num)            ( \
                                                     ((port_num) == GPIOA) ? __HAL_RCC_GPIOA_IS_CLK_ENABLED() : \
                                                     ((port_num) == GPIOB) ? __HAL_RCC_GPIOB_IS_CLK_ENABLED() : \
                                                     ((port_num) == GPIOC) ? __HAL_RCC_GPIOC_IS_CLK_ENABLED() : \
                                                     ((port_num) == GPIOD) ? __HAL_RCC_GPIOD_IS_CLK_ENABLED() : \
                                                     ((port_num) == GPIOH) ? __HAL_RCC_GPIOH_IS_CLK_ENABLED() : 0u)

#    define NUM_TO_GPIO_PORT_STR(num)                ( \
                                                     (NUM_TO_GPIO_PORT((num)) == GPIO_PORTA) ? "A" : \
                                                     (NUM_TO_GPIO_PORT((num)) == GPIO_PORTB) ? "B" : \
                                                     (NUM_TO_GPIO_PORT((num)) == GPIO_PORTC) ? "C" : \
                                                     (NUM_TO_GPIO_PORT((num)) == GPIO_PORTD) ? "D" : \
                                                     (NUM_TO_GPIO_PORT((num)) == GPIO_PORTH) ? "H" : "unknown")

#    define NUM_TO_PWR_GPIO_PORT(num)                ( \
                                                     (NUM_TO_GPIO_PORT(num) == GPIO_PORTA) ? PWR_GPIO_A : \
                                                     (NUM_TO_GPIO_PORT(num) == GPIO_PORTB) ? PWR_GPIO_B : \
                                                     (NUM_TO_GPIO_PORT(num) == GPIO_PORTC) ? PWR_GPIO_C : \
                                                     (NUM_TO_GPIO_PORT(num) == GPIO_PORTD) ? PWR_GPIO_D : \
                                                     (NUM_TO_GPIO_PORT(num) == GPIO_PORTH) ? PWR_GPIO_H : 0xFFu)
#  elif defined(STM32WBA62xx) || defined(STM32WBA65xx) || defined(STM32WBA6Mxx)
#    define GET_LL_EXTI_CONFIG_PORT(port_num)        (\
                                                     ((port_num) == GPIOA) ? LL_EXTI_CONFIG_PORTA : \
                                                     ((port_num) == GPIOB) ? LL_EXTI_CONFIG_PORTB : \
                                                     ((port_num) == GPIOC) ? LL_EXTI_CONFIG_PORTC : \
                                                     ((port_num) == GPIOD) ? LL_EXTI_CONFIG_PORTD : \
                                                     ((port_num) == GPIOE) ? LL_EXTI_CONFIG_PORTE : \
                                                     ((port_num) == GPIOG) ? LL_EXTI_CONFIG_PORTG : \
                                                     ((port_num) == GPIOH) ? LL_EXTI_CONFIG_PORTH : 0u)

#    define IS_GPIO_CLK_ENABLED(port_num)            ( \
                                                     ((port_num) == GPIOA) ? __HAL_RCC_GPIOA_IS_CLK_ENABLED() : \
                                                     ((port_num) == GPIOB) ? __HAL_RCC_GPIOB_IS_CLK_ENABLED() : \
                                                     ((port_num) == GPIOC) ? __HAL_RCC_GPIOC_IS_CLK_ENABLED() : \
                                                     ((port_num) == GPIOD) ? __HAL_RCC_GPIOD_IS_CLK_ENABLED() : \
                                                     ((port_num) == GPIOE) ? __HAL_RCC_GPIOE_IS_CLK_ENABLED() : \
                                                     ((port_num) == GPIOG) ? __HAL_RCC_GPIOG_IS_CLK_ENABLED() : \
                                                     ((port_num) == GPIOH) ? __HAL_RCC_GPIOH_IS_CLK_ENABLED() : 0u)

#    define NUM_TO_GPIO_PORT_STR(num)                ( \
                                                     (NUM_TO_GPIO_PORT((num)) == GPIO_PORTA) ? "A" : \
                                                     (NUM_TO_GPIO_PORT((num)) == GPIO_PORTB) ? "B" : \
                                                     (NUM_TO_GPIO_PORT((num)) == GPIO_PORTC) ? "C" : \
                                                     (NUM_TO_GPIO_PORT((num)) == GPIO_PORTD) ? "D" : \
                                                     (NUM_TO_GPIO_PORT((num)) == GPIO_PORTE) ? "E" : \
                                                     (NUM_TO_GPIO_PORT((num)) == GPIO_PORTG) ? "G" : \
                                                     (NUM_TO_GPIO_PORT((num)) == GPIO_PORTH) ? "H" : "unknown")

#    define NUM_TO_PWR_GPIO_PORT(num)                ( \
                                                     (NUM_TO_GPIO_PORT(num) == GPIO_PORTA) ? PWR_GPIO_A : \
                                                     (NUM_TO_GPIO_PORT(num) == GPIO_PORTB) ? PWR_GPIO_B : \
                                                     (NUM_TO_GPIO_PORT(num) == GPIO_PORTC) ? PWR_GPIO_C : \
                                                     (NUM_TO_GPIO_PORT(num) == GPIO_PORTD) ? PWR_GPIO_D : \
                                                     (NUM_TO_GPIO_PORT(num) == GPIO_PORTE) ? PWR_GPIO_E : \
                                                     (NUM_TO_GPIO_PORT(num) == GPIO_PORTG) ? PWR_GPIO_G : \
                                                     (NUM_TO_GPIO_PORT(num) == GPIO_PORTH) ? PWR_GPIO_H : 0xFFu)
#  else
#    error "Unknown STM32WBA device"
#  endif /* STM32WBAxx */
#else
#  error "This implementation applies to the STM32WBAxx MCU family only"
#endif /* STM32WBA */

/* Private variables ---------------------------------------------------------*/

/* Pool for sid_pall callbacks */
static gpio_irq_callback_cfg_t sid_pal_callbacks[GPIO_PINS_PER_PORT];

/* Global variables ----------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

static inline void register_gpio_irq_callback(const uint32_t gpio_number, const sid_pal_gpio_irq_trigger_t irq_trigger, const sid_pal_gpio_irq_handler_t callback, void * const callback_arg);
static inline void deregister_gpio_irq_callback(const uint32_t gpio_number);
static inline void gpio_clk_check_and_enable(const GPIO_TypeDef * const st_port);
static sid_error_t get_pwr_wakeup_pin_flag(const uint32_t gpio_number, const uint32_t wakeup_polarity, uint32_t * const out_wakeup_pin, uint32_t * const out_wakeup_flag);

/* Private function definitions ----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline void register_gpio_irq_callback(const uint32_t gpio_number, const sid_pal_gpio_irq_trigger_t irq_trigger, const sid_pal_gpio_irq_handler_t callback, void * const callback_arg)
{
    const uint32_t pin_index = GET_GPIO_PIN_POSITION(NUM_TO_GPIO_PIN(gpio_number));

    SID_PAL_ASSERT(pin_index < GPIO_PINS_PER_PORT);

    /**
     * Disable all interrupts when reconfiguring GPIO IRQs. Don't use sid_pal_enter_critical_region() here since a true critical section is needed here,
     * but sid_pal_enter_critical_region() may disable only some interrupts, depending on implementation
     */
    const uint32_t primask_bit = __get_PRIMASK();
    __disable_irq();

    sid_pal_callbacks[pin_index].callback     = callback;
    sid_pal_callbacks[pin_index].callback_arg = callback_arg;
    sid_pal_callbacks[pin_index].gpio_number  = gpio_number;
    sid_pal_callbacks[pin_index].trigger      = irq_trigger;

    __set_PRIMASK(primask_bit);
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline void deregister_gpio_irq_callback(const uint32_t gpio_number)
{
    const uint32_t pin_index = GET_GPIO_PIN_POSITION(NUM_TO_GPIO_PIN(gpio_number));

    SID_PAL_ASSERT(pin_index < GPIO_PINS_PER_PORT);

    /**
     * Disable all interrupts when reconfiguring GPIO IRQs. Don't use sid_pal_enter_critical_region() here since a true critical section is needed here,
     * but sid_pal_enter_critical_region() may disable only some interrupts, depending on implementation
     */
    const uint32_t primask_bit = __get_PRIMASK();
    __disable_irq();

    SID_STM32_UTIL_fast_memset(&sid_pal_callbacks[pin_index], 0x00u, sizeof(sid_pal_callbacks[pin_index]));

    __set_PRIMASK(primask_bit);
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline void gpio_clk_check_and_enable(const GPIO_TypeDef * const st_port)
{
    /* IMPORTNAT: while normally it's not a good idea to use switch() with pointers, here it is fine since
     * pointers to the GPIO ports are constant and they cannot be changed during the runtime. For this special
     * case using the switch() may gain some performance benefits over if()...else if()...else() construct
     */
    switch ((uint32_t)(void*)st_port)
    {
#ifdef GPIOA
        case (uint32_t)(void *)GPIOA:
            if (!__HAL_RCC_GPIOA_IS_CLK_ENABLED())
            {
                __HAL_RCC_GPIOA_CLK_ENABLE();
            }
            break;
#endif /* GPIOA */

#ifdef GPIOB
        case (uint32_t)(void *)GPIOB:
            if (!__HAL_RCC_GPIOB_IS_CLK_ENABLED())
            {
                __HAL_RCC_GPIOB_CLK_ENABLE();
            }
            break;
#endif /* GPIOB */

#ifdef GPIOC
        case (uint32_t)(void *)GPIOC:
            if (!__HAL_RCC_GPIOC_IS_CLK_ENABLED())
            {
                __HAL_RCC_GPIOC_CLK_ENABLE();
            }
            break;
#endif /* GPIOC */

#ifdef GPIOD
        case (uint32_t)(void *)GPIOD:
            if (!__HAL_RCC_GPIOD_IS_CLK_ENABLED())
            {
                __HAL_RCC_GPIOD_CLK_ENABLE();
            }
            break;
#endif /* GPIOD */

#ifdef GPIOE
        case (uint32_t)(void *)GPIOE:
            if (!__HAL_RCC_GPIOE_IS_CLK_ENABLED())
            {
                __HAL_RCC_GPIOE_CLK_ENABLE();
            }
            break;
#endif /* GPIOE */

#ifdef GPIOG
        case (uint32_t)(void *)GPIOG:
            if (!__HAL_RCC_GPIOG_IS_CLK_ENABLED())
            {
                __HAL_RCC_GPIOG_CLK_ENABLE();
            }
            break;
#endif /* GPIOG */

#ifdef GPIOH
        case (uint32_t)(void *)GPIOH:
            if (!__HAL_RCC_GPIOH_IS_CLK_ENABLED())
            {
                __HAL_RCC_GPIOH_CLK_ENABLE();
            }
            break;
#endif /* GPIOH */

        default:
            SID_PAL_LOG_ERROR("gpio_clk_check_and_enable() failed - unknown GPIO port referenced (0x%08X)", (uint32_t)(void *)st_port);
            break;
    }
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t get_pwr_wakeup_pin_flag(const uint32_t gpio_number, const uint32_t wakeup_polarity, uint32_t * const out_wakeup_pin, uint32_t * const out_wakeup_flag)
{
    sid_error_t err = SID_ERROR_NONE;

    switch (gpio_number)
    {
#if defined(PWR_WUCR1_WUPEN1) && defined(PWR_WAKEUP1_SOURCE_SELECTION_0)
        case GPIO_PORT_PIN_TO_NUM(GPIOA, GPIO_PIN_0):
            *out_wakeup_pin = PWR_WUCR1_WUPEN1 | PWR_WAKEUP1_SOURCE_SELECTION_0;
            *out_wakeup_flag = PWR_WAKEUP_FLAG1;
            if (SID_PAL_GPIO_STM32WBAxx_WAKEUP_POLARITY_LOW == wakeup_polarity)
            {
                *out_wakeup_pin |= PWR_WAKEUP1_POLARITY_LOW;
            }
            break;
#endif /* PWR_WUCR1_WUPEN1 && PWR_WAKEUP1_SOURCE_SELECTION_0 */

#if defined(PWR_WUCR1_WUPEN1) && defined(PWR_WAKEUP1_SOURCE_SELECTION_1)
        case GPIO_PORT_PIN_TO_NUM(GPIOB, GPIO_PIN_2):
            *out_wakeup_pin = PWR_WUCR1_WUPEN1 | PWR_WAKEUP1_SOURCE_SELECTION_1;
            *out_wakeup_flag = PWR_WAKEUP_FLAG1;
            if (SID_PAL_GPIO_STM32WBAxx_WAKEUP_POLARITY_LOW == wakeup_polarity)
            {
                *out_wakeup_pin |= PWR_WAKEUP1_POLARITY_LOW;
            }
            break;
#endif /* PWR_WUCR1_WUPEN1 && PWR_WAKEUP1_SOURCE_SELECTION_1 */

#if defined(PWR_WUCR1_WUPEN2) && defined(PWR_WAKEUP2_SOURCE_SELECTION_0)
        case GPIO_PORT_PIN_TO_NUM(GPIOA, GPIO_PIN_4):
            *out_wakeup_pin = PWR_WUCR1_WUPEN2 | PWR_WAKEUP2_SOURCE_SELECTION_0;
            *out_wakeup_flag = PWR_WAKEUP_FLAG2;
            if (SID_PAL_GPIO_STM32WBAxx_WAKEUP_POLARITY_LOW == wakeup_polarity)
            {
                *out_wakeup_pin |= PWR_WAKEUP2_POLARITY_LOW;
            }
            break;
#endif /* PWR_WUCR1_WUPEN2 && PWR_WAKEUP2_SOURCE_SELECTION_0 */

#if defined(PWR_WUCR1_WUPEN2) && defined(PWR_WAKEUP2_SOURCE_SELECTION_1)
        case GPIO_PORT_PIN_TO_NUM(GPIOC, GPIO_PIN_13):
            *out_wakeup_pin = PWR_WUCR1_WUPEN2 | PWR_WAKEUP2_SOURCE_SELECTION_1;
            *out_wakeup_flag = PWR_WAKEUP_FLAG2;
            if (SID_PAL_GPIO_STM32WBAxx_WAKEUP_POLARITY_LOW == wakeup_polarity)
            {
                *out_wakeup_pin |= PWR_WAKEUP2_POLARITY_LOW;
            }
            break;
#endif /* PWR_WUCR1_WUPEN2 && PWR_WAKEUP2_SOURCE_SELECTION_1 */

#if defined(PWR_WUCR1_WUPEN3) && defined(PWR_WAKEUP3_SOURCE_SELECTION_1)
        case GPIO_PORT_PIN_TO_NUM(GPIOA, GPIO_PIN_1):
            *out_wakeup_pin = PWR_WUCR1_WUPEN3 | PWR_WAKEUP3_SOURCE_SELECTION_1;
            *out_wakeup_flag = PWR_WAKEUP_FLAG3;
            if (SID_PAL_GPIO_STM32WBAxx_WAKEUP_POLARITY_LOW == wakeup_polarity)
            {
                *out_wakeup_pin |= PWR_WAKEUP3_POLARITY_LOW;
            }
            break;
#endif /* PWR_WUCR1_WUPEN3 && PWR_WAKEUP3_SOURCE_SELECTION_1 */

#if defined(PWR_WUCR1_WUPEN3) && defined(PWR_WAKEUP3_SOURCE_SELECTION_2)
        case GPIO_PORT_PIN_TO_NUM(GPIOB, GPIO_PIN_6):
            *out_wakeup_pin = PWR_WUCR1_WUPEN3 | PWR_WAKEUP3_SOURCE_SELECTION_2;
            *out_wakeup_flag = PWR_WAKEUP_FLAG3;
            if (SID_PAL_GPIO_STM32WBAxx_WAKEUP_POLARITY_LOW == wakeup_polarity)
            {
                *out_wakeup_pin |= PWR_WAKEUP3_POLARITY_LOW;
            }
            break;
#endif /* PWR_WUCR1_WUPEN3 && PWR_WAKEUP3_SOURCE_SELECTION_2 */

#if defined(PWR_WUCR1_WUPEN4) && defined(PWR_WAKEUP4_SOURCE_SELECTION_0)
        case GPIO_PORT_PIN_TO_NUM(GPIOA, GPIO_PIN_2):
            *out_wakeup_pin = PWR_WUCR1_WUPEN4 | PWR_WAKEUP4_SOURCE_SELECTION_0;
            *out_wakeup_flag = PWR_WAKEUP_FLAG4;
            if (SID_PAL_GPIO_STM32WBAxx_WAKEUP_POLARITY_LOW == wakeup_polarity)
            {
                *out_wakeup_pin |= PWR_WAKEUP4_POLARITY_LOW;
            }
            break;
#endif /* PWR_WUCR1_WUPEN4 && PWR_WAKEUP4_SOURCE_SELECTION_0 */

#if defined(PWR_WUCR1_WUPEN4) && defined(PWR_WAKEUP4_SOURCE_SELECTION_1)
        case GPIO_PORT_PIN_TO_NUM(GPIOB, GPIO_PIN_1):
            *out_wakeup_pin = PWR_WUCR1_WUPEN4 | PWR_WAKEUP4_SOURCE_SELECTION_1;
            *out_wakeup_flag = PWR_WAKEUP_FLAG4;
            if (SID_PAL_GPIO_STM32WBAxx_WAKEUP_POLARITY_LOW == wakeup_polarity)
            {
                *out_wakeup_pin |= PWR_WAKEUP4_POLARITY_LOW;
            }
            break;
#endif /* PWR_WUCR1_WUPEN4 && PWR_WAKEUP4_SOURCE_SELECTION_1 */

#if defined(PWR_WUCR1_WUPEN5) && defined(PWR_WAKEUP5_SOURCE_SELECTION_1)
        case GPIO_PORT_PIN_TO_NUM(GPIOA, GPIO_PIN_3):
            *out_wakeup_pin = PWR_WUCR1_WUPEN5 | PWR_WAKEUP5_SOURCE_SELECTION_1;
            *out_wakeup_flag = PWR_WAKEUP_FLAG5;
            if (SID_PAL_GPIO_STM32WBAxx_WAKEUP_POLARITY_LOW == wakeup_polarity)
            {
                *out_wakeup_pin |= PWR_WAKEUP5_POLARITY_LOW;
            }
            break;
#endif /* PWR_WUCR1_WUPEN5 && PWR_WAKEUP5_SOURCE_SELECTION_1 */

#if defined(PWR_WUCR1_WUPEN5) && defined(PWR_WAKEUP5_SOURCE_SELECTION_2)
        case GPIO_PORT_PIN_TO_NUM(GPIOB, GPIO_PIN_7):
            *out_wakeup_pin = PWR_WUCR1_WUPEN5 | PWR_WAKEUP5_SOURCE_SELECTION_2;
            *out_wakeup_flag = PWR_WAKEUP_FLAG5;
            if (SID_PAL_GPIO_STM32WBAxx_WAKEUP_POLARITY_LOW == wakeup_polarity)
            {
                *out_wakeup_pin |= PWR_WAKEUP5_POLARITY_LOW;
            }
            break;
#endif /* PWR_WUCR1_WUPEN5 && PWR_WAKEUP5_SOURCE_SELECTION_2 */

#if defined(PWR_WUCR1_WUPEN6) && defined(PWR_WAKEUP6_SOURCE_SELECTION_0)
        case GPIO_PORT_PIN_TO_NUM(GPIOA, GPIO_PIN_12):
            *out_wakeup_pin = PWR_WUCR1_WUPEN6 | PWR_WAKEUP6_SOURCE_SELECTION_0;
            *out_wakeup_flag = PWR_WAKEUP_FLAG6;
            if (SID_PAL_GPIO_STM32WBAxx_WAKEUP_POLARITY_LOW == wakeup_polarity)
            {
                *out_wakeup_pin |= PWR_WAKEUP6_POLARITY_LOW;
            }
            break;
#endif /* PWR_WUCR1_WUPEN6 && PWR_WAKEUP6_SOURCE_SELECTION_0 */

#if defined(PWR_WUCR1_WUPEN6) && defined(PWR_WAKEUP6_SOURCE_SELECTION_1)
        case GPIO_PORT_PIN_TO_NUM(GPIOA, GPIO_PIN_5):
            *out_wakeup_pin = PWR_WUCR1_WUPEN6 | PWR_WAKEUP6_SOURCE_SELECTION_1;
            *out_wakeup_flag = PWR_WAKEUP_FLAG6;
            if (SID_PAL_GPIO_STM32WBAxx_WAKEUP_POLARITY_LOW == wakeup_polarity)
            {
                *out_wakeup_pin |= PWR_WAKEUP6_POLARITY_LOW;
            }
            break;
#endif /* PWR_WUCR1_WUPEN6 && PWR_WAKEUP6_SOURCE_SELECTION_1 */

#if defined(PWR_WUCR1_WUPEN7) && defined(PWR_WAKEUP7_SOURCE_SELECTION_0)
        case GPIO_PORT_PIN_TO_NUM(GPIOB, GPIO_PIN_14):
            *out_wakeup_pin = PWR_WUCR1_WUPEN7 | PWR_WAKEUP7_SOURCE_SELECTION_0;
            *out_wakeup_flag = PWR_WAKEUP_FLAG7;
            if (SID_PAL_GPIO_STM32WBAxx_WAKEUP_POLARITY_LOW == wakeup_polarity)
            {
                *out_wakeup_pin |= PWR_WAKEUP7_POLARITY_LOW;
            }
            break;
#endif /* PWR_WUCR1_WUPEN7 && PWR_WAKEUP7_SOURCE_SELECTION_0 */

#if defined(PWR_WUCR1_WUPEN7) && defined(PWR_WAKEUP7_SOURCE_SELECTION_1)
        case GPIO_PORT_PIN_TO_NUM(GPIOA, GPIO_PIN_6):
            *out_wakeup_pin = PWR_WUCR1_WUPEN7 | PWR_WAKEUP7_SOURCE_SELECTION_1;
            *out_wakeup_flag = PWR_WAKEUP_FLAG7;
            if (SID_PAL_GPIO_STM32WBAxx_WAKEUP_POLARITY_LOW == wakeup_polarity)
            {
                *out_wakeup_pin |= PWR_WAKEUP7_POLARITY_LOW;
            }
            break;
#endif /* PWR_WUCR1_WUPEN7 && PWR_WAKEUP7_SOURCE_SELECTION_1 */

#if defined(PWR_WUCR1_WUPEN8) && defined(PWR_WAKEUP8_SOURCE_SELECTION_1)
        case GPIO_PORT_PIN_TO_NUM(GPIOA, GPIO_PIN_7):
            *out_wakeup_pin = PWR_WUCR1_WUPEN8 | PWR_WAKEUP8_SOURCE_SELECTION_1;
            *out_wakeup_flag = PWR_WAKEUP_FLAG8;
            if (SID_PAL_GPIO_STM32WBAxx_WAKEUP_POLARITY_LOW == wakeup_polarity)
            {
                *out_wakeup_pin |= PWR_WAKEUP8_POLARITY_LOW;
            }
            break;
#endif /* PWR_WUCR1_WUPEN8 && PWR_WAKEUP8_SOURCE_SELECTION_1 */

#if defined(PWR_WUCR1_WUPEN8) && defined(PWR_WAKEUP1_SOURCE_SELECTION_1)
        case GPIO_PORT_PIN_TO_NUM(GPIOB, GPIO_PIN_9):
            *out_wakeup_pin = PWR_WUCR1_WUPEN8 | PWR_WAKEUP8_SOURCE_SELECTION_2;
            *out_wakeup_flag = PWR_WAKEUP_FLAG8;
            if (SID_PAL_GPIO_STM32WBAxx_WAKEUP_POLARITY_LOW == wakeup_polarity)
            {
                *out_wakeup_pin |= PWR_WAKEUP8_POLARITY_LOW;
            }
            break;
#endif /* PWR_WUCR1_WUPEN8 && PWR_WAKEUP8_SOURCE_SELECTION_2 */

        default:
            err = SID_ERROR_INVALID_ARGS;
            break;
    }

    return err;
}

/* Global function definitions -----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_pal_gpio_set_direction(uint32_t gpio_number, sid_pal_gpio_direction_t direction)
{
    /* If pin is not connected then just ignore it*/
    if(GPIO_PORT_PIN_NOT_CONNECTED == gpio_number)
    {
        return SID_ERROR_NONE;
    }

    GPIO_TypeDef * const st_port = NUM_TO_GPIO_ST_PORT(gpio_number);
    const uint32_t st_pin = NUM_TO_GPIO_PIN(gpio_number);
    SID_PAL_ASSERT(IS_GPIO_ALL_INSTANCE(st_port));
    SID_PAL_ASSERT(IS_GPIO_PIN(st_pin));
    gpio_clk_check_and_enable(st_port);

    if (direction > SID_PAL_GPIO_DIRECTION_OUTPUT)
    {
        return SID_ERROR_INVALID_ARGS;
    }

    uint32_t pin_mode;

    if (direction == SID_PAL_GPIO_DIRECTION_INPUT) 
    {
        SID_PAL_GPIO_STM32WBAxx_LOG_DEBUG("Set P%s%d direction to Input", NUM_TO_GPIO_PORT_STR(gpio_number), GET_GPIO_PIN_POSITION(NUM_TO_GPIO_PIN(gpio_number)));
        pin_mode = LL_GPIO_MODE_INPUT;
    }
    else 
    {
        SID_PAL_GPIO_STM32WBAxx_LOG_DEBUG("Set P%s%d direction to Output", NUM_TO_GPIO_PORT_STR(gpio_number), GET_GPIO_PIN_POSITION(NUM_TO_GPIO_PIN(gpio_number)));
        pin_mode = LL_GPIO_MODE_OUTPUT;
        LL_GPIO_SetPinSpeed(st_port, st_pin, LL_GPIO_SPEED_FREQ_HIGH);
    }

    LL_GPIO_SetPinMode(st_port, st_pin, pin_mode);

    return SID_ERROR_NONE;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_pal_gpio_input_mode(uint32_t gpio_number, sid_pal_gpio_input_t mode)
{
    if(GPIO_PORT_PIN_NOT_CONNECTED == gpio_number)
    {
        return SID_ERROR_NONE;
    }

    if (mode > SID_PAL_GPIO_INPUT_DISCONNECT) 
    {
        return SID_ERROR_INVALID_ARGS;
    }

    GPIO_TypeDef * const st_port = NUM_TO_GPIO_ST_PORT(gpio_number);
    const uint32_t st_pin = NUM_TO_GPIO_PIN(gpio_number);
    SID_PAL_ASSERT(IS_GPIO_ALL_INSTANCE(st_port));
    SID_PAL_ASSERT(IS_GPIO_PIN(st_pin));
    gpio_clk_check_and_enable(st_port);

    uint32_t pin_mode;

    if (mode == SID_PAL_GPIO_INPUT_CONNECT)
    {
        SID_PAL_GPIO_STM32WBAxx_LOG_DEBUG("Set P%s%d input mode to Input", NUM_TO_GPIO_PORT_STR(gpio_number), GET_GPIO_PIN_POSITION(NUM_TO_GPIO_PIN(gpio_number)));
        pin_mode = LL_GPIO_MODE_INPUT;
    }
    else 
    {
        SID_PAL_GPIO_STM32WBAxx_LOG_DEBUG("Set P%s%d input mode to Analog", NUM_TO_GPIO_PORT_STR(gpio_number), GET_GPIO_PIN_POSITION(NUM_TO_GPIO_PIN(gpio_number)));
        pin_mode = LL_GPIO_MODE_ANALOG;
    }

    LL_GPIO_SetPinMode(st_port, st_pin, pin_mode);

    return SID_ERROR_NONE;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_pal_gpio_output_mode(uint32_t gpio_number, sid_pal_gpio_output_t mode)
{
    if(GPIO_PORT_PIN_NOT_CONNECTED == gpio_number)
    {
        return SID_ERROR_NONE;
    }

    GPIO_TypeDef * const st_port = NUM_TO_GPIO_ST_PORT(gpio_number);
    const uint32_t st_pin = NUM_TO_GPIO_PIN(gpio_number);
    SID_PAL_ASSERT(IS_GPIO_ALL_INSTANCE(st_port));
    SID_PAL_ASSERT(IS_GPIO_PIN(st_pin));
    gpio_clk_check_and_enable(st_port);

    if (mode > SID_PAL_GPIO_OUTPUT_OPEN_DRAIN) 
    {
        return SID_ERROR_INVALID_ARGS;
    }

    uint32_t output_type;
    if (mode == SID_PAL_GPIO_OUTPUT_OPEN_DRAIN) 
    {
        SID_PAL_GPIO_STM32WBAxx_LOG_DEBUG("Set P%s%d output mode to Open Drain", NUM_TO_GPIO_PORT_STR(gpio_number), GET_GPIO_PIN_POSITION(NUM_TO_GPIO_PIN(gpio_number)));
        output_type = LL_GPIO_OUTPUT_OPENDRAIN;
    }
    else 
    {
        SID_PAL_GPIO_STM32WBAxx_LOG_DEBUG("Set P%s%d output mode to Push-Pull", NUM_TO_GPIO_PORT_STR(gpio_number), GET_GPIO_PIN_POSITION(NUM_TO_GPIO_PIN(gpio_number)));
        output_type = LL_GPIO_OUTPUT_PUSHPULL;
    }

    LL_GPIO_SetPinOutputType(st_port, st_pin, output_type);

    return SID_ERROR_NONE;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_pal_gpio_pull_mode(uint32_t gpio_number, sid_pal_gpio_pull_t pull)
{
    if(GPIO_PORT_PIN_NOT_CONNECTED == gpio_number)
    {
        return SID_ERROR_NONE;
    }

    GPIO_TypeDef * const st_port = NUM_TO_GPIO_ST_PORT(gpio_number);
    const uint32_t st_pin = NUM_TO_GPIO_PIN(gpio_number);
    SID_PAL_ASSERT(IS_GPIO_ALL_INSTANCE(st_port));
    SID_PAL_ASSERT(IS_GPIO_PIN(st_pin));
    gpio_clk_check_and_enable(st_port);

    if (pull > SID_PAL_GPIO_PULL_DOWN)
    {
        return SID_ERROR_INVALID_ARGS;
    }

    uint32_t pin_pull = LL_GPIO_PULL_NO;
    switch (pull)
    {
        case SID_PAL_GPIO_PULL_NONE:
            SID_PAL_GPIO_STM32WBAxx_LOG_DEBUG("Disable pull-up & pull-down on P%s%d", NUM_TO_GPIO_PORT_STR(gpio_number), GET_GPIO_PIN_POSITION(NUM_TO_GPIO_PIN(gpio_number)));
            pin_pull = LL_GPIO_PULL_NO;
            break;

        case SID_PAL_GPIO_PULL_UP:
            SID_PAL_GPIO_STM32WBAxx_LOG_DEBUG("Enable pull-up on P%s%d", NUM_TO_GPIO_PORT_STR(gpio_number), GET_GPIO_PIN_POSITION(NUM_TO_GPIO_PIN(gpio_number)));
            pin_pull = LL_GPIO_PULL_UP;
            break;

        case SID_PAL_GPIO_PULL_DOWN:
            SID_PAL_GPIO_STM32WBAxx_LOG_DEBUG("Enable pull-down on P%s%d", NUM_TO_GPIO_PORT_STR(gpio_number), GET_GPIO_PIN_POSITION(NUM_TO_GPIO_PIN(gpio_number)));
            pin_pull = LL_GPIO_PULL_DOWN;
            break;

        default:
            SID_PAL_LOG_ERROR("Invalid GPIO pull mode: 0x%02X", pull);
            break;
    }

    LL_GPIO_SetPinPull(st_port, st_pin, pin_pull);

    return SID_ERROR_NONE;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_pal_gpio_read(uint32_t gpio_number, uint8_t *value)
{
    if(GPIO_PORT_PIN_NOT_CONNECTED == gpio_number)
    {
        return SID_ERROR_NONE;
    }

    GPIO_TypeDef * const st_port = NUM_TO_GPIO_ST_PORT(gpio_number);
    const uint32_t st_pin = NUM_TO_GPIO_PIN(gpio_number);
    SID_PAL_ASSERT(IS_GPIO_ALL_INSTANCE(st_port));
    SID_PAL_ASSERT(IS_GPIO_PIN(st_pin));
    SID_PAL_ASSERT( false != IS_GPIO_CLK_ENABLED(st_port) );

    if ((st_port->IDR & st_pin) != 0x00u)
    {
        *value = GPIO_PIN_SET;
    }
    else
    {
        *value = GPIO_PIN_RESET;
    }

    return SID_ERROR_NONE;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_pal_gpio_write(uint32_t gpio_number, uint8_t value)
{      
    if(GPIO_PORT_PIN_NOT_CONNECTED == gpio_number)
    {
        return SID_ERROR_NONE;
    }

    GPIO_TypeDef * const st_port = NUM_TO_GPIO_ST_PORT(gpio_number);
    const uint32_t st_pin = NUM_TO_GPIO_PIN(gpio_number);
    SID_PAL_ASSERT(IS_GPIO_ALL_INSTANCE(st_port));
    SID_PAL_ASSERT(IS_GPIO_PIN(st_pin));
    SID_PAL_ASSERT( false != IS_GPIO_CLK_ENABLED(st_port) );

    if (value != GPIO_PIN_RESET)
    {
        st_port->BSRR = (uint32_t)st_pin;
    }
    else
    {
        st_port->BRR = (uint32_t)st_pin;
    }

    return SID_ERROR_NONE;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_pal_gpio_toggle(uint32_t gpio_number)
{
    if(GPIO_PORT_PIN_NOT_CONNECTED == gpio_number)
    {
        return SID_ERROR_NONE;
    }

    GPIO_TypeDef * const st_port = NUM_TO_GPIO_ST_PORT(gpio_number);
    const uint32_t st_pin = NUM_TO_GPIO_PIN(gpio_number);
    SID_PAL_ASSERT(IS_GPIO_ALL_INSTANCE(st_port));
    SID_PAL_ASSERT(IS_GPIO_PIN(st_pin));
    SID_PAL_ASSERT( false != IS_GPIO_CLK_ENABLED(st_port) );

    /* get current Output Data Register value */
    uint32_t odr = st_port->ODR;

    /* Set selected pins that were at low level, and reset ones that were high */
    st_port->BSRR = ((odr & st_pin) << GPIO_PINS_PER_PORT) | (~odr & st_pin);

    return SID_ERROR_NONE;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_pal_gpio_ext_ifc_set_latch(const uint32_t gpio_number, const sid_pal_gpio_irq_trigger_t edge)
{
    sid_error_t err = SID_ERROR_GENERIC;

    if (GPIO_PORT_PIN_NOT_CONNECTED == gpio_number)
    {
        return SID_ERROR_NONE;
    }

    GPIO_TypeDef * const st_port = NUM_TO_GPIO_ST_PORT(gpio_number);
    const uint32_t st_pin = NUM_TO_GPIO_PIN(gpio_number);
    SID_PAL_ASSERT(IS_GPIO_ALL_INSTANCE(st_port));
    SID_PAL_ASSERT(IS_GPIO_PIN(st_pin));
    gpio_clk_check_and_enable(st_port);
    LL_EXTI_InitTypeDef EXTI_InitStruct = {0};

    uint32_t exti_trigger = LL_EXTI_TRIGGER_NONE;
    switch (edge)
    {
        case SID_PAL_GPIO_IRQ_TRIGGER_NONE:
            SID_PAL_GPIO_STM32WBAxx_LOG_DEBUG("Configure no EXTI trigger on P%s%d", NUM_TO_GPIO_PORT_STR(gpio_number), GET_GPIO_PIN_POSITION(NUM_TO_GPIO_PIN(gpio_number)));
            exti_trigger = LL_EXTI_TRIGGER_NONE;
            break;

        case SID_PAL_GPIO_IRQ_TRIGGER_RISING:
            SID_PAL_GPIO_STM32WBAxx_LOG_DEBUG("Configure EXTI rising edge trigger on P%s%d", NUM_TO_GPIO_PORT_STR(gpio_number), GET_GPIO_PIN_POSITION(NUM_TO_GPIO_PIN(gpio_number)));
            exti_trigger = LL_EXTI_TRIGGER_RISING;
            break;

        case SID_PAL_GPIO_IRQ_TRIGGER_FALLING:
            SID_PAL_GPIO_STM32WBAxx_LOG_DEBUG("Configure EXTI falling edge trigger on P%s%d", NUM_TO_GPIO_PORT_STR(gpio_number), GET_GPIO_PIN_POSITION(NUM_TO_GPIO_PIN(gpio_number)));
            exti_trigger = LL_EXTI_TRIGGER_FALLING;
            break;

        case SID_PAL_GPIO_IRQ_TRIGGER_EDGE:
            SID_PAL_GPIO_STM32WBAxx_LOG_DEBUG("Configure EXTI rising & falling edges trigger on P%s%d", NUM_TO_GPIO_PORT_STR(gpio_number), GET_GPIO_PIN_POSITION(NUM_TO_GPIO_PIN(gpio_number)));
            exti_trigger = LL_EXTI_TRIGGER_RISING_FALLING;
            break;

        default:
            SID_PAL_LOG_ERROR("Error configuring P%s%d. 0x%02X is not a valid trigger", NUM_TO_GPIO_PORT_STR(gpio_number), GET_GPIO_PIN_POSITION(NUM_TO_GPIO_PIN(gpio_number)), (uint32_t)edge);
            break;
    }

    if (SID_PAL_GPIO_IRQ_TRIGGER_NONE == edge)
    {
        EXTI_InitStruct.Line_0_31 = GET_LL_EXTI_LINE(st_pin);
        EXTI_InitStruct.LineCommand = DISABLE;
        EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
        EXTI_InitStruct.Trigger = exti_trigger;
        LL_EXTI_Init(&EXTI_InitStruct);

        err = SID_ERROR_NONE;
    }
    else if ((edge > SID_PAL_GPIO_IRQ_TRIGGER_NONE) && (edge <= SID_PAL_GPIO_IRQ_TRIGGER_EDGE))
    {
        LL_EXTI_SetEXTISource(GET_LL_EXTI_CONFIG_PORT(st_port), GET_LL_EXTI_CONFIG_PIN(st_pin));

        EXTI_InitStruct.Line_0_31 = GET_LL_EXTI_LINE(st_pin);
        EXTI_InitStruct.LineCommand = ENABLE;
        EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
        EXTI_InitStruct.Trigger = exti_trigger;
        LL_EXTI_Init(&EXTI_InitStruct); 

        err = SID_ERROR_NONE;
    }
    else
    {
        err = SID_ERROR_NOSUPPORT;
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_pal_gpio_set_irq(uint32_t gpio_number, sid_pal_gpio_irq_trigger_t irq_trigger, sid_pal_gpio_irq_handler_t gpio_callback, void * callback_arg)
{
    sid_error_t err = SID_ERROR_GENERIC;

    if (GPIO_PORT_PIN_NOT_CONNECTED == gpio_number)
    {
        return SID_ERROR_NONE;
    }

    const uint32_t irq_num = GET_GPIO_PIN_IRQN(NUM_TO_GPIO_PIN(gpio_number));
    SID_PAL_ASSERT(irq_num != 0u); /* Ensure gpio_number refers to a valid GPIO pin */

    do
    { 
        /* Configure EXTI peripheral to generate events on the desired edge(s) */
        switch (irq_trigger)
        {
            case SID_PAL_GPIO_IRQ_TRIGGER_LOW:
                /* Use falling edge EXTI trigger for hardware detection, level-based detection is implemented in software as there's no hardware support for it */
                err = sid_pal_gpio_ext_ifc_set_latch(gpio_number, SID_PAL_GPIO_IRQ_TRIGGER_FALLING);
                break;

            case SID_PAL_GPIO_IRQ_TRIGGER_HIGH:
                /* Use rising edge EXTI trigger for hardware detection, level-based detection is implemented in software as there's no hardware support for it */
                err = sid_pal_gpio_ext_ifc_set_latch(gpio_number, SID_PAL_GPIO_IRQ_TRIGGER_RISING);
                break;

            default:
                /* For any kind of edge transition-based trigger we can use hardware mechanisms directly */
                err = sid_pal_gpio_ext_ifc_set_latch(gpio_number, irq_trigger);
                break;
        }
        if(err != SID_ERROR_NONE)
        {
            break;
        }

        /* Configure NVIC to generate interrupts based on inputs from EXTI */
        if (SID_PAL_GPIO_IRQ_TRIGGER_NONE == irq_trigger)
        {
            err = sid_pal_gpio_irq_disable(gpio_number);
            if (err != SID_ERROR_NONE)
            {
                break;
            }

            /* De-register user callback */
            deregister_gpio_irq_callback(gpio_number);
        }
        else
        {
            /* Assign callback first */
            register_gpio_irq_callback(gpio_number, irq_trigger, gpio_callback, callback_arg);
            __COMPILER_BARRIER();

            /* Set EXTI Interrupt to the default priority */
            HAL_NVIC_SetPriority(irq_num, GPIO_EXTI_DEFAULT_PRIORITY, 0u);

            /* Enable the corresponding IRQ in NVIC */
            err = sid_pal_gpio_irq_enable(gpio_number);
            if (err != SID_ERROR_NONE)
            {
                break;
            }
        }

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_pal_gpio_irq_enable(uint32_t gpio_number)
{
    if(GPIO_PORT_PIN_NOT_CONNECTED == gpio_number)
    {
        return SID_ERROR_NONE;
    }

    GPIO_TypeDef * const st_port = NUM_TO_GPIO_ST_PORT(gpio_number);
    const uint32_t st_pin = NUM_TO_GPIO_PIN(gpio_number);
    const uint32_t pin_index = GET_GPIO_PIN_POSITION(st_pin);
    const uint32_t exti_line = GET_LL_EXTI_LINE(st_pin);
    const uint32_t irq_num = GET_GPIO_PIN_IRQN(st_pin);

    SID_PAL_ASSERT(IS_GPIO_ALL_INSTANCE(st_port));
    SID_PAL_ASSERT(IS_GPIO_PIN(st_pin));
    SID_PAL_ASSERT( false != IS_GPIO_CLK_ENABLED(st_port) );

    if (sid_pal_callbacks[pin_index].gpio_number == gpio_number)
    {
        if (((SID_PAL_GPIO_IRQ_TRIGGER_LOW == sid_pal_callbacks[pin_index].trigger) && ((st_port->IDR & st_pin) == 0x00u))
        || ((SID_PAL_GPIO_IRQ_TRIGGER_HIGH == sid_pal_callbacks[pin_index].trigger) && ((st_port->IDR & st_pin) != 0x00u)))
        {
            /* Software emulation of level-triggered GPIO IRQ - generate SWI if level-based trigger is configured for the pin and pin is at corresponding level when IRQ is activated */
            LL_EXTI_GenerateSWI_0_31(exti_line);
        }
    }
    __COMPILER_BARRIER(); /* Ensure EXTI event flag is set before IRQ line is activated */

    HAL_NVIC_EnableIRQ(irq_num);
    __DSB();
    __ISB();
    __COMPILER_BARRIER();

    /* Once IRQ is enabled the timing constraints are relaxed and we can write something in the log */
    SID_PAL_GPIO_STM32WBAxx_LOG_DEBUG("Enable EXTI IRQ for P%s%d", NUM_TO_GPIO_PORT_STR(gpio_number), GET_GPIO_PIN_POSITION(NUM_TO_GPIO_PIN(gpio_number)));

    return SID_ERROR_NONE;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_pal_gpio_irq_disable(uint32_t gpio_number)
{
    if(GPIO_PORT_PIN_NOT_CONNECTED == gpio_number)
    {
        return SID_ERROR_NONE;
    }

    GPIO_TypeDef * const st_port = NUM_TO_GPIO_ST_PORT(gpio_number);
    const uint32_t st_pin = NUM_TO_GPIO_PIN(gpio_number);
    SID_PAL_ASSERT(IS_GPIO_ALL_INSTANCE(st_port));
    SID_PAL_ASSERT(IS_GPIO_PIN(st_pin));
    SID_PAL_ASSERT( false != IS_GPIO_CLK_ENABLED(st_port) );

    HAL_NVIC_DisableIRQ(GET_GPIO_PIN_IRQN(st_pin));
    __DSB();
    __ISB();
    __COMPILER_BARRIER();

    /* Once IRQ is disabled the timing constraints are relaxed and we can write something in the log */
    SID_PAL_GPIO_STM32WBAxx_LOG_DEBUG("Disable EXTI IRQ for P%s%d", NUM_TO_GPIO_PORT_STR(gpio_number), GET_GPIO_PIN_POSITION(NUM_TO_GPIO_PIN(gpio_number)));

    return SID_ERROR_NONE;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_pal_gpio_ext_ifc_set_irq_priority(const uint32_t gpio_number, const uint32_t irq_preempt_prio, const uint32_t irq_sub_prio)
{
    if (GPIO_PORT_PIN_NOT_CONNECTED == gpio_number)
    {
        return SID_ERROR_NONE;
    }

    GPIO_TypeDef * const st_port = NUM_TO_GPIO_ST_PORT(gpio_number);
    const uint32_t st_pin = NUM_TO_GPIO_PIN(gpio_number);
    SID_PAL_ASSERT(IS_GPIO_ALL_INSTANCE(st_port));
    SID_PAL_ASSERT(IS_GPIO_PIN(st_pin));
    SID_PAL_ASSERT( false != IS_GPIO_CLK_ENABLED(st_port) );

    HAL_NVIC_SetPriority(GET_GPIO_PIN_IRQN(st_pin), irq_preempt_prio, irq_sub_prio);
    __DSB();
    __ISB();
    __COMPILER_BARRIER();

    /* Once IRQ priority is configured the timing constraints are relaxed and we can write something in the log */
    SID_PAL_GPIO_STM32WBAxx_LOG_DEBUG("Set EXTI IRQ priority for P%s%d to %u.%u", NUM_TO_GPIO_PORT_STR(gpio_number), GET_GPIO_PIN_POSITION(NUM_TO_GPIO_PIN(gpio_number)), irq_preempt_prio, irq_sub_prio);

    return SID_ERROR_NONE;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED void sid_pal_gpio_exti_irq_handler(const uint32_t st_pin)
{
    const uint32_t exti_line = GET_LL_EXTI_LINE(st_pin);

    /* Clear Rising and Falling flags in EXTI first to resume edge detection ASAP */
    LL_EXTI_ClearRisingFlag_0_31(exti_line);
    LL_EXTI_ClearFallingFlag_0_31(exti_line);
    /* Ensure all states are fully synchronized before proceeding */
    __DSB();
    __ISB();
    __COMPILER_BARRIER();

    const uint32_t pin_index = GET_GPIO_PIN_POSITION(st_pin);
    SID_PAL_ASSERT(pin_index < GPIO_PINS_PER_PORT);
    GPIO_TypeDef * const st_port = NUM_TO_GPIO_ST_PORT(sid_pal_callbacks[pin_index].gpio_number);
    const uint32_t irq_num = GET_GPIO_PIN_IRQN(st_pin);

    /* Proceed with user callback invocation if such callback is assigned */
    if (sid_pal_callbacks[pin_index].callback != NULL)
    {
        /* Call the user-defined callback */
        sid_pal_callbacks[pin_index].callback(sid_pal_callbacks[pin_index].gpio_number, sid_pal_callbacks[pin_index].callback_arg);
    }

    if (
        ( /* Software emulation of the low level-triggered GPIO IRQ */
               (SID_PAL_GPIO_IRQ_TRIGGER_LOW == sid_pal_callbacks[pin_index].trigger)
            && ((st_port->IDR & st_pin) == 0x00u) /* Pin remains low */
            && (NVIC_GetEnableIRQ(irq_num)) /* Associated IRQ in NVIC remains enabled */
        )
        ||
        ( /* Software emulation of the high level-triggered GPIO IRQ */
               (SID_PAL_GPIO_IRQ_TRIGGER_HIGH == sid_pal_callbacks[pin_index].trigger)
            && ((st_port->IDR & st_pin) != 0x00u) /* Pin remains high */
            && (NVIC_GetEnableIRQ(irq_num)) /* Associated IRQ in NVIC remains enabled */
        )
    )
    {
        /* Generate an SWI if level-based trigger is configured for the pin and the pin remains at the tirggering level even after the user callback invocation */
        LL_EXTI_GenerateSWI_0_31(exti_line);
    }
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_pal_gpio_ext_ifc_check_latch_state(const uint32_t gpio_number, const sid_pal_gpio_irq_trigger_t edge, uint32_t * const is_edge_detected)
{
    sid_error_t err = SID_ERROR_NONE;
    const uint32_t st_pin = NUM_TO_GPIO_PIN(gpio_number);
    const uint32_t exti_line = GET_LL_EXTI_LINE(st_pin);

    if (GPIO_PORT_PIN_NOT_CONNECTED == gpio_number)
    {
        return SID_ERROR_NONE;
    }

    SID_PAL_ASSERT(IS_GPIO_PIN(st_pin));

    switch (edge)
    {
        case SID_PAL_GPIO_IRQ_TRIGGER_RISING:
            *is_edge_detected = LL_EXTI_IsActiveRisingFlag_0_31(exti_line);
            break;

        case SID_PAL_GPIO_IRQ_TRIGGER_FALLING:
            *is_edge_detected = LL_EXTI_IsActiveFallingFlag_0_31(exti_line);
            break;

        case SID_PAL_GPIO_IRQ_TRIGGER_EDGE:
            *is_edge_detected = (LL_EXTI_IsActiveRisingFlag_0_31(exti_line) || LL_EXTI_IsActiveFallingFlag_0_31(exti_line));
            break;

        default:
            err = SID_ERROR_NOSUPPORT;
            break;
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_pal_gpio_ext_ifc_clear_latch_state(const uint32_t gpio_number, const sid_pal_gpio_irq_trigger_t edge)
{
    sid_error_t err = SID_ERROR_NONE;
    const uint32_t st_pin = NUM_TO_GPIO_PIN(gpio_number);
    const uint32_t exti_line = GET_LL_EXTI_LINE(st_pin);

    if (GPIO_PORT_PIN_NOT_CONNECTED == gpio_number)
    {
        err = SID_ERROR_NONE;
    }

    SID_PAL_ASSERT(IS_GPIO_PIN(st_pin));

    switch (edge)
    {
        case SID_PAL_GPIO_IRQ_TRIGGER_RISING:
            LL_EXTI_ClearRisingFlag_0_31(exti_line);
            break;

        case SID_PAL_GPIO_IRQ_TRIGGER_FALLING:
            LL_EXTI_ClearFallingFlag_0_31(exti_line);
            break;

        case SID_PAL_GPIO_IRQ_TRIGGER_EDGE:
            LL_EXTI_ClearRisingFlag_0_31(exti_line);
            LL_EXTI_ClearFallingFlag_0_31(exti_line);
            break;

        default:
            err = SID_ERROR_NOSUPPORT;
            break;
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_pal_gpio_ext_ifc_set_as_wakeup(const uint32_t gpio_number, const sid_pal_gpio_irq_trigger_t edge)
{
    sid_error_t err;
    HAL_StatusTypeDef hal_ret;
    uint32_t pwr_wakeup_pin;
    uint32_t pwr_wakeup_flag;

    do
    {
        if (GPIO_PORT_PIN_NOT_CONNECTED == gpio_number)
        {
            err = SID_ERROR_NONE;
            break;
        }

        if ((edge != SID_PAL_GPIO_IRQ_TRIGGER_RISING) && (edge != SID_PAL_GPIO_IRQ_TRIGGER_FALLING))
        {
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        const uint32_t wakeup_polarity = (SID_PAL_GPIO_IRQ_TRIGGER_FALLING == edge) ? SID_PAL_GPIO_STM32WBAxx_WAKEUP_POLARITY_LOW : SID_PAL_GPIO_STM32WBAxx_WAKEUP_POLARITY_HIGH;

        err = get_pwr_wakeup_pin_flag(gpio_number, wakeup_polarity, &pwr_wakeup_pin, &pwr_wakeup_flag);
        if (err != SID_ERROR_NONE)
        {
            break;
        } 

        const uint32_t pwr_gpio_port = NUM_TO_PWR_GPIO_PORT(gpio_number);
        const uint16_t pwr_gpio_pin = (uint16_t)NUM_TO_GPIO_PIN(gpio_number);

        /* Enable GPIO retention for the selected wakeup pin */
        hal_ret = HAL_PWREx_EnableStandbyIORetention(pwr_gpio_port, pwr_gpio_pin);
        if (hal_ret != HAL_OK)
        {
            err = SID_ERROR_GENERIC;
            break;
        }

        /* Enable wakeup for the selected pin and trigger edge */
        HAL_PWR_EnableWakeUpPin(pwr_wakeup_pin);

        /* Clear wakeup flag for the selected pin in PWR */
        __HAL_PWR_CLEAR_FLAG(pwr_wakeup_flag);

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}
