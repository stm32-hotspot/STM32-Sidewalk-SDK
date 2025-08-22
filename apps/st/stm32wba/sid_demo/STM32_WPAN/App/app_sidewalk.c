/**
  ******************************************************************************
  * @file    app_sidewalk.c
  * @brief   Standardized Sidewalk Demo application
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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "app_freertos.h"

#include <sid_api.h>
#include <sid_demo_parser.h>
#include <sid_hal_reset_ifc.h>
#include <sid_log_control.h>
#include <sid_pal_assert_ifc.h>
#include <sid_pal_common_ifc.h>
#include <sid_pal_gpio_ifc.h>
#include <sid_pal_log_ifc.h>
#include <sid_pal_temperature_ifc.h>
#include <sid_900_cfg.h>

#include <sid_app_demo.h>
#include <sid_demo_parser.h>

#include SID_APP_VERSION_HEADER
#include <sid_sdk_version.h>

#include "target/memory.h"

#include "app_ble_config.h"
#include "app_900_config.h"
#include "app_common.h"
#include "sid_pal_gpio_ext_ifc.h"
#include <stm32_mcu_info.h>

#include "stm32wbaxx_nucleo.h"

/* Private defines -----------------------------------------------------------*/

#if ((SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 == 1) && (SID_SDK_CONFIG_ENABLE_LINK_TYPE_1 == 1))
#define DEMO_LINK_TYPE             (SID_LINK_TYPE_1 | SID_LINK_TYPE_3)
#elif((SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 == 1) && (SID_SDK_CONFIG_ENABLE_LINK_TYPE_1 == 1))
#define DEMO_LINK_TYPE             (SID_LINK_TYPE_1 | SID_LINK_TYPE_2)
#elif(SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 == 1)
#define DEMO_LINK_TYPE             (SID_LINK_TYPE_2)
#elif(SID_SDK_CONFIG_ENABLE_LINK_TYPE_1 == 1)
#define DEMO_LINK_TYPE             (SID_LINK_TYPE_1)
#else
#define DEMO_LINK_TYPE             (SID_LINK_TYPE_1)
#endif

#define BUTTON_LONG_PRESS_MS    (1500u)

#define SID_EVENT_WAIT_MS       (100u)

#if (CFG_BUTTON_SUPPORTED == 1)
#define DEMO_BUTTON_1_ID (0u)
#define DEMO_BUTTON_2_ID (1u)
#define DEMO_BUTTON_3_ID (2u)
#define DEMO_BUTTONS_MAX (3u)
#else
#define DEMO_BUTTONS_MAX (0u)
#endif

#if (CFG_LED_SUPPORTED == 1)
#define DEMO_LEDS_MAX (3u)
#else
#define DEMO_LEDS_MAX (0u)
#endif

#define DEMO_LEDS_NUM             (2u)
#define DEMO_LEDS_OFFSET          (1u)

/* Private macro -----------------------------------------------------------*/

#define OS_MS_TO_TICKS( xTimeInMs )    ( ( uint32_t ) ( ( ( uint32_t ) ( xTimeInMs ) * ( uint32_t ) osKernelGetTickFreq() ) / ( uint32_t ) 1000U ) )

/* Private typedef -----------------------------------------------------------*/

enum demo_app_led_id
{
    DEMO_APP_LED_ID_0 = 0,
    DEMO_APP_LED_ID_1 = 1,
    DEMO_APP_LED_ID_2 = 2,
    DEMO_APP_LED_ID_LAST,
};

typedef __PACKED_UNION {
    __PACKED_STRUCT {
        uint8_t build;
        uint8_t patch;
        uint8_t minor;
        uint8_t major;
    };
    uint32_t raw;
} hal_version_info_t;

/* Private variables ---------------------------------------------------------*/

static uint8_t led_array[DEMO_LEDS_MAX];
static uint8_t button_array[DEMO_BUTTONS_MAX];

/* Private constants ---------------------------------------------------------*/

static const osThreadAttr_t sidewalk_stack_task_attributes = {
    .name         = "Sidewalk Stack Task",
    .priority     = SID_MAIN_TASK_PRIO,
    .stack_size   = SID_MAIN_TASK_STACK_SIZE,
    .attr_bits    = TASK_DEFAULT_ATTR_BITS,
    .cb_mem       = TASK_DEFAULT_CB_MEM,
    .cb_size      = TASK_DEFAULT_CB_SIZE,
    .stack_mem    = TASK_DEFAULT_STACK_MEM,
};

static const osThreadAttr_t receive_task_attributes = {
    .name         = "Demo Receive Task",
    .priority     = RECEIVE_TASK_PRIO,
    .stack_size   = RECEIVE_TASK_STACK_SIZE,
    .attr_bits    = TASK_DEFAULT_ATTR_BITS,
    .cb_mem       = TASK_DEFAULT_CB_MEM,
    .cb_size      = TASK_DEFAULT_CB_SIZE,
    .stack_mem    = TASK_DEFAULT_STACK_MEM,
};

/* Private function prototypes -----------------------------------------------*/

static void platform_reset(void);
static int32_t init_and_start_link(app_context_t *context, struct sid_config *config, uint32_t link_mask);

static void sidewalk_stack_task_entry(void *context);
static void receive_task_entry(void *context);
#if (CFG_BUTTON_SUPPORTED == 1)
#  if (defined(STM32WBA6x) && !defined(SID_RADIO_PLATFORM_LR11XX) && !defined(SID_RADIO_PLATFORM_SX126X)) || !defined(STM32WBA6x) /* Semtech shields share the same EXTI line for BUSY signal */
static void button1_irq_handler(uint32_t pin, void * callback_arg);
#  endif /* (STM32WBA6x && !SID_RADIO_PLATFORM_LR11XX && !SID_RADIO_PLATFORM_SX126X) || !STM32WBA6x */
#  if (defined(STM32WBA5x) && !defined(SID_RADIO_PLATFORM_LR11XX) && !defined(SID_RADIO_PLATFORM_SX126X)) || !defined(STM32WBA5x) /* Semtech shields use the same GPIO pin for BUSY signal */
static void button2_irq_handler(uint32_t pin, void * callback_arg);
#  endif /* (STM32WBA5x && !SID_RADIO_PLATFORM_LR11XX && !SID_RADIO_PLATFORM_SX126X) || !STM32WBA5x */
static void button3_irq_handler(uint32_t pin, void * callback_arg);
#endif

/* Private function definitions ----------------------------------------------*/

static void platform_reset(void)
{
    (void)sid_hal_reset(SID_HAL_RESET_NORMAL);
}

/*----------------------------------------------------------------------------*/

static void turn_on_leds(enum demo_app_led_id id)
{
#if (CFG_LED_SUPPORTED == 1)
    switch (id) {
        case DEMO_APP_LED_ID_0:
            BSP_LED_On(LED_BLUE);
            break;
        case DEMO_APP_LED_ID_1:
            BSP_LED_On(LED_GREEN);
            break;
        case DEMO_APP_LED_ID_2:
            BSP_LED_On(LED_RED);
            break;
        default:
            break;
    }
#endif
}

/*----------------------------------------------------------------------------*/

static void turn_off_leds(enum demo_app_led_id id)
{
#if (CFG_LED_SUPPORTED == 1)
    switch (id) {
        case DEMO_APP_LED_ID_0:
            BSP_LED_Off(LED_BLUE);
            break;
        case DEMO_APP_LED_ID_1:
            BSP_LED_Off(LED_GREEN);
            break;
        case DEMO_APP_LED_ID_2:
            BSP_LED_Off(LED_RED);
            break;
        default:
            break;
    }
#endif
}

/*----------------------------------------------------------------------------*/

static bool is_led_on(enum demo_app_led_id id)
{
    bool ret = true;
#if (CFG_LED_SUPPORTED == 1)
    switch (id) {
        case DEMO_APP_LED_ID_0:
            ret = BSP_LED_GetState(LED_BLUE);
            break;
        case DEMO_APP_LED_ID_1:
            ret = BSP_LED_GetState(LED_GREEN);
            break;
        case DEMO_APP_LED_ID_2:
            ret = BSP_LED_GetState(LED_RED);
            break;
        default:
            break;
    }
#endif

    return ret;
}

/*----------------------------------------------------------------------------*/

static int32_t init_and_start_link(app_context_t *context, struct sid_config *config, uint32_t link_mask)
{
    sid_error_t ret = SID_ERROR_NONE;
    struct sid_handle *sid_handle = NULL;
    config->link_mask = link_mask;
    // Initialize sidewalk
    ret = sid_init(config, &sid_handle);
    if (ret != SID_ERROR_NONE) {
        SID_PAL_LOG_ERROR("failed to initialize sidewalk link_mask:%x, err:%d", link_mask, (int)ret);
        goto error;
    }

    /* Register sidewalk handler to the application context */
    context->sidewalk_handle = sid_handle;

    /* Start the sidewalk stack */
    ret = sid_start(sid_handle, link_mask);
    if (ret != SID_ERROR_NONE) {
        SID_PAL_LOG_ERROR("failed to start sidewalk, link_mask:%x, err:%d", link_mask, (int)ret);
        goto error;
    }
    SID_PAL_LOG_INFO("start sidewalk, link_mask:%x", link_mask);
    return ret;

error:
    context->sidewalk_handle = NULL;
    config->link_mask = 0;
    return ret;
}

/*----------------------------------------------------------------------------*/

#if (CFG_BUTTON_SUPPORTED == 1)
#  if defined(STM32WBA6x) && !defined(SID_RADIO_PLATFORM_LR11XX) && !defined(SID_RADIO_PLATFORM_SX126X) || !defined(STM32WBA6x) /* Semtech shields share the same EXTI line for BUSY signal */
static void button1_irq_handler(uint32_t pin, void * callback_arg)
{
    static uint32_t uStart = 0u;
    static uint32_t uEnd   = 0u;
    uint8_t pinState;
    sid_error_t ret;

    (void)callback_arg;

    ret = sid_pal_gpio_read(pin, &pinState);
    if (SID_ERROR_NONE == ret)
    {
        if (GPIO_PIN_RESET == pinState)
        {
            uStart = osKernelGetTickCount();
        }
        else
        {
            uEnd = osKernelGetTickCount();

            if ((uEnd - uStart) < OS_MS_TO_TICKS(BUTTON_LONG_PRESS_MS))
            {
                /* Short-press event */
                demo_app_button_event_handler(DEMO_BUTTON_1_ID);
            }
            else
            {
                /* Long-press event */
                demo_app_factory_reset_trigger_handler();
            }
        }
    }
    else
    {
        SID_PAL_LOG_ERROR("Failed to read B1 state. Error %d", (int32_t)ret);
    }
}
#  endif /* (STM32WBA6x && !SID_RADIO_PLATFORM_LR11XX && !SID_RADIO_PLATFORM_SX126X) || !STM32WBA6x */

/*----------------------------------------------------------------------------*/

#  if defined(STM32WBA5x) && !defined(SID_RADIO_PLATFORM_LR11XX) && !defined(SID_RADIO_PLATFORM_SX126X) || !defined(STM32WBA5x) /* Semtech shields use the same GPIO pin for BUSY signal */
static void button2_irq_handler(uint32_t pin, void * callback_arg)
{
    (void)pin;
    (void)callback_arg;

#    if  defined(STM32WBA6x) && (defined(SID_RADIO_PLATFORM_LR11XX) || defined(SID_RADIO_PLATFORM_SX126X)) /* Semtech shields occupy B1 EXTI line for BUSY signal, use long B2 press instead */
static uint32_t uStart = 0u;
    static uint32_t uEnd   = 0u;
    uint8_t pinState;
    sid_error_t ret;

     ret = sid_pal_gpio_read(pin, &pinState);
    if (SID_ERROR_NONE == ret)
    {
        if (GPIO_PIN_RESET == pinState)
        {
            uStart = osKernelGetTickCount();
        }
        else
        {
            uEnd = osKernelGetTickCount();

            if((uEnd - uStart) < OS_MS_TO_TICKS(BUTTON_LONG_PRESS_MS))
            {
                /* Short-press event */
                demo_app_button_event_handler(DEMO_BUTTON_2_ID);
            }
            else
            {
                /* Long-press event */
                demo_app_button_event_handler(DEMO_BUTTON_1_ID);
            }
        }
    }
    else
    {
        SID_PAL_LOG_ERROR("Failed to read B2 state. Error %d", (int32_t)ret);
    }
#    else
    demo_app_button_event_handler(DEMO_BUTTON_2_ID);
#    endif  /* STM32WBA6x && (SID_RADIO_PLATFORM_LR11XX || SID_RADIO_PLATFORM_SX126X) */
}
#  endif /* (STM32WBA5x && !SID_RADIO_PLATFORM_LR11XX && !SID_RADIO_PLATFORM_SX126X) || !STM32WBA5x */

/*----------------------------------------------------------------------------*/

static void button3_irq_handler(uint32_t pin, void * callback_arg)
{
#  if defined(STM32WBA5x) && (defined(SID_RADIO_PLATFORM_LR11XX) || defined(SID_RADIO_PLATFORM_SX126X)) /* Semtech shields occupy B2 GPIO pin for BUSY signal, use extra-long B3 press instead */
    static uint32_t uStart = 0u;
    static uint32_t uEnd   = 0u;
    uint8_t pinState;
    sid_error_t ret;

    (void)callback_arg;

    ret = sid_pal_gpio_read(pin, &pinState);
    if (SID_ERROR_NONE == ret)
    {
        if (GPIO_PIN_RESET == pinState)
        {
            uStart = osKernelGetTickCount();
        }
        else
        {
            uEnd = osKernelGetTickCount();

            if ((uEnd - uStart) >= (4u * OS_MS_TO_TICKS(BUTTON_LONG_PRESS_MS)))
            {
                /* Extra-long-press event */
                demo_app_button_event_handler(DEMO_BUTTON_2_ID);
            }
            else
            {
                /* Short-press event */
                demo_app_button_event_handler(DEMO_BUTTON_3_ID);
            }
        }
    }
    else
    {
        SID_PAL_LOG_ERROR("Failed to read B3 state. Error %d", (int32_t)ret);
    }
#  else
    (void)pin;
    (void)callback_arg;

    demo_app_button_event_handler(DEMO_BUTTON_3_ID);
#  endif /* STM32WBA5x && (SID_RADIO_PLATFORM_LR11XX || SID_RADIO_PLATFORM_SX126X) */
}
#endif /* CFG_BUTTON_SUPPORTED */

/*----------------------------------------------------------------------------*/

static void sidewalk_stack_task_entry(void *context)
{
    SID_PAL_LOG_INFO("Sidewalk demo started");

    app_context_t *app_context = (app_context_t *)context;

    struct sid_config config = {
        .link_mask = 0,
        .callbacks = demo_app_get_sid_event_callbacks(),
        .link_config = app_get_sidewalk_ble_config(),
        .sub_ghz_link_config = app_get_sub_ghz_config(),
    };

    /* Performing a cold start of the SIdewalk since device registration status is unknown at this point */
    uint32_t link_mask = DEMO_LINK_TYPE;
    sid_error_t ret = init_and_start_link(app_context, &config, link_mask);
    if (ret != SID_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("failed to initialize sidewalk. Error code: %d", (int)ret);
        goto error;
    }

    while (1)
    {
        struct app_demo_event event;

        if (osMessageQueueGet(app_context->event_queue, &event, 0u, OS_MS_TO_TICKS(SID_EVENT_WAIT_MS)) == osOK)
        {
            demo_app_main_task_event_handler(&event);
        }
    }

error:
    if (app_context->sidewalk_handle != NULL)
    {
        sid_stop(app_context->sidewalk_handle, config.link_mask);
        sid_deinit(app_context->sidewalk_handle);
        app_context->sidewalk_handle = NULL;
    }

    SID_PAL_LOG_INFO("Sidewalk demo terminated");
    osThreadExit();
}

/*----------------------------------------------------------------------------*/

static void receive_task_entry(void *context)
{
    receive_context_t *rcv_context = (receive_context_t *)context;

    while (1) {
        SID_PAL_LOG_FLUSH();
        struct app_demo_rx_msg rx_msg;

        if (osMessageQueueGet(rcv_context->receive_event_queue, &rx_msg, NULL, portMAX_DELAY) == osOK) {  
            demo_app_receive_task_event_handler(&rx_msg);
        }
    }
}

/* Global function definitions -----------------------------------------------*/

void SID_APP_Init(void)
{
    /* Adjust Sidewalk log level dynamically - this is needed if the firmware is compiled using
     * Sidewalk SDK static library and the library was assembled with different debug level
     */
    struct sid_log_control_severity sid_log_settings;
    sid_log_control_get_severity(&sid_log_settings);
    if (SID_PAL_LOG_LEVEL < sid_log_settings.level)
    {
        sid_log_settings.level = SID_PAL_LOG_LEVEL;
        sid_log_control_set_severity(&sid_log_settings);
    }

    /* Printout application version info */
    SID_PAL_LOG_INFO("Application name: %s", SID_APP_PROJECT_NAME);
    SID_PAL_LOG_INFO("Application version %s", SID_APP_PROJECT_VERSION_STRING);
    SID_PAL_LOG_INFO("Application build type: %s", SID_APP_PROJECT_BUILD_TYPE);
    SID_PAL_LOG_INFO("Application commit hash: %s", SID_APP_PROJECT_COMMIT_HASH_STRING);
    SID_PAL_LOG_INFO("Application commit description: %s", SID_APP_PROJECT_COMMIT_DESCRIPTION);
    SID_PAL_LOG_INFO("Sidewalk SDK: %u.%u.%u-%u", SID_SDK_MAJOR_VERSION, SID_SDK_MINOR_VERSION, SID_SDK_PATCH_VERSION, SID_SDK_BUILD_VERSION);
    SID_PAL_LOG_INFO("FreeRTOS Kernel: %s", tskKERNEL_VERSION_NUMBER);

    /* CubeMX pack version */
    const hal_version_info_t cubemx_fw_pack_ver = {
        .raw = HAL_GetHalVersion(),
    };
    SID_PAL_LOG_INFO("STM32CubeWBA: %u.%u.%u", cubemx_fw_pack_ver.major, cubemx_fw_pack_ver.minor, cubemx_fw_pack_ver.patch);

    /* Printout MCU details */
    stm32_mcu_info_t mcu_info = stm32_mcu_info_describe_host();
    SID_PAL_LOG_INFO("Host MCU: %s (0x%X), revision: %s (0x%X)", mcu_info.device_name, mcu_info.device_id, mcu_info.rev_name, mcu_info.rev_id);

    platform_parameters_t platform_parameters = {
        .mfg_store_region.addr_start = MANUFACTURE_FLASH_START,
        .mfg_store_region.addr_end   = MANUFACTURE_FLASH_END,

#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 || SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
        .platform_init_parameters.radio_cfg = get_radio_cfg(),
#endif
    };

    sid_error_t ret_code = sid_platform_init(&platform_parameters);
    if (ret_code != SID_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("Sidewalk Platform Init err: %d", ret_code);
        SID_PAL_ASSERT(0);
    }

    uint32_t num_leds = 0;
#if (CFG_LED_SUPPORTED == 1)
    uint32_t available_led_offset = DEMO_LEDS_OFFSET;
    num_leds = DEMO_LEDS_NUM;

    for (uint32_t i = 0; i < num_leds; i++) 
    {
        led_array[i] = (uint8_t) (i + available_led_offset);
    } 
#endif

#if (CFG_BUTTON_SUPPORTED == 1)
    for (uint32_t i = 0u; i < DEMO_BUTTONS_MAX; i++)
    {
        button_array[i] = (uint8_t)i;
    }

#  if defined(STM32WBA6x) && !defined(SID_RADIO_PLATFORM_LR11XX) && !defined(SID_RADIO_PLATFORM_SX126X) || !defined(STM32WBA6x) /* Semtech shields share the same EXTI line for BUSY signal */
    ret_code = sid_pal_gpio_pull_mode(GPIO_PORT_PIN_TO_NUM(B1_GPIO_PORT, B1_PIN), SID_PAL_GPIO_PULL_UP);
    SID_PAL_ASSERT(ret_code == SID_ERROR_NONE);
    ret_code = sid_pal_gpio_set_direction(GPIO_PORT_PIN_TO_NUM(B1_GPIO_PORT, B1_PIN), SID_PAL_GPIO_DIRECTION_INPUT);
    SID_PAL_ASSERT(ret_code == SID_ERROR_NONE);
#  endif /* (STM32WBA6x && !SID_RADIO_PLATFORM_LR11XX && !SID_RADIO_PLATFORM_SX126X) || !STM32WBA6x */

#  if defined(STM32WBA5x) && !defined(SID_RADIO_PLATFORM_LR11XX) && !defined(SID_RADIO_PLATFORM_SX126X) || !defined(STM32WBA5x) /* Semtech shields use the same GPIO pin for BUSY signal */
    ret_code = sid_pal_gpio_pull_mode(GPIO_PORT_PIN_TO_NUM(B2_GPIO_PORT, B2_PIN), SID_PAL_GPIO_PULL_UP);
    SID_PAL_ASSERT(ret_code == SID_ERROR_NONE);
    ret_code = sid_pal_gpio_set_direction(GPIO_PORT_PIN_TO_NUM(B2_GPIO_PORT, B2_PIN), SID_PAL_GPIO_DIRECTION_INPUT);
    SID_PAL_ASSERT(ret_code == SID_ERROR_NONE);
#  endif /* (STM32WBA5x && !SID_RADIO_PLATFORM_LR11XX && !SID_RADIO_PLATFORM_SX126X) || !STM32WBA5x */

    ret_code = sid_pal_gpio_pull_mode(GPIO_PORT_PIN_TO_NUM(B3_GPIO_PORT, B3_PIN), SID_PAL_GPIO_PULL_UP);
    SID_PAL_ASSERT(ret_code == SID_ERROR_NONE);
    ret_code = sid_pal_gpio_set_direction(GPIO_PORT_PIN_TO_NUM(B3_GPIO_PORT, B3_PIN), SID_PAL_GPIO_DIRECTION_INPUT);
    SID_PAL_ASSERT(ret_code == SID_ERROR_NONE);

    /* Give the GPIO pins some time to reach a stable state before enabling IRQs */
    const uint32_t gpio_settle_delay = 75u * (SystemCoreClock / 1000000u); /* Approx. 75us */
    for (uint32_t i = 0u; i < gpio_settle_delay; i++)
    {
        /* Can't use timers here because initialization is not finished yet */
        __NOP();
    }

#  if defined(STM32WBA6x) && !defined(SID_RADIO_PLATFORM_LR11XX) && !defined(SID_RADIO_PLATFORM_SX126X) || !defined(STM32WBA6x) /* Semtech shields share the same EXTI line for BUSY signal */
    ret_code = sid_pal_gpio_set_irq(GPIO_PORT_PIN_TO_NUM(B1_GPIO_PORT, B1_PIN), SID_PAL_GPIO_IRQ_TRIGGER_EDGE, button1_irq_handler, NULL);
    SID_PAL_ASSERT(ret_code == SID_ERROR_NONE);
#  endif /* (STM32WBA6x && !SID_RADIO_PLATFORM_LR11XX && !SID_RADIO_PLATFORM_SX126X) || !STM32WBA6x */

#  if defined(STM32WBA6x) && (defined(SID_RADIO_PLATFORM_LR11XX) || defined(SID_RADIO_PLATFORM_SX126X))
    ret_code = sid_pal_gpio_set_irq(GPIO_PORT_PIN_TO_NUM(B2_GPIO_PORT, B2_PIN), SID_PAL_GPIO_IRQ_TRIGGER_EDGE, button2_irq_handler, NULL);
    SID_PAL_ASSERT(ret_code == SID_ERROR_NONE);
#  elif (defined(STM32WBA5x) && !defined(SID_RADIO_PLATFORM_LR11XX) && !defined(SID_RADIO_PLATFORM_SX126X)) || !defined(STM32WBA5x) /* Semtech shields use the same GPIO pin for BUSY signal */
    ret_code = sid_pal_gpio_set_irq(GPIO_PORT_PIN_TO_NUM(B2_GPIO_PORT, B2_PIN), SID_PAL_GPIO_IRQ_TRIGGER_FALLING, button2_irq_handler, NULL);
    SID_PAL_ASSERT(ret_code == SID_ERROR_NONE);
#  endif

#  if defined(STM32WBA5x) && (defined(SID_RADIO_PLATFORM_LR11XX) || defined(SID_RADIO_PLATFORM_SX126X)) /* Semtech shields occupy B2 GPIO pin for BUSY signal, use extra-long B3 press instead */
    /* Trigger B3 on both edges since B2 press is substituted by a long press on B3 */
    ret_code = sid_pal_gpio_set_irq(GPIO_PORT_PIN_TO_NUM(B3_GPIO_PORT, B3_PIN), SID_PAL_GPIO_IRQ_TRIGGER_EDGE, button3_irq_handler, NULL);
    SID_PAL_ASSERT(ret_code == SID_ERROR_NONE);
#  else
    ret_code = sid_pal_gpio_set_irq(GPIO_PORT_PIN_TO_NUM(B3_GPIO_PORT, B3_PIN), SID_PAL_GPIO_IRQ_TRIGGER_FALLING, button3_irq_handler, NULL);
    SID_PAL_ASSERT(ret_code == SID_ERROR_NONE);
#  endif /* STM32WBA5x && (SID_RADIO_PLATFORM_LR11XX || SID_RADIO_PLATFORM_SX126X) */
#endif /* CFG_BUTTON_SUPPORTED */

    static app_context_t demo_app_context = {
        .led_id_arr = led_array,
        .led_num = DEMO_LEDS_NUM,
        .button_id_arr = button_array,
        .button_num = DEMO_BUTTONS_MAX,
        .hw_ifc = {
            .turn_on_leds = turn_on_leds,
            .turn_off_leds = turn_off_leds,
            .is_led_on = is_led_on,
            .temp_read = sid_pal_temperature_get,
            .platform_reset = platform_reset,
        }
    };

    static receive_context_t demo_receive_context;
    demo_app_init(&demo_app_context, &demo_receive_context);

    demo_app_context.main_task = osThreadNew(sidewalk_stack_task_entry, &demo_app_context, &sidewalk_stack_task_attributes);
    if (NULL == demo_app_context.main_task)
    {
        SID_PAL_LOG_ERROR("Can't create Sidewalk processing thread. No memory");
        SID_PAL_ASSERT(0);
    }

    demo_receive_context.receive_task = osThreadNew(receive_task_entry, &demo_receive_context, &receive_task_attributes);
    if (NULL == demo_receive_context.receive_task)
    {
        SID_PAL_LOG_ERROR("Can't create Sidewalk demo thread. No memory");
        SID_PAL_ASSERT(0);
    }
}
